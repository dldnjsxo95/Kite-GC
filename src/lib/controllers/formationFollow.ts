// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Marc Hoffmann (b14ckyy)

// GCS-driven formation trajectory follower (formation stage 2). Instead of uploading offset copies of
// the route, the GCS runs ONE clock: a virtual leader advances along the arc-length parametrised
// reference path at the chosen speed, and every vehicle is streamed the position of ITS slot around
// the leader (SET_POSITION_TARGET_GLOBAL_INT, ~5 Hz, no ACK, velocity feed-forward). Because all
// targets derive from the same `s(t)`, command latency, wind and acceleration differences are
// corrected continuously instead of becoming permanent offsets — the drone-show principle, with the
// clock in the GCS rather than on board.
//
// Phases: idle → engaging (stream slot-at-start targets, then GUIDED/OFFBOARD) → assembling (hold at
// s = 0 until the slots are filled) → running (leader advances) ⇄ paused → finished (hold at the end)
// → stop (Loiter/Hold on every vehicle, streaming off). Losing the GCS link stops the stream; each
// FC's own GCS failsafe then applies — operators must have that configured.
// See docs/02-design/features/formation-flight.design.md §9.

import { invoke } from '@tauri-apps/api/core';
import { writable, derived, get } from 'svelte/store';
import type { VehicleSummary } from '$lib/stores/vehicles';
import { vehicles } from '$lib/stores/vehicles';
import { allTelemetry } from '$lib/stores/telemetry';
import type { ArduWaypoint } from '$lib/stores/missionArdupilot';
import { formationPreview, formationPathPreview, type FormationPreviewSlot } from '$lib/stores/formation';
import { modesFor } from '$lib/helpers/mavModes';
import { type FormationSlot, bodyToNorthEast } from '$lib/helpers/formationMission';
import { metresToLatLonDelta } from '$lib/helpers/missionGeo';
import { buildPath, pointAt, velocityNed, curvatureAt, headingDelta, type FormationPath } from '$lib/helpers/trajectory';
import {
  orderedVehicles, vehicleColorFor, vehicleSystem, vehicleClassOf, isFixedWingClass, resolveIntentMode, linkState,
} from '$lib/helpers/fleetStatus';
import { isValidGpsCoordinate } from '$lib/helpers/telemetry';
import { haversineDistance } from '$lib/utils/geo';

export type FollowPhase = 'idle' | 'engaging' | 'assembling' | 'running' | 'paused' | 'finished' | 'stopping';

export interface FollowVehicle {
  vehicleId: string;
  name: string;
  color: string;
  slot: FormationSlot;
  plane: boolean;
  /** Current streamed target. */
  targetLat: number;
  targetLon: number;
  targetAlt: number;
  /** Horizontal distance vehicle → target (m), NaN without a fix. */
  errorM: number;
  inPosition: boolean;
  linkLost: boolean;
  /** Last mode-switch / send error, '' when fine. */
  error: string;
}

export interface FollowState {
  phase: FollowPhase;
  /** Leader position along the path (m). */
  s: number;
  lengthM: number;
  /** Commanded route speed (m/s). */
  speedMs: number;
  /** Speed the leader is actually moving at right now (ramped + paced), m/s. */
  currentSpeedMs: number;
  /** 0..1 — how much the formation lag is throttling the leader (1 = full speed). */
  pace: number;
  /** Largest slot error among tracked vehicles (m), NaN without fixes. */
  maxErrorM: number;
  headingRelative: boolean;
  vehicles: FollowVehicle[];
  startedAt: number | null;
  message: string;
}

const IDLE: FollowState = {
  phase: 'idle', s: 0, lengthM: 0, speedMs: 5, currentSpeedMs: 0, pace: 1, maxErrorM: NaN,
  headingRelative: true, vehicles: [], startedAt: null, message: '',
};

export const followState = writable<FollowState>({ ...IDLE });
export const followActive = derived(followState, (st) => st.phase !== 'idle');
export const followAllInPosition = derived(followState, (st) => st.vehicles.length > 0 && st.vehicles.every((v) => v.inPosition));

const TICK_MS = 100;             // 10 Hz setpoint stream
const ENGAGE_STREAM_MS = 1200;   // PX4 wants setpoints flowing before OFFBOARD is accepted
const COPTER_RADIUS_M = 5;
const COPTER_ALT_M = 3;
const PLANE_MARGIN_M = 15;
const PLANE_ALT_M = 10;

// ── Closed-loop leader pacing ──
// An open-loop clock runs away from the aircraft: every bit of acceleration limit, transport delay and
// wind becomes lag that nothing takes back. So the leader (a) ramps its speed instead of stepping it,
// (b) slows down as the worst slot error grows and stops when it is large, (c) sends the setpoint a
// little AHEAD of the leader (position + velocity feed-forward) so the FCs don't decelerate towards a
// point that is only a tick away, and (d) pauses itself when a vehicle is clearly not following.
// Telemetry latency alone shows as ~speed × 0.5 s of "error", so the thresholds start above that.
const LEADER_ACCEL_MS2 = 1.0;    // ramp for the leader speed (up and down)
const PACE_FULL_ERR_M = 4;       // below this slot error the leader runs at full speed
const PACE_STOP_ERR_M = 12;      // at/above this the leader waits (pace 0)
const LOOKAHEAD_S = 0.8;         // setpoint sent this far ahead of the leader along the path
const AUTO_PAUSE_ERR_M = 25;     // a vehicle this far off for AUTO_PAUSE_S pauses the run
const AUTO_PAUSE_S = 3;

// ── Smooth turns ──
// The formation frame does NOT snap to the track heading: it turns towards it at most `maxYawRate`
// deg/s, so the slots swing round gradually. The reference route is corner-rounded (fillets) when
// built, and in a turn the leader slows so that (a) the frame never has to rotate faster than that
// limit (ω = v·κ) and (b) the outermost slot never has to fly more than ~30 % faster than the set
// route speed (its speed is v·(1 + r_out·κ)).
const OUTER_SPEED_FACTOR = 1.3;

let path: FormationPath | null = null;
let timer: ReturnType<typeof setInterval> | null = null;
let lastTick = 0;
let lagSince = 0;
let fHeadingDeg = 0;          // smoothed formation heading
let maxYawRateDegS = 12;
let outerRadiusM = 0;         // largest lateral slot offset (|right| plus |forward| for safety)

function patch(p: Partial<FollowState>): void {
  followState.update((st) => ({ ...st, ...p }));
}

/** Slot target around the leader point. */
function slotTarget(lat: number, lon: number, alt: number, hDeg: number, slot: FormationSlot, headingRelative: boolean): { lat: number; lon: number; alt: number } {
  const { north, east } = bodyToNorthEast(slot.forward, slot.right, headingRelative ? hDeg : 0);
  const { dLat, dLon } = metresToLatLonDelta(lat, north, east);
  return { lat: lat + dLat, lon: lon + dLon, alt: alt + slot.up };
}

/** Leader throttle from the worst slot error: 1 below PACE_FULL_ERR_M, 0 at/above PACE_STOP_ERR_M. */
function paceFor(maxErr: number): number {
  if (!Number.isFinite(maxErr)) return 1;
  if (maxErr <= PACE_FULL_ERR_M) return 1;
  if (maxErr >= PACE_STOP_ERR_M) return 0;
  return 1 - (maxErr - PACE_FULL_ERR_M) / (PACE_STOP_ERR_M - PACE_FULL_ERR_M);
}

function tick(): void {
  const st = get(followState);
  if (!path || st.phase === 'idle' || st.phase === 'stopping') return;
  const now = Date.now();
  const dt = lastTick ? Math.min(1, (now - lastTick) / 1000) : 0;
  lastTick = now;
  const amsl = path.frame === 0;
  const telem = get(allTelemetry);

  // 1. Where is everyone relative to the target streamed last tick? (drives the pacing)
  let maxErr = NaN;
  const measured = st.vehicles.map((fv) => {
    const tv = telem.get(fv.vehicleId);
    const linkLost = !tv || linkState(tv, now) === 'lost';
    let errorM = NaN;
    let dAlt = NaN;
    if (tv && isValidGpsCoordinate(tv.lat, tv.lon) && fv.targetLat !== 0) {
      errorM = haversineDistance(tv.lat, tv.lon, fv.targetLat, fv.targetLon);
      dAlt = Math.abs((amsl ? tv.altMsl : tv.altitude) - fv.targetAlt);
      if (!linkLost && (!Number.isFinite(maxErr) || errorM > maxErr)) maxErr = errorM;
    }
    return { fv, tv, linkLost, errorM, dAlt };
  });

  // 2. Advance the leader: ramped speed × pacing. Pause itself if someone is clearly not following.
  let s = st.s;
  let phase = st.phase;
  let current = st.currentSpeedMs;
  let pace = st.pace;
  let message = st.message;
  if (phase === 'running') {
    pace = paceFor(maxErr);
    // Turn slowdown: keep the frame rotation ≤ maxYawRate and the outer slot ≤ 1.3× the route speed.
    const kappa = curvatureAt(path, s);
    let vTurn = Infinity;
    if (kappa > 1e-4) {
      const omegaMax = (maxYawRateDegS * Math.PI) / 180;
      vTurn = Math.min(omegaMax / kappa, (st.speedMs * OUTER_SPEED_FACTOR) / (1 + outerRadiusM * kappa));
    }
    const wanted = Math.min(st.speedMs, vTurn) * pace;
    const maxStep = LEADER_ACCEL_MS2 * dt;
    current = current < wanted ? Math.min(wanted, current + maxStep) : Math.max(wanted, current - maxStep);
    s = Math.min(path.lengthM, s + current * dt);
    if (s >= path.lengthM) { phase = 'finished'; current = 0; }
    if (Number.isFinite(maxErr) && maxErr >= AUTO_PAUSE_ERR_M) {
      if (!lagSince) lagSince = now;
      else if (now - lagSince > AUTO_PAUSE_S * 1000) { phase = 'paused'; current = 0; message = `Paused: a vehicle is ${maxErr.toFixed(0)} m off its slot`; lagSince = 0; }
    } else {
      lagSince = 0;
    }
  } else {
    current = 0;
    pace = 1;
    lagSince = 0;
  }
  const leader = pointAt(path, s);
  if (!leader) return;
  const moving = phase === 'running' && current > 0.05;
  // 3. Setpoint a little ahead of the leader so the FC keeps its speed up instead of braking for a
  //    point one tick away; velocity feed-forward along the track at the leader's actual speed.
  const ahead = moving ? pointAt(path, Math.min(path.lengthM, s + current * LOOKAHEAD_S)) ?? leader : leader;
  const vel = moving ? velocityNed(current, ahead.headingDeg) : [0, 0, 0];
  // Formation frame heading: turns towards the track heading at ≤ maxYawRate, never snaps.
  if (dt > 0) {
    const maxTurn = maxYawRateDegS * dt;
    const dh = headingDelta(fHeadingDeg, ahead.headingDeg);
    fHeadingDeg = (fHeadingDeg + Math.max(-maxTurn, Math.min(maxTurn, dh)) + 360) % 360;
  }

  const preview: FormationPreviewSlot[] = [];
  const vehiclesNext = measured.map(({ fv, linkLost, errorM, dAlt }) => {
    const tgt = slotTarget(ahead.lat, ahead.lon, ahead.alt, fHeadingDeg, fv.slot, st.headingRelative);
    const inPosition = Number.isFinite(errorM) && (fv.plane
      ? errorM <= PLANE_MARGIN_M + 20 && dAlt <= PLANE_ALT_M
      : errorM <= COPTER_RADIUS_M && dAlt <= COPTER_ALT_M);
    if (!linkLost) {
      // Fire-and-forget; a failed send is recorded on the row, the stream carries on.
      void invoke('mav_set_position_target', {
        vehicleId: fv.vehicleId,
        lat: tgt.lat, lon: tgt.lon, alt: tgt.alt,
        vx: vel[0], vy: vel[1], vz: vel[2],
        yawDeg: fv.plane ? undefined : fHeadingDeg,
        amsl,
      }).catch((e: unknown) => {
        followState.update((cur) => ({
          ...cur,
          vehicles: cur.vehicles.map((x) => (x.vehicleId === fv.vehicleId ? { ...x, error: String(e) } : x)),
        }));
      });
    }
    preview.push({ index: fv.slot.index, lat: tgt.lat, lon: tgt.lon, up: fv.slot.up, vehicleId: fv.vehicleId, color: fv.color, name: fv.name });
    return { ...fv, targetLat: tgt.lat, targetLon: tgt.lon, targetAlt: tgt.alt, errorM, inPosition, linkLost };
  });
  formationPreview.set(preview);
  followState.update((cur) => ({
    ...cur,
    s,
    phase: cur.phase === phase ? cur.phase : phase,
    currentSpeedMs: current,
    pace,
    maxErrorM: maxErr,
    message,
    vehicles: vehiclesNext,
  }));
}

function startTimer(): void {
  if (timer) return;
  lastTick = 0;
  timer = setInterval(tick, TICK_MS);
}
function stopTimer(): void {
  if (timer) { clearInterval(timer); timer = null; }
  lastTick = 0;
}

async function setMode(v: VehicleSummary, main: number, sub: number): Promise<void> {
  await invoke('mav_set_mode', { vehicleId: v.vehicleId, main, sub });
}

/** Mode that accepts external position setpoints: ArduPilot GUIDED, PX4 OFFBOARD. */
function setpointMode(v: VehicleSummary): { main: number; sub: number } | null {
  const sys = vehicleSystem(v);
  if (!sys) return null;
  if (sys === 'px4') {
    const m = modesFor('px4', vehicleClassOf(v)).find((x) => x.key === 'offboard');
    return m ? { main: m.main, sub: m.sub } : null;
  }
  const g = resolveIntentMode(v, 'guided');
  return g ? { main: g.main, sub: g.sub } : null;
}

export interface EngageOptions {
  plan: readonly ArduWaypoint[];
  /** slot index → vehicle id */
  assignments: ReadonlyMap<number, string>;
  slots: readonly FormationSlot[];
  speedMs: number;
  headingRelative: boolean;
  /** Corner rounding radius for the route (m, 0 = sharp). */
  turnRadiusM?: number;
  /** Max rotation rate of the formation frame (deg/s). */
  maxYawRateDegS?: number;
}

/** Start the follower: build the (corner-rounded) path, stream slot-at-start targets, switch every
 *  vehicle into its setpoint mode, then hold at s = 0 ("assembling") until `go()`. */
export async function engageFollow(opts: EngageOptions): Promise<void> {
  if (get(followState).phase !== 'idle') return;
  const p = buildPath(opts.plan, { filletRadiusM: Math.max(0, opts.turnRadiusM ?? 0) });
  if (p.vertices.length < 2) { patch({ message: 'Reference route needs at least 2 located waypoints' }); return; }
  maxYawRateDegS = Math.max(1, opts.maxYawRateDegS ?? 12);
  outerRadiusM = opts.slots.reduce((m, sl) => Math.max(m, Math.hypot(sl.forward, sl.right)), 0);
  fHeadingDeg = pointAt(p, 0)?.headingDeg ?? 0;
  formationPathPreview.set(p.vertices.map((v) => ({ lat: v.lat, lon: v.lon })));
  const known = get(vehicles);
  const ordered = orderedVehicles(known);
  const fvs: FollowVehicle[] = [];
  for (const [slotIdx, id] of opts.assignments) {
    const v = known.get(id);
    const slot = opts.slots[slotIdx];
    if (!v || !slot || !vehicleSystem(v)) continue;
    fvs.push({
      vehicleId: id, name: v.name, color: vehicleColorFor(ordered, id), slot, plane: isFixedWingClass(vehicleClassOf(v)),
      targetLat: 0, targetLon: 0, targetAlt: 0, errorM: NaN, inPosition: false, linkLost: false, error: '',
    });
  }
  if (fvs.length < 2) { patch({ message: 'Assign at least 2 vehicles' }); return; }
  path = p;
  lagSince = 0;
  followState.set({ ...IDLE, phase: 'engaging', lengthM: p.lengthM, speedMs: opts.speedMs, headingRelative: opts.headingRelative, vehicles: fvs });
  startTimer();
  // Let setpoints flow first (PX4 rejects OFFBOARD without a live stream), then switch modes.
  await new Promise((r) => setTimeout(r, ENGAGE_STREAM_MS));
  if (get(followState).phase !== 'engaging') return; // stopped meanwhile
  await Promise.all(fvs.map(async (fv) => {
    const v = known.get(fv.vehicleId);
    const m = v ? setpointMode(v) : null;
    if (!v || !m) { markError(fv.vehicleId, 'No GUIDED/OFFBOARD mode for this vehicle'); return; }
    try { await setMode(v, m.main, m.sub); } catch (e) { markError(fv.vehicleId, String(e)); }
  }));
  if (get(followState).phase === 'engaging') patch({ phase: 'assembling' });
}

function markError(vehicleId: string, error: string): void {
  followState.update((cur) => ({ ...cur, vehicles: cur.vehicles.map((x) => (x.vehicleId === vehicleId ? { ...x, error } : x)) }));
}

/** Release the leader along the path. */
export function goFollow(): void {
  const st = get(followState);
  if (st.phase !== 'assembling' && st.phase !== 'paused') return;
  patch({ phase: 'running', startedAt: st.startedAt ?? Date.now(), message: '' });
}

export function pauseFollow(): void {
  if (get(followState).phase === 'running') patch({ phase: 'paused' });
}

export function setFollowSpeed(speedMs: number): void {
  patch({ speedMs: Math.max(0.5, speedMs) });
}

/** Jump the leader to `s` (m along the path) — the progress slider. Holds there until go(). */
export function seekFollow(s: number): void {
  const st = get(followState);
  if (st.phase === 'idle' || st.phase === 'stopping') return;
  patch({ s: Math.min(Math.max(0, s), st.lengthM), phase: st.phase === 'running' ? 'paused' : st.phase === 'finished' ? 'paused' : st.phase });
}

/** Stop streaming and park every vehicle in Loiter/Hold (position hold), then go idle. */
export async function stopFollow(): Promise<void> {
  const st = get(followState);
  if (st.phase === 'idle') return;
  patch({ phase: 'stopping' });
  stopTimer();
  const known = get(vehicles);
  await Promise.all(st.vehicles.map(async (fv) => {
    const v = known.get(fv.vehicleId);
    if (!v) return;
    const m = resolveIntentMode(v, 'hold');
    if (!m) return;
    try { await setMode(v, m.main, m.sub); } catch { /* the FC's own failsafe covers a lost link */ }
  }));
  path = null;
  formationPreview.set(null);
  formationPathPreview.set(null);
  followState.set({ ...IDLE, speedMs: st.speedMs, headingRelative: st.headingRelative });
}

// A vehicle that leaves the registry leaves the formation; with fewer than 2 left the follower stops.
vehicles.subscribe((known) => {
  const st = get(followState);
  if (st.phase === 'idle') return;
  const remaining = st.vehicles.filter((v) => known.has(v.vehicleId));
  if (remaining.length === st.vehicles.length) return;
  if (remaining.length < 2) { void stopFollow(); return; }
  patch({ vehicles: remaining, message: 'A vehicle left the formation' });
});

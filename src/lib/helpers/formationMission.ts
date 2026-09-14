// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Marc Hoffmann (b14ckyy)

// Formation geometry + mission-based formation flight (pure functions).
//
// A formation is a list of SLOTS, each a body-frame offset from a reference: `forward` (m, along the
// path), `right` (m), `up` (m). The reference plan is whatever the mission editor holds; every
// assigned vehicle gets a copy of it with its slot offset applied to each located waypoint. With
// `headingRelative` the offset rotates with the path heading at that waypoint (bearing of the
// incoming/outgoing legs), so the shape is kept around corners; otherwise it is a fixed north-up
// offset. Slot 0 is the reference slot for most shapes (offset 0) — grid and circle centre the shape
// instead, so slot 0 is offset too. See docs/02-design/features/formation-flight.design.md.

import type { ArduWaypoint } from '$lib/stores/missionArdupilot';
import type { FormationShape } from '$lib/stores/settings';
import { cmdHasLocation } from '$lib/helpers/arduCommandCatalog';
import { bearing } from '$lib/utils/geo';
import { metresToLatLonDelta } from '$lib/helpers/missionGeo';

export interface FormationSlot {
  index: number;
  forward: number;
  right: number;
  up: number;
}

export const FORMATION_SHAPES: readonly FormationShape[] = ['line', 'trail', 'vee', 'wedge', 'grid', 'circle'];

/** Slot offsets for `count` vehicles. `spacing` = neighbour distance (m), `altStep` = m per slot index. */
export function formationSlots(shape: FormationShape, count: number, spacing: number, altStep: number): FormationSlot[] {
  const n = Math.max(0, Math.floor(count));
  const s = Math.max(1, spacing);
  const out: FormationSlot[] = [];
  const side = (i: number) => (i % 2 === 1 ? 1 : -1); // odd → right, even → left
  const rank = (i: number) => Math.ceil(i / 2);
  for (let i = 0; i < n; i++) {
    let forward = 0;
    let right = 0;
    switch (shape) {
      case 'line':   // abreast, alternating right/left of the reference
        right = i === 0 ? 0 : rank(i) * s * side(i);
        break;
      case 'trail':  // in line astern
        forward = -i * s;
        break;
      case 'vee':    // reference leads; pairs fan out behind and to the sides
        forward = -rank(i) * s;
        right = i === 0 ? 0 : rank(i) * s * side(i);
        break;
      case 'wedge':  // echelon right
        forward = -i * s;
        right = i * s;
        break;
      case 'grid': { // centred rows
        const cols = Math.max(1, Math.ceil(Math.sqrt(n)));
        const row = Math.floor(i / cols);
        const col = i % cols;
        forward = -row * s;
        right = (col - (cols - 1) / 2) * s;
        break;
      }
      case 'circle': { // evenly around a centre; radius keeps neighbours ≥ spacing apart
        const r = n <= 1 ? 0 : Math.max(s, (s * n) / (2 * Math.PI));
        const a = (2 * Math.PI * i) / Math.max(1, n);
        forward = r * Math.cos(a);
        right = r * Math.sin(a);
        break;
      }
    }
    out.push({ index: i, forward, right, up: i * altStep });
  }
  return out;
}

/** Rotate a body-frame offset by heading `hDeg` (0 = north) into north/east metres. */
export function bodyToNorthEast(forward: number, right: number, hDeg: number): { north: number; east: number } {
  const h = (hDeg * Math.PI) / 180;
  return {
    north: forward * Math.cos(h) - right * Math.sin(h),
    east: forward * Math.sin(h) + right * Math.cos(h),
  };
}

/** Indices of waypoints with a real coordinate. */
function locatedIndices(wps: readonly ArduWaypoint[]): number[] {
  const out: number[] = [];
  wps.forEach((w, i) => { if (cmdHasLocation(w.command) && (w.lat !== 0 || w.lon !== 0)) out.push(i); });
  return out;
}

/** Path heading at located waypoint `k` (position in `loc`): mean of the incoming and outgoing leg
 *  bearings; a single leg at the ends; `fallback` for a one-point plan. */
export function pathHeadingAt(wps: readonly ArduWaypoint[], loc: readonly number[], k: number, fallback = 0): number {
  if (loc.length < 2) return fallback;
  const p = (i: number) => wps[loc[i]];
  const brg = (a: ArduWaypoint, b: ArduWaypoint) => bearing(a.lat / 1e7, a.lon / 1e7, b.lat / 1e7, b.lon / 1e7);
  if (k === 0) return brg(p(0), p(1));
  if (k === loc.length - 1) return brg(p(k - 1), p(k));
  const hIn = brg(p(k - 1), p(k));
  const hOut = brg(p(k), p(k + 1));
  // Circular mean of the two leg headings.
  const x = Math.cos((hIn * Math.PI) / 180) + Math.cos((hOut * Math.PI) / 180);
  const y = Math.sin((hIn * Math.PI) / 180) + Math.sin((hOut * Math.PI) / 180);
  return ((Math.atan2(y, x) * 180) / Math.PI + 360) % 360;
}

/** The reference plan offset into `slot`. Non-located items (DO_ commands, take-off-in-place) copy
 *  through untouched; altitude gets `slot.up`. */
export function formationPlan(base: readonly ArduWaypoint[], slot: FormationSlot, headingRelative: boolean, fallbackHeadingDeg = 0): ArduWaypoint[] {
  const loc = locatedIndices(base);
  const out = base.map((w) => ({ ...w }));
  if (slot.forward === 0 && slot.right === 0 && slot.up === 0) return out;
  loc.forEach((idx, k) => {
    const w = out[idx];
    const h = headingRelative ? pathHeadingAt(base, loc, k, fallbackHeadingDeg) : 0;
    const { north, east } = bodyToNorthEast(slot.forward, slot.right, h);
    const { dLat, dLon } = metresToLatLonDelta(w.lat / 1e7, north, east);
    out[idx] = { ...w, lat: w.lat + Math.round(dLat * 1e7), lon: w.lon + Math.round(dLon * 1e7), alt: w.alt + slot.up };
  });
  return out;
}

// ── Assembly (rendezvous) stage ──────────────────────────────────────────────
// Formation needs every craft IN its slot before the route starts. The plan gets an assembly item —
// NAV_LOITER_UNLIM at the slot-offset position of the first route waypoint — inserted after any
// take-off items. "Assemble" = start the mission (each craft flies to its slot and waits there for as
// long as it takes); the GCS watches the slots fill; "Go" = MISSION_SET_CURRENT to the first route
// item on every craft at once. Works on ArduPilot and PX4 without GCS involvement in between.

export interface FormationBuildOptions {
  /** Insert the assembly loiter (default true). */
  assemble?: boolean;
  /** Loiter radius (m) for fixed-wing classes; multirotors hover on the point. */
  loiterRadiusM?: number;
  /** Insert a DO_CHANGE_SPEED right after assembly so every craft flies the route at one speed (m/s). 0/undefined = none. */
  speedMs?: number;
  /** DO_CHANGE_SPEED type: 0 = airspeed (fixed-wing), 1 = ground speed. */
  speedType?: 0 | 1;
}

export interface FormationBuild {
  plan: ArduWaypoint[];
  /** Index (in `plan`) of the assembly loiter, or -1 when none was inserted. */
  assemblyIdx: number;
  /** Index (in `plan`) of the first route item after assembly — the MISSION_SET_CURRENT target for "Go". */
  goIdx: number;
  /** Assembly point (degrees / m) for the in-position monitor, or null. */
  assembly: { lat: number; lon: number; alt: number; frame: number } | null;
}

const CMD_NAV_TAKEOFF = 22;
const CMD_NAV_LOITER_UNLIM = 17;
const CMD_DO_CHANGE_SPEED = 178;

/** Slot-offset copy of the reference plan plus the assembly stage. */
export function buildFormationPlan(base: readonly ArduWaypoint[], slot: FormationSlot, headingRelative: boolean, fallbackHeadingDeg: number, opts: FormationBuildOptions = {}): FormationBuild {
  const offset = formationPlan(base, slot, headingRelative, fallbackHeadingDeg);
  const assemble = opts.assemble !== false;
  // First route item = first located, non-take-off command.
  const firstRoute = offset.findIndex((w) => cmdHasLocation(w.command) && w.command !== CMD_NAV_TAKEOFF && (w.lat !== 0 || w.lon !== 0));
  if (!assemble || firstRoute < 0) {
    return { plan: offset, assemblyIdx: -1, goIdx: Math.max(0, firstRoute), assembly: null };
  }
  const ref = offset[firstRoute];
  const loiter: ArduWaypoint = {
    command: CMD_NAV_LOITER_UNLIM, frame: ref.frame,
    param1: 0, param2: 0, param3: opts.loiterRadiusM ?? 0, param4: 0,
    lat: ref.lat, lon: ref.lon, alt: ref.alt, autocontinue: true,
  };
  // Speed BEFORE the loiter: a DO_ item runs once the preceding NAV completes, so it takes effect on
  // the way to the assembly point and stays for the route. (After the loiter it would be skipped by
  // the "Go" jump straight to the first route item.)
  const inserted: ArduWaypoint[] = [];
  if (opts.speedMs && opts.speedMs > 0) {
    inserted.push({
      command: CMD_DO_CHANGE_SPEED, frame: 0,
      param1: opts.speedType ?? 1, param2: opts.speedMs, param3: -1, param4: 0,
      lat: 0, lon: 0, alt: 0, autocontinue: true,
    });
  }
  inserted.push(loiter);
  const plan = [...offset.slice(0, firstRoute), ...inserted, ...offset.slice(firstRoute)];
  return {
    plan,
    assemblyIdx: firstRoute + inserted.length - 1,
    goIdx: firstRoute + inserted.length,
    assembly: { lat: ref.lat / 1e7, lon: ref.lon / 1e7, alt: ref.alt, frame: ref.frame },
  };
}

/** Where each slot sits around a reference point with heading `hDeg` — the map preview. */
export function slotPositions(slots: readonly FormationSlot[], lat: number, lon: number, hDeg: number, headingRelative: boolean): { index: number; lat: number; lon: number; up: number }[] {
  return slots.map((s) => {
    const { north, east } = bodyToNorthEast(s.forward, s.right, headingRelative ? hDeg : 0);
    const { dLat, dLon } = metresToLatLonDelta(lat, north, east);
    return { index: s.index, lat: lat + dLat, lon: lon + dLon, up: s.up };
  });
}

/** Smallest horizontal distance between any two slots (m) — sanity check against the separation alarm. */
export function minSlotSpacing(slots: readonly FormationSlot[]): number {
  let min = Infinity;
  for (let i = 0; i < slots.length; i++) {
    for (let j = i + 1; j < slots.length; j++) {
      const d = Math.hypot(slots[i].forward - slots[j].forward, slots[i].right - slots[j].right);
      if (d < min) min = d;
    }
  }
  return min;
}

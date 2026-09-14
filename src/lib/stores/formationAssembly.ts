// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Marc Hoffmann (b14ckyy)

// Formation assembly monitor: after a formation upload, each vehicle has an assembly (loiter) point
// and a "go" item index. This store watches the fleet telemetry and says, per vehicle, whether it is
// IN its slot (close to the assembly point at the right height and, for multirotors, stopped) so the
// operator — or the auto-go option — releases the route only when the whole formation is in place.
// See docs/02-design/features/formation-flight.design.md §7.

import { writable, derived, get } from 'svelte/store';
import { allTelemetry } from '$lib/stores/telemetry';
import { vehicles } from '$lib/stores/vehicles';
import { isValidGpsCoordinate } from '$lib/helpers/telemetry';
import { haversineDistance } from '$lib/utils/geo';
import { vehicleClassOf, isFixedWingClass, isVehicleArmed, linkState } from '$lib/helpers/fleetStatus';

export interface AssemblyTarget {
  vehicleId: string;
  lat: number;
  lon: number;
  /** Assembly altitude (m) in the plan's frame — compared against relative altitude for REL plans. */
  alt: number;
  frame: number;
  loiterRadiusM: number;
  /** MISSION_SET_CURRENT target (frontend index, home slot NOT included). */
  goIdx: number;
  /** Frontend index of the assembly loiter item. */
  assemblyIdx: number;
}

export interface AssemblySet {
  byVehicle: ReadonlyMap<string, AssemblyTarget>;
  uploadedAt: number;
}

/** The last uploaded formation's assembly targets (null = nothing uploaded this session). */
export const formationAssembly = writable<AssemblySet | null>(null);

export type AssemblyState = 'noTelemetry' | 'notArmed' | 'enRoute' | 'inPosition';

export interface AssemblyStatus {
  vehicleId: string;
  state: AssemblyState;
  distanceM: number;
  altDiffM: number;
}

// Tolerances. Multirotors hover on the point; fixed-wing craft circle it, so their box is the loiter
// radius plus a margin. Altitude tolerance is looser for planes (TECS overshoot).
const COPTER_RADIUS_M = 5;
const COPTER_ALT_M = 3;
const COPTER_STOPPED_MS = 1.5;
const PLANE_MARGIN_M = 15;
const PLANE_ALT_M = 10;
const MAV_FRAME_GLOBAL = 0;

export function setFormationAssembly(targets: AssemblyTarget[]): void {
  const m = new Map<string, AssemblyTarget>();
  for (const t of targets) m.set(t.vehicleId, t);
  formationAssembly.set({ byVehicle: m, uploadedAt: Date.now() });
}

export function clearFormationAssembly(): void {
  formationAssembly.set(null);
}

/** Per-vehicle assembly status, recomputed on every fleet telemetry flush (~10 Hz, cheap). */
export const assemblyStatus = derived(
  [formationAssembly, allTelemetry, vehicles],
  ([set, telem, known]): ReadonlyMap<string, AssemblyStatus> => {
    const out = new Map<string, AssemblyStatus>();
    if (!set) return out;
    const now = Date.now();
    for (const t of set.byVehicle.values()) {
      const v = known.get(t.vehicleId);
      const tv = telem.get(t.vehicleId);
      if (!v || !tv || linkState(tv, now) === 'lost' || !isValidGpsCoordinate(tv.lat, tv.lon)) {
        out.set(t.vehicleId, { vehicleId: t.vehicleId, state: 'noTelemetry', distanceM: NaN, altDiffM: NaN });
        continue;
      }
      const d = haversineDistance(tv.lat, tv.lon, t.lat, t.lon);
      // REL / terrain frames → compare with the relative altitude; AMSL → with altMsl.
      const cur = t.frame === MAV_FRAME_GLOBAL ? tv.altMsl : tv.altitude;
      const dAlt = Math.abs(cur - t.alt);
      if (!isVehicleArmed(tv)) {
        out.set(t.vehicleId, { vehicleId: t.vehicleId, state: 'notArmed', distanceM: d, altDiffM: dAlt });
        continue;
      }
      const plane = isFixedWingClass(vehicleClassOf(v));
      const inPos = plane
        ? d <= Math.max(t.loiterRadiusM, 1) + PLANE_MARGIN_M && dAlt <= PLANE_ALT_M
        : d <= COPTER_RADIUS_M && dAlt <= COPTER_ALT_M && tv.groundSpeed <= COPTER_STOPPED_MS;
      out.set(t.vehicleId, { vehicleId: t.vehicleId, state: inPos ? 'inPosition' : 'enRoute', distanceM: d, altDiffM: dAlt });
    }
    return out;
  },
);

/** True when every assembled vehicle reports `inPosition` (and there is at least one). */
export const allInPosition = derived(assemblyStatus, (m) => m.size > 0 && [...m.values()].every((s) => s.state === 'inPosition'));

/** `[inPosition, total]` for the progress line. */
export const assemblyProgress = derived(assemblyStatus, (m) => {
  let n = 0;
  for (const s of m.values()) if (s.state === 'inPosition') n++;
  return [n, m.size] as const;
});

// Vehicles that leave the registry drop out of the assembly set.
vehicles.subscribe((known) => {
  const set = get(formationAssembly);
  if (!set) return;
  let changed = false;
  const m = new Map<string, AssemblyTarget>();
  for (const [id, t] of set.byVehicle) { if (known.has(id)) m.set(id, t); else changed = true; }
  if (changed) formationAssembly.set(m.size ? { byVehicle: m, uploadedAt: set.uploadedAt } : null);
});

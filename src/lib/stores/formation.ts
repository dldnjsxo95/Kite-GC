// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Marc Hoffmann (b14ckyy)

// Formation flight state: which vehicle flies which slot (session-scoped — vehicle ids are per
// connection), and the map preview of the slot layout. Shape/spacing live in `settings.formation` so
// they persist. See docs/02-design/features/formation-flight.design.md.

import { writable, derived, get } from 'svelte/store';
import { settings } from '$lib/stores/settings';
import { vehicles } from '$lib/stores/vehicles';
import { allTelemetry } from '$lib/stores/telemetry';
import { isValidGpsCoordinate } from '$lib/helpers/telemetry';
import { haversineDistance } from '$lib/utils/geo';
import { formationSlots, slotPositions, type FormationSlot } from '$lib/helpers/formationMission';

/** slot index → vehicle id. */
export const formationAssignments = writable<ReadonlyMap<number, string>>(new Map());

/** How many slots the layout has (defaults to the number of assigned vehicles, min 2). */
export const formationSlotCount = writable<number>(2);

export const formationSlotsStore = derived(
  [settings, formationSlotCount],
  ([s, n]): FormationSlot[] => formationSlots(s.formation.shape, n, s.formation.spacingM, s.formation.altStepM),
);

export function assignSlot(slot: number, vehicleId: string | null): void {
  formationAssignments.update((m) => {
    const next = new Map(m);
    // A vehicle flies one slot only.
    for (const [k, v] of next) if (v === vehicleId) next.delete(k);
    if (vehicleId) next.set(slot, vehicleId); else next.delete(slot);
    return next;
  });
}

export function clearAssignments(): void {
  formationAssignments.set(new Map());
}

/** Fill the slots with `ids` in order (slot 0 first). */
export function assignInOrder(ids: readonly string[]): void {
  const m = new Map<number, string>();
  ids.forEach((id, i) => m.set(i, id));
  formationAssignments.set(m);
  if (ids.length > get(formationSlotCount)) formationSlotCount.set(ids.length);
}

/** Greedy nearest-slot assignment from the vehicles' current positions to the slot layout previewed
 *  around (`lat`, `lon`, `hDeg`). Minimises total repositioning for the first move into formation. */
export function autoAssign(ids: readonly string[], lat: number, lon: number, hDeg: number): void {
  const s = get(settings).formation;
  const slots = formationSlots(s.shape, ids.length, s.spacingM, s.altStepM);
  const pos = slotPositions(slots, lat, lon, hDeg, s.headingRelative);
  const telem = get(allTelemetry);
  const pending = ids.filter((id) => { const tv = telem.get(id); return tv && isValidGpsCoordinate(tv.lat, tv.lon); });
  const noFix = ids.filter((id) => !pending.includes(id));
  const result = new Map<number, string>();
  const freeSlots = new Set(pos.map((p) => p.index));
  // Repeatedly take the globally closest (vehicle, slot) pair.
  while (pending.length && freeSlots.size) {
    let best: { id: string; slot: number; d: number } | null = null;
    for (const id of pending) {
      const tv = telem.get(id)!;
      for (const slot of freeSlots) {
        const p = pos[slot];
        const d = haversineDistance(tv.lat, tv.lon, p.lat, p.lon);
        if (!best || d < best.d) best = { id, slot, d };
      }
    }
    if (!best) break;
    result.set(best.slot, best.id);
    freeSlots.delete(best.slot);
    pending.splice(pending.indexOf(best.id), 1);
  }
  for (const id of noFix) { const slot = [...freeSlots][0]; if (slot == null) break; result.set(slot, id); freeSlots.delete(slot); }
  formationAssignments.set(result);
  if (ids.length > get(formationSlotCount)) formationSlotCount.set(ids.length);
}

// Drop assignments of vehicles that left the registry.
vehicles.subscribe((known) => {
  const m = get(formationAssignments);
  let changed = false;
  const next = new Map<number, string>();
  for (const [k, v] of m) { if (known.has(v)) next.set(k, v); else changed = true; }
  if (changed) formationAssignments.set(next);
});

// ── Map preview ─────────────────────────────────────────────────────────────

export interface FormationPreviewSlot {
  index: number;
  lat: number;
  lon: number;
  up: number;
  vehicleId: string | null;
  color: string | null;
  name: string | null;
}

/** Ghost slot layout for the map (null = preview off). Written by the formation panel. */
export const formationPreview = writable<FormationPreviewSlot[] | null>(null);

/** The follower's smoothed (corner-rounded) reference track for the map (null = not engaged). */
export const formationPathPreview = writable<{ lat: number; lon: number }[] | null>(null);

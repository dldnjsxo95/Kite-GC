// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Marc Hoffmann (b14ckyy)

// Fleet separation monitor — the safety net under formation flight. Every second, every pair of
// connected vehicles with a fresh fix is checked: current horizontal distance, and the distance ~5 s
// ahead assuming each keeps its ground speed / course. A pair under `settings.formation.minSeparationM`
// (now or projected) raises an alert (fleet panel banner) and a warning toast, rate-limited per pair.
// Self-starting on import (+page imports it for its side effect).

import { writable, get } from 'svelte/store';
import { allTelemetry } from '$lib/stores/telemetry';
import { vehicles } from '$lib/stores/vehicles';
import { settings } from '$lib/stores/settings';
import { pushLocalStatus } from '$lib/stores/statusText';
import { isValidGpsCoordinate } from '$lib/helpers/telemetry';
import { haversineDistance } from '$lib/utils/geo';
import { metresToLatLonDelta } from '$lib/helpers/missionGeo';
import { linkState } from '$lib/helpers/fleetStatus';

export interface SeparationAlert {
  a: string;
  b: string;
  aName: string;
  bName: string;
  distanceM: number;
  projectedM: number;
  /** Vertical separation (m), from relative altitude. */
  verticalM: number;
}

export const separationAlerts = writable<SeparationAlert[]>([]);

const CHECK_INTERVAL_MS = 1000;
const LOOKAHEAD_S = 5;
const TOAST_GAP_MS = 10_000;
/** Vertical gap that makes a horizontal conflict a non-issue (multirotor stack). */
const VERTICAL_CLEAR_M = 15;

let lastCheck = 0;
const lastToast = new Map<string, number>();

function projected(lat: number, lon: number, courseDeg: number, speed: number): { lat: number; lon: number } {
  const d = speed * LOOKAHEAD_S;
  const c = (courseDeg * Math.PI) / 180;
  const { dLat, dLon } = metresToLatLonDelta(lat, d * Math.cos(c), d * Math.sin(c));
  return { lat: lat + dLat, lon: lon + dLon };
}

function check(): void {
  const now = Date.now();
  if (now - lastCheck < CHECK_INTERVAL_MS) return;
  lastCheck = now;
  const known = get(vehicles);
  if (known.size < 2) { if (get(separationAlerts).length) separationAlerts.set([]); return; }
  const telem = get(allTelemetry);
  const min = Math.max(1, get(settings).formation.minSeparationM);
  const ids = [...known.keys()].filter((id) => {
    const tv = telem.get(id);
    return tv && linkState(tv, now) !== 'lost' && isValidGpsCoordinate(tv.lat, tv.lon) && tv.fixType >= 3;
  });
  const alerts: SeparationAlert[] = [];
  for (let i = 0; i < ids.length; i++) {
    for (let j = i + 1; j < ids.length; j++) {
      const ta = telem.get(ids[i])!;
      const tb = telem.get(ids[j])!;
      const d = haversineDistance(ta.lat, ta.lon, tb.lat, tb.lon);
      const pa = projected(ta.lat, ta.lon, ta.course, ta.groundSpeed);
      const pb = projected(tb.lat, tb.lon, tb.course, tb.groundSpeed);
      const dp = haversineDistance(pa.lat, pa.lon, pb.lat, pb.lon);
      const dv = Math.abs(ta.altitude - tb.altitude);
      if (dv >= VERTICAL_CLEAR_M) continue;
      if (d < min || dp < min) {
        const va = known.get(ids[i])!;
        const vb = known.get(ids[j])!;
        alerts.push({ a: ids[i], b: ids[j], aName: va.name, bName: vb.name, distanceM: d, projectedM: dp, verticalM: dv });
        const key = `${ids[i]}|${ids[j]}`;
        if (now - (lastToast.get(key) ?? 0) > TOAST_GAP_MS) {
          lastToast.set(key, now);
          pushLocalStatus(4, `Separation ${va.name} ↔ ${vb.name}: ${d.toFixed(0)} m (${dp.toFixed(0)} m in ${LOOKAHEAD_S} s)`);
        }
      }
    }
  }
  const cur = get(separationAlerts);
  if (alerts.length || cur.length) separationAlerts.set(alerts);
}

allTelemetry.subscribe(() => check());

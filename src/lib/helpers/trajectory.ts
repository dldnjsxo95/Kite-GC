// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Marc Hoffmann (b14ckyy)

// Arc-length parametrised path for the GCS-driven formation follower. The reference route (the
// mission editor's located, non-take-off waypoints) becomes a polyline; `pointAt(s)` gives the
// position, altitude and heading `s` metres along it. A virtual leader advances along `s` with time;
// every vehicle's target is that point plus its slot offset — one clock, one path, the shape holds.
// See docs/02-design/features/formation-flight.design.md §9.

import type { ArduWaypoint } from '$lib/stores/missionArdupilot';
import { cmdHasLocation } from '$lib/helpers/arduCommandCatalog';
import { haversineDistance, bearing } from '$lib/utils/geo';

const CMD_NAV_TAKEOFF = 22;

export interface PathVertex {
  lat: number;
  lon: number;
  alt: number;
  frame: number;
  /** Cumulative distance from the start (m). */
  s: number;
}

export interface PathPoint {
  lat: number;
  lon: number;
  alt: number;
  /** Track heading at this point (deg, 0 = north). */
  headingDeg: number;
  /** Index of the segment the point lies on (0-based), -1 for an empty path. */
  segment: number;
}

export interface FormationPath {
  vertices: PathVertex[];
  lengthM: number;
  frame: number;
}

export interface BuildPathOptions {
  /** Round every corner with a circular arc of this radius (m). 0 = sharp corners. The radius is
   *  reduced automatically where the adjacent legs are too short for it. */
  filletRadiusM?: number;
  /** Arc sampling step (m). */
  arcStepM?: number;
}

interface RawVertex { lat: number; lon: number; alt: number; frame: number }

const EARTH_R = 6_371_000;
const DEG = Math.PI / 180;

/** Build the path from a plan. Located waypoints only (take-off excluded, 0,0 placeholders skipped).
 *  With a fillet radius the corners become arcs, so the track heading — and with it the formation's
 *  rotation — changes gradually instead of stepping at each waypoint. */
export function buildPath(plan: readonly ArduWaypoint[], opts: BuildPathOptions = {}): FormationPath {
  const raw: RawVertex[] = [];
  for (const w of plan) {
    if (!cmdHasLocation(w.command) || w.command === CMD_NAV_TAKEOFF || (w.lat === 0 && w.lon === 0)) continue;
    raw.push({ lat: w.lat / 1e7, lon: w.lon / 1e7, alt: w.alt, frame: w.frame });
  }
  const frame = raw[0]?.frame ?? 3;
  const pts = (opts.filletRadiusM ?? 0) > 0 && raw.length >= 3 ? filletCorners(raw, opts.filletRadiusM!, opts.arcStepM ?? 2) : raw;
  const vertices: PathVertex[] = [];
  let s = 0;
  for (const p of pts) {
    if (vertices.length) {
      const q = vertices[vertices.length - 1];
      s += haversineDistance(q.lat, q.lon, p.lat, p.lon);
    }
    vertices.push({ lat: p.lat, lon: p.lon, alt: p.alt, frame: p.frame, s });
  }
  return { vertices, lengthM: s, frame };
}

/** Replace each interior corner by a tangent circular arc (local flat-earth geometry around the
 *  first vertex — fine at mission scale). Tangent length t = R·tan(θ/2) is capped at half the shorter
 *  adjacent leg so neighbouring arcs never overlap; the radius shrinks accordingly. */
function filletCorners(raw: RawVertex[], radiusM: number, stepM: number): RawVertex[] {
  const lat0 = raw[0].lat;
  const lon0 = raw[0].lon;
  const cos0 = Math.cos(lat0 * DEG);
  const toXY = (v: RawVertex) => ({ x: (v.lon - lon0) * DEG * EARTH_R * cos0, y: (v.lat - lat0) * DEG * EARTH_R });
  const toLL = (x: number, y: number) => ({ lat: lat0 + y / (DEG * EARTH_R), lon: lon0 + x / (DEG * EARTH_R * cos0) });
  const xy = raw.map(toXY);
  const out: RawVertex[] = [raw[0]];
  for (let i = 1; i < raw.length - 1; i++) {
    const P = xy[i];
    const A0 = xy[i - 1];
    const B0 = xy[i + 1];
    const d1 = { x: P.x - A0.x, y: P.y - A0.y };
    const d2 = { x: B0.x - P.x, y: B0.y - P.y };
    const len1 = Math.hypot(d1.x, d1.y);
    const len2 = Math.hypot(d2.x, d2.y);
    if (len1 < 1e-3 || len2 < 1e-3) { out.push(raw[i]); continue; }
    const u1 = { x: d1.x / len1, y: d1.y / len1 };
    const u2 = { x: d2.x / len2, y: d2.y / len2 };
    const cross = u1.x * u2.y - u1.y * u2.x;
    const dot = u1.x * u2.x + u1.y * u2.y;
    const theta = Math.atan2(cross, dot); // signed turn angle, + = left (CCW)
    const aTheta = Math.abs(theta);
    if (aTheta < 1 * DEG || aTheta > 179 * DEG) { out.push(raw[i]); continue; } // straight or hairpin: keep the vertex
    let t = radiusM * Math.tan(aTheta / 2);
    const tMax = Math.min(len1, len2) / 2;
    let R = radiusM;
    if (t > tMax) { t = tMax; R = t / Math.tan(aTheta / 2); }
    const A = { x: P.x - u1.x * t, y: P.y - u1.y * t };
    const sgn = theta > 0 ? 1 : -1;
    const n1 = { x: -u1.y * sgn, y: u1.x * sgn }; // normal towards the turn centre
    const C = { x: A.x + n1.x * R, y: A.y + n1.y * R };
    const a0 = Math.atan2(A.y - C.y, A.x - C.x);
    const steps = Math.max(2, Math.ceil((aTheta * R) / Math.max(0.5, stepM)));
    const altA = raw[i - 1].alt + (raw[i].alt - raw[i - 1].alt) * (1 - t / len1);
    const altB = raw[i].alt + (raw[i + 1].alt - raw[i].alt) * (t / len2);
    for (let k = 0; k <= steps; k++) {
      const f = k / steps;
      const a = a0 + sgn * aTheta * f;
      const ll = toLL(C.x + R * Math.cos(a), C.y + R * Math.sin(a));
      out.push({ lat: ll.lat, lon: ll.lon, alt: altA + (altB - altA) * f, frame: raw[i].frame });
    }
  }
  out.push(raw[raw.length - 1]);
  return out;
}

/** Track curvature at `s` (rad/m, ≥ 0) from the heading change across ±`ds`. */
export function curvatureAt(path: FormationPath, s: number, ds = 4): number {
  if (path.lengthM <= 0) return 0;
  const a = pointAt(path, Math.max(0, s - ds));
  const b = pointAt(path, Math.min(path.lengthM, s + ds));
  if (!a || !b) return 0;
  let dh = b.headingDeg - a.headingDeg;
  while (dh > 180) dh -= 360;
  while (dh < -180) dh += 360;
  const span = Math.min(path.lengthM, s + ds) - Math.max(0, s - ds);
  return span > 0 ? Math.abs(dh * DEG) / span : 0;
}

/** Shortest signed difference b - a in degrees, in (-180, 180]. */
export function headingDelta(a: number, b: number): number {
  let d = b - a;
  while (d > 180) d -= 360;
  while (d <= -180) d += 360;
  return d;
}

/** Position on the path at arc length `s` (clamped to [0, length]). */
export function pointAt(path: FormationPath, s: number): PathPoint | null {
  const v = path.vertices;
  if (v.length === 0) return null;
  if (v.length === 1) return { lat: v[0].lat, lon: v[0].lon, alt: v[0].alt, headingDeg: 0, segment: 0 };
  const sc = Math.min(Math.max(0, s), path.lengthM);
  let i = 0;
  while (i < v.length - 2 && v[i + 1].s <= sc) i++;
  const a = v[i];
  const b = v[i + 1];
  const segLen = b.s - a.s;
  const f = segLen > 0 ? (sc - a.s) / segLen : 0;
  return {
    lat: a.lat + (b.lat - a.lat) * f,
    lon: a.lon + (b.lon - a.lon) * f,
    alt: a.alt + (b.alt - a.alt) * f,
    headingDeg: bearing(a.lat, a.lon, b.lat, b.lon),
    segment: i,
  };
}

/** NED velocity (m/s) for speed `v` along heading `hDeg` (level flight). */
export function velocityNed(v: number, hDeg: number): [number, number, number] {
  const h = (hDeg * Math.PI) / 180;
  return [v * Math.cos(h), v * Math.sin(h), 0];
}

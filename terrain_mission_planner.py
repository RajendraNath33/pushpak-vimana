#!/usr/bin/env python3
"""
Pushpak Vimana - Terrain-aware mission planner (hilly terrain)
===============================================================

Generates ArduPilot-compatible mission files ("QGC WPL 110", loadable in
Mission Planner and QGroundControl) for the two operating modes of the
Pushpak Vimana dual-purpose drone:

  delivery : healthcare logistics leg (hospital -> PHC). Builds a terrain-
             clearance altitude profile, limits climb/descent gradients,
             simplifies it to a minimal waypoint set that never dips below
             the required clearance, runs an energy/reserve check and
             computes a safe RTL altitude.
  spray    : agricultural coverage (boustrophedon / lawn-mower) over a field
             polygon, using terrain-relative waypoints (MAV_FRAME 10) so
             ArduPilot holds a constant spray height above the ground.

Only the Python standard library is required. A DEM can be supplied as an
ESRI ASCII grid in geographic coordinates (EPSG:4326); otherwise a synthetic
hill DEM is used so the tool runs anywhere (for SITL demos and unit tests).

SAFETY NOTICE: research/educational software. Output must be reviewed in the
GCS, simulated in ArduPilot SITL, and flown only in compliance with DGCA
Drone Rules and Digital Sky airspace permissions. Never rely on this script
as the sole source of obstacle clearance (wires, towers and trees are not in
any DEM).

Examples
--------
  python terrain_mission_planner.py delivery \
      --origin 29.2183,79.5130 --dest 29.2450,79.5400 --payload-kg 4 --out demo_delivery

  python terrain_mission_planner.py spray --rate-l-ha 10 --swath 4 --out demo_spray
"""
from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

R_EARTH = 6_371_000.0
G = 9.81
LatLon = Tuple[float, float]

# MAVLink constants used in the mission file
CMD_WAYPOINT, CMD_RTL, CMD_LAND, CMD_TAKEOFF = 16, 20, 21, 22
CMD_CHANGE_SPEED, CMD_SET_RELAY = 178, 181
FRAME_GLOBAL_REL_ALT, FRAME_TERRAIN_ALT = 3, 10


# --------------------------------------------------------------------------
# Geometry helpers
# --------------------------------------------------------------------------
def haversine(a: LatLon, b: LatLon) -> float:
    """Great-circle distance in metres."""
    la1, lo1, la2, lo2 = map(math.radians, (*a, *b))
    h = math.sin((la2 - la1) / 2) ** 2 + math.cos(la1) * math.cos(la2) * math.sin((lo2 - lo1) / 2) ** 2
    return 2 * R_EARTH * math.asin(math.sqrt(h))


def lerp_latlon(a: LatLon, b: LatLon, t: float) -> LatLon:
    """Linear interpolation (adequate for the few-km legs flown by this drone)."""
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)


def to_local(origin: LatLon, p: LatLon) -> Tuple[float, float]:
    """lat/lon -> local east/north metres (equirectangular, small areas)."""
    x = math.radians(p[1] - origin[1]) * R_EARTH * math.cos(math.radians(origin[0]))
    y = math.radians(p[0] - origin[0]) * R_EARTH
    return x, y


def to_latlon(origin: LatLon, x: float, y: float) -> LatLon:
    lat = origin[0] + math.degrees(y / R_EARTH)
    lon = origin[1] + math.degrees(x / (R_EARTH * math.cos(math.radians(origin[0]))))
    return lat, lon


def polygon_area_m2(pts: Sequence[Tuple[float, float]]) -> float:
    s = 0.0
    for i in range(len(pts)):
        x1, y1 = pts[i]
        x2, y2 = pts[(i + 1) % len(pts)]
        s += x1 * y2 - x2 * y1
    return abs(s) / 2.0


# --------------------------------------------------------------------------
# Terrain models
# --------------------------------------------------------------------------
class SyntheticDEM:
    """Demo terrain: rising ground to the north-east plus two ridges."""

    def __init__(self, lat0: float = 29.20, lon0: float = 79.50, base: float = 1450.0):
        self.lat0, self.lon0, self.base = lat0, lon0, base

    def elevation(self, lat: float, lon: float) -> float:
        x, y = to_local((self.lat0, self.lon0), (lat, lon))
        slope = 0.05 * y + 0.02 * x
        ridge1 = 280.0 * math.exp(-(((x - 3500.0) / 650.0) ** 2))
        ridge2 = 180.0 * math.exp(-(((y - 5200.0) / 500.0) ** 2))
        undulation = 25.0 * math.sin(x / 400.0) * math.cos(y / 520.0)
        return self.base + slope + ridge1 + ridge2 + undulation


class AsciiGridDEM:
    """ESRI ASCII grid DEM (EPSG:4326, cellsize in degrees), bilinear sampling."""

    def __init__(self, path: str):
        hdr, rows = {}, []
        with open(path, "r", encoding="utf-8") as fh:
            for line in fh:
                parts = line.split()
                if not parts:
                    continue
                if parts[0].lower() in ("ncols", "nrows", "xllcorner", "yllcorner",
                                        "xllcenter", "yllcenter", "cellsize", "nodata_value"):
                    hdr[parts[0].lower()] = float(parts[1])
                else:
                    rows.append([float(v) for v in parts])
        self.nc, self.nr, self.cs = int(hdr["ncols"]), int(hdr["nrows"]), hdr["cellsize"]
        self.nodata = hdr.get("nodata_value", -9999.0)
        self.xll = hdr.get("xllcorner", hdr.get("xllcenter", 0.0) - self.cs / 2)
        self.yll = hdr.get("yllcorner", hdr.get("yllcenter", 0.0) - self.cs / 2)
        self.rows = rows

    def _z(self, c: int, r: int) -> float:
        c = min(max(c, 0), self.nc - 1)
        r = min(max(r, 0), self.nr - 1)
        v = self.rows[r][c]
        return 0.0 if v == self.nodata else v

    def elevation(self, lat: float, lon: float) -> float:
        fc = (lon - self.xll) / self.cs - 0.5
        fr = (self.nr - 1) - ((lat - self.yll) / self.cs - 0.5)
        c0, r0 = int(math.floor(fc)), int(math.floor(fr))
        tc, tr = fc - c0, fr - r0
        top = self._z(c0, r0) * (1 - tc) + self._z(c0 + 1, r0) * tc
        bot = self._z(c0, r0 + 1) * (1 - tc) + self._z(c0 + 1, r0 + 1) * tc
        return top * (1 - tr) + bot * tr


# --------------------------------------------------------------------------
# Aircraft model
# --------------------------------------------------------------------------
@dataclass
class Aircraft:
    empty_kg: float = 9.5            # airframe + batteries (no payload)
    cruise_ms: float = 12.0          # planned airspeed
    wind_ms: float = 5.0             # assumed headwind (conservative, both legs)
    max_climb_deg: float = 20.0      # path-gradient limit, climbing
    max_descent_deg: float = 15.0    # path-gradient limit, descending
    hover_w_per_kg: float = 130.0    # electrical W per kg of AUW (calibrate on bench!)
    battery_wh: float = 780.0        # usable pack energy (e.g. 12S 22 Ah x ~80%)
    reserve_frac: float = 0.25       # mandatory landing reserve
    climb_eff: float = 0.60          # overall climb efficiency


# --------------------------------------------------------------------------
# Delivery planning
# --------------------------------------------------------------------------
@dataclass
class Leg:
    waypoints: List[Tuple[LatLon, float]]   # (lat/lon, altitude metres above HOME elevation)
    home_elev: float
    distance_m: float                        # 3-D path length
    climb_m: float
    max_terrain: float
    rtl_alt_rel: float                       # safe RTL altitude above home (m)


def _window_max(v: Sequence[float], w: int) -> List[float]:
    n = len(v)
    return [max(v[max(0, i - w): min(n, i + w + 1)]) for i in range(n)]


def simplify_upper(s: Sequence[float], prof: Sequence[float], tol: float) -> List[int]:
    """Keep a minimal set of profile points so the piecewise-linear path never
    falls more than `tol` metres below the required profile (clearance-safe)."""
    keep = [0, len(s) - 1]
    while True:
        worst, worst_err = -1, tol
        ks = sorted(keep)
        for a, b in zip(ks, ks[1:]):
            for i in range(a + 1, b):
                t = (s[i] - s[a]) / (s[b] - s[a])
                err = prof[i] - (prof[a] + t * (prof[b] - prof[a]))
                if err > worst_err:
                    worst, worst_err = i, err
        if worst < 0:
            return sorted(keep)
        keep.append(worst)


def plan_leg(dem, origin: LatLon, dest: LatLon, ac: Aircraft, clearance: float = 60.0,
             step: float = 25.0, approach_agl: float = 30.0, tol: float = 3.0) -> Leg:
    dist = haversine(origin, dest)
    n = max(3, int(math.ceil(dist / step)) + 1)
    ds = dist / (n - 1)
    pts = [lerp_latlon(origin, dest, i / (n - 1)) for i in range(n)]
    s = [i * ds for i in range(n)]
    ground = [dem.elevation(*p) for p in pts]

    # 1) required altitude (AMSL): terrain + clearance, widened +/-2 samples so
    #    terrain between samples and small DEM errors are covered
    req = _window_max([g + clearance for g in ground], 2)
    req[-1] = max(ground[-1] + approach_agl, min(req[-1], ground[-1] + clearance))
    prof = req[:]

    # 2) gradient limiting: look-ahead for climbs, then forward pass for descents
    up, down = math.tan(math.radians(ac.max_climb_deg)), math.tan(math.radians(ac.max_descent_deg))
    for i in range(n - 2, -1, -1):
        prof[i] = max(prof[i], prof[i + 1] - up * ds)
    for i in range(1, n):
        prof[i] = max(prof[i], prof[i - 1] - down * ds)

    # 3) waypoint simplification
    keep = simplify_upper(s, prof, tol)
    home_elev = ground[0]
    wps = [(pts[i], prof[i] - home_elev) for i in keep]

    # 4) 3-D length, climb, RTL altitude
    length, climb = 0.0, 0.0
    for (p1, a1), (p2, a2) in zip(wps, wps[1:]):
        h = haversine(p1, p2)
        length += math.hypot(h, a2 - a1)
        climb += max(0.0, a2 - a1)
    climb += wps[0][1]  # initial vertical climb from takeoff
    max_terrain = max(ground)
    rtl_alt_rel = max_terrain + clearance - home_elev
    return Leg(wps, home_elev, length, climb, max_terrain, rtl_alt_rel)


def energy_wh(leg: Leg, mass_kg: float, ac: Aircraft) -> float:
    gs = max(ac.cruise_ms - ac.wind_ms, 1.0)
    t = leg.distance_m / gs
    return mass_kg * ac.hover_w_per_kg * t / 3600.0 + mass_kg * G * leg.climb_m / ac.climb_eff / 3600.0


# --------------------------------------------------------------------------
# Spray coverage planning
# --------------------------------------------------------------------------
def boustrophedon(poly: Sequence[LatLon], swath: float, angle_deg: float = 0.0,
                  edge_margin: float = 1.0) -> Tuple[List[Tuple[LatLon, LatLon]], float]:
    """Parallel spray lines clipped to the polygon. Returns (segments, area_m2).
    Each segment = (start, end) already ordered for alternating (snake) flight.
    NOTE: concave fields should be split into convex blocks first."""
    origin = poly[0]
    loc = [to_local(origin, p) for p in poly]
    th = math.radians(angle_deg)
    c, sn = math.cos(th), math.sin(th)
    rot = [(x * c + y * sn, -x * sn + y * c) for x, y in loc]     # tracks parallel to x'
    ys = [p[1] for p in rot]
    ymin, ymax = min(ys), max(ys)
    segs, k, y = [], 0, ymin + swath / 2.0
    while y < ymax:
        xs = []
        for i in range(len(rot)):
            (x1, y1), (x2, y2) = rot[i], rot[(i + 1) % len(rot)]
            if (y1 <= y < y2) or (y2 <= y < y1):
                xs.append(x1 + (y - y1) * (x2 - x1) / (y2 - y1))
        xs.sort()
        for a, b in zip(xs[0::2], xs[1::2]):
            a, b = a + edge_margin, b - edge_margin
            if b - a < 1.0:
                continue
            p1, p2 = ((a, y), (b, y)) if k % 2 == 0 else ((b, y), (a, y))
            segs.append(tuple(to_latlon(origin, px * c - py * sn, px * sn + py * c) for px, py in (p1, p2)))
        y += swath
        k += 1
    return segs, polygon_area_m2(loc)


# --------------------------------------------------------------------------
# Mission file writer (QGC WPL 110)
# --------------------------------------------------------------------------
class MissionWriter:
    def __init__(self, home: LatLon, home_amsl: float):
        self.rows: List[str] = [
            "QGC WPL 110",
            f"0\t1\t0\t{CMD_WAYPOINT}\t0\t0\t0\t0\t{home[0]:.8f}\t{home[1]:.8f}\t{home_amsl:.2f}\t1",
        ]

    def add(self, cmd: int, frame: int = FRAME_GLOBAL_REL_ALT, lat: float = 0.0, lon: float = 0.0,
            alt: float = 0.0, p1: float = 0, p2: float = 0, p3: float = 0, p4: float = 0) -> None:
        seq = len(self.rows) - 1
        self.rows.append(f"{seq}\t0\t{frame}\t{cmd}\t{p1}\t{p2}\t{p3}\t{p4}\t{lat:.8f}\t{lon:.8f}\t{alt:.2f}\t1")

    def save(self, path: str) -> None:
        with open(path, "w", encoding="utf-8", newline="\n") as fh:
            fh.write("\n".join(self.rows) + "\n")


def write_leg(path: str, leg: Leg, final: str = "land") -> None:
    mw = MissionWriter(leg.waypoints[0][0], leg.home_elev)
    mw.add(CMD_TAKEOFF, alt=round(leg.waypoints[0][1], 1))
    for (lat, lon), alt in leg.waypoints:
        mw.add(CMD_WAYPOINT, lat=lat, lon=lon, alt=round(alt, 1))
    (lat, lon), _ = leg.waypoints[-1]
    mw.add(CMD_LAND if final == "land" else CMD_RTL, lat=lat, lon=lon)
    mw.save(path)


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------
def _ll(text: str) -> LatLon:
    a, b = text.split(",")
    return float(a), float(b)


def _load_dem(path: Optional[str]):
    if path:
        return AsciiGridDEM(path)
    print("[info] no --dem given: using SYNTHETIC demo terrain (not real ground!)")
    return SyntheticDEM()


def cmd_delivery(a: argparse.Namespace) -> int:
    dem, ac = _load_dem(a.dem), Aircraft(empty_kg=a.empty_kg, cruise_ms=a.cruise, wind_ms=a.wind)
    origin, dest = _ll(a.origin), _ll(a.dest)
    out_leg = plan_leg(dem, origin, dest, ac, a.clearance)
    ret_leg = plan_leg(dem, dest, origin, ac, a.clearance)
    e_out = energy_wh(out_leg, ac.empty_kg + a.payload_kg, ac)
    e_ret = energy_wh(ret_leg, ac.empty_kg, ac)
    budget = ac.battery_wh * (1 - ac.reserve_frac)
    ok = (e_out + e_ret) <= budget
    write_leg(f"{a.out}_outbound.waypoints", out_leg, "land")
    write_leg(f"{a.out}_return.waypoints", ret_leg, "land")
    print(f"Route length (ground)     : {haversine(origin, dest) / 1000:.2f} km")
    print(f"Outbound waypoints        : {len(out_leg.waypoints)}  | 3-D path {out_leg.distance_m / 1000:.2f} km | climb {out_leg.climb_m:.0f} m")
    print(f"Return   waypoints        : {len(ret_leg.waypoints)}  | 3-D path {ret_leg.distance_m / 1000:.2f} km | climb {ret_leg.climb_m:.0f} m")
    print(f"Highest terrain on route  : {out_leg.max_terrain:.0f} m AMSL")
    print(f"Suggested RTL_ALT         : {out_leg.rtl_alt_rel * 100:.0f} cm above home (upper bound; alternatively use RTL_ALT_TYPE=1 terrain-relative RTL)")
    print(f"Energy out+back           : {e_out + e_ret:.0f} Wh of {budget:.0f} Wh usable (after {ac.reserve_frac:.0%} reserve)")
    print("MISSION FEASIBLE" if ok else "MISSION REJECTED: insufficient energy - reduce payload/distance or add battery")
    print(f"Files written             : {a.out}_outbound.waypoints, {a.out}_return.waypoints")
    return 0 if ok else 2


DEMO_FIELD = "29.2200,79.5100;29.2200,79.5108;29.2206,79.5108;29.2206,79.5100"


def cmd_spray(a: argparse.Namespace) -> int:
    dem = _load_dem(a.dem)
    poly = [_ll(p) for p in (a.polygon or DEMO_FIELD).split(";")]
    segs, area = boustrophedon(poly, a.swath, a.angle)
    if not segs:
        print("No spray lines generated - check polygon/swath")
        return 1
    home = poly[0]
    mw = MissionWriter(home, dem.elevation(*home))
    mw.add(CMD_TAKEOFF, alt=10)
    mw.add(CMD_CHANGE_SPEED, alt=0, p1=1, p2=a.speed, p3=-1)        # ground speed
    for start, end in segs:
        mw.add(CMD_WAYPOINT, FRAME_TERRAIN_ALT, *start, a.height)   # fly to line start
        if a.relay is not None:
            mw.add(CMD_SET_RELAY, p1=a.relay, p2=1)                 # pump ON
        mw.add(CMD_WAYPOINT, FRAME_TERRAIN_ALT, *end, a.height)     # spray along the line
        if a.relay is not None:
            mw.add(CMD_SET_RELAY, p1=a.relay, p2=0)                 # pump OFF
    mw.add(CMD_RTL)
    mw.save(f"{a.out}.waypoints")

    ha = area / 10_000.0
    volume = ha * a.rate_l_ha
    flow = a.rate_l_ha * a.speed * a.swath / 166.67                 # L/min needed
    line_len = sum(haversine(s, e) for s, e in segs)
    print(f"Field area                : {ha:.3f} ha")
    print(f"Spray lines               : {len(segs)} x swath {a.swath} m at {a.height} m above terrain")
    print(f"Total spray-line length   : {line_len:.0f} m ({line_len / a.speed / 60:.1f} min at {a.speed} m/s)")
    print(f"Liquid needed             : {volume:.1f} L at {a.rate_l_ha} L/ha")
    print(f"Required pump flow        : {flow:.2f} L/min | tank trips ({a.tank_l} L): {math.ceil(volume / a.tank_l)}")
    print(f"File written              : {a.out}.waypoints")
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="mode", required=True)

    d = sub.add_parser("delivery", help="healthcare delivery leg (out + return)")
    d.add_argument("--origin", default="29.2183,79.5130", help="lat,lon of launch point")
    d.add_argument("--dest", default="29.2450,79.5400", help="lat,lon of PHC landing pad")
    d.add_argument("--payload-kg", type=float, default=4.0)
    d.add_argument("--empty-kg", type=float, default=9.5)
    d.add_argument("--clearance", type=float, default=60.0, help="min metres above terrain")
    d.add_argument("--cruise", type=float, default=12.0)
    d.add_argument("--wind", type=float, default=5.0, help="assumed headwind m/s")
    d.add_argument("--dem", help="ESRI ASCII grid DEM (EPSG:4326)")
    d.add_argument("--out", default="delivery")
    d.set_defaults(fn=cmd_delivery)

    s = sub.add_parser("spray", help="agricultural coverage mission")
    s.add_argument("--polygon", help='"lat,lon;lat,lon;..." field boundary (default: demo field)')
    s.add_argument("--swath", type=float, default=4.0, help="effective swath width (m)")
    s.add_argument("--angle", type=float, default=0.0, help="track direction (deg from east)")
    s.add_argument("--height", type=float, default=3.0, help="spray height above canopy/terrain (m)")
    s.add_argument("--speed", type=float, default=4.0, help="spray ground speed (m/s)")
    s.add_argument("--rate-l-ha", type=float, default=10.0, help="target application rate (L/ha)")
    s.add_argument("--tank-l", type=float, default=5.0)
    s.add_argument("--relay", type=int, help="relay number to gate the pump per line (optional)")
    s.add_argument("--dem", help="ESRI ASCII grid DEM (EPSG:4326)")
    s.add_argument("--out", default="spray")
    s.set_defaults(fn=cmd_spray)

    args = ap.parse_args(argv)
    return args.fn(args)


if __name__ == "__main__":
    sys.exit(main())

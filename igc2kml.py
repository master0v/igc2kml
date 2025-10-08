#!/usr/bin/env python3
"""
Drop-in replacement script: Convert an IGC file to a KML that
(1) draws the CORRECT track, (2) colors by altitude, (3) adds takeoff/landing,
and (4) if points > 10,000, resamples (interpolates) down to 10,000.

Usage:
  python igc_to_kml.py path/to/flight.igc
  python igc_to_kml.py path/to/flight.igc -o out.kml
"""

import re
from math import radians, sin, cos, sqrt, atan2
from datetime import datetime, timedelta
from pathlib import Path
from typing import List, Tuple, Optional

# --------------------------- IGC PARSING ---------------------------

def parse_igc_datetime_and_points(igc_text: str):
    """
    Parses an IGC file into (date, points). Each point is a dict with keys:
    time (datetime), lat (float), lon (float), alt_gps (int), alt_press (int), valid (bool)
    """
    # Date (prefer HFDTEDATE, fall back to HFDTE)
    m = re.search(r"HFDTEDATE:(\d{6})", igc_text)
    if not m:
        m = re.search(r"HFDTE(\d{6})", igc_text)
    if not m:
        raise ValueError("Could not find date (HFDTEDATE/HFDTE) in IGC.")
    dd, mm, yy = m.group(1)[0:2], m.group(1)[2:4], m.group(1)[4:6]
    year = 2000 + int(yy)  # IGC YY assumed 2000-2099
    igc_date = datetime(year, int(mm), int(dd))

    points: List[dict] = []
    for line in igc_text.splitlines():
        if not line.startswith("B"):
            continue
        # B-record per IGC spec:
        #  0   'B'
        #  1-6 time HHMMSS
        #  7-11 lat DDMMmmm  (we read as 5 digits: MMmmm)
        #  12-? handled by slicing below (we'll use 0-based python indices)
        try:
            hh = int(line[1:3]); mi = int(line[3:5]); ss = int(line[5:7])

            # Latitude
            lat_deg = int(line[7:9])                     # DD
            lat_mm_thousandths = int(line[9:14])         # MMmmm (2+3 digits)
            lat_hem = line[14]                           # N/S

            # Longitude
            lon_deg = int(line[15:18])                   # DDD
            lon_mm_thousandths = int(line[18:23])        # MMmmm
            lon_hem = line[23]                           # E/W

            fix_valid = line[24]                         # 'A' or 'V'
            alt_press = int(line[25:30])                 # 5 chars
            alt_gps = int(line[30:35])                   # 5 chars

            # Convert to decimal degrees:
            # MMmmm is minutes with thousandths: e.g., "30234" => 30.234 minutes
            lat = lat_deg + (lat_mm_thousandths / 1000.0) / 60.0
            lon = lon_deg + (lon_mm_thousandths / 1000.0) / 60.0
            if lat_hem == 'S': lat = -lat
            if lon_hem == 'W': lon = -lon

            timestamp = igc_date + timedelta(hours=hh, minutes=mi, seconds=ss)

            points.append({
                "time": timestamp,
                "lat": lat,
                "lon": lon,
                "alt_gps": alt_gps,
                "alt_press": alt_press,
                "valid": (fix_valid == 'A')
            })
        except Exception:
            # Skip malformed lines instead of failing the whole file
            continue

    # Handle possible midnight rollover within a single file
    for i in range(1, len(points)):
        if points[i]["time"] < points[i-1]["time"]:
            # Consider rollover real if gap exceeds 12 hours
            if (points[i-1]["time"] - points[i]["time"]).total_seconds() > 12*3600:
                for j in range(i, len(points)):
                    points[j]["time"] += timedelta(days=1)
            break

    return igc_date.date(), points

# --------------------------- UTILITIES ---------------------------

def haversine_m(gps1: Tuple[float,float], gps2: Tuple[float,float]) -> float:
    R = 6371000.0
    lat1, lon1 = gps1
    lat2, lon2 = gps2
    phi1, phi2 = radians(lat1), radians(lat2)
    dphi = radians(lat2 - lat1)
    dlambda = radians(lon2 - lon1)
    a = sin(dphi/2)**2 + cos(phi1)*cos(phi2)*sin(dlambda/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    return R * c

def compute_speeds(points: List[dict]) -> List[float]:
    speeds = [0.0] * len(points)
    for i in range(1, len(points)):
        dt = (points[i]["time"] - points[i-1]["time"]).total_seconds()
        if dt <= 0:
            speeds[i] = 0.0
            continue
        d = haversine_m((points[i-1]["lat"], points[i-1]["lon"]), (points[i]["lat"], points[i]["lon"]))
        speeds[i] = d / dt
    return speeds

def detect_takeoff_landing(points: List[dict]) -> Tuple[int, int]:
    """
    Returns (takeoff_idx, landing_idx) using speed thresholds with hysteresis.
    """
    if len(points) < 2:
        return 0, max(0, len(points)-1)

    speeds = compute_speeds(points)

    # Ignore invalid fixes up-front to reduce noise.
    # We'll treat an invalid fix as zero speed (keeps indices aligned).
    for i, p in enumerate(points):
        if not p.get("valid", True):
            speeds[i] = 0.0

    # Takeoff when speed > 8 m/s for 10 consecutive fixes
    takeoff_idx = 0
    take_win = 10
    for i in range(take_win - 1, len(points)):
        if all(speeds[j] > 8.0 for j in range(i - take_win + 1, i + 1)):
            takeoff_idx = i - take_win + 1
            break

    # Landing when speed < 5 m/s for 15 consecutive fixes scanning backwards
    landing_idx = len(points) - 1
    land_win = 15
    for i in range(len(points) - land_win - 1, takeoff_idx + land_win - 1, -1):
        if all(speeds[j] < 5.0 for j in range(i, i + land_win)):
            landing_idx = i + land_win - 1
            break

    # Sanity clamp
    takeoff_idx = max(0, min(takeoff_idx, len(points)-1))
    landing_idx = max(takeoff_idx, min(landing_idx, len(points)-1))
    return takeoff_idx, landing_idx

def linear_interpolate(p1: dict, p2: dict, t: datetime) -> dict:
    """Linear interpolation of lat, lon, alt between p1 at t1 and p2 at t2 to time t."""
    t1, t2 = p1["time"], p2["time"]
    if t2 == t1:
        return dict(p1)
    u = (t - t1).total_seconds() / (t2 - t1).total_seconds()
    u = max(0.0, min(1.0, u))
    lat = p1["lat"] + u * (p2["lat"] - p1["lat"])
    lon = p1["lon"] + u * (p2["lon"] - p1["lon"])
    alt = int(round(p1["alt_gps"] + u * (p2["alt_gps"] - p1["alt_gps"])))
    return {"time": t, "lat": lat, "lon": lon, "alt_gps": alt, "alt_press": None, "valid": True}

def resample_to_max(points: List[dict], max_points: int = 10000) -> List[dict]:
    """
    If len(points) > max_points, build a uniformly time-spaced interpolation
    with exactly max_points samples from start to end (inclusive).
    """
    n = len(points)
    if n <= max_points or n < 2:
        return points

    t0, tN = points[0]["time"], points[-1]["time"]
    total = (tN - t0).total_seconds()
    if total <= 0:
        # Fallback: simple decimation if timestamps are not increasing
        step = n / max_points
        return [points[int(i * step)] for i in range(max_points)]

    # Build target times
    res = []
    for i in range(max_points):
        # Evenly spaced in time, include endpoints
        ti = t0 + timedelta(seconds = (total * i) / (max_points - 1))
        # Find bracketing points (advance j monotonically)
        # Two-pointer search:
        # Maintain current index cursor across iterations for efficiency
        if i == 0:
            j = 0
        else:
            # reuse previous j
            pass
        # initialize j based on previous loop
        if not res:
            j = 0
        else:
            j = getattr(resample_to_max, "_cursor", 0)

        while j + 1 < n and points[j + 1]["time"] < ti:
            j += 1
        setattr(resample_to_max, "_cursor", j)
        if j + 1 >= n:
            res.append(points[-1])
        else:
            res.append(linear_interpolate(points[j], points[j+1], ti))
    # Reset cursor for next call safety
    if hasattr(resample_to_max, "_cursor"):
        delattr(resample_to_max, "_cursor")
    return res

# --------------------------- COLOR HELPERS ---------------------------

def color_argb(a: int, r: int, g: int, b: int) -> str:
    # KML color is aabbggrr hex (note reversed channel order)
    return f"{a:02x}{b:02x}{g:02x}{r:02x}"

def ramp_color(v: float) -> Tuple[int, int, int]:
    """
    v in [0,1] -> RGB gradient from blue (low) to red (high) via cyan/green/yellow.
    """
    v = max(0.0, min(1.0, v))
    if v < 0.25:
        t = v/0.25
        return (0, int(255*t), 255)              # blue -> cyan
    elif v < 0.5:
        t = (v-0.25)/0.25
        return (int(255*t), 255, int(255*(1-t))) # cyan -> green
    elif v < 0.75:
        t = (v-0.5)/0.25
        return (255, int(255*(1-t)), 0)          # green -> yellow
    else:
        t = (v-0.75)/0.25
        return (255, int(255*(1-t)), 0)          # yellow -> red

# --------------------------- KML BUILDING ---------------------------

def build_kml(points: List[dict], date_str: str, takeoff_idx: int, landing_idx: int, max_out_pts: int = 10000) -> str:
    """
    Build a KML with:
      - Correct track (preserves chronological order)
      - Colored by altitude bands
      - Takeoff and landing placemarks
      - Resampled to <= max_out_pts points (interpolated) if needed
    """
    # Trim to flight segment between takeoff and landing
    segment_points = points[takeoff_idx:landing_idx+1] if landing_idx >= takeoff_idx else points[:]
    # Filter to valid fixes first (but keep time continuity). If no 'A' fixes, fall back to all.
    valid_segment = [p for p in segment_points if p.get("valid", True)]
    if len(valid_segment) >= 2:
        segment_points = valid_segment

    # Ensure strictly increasing time (drop any non-increasing glitches)
    cleaned = [segment_points[0]]
    for p in segment_points[1:]:
        if p["time"] > cleaned[-1]["time"]:
            cleaned.append(p)
    segment_points = cleaned

    # Resample to max_out_pts if needed (interpolate)
    segment_points = resample_to_max(segment_points, max_points=max_out_pts)

    if not segment_points or len(segment_points) < 2:
        raise ValueError("Not enough track points after cleaning/resampling to build KML.")

    # Altitude bands (based on resampled points)
    alts = [p["alt_gps"] for p in segment_points if p["alt_gps"] is not None]
    alt_min, alt_max = (min(alts), max(alts)) if alts else (0, 1000)
    bands = 8
    if alt_max == alt_min:
        alt_max = alt_min + 1  # avoid zero span
    band_edges = [alt_min + i*(alt_max-alt_min)/bands for i in range(bands+1)]

    def band_index(alt: Optional[float]) -> int:
        if alt is None:
            return 0
        for i in range(bands):
            if band_edges[i] <= alt <= band_edges[i+1]:
                return i
        return bands-1

    # Build styles (one per band)
    style_defs = []
    for i in range(bands):
        v = i/(bands-1) if bands > 1 else 0.0
        r, g, b = ramp_color(v)
        col = color_argb(0xFF, r, g, b)
        style_defs.append(f"""
    <Style id="band{i}">
      <LineStyle><color>{col}</color><width>3</width></LineStyle>
    </Style>""")

    # CRITICAL FIX FOR "GARBAGE" LINES:
    # We must NOT connect non-contiguous parts that share the same altitude band.
    # Instead, we create a separate LineString for each CONTIGUOUS RUN of the same band.
    placemarks = []
    current_band = band_index(segment_points[0]["alt_gps"])
    run_coords: List[Tuple[float, float, int]] = []

    def flush_run(band: int, coords: List[Tuple[float,float,int]], idx: int):
        if len(coords) < 2:
            return
        coord_str = "\n".join(f"{lon:.6f},{lat:.6f},{alt}" for lon,lat,alt in coords)
        placemarks.append(f"""
    <Placemark>
      <name>Track band {band+1} (seg {idx})</name>
      <styleUrl>#band{band}</styleUrl>
      <LineString>
        <tessellate>1</tessellate>
        <altitudeMode>absolute</altitudeMode>
        <coordinates>
{coord_str}
        </coordinates>
      </LineString>
    </Placemark>""")

    seg_counter = 1
    for i, p in enumerate(segment_points):
        b = band_index(p["alt_gps"])
        if b != current_band and run_coords:
            flush_run(current_band, run_coords, seg_counter)
            seg_counter += 1
            run_coords = []
            current_band = b
        run_coords.append((p["lon"], p["lat"], int(p["alt_gps"])))
    flush_run(current_band, run_coords, seg_counter)

    # Takeoff/Landing icons (use the exact takeoff/landing from original points,
    # but if out of range due to cleaning, fall back to segment endpoints)
    take = points[takeoff_idx] if 0 <= takeoff_idx < len(points) else segment_points[0]
    land = points[landing_idx] if 0 <= landing_idx < len(points) else segment_points[-1]

    icons = """
    <Style id="takeoff">
      <IconStyle><scale>1.2</scale>
        <Icon><href>http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png</href></Icon>
      </IconStyle>
      <LabelStyle><scale>0.9</scale></LabelStyle>
    </Style>
    <Style id="landing">
      <IconStyle><scale>1.2</scale>
        <Icon><href>http://maps.google.com/mapfiles/kml/shapes/flag.png</href></Icon>
      </IconStyle>
      <LabelStyle><scale>0.9</scale></LabelStyle>
    </Style>
    """

    to_pm = f"""
    <Placemark>
      <name>Takeoff {take['time'].strftime('%H:%M:%S')}</name>
      <styleUrl>#takeoff</styleUrl>
      <Point><coordinates>{take['lon']:.6f},{take['lat']:.6f},{int(take['alt_gps'])}</coordinates></Point>
    </Placemark>"""

    ld_pm = f"""
    <Placemark>
      <name>Landing {land['time'].strftime('%H:%M:%S')}</name>
      <styleUrl>#landing</styleUrl>
      <Point><coordinates>{land['lon']:.6f},{land['lat']:.6f},{int(land['alt_gps'])}</coordinates></Point>
    </Placemark>"""

    kml = f"""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>IGC Flight {date_str}</name>
    {''.join(style_defs)}
    {icons}
    {''.join(placemarks)}
    {to_pm}
    {ld_pm}
  </Document>
</kml>"""
    return kml

# --------------------------- MAIN CONVERSION ---------------------------

def convert_igc_to_kml(igc_path: Path, out_kml_path: Path, max_out_pts: int = 10000):
    txt = igc_path.read_text(errors="ignore")
    date, points = parse_igc_datetime_and_points(txt)
    if len(points) < 2:
        raise ValueError("Not enough B-record points to build a track.")

    takeoff_idx, landing_idx = detect_takeoff_landing(points)
    kml_text = build_kml(points, date.strftime("%Y-%m-%d"), takeoff_idx, landing_idx, max_out_pts)
    out_kml_path.write_text(kml_text, encoding="utf-8")
    # Print summary for user
    print(f"Wrote KML: {out_kml_path}")
    print(f"Detected takeoff index: {takeoff_idx}, landing index: {landing_idx}, total points: {len(points)}")
    # Also report output point cap (informational)
    print(f"Output track was limited to at most {max_out_pts} points (interpolated if necessary).")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Convert IGC to KML with altitude-colored continuous track and takeoff/landing markers. Limits output to <= 10,000 points via interpolation.")
    parser.add_argument("igc", help="Path to input .igc file")
    parser.add_argument("-o", "--out", help="Output KML path (default: same name with .kml)", default=None)
    parser.add_argument("--max-points", type=int, default=10000, help="Maximum output points (default 10000)")
    args = parser.parse_args()

    igc_path = Path(args.igc)
    out_path = Path(args.out) if args.out else igc_path.with_suffix(".kml")
    convert_igc_to_kml(igc_path, out_path, max_out_pts=args.max_points)

#!/usr/bin/env python3
"""
IGC → KML (drop-in replacement, corrected climb methodology)
- Continuous flight path + matching “curtains” (via <extrude>1</extrude>)
- Three visualizations: Altitude, Climb rate, Speed — colored violet→…→red
- Altitude colors: exact min (violet) → max (red)
- Speed & Climb colors: robust mean-centered ranges (middle 5–95%), speed ignores near-zero for range
- Climb rate now computed from **barometric altitude** when available (auto-fallback to GPS)
- Prevents 0,0 import errors; curtains always match segment color
- ≤ 10,000 output points (uniform time resampling)
"""

import re
from math import radians, sin, cos, sqrt, atan2
from datetime import datetime, timedelta
from pathlib import Path
from typing import List, Tuple, Dict

# --------------------------- CONFIG ---------------------------

Q_LOW  = 0.05   # robust trimming lower quantile
Q_HIGH = 0.95   # robust trimming upper quantile
MIN_RUN_PTS = 5 # merge tiny color runs to avoid “dashes”

# --------------------------- IGC PARSING ---------------------------

def parse_igc_datetime_and_points(igc_text: str):
    m = re.search(r"HFDTEDATE:(\d{6})", igc_text)
    if not m:
        m = re.search(r"HFDTE(\d{6})", igc_text)
    if not m:
        raise ValueError("Could not find date (HFDTEDATE/HFDTE) in IGC.")
    dd, mm, yy = m.group(1)[0:2], m.group(1)[2:4], m.group(1)[4:6]
    year = 2000 + int(yy)
    igc_date = datetime(year, int(mm), int(dd))

    points: List[dict] = []
    for line in igc_text.splitlines():
        if not line.startswith("B"):
            continue
        try:
            hh = int(line[1:3]); mi = int(line[3:5]); ss = int(line[5:7])
            lat_deg = int(line[7:9])
            lat_mm_thousandths = int(line[9:14])
            lat_hem = line[14]
            lon_deg = int(line[15:18])
            lon_mm_thousandths = int(line[18:23])
            lon_hem = line[23]
            fix_valid = line[24]
            alt_press = int(line[25:30])      # meters (barometric altitude)
            alt_gps = float(line[30:35])      # meters (GPS altitude)

            lat = lat_deg + (lat_mm_thousandths / 1000.0) / 60.0
            lon = lon_deg + (lon_mm_thousandths / 1000.0) / 60.0
            if lat_hem == 'S': lat = -lat
            if lon_hem == 'W': lon = -lon

            timestamp = igc_date + timedelta(hours=hh, minutes=mi, seconds=ss)

            points.append({
                "time": timestamp,
                "lat": lat,
                "lon": lon,
                "alt_gps": alt_gps,     # meters (used for KML Z)
                "alt_press": alt_press, # meters (preferred for climb rate)
                "valid": (fix_valid == 'A')
            })
        except Exception:
            continue

    # Midnight rollover handling
    for i in range(1, len(points)):
        if points[i]["time"] < points[i-1]["time"]:
            if (points[i-1]["time"] - points[i]["time"]).total_seconds() > 12*3600:
                for j in range(i, len(points)):
                    points[j]["time"] += timedelta(days=1)
            break

    return igc_date.date(), points

# --------------------------- METRICS & RESAMPLING ---------------------------

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

def compute_speeds_mps(points: List[dict]) -> List[float]:
    speeds = [0.0] * len(points)
    for i in range(1, len(points)):
        dt = (points[i]["time"] - points[i-1]["time"]).total_seconds()
        if dt <= 0:
            speeds[i] = 0.0
            continue
        d = haversine_m((points[i-1]["lat"], points[i-1]["lon"]), (points[i]["lat"], points[i]["lon"]))
        speeds[i] = d / dt
    return speeds

def choose_altitude_series_for_climb(points: List[dict]) -> List[float]:
    """Prefer barometric altitude if it looks valid and non-flat; else GPS."""
    alt_press = [p["alt_press"] for p in points]
    alt_gps   = [p["alt_gps"]   for p in points]

    # Heuristics: enough nonzero, enough variance
    nonzero_press = sum(1 for a in alt_press if a != 0)
    var_press = (max(alt_press) - min(alt_press)) if alt_press else 0
    if nonzero_press > len(points) * 0.7 and var_press > 1.0:
        return alt_press
    return alt_gps

def compute_climb_mps(points: List[dict]) -> List[float]:
    """
    Vertical speed for segment (i-1 → i), in m/s.
    Uses barometric altitude when available; falls back to GPS otherwise.
    The first sample inherits the second segment’s rate for consistent coloring.
    """
    alts = choose_altitude_series_for_climb(points)
    climbs = [0.0] * len(points)
    for i in range(1, len(points)):
        dt = (points[i]["time"] - points[i-1]["time"]).total_seconds()
        if dt <= 0:
            climbs[i] = climbs[i-1] if i > 0 else 0.0
            continue
        climbs[i] = (alts[i] - alts[i-1]) / dt
    if len(climbs) >= 2:
        climbs[0] = climbs[1]
    return climbs

def linear_interpolate(p1: dict, p2: dict, t: datetime) -> dict:
    t1, t2 = p1["time"], p2["time"]
    if t2 == t1:
        return dict(p1)
    u = (t - t1).total_seconds() / (t2 - t1).total_seconds()
    u = max(0.0, min(1.0, u))
    return {
        "time": t,
        "lat": p1["lat"] + u * (p2["lat"] - p1["lat"]),
        "lon": p1["lon"] + u * (p2["lon"] - p1["lon"]),
        "alt_gps": p1["alt_gps"] + u * (p2["alt_gps"] - p1["alt_gps"]),
        "alt_press": int(round(p1["alt_press"] + u * (p2["alt_press"] - p1["alt_press"]))),
        "valid": True,
    }

def resample_to_max(points: List[dict], max_points: int = 10000) -> List[dict]:
    n = len(points)
    if n <= max_points or n < 2:
        return points
    t0, tN = points[0]["time"], points[-1]["time"]
    total = (tN - t0).total_seconds()
    if total <= 0:
        step = n / max_points
        return [points[int(i * step)] for i in range(max_points)]
    res = []
    j = 0
    for i in range(max_points):
        ti = t0 + timedelta(seconds = (total * i) / (max_points - 1))
        while j + 1 < n and points[j + 1]["time"] < ti:
            j += 1
        if j + 1 >= n:
            res.append(points[-1])
        else:
            res.append(linear_interpolate(points[j], points[j+1], ti))
    return res

# --------------------------- UNIT CONVERSIONS ---------------------------

def mps_to_kmh(v): return v * 3.6
def mps_to_mph(v): return v * 2.2369362921
def mps_to_fpm(v): return v * 196.8503937
def m_to_ft(h):    return h * 3.280839895

# --------------------------- COLOR MAP ---------------------------

def color_argb(a: int, r: int, g: int, b: int) -> str:
    # KML expects aabbggrr
    return f"{a:02x}{b:02x}{g:02x}{r:02x}"

def hex_to_rgb(hexstr: str) -> Tuple[int,int,int]:
    hexstr = hexstr.lstrip('#')
    return int(hexstr[0:2],16), int(hexstr[2:4],16), int(hexstr[4:6],16)

def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t

def lerp_rgb(c1: Tuple[int,int,int], c2: Tuple[int,int,int], t: float) -> Tuple[int,int,int]:
    return (int(round(lerp(c1[0], c2[0], t))),
            int(round(lerp(c1[1], c2[1], t))),
            int(round(lerp(c1[2], c2[2], t))))

# Rainbow stops: violet → blue → cyan → green → yellow → orange → red
RAINBOW_STOPS = [
    (0.00, hex_to_rgb("#4B0082")),  # violet
    (0.16, hex_to_rgb("#0000FF")),  # blue
    (0.33, hex_to_rgb("#00FFFF")),  # cyan
    (0.50, hex_to_rgb("#00FF00")),  # green
    (0.66, hex_to_rgb("#FFFF00")),  # yellow
    (0.83, hex_to_rgb("#FFA500")),  # orange
    (1.00, hex_to_rgb("#FF0000")),  # red
]
STOP_NAMES = ["violet","blue","cyan","green","yellow","orange","red"]

def rainbow_color(val: float, vmin: float, vmax: float) -> Tuple[int,int,int]:
    if vmax <= vmin:
        t = 0.5
    else:
        t = (val - vmin) / (vmax - vmin)
        t = max(0.0, min(1.0, t))
    for i in range(len(RAINBOW_STOPS)-1):
        t0, c0 = RAINBOW_STOPS[i]
        t1, c1 = RAINBOW_STOPS[i+1]
        if t0 <= t <= t1:
            u = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
            return lerp_rgb(c0, c1, u)
    return RAINBOW_STOPS[-1][1]

def color_name_for(val: float, vmin: float, vmax: float) -> str:
    if vmax <= vmin:
        t = 0.5
    else:
        t = (val - vmin) / (vmax - vmin)
        t = max(0.0, min(1.0, t))
    idx = min(range(len(RAINBOW_STOPS)), key=lambda i: abs(RAINBOW_STOPS[i][0] - t))
    return STOP_NAMES[idx]

# --------------------------- DISTRIBUTION HELPERS ---------------------------

def percentile(vals: List[float], p: float) -> float:
    if not vals: return 0.0
    s = sorted(vals)
    k = max(0, min(len(s)-1, int(round(p*(len(s)-1)))))
    return s[k]

def trimmed_values(vals: List[float], q_low: float, q_high: float) -> List[float]:
    if not vals: return []
    lo = percentile(vals, q_low)
    hi = percentile(vals, q_high)
    if hi < lo:
        lo, hi = hi, lo
    return [v for v in vals if lo <= v <= hi]

# --------------------------- KML HELPERS ---------------------------

def kml_coords_string(coords: List[Tuple[float,float,float]]) -> str:
    return "\n".join(f"{lon:.6f},{lat:.6f},{alt:.2f}" for lon,lat,alt in coords)

def make_style(style_id: str, color_hex: str, width: int = 3) -> str:
    return (
        f"\n    <Style id=\"{style_id}\">\n"
        f"      <LineStyle><color>{color_hex}</color><width>{width}</width></LineStyle>\n"
        f"    </Style>"
    )

def ensure_style(global_styles: Dict[str,str], style_id: str, color_hex: str, width: int = 3):
    if style_id not in global_styles:
        global_styles[style_id] = make_style(style_id, color_hex, width)

def compress_coords(coords: List[Tuple[float,float,float]], min_2d_m: float = 0.5, min_dalt_m: float = 0.5) -> List[Tuple[float,float,float]]:
    if not coords: return coords
    kept = [coords[0]]
    for lon,lat,alt in coords[1:]:
        lon0,lat0,alt0 = kept[-1]
        if lon == lon0 and lat == lat0 and abs(alt-alt0) < 1e-6:
            continue
        d = haversine_m((lat0,lon0),(lat,lon))
        if d < min_2d_m and abs(alt-alt0) < min_dalt_m:
            continue
        kept.append((lon,lat,alt))
    return kept

def has_two_unique_coords(coords: List[Tuple[float,float,float]]) -> bool:
    if len(coords) < 2:
        return False
    seen = set()
    for lon,lat,alt in coords:
        key = (round(lon,6), round(lat,6), round(alt,2))
        seen.add(key)
        if len(seen) >= 2:
            return True
    return False

def lin_edges(vmin: float, vmax: float, bands: int) -> List[float]:
    if vmax <= vmin:
        vmax = vmin + 1.0
    return [vmin + i*(vmax-vmin)/bands for i in range(bands+1)]

def _band_index(val: float, edges: List[float]) -> int:
    for i in range(len(edges)-1):
        if edges[i] <= val <= edges[i+1]:
            return i
    return len(edges)-2

def segment_runs_by_band(values: List[float], edges: List[float], min_run_pts: int = MIN_RUN_PTS) -> List[Tuple[int,int,int]]:
    """Return list of (start_idx, end_idx_inclusive, band_index), merging tiny runs."""
    if not values: return []
    cur_band = _band_index(values[0], edges)
    start = 0
    runs = []
    for i in range(1, len(values)):
        b = _band_index(values[i], edges)
        if b != cur_band:
            runs.append([start, i-1, cur_band])
            start = i
            cur_band = b
    runs.append([start, len(values)-1, cur_band])

    if min_run_pts <= 1 or len(runs) <= 1:
        return [(a,b,c) for a,b,c in runs]

    # Merge short runs into neighbors to avoid jitter
    merged = []
    i = 0
    while i < len(runs):
        s,e,b = runs[i]
        length = e - s + 1
        if length >= min_run_pts or len(runs) == 1:
            merged.append([s,e,b]); i += 1; continue
        if merged and merged[-1][2] == b:
            merged[-1][1] = e
            i += 1
        elif i+1 < len(runs) and runs[i+1][2] == b:
            runs[i+1][0] = s
            i += 1
        else:
            left_len = merged[-1][1]-merged[-1][0]+1 if merged else -1
            right_len = runs[i+1][1]-runs[i+1][0]+1 if i+1 < len(runs) else -1
            if right_len > left_len and i+1 < len(runs):
                runs[i+1][0] = s
            elif merged:
                merged[-1][1] = e
            else:
                merged.append([s,e,b])
            i += 1
    return [(a,b,c) for a,b,c in merged]

# --------------------------- RANGE / COLORS ---------------------------

def build_ranges_and_colors(
    alts_val: List[float],
    speeds_val: List[float],
    climbs_val: List[float],
    units: str,
):
    # Units & defaults
    if units == "imperial":
        speed_unit, climb_unit, alt_unit = "mph", "ft/min", "ft"
        speed_floor = 5.0
        climb_clamp = (-1200.0, 1200.0)
    else:
        speed_unit, climb_unit, alt_unit = "km/h", "m/s", "m"
        speed_floor = 8.0
        climb_clamp = (-6.0, 6.0)

    # Altitude: exact min/max
    alt_min_enc = min(alts_val) if alts_val else 0.0
    alt_max_enc = max(alts_val) if alts_val else 1.0
    if alt_max_enc <= alt_min_enc:
        alt_max_enc = alt_min_enc + 1.0
    alt_edges = lin_edges(alt_min_enc, alt_max_enc, 12)

    # Speed: ignore <= speed_floor for range, then mean-centered around trimmed middle
    speeds_nonzero = [v for v in speeds_val if v > speed_floor] or speeds_val[:]
    sp_trim = trimmed_values(speeds_nonzero, Q_LOW, Q_HIGH) or speeds_nonzero
    sp_mean = sum(sp_trim)/len(sp_trim) if sp_trim else 0.0
    sp_lo_q = percentile(speeds_nonzero, Q_LOW) if speeds_nonzero else 0.0
    sp_hi_q = percentile(speeds_nonzero, Q_HIGH) if speeds_nonzero else 1.0
    sp_half = max(sp_mean - sp_lo_q, sp_hi_q - sp_mean)
    sp_range = (sp_mean - sp_half, sp_mean + sp_half)
    sp_edges = lin_edges(sp_range[0], sp_range[1], 12)

    # Climb: trimmed mean-centered, clamped to realistic bounds
    cl_trim = trimmed_values(climbs_val, Q_LOW, Q_HIGH) or climbs_val[:]
    cl_mean = sum(cl_trim)/len(cl_trim) if cl_trim else 0.0
    cl_lo_q = percentile(climbs_val, Q_LOW) if climbs_val else -1.0
    cl_hi_q = percentile(climbs_val, Q_HIGH) if climbs_val else 1.0
    cl_half = max(cl_mean - cl_lo_q, cl_hi_q - cl_mean)
    cl_range = (cl_mean - cl_half, cl_mean + cl_half)
    cl_range = (max(cl_range[0], climb_clamp[0]), min(cl_range[1], climb_clamp[1]))
    if cl_range[1] <= cl_range[0]:
        cl_range = climb_clamp
    cl_edges = lin_edges(cl_range[0], cl_range[1], 12)

    # Color functions
    alt_color = lambda v: rainbow_color(v, alt_edges[0], alt_edges[-1])
    spd_color = lambda v: rainbow_color(v, sp_edges[0], sp_edges[-1])
    clb_color = lambda v: rainbow_color(v, cl_edges[0], cl_edges[-1])

    # Means for reporting
    alt_mean = sum(alts_val)/len(alts_val) if alts_val else 0.0
    sp_mean_all = sum(speeds_val)/len(speeds_val) if speeds_val else 0.0
    cl_mean_all = sum(climbs_val)/len(climbs_val) if climbs_val else 0.0

    report = {
        "units": {"alt": alt_unit, "speed": speed_unit, "climb": climb_unit},
        "quantiles": {"q_low": Q_LOW, "q_high": Q_HIGH},
        "speed_floor": speed_floor,
        "Altitude": {
            "enc_min": alt_min_enc, "enc_max": alt_max_enc, "mean": alt_mean,
            "range_used": (alt_edges[0], alt_edges[-1]),
            "min_color_name": color_name_for(alt_edges[0], alt_edges[0], alt_edges[-1]),
            "max_color_name": color_name_for(alt_edges[-1], alt_edges[0], alt_edges[-1]),
        },
        "Speed": {
            "enc_min": min(speeds_val) if speeds_val else 0.0,
            "enc_max": max(speeds_val) if speeds_val else 1.0,
            "mean": sp_mean_all,
            "filtered_min": min(speeds_nonzero) if speeds_nonzero else 0.0,
            "filtered_max": max(speeds_nonzero) if speeds_nonzero else 1.0,
            "range_used": (sp_edges[0], sp_edges[-1]),
            "min_color_name": color_name_for(sp_edges[0], sp_edges[0], sp_edges[-1]),
            "max_color_name": color_name_for(sp_edges[-1], sp_edges[0], sp_edges[-1]),
        },
        "Climb": {
            "enc_min": min(climbs_val) if climbs_val else 0.0,
            "enc_max": max(climbs_val) if climbs_val else 0.0,
            "mean": cl_mean_all,
            "range_used": (cl_edges[0], cl_edges[-1]),
            "min_color_name": color_name_for(cl_edges[0], cl_edges[0], cl_edges[-1]),
            "max_color_name": color_name_for(cl_edges[-1], cl_edges[0], cl_edges[-1]),
        },
        "color_funcs": {"alt": alt_color, "speed": spd_color, "climb": clb_color},
        "edges": {"alt": alt_edges, "speed": sp_edges, "climb": cl_edges},
    }
    return report

# --------------------------- VISUALIZATION ---------------------------

def kml_coords_string(coords: List[Tuple[float,float,float]]) -> str:
    return "\n".join(f"{lon:.6f},{lat:.6f},{alt:.2f}" for lon,lat,alt in coords)

def make_style(style_id: str, color_hex: str, width: int = 3) -> str:
    return (
        f"\n    <Style id=\"{style_id}\">\n"
        f"      <LineStyle><color>{color_hex}</color><width>{width}</width></LineStyle>\n"
        f"    </Style>"
    )

def ensure_style(global_styles: Dict[str,str], style_id: str, color_hex: str, width: int = 3):
    if style_id not in global_styles:
        global_styles[style_id] = make_style(style_id, color_hex, width)

def has_two_unique_coords(coords: List[Tuple[float,float,float]]) -> bool:
    if len(coords) < 2:
        return False
    seen = set()
    for lon,lat,alt in coords:
        key = (round(lon,6), round(lat,6), round(alt,2))
        seen.add(key)
        if len(seen) >= 2:
            return True
    return False

def compress_coords(coords: List[Tuple[float,float,float]], min_2d_m: float = 0.5, min_dalt_m: float = 0.5) -> List[Tuple[float,float,float]]:
    if not coords: return coords
    kept = [coords[0]]
    for lon,lat,alt in coords[1:]:
        lon0,lat0,alt0 = kept[-1]
        if lon == lon0 and lat == lat0 and abs(alt-alt0) < 1e-6:
            continue
        d = haversine_m((lat0,lon0),(lat,lon))
        if d < min_2d_m and abs(alt-alt0) < min_dalt_m:
            continue
        kept.append((lon,lat,alt))
    return kept

def build_visualization_folder(
    title: str,
    metric_values: List[float],
    points: List[dict],
    band_edges: List[float],
    color_fn,  # value -> (r,g,b)
    base_style_prefix: str,
    global_styles: Dict[str,str],
) -> str:
    # Styles per band (opaque for path, semi-opaque for curtain)
    for i in range(len(band_edges)-1):
        mid_val = (band_edges[i] + band_edges[i+1]) * 0.5
        r,g,b = color_fn(mid_val)
        ensure_style(global_styles, f"{base_style_prefix}_band{i}", color_argb(0xFF, r, g, b), width=3)
        ensure_style(global_styles, f"{base_style_prefix}_band{i}_curtain", color_argb(0xCC, r, g, b), width=2)

    folder_xml = []

    # Continuous full path (single line)
    full_coords_raw = [(p["lon"], p["lat"], p["alt_gps"]) for p in points]
    full_coords = compress_coords(full_coords_raw)
    if has_two_unique_coords(full_coords):
        mid_val = (band_edges[0] + band_edges[-1]) * 0.5
        r_mid,g_mid,b_mid = color_fn(mid_val)
        full_style_id = f"{base_style_prefix}_full"
        ensure_style(global_styles, full_style_id, color_argb(0xFF, r_mid, g_mid, b_mid), width=3)

        folder_xml.append(f"""
    <Placemark>
      <name>Full Path (single line)</name>
      <description>{title}</description>
      <styleUrl>#{full_style_id}</styleUrl>
      <LineString>
        <tessellate>1</tessellate>
        <altitudeMode>absolute</altitudeMode>
        <coordinates>
{kml_coords_string(full_coords)}
        </coordinates>
      </LineString>
    </Placemark>""")

    # Colored segments
    runs = segment_runs_by_band(metric_values, band_edges, min_run_pts=MIN_RUN_PTS)
    colored_parts = []
    for (s, e, bidx) in runs:
        if e - s + 1 < 2:
            continue
        seg_coords_raw = [(points[i]["lon"], points[i]["lat"], points[i]["alt_gps"]) for i in range(s, e+1)]
        seg_coords = compress_coords(seg_coords_raw)
        if not has_two_unique_coords(seg_coords):
            continue

        # Path segment
        colored_parts.append(f"""
    <Placemark>
      <name>Flight Path Segment</name>
      <description>{title}</description>
      <styleUrl>#{base_style_prefix}_band{bidx}</styleUrl>
      <LineString>
        <tessellate>1</tessellate>
        <altitudeMode>absolute</altitudeMode>
        <coordinates>
{kml_coords_string(seg_coords)}
        </coordinates>
      </LineString>
    </Placemark>""")

        # Matching curtain (same color)
        colored_parts.append(f"""
    <Placemark>
      <name>Flight Path Curtain</name>
      <styleUrl>#{base_style_prefix}_band{bidx}_curtain</styleUrl>
      <LineString>
        <extrude>1</extrude>
        <tessellate>1</tessellate>
        <altitudeMode>absolute</altitudeMode>
        <coordinates>
{kml_coords_string(seg_coords)}
        </coordinates>
      </LineString>
    </Placemark>""")

    colored_folder = f"""
      <Folder>
        <name>Colored Segments</name>
        {''.join(colored_parts)}
      </Folder>
    """ if colored_parts else ""

    return f"""
    <Folder>
      <name>{title}</name>
      {''.join(folder_xml)}
      {colored_folder}
    </Folder>
    """

# --------------------------- DOCUMENT BUILD ---------------------------

def detect_takeoff_landing(points: List[dict]) -> Tuple[int, int]:
    if len(points) < 2:
        return 0, max(0, len(points)-1)
    speeds = compute_speeds_mps(points)
    for i, p in enumerate(points):
        if not p.get("valid", True):
            speeds[i] = 0.0
    takeoff_idx = 0
    take_win = 10
    for i in range(take_win - 1, len(points)):
        if all(speeds[j] > 8.0 for j in range(i - take_win + 1, i + 1)):
            takeoff_idx = i - take_win + 1
            break
    landing_idx = len(points) - 1
    land_win = 15
    for i in range(len(points) - land_win - 1, takeoff_idx + land_win - 1, -1):
        if all(speeds[j] < 5.0 for j in range(i, i + land_win)):
            landing_idx = i + land_win - 1
            break
    return max(0, min(takeoff_idx, len(points)-1)), max(takeoff_idx, min(landing_idx, len(points)-1))

def build_kml_document(points: List[dict], takeoff_idx: int, landing_idx: int, units: str):
    # Segment (valid fixes, strictly increasing time)
    segment = points[takeoff_idx:landing_idx+1]
    seg_valid = [p for p in segment if p.get('valid', True)]
    if len(seg_valid) >= 2:
        segment = seg_valid
    cleaned = [segment[0]]
    for p in segment[1:]:
        if p["time"] > cleaned[-1]["time"]:
            cleaned.append(p)
    segment = cleaned

    # Metrics (base units)
    speeds_mps = compute_speeds_mps(segment)
    climbs_mps = compute_climb_mps(segment)
    alts_m = [p["alt_gps"] for p in segment]

    # Convert for coloring/report (KML z stays meters)
    if units == "imperial":
        speeds_val = [mps_to_mph(v) for v in speeds_mps]
        climbs_val = [mps_to_fpm(v) for v in climbs_mps]
        alts_val   = [m_to_ft(h)   for h in alts_m]
    else:
        speeds_val = [mps_to_kmh(v) for v in speeds_mps]
        climbs_val = climbs_mps[:]
        alts_val   = alts_m[:]

    # Build ranges + colors
    report = build_ranges_and_colors(alts_val, speeds_val, climbs_val, units)
    alt_edges = report["edges"]["alt"]; sp_edges = report["edges"]["speed"]; cl_edges = report["edges"]["climb"]
    alt_color = report["color_funcs"]["alt"]; spd_color = report["color_funcs"]["speed"]; clb_color = report["color_funcs"]["climb"]

    # Styles registry
    global_styles: Dict[str,str] = {}

    # Folders (Altitude / Climb / Speed)
    viz_folders = []
    viz_folders.append(build_visualization_folder(
        title=f"Altitude Colored Path ({report['units']['alt']})",
        metric_values=alts_val, points=segment, band_edges=alt_edges,
        color_fn=alt_color, base_style_prefix="alt", global_styles=global_styles
    ))
    viz_folders.append(build_visualization_folder(
        title=f"Climb Rate Colored Path ({report['units']['climb']})",
        metric_values=climbs_val, points=segment, band_edges=cl_edges,
        color_fn=clb_color, base_style_prefix="climb", global_styles=global_styles
    ))
    viz_folders.append(build_visualization_folder(
        title=f"Speed Colored Path ({report['units']['speed']})",
        metric_values=speeds_val, points=segment, band_edges=sp_edges,
        color_fn=spd_color, base_style_prefix="speed", global_styles=global_styles
    ))

    # Waypoints
    take = points[takeoff_idx]; land = points[landing_idx]
    global_styles["takeoff"] = """
    <Style id="takeoff">
      <IconStyle><scale>1.2</scale><Icon><href>http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png</href></Icon></IconStyle>
      <LabelStyle><scale>0.9</scale></LabelStyle>
    </Style>"""
    global_styles["landing"] = """
    <Style id="landing">
      <IconStyle><scale>1.2</scale><Icon><href>http://maps.google.com/mapfiles/kml/shapes/flag.png</href></Icon></IconStyle>
      <LabelStyle><scale>0.9</scale></LabelStyle>
    </Style>"""

    waypoints = f"""
    <Folder>
      <name>Waypoints</name>
      <Placemark><name>Takeoff {take['time'].strftime('%H:%M:%S')}</name><styleUrl>#takeoff</styleUrl><Point><coordinates>{take['lon']:.6f},{take['lat']:.6f},{take['alt_gps']:.2f}</coordinates></Point></Placemark>
      <Placemark><name>Landing {land['time'].strftime('%H:%M:%S')}</name><styleUrl>#landing</styleUrl><Point><coordinates>{land['lon']:.6f},{land['lat']:.6f},{land['alt_gps']:.2f}</coordinates></Point></Placemark>
    </Folder>
    """

    doc_content = f"""
  <Folder>
    <name>Flight Path Visualization</name>
    {''.join(viz_folders)}
    {waypoints}
  </Folder>
  """

    return doc_content, global_styles, report

# --------------------------- MAIN ---------------------------

def convert_igc_to_kml(igc_path: Path, out_kml_path: Path, max_out_pts: int = 10000, units: str = "metric"):
    txt = igc_path.read_text(errors="ignore")
    date, points = parse_igc_datetime_and_points(txt)
    if len(points) < 2:
        raise ValueError("Not enough B-record points to build a track.")

    # Detect flight segment on original points
    to_idx, ld_idx = detect_takeoff_landing(points)

    # Resample to ≤ max points
    points_rs = resample_to_max(points, max_points=max_out_pts)

    # Map indices to resampled by nearest timestamps
    t_take = points[to_idx]["time"]; t_land = points[ld_idx]["time"]
    def nearest_index(ts):
        best_i, best_dt = 0, None
        for i, p in enumerate(points_rs):
            dt = abs((p["time"] - ts).total_seconds())
            if best_dt is None or dt < best_dt:
                best_dt, best_i = dt, i
        return best_i
    to_idx_rs = nearest_index(t_take)
    ld_idx_rs = max(to_idx_rs+1, nearest_index(t_land))

    # Build KML + styles + report
    doc_content, global_styles, report = build_kml_document(points_rs, to_idx_rs, ld_idx_rs, units=units)

    # Emit styles before features
    styles_xml = "\n".join(global_styles[k] for k in sorted(global_styles.keys()))

    kml = f"""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:gx="http://www.google.com/kml/ext/2.2">
  <Document>
    <name>IGC Flight {date.strftime('%Y-%m-%d')}</name>
    {styles_xml}
    {doc_content}
  </Document>
</kml>"""

    out_kml_path.write_text(kml, encoding="utf-8")

    # ---------- Console report (color names) ----------
    print(f"Wrote KML: {out_kml_path}")
    print(f"Detected takeoff index: {to_idx}, landing index: {ld_idx}, total points: {len(points)}")
    print(f"Output track was limited to at most {max_out_pts} points (interpolated if necessary).")
    print(f"Units: {units} | Robust trimming quantiles: [{Q_LOW:.2f}, {Q_HIGH:.2f}]")

    # Altitude
    alt = report["Altitude"]
    print(f"\nAltitude ({report['units']['alt']}): mean={alt['mean']:.3f}")
    print(f"  Encountered min: {alt['enc_min']:.3f} -> color {alt['min_color_name']}")
    print(f"  Encountered max: {alt['enc_max']:.3f} -> color {alt['max_color_name']}")
    print(f"  Color range used: [{alt['range_used'][0]:.3f}, {alt['range_used'][1]:.3f}] (exact min/max)")

    # Speed
    spd = report["Speed"]
    print(f"\nSpeed ({report['units']['speed']}): mean={spd['mean']:.3f}")
    print(f"  Encountered min/max: {spd['enc_min']:.3f} / {spd['enc_max']:.3f} (filtered for range: {spd['filtered_min']:.3f}+)")
    print(f"  Color range used: [{spd['range_used'][0]:.3f}, {spd['range_used'][1]:.3f}] "
          f"→ min color {spd['min_color_name']}, max color {spd['max_color_name']}")

    # Climb
    clb = report["Climb"]
    print(f"\nClimb ({report['units']['climb']}): mean={clb['mean']:.3f}")
    print(f"  Encountered min/max: {clb['enc_min']:.3f} / {clb['enc_max']:.3f}")
    print(f"  Color range used: [{clb['range_used'][0]:.3f}, {clb['range_used'][1]:.3f}] "
          f"→ min color {clb['min_color_name']}, max color {clb['max_color_name']}")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="IGC → KML with rainbow colors, baro-based climb, robust ranges, steady segments, and matching curtains.")
    parser.add_argument("igc", help="Path to input .igc file")
    parser.add_argument("-o", "--out", help="Output KML path (default: same name with .kml)", default=None)
    parser.add_argument("--max-points", type=int, default=10000, help="Maximum output points (default 10000)")
    parser.add_argument("--units", choices=["metric","imperial"], default="metric",
                        help="Units for band ranges (metric: km/h & m/s; imperial: mph & ft/min).")
    args = parser.parse_args()

    igc_path = Path(args.igc)
    out_path = Path(args.out) if args.out else igc_path.with_suffix(".kml")
    convert_igc_to_kml(igc_path, out_path, max_out_pts=args.max_points, units=args.units)

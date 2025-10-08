#!/usr/bin/env python3
"""
Drop-in replacement script (robust + unit-aware):

- Builds a single, continuous flight path (unbroken) and a matching “curtain”
  (vertical drop to terrain) using <extrude>1</extrude>.
- Provides separate visualizations for Altitude, Climb Rate, and Speed:
  • a single continuous “Full Path (single line)” + curtain
  • a “Colored Segments” subfolder (to mimic your sample’s color-by-value look)
- Takeoff/Landing markers.
- If input has >10,000 points, resamples (interpolates) uniformly in time to ≤10,000.
- **All styles are defined once at the top-level <Document>** (prevents “style does not exist”).
- **Defensively avoids zero-length lines**: any line with < 2 unique coordinates is skipped.
- **Unit control**: `--units {metric,imperial}` affects the value *ranges* used to color Speed
  and Climb (and Altitude if you choose fixed bands). KML altitudes remain meters as required.

Usage:
  python igc_to_kml.py path/to/flight.igc
  python igc_to_kml.py path/to/flight.igc -o out.kml --max-points 10000 --units imperial
  python igc_to_kml.py path/to/flight.igc --band-mode auto  # use data-driven color bands
"""

import re
from math import radians, sin, cos, sqrt, atan2
from datetime import datetime, timedelta
from pathlib import Path
from typing import List, Tuple, Optional, Dict

# --------------------------- IGC PARSING ---------------------------

def parse_igc_datetime_and_points(igc_text: str):
    """
    Parses an IGC file into (date, points). Each point is a dict with keys:
    time (datetime), lat (float), lon (float), alt_gps (float m), alt_press (int), valid (bool)
    """
    m = re.search(r"HFDTEDATE:(\d{6})", igc_text)
    if not m:
        m = re.search(r"HFDTE(\d{6})", igc_text)
    if not m:
        raise ValueError("Could not find date (HFDTEDATE/HFDTE) in IGC.")
    dd, mm, yy = m.group(1)[0:2], m.group(1)[2:4], m.group(1)[4:6]
    year = 2000 + int(yy)  # interpret YY as 20YY
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
            alt_press = int(line[25:30])      # pressure altitude (m)
            alt_gps = float(line[30:35])      # GPS altitude (m)

            lat = lat_deg + (lat_mm_thousandths / 1000.0) / 60.0
            lon = lon_deg + (lon_mm_thousandths / 1000.0) / 60.0
            if lat_hem == 'S': lat = -lat
            if lon_hem == 'W': lon = -lon

            timestamp = igc_date + timedelta(hours=hh, minutes=mi, seconds=ss)

            points.append({
                "time": timestamp,
                "lat": lat,
                "lon": lon,
                "alt_gps": alt_gps,   # meters
                "alt_press": alt_press,
                "valid": (fix_valid == 'A')
            })
        except Exception:
            continue

    # Handle potential midnight rollover (large backward jump)
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

def compute_climb_mps(points: List[dict]) -> List[float]:
    climbs = [0.0] * len(points)
    for i in range(1, len(points)):
        dt = (points[i]["time"] - points[i-1]["time"]).total_seconds()
        if dt <= 0:
            climbs[i] = 0.0
            continue
        climbs[i] = (points[i]["alt_gps"] - points[i-1]["alt_gps"]) / dt
    return climbs

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
    takeoff_idx = max(0, min(takeoff_idx, len(points)-1))
    landing_idx = max(takeoff_idx, min(landing_idx, len(points)-1))
    return takeoff_idx, landing_idx

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
        "alt_press": None,
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

# --------------------------- UNITS & COLOR SCALES ---------------------------

def color_argb(a: int, r: int, g: int, b: int) -> str:
    return f"{a:02x}{b:02x}{g:02x}{r:02x}"

def ramp_blue_red(t: float) -> Tuple[int,int,int]:
    t = max(0.0, min(1.0, t))
    if t < 0.25:
        u = t/0.25; return (0, int(255*u), 255)
    elif t < 0.5:
        u = (t-0.25)/0.25; return (int(255*u), 255, int(255*(1-u)))
    elif t < 0.75:
        u = (t-0.5)/0.25; return (255, int(255*(1-u)), 0)
    else:
        u = (t-0.75)/0.25; return (255, int(255*(1-u)), 0)

def ramp_diverging(val: float, vmax: float) -> Tuple[int,int,int]:
    """Blue (negative) -> white (0) -> red (positive), symmetric range [-vmax, vmax]."""
    if vmax <= 0: return (200,200,200)
    x = max(-vmax, min(vmax, val)) / vmax  # in [-1,1]
    if x >= 0:
        r = int(255 * x + 255 * (1-x))
        g = int(255 * (1-x))
        b = int(255 * (1-x))
    else:
        x = -x
        r = int(255 * (1-x))
        g = int(255 * (1-x))
        b = int(255 * x + 255 * (1-x))
    return (r,g,b)

def mps_to_kmh(v): return v * 3.6
def mps_to_mph(v): return v * 2.2369362921
def mps_to_fpm(v): return v * 196.8503937  # 1 m/s = 196.85 ft/min
def m_to_ft(h):    return h * 3.280839895

# --------------------------- KML HELPERS ---------------------------

def kml_coords_string(coords: List[Tuple[float,float,float]]) -> str:
    return "\n".join(f"{lon:.6f},{lat:.6f},{alt:.2f}" for lon,lat,alt in coords)

def make_style(style_id: str, color_hex: str, width: int = 3) -> str:
    return f"""
    <Style id="{style_id}">
      <LineStyle><color>{color_hex}</color><width>{width}</width></LineStyle>
    </Style>"""

def segment_runs_by_band(values: List[float], bands: List[float]) -> List[Tuple[int,int,int]]:
    """Return list of (start_idx, end_idx_inclusive, band_index) for contiguous runs."""
    def band_index(v: float) -> int:
        for i in range(len(bands)-1):
            if bands[i] <= v <= bands[i+1]:
                return i
        return len(bands)-2
    runs = []
    if not values:
        return runs
    cur_band = band_index(values[0])
    start = 0
    for i in range(1, len(values)):
        b = band_index(values[i])
        if b != cur_band:
            runs.append((start, i-1, cur_band))
            start = i
            cur_band = b
    runs.append((start, len(values)-1, cur_band))
    return runs

def ensure_style(global_styles: Dict[str,str], style_id: str, color_hex: str, width: int = 3):
    if style_id not in global_styles:
        global_styles[style_id] = make_style(style_id, color_hex, width)

# Robustly remove zero-length/duplicate points so KML doesn’t create 0,0 lines
def compress_coords(coords: List[Tuple[float,float,float]], min_2d_m: float = 0.5, min_dalt_m: float = 0.5) -> List[Tuple[float,float,float]]:
    if not coords: return coords
    kept = [coords[0]]
    for lon,lat,alt in coords[1:]:
        lon0,lat0,alt0 = kept[-1]
        # quick reject on exact equality
        if lon == lon0 and lat == lat0 and abs(alt-alt0) < 1e-6:
            continue
        # distance check
        d = haversine_m((lat0,lon0),(lat,lon))
        if d < min_2d_m and abs(alt-alt0) < min_dalt_m:
            continue
        kept.append((lon,lat,alt))
    return kept

# --------------------------- KML BUILD ---------------------------

def build_visualization_folder(
    title: str,
    metric_values: List[float],
    points: List[dict],
    band_edges: List[float],
    color_fn,
    base_style_prefix: str,
    global_styles: Dict[str,str]
) -> str:
    """
    Create a Folder with:
      - one continuous "Full Path (single line)" Placemark (unbroken) with curtain (extrude=1)
      - a "Colored Segments" subfolder (segments + curtains)

    Skips any line that would have <2 unique coordinates after compression.
    """
    # Pre-create band styles
    for i in range(len(band_edges)-1):
        mid_val = (band_edges[i] + band_edges[i+1]) * 0.5
        r,g,b = color_fn(mid_val)
        col_opaq = color_argb(0xFF, r, g, b)
        col_trans = color_argb(0x99, r, g, b)
        ensure_style(global_styles, f"{base_style_prefix}_band{i}", col_opaq, width=3)
        ensure_style(global_styles, f"{base_style_prefix}_band{i}_curtain", col_trans, width=2)

    folder_xml = []

    # Continuous full path + curtain
    full_coords_raw = [(p["lon"], p["lat"], p["alt_gps"]) for p in points]
    full_coords = compress_coords(full_coords_raw)
    if len(full_coords) >= 2:
        mid_idx = (len(band_edges)-1)//2
        mid_center = (band_edges[max(0, mid_idx-1)] + band_edges[mid_idx]) * 0.5 if mid_idx > 0 else band_edges[0]
        mid_color = color_argb(0xFF, *color_fn(mid_center))
        full_style_id = f"{base_style_prefix}_full"
        ensure_style(global_styles, full_style_id, mid_color, width=3)
        full_curtain_style_id = f"{base_style_prefix}_full_curtain"
        ensure_style(global_styles, full_curtain_style_id, color_argb(0x99, *color_fn(mid_center)), width=2)

        folder_xml.append(f"""
    <Placemark>
      <name>Full Path (single line)</name>
      <description>{title}</description>
      <styleUrl>#{full_style_id}</styleUrl>
      <LineString>
        <extrude>1</extrude>
        <tessellate>1</tessellate>
        <altitudeMode>absolute</altitudeMode>
        <coordinates>
{kml_coords_string(full_coords)}
        </coordinates>
      </LineString>
    </Placemark>""")

        folder_xml.append(f"""
    <Placemark>
      <name>Full Path Curtain</name>
      <styleUrl>#{full_curtain_style_id}</styleUrl>
      <LineString>
        <extrude>1</extrude>
        <tessellate>1</tessellate>
        <altitudeMode>absolute</altitudeMode>
        <coordinates>
{kml_coords_string(full_coords)}
        </coordinates>
      </LineString>
    </Placemark>""")

    # Colored segments subfolder
    runs = segment_runs_by_band(metric_values, band_edges)
    colored_parts = []
    for (s, e, bidx) in runs:
        if e - s + 1 < 2:
            continue
        seg_coords_raw = [(points[i]["lon"], points[i]["lat"], points[i]["alt_gps"]) for i in range(s, e+1)]
        seg_coords = compress_coords(seg_coords_raw)
        if len(seg_coords) < 2:
            continue  # skip zero-length segments
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
    </Placemark>
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

    # Only include the "Colored Segments" folder if something is inside
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

def build_kml_document(points: List[dict], takeoff_idx: int, landing_idx: int,
                       units: str = "metric", band_mode: str = "fixed") -> Tuple[str, Dict[str,str]]:
    """
    units: "metric" or "imperial"
    band_mode: "fixed" (use typical ranges by unit) or "auto" (data-driven)
    """
    # Work on flight segment
    segment = points[takeoff_idx:landing_idx+1]
    seg_valid = [p for p in segment if p.get('valid', True)]
    if len(seg_valid) >= 2:
        segment = seg_valid
    # Ensure strictly increasing time
    cleaned = [segment[0]]
    for p in segment[1:]:
        if p["time"] > cleaned[-1]["time"]:
            cleaned.append(p)
    segment = cleaned

    # Metrics (base units: m, m/s)
    speeds_mps = compute_speeds_mps(segment)
    climbs_mps = compute_climb_mps(segment)
    alts_m = [p["alt_gps"] for p in segment]

    # Convert for color ranges based on unit choice (KML alt stays in meters)
    if units == "imperial":
        # speed -> mph, climb -> ft/min, altitude -> feet (for band edges only)
        speeds_val = [mps_to_mph(v) for v in speeds_mps]
        climbs_val = [mps_to_fpm(v) for v in climbs_mps]
        alts_val   = [m_to_ft(h)   for h in alts_m]
        # Typical fixed ranges resembling imperial-styled visuals
        fixed_speed_range = (0.0, 120.0)    # mph
        fixed_climb_abs   = 1000.0          # ±1000 fpm
        # altitude uses min/max of flight in chosen unit
        fixed_alt_range   = (min(alts_val), max(alts_val)) if alts_val else (0.0, 1000.0)
    else:
        # metric (default): speed -> km/h, climb -> m/s, altitude -> meters
        speeds_val = [mps_to_kmh(v) for v in speeds_mps]
        climbs_val = climbs_mps[:]
        alts_val   = alts_m[:]
        fixed_speed_range = (0.0, 200.0)    # km/h
        fixed_climb_abs   = 5.0             # ±5 m/s (~±984 fpm)
        fixed_alt_range   = (min(alts_val), max(alts_val)) if alts_val else (0.0, 1000.0)

    # Build band edges
    def edges_linear(vmin: float, vmax: float, bands: int = 12) -> List[float]:
        if vmax <= vmin:
            vmax = vmin + 1.0
        return [vmin + i*(vmax-vmin)/bands for i in range(bands+1)]

    def edges_auto(vals: List[float], bands: int = 12) -> List[float]:
        if not vals:
            return edges_linear(0.0, 1.0, bands)
        vmin, vmax = min(vals), max(vals)
        return edges_linear(vmin, vmax, bands)

    # Altitude edges
    alt_edges = edges_auto(alts_val, 12) if band_mode == "auto" else edges_linear(fixed_alt_range[0], fixed_alt_range[1], 12)
    # Speed edges
    spd_edges = edges_auto(speeds_val, 12) if band_mode == "auto" else edges_linear(fixed_speed_range[0], fixed_speed_range[1], 12)
    # Climb edges (diverging)
    if band_mode == "auto":
        # symmetric around 0 using 5th/95th percentiles in chosen unit
        def percentile(vals: List[float], p: float) -> float:
            if not vals: return 0.0
            s = sorted(vals)
            k = max(0, min(len(s)-1, int(round(p*(len(s)-1)))))
            return s[k]
        c5, c95 = percentile(climbs_val, 0.05), percentile(climbs_val, 0.95)
        cmax = max(abs(c5), abs(c95), (fixed_climb_abs if units=="imperial" else fixed_climb_abs)) or 1.0
    else:
        cmax = fixed_climb_abs
    climb_edges = [-cmax + i*(2*cmax)/12 for i in range(13)]

    # Color functions
    def alt_color(val):
        vmin, vmax = alt_edges[0], alt_edges[-1]
        t = (val - vmin) / (vmax - vmin) if vmax > vmin else 0.5
        return ramp_blue_red(t)

    def spd_color(val):
        vmin, vmax = spd_edges[0], spd_edges[-1]
        t = (val - vmin) / (vmax - vmin) if vmax > vmin else 0.5
        return ramp_blue_red(t)

    def climb_color(val):
        return ramp_diverging(val, vmax=cmax)

    # Global styles registry
    global_styles: Dict[str,str] = {}

    # Visualization folders
    viz_folders = []
    viz_folders.append(build_visualization_folder(
        title="Altitude Colored Path",
        metric_values=alts_val,
        points=segment,
        band_edges=alt_edges,
        color_fn=alt_color,
        base_style_prefix="alt",
        global_styles=global_styles
    ))
    viz_folders.append(build_visualization_folder(
        title=f"Climb Rate Colored Path ({'ft/min' if units=='imperial' else 'm/s'})",
        metric_values=climbs_val,
        points=segment,
        band_edges=climb_edges,
        color_fn=climb_color,
        base_style_prefix="climb",
        global_styles=global_styles
    ))
    viz_folders.append(build_visualization_folder(
        title=f"Speed Colored Path ({'mph' if units=='imperial' else 'km/h'})",
        metric_values=speeds_val,
        points=segment,
        band_edges=spd_edges,
        color_fn=spd_color,
        base_style_prefix="speed",
        global_styles=global_styles
    ))

    # Takeoff/Landing placemarks
    take = points[takeoff_idx]
    land = points[landing_idx]
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
    return doc_content, global_styles

# --------------------------- MAIN ---------------------------

def convert_igc_to_kml(igc_path: Path, out_kml_path: Path, max_out_pts: int = 10000,
                       units: str = "metric", band_mode: str = "fixed"):
    txt = igc_path.read_text(errors="ignore")
    date, points = parse_igc_datetime_and_points(txt)
    if len(points) < 2:
        raise ValueError("Not enough B-record points to build a track.")

    # Detect flight segment on original points
    to_idx, ld_idx = detect_takeoff_landing(points)

    # Resample entire points first so visuals are consistent and ≤ max points
    points_rs = resample_to_max(points, max_points=max_out_pts)

    # Map detected indices to resampled list via nearest timestamps
    t_take = points[to_idx]["time"]
    t_land = points[ld_idx]["time"]
    def nearest_index(ts):
        best_i, best_dt = 0, None
        for i, p in enumerate(points_rs):
            dt = abs((p["time"] - ts).total_seconds())
            if best_dt is None or dt < best_dt:
                best_dt, best_i = dt, i
        return best_i
    to_idx_rs = nearest_index(t_take)
    ld_idx_rs = max(to_idx_rs+1, nearest_index(t_land))

    # Build KML document content and collect ALL styles
    doc_content, global_styles = build_kml_document(points_rs, to_idx_rs, ld_idx_rs,
                                                    units=units, band_mode=band_mode)

    # Emit styles at top-level Document BEFORE features
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
    print(f"Wrote KML: {out_kml_path}")
    print(f"Detected takeoff index: {to_idx}, landing index: {ld_idx}, total points: {len(points)}")
    print(f"Output track was limited to at most {max_out_pts} points (interpolated if necessary).")
    print(f"Units: {units} | Band mode: {band_mode}")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Convert IGC to KML with continuous path + curtain, colored visualizations (Altitude/Climb/Speed), unit-aware banding, and ≤ max points.")
    parser.add_argument("igc", help="Path to input .igc file")
    parser.add_argument("-o", "--out", help="Output KML path (default: same name with .kml)", default=None)
    parser.add_argument("--max-points", type=int, default=10000, help="Maximum output points (default 10000)")
    parser.add_argument("--units", choices=["metric","imperial"], default="metric",
                        help="Color band units/ranges (metric: km/h & m/s, imperial: mph & ft/min).")
    parser.add_argument("--band-mode", choices=["fixed","auto"], default="fixed",
                        help="fixed: use typical unit-based ranges; auto: data-driven min/max (and percentiles for climb).")
    args = parser.parse_args()

    igc_path = Path(args.igc)
    out_path = Path(args.out) if args.out else igc_path.with_suffix(".kml")
    convert_igc_to_kml(igc_path, out_path, max_out_pts=args.max_points,
                       units=args.units, band_mode=args.band_mode)

#!/usr/bin/env python3
"""
IGC → KML (drop-in replacement)
- Uses TAS (true airspeed) for speed coloring by default (fallback: GSP → derived groundspeed).
  Force fallback with --use-derived-speed.
- Uses VAT (variometer / vertical speed) for climb coloring by default (fallback: derived from baro/GPS).
  Force fallback with --use-derived-climb.
- Adds OAT (outside air temperature) colored path.
- Single continuous path per layer + matching “curtains” (extruded), takeoff/landing placemarks.
- Adds placemarks for:
    1) Highest altitude
    2) Fastest airspeed
    3) Highest climb rate
    4) Highest descent rate
    5) Lowest temperature
- Robust ranges (5–95%) for speed/climb/OAT; altitude uses exact min→max.
- Resamples uniformly in time to ≤ 10,000 points.
- Skips any LineString with <2 unique coords (prevents 0,0 import errors).

CLI examples
  python igc2kml.py flight.igc
  python igc2kml.py flight.igc --units imperial
  python igc2kml.py flight.igc --use-derived-speed --use-derived-climb
"""

import re
from math import radians, sin, cos, sqrt, atan2
from datetime import datetime, timedelta
from pathlib import Path
from typing import List, Tuple, Dict, Optional

Q_LOW  = 0.05
Q_HIGH = 0.95
MIN_RUN_PTS = 5

# --------------------------- IGC parsing ---------------------------

def parse_igc_date(igc_text: str) -> datetime.date:
    m = re.search(r"HFDTEDATE:(\d{6})", igc_text) or re.search(r"HFDTE(\d{6})", igc_text)
    if not m:
        raise ValueError("Could not find date (HFDTEDATE/HFDTE) in IGC.")
    dd, mm, yy = m.group(1)[0:2], m.group(1)[2:4], m.group(1)[4:6]
    return datetime(2000 + int(yy), int(mm), int(dd)).date()

def parse_b_extensions_map(i_line: str) -> List[Tuple[str,int,int]]:
    """
    Parse an I-record line, robust to spaces between triplets.
    Return list of (code, start_0based, end_exclusive) for slicing the whole B line.
    """
    if not i_line or not i_line.startswith('I'):
        return []
    m = re.match(r'^I(\d{2,3})\s*(.*)$', i_line.strip())
    if not m:
        return []
    count = int(m.group(1))
    rest = re.sub(r'\s+', '', m.group(2))
    triplets = re.findall(r'(\d{2})(\d{2})([A-Z0-9]{3})', rest)
    exts = []
    for ss, ee, code in triplets[:count]:
        try:
            start_0 = int(ss) - 1
            end_ex = int(ee)
            exts.append((code, start_0, end_ex))
        except ValueError:
            continue
    exts.sort(key=lambda t: t[1])
    return exts

def _parse_b_core(line: str):
    hh = int(line[1:3]); mi = int(line[3:5]); ss = int(line[5:7])
    lat_deg = int(line[7:9]); lat_mm_thousandths = int(line[9:14]); lat_hem = line[14]
    lon_deg = int(line[15:18]); lon_mm_thousandths = int(line[18:23]); lon_hem = line[23]
    fix_valid = line[24]
    alt_press = int(line[25:30])      # meters
    alt_gps = float(line[30:35])      # meters
    lat = lat_deg + (lat_mm_thousandths / 1000.0) / 60.0
    lon = lon_deg + (lon_mm_thousandths / 1000.0) / 60.0
    if lat_hem == 'S': lat = -lat
    if lon_hem == 'W': lon = -lon
    return hh, mi, ss, lat, lon, (fix_valid == 'A'), alt_press, alt_gps

def parse_igc_with_ext(igc_text: str):
    date = parse_igc_date(igc_text)
    i_line = next((ln.strip() for ln in igc_text.splitlines() if ln.startswith('I')), None)
    exts = parse_b_extensions_map(i_line) if i_line else []
    points: List[dict] = []
    for line in igc_text.splitlines():
        if not line.startswith('B'):
            continue
        try:
            hh, mi, ss, lat, lon, valid, alt_press, alt_gps = _parse_b_core(line.strip())
            ext_values = {}
            for code, s, e in exts:
                ext_values[code] = line[s:e] if e <= len(line) and s < e else ""
            t = datetime.combine(date, datetime.min.time()) + timedelta(hours=hh, minutes=mi, seconds=ss)
            points.append({
                "time": t, "lat": lat, "lon": lon, "valid": valid,
                "alt_press": alt_press, "alt_gps": alt_gps, "ext": ext_values
            })
        except Exception:
            continue
    # Midnight wrap
    for i in range(1, len(points)):
        if points[i]["time"] < points[i-1]["time"]:
            if (points[i-1]["time"] - points[i]["time"]).total_seconds() > 12*3600:
                for j in range(i, len(points)):
                    points[j]["time"] += timedelta(days=1)
            break
    return date, points, exts

# --------------------------- Helpers & metrics ---------------------------

def haversine_m(g1: Tuple[float,float], g2: Tuple[float,float]) -> float:
    R = 6371000.0
    lat1, lon1 = g1; lat2, lon2 = g2
    phi1, phi2 = radians(lat1), radians(lat2)
    dphi = radians(lat2 - lat1); dl = radians(lon2 - lon1)
    a = sin(dphi/2)**2 + cos(phi1)*cos(phi2)*sin(dl/2)**2
    return 2 * R * atan2(sqrt(a), sqrt(1 - a))

def compute_groundspeed_mps(points: List[dict]) -> List[float]:
    v = [0.0]*len(points)
    for i in range(1, len(points)):
        dt = (points[i]["time"] - points[i-1]["time"]).total_seconds()
        if dt <= 0:
            v[i] = v[i-1]
            continue
        d = haversine_m((points[i-1]["lat"], points[i-1]["lon"]), (points[i]["lat"], points[i]["lon"]))
        v[i] = d/dt
    if len(v) >= 2: v[0] = v[1]
    return v

def choose_alt_series(points: List[dict]) -> List[float]:
    ap = [p["alt_press"] for p in points]; ag = [p["alt_gps"] for p in points]
    if sum(1 for x in ap if x != 0) > 0.7*len(points) and (max(ap)-min(ap) > 1.0):
        return ap
    return ag

def compute_derived_climb_mps(points: List[dict]) -> List[float]:
    alts = choose_alt_series(points)
    w = [0.0]*len(points)
    for i in range(1, len(points)):
        dt = (points[i]["time"] - points[i-1]["time"]).total_seconds()
        if dt <= 0:
            w[i] = w[i-1]
            continue
        w[i] = (alts[i]-alts[i-1])/dt
    if len(w) >= 2: w[0] = w[1]
    return w

def linear_interpolate(p1: dict, p2: dict, t: datetime) -> dict:
    t1, t2 = p1["time"], p2["time"]
    if t2 == t1: return dict(p1)
    u = max(0.0, min(1.0, (t - t1).total_seconds() / (t2 - t1).total_seconds()))
    return {
        "time": t,
        "lat": p1["lat"] + u*(p2["lat"] - p1["lat"]),
        "lon": p1["lon"] + u*(p2["lon"] - p1["lon"]),
        "alt_gps": p1["alt_gps"] + u*(p2["alt_gps"] - p1["alt_gps"]),
        "alt_press": int(round(p1["alt_press"] + u*(p2["alt_press"] - p1["alt_press"]))),
        "valid": True,
        "ext": p1["ext"],  # keep last-known extension packet
    }

def resample_to_max(points: List[dict], max_points: int = 10000) -> List[dict]:
    if len(points) <= max_points or len(points) < 2:
        return points
    t0, tN = points[0]["time"], points[-1]["time"]
    total = (tN - t0).total_seconds()
    if total <= 0:
        step = len(points)/max_points
        return [points[int(i*step)] for i in range(max_points)]
    res = []
    j = 0
    for i in range(max_points):
        ti = t0 + timedelta(seconds = (total*i)/(max_points-1))
        while j+1 < len(points) and points[j+1]["time"] < ti:
            j += 1
        res.append(points[-1] if j+1 >= len(points) else linear_interpolate(points[j], points[j+1], ti))
    return res

# --------------------------- Extension decoding ---------------------------

def _parse_signed(s: str) -> Optional[int]:
    if not s: return None
    s = s.strip()
    if not s: return None
    neg = False
    if s.endswith('-'): neg = True; s = s[:-1]
    if s.startswith('-'): neg = True; s = s[1:]
    if not s.isdigit(): return None
    v = int(s)
    return -v if neg else v

def _is_sentinel(s: str) -> bool:
    if not s: return True
    s = s.strip()
    return not s or all(ch == '9' for ch in s)

def _collect_raw(points: List[dict], code: str) -> List[Optional[int]]:
    out = []
    for p in points:
        s = p["ext"].get(code, "")
        if _is_sentinel(s): out.append(None)
        else: out.append(_parse_signed(s))
    return out

def _decide_speed_factor(raw_any: List[Optional[int]]) -> float:
    xs = [v for v in raw_any if v is not None and v < 99990]
    if not xs: return 0.01  # safe default
    mx = max(xs)
    # Common encodings (your file: 0.01 km/h)
    return 0.01 if mx >= 1500 else (0.1 if mx >= 200 else 1.0)

def _decide_vat_factor(raw: List[Optional[int]]) -> float:
    xs = [v for v in raw if v is not None and abs(v) < 9999]
    if not xs: return 0.01
    mxa = max(abs(v) for v in xs)
    return 0.01 if mxa >= 300 else (0.1 if mxa >= 50 else 1.0)

def _decide_oat_factor(raw: List[Optional[int]]) -> float:
    xs = [v for v in raw if v is not None and abs(v) < 9999]
    if not xs: return 0.1
    mxa = max(abs(v) for v in xs)
    return 0.1 if mxa >= 80 else 1.0  # typical OAT 0.1°C units

def _fill_forward(series: List[Optional[float]]) -> List[Optional[float]]:
    last = None; out=[]
    for v in series:
        if v is None: out.append(last)
        else: out.append(v); last=v
    return out

def _fill_missing_numeric(arr: List[Optional[float]], fallback: float = 0.0) -> List[float]:
    arr2 = _fill_forward(arr)
    first = next((v for v in arr2 if v is not None), None)
    if first is None:
        return [fallback]*len(arr2)
    carry = first; out=[]
    for v in arr2:
        if v is None: out.append(carry)
        else: out.append(v); carry=v
    return out

# --------------------------- Ranging & colors ---------------------------

def percentile(vals: List[float], p: float) -> float:
    if not vals: return 0.0
    s = sorted(vals); k = max(0, min(len(s)-1, int(round(p*(len(s)-1)))))
    return s[k]

def trimmed(vals: List[float], ql: float, qh: float) -> List[float]:
    if not vals: return []
    lo, hi = percentile(vals, ql), percentile(vals, qh)
    if hi < lo: lo, hi = hi, lo
    return [v for v in vals if lo <= v <= hi]

def lin_edges(vmin: float, vmax: float, bands: int) -> List[float]:
    if vmax <= vmin: vmax = vmin + 1.0
    return [vmin + i*(vmax-vmin)/bands for i in range(bands+1)]

def color_argb(a: int, r: int, g: int, b: int) -> str:
    return f"{a:02x}{b:02x}{g:02x}{r:02x}"

def lerp(a: float, b: float, t: float) -> float: return a + (b-a)*t
def hex_to_rgb(h: str): h=h.lstrip('#'); return (int(h[0:2],16), int(h[2:4],16), int(h[4:6],16))
def lerp_rgb(c1, c2, t): return (int(round(lerp(c1[0],c2[0],t))), int(round(lerp(c1[1],c2[1],t))), int(round(lerp(c1[2],c2[2],t))))
RAINBOW = [(0.00, hex_to_rgb("#4B0082")), (0.16,hex_to_rgb("#0000FF")), (0.33,hex_to_rgb("#00FFFF")),
           (0.50,hex_to_rgb("#00FF00")), (0.66,hex_to_rgb("#FFFF00")), (0.83,hex_to_rgb("#FFA500")),
           (1.00,hex_to_rgb("#FF0000"))]
def rainbow_color(v, vmin, vmax):
    if vmax<=vmin: t=0.5
    else: t = max(0.0, min(1.0, (v-vmin)/(vmax-vmin)))
    for i in range(len(RAINBOW)-1):
        t0,c0 = RAINBOW[i]; t1,c1 = RAINBOW[i+1]
        if t0 <= t <= t1:
            u = 0.0 if t1==t0 else (t-t0)/(t1-t0)
            return lerp_rgb(c0,c1,u)
    return RAINBOW[-1][1]

def ensure_style(styles: Dict[str,str], style_id: str, color_hex: str, width: int=3):
    if style_id not in styles:
        styles[style_id] = (
            f"\n    <Style id=\"{style_id}\">\n"
            f"      <LineStyle><color>{color_hex}</color><width>{width}</width></LineStyle>\n"
            f"    </Style>"
        )

def kml_coords_string(coords: List[Tuple[float,float,float]]) -> str:
    return "\n".join(f"{lon:.6f},{lat:.6f},{alt:.2f}" for lon,lat,alt in coords)

def compress_coords(coords: List[Tuple[float,float,float]], min_2d_m: float = 0.5, min_dalt_m: float = 0.5):
    if not coords: return coords
    kept=[coords[0]]
    for lon,lat,alt in coords[1:]:
        lon0,lat0,alt0 = kept[-1]
        if lon==lon0 and lat==lat0 and abs(alt-alt0)<1e-6: continue
        d = haversine_m((lat0,lon0),(lat,lon))
        if d < min_2d_m and abs(alt-alt0) < min_dalt_m: continue
        kept.append((lon,lat,alt))
    return kept

def has_two_unique_coords(coords: List[Tuple[float,float,float]]):
    if len(coords)<2: return False
    seen=set()
    for lon,lat,alt in coords:
        key=(round(lon,6),round(lat,6),round(alt,2))
        seen.add(key)
        if len(seen)>=2: return True
    return False

def segment_runs_by_band(values: List[float], edges: List[float], min_run_pts: int=MIN_RUN_PTS):
    if not values: return []
    def band(v):
        for i in range(len(edges)-1):
            if edges[i] <= v <= edges[i+1]: return i
        return len(edges)-2
    cur = band(values[0]); start=0; runs=[]
    for i in range(1,len(values)):
        b = band(values[i])
        if b != cur:
            runs.append([start,i-1,cur])
            start=i; cur=b
    runs.append([start,len(values)-1,cur])
    if min_run_pts<=1 or len(runs)<=1: return [(a,b,c) for a,b,c in runs]
    merged=[]; i=0
    while i<len(runs):
        s,e,b = runs[i]; L=e-s+1
        if L>=min_run_pts or len(runs)==1:
            merged.append([s,e,b]); i+=1; continue
        if merged and merged[-1][2]==b: merged[-1][1]=e; i+=1
        elif i+1<len(runs) and runs[i+1][2]==b: runs[i+1][0]=s; i+=1
        else:
            left = merged[-1][1]-merged[-1][0]+1 if merged else -1
            right = runs[i+1][1]-runs[i+1][0]+1 if i+1<len(runs) else -1
            if right>left and i+1<len(runs): runs[i+1][0]=s
            elif merged: merged[-1][1]=e
            else: merged.append([s,e,b])
            i+=1
    return [(a,b,c) for a,b,c in merged]

def build_visualization_folder(title: str, metric_values: List[float], points: List[dict],
                               edges: List[float], color_fn, style_prefix: str, styles: Dict[str,str]) -> str:
    # styles per band (opaque for path, semi-opaque for curtain)
    for i in range(len(edges)-1):
        mid = 0.5*(edges[i]+edges[i+1]); r,g,b = color_fn(mid)
        ensure_style(styles, f"{style_prefix}_band{i}", color_argb(0xFF,r,g,b), width=3)
        ensure_style(styles, f"{style_prefix}_band{i}_curtain", color_argb(0xCC,r,g,b), width=2)

    # full path
    full_coords = compress_coords([(p["lon"],p["lat"],p["alt_gps"]) for p in points])
    folder_xml=[]
    if has_two_unique_coords(full_coords):
        mid = 0.5*(edges[0]+edges[-1]); r,g,b = color_fn(mid); sid=f"{style_prefix}_full"
        ensure_style(styles, sid, color_argb(0xFF,r,g,b), width=3)
        folder_xml.append(f"""
    <Placemark>
      <name>Full Path (single line)</name>
      <description>{title}</description>
      <styleUrl>#{sid}</styleUrl>
      <LineString><tessellate>1</tessellate><altitudeMode>absolute</altitudeMode>
        <coordinates>
{kml_coords_string(full_coords)}
        </coordinates>
      </LineString>
    </Placemark>""")

    # colored segments + curtains
    runs = segment_runs_by_band(metric_values, edges, min_run_pts=MIN_RUN_PTS)
    parts=[]
    for s,e,bidx in runs:
        if e-s+1 < 2: continue
        seg = compress_coords([(points[i]["lon"],points[i]["lat"],points[i]["alt_gps"]) for i in range(s,e+1)])
        if not has_two_unique_coords(seg): continue
        parts.append(f"""
    <Placemark>
      <name>Flight Path Segment</name>
      <description>{title}</description>
      <styleUrl>#{style_prefix}_band{bidx}</styleUrl>
      <LineString><tessellate>1</tessellate><altitudeMode>absolute</altitudeMode>
        <coordinates>
{kml_coords_string(seg)}
        </coordinates>
      </LineString>
    </Placemark>""")
        parts.append(f"""
    <Placemark>
      <name>Flight Path Curtain</name>
      <styleUrl>#{style_prefix}_band{bidx}_curtain</styleUrl>
      <LineString><extrude>1</extrude><tessellate>1</tessellate><altitudeMode>absolute</altitudeMode>
        <coordinates>
{kml_coords_string(seg)}
        </coordinates>
      </LineString>
    </Placemark>""")
    colored = f"""<Folder><name>Colored Segments</name>{''.join(parts)}</Folder>""" if parts else ""
    return f"""<Folder><name>{title}</name>{''.join(folder_xml)}{colored}</Folder>"""

# --------------------------- Build document ---------------------------

def detect_takeoff_landing(points: List[dict]) -> Tuple[int,int]:
    v = compute_groundspeed_mps(points)
    # takeoff: 10 consecutive > 8 km/h; landing: 15 consecutive < 5 km/h
    to=0
    for i in range(9,len(points)):
        if all(v[j] > 8/3.6 for j in range(i-9,i+1)): to=i-9; break
    ld=len(points)-1
    for i in range(len(points)-16, to+14, -1):
        if all(v[j] < 5/3.6 for j in range(i, i+15)): ld = i+14; break
    return max(0,min(to,len(points)-1)), max(to,min(ld,len(points)-1))

def build_ranges(values: List[float], mode: str, clamp: Optional[Tuple[float,float]]=None):
    if not values: return (0.0,1.0), lin_edges(0.0,1.0,12), (lambda v: rainbow_color(v,0.0,1.0))
    if mode == "alt":
        vmin, vmax = min(values), max(values)
        if vmax<=vmin: vmax = vmin+1.0
    else:
        trimmed_vals = trimmed(values, Q_LOW, Q_HIGH) or values
        mean = sum(trimmed_vals)/len(trimmed_vals)
        loq, hiq = percentile(values, Q_LOW), percentile(values, Q_HIGH)
        half = max(mean-loq, hiq-mean)
        vmin, vmax = mean-half, mean+half
        if clamp:
            vmin = max(vmin, clamp[0]); vmax = min(vmax, clamp[1])
            if vmax<=vmin: vmin, vmax = clamp
    edges = lin_edges(vmin, vmax, 12)
    return (vmin, vmax), edges, (lambda v: rainbow_color(v, edges[0], edges[-1]))

def _make_marker_style(styles: Dict[str,str], sid: str, icon_href: str, color_hex: str = "ff0000ff"):
    if sid in styles: return
    styles[sid] = f"""
    <Style id="{sid}">
      <IconStyle>
        <color>{color_hex}</color>
        <scale>1.2</scale>
        <Icon><href>{icon_href}</href></Icon>
      </IconStyle>
      <LabelStyle><scale>0.9</scale></LabelStyle>
    </Style>"""

def build_kml_document(points: List[dict], to_idx: int, ld_idx: int, *,
                       units: str, use_derived_speed: bool, use_derived_climb: bool,
                       scale_speed: float, scale_vat: float, scale_oat: float):
    # select flight segment, clean monotonic time
    seg = points[to_idx:ld_idx+1]
    seg = [p for p in seg if p.get('valid', True)]
    cleaned = [seg[0]]
    for p in seg[1:]:
        if p["time"] > cleaned[-1]["time"]:
            cleaned.append(p)
    seg = cleaned

    # --- Extensions (raw → scaled) ---
    def ext_series(code: str, factor: float) -> List[Optional[float]]:
        arr=[]
        for p in seg:
            s = p["ext"].get(code, "")
            if _is_sentinel(s): arr.append(None); continue
            iv = _parse_signed(s)
            if iv is None: arr.append(None); continue
            arr.append(iv * factor)
        return arr

    tas = ext_series("TAS", scale_speed)  # km/h after factor
    gsp = ext_series("GSP", scale_speed)  # km/h after factor
    vat = ext_series("VAT", scale_vat)    # m/s  after factor
    oat = ext_series("OAT", scale_oat)    # °C   after factor

    # Speed values (default TAS with fallback)
    if use_derived_speed:
        speed_vals = [v*3.6 for v in compute_groundspeed_mps(seg)]  # km/h
        speed_src = "ground track"
    else:
        gs_from_track = [v*3.6 for v in compute_groundspeed_mps(seg)]
        speed_vals = []
        for i in range(len(seg)):
            v = tas[i] if tas[i] is not None else (gsp[i] if gsp[i] is not None else gs_from_track[i])
            speed_vals.append(v)
        speed_src = "TAS/GSP"

    # Climb values (default VAT with fallback to derived)
    if use_derived_climb:
        climb_mps = compute_derived_climb_mps(seg)
        climb_src = "derived"
    else:
        derived = compute_derived_climb_mps(seg)
        if all(v is None for v in vat):
            climb_mps = derived
            climb_src = "derived"
        else:
            climb_mps = [(vat[i] if vat[i] is not None else derived[i]) for i in range(len(seg))]
            climb_src = "VAT"

    # Altitude (for KML z & altitude layer)
    alts_m = [p["alt_gps"] for p in seg]

    # Unit conversions (for labels/ranging only)
    if units == "imperial":
        speed_vals_u = [v*0.621371 for v in speed_vals]       # mph
        climb_vals_u = [v*196.850394 for v in climb_mps]      # ft/min
        alts_val_u   = [h*3.280839895 for h in alts_m]        # ft
        speed_unit, climb_unit, alt_unit, oat_unit = "mph","ft/min","ft","°C"
        climb_clamp = (-1200.0, 1200.0)
    else:
        speed_vals_u = speed_vals                              # km/h
        climb_vals_u = climb_mps                               # m/s
        alts_val_u   = alts_m                                  # m
        speed_unit, climb_unit, alt_unit, oat_unit = "km/h","m/s","m","°C"
        climb_clamp = (-6.0, 6.0)

    # Fill minor gaps for coloring continuity
    speed_vals_u = _fill_missing_numeric(speed_vals_u, fallback=0.0)
    climb_vals_u = _fill_missing_numeric(climb_vals_u, fallback=0.0)
    oat_vals     = _fill_missing_numeric(oat, fallback=(oat[0] if oat and oat[0] is not None else 0.0))

    # Ranges & color functions
    (alt_mm, alt_edges, alt_cfn) = build_ranges(alts_val_u, mode="alt")
    (spd_mm, sp_edges, sp_cfn)   = build_ranges(speed_vals_u, mode="robust")
    (clb_mm, cl_edges, cl_cfn)   = build_ranges(climb_vals_u, mode="robust", clamp=climb_clamp)
    (oat_mm, ot_edges, ot_cfn)   = build_ranges(oat_vals, mode="robust")

    styles: Dict[str,str] = {}

    # Folders (Altitude / Climb / Speed / OAT)
    folders = []
    folders.append(build_visualization_folder(f"Altitude Colored Path ({alt_unit})",
                    alts_val_u, seg, alt_edges, alt_cfn, "alt", styles))
    folders.append(build_visualization_folder(f"Climb Rate Colored Path ({climb_unit}) [{climb_src}]",
                    climb_vals_u, seg, cl_edges, cl_cfn, "climb", styles))
    folders.append(build_visualization_folder(f"Speed Colored Path ({speed_unit}) [{speed_src}]",
                    speed_vals_u, seg, sp_edges, sp_cfn, "speed", styles))
    folders.append(build_visualization_folder(f"Temperature Colored Path ({oat_unit}) [OAT]",
                    oat_vals, seg, ot_edges, ot_cfn, "oat", styles))

    # Waypoints: takeoff/landing + requested extrema
    # Base styles for pins
    _make_marker_style(styles, "takeoff", "http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png", "ff00ff00")
    _make_marker_style(styles, "landing", "http://maps.google.com/mapfiles/kml/shapes/flag.png", "ff00ffff")
    _make_marker_style(styles, "wp_peak_alt", "http://maps.google.com/mapfiles/kml/paddle/red-circle.png", "ff0000ff")
    _make_marker_style(styles, "wp_fast_tas", "http://maps.google.com/mapfiles/kml/paddle/ylw-stars.png", "ff00ffff")
    _make_marker_style(styles, "wp_max_climb", "http://maps.google.com/mapfiles/kml/paddle/grn-circle.png", "ff00ff00")
    _make_marker_style(styles, "wp_max_sink", "http://maps.google.com/mapfiles/kml/paddle/purple-circle.png", "ff800080")
    _make_marker_style(styles, "wp_min_oat", "http://maps.google.com/mapfiles/kml/paddle/blu-circle.png", "ffff0000")

    take = points[to_idx]; land = points[ld_idx]

    def placemark(name: str, style_id: str, pt: dict, extra_desc: str = "") -> str:
        desc = f"{extra_desc}" if extra_desc else ""
        return f"""
      <Placemark>
        <name>{name}</name>
        <styleUrl>#{style_id}</styleUrl>
        <description>{desc}</description>
        <Point><coordinates>{pt['lon']:.6f},{pt['lat']:.6f},{pt['alt_gps']:.2f}</coordinates></Point>
      </Placemark>"""

    # Compute extrema indices on the segment actually used (seg)
    # Highest altitude
    idx_alt_max = max(range(len(seg)), key=lambda i: alts_m[i])
    # Fastest airspeed (based on the speed source actually used for coloring BEFORE unit conversion)
    idx_fast = max(range(len(seg)), key=lambda i: speed_vals[i])
    # Highest climb (max positive)
    idx_climb = max(range(len(seg)), key=lambda i: climb_mps[i])
    # Highest descent (most negative)
    idx_sink = min(range(len(seg)), key=lambda i: climb_mps[i])
    # Lowest OAT
    idx_oat_min = min(range(len(seg)), key=lambda i: oat_vals[i] if i < len(oat_vals) else float('inf'))

    waypoints = f"""
    <Folder>
      <name>Waypoints</name>
      {placemark(f"Takeoff {take['time'].strftime('%H:%M:%S')}", "takeoff", take)}
      {placemark(f"Landing {land['time'].strftime('%H:%M:%S')}", "landing", land)}
      {placemark(f"Highest Altitude: {alts_val_u[idx_alt_max]:.1f} {alt_unit}", "wp_peak_alt", seg[idx_alt_max],
                 extra_desc=f"Time: {seg[idx_alt_max]['time'].strftime('%H:%M:%S')}")}
      {placemark(f"Fastest Airspeed: {speed_vals_u[idx_fast]:.1f} {speed_unit}", "wp_fast_tas", seg[idx_fast],
                 extra_desc=f"Source: {speed_src} | Time: {seg[idx_fast]['time'].strftime('%H:%M:%S')}")}
      {placemark(f"Highest Climb: {climb_vals_u[idx_climb]:.2f} {climb_unit}", "wp_max_climb", seg[idx_climb],
                 extra_desc=f"Source: {climb_src} | Time: {seg[idx_climb]['time'].strftime('%H:%M:%S')}")}
      {placemark(f"Highest Descent: {climb_vals_u[idx_sink]:.2f} {climb_unit}", "wp_max_sink", seg[idx_sink],
                 extra_desc=f"Source: {climb_src} | Time: {seg[idx_sink]['time'].strftime('%H:%M:%S')}")}
      {placemark(f"Lowest Temperature: {oat_vals[idx_oat_min]:.1f} {oat_unit}", "wp_min_oat", seg[idx_oat_min],
                 extra_desc=f"Time: {seg[idx_oat_min]['time'].strftime('%H:%M:%S')}")}
    </Folder>
    """

    doc = f"""<Folder><name>Flight Path Visualization</name>{''.join(folders)}{waypoints}</Folder>"""

    # Debug info to console
    print(f"Speed source: {speed_src} | Climb source: {climb_src}")
    print(f"Speed ({speed_unit}) encountered: min={min(speed_vals_u):.3f}, max={max(speed_vals_u):.3f}")
    print(f"Climb ({climb_unit}) encountered: min={min(climb_vals_u):.3f}, max={max(climb_vals_u):.3f}")
    print(f"OAT ({oat_unit}) encountered: min={min(oat_vals):.3f}, max={max(oat_vals):.3f}")
    print(f"Color ranges → Alt: [{alt_mm[0]:.3f}, {alt_mm[1]:.3f}] {alt_unit} | "
          f"Speed: [{spd_mm[0]:.3f}, {spd_mm[1]:.3f}] {speed_unit} | "
          f"Climb: [{clb_mm[0]:.3f}, {clb_mm[1]:.3f}] {climb_unit} | "
          f"OAT: [{oat_mm[0]:.3f}, {oat_mm[1]:.3f}] {oat_unit}")

    return doc, styles

# --------------------------- Main conversion ---------------------------

def convert_igc_to_kml(igc_path: Path, out_kml_path: Path, *,
                       max_out_pts: int = 10000, units: str = "metric",
                       use_derived_speed: bool = False, use_derived_climb: bool = False):
    text = igc_path.read_text(errors="ignore")
    date, points, exts = parse_igc_with_ext(text)
    if len(points) < 2:
        raise ValueError("Not enough B-record points to build a track.")
    to_idx, ld_idx = detect_takeoff_landing(points)

    # Decide scaling FACTORS from whole file (robust to resampling)
    tas_raw = _collect_raw(points, "TAS")
    gsp_raw = _collect_raw(points, "GSP")
    vat_raw = _collect_raw(points, "VAT")
    oat_raw = _collect_raw(points, "OAT")
    speed_factor = _decide_speed_factor([v for v in tas_raw if v is not None] + [v for v in gsp_raw if v is not None])
    vat_factor   = _decide_vat_factor(vat_raw)
    oat_factor   = _decide_oat_factor(oat_raw)

    # Resample to ≤ max points
    points_rs = resample_to_max(points, max_points=max_out_pts)

    # Map indices by nearest time
    def nearest_idx(ts):
        best_i, best_dt = 0, None
        for i,p in enumerate(points_rs):
            dt = abs((p["time"] - ts).total_seconds())
            if best_dt is None or dt < best_dt:
                best_dt, best_i = dt, i
        return best_i
    to_rs = nearest_idx(points[to_idx]["time"])
    ld_rs = max(to_rs+1, nearest_idx(points[ld_idx]["time"]))

    # Build doc
    doc, styles = build_kml_document(points_rs, to_rs, ld_rs,
                                     units=units,
                                     use_derived_speed=use_derived_speed,
                                     use_derived_climb=use_derived_climb,
                                     scale_speed=speed_factor, scale_vat=vat_factor, scale_oat=oat_factor)

    styles_xml = "\n".join(styles[k] for k in sorted(styles.keys()))
    kml = f"""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2" xmlns:gx="http://www.google.com/kml/ext/2.2">
  <Document>
    <name>IGC Flight {date.strftime('%Y-%m-%d')}</name>
    {styles_xml}
    {doc}
  </Document>
</kml>"""
    out_kml_path.write_text(kml, encoding="utf-8")

    print(f"Wrote KML: {out_kml_path}")
    print(f"Detected takeoff index: {to_idx}, landing index: {ld_idx}, total points: {len(points)}")
    print(f"Output track limited to ≤ {max_out_pts} points (uniform in time).")
    print(f"Units: {units} | Robust quantiles: [{Q_LOW:.2f}, {Q_HIGH:.2f}]")
    print(f"Scaling used → Speed: ×{speed_factor} (km/h per unit), VAT: ×{vat_factor} (m/s per unit), OAT: ×{oat_factor} (°C per unit)")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="IGC → KML with TAS/VAT by default, optional derived speed/climb, OAT layer, and extrema waypoints.")
    parser.add_argument("igc", help="Path to input .igc file")
    parser.add_argument("-o", "--out", default=None, help="Output KML path (default: same name with .kml)")
    parser.add_argument("--max-points", type=int, default=10000, help="Maximum output points (default 10000)")
    parser.add_argument("--units", choices=["metric","imperial"], default="metric", help="Units for labels/ranges")
    parser.add_argument("--use-derived-speed", action="store_true", help="Force derived ground speed from GPS instead of TAS")
    parser.add_argument("--use-derived-climb", action="store_true", help="Force derived climb from altitude instead of VAT")
    args = parser.parse_args()

    igc_path = Path(args.igc)
    out_path = Path(args.out) if args.out else igc_path.with_suffix(".kml")
    convert_igc_to_kml(igc_path, out_path,
                       max_out_pts=args.max_points,
                       units=args.units,
                       use_derived_speed=args.use_derived_speed,
                       use_derived_climb=args.use_derived_climb)

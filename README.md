# igc2kml
IGC converter that creates KML tracks importable into Google Earth Web with colored speed, altitude, climb rate, etc.

## Description
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



## to run it
```
1. clone this repository
2. chmod 755 igc2kml.py
3. ./igc2kml.py <input file.igc>
```
## Options
To get the colors right experiment with --units and --band-mode
```
./igc_to_kml.py path/to/flight.igc -o out.kml --max-points 10000 --units imperial
./igc_to_kml.py path/to/flight.igc --band-mode auto  # use data-driven color bands
```

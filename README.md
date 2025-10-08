# igc2kml
IGC converter that creates KML tracks importable into Google Earth Web with colored speed, altitude, climb rate, etc.

## Description
- Builds a single, continuous flight path (unbroken) and a matching “curtain”
  (vertical drop to terrain) using <extrude>1</extrude>.
- Provides separate visualizations for Altitude, Climb Rate, and Speed:
  - a single continuous “Full Path (single line)” + curtain
  - a “Colored Segments” subfolder (to mimic your sample’s color-by-value look)
- Takeoff/Landing markers.
- If input has >10,000 points, resamples (interpolates) uniformly in time to ≤10,000.
## to run it

1. clone this repository
2. `chmod 755 igc2kml.py`
3. `./igc2kml.py <input file.igc>`

## Options
To get the colors right experiment with `--units {metric,imperial}` and `--band-mode`
```
./igc_to_kml.py path/to/flight.igc -o out.kml --max-points 10000 --units imperial
```
```
./igc_to_kml.py path/to/flight.igc --band-mode auto  # use data-driven color bands
```

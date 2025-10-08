# igc2kml
IGC converter that creates KML tracks importable into Google Earth Web with colored speed, altitude, climb rate, etc.

## Description
- Builds a single, continuous flight path (unbroken) and a matching “curtain”
- Provides separate visualizations for Altitude, Climb Rate, Speed, and Temperature:
- Takeoff/Landing markers.
- If input has >10,000 points, resamples (interpolates) uniformly in time to ≤10,000.
## to run it

1. clone this repository
2. `chmod 755 igc2kml.py`
3. `./igc2kml.py <input file.igc>`

## Options
```
  python igc2kml.py flight.igc
  python igc2kml.py flight.igc --units imperial
  python igc2kml.py flight.igc --use-derived-speed --use-derived-climb
```

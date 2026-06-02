# City Config Schema

Each file in this folder defines one reproducible city demo for the OpenStreetMap pipeline.

Required top-level fields:

- `city_id`: stable slug used for output prefixes
- `display_name`: human-readable label for logs and docs
- `full_bbox`: `{west, south, east, north}` bounding box for the full route
- `full_grid_width`: raster width used for the full-route scenario
- `route`: full-city route definition
- `district`: smaller multi-agent slice for coordination runs
- `simulation` (optional): traffic-control and behavior overlay for the district simulation

`route` fields:

- `scenario_name`: output prefix for the full route files
- `start`: `{lat, lon}` seed snapped to the nearest drivable cell
- `goal`: `{lat, lon}` seed snapped to the nearest drivable cell

`district` fields:

- `name`: display label for the district slice
- `scenario_name`: output prefix for the district files
- `bbox`: `{west, south, east, north}` bounding box for the district
- `agents`: list of `{id, start, goal}` entries using `{lat, lon}` seeds

`simulation` fields:

- `tick_seconds`: wall-clock seconds represented by one simulation tick
- `vehicle_model`: lightweight acceleration envelope and turn-speed parameters
- `sensor_model`: caution penalties and planning buffers near controls/intersections
- `human_driver_model`: reaction and scan delays used by the independent-car baseline
- `traffic_lights`: list of `{id, lat, lon, cycle_ticks, green_ticks, offset_ticks}` entries
- `stop_signs`: list of `{id, lat, lon, hold_ticks}` entries

The builder snaps those traffic-control seeds to the district grid and writes a `*_controls.json` file next to the district scenario, agents, and network outputs.

Generate a city demo with:

```bash
LATTPATH_CITY_CONFIG=configs/cities/manhattan.json \
LATTPATH_CITY_BACKEND=custom \
LATTPATH_OUTPUT_TAG=custom \
bash tools/generate_city_demo.sh
```

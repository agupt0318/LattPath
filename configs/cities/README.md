# City Config Schema

Each file in this folder defines one reproducible city demo for the OpenStreetMap pipeline.

Required top-level fields:

- `city_id`: stable slug used for output prefixes
- `display_name`: human-readable label for logs and docs
- `full_bbox`: `{west, south, east, north}` bounding box for the full route
- `full_grid_width`: raster width used for the full-route scenario
- `route`: full-city route definition
- `district`: smaller multi-agent slice for coordination runs

`route` fields:

- `scenario_name`: output prefix for the full route files
- `start`: `{lat, lon}` seed snapped to the nearest drivable cell
- `goal`: `{lat, lon}` seed snapped to the nearest drivable cell

`district` fields:

- `name`: display label for the district slice
- `scenario_name`: output prefix for the district files
- `bbox`: `{west, south, east, north}` bounding box for the district
- `agents`: list of `{id, start, goal}` entries using `{lat, lon}` seeds

Generate a city demo with:

```bash
LATTPATH_CITY_CONFIG=configs/cities/manhattan.json tools/generate_city_demo.sh
```

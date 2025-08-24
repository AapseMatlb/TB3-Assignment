# Test Case — Semantic Query → Navigation

This short test demonstrates that a free‑text semantic query (e.g., “Where is the toilet?”)
results in navigation to the correct location.

---

## Option A — Offline (works without ROS/RViz)

**Goal:** prove end‑to‑end behavior (query → pose → planned path image) using the mock semantic DB.

```bash
python scripts/semantic_nav_test.py --label toilet --map maps_demo/office.yaml --offline
```

**Expected output:**
- Terminal prints something like:
  ```
  [offline] goal ← toilet: (2.00,-1.50,0.00)
  Saved artifacts/rrt_path_example.png
  [offline] DONE. See artifacts/rrt_path_example.png
  ```
- `artifacts/rrt_path_example.png` contains the computed path to the **toilet** coordinates.

**Pass criteria:**
- The script runs without error and the PNG file is created.
- The plotted path terminates at the configured **toilet** location from `artifacts/semantic_mock.json`.

---

## Option B — ROS 2 (Linux, optional RViz)

**Goal:** call the semantic service and pass the returned pose to the planner.

1) Start with a static map (or set `use_sim:=false` to run live SLAM):
```bash
ros2 launch tb3_exploration slam_explore.launch.py use_sim:=true map_yaml:=maps_demo/office.yaml
```

2) Run the semantic service:
```bash
ros2 run tb3_semantics semantic_mapper
```

3) Ask a free‑text question:
```bash
ros2 service call /get_location example_interfaces/srv/String "{data: 'toilet'}"
```
**Expected response:**
```json
{"label": "toilet", "x": 2.0, "y": -1.5, "theta": 0.0}
```

4) Start the planner to navigate to that pose:
```bash
ros2 run tb3_rrt_planner rrt_planner --ros-args   -p map_yaml:=maps_demo/office.yaml   -p goal:="[2.0, -1.5, 0.0]"
```

5) (Optional) Visualize in RViz:
```bash
ros2 launch tb3_exploration view_map.launch.py
```
Add `/map` (OccupancyGrid) and `/plan` (Path) if not already shown.

**Pass criteria:**
- The semantic service returns a valid pose for the query.
- The planner publishes a non‑empty path on `/plan`.
- In RViz (if used), the path ends at the returned coordinates.

---

## Notes

- Queries are **not hard‑coded**; any string is accepted. Unknown labels produce a clear error.
- The **mock** semantic DB (`artifacts/semantic_mock.json`) can be edited to add more locations.
- The test can be repeated with `--label pantry`, etc.

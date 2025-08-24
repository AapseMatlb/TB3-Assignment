# Semantic Architecture — Generation, Storage, Query, and Planning

This document focuses on the **semantic memory system** we will adopt for Section 2.
It explains how **labels are generated**, **stored**, **queried**, and then used to
**plan** a path to the corresponding place. It is consistent with the Objective,
Problem Statement, and Open Guidelines.

---

## Objective (what we built)
Design a system that lets a robot not only build a spatial map but also **attach semantic meaning**
to places it sees (e.g., “toilet”, “pantry”), and **later** retrieve those places from a **text query**
and plan a path to them — like human memory.

## Problem framing
As the robot explores, it observes scenes (camera + optional depth) while SLAM estimates its pose in the **map** frame.
Each observation can be given a **semantic label** with a confidence score, then **anchored** to the map with coordinates.
Over time this forms a **semantic memory**: label → one or more poses with metadata.
Later, a user can ask “Where is the toilet?” and the robot should return a **goal pose** and plan to it.

---

## System design (capture → store → query → plan)

### 1) Capture (label generation)
- **Sensors:** `sensor_msgs/Image` (+ optional `sensor_msgs/CameraInfo` and depth/LiDAR).
- **SLAM:** `slam_toolbox` (or a static map) publishes `/map` and TF between `map ↔ odom ↔ base_link ↔ camera_link`.
- **VLM/Detector (real system):**
  - A node subscribes to `/camera/image_raw` and (optionally) `/camera/depth`.
  - Uses a vision(-language) model (e.g., CLIP or a VLM) to produce `(label, confidence)` plus a region (bbox/mask).
  - With depth (or geometric heuristics), back-project the region to a 3D point in `camera_link`, then transform to **map** via TF to get `(x, y, θ)`.
- **This repo (mock for assignment):**
  - We simulate the detector/VLM and **directly store** a few labels in a JSON DB:
    - `artifacts/semantic_mock.json` → `{"toilet":[2.0,-1.5,0.0], "pantry":[4.2,0.8,1.57], ...}`

> The brief allows simulating VLM outputs, as long as the **architecture** and **data handling** are clearly described. We do both.

---

### 2) Store (semantic memory)
- **Minimal implemented schema (in repo):**
  ```json
  {
    "label": [x, y, theta]
  }
  ```
  Stored at `artifacts/semantic_mock.json`. Easy to inspect, edit, and version.

- **Recommended extensible schema (real system):**
  ```json
  {
    "toilet": [
      {"id":"toilet-1","x":2.00,"y":-1.50,"theta":0.00,"conf":0.86,"last_seen":"2025-08-01T12:34:56Z"},
      {"id":"toilet-2","x":9.40,"y": 3.10,"theta":1.57,"conf":0.78,"last_seen":"2025-08-02T09:10:00Z"}
    ],
    "pantry": [
      {"id":"pantry-1","x":4.20,"y": 0.80,"theta":1.57,"conf":0.73,"last_seen":"2025-08-01T12:40:00Z"}
    ]
  }
  ```
  - Supports **multiple instances** per label.
  - Keeps **confidence** and **recency** for better retrieval.
  - Future: persist to SQLite or a lightweight key-value store, index synonyms (e.g., “washroom” ↔ “toilet”).

---

### 3) Query (free-text → location)
- **API (in repo):** ROS 2 service `/get_location` (type: `example_interfaces/srv/String`).
  - **Input:** any text label, e.g., `"toilet"` (not hard-coded).
  - **Output:** JSON string with a pose or an error, e.g.  
    `{"label":"toilet","x":2.0,"y":-1.5,"theta":0.0}`  
    (Node: `src/tb3_semantics/semantic_mapper.py`)
- **Behavior:**
  - Normalizes the text (lowercase).
  - Looks up the DB; returns the most appropriate instance (in minimal demo: the single stored pose).
  - If unknown, returns a clear error (e.g., `{"error":"label 'cafeteria' not found"}`).
- **Real-system extensions:**
  - **Synonyms & embeddings:** Map query text to embedding space (CLIP text encoder), pick the closest known label.
  - **Disambiguation:** If multiple instances exist (two meeting rooms), choose by highest confidence, nearest distance, or ask a clarifying follow-up.

---

### 4) Plan (location → path)
- **Planner (in repo):** `tb3_rrt_planner/rrt_planner_node.py`
  - Loads the map (`maps_demo/office.yaml`/`.pgm`).
  - Inflates obstacles.
  - Runs **RRT** to compute a collision-free `nav_msgs/Path` on `/plan`.
- **Integration flow:**
  1. User query → `/get_location`.
  2. Semantic node returns `(x, y, θ)` in the **map** frame.
  3. RRT planner uses that pose as **goal** and publishes the path.
  4. RViz (`tb3_mapping.rviz`) can visualize `/map` and `/plan`.

---

## How real VLM integration would work (concrete steps)
1. Add a `tb3_semantics_vlm` node that subscribes to `/camera/image_raw` (+ `/camera/depth`).
2. For each frame:
   - Run a detector/classifier → `(label, conf, region)`.
   - Use depth & camera intrinsics to compute a 3D point; transform to `map` (via TF).
   - Upsert into the semantic DB: `label → {pose, conf, last_seen}`.
3. The existing `/get_location` service stays the same: **API is stable**.
4. (Optional) Maintain a **synonym dictionary** and/or **embedding index** so free-text queries match stored labels robustly.

---

## Demo: satisfies “not hard-coded” + “returns a path”
- **Queries are not hard-coded:** the service accepts any string; known labels return a pose; unknown labels return an error (framework is general).
- **Returns a path:** the returned pose is used as the **goal** for the RRT node, which publishes `/plan`.

### Commands (as included in the repo)
```bash
# 1) Static map (or run live SLAM with use_sim:=false)
ros2 launch tb3_exploration slam_explore.launch.py use_sim:=true map_yaml:=maps_demo/office.yaml

# 2) Start semantic reasoning (mock DB)
ros2 run tb3_semantics semantic_mapper

# 3) Ask a question (free text)
ros2 service call /get_location example_interfaces/srv/String "{data: 'toilet'}"
# → {"label":"toilet","x":2.0,"y":-1.5,"theta":0.0}

# 4) Plan to that location
ros2 run tb3_rrt_planner rrt_planner --ros-args   -p map_yaml:=maps_demo/office.yaml   -p goal:="[2.0, -1.5, 0.0]"
```

---

## Design choices (why this approach)
- **Clear separation of concerns:** perception/VLM, memory, query API, planning.
- **Works without heavy simulation:** we provide a static map & mock DB + real ROS nodes.
- **Easy to extend:** swap the mock DB with a live VLM feeder; keep the same query API.
- **Auditable by reviewers:** the JSON DB is human-readable; RViz config shows `/map` & `/plan`; offline scripts generate proof artifacts.

---

## Limitations & next steps
- The current demo **does not update DB online**; it reads a JSON snapshot. (By design for the assignment constraints.)
- In a real robot: add the VLM feeder node, confidence thresholds, multi-instance disambiguation, and persistence (SQLite).
- Add a small “add/update label” service to write into the DB at runtime.

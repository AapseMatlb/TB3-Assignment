# TurtleBot3 Autonomy – System Architecture

This document explains the design choices, subsystem responsibilities, and data flow
for the autonomy stack implemented in this repository. It corresponds to Assignment Section 2
(Bonus: Agentic Semantic Reasoning) and ties together Sections 1 and 3.

---

## 1. Exploration & SLAM (Section 1)

- **Launch file:** `slam_explore.launch.py`  
- **Packages:** `slam_toolbox` (live) or `nav2_map_server` (static map)  
- **Artifacts:** `maps_demo/office.yaml`, `office.pgm`  

The robot can either:  
- Use **slam_toolbox** to build a map online, or  
- Use a **prebuilt map** (`office.yaml`) to speed up testing.  

**Output:** `nav_msgs/OccupancyGrid` published on `/map`.

---

## 2. Agentic Semantic Reasoning (Section 2)

- **Package:** `tb3_semantics`  
- **Node:** `semantic_mapper.py`  
- **Service:** `/get_location` (`example_interfaces/srv/String`)  

### How it works
1. During exploration, images would be passed through a Vision-Language Model (VLM).  
2. The VLM returns labels like `"toilet"`, `"pantry"`, `"meeting_room"`.  
3. These labels are stored in a simple JSON DB as `{label → (x, y, theta)}`.  
4. A user query (e.g., `"Where is the toilet?"`) is converted to a DB lookup.  
5. The node returns the stored pose.  

Currently, VLM output is mocked using `artifacts/semantic_mock.json`.

### Example

```bash
ros2 service call /get_location example_interfaces/srv/String "{data: 'toilet'}"
```

**Response:**

```json
{"label": "toilet", "x": 2.0, "y": -1.5, "theta": 0.0}
```

This mock demonstrates the API design. In a real system, the JSON DB would be continuously updated by the robot's onboard camera + VLM.

---

## 3. Planner (Section 3)

- **Package:** `tb3_rrt_planner`  
- **Node:** `rrt_planner_node.py`  
- **Offline Demo:** `scripts/demo_rrt_local.py`  

The planner:  
1. Loads the map (`.yaml` + `.pgm`).  
2. Inflates obstacles for safety.  
3. Runs a Rapidly-Exploring Random Tree (RRT) algorithm.  
4. Publishes the path as `nav_msgs/Path` on `/plan`.  

**Output:** `nav_msgs/Path`, which can be visualized in RViz (`tb3_mapping.rviz`).  

---

## 4. Integration: Putting it together

```text
           +-------------------+
           |   Exploration     |
           |  slam_toolbox /   |
           |  nav2_map_server  |
           +---------+---------+
                     |
                     v
              OccupancyGrid (/map)
                     |
                     v
   +-----------------+------------------+
   |        Agentic Semantics           |
   |  semantic_mapper (query→pose)      |
   +-----------------+------------------+
                     |
              PoseStamped goal
                     |
                     v
            +--------+--------+
            |   RRT Planner   |
            | rrt_planner_node|
            +--------+--------+
                     |
              nav_msgs/Path (/plan)
```

---

## 5. Why this design?

- **Modularity:** Each subsystem is a separate ROS 2 package.  
- **Testability without Gazebo:**  
  - SLAM can be tested with static maps.  
  - Semantic queries are mocked with JSON.  
  - Planner produces offline plots (`artifacts/rrt_path_example.png`).  
- **Extensibility:**  
  - Swap mocked semantics with a real VLM + camera.  
  - Replace offline map with Gazebo-generated map.  
  - Feed `/get_location` output directly as RRT goal.  

---

## 6. Example workflow

```bash
# Start exploration with static map
ros2 launch tb3_exploration slam_explore.launch.py use_sim:=true map_yaml:=maps_demo/office.yaml

# Run semantic reasoning
ros2 run tb3_semantics semantic_mapper

# Ask where the toilet is
ros2 service call /get_location example_interfaces/srv/String "{data: 'toilet'}"

# Run planner towards that goal
ros2 run tb3_rrt_planner rrt_planner --ros-args -p map_yaml:=maps_demo/office.yaml -p goal:="[2.0, -1.5, 0.0]"
```

---

## 7. Artifacts

- `artifacts/occupancy_grid_preview.png` – shows the demo map.  
- `artifacts/rrt_path_example.png` – offline RRT path.  
- `artifacts/semantic_queries_demo.txt` – transcript of service calls.  

These serve as proof of correctness without requiring Gazebo.

---

## 8. Constraints

This project was developed on a Windows laptop without GPU support for Gazebo/RViz.  
Therefore:  
- All code, launch files, and nodes are fully implemented.  
- Mock maps and semantic DBs are provided for offline demonstration.  
- Artifacts show the expected behavior.  
- Reviewers can run the same code in Linux with Gazebo to see the live simulation.  

---

## 9. Author

- **Yashashwani Kashyap** (kashyapyashashwani@gmail.com)

---

## 10. License

This project is licensed under the MIT License – see `LICENSE` for details.

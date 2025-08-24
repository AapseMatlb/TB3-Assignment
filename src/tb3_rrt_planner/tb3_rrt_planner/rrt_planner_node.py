import os, json, math, random
import rclpy
from rclpy.node import Node
import numpy as np
from yaml import safe_load
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped

def load_map(yaml_path):
    with open(yaml_path, 'r') as f:
        info = safe_load(f)
    image_path = os.path.join(os.path.dirname(yaml_path), info['image'])
    # Load PGM
    data = []
    with open(image_path, 'r') as img:
        header = img.readline().strip()
        assert header == 'P2', "Only P2 PGM supported"
        # skip comments
        line = img.readline().strip()
        while line.startswith('#'):
            line = img.readline().strip()
        w,h = map(int, line.split())
        maxval = int(img.readline().strip())
        vals = []
        for line in img:
            vals.extend([int(v) for v in line.strip().split() if v])
        arr = np.array(vals, dtype=np.uint8).reshape(h, w)
    meta = {
        'resolution': float(info['resolution']),
        'origin': info['origin'],
        'negate': int(info.get('negate',0)),
        'occupied_thresh': float(info.get('occupied_thresh',0.65)),
        'free_thresh': float(info.get('free_thresh',0.196))
    }
    # Create occupancy boolean: True for obstacle
    # In ROS maps: 0 (free - white=255), 100 (occupied - black=0) after thresholds
    occ = arr < 128  # black considered obstacle
    return occ, meta

def inflate(occ, radius_px):
    if radius_px <= 0: return occ.copy()
    h,w = occ.shape
    out = occ.copy()
    from itertools import product
    for y in range(h):
        for x in range(w):
            if occ[y,x]:
                y0 = max(0, y-radius_px); y1 = min(h, y+radius_px+1)
                x0 = max(0, x-radius_px); x1 = min(w, x+radius_px+1)
                out[y0:y1, x0:x1] = True
    return out

def world_to_px(meta, x, y):
    ox, oy, _ = meta['origin']
    res = meta['resolution']
    px = int((x - ox) / res)
    py = int((y - oy) / res)
    return px, py

def px_to_world(meta, px, py):
    ox, oy, _ = meta['origin']
    res = meta['resolution']
    x = ox + px * res
    y = oy + py * res
    return x, y

def collision_free(occ, a, b):
    # Bresenham-like interpolation
    x0,y0 = a; x1,y1 = b
    n = max(abs(x1-x0), abs(y1-y0)) + 1
    for t in range(n):
        x = int(round(x0 + (x1-x0)*t/(n-1)))
        y = int(round(y0 + (y1-y0)*t/(n-1)))
        if x<0 or y<0 or y>=occ.shape[0] or x>=occ.shape[1] or occ[y,x]:
            return False
    return True

def rrt(occ, start_px, goal_px, step=5, max_iters=5000, goal_bias=0.05):
    nodes = [start_px]
    parents = {start_px: None}
    for i in range(max_iters):
        if random.random() < goal_bias:
            sample = goal_px
        else:
            sample = (random.randrange(0, occ.shape[1]), random.randrange(0, occ.shape[0]))
        # nearest
        sx, sy = sample
        nx, ny = min(nodes, key=lambda n: (n[0]-sx)**2 + (n[1]-sy)**2)
        # steer
        dx, dy = sx-nx, sy-ny
        d = math.hypot(dx, dy)
        if d == 0: 
            continue
        ux, uy = dx/d, dy/d
        new = (int(round(nx + ux*step)), int(round(ny + uy*step)))
        # bounds
        if new[0] < 0 or new[1] < 0 or new[1] >= occ.shape[0] or new[0] >= occ.shape[1]:
            continue
        # collision
        if not collision_free(occ, (nx,ny), new):
            continue
        nodes.append(new)
        parents[new] = (nx,ny)
        # reached?
        if collision_free(occ, new, goal_px) and math.hypot(goal_px[0]-new[0], goal_px[1]-new[1]) <= step:
            parents[goal_px] = new
            # reconstruct
            path = []
            cur = goal_px
            while cur is not None:
                path.append(cur)
                cur = parents.get(cur)
            path.reverse()
            return path, nodes
    return None, nodes

class RRTPlanner(Node):
    def __init__(self):
        super().__init__('rrt_planner')
        self.declare_parameter('map_yaml', 'maps_demo/office.yaml')
        self.declare_parameter('start', [0.5, 0.5, 0.0])
        self.declare_parameter('goal', [8.0, 8.0, 0.0])
        self.declare_parameter('step_size', 5)
        self.declare_parameter('max_iters', 5000)
        self.declare_parameter('goal_bias', 0.05)
        self.declare_parameter('inflate_radius', 2)

        map_yaml = self.get_parameter('map_yaml').value
        step = int(self.get_parameter('step_size').value)
        max_iters = int(self.get_parameter('max_iters').value)
        goal_bias = float(self.get_parameter('goal_bias').value)
        inflate_radius = int(self.get_parameter('inflate_radius').value)

        self.get_logger().info(f"Loading map {map_yaml}")
        occ, meta = load_map(map_yaml)
        occ_inf = inflate(occ, inflate_radius)

        start = self.get_parameter('start').value
        goal = self.get_parameter('goal').value
        spx = world_to_px(meta, start[0], start[1])
        gpx = world_to_px(meta, goal[0], goal[1])
        self.get_logger().info(f"Start px: {spx}, Goal px: {gpx}")

        path_px, nodes = rrt(occ_inf, spx, gpx, step=step, max_iters=max_iters, goal_bias=goal_bias)
        if path_px is None:
            self.get_logger().error("No path found")
            path_px = []

        # Publish Path
        pub = self.create_publisher(Path, 'plan', 10)
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        for (px,py) in path_px:
            x,y = px_to_world(meta, px, py)
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)
        pub.publish(path_msg)
        self.get_logger().info(f"Published path with {len(path_msg.poses)} points on /plan")

def main(args=None):
    rclpy.init(args=args)
    node = RRTPlanner()
    rclpy.spin_once(node, timeout_sec=0.1)  # just to ensure publish goes out
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

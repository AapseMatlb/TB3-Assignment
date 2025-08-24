#!/usr/bin/env python3
import os, math, random
import rclpy
from rclpy.node import Node
import numpy as np
from yaml import safe_load
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def read_pgm(path):
    """Return (array[h,w] uint8, maxval, magic). Supports P2 (ASCII) and P5 (binary)."""
    with open(path, 'rb') as f:
        magic = f.readline().strip()
        # skip comments
        def _read_non_comment_line():
            line = f.readline()
            while line.startswith(b'#'):
                line = f.readline()
            return line
        wh = _read_non_comment_line().strip()
        while not wh:
            wh = _read_non_comment_line().strip()
        w, h = [int(x) for x in wh.split()]
        maxval = int(_read_non_comment_line().strip())
        if magic == b'P5':
            data = np.frombuffer(f.read(w*h), dtype=np.uint8).reshape(h, w)
            return data, maxval, 'P5'
        elif magic == b'P2':
            # ASCII ints separated by whitespace
            rest = f.read().decode('ascii').split()
            vals = np.array([int(v) for v in rest[:w*h]], dtype=np.uint16).reshape(h, w)
            # scale to 0..255 if needed
            if maxval != 255:
                vals = (vals.astype(np.float32) * (255.0 / maxval)).astype(np.uint8)
            else:
                vals = vals.astype(np.uint8)
            return vals, 255, 'P2'
        else:
            raise ValueError(f"Unsupported PGM magic: {magic}")

def load_map(yaml_path):
    with open(yaml_path, 'r') as f:
        info = safe_load(f)
    image_path = info['image']
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(yaml_path), image_path)

    gray, maxval, magic = read_pgm(image_path)

    res = float(info.get('resolution', 0.05))
    origin = info.get('origin', [0.0, 0.0, 0.0])
    negate = int(info.get('negate', 0))
    occ_th = float(info.get('occupied_thresh', 0.65))
    free_th = float(info.get('free_thresh', 0.196))

    # Normalize gray to [0,1]
    g = gray.astype(np.float32) / 255.0
    # Map-server convention:
    # if negate==0: occ_prob = 1 - g (black=occupied)
    # else:         occ_prob = g     (white=occupied)
    occ_prob = (1.0 - g) if negate == 0 else g

    # Boolean occupancy: True = obstacle
    occ = occ_prob > occ_th
    return occ, {'resolution': res, 'origin': origin, 'occupied_thresh': occ_th, 'free_thresh': free_th}

def inflate(occ, radius_px):
    if radius_px <= 0:
        return occ.copy()
    h, w = occ.shape
    out = occ.copy()
    rr = radius_px
    for y in range(h):
        y0 = max(0, y - rr); y1 = min(h, y + rr + 1)
        for x in range(w):
            if occ[y, x]:
                x0 = max(0, x - rr); x1 = min(w, x + rr + 1)
                out[y0:y1, x0:x1] = True
    return out

def world_to_px(meta, x, y):
    ox, oy, _ = meta['origin']; res = meta['resolution']
    return int((x - ox) / res), int((y - oy) / res)

def px_to_world(meta, px, py):
    ox, oy, _ = meta['origin']; res = meta['resolution']
    return (ox + px * res), (oy + py * res)

def collision_free(occ, a, b):
    x0, y0 = a; x1, y1 = b
    n = max(abs(x1 - x0), abs(y1 - y0)) + 1
    for t in range(n):
        x = int(round(x0 + (x1 - x0) * t / max(n - 1, 1)))
        y = int(round(y0 + (y1 - y0) * t / max(n - 1, 1)))
        if x < 0 or y < 0 or y >= occ.shape[0] or x >= occ.shape[1] or occ[y, x]:
            return False
    return True

def rrt(occ, start_px, goal_px, step=5, max_iters=5000, goal_bias=0.05):
    nodes = [start_px]
    parents = {start_px: None}
    H, W = occ.shape
    for _ in range(max_iters):
        sample = goal_px if (random.random() < goal_bias) else (random.randrange(0, W), random.randrange(0, H))
        # nearest
        sx, sy = sample
        nx, ny = min(nodes, key=lambda n: (n[0] - sx) ** 2 + (n[1] - sy) ** 2)
        # steer
        dx, dy = sx - nx, sy - ny
        d = math.hypot(dx, dy)
        if d == 0:
            continue
        ux, uy = dx / d, dy / d
        new = (int(round(nx + ux * step)), int(round(ny + uy * step)))
        # bounds
        if not (0 <= new[0] < W and 0 <= new[1] < H):
            continue
        # collision
        if not collision_free(occ, (nx, ny), new):
            continue
        nodes.append(new)
        parents[new] = (nx, ny)
        # goal connect
        if collision_free(occ, new, goal_px) and math.hypot(goal_px[0] - new[0], goal_px[1] - new[1]) <= step:
            parents[goal_px] = new
            # reconstruct
            path = []
            cur = goal_px
            while cur is not None:
                path.append(cur)
                cur = parents.get(cur)
            path.reverse()
            return path
    return None

class RRTPlanner(Node):
    def __init__(self):
        super().__init__('rrt_planner')
        self.declare_parameter('map_yaml', 'maps_demo/office.yaml')
        self.declare_parameter('start', [0.5, 0.5, 0.0])
        self.declare_parameter('goal', [2.0, -1.5, 0.0])
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
        occ = inflate(occ, inflate_radius)

        start = self.get_parameter('start').value
        goal = self.get_parameter('goal').value
        spx = world_to_px(meta, start[0], start[1])
        gpx = world_to_px(meta, goal[0], goal[1])

        path_px = rrt(occ, spx, gpx, step=step, max_iters=max_iters, goal_bias=goal_bias)
        if path_px is None:
            self.get_logger().error("No path found")
            path_px = []

        self.plan_pub = self.create_publisher(Path, 'plan', 10)
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for (px, py) in path_px:
            x, y = px_to_world(meta, px, py)
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp = msg.header.stamp
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.plan_pub.publish(msg)
        self.get_logger().info(f"Published path with {len(msg.poses)} points on /plan")

def main(args=None):
    rclpy.init(args=args)
    node = RRTPlanner()
    # Give the message time to go out for simple tests
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

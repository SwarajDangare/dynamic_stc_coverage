#!/usr/bin/env python3
import math
import time
from threading import Thread, Lock

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates


class STCController(Node):
    def __init__(self):
        super().__init__('stc_controller')

        # Grid setup
        self.GRID_SIZE = 10
        self.CELL_SIZE = 1.0
        self.origin_x = -(self.GRID_SIZE * self.CELL_SIZE) / 2.0
        self.origin_y = -(self.GRID_SIZE * self.CELL_SIZE) / 2.0

        # State
        self.visited = set()
        self.obstacles = set()
        self.dynamic_obs = set()
        self.robot_cell = (0, 0)
        self.robot_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.front_dist = float('inf')
        self.first_models = True
        self.lock = Lock()

        # Stats
        self.step_count = 0
        self.revisit_count = 0
        self.replan_count = 0
        self.unreachable = []

        # Publishers
        self.vis_pub = self.create_publisher(Int32MultiArray, '/stc/visited', 10)
        self.obs_pub = self.create_publisher(Int32MultiArray, '/stc/obstacles', 10)
        self.dyn_pub = self.create_publisher(Int32MultiArray, '/stc/dynamic_obstacles', 10)
        self.create_timer(1.0, self.publish_all)

        # Subscribers
        self.create_subscription(Odometry, 'tb1/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, 'tb1/scan', self.scan_cb, 10)
        self.create_subscription(ModelStates, '/model_states', self.models_cb, 10)

        self.cmd_pub = self.create_publisher(Twist, 'tb1/cmd_vel', 10)

        Thread(target=self.run_stc, daemon=True).start()

    def publish_all(self):
        with self.lock:
            vis = [c for cell in self.visited for c in cell]
            obs = [c for cell in self.obstacles for c in cell]
            dyn = [c for cell in self.dynamic_obs for c in cell]
        self.vis_pub.publish(Int32MultiArray(data=vis))
        self.obs_pub.publish(Int32MultiArray(data=obs))
        self.dyn_pub.publish(Int32MultiArray(data=dyn))

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        with self.lock:
            self.robot_pose = {'x': x, 'y': y, 'yaw': yaw}
            self.robot_cell = self.world_to_cell(x, y)

    def scan_cb(self, msg):
        rngs = msg.ranges
        mid = len(rngs) // 2
        window = [r for r in rngs[mid - 10:mid + 10] if not math.isinf(r)]
        with self.lock:
            self.front_dist = min(window) if window else float('inf')

    def models_cb(self, msg):
        static_names = {'red_box_1', 'red_box_2', 'red_box_3', 'red_box_4'}
        dynamic_names = {'blue_box_1', 'blue_box_2'}
        static = set()
        dynamic = set()

        if self.first_models:
            self.get_logger().info(f"Detected models: {msg.name}")
            self.first_models = False

        for name, pose in zip(msg.name, msg.pose):
            cell = self.world_to_cell(pose.position.x, pose.position.y)
            if name in dynamic_names:
                dynamic.add(cell)
            elif name in static_names:
                static.add(cell)

        with self.lock:
            self.obstacles = static.union(dynamic)
            self.dynamic_obs = dynamic

    def world_to_cell(self, wx, wy):
        i = int(math.floor((wx - self.origin_x) / self.CELL_SIZE))
        j = int(math.floor((wy - self.origin_y) / self.CELL_SIZE))
        return (max(0, min(self.GRID_SIZE - 1, i)),
                max(0, min(self.GRID_SIZE - 1, j)))

    def cell_to_center(self, cell):
        i, j = cell
        x = self.origin_x + (i + 0.5) * self.CELL_SIZE
        y = self.origin_y + (j + 0.5) * self.CELL_SIZE
        return x, y

    def bfs_path(self, start, goal):
        from collections import deque
        DIR = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        q = deque([(start, [start])])
        seen = {start}
        with self.lock:
            obs = set(self.obstacles)
        while q:
            cur, path = q.popleft()
            if cur == goal:
                return path
            for dx, dy in DIR:
                nb = (cur[0] + dx, cur[1] + dy)
                if (0 <= nb[0] < self.GRID_SIZE and
                        0 <= nb[1] < self.GRID_SIZE and
                        nb not in seen and nb not in obs):
                    seen.add(nb)
                    q.append((nb, path + [nb]))
        return []

    def rotate_to(self, target_yaw):
        rate = 10
        while True:
            with self.lock:
                cy = self.robot_pose['yaw']
            err = (target_yaw - cy + math.pi) % (2 * math.pi) - math.pi
            if abs(err) < 0.02:
                break
            cmd = Twist()
            cmd.angular.z = 0.5 * err
            self.cmd_pub.publish(cmd)
            time.sleep(1.0 / rate)
        self.cmd_pub.publish(Twist())
        time.sleep(0.1)

    def drive_forward(self, distance):
        with self.lock:
            sx, sy = self.robot_pose['x'], self.robot_pose['y']
        rate = 10
        while True:
            with self.lock:
                dx = self.robot_pose['x'] - sx
                dy = self.robot_pose['y'] - sy
                traveled = math.hypot(dx, dy)
                front = self.front_dist
            if front < 0.2:
                self.cmd_pub.publish(Twist())
                time.sleep(0.1)
                continue
            if traveled >= distance - 0.05:
                break
            cmd = Twist()
            cmd.linear.x = 0.2
            self.cmd_pub.publish(cmd)
            time.sleep(1.0 / rate)
        self.cmd_pub.publish(Twist())

    def move_to_cell(self, cell):
        if cell in self.visited:
            self.revisit_count += 1

        with self.lock:
            curr = self.robot_cell

        dx, dy = cell[0] - curr[0], cell[1] - curr[1]
        if abs(dx) + abs(dy) != 1:
            self.get_logger().error(f"Invalid step {curr}→{cell}")
            return False

        yaw_map = {(1, 0): 0.0, (-1, 0): math.pi,
                   (0, 1): math.pi / 2, (0, -1): -math.pi / 2}
        self.rotate_to(yaw_map[(dx, dy)])

        with self.lock:
            if self.front_dist < 0.2:
                self.get_logger().info(f"Blocked at {cell}")
                return False

        self.drive_forward(self.CELL_SIZE)

        with self.lock:
            self.robot_cell = cell
            self.visited.add(cell)

        self.step_count += 1
        return True

    def run_stc(self):
        t0 = time.time()
        time.sleep(1.0)

        with self.lock:
            self.get_logger().info(f"Static obstacles: {sorted(self.obstacles)}")

        order = []
        for j in range(self.GRID_SIZE):
            row = list(range(self.GRID_SIZE))
            if j % 2 == 1:
                row.reverse()
            for i in row:
                order.append((i, j))

        for goal in order:
            if goal in self.visited or goal in self.obstacles:
                continue
            path = self.bfs_path(self.robot_cell, goal)
            if not path:
                continue
            for step in path[1:]:
                time.sleep(0.2)
                with self.lock:
                    if step in self.dynamic_obs:
                        self.get_logger().info(f"Dynamic block at {step}")
                        break
                if self.move_to_cell(step):
                    self.publish_all()

        remaining = [
            (x, y)
            for x in range(self.GRID_SIZE)
            for y in range(self.GRID_SIZE)
            if (x, y) not in self.visited and (x, y) not in self.obstacles
        ]

        for cell in remaining:
            if not rclpy.ok():
                return
            path = self.bfs_path(self.robot_cell, cell)
            retries = 0
            while not path and retries < 5:
                retries += 1
                self.replan_count += 1
                time.sleep(0.2)
                path = self.bfs_path(self.robot_cell, cell)
            if not path:
                self.unreachable.append(cell)
                self.get_logger().error(f"No path to {cell} after retries")
                continue
            for step in path[1:]:
                time.sleep(0.2)
                with self.lock:
                    if step in self.dynamic_obs:
                        self.get_logger().info(f"Blocked at {step}, aborting")
                        break
                if self.move_to_cell(step):
                    self.publish_all()

        t1 = time.time()
        total_cells = self.GRID_SIZE * self.GRID_SIZE - len(self.obstacles)
        unique_visits = len(self.visited)
        efficiency = unique_visits / max(1, self.step_count)

        self.get_logger().info("=== Coverage Statistics ===")
        self.get_logger().info(f"Time elapsed: {t1 - t0:.1f}s")
        self.get_logger().info(f"Steps: {self.step_count}, Unique: {unique_visits}/{total_cells}")
        self.get_logger().info(f"Revisits: {self.revisit_count}, Replans: {self.replan_count}")
        self.get_logger().info(f"Efficiency: {efficiency * 100:.1f}%")
        if self.unreachable:
            self.get_logger().warn(f"Unreachable cells: {self.unreachable}")
        self.get_logger().info("=== End Statistics ===")


def main():
    rclpy.init()
    node = STCController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

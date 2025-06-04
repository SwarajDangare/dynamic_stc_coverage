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

        # --- Parameters for grid & subgrid bounds ---
        self.declare_parameter('grid_size', 60)
        self.declare_parameter('cell_size', 0.5)
        self.declare_parameter('robot_list', ['tb1', 'tb2', 'tb3', 'tb4', 'tb5', 'tb6'])
        self.declare_parameter('sub_imin', 0)
        self.declare_parameter('sub_imax', 59)
        self.declare_parameter('sub_jmin', 0)
        self.declare_parameter('sub_jmax', 59)

        # Grid setup
        self.GRID_SIZE = self.get_parameter('grid_size').value       # 60
        self.CELL_SIZE = self.get_parameter('cell_size').value       # 0.5
        half_world = (self.GRID_SIZE * self.CELL_SIZE) / 2.0
        self.origin_x = -half_world     # -15.0
        self.origin_y = -half_world     # -15.0

        # Subgrid bounds (integer cell indices)
        self.sub_imin = self.get_parameter('sub_imin').value
        self.sub_imax = self.get_parameter('sub_imax').value
        self.sub_jmin = self.get_parameter('sub_jmin').value
        self.sub_jmax = self.get_parameter('sub_jmax').value

        # List of all robot namespaces, to detect other robots
        all_robots = set(self.get_parameter('robot_list').value)
        self.robot_ns = self.get_namespace().lstrip('/')  # e.g. 'tb3'
        self.other_robots = all_robots - {self.robot_ns}

        # State
        self.visited = set()       # set of integer cells visited
        self.obstacles = set()     # set of integer cells with any obstacle
        self.dynamic_obs = set()   # set of integer cells with moving obstacles
        self.robot_cell = (0, 0)   # current integer cell of this robot
        self.robot_pose = {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
        self.front_dist = float('inf')
        self.first_models = True
        self.lock = Lock()

        # Static & dynamic box names
        # (adjust 1..30 if you truly have 30 static boxes; here we assume 30)
        self.static_names = {f'red_box_{i}' for i in range(1, 31)}   # 30 static boxes
        self.dynamic_box_names = {f'blue_box_{i}' for i in range(1, 11)}  # 10 dynamic boxes

        # Stats
        self.step_count = 0
        self.revisit_count = 0
        self.replan_count = 0
        self.unreachable = []

        # Publishers (namespaced automatically, because this node is under /<ns>)
        self.vis_pub = self.create_publisher(Int32MultiArray, 'stc/visited', 10)
        self.obs_pub = self.create_publisher(Int32MultiArray, 'stc/obstacles', 10)
        self.dyn_pub = self.create_publisher(Int32MultiArray, 'stc/dynamic_obstacles', 10)
        self.create_timer(1.0, self.publish_all)  # publish once per second

        # Subscribers (all namespaced: /<ns>/odom, /<ns>/scan)
        self.create_subscription(Odometry, 'odom',       self.odom_cb,   10)
        self.create_subscription(LaserScan, 'scan',      self.scan_cb,   10)
        # ModelStates is global (/model_states), not namespaced
        self.create_subscription(ModelStates, '/model_states', self.models_cb, 10)

        # Command publisher (namespaced: publishes to /<ns>/cmd_vel)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Start STC sweep in a background thread (so we never block ROS spin)
        Thread(target=self.run_stc, daemon=True).start()

    def publish_all(self):
        """Publish visited, obstacles, and dynamic obstacle sets once per second."""
        with self.lock:
            vis = [c for cell in self.visited    for c in cell]
            obs = [c for cell in self.obstacles  for c in cell]
            dyn = [c for cell in self.dynamic_obs for c in cell]
        self.vis_pub.publish(Int32MultiArray(data=vis))
        self.obs_pub.publish(Int32MultiArray(data=obs))
        self.dyn_pub.publish(Int32MultiArray(data=dyn))

    def odom_cb(self, msg: Odometry):
        """Track the robot’s current x, y, yaw, and compute integer cell."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        with self.lock:
            self.robot_pose = {'x': x, 'y': y, 'yaw': yaw}
            self.robot_cell = self.world_to_cell(x, y)

    def scan_cb(self, msg: LaserScan):
        """Keep track of the minimum forward distance (middle ~20 rays)."""
        rngs = msg.ranges
        mid = len(rngs) // 2
        window = [r for r in rngs[mid - 10:mid + 10] if not math.isinf(r)]
        with self.lock:
            self.front_dist = min(window) if window else float('inf')

    def models_cb(self, msg: ModelStates):
        """
        Every time /model_states updates, compute which integer cells are statically
        occupied (red boxes) or dynamically occupied (blue boxes or other robots).
        """
        static_cells = set()
        dynamic_cells = set()

        for name, pose in zip(msg.name, msg.pose):
            cell = self.world_to_cell(pose.position.x, pose.position.y)
            if name in self.static_names:
                static_cells.add(cell)
            if name in self.dynamic_box_names or name in self.other_robots:
                dynamic_cells.add(cell)

        with self.lock:
            # Everything that’s either a red_box or a blue_box or another robot counts as an obstacle
            self.obstacles = static_cells.union(dynamic_cells)
            self.dynamic_obs = dynamic_cells

    def world_to_cell(self, wx, wy):
        """Convert world (x,y) → integer cell (i,j).  Clamped to [0..GRID_SIZE-1]."""
        i = int(math.floor((wx - self.origin_x) / self.CELL_SIZE))
        j = int(math.floor((wy - self.origin_y) / self.CELL_SIZE))
        return (
            max(0, min(self.GRID_SIZE - 1, i)),
            max(0, min(self.GRID_SIZE - 1, j))
        )

    def cell_to_center(self, cell):
        """Given integer cell (i,j), return world coordinates of that cell’s center."""
        i, j = cell
        x = self.origin_x + (i + 0.5) * self.CELL_SIZE
        y = self.origin_y + (j + 0.5) * self.CELL_SIZE
        return x, y

    def bfs_path(self, start_cell, goal_cell):
        """
        Standard 4‐connected BFS on integer cells, avoiding any cells in self.obstacles.
        Returns a list [start_cell, …, goal_cell] if reachable, else [].
        """
        from collections import deque
        DIR = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        q = deque([(start_cell, [start_cell])])
        seen = {start_cell}
        with self.lock:
            obs = set(self.obstacles)

        while q:
            cur, path = q.popleft()
            if cur == goal_cell:
                return path
            for dx, dy in DIR:
                nb = (cur[0] + dx, cur[1] + dy)
                if (
                    0 <= nb[0] < self.GRID_SIZE and
                    0 <= nb[1] < self.GRID_SIZE and
                    nb not in seen and
                    nb not in obs
                ):
                    seen.add(nb)
                    q.append((nb, path + [nb]))
        return []

    def rotate_to(self, target_yaw):
        """
        Rotate in place until the robot’s yaw is within ±0.02 rad of target_yaw.
        We spin at ±1.0 rad/s for faster turning (gain = 1.0).
        """
        rate = 20  # 20 Hz update
        while rclpy.ok():
            with self.lock:
                cy = self.robot_pose['yaw']
            err = (target_yaw - cy + math.pi) % (2 * math.pi) - math.pi
            if abs(err) < 0.02:
                break
            cmd = Twist()
            cmd.angular.z = 1.0 * err  # 1.0 gain → ~1 rad/s turn when err ≈ 1 rad
            self.cmd_pub.publish(cmd)
            time.sleep(1.0 / rate)

        # Stop rotation
        self.cmd_pub.publish(Twist())
        # Small pause to ensure robot has settled
        time.sleep(0.05)

    def drive_forward(self, distance):
        """
        Drive straight for exactly ‘distance’ meters, at 0.5 m/s (instead of 0.2). 
        If an obstacle is detected within 0.5 m, we halt and retry after 0.05 s.
        """
        with self.lock:
            sx, sy = self.robot_pose['x'], self.robot_pose['y']
        rate = 20  # 20 Hz
        while rclpy.ok():
            with self.lock:
                dx = self.robot_pose['x'] - sx
                dy = self.robot_pose['y'] - sy
                traveled = math.hypot(dx, dy)
                front = self.front_dist
            if front < 0.5:
                # If something is <0.5 m ahead, stop, wait, then try again
                self.cmd_pub.publish(Twist())
                time.sleep(0.05)
                continue
            if traveled >= distance - 0.01:  # 1 cm tolerance
                break
            cmd = Twist()
            cmd.linear.x = 0.5  # 0.5 m/s forward (was 0.2)
            self.cmd_pub.publish(cmd)
            time.sleep(1.0 / rate)

        # Stop
        self.cmd_pub.publish(Twist())

    def move_to_cell(self, cell):
        """
        Step from current integer cell to an adjacent cell ‘cell’ (1‐step in i or j).
        Return True if we actually moved.  If already in visited, increments revisit count.
        """
        if cell in self.visited:
            self.revisit_count += 1

        with self.lock:
            curr = self.robot_cell

        dx = cell[0] - curr[0]
        dy = cell[1] - curr[1]
        # Must be a 4‐neighbor
        if abs(dx) + abs(dy) != 1:
            self.get_logger().error(f"Invalid step {curr}→{cell}")
            return False

        # Map the direction to yaw
        yaw_map = {
            ( 1,  0):  0.0,        # face +X
            (-1,  0):  math.pi,    # face –X
            ( 0,  1):  math.pi/2,  # face +Y
            ( 0, -1): -math.pi/2   # face –Y
        }
        self.rotate_to(yaw_map[(dx, dy)])

        with self.lock:
            if self.front_dist < 0.2:
                # If blocked right in front, abort this step
                self.get_logger().info(f"Blocked at {cell}")
                return False

        # Move forward one full cell
        self.drive_forward(self.CELL_SIZE)

        with self.lock:
            self.robot_cell = cell
            self.visited.add(cell)

        self.step_count += 1
        return True

    def run_stc(self):
        """
        This method runs in a **background thread** (daemon=True).  It:
        1) Builds a serpentine order over the integer subgrid [sub_imin..sub_imax]×[sub_jmin..sub_jmax].  
        2) First pass: attempts each cell by BFS→move.  
        3) Second pass: any leftovers after re‐planning.
        Because this is in a separate thread, it never blocks ROS2’s spin() in the main thread.
        """
        # Let everything come up (a 1 s pause)
        time.sleep(1.0)

        with self.lock:
            self.get_logger().info(f"Static obstacles: {sorted(self.obstacles)}")

        # 1) Build integer‐cell serpentine order
        order = []
        w, e = self.sub_imin, self.sub_imax
        s, n = self.sub_jmin, self.sub_jmax
        for j in range(s, n + 1):
            row = list(range(w, e + 1))
            # serpentine: reverse every other row
            if (j - s) % 2 == 1:
                row.reverse()
            for i in row:
                order.append((i, j))

        # 2) First pass: visit cells in that exact serpentine order
        for goal_cell in order:
            with self.lock:
                if goal_cell in self.visited or goal_cell in self.obstacles:
                    continue

            path = self.bfs_path(self.robot_cell, goal_cell)
            if not path:
                continue

            for next_cell in path[1:]:
                # Very small pause so we don’t hammer the CPU
                time.sleep(0.05)

                with self.lock:
                    if next_cell in self.dynamic_obs:
                        self.get_logger().info(f"Dynamic block at {next_cell}")
                        break

                if self.move_to_cell(next_cell):
                    self.publish_all()

        # 3) Second pass: any remaining unvisited cells inside the subgrid
        subgrid_cells = [
            (i, j)
            for i in range(self.sub_imin, self.sub_imax + 1)
            for j in range(self.sub_jmin, self.sub_jmax + 1)
            if (i, j) not in self.visited and (i, j) not in self.obstacles
        ]

        for goal_cell in subgrid_cells:
            if not rclpy.ok():
                return

            path = self.bfs_path(self.robot_cell, goal_cell)
            retries = 0
            while not path and retries < 5:
                retries += 1
                self.replan_count += 1
                time.sleep(0.05)
                path = self.bfs_path(self.robot_cell, goal_cell)

            if not path:
                self.unreachable.append(goal_cell)
                self.get_logger().error(f"No path to {goal_cell} after retries")
                continue

            for next_cell in path[1:]:
                time.sleep(0.05)
                with self.lock:
                    if next_cell in self.dynamic_obs:
                        self.get_logger().info(f"Blocked at {next_cell}, aborting")
                        break
                if self.move_to_cell(next_cell):
                    self.publish_all()

        # 4) Print coverage statistics (only counting that robot’s subgrid)
        t1 = time.time()
        subgrid_cells_set = {
            (i, j)
            for i in range(self.sub_imin, self.sub_imax + 1)
            for j in range(self.sub_jmin, self.sub_jmax + 1)
        }
        blocked_in_subgrid = len(self.obstacles.intersection(subgrid_cells_set))
        total_cells = ((self.sub_imax - self.sub_imin + 1) *
                       (self.sub_jmax - self.sub_jmin + 1)
                       - blocked_in_subgrid)
        unique_visits = len(self.visited)
        efficiency = unique_visits / max(1, self.step_count)

        self.get_logger().info("=== Coverage Statistics ===")
        # self.get_logger().info(f"Time elapsed: {t1 - t0:.1f}s")
        self.get_logger().info(
            f"Steps: {self.step_count}, Unique: {unique_visits}/{total_cells}")
        self.get_logger().info(
            f"Revisits: {self.revisit_count}, Replans: {self.replan_count}")
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

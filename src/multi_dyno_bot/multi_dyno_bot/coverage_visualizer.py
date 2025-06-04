#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

import math

class CoverageVisualizer(Node):
    def __init__(self):
        super().__init__('coverage_visualizer')

        # --- parameters ---
        self.declare_parameter('robot_list', ['tb1', 'tb2', 'tb3', 'tb4', 'tb5', 'tb6'])
        robots = self.get_parameter('robot_list').value
        self.robot_list = robots   # e.g. ['tb1','tb2',…, 'tb6']

        # GRID parameters must match STCController
        self.GRID_SIZE = 60
        self.CELL_SIZE = 0.5
        half = (self.GRID_SIZE * self.CELL_SIZE) / 2.0
        self.origin_x = -half  # -15.0
        self.origin_y = -half  # -15.0

        # Store visited cells per robot: { 'tb1': set((i,j), …), … }
        self.visited = {rb: set() for rb in robots}
        # Store path (list of geometry_msgs/Point) per robot
        self.paths   = {rb: [] for rb in robots}

        # colors for each robot (RGB)
        self.colors = {
            'tb1': (1.0, 0.0, 0.0),  # red
            'tb2': (0.0, 1.0, 0.0),  # green
            'tb3': (0.0, 0.0, 1.0),  # blue
            'tb4': (1.0, 1.0, 0.0),  # yellow
            'tb5': (1.0, 0.0, 1.0),  # magenta
            'tb6': (0.0, 1.0, 1.0),  # cyan
        }

        # Subscribers
        for rb in robots:
            # Subscribe to /<rb>/stc/visited
            self.create_subscription(
                Int32MultiArray,
                f'{rb}/stc/visited',
                lambda msg, r=rb: self.visited_cb(msg, r),
                10
            )
            # Subscribe to /<rb>/odom to record path
            self.create_subscription(
                Odometry,
                f'{rb}/odom',
                lambda msg, r=rb: self.odom_cb(msg, r),
                10
            )

        # Publisher for MarkerArray
        self.marker_pub = self.create_publisher(MarkerArray, 'coverage_markers', 10)

        # Timer: publish markers at 10 Hz
        self.create_timer(0.1, self.publish_markers)

    def visited_cb(self, msg: Int32MultiArray, robot: str):
        """
        msg.data is a flat list [i0, j0,  i1, j1,  …].
        We group them into (i,j) pairs.
        """
        cells = set()
        data = msg.data
        for k in range(0, len(data), 2):
            i = data[k]
            j = data[k+1]
            cells.add((i, j))
        self.visited[robot] = cells

    def odom_cb(self, msg: Odometry, robot: str):
        """
        Append the robot’s current pose to its path list.
        We only record one point per callback to avoid flooding.
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        pt = Point()
        pt.x = x
        pt.y = y
        pt.z = 0.0
        self.paths[robot].append(pt)

    def cell_to_center(self, cell):
        """
        Convert integer cell (i,j) → world coordinate (x,y) at cell center.
        """
        i, j = cell
        x = self.origin_x + (i + 0.5) * self.CELL_SIZE
        y = self.origin_y + (j + 0.5) * self.CELL_SIZE
        return x, y

    def publish_markers(self):
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        for idx, robot in enumerate(self.robot_list):
            col = self.colors.get(robot, (1.0, 1.0, 1.0))
            r, g, b = col

            # 1) One Marker for ALL visited‐cell cubes (as CUBE_LIST)
            m_cells = Marker()
            m_cells.header.frame_id = 'odom'
            m_cells.header.stamp = now
            m_cells.ns = f'{robot}_visited'
            m_cells.id = idx * 2          # unique ID per robot
            m_cells.type = Marker.CUBE_LIST
            m_cells.action = Marker.ADD
            m_cells.pose.orientation.w = 1.0
            m_cells.scale.x = self.CELL_SIZE * 0.9  # slightly smaller than cell
            m_cells.scale.y = self.CELL_SIZE * 0.9
            m_cells.scale.z = 0.01               # flat on floor
            m_cells.color.r = r
            m_cells.color.g = g
            m_cells.color.b = b
            m_cells.color.a = 0.6

            # add one point per visited cell
            for (i, j) in self.visited[robot]:
                x, y = self.cell_to_center((i, j))
                pt = Point()
                pt.x = x
                pt.y = y
                pt.z = 0.005   # half‐thickness
                m_cells.points.append(pt)

            ma.markers.append(m_cells)

            # 2) One Marker for robot’s path (LINE_STRIP)
            m_path = Marker()
            m_path.header.frame_id = 'odom'
            m_path.header.stamp = now
            m_path.ns = f'{robot}_path'
            m_path.id = idx * 2 + 1
            m_path.type = Marker.LINE_STRIP
            m_path.action = Marker.ADD
            m_path.pose.orientation.w = 1.0
            m_path.scale.x = 0.05   # line thickness
            m_path.color.r = r
            m_path.color.g = g
            m_path.color.b = b
            m_path.color.a = 1.0

            # take the last 200 points to avoid huge memory
            pts = self.paths[robot][-200:]
            m_path.points = pts

            ma.markers.append(m_path)

        # Publish the entire MarkerArray
        self.marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = CoverageVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

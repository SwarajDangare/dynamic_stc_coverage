#!/usr/bin/env python3
import math
from threading import Lock

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from visualization_msgs.msg import Marker, MarkerArray

class CoverageVisualizer(Node):
    def __init__(self):
        super().__init__('coverage_visualizer')
        self.GRID_SIZE = 10
        self.CELL_SIZE = 1.0
        self.origin_x  = -(self.GRID_SIZE * self.CELL_SIZE) / 2.0
        self.origin_y  = -(self.GRID_SIZE * self.CELL_SIZE) / 2.0

        self.visited     = set()
        self.obstacles   = set()
        self.dynamic_obs = set()
        self.lock        = Lock()

        # Subscriptions to the three Int32MultiArray topics
        self.create_subscription(
            Int32MultiArray, '/stc/visited', self.vis_cb, 10
        )
        self.create_subscription(
            Int32MultiArray, '/stc/obstacles', self.obs_cb, 10
        )
        self.create_subscription(
            Int32MultiArray, '/stc/dynamic_obstacles', self.dyn_cb, 10
        )

        # Publisher for the MarkerArray
        self.marker_pub = self.create_publisher(MarkerArray, '/coverage_map', 10)

        # Publish once a second
        self.create_timer(1.0, self.publish_markers)

    def vis_cb(self, msg: Int32MultiArray):
        with self.lock:
            self.visited = {
                (msg.data[i], msg.data[i+1])
                for i in range(0, len(msg.data), 2)
            }

    def obs_cb(self, msg: Int32MultiArray):
        with self.lock:
            self.obstacles = {
                (msg.data[i], msg.data[i+1])
                for i in range(0, len(msg.data), 2)
            }

    def dyn_cb(self, msg: Int32MultiArray):
        with self.lock:
            self.dynamic_obs = {
                (msg.data[i], msg.data[i+1])
                for i in range(0, len(msg.data), 2)
            }

    def cell_to_world(self, cell):
        i, j = cell
        x = self.origin_x + (i + 0.5) * self.CELL_SIZE
        y = self.origin_y + (j + 0.5) * self.CELL_SIZE
        return x, y

    def publish_markers(self):
        ma = MarkerArray()
        mid = 0

        with self.lock:
            vis = set(self.visited)
            obs = set(self.obstacles)
            dyn = set(self.dynamic_obs)

        for i in range(self.GRID_SIZE):
            for j in range(self.GRID_SIZE):
                cell = (i, j)
                wx, wy = self.cell_to_world(cell)

                m = Marker()
                m.header.frame_id = 'odom'
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = 'coverage'
                m.id = mid
                mid += 1

                m.type   = Marker.CUBE
                m.action = Marker.ADD
                m.pose.position.x = wx
                m.pose.position.y = wy
                m.pose.position.z = 0.01
                m.pose.orientation.w = 1.0

                m.scale.x = self.CELL_SIZE
                m.scale.y = self.CELL_SIZE
                m.scale.z = 0.02

                # color as floats
                if cell in obs:
                    m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0; m.color.a = 0.6
                elif cell in dyn:
                    m.color.r = 0.0; m.color.g = 0.0; m.color.b = 1.0; m.color.a = 0.6
                elif cell in vis:
                    m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 0.6
                else:
                    m.color.r = 0.5; m.color.g = 0.5; m.color.b = 0.5; m.color.a = 0.2

                ma.markers.append(m)

        self.marker_pub.publish(ma)


def main():
    rclpy.init()
    node = CoverageVisualizer()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

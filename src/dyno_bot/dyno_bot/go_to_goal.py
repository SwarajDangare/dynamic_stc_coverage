#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal')
        # parameters: goal coordinates
        self.declare_parameter('goal_x', 3.0)
        self.declare_parameter('goal_y', 0.0)
        self.goal_x = self.get_parameter('goal_x').value
        self.goal_y = self.get_parameter('goal_y').value

        # state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.front_dist = float('inf')

        # publishers & subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        # timer
        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_cb(self, msg):
        # extract robot pose
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # compute yaw
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.yaw = math.atan2(siny, cosy)

    def scan_cb(self, msg):
        # take the minimum range in front ±10°
        ranges = msg.ranges
        mid = len(ranges)//2
        window = ranges[mid-10:mid+10]
        self.front_dist = min(window)

    def control_loop(self):
        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        dist = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        angle_diff = self._angle_diff(angle_to_goal, self.yaw)

        cmd = Twist()

        # if close enough, stop
        if dist < 0.1:
            self.get_logger().info('Reached goal')
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            return

        # if obstacle too close in front, rotate in place
        if self.front_dist < 0.5:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # turn away
        else:
            # first rotate toward goal, then drive forward
            if abs(angle_diff) > 0.1:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.5 * math.copysign(1.0, angle_diff)
            else:
                cmd.linear.x = 0.2
                cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    def _angle_diff(self, target, current):
        a = target - current
        return (a + math.pi) % (2*math.pi) - math.pi

def main(args=None):
    rclpy.init(args=args)
    node = GoToGoal()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

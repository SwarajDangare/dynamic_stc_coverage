#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray , Int32
from collections import defaultdict
import time
from gazebo_msgs.msg import ModelStates

class CoverageAggregator(Node):
    def __init__(self):
        super().__init__('coverage_aggregator')

        self.robot_names = ['tb1', 'tb2', 'tb3', 'tb4', 'tb5', 'tb6']
        self.visited_cells = set()
        self.static_obstacle = 30
        self.dynamic_obstacle = 10
        self.total_cells = 60 * 60  # adjust to your grid size
        self.total_steps = 0
        self.replans = 0
        self.revisits = 0

        self.create_subscription(ModelStates, '/model_states', self.models_cb, 10)
        self.subs = []
        for name in self.robot_names:
            self.subs.append(self.create_subscription(Int32MultiArray, f'/{name}/stc/visited', self.visited_cb, 10))
            self.subs.append(self.create_subscription(Int32, f'/{name}/stc/revisited', self.revisited_cb, 10))
            self.subs.append(self.create_subscription(Int32, f'/{name}/stc/steps', self.steps_cb, 10))


        # Print stats every 5 seconds
        self.create_timer(5.0, self.print_stats)

    def visited_cb(self, msg: Int32MultiArray):
        cells = list(zip(msg.data[::2], msg.data[1::2]))
        self.visited_cells.update(cells)

    def revisited_cb(self, msg: Int32):
        self.revisits = msg.data

    def steps_cb(self, msg: Int32):
        self.total_steps = msg.data

    def models_cb(self, msg: ModelStates):
        """
        Every time /model_states updates, compute which integer cells are statically
        occupied (red boxes) or dynamically occupied (blue boxes or other robots).
        """
        static_cells = set()
        dynamic_cells = set()

        # Everything that’s either a red_box or a blue_box or another robot counts as an obstacle
        self.obstacles = static_cells.union(dynamic_cells)
        self.dynamic_obs = dynamic_cells
        self.static_obs = static_cells

    def print_stats(self):
        visited = len(self.visited_cells)
        reachable_cells = self.total_cells - self.static_obstacle
        coverage = (visited / reachable_cells) * 100 if reachable_cells > 0 else 0.0
        overlap = (self.revisits / self.total_steps) * 100 if self.total_steps > 0 else 0.0 
        self.get_logger().info("-------- Global Coverage Stats --------")
        self.get_logger().info(f"Total cells in Grid: {self.total_cells}")
        self.get_logger().info(f"No. of Static obstacles: {self.static_obstacle}, No. of Dynamic obstacles: {self.dynamic_obstacle}")
        self.get_logger().info(f"Cells Visited: {visited} / {reachable_cells}")
        self.get_logger().info(f"Coverage: {coverage:.2f}%")
        self.get_logger().info(f"Overlap Rate: {overlap:.2f}%")
        self.get_logger().info("---------------------------------------")
        self.get_logger().info("")

def main():     
    rclpy.init()
    node = CoverageAggregator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

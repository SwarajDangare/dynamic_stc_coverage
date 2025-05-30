#!/usr/bin/env python3
import random
import math
import subprocess
import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from threading import Lock

class RandomMover(Node):
    def __init__(self):
        super().__init__('random_mover')
        # ---- GRID CONFIGURATION ----
        self.grid_size   = 60         # 60×60 cells
        self.cell_size   = 0.5        # each cell is 0.5 m
        self.half_extent = self.grid_size * self.cell_size / 2.0   # 15.0 m
        self.cell_half   = self.cell_size / 2.0                    # 0.25 m

        # ---- Z HEIGHT FOR BOXES ----
        self.Z = 0.35   # match your box visuals

        # ---- DYNAMIC BOXES ----
        self.models = [f"blue_box_{i}" for i in range(1, 11)]

        # ---- STATIC OBSTACLES (your red boxes) ----
        static_centers = [
            (5.25, -11.25), (-14.25,  8.75), (-6.25,  -7.25),
            (-7.75, -10.75), ( 8.75, -11.75), ( 6.75,   8.75),
            (13.75,  2.25), (-12.25,  3.75), (-1.25, -13.75),
            (-14.25,-12.25)
        ]
        self.static_cells = {
            self.world_to_cell(x, y)
            for x, y in static_centers
        }

        # ---- STATE ----
        self.positions = {}   # world‐coords for each blue_box_N
        self.bot_cell  = None # turtlebot3 location in cells
        self.lock      = Lock()

        # ---- MOVEMENT STEPS ----
        # one cell step at a time (0.5 m)
        self.moves = [
            ( self.cell_size,  0.0),
            (-self.cell_size,  0.0),
            ( 0.0,  self.cell_size),
            ( 0.0, -self.cell_size),
        ]

        # ---- ROS 2 SETUP ----
        self.create_subscription(
            ModelStates,
            '/model_states',
            self.models_cb,
            10
        )
        # every 2 seconds attempt to move each box
        self.create_timer(2.0, self.move_boxes)

    def world_to_cell(self, wx: float, wy: float):
        """Convert world coords to a (i,j) cell index."""
        i = int(math.floor((wx + self.half_extent) / self.cell_size))
        j = int(math.floor((wy + self.half_extent) / self.cell_size))
        i = max(0, min(self.grid_size - 1, i))
        j = max(0, min(self.grid_size - 1, j))
        return (i, j)

    def clamp(self, v: float):
        """Clamp world coord to the nearest cell‐center within the grid."""
        return max(
            min(v, self.half_extent - self.cell_half),
            -self.half_extent + self.cell_half
        )

    def models_cb(self, msg: ModelStates):
        """Track current positions of blue boxes and the turtlebot."""
        with self.lock:
            for name, pose in zip(msg.name, msg.pose):
                if name in self.models:
                    self.positions[name] = (
                        pose.position.x,
                        pose.position.y
                    )
                if name == 'turtlebot3_burger':
                    self.bot_cell = self.world_to_cell(
                        pose.position.x,
                        pose.position.y
                    )

    def move_model(self, name: str, x: float, y: float):
        """Send the Gazebo CLI command to teleport the model."""
        subprocess.run([
            'gz', 'model',
            '-m', name,
            '-x', f"{x:.2f}",
            '-y', f"{y:.2f}",
            '-z', f"{self.Z:.2f}"
        ], check=True)

    def move_boxes(self):
        """For each blue box, pick a random adjacent cell and move there."""
        with self.lock:
            bot_cell = self.bot_cell
            pos = dict(self.positions)

        for name in self.models:
            x, y = pos.get(name, (0.0, 0.0))
            # try up to 10 random directions
            for _ in range(10):
                dx, dy = random.choice(self.moves)
                nx = self.clamp(x + dx)
                ny = self.clamp(y + dy)
                cell = self.world_to_cell(nx, ny)
                if cell not in self.static_cells and cell != bot_cell:
                    x, y = nx, ny
                    break

            with self.lock:
                self.positions[name] = (x, y)
            self.move_model(name, x, y)
            # self.get_logger().info(
            #     f"Moved {name} → x={x:.2f}, y={y:.2f}"
            # )

def main():
    rclpy.init()
    node = RandomMover()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

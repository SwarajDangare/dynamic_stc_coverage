#!/usr/bin/env python3
import random, math, subprocess, time
from threading import Lock

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates

class RandomMover(Node):
    def __init__(self):
        super().__init__('random_mover')
        self.models      = ["blue_box_1", "blue_box_2"]
        self.Z           = 0.5
        self.moves       = [(1,0),(-1,0),(0,1),(0,-1)]
        # static boxes at four corners
        static_centers  = [(-2.5,-2.5),(2.5,-2.5),(-2.5,2.5),(2.5,2.5)]
        self.static_cells= {self.world_to_cell(x,y) for x,y in static_centers}

        # initial positions match world file
        self.positions   = {
            "blue_box_1": (0.0, -2.0),
            "blue_box_2": (0.0,  2.0),
        }
        self.bot_cell    = None
        self.lock        = Lock()

        self.create_subscription(
            ModelStates, '/model_states', self.models_cb, 10
        )
        self.create_timer(2.0, self.move_boxes)

    def world_to_cell(self, wx, wy):
        i = int(math.floor((wx + 5.0)/1.0))
        j = int(math.floor((wy + 5.0)/1.0))
        return (max(0,min(9,i)), max(0,min(9,j)))

    def clamp(self, v):
        return max(min(v,4.5), -4.5)

    def models_cb(self, msg):
        bot_cell = None
        for name, pose in zip(msg.name, msg.pose):
            if name in self.models:
                self.positions[name] = (pose.position.x, pose.position.y)
            if name == 'turtlebot3_burger':
                bot_cell = self.world_to_cell(
                    pose.position.x, pose.position.y
                )
        with self.lock:
            self.bot_cell = bot_cell

    def move_model(self, name, x, y):
        subprocess.run([
            'gz','model',
            '-m', name,
            '-x', str(x),
            '-y', str(y),
            '-z', str(self.Z)
        ], check=True)

    def move_boxes(self):
        with self.lock:
            bot = self.bot_cell
            pos = dict(self.positions)
        for name in self.models:
            x,y = pos.get(name, (0.0,0.0))
            for _ in range(10):
                dx,dy = random.choice(self.moves)
                nx,ny = self.clamp(x+dx), self.clamp(y+dy)
                cell = self.world_to_cell(nx, ny)
                if cell not in self.static_cells and cell != bot:
                    x,y = nx,ny
                    break
            with self.lock:
                self.positions[name] = (x,y)
            self.move_model(name, x, y)
            self.get_logger().info(f'Moved {name} → x={x:.1f}, y={y:.1f}')

def main(args=None):
    rclpy.init(args=args)
    node = RandomMover()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

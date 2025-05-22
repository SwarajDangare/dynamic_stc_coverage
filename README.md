# Dynamic Coverage Path Planning Using STC

This project implements an **online Spanning Tree Coverage (STC)** algorithm for a mobile robot navigating unknown environments with **static and dynamic obstacles**. Built using **ROS 2 Humble** and **Gazebo**, the system simulates a robot performing real-time obstacle-aware area coverage using LiDAR.

---

## Project Overview

Traditional coverage algorithms often assume static maps or pre-defined environments. This project extends the classical **STC algorithm** to work in **online mode**, where the robot has:
- **No prior knowledge of the map**
- Must **build coverage paths in real time**
- Must handle **dynamic obstacles** intelligently

Key design choices include:
- A **4D × 4D cell representation** where each major cell contains 4 subcells, enabling fine-grained movement and sensing.
- Use of **LiDAR sensor data** (360° scan) to detect obstacles and classify them as static or dynamic based on motion patterns.
- A **directional escape mechanism** where the robot selects movement orthogonal to the direction of an approaching dynamic obstacle.
- **Energy-aware sensor operation**, where the LiDAR is turned off in obstacle-free zones to conserve resources.

---

## Features

- **Online STC with LiDAR-based reactive planning**
- Obstacle classification: **static vs. dynamic**
- Efficient escape and re-entry to maintain full coverage
- No repetitive coverage or loops
- LiDAR-aware power saving
- Gazebo simulation of dynamic world environment

---

## Getting Started

### 1. Clone the Repository

```bash
git clone git@github.com:SwarajDangare/dynamic-stc-coverage.git
cd dynamic-stc-coverage
```

### 2. Build the Workspace

```bash
colcon build
source install/setup.bash
```

---

## Running the Simulation

Launch Gazebo simulation, spawn the robot, and start the STC node:

```bash
ros2 launch dyno_bot coverage_launch.py
```

If you have multiple terminals or want to launch components separately:

```bash
# Terminal 1 - Source the workspace
source install/setup.bash

# Terminal 2 - Launch Gazebo and robot
ros2 launch dyno_bot coverage_launch.py

# Terminal 3 - Manually run STC node (if needed)
ros2 run dyno_bot stc_planner.py
```

---

## Tech Stack

- **ROS 2 Humble** (rclpy, sensor_msgs, nav_msgs)
- **Gazebo** (simulation and LiDAR plugin)
- **Python 3** for all control and logic nodes
- **RViz2** for visualization (optional)

---

## 📄 License

This project is licensed under the MIT License. See [`LICENSE`](LICENSE) for details.

---

## 👤 Author

**Swaraj Dangare**  
Embedded Firmware & Robotics Engineer  
[LinkedIn](https://www.linkedin.com/in/swaraj-dangare) | [GitHub](https://github.com/SwarajDangare)

---

## 📚 Reference

- Y. Gabriely and E. Rimon, *Spanning Tree Coverage of Continuous Areas by a Mobile Robot*, IEEE ICRA 2001.
- Research and implementation as part of the final-year project at Manipal Institute of Technology (Jan–May 2025).
import numpy as np
import matplotlib.pyplot as plt
import time
import random
from collections import deque
import signal

# Configuration
GRID_SIZE = 15
NUM_OBSTACLES = 10
NUM_DYNAMIC_OBS = 5
VELOCITY_MODEL = "bot_faster"  # Options: "bot_faster", "bot_slower", "random"

DIRECTIONS = [(0, 1), (1, 0), (0, -1), (-1, 0)]
start_cell = (0, 0)

# State
visited_cells = set()
tree_edges = []
robot_pos = start_cell
step_count = 0
obstacles = set()
dynamic_obstacles = []
interrupted = False
obstacle_step = 0


def handle_interrupt(sig, frame):
    global interrupted
    print("\\n Interrupted by user.")
    interrupted = True

signal.signal(signal.SIGINT, handle_interrupt)

def generate_static_obstacles():
    obs = set()
    obs.add((0 ,4))
    obs.add((0 ,6)) # fixed some static obstacles
    obs.add((1 ,5))
    while len(obs) < NUM_OBSTACLES:
        x = random.randint(0, GRID_SIZE - 1)
        y = random.randint(0, GRID_SIZE - 1)
        if (x, y) != start_cell:
            obs.add((x, y))
    return obs

obstacles = generate_static_obstacles()
dynamic_obstacles = [(random.randint(0, GRID_SIZE - 1), random.randint(0, GRID_SIZE - 1)) for _ in range(NUM_DYNAMIC_OBS)]

def move_dynamic_obstacles(force=False):
    global dynamic_obstacles, obstacle_step
    obstacle_step += 1
    if not force:
        if VELOCITY_MODEL == "bot_faster" and obstacle_step % 2 != 0:
            return
        if VELOCITY_MODEL == "bot_slower" and obstacle_step % 4 == 0:
            return

    new_positions = []
    for x, y in dynamic_obstacles:
        random.shuffle(DIRECTIONS)
        for dx, dy in DIRECTIONS:
            nx, ny = x + dx, y + dy
            if (0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE and (nx, ny) not in obstacles):
                new_positions.append((nx, ny))
                break
        else:
            new_positions.append((x, y))
    dynamic_obstacles = new_positions


def draw_grid(current_cell):
    plt.clf()
    ax = plt.gca()
    plt.title("Stable Snake Coverage (with dynamic wait)")
    plt.axis('off')
    for i in range(GRID_SIZE):
        for j in range(GRID_SIZE):
            ax.add_patch(plt.Rectangle((j * 2, i * 2), 2, 2, edgecolor='gray', facecolor='none'))
    for a, b in tree_edges:
        x1, y1 = a[0]*2 + 1, a[1]*2 + 1
        x2, y2 = b[0]*2 + 1, b[1]*2 + 1
        plt.plot([x1, x2], [y1, y2], 'g-', linewidth=1.5)
    for (x, y) in visited_cells:
        ax.add_patch(plt.Rectangle((x*2, y*2), 2, 2, color='lightgreen'))
    for (ox, oy) in obstacles:
        ax.add_patch(plt.Rectangle((ox*2, oy*2), 2, 2, color='red', alpha=0.6))
    for (dx, dy) in dynamic_obstacles:
        ax.add_patch(plt.Rectangle((dx*2, dy*2), 2, 2, color='blue', alpha=0.4))
    x, y = current_cell
    ax.add_patch(plt.Circle((x*2 + 1, y*2 + 1), 0.4, color='black'))
    plt.xlim(0, GRID_SIZE * 2)
    plt.ylim(0, GRID_SIZE * 2)
    ax.set_aspect('equal')
    plt.pause(0.0001)

def is_safe(cell):
    for dx, dy in DIRECTIONS + [(0, 0)]:
        cx, cy = cell[0] + dx, cell[1] + dy
        if (cx, cy) in dynamic_obstacles:
            return False
    return True

def visit_cell(cell):
    global robot_pos, step_count
    robot_pos = cell
    visited_cells.add(cell)
    draw_grid(cell)
    step_count += 1

def bfs_path(start, goal):
    queue = deque([(start, [start])])
    visited = set([start])
    while queue:
        current, path = queue.popleft()
        if current == goal:
            return path
        for dx, dy in DIRECTIONS:
            nx, ny = current[0]+dx, current[1]+dy
            neighbor = (nx, ny)
            if (0 <= nx < GRID_SIZE and 0 <= ny < GRID_SIZE and
                neighbor not in visited and
                neighbor not in obstacles and
                neighbor not in dynamic_obstacles):
                visited.add(neighbor)
                queue.append((neighbor, path + [neighbor]))
    return []

def move_robot_to(path):
    global robot_pos
    for cell in path[1:]:
        if interrupted:
            return False
        retries = 0
        while not is_safe(cell) and retries < 20:
            move_dynamic_obstacles(force=True)
            draw_grid(robot_pos)
            time.sleep(0.001)
            retries += 1
        if not is_safe(cell):
            print(f"❌ Still blocked after wait. Skipping {cell}.")
            return False
        robot_pos = cell
        draw_grid(cell)
        move_dynamic_obstacles()
    return True

def generate_snake_order():
    order = []
    for y in range(GRID_SIZE):
        row = list(range(GRID_SIZE))
        if y % 2 == 1:
            row.reverse()
        for x in row:
            order.append((x, y))
    return order

def snake_stc():
    snake_order = generate_snake_order()
    for cell in snake_order:
        if interrupted:
            break
        if cell in visited_cells or cell in obstacles:
            continue
        path = bfs_path(robot_pos, cell)
        if path:
            success = move_robot_to(path)
            if success:
                tree_edges.append((robot_pos, cell))
                visit_cell(cell)
    
    remaining = [
        (x, y)
        for x in range(GRID_SIZE)
        for y in range(GRID_SIZE)
        if (x, y) not in visited_cells and (x, y) not in obstacles
    ]
    if remaining:
        print(f"⚠️ {len(remaining)} cells left unvisited:")
        print(remaining)
        for cell in remaining:
            if interrupted:
                break
            path = bfs_path(robot_pos, cell)
            retries = 0
            while(not path and retries < 5):
                move_dynamic_obstacles(force=True)
                path = bfs_path(robot_pos, cell)
                retries += 1
            if path:   
                success = move_robot_to(path)
                if success:
                    tree_edges.append((robot_pos, cell))
                    visit_cell(cell)
            else:
                print(f"❌ No path to {cell} after retries.")
                continue
        print(f"⚠️ {len(remaining)} cells left unvisited after retries:")
        print(remaining)
        print("🎉 All reachable cells visited!")

if __name__ == "__main__":
    try:
        plt.ion()
        snake_stc()
        plt.ioff()
        plt.show()
    except KeyboardInterrupt:
        print("\\nSimulation interrupted.")
        plt.ioff()
        plt.close()
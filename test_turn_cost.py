#!/usr/bin/env python3
"""Test script to verify turn_cost logic is working correctly"""

import yaml
import numpy as np
from micromouse import Map
from pathPlanning import AStar

# Load config
with open('config.yaml', 'r') as f:
    config = yaml.safe_load(f)

# Create a simple test maze with a choice between:
# 1. A straight path (no turns)
# 2. A zigzag path (many turns) of similar length
maze = np.zeros((10, 10))
maze[0:10, 0] = 1   # Left wall
maze[0:10, 9] = 1   # Right wall
maze[0, :] = 1      # Top wall
maze[9, :] = 1      # Bottom wall

# Create two paths from start to goal
# Path 1: Straight path (row 5, columns 2-7)
# Path 2: Zigzag path (requires turns)
maze[3, 5] = 1  # Block the straight path to force consideration of alternatives
maze[4, 5] = 1
maze[5, 5] = 1
maze[6, 5] = 1

# Set start and goal
start_pos = (5, 2)
goal_pos = (5, 7)
maze[start_pos] = 2  # Start
maze[goal_pos] = 4   # Goal

# Test 1: With turn cost enabled
print("=" * 60)
print("Test 1: Turn cost ENABLED")
print("=" * 60)
config['astar']['turn_cost_enabled'] = True
test_config = {
    'astar': config['astar'],
    'wall_cost': config.get('wall_cost', {})
}

map1 = Map(maze)
planner1 = AStar(map1, test_config)
print(f"Turn cost enabled: {planner1.turn_cost_enabled}")
print(f"Turn cost weight: {planner1.turn_cost_weight}")
print(f"Turn cost threshold: {planner1.turn_cost_threshold}°")

start_state = map1.map[start_pos[0]][start_pos[1]]
goal_state = map1.map[goal_pos[0]][goal_pos[1]]
path_x1, path_y1 = planner1.plan_path(start_state, goal_state)

if path_x1 and path_y1:
    path1 = list(zip(path_x1, path_y1))
    print(f"\nPath found: {path1}")
    distance = planner1.calculate_path_distance(path_x1, path_y1)
    total_turn_1, num_turns_1 = planner1.calculate_path_turns(path_x1, path_y1)
    print(f"Path length: {distance:.2f}")
    print(f"Total turn angle: {total_turn_1:.1f}°")
    print(f"Number of significant turns: {num_turns_1}")
else:
    print("No path found!")

# Test 2: With turn cost disabled
print("\n" + "=" * 60)
print("Test 2: Turn cost DISABLED")
print("=" * 60)
config['astar']['turn_cost_enabled'] = False
test_config2 = {
    'astar': config['astar'],
    'wall_cost': config.get('wall_cost', {})
}

map2 = Map(maze)
planner2 = AStar(map2, test_config2)
print(f"Turn cost enabled: {planner2.turn_cost_enabled}")

start_state2 = map2.map[start_pos[0]][start_pos[1]]
goal_state2 = map2.map[goal_pos[0]][goal_pos[1]]
path_x2, path_y2 = planner2.plan_path(start_state2, goal_state2)

if path_x2 and path_y2:
    path2 = list(zip(path_x2, path_y2))
    print(f"\nPath found: {path2}")
    distance = planner2.calculate_path_distance(path_x2, path_y2)
    total_turn_2, num_turns_2 = planner2.calculate_path_turns(path_x2, path_y2)
    print(f"Path length: {distance:.2f}")
    print(f"Total turn angle: {total_turn_2:.1f}°")
    print(f"Number of significant turns: {num_turns_2}")
else:
    print("No path found!")

print("\n" + "=" * 60)
print("COMPARISON")
print("=" * 60)
if path_x1 and path_x2:
    if path1 == path2:
        print("⚠ WARNING: Paths are IDENTICAL (turn cost may not be working!)")
    else:
        print("✓ SUCCESS: Paths are DIFFERENT (turn cost is working!)")
        print(f"\nWith turn cost: {num_turns_1} turns")
        print(f"Without turn cost: {num_turns_2} turns")

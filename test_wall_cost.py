#!/usr/bin/env python3
"""Test script to verify wall_cost logic is working correctly"""

import yaml
import numpy as np
from micromouse import Map
from pathPlanning import AStar

# Load config
with open('config.yaml', 'r') as f:
    config = yaml.safe_load(f)

# Create a simple test maze with a corridor and an open space
# The path should prefer the open space if wall_cost is working
maze = np.zeros((12, 12))
maze[0:12, 0] = 1   # Left wall
maze[0:12, 11] = 1  # Right wall
maze[0, :] = 1      # Top wall
maze[11, :] = 1     # Bottom wall

# Create a narrow corridor (1 cell wide)
maze[3:9, 6] = 1    # Wall creating narrow corridor on left side
maze[4, 4] = 1      # Extra walls to narrow the corridor
maze[5, 4] = 1
maze[6, 4] = 1
maze[7, 4] = 1

# Set start and goal - path can go through narrow corridor or wider space
start_pos = (6, 2)
goal_pos = (6, 9)
maze[start_pos] = 2  # Start
maze[goal_pos] = 4   # Goal

print("Maze layout (1=wall, 0=free, 2=start, 4=goal):")
print(maze.astype(int))

# Test 1: With wall cost enabled
print("\n" + "=" * 60)
print("Test 1: Wall cost ENABLED")
print("=" * 60)
config['wall_cost']['enabled'] = True
test_config = {
    'astar': config.get('astar', {}),
    'wall_cost': config['wall_cost']
}

map1 = Map(maze)
planner1 = AStar(map1, test_config)
print(f"Wall cost enabled: {planner1.wall_cost_enabled}")
print(f"Wall cost weight: {planner1.wc_weight}")
print(f"Wall cost threshold: {planner1.wc_threshold}")
print(f"Wall distance map computed: {hasattr(map1, 'wall_distance')}")

if hasattr(map1, 'wall_distance'):
    print(f"\nWall distance at start {start_pos}: {map1.wall_distance[start_pos]:.2f}")
    print(f"Wall distance at goal {goal_pos}: {map1.wall_distance[goal_pos]:.2f}")
    print(f"Wall distance at (6, 5) [narrow]: {map1.wall_distance[6, 5]:.2f}")
    print(f"Wall distance at (6, 8) [wider]: {map1.wall_distance[6, 8]:.2f}")

start_state = map1.map[start_pos[0]][start_pos[1]]
goal_state = map1.map[goal_pos[0]][goal_pos[1]]
path_x1, path_y1 = planner1.plan_path(start_state, goal_state)

if path_x1 and path_y1:
    path1 = list(zip(path_x1, path_y1))
    print(f"\nPath found with {len(path1)} waypoints:")
    
    # Calculate average distance from walls along path
    if hasattr(map1, 'wall_distance'):
        avg_wall_dist = sum(map1.wall_distance[x, y] for x, y in path1) / len(path1)
        print(f"Average wall distance along path: {avg_wall_dist:.2f}")
    
    distance = planner1.calculate_path_distance(path_x1, path_y1)
    print(f"Path length: {distance:.2f}")
else:
    print("No path found!")

# Test 2: With wall cost disabled
print("\n" + "=" * 60)
print("Test 2: Wall cost DISABLED")
print("=" * 60)
config['wall_cost']['enabled'] = False
test_config2 = {
    'astar': config.get('astar', {}),
    'wall_cost': config['wall_cost']
}

map2 = Map(maze)
planner2 = AStar(map2, test_config2)
print(f"Wall cost enabled: {planner2.wall_cost_enabled}")
print(f"Wall distance map computed: {hasattr(map2, 'wall_distance')}")

start_state2 = map2.map[start_pos[0]][start_pos[1]]
goal_state2 = map2.map[goal_pos[0]][goal_pos[1]]
path_x2, path_y2 = planner2.plan_path(start_state2, goal_state2)

if path_x2 and path_y2:
    path2 = list(zip(path_x2, path_y2))
    print(f"\nPath found with {len(path2)} waypoints")
    distance = planner2.calculate_path_distance(path_x2, path_y2)
    print(f"Path length: {distance:.2f}")
else:
    print("No path found!")

print("\n" + "=" * 60)
print("COMPARISON")
print("=" * 60)
if path_x1 and path_x2:
    if path1 == path2:
        print("⚠ WARNING: Paths are IDENTICAL (wall cost may not be working!)")
        print(f"\nBoth paths: {path1[:5]}...{path1[-3:]}")
    else:
        print("✓ SUCCESS: Paths are DIFFERENT (wall cost is working!)")
        print(f"\nWith wall cost: {len(path1)} waypoints, length {planner1.calculate_path_distance(path_x1, path_y1):.2f}")
        print(f"Without wall cost: {len(path2)} waypoints, length {planner2.calculate_path_distance(path_x2, path_y2):.2f}")
        
        # Check which path stays farther from walls
        if hasattr(map1, 'wall_distance'):
            avg_dist1 = sum(map1.wall_distance[x, y] for x, y in path1) / len(path1)
            avg_dist2 = sum(map2.wall_distance[x, y] for x, y in path2) / len(path2)
            print(f"\nAverage wall distance with wall cost: {avg_dist1:.2f}")
            print(f"Average wall distance without wall cost: {avg_dist2:.2f}")
            if avg_dist1 > avg_dist2:
                print("✓ Path with wall cost stays farther from walls (as expected)")
            else:
                print("⚠ Path with wall cost is NOT farther from walls")

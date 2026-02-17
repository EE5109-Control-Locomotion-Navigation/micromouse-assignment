#!/usr/bin/env python3
"""Test wall_cost with the actual maze from config.yaml"""

import yaml
import numpy as np
from micromouse import read_maze_from_csv, read_maze_from_txt, Map
from pathPlanning import AStar

# Load config
with open('config.yaml', 'r') as f:
    config = yaml.safe_load(f)

maze_file = config['maze_file']
print(f"Loading maze: {maze_file}")

# Load the actual maze based on file extension
if maze_file.endswith('.csv'):
    maze_array = read_maze_from_csv(maze_file)
else:
    maze_array = read_maze_from_txt(maze_file)
print(f"Maze size: {len(maze_array)}x{len(maze_array[0])}")

# Test 1: With wall cost enabled
print("\n" + "=" * 60)
print("Test 1: Wall cost ENABLED")
print("=" * 60)
config['wall_cost']['enabled'] = True
test_config1 = {
    'astar': config['astar'],
    'wall_cost': config['wall_cost']
}

map1 = Map(maze_array)
planner1 = AStar(map1, test_config1)
print(f"Wall cost enabled: {planner1.wall_cost_enabled}")
print(f"Wall cost weight: {planner1.wc_weight}")
print(f"Wall cost decay: {planner1.wc_decay}")
print(f"Wall cost threshold: {planner1.wc_threshold}")

start_state = map1.map[map1.start[0]][map1.start[1]]
goal_state = map1.map[map1.goal[0]][map1.goal[1]]
path_x1, path_y1 = planner1.plan_path(start_state, goal_state)

if path_x1 and path_y1:
    path1 = list(zip(path_x1, path_y1))
    print(f"\nPath length (cells): {len(path1)}")
    distance = planner1.calculate_path_distance(path_x1, path_y1)
    print(f"Path distance: {distance:.2f}")
    total_turn, num_turns = planner1.calculate_path_turns(path_x1, path_y1)
    print(f"Total turn angle: {total_turn:.1f}°")
    print(f"Number of significant turns: {num_turns}")
    
    # Calculate average wall distance along path
    wall_distances = []
    for x, y in path1:
        wd = map1.wall_distance[x, y]
        wall_distances.append(wd)
    avg_wall_dist = np.mean(wall_distances)
    min_wall_dist = np.min(wall_distances)
    print(f"Average wall distance: {avg_wall_dist:.2f}")
    print(f"Minimum wall distance: {min_wall_dist:.2f}")
else:
    print("No path found!")

# Test 2: With wall cost disabled
print("\n" + "=" * 60)
print("Test 2: Wall cost DISABLED")
print("=" * 60)
config['wall_cost']['enabled'] = False
test_config2 = {
    'astar': config['astar'],
    'wall_cost': config['wall_cost']
}

map2 = Map(maze_array)
planner2 = AStar(map2, test_config2)
print(f"Wall cost enabled: {planner2.wall_cost_enabled}")

start_state2 = map2.map[map2.start[0]][map2.start[1]]
goal_state2 = map2.map[map2.goal[0]][map2.goal[1]]
path_x2, path_y2 = planner2.plan_path(start_state2, goal_state2)

if path_x2 and path_y2:
    path2 = list(zip(path_x2, path_y2))
    print(f"\nPath length (cells): {len(path2)}")
    distance = planner2.calculate_path_distance(path_x2, path_y2)
    print(f"Path distance: {distance:.2f}")
    total_turn, num_turns = planner2.calculate_path_turns(path_x2, path_y2)
    print(f"Total turn angle: {total_turn:.1f}°")
    print(f"Number of significant turns: {num_turns}")
    
    # Calculate average wall distance along path (use map2's wall_distance if computed)
    if hasattr(map2, 'wall_distance') and map2.wall_distance is not None:
        wall_distances = []
        for x, y in path2:
            wd = map2.wall_distance[x, y]
            wall_distances.append(wd)
        avg_wall_dist = np.mean(wall_distances)
        min_wall_dist = np.min(wall_distances)
        print(f"Average wall distance: {avg_wall_dist:.2f}")
        print(f"Minimum wall distance: {min_wall_dist:.2f}")
else:
    print("No path found!")

print("\n" + "=" * 60)
print("COMPARISON")
print("=" * 60)
if path_x1 and path_x2:
    if path1 == path2:
        print("⚠ WARNING: Paths are IDENTICAL (wall cost may not be working!)")
    else:
        print("✓ SUCCESS: Paths are DIFFERENT (wall cost is working!)")
        print(f"\nWith wall cost:")
        print(f"  - Path length: {len(path1)} cells")
        print(f"  - Avg wall distance: {np.mean([map1.wall_distance[x, y] for x, y in path1]):.2f}")
        print(f"\nWithout wall cost:")
        print(f"  - Path length: {len(path2)} cells")
        if hasattr(map2, 'wall_distance') and map2.wall_distance is not None:
            print(f"  - Avg wall distance: {np.mean([map2.wall_distance[x, y] for x, y in path2]):.2f}")

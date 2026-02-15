# Micromouse Path Planning and Tracking

This repository implements path planning and path tracking algorithms for a micromouse robot navigation system.

## Table of Contents

- [Overview](#overview)
- [A* Pathfinding Algorithm](#a-pathfinding-algorithm)
- [Pure Pursuit Controller](#pure-pursuit-controller)
- [Configuration](#configuration)
- [Usage](#usage)

---

## Overview

The system consists of two main components:

1. **Path Planning (A*)**: Finds an optimal path from start to goal through a maze
2. **Path Tracking (Pure Pursuit)**: Controls the robot to follow the planned path

---

## A* Pathfinding Algorithm

### What is A*?

A* (pronounced "A-star") is an informed search algorithm that finds the shortest path between two points in a graph or grid. It's widely used in robotics, video games, and navigation systems because it's both optimal and efficient.

### How A* Works

A* combines two key pieces of information:

- **g(n)**: The actual cost from the start node to the current node
- **h(n)**: The estimated cost (heuristic) from the current node to the goal
- **f(n) = g(n) + h(n)**: The total estimated cost of the path through this node

The algorithm maintains two lists:

1. **Open List**: Nodes to be evaluated
2. **Closed List**: Nodes already evaluated

### Algorithm Steps

1. Add the start node to the open list
2. While the open list is not empty:
   - Select the node with the lowest f(n) from the open list
   - If this node is the goal, reconstruct and return the path
   - Move the current node to the closed list
   - For each neighbor of the current node:
     - Skip if it's a wall or in the closed list
     - Calculate tentative g score
     - If the neighbor is not in the open list or this path is better:
       - Update the neighbor's parent, g, and f values
       - Add to open list if not already present

### Implementation Features

#### Heuristic Function

```python
def heuristic(self, a, b):
    return self.heuristic_weight * (abs(a.x - b.x) + abs(a.y - b.y))
```

The implementation uses **Manhattan distance** (taxicab geometry), which is ideal for grid-based movement where diagonal moves aren't allowed or are more expensive. The heuristic is admissible (never overestimates) and consistent, ensuring optimal paths.

#### Wall Proximity Cost

An advanced feature that adds extra cost to paths near walls:

```python
def wall_proximity_cost(self, state):
    if not self.wall_cost_enabled:
        return 0.0
    d = float(self.map.wall_distance[state.x, state.y])
    if d >= self.wc_threshold:
        return 0.0
    if self.wc_decay == 'exponential':
        return self.wc_weight * math.exp(-self.wc_decay_rate * d)
```

This encourages the robot to stay away from walls, which is useful for:
- Avoiding sensor noise near walls
- Reducing collision risk
- Creating smoother paths for the controller

**Decay modes**:
- **Exponential**: Cost decreases exponentially with distance
- **Inverse**: Cost is inversely proportional to distance
- **Linear**: Cost decreases linearly to zero at threshold

#### Real-time Visualization

When debug mode is enabled, you can see:
- **Open Set** (yellow): Nodes being considered
- **Closed Set** (cyan): Nodes already evaluated
- **Current Path** (magenta): Best path found so far
- **Final Path** (marked with asterisks)

### Complexity

- **Time Complexity**: O(b^d) in worst case, but typically much better with a good heuristic
- **Space Complexity**: O(b^d) where b is the branching factor and d is the depth

---

## Pure Pursuit Controller

### What is Pure Pursuit?

Pure Pursuit is a path tracking algorithm that calculates the curvature needed to reach a "lookahead point" on the path. It's geometrically intuitive and widely used in mobile robotics.

### How Pure Pursuit Works

The controller uses a **lookahead distance** to select a target point ahead on the path, then calculates the steering angle needed to reach that point.

### Core Concept

Imagine you're driving a car and looking at a point on the road ahead (not directly in front, but some distance forward). You steer to aim toward that point. As you move, you continuously update which point you're looking at. This is the essence of Pure Pursuit.

### Algorithm Steps

1. **Find Lookahead Point**: Search along the path for the first point that is at least `lookahead_distance` away from the robot

2. **Transform to Local Coordinates**: Convert the target point from global coordinates to the robot's local reference frame

3. **Calculate Curvature**: Using the lookahead point's position, calculate the path curvature:
   ```
   curvature = 2 * y_local / lookahead_distance²
   ```

4. **Compute Steering Angle**: Convert curvature to steering angle:
   ```
   steering_angle = atan(curvature * steering_gain)
   ```

5. **Adjust Speed**: Reduce speed when making sharp turns

### Mathematical Foundation

The Pure Pursuit algorithm is based on finding the arc of a circle that:
- Passes through the robot's current position
- Is tangent to the robot's current heading
- Passes through the lookahead point

The curvature formula `2y/L²` comes from circle geometry, where:
- `y` is the lateral offset of the lookahead point in robot coordinates
- `L` is the lookahead distance

### Implementation Features

#### Lookahead Point Selection

```python
def find_lookahead_point(self, current_pos):
    for i in range(self.current_index, len(self.path)):
        dx = self.path[i][0] - current_pos[0]
        dy = self.path[i][1] - current_pos[1]
        dist = math.sqrt(dx**2 + dy**2)
        if dist >= self.lookahead_dist:
            self.current_index = i
            return self.path[i]
    return self.path[-1]
```

The algorithm searches forward from the last used point for efficiency and returns the goal if the path end is reached.

#### Coordinate Transformation

```python
# Global to local transformation
target_local_x = dx * math.cos(-current_heading) - dy * math.sin(-current_heading)
target_local_y = dx * math.sin(-current_heading) + dy * math.cos(-current_heading)
```

This rotation matrix transforms the target point into the robot's coordinate frame, where:
- x-axis points forward
- y-axis points left
- The heading angle determines the rotation

#### Adaptive Speed Control

```python
speed = self.max_speed if abs(steering_angle) < self.slow_steering_threshold else self.min_speed
```

The robot slows down for sharp turns (typically > 45°) to maintain stability and tracking accuracy.

### Key Parameters

- **lookahead_distance**: Larger values create smoother paths but may cut corners; smaller values track more precisely but may oscillate
- **steering_gain**: Amplifies the steering response; typically set to 1.0
- **max_speed / min_speed**: Define the speed range based on path curvature
- **slow_steering_threshold**: Angle threshold for speed reduction (default: π/4 or 45°)

### Real-time Visualization

When debug mode is enabled, you can see:
- **Planned Path** (yellow): The reference trajectory
- **Lookahead Point** (magenta circle): Current target
- **Target Vector** (magenta line): Direction to lookahead point
- **Heading** (green line): Robot's current orientation
- **Robot Position** (red circle): Current location

---

## Configuration

Both algorithms are highly configurable through a YAML configuration file:

### A* Configuration

```yaml
astar:
  debug: true                    # Enable visualization
  visualize_every: 10           # Update frequency
  heuristic_weight: 1.0         # Heuristic scaling factor
  
wall_cost:
  enabled: true                 # Enable wall proximity cost
  weight: 2.0                   # Cost magnitude
  decay: 'exponential'          # 'exponential', 'inverse', or 'linear'
  decay_rate: 0.5               # Decay speed
  threshold: 5.0                # Distance threshold
```

### Pure Pursuit Configuration

```yaml
pure_pursuit:
  debug: true                   # Enable visualization
  visualize_every: 5            # Update frequency
  lookahead_distance: 1.5       # Lookahead distance
  max_speed: 0.5                # Maximum speed
  min_speed: 0.2                # Minimum speed for turns
  steering_gain: 1.0            # Steering amplification
  slow_steering_threshold: 0.785 # Radians (π/4)
```

---

## Usage

### Basic Workflow

```python
# 1. Create map and algorithms
map = Map(maze_data)
astar = AStar(map, config)
pure_pursuit = PurePursuit(map, config)

# 2. Plan path
path_x, path_y = astar.plan_path(start_state, goal_state)
path = list(zip(path_x, path_y))

# 3. Set path for controller
pure_pursuit.set_path(path)

# 4. Control loop
while not reached_goal:
    speed, steering = pure_pursuit.get_control(current_pos, current_heading)
    # Apply control commands to robot
    # Update current_pos and current_heading
```

### Tuning Guidelines

**For A***:
- Increase `heuristic_weight` (> 1.0) for faster but potentially suboptimal paths
- Enable `wall_cost` for safer navigation in narrow spaces
- Adjust `wall_cost.weight` to balance safety vs path length

**For Pure Pursuit**:
- Increase `lookahead_distance` for smoother but less precise tracking
- Decrease `lookahead_distance` for tighter corners but potential oscillation
- Adjust speed limits based on robot dynamics and sensor capabilities
- Tune `steering_gain` if the robot under/over-steers

---

## Advantages and Limitations

### A* Algorithm

**Advantages**:
- Guarantees optimal path (shortest)
- Efficient with good heuristic
- Complete (finds a solution if one exists)

**Limitations**:
- Memory intensive for large maps
- Recomputation needed if obstacles change
- Path may have sharp angles requiring smoothing

### Pure Pursuit

**Advantages**:
- Simple and intuitive
- Smooth control outputs
- Computationally lightweight
- Works well for most paths

**Limitations**:
- May cut corners on sharp turns
- Lookahead distance requires tuning
- Can oscillate with poor parameter choices
- No obstacle avoidance (follows path blindly)

---

## Future Enhancements

Potential improvements could include:

1. **Path smoothing** after A* planning
2. **Dynamic obstacle avoidance** in Pure Pursuit
3. **Alternative planning algorithms** (RRT, D* Lite)
4. **Alternative controllers** (LQR, MPC)
5. **Adaptive lookahead distance** based on path curvature

---

## References

- Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). A Formal Basis for the Heuristic Determination of Minimum Cost Paths. *IEEE Transactions on Systems Science and Cybernetics*.

- Coulter, R. C. (1992). Implementation of the Pure Pursuit Path Tracking Algorithm. *Carnegie Mellon University Robotics Institute Technical Report*.

---

## License

This implementation is provided for educational purposes.
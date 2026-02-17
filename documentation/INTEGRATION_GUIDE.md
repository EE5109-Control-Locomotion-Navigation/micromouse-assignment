# Quick Integration Guide

This document shows how to integrate the different path planning algorithms with the existing simulation framework.

## Architecture Flow

```
config.yaml
    ↓ (select planner)
pathPlanning.py (AStar / ThetaStar / RRT)
    ↓ (produces waypoints)
optimize_path() (clothoid smoothing)
    ↓ (smoothed path)
pathTracking.py (PurePursuit / Stanley)
    ↓ (control commands)
DiffDriveRobot (execution)
```

---

## Selecting Path Planner

### In config.yaml:
```yaml
# Choose which planner to use
path_planner: "theta_star"  # Options: "astar", "theta_star", "rrt"
```

### In your simulation code:
```python
from pathPlanning import AStar, ThetaStar, RRT

# Read config
path_planner = config.get('path_planner', 'astar')

# Create appropriate planner
if path_planner == 'astar':
    planner = AStar(maze_map, config)
    path = planner.a_star()
elif path_planner == 'theta_star':
    planner = ThetaStar(maze_map, config)
    path = planner.plan()
elif path_planner == 'rrt':
    planner = RRT(maze_map, config)
    path = planner.plan()
else:
    raise ValueError(f"Unknown planner: {path_planner}")

print(f"Planned path with {len(path)} waypoints")
```

---

## Path Optimization

After planning, apply clothoid smoothing:

```python
from micromouse import optimize_path

# Plan path
raw_path = planner.plan()  # or planner.a_star() for AStar

# Optimize
optimized_path = optimize_path(raw_path, maze_map, min_distance=0.3)

# Use optimized path for tracking
controller.set_path(optimized_path)
```

---

## Selecting Path Tracking Controller

### In config.yaml:
```yaml
tracking_controller: "stanley"  # Options: "pure_pursuit", "stanley"
```

### In your simulation code:
```python
from pathTracking import PurePursuit, Stanley

# Read config
controller_type = config.get('tracking_controller', 'pure_pursuit')

# Create controller
if controller_type == 'pure_pursuit':
    controller = PurePursuit(maze_map, config)
elif controller_type == 'stanley':
    controller = Stanley(maze_map, config)
else:
    raise ValueError(f"Unknown controller: {controller_type}")

controller.set_path(optimized_path)
```

---

## Complete Example

```python
import yaml
from pathPlanning import AStar, ThetaStar
from pathTracking import PurePursuit, Stanley
from micromouse import Map, optimize_path

# Load configuration
with open('config.yaml', 'r') as f:
    config = yaml.safe_load(f)

# Load maze
maze_map = Map(maze_array)

# === PATH PLANNING ===
planner_type = config.get('path_planner', 'astar')
print(f"Using {planner_type} for planning...")

if planner_type == 'theta_star':
    planner = ThetaStar(maze_map, config)
    raw_path = planner.plan()
else:  # default to astar
    planner = AStar(maze_map, config)
    raw_path = planner.a_star()

print(f"Raw path: {len(raw_path)} waypoints")

# === PATH OPTIMIZATION ===
if config.get('path_optimization', {}).get('enabled', True):
    optimized_path = optimize_path(raw_path, maze_map, min_distance=0.3)
    print(f"Optimized path: {len(optimized_path)} waypoints")
else:
    optimized_path = raw_path

# === PATH TRACKING ===
controller_type = config.get('tracking_controller', 'pure_pursuit')
print(f"Using {controller_type} for tracking...")

if controller_type == 'stanley':
    controller = Stanley(maze_map, config)
else:
    controller = PurePursuit(maze_map, config)

controller.set_path(optimized_path)

# === SIMULATION LOOP ===
while not at_goal:
    # Get control commands
    if controller_type == 'stanley':
        # Stanley outputs steering angle for differential drive
        v_left, v_right = controller.compute_control_diff_drive(
            robot.position, robot.heading, robot.velocity, 
            wheel_base=0.08
        )
        robot.set_wheel_velocities(v_left, v_right)
    else:
        # Pure Pursuit outputs angular velocity
        steering, velocity = controller.compute_control(
            robot.position, robot.heading
        )
        robot.set_velocity(velocity, steering)
    
    # Update simulation
    robot.update(dt)
```

---

## Configuration Template

Here's a complete config.yaml template with all options:

```yaml
# === PLANNER SELECTION ===
path_planner: "theta_star"  # "astar", "theta_star", "rrt"

# === CONTROLLER SELECTION ===
tracking_controller: "stanley"  # "pure_pursuit", "stanley"

# === A* PARAMETERS ===
astar:
  debug: false
  heuristic_weight: 1.0
  turn_cost_enabled: true
  turn_cost_weight: 2.0

wall_cost:
  enabled: true
  weight: 4.0
  decay: "exponential"
  decay_rate: 0.5

# === THETA* PARAMETERS ===
theta_star:
  resolution: 1.0
  robot_radius: 0.3
  debug: true

# === RRT PARAMETERS ===
rrt:
  max_iter: 1000
  step_size: 0.5
  goal_bias: 0.1
  goal_tolerance: 1.0

# === PATH OPTIMIZATION ===
path_optimization:
  enabled: true
  method: "clothoid"
  min_wall_clearance: 0.3
  turn_threshold: 15  # degrees

# === PURE PURSUIT PARAMETERS ===
pure_pursuit:
  lookahead_distance: 3.0
  max_speed: 0.5
  min_speed: 0.2

# === STANLEY PARAMETERS ===
stanley:
  k: 1.0
  k_soft: 1.0
  max_steer: 1.047  # radians (60 degrees)
  wheel_base: 0.08   # meters
  max_speed: 0.5
  min_speed: 0.2
```

---

## Algorithm Comparison Example

```python
def compare_planners(maze_map, config):
    """Compare different planning algorithms"""
    results = {}
    
    planners = {
        'A*': AStar(maze_map, config),
        'Theta*': ThetaStar(maze_map, config)
    }
    
    for name, planner in planners.items():
        start_time = time.time()
        
        if name == 'A*':
            path = planner.a_star()
        else:
            path = planner.plan()
        
        elapsed = time.time() - start_time
        
        # Calculate metrics
        path_length = calculate_path_length(path)
        num_turns = count_turns(path)
        
        results[name] = {
            'waypoints': len(path),
            'length': path_length,
            'turns': num_turns,
            'time': elapsed
        }
    
    return results

# Run comparison
results = compare_planners(maze_map, config)

for name, metrics in results.items():
    print(f"\n{name}:")
    print(f"  Waypoints: {metrics['waypoints']}")
    print(f"  Length: {metrics['length']:.2f}")
    print(f"  Turns: {metrics['turns']}")
    print(f"  Time: {metrics['time']:.3f}s")
```

---

## Troubleshooting

### "ThetaStar not available"
- Ensure `theta_star.py` is in the project directory
- Check that the import in `pathPlanning.py` succeeds

### "No path found"
- Check that start and goal are in free space
- Verify obstacle map is correct
- Try increasing RRT max_iter or adjusting parameters

### Controller oscillations
- **Pure Pursuit**: Increase `lookahead_distance`
- **Stanley**: Decrease `k` gain, increase `k_soft`

### Path goes through walls
- Increase `robot_radius` in planner config
- Increase `min_wall_clearance` in optimization
- Check coordinate system conversions (row/col vs x/y)

### Coordinate mismatch errors
- Map uses (row, col): `map[row][col]`
- Visualization uses (x, y): `plot(col, row)` or `plot(x, y)`
- Theta* uses (x, y): convert properly at boundaries

---

## For Your Report

When comparing algorithms, include:

1. **Quantitative Metrics Table**
   ```
   | Algorithm | Waypoints | Length (m) | Turns | Time (ms) |
   |-----------|-----------|------------|-------|-----------|
   | A*        | 45        | 12.3       | 8     | 3.2       |
   | Theta*    | 28        | 11.1       | 5     | 4.1       |
   ```

2. **Visualization Comparison**
   - Side-by-side path plots on same maze
   - Color-code different algorithms
   - Annotate key differences

3. **Discussion Points**
   - Why did you choose your two algorithms?
   - Which performed better? In what scenarios?
   - Trade-offs observed (speed vs path quality)
   - Impact of path smoothing on each algorithm
   - Controller performance on different paths

---

## Next Steps

1. **Implement your chosen algorithms** (2 of 3: A*, Theta*, RRT)
2. **Implement Stanley controller** (optional, compare with Pure Pursuit)
3. **Implement clothoid optimization** (optimize_path function)
4. **Run experiments** on multiple mazes
5. **Collect metrics** and create visualizations
6. **Write report** comparing your implementations

Good luck!

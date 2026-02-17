# Integration Guide - Quick Reference

This guide provides a quick reference for integrating all components in the micromouse assignment.

## Table of Contents
- [Quick Start](#quick-start)
- [Path Planning Integration](#path-planning-integration)
- [Path Tracking Integration](#path-tracking-integration)
- [Path Optimization Integration](#path-optimization-integration)
- [Configuration System](#configuration-system)
- [Common Workflow Patterns](#common-workflow-patterns)

---

## Quick Start

### 1. Select Your Algorithms

In `config.yaml`:
```yaml
path_planner: "astar"                    # astar | theta_star | rrt
tracking_controller: "pure_pursuit"       # pure_pursuit | stanley
path_optimization:
  enabled: true                           # true | false
  method: "clothoid"                      # clothoid | circular_arc | bezier
```

### 2. Run Simulation

```bash
python simulation.py
```

### 3. View Results

- Real-time visualization during execution
- Final path comparison plot
- Console output with metrics

---

## Path Planning Integration

### A* (Baseline - Already Integrated)

```python
from pathPlanning import AStar

planner = AStar(maze_map, config)
path = planner.a_star()
```

**Configuration:**
```yaml
astar:
  debug: false
  heuristic_weight: 1.0
  turn_cost_enabled: true
  turn_cost_weight: 2.0
```

**Key Features:**
- Grid-based optimal planning
- Wall proximity cost
- Turn cost penalty
- A* search visualization

---

### Theta* (Provided with Adapter)

**1. Import and Create Instance:**
```python
from pathPlanning import ThetaStar

planner = ThetaStar(maze_map, config)
path = planner.plan()
```

**2. Configuration:**
```yaml
theta_star:
  resolution: 1.0
  robot_radius: 0.3
  debug: true
```

**3. What the Adapter Does:**
- Converts Map grid → obstacle list for theta_star.py
- Handles (row, col) ↔ (x, y) coordinate conversion
- Wraps theta_star.planning() method
- Converts output back to waypoint list

**4. Coordinate Systems:**
```python
# Map: (row, col) - origin at top-left
# Theta*: (x, y) - origin at bottom-left
# Conversion in adapter:
x = col
y = row
```

**For Details:** See [theta_star_integration.md](./theta_star_integration.md)

---

### RRT (Sample Code Provided)

**1. Choose Your Variant:**
- **rrt.py**: Basic RRT (good for learning)
- **rrt_with_sobol_sampler.py**: Faster convergence (better space coverage)
- **rrt_with_pathsmoothing.py**: Smoothed output (better path quality)

**2. Create Adapter (similar to ThetaStar):**
```python
from pathPlanning import RRT

planner = RRT(maze_map, config)
path = planner.plan()
```

**3. Configuration:**
```yaml
rrt:
  expand_dis: 3.0
  path_resolution: 0.5
  goal_sample_rate: 5
  max_iter: 500
  robot_radius: 0.5
  debug: true
```

**4. Adapter Structure:**
```python
class RRT:
    def __init__(self, maps, config):
        # Convert Map to obstacles
        self.obstacle_list = self._map_to_obstacles()
        
    def _map_to_obstacles(self):
        # Map grid cells → [x, y, radius] obstacles
        obstacles = []
        for i in range(self.map.row):
            for j in range(self.map.col):
                if self.map.map[i][j].state == "#":
                    obstacles.append([float(j), float(i), 0.5])
        return obstacles
    
    def plan(self):
        # Run RRT and convert output
        rrt = RRTCore(start, goal, self.obstacle_list, ...)
        path_xy = rrt.planning(animation=False)
        # Convert [x,y] → (row,col)
        return path
```

**For Details:** See [rrt_integration.md](./rrt_integration.md)

---

## Path Tracking Integration

### Pure Pursuit (Baseline - Already Integrated)

```python
from pathTracking import PurePursuit

controller = PurePursuit(path, config)
v, omega = controller.compute_control(robot_state)
```

**Configuration:**
```yaml
pure_pursuit:
  lookahead_distance: 3.0
  max_speed: 0.5
  min_speed: 0.2
  debug: true
```

**How It Works:**
1. Finds lookahead point on path at fixed distance
2. Computes heading toward lookahead
3. Returns (v, ω) control commands

---

### Stanley Controller (To Implement)

**1. Add to pathTracking.py:**
```python
class Stanley:
    def __init__(self, path, config):
        self.path = path
        self.k = config.get('stanley', {}).get('k', 1.0)
        self.k_soft = config.get('stanley', {}).get('k_soft', 1.0)
        self.max_steer = config.get('stanley', {}).get('max_steer', 1.047)
        self.wheel_base = config.get('stanley', {}).get('wheel_base', 0.08)
        
    def compute_control(self, state):
        # 1. Find nearest point on path
        nearest_idx = self.find_nearest_point(state)
        
        # 2. Calculate heading error
        psi = heading_error(state, self.path[nearest_idx])
        
        # 3. Calculate cross-track error
        e = cross_track_error(state, self.path[nearest_idx])
        
        # 4. Stanley formula
        delta = psi + np.arctan2(self.k * e, state.v + self.k_soft)
        delta = np.clip(delta, -self.max_steer, self.max_steer)
        
        # 5. Convert to differential drive
        v, omega = self.to_differential_drive(state.v, delta)
        
        return v, omega
    
    def to_differential_drive(self, v, delta):
        # Ackermann → Differential drive conversion
        omega = v * np.tan(delta) / self.wheel_base
        return v, omega
```

**2. Configuration:**
```yaml
stanley:
  k: 1.0
  k_soft: 1.0
  max_steer: 1.047
  wheel_base: 0.08
  max_speed: 0.5
  debug: true
```

**3. Integrate in Simulation:**
```python
# In simulation.py or micromouse.py
if controller_type == "stanley":
    controller = Stanley(path, config)
else:
    controller = PurePursuit(path, config)
```

**For Details:** See [stanley.md](./stanley.md)

---

## Path Optimization Integration

### Clothoid Smoothing (To Implement)

**1. Add to micromouse.py:**
```python
def optimize_path(path, maze_map, config):
    """
    Smooth path corners with clothoid curves.
    
    Args:
        path: Raw waypoint list from planner
        maze_map: Map object for collision checking
        config: Path optimization configuration
    
    Returns:
        Smoothed path with more waypoints but continuous curvature
    """
    if not config.get('path_optimization', {}).get('enabled', False):
        return path
    
    min_clearance = config['path_optimization']['min_wall_clearance']
    turn_threshold = config['path_optimization']['turn_threshold']
    
    # 1. Detect corners
    corners = detect_corners(path, turn_threshold)
    
    # 2. Fit clothoids at corners
    smoothed_path = []
    for i, point in enumerate(path):
        if i in corners:
            # Replace corner with clothoid
            segment = fit_clothoid(path, i, maze_map, min_clearance)
            smoothed_path.extend(segment)
        else:
            smoothed_path.append(point)
    
    # 3. Resample uniformly
    smoothed_path = resample_path(smoothed_path, resolution=0.1)
    
    return smoothed_path
```

**2. Configuration:**
```yaml
path_optimization:
  enabled: true
  method: "clothoid"
  min_wall_clearance: 0.3
  turn_threshold: 15        # degrees
```

**3. Integration:**
```python
# After planning
raw_path = planner.plan()

# Optimize if enabled
if config.get('path_optimization', {}).get('enabled', False):
    optimized_path = optimize_path(raw_path, maze_map, config)
else:
    optimized_path = raw_path

# Use optimized path for tracking
controller = PurePursuit(optimized_path, config)
```

**For Details:** See [clothoids.md](./clothoids.md)

---

## Configuration System

### Structure

```yaml
# ALGORITHM SELECTION
path_planner: "astar"           # Which planner to use
tracking_controller: "pure_pursuit"  # Which controller to use

# MAZE CONFIGURATION
maze_file: "mazefiles/classic/AAMC16Maze.txt"

# ALGORITHM PARAMETERS
astar:
  # A* specific config
  
theta_star:
  # Theta* specific config
  
rrt:
  # RRT specific config
  
pure_pursuit:
  # Pure Pursuit specific config
  
stanley:
  # Stanley specific config
  
path_optimization:
  # Optimization config
```

### Accessing in Code

```python
import yaml

# Load config
with open('config.yaml', 'r') as f:
    config = yaml.safe_load(f)

# Access values
planner_type = config.get('path_planner', 'astar')
lookahead = config.get('pure_pursuit', {}).get('lookahead_distance', 3.0)
```

### Debug Flags

Enable visualization for specific components:

```yaml
astar:
  debug: true          # Shows A* search progress

theta_star:
  debug: true          # Shows Theta* exploration

rrt:
  debug: true          # Shows RRT tree growth

pure_pursuit:
  debug: true          # Shows lookahead point and trajectory

stanley:
  debug: true          # Shows nearest point and errors
```

---

## Common Workflow Patterns

### 1. Add New Planner

**Step 1: Implement Class**
```python
# In pathPlanning.py
class MyPlanner:
    def __init__(self, maps, config):
        self.map = maps
        self.config = config
        
    def plan(self):
        # Your planning logic
        return path
```

**Step 2: Add to Config**
```yaml
my_planner:
  parameter1: value1
  parameter2: value2
```

**Step 3: Integrate in Main**
```python
# In simulation.py or micromouse.py
planner_type = config.get('path_planner')

if planner_type == 'my_planner':
    planner = MyPlanner(maze_map, config)
    path = planner.plan()
```

---

### 2. Add New Controller

**Step 1: Implement Class**
```python
# In pathTracking.py
class MyController:
    def __init__(self, path, config):
        self.path = path
        self.config = config
        
    def compute_control(self, state):
        # Your control logic
        return v, omega
```

**Step 2: Add to Config**
```yaml
my_controller:
  gain: 1.0
  max_speed: 0.5
```

**Step 3: Integrate in Main**
```python
controller_type = config.get('tracking_controller')

if controller_type == 'my_controller':
    controller = MyController(path, config)
    
v, omega = controller.compute_control(robot_state)
```

---

### 3. Compare Algorithms

**Pattern 1: Sequential Runs**
```python
results = {}

for planner_type in ['astar', 'theta_star', 'rrt']:
    config['path_planner'] = planner_type
    
    planner = create_planner(planner_type, maze_map, config)
    path = planner.plan()
    
    results[planner_type] = {
        'length': calculate_length(path),
        'waypoints': len(path),
        'time': execution_time
    }
```

**Pattern 2: Side-by-Side Visualization**
```python
import matplotlib.pyplot as plt

fig, axes = plt.subplots(1, 3, figsize=(15, 5))

for idx, planner_type in enumerate(['astar', 'theta_star', 'rrt']):
    path = run_planner(planner_type, maze_map, config)
    
    axes[idx].imshow(maze_map.grid)
    axes[idx].plot(*zip(*path))
    axes[idx].set_title(f'{planner_type}: {len(path)} waypoints')

plt.show()
```

---

### 4. Metric Calculation

```python
import math

def calculate_path_length(path):
    """Euclidean path length"""
    length = 0.0
    for i in range(len(path) - 1):
        dx = path[i+1][1] - path[i][1]
        dy = path[i+1][0] - path[i][0]
        length += math.sqrt(dx*dx + dy*dy)
    return length

def calculate_curvature(path):
    """Path curvature at each point"""
    curvatures = []
    for i in range(1, len(path) - 1):
        # Three-point curvature estimation
        p0, p1, p2 = path[i-1], path[i], path[i+1]
        k = curvature_from_points(p0, p1, p2)
        curvatures.append(k)
    return curvatures

def count_turns(path):
    """Count direction changes"""
    turns = 0
    for i in range(1, len(path) - 1):
        angle_change = heading_difference(path[i-1:i+2])
        if abs(angle_change) > math.radians(15):
            turns += 1
    return turns
```

---

### 5. Debugging Tips

**Enable All Debug Flags:**
```yaml
astar:
  debug: true
theta_star:
  debug: true
rrt:
  debug: true
pure_pursuit:
  debug: true
stanley:
  debug: true
```

**Print State Information:**
```python
# In simulation loop
if debug:
    print(f"Step {step}: pos=({state.x:.2f}, {state.y:.2f}), "
          f"heading={math.degrees(state.theta):.1f}°, "
          f"v={state.v:.2f}, omega={omega:.2f}")
```

**Visualize Intermediate Steps:**
```python
import matplotlib.pyplot as plt

# After planning
plt.figure(figsize=(10, 10))
plt.imshow(maze_map.grid)
plt.plot(*zip(*raw_path), 'b-', label='Raw')
plt.plot(*zip(*optimized_path), 'r-', label='Optimized')
plt.legend()
plt.show()
```

---

## Quick Troubleshooting

| Problem | Likely Cause | Solution |
|---------|--------------|----------|
| Import error | Missing dependency | `pip install -r requirements.txt` |
| Coordinate mismatch | Row/col vs x/y | Check adapter conversion |
| Path through walls | Wrong obstacle format | Verify obstacle list in adapter |
| Controller oscillates | Gains too high | Reduce k in Stanley or lookahead in PP |
| Path not smooth | Optimization disabled | Set `path_optimization.enabled: true` |
| Slow planning | Complex maze + low iters | Increase RRT max_iter or use Theta* |
| Collision during tracking | Path too close to walls | Increase min_wall_clearance |

---

## File Organization Checklist

- [ ] `config.yaml` - Algorithm selection and parameters configured
- [ ] `pathPlanning.py` - Planners implemented (A* ✅, Theta* ✅, RRT 🎯)
- [ ] `pathTracking.py` - Controllers implemented (PP ✅, Stanley 🎯)
- [ ] `micromouse.py` - Path optimization implemented (clothoids 🎯)
- [ ] `simulation.py` - Main loop integrates all components
- [ ] `requirements.txt` - All dependencies listed

---

## Next Steps

1. **Choose Your Algorithms**: Decide which 2 planners to implement
2. **Read Implementation Guides**:
   - Theta*: [theta_star_integration.md](./theta_star_integration.md)
   - RRT: [rrt_integration.md](./rrt_integration.md)
   - Stanley: [stanley.md](./stanley.md)
   - Clothoids: [clothoids.md](./clothoids.md)
3. **Start with Adapters**: Get provided code working first
4. **Test Incrementally**: Test each component before integrating
5. **Compare and Analyze**: Run experiments and collect metrics
6. **Write Report**: Document your findings

---

**For detailed implementation guides, see:**
- [README.md](./README.md) - Complete project overview
- [assignment.md](./assignment.md) - Assignment requirements
- [theta_star_integration.md](./theta_star_integration.md) - Theta* integration
- [rrt_integration.md](./rrt_integration.md) - RRT integration
- [stanley.md](./stanley.md) - Stanley controller
- [clothoids.md](./clothoids.md) - Path optimization

**Good luck with your implementation! 🚀**

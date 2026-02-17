### **RRT Integration Guide**

#### **1. What is RRT?**

**RRT (Rapidly-exploring Random Tree)** is a **sampling-based path planning algorithm** that builds a tree of random configurations, gradually exploring the space until reaching the goal. Unlike grid-based methods (A*, Theta*), RRT doesn't require discretization and can handle high-dimensional spaces.

**Key Characteristics:**
- **Probabilistically complete**: Will find a path if one exists (given enough time)
- **Fast exploration**: Rapidly covers free space with bias toward unexplored regions
- **No grid required**: Works in continuous configuration space
- **Handles complex spaces**: Good for cluttered or high-dimensional environments

**Trade-offs vs Grid-Based:**
- ✅ No discretization artifacts, continuous paths
- ✅ Fast in open spaces
- ✅ Easily extends to higher dimensions
- ❌ Produces suboptimal, jagged paths
- ❌ Randomness can cause large variance in results
- ❌ Requires post-processing for smooth paths

---

#### **2. RRT Variants Provided**

Three RRT implementations are included from PythonRobotics:

| File | Description | Best For |
|------|-------------|----------|
| **rrt.py** | Basic RRT with uniform random sampling | Learning, simple mazes |
| **rrt_with_sobol_sampler.py** | RRT with Sobol quasi-random sampling | Better space coverage, faster convergence |
| **rrt_with_pathsmoothing.py** | RRT + post-processing path smoothing | Production use, smoother results |

**Recommendation**: Start with **rrt.py** for understanding, use **rrt_with_pathsmoothing.py** for best results.

---

#### **3. How RRT Works**

**Algorithm Overview:**

```
1. Initialize tree T with start node
2. For i = 1 to max_iterations:
   a. Sample random point x_rand in free space
   b. Find nearest node x_near in tree T
   c. Extend from x_near toward x_rand by step_size
   d. If collision-free:
      - Add new node x_new to tree
      - Connect x_new to x_near
   e. If x_new is close to goal:
      - Return path from start to goal through tree
3. Return failure if max_iterations reached
```

**Key Parameters:**
- **expand_dis**: Step size for tree extension (controls resolution)
- **goal_sample_rate**: % of time to sample goal directly (speeds convergence)
- **max_iter**: Maximum iterations before giving up
- **robot_radius**: Collision buffer around robot

**Visual:**
```
Start ●─────┐
      │     │
      │     ├──●
      │     │  │
      │     │  └──●
      ├──●  │
      │  │  └──●──● Goal
      │  └──●
      └──●
```
Tree grows randomly, exploring space until goal is reached.

---

#### **4. Integration with Existing Codebase**

The provided RRT implementations use a different interface than the Map class. You need an **adapter wrapper** (similar to ThetaStar).

### **Step 1: Create RRT Adapter Class**

Add this to `pathPlanning.py`:

```python
# At the top of pathPlanning.py
try:
    from rrt import RRT as RRTCore
except ImportError:
    RRTCore = None
    print("Warning: rrt.py not found. RRT planner will not be available.")

class RRT:
    """
    Adapter wrapper for RRT planner to work with Map class.
    
    RRT (Rapidly-exploring Random Tree) is a sampling-based algorithm
    that builds a tree of random configurations to find a path.
    """
    
    def __init__(self, maps, config=None):
        if RRTCore is None:
            raise ImportError("RRT not available. Ensure rrt.py is present.")
        
        self.map = maps
        self.config = config.get('rrt', {}) if config else {}
        
        # RRT Parameters
        self.expand_dis = self.config.get('expand_dis', 3.0)
        self.path_resolution = self.config.get('path_resolution', 0.5)
        self.goal_sample_rate = self.config.get('goal_sample_rate', 5)
        self.max_iter = self.config.get('max_iter', 500)
        self.robot_radius = self.config.get('robot_radius', 0.5)
        self.debug = self.config.get('debug', False)
        
        # Convert Map to obstacle list
        self.obstacle_list = self._map_to_obstacles()
        
        # Define search area
        self.rand_area = [0, max(self.map.row, self.map.col)]
        self.play_area = [0, self.map.col, 0, self.map.row]
        
        if self.debug:
            print(f"RRT initialized: {len(self.obstacle_list)} obstacles")
    
    def _map_to_obstacles(self):
        """
        Convert Map grid to obstacle list.
        
        RRT expects obstacles as circles: [x, y, radius]
        We approximate grid cells as circular obstacles.
        """
        obstacles = []
        
        for i in range(self.map.row):
            for j in range(self.map.col):
                if self.map.map[i][j].state == "#":
                    # Convert to (x, y, radius) format
                    # Map: (row, col) -> RRT: (x, y)
                    x = float(j)  # col -> x
                    y = float(i)  # row -> y
                    radius = 0.5  # Half cell width
                    obstacles.append([x, y, radius])
        
        return obstacles
    
    def plan(self):
        """
        Run RRT planning and return path.
        
        Returns:
            List of (row, col) tuples representing path from start to goal.
            Returns empty list if no path found.
        """
        # Get start and goal (convert row,col to x,y)
        start_row, start_col = self.map.start
        goal_row, goal_col = self.map.goal
        
        start = [float(start_col), float(start_row)]  # [x, y]
        goal = [float(goal_col), float(goal_row)]      # [x, y]
        
        if self.debug:
            print(f"RRT planning from {start} to {goal}")
        
        # Create RRT planner
        rrt = RRTCore(
            start=start,
            goal=goal,
            obstacle_list=self.obstacle_list,
            rand_area=self.rand_area,
            expand_dis=self.expand_dis,
            path_resolution=self.path_resolution,
            goal_sample_rate=self.goal_sample_rate,
            max_iter=self.max_iter,
            play_area=self.play_area,
            robot_radius=self.robot_radius
        )
        
        # Run planning (disable animation for integration)
        path_xy = rrt.planning(animation=False)
        
        if path_xy is None:
            print("RRT: No path found!")
            return []
        
        # Convert path from [x,y] to (row,col)
        path = []
        for point in path_xy:
            x, y = point
            col = int(round(x))
            row = int(round(y))
            path.append((row, col))
        
        if self.debug:
            print(f"RRT found path with {len(path)} waypoints")
        
        return path
    
    def get_path_length(self, path):
        """Calculate total Euclidean path length"""
        if len(path) < 2:
            return 0.0
        
        length = 0.0
        for i in range(len(path) - 1):
            dx = path[i+1][1] - path[i][1]
            dy = path[i+1][0] - path[i][0]
            length += math.sqrt(dx*dx + dy*dy)
        
        return length
```

---

### **Step 2: Update Configuration**

Add to `config.yaml`:

```yaml
# RRT PATH PLANNING PARAMETERS
rrt:
  expand_dis: 3.0           # Step size for tree extension
  path_resolution: 0.5      # Resolution for collision checking
  goal_sample_rate: 5       # % chance to sample goal (0-100)
  max_iter: 500             # Maximum iterations
  robot_radius: 0.5         # Collision buffer
  debug: true               # Enable debug output
  show_animation: false     # Disable matplotlib animation
```

---

### **Step 3: Disable Animation in rrt.py**

The original `rrt.py` has matplotlib animation. Modify:

```python
# At the top of rrt.py, change:
show_animation = False  # Changed from True
```

Or make it configurable by passing animation parameter in planning call.

---

### **Step 4: Update Main Simulation**

Modify `micromouse.py` or `simulation.py`:

```python
from pathPlanning import AStar, ThetaStar, RRT

# In planning section
planner_type = config.get('path_planner', 'astar')

if planner_type == 'astar':
    planner = AStar(maze_map, config)
    path = planner.a_star()
elif planner_type == 'theta_star':
    planner = ThetaStar(maze_map, config)
    path = planner.plan()
elif planner_type == 'rrt':
    planner = RRT(maze_map, config)
    path = planner.plan()
else:
    raise ValueError(f"Unknown planner: {planner_type}")

print(f"Path found with {len(path)} waypoints")
```

---

#### **5. RRT Variants**

### **A. Basic RRT (rrt.py)**

Standard implementation with uniform random sampling.

**Use when:**
- Learning RRT fundamentals
- Simple mazes with clear paths
- Debugging/understanding algorithm behavior

### **B. RRT with Sobol Sampling (rrt_with_sobol_sampler.py)**

Uses **Sobol sequences** (low-discrepancy quasi-random numbers) instead of uniform random sampling for better space coverage.

**Advantages:**
- **Faster convergence**: Sobol sequences fill space more uniformly
- **Better dispersion**: Avoids clustering of random samples
- **More deterministic**: Same seed produces same sequence

**Use when:**
- Want faster planning in complex mazes
- Need more consistent performance
- Research comparing sampling strategies

**Integration:**
```python
from rrt_with_sobol_sampler import RRTSobol

# Replace RRTCore with RRTSobol in adapter
# Everything else remains the same
```

### **C. RRT with Path Smoothing (rrt_with_pathsmoothing.py)**

Adds post-processing to smooth the jagged RRT path by shortcuts.

**Smoothing Process:**
1. Take RRT path
2. Try shortcuts between non-adjacent points
3. If shortcut is collision-free, replace path segment
4. Repeat until no more shortcuts possible

**Advantages:**
- **Shorter paths**: Removes unnecessary detours
- **Smoother**: Fewer sharp turns
- **Better for tracking**: Controllers perform better

**Use when:**
- Production/final results
- Path quality matters more than speed
- Comparing with optimized A*/Theta* paths

**Integration:**
```python
from rrt_with_pathsmoothing import path_smoothing

# After getting RRT path
path_xy = rrt.planning(animation=False)
smoothed_xy = path_smoothing(path_xy, max_iter=100)
# Convert smoothed_xy to (row,col) format
```

---

#### **6. Parameter Tuning Guide**

| Parameter | Typical Range | Effect |
|-----------|---------------|--------|
| `expand_dis` | 1.0 - 5.0 | Larger = faster exploration but coarser paths |
| `goal_sample_rate` | 5 - 20 | Higher = faster convergence but less exploration |
| `max_iter` | 500 - 5000 | More iterations = higher success rate but slower |
| `robot_radius` | 0.3 - 1.0 | Larger = safer but may fail in narrow passages |
| `path_resolution` | 0.1 - 1.0 | Finer = better collision detection but slower |

**Tuning Strategy:**
1. Start with defaults
2. If no path found → **increase max_iter** or **reduce robot_radius**
3. If path too jagged → **reduce expand_dis**
4. If too slow → **increase goal_sample_rate** or enable Sobol sampling
5. If path rough → use path smoothing variant

---

#### **7. Expected Performance vs Other Planners**

| Metric | A* | Theta* | RRT (basic) | RRT (smoothed) |
|--------|-------|--------|-------------|----------------|
| **Path length** | 100% | 85-95% | 120-150% | 100-110% |
| **Computation** | Fast | Fast | Variable | Slower |
| **Smoothness** | Grid-aligned | Smooth | Jagged | Moderate |
| **Optimality** | Optimal | Near-optimal | Suboptimal | Better |
| **Consistency** | Deterministic | Deterministic | Random | Random |

**When to use RRT:**
- ✅ Complex/cluttered environments
- ✅ Learning sampling-based methods
- ✅ When grid discretization is problematic
- ✅ High-dimensional spaces (beyond 2D)
- ❌ Need optimal paths (use A*/Theta*)
- ❌ Need deterministic results (use A*/Theta*)
- ❌ Simple grid mazes (A* is faster)

---

#### **8. Comparison Code Example**

```python
import time

def compare_all_planners(maze_map, config):
    """Compare A*, Theta*, and RRT"""
    results = {}
    
    planners = {
        'A*': AStar(maze_map, config),
        'Theta*': ThetaStar(maze_map, config),
        'RRT': RRT(maze_map, config)
    }
    
    for name, planner in planners.items():
        start_time = time.time()
        
        # Get path
        if name == 'A*':
            path = planner.a_star()
        else:
            path = planner.plan()
        
        elapsed = time.time() - start_time
        
        # Calculate metrics
        length = planner.get_path_length(path)
        turns = count_direction_changes(path)
        
        results[name] = {
            'waypoints': len(path),
            'length': length,
            'turns': turns,
            'time_ms': elapsed * 1000
        }
    
    return results

# Run comparison
results = compare_all_planners(maze_map, config)

# Print table
print("| Algorithm | Waypoints | Length | Turns | Time (ms) |")
print("|-----------|-----------|--------|-------|-----------|")
for name, metrics in results.items():
    print(f"| {name:9s} | {metrics['waypoints']:9d} | "
          f"{metrics['length']:6.2f} | {metrics['turns']:5d} | "
          f"{metrics['time_ms']:9.2f} |")
```

---

#### **9. Visualizing RRT Exploration**

```python
# Enable RRT tree visualization
from rrt import RRT as RRTCore

rrt = RRTCore(start, goal, obstacles, rand_area, ...)
path = rrt.planning(animation=True)  # Shows tree growth

# Or save final tree
import matplotlib.pyplot as plt

# After planning, visualize tree
fig, ax = plt.subplots(figsize=(10, 10))

# Draw obstacles
for obs in obstacles:
    circle = plt.Circle((obs[0], obs[1]), obs[2], color='black')
    ax.add_patch(circle)

# Draw tree
for node in rrt.node_list:
    if node.parent:
        ax.plot([node.x, node.parent.x], 
                [node.y, node.parent.y], 
                'c-', linewidth=0.5)

# Draw final path
if path:
    path_x = [p[0] for p in path]
    path_y = [p[1] for p in path]
    ax.plot(path_x, path_y, 'r-', linewidth=2, label='Path')

ax.plot(start[0], start[1], 'go', markersize=10, label='Start')
ax.plot(goal[0], goal[1], 'ro', markersize=10, label='Goal')
ax.legend()
ax.set_aspect('equal')
plt.show()
```

---

#### **10. Common Issues and Solutions**

| Problem | Cause | Solution |
|---------|-------|----------|
| No path found | max_iter too low | Increase to 1000-5000 |
| Path through walls | robot_radius too small | Increase to 0.5-1.0 |
| Very slow | Too many obstacles | Use grid centers only, not all cells |
| Path too jagged | Large expand_dis | Reduce expand_dis or use smoothing |
| Different results each run | Random sampling | Use Sobol variant for consistency |
| Tree grows away from goal | Low goal_sample_rate | Increase to 10-20% |

---

#### **11. Advanced: Combining RRT with Clothoids**

Since RRT produces jagged paths, it benefits greatly from path optimization:

```python
# 1. Plan with RRT
rrt_planner = RRT(maze_map, config)
raw_path = rrt_planner.plan()

# 2. Optional: Apply RRT path smoothing first
smoothed_path = apply_rrt_smoothing(raw_path)

# 3. Apply clothoid optimization
optimized_path = optimize_path(smoothed_path, maze_map, min_distance=0.3)

# Result: Smooth, kinematically-optimal path
controller.set_path(optimized_path)
```

**Benefits:**
- RRT explores space efficiently
- Smoothing removes obvious shortcuts
- Clothoids ensure constant curvature rate
- Final path is both short and smooth

---

#### **12. For Your Report**

When analyzing RRT, discuss:

1. **Success Rate vs Iterations**:
   - Plot % success vs max_iter
   - Show convergence behavior

2. **Path Quality**:
   - Compare raw RRT, smoothed RRT, RRT+clothoid
   - Show curvature profiles

3. **Randomness Effects**:
   - Run 10 times, plot variance
   - Compare uniform vs Sobol sampling

4. **Computational Cost**:
   - Planning time vs maze complexity
   - Trade-off: iterations vs path quality

5. **Integration Performance**:
   - How does controller handle jagged RRT paths?
   - Tracking error: raw vs smoothed paths

---

#### **13. References**

- LaValle, S. M. (1998). "Rapidly-Exploring Random Trees: A New Tool for Path Planning"
- LaValle, S. M. (2006). *Planning Algorithms*. Cambridge University Press.
- Karaman, S., & Frazzoli, E. (2011). "Sampling-based algorithms for optimal motion planning"
- PythonRobotics: https://github.com/AtsushiSakai/PythonRobotics

---

**Bottom Line**: RRT is excellent for learning sampling-based planning and handles complex spaces well, but produces suboptimal paths that benefit from post-processing. For micromouse (simple grid mazes), A* or Theta* are typically better choices, but RRT provides valuable algorithmic diversity for the assignment.

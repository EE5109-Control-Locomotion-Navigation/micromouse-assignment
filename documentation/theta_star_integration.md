### **Theta* Integration Guide**

#### **1. What is Theta*?**

**Theta*** is an **any-angle path planning algorithm** that extends A* to produce shorter, smoother paths by allowing line-of-sight connections between non-adjacent grid cells.

**Key Differences from A*:**
- **A***: Searches along grid edges (8-connected or 4-connected movement)
- **Theta***: Allows direct connections if line-of-sight exists (any-angle movement)

**Result**: 5-20% shorter paths with fewer unnecessary turns, reducing the need for aggressive path smoothing.

---

#### **2. How Theta* Works**

**Core Idea**: During neighbor expansion, check if the **grandparent** node has line-of-sight to the **neighbor**. If yes, connect directly to grandparent (path compression).

**Algorithm Modification** (compared to A*):
```python
# Standard A* neighbor expansion:
for neighbor in neighbors:
    neighbor.parent = current
    neighbor.cost = current.cost + distance(current, neighbor)

# Theta* modification:
for neighbor in neighbors:
    if line_of_sight(current.parent, neighbor):
        # Direct connection possible
        neighbor.parent = current.parent
        neighbor.cost = current.parent.cost + distance(current.parent, neighbor)
    else:
        # Standard A* connection
        neighbor.parent = current
        neighbor.cost = current.cost + distance(current, neighbor)
```

**Line-of-Sight Check**: Uses Bresenham's line algorithm to verify no obstacles between two points.

---

#### **3. Integration with Existing Codebase**

The provided `theta_star.py` uses a different data structure than the existing `Map` class. You need an **adapter wrapper**.

### **Step 1: Create Adapter Class**

Add this to `pathPlanning.py`:

```python
# At the top of pathPlanning.py
from theta_star import ThetaStarPlanner

class ThetaStar:
    """
    Adapter wrapper for ThetaStarPlanner to work with Map class.
    
    Theta* is an any-angle path planning algorithm that produces smoother
    paths than A* by allowing direct line-of-sight connections.
    """
    
    def __init__(self, maps, config=None):
        self.map = maps
        self.config = config.get('theta_star', {}) if config else {}
        
        # Configuration
        self.resolution = self.config.get('resolution', 1.0)
        self.robot_radius = self.config.get('robot_radius', 0.3)
        self.debug = self.config.get('debug', False)
        
        # Convert Map to obstacle lists
        self.ox, self.oy = self._map_to_obstacles()
        
        # Create underlying Theta* planner
        self.planner = ThetaStarPlanner(self.ox, self.oy, self.resolution, self.robot_radius)
        
        if self.debug:
            print(f"Theta* initialized: {len(self.ox)} obstacle points")
    
    def _map_to_obstacles(self):
        """
        Convert Map grid to obstacle point lists.
        
        The theta_star.py implementation expects lists of obstacle coordinates,
        while our Map class uses a 2D grid of State objects.
        """
        ox, oy = [], []
        
        for i in range(self.map.row):
            for j in range(self.map.col):
                if self.map.map[i][j].state == "#":
                    # Note: Coordinate system conversion
                    # Map uses (row, col) indexing where row=y, col=x
                    # Theta* expects (x, y) coordinates
                    ox.append(float(j))  # column -> x
                    oy.append(float(i))  # row -> y
        
        return ox, oy
    
    def plan(self):
        """
        Run Theta* planning and return path.
        
        Returns:
            List of (row, col) tuples representing the path from start to goal.
            Returns empty list if no path found.
        """
        # Get start and goal from map (row, col format)
        start_row, start_col = self.map.start
        goal_row, goal_col = self.map.goal
        
        # Convert to (x, y) for theta_star
        sx = float(start_col)
        sy = float(start_row)
        gx = float(goal_col)
        gy = float(goal_row)
        
        if self.debug:
            print(f"Planning from ({sx}, {sy}) to ({gx}, {gy})")
        
        # Run Theta* planning
        # Returns (rx, ry) where rx[0] is goal x, rx[-1] is start x
        rx, ry = self.planner.planning(sx, sy, gx, gy)
        
        if not rx or not ry:
            print("Theta*: No path found!")
            return []
        
        # Convert back to (row, col) format and reverse order
        # theta_star returns path from goal to start, we want start to goal
        path = []
        for i in range(len(rx) - 1, -1, -1):  # Reverse iteration
            row = int(round(ry[i]))  # y -> row
            col = int(round(rx[i]))  # x -> col
            path.append((row, col))
        
        if self.debug:
            print(f"Theta* found path with {len(path)} waypoints")
        
        return path
    
    def get_path_length(self, path):
        """Calculate total Euclidean path length"""
        if len(path) < 2:
            return 0.0
        
        length = 0.0
        for i in range(len(path) - 1):
            dx = path[i+1][1] - path[i][1]  # col difference
            dy = path[i+1][0] - path[i][0]  # row difference
            length += math.sqrt(dx*dx + dy*dy)
        
        return length
```

---

### **Step 2: Update Configuration**

Add to `config.yaml`:

```yaml
# Path Planning Configuration
path_planner: "theta_star"  # Options: "astar", "theta_star", "rrt"

theta_star:
  resolution: 1.0        # Grid resolution (should match maze cell size)
  robot_radius: 0.3      # Robot radius for obstacle inflation
  debug: true            # Enable debug output
  show_animation: false  # Disable matplotlib animation during planning
```

---

### **Step 3: Update Main Simulation**

Modify `micromouse.py` or `simulation.py` to use Theta*:

```python
from pathPlanning import AStar, ThetaStar

# In planning section
planner_type = config.get('path_planner', 'astar')

if planner_type == 'astar':
    planner = AStar(maze_map, config)
    path = planner.a_star()
elif planner_type == 'theta_star':
    planner = ThetaStar(maze_map, config)
    path = planner.plan()
else:
    raise ValueError(f"Unknown planner type: {planner_type}")

print(f"Path length: {len(path)} waypoints")
```

---

### **Step 4: Disable Animation in theta_star.py**

The original `theta_star.py` has matplotlib animation that conflicts with the main simulation. Modify it:

```python
# At the top of theta_star.py, change:
show_animation = False  # Changed from True

# Or make it configurable:
class ThetaStarPlanner:
    def __init__(self, ox, oy, resolution, rr, show_animation=False):
        self.show_animation = show_animation
        # ... rest of init
    
    def planning(self, sx, sy, gx, gy):
        # ... in the main loop, change:
        if self.show_animation:  # Instead of global show_animation
            # ... animation code
```

---

#### **4. Coordinate System Notes**

**Critical Issue**: Different coordinate conventions!

| System | Format | Origin |
|--------|--------|--------|
| **Map class** | `(row, col)` where `map[row][col]` | Top-left (0,0) |
| **Theta*** | `(x, y)` like Cartesian | Bottom-left typically |
| **Conversion** | `x = col`, `y = row` | Preserve topology |

**Always convert at boundaries** between systems:
- **Before planning**: `(row, col) → (x, y)`
- **After planning**: `(x, y) → (row, col)`

---

#### **5. Expected Performance vs A***

| Metric | A* | Theta* | Improvement |
|--------|-------|--------|-------------|
| **Path length** | 100% | 85-95% | 5-15% shorter |
| **Waypoints** | High | Low | 30-50% fewer |
| **Turns** | Many | Few | Smoother |
| **Computation** | Fast | Slightly slower | +10-30% time |
| **Memory** | O(n) | O(n) | Similar |

**When to use Theta*:**
- ✅ Want shorter, smoother paths
- ✅ Willing to trade slight computation time for path quality
- ✅ Grid-based environment (like maze)
- ✅ Path smoothing is expensive (Theta* reduces need)

**When A* might be better:**
- Real-time replanning required (A* slightly faster)
- Highly constrained spaces where any-angle doesn't help
- When grid alignment is required (e.g., discrete actions only)

---

#### **6. Testing Your Implementation**

**Test 1: Simple Path**
```python
# Create simple maze (corridor with turn)
# Compare A* vs Theta* paths
# Theta* should cut the corner

astar_planner = AStar(maze_map, config)
astar_path = astar_planner.a_star()

theta_planner = ThetaStar(maze_map, config)
theta_path = theta_planner.plan()

print(f"A* waypoints: {len(astar_path)}")
print(f"Theta* waypoints: {len(theta_path)}")
print(f"A* length: {compute_path_length(astar_path):.2f}")
print(f"Theta* length: {compute_path_length(theta_path):.2f}")
```

**Test 2: Complex Maze**
Use one of the All-Japan or AAMC mazes to see Theta* path improvement in realistic scenarios.

**Test 3: Visualization**
```python
import matplotlib.pyplot as plt

fig, axes = plt.subplots(1, 2, figsize=(14, 6))

# Plot A* path
astar_arr = np.array(astar_path)
axes[0].imshow(maze_grid, cmap='gray')
axes[0].plot(astar_arr[:, 1], astar_arr[:, 0], 'r-', linewidth=2)
axes[0].set_title(f'A* Path ({len(astar_path)} waypoints)')

# Plot Theta* path
theta_arr = np.array(theta_path)
axes[1].imshow(maze_grid, cmap='gray')
axes[1].plot(theta_arr[:, 1], theta_arr[:, 0], 'b-', linewidth=2)
axes[1].set_title(f'Theta* Path ({len(theta_path)} waypoints)')

plt.show()
```

---

#### **7. Common Issues and Solutions**

| Problem | Cause | Solution |
|---------|-------|----------|
| Empty path returned | Start/goal coordinates swapped | Check (row,col)→(x,y) conversion |
| Path goes through walls | Resolution mismatch | Set `resolution=1.0` to match maze grid |
| "Open set empty" error | Goal unreachable or blocked | Check maze has valid path, verify collision checking |
| Coordinate mismatch visualization | Row/col vs x/y confusion | Always use `plot(col, row)` not `plot(row, col)` |
| Slow performance | Too many obstacle points | Use only wall boundaries, not filled areas |

---

#### **8. Advantages for Path Optimization**

**Synergy with Clothoids:**

Since Theta* already produces smoother paths with fewer turns:
- **Fewer corners to smooth** → faster clothoid optimization
- **More gentle turns** → easier to fit clothoids without collisions
- **Better initial guess** → clothoid fitting more likely to succeed

**Example workflow:**
1. **Theta* planning** → produces smooth any-angle path
2. **Corner detection** → identify remaining sharp turns
3. **Clothoid smoothing** → apply only where needed (less aggressive)
4. **Result** → Near-optimal path with minimal computation

---

#### **9. Report Analysis Suggestions**

When comparing Theta* to other planners, analyze:

1. **Path Metrics:**
   - Total Euclidean length
   - Number of waypoints
   - Average turn angle
   - Total absolute curvature

2. **Computational Metrics:**
   - Planning time
   - Number of nodes expanded
   - Memory usage

3. **Tracking Performance:**
   - How does smoother Theta* path affect controller tracking error?
   - Average cross-track error with Pure Pursuit vs Stanley
   - Total navigation time (planning + execution)

4. **Optimization Impact:**
   - How much does clothoid smoothing improve Theta* paths vs A* paths?
   - Is the improvement worth the computation?

---

#### **10. References**

- Nash, A., et al. (2007). "Theta*: Any-Angle Path Planning on Grids." *AAAI Conference on Artificial Intelligence*.
- Nash, A., & Koenig, S. (2013). "Any-Angle Path Planning." *AI Magazine*, 34(4), 85-107.
- Daniel, K., et al. (2010). "Theta*: Any-angle path planning on grids." *Journal of Artificial Intelligence Research*, 39, 533-579.

---

**Bottom Line**: Theta* provides a significant path quality improvement over A* with minimal additional complexity. It's an excellent choice for students who want shorter paths and smoother navigation without implementing more complex sampling-based methods like RRT.

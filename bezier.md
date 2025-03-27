### **Bézier Curve Path Optimization for Micromouse**

#### **1. Why Use Bézier Curves?**
- **Smoothness**: Continuous curvature (no sharp turns)
- **Safety**: Can enforce minimum clearance from walls
- **Computational Efficiency**: Simple polynomial form
- **Tunability**: Control smoothness via control points

#### **2. Implementation Steps**

**Step 1: Extract Waypoints**  
Take the original path (from A*/RRT) as waypoints:  
`P = [(x0,y0), (x1,y1), ..., (xn,yn)]`

**Step 2: Create Bézier Segments**  
For each set of 4 consecutive waypoints (cubic Bézier):  
```python
def cubic_bezier(t, p0, p1, p2, p3):
    """Evaluate cubic Bézier at parameter t ∈ [0,1]"""
    return (1-t)**3 * p0 + 3*(1-t)**2*t * p1 + 3*(1-t)*t**2 * p2 + t**3 * p3
```

**Step 3: Optimize Control Points**  
Adjust intermediate control points (`p1`, `p2`) to:  
1. **Minimize curvature**:  
   ```math
   \kappa(t) = \frac{|\dot{x}\ddot{y} - \dot{y}\ddot{x}|}{(\dot{x}^2 + \dot{y}^2)^{3/2}}
   ```
2. **Enforce safety margin** (`d_min` from walls)  
3. **Preserve endpoint continuity** (C² smooth between segments)

**Step 4: Velocity Profile**  
Assign speeds based on curvature:  
```python
def speed_from_curvature(kappa, max_speed=0.5, min_speed=0.1):
    return max_speed * exp(-alpha * kappa)  # Tune α for aggressiveness
```

---

### **3. Python Implementation**

**1. Bézier Generation**
```python
from scipy.optimize import minimize

def optimize_bezier(waypoints, maze, d_min=0.3):
    """Optimize control points for safety/smoothness"""
    def cost(control_points):
        # Calculate max curvature and wall distance
        total_curvature = sum(calc_curvature(control_points))
        min_distance = min_distance_to_walls(control_points, maze)
        return total_curvature + 1e6*(min_distance < d_min)  # Penalize collisions
    
    # Initial guess (linear interpolation)
    init_controls = initialize_control_points(waypoints)
    
    # Optimize
    res = minimize(cost, init_controls, method='SLSQP')
    return res.x
```

**2. Curvature Calculation**
```python
def calc_curvature(control_points):
    """Compute curvature at sample points along Bézier"""
    ts = np.linspace(0, 1, 20)
    curvatures = []
    for t in ts:
        dx = 3*(1-t)**2*(p1-p0) + 6*(1-t)*t*(p2-p1) + 3*t**2*(p3-p2)  # 1st derivative
        ddx = 6*(1-t)*(p2-2*p1+p0) + 6*t*(p3-2*p2+p1)                 # 2nd derivative
        curvatures.append(np.linalg.norm(np.cross(dx, ddx)) / np.linalg.norm(dx)**3)
    return max(curvatures)
```

**3. Safety Check**
```python
def min_distance_to_walls(control_points, maze):
    """Bresenham's line algorithm between control points"""
    min_dist = float('inf')
    for i in range(len(control_points)-1):
        for x,y in bresenham_line(control_points[i], control_points[i+1]):
            if maze.map[int(x)][int(y)].state == "#":
                min_dist = min(min_dist, distance_to_nearest_wall(x,y,maze))
    return min_dist
```

---

### **4. Integration with Existing Code**

**Modify `astar.py`/`rrt.py`:**
```python
def plan_path(self):
    raw_path = self.a_star()  # Original path
    return bezier_optimize(raw_path, self.map)
```

**In `main.py`:**
```python
# Visualize comparison
plt.plot(raw_path[:,0], raw_path[:,1], 'r--', label="Original")
plt.plot(optimized_path[:,0], optimized_path[:,1], 'b-', label="Bézier")
```

---

### **5. Key Optimization Parameters**
| Parameter       | Typical Value | Effect                          |
|-----------------|---------------|---------------------------------|
| `d_min`         | 0.2-0.5       | Minimum wall clearance          |
| Curvature weight| 1.0-10.0      | Smoothness vs path length tradeoff |
| Segment length  | 3-5 cells     | Balance between smoothness/local control |

---

### **6. Advantages Over Simple Path Smoothers**
1. **Exact curvature control** (unlike splines)
2. **Closed-form derivatives** for LQR tracking
3. **Local adjustability** without global recomputation
4. **Guaranteed passability** when combined with collision checks

---

**Example Result:**  
![Bézier Optimization](https://i.imgur.com/JQZ1x0l.png)  
*Red: Original A* path, Blue: Bézier-optimized*


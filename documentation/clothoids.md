### **Clothoid Curves for Path Optimization**

#### **1. Why Clothoids (Euler Spirals)?**

- **Linear Curvature Growth**: Curvature κ increases linearly with arc length → smooth steering commands
- **Kinematically Optimal**: Matches natural motion of differential drive robots (constant angular acceleration)
- **G² Continuity**: Smooth position, tangent, AND curvature at all points
- **Minimizes Jerk**: Reduces mechanical stress and improves tracking accuracy
- **Industry Standard**: Used in highway design, railway curves, and autonomous vehicle planning

---

#### **2. Mathematical Foundation**

A **clothoid** (Euler spiral) is defined by the property:

```math
\kappa(s) = \kappa_0 + \frac{s}{L} \cdot (\kappa_f - \kappa_0)
```

Where:
- **κ(s)**: Curvature at arc length `s`
- **κ₀**: Initial curvature (often 0 for straight entry)
- **κ_f**: Final curvature (often 0 for straight exit)
- **L**: Total arc length of the spiral
- **s**: Distance along curve from start

**Key Insight**: Unlike circular arcs (constant κ) or Bézier curves (arbitrary κ), clothoids have **monotonic curvature** which is naturally trackable by wheeled robots.

**Parametric Form** (using Fresnel integrals):

```math
x(s) = \int_0^s \cos\left(\frac{\pi t^2}{2L^2}\right) dt
```

```math
y(s) = \int_0^s \sin\left(\frac{\pi t^2}{2L^2}\right) dt
```

These integrals don't have closed-form solutions, requiring numerical methods or lookup tables.

---

#### **3. Implementation Approaches**

### **Approach A: Using pyclothoids Library** ⭐ (Recommended)

The `pyclothoids` library provides robust clothoid fitting with boundary value solvers.

**Installation:**
```bash
pip install pyclothoids
```

**Basic Usage:**
```python
from pyclothoids import Clothoid
import numpy as np

def fit_clothoid_segment(x0, y0, theta0, x1, y1, theta1):
    """
    Fit a clothoid curve between two poses.
    
    Args:
        x0, y0: Start position
        theta0: Start heading (radians)
        x1, y1: End position
        theta1: End heading (radians)
        
    Returns:
        Array of (x, y) points along clothoid
    """
    try:
        # Create clothoid connecting two poses
        clothoid = Clothoid.StandardParams(
            x0, y0, theta0,
            x1, y1, theta1
        )
        
        # Sample points along the curve
        num_points = 20
        s_values = np.linspace(0, clothoid.length, num_points)
        
        points = []
        for s in s_values:
            x, y = clothoid.XY(s)
            points.append([x, y])
        
        return np.array(points)
    
    except Exception as e:
        # Clothoid fitting failed (e.g., poses too close or incompatible)
        # Fall back to straight line
        print(f"Clothoid fitting failed: {e}")
        return np.array([[x0, y0], [x1, y1]])
```

---

### **Approach B: G¹ Clothoid Pairs** (Simpler)

For 90° corners in mazes, use symmetric clothoid pairs:

```python
def create_corner_clothoid(entry_point, corner_point, exit_point, map, min_clearance=0.3):
    """
    Create a smooth corner using two symmetric clothoids.
    
    Args:
        entry_point: (x, y) point before corner
        corner_point: (x, y) corner waypoint
        exit_point: (x, y) point after corner
        map: Maze map for collision checking
        min_clearance: Minimum distance from walls
        
    Returns:
        List of points forming smooth corner
    """
    # Calculate approach and exit headings
    theta_in = math.atan2(corner_point[1] - entry_point[1],
                          corner_point[0] - entry_point[0])
    theta_out = math.atan2(exit_point[1] - corner_point[1],
                           exit_point[0] - corner_point[0])
    
    # Turn angle
    delta_theta = normalize_angle(theta_out - theta_in)
    
    # If nearly straight, skip smoothing
    if abs(delta_theta) < 0.1:  # ~6 degrees
        return [corner_point]
    
    # Determine clothoid length based on available space
    max_length = min(
        distance(entry_point, corner_point),
        distance(corner_point, exit_point)
    ) * 0.8  # Use 80% of available space
    
    # Calculate middle point for symmetric pair
    mid_theta = theta_in + delta_theta / 2
    offset = max_length / 2
    mid_point = [
        corner_point[0] + offset * math.cos(mid_theta),
        corner_point[1] + offset * math.sin(mid_theta)
    ]
    
    # Create two clothoid segments
    try:
        # Entry clothoid: straight → turning
        seg1 = fit_clothoid_segment(
            entry_point[0], entry_point[1], theta_in,
            mid_point[0], mid_point[1], mid_theta
        )
        
        # Exit clothoid: turning → straight
        seg2 = fit_clothoid_segment(
            mid_point[0], mid_point[1], mid_theta,
            exit_point[0], exit_point[1], theta_out
        )
        
        # Combine segments
        smoothed_corner = np.vstack([seg1[:-1], seg2])
        
        # Verify safety
        if check_path_clearance(smoothed_corner, map, min_clearance):
            return smoothed_corner.tolist()
        else:
            # Collision detected, fall back to original waypoint
            return [corner_point]
    
    except:
        # Clothoid fitting failed
        return [corner_point]
```

---

### **Approach C: Lookup Table Method** (For Performance)

If computational speed is critical, pre-compute clothoid segments:

```python
class ClothoidLookupTable:
    """Pre-computed clothoid segments for common angles"""
    
    def __init__(self):
        # Pre-compute clothoids for common turn angles
        self.angles = [30, 45, 60, 90, 120, 135, 150]  # degrees
        self.table = {}
        
        for angle_deg in self.angles:
            angle_rad = math.radians(angle_deg)
            points = self._compute_standard_clothoid(angle_rad)
            self.table[angle_deg] = points
    
    def _compute_standard_clothoid(self, delta_theta, num_points=20):
        """Compute normalized clothoid for given turn angle"""
        # Standard clothoid from (0,0,0) to (x,y,delta_theta)
        # This would use numerical integration or pyclothoids
        # Returns normalized points that can be scaled/rotated
        pass
    
    def get_clothoid(self, angle_deg):
        """Get closest pre-computed clothoid"""
        closest_angle = min(self.angles, key=lambda x: abs(x - angle_deg))
        return self.table[closest_angle]
```

---

#### **4. Complete Path Optimization Implementation**

```python
import math
import numpy as np
from pyclothoids import Clothoid

def optimize_path(path, map, min_distance=0.3):
    """
    Optimize path using clothoid curves at corners.
    
    Args:
        path: List of (x, y) waypoints from planner
        map: Maze map object
        min_distance: Minimum clearance from walls
        
    Returns:
        Optimized path as list of (x, y) points
    """
    if len(path) < 3:
        return path
    
    optimized_path = []
    i = 0
    
    while i < len(path):
        optimized_path.append(path[i])
        
        # Look ahead for corners (3-waypoint window)
        if i < len(path) - 2:
            p0 = path[i]
            p1 = path[i + 1]
            p2 = path[i + 2]
            
            # Calculate turn angle
            angle = calculate_turn_angle(p0, p1, p2)
            
            # If significant turn, apply clothoid smoothing
            if abs(angle) > math.radians(15):  # 15 degree threshold
                smoothed = smooth_corner_with_clothoid(p0, p1, p2, map, min_distance)
                
                if smoothed is not None:
                    # Add smoothed points (skip original corner)
                    optimized_path.extend(smoothed[1:-1])
                    i += 1  # Skip the corner waypoint
        
        i += 1
    
    # Ensure end point is included
    if path[-1] not in optimized_path:
        optimized_path.append(path[-1])
    
    return optimized_path


def calculate_turn_angle(p0, p1, p2):
    """Calculate signed turn angle at p1"""
    v1 = [p1[0] - p0[0], p1[1] - p0[1]]
    v2 = [p2[0] - p1[0], p2[1] - p1[1]]
    
    angle1 = math.atan2(v1[1], v1[0])
    angle2 = math.atan2(v2[1], v2[0])
    
    angle = normalize_angle(angle2 - angle1)
    return angle


def smooth_corner_with_clothoid(p0, p1, p2, map, min_clearance):
    """
    Smooth a corner using clothoid curve.
    
    Args:
        p0, p1, p2: Three consecutive waypoints
        map: Maze map
        min_clearance: Minimum wall distance
        
    Returns:
        Smoothed path segment or None if infeasible
    """
    # Calculate headings
    theta_in = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
    theta_out = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
    
    # Calculate approach and departure points (offset from corner)
    approach_dist = min(distance(p0, p1), 1.0) * 0.6
    depart_dist = min(distance(p1, p2), 1.0) * 0.6
    
    approach_point = [
        p1[0] - approach_dist * math.cos(theta_in),
        p1[1] - approach_dist * math.sin(theta_in)
    ]
    
    depart_point = [
        p1[0] + depart_dist * math.cos(theta_out),
        p1[1] + depart_dist * math.sin(theta_out)
    ]
    
    # Try to fit clothoid
    try:
        clothoid = Clothoid.StandardParams(
            approach_point[0], approach_point[1], theta_in,
            depart_point[0], depart_point[1], theta_out
        )
        
        # Sample points
        num_samples = max(10, int(clothoid.length / 0.2))
        s_values = np.linspace(0, clothoid.length, num_samples)
        
        smoothed_points = []
        for s in s_values:
            x, y = clothoid.XY(s)
            smoothed_points.append([x, y])
        
        # Safety check
        if not check_path_clearance(smoothed_points, map, min_clearance):
            return None  # Collision detected
        
        return smoothed_points
    
    except Exception as e:
        # Clothoid fitting failed (common for incompatible poses)
        return None


def check_path_clearance(points, map, min_clearance):
    """
    Verify all points maintain minimum clearance from walls.
    
    Args:
        points: List of (x, y) coordinates
        map: Maze map
        min_clearance: Minimum distance required
        
    Returns:
        True if safe, False if collision or too close
    """
    for point in points:
        x, y = int(round(point[0])), int(round(point[1]))
        
        # Check bounds
        if x < 0 or x >= map.row or y < 0 or y >= map.col:
            return False
        
        # Check if in wall
        if map.map[x][y].state == "#":
            return False
        
        # Check clearance to nearest wall (requires wall distance map)
        if hasattr(map, 'wall_distance'):
            if map.wall_distance[x, y] < min_clearance:
                return False
    
    return True


def distance(p1, p2):
    """Euclidean distance between two points"""
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


def normalize_angle(angle):
    """Normalize angle to [-π, π]"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle
```

---

#### **5. Integration with Existing Code**

**In `micromouse.py`:**

```python
from pathPlanning import AStar, RRT

# After path planning
if planner_type == "astar":
    planner = AStar(maze_map, config)
elif planner_type == "rrt":
    planner = RRT(maze_map, config)

raw_path = planner.plan()

# Apply clothoid optimization
optimized_path = optimize_path(raw_path, maze_map, min_distance=0.3)

# Pass to controller
controller.set_path(optimized_path)
```

**In `config.yaml`:**

```yaml
path_optimization:
  enabled: true
  method: "clothoid"  # or "circular_arc", "bezier"
  min_wall_clearance: 0.3
  turn_threshold: 15  # degrees - minimum angle to trigger smoothing
  approach_distance_factor: 0.6  # fraction of segment length
  
  clothoid:
    max_samples_per_meter: 5
    fallback_to_circular: true  # Use circular arcs if clothoid fails
```

---

#### **6. Velocity Profile from Curvature**

A key advantage of clothoids is smooth curvature for velocity planning:

```python
def compute_velocity_profile(path, max_speed=0.5, min_speed=0.1, max_accel=0.3):
    """
    Compute velocity at each point based on path curvature.
    
    Args:
        path: List of (x, y) points
        max_speed: Maximum velocity (m/s)
        min_speed: Minimum velocity (m/s)
        max_accel: Maximum lateral acceleration (m/s²)
        
    Returns:
        List of velocities for each point
    """
    velocities = []
    
    for i in range(len(path)):
        # Estimate local curvature (requires 3 points)
        if i == 0 or i == len(path) - 1:
            kappa = 0.0
        else:
            kappa = estimate_curvature(path[i-1], path[i], path[i+1])
        
        # Velocity limit from lateral acceleration: v² = a_max / κ
        if abs(kappa) > 1e-6:
            v_curve = math.sqrt(max_accel / abs(kappa))
            v = min(max_speed, max(min_speed, v_curve))
        else:
            v = max_speed
        
        velocities.append(v)
    
    return velocities


def estimate_curvature(p1, p2, p3):
    """
    Estimate curvature at p2 using three points.
    Formula: κ = 4A / (|p1-p2| * |p2-p3| * |p3-p1|)
    where A is the area of triangle p1-p2-p3
    """
    # Calculate triangle area using cross product
    v1 = [p2[0] - p1[0], p2[1] - p1[1]]
    v2 = [p3[0] - p2[0], p3[1] - p2[1]]
    area = abs(v1[0] * v2[1] - v1[1] * v2[0]) / 2
    
    # Calculate side lengths
    d12 = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
    d23 = math.sqrt((p3[0] - p2[0])**2 + (p3[1] - p2[1])**2)
    d31 = math.sqrt((p1[0] - p3[0])**2 + (p1[1] - p3[1])**2)
    
    if d12 * d23 * d31 < 1e-6:
        return 0.0
    
    kappa = 4 * area / (d12 * d23 * d31)
    return kappa
```

---

#### **7. Visualization and Comparison**

```python
def visualize_path_comparison(raw_path, optimized_path, maze_map):
    """
    Visualize original vs optimized path with curvature.
    """
    import matplotlib.pyplot as plt
    
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    
    # Plot 1: Path comparison
    grid = maze_map.get_grid_representation()
    axes[0].imshow(grid, cmap='gray')
    
    if len(raw_path) > 0:
        raw_arr = np.array(raw_path)
        axes[0].plot(raw_arr[:, 1], raw_arr[:, 0], 'r--', linewidth=2, label='Original')
    
    if len(optimized_path) > 0:
        opt_arr = np.array(optimized_path)
        axes[0].plot(opt_arr[:, 1], opt_arr[:, 0], 'b-', linewidth=2, label='Clothoid')
    
    axes[0].set_title('Path Comparison')
    axes[0].legend()
    axes[0].axis('equal')
    
    # Plot 2: Curvature profile
    if len(optimized_path) >= 3:
        curvatures = []
        for i in range(1, len(optimized_path) - 1):
            kappa = estimate_curvature(
                optimized_path[i-1],
                optimized_path[i],
                optimized_path[i+1]
            )
            curvatures.append(abs(kappa))
        
        axes[1].plot(curvatures, 'b-', linewidth=2)
        axes[1].set_xlabel('Waypoint Index')
        axes[1].set_ylabel('Curvature |κ| (1/m)')
        axes[1].set_title('Path Curvature Profile')
        axes[1].grid(True)
    
    plt.tight_layout()
    plt.show()
```

---

#### **8. Parameter Tuning Guide**

| Parameter | Typical Range | Effect |
|-----------|---------------|--------|
| `turn_threshold` | 10° - 30° | Lower = smooth more corners, higher = only sharp turns |
| `approach_distance_factor` | 0.4 - 0.8 | How far before corner to start curve |
| `min_clearance` | 0.2 - 0.5 | Safety margin (higher = more conservative) |
| `max_samples_per_meter` | 3 - 10 | Point density (higher = smoother but slower) |

**Tuning Process:**
1. Start with defaults: `turn_threshold=15`, `approach_factor=0.6`, `clearance=0.3`
2. Test on maze with mixed corner types (90°, 45°, S-curves)
3. If too conservative (many straight segments) → **lower turn threshold**
4. If hitting walls → **increase clearance** or **reduce approach factor**
5. If controller oscillates → **increase sample density**

---

#### **9. Fallback Strategy for Tight Spaces**

Not all corners have space for full clothoids. Implement graceful fallback:

```python
def smart_corner_smoothing(p0, p1, p2, map, min_clearance):
    """
    Try clothoid first, fall back to circular arc or original waypoint.
    """
    # Try clothoid
    clothoid_result = smooth_corner_with_clothoid(p0, p1, p2, map, min_clearance)
    if clothoid_result is not None:
        return clothoid_result, "clothoid"
    
    # Try circular arc
    arc_result = smooth_corner_with_arc(p0, p1, p2, map, min_clearance)
    if arc_result is not None:
        return arc_result, "circular_arc"
    
    # Keep original
    return [p1], "original"


def smooth_corner_with_arc(p0, p1, p2, map, min_clearance):
    """Simple circular arc as fallback (simpler than clothoid)"""
    # Calculate arc center and radius for tangent continuity
    # Sample points along arc
    # Check clearance
    # Return points or None
    pass
```

---

#### **10. Expected Performance**

With proper clothoid optimization, expect:

- **Curvature smoothness**: 70-90% reduction in peak curvature vs. raw A* path
- **Tracking error**: 30-50% improvement vs. unsmoothed path
- **Average speed**: 15-25% increase (due to smoother curves allowing higher speeds)
- **Computation time**: 10-50ms for typical maze (acceptable for pre-planning)

**Success Criteria for Report:**
1. All smoothed paths maintain `min_clearance` from walls
2. Curvature is continuous (no jumps) at junction points
3. Total path length increases by < 10% vs. original
4. Visual comparison shows clear smoothing at corners
5. Controller tracking error reduced measurably

---

#### **11. Troubleshooting Common Issues**

| Problem | Cause | Solution |
|---------|-------|----------|
| Clothoid goes through wall | Approach points too close to corner | Reduce `approach_distance_factor` |
| Path looks jagged | Too few sample points | Increase `max_samples_per_meter` |
| Fitting failures | Incompatible start/end poses | Check pose angles, add validation |
| Slow computation | Too many tight corners | Use lookup tables for common angles |
| Path too long | Over-smoothing | Increase `turn_threshold` |

---

#### **12. References**

- Bertolazzi, E., & Frego, M. (2015). "G¹ fitting with clothoids." *Mathematical Methods in the Applied Sciences*.

- Walton, D. J., & Meek, D. S. (2005). "A controlled clothoid spline." *Computers & Graphics*, 29(3), 353-363.

- Scheuer, A., & Fraichard, T. (1997). "Continuous-curvature path planning for car-like vehicles." *IEEE/RSJ International Conference on Intelligent Robots and Systems*.

- PyClothoids Documentation: https://github.com/philippeller/pyclothoids

---

**Key Takeaway:** Clothoids provide the optimal balance between path smoothness and computational tractability for micromouse navigation, naturally matching the kinematic constraints of differential drive robots while enabling higher-speed navigation through maze corridors.

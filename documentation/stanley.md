### **Stanley Controller for Path Tracking**

#### **1. Why Stanley Controller?**
- **Proven Performance**: Used by Stanford's winning entry in DARPA Grand Challenge 2005
- **Works with Waypoints**: Directly handles discrete path points (no reference trajectory needed)
- **Geometric Intuition**: Combines heading alignment + cross-track error correction
- **Velocity Adaptive**: Performance automatically adjusts with speed
- **Simple Implementation**: More sophisticated than Pure Pursuit, simpler than LQR/MPC

---

#### **2. How Stanley Works**

The Stanley controller computes a steering angle `δ` that drives the vehicle toward the path using two components:

```math
\delta(t) = \psi(t) + \arctan\left(\frac{k \cdot e(t)}{v(t) + k_{soft}}\right)
```

Where:
- **ψ(t)**: Heading error (angle between vehicle heading and path tangent)
- **e(t)**: Cross-track error (lateral distance from path)
- **v(t)**: Current velocity
- **k**: Gain parameter (typical: 0.5 - 2.5)
- **k_soft**: Softening constant to prevent division by zero at low speeds (typical: 0.5 - 1.0)

**Physical Interpretation:**
1. **First term (ψ)**: Points the vehicle along the path direction
2. **Second term (arctan)**: Points the vehicle toward the path
   - Large cross-track error → steer more aggressively
   - High velocity → reduce steering sensitivity (stability)
   - Low velocity → increase steering gain (responsiveness)

---

#### **3. Adaptation for Differential Drive Robots**

**Important**: Stanley was originally designed for **Ackermann steering** (car-like) vehicles and outputs a **steering angle δ**. For **differential drive** robots (like micromouse), we need to convert this to **left/right wheel velocities**.

**Conversion from Steering Angle to Wheel Velocities:**

```python
def stanley_to_differential_drive(steering_angle, forward_velocity, wheel_base):
    """
    Convert Stanley steering angle to differential drive wheel velocities.
    
    Args:
        steering_angle: Desired steering angle δ (radians)
        forward_velocity: Desired forward speed v (m/s)
        wheel_base: Distance between left and right wheels L (meters)
        
    Returns:
        (v_left, v_right): Left and right wheel velocities
    """
    # Convert steering angle to angular velocity
    # For small angles: ω ≈ v * δ / L
    # More accurate: ω = v * tan(δ) / L
    angular_velocity = forward_velocity * math.tan(steering_angle) / wheel_base
    
    # Convert to wheel velocities
    # v = (v_left + v_right) / 2
    # ω = (v_right - v_left) / wheel_base
    v_left = forward_velocity - (angular_velocity * wheel_base / 2)
    v_right = forward_velocity + (angular_velocity * wheel_base / 2)
    
    return v_left, v_right
```

**Alternative: Direct Angular Velocity Control**

For pure differential drive (no virtual steering), you can also compute angular velocity directly:

```python
def stanley_to_angular_velocity(steering_angle, forward_velocity, wheel_base):
    """
    Compute angular velocity from Stanley output.
    
    Returns:
        (v, ω): Linear and angular velocities for diff drive
    """
    omega = forward_velocity * math.tan(steering_angle) / wheel_base
    return forward_velocity, omega
```

**Micromouse Typical Parameters:**
- **Wheel base (L)**: 0.05 - 0.1 m (5-10 cm)
- **Max wheel velocity**: 0.3 - 1.0 m/s
- **Wheel radius**: 0.01 - 0.02 m

---

#### **4. Implementation Steps**

**Step 1: Find Nearest Path Point**
Given current position `(x, y)`, find the closest point on the path:

```python
def find_nearest_point_on_path(self, current_pos, path):
    """Find nearest point on path and its index"""
    min_dist = float('inf')
    nearest_idx = 0
    
    for i, waypoint in enumerate(path):
        dist = math.sqrt((waypoint[0] - current_pos[0])**2 + 
                        (waypoint[1] - current_pos[1])**2)
        if dist < min_dist:
            min_dist = dist
            nearest_idx = i
    
    return nearest_idx, path[nearest_idx]
```

**Step 2: Compute Cross-Track Error**
Calculate signed distance from vehicle to path:

```python
def compute_cross_track_error(self, current_pos, nearest_point, path_heading):
    """Compute perpendicular distance to path (signed)"""
    # Vector from nearest path point to vehicle
    dx = current_pos[0] - nearest_point[0]
    dy = current_pos[1] - nearest_point[1]
    
    # Cross-track error is perpendicular component
    # Positive if vehicle is to the left of path direction
    cross_track_error = -dx * math.sin(path_heading) + dy * math.cos(path_heading)
    
    return cross_track_error
```

**Step 3: Compute Path Heading**
Determine the path's desired heading at the nearest point:

```python
def compute_path_heading(self, path, nearest_idx):
    """Calculate path tangent direction"""
    if nearest_idx < len(path) - 1:
        # Use vector to next waypoint
        dx = path[nearest_idx + 1][0] - path[nearest_idx][0]
        dy = path[nearest_idx + 1][1] - path[nearest_idx][1]
    else:
        # At end of path, use vector from previous point
        dx = path[nearest_idx][0] - path[nearest_idx - 1][0]
        dy = path[nearest_idx][1] - path[nearest_idx - 1][1]
    
    path_heading = math.atan2(dy, dx)
    return path_heading
```

**Step 4: Compute Heading Error**
Calculate angle difference (handle wraparound):

```python
def normalize_angle(angle):
    """Normalize angle to [-π, π]"""
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def compute_heading_error(self, current_heading, path_heading):
    """Calculate heading error with proper wraparound"""
    heading_error = normalize_angle(path_heading - current_heading)
    return heading_error
```

**Step 5: Compute Stanley Control**
Combine both error terms:

```python
def stanley_control(self, cross_track_error, heading_error, velocity):
    """Compute steering angle using Stanley law"""
    # Steering angle = heading error + cross-track correction
    cross_track_term = math.atan2(self.k * cross_track_error, 
                                   velocity + self.k_soft)
    
    steering_angle = heading_error + cross_track_term
    
    # Clamp to maximum steering angle
    steering_angle = np.clip(steering_angle, -self.max_steer, self.max_steer)
    
    return steering_angle
```

---

#### **4. Complete Python Implementation**

```python
class Stanley:
    """
    Stanley Controller for path tracking.
    
    Based on the controller used by Stanford Racing Team in DARPA Grand Challenge 2005.
    """
    def __init__(self, maps, config=None):
        self.map = maps
        
        # Load configuration with defaults
        self.config = config.get('stanley', {}) if config else {}
        self.k = self.config.get('k', 1.0)  # Cross-track error gain
        self.k_soft = self.config.get('k_soft', 1.0)  # Softening constant
        self.max_steer = self.config.get('max_steer', np.pi / 3)  # Max steering angle
        self.max_speed = self.config.get('max_speed', 0.5)
        self.min_speed = self.config.get('min_speed', 0.2)
        
        self.path = []
        self.nearest_idx = 0
        
    def set_path(self, path):
        """Set the path to follow"""
        self.path = path
        self.nearest_idx = 0
    
    def compute_control(self, current_pos, current_heading, current_velocity):
        """
        Compute steering angle and velocity for path tracking.
        
        Args:
            current_pos: (x, y) position
            current_heading: Current heading angle in radians
            current_velocity: Current forward velocity
            
        Returns:
            (steering_angle, target_velocity)
        """
        if len(self.path) < 2:
            return 0.0, 0.0
        
        # Step 1: Find nearest point on path
        self.nearest_idx, nearest_point = self.find_nearest_point_on_path(
            current_pos, self.path)
        
        # Step 2: Compute path heading at nearest point
        path_heading = self.compute_path_heading(self.path, self.nearest_idx)
        
        # Step 3: Compute heading error
        heading_error = self.normalize_angle(path_heading - current_heading)
        
        # Step 4: Compute cross-track error
        cross_track_error = self.compute_cross_track_error(
            current_pos, nearest_point, path_heading)
        
        # Step 5: Compute Stanley steering control
        steering_angle = self.stanley_control(
            cross_track_error, heading_error, current_velocity)
        
        # Step 6: Compute target velocity (can be constant or adaptive)
        target_velocity = self.compute_target_velocity(steering_angle)
        
        return steering_angle, target_velocity
    
    def compute_target_velocity(self, steering_angle):
        """Reduce speed for large steering angles"""
        # Linear velocity reduction based on steering angle
        speed_factor = 1.0 - abs(steering_angle) / self.max_steer
        target_velocity = self.min_speed + speed_factor * (self.max_speed - self.min_speed)
        return target_velocity
    
    def find_nearest_point_on_path(self, current_pos, path):
        """Find nearest point on path, searching forward from last known position"""
        # Search window: look ahead and behind current index
        search_start = max(0, self.nearest_idx - 5)
        search_end = min(len(path), self.nearest_idx + 20)
        
        min_dist = float('inf')
        best_idx = self.nearest_idx
        
        for i in range(search_start, search_end):
            dist = math.sqrt((path[i][0] - current_pos[0])**2 + 
                           (path[i][1] - current_pos[1])**2)
            if dist < min_dist:
                min_dist = dist
                best_idx = i
        
        return best_idx, path[best_idx]
    
    def compute_path_heading(self, path, idx):
        """Calculate path tangent direction at given index"""
        if idx < len(path) - 1:
            dx = path[idx + 1][0] - path[idx][0]
            dy = path[idx + 1][1] - path[idx][1]
        else:
            dx = path[idx][0] - path[idx - 1][0]
            dy = path[idx][1] - path[idx - 1][1]
        
        return math.atan2(dy, dx)
    
    def compute_cross_track_error(self, current_pos, nearest_point, path_heading):
        """Compute signed perpendicular distance to path"""
        dx = current_pos[0] - nearest_point[0]
        dy = current_pos[1] - nearest_point[1]
        
        # Project onto perpendicular to path
        cross_track_error = -dx * math.sin(path_heading) + dy * math.cos(path_heading)
        return cross_track_error
    
    def stanley_control(self, cross_track_error, heading_error, velocity):
        """Compute Stanley steering law"""
        # Avoid numerical issues at zero velocity
        velocity = max(velocity, 0.1)
        
        # Stanley control law
        cross_track_term = math.atan2(self.k * cross_track_error, velocity + self.k_soft)
        steering_angle = heading_error + cross_track_term
        
        # Saturate to max steering angle
        steering_angle = np.clip(steering_angle, -self.max_steer, self.max_steer)
        
        return steering_angle
    
    @staticmethod
    def normalize_angle(angle):
        """Normalize angle to [-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def to_differential_drive(self, steering_angle, forward_velocity, wheel_base=0.08):
        """
        Convert Stanley steering output to differential drive wheel velocities.
        
        Args:
            steering_angle: Steering angle from Stanley controller (radians)
            forward_velocity: Desired forward velocity (m/s)
            wheel_base: Distance between wheels (meters), typical: 0.05-0.10m
            
        Returns:
            (v_left, v_right): Left and right wheel velocities (m/s)
        """
        # Convert steering to angular velocity
        angular_velocity = forward_velocity * math.tan(steering_angle) / wheel_base
        
        # Compute individual wheel velocities
        v_left = forward_velocity - (angular_velocity * wheel_base / 2)
        v_right = forward_velocity + (angular_velocity * wheel_base / 2)
        
        return v_left, v_right
    
    def compute_control_diff_drive(self, current_pos, current_heading, current_velocity, wheel_base=0.08):
        """
        Compute differential drive wheel velocities directly.
        
        This is a convenience method that combines Stanley control with diff drive conversion.
        
        Args:
            current_pos: (x, y) position
            current_heading: Current heading angle (radians)
            current_velocity: Current forward velocity (m/s)
            wheel_base: Wheel separation distance (meters)
            
        Returns:
            (v_left, v_right): Left and right wheel velocities
        """
        # Get Stanley steering angle
        steering_angle, target_velocity = self.compute_control(
            current_pos, current_heading, current_velocity)
        
        # Convert to differential drive
        v_left, v_right = self.to_differential_drive(
            steering_angle, target_velocity, wheel_base)
        
        return v_left, v_right
```

---

#### **5. Integration with Existing Code**

**In `pathTracking.py`:**
Add the Stanley class alongside the existing PurePursuit class.

**In `micromouse.py`:**
```python
from pathTracking import PurePursuit, Stanley

# In main simulation loop:
if tracking_method == "pure_pursuit":
    controller = PurePursuit(maze_map, config)
elif tracking_method == "stanley":
    controller = Stanley(maze_map, config)

controller.set_path(planned_path)
```

**In `config.yaml`:**
```yaml
stanley:
  k: 1.0              # Cross-track error gain
  k_soft: 1.0         # Softening constant
  max_steer: 1.047    # Max steering (60 degrees)
  max_speed: 0.5
  min_speed: 0.2
  wheel_base: 0.08    # Distance between wheels (meters) for diff drive
  debug: true
  visualize_every: 5
  
# Micromouse robot parameters (for differential drive)
robot:
  wheel_base: 0.08    # Distance between left and right wheels (m)
  wheel_radius: 0.016 # Wheel radius (m)
  max_wheel_speed: 10.0  # Maximum wheel angular velocity (rad/s)
```

**Example usage for differential drive:**
```python
# Create Stanley controller
stanley = Stanley(maze_map, config)
stanley.set_path(optimized_path)

# In control loop
current_pos = (robot.x, robot.y)
current_heading = robot.theta
current_velocity = robot.v

# Option 1: Get wheel velocities directly
v_left, v_right = stanley.compute_control_diff_drive(
    current_pos, current_heading, current_velocity, 
    wheel_base=0.08
)

# Option 2: Get steering angle, then convert
steering_angle, target_velocity = stanley.compute_control(
    current_pos, current_heading, current_velocity
)
v_left, v_right = stanley.to_differential_drive(
    steering_angle, target_velocity, wheel_base=0.08
)

# Apply to motors
robot.set_wheel_velocities(v_left, v_right)
```

---

#### **6. Parameter Tuning Guide**

| Parameter    | Typical Range | Effect |
|--------------|---------------|--------|
| `k`          | 0.5 - 2.5     | Higher = more aggressive cross-track correction (risk: oscillation) |
| `k_soft`     | 0.5 - 1.5     | Higher = smoother behavior at low speeds |
| `max_steer`  | π/6 - π/3     | Physical constraint of steering mechanism |

**Tuning Strategy:**
1. Start with `k = 1.0`, `k_soft = 1.0`
2. If vehicle oscillates around path → **reduce k**
3. If vehicle doesn't return to path quickly → **increase k**
4. If unstable at low speeds → **increase k_soft**
5. If overshooting corners → **reduce max_speed** or add velocity adaptation

**Additional Considerations for Differential Drive:**
- **Wheel base matters**: Smaller wheel base (typical for micromouse: 5-8cm) → more responsive turning but potentially more oscillation
- **Maximum wheel speed limits**: Ensure `v_left` and `v_right` don't exceed motor limits
- **Slippage**: Differential drive is more prone to wheel slip during sharp turns on smooth surfaces
- **Asymmetric loading**: If robot has asymmetric mass distribution, you may need to add compensation factors

---

#### **7. Advantages Over Pure Pursuit**

| Feature | Pure Pursuit | Stanley |
|---------|--------------|---------|
| Lookahead distance | Must be tuned carefully | Not needed (uses nearest point) |
| Low-speed behavior | Can be unstable | Naturally stable (velocity-adaptive gain) |
| Sharp corners | May cut corners | Better adherence to path |
| Cross-track error | Indirect correction | Direct proportional correction |
| Implementation | Slightly simpler | Slightly more complex |

**When to Use Stanley:**
- ✅ Tighter path following required
- ✅ Variable speed operation
- ✅ Complex paths with sharp turns
- ✅ Need consistent performance across speeds

**When Pure Pursuit is Better:**
- Smooth, gentle paths
- Constant high speed
- Computational constraints
- Simplicity preferred

---

#### **8. Expected Results**

With proper tuning, Stanley controller should:
- **Cross-track error**: < 0.5 cell widths during straight segments
- **Corner tracking**: Stays within 0.3 units of optimal path
- **Stability**: No oscillations at any speed
- **Convergence**: Returns to path within 2-3 cells after disturbance

**Comparison Metrics for Report:**
1. **Average cross-track error** (lower is better)
2. **Maximum cross-track error** (stability metric)
3. **Path completion time** (efficiency)
4. **Control smoothness** (standard deviation of steering commands)
5. **Success rate** on different maze configurations

---

#### **9. Common Issues and Solutions**

| Problem | Likely Cause | Solution |
|---------|-------------|----------|
| Oscillation at high speed | `k` too high | Reduce `k` to 0.5-1.0 |
| Not returning to path | `k` too low | Increase `k` to 1.5-2.0 |
| Unstable at low speed | `k_soft` too low | Increase `k_soft` |
| Cutting corners | Looking at wrong path point | Implement forward search window |
| Jerky steering | Nearest point jumping | Smooth path or use search window |

---

#### **10. Theoretical Notes: Stanley for Differential Drive**

**Is this theoretically sound?**

Yes! The adaptation is valid because:

1. **Kinematic Equivalence**: At the velocity level, both Ackermann and differential drive robots are nonholonomic systems that can be described by:
   - Forward velocity: `v`
   - Angular velocity: `ω`
   
2. **The Conversion**: The relationship `ω = v·tan(δ)/L` comes from the instantaneous center of rotation (ICR) and is geometrically valid for mapping virtual steering to actual turning rate.

3. **Small Angle Approximation**: For small steering angles (typical in path following), `tan(δ) ≈ δ`, simplifying to `ω ≈ v·δ/L`.

**Limitations to be aware of:**

| Aspect | Ackermann Vehicle | Differential Drive | Impact |
|--------|-------------------|-------------------|---------|
| **Zero-radius turns** | Impossible | Possible (rotate in place) | Stanley doesn't exploit this capability |
| **Kinematic constraints** | Limited by steering angle | Limited by wheel speeds | Need to check wheel velocity limits |
| **Slip characteristics** | Front wheels slip | Both wheels can slip | Diff drive more sensitive to surface |
| **Maneuverability** | Larger turning radius | Can be more agile | Stanley may be conservative for diff drive |

**When Stanley is optimal for diff drive:**
- ✅ Smooth paths with moderate curvature (like optimized micromouse paths)
- ✅ Forward-dominant motion (minimal backward driving)
- ✅ When you want proven, stable behavior

**When other controllers might be better:**
- ❌ Need to exploit zero-radius turns (use Pure Pursuit with backward motion)
- ❌ Highly dynamic environments requiring rapid direction changes (use MPC)
- ❌ Very tight corridors requiring precise point-turn maneuvers

**For micromouse specifically**: Stanley is **excellent** because:
- Maze paths are typically smooth after optimization
- High speed is important (Stanley's velocity adaptation helps)
- Stability is critical (don't want to hit walls)
- The cross-track error correction provides better wall clearance maintenance

---

#### **11. References**

- Thrun, S., et al. (2006). "Stanley: The robot that won the DARPA Grand Challenge." *Journal of Field Robotics*, 23(9), 661-692.
- Snider, J. M. (2009). "Automatic Steering Methods for Autonomous Automobile Path Tracking." CMU-RI-TR-09-08.
- Hoffmann, G. M., et al. (2007). "Autonomous automobile trajectory tracking for off-road driving: Controller design, experimental validation and racing." *American Control Conference*.

---

**Example Visualization:**

```
         Path
          |
    ψ --> * <-- Vehicle heading
         /|
        / |
       /  | e (cross-track error)
      /   |
     •----+
   Vehicle
   
   δ = ψ + arctan(k·e / v)
```

The Stanley controller continuously adjusts to minimize both heading error (ψ) and cross-track error (e), making it robust and intuitive.

"""
Pygame-based Micromouse Simulation
-----------------------------------
Interactive real-time visualization of micromouse maze navigation.

Controls:
    SPACE   - Pause / Resume  (resets after collision or goal)
    R       - Full reset (rebuild maze, re-plan, restart)
    UP/DOWN - Increase / Decrease simulation speed
    RIGHT   - Step one tick (when paused)
    H       - Toggle wall-cost heatmap overlay
    D       - Toggle wall-distance field overlay
    G       - Toggle grid-lines overlay
    Q / ESC - Quit

Usage:
    python simulation.py [config.yaml]              # Interactive mode
    python simulation.py --benchmark [config.yaml]  # Headless benchmark mode
"""

import sys
import math
import numpy as np
import yaml
import pygame

from micromouse import Map, load_maze, upsample_maze
from pathPlanning import AStar
from pathTracking import PurePursuit
from diff_drive_robot import DifferentialDriveRobot, DiffDriveController

# ── Colour palette ────────────────────────────────────────────────────────── #

C_BG          = (20, 20, 30)
C_WALL        = (35, 35, 48)
C_FREE        = (230, 230, 235)
C_START       = (50, 205, 50)
C_GOAL        = (220, 60, 60)
C_PATH        = (255, 215, 0)       # planned path
C_TRAJ        = (0, 200, 210)       # actual trajectory
C_ROBOT       = (255, 140, 0)       # robot body (running)
C_ROBOT_GOAL  = (50, 220, 50)       # robot body (goal reached)
C_ROBOT_COLL  = (220, 30, 30)       # robot body (collision)
C_ROBOT_SPIN  = (255, 50, 255)      # robot body (spinning out!)
C_SPIN_GLOW   = (255, 0, 100)       # spinout effect glow
C_HEADING     = (255, 255, 255)     # heading indicator
C_LOOKAHEAD   = (200, 50, 200)      # lookahead target point

C_HUD_BG      = (28, 28, 38)
C_HUD_BORDER  = (70, 70, 90)
C_HUD_TITLE   = (0, 200, 210)
C_HUD_LABEL   = (140, 140, 160)
C_HUD_VALUE   = (230, 230, 240)
C_HUD_KEY     = (255, 215, 0)
C_WHITE       = (255, 255, 255)


# ── Simulation class ─────────────────────────────────────────────────────── #

class PygameSimulation:
    """Interactive pygame-based micromouse simulation."""

    def __init__(self, config_file="config.yaml"):
        # ── config ──
        with open(config_file) as f:
            self.config = yaml.safe_load(f)

        # ── maze (keep raw array for resets) ──
        maze_file = self.config.get("maze_file", "maze.csv")
        self.maze_array = load_maze(maze_file)
        
        # Apply upsampling if enabled
        if self.config.get('maze_upsampling', {}).get('enabled', False):
            factor = self.config.get('maze_upsampling', {}).get('factor', 1)
            self.maze_array = upsample_maze(self.maze_array, factor)
        
        self.maze = Map(self.maze_array)
        
        # ── Convert metric speeds to grid units ──
        self._convert_metric_speeds()

        # ── window geometry ──
        sim_cfg = self.config.get("simulation", {})
        self.window_size = sim_cfg.get("window_size", 900)
        self.hud_width = 260
        self.target_fps = sim_cfg.get("fps", 60)

        maze_dim = max(self.maze.row, self.maze.col)
        self.cell_size = self.window_size / maze_dim
        self.maze_w = int(self.maze.col * self.cell_size)
        self.maze_h = int(self.maze.row * self.cell_size)

        # ── pygame init ──
        pygame.init()
        self.screen = pygame.display.set_mode(
            (self.maze_w + self.hud_width, self.maze_h)
        )
        pygame.display.set_caption("Micromouse Simulation")
        self.clock = pygame.time.Clock()

        self.font    = pygame.font.Font(None, 22)
        self.font_lg = pygame.font.Font(None, 26)
        self.font_sm = pygame.font.Font(None, 18)

        # ── pre-render static maze surface (before A* modifies states) ──
        self.maze_surface = self._render_maze_surface()

        # ── path planning ──
        self.path = []
        self.planned_distance = 0.0
        self._plan_path()

        # ── visualisation overlays (pre-rendered after planning) ──
        self.show_heatmap  = False    # H  – wall cost heatmap
        self.show_distance = False    # D  – wall distance field
        self.show_grid     = False    # G  – grid lines
        self._build_overlay_surfaces()

        # ── robot state ──
        self.sim_speed = 1.0
        self.paused = True
        self.running = True
        
        # ── differential drive robot ──
        self.robot = DifferentialDriveRobot(self.config)
        self.controller = DiffDriveController(self.robot.wheel_radius, self.robot.wheelbase)
        
        self._reset_robot()

    def _convert_metric_speeds(self):
        """Convert speeds from m/s to grid units/s if use_metric_speeds is enabled."""
        phys_dims = self.config.get('physical_dimensions', {})
        use_metric_speeds = phys_dims.get('use_metric_speeds', False)
        
        # Store conversion factor for later use
        self.meters_per_cell = 1.0
        
        if not use_metric_speeds:
            return
        
        maze_width_meters = phys_dims.get('maze_width_meters', 2.88)
        maze_height_meters = phys_dims.get('maze_height_meters', 2.88)
        
        # Calculate meters per grid cell
        num_cols = self.maze.col
        num_rows = self.maze.row
        meters_per_cell_x = maze_width_meters / num_cols
        meters_per_cell_y = maze_height_meters / num_rows
        self.meters_per_cell = (meters_per_cell_x + meters_per_cell_y) / 2.0
        
        # Convert Pure Pursuit speeds
        if 'pure_pursuit' in self.config:
            pp = self.config['pure_pursuit']
            if 'max_speed' in pp:
                pp['max_speed'] = pp['max_speed'] / self.meters_per_cell
            if 'min_speed' in pp:
                pp['min_speed'] = pp['min_speed'] / self.meters_per_cell
        
        # Convert Micromouse speed threshold
        if 'micromouse' in self.config:
            mm = self.config['micromouse']
            if 'min_speed_threshold' in mm:
                mm['min_speed_threshold'] = mm['min_speed_threshold'] / self.meters_per_cell

    # ------------------------------------------------------------------ #
    #  Maze rendering                                                     #
    # ------------------------------------------------------------------ #

    def _render_maze_surface(self):
        """Pre-render the maze grid into a pygame Surface using numpy."""
        grid = np.asarray(self.maze.get_grid_representation())
        h, w = grid.shape

        rgb = np.zeros((h, w, 3), dtype=np.uint8)
        rgb[grid == 0] = C_FREE
        rgb[grid == 1] = C_WALL
        rgb[grid == 2] = C_FREE        # path marker → show as free
        rgb[grid == 3] = C_START
        rgb[grid == 4] = C_GOAL

        # pygame.surfarray expects shape (width, height, 3) = (cols, rows, 3)
        arr = np.ascontiguousarray(np.transpose(rgb, (1, 0, 2)))
        surface = pygame.surfarray.make_surface(arr)
        return pygame.transform.scale(surface, (self.maze_w, self.maze_h))

    # ------------------------------------------------------------------ #
    #  Visualisation overlay surfaces                                     #
    # ------------------------------------------------------------------ #

    def _build_overlay_surfaces(self):
        """Pre-render the toggleable overlay surfaces.

        Called once after path planning so that the wall_distance map
        (computed by AStar when wall_cost is enabled) is available.
        """
        self._heatmap_surface  = self._make_heatmap_surface()
        self._distance_surface = self._make_distance_surface()
        self._grid_surface     = self._make_grid_surface()

    # ---- wall-cost heatmap (red = high cost, transparent = zero cost) ----

    def _make_heatmap_surface(self):
        """Render the wall-proximity *cost* as a red-hot overlay.

        Uses the same cost function parameters from config so the
        visualisation matches what A* actually sees.
        """
        if not hasattr(self.maze, "wall_distance"):
            return None

        wc  = self.config.get("wall_cost", {})
        if not wc.get("enabled", False):
            return None

        weight    = wc.get("weight", 2.0)
        decay     = wc.get("decay", "exponential")
        rate      = wc.get("decay_rate", 0.5)
        threshold = wc.get("threshold", 5.0)
        wd = self.maze.wall_distance

        h, w = self.maze.row, self.maze.col
        cost = np.zeros((h, w), dtype=np.float64)
        for i in range(h):
            for j in range(w):
                d = wd[i, j]
                if d >= threshold:
                    continue
                if decay == "exponential":
                    cost[i, j] = weight * math.exp(-rate * d)
                elif decay == "inverse":
                    cost[i, j] = weight / (d + 0.1)
                elif decay == "linear":
                    cost[i, j] = weight * (1.0 - d / threshold)

        # Normalise to 0-255 for colour mapping
        cmax = cost.max() if cost.max() > 0 else 1.0
        norm = (cost / cmax * 255).astype(np.uint8)

        # RGBA: red channel = intensity, alpha = intensity
        rgba = np.zeros((h, w, 4), dtype=np.uint8)
        rgba[:, :, 0] = norm                       # red
        rgba[:, :, 1] = (norm * 0.15).astype(np.uint8)  # slight orange tint
        rgba[:, :, 3] = (norm * 0.65).astype(np.uint8)  # alpha

        # Walls fully transparent so they still look dark
        grid = np.asarray(self.maze.get_grid_representation())
        rgba[grid == 1] = 0

        surf = pygame.Surface((w, h), pygame.SRCALPHA)
        # Manually set pixels via surfarray (RGBA)
        pygame.surfarray.pixels_alpha(surf)[:] = np.ascontiguousarray(rgba[:, :, 3].T)
        rgb_arr = np.ascontiguousarray(np.transpose(rgba[:, :, :3], (1, 0, 2)))
        pygame.surfarray.pixels3d(surf)[:] = rgb_arr

        return pygame.transform.scale(surf, (self.maze_w, self.maze_h))

    # ---- wall-distance field (blue = far, cyan = close) ----

    def _make_distance_surface(self):
        """Render wall distance as a cool-toned semi-transparent overlay."""
        if not hasattr(self.maze, "wall_distance"):
            # Compute on demand so the overlay works even without wall_cost
            self.maze.compute_wall_distance_map()

        wd = self.maze.wall_distance
        h, w = self.maze.row, self.maze.col

        dmax = wd[wd < np.inf].max() if np.any(wd < np.inf) else 1.0
        norm = np.clip(wd / dmax, 0, 1)  # 0 = wall, 1 = far from wall

        rgba = np.zeros((h, w, 4), dtype=np.uint8)
        # Colour ramp: near-wall = bright cyan, far = dark blue, walls = transparent
        rgba[:, :, 0] = (40  * (1 - norm)).astype(np.uint8)       # R
        rgba[:, :, 1] = (220 * (1 - norm) + 60 * norm).astype(np.uint8)  # G
        rgba[:, :, 2] = (240 * (1 - norm) + 100 * norm).astype(np.uint8) # B
        rgba[:, :, 3] = (150 * (1 - norm)).astype(np.uint8)       # A (fades out far from wall)

        # Walls transparent
        grid = np.asarray(self.maze.get_grid_representation())
        rgba[grid == 1] = 0

        surf = pygame.Surface((w, h), pygame.SRCALPHA)
        pygame.surfarray.pixels_alpha(surf)[:] = np.ascontiguousarray(rgba[:, :, 3].T)
        pygame.surfarray.pixels3d(surf)[:] = np.ascontiguousarray(
            np.transpose(rgba[:, :, :3], (1, 0, 2))
        )
        return pygame.transform.scale(surf, (self.maze_w, self.maze_h))

    # ---- grid lines ----

    def _make_grid_surface(self):
        """Thin grid lines at cell boundaries (useful for small mazes)."""
        surf = pygame.Surface((self.maze_w, self.maze_h), pygame.SRCALPHA)
        line_colour = (255, 255, 255, 30)  # very faint white
        cs = self.cell_size
        if cs < 3:
            return surf  # too dense to be useful
        for i in range(self.maze.row + 1):
            y = int(i * cs)
            pygame.draw.line(surf, line_colour, (0, y), (self.maze_w, y))
        for j in range(self.maze.col + 1):
            x = int(j * cs)
            pygame.draw.line(surf, line_colour, (x, 0), (x, self.maze_h))
        return surf

    # ------------------------------------------------------------------ #
    #  Path planning                                                      #
    # ------------------------------------------------------------------ #

    def _plan_path(self):
        """Run A* (with optional matplotlib debug) and store the result."""
        cfg = {**self.config}
        # Do NOT override astar debug here; respect config.yaml
        start = self.maze.map[self.maze.start[0]][self.maze.start[1]]
        goal  = self.maze.map[self.maze.goal[0]][self.maze.goal[1]]

        astar = AStar(self.maze, cfg)
        px, py = astar.plan_path(start, goal)
        self.path = list(zip(px, py)) if px and py else []
        self.planned_distance = astar.calculate_path_distance(px, py)

    # ------------------------------------------------------------------ #
    #  Robot reset                                                        #
    # ------------------------------------------------------------------ #

    def _reset_robot(self):
        """Reset robot to start; reuse the already-planned path."""
        sp = self.maze.start
        mc = self.config.get("micromouse", {})

        self.pos       = list(sp)            # [row, col]
        self.heading   = mc.get("initial_heading", math.pi / 4)
        self.speed     = 0.0
        self.steering  = 0.0
        self.trajectory = [list(sp)]
        self.traversed = 0.0
        self.sim_time  = 0.0
        self.collision    = False
        self.goal_reached = False
        self.step_acc  = 0.0

        if not self.path:
            self.status = "NO PATH FOUND"
            self.paused = True
            return

        # Pure Pursuit controller (no matplotlib debug)
        cfg = {**self.config}
        cfg["pure_pursuit"] = {**cfg.get("pure_pursuit", {}), "debug": False}
        self.pp = PurePursuit(self.maze, cfg)
        self.pp.set_path(self.path)

        # Reset robot's wheel velocities (position is tracked separately)
        self.robot.stop()
        self.status = "PAUSED  --  press SPACE"
        self.paused = True

    # ------------------------------------------------------------------ #
    #  Simulation step                                                    #
    # ------------------------------------------------------------------ #

    def _step(self):
        """Advance simulation by one dt."""
        if self.collision or self.goal_reached or not self.path:
            return

        dt = self.config.get("micromouse", {}).get("dt", 0.1)
        prev = list(self.pos)

        # ── controller ──
        spd, steer = self.pp.get_control(self.pos, self.heading)
        self.speed    = spd
        self.steering = steer

        # ── differential drive kinematics with spinout ──
        # Convert from grid units to physical units (meters)
        spd_meters = spd * self.meters_per_cell
        
        # Convert steering command to angular velocity
        # Pure pursuit outputs steering for implicit wheelbase=1.0 in grid space
        # So we convert using the grid-space wheelbase equivalent
        wheelbase_grid = 1.0 * self.meters_per_cell  # Convert implicit wheelbase to meters
        omega = self.controller.steering_to_omega(spd_meters, steer, wheelbase_equiv=wheelbase_grid)
        
        # Convert to wheel velocities (in rad/s)
        v_left, v_right = self.controller.velocity_to_wheels(spd_meters, omega)
        
        # Apply to robot with acceleration limits
        self.robot.set_wheel_velocities(v_left, v_right, dt)
        
        # Update kinematics to get actual velocities (includes slip and spinout)
        actual_v_meters, actual_omega = self.robot.update_kinematics(dt)
        
        # Convert actual velocity back to grid units
        actual_v = actual_v_meters / self.meters_per_cell
        
        # Integrate position manually in maze coordinate system
        # Maze uses (row, col) where row is down and col is right
        self.pos[0] += actual_v * math.cos(self.heading) * dt  # row component
        self.pos[1] += actual_v * math.sin(self.heading) * dt  # col component
        self.heading += actual_omega * dt
        self.heading = (self.heading + math.pi) % (2 * math.pi) - math.pi
        
        self.sim_time += dt
        self.trajectory.append(list(self.pos))

        # ── traversed distance ──
        dx = self.pos[0] - prev[0]
        dy = self.pos[1] - prev[1]
        self.traversed += math.hypot(dx, dy)

        # ── collision ──
        r, c = self.pos
        if not (0 <= r < self.maze.row and 0 <= c < self.maze.col):
            self.collision = True
            self.status = "COLLISION  (out of bounds)"
            return
        
        # Check collision using robot's actual physical dimensions
        # Robot body dimensions in grid units
        wheelbase_meters = self.robot.wheelbase  # 0.10m
        body_width_grid = wheelbase_meters / self.meters_per_cell
        body_length_grid = body_width_grid * 1.1  # Slightly longer than wide
        
        # Use circular collision approximation (conservative)
        collision_radius = max(body_width_grid, body_length_grid) / 2.0
        
        # Check points around the robot's perimeter
        if self._check_collision_at_radius(r, c, collision_radius):
            self.collision = True
            self.status = f"COLLISION  ({r:.1f}, {c:.1f})"
            return

        # ── goal check ──
        gr, gc = self.path[-1]
        goal_thr = self.config.get("micromouse", {}).get("goal_threshold", 0.5)
        if math.hypot(r - gr, c - gc) < goal_thr:
            self.goal_reached = True
            self.status = "GOAL REACHED!  Press SPACE/R"
            
            # Print competition metrics
            print("\n" + "="*60)
            print("🏁 COMPETITION RESULTS")
            print("="*60)
            print(f"  Completion Time:    {self.sim_time:.3f} seconds ⭐ COMPETITION SCORE")
            print(f"  Path Length:        {self.traversed:.3f} units (reference)")
            print(f"  Planned Distance:   {self.planned_distance:.3f} units")
            print(f"  Path Efficiency:    {(self.planned_distance / max(self.traversed, 0.01) * 100):.1f}%")
            print(f"  Collisions:         {1 if self.collision else 0}")
            print(f"  Status:             {'✅ QUALIFIED' if not self.collision else '❌ DISQUALIFIED'}")
            print("="*60)
            print("⚡ Lower time is better - be the fastest!")
            print("="*60 + "\n")
            
            return

        # ── stall check ──
        min_spd = self.config.get("micromouse", {}).get("min_speed_threshold", 0.01)
        if abs(spd) < min_spd:
            self.status = "STALLED  (speed ~ 0)"
            self.paused = True
            return

        self.status = f"Running  v={spd:.2f}  d={math.degrees(steer):.0f} deg"

    # ------------------------------------------------------------------ #
    #  Coordinate helper                                                  #
    # ------------------------------------------------------------------ #

    def _m2s(self, pos):
        """Maze (row, col) -> screen (px_x, px_y)."""
        return (pos[1] * self.cell_size, pos[0] * self.cell_size)

    def _check_collision_at_radius(self, r, c, radius):
        """Check if robot with given radius collides with any walls.
        
        Args:
            r, c: robot center position in grid coordinates
            radius: collision radius in grid units
            
        Returns:
            True if collision detected, False otherwise
        """
        # Check center point
        if self._is_wall(r, c):
            return True
        
        # Check points around the perimeter (8 directions + some intermediate)
        num_check_points = 16
        for i in range(num_check_points):
            angle = 2 * math.pi * i / num_check_points
            check_r = r + radius * math.cos(angle)
            check_c = c + radius * math.sin(angle)
            
            if self._is_wall(check_r, check_c):
                return True
        
        # Also check the four corners at full diagonal
        diag_radius = radius * 0.707  # sqrt(2)/2
        for dr, dc in [(diag_radius, diag_radius), (diag_radius, -diag_radius),
                       (-diag_radius, diag_radius), (-diag_radius, -diag_radius)]:
            if self._is_wall(r + dr, c + dc):
                return True
        
        return False
    
    def _is_wall(self, r, c):
        """Check if a position is out of bounds or inside a wall.
        
        Args:
            r, c: position in grid coordinates
            
        Returns:
            True if position is wall or out of bounds, False otherwise
        """
        # Out of bounds check
        if not (0 <= r < self.maze.row and 0 <= c < self.maze.col):
            return True
        
        # Wall check
        ir, ic = int(r), int(c)
        if 0 <= ir < self.maze.row and 0 <= ic < self.maze.col:
            return self.maze.map[ir][ic].state == "#"
        
        return True

    # ------------------------------------------------------------------ #
    #  Robot rendering helpers                                            #
    # ------------------------------------------------------------------ #

    def _draw_rounded_robot(self, cx, cy, length, width, heading, color, is_spinning,
                           wheel_length, wheel_width):
        """Draw complete robot (body + wheels) as one rigid unit.
        
        Args:
            cx, cy: center position in screen coordinates
            length: body length in pixels (front-to-back)
            width: body width in pixels (side-to-side, ~ wheelbase)
            heading: robot heading angle in radians (0 = right, π/2 = up)
            color: body color
            is_spinning: whether robot is spinning out
            wheel_length: wheel length in pixels
            wheel_width: wheel thickness in pixels
        """
        # Create a surface large enough for body + wheels
        max_dim = int(max(length, width) * 2)
        robot_surf = pygame.Surface((max_dim, max_dim), pygame.SRCALPHA)
        center = max_dim // 2
        
        # ---- Draw wheels FIRST (so they appear behind body) ----
        wheel_offset = width / 2  # Wheels at edge of body
        
        # Left wheel (when facing forward)
        left_wheel_x = int(center)
        left_wheel_y = int(center - wheel_offset)
        self._draw_wheel_on_surface(robot_surf, left_wheel_x, left_wheel_y, 
                                    wheel_length, wheel_width)
        
        # Right wheel
        right_wheel_x = int(center)
        right_wheel_y = int(center + wheel_offset)
        self._draw_wheel_on_surface(robot_surf, right_wheel_x, right_wheel_y,
                                     wheel_length, wheel_width)
        
        # ---- Draw robot body ----
        rect_x = center - length // 2
        rect_y = center - width // 2
        corner_radius = int(width * 0.25)
        
        # Main body
        body_rect = pygame.Rect(rect_x, rect_y, length, width)
        pygame.draw.rect(robot_surf, color, body_rect, border_radius=corner_radius)
        
        # Body outline
        outline_color = C_WHITE if not is_spinning else (255, 255, 100)
        pygame.draw.rect(robot_surf, outline_color, body_rect, 2, border_radius=corner_radius)
        
        # Front sensors (3 small circles at the front)
        sensor_y = center
        sensor_x_start = center + length // 2 - length * 0.15
        sensor_r = max(2, int(width * 0.08))
        sensor_spacing = width * 0.3
        
        for i in [-1, 0, 1]:
            sy = int(sensor_y + i * sensor_spacing)
            sx = int(sensor_x_start)
            pygame.draw.circle(robot_surf, (100, 150, 255), (sx, sy), sensor_r)
        
        # ---- Rotate the entire surface (body + wheels together) ----
        # pygame rotation is counter-clockwise from 0 = right
        # our heading is: 0 = down, π/2 = right, π = up, 3π/2 = left
        # The sprite naturally points right (front at +x), so:
        # pygame_angle = heading_degrees - 90
        heading_deg = math.degrees(heading)
        pygame_angle = heading_deg - 90
        rotated = pygame.transform.rotate(robot_surf, pygame_angle)
        
        # Blit centered
        rotated_rect = rotated.get_rect(center=(int(cx), int(cy)))
        self.screen.blit(rotated, rotated_rect)

    def _draw_wheel_on_surface(self, surface, x, y, length, width):
        """Draw a single wheel directly on a surface (before rotation).
        
        Args:
            surface: pygame Surface to draw on
            x, y: wheel center position on the surface
            length: wheel length in pixels
            width: wheel thickness in pixels
        """
        # Rotate wheel 90 degrees: swap length/width dimensions
        rect_x = int(x - width // 2)
        rect_y = int(y - length // 2)
        wheel_rect = pygame.Rect(rect_x, rect_y, int(width), int(length))
        
        # Black wheel with gray outline
        pygame.draw.rect(surface, (40, 40, 40), wheel_rect, border_radius=int(width * 0.3))
        pygame.draw.rect(surface, (120, 120, 120), wheel_rect, 1, border_radius=int(width * 0.3))

    # ------------------------------------------------------------------ #
    #  Drawing                                                            #
    # ------------------------------------------------------------------ #

    def _draw(self):
        self.screen.fill(C_BG)

        # 1. maze background
        self.screen.blit(self.maze_surface, (0, 0))

        # 1b. toggleable overlays (drawn on top of maze, under paths)
        if self.show_distance and self._distance_surface is not None:
            self.screen.blit(self._distance_surface, (0, 0))
        if self.show_heatmap and self._heatmap_surface is not None:
            self.screen.blit(self._heatmap_surface, (0, 0))
        if self.show_grid and self._grid_surface is not None:
            self.screen.blit(self._grid_surface, (0, 0))

        # 2. planned path
        if len(self.path) > 1:
            pts = [self._m2s(p) for p in self.path]
            w = max(1, int(self.cell_size * 0.35))
            pygame.draw.lines(self.screen, C_PATH, False, pts, w)

        # 3. actual trajectory
        if len(self.trajectory) > 1:
            pts = [self._m2s(p) for p in self.trajectory]
            w = max(1, int(self.cell_size * 0.45))
            pygame.draw.lines(self.screen, C_TRAJ, False, pts, w)

        # 4. start / goal markers (rings drawn on overlay for visibility)
        marker_r = max(4, int(self.cell_size * 1.8))
        sx, sy = self._m2s(self.maze.start)
        gx, gy = self._m2s(self.maze.goal)
        pygame.draw.circle(self.screen, C_START, (int(sx), int(sy)), marker_r, 2)
        pygame.draw.circle(self.screen, C_GOAL,  (int(gx), int(gy)), marker_r, 2)

        # 5. lookahead point
        if hasattr(self, "pp") and self.pp.path:
            idx = min(self.pp.current_index, len(self.pp.path) - 1)
            lx, ly = self._m2s(self.pp.path[idx])
            pygame.draw.circle(
                self.screen, C_LOOKAHEAD,
                (int(lx), int(ly)),
                max(3, int(self.cell_size * 0.7)), 2,
            )

        # 6. robot (realistic rendering based on physical dimensions)
        rx, ry = self._m2s(self.pos)
        
        # Calculate robot dimensions based on physical specs from config
        # Wheelbase is 0.10m, typical micromouse body is slightly larger
        wheelbase_meters = self.robot.wheelbase  # 0.10m
        wheel_radius_meters = self.robot.wheel_radius  # 0.033m
        
        # Convert physical dimensions to screen pixels
        # Body width ~= wheelbase, body length slightly larger for aesthetics
        body_width_px = (wheelbase_meters / self.meters_per_cell) * self.cell_size
        body_length_px = body_width_px * 1.1  # Slightly longer than wide
        wheel_width_px = (wheel_radius_meters * 2 / self.meters_per_cell) * self.cell_size
        wheel_length_px = wheel_width_px * 0.4  # Wheels are thinner
        
        # Get diagnostics for visual feedback
        diag = self.robot.get_diagnostics()
        is_spinning = diag['is_spinning_out']
        
        # Spinout glow effect
        if is_spinning:
            glow_r = int(body_width_px * 1.5)
            glow_surf = pygame.Surface((glow_r * 2, glow_r * 2), pygame.SRCALPHA)
            for i in range(3):
                alpha = 80 - i * 25
                radius = glow_r - i * int(body_width_px * 0.3)
                pygame.draw.circle(glow_surf, (*C_SPIN_GLOW, alpha), (glow_r, glow_r), radius)
            self.screen.blit(glow_surf, (int(rx - glow_r), int(ry - glow_r)))
        
        # Robot body color
        body_c = (
            C_ROBOT_COLL if self.collision
            else C_ROBOT_GOAL if self.goal_reached
            else C_ROBOT_SPIN if is_spinning
            else C_ROBOT
        )
        
        # Draw robot body + wheels as one rigid unit
        self._draw_rounded_robot(rx, ry, body_length_px, body_width_px, 
                                  self.heading, body_c, is_spinning,
                                  wheel_length_px, wheel_width_px)
        
        # heading indicator (front sensor array)
        hl = body_length_px * 0.7
        hx = rx + hl * math.sin(self.heading)
        hy = ry + hl * math.cos(self.heading)
        pygame.draw.line(
            self.screen, C_HEADING,
            (int(rx), int(ry)), (int(hx), int(hy)), 2,
        )

        # 7. HUD
        self._draw_hud()

        pygame.display.flip()

    # ------------------------------------------------------------------ #

    def _draw_hud(self):
        """Draw the right-hand side heads-up display."""
        x0  = self.maze_w
        htot = self.maze_h
        pad = 12
        y   = 12

        # background + border
        pygame.draw.rect(self.screen, C_HUD_BG, (x0, 0, self.hud_width, htot))
        pygame.draw.line(self.screen, C_HUD_BORDER, (x0, 0), (x0, htot), 2)

        # helper closures
        def section(text):
            nonlocal y
            surf = self.font_lg.render(text, True, C_HUD_TITLE)
            self.screen.blit(surf, (x0 + pad, y))
            y += 28

        def divider():
            nonlocal y
            pygame.draw.line(
                self.screen, C_HUD_BORDER,
                (x0 + pad, y), (x0 + self.hud_width - pad, y),
            )
            y += 8

        def metric(label, value, vc=C_HUD_VALUE):
            nonlocal y
            self.screen.blit(self.font.render(label, True, C_HUD_LABEL), (x0 + pad, y))
            self.screen.blit(self.font.render(str(value), True, vc),     (x0 + 138, y))
            y += 22

        def key_hint(key, desc):
            nonlocal y
            self.screen.blit(self.font_sm.render(key,  True, C_HUD_KEY),   (x0 + pad, y))
            self.screen.blit(self.font_sm.render(desc, True, C_HUD_LABEL), (x0 + 90, y))
            y += 20

        # ── Title ──
        section("MICROMOUSE")
        divider()

        # ── Status ──
        sc = (
            C_ROBOT_COLL if self.collision
            else C_ROBOT_GOAL if self.goal_reached
            else C_PATH
        )
        self.screen.blit(self.font.render("Status:", True, C_HUD_LABEL), (x0 + pad, y))
        y += 22
        text = self.status
        while text:
            chunk, text = text[:28], text[28:]
            self.screen.blit(self.font_sm.render(chunk, True, sc), (x0 + pad, y))
            y += 18
        y += 6
        divider()

        # ── Metrics ──
        section("Metrics")
        metric("Sim time",   f"{self.sim_time:.1f} s")
        metric("Sim speed",  f"{self.sim_speed:.2g}x")
        metric("Position",   f"({self.pos[0]:.1f}, {self.pos[1]:.1f})")
        metric("Heading",    f"{math.degrees(self.heading):.1f} deg")
        metric("Speed",      f"{self.speed:.3f}")
        metric("Steering",   f"{math.degrees(self.steering):.1f} deg")
        y += 4
        divider()
        
        # ── Traction ──
        section("Traction")
        diag = self.robot.get_diagnostics()
        grip_pct = diag['grip_percentage']
        grip_color = (
            (220, 30, 30) if grip_pct < 70 else
            (255, 180, 0) if grip_pct < 85 else
            (50, 220, 50)
        )
        metric("Grip",       f"{grip_pct:.1f}%", vc=grip_color)
        
        if diag['is_spinning_out']:
            self.screen.blit(self.font.render("⚠ SPINOUT!", True, (255, 50, 255)), (x0 + pad, y))
            y += 22
        
        v_l, v_r = self.robot.get_wheel_velocities()
        metric("Wheel L",    f"{v_l:.2f} rad/s")
        metric("Wheel R",    f"{v_r:.2f} rad/s")
        y += 4
        metric("Plan dist",  f"{self.planned_distance:.1f}")
        metric("Traversed",  f"{self.traversed:.1f}")
        eff = self.planned_distance / max(self.traversed, 0.01) * 100
        eff_c = C_ROBOT_GOAL if eff > 90 else (C_PATH if eff > 70 else C_ROBOT_COLL)
        metric("Efficiency", f"{eff:.1f}%", eff_c)
        # Wall distance at robot position
        if hasattr(self.maze, "wall_distance"):
            ri, ci = int(self.pos[0]), int(self.pos[1])
            if 0 <= ri < self.maze.row and 0 <= ci < self.maze.col:
                wd = float(self.maze.wall_distance[ri, ci])
                wd_c = C_ROBOT_COLL if wd < 2 else (C_PATH if wd < 4 else C_ROBOT_GOAL)
                metric("Wall dist", f"{wd:.1f}", wd_c)
        y += 4
        divider()

        # ── Controls ──
        section("Controls")
        key_hint("SPACE",    "Pause / Resume")
        key_hint("R",        "Reset")
        key_hint("UP / DN",  "Speed  +/-")
        key_hint("RIGHT",    "Step (paused)")
        key_hint("H",        "Cost heatmap " + ("ON" if self.show_heatmap else "off"))
        key_hint("D",        "Distance fld " + ("ON" if self.show_distance else "off"))
        key_hint("G",        "Grid lines "   + ("ON" if self.show_grid else "off"))
        key_hint("Q / Esc",  "Quit")
        y += 4
        divider()

        # ── Legend ──
        section("Legend")
        legend_items = [
            (C_PATH,      "Planned path"),
            (C_TRAJ,      "Trajectory"),
            (C_ROBOT,     "Robot"),
            (C_LOOKAHEAD, "Lookahead pt"),
            (C_START,     "Start"),
            (C_GOAL,      "Goal"),
        ]
        if self.show_heatmap:
            legend_items.append(((220, 40, 10), "High wall cost"))
        if self.show_distance:
            legend_items.append(((40, 220, 240), "Near wall"))
        for colour, label in legend_items:
            pygame.draw.circle(self.screen, colour, (x0 + pad + 6, y + 7), 5)
            self.screen.blit(
                self.font_sm.render(label, True, C_HUD_LABEL), (x0 + pad + 18, y)
            )
            y += 20

    # ------------------------------------------------------------------ #
    #  Event handling                                                     #
    # ------------------------------------------------------------------ #

    def _handle_events(self):
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                self.running = False

            elif ev.type == pygame.KEYDOWN:
                if ev.key in (pygame.K_q, pygame.K_ESCAPE):
                    self.running = False

                elif ev.key == pygame.K_SPACE:
                    if self.collision or self.goal_reached:
                        self._reset_robot()          # quick restart
                    else:
                        self.paused = not self.paused

                elif ev.key == pygame.K_r:
                    # full reset: fresh maze, re-plan, rebuild overlays, restart
                    self.maze = Map(self.maze_array)
                    self.maze_surface = self._render_maze_surface()
                    self._plan_path()
                    self._build_overlay_surfaces()
                    self._reset_robot()

                elif ev.key == pygame.K_UP:
                    self.sim_speed = min(self.sim_speed * 2, 128)

                elif ev.key == pygame.K_DOWN:
                    self.sim_speed = max(self.sim_speed / 2, 0.25)

                elif ev.key == pygame.K_h:
                    self.show_heatmap = not self.show_heatmap

                elif ev.key == pygame.K_d:
                    self.show_distance = not self.show_distance

                elif ev.key == pygame.K_g:
                    self.show_grid = not self.show_grid

                elif ev.key == pygame.K_RIGHT and self.paused:
                    self._step()

    # ------------------------------------------------------------------ #
    #  Main loop                                                          #
    # ------------------------------------------------------------------ #

    def run(self):
        """Start the interactive simulation loop."""
        while self.running:
            self._handle_events()

            if not self.paused and not self.collision and not self.goal_reached:
                self.step_acc += self.sim_speed
                while self.step_acc >= 1.0:
                    self._step()
                    self.step_acc -= 1.0
                    if self.collision or self.goal_reached:
                        self.paused = True
                        break

            self._draw()
            self.clock.tick(self.target_fps)

        pygame.quit()


# ── Benchmark Mode (Headless) ────────────────────────────────────────────── #

class BenchmarkSimulation:
    """Headless simulation for fast benchmarking without visualization."""
    
    def __init__(self, config_file="config.yaml"):
        # ── config ──
        with open(config_file) as f:
            self.config = yaml.safe_load(f)
        
        # ── maze ──
        maze_file = self.config.get("maze_file", "maze.csv")
        self.maze_array = load_maze(maze_file)
        
        # Apply upsampling if enabled
        if self.config.get('maze_upsampling', {}).get('enabled', False):
            factor = self.config.get('maze_upsampling', {}).get('factor', 1)
            self.maze_array = upsample_maze(self.maze_array, factor)
        
        self.maze = Map(self.maze_array)
        
        # ── Convert metric speeds ──
        self._convert_metric_speeds()
        
        # ── timestep ──
        mc = self.config.get("micromouse", {})
        self.dt = mc.get("dt", 0.05)
        self.max_steps = mc.get("max_steps", 10000)  # Safety limit
        
        # ── path planning ──
        print("\n" + "="*60)
        print("🏁 BENCHMARK MODE - Planning...")
        print("="*60)
        
        start = self.maze.map[self.maze.start[0]][self.maze.start[1]]
        goal = self.maze.map[self.maze.goal[0]][self.maze.goal[1]]
        
        planner = AStar(self.maze, self.config)
        px, py = planner.plan_path(start, goal)
        self.path = list(zip(px, py)) if px and py else []
        
        if not self.path:
            print("❌ ERROR: No path found!")
            return
        
        print(f"✅ Path found: {len(self.path)} waypoints")
        
        # Calculate planned distance
        self.planned_distance = planner.calculate_path_distance(px, py)
        
        # ── robot & controller ──
        self.robot = DifferentialDriveRobot(self.config)
        self.controller = DiffDriveController(self.robot.wheel_radius, self.robot.wheelbase)
        
        # ── Pure Pursuit controller (no debug visualization) ──
        cfg = {**self.config}
        cfg["pure_pursuit"] = {**cfg.get("pure_pursuit", {}), "debug": False}
        self.pp = PurePursuit(self.maze, cfg)
        self.pp.set_path(self.path)
        
        # ── Initialize state ──
        sp = self.maze.start
        self.pos = list(sp)
        self.heading = mc.get("initial_heading", math.pi / 4)
        self.speed = 0.0
        self.steering = 0.0
        self.traversed = 0.0
        self.sim_time = 0.0
        self.collision = False
        self.goal_reached = False
        self.robot.stop()
        
    def _convert_metric_speeds(self):
        """Convert speeds from m/s to grid units/s if use_metric_speeds is enabled."""
        phys_dims = self.config.get('physical_dimensions', {})
        use_metric_speeds = phys_dims.get('use_metric_speeds', False)
        
        self.meters_per_cell = 1.0
        
        if not use_metric_speeds:
            return
        
        maze_width_meters = phys_dims.get('maze_width_meters', 2.88)
        maze_height_meters = phys_dims.get('maze_height_meters', 2.88)
        
        num_cols = self.maze.col
        num_rows = self.maze.row
        meters_per_cell_x = maze_width_meters / num_cols
        meters_per_cell_y = maze_height_meters / num_rows
        self.meters_per_cell = (meters_per_cell_x + meters_per_cell_y) / 2.0
        
        if 'pure_pursuit' in self.config:
            pp = self.config['pure_pursuit']
            if 'max_speed' in pp:
                pp['max_speed'] = pp['max_speed'] / self.meters_per_cell
            if 'min_speed' in pp:
                pp['min_speed'] = pp['min_speed'] / self.meters_per_cell
        
        if 'micromouse' in self.config:
            mm = self.config['micromouse']
            if 'min_speed_threshold' in mm:
                mm['min_speed_threshold'] = mm['min_speed_threshold'] / self.meters_per_cell
    
    def _check_collision(self, r, c):
        """Check if position collides with walls."""
        wheelbase_meters = self.robot.wheelbase
        body_width_grid = wheelbase_meters / self.meters_per_cell
        collision_radius = body_width_grid / 2.0
        
        # Check grid cells around robot
        check_radius = int(math.ceil(collision_radius)) + 1
        for dr in range(-check_radius, check_radius + 1):
            for dc in range(-check_radius, check_radius + 1):
                gr = int(round(r + dr))
                gc = int(round(c + dc))
                
                if 0 <= gr < self.maze.row and 0 <= gc < self.maze.col:
                    if self.maze.map[gr][gc].state == "#":
                        # Check actual distance
                        dist = math.hypot(r - gr, c - gc)
                        if dist < collision_radius:
                            return True
        return False
    
    def run(self):
        """Run simulation in tight loop without visualization."""
        if not self.path:
            print("❌ Cannot run: No valid path")
            return
        
        print("\n🚀 Running simulation...")
        
        step = 0
        while step < self.max_steps:
            step += 1
            
            # ── Pure Pursuit control ──
            spd, steer = self.pp.get_control(self.pos, self.heading)
            
            # ── Convert to wheel velocities ──
            spd_meters = spd * self.meters_per_cell
            wheelbase_grid = 1.0 * self.meters_per_cell
            omega = self.controller.steering_to_omega(spd_meters, steer, wheelbase_equiv=wheelbase_grid)
            v_left, v_right = self.controller.velocity_to_wheels(spd_meters, omega)
            self.robot.set_wheel_velocities(v_left, v_right, self.dt)
            
            # ── Update kinematics ──
            actual_v_meters, actual_omega = self.robot.update_kinematics(self.dt)
            v_grid = actual_v_meters / self.meters_per_cell
            
            # ── Save old position for distance tracking ──
            old_pos = list(self.pos)
            
            # ── Update pose ──
            self.heading += actual_omega * self.dt
            self.pos[0] += v_grid * math.sin(self.heading) * self.dt
            self.pos[1] += v_grid * math.cos(self.heading) * self.dt
            self.speed = v_grid
            self.steering = steer
            
            # ── Track distance ──
            dx = self.pos[1] - old_pos[1]
            dy = self.pos[0] - old_pos[0]
            self.traversed += math.hypot(dx, dy)
            self.sim_time += self.dt
            
            # ── Check collision ──
            r, c = self.pos
            if self._check_collision(r, c):
                self.collision = True
                break
            
            # ── Check goal ──
            gr, gc = self.path[-1]
            goal_thr = self.config.get("micromouse", {}).get("goal_threshold", 0.5)
            if math.hypot(r - gr, c - gc) < goal_thr:
                self.goal_reached = True
                break
            
            # ── Progress indicator (every 1000 steps) ──
            if step % 1000 == 0:
                print(f"  Step {step}: time={self.sim_time:.1f}s, pos=({r:.1f}, {c:.1f}), traversed={self.traversed:.1f}")
        
        # ── Print results ──
        self._print_results()
    
    def _print_results(self):
        """Print competition results."""
        print("\n" + "="*60)
        print("🏁 COMPETITION RESULTS")
        print("="*60)
        print(f"  Completion Time:    {self.sim_time:.3f} seconds ⭐ COMPETITION SCORE")
        print(f"  Path Length:        {self.traversed:.3f} units (reference)")
        print(f"  Planned Distance:   {self.planned_distance:.3f} units")
        print(f"  Path Efficiency:    {(self.planned_distance / max(self.traversed, 0.01) * 100):.1f}%")
        print(f"  Collisions:         {1 if self.collision else 0}")
        
        if self.collision:
            print(f"  Status:             ❌ DISQUALIFIED (collision)")
        elif self.goal_reached:
            print(f"  Status:             ✅ QUALIFIED")
        else:
            print(f"  Status:             ⚠️  TIMEOUT (did not reach goal)")
        
        print("="*60)
        print("⚡ Lower time is better - be the fastest!")
        print("="*60 + "\n")


# ── Entry point ──────────────────────────────────────────────────────────── #

def main():
    # Check for benchmark mode flag
    if len(sys.argv) > 1 and sys.argv[1] == "--benchmark":
        config_file = sys.argv[2] if len(sys.argv) > 2 else "config.yaml"
        print(f"\n📊 Starting benchmark mode with {config_file}")
        sim = BenchmarkSimulation(config_file)
        sim.run()
    else:
        config_file = sys.argv[1] if len(sys.argv) > 1 else "config.yaml"
        sim = PygameSimulation(config_file)
        sim.run()


if __name__ == "__main__":
    main()

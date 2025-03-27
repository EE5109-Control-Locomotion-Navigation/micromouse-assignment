import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from sys import maxsize

class LQR:
    def __init__(self, Q=np.eye(4), R=np.eye(2)):
        # Q: State cost matrix
        # R: Control cost matrix
        None # this is just added to avoid error, delete this

class PurePursuit:
    """
    Pure Pursuit Controller for path following
    Implements the pure pursuit algorithm to follow a pre-computed path
    """
    def __init__(self, maps, config=None):
        self.map = maps
        
        # Load configuration with defaults
        self.config = config.get('pure_pursuit', {}) if config else {}
        self.lookahead_dist = self.config.get('lookahead_distance', 1.5)
        self.debug = self.config.get('debug', False)
        self.visualize_every = self.config.get('visualize_every', 5)
        self.max_speed = self.config.get('max_speed', 0.5)
        self.min_speed = self.config.get('min_speed', 0.2)
        self.steering_gain = self.config.get('steering_gain', 1.0)
        self.slow_steering_threshold = self.config.get('slow_steering_threshold', math.pi/4)
        
        self.path = []
        self.current_index = 0
        self.counter = 0
        
        if self.debug:
            plt.ion()
            figsize = config.get('visualization', {}).get('figure_size', [12, 12])
            self.fig, self.ax = plt.subplots(figsize=figsize)
            cmap_colors = config.get('visualization', {}).get('cmap_colors', ['white', 'black', 'green', 'red', 'blue'])
            self.cmap = ListedColormap(cmap_colors)
            self.initialize_debug_plot()

    def initialize_debug_plot(self):
        """Set up real-time visualization for debugging"""
        grid = self.map.get_grid_representation()
        self.background = self.ax.imshow(grid, cmap=self.cmap)
        # Visualization elements for path following
        self.path_plot = self.ax.plot([], [], 'y-', linewidth=2, label='Planned Path')[0]
        self.lookahead_plot = self.ax.plot([], [], 'mo', markersize=6, alpha=0.8, label='Lookahead')[0]
        self.vector_plot = self.ax.plot([], [], 'm-', linewidth=1, alpha=0.8, label='Target Vector')[0]
        self.heading_plot = self.ax.plot([], [], 'g-', linewidth=2, alpha=0.8, label='Heading')[0]
        self.robot_plot = self.ax.plot([], [], 'ro', markersize=8, label='Robot')[0]
        self.ax.set_title("Pure Pursuit Controller")
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        self.ax.legend()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def set_path(self, path):
        """Set the path to follow"""
        self.path = path
        self.current_index = 0
        if self.debug and len(path) > 0:
            path_arr = np.array(path)
            self.path_plot.set_data(path_arr[:,1], path_arr[:,0])
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

    def find_lookahead_point(self, current_pos):
        """Find the lookahead point on the path"""
        for i in range(self.current_index, len(self.path)):
            dx = self.path[i][0] - current_pos[0]
            dy = self.path[i][1] - current_pos[1]
            dist = math.sqrt(dx**2 + dy**2)
            if dist >= self.lookahead_dist:
                self.current_index = i
                return self.path[i]
        return self.path[-1] if self.path else None

    def update_debug_plot(self, current_pos, current_heading, target_pos):
        """Update the real-time visualization"""
        if not self.debug or self.counter % self.visualize_every != 0:
            self.counter += 1
            return
            
        self.lookahead_plot.set_data([target_pos[1]], [target_pos[0]])
        self.vector_plot.set_data([current_pos[1], target_pos[1]], 
                                 [current_pos[0], target_pos[0]])
        
        heading_length = 2
        self.heading_plot.set_data(
            [current_pos[1], current_pos[1] + heading_length * math.sin(current_heading)],
            [current_pos[0], current_pos[0] + heading_length * math.cos(current_heading)])
        
        self.robot_plot.set_data([current_pos[1]], [current_pos[0]])
        
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        self.counter += 1

    def get_control(self, current_pos, current_heading):
        """Calculate control commands (speed, steering) to follow the path"""
        target = self.find_lookahead_point(current_pos)
        if target is None:
            return 0, 0  # Stop if no target

        if self.debug:
            self.update_debug_plot(current_pos, current_heading, target)
        
        # Convert target to robot's local coordinate system
        dx = target[0] - current_pos[0]
        dy = target[1] - current_pos[1]
        target_local_x = dx * math.cos(-current_heading) - dy * math.sin(-current_heading)
        target_local_y = dx * math.sin(-current_heading) + dy * math.cos(-current_heading)
        
        # Calculate curvature and steering angle
        curvature = 2 * target_local_y / (self.lookahead_dist**2)
        steering_angle = math.atan(curvature * self.steering_gain)
        
        # Adjust speed based on steering angle
        speed = self.max_speed if abs(steering_angle) < self.slow_steering_threshold else self.min_speed
        
        return speed, steering_angle
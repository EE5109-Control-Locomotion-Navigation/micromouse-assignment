import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
from sys import maxsize


class RRT:
    def __init__(self, map, max_iter=1000, step_size=0.5, goal_bias=0.1):
        # Your implementation here
        None # this is just added to avoid error, delete this

class AStar:
    """
    A* Pathfinding Algorithm implementation for finding optimal path through maze
    """
    def __init__(self, maps, config=None):
        self.map = maps
        self.open_list = set()  # States to be evaluated
        self.closed_list = set()  # Evaluated states
        
        # Load configuration with defaults
        self.config = config.get('astar', {}) if config else {}
        self.debug = self.config.get('debug', False)
        self.visualize_every = self.config.get('visualize_every', 10)
        self.heuristic_weight = self.config.get('heuristic_weight', 1.0)
        self.counter = 0

        # Wall proximity cost configuration
        wc = config.get('wall_cost', {}) if config else {}
        self.wall_cost_enabled   = wc.get('enabled', False)
        self.wc_weight           = wc.get('weight', 2.0)
        self.wc_decay            = wc.get('decay', 'exponential')
        self.wc_decay_rate       = wc.get('decay_rate', 0.5)
        self.wc_threshold        = wc.get('threshold', 5.0)
        if self.wall_cost_enabled:
            self.map.compute_wall_distance_map()
        
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
        # Visualization elements for open set, closed set, and current path
        self.open_set_plot = self.ax.plot([], [], 'yo', markersize=2, alpha=0.5, label='Open Set')[0]
        self.closed_set_plot = self.ax.plot([], [], 'co', markersize=2, alpha=0.3, label='Closed Set')[0]
        self.path_plot = self.ax.plot([], [], 'm-', linewidth=1, alpha=0.7, label='Current Path')[0]
        self.ax.set_title("A* Path Planning (Start to Goal)")
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        self.ax.legend()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def heuristic(self, a, b):
        """Manhattan distance heuristic with configurable weight"""
        return self.heuristic_weight * (abs(a.x - b.x) + abs(a.y - b.y))

    def wall_proximity_cost(self, state):
        """Return extra traversal cost based on proximity to walls.

        Configurable via ``wall_cost`` in config.yaml:
          - **weight**:     overall scaling factor
          - **decay**:      'exponential', 'inverse', or 'linear'
          - **decay_rate**: rate parameter for exponential decay
          - **threshold**:  distance beyond which the cost is zero
        """
        if not self.wall_cost_enabled:
            return 0.0
        d = float(self.map.wall_distance[state.x, state.y])
        if d >= self.wc_threshold:
            return 0.0
        if self.wc_decay == 'exponential':
            return self.wc_weight * math.exp(-self.wc_decay_rate * d)
        elif self.wc_decay == 'inverse':
            return self.wc_weight / (d + 0.1)
        elif self.wc_decay == 'linear':
            return self.wc_weight * (1.0 - d / self.wc_threshold)
        return 0.0

    def update_debug_plot(self, current_state=None):
        """Update the real-time visualization"""
        if not self.debug or self.counter % self.visualize_every != 0:
            self.counter += 1
            return
        
        # Update visualization elements
        open_x = [s.x for s in self.open_list]
        open_y = [s.y for s in self.open_list]
        self.open_set_plot.set_data(open_y, open_x)

        closed_x = [s.x for s in self.closed_list]
        closed_y = [s.y for s in self.closed_list]
        self.closed_set_plot.set_data(closed_y, closed_x)

        if current_state:  # Draw current best path
            path_x, path_y = [], []
            temp_state = current_state
            while temp_state:
                path_x.append(temp_state.x)
                path_y.append(temp_state.y)
                temp_state = temp_state.parent
            self.path_plot.set_data(path_y, path_x)

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        self.counter += 1

    def plan_path(self, start, goal):
        """Find optimal path from start to goal using A* algorithm"""
        self.open_list.add(start)
        start.h = self.heuristic(start, goal)
        start.k = start.h

        while self.open_list:
            current = min(self.open_list, key=lambda x: x.k)  # Get state with lowest cost
            
            if current == goal:  # Path found
                return self.reconstruct_path(current)

            self.open_list.remove(current)
            self.closed_list.add(current)
            self.update_debug_plot(current)

            for neighbor in self.map.get_neighbors(current):
                if neighbor in self.closed_list or neighbor.state == "#":
                    continue  # Skip walls and already evaluated states

                tentative_g = current.h + current.cost(neighbor) + self.wall_proximity_cost(neighbor)
                
                if neighbor not in self.open_list:
                    self.open_list.add(neighbor)
                elif tentative_g >= neighbor.h:
                    continue  # Not a better path

                # Update neighbor with better path
                neighbor.parent = current
                neighbor.h = tentative_g
                neighbor.k = neighbor.h + self.heuristic(neighbor, goal)

        return [], []  # Return empty path if none found

    def reconstruct_path(self, current):
        """Backtrack from goal to start to get the final path"""
        path_x, path_y = [], []
        while current:
            path_x.append(current.x)
            path_y.append(current.y)
            current.set_state("*")  # Mark path in the map
            current = current.parent
        
        path_x.reverse()
        path_y.reverse()
        
        if self.debug:
            plt.close(self.fig)
        return path_x, path_y
    
    def calculate_path_distance(self, path_x, path_y):
        """Calculate total length of the planned path"""
        if not path_x or not path_y or len(path_x) != len(path_y):
            return 0
        distance = 0
        for i in range(1, len(path_x)):
            dx = path_x[i] - path_x[i-1]
            dy = path_y[i] - path_y[i-1]
            distance += math.sqrt(dx**2 + dy**2)
        return distance
"""
Micromouse Simulation Program
----------------------------
This program simulates a micromouse robot navigating through a maze using:
- A* algorithm for path planning
- Pure Pursuit controller for path following
- Collision detection and reporting

The program reads maze configuration from a CSV file and parameters from a YAML config file.
"""

import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import csv
from sys import maxsize
import time
import yaml

from pathPlanning import AStar
from pathTracking import PurePursuit

def optimize_path(path, map, min_distance=0.3):
    # Your safety and smoothing implementation
    None # this is just added to avoid error, delete this

class State:
    """
    Represents a single cell in the maze with position, state, and pathfinding information
    """
    def __init__(self, x, y):
        self.x = x  # x-coordinate in the maze
        self.y = y  # y-coordinate in the maze
        self.parent = None  # Parent state in pathfinding
        self.state = "."  # Cell state: ".", "#", "s", "e", or "*"
        self.t = "new"  # State tag for algorithms
        self.h = 0  # Heuristic cost
        self.k = 0  # Total estimated cost

    def cost(self, state):
        """Calculate movement cost to another state (infinity if wall)"""
        if self.state == "#" or state.state == "#":
            return maxsize  # Infinite cost for walls
        return math.sqrt((self.x - state.x)**2 + (self.y - state.y)**2)

    def set_state(self, state):
        """Set the cell state if valid"""
        if state in ["s", ".", "#", "e", "*"]:
            self.state = state

class Map:
    """
    Represents the maze environment and provides maze-related operations
    """
    def __init__(self, maze_array):
        self.row = len(maze_array)
        self.col = len(maze_array[0]) if self.row > 0 else 0
        self.map = self.init_map()
        self.process_maze_array(maze_array)

    def init_map(self):
        """Initialize 2D grid of State objects"""
        return [[State(i, j) for j in range(self.col)] for i in range(self.row)]

    def process_maze_array(self, maze_array):
        """Convert numerical maze array to State objects with proper states"""
        for i in range(self.row):
            for j in range(self.col):
                if maze_array[i][j] == 1:  # Wall
                    self.map[i][j].set_state("#")
                elif maze_array[i][j] == 2:  # Start
                    self.map[i][j].set_state("s")
                    self.start = (i, j)
                elif maze_array[i][j] == 4:  # End
                    self.map[i][j].set_state("e")
                    self.goal = (i, j)

    def get_neighbors(self, state):
        """Get valid neighboring states (up, down, left, right)"""
        neighbors = []
        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:  # 4-directional movement
            x, y = state.x + dx, state.y + dy
            if 0 <= x < self.row and 0 <= y < self.col:
                neighbors.append(self.map[x][y])
        return neighbors

    def get_grid_representation(self):
        """Convert State map to numerical grid for visualization"""
        grid = np.zeros((self.row, self.col))
        for i in range(self.row):
            for j in range(self.col):
                if self.map[i][j].state == "#":  # Wall
                    grid[i][j] = 1
                elif self.map[i][j].state == "s":  # Start
                    grid[i][j] = 3
                elif self.map[i][j].state == "e":  # End
                    grid[i][j] = 4
                elif self.map[i][j].state == "*":  # Path
                    grid[i][j] = 2
        return grid

def read_maze_from_csv(filename):
    """Read maze configuration from CSV file"""
    maze = []
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            # Convert valid cells to integers, default to 1 (wall) for invalid cells
            filtered_row = [int(cell) if cell.strip().isdigit() else 1 for cell in row if cell.strip() != '']
            if filtered_row:
                maze.append(filtered_row)
    return maze

class Micromouse:
    """
    Micromouse robot simulation
    Combines path planning and path following with collision detection
    """
    def __init__(self, maps, start_pos, config=None):
        self.map = maps
        self.position = list(start_pos)
        
        # Load configuration with defaults
        self.config = config.get('micromouse', {}) if config else {}
        self.heading = self.config.get('initial_heading', math.pi/4)
        self.debug = self.config.get('debug', False)
        self.visualize_every = self.config.get('visualize_every', 5)
        self.dt = self.config.get('dt', 0.1)
        self.goal_threshold = self.config.get('goal_threshold', 0.5)
        self.min_speed_threshold = self.config.get('min_speed_threshold', 0.01)
        
        self.trajectory = [list(start_pos)]  # Record of all positions
        self.speed = 0
        self.steering = 0
        self.counter = 0
        self.pp = PurePursuit(maps, config)  # Pure Pursuit controller
        
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
        # Visualization elements for micromouse
        self.path_plot = self.ax.plot([], [], 'y-', linewidth=2, label='Planned Path')[0]
        self.traj_plot = self.ax.plot([], [], 'r-', linewidth=1, alpha=0.7, label='Trajectory')[0]
        self.pos_plot = self.ax.plot([], [], 'ro', markersize=8, label='Position')[0]
        self.heading_plot = self.ax.plot([], [], 'g-', linewidth=2, label='Heading')[0]
        self.ax.set_title("Micromouse Simulation")
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        self.ax.legend()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def update_debug_plot(self, path=None):
        """Update the real-time visualization"""
        if not self.debug or self.counter % self.visualize_every != 0:
            self.counter += 1
            return
            
        grid = self.map.get_grid_representation()
        self.background.set_array(grid)
        
        if path is not None and len(path) > 0:
            path_arr = np.array(path)
            self.path_plot.set_data(path_arr[:,1], path_arr[:,0])
        
        traj = np.array(self.trajectory)
        if len(traj) > 0:
            self.traj_plot.set_data(traj[:,1], traj[:,0])
        
        self.pos_plot.set_data([self.position[1]], [self.position[0]])
        
        heading_length = 2
        self.heading_plot.set_data(
            [self.position[1], self.position[1] + heading_length * math.sin(self.heading)],
            [self.position[0], self.position[0] + heading_length * math.cos(self.heading)])
        
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        self.counter += 1

    def update(self, dt, path=None):
        """Update robot position based on current speed and steering"""
        self.position[0] += self.speed * math.cos(self.heading) * dt
        self.position[1] += self.speed * math.sin(self.heading) * dt
        self.heading += self.speed * math.tan(self.steering) * dt
        self.heading = (self.heading + math.pi) % (2*math.pi) - math.pi  # Normalize angle
        self.trajectory.append(list(self.position))  # Record new position
        
        if self.debug:
            self.update_debug_plot(path)

    def check_collision(self, position):
        """
        Check if current position collides with a wall
        Returns True if collision detected, False otherwise
        """
        x, y = position
        # Check if position is out of bounds
        if not (0 <= x < self.map.row and 0 <= y < self.map.col):
            return True
        # Check if position is inside a wall
        return self.map.map[int(x)][int(y)].state == "#"

    def follow_path(self, path, dt=0.1):
        """
        Follow the given path using Pure Pursuit controller
        Returns the total distance traveled before stopping
        """
        self.pp.set_path(path)
        traversed_distance = 0
        
        while True:
            prev_position = list(self.position)
            speed, steering = self.pp.get_control(self.position, self.heading)
            self.speed = speed
            self.steering = steering
            self.update(dt, path)
            
            # Calculate distance moved in this step
            dx = self.position[0] - prev_position[0]
            dy = self.position[1] - prev_position[1]
            traversed_distance += math.sqrt(dx**2 + dy**2)
            
            # Check for collision with walls
            if self.check_collision(self.position):
                print(f"COLLISION DETECTED! Distance traveled: {traversed_distance:.2f} units")
                return traversed_distance
            
            if len(path) == 0:  # No path to follow
                break
                
            goal = path[-1]
            dist_to_goal = math.sqrt((self.position[0]-goal[0])**2 + 
                          (self.position[1]-goal[1])**2)
            
            if dist_to_goal < self.goal_threshold:  # Reached goal
                break
            
            if abs(self.speed) < self.min_speed_threshold:  # Stopped moving
                break
        
        return traversed_distance

def visualize(maze, path=None, mouse_traj=None, title="Micromouse Simulation"):
    """
    Create a static visualization of the maze, planned path, and actual trajectory
    """
    plt.figure(figsize=(12, 12))
    
    grid = maze.get_grid_representation()
    cmap = ListedColormap(['white', 'black', 'green', 'red', 'blue'])
    plt.imshow(grid, cmap=cmap)
    
    if path and len(path) > 0:  # Draw planned path
        path_arr = np.array(path)
        plt.plot(path_arr[:,1], path_arr[:,0], 'y-', linewidth=2, label='Planned Path')
    
    if mouse_traj:  # Draw actual trajectory
        traj = np.array(mouse_traj)
        plt.plot(traj[:,1], traj[:,0], 'r-', linewidth=1, label='Actual Path')
        plt.plot(traj[0,1], traj[0,0], 'go', markersize=10, label='Start')
        plt.plot(traj[-1,1], traj[-1,0], 'bx', markersize=10, label='End')
    
    plt.legend()
    plt.title(title)
    plt.xticks([])
    plt.yticks([])
    plt.show()

def main(config_file='config.yaml'):
    """
    Main function to run the micromouse simulation
    """
    # Load configuration from YAML file
    with open(config_file) as f:
        config = yaml.safe_load(f)
    
    # Load and initialize maze
    maze_array = read_maze_from_csv('maze.csv')
    maze = Map(maze_array)
    start_pos = maze.start
    goal_pos = maze.goal
    start_state = maze.map[start_pos[0]][start_pos[1]]
    goal_state = maze.map[goal_pos[0]][goal_pos[1]]
    
    # Plan path using A*
    astar = AStar(maze, config)
    path_x, path_y = astar.plan_path(start_state, goal_state)
    path = list(zip(path_x, path_y)) if path_x and path_y else []
    
    # Calculate and display planned path distance
    planned_distance = astar.calculate_path_distance(path_x, path_y)
    print(f"Planned path distance: {planned_distance:.2f} units")
    
    # Run micromouse simulation
    mouse = Micromouse(maze, start_pos, config)
    traversed_distance = mouse.follow_path(path)
    
    # Display results
    print(f"Actual traversed distance: {traversed_distance:.2f} units")
    
    # Show final visualization if not in debug mode
    if not config.get('astar', {}).get('debug', False):
        visualize(maze, path, mouse.trajectory, 
                 title=config.get('visualization', {}).get('title', "Micromouse Simulation"))

if __name__ == "__main__":
    main()
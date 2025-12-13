import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon
from robot import Robot
from obstacle import Obstacle
import numpy as np
import math
import numpy as np
import math
from obstacle import Obstacle

class Animation:
    def __init__(self, robot: Robot, obstacles: list[Polygon]):
        self.robot = robot
        self.obstacles = obstacles
        self.user_points = []
        self.obs_helper = Obstacle()
        self.c_space_grid = None
        self.c_space_resolution = 50
        self.q1_range = np.linspace(-6, 6, self.c_space_resolution)
        self.q2_range = np.linspace(-np.pi, np.pi, self.c_space_resolution)

    def compute_c_space_grid(self):
        print("Computing C-Space Grid...")
        self.c_space_grid = np.zeros((self.c_space_resolution, self.c_space_resolution))
        q3_fixed = self.q_start[2]
        q4_fixed = self.q_start[3]
        
        for i, q1 in enumerate(self.q1_range):
            for j, q2 in enumerate(self.q2_range):
                q = np.array([q1, q2, q3_fixed, q4_fixed])
                segments = self.robot.get_link_segments(q)
                collision = False
                for obstacle in self.obstacles:
                    poly_points = obstacle.get_xy()
                    for segment in segments:
                        p1, p2 = segment
                        if self.obs_helper.check_collision(poly_points, p1, p2):
                            collision = True
                            break
                    if collision: break
                
                if collision:
                    self.c_space_grid[j, i] = 1

    def solve_ik(self, target_point):
        best_q = None
        min_dist = float('inf')
        q3_fixed = self.q_start[2]
        q4_fixed = self.q_start[3]
        
        for q1 in self.q1_range:
            for q2 in self.q2_range:
                q = np.array([q1, q2, q3_fixed, q4_fixed])
                segments = self.robot.get_link_segments(q)
                eff_pos = segments[-1][1]
                
                dist = np.linalg.norm(eff_pos - np.array(target_point))
                if dist < min_dist:
                    min_dist = dist
                    best_q = (q1, q2)
                    
        return best_q

    def update(self, frame):
        alpha = frame / 100.0
        if alpha > 1.0: alpha = 1.0
        q_current = (1 - alpha) * self.q_start + alpha * self.q_end
        
        segments = self.robot.get_link_segments(q_current)
        
        for line, segment in zip(self.lines_2d, segments):
            p1, p2 = segment
            line.set_data([p1[0], p2[0]], [p1[1], p2[1]])
            
        self.c_space_dot.set_data([q_current[0]], [q_current[1]])
            
        return self.lines_2d + [self.c_space_dot]

    def animate(self, q_start, q_end):
        self.q_start = q_start
        self.q_end = q_end
        
        self.compute_c_space_grid()
        
        fig = plt.figure(figsize=(12, 6))

        segments = self.robot.get_link_segments(q_start)

        ax_2d = fig.add_subplot(121)
        self.lines_2d = []
        for segment in segments:
            p1, p2 = segment
            line, = ax_2d.plot([p1[0], p2[0]], [p1[1], p2[1]], color='blue', linewidth=2)
            self.lines_2d.append(line)

        for obstacle in self.obstacles:
            ax_2d.add_patch(obstacle)

        for point in self.user_points:
            ax_2d.plot(point[0], point[1], 'go', markersize=10)

        ax_2d.set_title("Workspace")
        ax_2d.set_xlabel('X')
        ax_2d.set_ylabel('Y')
        ax_2d.set_xlim(-6, 6)
        ax_2d.set_ylim(-6, 6)
        ax_2d.set_aspect('equal', adjustable='box')
        ax_2d.grid(True)
        
        ax_cs = fig.add_subplot(122)
        ax_cs.set_title("Configuration space")
        ax_cs.set_xlabel("q1 (rad)")
        ax_cs.set_ylabel("q2 (rad)")
        ax_cs.imshow(self.c_space_grid, origin='lower', extent=[-np.pi, np.pi, -np.pi, np.pi], cmap='Greys')
        
        for point in self.user_points:
            best_q = self.solve_ik(point)
            if best_q:
                ax_cs.plot(best_q[0], best_q[1], 'go', markersize=12, label='Goal')
        
        self.c_space_dot, = ax_cs.plot([], [], 'bo', markersize=8, label='Robot Config')
        ax_cs.legend()

        ani = FuncAnimation(fig, self.update, frames=100, interval=50, blit=False, repeat=False)

        plt.show()
    
    def show_robot_configuration(self):
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
    
        segments = self.robot.get_link_segments(self.q_end)
        for segment in segments:
            p1, p2 = segment
            ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [0, 0], marker='o', color='blue')
    
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_zlim(-1, 1)
        ax.set_xlim(-2, 5)
        ax.set_ylim(-4, 4)
        plt.show()

    def add_green_point(self):
        try:
            inp = input("Enter x,y for a green point (e.g. 1.5,2): ")
            parts = inp.split(',')
            if len(parts) == 2:
                x = float(parts[0].strip())
                y = float(parts[1].strip())
                self.user_points.append((x, y))
                print(f"Added point: ({x}, {y})")
            else:
                print("Invalid format. Please use x,y")
        except ValueError:
            print("Invalid input. Please enter numbers.")
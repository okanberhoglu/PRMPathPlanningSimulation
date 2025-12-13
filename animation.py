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
    def __init__(self, goal_point, robot: Robot, obstacles: list[Polygon], path=None):
        self.robot = robot
        self.obstacles = obstacles
        self.goal_point = goal_point
        self.path = path

    def update(self, frame):
        if self.path is not None and len(self.path) > 1:
            # Interpolate through path waypoints
            total_frames = 100
            segment_length = total_frames / (len(self.path) - 1)
            segment_idx = int(frame / segment_length)
            
            if segment_idx >= len(self.path) - 1:
                q_current = self.path[-1]
            else:
                alpha = (frame - segment_idx * segment_length) / segment_length
                q_current = (1 - alpha) * self.path[segment_idx] + alpha * self.path[segment_idx + 1]
        else:
            # Simple linear interpolation
            alpha = frame / 100.0
            if alpha > 1.0: alpha = 1.0
            q_current = (1 - alpha) * self.q_start + alpha * self.q_end
        
        segments = self.robot.get_link_segments(q_current)
        
        for line, segment in zip(self.lines_2d, segments):
            p1, p2 = segment
            line.set_data([p1[0], p2[0]], [p1[1], p2[1]])
            
        return self.lines_2d

    def animate(self, q_start, q_end):
        self.q_start = q_start
        self.q_end = q_end
        
        # Verify goal distance
        segments_end = self.robot.get_link_segments(q_end)
        end_effector_pos = segments_end[-1][1]
        goal_distance = np.linalg.norm(end_effector_pos - np.array(self.goal_point))
        print(f"Final end effector position: ({end_effector_pos[0]:.3f}, {end_effector_pos[1]:.3f})")
        print(f"Goal position: ({self.goal_point[0]:.3f}, {self.goal_point[1]:.3f})")
        print(f"Distance to goal: {goal_distance:.3f} units")
        
        fig = plt.figure(figsize=(12, 6))

        segments = self.robot.get_link_segments(q_start)

        ax_2d = fig.add_subplot(111)
        self.lines_2d = []
        for segment in segments:
            p1, p2 = segment
            line, = ax_2d.plot([p1[0], p2[0]], [p1[1], p2[1]], color='blue', linewidth=2)
            self.lines_2d.append(line)

        for obstacle in self.obstacles:
            ax_2d.add_patch(obstacle)

        # Plot the planned path if available
        if self.path is not None and len(self.path) > 1:
            path_positions = []
            for q in self.path:
                segments = self.robot.get_link_segments(q)
                end_effector_pos = segments[-1][1]  # End of last link
                path_positions.append(end_effector_pos)
            
            path_positions = np.array(path_positions)
            ax_2d.plot(path_positions[:, 0], path_positions[:, 1], 
                      'r--', linewidth=1.5, alpha=0.7, label='Planned Path')
            ax_2d.plot(path_positions[:, 0], path_positions[:, 1], 
                      'ro', markersize=4, alpha=0.5)

        ax_2d.plot(self.goal_point[0], self.goal_point[1], 'go', markersize=10, label='Goal')
        ax_2d.legend()

        ax_2d.set_title("Workspace")
        ax_2d.set_xlabel('X')
        ax_2d.set_ylabel('Y')
        ax_2d.set_xlim(-6, 6)
        ax_2d.set_ylim(-6, 6)
        ax_2d.set_aspect('equal', adjustable='box')
        ax_2d.grid(True)

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
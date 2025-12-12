import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from robot import Robot

class Animation:
    def __init__(self, robot: Robot):
        self.robot = robot
    
    def update(self, frame):
        alpha = frame / 100.0
        if alpha > 1.0: alpha = 1.0
        q_current = (1 - alpha) * self.q_start + alpha * self.q_end
        
        segments = self.robot.get_link_segments(q_current)
        
        for line, segment in zip(self.lines, segments):
            p1, p2 = segment
            line.set_data([p1[0], p2[0]], [p1[1], p2[1]])
            line.set_3d_properties([0, 0])
        return self.lines

    def animate(self, q_start, q_end):
        self.q_start = q_start
        self.q_end = q_end
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        segments = self.robot.get_link_segments(q_start)
        self.lines = []
        for segment in segments:
            p1, p2 = segment
            line, = ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [0, 0], color='blue', linewidth=2)
            self.lines.append(line)

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_zlim(-1, 1)
        ax.set_xlim(-2, 5)
        ax.set_ylim(-4, 4)

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
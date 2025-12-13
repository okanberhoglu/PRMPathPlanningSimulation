import numpy as np
from robot import Robot
from animation import Animation
from obstacle import Obstacle
from matplotlib.patches import Polygon
import math
    
def main():
    prrr_robot = Robot()

    user_point = None
    try:
        inp = input("Enter x,y for a green point (e.g. 1.5,2): ")
        parts = inp.split(',')
        if len(parts) == 2:
            x = float(parts[0].strip())
            y = float(parts[1].strip())
            user_point = (x, y)
            print(f"Point accepted: {user_point}")
        else:
            print("Invalid format. Using default.")
    except ValueError:
        print("Invalid input. Using default.")

    avoid = [user_point] if user_point else []

    obstacles = Obstacle().generate_obstacles(num_obstacles=5, x_lim=(-6, 6), y_lim=(-6, 6), avoid_points=avoid)

    animation = Animation(prrr_robot, obstacles)
    if user_point:
        animation.user_points.append(user_point)

    q_start = np.array([0.0, 0.0, 0.0, 0.0])
    q_end = np.array([1.0, math.pi/2, -math.pi/2, 0.0])

    animation.animate(q_start, q_end)

if __name__ == '__main__':
    main()

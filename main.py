import numpy as np
from robot import Robot
from animation import Animation
from obstacle import Obstacle
from planner import Planner
from matplotlib.patches import Polygon
    
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

    q_start = np.array([0.0, 0.0, 0.0, 0.0])
    
    planner = Planner(prrr_robot, obstacles, q_start)
    path, q_end = planner.plan(user_point)
    
    if path is None or q_end is None:
        print("Planning failed!")
    else:
        print(f"Goal configuration: q_end = {q_end}")
        
        # Visualize the PRM roadmap and planned path
        planner.visualize_roadmap(path_indices=planner.path_indices)

        animation = Animation(user_point, prrr_robot, obstacles, path=path)
        animation.animate(q_start, q_end)

if __name__ == '__main__':
    main()

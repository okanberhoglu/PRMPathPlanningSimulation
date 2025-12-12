import numpy as np
from robot import Robot
from animation import Animation
import math
    
def main():
    prrr_robot = Robot()
    animation = Animation(prrr_robot)
    q_start = np.array([0.0, 0.0, 0.0, 0.0])
    q_end = np.array([1.0, math.pi/2, -math.pi/2, 0.0])

    animation.animate(q_start, q_end)

if __name__ == '__main__':
    main()

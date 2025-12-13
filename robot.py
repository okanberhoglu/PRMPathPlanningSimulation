import numpy as np

class Robot:
    def __init__(self, link_lengths = 1.0):
        self.link_lengths = link_lengths

    def __translate(self, x, y):
        matrix = np.array([
            [1, 0, x],
            [0, 1, y],
            [0, 0, 1],
        ])
        return matrix
    
    def __rotate_z(self, th):
        matrix = np.array([
            [np.cos(th), -np.sin(th), 0],
            [np.sin(th), np.cos(th), 0],
            [0, 0, 1],
        ])
        return matrix
    
    def forward_kinematics(self, q):
        q1, q2, q3, q4 = q

        origin = np.array([0, 0, 1])
        
        H0 = np.eye(3)
        p0 = (H0 @ origin)[:2]

        H1 = H0 @ self._Robot__translate(q1, 0)
        p1 = (H1 @ origin)[:2]

        T2 = self._Robot__rotate_z(q2) @ self._Robot__translate(self.link_lengths, 0)
        H2 = H1 @ T2
        p2 = (H2 @ origin)[:2]

        T3 = self._Robot__rotate_z(q3) @ self._Robot__translate(self.link_lengths, 0)
        H3 = H2 @ T3
        p3 = (H3 @ origin)[:2]

        T4 = self._Robot__rotate_z(q4) @ self._Robot__translate(self.link_lengths, 0)
        H4 = H3 @ T4
        p4 = (H4 @ origin)[:2]

        pts = np.array([p0, p1, p2, p3, p4])

        return pts
        

    def get_link_segments(self, q):
        pts = self.forward_kinematics(q)
        segments = []
        for i in range(4):
            segments.append((pts[i], pts[i+1]))
        return segments
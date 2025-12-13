import numpy as np
from matplotlib.patches import Polygon
import random

class Obstacle:
    def generate_obstacles(self, num_obstacles, x_lim, y_lim, min_radius=1.0, max_radius=2.0, avoid_points=None):
        obstacles = []
        obstacle_vertices_list = []
        
        if avoid_points is None:
            avoid_points = []
        
        robot_start_p1 = np.array([0, 0])
        robot_start_p2 = np.array([3, 0])

        count = 0
        max_attempts = num_obstacles * 200
        attempts = 0

        while count < num_obstacles and attempts < max_attempts:
            attempts += 1
            
            cx = random.uniform(x_lim[0] + max_radius, x_lim[1] - max_radius)
            cy = random.uniform(y_lim[0] + max_radius, y_lim[1] - max_radius)
            
            poly_points = self.generate_random_polygon_points(cx, cy, radius_range=(min_radius, max_radius))
            
            if self.check_collision(poly_points, robot_start_p1, robot_start_p2):
                continue

            # Check collision with avoid_points
            point_collision = False
            for point in avoid_points:
                if self.is_point_in_polygon(point, poly_points):
                    point_collision = True
                    break
            if point_collision:
                continue

            overlap = False
            for existing_verts in obstacle_vertices_list:
                if self.check_polygon_collision(poly_points, existing_verts):
                    overlap = True
                    break
            
            if not overlap:
                poly_patch = Polygon(poly_points, closed=True, fill=True, color='gray')
                obstacles.append(poly_patch)
                obstacle_vertices_list.append(poly_points)
                count += 1
                
        return obstacles

    def generate_random_polygon_points(self, cx, cy, num_vertices_range=(3, 6), radius_range=(0.5, 1.5)):
        num_vertices = random.randint(*num_vertices_range)
        angles = sorted([random.uniform(0, 2 * np.pi) for _ in range(num_vertices)])
        
        points = []
        min_r, max_r = radius_range
        for angle in angles:
            r = random.uniform(min_r, max_r)
            x = cx + r * np.cos(angle)
            y = cy + r * np.sin(angle)
            points.append([x, y])
            
        return np.array(points)

    def check_collision(self, poly_points, p1, p2):
        num_points = len(poly_points)
        for i in range(num_points):
            v1 = poly_points[i]
            v2 = poly_points[(i + 1) % num_points]
            if self.segments_intersect(p1, p2, v1, v2):
                return True
                
        if self.is_point_in_polygon(p1, poly_points) or self.is_point_in_polygon(p2, poly_points):
            return True
            
        return False

    def check_polygon_collision(self, poly1_points, poly2_points):
        n1 = len(poly1_points)
        n2 = len(poly2_points)
        
        for i in range(n1):
            p1a = poly1_points[i]
            p1b = poly1_points[(i + 1) % n1]
            
            for j in range(n2):
                p2a = poly2_points[j]
                p2b = poly2_points[(j + 1) % n2]
                
                if self.segments_intersect(p1a, p1b, p2a, p2b):
                    return True
                    
        for p in poly1_points:
            if self.is_point_in_polygon(p, poly2_points):
                return True
                
        for p in poly2_points:
            if self.is_point_in_polygon(p, poly1_points):
                return True
                
        return False

    def segments_intersect(self, p1, p2, p3, p4):
        d1 = np.cross(p2 - p1, p3 - p1)
        d2 = np.cross(p2 - p1, p4 - p1)
        d3 = np.cross(p4 - p3, p1 - p3)
        d4 = np.cross(p4 - p3, p2 - p3)
        
        if ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and \
        ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)):
            return True
        
        return False

    def is_point_in_polygon(self, point, poly_points):
        x, y = point
        n = len(poly_points)
        inside = False
        p1x, p1y = poly_points[0]
        for i in range(n + 1):
            p2x, p2y = poly_points[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        return inside

import numpy as np
import heapq
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from obstacle import Obstacle
from robot import Robot

class Planner:
    def __init__(self, robot: Robot, obstacles: list, q_start: np.ndarray):
        self.robot = robot
        self.obstacles = obstacles
        self.q_start = q_start
        self.obs_helper = Obstacle()
        
        self.q1_min, self.q1_max = -1.0, 1.0
        self.q2_min, self.q2_max = -np.pi, np.pi
        self.q3_min, self.q3_max = -np.pi, np.pi
        self.q4_min, self.q4_max = -np.pi, np.pi
        
        self.num_samples = 100 
        self.k_neighbors = 25   
        
        self.goal_tolerance = 0.02 
        
        self.nodes = []  
        self.edges = {} 
        
    def solve_ik(self, target_point, num_attempts=1000):
        best_q = None
        min_dist = float('inf')
        
        for _ in range(num_attempts):
            q = self._sample_random_config()
            
            if self._is_collision(q):
                continue
                
            segments = self.robot.get_link_segments(q)
            eff_pos = segments[-1][1] 
            
            dist = np.linalg.norm(eff_pos - np.array(target_point))
            if dist < min_dist:
                min_dist = dist
                best_q = q
                
                if dist < self.goal_tolerance:
                    break
        
        print(f"IK solution distance to target: {min_dist:.3f}")
        
        if min_dist > self.goal_tolerance:
            print(f"WARNING: Goal is not reachable within tolerance ({self.goal_tolerance:.2f} units)")
            print(f"Best achievable distance: {min_dist:.3f} units")
        
        return best_q
    
    def _sample_random_config(self):
        q1 = np.random.uniform(self.q1_min, self.q1_max)
        q2 = np.random.uniform(self.q2_min, self.q2_max)
        q3 = np.random.uniform(self.q3_min, self.q3_max)
        q4 = np.random.uniform(self.q4_min, self.q4_max)
        return np.array([q1, q2, q3, q4])
    
    def _is_collision(self, q):
        segments = self.robot.get_link_segments(q)
        for obstacle in self.obstacles:
            poly_points = obstacle.get_xy()
            for segment in segments:
                p1, p2 = segment
                if self.obs_helper.check_collision(poly_points, p1, p2):
                    return True
        return False
    
    def _config_distance(self, q1, q2):
        d1 = abs(q1[0] - q2[0])
        
        d2 = min(abs(q1[1] - q2[1]), 2*np.pi - abs(q1[1] - q2[1]))
        d3 = min(abs(q1[2] - q2[2]), 2*np.pi - abs(q1[2] - q2[2]))
        d4 = min(abs(q1[3] - q2[3]), 2*np.pi - abs(q1[3] - q2[3]))
        
        return np.sqrt(d1**2 + d2**2 + d3**2 + d4**2)
    
    def _interpolate_config(self, q1, q2, alpha):
        q_interp = np.zeros(4)
        
        q_interp[0] = (1 - alpha) * q1[0] + alpha * q2[0]
        for i in range(1, 4):
            diff = q2[i] - q1[i]
            if abs(diff) > np.pi:
                if diff > 0:
                    diff = diff - 2*np.pi
                else:
                    diff = diff + 2*np.pi
            q_interp[i] = q1[i] + alpha * diff
            q_interp[i] = np.arctan2(np.sin(q_interp[i]), np.cos(q_interp[i]))
            
        return q_interp
    
    def _is_edge_collision_free(self, q1, q2, num_checks=10):
        for i in range(num_checks + 1):
            alpha = i / num_checks
            q_interp = self._interpolate_config(q1, q2, alpha)
            if self._is_collision(q_interp):
                return False
        return True
    
    def _build_roadmap(self):
        print(f"Building PRM roadmap with {self.num_samples} samples...")
        
        self.nodes = [self.q_start.copy()]
        self.edges = {0: []}
        
        attempts = 0
        max_attempts = self.num_samples * 10
        
        while len(self.nodes) < self.num_samples and attempts < max_attempts:
            attempts += 1
            q = self._sample_random_config()
            
            if not self._is_collision(q):
                node_idx = len(self.nodes)
                self.nodes.append(q)
                self.edges[node_idx] = []
        
        print(f"Sampled {len(self.nodes)} collision-free configurations")
        
        print("Connecting nodes...")
        for i in range(len(self.nodes)):
            distances = []
            for j in range(len(self.nodes)):
                if i != j:
                    dist = self._config_distance(self.nodes[i], self.nodes[j])
                    distances.append((dist, j))
            
            distances.sort()
            neighbors = distances[:self.k_neighbors]
            
            for dist, j in neighbors:
                if j not in self.edges[i] and i not in self.edges[j]:
                    if self._is_edge_collision_free(self.nodes[i], self.nodes[j]):
                        self.edges[i].append(j)
                        self.edges[j].append(i)
        
        total_edges = sum(len(neighbors) for neighbors in self.edges.values()) // 2
        print(f"Built roadmap with {len(self.nodes)} nodes and {total_edges} edges")
    
    def _find_nearest_node(self, q):
        min_dist = float('inf')
        min_dist = float('inf')
        nearest_idx = 0
        
        for i, node in enumerate(self.nodes):
            dist = self._config_distance(q, node)
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i
                
        return nearest_idx, min_dist
    
    def _dijkstra(self, start_idx, goal_idx):
        pq = [(0, start_idx)]
        distances = {start_idx: 0}
        came_from = {}
        visited = set()
        
        while pq:
            current_dist, current = heapq.heappop(pq)
            
            if current in visited:
                continue
            visited.add(current)
            
            if current == goal_idx:
                path_indices = []
                while current in came_from:
                    path_indices.append(current)
                    current = came_from[current]
                path_indices.append(start_idx)
                path_indices.reverse()
                return path_indices
            
            for neighbor in self.edges[current]:
                if neighbor not in visited:
                    edge_dist = self._config_distance(self.nodes[current], self.nodes[neighbor])
                    new_dist = current_dist + edge_dist
                    
                    if neighbor not in distances or new_dist < distances[neighbor]:
                        distances[neighbor] = new_dist
                        came_from[neighbor] = current
                        heapq.heappush(pq, (new_dist, neighbor))
        
        return None
    
    def plan(self, goal_point):
        q_end = self.solve_ik(goal_point)
        
        if q_end is None:
            print("IK failed: Could not find configuration to reach goal point")
            return None, None
        
        print(f"Goal configuration: {q_end}")
        
        self._build_roadmap()
        
        goal_idx = len(self.nodes)
        self.nodes.append(q_end)
        self.edges[goal_idx] = []
        
        distances = []
        for i in range(len(self.nodes) - 1):  
            dist = self._config_distance(self.nodes[i], q_end)
            distances.append((dist, i))
        
        distances.sort()
        for dist, i in distances[:self.k_neighbors]:
            if self._is_edge_collision_free(self.nodes[i], q_end):
                self.edges[i].append(goal_idx)
                self.edges[goal_idx].append(i)
        
        path_indices = self._dijkstra(0, goal_idx)  
        
        if path_indices is None:
            print("No path found in roadmap!")
            return None, None
        
        path = [self.nodes[i] for i in path_indices]
        
        self.path_indices = path_indices
        
        print(f"Path found with {len(path)} waypoints")
        return path, q_end
    
    def visualize_roadmap(self, path_indices=None):
        if len(self.nodes) == 0:
            print("No roadmap to visualize!")
            return
        
        fig = plt.figure(figsize=(14, 6))
        
        nodes_array = np.array(self.nodes)
        q1_vals = nodes_array[:, 0]
        q2_vals = nodes_array[:, 1]
        q3_vals = nodes_array[:, 2]
        q4_vals = nodes_array[:, 3]
        
        ax1 = fig.add_subplot(121)
        ax1.set_title("Configuration Space: (q1, q2) projection")
        ax1.set_xlabel("q1 (m)")
        ax1.set_ylabel("q2 (rad)")
        
        for node_idx, neighbors in self.edges.items():
            for neighbor_idx in neighbors:
                if node_idx < neighbor_idx:  
                    ax1.plot([q1_vals[node_idx], q1_vals[neighbor_idx]],
                            [q2_vals[node_idx], q2_vals[neighbor_idx]],
                            'b-', alpha=0.2, linewidth=0.5)
        
        ax1.scatter(q1_vals, q2_vals, c='lightblue', s=10, alpha=0.6, label='Roadmap nodes')
        
        ax1.scatter(q1_vals[0], q2_vals[0], c='green', s=100, marker='o', 
                   edgecolors='black', linewidths=2, label='Start', zorder=5)
        if len(self.nodes) > 1:
            ax1.scatter(q1_vals[-1], q2_vals[-1], c='red', s=100, marker='*', 
                       edgecolors='black', linewidths=2, label='Goal', zorder=5)
        
        if path_indices is not None and len(path_indices) > 1:
            path_q1 = [q1_vals[i] for i in path_indices]
            path_q2 = [q2_vals[i] for i in path_indices]
            ax1.plot(path_q1, path_q2, 'r-', linewidth=2.5, alpha=0.8, label='Planned path')
            ax1.scatter(path_q1, path_q2, c='red', s=50, zorder=4)
        
        ax1.set_xlim(self.q1_min - 0.1, self.q1_max + 0.1)
        ax1.set_ylim(self.q2_min - 0.2, self.q2_max + 0.2)
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        ax2 = fig.add_subplot(122)
        ax2.set_title("Configuration Space: (q3, q4) projection")
        ax2.set_xlabel("q3 (rad)")
        ax2.set_ylabel("q4 (rad)")
        
        for node_idx, neighbors in self.edges.items():
            for neighbor_idx in neighbors:
                if node_idx < neighbor_idx:
                    ax2.plot([q3_vals[node_idx], q3_vals[neighbor_idx]],
                            [q4_vals[node_idx], q4_vals[neighbor_idx]],
                            'b-', alpha=0.2, linewidth=0.5)
        
        ax2.scatter(q3_vals, q4_vals, c='lightblue', s=10, alpha=0.6, label='Roadmap nodes')
        
        ax2.scatter(q3_vals[0], q4_vals[0], c='green', s=100, marker='o',
                   edgecolors='black', linewidths=2, label='Start', zorder=5)
        if len(self.nodes) > 1:
            ax2.scatter(q3_vals[-1], q4_vals[-1], c='red', s=100, marker='*',
                       edgecolors='black', linewidths=2, label='Goal', zorder=5)
        
        if path_indices is not None and len(path_indices) > 1:
            path_q3 = [q3_vals[i] for i in path_indices]
            path_q4 = [q4_vals[i] for i in path_indices]
            ax2.plot(path_q3, path_q4, 'r-', linewidth=2.5, alpha=0.8, label='Planned path')
            ax2.scatter(path_q3, path_q4, c='red', s=50, zorder=4)
        
        ax2.set_xlim(self.q3_min - 0.2, self.q3_max + 0.2)
        ax2.set_ylim(self.q4_min - 0.2, self.q4_max + 0.2)
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()
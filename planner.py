import numpy as np
import heapq
import scipy.interpolate as si

class AStarPlanner:
    
    def __costmap(self, data, width, height, expansion_size=0):
        data = np.array(data).reshape(height, width)
        
        binary_map = np.zeros_like(data)
        binary_map[data == 100] = 1
        
        wall = np.where(binary_map == 1)
        for i in range(-1, 2):  # -1, 0, 1
            for j in range(-1, 2):  # -1, 0, 1
                if i == 0 and j == 0:
                    continue
                x = wall[0] + i
                y = wall[1] + j
                x = np.clip(x, 0, height - 1)
                y = np.clip(y, 0, width - 1)
                binary_map[x, y] = 1
        
        if expansion_size > 0:
            wall = np.where(binary_map == 1)
            for i in range(-expansion_size, expansion_size + 1):
                for j in range(-expansion_size, expansion_size + 1):
                    if i == 0 and j == 0:
                        continue
                    x = wall[0] + i
                    y = wall[1] + j
                    x = np.clip(x, 0, height - 1)
                    y = np.clip(y, 0, width - 1)
                    binary_map[x, y] = 1
        
        return binary_map
    
    def __heuristic(self, a, b):
        return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)
    
    def __astar(self, array, start, goal):
        neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
        close_set = set()
        came_from = {}
        gscore = {start:0}
        fscore = {start:self.__heuristic(start, goal)}
        oheap = []
        heapq.heappush(oheap, (fscore[start], start))
        while oheap:
            current = heapq.heappop(oheap)[1]
            if current == goal:
                data = []
                while current in came_from:
                    data.append(current)
                    current = came_from[current]
                data = data + [start]
                data = data[::-1]
                return data
            close_set.add(current)
            for i, j in neighbors:
                neighbor = current[0] + i, current[1] + j
                tentative_g_score = gscore[current] + self.__heuristic(current, neighbor)
                if 0 <= neighbor[0] < array.shape[0]:
                    if 0 <= neighbor[1] < array.shape[1]:                
                        if array[neighbor[0]][neighbor[1]] == 1:
                            continue
                    else:
                        # array bound y walls
                        continue
                else:
                    # array bound x walls
                    continue
                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                    continue
                if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = tentative_g_score + self.__heuristic(neighbor, goal)
                    heapq.heappush(oheap, (fscore[neighbor], neighbor))
        if goal not in came_from:
            closest_node = None
            closest_dist = float('inf')
            for node in close_set:
                dist = self.__heuristic(node, goal)
                if dist < closest_dist:
                    closest_node = node
                    closest_dist = dist
            if closest_node is not None:
                data = []
                while closest_node in came_from:
                    data.append(closest_node)
                    closest_node = came_from[closest_node]
                data = data + [start]
                data = data[::-1]
                return data
        return False

    def plan(self):
        pass
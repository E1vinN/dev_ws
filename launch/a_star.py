import heapq

class AStarPlanner:
    def __init__(self, grid):
        self.grid = grid
        self.open_list = []
        self.closed_list = set()
        self.start = None
        self.goal = None

    def heuristic(self, node, goal):
        return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

    def get_neighbors(self, node):
        neighbors = [(node[0] + dx, node[1] + dy) for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]]
        return [n for n in neighbors if 0 <= n[0] < len(self.grid) and 0 <= n[1] < len(self.grid[0]) and self.grid[n[0]][n[1]] == 0]

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def plan(self, start, goal):
        self.start = start
        self.goal = goal
        heapq.heappush(self.open_list, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while self.open_list:
            current = heapq.heappop(self.open_list)[1]

            if current == goal:
                return self.reconstruct_path(came_from, current)

            self.closed_list.add(current)

            for neighbor in self.get_neighbors(current):
                if neighbor in self.closed_list:
                    continue
                tentative_g_score = g_score[current] + 1

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    heapq.heappush(self.open_list, (f_score[neighbor], neighbor))

        return []


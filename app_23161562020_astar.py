import heapq
import time
import random

dirs = [(-1, 0), (1, 0), (0, -1), (0, 1)]

def manhattan(p1, p2):
    return abs(p1[0]-p2[0]) + abs(p1[1]-p2[1])

def generate_grid(n_nodes, n_obstacles):
    side = int(n_nodes ** 0.5)
    grid = [['.' for _ in range(side)] for _ in range(side)]
    grid[0][0] = 'S'
    grid[side - 1][side - 1] = 'G'

    placed = 0
    while placed < n_obstacles:
        x, y = random.randint(0, side-1), random.randint(0, side-1)
        if grid[x][y] == '.' and (x, y) not in [(0,0), (side-1, side-1)]:
            grid[x][y] = '#'
            placed += 1
    return grid

def find_pos(grid, char):
    for i, row in enumerate(grid):
        for j, val in enumerate(row):
            if val == char:
                return (i, j)

def reconstruct_path(came_from, current):
    path = []
    while current in came_from:
        path.append(current)
        current = came_from[current]
    return path[::-1]

def astar_search(grid):
    start = find_pos(grid, 'S')
    goal = find_pos(grid, 'G')
    frontier = []
    came_from = {}
    cost_so_far = {start: 0}
    heapq.heappush(frontier, (0 + manhattan(start, goal), start))

    start_time = time.time()
    while frontier:
        _, current = heapq.heappop(frontier)
        if current == goal:
            path = reconstruct_path(came_from, current)
            return len(path), (time.time() - start_time) * 1000

        for dx, dy in dirs:
            nx, ny = current[0] + dx, current[1] + dy
            next_node = (nx, ny)
            if 0 <= nx < len(grid) and 0 <= ny < len(grid[0]) and grid[nx][ny] != '#':
                new_cost = cost_so_far[current] + 1
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + manhattan(next_node, goal)
                    heapq.heappush(frontier, (priority, next_node))
                    came_from[next_node] = current
    return 0, (time.time() - start_time) * 1000

# Eksperimen
experiments = [
    (5000, 10),
    (50000, 100),
    (500000, 1000),
    (5000000, 10000),
    (50000000, 100000)
]

print(f"{'Experiment':<12} {'A* Time (ms)':<15} {'A* Path Length':<20}")
for idx, (nodes, obstacles) in enumerate(experiments, 1):
    grid = generate_grid(nodes, obstacles)
    path_len, exec_time = astar_search(grid)
    print(f"# {idx:<10} {exec_time:<15.2f} {path_len:<20}")

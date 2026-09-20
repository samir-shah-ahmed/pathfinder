"""Conservative grid A* baseline; output is NOT a rideable trajectory."""

import heapq

import numpy as np


def inflate(grid, radius_cells):
    grid = np.asarray(grid)
    if grid.ndim != 2 or radius_cells < 0:
        raise ValueError("Expected 2-D grid and nonnegative radius")
    blocked = (grid < 0) | (grid >= 50)
    result = blocked.copy()
    for y, x in np.argwhere(blocked):
        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy <= radius_cells**2:
                    yy, xx = y + dy, x + dx
                    if 0 <= yy < grid.shape[0] and 0 <= xx < grid.shape[1]:
                        result[yy, xx] = True
    return result


def astar(grid, start, goal, radius_cells=1):
    blocked = inflate(grid, radius_cells)
    height, width = blocked.shape

    def valid(c):
        return 0 <= c[0] < width and 0 <= c[1] < height and not blocked[c[1], c[0]]

    if not valid(start) or not valid(goal):
        return []
    queue, costs, parent = [(0, start)], {start: 0}, {}
    while queue:
        _, cell = heapq.heappop(queue)
        if cell == goal:
            path = [cell]
            while cell in parent:
                cell = parent[cell]
                path.append(cell)
            return path[::-1]
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nxt = cell[0] + dx, cell[1] + dy
            cost = costs[cell] + 1
            if valid(nxt) and cost < costs.get(nxt, float("inf")):
                costs[nxt], parent[nxt] = cost, cell
                heuristic = abs(nxt[0] - goal[0]) + abs(nxt[1] - goal[1])
                heapq.heappush(queue, (cost + heuristic, nxt))
    return []

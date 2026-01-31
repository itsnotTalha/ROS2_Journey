import numpy as np
from collections import deque

def shortest_path(grid, start, goal):
    rows = len(grid)
    cols = len(grid[0])

    q = deque()
    q.append(start)
    visited = set([start])
    parent = {start: None}

    directions = [(1,0), (0,1), (-1,0), (0,-1)]

    while q:
        r, c = q.popleft()

        if (r, c) == goal:
            # reconstruct path safely
            path = []
            cur = (r, c)
            while cur is not None:
                path.append(cur)
                cur = parent[cur]
            return path[::-1]

        for dr, dc in directions:
            nr, nc = r + dr, c + dc

            if 0 <= nr < rows and 0 <= nc < cols:
                if grid[nr][nc] != 1 and (nr, nc) not in visited:
                    visited.add((nr, nc))
                    parent[(nr, nc)] = (r, c)
                    q.append((nr, nc))

    return None


# GRID INITIALIZATION

arr = np.zeros((5, 5), dtype=int)

arr[0][2] = 1
arr[1][2] = 1
arr[1][1] = 1
arr[2][1] = 1
arr[4][0] = 1
arr[4][3] = 1

xy = list(map(int, input("Enter starting pos: ").split()))
ab = list(map(int, input("Enter target pos: ").split()))

start = tuple(xy)
goal = tuple(ab)

print("Grid (0=free, 1=obstacle):")
for row in arr:
    print(row)

# FIND SHORTEST PATH

path = shortest_path(arr, start, goal)

print("\nShortest path:", path)

# Print path in grid visually
print("\nPath visualization (- = path):")
visual = arr.astype(str)

if path:
    for r, c in path:
        visual[r][c] = '-'

    # mark start and goal
    visual[start[0]][start[1]] = 'S'
    visual[goal[0]][goal[1]] = 'G'

for row in visual:
    print(" ".join(row))

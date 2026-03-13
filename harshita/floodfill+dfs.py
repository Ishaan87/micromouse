import API
import sys
from collections import deque

def log(msg):
    sys.stderr.write(str(msg) + "\n")
    sys.stderr.flush()

WIDTH, HEIGHT = API.mazeWidth(), API.mazeHeight()
NORTH, EAST, SOUTH, WEST = 0, 1, 2, 3
DX, DY = [0, 1, 0, -1], [1, 0, -1, 0]
OPPOSITE = {NORTH: SOUTH, SOUTH: NORTH, EAST: WEST, WEST: EAST}
DIR_CHARS = ['n', 'e', 's', 'w']

GOALS = {(WIDTH//2, HEIGHT//2), (WIDTH//2-1, HEIGHT//2), 
         (WIDTH//2, HEIGHT//2-1), (WIDTH//2-1, HEIGHT//2-1)}

walls = [[set() for _ in range(HEIGHT)] for _ in range(WIDTH)]
visited = [[False] * HEIGHT for _ in range(WIDTH)]

# ===========================================================================
# LOGIC & NAVIGATION
# ===========================================================================

def run_flood_fill():
    dist = [[float('inf')] * HEIGHT for _ in range(WIDTH)]
    queue = deque()
    for gx, gy in GOALS:
        dist[gx][gy] = 0
        queue.append((gx, gy))

    while queue:
        x, y = queue.popleft()
        for d in range(4):
            if d in walls[x][y]: continue
            nx, ny = x + DX[d], y + DY[d]
            if 0 <= nx < WIDTH and 0 <= ny < HEIGHT:
                if dist[nx][ny] == float('inf'):
                    dist[nx][ny] = dist[x][y] + 1
                    queue.append((nx, ny))
    return dist

def sense_walls(state):
    x, y, f = state['x'], state['y'], state['facing']
    sensor_map = [('front', f), ('right', (f+1)%4), ('left', (f+3)%4)]
    for rel, abs_dir in sensor_map:
        has_wall = (API.wallFront() if rel == 'front' else 
                    API.wallRight() if rel == 'right' else API.wallLeft())
        if has_wall and abs_dir not in walls[x][y]:
            walls[x][y].add(abs_dir)
            API.setWall(x, y, DIR_CHARS[abs_dir])
            nx, ny = x + DX[abs_dir], y + DY[abs_dir]
            if 0 <= nx < WIDTH and 0 <= ny < HEIGHT:
                walls[nx][ny].add(OPPOSITE[abs_dir])
                API.setWall(nx, ny, DIR_CHARS[OPPOSITE[abs_dir]])

def move(target_dir, state):
    diff = (target_dir - state['facing']) % 4
    if diff == 1: API.turnRight()
    elif diff == 2: API.turnRight(); API.turnRight()
    elif diff == 3: API.turnLeft()
    API.moveForward()
    state['x'] += DX[target_dir]
    state['y'] += DY[target_dir]
    state['facing'] = target_dir
    visited[state['x']][state['y']] = True

# ===========================================================================
# MAIN RUN
# ===========================================================================

def main():
    state = {'x': 0, 'y': 0, 'facing': NORTH}
    visited[0][0] = True
    path_stack = [(0, 0)] # Stores the path for backtracking
    
    # --- PART A: FLOOD FILL TO GOAL ---
    log("Run 1: Outbound (Flood Fill)")
    while (state['x'], state['y']) not in GOALS:
        sense_walls(state)
        dist_map = run_flood_fill()
        
        # Display distances
        for x in range(WIDTH):
            for y in range(HEIGHT):
                API.setText(x, y, str(dist_map[x][y]))

        best_dir = None
        min_dist = float('inf')
        for d in range(4):
            if d in walls[state['x']][state['y']]: continue
            nx, ny = state['x'] + DX[d], state['y'] + DY[d]
            if 0 <= nx < WIDTH and 0 <= ny < HEIGHT:
                if dist_map[nx][ny] < min_dist:
                    min_dist = dist_map[nx][ny]
                    best_dir = d
        
        if best_dir is not None:
            move(best_dir, state)
            path_stack.append((state['x'], state['y']))
            API.setColor(state['x'], state['y'], 'G')
        else: break

    # --- PART B: DFS BACK TO START ---
    log("Run 1: Inbound (DFS Exploration)")
    
    while (state['x'], state['y']) != (0, 0):
        sense_walls(state)
        x, y = state['x'], state['y']
        
        # Look for ANY unvisited neighbor (Exploration Mode)
        found_unvisited = False
        for d in range(4):
            if d in walls[x][y]: continue
            nx, ny = x + DX[d], y + DY[d]
            if 0 <= nx < WIDTH and 0 <= ny < HEIGHT and not visited[nx][ny]:
                move(d, state)
                path_stack.append((state['x'], state['y']))
                API.setColor(state['x'], state['y'], 'B')
                found_unvisited = True
                break
        
        # If no unvisited neighbors, backtrack one step toward start
        if not found_unvisited:
            path_stack.pop() # Remove current cell
            if path_stack:
                tx, ty = path_stack[-1]
                # Determine direction to parent cell
                for d in range(4):
                    if x + DX[d] == tx and y + DY[d] == ty:
                        move(d, state)
                        break

    log("Back at start. Exploration complete.")

if __name__ == "__main__":
    main()
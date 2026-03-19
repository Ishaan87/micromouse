import sys
from collections import deque

# ─── Maze constants ──────────────────────────────────────────
MAZE_SIZE = 16

# ─── Direction constants ─────────────────────────────────────
NORTH = 0
EAST  = 1
SOUTH = 2
WEST  = 3

# ─── Robot state ─────────────────────────────────────────────
robot_x   = 0
robot_y   = 0
robot_dir = NORTH

# ─── Maze data ───────────────────────────────────────────────
walls   = [[[False] * 4 for _ in range(MAZE_SIZE)] for _ in range(MAZE_SIZE)]
visited = [[False]      * MAZE_SIZE                for _ in range(MAZE_SIZE)]

# ─── MMS API ─────────────────────────────────────────────────
def move_forward():
    print("moveForward", flush=True)
    input()

def turn_left():
    print("turnLeft", flush=True)
    input()

def turn_right():
    print("turnRight", flush=True)
    input()

def wall_front():
    print("wallFront", flush=True)
    return input() == "true"

def wall_left():
    print("wallLeft", flush=True)
    return input() == "true"

def wall_right():
    print("wallRight", flush=True)
    return input() == "true"

def log(text):
    print(f"log {text}", flush=True)

def set_color(x, y, color):
    print(f"setColor {x} {y} {color}", flush=True)

def set_text(x, y, text):
    print(f"setText {x} {y} {text}", flush=True)

# ─── Helpers ─────────────────────────────────────────────────
def is_goal(x, y):
    return x in [7, 8] and y in [7, 8]

def in_bounds(x, y):
    return 0 <= x < MAZE_SIZE and 0 <= y < MAZE_SIZE

def neighbour(x, y, direction):
    if direction == NORTH: return x,     y + 1
    if direction == EAST:  return x + 1, y
    if direction == SOUTH: return x,     y - 1
    if direction == WEST:  return x - 1, y

# ─── Sense walls around current cell ─────────────────────────
def sense_walls():
    front = wall_front()
    left  = wall_left()
    right = wall_right()

    dir_front = robot_dir
    dir_left  = (robot_dir - 1) % 4
    dir_right = (robot_dir + 1) % 4

    x, y = robot_x, robot_y

    for has_wall, direction in [(front, dir_front), (left, dir_left), (right, dir_right)]:
        if has_wall:
            walls[x][y][direction] = True
            nx, ny = neighbour(x, y, direction)
            if in_bounds(nx, ny):
                walls[nx][ny][(direction + 2) % 4] = True  # shared wall

# ─── Turn to face a direction ─────────────────────────────────
def face_direction(target_dir):
    global robot_dir
    turns = (target_dir - robot_dir) % 4
    if turns == 1:
        turn_right()
    elif turns == 2:
        turn_right()
        turn_right()
    elif turns == 3:
        turn_left()
    robot_dir = target_dir

# ─── Update position after moving ────────────────────────────
def update_position():
    global robot_x, robot_y
    robot_x, robot_y = neighbour(robot_x, robot_y, robot_dir)
    visited[robot_x][robot_y] = True
    set_color(robot_x, robot_y, "green")

# ─── BFS to find shortest path using known walls ─────────────
# This is the core BFS — finds path from current pos to goal
# Returns list of directions to follow
def bfs_find_path():
    start = (robot_x, robot_y)

    # each item in queue = (x, y, path taken to get here)
    queue   = deque()
    queue.append((start[0], start[1], []))

    seen = [[False] * MAZE_SIZE for _ in range(MAZE_SIZE)]
    seen[start[0]][start[1]] = True

    while queue:
        x, y, path = queue.popleft()

        # reached goal?
        if is_goal(x, y):
            return path

        # explore all 4 neighbours
        for direction in [NORTH, EAST, SOUTH, WEST]:
            if walls[x][y][direction]:
                continue  # wall here, skip

            nx, ny = neighbour(x, y, direction)

            if not in_bounds(nx, ny):
                continue  # out of bounds, skip

            if seen[nx][ny]:
                continue  # already visited in this BFS, skip

            seen[nx][ny] = True
            queue.append((nx, ny, path + [direction]))  # add direction to path

    return []  # no path found

# ─── Exploration phase — discover walls by moving around ──────
# Uses simple left hand rule to explore until goal found
def explore():
    log("Phase 1 — Exploring maze...")

    while not is_goal(robot_x, robot_y):
        sense_walls()

        x, y = robot_x, robot_y

        # try left → front → right → back (left hand rule)
        dir_left  = (robot_dir - 1) % 4
        dir_front = robot_dir
        dir_right = (robot_dir + 1) % 4
        dir_back  = (robot_dir + 2) % 4

        moved = False
        for direction in [dir_left, dir_front, dir_right, dir_back]:
            if not walls[x][y][direction]:
                nx, ny = neighbour(x, y, direction)
                if in_bounds(nx, ny):
                    face_direction(direction)
                    move_forward()
                    update_position()
                    moved = True
                    break

        if not moved:
            log("Stuck! No moves available")
            break

    log("Goal found during exploration!")

# ─── Speed run phase — follow BFS shortest path ───────────────
def speed_run(path):
    log(f"Phase 2 — Running shortest path ({len(path)} steps)...")

    # color the planned path yellow before running
    x, y = 0, 0
    for direction in path:
        nx, ny = neighbour(x, y, direction)
        set_color(nx, ny, "yellow")
        x, y = nx, ny

    # now actually follow the path
    for direction in path:
        face_direction(direction)
        move_forward()
        update_position()

    log("Speed run complete!")

# ─── Main ─────────────────────────────────────────────────────
def solve():
    global robot_x, robot_y, robot_dir

    # mark start cell
    visited[0][0] = True
    set_color(0, 0, "green")

    # Phase 1 — explore until goal is reached
    explore()

    # reset to start
    log("Resetting to start for speed run...")
    robot_x   = 0
    robot_y   = 0
    robot_dir = NORTH

    # Phase 2 — BFS on discovered walls to find shortest path
    path = bfs_find_path()

    if not path:
        log("No path found after exploration!")
        return

    log(f"BFS found path of length {len(path)}")

    # Phase 3 — run the shortest path
    speed_run(path)

    log("Done!")

solve()
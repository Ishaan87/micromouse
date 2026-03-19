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
walls    = [[[False] * 4 for _ in range(MAZE_SIZE)] for _ in range(MAZE_SIZE)]
distance = [[999]        * MAZE_SIZE                for _ in range(MAZE_SIZE)]
visited  = [[False]      * MAZE_SIZE                for _ in range(MAZE_SIZE)]

# ─── MMS API — correct format ─────────────────────────────────
def move_forward():
    print("moveForward", flush=True)
    input()  # waits for "ack"

def turn_left():
    print("turnLeft", flush=True)
    input()  # waits for "ack"

def turn_right():
    print("turnRight", flush=True)
    input()  # waits for "ack"

def wall_front():
    print("wallFront", flush=True)
    return input() == "true"

def wall_left():
    print("wallLeft", flush=True)
    return input() == "true"

def wall_right():
    print("wallRight", flush=True)
    return input() == "true"

def set_wall(x, y, direction):
    # direction must be n/e/s/w lowercase
    d_char = ['n', 'e', 's', 'w'][direction]
    print(f"setWall {x} {y} {d_char}", flush=True)
    # no response expected

def set_color(x, y, color):
    # color must be single letter: r=red, g=green, b=blue,
    # y=yellow, o=orange, c=cyan, p=pink, w=white etc.
    print(f"setColor {x} {y} {color}", flush=True)
    # no response expected

def set_text(x, y, text):
    print(f"setText {x} {y} {text}", flush=True)
    # no response expected

def clear_all_color():
    print("clearAllColor", flush=True)

def clear_all_text():
    print("clearAllText", flush=True)

# ─── log goes to stderr NOT stdout ───────────────────────────
def log(text):
    sys.stderr.write(f"{text}\n")
    sys.stderr.flush()

# ─── Helpers ─────────────────────────────────────────────────
def in_bounds(x, y):
    return 0 <= x < MAZE_SIZE and 0 <= y < MAZE_SIZE

def neighbour(x, y, direction):
    if direction == NORTH: return x,     y + 1
    if direction == EAST:  return x + 1, y
    if direction == SOUTH: return x,     y - 1
    if direction == WEST:  return x - 1, y

# ─── Goal cells — dynamic center ─────────────────────────────
def get_goal_cells():
    mid = MAZE_SIZE // 2
    if MAZE_SIZE % 2 == 0:
        return [
            (mid - 1, mid - 1),
            (mid - 1, mid),
            (mid,     mid - 1),
            (mid,     mid)
        ]
    return [(mid, mid)]

def is_goal(x, y):
    return (x, y) in get_goal_cells()

# ─── Boundary walls ───────────────────────────────────────────
def init_boundary_walls():
    for i in range(MAZE_SIZE):
        walls[i][0][SOUTH]            = True
        walls[i][MAZE_SIZE - 1][NORTH] = True
        walls[0][i][WEST]             = True
        walls[MAZE_SIZE - 1][i][EAST] = True

# ─── Floodfill ────────────────────────────────────────────────
def floodfill():
    for x in range(MAZE_SIZE):
        for y in range(MAZE_SIZE):
            distance[x][y] = 999

    queue = deque()

    for (gx, gy) in get_goal_cells():
        distance[gx][gy] = 0
        queue.append((gx, gy))

    while queue:
        x, y = queue.popleft()
        for d in [NORTH, EAST, SOUTH, WEST]:
            if walls[x][y][d]:
                continue
            nx, ny = neighbour(x, y, d)
            if not in_bounds(nx, ny):
                continue
            if distance[nx][ny] > distance[x][y] + 1:
                distance[nx][ny] = distance[x][y] + 1
                queue.append((nx, ny))

    # update display
    for x in range(MAZE_SIZE):
        for y in range(MAZE_SIZE):
            set_text(x, y, str(distance[x][y]))

# ─── Sense walls ─────────────────────────────────────────────
def sense_walls():
    front = wall_front()
    left  = wall_left()
    right = wall_right()

    dir_front = robot_dir
    dir_left  = (robot_dir - 1) % 4
    dir_right = (robot_dir + 1) % 4

    x, y = robot_x, robot_y

    for has_wall, direction in [
        (front, dir_front),
        (left,  dir_left),
        (right, dir_right)
    ]:
        if has_wall:
            walls[x][y][direction] = True
            # show wall on display
            set_wall(x, y, direction)
            nx, ny = neighbour(x, y, direction)
            if in_bounds(nx, ny):
                opposite = (direction + 2) % 4
                walls[nx][ny][opposite] = True
                set_wall(nx, ny, opposite)

# ─── Turn to face direction ───────────────────────────────────
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

# ─── Update position ─────────────────────────────────────────
def update_position():
    global robot_x, robot_y
    robot_x, robot_y = neighbour(robot_x, robot_y, robot_dir)
    visited[robot_x][robot_y] = True
    set_color(robot_x, robot_y, "g")  # green = visited

# ─── Best direction ───────────────────────────────────────────
def best_direction():
    x, y    = robot_x, robot_y
    best_dist = 999
    best_dir  = None

    # prefer unvisited cells first
    for d in [NORTH, EAST, SOUTH, WEST]:
        if walls[x][y][d]:
            continue
        nx, ny = neighbour(x, y, d)
        if not in_bounds(nx, ny):
            continue
        if not visited[nx][ny] and distance[nx][ny] < best_dist:
            best_dist = distance[nx][ny]
            best_dir  = d

    # fall back to any lowest distance cell
    if best_dir is None:
        for d in [NORTH, EAST, SOUTH, WEST]:
            if walls[x][y][d]:
                continue
            nx, ny = neighbour(x, y, d)
            if not in_bounds(nx, ny):
                continue
            if distance[nx][ny] < best_dist:
                best_dist = distance[nx][ny]
                best_dir  = d

    return best_dir

# ─── Main ─────────────────────────────────────────────────────
def solve():
    init_boundary_walls()

    visited[0][0] = True
    set_color(0, 0, "g")

    floodfill()

    log(f"Goal cells: {get_goal_cells()}")
    log("Starting Floodfill...")

    step = 0

    while not is_goal(robot_x, robot_y):
        sense_walls()
        floodfill()

        direction = best_direction()

        if direction is None:
            log("No valid move!")
            break

        face_direction(direction)
        move_forward()
        update_position()

        step += 1
        log(f"Step {step} → ({robot_x},{robot_y}) dist={distance[robot_x][robot_y]}")

    log(f"Goal reached in {step} steps!")

    for (gx, gy) in get_goal_cells():
        set_color(gx, gy, "r")  # red = goal

solve()
import API
import sys
from collections import deque


def log(msg):
    sys.stderr.write(str(msg) + "\n")
    sys.stderr.flush()



WIDTH  = API.mazeWidth()
HEIGHT = API.mazeHeight()


NORTH = 0
EAST  = 1
SOUTH = 2
WEST  = 3


DX = [0, 1,  0, -1]
DY = [1, 0, -1,  0]  

# Human-readable labels — used only for logging, never for logic.
DIR_NAMES = {NORTH: "NORTH", EAST: "EAST", SOUTH: "SOUTH", WEST: "WEST"}


def compute_goal_cells(w, h):
    cx = w // 2
    cy = h // 2
    goals = set()
    goals.add((cx, cy))
    if w % 2 == 0:
        goals.add((cx - 1, cy))
    if h % 2 == 0:
        goals.add((cx, cy - 1))
    if w % 2 == 0 and h % 2 == 0:
        goals.add((cx - 1, cy - 1))
    return goals

GOAL_CELLS = compute_goal_cells(WIDTH, HEIGHT)


walls = [[set() for _ in range(HEIGHT)] for _ in range(WIDTH)]

INF = float('inf')
flood = [[INF] * HEIGHT for _ in range(WIDTH)]


def init_border_walls():
    for x in range(WIDTH):
        for y in range(HEIGHT):
            if y == HEIGHT - 1: walls[x][y].add(NORTH)  # top row
            if y == 0:          walls[x][y].add(SOUTH)  # bottom row
            if x == WIDTH - 1:  walls[x][y].add(EAST)   # right column
            if x == 0:          walls[x][y].add(WEST)   # left column



def run_flood_fill():
    for x in range(WIDTH):
        for y in range(HEIGHT):
            flood[x][y] = INF

    queue = deque()
    for (gx, gy) in GOAL_CELLS:
        flood[gx][gy] = 0
        queue.append((gx, gy))


    while queue:
        x, y = queue.popleft()
        current_dist = flood[x][y]

        for direction in [NORTH, EAST, SOUTH, WEST]:

            if direction in walls[x][y]:
                continue

            nx = x + DX[direction]
            ny = y + DY[direction]


            if not (0 <= nx < WIDTH and 0 <= ny < HEIGHT):
                continue


            if flood[nx][ny] > current_dist + 1:
                flood[nx][ny] = current_dist + 1
                queue.append((nx, ny))


    update_display()



def update_display():
    for x in range(WIDTH):
        for y in range(HEIGHT):
            val = flood[x][y]
            API.setText(x, y, str(val) if val != INF else "?")

    for (gx, gy) in GOAL_CELLS:
        API.setColor(gx, gy, "c")  




def abs_direction(facing, relative):
    """Convert a relative direction ('front','left','right') to absolute."""
    if relative == 'front': return facing
    if relative == 'right': return (facing + 1) % 4
    if relative == 'left':  return (facing + 3) % 4

OPPOSITE = {NORTH: SOUTH, SOUTH: NORTH, EAST: WEST, WEST: EAST}

def record_wall(x, y, direction):
    """Store a wall and its mirror on the neighbouring cell. Returns True if new."""
    if direction in walls[x][y]:
        return False                          

    walls[x][y].add(direction)              

    nx = x + DX[direction]
    ny = y + DY[direction]
    if 0 <= nx < WIDTH and 0 <= ny < HEIGHT:
        walls[nx][ny].add(OPPOSITE[direction]) 

    return True                              


def sense_and_record_walls(x, y, facing):
    """
    Call all three wall sensors, convert to absolute directions,
    and record any new walls. Returns True if any NEW wall was found
    (which means we need to re-flood).
    """
    new_wall_found = False

    sensor_checks = [
        ('front', API.wallFront()),
        ('left',  API.wallLeft()),
        ('right', API.wallRight()),
    ]

    for relative, has_wall in sensor_checks:
        if has_wall:
            abs_dir = abs_direction(facing, relative)
            if record_wall(x, y, abs_dir):
                new_wall_found = True
                dir_char = ['n', 'e', 's', 'w'][abs_dir]
                API.setWall(x, y, dir_char)

    return new_wall_found



def turn_left(state):
    """Turn the mouse left: update facing, issue API command."""
    API.turnLeft()
    state['facing'] = (state['facing'] + 3) % 4

def turn_right(state):
    """Turn the mouse right: update facing, issue API command."""
    API.turnRight()
    state['facing'] = (state['facing'] + 1) % 4

def move_forward(state):
    """Move one cell forward: issue API command, then update (x,y)."""
    API.moveForward()
    state['x'] += DX[state['facing']]
    state['y'] += DY[state['facing']]
    API.setColor(state['x'], state['y'], "G")


def best_neighbor(x, y):
    """
    Returns the absolute direction of the open neighbour with the minimum
    flood value. Returns None if no open neighbour exists (shouldn't happen
    in a valid maze, but safe to handle).
    """
    best_dir = None
    best_val = INF

    for direction in [NORTH, EAST, SOUTH, WEST]:
        if direction in walls[x][y]:
            continue                       

        nx = x + DX[direction]
        ny = y + DY[direction]

        if not (0 <= nx < WIDTH and 0 <= ny < HEIGHT):
            continue

        if flood[nx][ny] < best_val:
            best_val = flood[nx][ny]
            best_dir = direction

    return best_dir


def turn_to_face(target_dir, state):
    current = state['facing']
    if current == target_dir:
        return                               

    right_turns = (target_dir - current) % 4

    if right_turns == 1:
        turn_right(state)
    elif right_turns == 2:
        turn_right(state)
        turn_right(state)
    elif right_turns == 3:                  
        turn_left(state)




def main():
    log("=== Flood Fill Micromouse ===")
    log(f"Maze: {WIDTH}x{HEIGHT}")
    log(f"Goal cells: {GOAL_CELLS}")
    init_border_walls()
    run_flood_fill()
    
    #defining mouse starting state here
    state = {'x': 0, 'y': 0, 'facing': NORTH}

    # Marking the starting cell.
    API.setColor(0, 0, "G")

    while (state['x'], state['y']) not in GOAL_CELLS:

        x, y = state['x'], state['y']

        new_wall = sense_and_record_walls(x, y, state['facing'])
        if new_wall:
            log(f"New wall found at ({x},{y}). Re-flooding...")
            run_flood_fill()

        target_dir = best_neighbor(x, y)

        if target_dir is None:
            log("ERROR: No open neighbour found. Maze may be unsolvable.")
            break

        log(f"At ({x},{y}) flood={flood[x][y]} → moving {DIR_NAMES[target_dir]}")

        turn_to_face(target_dir, state)
        move_forward(state)

        if API.wasReset():
            log("Reset detected. Restarting...")
            API.ackReset()
            API.clearAllColor()
            API.clearAllText()
            walls[:] = [[set() for _ in range(HEIGHT)] for _ in range(WIDTH)]
            init_border_walls()
            run_flood_fill()
            state = {'x': 0, 'y': 0, 'facing': NORTH}
            API.setColor(0, 0, "G")
            continue

    log(f"Goal reached at ({state['x']}, {state['y']})!")
    API.setColor(state['x'], state['y'], "Y")


if __name__ == "__main__":
    main()
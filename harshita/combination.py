import API
import sys
from collections import deque
import heapq

# ===========================================================================
# LOGGING
# ===========================================================================
# stdout is reserved for MMS communication. ALL debug output uses stderr.
def log(msg):
    sys.stderr.write(str(msg) + "\n")
    sys.stderr.flush()


# ===========================================================================
# CONSTANTS & DIRECTIONS
# ===========================================================================
WIDTH  = API.mazeWidth()
HEIGHT = API.mazeHeight()

NORTH, EAST, SOUTH, WEST = 0, 1, 2, 3

# When moving in a direction, how do coordinates change?
DX = [ 0,  1,  0, -1]   # NORTH, EAST, SOUTH, WEST
DY = [ 1,  0, -1,  0]

OPPOSITE  = {NORTH: SOUTH, SOUTH: NORTH, EAST: WEST, WEST: EAST}
DIR_CHAR  = ['n', 'e', 's', 'w']    # for API.setWall()
DIR_NAME  = ['N', 'E', 'S', 'W']    # for logging

# Goal = centre 2x2 cells for a standard even-dimension maze
def compute_goals(w, h):
    cx, cy = w // 2, h // 2
    goals = {(cx, cy)}
    if w % 2 == 0: goals.add((cx - 1, cy))
    if h % 2 == 0: goals.add((cx,     cy - 1))
    if w % 2 == 0 and h % 2 == 0: goals.add((cx - 1, cy - 1))
    return goals

GOAL_CELLS = compute_goals(WIDTH, HEIGHT)
INF = float('inf')


# ===========================================================================
# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  DATA STRUCTURE 1 — THE MAZE MAP                                        ║
# ║                                                                          ║
# ║  YOUR CONFUSION: "Where is the maze mapped and how?"                     ║
# ║                                                                          ║
# ║  THIS is the map. walls[x][y] is a Python set containing the absolute   ║
# ║  directions (NORTH/EAST/SOUTH/WEST) that have a confirmed wall for       ║
# ║  cell (x, y).                                                            ║
# ║                                                                          ║
# ║  Example: if walls[3][4] = {NORTH, WEST}, it means:                     ║
# ║    - there is a wall on the north side of cell (3,4)                     ║
# ║    - there is a wall on the west  side of cell (3,4)                     ║
# ║    - east and south sides are open (or unknown)                          ║
# ║                                                                          ║
# ║  The map starts completely empty (no walls known except the border).     ║
# ║  It fills itself one cell at a time as the mouse physically visits       ║
# ║  each cell and calls sense_and_record_walls().                           ║
# ║                                                                          ║
# ║  Why not store lefts/rights/distances?                                   ║
# ║  Because this format lets you instantly answer: "Is there a wall on      ║
# ║  the north side of (3,4)?" → just check NORTH in walls[3][4].           ║
# ║  A sequence of turns and distances cannot answer that question easily.   ║
# ╚══════════════════════════════════════════════════════════════════════════╝
walls = [[set() for _ in range(HEIGHT)] for _ in range(WIDTH)]


# ===========================================================================
# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  DATA STRUCTURE 2 — FLOOD FILL VALUES                                   ║
# ║                                                                          ║
# ║  YOUR CONFUSION: "When and HOW are cell values assigned?"                ║
# ║                                                                          ║
# ║  flood[x][y] stores the answer to: "Given all walls I currently know,   ║
# ║  how many cell-steps does it take to reach the goal from (x,y)?"         ║
# ║                                                                          ║
# ║  WHEN are they assigned?                                                 ║
# ║    1. Once at startup (with only border walls known)                     ║
# ║    2. Again every time a NEW wall is discovered                          ║
# ║    The function run_flood_fill() handles both cases identically.         ║
# ║                                                                          ║
# ║  HOW are they assigned? (see run_flood_fill() below for full detail)     ║
# ║    BFS from all goal cells simultaneously. Goal gets 0, its open         ║
# ║    neighbours get 1, their open neighbours get 2, etc.                   ║
# ║    "Open" means no wall between the two cells in walls[x][y].            ║
# ╚══════════════════════════════════════════════════════════════════════════╝
flood = [[INF] * HEIGHT for _ in range(WIDTH)]


# ===========================================================================
# DATA STRUCTURE 3 — VISITED CELLS (for A* exploration targeting)
# ===========================================================================
# visited[x][y] = True once the mouse has physically stood on cell (x,y)
# and sensed all four walls around it.
# This is how A* knows WHERE to send the mouse to gather new information.
visited = [[False] * HEIGHT for _ in range(WIDTH)]


# ===========================================================================
# BORDER WALLS (always true — maze has an outer boundary)
# ===========================================================================
def init_border_walls():
    for x in range(WIDTH):
        for y in range(HEIGHT):
            if y == HEIGHT - 1: walls[x][y].add(NORTH)
            if y == 0:          walls[x][y].add(SOUTH)
            if x == WIDTH - 1:  walls[x][y].add(EAST)
            if x == 0:          walls[x][y].add(WEST)


# ===========================================================================
# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  FLOOD FILL — THE VALUE ASSIGNMENT ENGINE                               ║
# ║                                                                          ║
# ║  This is the function that answers "HOW are values assigned?"            ║
# ║                                                                          ║
# ║  Step-by-step:                                                           ║
# ║  1. Reset every cell to infinity (unknown / unreachable for now)         ║
# ║  2. Put ALL goal cells in a queue with value 0                           ║
# ║  3. BFS outward — for each cell popped, look at all 4 neighbours.        ║
# ║     If no wall separates them AND neighbour hasn't been assigned yet,    ║
# ║     assign neighbour = current_value + 1, add to queue.                  ║
# ║  4. Continue until queue is empty — every reachable cell has a value.    ║
# ║                                                                          ║
# ║  WHEN is it called?                                                      ║
# ║  - Once at startup: gives Manhattan-like initial estimates                ║
# ║  - After every new wall discovery: corrects any estimates that assumed   ║
# ║    a path through that wall was available                                 ║
# ║  - Once before speed run: final accurate values on complete map           ║
# ╚══════════════════════════════════════════════════════════════════════════╝
def run_flood_fill():
    # Step 1: Reset all values
    for x in range(WIDTH):
        for y in range(HEIGHT):
            flood[x][y] = INF

    # Step 2: Seed queue with all goal cells at distance 0
    queue = deque()
    for (gx, gy) in GOAL_CELLS:
        flood[gx][gy] = 0
        queue.append((gx, gy))

    # Step 3: BFS outward
    while queue:
        x, y = queue.popleft()
        current_dist = flood[x][y]

        for direction in [NORTH, EAST, SOUTH, WEST]:
            # ← KEY: only cross a boundary if no wall blocks it
            if direction in walls[x][y]:
                continue

            nx, ny = x + DX[direction], y + DY[direction]
            if not (0 <= nx < WIDTH and 0 <= ny < HEIGHT):
                continue

            # Only update if we found a shorter path
            if flood[nx][ny] > current_dist + 1:
                flood[nx][ny] = current_dist + 1
                queue.append((nx, ny))

    # Update visual display in MMS
    for x in range(WIDTH):
        for y in range(HEIGHT):
            v = flood[x][y]
            API.setText(x, y, str(v) if v != INF else "?")
    for (gx, gy) in GOAL_CELLS:
        API.setColor(gx, gy, "c")


# ===========================================================================
# WALL SENSING & RECORDING
# ===========================================================================
# This is WHERE THE MAP GETS BUILT.
# At every cell the mouse visits, this function is called.
# It reads the three sensors, converts relative→absolute directions,
# and writes confirmed walls into walls[x][y].

def abs_direction(facing, relative):
    """Convert relative ('front','left','right') to absolute direction."""
    if relative == 'front': return facing
    if relative == 'right': return (facing + 1) % 4
    if relative == 'left':  return (facing + 3) % 4

def record_wall(x, y, direction):
    """
    Store a wall on both sides of the shared boundary.
    Returns True if this is NEW information (triggers re-flood).

    WHY both sides?
    A wall between (3,4) and (3,5) must be stored as:
      - NORTH wall of (3,4)   AND
      - SOUTH wall of (3,5)
    Otherwise the flood fill would think you can enter from one side
    but not the other — which is physically impossible.
    """
    if direction in walls[x][y]:
        return False  # already knew this wall

    walls[x][y].add(direction)
    API.setWall(x, y, DIR_CHAR[direction])  # show wall in MMS display

    nx, ny = x + DX[direction], y + DY[direction]
    if 0 <= nx < WIDTH and 0 <= ny < HEIGHT:
        walls[nx][ny].add(OPPOSITE[direction])
        API.setWall(nx, ny, DIR_CHAR[OPPOSITE[direction]])

    return True  # new wall found → caller must re-flood

def sense_and_record_walls(x, y, facing):
    """
    Call all three sensors. Convert to absolute. Record any new walls.
    Returns True if ANY new wall was found (meaning re-flood is needed).

    This is called at EVERY cell the mouse visits.
    Over time, as the mouse visits more cells, walls[][] fills up —
    and that growing walls[][] IS the maze map.
    """
    new_wall_found = False
    for relative, has_wall in [('front', API.wallFront()),
                                 ('left',  API.wallLeft()),
                                 ('right', API.wallRight())]:
        if has_wall:
            abs_dir = abs_direction(facing, relative)
            if record_wall(x, y, abs_dir):
                new_wall_found = True
    return new_wall_found


# ===========================================================================
# MOVEMENT HELPERS
# ===========================================================================
def turn_left(state):
    API.turnLeft()
    state['facing'] = (state['facing'] + 3) % 4

def turn_right(state):
    API.turnRight()
    state['facing'] = (state['facing'] + 1) % 4

def move_forward(state):
    API.moveForward()
    state['x'] += DX[state['facing']]
    state['y'] += DY[state['facing']]

def turn_to_face(target_dir, state):
    right_turns = (target_dir - state['facing']) % 4
    if right_turns == 1:   turn_right(state)
    elif right_turns == 2: turn_right(state); turn_right(state)
    elif right_turns == 3: turn_left(state)

def move_to_neighbor(target_x, target_y, state):
    """
    Turn to face (target_x, target_y) and move there.
    The target must be directly adjacent (one step away).
    """
    dx = target_x - state['x']
    dy = target_y - state['y']
    if   dx ==  1: target_dir = EAST
    elif dx == -1: target_dir = WEST
    elif dy ==  1: target_dir = NORTH
    else:          target_dir = SOUTH
    turn_to_face(target_dir, state)
    move_forward(state)


# ===========================================================================
# FLOOD FILL NAVIGATION — pick the best next cell to move to
# ===========================================================================
def best_flood_neighbor(x, y):
    """
    This is the core movement rule for both exploration and speed run.
    Look at all open neighbours, return the one with the lowest flood value.
    The mouse always steps DOWNHILL on the flood gradient toward the goal.
    """
    best_dir, best_val = None, INF
    for direction in [NORTH, EAST, SOUTH, WEST]:
        if direction in walls[x][y]:
            continue
        nx, ny = x + DX[direction], y + DY[direction]
        if not (0 <= nx < WIDTH and 0 <= ny < HEIGHT):
            continue
        if flood[nx][ny] < best_val:
            best_val = flood[nx][ny]
            best_dir = direction
    return best_dir


# ===========================================================================
# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  A* — EXPLORATION PATHFINDER                                            ║
# ║                                                                          ║
# ║  WHAT A* DOES HERE (not what you might expect):                         ║
# ║  A* is NOT used to move the mouse cell-by-cell toward the goal.         ║
# ║  Flood fill handles that. A* answers a different question:               ║
# ║                                                                          ║
# ║  "The mouse has finished one run. It needs to go back and visit          ║
# ║  unexplored cells to fill gaps in the map. Which is the closest          ║
# ║  unexplored cell, and what is the shortest known path to get there?"     ║
# ║                                                                          ║
# ║  A* f(n) = g(n) + h(n) where:                                           ║
# ║    g(n) = actual steps taken from current mouse position to cell n       ║
# ║           (using only walls currently known — real cost so far)          ║
# ║    h(n) = Manhattan distance from n to the TARGET unexplored cell        ║
# ║           (heuristic estimate of cost remaining)                         ║
# ║                                                                          ║
# ║  The result is a sequence of (x,y) waypoints the mouse should follow    ║
# ║  to reach the nearest unexplored cell efficiently.                       ║
# ╚══════════════════════════════════════════════════════════════════════════╝
def astar_path_to_target(start_x, start_y, target_x, target_y):
    """
    Find the shortest path from (start_x, start_y) to (target_x, target_y)
    using only currently known wall information.

    Returns: list of (x,y) tuples from start (exclusive) to target (inclusive).
             Returns empty list if no path found.
    """
    def h(x, y):
        return abs(x - target_x) + abs(y - target_y)

    # Priority queue entries: (f_score, g_score, x, y)
    open_heap = []
    heapq.heappush(open_heap, (h(start_x, start_y), 0, start_x, start_y))

    g_score = {(start_x, start_y): 0}
    came_from = {}   # to reconstruct the path
    closed = set()

    while open_heap:
        f, g, x, y = heapq.heappop(open_heap)

        if (x, y) in closed:
            continue
        closed.add((x, y))

        # Reached target — reconstruct path
        if x == target_x and y == target_y:
            path = []
            cur = (x, y)
            while cur in came_from:
                path.append(cur)
                cur = came_from[cur]
            path.reverse()
            return path

        # Expand neighbours (only through known open boundaries)
        for direction in [NORTH, EAST, SOUTH, WEST]:
            if direction in walls[x][y]:
                continue  # wall blocks this direction
            nx, ny = x + DX[direction], y + DY[direction]
            if not (0 <= nx < WIDTH and 0 <= ny < HEIGHT):
                continue
            if (nx, ny) in closed:
                continue

            new_g = g + 1
            if new_g < g_score.get((nx, ny), INF):
                g_score[(nx, ny)] = new_g
                came_from[(nx, ny)] = (x, y)
                heapq.heappush(open_heap, (new_g + h(nx, ny), new_g, nx, ny))

    return []  # no path found through known walls


def find_nearest_unexplored(start_x, start_y):
    """
    Find the closest unvisited cell using BFS through known open passages.
    BFS (not A*) is used here because we want the closest one overall —
    not the closest to any particular target.

    Returns (target_x, target_y) or None if all reachable cells visited.
    """
    queue = deque([(start_x, start_y, 0)])
    seen = {(start_x, start_y)}

    while queue:
        x, y, dist = queue.popleft()

        if not visited[x][y]:
            return (x, y)

        for direction in [NORTH, EAST, SOUTH, WEST]:
            if direction in walls[x][y]:
                continue
            nx, ny = x + DX[direction], y + DY[direction]
            if (nx, ny) not in seen and 0 <= nx < WIDTH and 0 <= ny < HEIGHT:
                seen.add((nx, ny))
                queue.append((nx, ny, dist + 1))

    return None  # all reachable cells have been visited


# ===========================================================================
# VISUAL HELPERS
# ===========================================================================
EXPLORE_COLOR  = "G"  # green  — cells visited during exploration
SPEEDRUN_COLOR = "Y"  # yellow — cells on the final speed run path
GOAL_COLOR     = "c"  # cyan   — goal cells

def color_cell(x, y, color):
    API.setColor(x, y, color)

def mark_visited(x, y):
    visited[x][y] = True
    if (x, y) not in GOAL_CELLS:
        color_cell(x, y, EXPLORE_COLOR)


# ===========================================================================
# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  EXPLORATION RUN                                                        ║
# ║                                                                          ║
# ║  HOW A RUN WORKS — answering your core question:                        ║
# ║                                                                          ║
# ║  The mouse does the following in a loop:                                ║
# ║    1. Arrive at cell (x, y)                                              ║
# ║    2. SENSE walls → record them into walls[x][y]  ← MAP BUILDING        ║
# ║    3. If new wall found → re-run flood fill        ← VALUE ASSIGNMENT    ║
# ║    4. Mark cell as visited                                               ║
# ║    5. Check if goal reached → if yes, stop                              ║
# ║    6. Move to neighbour with lowest flood value   ← NAVIGATION          ║
# ║                                                                          ║
# ║  The map (walls) and values (flood) are built SIMULTANEOUSLY as         ║
# ║  the mouse moves. They are not two separate phases within a run —        ║
# ║  every single cell visit does both.                                      ║
# ╚══════════════════════════════════════════════════════════════════════════╝
def exploration_run(state, run_number):
    """
    One full exploration pass: mouse navigates from start toward goal using
    flood fill, building the map as it goes.

    After reaching the goal, it also tries to return to start while
    visiting unexplored cells (using A* to target them).
    """
    log(f"=== Exploration Run {run_number} START ===")
    goal_reached = False

    # ── PHASE A: Navigate to goal using flood fill ──────────────────────
    while not goal_reached:
        x, y = state['x'], state['y']

        # STEP 2: Sense and record walls → THIS BUILDS THE MAP
        new_wall = sense_and_record_walls(x, y, state['facing'])

        # STEP 3: If new wall found → re-assign all flood values
        if new_wall:
            log(f"  New wall at ({x},{y}) → re-flooding")
            run_flood_fill()

        # STEP 4: Mark as visited
        mark_visited(x, y)

        # STEP 5: Check if goal
        if (x, y) in GOAL_CELLS:
            log(f"  Goal reached at ({x},{y})!")
            goal_reached = True
            color_cell(x, y, GOAL_COLOR)
            break

        # STEP 6: Move to lowest flood neighbour
        best_dir = best_flood_neighbor(x, y)
        if best_dir is None:
            log("  ERROR: No open neighbour — maze may be unsolvable")
            break

        turn_to_face(best_dir, state)
        move_forward(state)

    # ── PHASE B: Return to start, targeting unexplored cells via A* ─────
    # This is where A* adds value over pure flood fill.
    # We re-flood from START now (swap goal temporarily) and navigate back,
    # but deliberately route through unexplored cells.
    log(f"  Goal reached. Now returning to start via unexplored cells...")
    _return_via_unexplored(state)
    log(f"=== Exploration Run {run_number} END ===")


def _return_via_unexplored(state):
    """
    After reaching the goal, navigate back to start.
    Uses A* to find paths to nearby unexplored cells along the way,
    so each return journey fills in more of the map.
    """
    start_cell = (0, 0)
    max_detours = 10  # limit how many A* detours we take per return trip

    detours_taken = 0
    while (state['x'], state['y']) != start_cell and detours_taken < max_detours:

        # Find the nearest unexplored cell reachable from current position
        target = find_nearest_unexplored(state['x'], state['y'])

        if target is None:
            log("  All reachable cells visited!")
            break

        tx, ty = target
        # Only detour if the unexplored cell is reasonably close
        manhattan_to_target = abs(tx - state['x']) + abs(ty - state['y'])
        manhattan_to_start  = abs(state['x']) + abs(state['y'])

        if manhattan_to_target > manhattan_to_start:
            # The unexplored cell is farther than start — skip, go home
            break

        # Use A* to find a path to this unexplored cell
        path = astar_path_to_target(state['x'], state['y'], tx, ty)

        if not path:
            break

        # Follow the A* path step by step
        for (nx, ny) in path:
            move_to_neighbor(nx, ny, state)
            x, y = state['x'], state['y']

            # Sense and record at each new cell
            new_wall = sense_and_record_walls(x, y, state['facing'])
            if new_wall:
                run_flood_fill()
            mark_visited(x, y)

        detours_taken += 1

    # Final straight return to start using flood fill
    # Re-flood from start to home
    _navigate_to(state, 0, 0)


def _navigate_to(state, target_x, target_y):
    """
    Navigate from current position to (target_x, target_y) using A*.
    Follows the computed path step by step, sensing walls along the way.
    """
    safety = 0
    while (state['x'], state['y']) != (target_x, target_y) and safety < 300:
        safety += 1
        x, y = state['x'], state['y']

        path = astar_path_to_target(x, y, target_x, target_y)
        if not path:
            log(f"  Cannot find path to ({target_x},{target_y})")
            break

        # Take one step along A* path, re-plan if new wall found
        nx, ny = path[0]
        move_to_neighbor(nx, ny, state)

        new_wall = sense_and_record_walls(state['x'], state['y'], state['facing'])
        if new_wall:
            run_flood_fill()
            # Path might be invalidated — re-plan next iteration
        mark_visited(state['x'], state['y'])


# ===========================================================================
# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  SPEED RUN — THE FINAL OPTIMAL RUN                                      ║
# ║                                                                          ║
# ║  YOUR UNDERSTANDING: "the final path is the one with decreasing          ║
# ║  cell numbers" — THIS IS EXACTLY CORRECT.                               ║
# ║                                                                          ║
# ║  After all exploration runs, we have a near-complete wall map.          ║
# ║  We run flood fill ONE final time on this complete map.                  ║
# ║  Then the mouse simply follows decreasing flood values — no              ║
# ║  re-flooding, no sensing, just pure speed.                               ║
# ║                                                                          ║
# ║  This is the path that wins competitions.                                ║
# ╚══════════════════════════════════════════════════════════════════════════╝
def speed_run(state):
    log("=== SPEED RUN ===")
    log("Final flood fill on complete known map...")
    run_flood_fill()  # one final accurate computation

    API.clearAllColor()
    for (gx, gy) in GOAL_CELLS:
        color_cell(gx, gy, GOAL_COLOR)

    while (state['x'], state['y']) not in GOAL_CELLS:
        x, y = state['x'], state['y']

        # NO wall sensing, NO re-flooding — map is complete.
        # Just follow the gradient: always step to lowest flood neighbour.
        best_dir = best_flood_neighbor(x, y)

        if best_dir is None:
            log("ERROR: Stuck during speed run!")
            break

        color_cell(x, y, SPEEDRUN_COLOR)
        turn_to_face(best_dir, state)
        move_forward(state)

    log(f"Speed run complete! Arrived at ({state['x']},{state['y']})")
    color_cell(state['x'], state['y'], GOAL_COLOR)


# ===========================================================================
# RESET HANDLER
# ===========================================================================
def handle_reset(state):
    """
    When user presses Reset in MMS, the mouse returns to (0,0).
    We clear the visual display but KEEP the wall map —
    we've earned that knowledge and should use it in subsequent runs.
    """
    log("Reset detected — keeping map, clearing visuals")
    API.ackReset()
    API.clearAllColor()
    state['x'] = 0
    state['y'] = 0
    state['facing'] = NORTH
    # Re-flood with accumulated wall knowledge
    run_flood_fill()


# ===========================================================================
# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  MAIN — THE RUN STRATEGY                                                ║
# ║                                                                          ║
# ║  Run 1: Explore → reach goal, learn walls along the way                 ║
# ║  Run 2: Return trip → target unexplored cells via A*                    ║
# ║  Run 3: Another exploration pass if significant gaps remain             ║
# ║  Run 4+: Speed runs on the best known path                              ║
# ║                                                                          ║
# ║  The wall map ACCUMULATES across all runs — it is never reset.          ║
# ║  Only the visual display and mouse position reset between runs.          ║
# ╚══════════════════════════════════════════════════════════════════════════╝
def main():
    log(f"=== Micromouse: A* + FloodFill ===")
    log(f"Maze: {WIDTH}x{HEIGHT}, Goal: {GOAL_CELLS}")

    # One-time initialisation
    init_border_walls()
    run_flood_fill()

    state = {'x': 0, 'y': 0, 'facing': NORTH}
    run_number = 0

    # How many exploration runs before switching to speed-only mode?
    EXPLORATION_RUNS = 3

    while True:
        # Check for reset between runs
        if API.wasReset():
            handle_reset(state)
            continue

        run_number += 1
        log(f"\n--- Starting run {run_number} ---")

        # Count what fraction of the maze is explored
        explored_count = sum(1 for x in range(WIDTH)
                              for y in range(HEIGHT) if visited[x][y])
        coverage = explored_count / (WIDTH * HEIGHT) * 100
        log(f"Current map coverage: {coverage:.1f}%")

        if run_number <= EXPLORATION_RUNS:
            # ── Exploration run ─────────────────────────────────────────
            exploration_run(state, run_number)

            # After the run, navigate back to start for next run
            if (state['x'], state['y']) != (0, 0):
                log("Navigating back to start...")
                _navigate_to(state, 0, 0)

        else:
            # ── Speed run ───────────────────────────────────────────────
            speed_run(state)
            log("Speed run complete. Waiting for reset...")

            # Wait here until user resets for another speed run
            while not API.wasReset():
                pass
            handle_reset(state)


if __name__ == "__main__":
    main()
import API
import sys
from collections import deque
import heapq

# ===========================================================================
# LOGGING
# ===========================================================================
# stdout is reserved for MMS communication — ALL debug output uses stderr.
def log(msg):
    sys.stderr.write(str(msg) + "\n")
    sys.stderr.flush()


# ===========================================================================
# CONSTANTS & DIRECTION SYSTEM
# ===========================================================================
WIDTH  = API.mazeWidth()
HEIGHT = API.mazeHeight()

NORTH, EAST, SOUTH, WEST = 0, 1, 2, 3

DX = [ 0,  1,  0, -1]   # x-change when moving NORTH / EAST / SOUTH / WEST
DY = [ 1,  0, -1,  0]   # y-change

OPPOSITE = {NORTH: SOUTH, SOUTH: NORTH, EAST: WEST, WEST: EAST}
DIR_CHAR = ['n', 'e', 's', 'w']    # MMS requires lowercase letters

# Goal = centre 2×2 block of the 16×16 maze
def compute_goals(w, h):
    cx, cy = w // 2, h // 2
    goals = {(cx, cy)}
    if w % 2 == 0: goals.add((cx - 1, cy))
    if h % 2 == 0: goals.add((cx,     cy - 1))
    if w % 2 == 0 and h % 2 == 0: goals.add((cx - 1, cy - 1))
    return goals

GOAL_CELLS = compute_goals(WIDTH, HEIGHT)
START_CELL = (0, 0)
INF        = float('inf')


# ===========================================================================
# MAZE STATE
# ===========================================================================
# walls[x][y]   — set of directions with confirmed walls at cell (x, y)
# visited[x][y] — True once the mouse has physically stood on that cell
walls   = [[set() for _ in range(HEIGHT)] for _ in range(WIDTH)]
visited = [[False]  * HEIGHT              for _ in range(WIDTH)]

# flood_dist[x][y] — true wall-aware BFS distance to nearest goal.
# Populated by build_flood_fill() after DFS completes.
# Until then every cell reads INF so A* never silently uses a stale value.
flood_dist = [[INF] * HEIGHT for _ in range(WIDTH)]


# ===========================================================================
# BORDER WALLS  (always true — maze has an outer boundary)
# ===========================================================================
def init_border_walls():
    for x in range(WIDTH):
        for y in range(HEIGHT):
            if y == HEIGHT - 1: walls[x][y].add(NORTH)
            if y == 0:          walls[x][y].add(SOUTH)
            if x == WIDTH - 1:  walls[x][y].add(EAST)
            if x == 0:          walls[x][y].add(WEST)


# ===========================================================================
# WALL SENSING & RECORDING
# ===========================================================================
def abs_direction(facing, relative):
    """Convert a sensor's relative direction to an absolute compass direction."""
    if relative == 'front': return facing
    if relative == 'right': return (facing + 1) % 4
    if relative == 'left':  return (facing + 3) % 4

def record_wall(x, y, direction):
    """
    Store a wall on BOTH sides of the shared boundary.
    A wall between (3,4) and (3,5) must be stored as:
      - NORTH of (3,4)   AND   - SOUTH of (3,5)
    Returns True if this is new information (wall was not already known).
    """
    if direction in walls[x][y]:
        return False                          # already knew this
    walls[x][y].add(direction)
    API.setWall(x, y, DIR_CHAR[direction])
    nx, ny = x + DX[direction], y + DY[direction]
    if 0 <= nx < WIDTH and 0 <= ny < HEIGHT:
        walls[nx][ny].add(OPPOSITE[direction])
        API.setWall(nx, ny, DIR_CHAR[OPPOSITE[direction]])
    return True                               # new wall discovered

def sense_and_record_walls(x, y, facing):
    """Read all three sensors and write any new walls into walls[][]."""
    for relative, has_wall in [('front', API.wallFront()),
                                ('left',  API.wallLeft()),
                                ('right', API.wallRight())]:
        if has_wall:
            record_wall(x, y, abs_direction(facing, relative))


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
    """Turn the mouse to face target_dir using the minimum number of turns."""
    right_turns = (target_dir - state['facing']) % 4
    if right_turns == 1:   turn_right(state)
    elif right_turns == 2: turn_right(state); turn_right(state)   # U-turn
    elif right_turns == 3: turn_left(state)
    # right_turns == 0: already facing the right way

def move_to_cell(nx, ny, state):
    """Turn to face (nx, ny) — which must be adjacent — and move there."""
    dx, dy = nx - state['x'], ny - state['y']
    if   dx ==  1: target_dir = EAST
    elif dx == -1: target_dir = WEST
    elif dy ==  1: target_dir = NORTH
    else:          target_dir = SOUTH
    turn_to_face(target_dir, state)
    move_forward(state)


# ===========================================================================
# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  RUN 1 — DFS FULL MAZE EXPLORATION                                      ║
# ║                                                                          ║
# ║  GOAL: Visit every reachable cell in the maze and record all walls.      ║
# ║  The richer the map built here, the more optimal A* will be in Run 2.   ║
# ║                                                                          ║
# ║  ALGORITHM: Iterative DFS with a backtracking stack.                    ║
# ║    - At each cell, sense walls.                                          ║
# ║    - If an unvisited open neighbour exists → move there (go deeper).     ║
# ║    - If no unvisited neighbours remain → backtrack one step (pop stack). ║
# ║    - Repeat until the stack is empty (entire reachable maze explored).   ║
# ║                                                                          ║
# ║  COLOUR CODE (MMS display):                                              ║
# ║    Cyan   — goal cells                                                   ║
# ║    Green  — cells explored during DFS                                    ║
# ╚══════════════════════════════════════════════════════════════════════════╝
def dfs_explore(state):
    log("=== RUN 1: DFS Full Exploration ===")

    # Initialise: mark start as visited, push it onto the backtrack stack
    visited[state['x']][state['y']] = True
    dfs_stack = [(state['x'], state['y'])]   # stack of (x, y) — current DFS path

    cells_visited = 0

    while dfs_stack:
        x, y = state['x'], state['y']

        # ── Step 1: Sense walls at current cell ────────────────────────────
        sense_and_record_walls(x, y, state['facing'])

        # ── Step 2: Find an unvisited open neighbour ───────────────────────
        next_dir = None
        for d in range(4):
            if d in walls[x][y]:
                continue                     # wall blocks this direction
            nx, ny = x + DX[d], y + DY[d]
            if not (0 <= nx < WIDTH and 0 <= ny < HEIGHT):
                continue
            if not visited[nx][ny]:
                next_dir = d
                break                        # take the first unvisited we find

        # ── Step 3a: Unvisited neighbour found → go deeper ─────────────────
        if next_dir is not None:
            nx, ny = x + DX[next_dir], y + DY[next_dir]
            move_to_cell(nx, ny, state)
            visited[nx][ny] = True
            dfs_stack.append((nx, ny))
            cells_visited += 1

            # Colour the cell (goal cells stay cyan)
            if (nx, ny) in GOAL_CELLS:
                API.setColor(nx, ny, 'c')
            else:
                API.setColor(nx, ny, 'G')

        # ── Step 3b: No unvisited neighbours → backtrack ───────────────────
        else:
            dfs_stack.pop()                  # this cell is fully explored
            if dfs_stack:
                # Move back to the previous cell in the DFS path
                px, py = dfs_stack[-1]
                move_to_cell(px, py, state)
                # Note: we do NOT re-colour the backtrack cell — it keeps its
                # green colour from when it was first visited

    log(f"=== DFS COMPLETE: {cells_visited} cells explored ===")
    log(f"    Map coverage: {_coverage():.1f}%")
    log(f"    Mouse is back at ({state['x']},{state['y']})")


def _coverage():
    n = sum(1 for x in range(WIDTH) for y in range(HEIGHT) if visited[x][y])
    return n / (WIDTH * HEIGHT) * 100


# ===========================================================================
# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  FLOOD-FILL DISTANCE TABLE                                               ║
# ║                                                                          ║
# ║  Called ONCE after DFS finishes — at that point walls[][] is complete.  ║
# ║                                                                          ║
# ║  ALGORITHM: Multi-source BFS starting simultaneously from every goal    ║
# ║  cell, expanding only through passable (wall-free) boundaries.          ║
# ║                                                                          ║
# ║  RESULT: flood_dist[x][y] = exact minimum number of steps from (x,y)   ║
# ║  to the nearest goal, given the real wall layout.                        ║
# ║                                                                          ║
# ║  WHY THIS IS A PERFECT HEURISTIC FOR A*:                                ║
# ║    • It never overestimates  → A* remains optimal (admissible).         ║
# ║    • It equals the true cost → A* expands almost no wasted nodes        ║
# ║      (consistent / perfect heuristic).                                  ║
# ║    • Manhattan distance ignores walls and can be far too optimistic;    ║
# ║      flood-fill knows exactly which corridors exist.                    ║
# ╚══════════════════════════════════════════════════════════════════════════╝
def build_flood_fill():
    """
    Multi-source BFS from all goal cells through the complete wall map.
    Populates flood_dist[x][y] with the true wall-aware distance to the
    nearest goal for every cell in the maze.
    """
    log("  Building flood-fill distance table (wall-aware BFS)...")

    # Reset the table
    for x in range(WIDTH):
        for y in range(HEIGHT):
            flood_dist[x][y] = INF

    queue = deque()

    # Seed: every goal cell has distance 0
    for (gx, gy) in GOAL_CELLS:
        flood_dist[gx][gy] = 0
        queue.append((gx, gy))

    # BFS — expand only through open (wall-free) passages
    while queue:
        x, y = queue.popleft()
        for d in range(4):
            if d in walls[x][y]:
                continue                     # wall blocks passage
            nx, ny = x + DX[d], y + DY[d]
            if not (0 <= nx < WIDTH and 0 <= ny < HEIGHT):
                continue
            new_dist = flood_dist[x][y] + 1
            if new_dist < flood_dist[nx][ny]:
                flood_dist[nx][ny] = new_dist
                queue.append((nx, ny))

    reachable = sum(
        1 for x in range(WIDTH) for y in range(HEIGHT)
        if flood_dist[x][y] < INF
    )
    log(f"  Flood-fill complete. {reachable}/{WIDTH*HEIGHT} cells reachable from goal.")
    log(f"  Start cell (0,0) distance to goal: {flood_dist[0][0]}")


# ===========================================================================
# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  RUN 2 — A* SPEED RUN  (flood-fill heuristic)                           ║
# ║                                                                          ║
# ║  GOAL: Find and follow the optimal path from start to goal using the    ║
# ║  complete wall map AND the pre-computed flood-fill distance table.       ║
# ║                                                                          ║
# ║  HEURISTIC — flood_dist[x][y]:                                          ║
# ║    Unlike Manhattan distance (which ignores walls), flood_dist gives     ║
# ║    the EXACT minimum wall-respecting distance from every cell to the     ║
# ║    nearest goal.  It is:                                                 ║
# ║      • Admissible   — never overestimates the true remaining cost.       ║
# ║      • Consistent   — satisfies the triangle inequality across edges.   ║
# ║      • Near-perfect — h(n) ≈ true cost, so A* barely opens any node    ║
# ║        that isn't on the optimal path.                                   ║
# ║                                                                          ║
# ║  COLOUR CODE (MMS display):                                              ║
# ║    Yellow — cells on the A* planned path                                 ║
# ║    Cyan   — goal cell reached                                            ║
# ╚══════════════════════════════════════════════════════════════════════════╝
def heuristic(x, y):
    """
    Wall-aware flood-fill distance to the nearest goal.
    This is the h(n) in f(n) = g(n) + h(n).

    flood_dist[x][y] was computed by multi-source BFS from all goal cells
    through the real wall map, so it reflects the exact shortest distance
    achievable — no wall is ignored, no shortcut is assumed.

    Because it equals the true remaining cost it is a perfect heuristic:
    A* will follow the optimal path with minimal wasted exploration.
    """
    return flood_dist[x][y]


def astar_plan(start_x, start_y):
    """
    Plan the optimal path from (start_x, start_y) to the nearest goal cell
    using the wall map in walls[][] and the flood-fill heuristic.

    Returns a list of (x, y) tuples: the path from start (exclusive)
    to goal (inclusive). Returns an empty list if no path is found.

    HOW A* WORKS:
      A priority queue (min-heap) always gives us the cell with the lowest
      f(n) = g(n) + h(n) to expand next.

      g(n)  = actual steps taken from start to reach cell n
              (exact, accumulated step by step)
      h(n)  = flood_dist[n] — exact wall-aware distance to nearest goal
              (perfect heuristic: equals true remaining cost)
      f(n)  = total estimated cost of the path through n

      With a perfect heuristic, f(n) equals the true path cost for every
      node on the optimal path, so A* expands them in order without detours.

      When the goal is popped from the heap, the path through came_from[]
      is reconstructed by walking backward from goal to start.
    """
    sx, sy = start_x, start_y

    # Priority queue entries: (f_score, g_score, x, y)
    open_heap = []
    heapq.heappush(open_heap, (heuristic(sx, sy), 0, sx, sy))

    g_score   = {(sx, sy): 0}   # actual cost from start to each cell
    came_from = {}               # for path reconstruction
    closed    = set()            # cells already fully expanded

    while open_heap:
        f, g, x, y = heapq.heappop(open_heap)

        # Skip stale heap entries (g_score was improved after this was pushed)
        if (x, y) in closed:
            continue
        closed.add((x, y))

        # Goal reached — reconstruct and return the path
        if (x, y) in GOAL_CELLS:
            path = []
            cur  = (x, y)
            while cur in came_from:
                path.append(cur)
                cur = came_from[cur]
            path.reverse()
            log(f"  A* path found: {len(path)} steps")
            return path

        # Expand all open neighbours
        for d in range(4):
            if d in walls[x][y]:
                continue                  # wall blocks this direction
            nx, ny = x + DX[d], y + DY[d]
            if not (0 <= nx < WIDTH and 0 <= ny < HEIGHT):
                continue
            if (nx, ny) in closed:
                continue

            new_g = g + 1                 # every step costs 1
            if new_g < g_score.get((nx, ny), INF):
                g_score[(nx, ny)]   = new_g
                came_from[(nx, ny)] = (x, y)
                f_score             = new_g + heuristic(nx, ny)
                heapq.heappush(open_heap, (f_score, new_g, nx, ny))

    log("  A* found no path — maze may be unsolvable!")
    return []


def astar_run(state):
    """
    Run 2: Use the wall map from DFS + flood-fill distances to plan and
    execute the optimal path. A* plans the full route before the mouse
    moves a single step, guided by the wall-aware flood-fill heuristic.
    """
    log("=== RUN 2: A* Speed Run (flood-fill heuristic) ===")
    API.clearAllColor()
    for (gx, gy) in GOAL_CELLS:
        API.setColor(gx, gy, 'c')

    # ── Step 1: Plan the full path using A* ───────────────────────────────
    log("  Planning path with A* (flood-fill h)...")
    path = astar_plan(state['x'], state['y'])

    if not path:
        log("  Cannot find a path. Aborting speed run.")
        return

    # Visualise the planned path in yellow before moving
    for (px, py) in path:
        if (px, py) not in GOAL_CELLS:
            API.setColor(px, py, 'Y')

    log(f"  Path planned: {len(path)} steps. Starting run...")

    # ── Step 2: Follow the planned path step by step ──────────────────────
    for step_num, (nx, ny) in enumerate(path):
        move_to_cell(nx, ny, state)

        if (nx, ny) in GOAL_CELLS:
            API.setColor(nx, ny, 'c')
            log(f"  Goal reached at ({nx},{ny}) in {step_num + 1} steps!")
            break

    log("=== A* RUN COMPLETE ===")


# ===========================================================================
# RESET HANDLER
# ===========================================================================
def handle_reset(state):
    """
    When the user presses Reset in MMS, the mouse returns to (0,0).
    We preserve the wall map AND flood-fill table — we earned that
    knowledge during DFS. Only position and display are reset.
    """
    log("Reset detected — preserving wall map and flood-fill table.")
    API.ackReset()
    API.clearAllColor()
    state['x']      = 0
    state['y']      = 0
    state['facing'] = NORTH
    for (gx, gy) in GOAL_CELLS:
        API.setColor(gx, gy, 'c')


# ===========================================================================
# MAIN
# ===========================================================================
def main():
    log("=== Micromouse: DFS Mapping + Flood-Fill + A* Speed Run ===")
    log(f"Maze: {WIDTH}x{HEIGHT}  |  Goal: {GOAL_CELLS}")
    log("")
    log("Strategy:")
    log("  Run 1 — DFS:        explore the entire maze, build a complete wall map.")
    log("  Post-DFS:           compute flood-fill distance table (BFS from goals).")
    log("  Run 2 — A*:         use flood-fill distances as perfect heuristic h(n).")

    # One-time setup
    init_border_walls()

    state = {'x': 0, 'y': 0, 'facing': NORTH}

    # Mark goal cells in the display
    for (gx, gy) in GOAL_CELLS:
        API.setColor(gx, gy, 'c')

    run_number = 0

    while True:
        if API.wasReset():
            handle_reset(state)
            continue

        run_number += 1
        log(f"\n{'='*55}")
        log(f"RUN {run_number}")

        if run_number == 1:
            # ── Run 1: DFS full exploration ───────────────────────────────
            dfs_explore(state)

            # ── Post-DFS: build flood-fill table over the complete wall map
            build_flood_fill()

            log("")
            log("DFS + flood-fill complete. Press Reset to start the A* speed run.")
            log(f"Map coverage: {_coverage():.1f}%")

            # Wait at start for the user to press Reset
            while not API.wasReset():
                pass
            handle_reset(state)

        else:
            # ── Run 2+: A* speed run ──────────────────────────────────────
            astar_run(state)
            log("")
            log("Speed run complete. Press Reset to run again.")

            # Wait for reset before another speed run
            while not API.wasReset():
                pass
            handle_reset(state)


if __name__ == "__main__":
    main()

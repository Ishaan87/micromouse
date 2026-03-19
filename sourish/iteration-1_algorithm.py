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
# ║  RUN 1 — FLOODFILL-DRIVEN EXPLORATION  (go to goal, return to start)   ║
# ║                                                                          ║
# ║  GOAL: Build a good-enough wall map by walking to the goal and back,   ║
# ║  sensing walls at every step.  Total travel is roughly 2× the optimal  ║
# ║  path length — far cheaper than visiting every cell.                    ║
# ║                                                                          ║
# ║  ALGORITHM                                                               ║
# ║  At every cell the mouse recomputes a live floodfill distance table     ║
# ║  (multi-source BFS from the current target, respecting known walls).    ║
# ║  It then steps to whichever open neighbour has the lowest flood value.  ║
# ║  When new walls are sensed the table is rebuilt instantly, so the       ║
# ║  mouse always follows the best currently-known route.                   ║
# ║                                                                          ║
# ║  TWO PHASES                                                              ║
# ║    Phase 1 — target = GOAL_CELLS  (navigate to goal)                   ║
# ║    Phase 2 — target = {(0,0)}     (navigate back to start)             ║
# ║  Walls discovered in Phase 1 are reused in Phase 2.                    ║
# ║                                                                          ║
# ║  WHY THIS BEATS FULL EXPLORATION                                         ║
# ║    • Only goal-relevant corridors are walked — dead ends are skipped.  ║
# ║    • The partial wall map is still highly accurate for those corridors. ║
# ║    • Run 2 A* operates on exactly the paths the mouse will actually use.║
# ║                                                                          ║
# ║  COLOUR CODE (MMS display)                                               ║
# ║    Cyan   — goal cells                                                   ║
# ║    Green  — cells visited during Phase 1 (outbound)                    ║
# ║    Orange — cells visited during Phase 2 (return)                      ║
# ╚══════════════════════════════════════════════════════════════════════════╝

def _build_live_flood(targets):
    """
    Multi-source BFS from `targets` (a set of (x,y)) over the current
    wall map.  Returns a 2-D distance table live_dist[x][y].
    Rebuilt whenever new walls are discovered so the mouse always follows
    the best currently-known route.
    """
    live_dist = [[INF] * HEIGHT for _ in range(WIDTH)]
    queue = deque()
    for (gx, gy) in targets:
        live_dist[gx][gy] = 0
        queue.append((gx, gy))
    while queue:
        x, y = queue.popleft()
        for d in range(4):
            if d in walls[x][y]:
                continue
            nx, ny = x + DX[d], y + DY[d]
            if not (0 <= nx < WIDTH and 0 <= ny < HEIGHT):
                continue
            if live_dist[nx][ny] == INF:
                live_dist[nx][ny] = live_dist[x][y] + 1
                queue.append((nx, ny))
    return live_dist


def _floodfill_phase(state, targets, phase_label, visited_color):
    """
    Navigate from the current position to any cell in `targets` by
    always stepping to the open neighbour with the lowest live flood value.
    Senses and records walls at every cell; rebuilds the flood table
    whenever new information arrives.

    Returns True if a target was reached, False if trapped (should not
    happen in a well-formed maze).
    """
    live_dist = _build_live_flood(targets)
    steps     = 0

    while (state['x'], state['y']) not in targets:
        x, y = state['x'], state['y']

        # ── Sense walls; rebuild flood table if anything changed ───────────
        new_wall = False
        for relative, has_wall in [('front', API.wallFront()),
                                    ('left',  API.wallLeft()),
                                    ('right', API.wallRight())]:
            if has_wall:
                changed = record_wall(x, y, abs_direction(state['facing'], relative))
                if changed:
                    new_wall = True
        if new_wall:
            live_dist = _build_live_flood(targets)

        # ── Choose the open neighbour with the lowest flood distance ───────
        best_d, best_val = None, INF
        for d in range(4):
            if d in walls[x][y]:
                continue
            nx, ny = x + DX[d], y + DY[d]
            if not (0 <= nx < WIDTH and 0 <= ny < HEIGHT):
                continue
            if live_dist[nx][ny] < best_val:
                best_val = live_dist[nx][ny]
                best_d   = d

        if best_d is None or best_val == INF:
            log(f"  {phase_label}: trapped at ({x},{y}) — no passable neighbour!")
            return False

        # ── Move ───────────────────────────────────────────────────────────
        nx, ny = x + DX[best_d], y + DY[best_d]
        move_to_cell(nx, ny, state)
        steps += 1

        if not visited[nx][ny]:
            visited[nx][ny] = True
            if (nx, ny) in GOAL_CELLS:
                API.setColor(nx, ny, 'c')
            else:
                API.setColor(nx, ny, visited_color)

    log(f"  {phase_label}: reached target in {steps} steps.")
    return True


def floodfill_explore(state):
    log("=== RUN 1: Floodfill Explore (start->goal->start) ===")

    # Mark start visited
    visited[state['x']][state['y']] = True

    # One lap: go to goal, come back. Any walls missed will be caught
    # by the live replanning in astar_run.
    #
    # Colour key:
    #   G (green)  — outbound  (start -> goal)
    #   o (orange) — return    (goal  -> start)
    phases = [
        (GOAL_CELLS, "out  (start->goal)", 'G'),
        ({(0, 0)},   "back (goal->start)", 'o'),
    ]

    for targets, label, color in phases:
        log(f"  {label}...")
        reached = _floodfill_phase(state, targets, label, color)
        if not reached:
            log(f"  Could not complete {label} — maze may be disconnected.")
            return
        log(f"  {label} done. Mouse at ({state['x']},{state['y']}). "
            f"Coverage: {_coverage():.1f}%")

    log("=== FLOODFILL EXPLORATION COMPLETE ===")
    log(f"    Final map coverage: {_coverage():.1f}%")
    log(f"    Mouse is at ({state['x']},{state['y']})")


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
    Run 2: A* speed run with live wall sensing and on-the-fly replanning.

    NORMAL CASE
    -----------
    A* plans the full route using flood_dist as the heuristic, then the
    mouse follows it step by step.

    UNKNOWN WALL DISCOVERED MID-RUN
    --------------------------------
    Before each physical move the mouse senses the three adjacent walls.
    If a new wall is found that sits on the planned path ahead:
      1. Record the wall (updates walls[][] and MMS display).
      2. Rebuild flood_dist via BFS — the heuristic is now accurate for
         the updated map.
      3. Replan A* from the current position using the new flood_dist.
      4. Clear the stale yellow path and redraw the new one.
    This guarantees the mouse never attempts to move through a wall it
    has just discovered, eliminating the crash risk entirely.

    Replanning is triggered only when the new wall actually lies on the
    current planned path (cheap set-membership check), so in the common
    case where the exploration covered all relevant corridors no replanning
    occurs and the run is as fast as before.
    """
    log("=== RUN 2: A* Speed Run (flood-fill heuristic, live replanning) ===")
    API.clearAllColor()
    for (gx, gy) in GOAL_CELLS:
        API.setColor(gx, gy, 'c')

    def plan_and_draw(from_x, from_y):
        """(Re)plan from (from_x, from_y), redraw yellow path, return path."""
        API.clearAllColor()
        for (gx, gy) in GOAL_CELLS:
            API.setColor(gx, gy, 'c')
        p = astar_plan(from_x, from_y)
        for (px, py) in p:
            if (px, py) not in GOAL_CELLS:
                API.setColor(px, py, 'Y')
        return p

    # ── Initial plan ──────────────────────────────────────────────────────
    log("  Planning initial path...")
    path = plan_and_draw(state['x'], state['y'])
    if not path:
        log("  Cannot find a path. Aborting speed run.")
        return
    log(f"  Path planned: {len(path)} steps. Starting run...")

    path_set    = set(path)   # fast membership test for replanning trigger
    total_steps = 0

    while (state['x'], state['y']) not in GOAL_CELLS:
        if not path:
            log("  Path exhausted without reaching goal — replanning...")
            path = plan_and_draw(state['x'], state['y'])
            if not path:
                log("  No path found. Aborting.")
                return
            path_set = set(path)

        # ── Sense walls BEFORE moving ─────────────────────────────────────
        # Record any previously unknown walls around the current cell.
        # If a new wall blocks a cell on our planned path ahead, replan
        # before taking the next step — this is what prevents crashes.
        x, y     = state['x'], state['y']
        new_wall = False
        for relative, has_wall in [('front', API.wallFront()),
                                    ('left',  API.wallLeft()),
                                    ('right', API.wallRight())]:
            if has_wall:
                changed = record_wall(x, y, abs_direction(state['facing'], relative))
                if changed:
                    new_wall = True

        if new_wall:
            # Check whether any newly blocked neighbour appears on the
            # current planned path. If so, the path is now invalid.
            blocked_on_path = any(
                (x + DX[d], y + DY[d]) in path_set
                for d in range(4)
                if d in walls[x][y]
                and 0 <= x + DX[d] < WIDTH
                and 0 <= y + DY[d] < HEIGHT
            )
            if blocked_on_path:
                log(f"  New wall near ({x},{y}) blocks planned path — replanning...")
                build_flood_fill()           # refresh heuristic with new wall
                path = plan_and_draw(x, y)
                if not path:
                    log("  No path found after replanning. Aborting.")
                    return
                path_set = set(path)
                log(f"  Replanned: {len(path)} steps remaining.")

        # ── Move to next waypoint ─────────────────────────────────────────
        nx, ny = path.pop(0)
        move_to_cell(nx, ny, state)
        total_steps += 1

        if (nx, ny) in GOAL_CELLS:
            API.setColor(nx, ny, 'c')
            log(f"  Goal reached at ({nx},{ny}) in {total_steps} steps!")

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
    log("=== Micromouse: Floodfill Explore + Flood-Fill Heuristic + A* ===")
    log(f"Maze: {WIDTH}x{HEIGHT}  |  Goal: {GOAL_CELLS}")
    log("")
    log("Strategy:")
    log("  Run 1 — Floodfill:  navigate to goal and back, sensing walls.")
    log("  Post-Run1:          compute flood-fill distance table (BFS from goals).")
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
            # ── Run 1: Floodfill-driven goal + return exploration ─────────
            floodfill_explore(state)

            # ── Post-exploration: build flood-fill table over known wall map
            build_flood_fill()

            log("")
            log("Exploration + flood-fill complete. Press Reset for A* speed run.")
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
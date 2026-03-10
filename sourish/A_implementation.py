import API
import sys
import heapq
from typing import Tuple, Dict, List

def log(msg):
    sys.stderr.write(str(msg) + "\n")
    sys.stderr.flush()


WIDTH  = API.mazeWidth()
HEIGHT = API.mazeHeight()

NORTH = 0
EAST  = 1
SOUTH = 2
WEST  = 3

DX = [0, 1, 0, -1]
DY = [1, 0, -1, 0]

DIR_NAMES = {NORTH:"NORTH", EAST:"EAST", SOUTH:"SOUTH", WEST:"WEST"}

walls = [[set() for _ in range(HEIGHT)] for _ in range(WIDTH)]

OPPOSITE = {NORTH:SOUTH, SOUTH:NORTH, EAST:WEST, WEST:EAST}

def compute_goal_cells(w,h):
    cx = w//2
    cy = h//2

    goals=set()
    goals.add((cx,cy))

    if w%2==0:
        goals.add((cx-1,cy))
    if h%2==0:
        goals.add((cx,cy-1))
    if w%2==0 and h%2==0:
        goals.add((cx-1,cy-1))

    return goals

GOAL_CELLS = compute_goal_cells(WIDTH,HEIGHT)

def init_border_walls():

    for x in range(WIDTH):
        for y in range(HEIGHT):

            if y==HEIGHT-1:
                walls[x][y].add(NORTH)
            if y==0:
                walls[x][y].add(SOUTH)
            if x==WIDTH-1:
                walls[x][y].add(EAST)
            if x==0:
                walls[x][y].add(WEST)


def record_wall(x,y,direction):

    if direction in walls[x][y]:
        return False

    walls[x][y].add(direction)

    nx = x + DX[direction]
    ny = y + DY[direction]

    if 0<=nx<WIDTH and 0<=ny<HEIGHT:
        walls[nx][ny].add(OPPOSITE[direction])

    return True



def abs_direction(facing,relative):

    if relative=='front': return facing
    if relative=='right': return (facing+1)%4
    if relative=='left': return (facing+3)%4



def sense_and_record_walls(x,y,facing):

    new_wall=False

    sensors=[
        ('front',API.wallFront()),
        ('left',API.wallLeft()),
        ('right',API.wallRight())
    ]

    for rel,val in sensors:

        if val:

            d = abs_direction(facing,rel)

            if record_wall(x,y,d):
                new_wall=True
                API.setWall(x,y,['n','e','s','w'][d])

    return new_wall



def get_neighbors(pos):

    x,y = pos

    neighbors=[]

    for d in [NORTH,EAST,SOUTH,WEST]:

        if d in walls[x][y]:
            continue

        nx = x + DX[d]
        ny = y + DY[d]

        if 0<=nx<WIDTH and 0<=ny<HEIGHT:
            neighbors.append((nx,ny))

    return neighbors



def heuristic(a,b):

    return abs(a[0]-b[0]) + abs(a[1]-b[1])



def reconstruct(node):

    path=[]

    while node:
        path.append(node['pos'])
        node=node['parent']

    return path[::-1]



def astar(start,goal):

    open_list=[]
    heapq.heappush(open_list,(0,start))

    nodes={}

    nodes[start]={
        'pos':start,
        'g':0,
        'h':heuristic(start,goal),
        'f':heuristic(start,goal),
        'parent':None
    }

    closed=set()

    while open_list:

        _,pos = heapq.heappop(open_list)

        if pos in closed:
            continue

        node = nodes[pos]

        if pos==goal:
            return reconstruct(node)

        closed.add(pos)

        for nb in get_neighbors(pos):

            if nb in closed:
                continue

            g = node['g'] + 1

            if nb not in nodes or g < nodes[nb]['g']:

                nodes[nb]={
                    'pos':nb,
                    'g':g,
                    'h':heuristic(nb,goal),
                    'f':g + heuristic(nb,goal),
                    'parent':node
                }

                heapq.heappush(open_list,(nodes[nb]['f'],nb))

    return []



def direction_from_cells(a,b):

    x1,y1 = a
    x2,y2 = b

    if x2==x1 and y2==y1+1: return NORTH
    if x2==x1+1 and y2==y1: return EAST
    if x2==x1 and y2==y1-1: return SOUTH
    if x2==x1-1 and y2==y1: return WEST



def turn_left(state):

    API.turnLeft()
    state['facing']=(state['facing']+3)%4



def turn_right(state):

    API.turnRight()
    state['facing']=(state['facing']+1)%4



def turn_to_face(target,state):

    current=state['facing']

    r=(target-current)%4

    if r==1:
        turn_right(state)
    elif r==2:
        turn_right(state)
        turn_right(state)
    elif r==3:
        turn_left(state)



def move_forward(state):

    API.moveForward()

    state['x'] += DX[state['facing']]
    state['y'] += DY[state['facing']]

    API.setColor(state['x'],state['y'],"G")



def main():

    log("=== A* Micromouse ===")

    init_border_walls()

    state={'x':0,'y':0,'facing':NORTH}

    API.setColor(0,0,"G")

    while (state['x'],state['y']) not in GOAL_CELLS:

        x,y = state['x'],state['y']

        sense_and_record_walls(x,y,state['facing'])

        goal = min(GOAL_CELLS, key=lambda g: heuristic((x,y),g))

        path = astar((x,y),goal)

        if len(path)<2:
            log("No path found")
            break

        next_cell = path[1]

        target_dir = direction_from_cells((x,y),next_cell)

        log(f"Moving {DIR_NAMES[target_dir]}")

        turn_to_face(target_dir,state)

        move_forward(state)

        if API.wasReset():

            log("Reset detected")

            API.ackReset()

            API.clearAllColor()

            walls[:] = [[set() for _ in range(HEIGHT)] for _ in range(WIDTH)]

            init_border_walls()

            state={'x':0,'y':0,'facing':NORTH}

            API.setColor(0,0,"G")


    log("Goal reached!")

    API.setColor(state['x'],state['y'],"Y")



if __name__=="__main__":
    main()
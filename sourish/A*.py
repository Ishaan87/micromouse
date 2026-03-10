from typing import List, Tuple, Dict, Set #import typing hint tools from Python library
import numpy as np #numpy used to represent the grid as matrix
import heapq #library used to create priority queue (no need to scan everytime, it keeps thing automatically sorted): heappush- add node, heappop- remove lowest cost node

def create_node(position: Tuple[int, int], g: float = float('inf'), h: float = 0.0, parent: Dict = None) -> Dict:
    # every grid cell that we explore is treated as a node
    return { #dictionary with specifications of the node
        'position': position, #coordinates of the node
        'g': g, #cost from start node to this node (default: infinity as we assume node very far until better path is found)
        'h': h, #heuristic estimate from this node to the endpoint
        'f': g + h,
        'parent': parent #stores the previous node in path, useful for reconstruction of path
    }

def get_valid_neighbors(grid: np.ndarray, position: Tuple[int, int]) -> List[Tuple[int, int]]: #np.ndarray: N-dimensional array object in the numpy library of python
#grid: 2 dimensional numpy array where 0 represents walkable cells and 1 represents obstacles
    x, y = position #current location
    rows, cols = grid.shape #attribute of np.ndarray
    possible_moves = [
        (x+1, y), (x-1, y),   
        (x, y+1), (x, y-1) #not allowing diagonal movement at the moment
    ]
    return [
        (nx, ny) for nx, ny in possible_moves
        if 0 <= nx < rows and 0 <= ny < cols  #within the grid, not outside the maze
        and grid[nx, ny] == 0 #not an obstacle
    ] 

def calculate_heuristic(pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float: #used to calculate the "h" value in this algorithm
    x1, y1 = pos1 #current node position
    x2, y2 = pos2 #position of the goal
    return (abs(x2 - x1) + abs(y2 - y1))  # We are using Manhattan heuristics for first iteration (movement only in four directions)

def find_path(grid: np.ndarray, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
    # we provide the grid, starting & ending coordinate. It will return list of positions representing shortest path
    start_node = create_node( 
        position=start,
        g=0,
        h=calculate_heuristic(start, goal)
    )
    open_list = [(start_node['f'], start)]
    open_dict = {start: start_node}
    closed_set = set()

    while open_list:
        val, current_pos= heapq.heappop(open_list)
        if current_pos in closed_set:
            continue
        current_node= open_dict[current_pos]

        if current_pos==goal:
            return reconstruct_path(current_node)
        
        closed_set.add(current_pos)
        for neighbour_pos in get_valid_neighbors(grid, current_pos):
            if neighbour_pos in closed_set:
                continue #already checked
            tentative_g= current_node['g'] + 1

            if neighbour_pos not in open_dict:
                neighbour= create_node(
                    position= neighbour_pos,
                    g= tentative_g,
                    h= calculate_heuristic(neighbour_pos, goal),
                    parent= current_node
                )
                heapq.heappush(open_list, (neighbour['f'], neighbour_pos))
                open_dict[neighbour_pos]= neighbour

            elif tentative_g < open_dict[neighbour_pos]['g']:
                neighbour= open_dict[neighbour_pos]
                neighbour['g']= tentative_g
                neighbour['f']= tentative_g + neighbour['h']
                neighbour['parent']= current_node
                heapq.heappush(open_list, (neighbour['f'], neighbour_pos))
    return []

def reconstruct_path(goal_node: Dict) -> List[Tuple[int, int]]: #Trace the winning path back to the start
    path = []
    current = goal_node #begin at goal and work backwards using parent pointers
    
    while current is not None: #loop ends when we reach start node because it has no parent
        path.append(current['position']) #add current node's position
        current = current['parent'] #then move to its parent
        
    return path[::-1] #outputs the list of coordinates representing full path from start to goal

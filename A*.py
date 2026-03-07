from typing import List, Tuple, Dict, Set
import numpy as np
import heapq
from math import sqrt

def create_node(position: Tuple[int, int], g: float = float('inf'), h: float = 0.0, parent: Dict = None) -> Dict:
    return {
        'position': position,
        'g': g,
        'h': h,
        'f': g + h,
        'parent': parent
    }

def calculate_heuristic(pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
    x1, y1 = pos1
    x2, y2 = pos2
    return (abs(x2 - x1) + abs(y2 - y1))  # We are using Manhattan heuristics for first iteration (movement only in four directions)

def get_valid_neighbors(grid: np.ndarray, position: Tuple[int, int]) -> List[Tuple[int, int]]:
#grid: 2D numpy array where 0 represents walkable cells and 1 represents obstacles
    x, y = position
    rows, cols = grid.shape
    possible_moves = [
        (x+1, y), (x-1, y),   
        (x, y+1), (x, y-1) #not allowing diagonal movement at the moment
    ]
    return [
        (nx, ny) for nx, ny in possible_moves
        if 0 <= nx < rows and 0 <= ny < cols  
        and grid[nx, ny] == 0 #not an obstacle
    ]

def reconstruct_path(goal_node: Dict) -> List[Tuple[int, int]]:
    path = []
    current = goal_node
    
    while current is not None:
        path.append(current['position'])
        current = current['parent']
        
    return path[::-1]


def find_path(grid: np.ndarray, start: Tuple[int, int], goal: Tuple[int, int]) -> List[Tuple[int, int]]:
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

def mazeWidth() -> int:
    pass

def mazeHeight() -> int:
    pass

def wallFront(numHalfSteps: int = 1) -> bool:
    pass

def wallRight(numHalfSteps: int = 1) -> bool:
    pass

def wallLeft(numHalfSteps: int = 1) -> bool:
    pass

def wallBack(numHalfSteps: int = 1) -> bool:
    pass

def moveForward(distance: int = 1) -> None:
    pass

def moveForwardHalf(numHalfSteps: int = 1) -> None:
    pass

def turnRight() -> None:
    pass

def turnLeft() -> None:
    pass

def turnRight45() -> None:
    pass

def turnLeft45() -> None:
    pass

def setWall(x: int, y: int, direction: str) -> None:
    pass

def clearWall(x: int, y: int, direction: str) -> None:
    pass

def setColor(x: int, y: int, color: str) -> None:
    pass

def clearColor(x: int, y: int) -> None:
    pass

def clearAllColor() -> None:
    pass

def setText(x: int, y: int, text: str) -> None:
    pass

def clearText(x: int, y: int) -> None:
    pass

def clearAllText() -> None:
    pass

def wasReset() -> bool:
    pass

def ackReset() -> None:
    pass

def getStat(stat: str) -> float:
    pass
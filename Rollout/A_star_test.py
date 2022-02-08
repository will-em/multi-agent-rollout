# Credit for this: Nicholas Swift
# as found at https://medium.com/@nicholas.w.swift/easy-a-star-pathfinding-7e6689c7f7b2
import numpy as np
from A_star import astar
import pyastar2d
import time
def main():

    maze = np.array([[1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
                     [1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
                     [1, 0, 1, 1, 1, 1, 1, 1, 1, 1],
                     [1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
                     [1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
                     [1, 0, 1, 1, 1, 1, 1, 0, 1, 1],
                     [1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
                     [1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
                     [1, 0, 0, 0, 0, 0, 0, 0, 0, 1],
                     [1, 1, 1, 1, 1, 1, 1, 1, 1, 1]], dtype=np.float32)

    start = (8, 1)
    end = (5, 7)
    py_start = time.time()
    for i in range(1000):
        path = astar(maze, start, end)
    print(f"Python time: {time.time()-py_start}")
    print(path)
    c_start = time.time()
    for i in range(1000):
        path = pyastar2d.astar_path(maze+1.0, start, end, allow_diagonal=False)
    print(f"C++ time: {time.time()-c_start}")
    print(path)
    '''
    delta_to_action = {
        (-1, 0): 1,
        (1, 0): 2,
        (0, -1): 3,
        (0, 1): 4}
    action = []
    for i in range(1, len(path)):
        action.append(
            delta_to_action[(path[i][0]-path[i-1][0], path[i][1]-path[i-1][1])])
    print(path)
    print(action)
    for pos in path:
        maze[pos[0]][pos[1]] = "-5"
    print(maze)
    '''
   

if __name__ == '__main__':
    main()

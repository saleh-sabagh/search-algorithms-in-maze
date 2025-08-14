"""
This file contains function headers for various pathfinding algorithms.
Students are responsible for implementing these algorithms.

IMPORTANT:
- Do NOT include any graphical elements in this file.
- Use `update_visualization function` to update the visualization.
"""

# imports are here 
from collections import deque , defaultdict
import datetime
from graphics import update_visualization , reconstruct_path
import heapq


def heuristic(cell, goal):
    """
    Computes the heuristic distance between two points.
    """
    return abs(goal[1] - cell[1]) + abs(goal[0] - cell[0])

def A_star(graph, start, goal, pathMap, ax2, ax3):
    """
    Arguments:
    - graph: The adjacency list representing the maze.
    - start: The starting position 
    - goal: The goal position 
    - pathMap: The matrix representation of the maze.
    - ax2: The visualization axis for search progress.
    - ax3: The visualization axis for the final path.
    """
    current = start
    open_list = []
    expanded_map = pathMap.copy()
    expanded = 0
    closed_set = set() 
    came_from = defaultdict(None)
    heapq.heappush(open_list , (0,[current , 0]))
    t0=datetime.datetime.now()
    while open_list:
        expanded+=1
        f , [current , g] = heapq.heappop(open_list)
        expanded_map[current] = 56
        update_visualization(expanded_map , ax2 , "A* search")
        if current == goal :
            t1=datetime.datetime.now()
            elapsed = (t1-t0).total_seconds()
            reconstruct_path(came_from , current , pathMap , ax3 , "Final path with A*" , start , goal)
            return None , expanded , elapsed
        
        if current in closed_set:
            continue
        closed_set.add(current)
        for node in graph[current]:
            if node not in closed_set:
                came_from[node] = current
                h = heuristic(node,goal)
                f = h + g+1
                heapq.heappush(open_list , (f , [node , g+1]))
        

def bfs(graph, start, goal, pathMap, ax2, ax3):
    """
    Arguments:
    - graph: The adjacency list representing the maze.
    - start: The starting position 
    - goal: The goal position
    - pathMap: The matrix representation of the maze.
    - ax2: The visualization axis for search progress.
    - ax3: The visualization axis for the final path.
    """
    visited = set()
    current = start    
    queue = deque()
    expanded = 0
    came_from = defaultdict(None)
    t0=datetime.datetime.now()
    expanded_map = pathMap.copy()
    while True:
        expanded_map[current]=56
        update_visualization(expanded_map , ax2 , "BFS Serach")
        expanded+=1
        if current == goal:
            t1=datetime.datetime.now()
            elapsed = (t1-t0).total_seconds()
            reconstruct_path(came_from , current , pathMap , ax3 , "Final path with BFS" , start , goal)
            return None , expanded , elapsed 
        
        visited.add(current)
        for node in graph[current]:
            if node not in visited:
                came_from[node] = current
                queue.append(node)
        try:
            temp = queue.popleft()
            current = temp
        except IndexError:
            break

def dfs(graph, start, goal, pathMap, ax2, ax3):
    """
    Arguments:
    - graph: The adjacency list representing the maze.
    - start: The starting position 
    - goal: The goal position 
    - pathMap: The matrix representation of the maze.
    - ax2: The visualization axis for search progress.
    - ax3: The visualization axis for the final path.
    """
    visited = set()
    current = start    
    stack = []
    expanded = 0
    came_from = defaultdict(None)
    t0=datetime.datetime.now()
    expanded_map = pathMap.copy()
    while True:
        expanded_map[current]=56
        update_visualization(expanded_map , ax2 , "DFS Serach")
        expanded+=1
        if current == goal:
            t1=datetime.datetime.now()
            elapsed = (t1-t0).total_seconds()
            reconstruct_path(came_from , current , pathMap , ax3 , "Final path with DFS" , start , goal)
            return None , expanded , elapsed 
        
        visited.add(current)
        for node in graph[current]:
            if node not in visited:
                stack.append(node)
        try:
            temp = stack.pop()
            if temp in graph[current]:
                came_from[temp] = current
            else:
                p = came_from[current]
                while True:
                    if temp in graph[p]:
                        came_from[temp] = p
                        break
                    p=came_from[p]
            current = temp
        except IndexError:
            break

    


def greedy(graph, start, goal, pathMap, ax2, ax3):
    """
    Arguments:
    - graph: The adjacency list representing the maze.
    - start: The starting position 
    - goal: The goal position 
    - pathMap: The matrix representation of the maze.
    - ax2: The visualization axis for search progress.
    - ax3: The visualization axis for the final path.
    """
    current = start
    open_list = []
    expanded_map = pathMap.copy()
    expanded = 0
    closed_set = set() 
    came_from = defaultdict(None)
    heapq.heappush(open_list , (0, current ))
    t0=datetime.datetime.now()
    while open_list:
        expanded+=1
        h , current = heapq.heappop(open_list)
        expanded_map[current] = 56
        update_visualization(expanded_map , ax2 , "Greedy search")
        if current == goal :
            t1=datetime.datetime.now()
            elapsed = (t1-t0).total_seconds()
            reconstruct_path(came_from , current , pathMap , ax3 , "Final path with Greedy search" , start , goal)
            return None , expanded , elapsed
        
        if current in closed_set:
            continue
        closed_set.add(current)
        for node in graph[current]:
            if node not in closed_set:
                came_from[node] = current
                h = heuristic(node,goal)
                heapq.heappush(open_list , (h , node))

def depth_limited_search(graph, current, goal, depth, came_from, pathMap, ax2, expanded_nodes):
    """
    Arguments:
    - graph: The adjacency list representing the maze.
    - current: The current position
    - goal: The goal position 
    - depth: The current depth limit
    - came_from: Dictionary tracking visited nodes.
    - pathMap: The matrix representation of the maze.
    - ax2: The visualization axis for search progress.
    - expanded_nodes: The count of expanded nodes.
    """
    def depth_size(current ,came_from):
        if current == (0,0):
            return 1
        temp=current
        d_size = 1
        while True:
            temp = came_from[temp]
            d_size+=1
            if temp == (0,0):
                
                return d_size
            
            
            
        
    
    visited = set()  
    stack = []
    came_from = defaultdict(None)
    expanded_map = pathMap.copy()
    while True:
        expanded_nodes+=1
        expanded_map[current]=56
        update_visualization(expanded_map,ax2,f"Depth limited search with depth {depth}")
        if current == goal:
            return came_from , expanded_nodes ,current,True
        visited.add(current)
        for node in graph[current]:
            if node not in visited:

                if depth_size(current , came_from)>depth:
                    visited.add(node)
                    continue

                stack.append(node)
        if stack:
            
            temp = stack.pop()
            if temp in graph[current]:
                came_from[temp] = current
            else:
                p = came_from[current]
                while True:
                    if temp in graph[p]:
                        came_from[temp] = p
                        break
                    p=came_from[p]
            current = temp
        else:
            return came_from , expanded_nodes ,(0,0), False


def iterative_deepening_search(graph, start, goal, pathMap, ax2, ax3):
    """
    Arguments:
    - graph: The adjacency list representing the maze.
    - start: The starting position 
    - goal: The goal position 
    - pathMap: The matrix representation of the maze.
    - ax2: The visualization axis for search progress.
    - ax3: The visualization axis for the final path.
    """
    current = start
    expanded = 0
    t0 = datetime.datetime.now()
    for depth in range(len(pathMap)**2):
        came_from , expanded ,current, key = depth_limited_search(graph , current , goal , depth , [] , pathMap ,ax2 , expanded)
        if key:
            
            break
    t1 = datetime.datetime.now()  
    reconstruct_path(came_from ,current ,pathMap , ax3 , "IDS Final path",start , goal )
    return None , expanded , (t1-t0).total_seconds()


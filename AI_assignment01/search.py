###### Write Your Library Here ###########


import queue


#########################################


def search(maze, func):
    return {
        "bfs": bfs,
        "astar": astar,
        "astar_four_circles": astar_four_circles,
        "astar_many_circles": astar_many_circles
    }.get(func)(maze)


# -------------------- Stage 01: One circle - BFS Algorithm ------------------------ #

def bfs(maze):
    """
    [문제 01] 제시된 stage1의 맵 세가지를 BFS Algorithm을 통해 최단 경로를 return하시오.(20점)
    """
    start_point=maze.startPoint()

    path=[]

    ####################### Write Your Code Here ################################

    bfsQueue = []
    bfsQueue.append(start_point)
    visited = set()
    visited.add(start_point)
    record = {}

    while bfsQueue:
        start_point = bfsQueue.pop(0)

        if maze.isObjective(start_point[0], start_point[1]):
            path = [start_point]
            
            while path[-1] != maze.startPoint():
                path.append(record[path[-1]])
            path.reverse()
            return path

        neighborPoints = maze.neighborPoints(start_point[0], start_point[1])

        for point in neighborPoints:
            if point not in visited and point not in bfsQueue:
                record[point] = start_point
                bfsQueue.append(point)
                visited.add(point)

    ############################################################################



class Node:
    def __init__(self,parent,location):
        self.parent=parent
        self.location=location #현재 노드

        self.obj=[]

        # F = G+H
        self.f=0
        self.g=0
        self.h=0

    def __eq__(self, other):
        return self.location==other.location and str(self.obj)==str(other.obj)

    def __le__(self, other):
        return self.g+self.h<=other.g+other.h

    def __lt__(self, other):
        return self.g+self.h<other.g+other.h

    def __gt__(self, other):
        return self.g+self.h>other.g+other.h

    def __ge__(self, other):
        return self.g+self.h>=other.g+other.h


# -------------------- Stage 01: One circle - A* Algorithm ------------------------ #

def manhatten_dist(p1,p2):
    return abs(p1[0]-p2[0])+abs(p1[1]-p2[1])

def astar(maze):

    """
    [문제 02] 제시된 stage1의 맵 세가지를 A* Algorithm을 통해 최단경로를 return하시오.(20점)
    (Heuristic Function은 위에서 정의한 manhatten_dist function을 사용할 것.)
    """

    start_point=maze.startPoint()

    end_point=maze.circlePoints()[0]

    path=[]

    ####################### Write Your Code Here ################################

    astarQueue = queue.PriorityQueue()
    newNode = (manhatten_dist(start_point, end_point), start_point)
    visited = set()
    visited.add(start_point)
    record = {}
    astarQueue.put(newNode)

    while astarQueue:
        agent = astarQueue.get()
        agentLocation = agent[1]

        if agentLocation == end_point:
            path = [end_point]
            
            while path[-1] != start_point:
                path.append(record[path[-1]])
            path.reverse()
            return path
            
        neighborPoints = maze.neighborPoints(agentLocation[0], agentLocation[1])

        for i in neighborPoints:
            if i not in visited:
                record[i] = agentLocation

                cost = [i]
                while cost[-1] != start_point:
                    cost.append(record[cost[-1]])

                aNewNode = (manhatten_dist(i, end_point)+ len(cost), i)
                cost.clear
                astarQueue.put(aNewNode)
                visited.add(agentLocation)

    ############################################################################


# -------------------- Stage 02: Four circles - A* Algorithm  ------------------------ #



def stage2_heuristic(agent, end_points, pointLeft):
    dists = []
    for i in range(pointLeft):
        dists.append(abs(agent[0]-end_points[i][0])+abs(agent[1]-end_points[i][1]))

    return min(dists)


def astar_four_circles(maze):
    """
    [문제 03] 제시된 stage2의 맵 세가지를 A* Algorithm을 통해 최단 경로를 return하시오.(30점)
    (단 Heurstic Function은 위의 stage2_heuristic function을 직접 정의하여 사용해야 한다.)
    """

    end_points=maze.circlePoints()
    end_points.sort()

    path=[]

    ####################### Write Your Code Here ################################

    start_point=maze.startPoint()
    flag = start_point
    paths = []
    pointLeft = 4
    counter = 0

    astarQueue = queue.PriorityQueue()
    newNode = (stage2_heuristic(start_point, end_points, pointLeft), start_point)
    visited = set()
    visited.add(start_point)
    record = {}
    astarQueue.put(newNode)

    while astarQueue:
        agent = astarQueue.get()
        agentLocation = agent[1]

        if agentLocation in end_points:
            pointLeft -= 1
            counter += 1
            path = [agentLocation]

            while path[-1] != flag:
                path.append(record[path[-1]])
            path.reverse()
            paths = paths + path
            path.clear

            if counter != 4:
                astarQueue = queue.PriorityQueue()
                newNode = (stage2_heuristic(agentLocation, end_points, pointLeft), agentLocation)
                visited = set()
                visited.add(agentLocation)
                record.clear
                astarQueue.put(newNode)
            else:
                return paths
        
            flag = agentLocation
            end_points.remove(flag)

        neighborPoints = maze.neighborPoints(agentLocation[0], agentLocation[1])

        for point in neighborPoints:
            if point not in visited:
                record[point] = agentLocation

                aNewNode = (stage2_heuristic(point, end_points, pointLeft), point)
                astarQueue.put(aNewNode)
                visited.add(agentLocation)
        path.append(agentLocation)

    ############################################################################



# -------------------- Stage 03: Many circles - A* Algorithm -------------------- #

def mst(objectives, edges):

    cost_sum=0
    ####################### Write Your Code Here ################################













    return cost_sum

    ############################################################################


def stage3_heuristic():
    pass


def astar_many_circles(maze):
    """
    [문제 04] 제시된 stage3의 맵 세가지를 A* Algorithm을 통해 최단 경로를 return하시오.(30점)
    (단 Heurstic Function은 위의 stage3_heuristic function을 직접 정의하여 사용해야 하고, minimum spanning tree
    알고리즘을 활용한 heuristic function이어야 한다.)
    """

    end_points= maze.circlePoints()
    end_points.sort()

    path=[]

    ####################### Write Your Code Here ################################





















    return path

    ############################################################################

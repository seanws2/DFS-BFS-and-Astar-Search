# search.py
# ---------------
# Licensing Information:  You are free to use or extend this projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to the University of Illinois at Urbana-Champaign
#
# Created by Michael Abir (abir2@illinois.edu) on 08/28/2018

"""
This is the main entry point for MP1. You should only modify code
within this file -- the unrevised staff files will be used for all other
files and classes when code is run, so be careful to not modify anything else.
"""
# Search should return the path.
# The path should be a list of tuples in the form (row, col) that correspond
# to the positions of the path taken by your search algorithm.
# maze is a Maze object based on the maze from the file specified by input filename
# searchMethod is the search method specified by --method flag (bfs,dfs,astar,astar_multi,extra)
import math
import itertools
import queue
import numpy as np

def search(maze, searchMethod):
    return {
        "bfs": bfs,
        "astar": astar,
        "astar_corner": astar_corner,
        "astar_multi": astar_multi,
        "extra": extra,
    }.get(searchMethod)(maze)


def sanity_check(maze, path):
    """
    Runs check functions for part 0 of the assignment.

    @param maze: The maze to execute the search on.
    @param path: a list of tuples containing the coordinates of each state in the computed path

    @return bool: whether or not the path pass the sanity check
    """
    # TODO: Write your code here
    return False


def bfs(maze):
    start = maze.getStart()
    BFS = queue.Queue()
    BFS.put(maze.getStart())
    # this is to log the path
    history = {}
    history[start] = None
    notEmpty = True
    keepGoin = True
    output = []

    while notEmpty:
        front = BFS.get()
        neighbors = maze.getNeighbors(front[0], front[1])

        if maze.isObjective(front[0], front[1]):
            output.append(front)
            prev_front = history[front]
            # build path taken

            while keepGoin:
                output.append(prev_front)
                prev_front = history[prev_front]
                if prev_front == start: keepGoin = False
            output.append(start)
            break

        for i in neighbors:
            if i in history:
                continue
            else:
                history[i] = front
                BFS.put(i)
        if BFS.empty():
            notEmpty = False
            break
    output.reverse()
    return output


def man_Dist(here, toThere, sum):
    # this feller calculates the city distance from here to there
    # basic heuristic
    for i in range(0, 2):
        sum += abs(here[i] - toThere[i])
    return sum


def astar1(maze, objective, state1):
    open = queue.PriorityQueue()
    history = {}
    history[state1] = [None]
    cost = {}
    cost[state1] = 0
    manDist = {}
    manDist[state1] = man_Dist(state1, objective, 0)
    output = []

    open.put((manDist[state1], state1))

    while open.empty != True:
        curr = open.get()[1]
        if curr == objective:
            while curr != state1:
                output.append(curr)
                curr = history[curr]
            output.append(state1)
            break

        Neighbors = maze.getNeighbors(curr[0], curr[1])
        for i in Neighbors:
            if i in cost:
                continue
            else:
                # print("i:", i)
                history[i] = curr
                cost[i] = cost[curr] + 1
                prior = cost[i] + man_Dist(i, objective, 0)
                open.put((prior, i))
                history[i] = curr
    return output


def astar(maze):
    """
    Runs A star for part 1 of the assignment.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    # TODO: Write your code here
    output = astar1(maze, maze.getObjectives()[0], maze.getStart())
    output.reverse()
    return output


def astar_corner(maze):
    """
    Runs A star for part 2 of the assignment in the case where there are four corner objectives.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
        """
    # TODO: Write your code here
    endNodes = maze.getObjectives()
    # endNodes.append(maze.getStart())
    start = maze.getStart()
    o_count = len(endNodes)
    paths = [[start]]
    snopes = True
    possible_paths_fromStart = []
    possible_paths = [None]
    i = 0
    getOut = False
    for o in endNodes:
        possible_paths_fromStart.append(astar1(maze, o, start))
        for o1 in endNodes:
            this_path = astar1(maze, o1, o)
            if len(this_path) > 1:

                if possible_paths[i] == None:
                    possible_paths[i] = this_path
                else:
                    possible_paths.append(this_path)
            if len(possible_paths) == len(endNodes) * (len(endNodes) - 1):
                getOut = True
                break
        i = i + 1
        if getOut: break

    allPaths = [possible_paths[8] + possible_paths[4] + possible_paths[0] + possible_paths_fromStart[0],
               possible_paths[11] + possible_paths[5] + possible_paths[0] + possible_paths_fromStart[0],
               possible_paths[5] + possible_paths[7] + possible_paths[1] + possible_paths_fromStart[0],
               possible_paths[10] + possible_paths[8] + possible_paths[1] + possible_paths_fromStart[0],
               possible_paths[7] + possible_paths[11] + possible_paths[2] + possible_paths_fromStart[0],
               possible_paths[4] + possible_paths[10] + possible_paths[2] + possible_paths_fromStart[0],
               possible_paths[8] + possible_paths[1] + possible_paths[3] + possible_paths_fromStart[1],
               possible_paths[11] + possible_paths[2] + possible_paths[3] + possible_paths_fromStart[1],
               possible_paths[2] + possible_paths[6] + possible_paths[4] + possible_paths_fromStart[1],
               possible_paths[9] + possible_paths[8] + possible_paths[4] + possible_paths_fromStart[1],
               possible_paths[1] + possible_paths[9] + possible_paths[5] + possible_paths_fromStart[1],
               possible_paths[6] + possible_paths[11] + possible_paths[5] + possible_paths_fromStart[1],
               possible_paths[5] + possible_paths[0] + possible_paths[6] + possible_paths_fromStart[2],
               possible_paths[10] + possible_paths[2] + possible_paths[6] + possible_paths_fromStart[2],
               possible_paths[2] + possible_paths[3] + possible_paths[7] + possible_paths_fromStart[2],
               possible_paths[9] + possible_paths[5] + possible_paths[7] + possible_paths_fromStart[2],
               possible_paths[0] + possible_paths[9] + possible_paths[8] + possible_paths_fromStart[2],
               possible_paths[3] + possible_paths[10] + possible_paths[8] + possible_paths_fromStart[2],
               possible_paths[4] + possible_paths[0] + possible_paths[9] + possible_paths_fromStart[3],
               possible_paths[7] + possible_paths[1] + possible_paths[9] + possible_paths_fromStart[3],
               possible_paths[1] + possible_paths[3] + possible_paths[10] + possible_paths_fromStart[3],
               possible_paths[6] + possible_paths[4] + possible_paths[10] + possible_paths_fromStart[3],
               possible_paths[0] + possible_paths[6] + possible_paths[11] + possible_paths_fromStart[3],
               possible_paths[3] + possible_paths[7] + possible_paths[11] + possible_paths_fromStart[3]]

    for path in allPaths:
        i = 1
        while i < len(path):
            if path[i-1] == path[i]: path.remove(path[i])
            i +=1
    mini = 100000000000
    i = None
    for path in allPaths:
        if len(path) < mini:
            mini = len(path)
            i = path

    i.reverse()
    return i


def astar_multi(maze):
    state1 = maze.getStart()
    endNodes = maze.getObjectives()
    if len(endNodes) > 10: return []
    possible_paths = {}
    possible_paths_fromStart = {}
    # construct all singular paths
    mini = 10000000000000
    for o in endNodes:
        path = astar1(maze, o, state1)
        possible_paths[(state1, o)] = path
        possible_paths_fromStart[(state1, o)] = len(path)
        for o1 in endNodes:
            this_path = astar1(maze, o1, o)
            if len(this_path) > 1:
                possible_paths[(o, o1)] = this_path
                possible_paths_fromStart[(o, o1)] = len(this_path)
    res = itertools.permutations(endNodes)
    out = None
    output = None
    count = 0

    for i in res:
        pathl = 0
        for l in range(0, 10):
            if l == 0:
                pathl += possible_paths_fromStart[(state1, i[l])]
            else:
                pathl += possible_paths_fromStart[(i[l-1], i[l])] - 1
        if pathl < mini:
            mini = pathl
            out = i
        count += 1
        if count > 3628800: break
    #build path
    for i in out:
        count = 0
        if i == out[0]:
            output = possible_paths[(state1, i)]
        else:
            save = output[0]
            k = possible_paths[(save, i)]
            k.remove(save)
            output = k+output

        for j in endNodes :
            if output != None:
                if j in output: count += 1
        if j == len(endNodes): break
    output.reverse()
    return output

def extra(maze):
    """
    Runs suboptimal search algorithm for part 4.

    @param maze: The maze to execute the search on.

    @return path: a list of tuples containing the coordinates of each state in the computed path
    """
    # TODO: Write your code here
    endNodes = (maze.getObjectives())
    # target = endNodes[0]

    start = maze.getStart()
    mergedPaths = [start]
    i = 0
    count = 0
    while len(endNodes) != 0:
        j = 0
        mini = 10000000000000
        thePatherino = None
        k = 0
        while j < len(endNodes):
            patherino = astar1(maze, endNodes[j], start)
            if len(patherino) < mini and len(patherino) != 0:
                thePatherino = patherino
                mini = len(patherino)
                k = j
            j = j + 1
        start = endNodes[k]
        endNodes.remove(endNodes[k])
        thePatherino.remove(thePatherino[len(thePatherino) - 1])
        mergedPaths = thePatherino + mergedPaths
    print(mergedPaths)
    return mergedPaths

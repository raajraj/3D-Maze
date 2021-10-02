#!/usr/bin/env python3
import numpy as np
from collections import deque
import uuid
import operator
import copy
import time

class path:
    def __init__(self, dataval= None):
          self.dataval = dataval
          self.parent = None
          self.next = None
          self.cost = None
          self.pathCost = 0
          self.f = 0

t0 = time.time()

def findH(entrance, exit):
    diff = tuple(i-j for i,j in zip(entrance, exit))
    return abs(diff[0]) + abs(diff[1]) + abs(diff[2])

def findUCSACost(parent, child):
    cost = 0
    p = parent.dataval
    diff = tuple(i-j for i,j in zip(p, child))
    totalDiff = abs(diff[0]) + abs(diff[1]) + abs(diff[2])
    if(parent == None):
        cost = 0
    else:
        if(totalDiff == 1):
            cost = 10
        elif(totalDiff == 2):
            cost = 14
        else:
            cost = 0
    return cost

def run(node, a):
    child = node[:]
    if(a == 1):
        child[0] = child[0] + 1
    elif(a == 2):
        child[0] = child[0] - 1
    elif(a == 3):
        child[1] = child[1] + 1
    elif(a == 4):
        child[1] = child[1] - 1
    elif(a == 5):
        child[2] = child[2] + 1
    elif(a == 6):
        child[2] = child[2] - 1
    elif(a == 7):
        child[0] = child[0] + 1
        child[1] = child[1] + 1
    elif(a == 8):
        child[0] = child[0] + 1
        child[1] = child[1] - 1
    elif(a == 9):
        child[0] = child[0] - 1
        child[1] = child[1] + 1
    elif(a == 10):
        child[0] = child[0] - 1
        child[1] = child[1] - 1
    elif(a == 11):
        child[0] = child[0] + 1
        child[2] = child[2] + 1
    elif(a == 12):
        child[0] = child[0] + 1
        child[2] = child[2] - 1
    elif(a == 13):
        child[0] = child[0] - 1
        child[2] = child[2] + 1
    elif(a == 14):
        child[0] = child[0] - 1
        child[2] = child[2] - 1
    elif(a == 15):
        child[1] = child[1] + 1
        child[2] = child[2] + 1
    elif(a == 16):
        child[1] = child[1] + 1
        child[2] = child[2] - 1
    elif(a == 17):
        child[1] = child[1] - 1
        child[2] = child[2] + 1
    elif(a == 18):
        child[1] = child[1] - 1
        child[2] = child[2] - 1
    return child

def findBFScost(x, entrance):
    if(x == entrance):
        return "0"
    else:
        return "1"

def findMinf(open):
    min = open[0].f
    minNode = open[0]
    for x in open:
        if(x.f < min):
            min = x.f
            minNode = x
    return minNode

def printBFSSolution(path, entrance):
    steps = 0
    route = []
    for x in path:
        temp = str(x[0]) + " " + str(x[1]) + " " + str(x[2]) + " " + findBFScost(x, entrance)
        route.append(temp)
        steps = steps + 1
    f = open("output.txt", "w")
    f.write(str(steps-1) + "\n")
    f.write(str(steps) + "\n")
    for x in route:
        f.write(x)
        f.write("\n")
    return

def printSolution(thisvalue):
    steps = 0
    totalCost = 0
    route = []
    while thisvalue:
        temp = str(thisvalue.dataval[0]) +  " " + str(thisvalue.dataval[1]) + " " + str(thisvalue.dataval[2]) + " " + str(thisvalue.cost)
        route.append(temp)
        steps = steps + 1
        totalCost = totalCost + thisvalue.cost
        thisvalue = thisvalue.parent
    f = open("output.txt", "w")
    f.write(str(totalCost) + "\n")
    f.write(str(steps) + "\n")
    for x in reversed(route):
        f.write(x)
        f.write("\n")
    return

def getData(i):
    i = i.split()
    data = [int(x) for x in i]
    return data

def getNode(q, kid):
    for m in q:
        if(kid == m.dataval):
            return m

#--------------------------------------------------------------------------------------------#
# BREADTH_FIRST SEARCH #
#--------------------------------------------------------------------------------------------#

def BFS(graph, entrance, exit):
    start = [entrance]
    q = deque()
    visited = set()
    q.append(start)
    while q:
        path = q.pop()
        node = path[-1]
        if tuple(node) not in visited:
            visited.add(tuple(node))
            neighbors = graph[tuple(node)]
            for n in neighbors:
                child = run(node, n)
                npath = list(path)
                npath.append(child)
                q.appendleft(npath)
                if(child == exit):
                    printBFSSolution(npath, entrance)
                    return
        if(len(q) == 0):
            f = open("output.txt", "w")
            f.write("FAIL")
            return
    return

#--------------------------------------------------------------------------------------------#
# UNIFORM-COST SEARCH #
#--------------------------------------------------------------------------------------------#

def UCS(graph, entrance, exit):
    root = path(entrance)
    root.cost = 0
    root.pathCost = 0
    q = deque()
    visited = set()
    frontier = set()
    q.append(root)
    frontier.add(tuple(entrance))
    while (1):
        if(len(q) == 0):
            f = open("output.txt", "w")
            f.write("FAIL")
            return
        node = q.pop()
        val = node.dataval
        frontier.remove(tuple(val))
        if(val == exit):
            printSolution(root)
            return
        visited.add(tuple(val))
        neighbors = graph[tuple(val)]
        for n in neighbors:
            child = run(val, n)
            kid = path(child)
            kid.parent = root
            kid.cost = findUCSACost(kid.parent, kid.dataval)
            kid.pathCost = kid.cost + kid.parent.pathCost
            if((tuple(child) not in visited) & (tuple(child) not in frontier)):
                q.append(kid)
                frontier.add(tuple(child))
            elif((tuple(child) in frontier) & (kid.pathCost > node.pathCost)):
                cnode = getNode(q, child)
                q.remove(cnode)
                q.append(kid)
        q = sorted(q, key = operator.attrgetter('pathCost'), reverse = True)
        root.next = q[len(q)-1]
        root = root.next

#--------------------------------------------------------------------------------------------#
# A* SEARCH #
#--------------------------------------------------------------------------------------------#

def A(graph, entrance, exit):
    root = path(entrance)
    root.cost = 0
    root.pathCost = 0
    root.f = findH(entrance, exit)
    q = deque()
    frontier = set()
    visited = set()
    q.append(root)
    frontier.add(tuple(entrance))
    while q:
        node = findMinf(q)
        val = node.dataval
        if(val == exit):
            printSolution(node)
            return
        q.remove(node)
        #frontier.remove(tuple(val))
        neighbors = graph[tuple(val)]
        for n in neighbors:
            child = run(val, n)
            kid = path(child)
            kid.parent = node
            kid.cost = findUCSACost(kid.parent, kid.dataval)
            kid.pathCost = kid.cost + kid.parent.pathCost
            kid.f = findH(kid.dataval, exit) + kid.pathCost
            if(tuple(child) not in visited):
                if(tuple(child) in frontier):
                    qnode = getNode(q, child)
                    if(kid.pathCost <= qnode.pathCost):
                        q.append(kid)
                        frontier.add(tuple(kid.dataval))
                else:
                    q.append(kid)
                    frontier.add(tuple(kid.dataval))
        visited.add(tuple(val))
        if(len(q) == 0):
            f = open("output.txt", "w")
            f.write("FAIL")
            return

# read input.txt in the current directory
f = open("input.txt", "r")

# read the first line to see if we use BFS, UCS, or A*
method = f.readline()
# read in data
input = f.readline()
size = getData(input)
input = f.readline()
entrance = getData(input)
input = f.readline()
exit = getData(input)
input = f.readline()
availActions = getData(input)
lines = f.readlines()
graph = {}
for l in lines:
    actions = []
    l = l.split()
    t = (int(l[0]), int(l[1]), int(l[2]))
    for x in range(3, len(l)):
        actions.append(int(l[x]))
    graph[t] = np.array(actions)

# start sorting by method
if(method == "BFS\n"):
    BFS(graph, entrance, exit)
elif(method == "UCS\n"):
    UCS(graph, entrance, exit)
elif(method == "A*\n"):
    A(graph, entrance, exit)


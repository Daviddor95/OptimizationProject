import math
from itertools import product
from networkx import minimum_cut, DiGraph
from mip import Model, xsum, BINARY, OptimizationStatus, CutType
import time

def minBasis(adj, i):
    min = float('inf')
    for e0 in range(n):
        if adj[i][e0] < min and i != e0:
            min = adj[i][e0]
    return min


def regularMin(adj, i):
    min0, min1 = float('inf'), float('inf')
    for e1 in range(n):
        if i != e1:
            if adj[i][e1] <= min0:
                min1 = min0
                min0 = adj[i][e1]
            elif adj[i][e1] <= min1 and adj[i][e1] != min0:
                min1 = adj[i][e1]
    return min1


def TSPRecBB(theMatrix, currentBound, currentWeight,
           lev, currentP, isVisit):
    global final_res
    if lev == n:
        if theMatrix[currentP[lev - 1]][currentP[0]] != 0:
            curr_res = currentWeight + theMatrix[currentP[lev - 1]] \
                [currentP[0]]
            if curr_res < final_res:
                final_path[:n + 1] = currentP[:]
                final_path[n] = currentP[0]
                final_res = curr_res
        return

    for i in range(n):
        if (theMatrix[currentP[lev - 1]][i] != 0 and
                isVisit[i] == False):
            tmp = currentBound
            currentWeight += theMatrix[currentP[lev - 1]][i]
            if lev == 1:
                currentBound = currentBound -(((minBasis(theMatrix, currentP[lev - 1]) +
                                  minBasis(theMatrix, i)) / 2))
            else:
                currentBound = currentBound - (((regularMin(theMatrix, currentP[lev - 1]) +
                                  minBasis(theMatrix, i)) / 2))
            if currentBound + currentWeight < final_res:
                isVisit[i] = True
                currentP[lev] = i
                TSPRecBB(theMatrix, currentBound, currentWeight,
                       lev + 1, currentP, isVisit)
            currentWeight -= theMatrix[currentP[lev - 1]][i]
            currentBound = tmp
            isVisit = [False] * len(isVisit)
            for j in range(lev):
                if currentP[j] != -1:
                    isVisit[currentP[j]] = True


def TSPForBranchAndBound(theMatrix):
    currentBound = 0
    currentP = [-1] * (n + 1)
    isVisit = [False] * n

    for i in range(n):
        currentBound += (minBasis(theMatrix, i) +
                       regularMin(theMatrix, i))

    currentBound = math.ceil(currentBound / 2)

    isVisit[0] = True
    currentP[0] = 0
    TSPRecBB(theMatrix, currentBound, 0, 1, currentP, isVisit)

def TSPRecCP(theMatrix, currentBound, currentWeight,
           lev, currentP, isVisit):
    global final_res
    if lev == n:
        if theMatrix[currentP[lev - 1]][currentP[0]] != 0:
            curr_res = currentWeight + theMatrix[currentP[lev - 1]] \
                [currentP[0]]
            if curr_res < final_res:
                final_path[:n + 1] = currentP[:]
                final_path[n] = currentP[0]
                final_res = curr_res
        return

    for i in range(n):
        if (theMatrix[currentP[lev - 1]][i] != 0 and
                isVisit[i] == False):
            tmp = currentBound
            currentWeight += theMatrix[currentP[lev - 1]][i]
            if lev == 1:
                currentBound = currentBound -(((minBasis(theMatrix, currentP[lev - 1]) +
                                  minBasis(theMatrix, i)) / 2))
            else:
                currentBound = currentBound - (((regularMin(theMatrix, currentP[lev - 1]) +
                                  minBasis(theMatrix, i)) / 2))
            if currentBound + currentWeight < final_res:
                isVisit[i] = True
                currentP[lev] = i
                TSPRecCP(theMatrix, currentBound, currentWeight,
                       lev + 1, currentP, isVisit)
            currentWeight -= theMatrix[currentP[lev - 1]][i]
            currentBound = tmp
            isVisit = [False] * len(isVisit)
            for j in range(lev):
                if currentP[j] != -1:
                    isVisit[currentP[j]] = True


def TSPForCP(theMatrix):
    currentBound = 0
    currentP = [-1] * (n + 1)
    isVisit = [False] * n

    for i in range(n):
        currentBound += (minBasis(theMatrix, i) +
                       regularMin(theMatrix, i))

    currentBound = math.ceil(currentBound / 2)

    isVisit[0] = True
    currentP[0] = 0
    TSPRecCP(theMatrix, currentBound, 0, 1, currentP, isVisit)

mat = [[0, 15, 10, 25],
       [15, 0, 30, 25],
       [10, 30, 0, 40],
       [25, 25, 40, 0]]
n = 4
final_path = [None] * (n + 1)
visited = [False] * n
final_res = float('inf')

start = time.time()
TSPForBranchAndBound(mat)
end = time.time()
print(end - start)


start = time.time()
TSPForCP(mat)
end = time.time()
print(end - start)

print("Minimum cost :", final_res)
print("Path Taken : ", end=' ')
for i in range(n + 1):
    print(final_path[i], end=' ')


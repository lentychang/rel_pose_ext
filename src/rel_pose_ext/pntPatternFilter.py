#!/usr/bin/python
# -*- coding: <utf-8> -*-

import random

import ipdb


def find_n_max(inputList):
    """[summary]
    compute the possible maximal (N)umber of matching points from one model.
    In practical the function should be applied on holeCountLists of two models. And then choose the minimal one.
    Arguments:
        inputList {list} -- each element reprecent the number of connections to a point.
                            The direction of connections is not considered.
                            Ex, PointA->point B and from point B-> pointA are consider as the same connection.
                            The connection number of point A is 1. Also connection number of point B is 1.

    Returns:
        maxPossibleN {int} -- N means the number of matching points
    """
    ls = list(inputList)
    ls.sort()
    n = len(ls)
    ipdb.set_trace(context=10)
    for i in range(max(ls), 0, -1):
        print(i)
        if i in ls:
            count = n - ls.index(i)
            if count >= i + 1:
                maxPossibleN = i + 1
                break
        else:
            # if in not in ls, then use val computed from previous computation
            if count >= i + 1:
                maxPossibleN = i + 1
                break
    return maxPossibleN

# [ToDo] Filter function is implemented in hole_detection.py, but not yet shift here!!


# each number in the list represent the number of connection to certain point
# so, if there are n points, the length of the list will be n.
n = random.randint(2, 15)

# We connect lines / vectors on each two points in order to comparaing two point sets by length of lines or angle between lines.

# Ideally, when the two point sets totally match, every point should be connected to all the other points
# which means, the list will look like below.
init_list = n * [n - 1]  # n points --> liest length = n, every point connects to all the other --> connection number = n-1

# k is used for determine how many steps are going to change the original list / connection
k = random.randint(1, 40)

# Now we start simulate that, there are other points which doesn't match on other point sets
new_list = list(init_list)

# Compute the original connectedSets, to record the initial connectedPointPairs
# Recording the connection with set class, because the order of points is not important
originalConnectedSets = [] 
for i in range(0, n):
    for j in range(i + 1, n):
        connectedSet = {i, j}
        originalConnectedSets.append(connectedSet)

# intialize the connectedSets, this is used for check whether the generated new connection has already been connected.
# if it has been connected, then k_skip+1
connectedSets = list(originalConnectedSets)
k_skip = 0
# When creating a new connection(line), it could be an independent line which doesn't connect to the original points or a dependent line whose one end connects to the existing points.
for i in range(0, k):
    # Create two new points as part of random picking points of connection, if the new point dosen't get selected, it will be removed / pop later
    new_list += [0, 0]
    n_new = len(new_list)
    # print('zero Index: (%d, %d)' % (n_new - 1, n_new - 2))

    # since we need to record all connection, and the point order is not important, we use set class.
    selectSet = set(random.sample(range(0, n_new), 2))

    # if the last one is select and the last second is not select, it is the same as last one doesn't get selected but the last second get selected
    if n_new - 2 not in selectSet and n_new - 1 in selectSet:
        # print('Before modifying slectSet:', selectSet)

        # shift the last one point to last second and pop the last one
        selectSet = list(selectSet)
        selectSet.sort()
        selectSet[-1] -= 1
        selectSet = set(selectSet)
        # print('After modifying slectSet:', selectSet)
        new_list.pop()

    # if the selected set has already been connected, after remove the new added but not connected point, reselect again
    if selectSet in connectedSets:
        # print('skip once!')
        k_skip += 1
        while new_list[-1] == 0:
            new_list.pop()
        continue

    # create the connection, the point connected should add up one to its connection number
    for selectIdx in selectSet:
        new_list[selectIdx] += 1
    while new_list[-1] == 0:
            new_list.pop()
    # record the new connected set
    connectedSets.append(selectSet)

# real steps that add connection to inital points
k -= k_skip
# new connection
newlyConnectedSets = [i for i in connectedSets if i not in originalConnectedSets]

# till here, the generation of connection list from initial list is done.
##########################################################################

# Below is testing the algorithm to find the possible most points which match to other point set
# Result: algorithm only works for 80% cases

nMax = find_n_max(new_list)
solutionList = nMax * [nMax - 1]

print('=======================================')
print('n: %d, k: %d' % (n, k))
print('- - - - - - -')
print('Initial List:  ', init_list)
print('Solution List: ', solutionList)
print('List Generated:', new_list)
# print('ConnectionSet List:', connectedSets)
print('\nNewly connected Sets:\n', newlyConnectedSets)
print('=======================================')
print('Sum of Initial List:', sum(init_list))
print('Sum of created List:', sum(new_list))

for i in range(2, 15, 1):
    isum = sum(i * [i - 1])
    k_infer = (sum(new_list) - isum) / 2
    print('%d sum: %d     possible n, k: %d, %d ' % (i, isum, i, k_infer))

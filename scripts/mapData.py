#!/usr/bin/env python2
## ---------------------------------------------------- ##
# file: mapData.py
# desc: defines the mapdata as a graph and methods for finding shortest routes
# author: gauthampdas, isrc, university of ulster, uk
# email: contact@gauthampdas.com
# license: MIT
#
# Copyright (c) 2015 Gautham P. Das
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included
# in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
# OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
# IN THE SOFTWARE.
## ---------------------------------------------------- ##

import heapq
import misc

class Node():
    def __init__(self, nodeId, x, y):
        self.x         = x
        self.y         = y
        self.id     = nodeId

class MapData():
    def __init__(self):
        self.nRooms     = 0     # number of rooms
        self.nNodes     = 0     # number of nodes
        self.roomBounds = {}    # dict(rId:[rxMin, rxMax, ryMin, ryMax])
        self.xMin         = 0.0    # workspace minX
        self.xMax         = 0.0    # workspace maxX
        self.yMin         = 0.0    # workspace minY
        self.yMax         = 0.0     # workspace maxY
        self.edges         = {}     # dict(nId:[connected nIDs])
        self.nodes         = {}     # dict(nId:node(nId,nX,nY))
        self.roomIds     = []     # list of roon Ids
        self.nodeIds     = []     # list of node Ids
        self.maxDist     = 0.0     # max distance in the workspace (diagonal)
        self.roomNodes     = {}     # dict(rId:[nIds])

    def setWSBounds(self, xMin, xMax, yMin, yMax):
        self.xMin         = xMin
        self.xMax         = xMax
        self.yMin         = yMin
        self.yMax         = yMax
        self.maxDist     = 4.0 * misc.getDist(self.xMin, self.yMin, self.xMax, self.yMax)

    def setRoomBounds(self, roomId, xMin, xMax, yMin, yMax):
        self.roomBounds[roomId]     = [xMin, xMax, yMin, yMax]
        if (self.roomIds.count(roomId) == 0):
            self.roomIds.append(roomId)
            self.nRooms         += 1

    def addNode(self, nodeId, x, y):
        self.nodes[nodeId]     = Node(nodeId, x, y)
        if (self.nodeIds.count(nodeId) == 0):
            self.nodeIds.append(nodeId)
            self.nNodes         += 1

    def addEdge(self, nodeId, connectedNodes=[]):
        self.edges[nodeId]     = []
        for nIdx in (connectedNodes):
            self.edges[nodeId].append(nIdx)

    def sortNodes(self):
        closedNodes = []
        openNodes     = []+self.nodeIds
        for rId in (self.roomIds):
            self.roomNodes[rId] = []
            rxMin     = self.roomBounds[rId][0]
            rxMax     = self.roomBounds[rId][1]
            ryMin     = self.roomBounds[rId][2]
            ryMax     = self.roomBounds[rId][3]
            for nId in (openNodes):
                if nId not in closedNodes:
                    x = self.nodes[nId].x
                    y = self.nodes[nId].y
                    if ((rxMin <= x <= rxMax) and (ryMin <= y <= ryMax)):
                        self.roomNodes[rId].append(nId)
                        closedNodes.append(nId)

    def getRoom(self, x, y):
        for rId in (self.roomIds):
            rxMin     = self.roomBounds[rId][0]
            rxMax     = self.roomBounds[rId][1]
            ryMin     = self.roomBounds[rId][2]
            ryMax     = self.roomBounds[rId][3]
            if ((rxMin <= x <= rxMax) and (ryMin <= y <= ryMax)):
                return rId
        return None

    def getClosestNode(self, x, y):
        # returns the roomid and the closest node id
        roomId         = self.getRoom(x, y)
        nodeId         = -1
        if (roomId != None):
            roomNodes     = self.roomNodes[roomId]
            minDist = float("inf")
            for item in (roomNodes):
                nX         = self.nodes[item].x
                nY         = self.nodes[item].y
                dist     = misc.getDist(x, y, nX, nY)
                if (minDist == float("inf")) or (misc.lt(dist, minDist)):
                    minDist     = dist
                    nodeId         = item
        elif (roomId == None):
            # assuming the point cannot be outside the workspace
            # eg: the point can be on a border
            # find the closest node among all nodes
            roomNodes     = self.nodeIds
            minDist = float("inf")
            for item in (roomNodes):
                nX         = self.nodes[item].x
                nY         = self.nodes[item].y
                dist     = misc.getDist(x, y, nX, nY)
                if (minDist == float("inf")) or (misc.lt(dist, minDist)):
                    minDist     = dist
                    nodeId         = item
        return (roomId, nodeId)

    def aStarHeap(self, start, goal):     # instead of the whole node object only the nodeIds are passed
        closedSet     = set()             # set of nodes already evaluated
        openSet     = set()                # set of tentative nodes to be evaluated, starting with the start node
        cameFrom     = {}                 # path of navigated nodes

        openHeap     = []
        gScore         = {}
        fScore         = {}

        openSet.add(start)
        openHeap.append((0.0, start))
        gScore[start]     = 0.0         # cost from start along the best known path
        fScore[start]     = gScore[start] + self.__heuristicCostEstimate(start, goal)

        while openSet:
            current = heapq.heappop(openHeap)[1]
            if (current == goal):
                return self.__reconstructPath(cameFrom, goal)

            openSet.remove(current)
            closedSet.add(current)
            nextNodes     = self.edges[current]
            for item in (nextNodes):
                if item not in closedSet:
                    gScore[item]     = gScore[current] + self.__getDist(current, item)
                    fScore[item]     = gScore[item] + self.__heuristicCostEstimate(item, goal)
                    heapq.heappush(openHeap, (fScore[item],item))
                    if item not in openSet:
                        openSet.add(item)
                    cameFrom[item]     = current
        return None

    def aStar(self, start, goal):
        closedSet     = set()         # set of nodes already evaluated
        openSet     = set()            # set of tentative nodes to be evaluated, starting with the start node
        cameFrom     = {}             # path of navigated nodes
        gScore         = {}
        fScore         = {}

        openSet.add(start)
        gScore[start]     = 0.0         # cost from start along the best known path
        fScore[start]     = gScore[start] + self.__heuristicCostEstimate(start, goal)

        while openSet:
            current     = min(openSet, key=lambda item: fScore[item])
            if (current == goal):
                return self.__reconstructPath(cameFrom, goal)

            openSet.remove(current)
            closedSet.add(current)
            nextNodes     = self.edges[current]
            for item in (nextNodes):
                if item not in closedSet:
                    gScore[item]     = gScore[current] + self.__getDist(current, item)
                    fScore[item]     = gScore[item] + self.__heuristicCostEstimate(item, goal)
                    if item not in openSet:
                        openSet.add(item)
                    cameFrom[item]     = current
        return None

    def __heuristicCostEstimate(self, start, goal):
        return self.__getDist(start, goal)

    def __getDist(self, fromNode, toNode):
        return misc.getDist(self.nodes[fromNode].x, self.nodes[fromNode].y, self.nodes[toNode].x, self.nodes[toNode].y)

    def __reconstructPath(self, cameFrom, current):
        path = []
        if (cameFrom.has_key(current)):
            items = self.__reconstructPath(cameFrom, cameFrom[current])
            for item in (items):
                path.append(item)
            path.append(current)
            return path
        else:
            return [current]

if __name__ == "__main__":
#    myMap     = mapData()
#    myMap.addNode(0, -6, 10)
#    myMap.addNode(1, -1, 10)
#    myMap.addNode(2, 9, 10)
#    myMap.addNode(3, -1, 2)
#    myMap.addNode(4, 9, 2)
#    myMap.addNode(5, -1, -4)
#    myMap.addNode(6, 9, -4)
#    myMap.addNode(7, -6, -5)
#    myMap.addNode(8, 9, -5)
#
#
#    myMap.addEdge(0, [1])
#    myMap.addEdge(1, [3])
#    myMap.addEdge(2, [4])
#    myMap.addEdge(3, [1, 4, 5, 7])
#    myMap.addEdge(4, [2, 3])
#    myMap.addEdge(5, [3, 6, 7])
#    myMap.addEdge(6, [5, 8])
#    myMap.addEdge(7, [3, 5])
#    myMap.addEdge(8, [6])
#
#    start     = 0
#    goal     = 2
#
#    path = myMap.aStar(start, goal)
#    if (path != None):
#        print (path)
#    else:
#        print ("None")
#
#    path = myMap.aStarHeap(start, goal)
#    if (path != None):
#        print (path)
#    else:
#        print ("None")
    import readInData
    import random

    (rList, tList, rConnect, mapInfo) = readInData.readInData("../data/labhouse_")
    mapInfo.sortNodes()

    xy         = [(random.randrange (mapInfo.roomBounds[i][0], mapInfo.roomBounds[i][1]),random.randrange (mapInfo.roomBounds[i][0], mapInfo.roomBounds[i][1])) for i in (range (mapInfo.nRooms))]
    xyRoom     = []
    xyNode     = []
    for (x,y) in (xy):
        (roomId, nodeId)     = mapInfo.getClosestNode(x,y)
        xyRoom.append(roomId)
        xyNode.append(xyNode)

    for j in (range (len(tList))): # for each task
        tX = tList[j].xOrg
        tY = tList[j].yOrg
        print (tX, tY)
        goalRoomId         = mapInfo.getRoom(tX, tY)
        goalNode         = -1
        if (goalRoomId != None):
            nodes     = mapInfo.roomNodes[goalRoomId]
            minDist = float("inf")
            for item in (nodes):
                nX         = mapInfo.nodes[item].x
                nY         = mapInfo.nodes[item].y
                dist     = misc.getDist(tX, tY, nX, nY)
                if (minDist == float("inf")) or (misc.lt(dist, minDist)):
                    minDist     = dist
                    goalNode     = item
        else:
            print ("<<<error: no room for goal>>>")
            exit()

        print (j, tX, tY, goalRoomId, goalNode)















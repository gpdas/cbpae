#!/usr/bin/env python2
## ---------------------------------------------------- ##
# file: readInData.py
# desc: reads from configuration data file and creates the models of robots, tasks and the environment mapdata
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

import robot
import task
import mapData

def readInData(filePrefix=""):
    ## Initialization phase:
    mapInfo = mapData.MapData() # map information

    """read from file & initialize"""
    dataFile = open(filePrefix+"inData.dat", "r")
    dataFile.readline() # map info:

    dataFile.readline() # no. of rooms, no. of nodes
    (nRoomsStr, nNodesStr) = (dataFile.readline().strip()).split(',')
    nRooms = int(nRoomsStr)
    nNodes = int(nNodesStr)

    dataFile.readline() # workspace boundary: xMin, xmax, yMin, yMax
    (xMin, xMax, yMin, yMax) = (dataFile.readline().strip()).split(',')
    mapInfo.setWSBounds(float(xMin), float(xMax), float(yMin), float(yMax))

    dataFile.readline() # room boundary: roomID, xMin, xmax, yMin, yMax
    for i in (range (nRooms)):
        (roomIdStr, rxMin, rxMax, ryMin, ryMax) = (dataFile.readline().strip()).split(',')
        mapInfo.setRoomBounds(int(roomIdStr), float(rxMin), float(rxMax), float(ryMin), float(ryMax))

    dataFile.readline() # nodes: nodeID, x, y
    for i in (range (nNodes)):
        (nodeIdStr, nX, nY) = (dataFile.readline().strip()).split(',')
        mapInfo.addNode(int(nodeIdStr), float(nX), float(nY))

    dataFile.readline() # edges: nodeID, IDs of connected nodes
    for i in (range (nNodes)):
        nodesStr = (dataFile.readline().strip()).split(',')
        nodes = []
        for j in (range (len(nodesStr)-1)):
            nodes.append(int(nodesStr[j+1]))
        mapInfo.addEdge(int(nodesStr[0]), nodes)
    # sort the nodes as per the rooms
    mapInfo.sortNodes()

    dataFile.readline() # robot info
    dataFile.readline() # no. of robots
    nRStr = dataFile.readline().strip()
    nR = int(nRStr)

    dataFile.readline() # sim info
    simStr = dataFile.readline().strip()
    if (simStr.lower() == "true"):
        sim = True
    else:
        sim = False
    dataFile.readline() # do task
    doTaskStr = dataFile.readline().strip()
    if (doTaskStr.lower() == "true"):
        doTask = True
    else:
        doTask = False

    robotList = {} # robot list
    robotIds = []
    ## robot list
    dataFile.readline()
    # robot: r-id, x-org, y-org, a-org, taskLimit, onTime, offTime, host-ip/name, port,
    # nSkills, skill-mobile, , expertise-mobile, skill-video, expertise-video, skill-audio, expertise-audio, skill-gripper, expertise-gripper, skill-arm, expertise-arm, skill-cleaner, expertise-cleaner
    for i in (range (0, nR)):
        rStr = (dataFile.readline().strip()).split(',')
        rLen = len(rStr)
        if (rLen < 8):
            print ("Error")
        else:
            nSkills = int(rStr[9])
            # setting skills and expertise
            skills = []
            expertise = []
            for idx in (range (10, (10+2*nSkills), 2)):
                skills.append(rStr[idx].strip())
                expertise.append(float(rStr[idx+1]))

            # only considering "test" and ignoring "player", "playerstage" and "ros" robot types here
            rType = "test"

            rId = int(rStr[0].strip(), 10)
            robotList[rId] = robot.Robot(rId, float(rStr[1]), float(rStr[2]), float(rStr[3]), int(rStr[4]), float(rStr[5]), float(rStr[6]), rStr[7].strip(), int(rStr[8]), doTask)

            # setting expertise
            robotList[rId].setExpertise(skills, expertise)
            robotIds.append(rId)

            # setting the mapdata
            robotList[rId].setMapData(mapInfo)

    ## task list
    dataFile.readline() # task info:
    dataFile.readline() # no of tasks
    nTStr = dataFile.readline().strip()
    nT = int(nTStr)
    taskList = {} # task list
    taskIds = []
    dataFile.readline() # tasks: x-org, y-org, x-finish, y-finish, onTime, offTime, priority, tasktype
    for j in (range (0, nT)):
        tStr = (dataFile.readline().strip()).split(',')
        tLen = len(tStr)
        # index, xOrg, yOrg, xFinish, yFinish, tOn, tOff, statPrio, tType
        if (tLen < 8):
            print ("Error: not enough task data")
        else:
            tId = int(tStr[0].strip(), 10)
            taskList[tId]  = task.Task(tId, float(tStr[1]), float(tStr[2]), float(tStr[3]), float(tStr[4]), float(tStr[5]), float(tStr[6]), int(tStr[7]), tStr[8].strip())
            taskIds.append(tId)
            # for comparison with CBBA or any algorithm without priority, ignore priority of tasks
#            taskList[tId].setStaticPriority(0)

    ## robot - task setting
    for rId in (robotIds):
        robotList[rId].setTaskList(taskIds, taskList) # sets the task list to each robot. to make it easier to calculate value by a method in the class

    return (robotIds, robotList, taskIds, taskList, mapInfo)

if __name__ == "__main__":

    readInData()

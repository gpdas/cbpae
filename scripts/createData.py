#!/usr/bin/env python2
## ---------------------------------------------------- ##
# file: createData.py
# desc: scritp to generate input data files describing environment, robots and tasks.
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

import random
import os

def rescale(A, B, a, b, x):
    # rescale x from range [A,B] to [a,b]
    if ((b-a) != 0):
        return ((x-A)*float((b-a))/(B-A))+a
    else:
        return float("+inf")

def getXY(roomXY):
    # 1.2 is the safe distance from the walls in meters
    # roomXY: roomId, X, Y
    return (random.uniform(roomXY[1]+1.2, roomXY[2]-1.2), random.uniform(roomXY[3]+1.2, roomXY[4]-1.2))

def createData():

    rType  = "test"

    dataDir  = os.path.abspath(os.path.join(os.getcwd(), os.pardir)) + "/data/"
    try:
        os.mkdir(dataDir)
    except:
        pass

    maps = ["openRooms", "cleanCave"]
    [mapItem, wsBoundary, mapBoundary, rooms, nodes, edges] = getMapDetails("openRooms")
    [mapClean, wsBoundaryClean, mapBoundaryClean, roomsClean, nodesClean, edgesClean] = getMapDetails("cleanCave")

    [xMin, xMax, yMin, yMax] = wsBoundary
    [mapLength, mapWidth] = mapBoundary

    nRooms = len(rooms)
    nNodes = len(nodes)

    nRoomsClean = len(roomsClean)
    nNodesClean = len(nodesClean)

    NR = [5, 10, 15]
    NT = [5, 10, 15, 20, 25, 30]

    # convert all values in rooms, nodes, edges from mapBoundary dimension to wsBoundary
    for i in range(nRooms):
        rooms[i][1] = rescale(0.0, mapLength, xMin, xMax, rooms[i][1])
        rooms[i][2] = rescale(0.0, mapLength, xMin, xMax, rooms[i][2])
        rooms[i][3] = rescale(0.0, mapWidth, yMin, yMax, rooms[i][3])
        rooms[i][4] = rescale(0.0, mapWidth, yMin, yMax, rooms[i][4])

    for i in range(nNodes):
        nodes[i][1] = rescale(0.0, mapLength, xMin, xMax, nodes[i][1])
        nodes[i][2] = rescale(0.0, mapWidth, yMin, yMax, nodes[i][2])

    for i in range(len(roomsClean)):
        roomsClean[i][1] = rescale(0.0, mapLength, xMin, xMax, roomsClean[i][1])
        roomsClean[i][2] = rescale(0.0, mapLength, xMin, xMax, roomsClean[i][2])
        roomsClean[i][3] = rescale(0.0, mapWidth, yMin, yMax, roomsClean[i][3])
        roomsClean[i][4] = rescale(0.0, mapWidth, yMin, yMax, roomsClean[i][4])

    for i in range(len(nodesClean)):
        nodesClean[i][1] = rescale(0.0, mapLength, xMin, xMax, nodesClean[i][1])
        nodesClean[i][2] = rescale(0.0, mapWidth, yMin, yMax, nodesClean[i][2])

    types = ["delivery", "door", "medicine", "clean", "surveillance"]
    prios = {"delivery":2, "door":2, "medicine":2, "clean":3, "surveillance":1}

    tList = []
    # creating all tasks
    for j in range(max(NT)):
        roomId = random.choice([k for k in range(nRooms)])
        [tX1, tY1] = getXY(rooms[roomId])
        tOn = 0.0
        tOff = float("inf")
        tType = random.choice(types)
        tPrio = prios[tType]
        if (tType == "delivery"):
            roomId = random.choice([k for k in range(nRooms)])
            [tX2, tY2] = getXY(rooms[roomId])
        else:
            [tX2, tY2] = [tX1, tY1]

        # for each task type there are some specific skills required
        # to make sure there is atleast one robot capable of doing the task,
        # set the skill and expertise for two randomly selected robots
        # and ignore, if they already have the skills

        tList.append([tX1, tY1, tX2, tY2, tOn, tOff, tPrio, tType])

    # creating the datafiles
    for nr in NR:
        skills = ["vision", "audition", "gripper", "manipulator", "cleaner"]

        rList = []
        rListHomo = []
        #creating all robots
        for i in range (nr):
            roomId = random.choice([k for k in range(nRooms)])
            [rX, rY] = getXY(rooms[roomId])
            rA = random.uniform(-180.0, 180.0)
            tLim = 100

            rOn = 0.0
            rOff = float("inf")
            rIp = "localhost"
            rPort = 6666+i

            nSkills = random.randint(0,5) + 1
            rSkills = ["navigation"]
            rExpert = [random.uniform(0.5, 0.9)]
            remSkills = []+skills
            for idx in range(nSkills-1):
                rSkills.append(random.choice(remSkills))
                remSkills.remove(rSkills[-1])
                rExpert.append(random.uniform(0.5, 0.9))

            nSkillsHomo = 6
            rSkillsHomo = ["navigation", "vision", "audition", "gripper", "cleaner", "manipulator"]
            rExpertHomo = []
            for idx in range(nSkillsHomo):
                rExpertHomo.append(0.8)

            rList.append([rX, rY, rA, tLim, rOn, rOff, rIp, rPort, nSkills, rSkills, rExpert])
            rListHomo.append([rX, rY, rA, tLim, rOn, rOff, rIp, rPort, nSkillsHomo, rSkillsHomo, rExpertHomo])

        skillsReq = {"fall":["vision", "audition"], "delivery":["vision", "gripper"], "door":["manipulator", "vision", "audition"], "medicine":["audition"], "clean":["cleaner"], "surveillance":["vision", "audition"]}
        for nt in NT:
            for j in range(nt):
                skills = skillsReq[tList[j][7]]
                rSel = random.randint(0, nr-1)
                for skill in skills:
                    if (rList[rSel][9].count(skill) == 0):
                        rList[rSel][8] += 1
                        rList[rSel][9].append(skill)
                        rList[rSel][10].append(random.uniform(0.5, 0.9))

            dataFile1 = open(dataDir + str(nr)+"_"+str(nt)+"_rooms_hetero_diffprio_equalstart_"+"inData.dat", "w")# few emergency prio, from start
            dataFile2 = open(dataDir + str(nr)+"_"+str(nt)+"_rooms_hetero_diffprio_randmstart_"+"inData.dat", "w")# few emergency prio, random start

            dataFile3 = open(dataDir + str(nr)+"_"+str(nt)+"_clean_hetero_diffprio_equalstart_"+"inData.dat", "w")# few emergency prio, from start
            dataFile4 = open(dataDir + str(nr)+"_"+str(nt)+"_clean_hetero_diffprio_randmstart_"+"inData.dat", "w")# few emergency prio, random start

            dataFile5 = open(dataDir + str(nr)+"_"+str(nt)+"_rooms_homo_diffprio_equalstart_"+"inData.dat", "w")# few emergency prio, from start
            dataFile6 = open(dataDir + str(nr)+"_"+str(nt)+"_rooms_homo_diffprio_randmstart_"+"inData.dat", "w")# few emergency prio, random start

            dataFile7 = open(dataDir + str(nr)+"_"+str(nt)+"_clean_homo_diffprio_equalstart_"+"inData.dat", "w")# few emergency prio, from start
            dataFile8 = open(dataDir + str(nr)+"_"+str(nt)+"_clean_homo_diffprio_randmstart_"+"inData.dat", "w")# few emergency prio, random start


            fileOpen = [dataFile1, dataFile2, dataFile5, dataFile6]
            fileClean = [dataFile3, dataFile4, dataFile7, dataFile8]

            for fileItem in (fileOpen+fileClean):
                print >>fileItem, "## map info:\n# no. of rooms, no. of nodes"

            for fileItem in (fileOpen):
                print >>fileItem, "%d, %d" %(nRooms, nNodes)
            for fileItem in (fileClean):
                print >>fileItem, "%d, %d" %(nRoomsClean, nNodesClean)

            for fileItem in (fileOpen):
                print >>fileItem, "# workspace boundary: xMin, xmax, yMin, yMax"
            for fileItem in (fileClean):
                print >>fileItem, "# workspace boundary: xMin, xmax, yMin, yMax"

            for fileItem in (fileOpen+fileClean):
                print >>fileItem, "%0.3f, %0.3f, %0.3f, %0.3f" %(xMin, xMax, yMin, yMax)

            for fileItem in (fileOpen+fileClean):
                print >>fileItem, "# room boundary: roomID, xMin, xmax, yMin, yMax"

            for fileItem in (fileOpen):
                for idx in range(nRooms):
                    print >>fileItem, "%d, %0.3f, %0.3f, %0.3f, %0.3f" %(rooms[idx][0], rooms[idx][1], rooms[idx][2], rooms[idx][3], rooms[idx][4])
            for fileItem in (fileClean):
                for idx in range(nRoomsClean):
                    print >>fileItem, "%d, %0.3f, %0.3f, %0.3f, %0.3f" %(roomsClean[idx][0], roomsClean[idx][1], roomsClean[idx][2], roomsClean[idx][3], roomsClean[idx][4])

            for fileItem in (fileOpen):
                print >>fileItem, "# nodes: nodeID, x, y"
            for fileItem in (fileClean):
                print >>fileItem, "# nodes: nodeID, x, y"

            for fileItem in (fileOpen):
                for idx in range(nNodes):
                    print >>fileItem, "%d, %0.3f, %0.3f" %(nodes[idx][0], nodes[idx][1], nodes[idx][2])
            for fileItem in (fileClean):
                for idx in range(nNodesClean):
                    print >>fileItem, "%d, %0.3f, %0.3f" %(nodesClean[idx][0], nodesClean[idx][1], nodesClean[idx][2])

            for fileItem in (fileOpen+fileClean):
                print >>fileItem, "# edges: nodeID, IDs of connected nodes"

            for fileItem in (fileOpen):
                if (nNodes > 0):
                    for idx1 in range(nNodes):
                        for idx2 in range(len(edges[idx1])-1):
                            print >>fileItem, "%d," %(edges[idx1][idx2])
                        print >>fileItem, "%d" %(edges[idx1][len(edges[idx1])-1])
            for fileItem in (fileClean):
                if (nNodesClean > 0):
                    for idx1 in range(nNodesClean):
                        for idx2 in range(len(edgesClean[idx1])-1):
                            print >>fileItem, "%d," %(edgesClean[idx1][idx2])
                        print >>fileItem, "%d" %(edgesClean[idx1][len(edgesClean[idx1])-1])

            for fileItem in (fileOpen+fileClean):
                print >>fileItem, "## robot info\n# no. of robots"
                print >>fileItem, "%d" %(nr)
                print >>fileItem, "# sim info - for stage simulation gripper proxy is not available in stage2"
                print >>fileItem, "true"
                print >>fileItem, "# do task"
                print >>fileItem, "true"
                print >>fileItem, "# robot: rId, xOrg, yOrg, aOrg, taskLimit, onTime, offTime, hostIP, port, nSkills, sNavigation, eNavigation, sVision, eVision, sAudition, eAudition, sGripper, eGripper, sCleaner, eCleaner, sManipulator, eManipulator, rtype"

            for fileItem in (fileOpen[:2]+fileClean[:2]):
                for i in range(0, nr):
                    print >>fileItem, "%d, %0.3f, %0.3f, %0.3f, %d, %0.3f, %0.3f, %s, %d, %d," %(i, rList[i][0], rList[i][1], rList[i][2], rList[i][3], rList[i][4], rList[i][5], rList[i][6], rList[i][7], rList[i][8]),
                    nSkill = rList[i][8]
                    for idx in range(nSkill-1):
                        print >>fileItem, "%s, %0.3f, " %(rList[i][9][idx], rList[i][10][idx]),
                    print >>fileItem, "%s, %0.3f, " %(rList[i][9][nSkill-1], rList[i][10][nSkill-1]),
                    print >>fileItem, "%s" %(rType)

            for fileItem in (fileOpen[2:]+fileClean[2:]):
                for i in range(0, nr):
                    print >>fileItem, "%d, %0.3f, %0.3f, %0.3f, %d, %0.3f, %0.3f, %s, %d, %d," %(i, rListHomo[i][0], rListHomo[i][1], rListHomo[i][2], rListHomo[i][3], rListHomo[i][4], rListHomo[i][5], rListHomo[i][6], rListHomo[i][7], rListHomo[i][8]),
                    nSkillHomo = rListHomo[i][8]
                    for idx in range(nSkillHomo-1):
                        print >>fileItem, "%s, %0.3f, " %(rListHomo[i][9][idx], rListHomo[i][10][idx]),
                    print >>fileItem, "%s, %0.3f, " %(rListHomo[i][9][nSkillHomo-1], rListHomo[i][10][nSkillHomo-1]),
                    print >>fileItem, "%s" %(rType)

            for fileItem in (fileOpen+fileClean):
                print >>fileItem, "## task info:\n# no of tasks"
                print >>fileItem, "%d" %(nt)
                print >>fileItem, "# tasks: tId, xOrg, yOrg, xFinish, yFinish, onTime, offTime, priority, tType"

            for j in range(0, nt):
                if (j < nt-3):
                    for fileItem in (fileOpen+fileClean):
                        print >>fileItem, "%d, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %d, %s" %(j, tList[j][0], tList[j][1], tList[j][2], tList[j][3], tList[j][4], tList[j][5], tList[j][6], tList[j][7])
                else:
                    print >>dataFile1, "%d, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %d, %s" %(j, tList[j][0], tList[j][1], tList[j][0], tList[j][1], tList[j][4], tList[j][5], 0, "fall")
                    print >>dataFile3, "%d, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %d, %s" %(j, tList[j][0], tList[j][1], tList[j][0], tList[j][1], tList[j][4], tList[j][5], 0, "fall")
                    print >>dataFile5, "%d, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %d, %s" %(j, tList[j][0], tList[j][1], tList[j][0], tList[j][1], tList[j][4], tList[j][5], 0, "fall")
                    print >>dataFile7, "%d, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %d, %s" %(j, tList[j][0], tList[j][1], tList[j][0], tList[j][1], tList[j][4], tList[j][5], 0, "fall")

                    print >>dataFile2, "%d, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %d, %s" %(j, tList[j][0], tList[j][1], tList[j][0], tList[j][1], 200.0, tList[j][5], 0, "fall")
                    print >>dataFile4, "%d, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %d, %s" %(j, tList[j][0], tList[j][1], tList[j][0], tList[j][1], 200.0, tList[j][5], 0, "fall")
                    print >>dataFile6, "%d, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %d, %s" %(j, tList[j][0], tList[j][1], tList[j][0], tList[j][1], 200.0, tList[j][5], 0, "fall")
                    print >>dataFile8, "%d, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %0.3f, %d, %s" %(j, tList[j][0], tList[j][1], tList[j][0], tList[j][1], 200.0, tList[j][5], 0, "fall")

            for fileItem in (fileOpen+fileClean):
                fileItem.close()

def getMapDetails(mapItem):
#    mapItem     = "openRooms"
#    mapItem     = "cleanCave"

    wsBoundary = [-16.0, 16.0, -12.0, 12.0]
    mapBoundary = [400, 300]

    if (mapItem == "openRooms"):
        nRooms = 7
        nNodes = 24

        rooms = []
        rooms.append([0, 10.0,  130.0,  10.0,  120.0])
        rooms.append([1, 135.0,  255.0,  10.0,  120.0])
        rooms.append([2, 260.0,  390.0,  10.0 ,  120.0])
        rooms.append([3, 10.0,  390.0,  125.0 ,  175.0])
        rooms.append([4, 10.0,  130.0,  180.0,  290.0])
        rooms.append([5, 135.0,  255.0,  180.0,  290.0])
        rooms.append([6, 260.0,  390.0,  180.0 ,  290.0])

        nodes = []
        nodes.append([0, 85, 90])
        nodes.append([1, 115, 90])
        nodes.append([2, 150, 90])
        nodes.append([3, 180, 90])
        nodes.append([4, 275, 90])
        nodes.append([5, 305, 90])
        nodes.append([6, 85, 140])
        nodes.append([7, 115, 140])
        nodes.append([8, 150, 140])
        nodes.append([9, 180, 140])
        nodes.append([10, 275, 140])
        nodes.append([11, 305, 140])
        nodes.append([12, 85, 160])
        nodes.append([13, 115, 160])
        nodes.append([14, 150, 160])
        nodes.append([15, 180, 160])
        nodes.append([16, 275, 160])
        nodes.append([17, 305, 160])
        nodes.append([18, 85, 210])
        nodes.append([19, 115, 210])
        nodes.append([20, 150, 210])
        nodes.append([21, 180, 210])
        nodes.append([22, 275, 210])
        nodes.append([23, 305, 210])

        edges = []
        edges.append([0, 1])
        edges.append([1, 7])
        edges.append([2, 3])
        edges.append([3, 9])
        edges.append([4, 5])
        edges.append([5, 11])
        edges.append([6, 0, 7])
        edges.append([7, 8, 10, 13])
        edges.append([8, 2, 9])
        edges.append([9, 10, 15])
        edges.append([10, 4, 11])
        edges.append([11, 17])
        edges.append([12, 6, 8, 10])
        edges.append([13, 12, 19])
        edges.append([14, 8, 13])
        edges.append([15, 14, 21])
        edges.append([16, 10, 13, 15])
        edges.append([17, 16, 23])
        edges.append([18, 12])
        edges.append([19, 18])
        edges.append([20, 14])
        edges.append([21, 20])
        edges.append([22, 16])
        edges.append([23, 22])

    elif (mapItem == "cleanCave"):
        nRooms = 1
        nNodes = 0

        rooms = []
        rooms.append([0, 10.0,  390.0,  10.0,  290.0])

        nodes = []
        edges = []

    return (mapItem, wsBoundary, mapBoundary, rooms, nodes, edges)

if __name__ == "__main__":


    createData()


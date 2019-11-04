#!/usr/bin/env python2
## ---------------------------------------------------- ##
# file: plotLogs.py
# desc: scritp to generate plots from logs.
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

import pylab
import misc
import os
from config import *
import string

class Robot():
    def __init__(self, xOrg, yOrg):
        self.xOrg     = 0.0 + xOrg
        self.yOrg     = 0.0 + yOrg
        pass

class Task():
    def __init__(self, xOrg, yOrg, xFinish, yFinish):
        self.xOrg     = 0.0 + xOrg
        self.yOrg     = 0.0 + yOrg
        self.xFinish     = 0.0 + xFinish
        self.yFinish     = 0.0 + yFinish
        pass

def readData(filePrefix, dataDir):
    robotList     = []
    taskList     = []
    print ("reading data")
    fName         = open(dataDir + filePrefix + "inData.dat", "r")
    for i in (range (2)):
        fName.readline()
    fData     = fName.readline().strip().split(",")
    nRooms     = 0 + int(fData[0])
    nNodes     = 0 + int(fData[1])
    for i in (range (1)):
        fName.readline()
    fData     = fName.readline().strip().split(",")
    xMin         = 0.0 + float(fData[0])
    xMax         = 0.0 + float(fData[1])
    yMin         = 0.0 + float(fData[2])
    yMax         = 0.0 + float(fData[3])
    for i in (range (nRooms + 1)):
        fName.readline()
    for i in (range (nNodes + 1)):
        fName.readline()
    for i in (range (nNodes + 1)):
        fName.readline()
    for i in (range (2)):
        fName.readline()
    nRobot     = 0 + int(fName.readline().strip())
    for i in (range (5)):
        fName.readline()
    for i in (range (nRobot)):
        fData     = fName.readline().strip().split(",")
        robotList.append(Robot(float(fData[0]), float(fData[1])))
    for i in (range (nRobot + 1)):
        fName.readline()
    for i in (range (2)):
        fName.readline()
    nTask     = 0 + int(fName.readline().strip())
    for i in (range (1)):
        fName.readline()
    for i in (range (nTask)):
        fData     = fName.readline().strip().split(",")
        taskList.append(Task(float(fData[0]), float(fData[1]), float(fData[2]), float(fData[3])))
    fName.close()

    return(xMin, xMax, yMin, yMax, nRobot, nTask, robotList, taskList)

def plotLogs(filePrefix, dataDir, logsDir, plotsDir):
        """"read all logs. merge them to a single one. plot all data in single plot"""
        (xMin, xMax, yMin, yMax, nRobot, nTask, robotList, taskList)     = readData(filePrefix, dataDir)

        print (">>> reading logs")
        rId         = []
        timeInit    = []
        startTime    = []
        stopTime    = []
        execTime    = []
        nTaskAlloc    = []
        rTasks        = []
        nRAct         = []
        bidTime     = []
        bidVal         = []
        tNavStart    = []
        tNavStop    = []
        tExecStart    = []
        tExecStop    = []
        eTasks        = []
        eTaskOn        = []
        eTaskStart    = []
        eTaskStop    = []
        rDist        = []
        x            = []
        y            = []

        for i in (range (nRobot)):
            fName         = open(logsDir + filePrefix + str(i) + "_" +"cbpae.log", "r")
            rId.append(int((fName.readline().strip().split(","))[1],10))

            timeInit.append(float((fName.readline().strip().split(","))[1]))
            startTime.append(float((fName.readline().strip().split(","))[1]))
            stopTime.append(float((fName.readline().strip().split(","))[1]))
            execTime.append(float((fName.readline().strip().split(","))[1]))

            nTaskAlloc.append(int((fName.readline().strip().split(","))[1],10))

            rTasks.append([])
            bidTime.append([])
            bidVal.append([])
            tNavStart.append([])
            tNavStop.append([])
            tExecStart.append([])
            tExecStop.append([])
            x.append([])
            y.append([])

            if (nTaskAlloc[i] > 0):
                nRAct.append(1)
                tasks    = fName.readline().strip().split(",")
                for j in (range (1, nTaskAlloc[i]+1)):
                    rTasks[-1].append(int(tasks[j], 10))

                bTime    = fName.readline().strip().split(",")
                for j in (range (1, nTaskAlloc[i]+1)):
                    bidTime[-1].append(float(bTime[j]))

                bVal    = fName.readline().strip().split(",")
                for j in (range (1, nTaskAlloc[i]+1)):
                    bidVal[-1].append(float(bVal[j]))

                start    = fName.readline().strip().split(",")
                for j in (range (1, nTaskAlloc[i]+1)):
                    tNavStart[-1].append(float(start[j]))

                stop    = fName.readline().strip().split(",")
                for j in (range (1, nTaskAlloc[i]+1)):
                    tNavStop[-1].append(float(stop[j]))

                start    = fName.readline().strip().split(",")
                for j in (range (1, nTaskAlloc[i]+1)):
                    tExecStart[-1].append(float(start[j]))

                stop    = fName.readline().strip().split(",")
                for j in (range (1, nTaskAlloc[i]+1)):
                    tExecStop[-1].append(float(stop[j]))

                numETasks    = int(fName.readline().strip().split(",")[1],10)
                if (numETasks > 0):
                    eTaskList    = fName.readline().strip().split(",")
                    for j in (range (1, numETasks+1)):
                        eTasks.append(int(eTaskList[j],10))

                    eTaskInitList    = fName.readline().strip().split(",")
                    for j in (range (1, numETasks+1)):
                        eTaskOn.append(float(eTaskInitList[j]))

                    eTaskStartList    = fName.readline().strip().split(",")
                    for j in (range (1, numETasks+1)):
                        eTaskStart.append(float(eTaskStartList[j]))

                    eTaskStopList    = fName.readline().strip().split(",")
                    for j in (range (1, numETasks+1)):
                        eTaskStop.append(float(eTaskStopList[j]))
                else:
                    fName.readline()
                    fName.readline()
                    fName.readline()
                    fName.readline()

                rDist.append(float(fName.readline().strip().split(",")[1]))
                while (True):
                    try:
                        xy        = fName.readline().strip().split(",")
                    except:
                        break
                    else:
                        if (len(xy) == 2):
                            x[-1].append(float(xy[0]))
                            y[-1].append(float(xy[1]))
                        else:
                            break
            else:
                nRAct.append(0)
                rDist.append(0)
            fName.close()

        eTaskResponseTime    = []
        eTaskExecutionTime    = []
        for idx in (range (len(eTasks))):
            eTaskResponseTime.append(eTaskStart[idx] - eTaskOn[idx])
            eTaskExecutionTime.append(eTaskStop[idx] - eTaskStart[idx])

        totalDist = sum(rDist)
        avgDist = totalDist/nRobot
        if (sum(nRAct) != 0):
            avgDistAct = totalDist/sum(nRAct)
        else:
            avgDistAct = 0

        print ("plotting logs")
        maxXY     = max([xMax-xMin, yMax-yMin])
        figLen     = misc.rescale(0, maxXY, 0, 8, xMax-xMin)
        figWid     = misc.rescale(0, maxXY, 0, 8, yMax-yMin)
        ## plotting the paths
        fig = pylab.figure(figsize=(figLen, (nRobot+1)*figWid))
        # plotting inidividual robot paths
        for n in (range (nRobot)):
            spfig = fig.add_subplot(nRobot+1,1,n+1)
            for j in (range (0, nTask)):
                spfig.plot(taskList[j].xOrg,taskList[j].yOrg,'yo', markersize=10)
                spfig.annotate('T-%d'%j, (taskList[j].xOrg,taskList[j].yOrg+0.5),size=10, )
            for j in (range (0, nTask)):
                if not ((misc.eq(taskList[j].xFinish, taskList[j].xOrg)) and misc.eq(taskList[j].yFinish, taskList[j].yOrg)):
                    spfig.plot(taskList[j].xFinish,taskList[j].yFinish,'rs', markersize=10)
                    spfig.annotate('T-%d'%j, (taskList[j].xFinish,taskList[j].yFinish+0.5),size=10, )
            for i in (range (0, nRobot)):
                spfig.plot(robotList[i].xOrg,robotList[i].yOrg,'b*', markersize=10)
                spfig.annotate('R-%d'%i, (robotList[i].xOrg+0.5,robotList[i].yOrg-0.5), size=10, )

            spfig.plot(x[n],y[n], "-", linewidth=3)
            spfig.set_xlim(xMin-1,xMax+1)
            spfig.set_ylim(yMin-1,yMax+1)
            spfig.set_title("Robot Paths")
            spfig.grid(False)

        # combined plots
        spfig = fig.add_subplot(nRobot+1,1,nRobot+1)

        for j in (range (0, nTask)):
            spfig.plot(taskList[j].xOrg,taskList[j].yOrg,'yo', markersize=10)
            spfig.annotate('T-%d'%j, (taskList[j].xOrg,taskList[j].yOrg+0.5),size=10, )
        for j in (range (0, nTask)):
            if not ((misc.eq(taskList[j].xFinish, taskList[j].xOrg)) and misc.eq(taskList[j].yFinish, taskList[j].yOrg)):
                spfig.plot(taskList[j].xFinish,taskList[j].yFinish,'rs', markersize=10)
                spfig.annotate('T-%d'%j, (taskList[j].xFinish,taskList[j].yFinish+0.5),size=10, )
        for i in (range (0, nRobot)):
            spfig.plot(robotList[i].xOrg,robotList[i].yOrg,'b*', markersize=10)
            spfig.annotate('R-%d'%i, (robotList[i].xOrg+0.5,robotList[i].yOrg-0.5), size=10, )

        for i in (range (0, nRobot)):
            spfig.plot(x[i],y[i], "-", linewidth=3)
        spfig.set_xlim(xMin-1,xMax+1)
        spfig.set_ylim(yMin-1,yMax+1)
        spfig.set_title("Robot Paths")
        spfig.grid(False)

        fig.savefig(plotsDir+filePrefix+ "exec_time_cbpae_"+string.join(VERSION.split("."),"")+".png")
        pylab.close(fig)

#        fig2 = pylab.figure(figsize=(8,6))
#        spfig2 = fig2.add_subplot(111)
#        for i in (range (0, nRobot)):
#            for j in (range (0, nTaskAlloc[i])):
#                spfig2.broken_barh([(tExecStart[i][j] ,(tExecStop[i][j] - tExecStart[i][j]))],(rId[i]-0.125,.25),facecolors=("darkolivegreen"), edgecolor=("black"), linewidth=0.5)#facecolors=((1.0/nRobot)*i, (1.0/nRobot)*i, (1.0/nRobot)*i))
#                spfig2.broken_barh([(tNavStart[i][j] ,(tNavStop[i][j] - tNavStart[i][j]))],(rId[i]-0.125,.25),facecolors=("darkkhaki"), edgecolor=("black"), linewidth=0.5)#facecolors=((1.0/nRobot)*i, (1.0/nRobot)*i, (1.0/nRobot)*i))
#                spfig2.annotate('T-%d'%(rTasks[i][j]), (tNavStart[i][j] + (tExecStop[i][j] - tNavStart[i][j])/2.0-5, rId[i]+.3),weight='bold',size=14,)
#
#        spfig2.set_xlim(-1,round(max(execTime))+1)
#        spfig2.set_xlim(-1,((int(max(execTime))/50)+1)*50)
#        spfig2.set_ylim(-0.5,nRobot)
#        spfig2.set_xlabel("Time (s)")
#        spfig2.set_ylabel("Robot")
#        spfig2.set_title("Robot Schedules (Navigation & Execution)")
#        spfig2.grid(False)
#
#        fig1.savefig(plotsDir+filePrefix+"path_cbpae_"+version+".png")
#        fig2.savefig(plotsDir+filePrefix+"exec_time_cbpae_"+version+".png")
#
##        pylab.show()
#
#        pylab.close(fig1)
#        pylab.close(fig2)

if (__name__ == "__main__"):
    NR = [3]
    sets = [1,2,3]
    NT = [3,6,9]

    envs = ["rooms"]
    rTypes = ["hetero"]
    prios = ["diffprio"]
    starts = ["equalstart","randmstart"]

    for setNum in (sets):
        for nR in (NR):
            for nT in (NT):
                for env in (envs):
                    for rType in (rTypes):
                        for prio in (prios):
                            for start in (starts):
                                fname = "%d_%d_%s_%s_%s_%s_" %(nR, nT, env, rType, prio, start)
                                dataDir ="../data/"
                                logsDir ="../logs/set %d/" %(setNum)
                                plotsDir ="../plots/set %d/" %(setNum)

                                try:
                                    os.mkdir(plotsDir)
                                except:
                                    print ("could not create plot folders")
                                    pass

                                plotLogs(fname, dataDir, logsDir, plotsDir)

#! /usr/bin/env python
## ---------------------------------------------------- ##
# file: cbpae.py
# desc: main program for CBPAE. generates separate threads for each robot. logs and plots data.
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

import matplotlib.pyplot
import time
import readInData
import multiprocessing
import Queue
import misc
import gui
import wx
import os
import sys
import string

## cbpae - version
VERSION = "05.01.00"

## enable/disable logging
LOGGING = True
#LOGGING = False

# Consensus Based Parallel Allocation and Execution (CBPAE, Das et al., JINT 2015)
class Cbpae():
    def __init__(self, fName, logsDir):
        """consensus based parallel auction and execution.
        creates a list of robot processes(robotProc). each robotProc will
        handle the cbpae of a single robot. each robotProc has two
        message queues. one for incoming messages and the
        other for outgoing messages.
        """

        self.version = VERSION
        self.plots = plotsDir
        self.logs = logsDir
        self.fNameSmall = fName.split("/")[-1].split("\\")[-1]

        self.nRobot = 0  # number of robots
        self.nTask = 0  # number of tasks
        self.rConnect = [] # connection between robots
        self.robotList = {} # list of robot objects
        self.taskList = {} # list of task objects
        self.robotIds = []
        self.taskIds = []

        (self.robotIds, self.robotList, self.taskIds, self.taskList, self.mapInfo) = readInData.readInData(fName)
        self.nRobot = len(self.robotIds)
        self.nTask = len(self.taskIds)

    def run(self):
        """ generates a new process to handle all robot processes.
            checks for keyboard interrupt to exit gracefully"""

        robotProc = {} # robot processes as dict
        iRMsg = {} # inter robot message queues and locks
        robotStat = {} # robot process finish status back to main process
        robotInfo = {} # robot info queue from robotProcess -> guiProcesss
        robotCmd = {} # robot command queue from guiProcess -> robotProcess
        bcMsg = {} # msgQueues between broadcaster and robots
        cbpaeRun = [multiprocessing.Value('i'), multiprocessing.Lock()]

        cbpaeRun[0].value = 0

        for rId in (self.robotIds):
            iRMsg[rId] = [multiprocessing.Queue(), multiprocessing.Lock()]
            robotStat[rId] = [multiprocessing.Value('i'), multiprocessing.Lock()]
            robotStat[rId][0].value = 0 # setting the initial robot status as 0

            robotInfo[rId] = None
            robotCmd[rId] = None
            bcMsg[rId] = multiprocessing.Queue()

        time.sleep(.1)
        # workspace information list
        # [[nR, nT],[ws boundary],[robot id & start pose], [task id & start and finish poses]]
        # workspace info: will be passed to GUI
        wsInfo = []
        wsInfo.append([self.nRobot, self.nTask])
        wsInfo.append([self.mapInfo.xMin, self.mapInfo.xMax, self.mapInfo.yMin, self.mapInfo.yMax])
        wsInfo.append([[self.robotList[rId].index, self.robotList[rId].xOrg, self.robotList[rId].yOrg] for rId in (self.robotIds)])
        wsInfo.append([[self.taskList[tId].index, self.taskList[tId].xOrg, self.taskList[tId].yOrg, self.taskList[tId].xFinish, self.taskList[tId].yFinish, self.taskList[tId].taskType] for tId in (self.taskIds)])

        for rId in (self.robotIds):
            # robot command queue from guiProcess -> robotProcess
            robotCmd[rId] = multiprocessing.Queue()
            # populating message queues in robotInfoQ
            # each robot sends its coordinates which are then retrieved and plotted in GUI
            robotInfo[rId] = multiprocessing.Queue()

        # set the timeInit for all robots and tasks to monitor the activation time
        timeInit     = time.time()
        for rId in (self.robotIds):
            self.robotList[rId].setInitTime(timeInit)

        # create broadcasterProcess
        broadcasterProcess = multiprocessing.Process(target=self.broadCaster, args=(cbpaeRun, bcMsg, iRMsg))
        broadcasterProcess.start()
        print ("broadcaster pid = %d" %(broadcasterProcess.pid))

        # create robotProcesses
        for rId in (self.robotIds):
            robotProc[rId] = multiprocessing.Process(target=self.robotProcess, args=(self.robotList[rId], iRMsg, bcMsg[rId], robotInfo[rId], robotCmd[rId], robotStat[rId]))
        # start the robotProcesses
            robotProc[rId].start()
            print ("robot[%d] pid = %d" %(rId, robotProc[rId].pid))

        # start the gui process
        # gui app and process
        guiApp = wx.App(False)
        guiFrame = gui.MyFrame(parent=None, ID=100, title="CBPAE "+self.version, wsInfo=wsInfo, robotInfo=robotInfo, robotCmd=robotCmd)
        guiFrame.Show(True)
        guiP = multiprocessing.Process(target=guiApp.MainLoop)
        guiP.start()

        # looking for robotProcesses which have finished execution
        rJoinable = []
        while (True):
            # check status of each robot process
            time.sleep(.1)

            for rId in (self.robotIds):
                if (rId not in rJoinable):
                    if (robotProc[rId].is_alive()):
                        try:
                            lockStat = robotStat[rId][1].acquire(0)
                        except:
                            pass
                        else:
                            if (lockStat):
                                if (robotStat[rId][0].value == 1):
                                    rJoinable.append(rId)
                                robotStat[rId][1].release()
                    else:
                        rJoinable.append(rId)
                else:
                    pass

            if (len(rJoinable) == self.nRobot):
                break
        print ("received stop status from all robotProcesses")

        # sending stop signal to broadcasterProcess
        while (True):
            try:
                lockStat = cbpaeRun[1].acquire(0)
            except:
                pass
            else:
                if (lockStat):
                    cbpaeRun[0].value = 1
                    cbpaeRun[1].release()
                    break

        # reading all data from iRMsgQs before the robot processes are joined
        # if data is left in the queue, the subprocess won't be joined
        for rId in (self.robotIds):
            msgs = misc.recvMsg(iRMsg[rId][0])

            cmds = misc.recvMsg(robotCmd[rId])
            infos = misc.recvMsg(robotInfo[rId])

        # joining robot processes
        for rId in (self.robotIds):
            print ("robotProc[%d] joining now." %(rId))
            robotProc[rId].join(0)
            time.sleep(0.1)
            if (robotProc[rId].is_alive()):
                print ("robotProc[%d] joining failed. Terminating now." %(rId))
                robotProc[rId].terminate()

        #===============================================================
        # Combined Logging
        #===============================================================
        if (LOGGING):
            logFile = open(os.path.join(logsDir, self.fNameSmall + "_cbpae.log"), "w")
            print >>logFile, ("--basic info: start--")
            print >>logFile, ("%0.3f, %0.3f, %0.3f, %0.3f" %(self.mapInfo.xMin, self.mapInfo.xMax, self.mapInfo.yMin, self.mapInfo.yMax))
            print >>logFile, ("robots, %d" %(self.nRobot))
            for rId in (self.robotIds):
                print >>logFile, ("%d, %0.3f, %0.3f" %(rId, self.robotList[rId].xOrg, self.robotList[rId].yOrg))
            print >>logFile, ("tasks, %d" %(self.nTask))
            for tId in (self.taskIds):
                print >>logFile, ("%d, %0.3f, %0.3f, %0.3f, %0.3f" %(tId, self.taskList[tId].xOrg, self.taskList[tId].yOrg, self.taskList[tId].xFinish, self.taskList[tId].yFinish))
            print >>logFile, ("--basic info: end--")
            logFile.close()
        print ("CBPAE Trial Finished!!!")

    def broadCaster(self, cbpaeRun=[], bcMsg = {}, iRMsg={}):
        # cbpaeRun = [mp.Value(), mp.Lock()]
        # bMsg = {rId:mp.Queue()}
        # iRMsg = {rId:[mp.Queue(), mp.Lock()]}

        if (not(len(bcMsg) == len(iRMsg) == self.nRobot)):
            print ("Number of robots and the number pf message queues and locks are different")
            return

        cbpaeRunValue = 0
        while (cbpaeRunValue != 1):
            time.sleep(0.1)
            while (True):
                try:
                    dRLockStat = cbpaeRun[1].acquire(0)
                except:
                    pass
                else:
                    if (dRLockStat):
                        if (cbpaeRun[0].value == 1):
                            cbpaeRunValue = 1
                        cbpaeRun[1].release()
                        break
            # received the stop signal - break outside the while loops
            if (cbpaeRunValue == 1):
                break
            time.sleep(0.1)

            # cbpae running (stop signal not received)
            # receive bMsgQs
            msgs = []
            for rId in (self.robotIds):
                msg = misc.recvMsg(bcMsg[rId])
                if (len(msg) != 0):
                    msgs = msgs + msg
            time.sleep(0.1)

            # send received msgs
            for msg in (msgs):
                for rId in (self.robotIds):
                    misc.sendMsg(iRMsg[rId][0], iRMsg[rId][1], msg)

    def robotProcess(self, robot, iRMsg, bcMsg, robotInfo, robotCmd, robotStat):
        """"robot process: handles the process of a single robot"""
        rIndex = robot.index
        print ("r[%d] starting the CBPAE" %(rIndex))

        startRecvd = 0
        stopRecvd = 0
        # if gui is enabled, wait for the start command.
        while (True):
            cmds = misc.recvMsg(robotCmd)
            for cmd in (cmds):
                if (cmd == 0):
                    startRecvd = 1
                    break
                elif (cmd == 1):
                    stopRecvd = 1
                    break
                else:
                    pass

            if ((startRecvd == 1) or (stopRecvd == 1)):
                break
            time.sleep(0.1)

        # progress from here only if the robot is active
        # keep clearing the queue till then without processing the received msgs
        while (True):
            robot.checkStatus()
            if (robot.active == 1):
                print ("r[%d] is active now" %(rIndex))
                break
            else:
                msgs = misc.recvMsg(iRMsg[rIndex][0]) # keep clearing the message queue until active - avoids queue full
                time.sleep(0.1)

        # update active status all tasks
        robot.checkActiveTasks()

        # iRMsg
        # {rid:[iRMsgQ[rid][0], iRMsgLock[rid][1]], ... } currently used instead of neighbourInfo <ip,port>
        neighbourInfo = {} # {rid: [ip, port] ... } dynamically populated whenever a robot is added to the NL
        neighbourList = [] # [rid, ... ] dynamically populated

        tempBeats = [] # used for temperory storage of procesed beats msgs
        prevBeats = {} # dict of previous beats sent to neighbours. used to avoid unwanted beats sending

        # Create the initial NL
        # for limiting the communication frequency
        waitBeats = 1.0 # time between two inter-robot beats message multicasts
        waitInit = 10.0 # wait time for initialisation
        waitHs = 5.0 # wait between hs message broadcasts

        # spend some time populating the neighbour list here
        timeStart = time.time()
        timeNow = 0 + timeStart
        timePrevHs = 0

        # wait for some time for populating the NL initially
        while (misc.lt(timeNow-timeStart, waitInit)):
            # send hs
            if (misc.gt(timeNow-timePrevHs, waitHs)): # braodcast hs in every 2 seconds
                print ("r[%d] has %d more seconds to go" %(rIndex, waitInit-(timeNow-timeStart)))
                hs = robot.encodeHS()
#                print hs
                misc.sendMsg(bcMsg, None, hs)
                timePrevHs = 0 + hs[1]

            time.sleep(0.1)
            # recv hs & send hsack, if required
            beats = []
            msgs = misc.recvMsg(iRMsg[rIndex][0])
            for msg in (msgs):
                if (msg[0] == "hs"):
                    (rId, hsAck) = robot.encodeHSACK(msg, neighbourList)
                    if (rId != None):
                        neighbourList.append(rId)
                        misc.sendMsg(iRMsg[rId][0], iRMsg[rId][1], hsAck)
                elif (msg[0] == "beats"):
                    beats.append(msg)
                elif (msg[0] == "hsack"):
                    (rId, _tempBeats) = robot.decodeHSACK(msg, neighbourList)
                    if (rId != None):
                        neighbourList.append(rId)
                    if (_tempBeats != []):
                        beats.append(_tempBeats)
                        robot.initialBeats = 1 # mark beats for initial task update is received. prevents from sending further hs with beats requests
                else:
                    pass

            # decode beats - for random robot joining at a later stage
            # remove all info from unknown (not in NL) robots
            # [[rId, taskIdx, taskBidVal, taskExec, taskAlloc, taskBidTime, taskDropTime], ... ]
            (tempBeats, prevBeats) = robot.decodeBEATs(beats, neighbourList, prevBeats)
            # no bids have been placed yet, so all task information received will be processed by running consensusUnbidTask
            for msg in (tempBeats):
                # index, taskIds, taskAlloc, taskBid, taskExec, taskBidTime, taskDropTime, bidUpdate
                robot.consensusUnbidTask(msg[0], msg[1], msg[2], msg[3], msg[4], msg[5], msg[6], 0)

            # check for stop command from gui
            cmds = misc.recvMsg(robotCmd)
            for cmd in (cmds):
                if (cmd == 1):
                    print ("r[%d] received stop command" %(rIndex))
                    stopRecvd = 1
            if (stopRecvd == 1):
                break
            timeNow = time.time()

        if (neighbourList == []):
            freeTasks = 0
        else:
            print ("r[%d]\'s neighbourlist: " %(rIndex)),
            print (neighbourList)
            freeTasks = robot.checkFreeTasks()

        # initialising variables
        print ("r[%d] has found %d free tasks" %(rIndex, freeTasks))
        bidLost = 0
        loopCount = 0
        timePrevBeats = {} # time at which the previous beats from this robot was sent
        prevIdx = 0

        while (freeTasks > 0):
            # break if the client is not connected
            if (((robot.rType == "player") or (robot.rType == "playerstage")) and (robot.rClient == None)):
                break

            # update task active status in every loop
            robot.checkActiveTasks()

            # update NL - remove neighbours not sending beats from NL
            neighbourList = robot.updateNeighbourList(neighbourList, prevBeats)
            robot.nNeighbour = len(neighbourList)

            #===============================================================
            # copying robot information to robotInfoQ -> to gui
            #===============================================================
            newIdx = len(robot.x)
            for idx in (range (prevIdx, newIdx)):
                misc.sendMsg(robotInfo, None, [robot.x[idx], robot.y[idx]])
            prevIdx = newIdx + 0

            time.sleep(.1)
            if (robot.active == 1):
                #===============================================================
                # reading commands for safe exit from execution
                #===============================================================
                cmds = misc.recvMsg(robotCmd)
                for cmd in (cmds):
                    if (cmd == 1):
                        stopRecvd = 1
                        break

                if (stopRecvd == 1):
                    if (robot.taskProcess != None):
#                        print ("got stop command at robotProcess")
                        robot.stopExecution()
                        # wait until the robot process is finished
                        while (True):
                            time.sleep(0.1)
                            if (not(robot.taskProcess.is_alive())):
                                break
                        break
                    else:
                        break

                #===============================================================
                # ::communication phase::
                # sends messages to all robots in NL
                # the robot uses a lock for writing messages to queue for each robot
                # the robot reads the messages from its message queue
                #===============================================================
                # communicate-write
                # send beats to NL
                if (neighbourList != []):
                    # prepare beats
                    beats = robot.encodeBEATs(neighbourList)
                    timeNow = time.time()
                    for rId in (neighbourList):
                        # make sure there is a timePrevMsg for all robot in the NL
                        if (not(timePrevBeats.has_key(rId))):
                            timePrevBeats[rId] = robot.timeInit
                        # send BEATs if the timePrevMsg is older by msgTime than current time
                        if (misc.gt((timeNow - timePrevBeats[rId]), waitBeats)):
                            if (rId != rIndex):
                                timePrevBeats[rId] = beats[1]
                                misc.sendMsg(iRMsg[rId][0], iRMsg[rId][1], beats)

                # send hs
                if (misc.gt(timeNow-timePrevHs, waitHs)): # braodcast hs in every 2 seconds
                    hs = robot.encodeHS()
#                    print hs
                    misc.sendMsg(bcMsg, None, hs)
                    timePrevHs = 0 + hs[1]

                # communicate-read
                # all msgs sent to this robot will be received here
                # reset beats
                beats = []
                msgs = misc.recvMsg(iRMsg[rIndex][0])
                for msg in (msgs):
                    if (msg[0] == "hs"):
                        (rId, hsAck) = robot.encodeHSACK(msg, neighbourList)
                        if (rId != None):
                            neighbourList.append(rId)
                            misc.sendMsg(iRMsg[rId][0], iRMsg[rId][1], hsAck)
                    elif (msg[0] == "beats"):
                        beats.append(msg)
                    elif (msg[0] == "hsack"):
                        rId = robot.decodeHSACK(msg, neighbourList)
                        if (rId != None):
                            neighbourList.append(rId)
                    else:
                        pass

                # decode beats
                # [[rId, taskIdx, taskBidVal, taskExec, taskAlloc, taskBidTime, taskDropTime], ... ]
                (tempBeats, prevBeats) = robot.decodeBEATs(beats, neighbourList, prevBeats)

                #===================================================================
                # ::cbpae-core::
                #===================================================================

                #===============================================================
                # ::bidding and consensus phase::
                # createBundle():
                #    finds all free tasks and biddable tasks from them.
                #    bids for the highest priority and bidvalue
                # consensusBidTask():
                #    consensus rules for the task on which the robot has bid
                # consensusUnbidTask():
                #    consensus rules for all tasks other than the one the robot has bid on
                #===============================================================
                bidUpdate = 0
                taskUpdate = 0
                bidLost = 0
                # if the number of tasks robot bid is less than its limit
                if (robot.nTaskBid <= robot.nTaskLimit):
                    # the robot has not placed any bid yet
                    if (robot.nTaskBid == 0):
                        for msg in (tempBeats):
                            # index, taskIds, taskAlloc, taskBid, taskExec, taskBidTime, taskDropTime, bidUpdate
                            taskUpdate += robot.consensusUnbidTask(msg[0], msg[1], msg[2], msg[3], msg[4], msg[5], msg[6], 0)
                        if (robot.checkFreeTasks() != 0):
                            robot.createBundle()
                    # robot has placed atleast one bid
                    elif (robot.nTaskBid > 0):
                        # robot has no extra task than the one assigned to it at this stage - bid if there is any task is of itnerest
                        if (robot.nTaskBid == robot.nTaskAllocated):
                            if (robot.taskExec[robot.taskBidOrder[robot.nTaskAllocated-1]] == -2):
                                # process all unbid tasks
                                for msg in (tempBeats):
                                    # index, taskIds, taskAlloc, taskBid, taskExec, taskBidTime, taskDropTime, bidUpdate
                                    taskUpdate += robot.consensusUnbidTask(msg[0], msg[1], msg[2], msg[3], msg[4], msg[5], msg[6], 0)
                                # bid on a new task only if the bid limit is not reached and there is atleast one free task
                                if ((robot.checkFreeTasks() != 0) and (robot.nTaskBid != robot.nTaskLimit)):
                                    robot.createBundle()
                            elif (robot.taskExec[robot.taskBidOrder[robot.nTaskAllocated-1]] == -3):
                                # check the execution progress and parallel execution and decide whether to drop the task
                                taskStat = robot.checkTaskProgress()
                                # if the task is not yet finished, check for parallel executions
                                if (taskStat == 0):
                                    for msg in (tempBeats):
                                        # taskIds, taskAlloc, taskBid, taskExec, taskBidTime, taskDropTime
                                        robot.checkParallel(msg[1], msg[2], msg[3], msg[4], msg[5], msg[6])
                                    if (robot.stage == 0):
                                        # checking for a task dropping condition only if in stage 0
                                        robot.checkDrop()
                                # process all unbid tasks
                                for msg in (tempBeats):
                                    # index, taskIds, taskAlloc, taskBid, taskExec, taskBidTime, taskDropTime, bidUpdate
                                    taskUpdate += robot.consensusUnbidTask(msg[0], msg[1], msg[2], msg[3], msg[4], msg[5], msg[6], 0)
                                # bid on a new task only if the bid limit is not reached and there is atleast one free task
                                if ((robot.checkFreeTasks() != 0) and (robot.nTaskBid != robot.nTaskLimit)):
                                    robot.createBundle()
                        # robot has placed a bid on the next possible task - consensusBidTask and consensusUnbidTask
                        elif (robot.nTaskBid > robot.nTaskAllocated):
                            # the current bid is the first bid task
                            if (robot.nTaskBid == 1):
                                # update all unbid tasks - consensusUnbidTask
                                for msg in (tempBeats):
                                    # index, taskIds, taskAlloc, taskBid, taskExec, taskBidTime, taskDropTime, bidUpdate
                                    taskUpdate += robot.consensusUnbidTask(msg[0], msg[1], msg[2], msg[3], msg[4], msg[5], msg[6], 0)
                                # reach consensus on current bid task
                                rIds     = {}
                                for msg in (tempBeats):
                                    if (bidLost != 1):
                                        # index, taskIds, taskAlloc, taskBid, taskExec, taskBidTime, taskDropTime
                                        bidLost = robot.consensusBidTask(msg[0], msg[1], msg[2], msg[3], msg[4], msg[5], msg[6])
                                        if (bidLost == 1):
                                            bidUpdate += 1
                                        else:
                                            if (not(rIds.has_key(msg[0]))):
                                                robot.msgCount += 1
                                                rIds[msg[0]] = 0
                                # if the current bid is not lost, update the current bid or place a new bid on a better task
                                # if the bid is lost, the robot places a bid
                                if (bidLost != 1):
                                    robot.updateBundle()
                                elif (bidLost == 1):
                                    robot.createBundle()

                            # robot is executing current task and has a bid task
                            elif (robot.taskExec[robot.taskBidOrder[robot.nTaskAllocated-1]] == -3):
                                # check the execution progress and parallel execution and decide whether to drop the task
                                taskStat = robot.checkTaskProgress()
                                if (taskStat == 0):
                                    for msg in (tempBeats):
                                        # taskIds, taskAlloc, taskBid, taskExec, taskBidTime, taskDropTime
                                        robot.checkParallel(msg[1], msg[2], msg[3], msg[4], msg[5], msg[6])
                                    if (robot.stage == 0):
                                        # checking for a task dropping condition only if in stage 0
                                        robot.checkDrop()
                                # update all unbid tasks - consensusUnbidTask
                                for msg in (tempBeats):
                                    # index, taskIds, taskAlloc, taskBid, taskExec, taskBidTime, taskDropTime, bidUpdate
                                    taskUpdate += robot.consensusUnbidTask(msg[0], msg[1], msg[2], msg[3], msg[4], msg[5], msg[6], 0)
                                # reach consensus on current bid task
                                rIds = {}
                                for msg in (tempBeats):
                                    if (bidLost != 1):
                                        # index, taskIds, taskAlloc, taskBid, taskExec, taskBidTime, taskDropTime
                                        bidLost = robot.consensusBidTask(msg[0], msg[1], msg[2], msg[3], msg[4], msg[5], msg[6])
                                        if (bidLost == 1):
                                            bidUpdate += 1
                                        else:
                                            if (not(rIds.has_key(msg[0]))):
                                                robot.msgCount += 1
                                                rIds[msg[0]] = 0
                                # if the current bid is not lost, update the current bid or place a new bid on a better task
                                # if the bid is lost, the robot places a bid
                                if (bidLost != 1):
                                    robot.updateBundle()
                                elif (bidLost == 1):
                                    robot.createBundle()

                            elif (robot.taskExec[robot.taskBidOrder[robot.nTaskAllocated-1]] == -2):
                                # update all unbid tasks - consensusUnbidTask
                                for msg in (tempBeats):
                                    # index, taskIds, taskAlloc, taskBid, taskExec, taskBidTime, taskDropTime, bidUpdate
                                    taskUpdate += robot.consensusUnbidTask(msg[0], msg[1], msg[2], msg[3], msg[4], msg[5], msg[6], 0)
                                # reach consensus on current bid task
                                rIds = {}
                                for msg in (tempBeats):
                                    if (bidLost != 1):
                                        # index, taskIds, taskAlloc, taskBid, taskExec, taskBidTime, taskDropTime
                                        bidLost = robot.consensusBidTask(msg[0], msg[1], msg[2], msg[3], msg[4], msg[5], msg[6])
                                        if (bidLost == 1):
                                            bidUpdate += 1
                                        else:
                                            if (not(rIds.has_key(msg[0]))):
                                                robot.msgCount += 1
                                                rIds[msg[0]] = 0
                                # if the current bid is not lost, update the current bid or place a new bid on a better task
                                # if the bid is lost, the robot places a bid
                                if (bidLost != 1):
                                    robot.updateBundle()
                                elif (bidLost == 1):
                                    robot.createBundle()
                            else:
                                print ("error 1")
                        else:
                            print ("error 2")
                    else:
                        print ("error 3")
                else:
                    print ("error 4")

                #===============================================================
                # ::task assignment phase::
                # allocateTasks():
                #    assigns tasks based on the msgCount and current task status
                #===============================================================
                robot.allocateTask()

            # if the robot is not active, check the status
            else:
                robot.checkStatus()
                # communicate-read to clear the queue
                msgs = misc.recvMsg(iRMsg[rIndex][0])

            #===================================================================
            # ::exit policy::
            # checkFreeTasks():
            #    finds if there are any tasks being executed or has to be executed.
            #    this is used as the exit condition from the while loop.
            #    in an real situation, this can run in an infinite loop.
            #===================================================================
            freeTasks = 0
            freeTasks = robot.checkFreeTasks()
            loopCount += 1

            # if all tasks are finished, continue communication process for a few times
            if (freeTasks == 0):
                timeNow = time.time()
                timeStart2 = 0 + timeNow
                tempBeats = []
                while (misc.lt(timeNow-timeStart2, waitInit)):
                    time.sleep(0.1)
                    #===============================================================
                    # ::final coomunication phase::
                    # a robot reaches here only when it finishes all tasks. the last
                    # robot finishing will not send the beats and causes others to
                    # wait in an infinite loop.
                    # receive messages (hs, hsack, beats)
                    #===============================================================
                    # recv hs & send hsack, if required
                    msgs = misc.recvMsg(iRMsg[rIndex][0])
                    for msg in (msgs):
                        if (msg[0] == "hs"):
                            (rId, hsAck) = robot.encodeHSACK(msg, neighbourList)
                            if (rId != None):
                                neighbourList.append(rId)
                                misc.sendMsg(iRMsg[rId][0], iRMsg[rId][1], hsAck)
                        elif (msg[0] == "beats"):
                            tempBeats.append(msg)
                        elif (msg[0] == "hsack"):
                            rId = robot.decodeHSACK(msg, neighbourList)
                            if (rId != None):
                                neighbourList.append(rId)
                        else:
                            pass

                    cmds = misc.recvMsg(robotCmd)
                    for cmd in (cmds):
                        if (cmd == 1):
                            print ("r[%d] received stop command" %(rIndex))
                            stopRecvd = 1
                    if (stopRecvd == 1):
                        break
                    #===============================================================
                    # send messages (hs, hsack, beats)
                    #===============================================================
                    # send hs
                    if (misc.gt(timeNow-timePrevHs, waitHs)):     # braodcast hs in every 2 seconds
                        print ("r[%d] has %d more seconds to go" %(rIndex, waitInit-(timeNow-timeStart2)))
                        hs = robot.encodeHS()
        #                print hs
                        misc.sendMsg(bcMsg, None, hs)
                        timePrevHs = 0 + hs[1]

                    # send beats to NL
                    if (neighbourList != []):
                        # prepare beats
                        beats = robot.encodeBEATs(neighbourList)
                        timeNow = time.time()
                        for rId in (neighbourList):
                            # make sure there is a timePrevMsg for all robot in the NL
                            if (not(timePrevBeats.has_key(rId))):
                                timePrevBeats[rId] = robot.timeInit
                            # send BEATs if the timePrevMsg is older by msgTime than current time
                            if (misc.gt((timeNow - timePrevBeats[rId]), waitBeats)):
                                if (rId != rIndex):
                                    timePrevBeats[rId] = beats[1]
                                    misc.sendMsg(iRMsg[rId][0], iRMsg[rId][1], beats)

                    timeNow = time.time()

            if (loopCount%1000 == 0):
                for tId in (self.taskIds):
                    if (robot.taskExec[tId] != -2):
                        print ("r[%d] thinks t[%d] is in '%s' state" %(rIndex, tId, robot.taskExec[tId]))

        timeStop = time.time()
        timeExec = timeStop - timeStart

        #=======================================================================
        # FIXME: when the rId of robots are not in order / continuous, problem can occur in reading
        # The solution could be using dicts instead of lists. The key shall be the ID of the robot/task
        #=======================================================================
        if (LOGGING):
            print ("r[%d] starting logging" %(rIndex))
            logFile = open(os.path.join(self.logs, string.join((self.fNameSmall, str(rIndex), "cbpae.log"), "_")), "w")

            rDist = 0
            lenX = len(robot.x)
            for m in (range (1, lenX, 1)):
                rDist += misc.getDist(robot.x[m-1], robot.y[m-1], robot.x[m], robot.y[m])

            print >> logFile, ("Robot, %d" %(rIndex))
            print >> logFile, ("Init Time, %.4f" %(robot.timeInit))
            print >> logFile, ("Start Time, %.4f" %(timeStart))
            print >> logFile, ("Stop Time, %.4f" %(timeStop))
            print >> logFile, ("Exec Time, %.4f" %(timeExec))
            print >> logFile, ("No of Tasks Allocated, %d" %(robot.nTaskAllocated))

            allocatedTasks = []
            print >> logFile, ("Tasks Allocated, "),
            if (robot.nTaskAllocated != 0):
                for j in (range (0, robot.nTaskAllocated-1, 1)):
                    print >> logFile, ("%d, " %(robot.taskBidOrder[j])),
                    allocatedTasks.append(robot.taskBidOrder[j])
                print >> logFile, (robot.taskBidOrder[robot.nTaskAllocated-1])
                allocatedTasks.append(robot.taskBidOrder[robot.nTaskAllocated-1])
            else:
                print >> logFile, ("")

            print >> logFile, ("Task Bid Time, "),
            if (robot.nTaskAllocated > 0):
                for t in (allocatedTasks):
                    print >> logFile, ("%.4f, " %(robot.taskBidTime[t])),
                print >> logFile, ("")
            else:
                print >> logFile, ("")

            print >> logFile, ("Task Bid Value, "),
            if (robot.nTaskAllocated > 0):
                for t in (allocatedTasks):
                    print >> logFile, ("%.4f, " %(robot.taskBidVal[t])),
                print >> logFile, ("")
            else:
                print >> logFile, ("")

            print >> logFile, ("Navigation to Task: Start Time, "),
            if (robot.nTaskAllocated > 0):
                for j in (range (0, robot.nTaskAllocated-1, 1)):
                    print >> logFile, ("%.4f, " %(robot.taskStartTime[j][0] - robot.timeInit)),
                print >> logFile, ("%.4f" %(robot.taskStartTime[robot.nTaskAllocated-1][0] - robot.timeInit))
            else:
                print >> logFile, ("")

            print >> logFile, ("Navigation to Task: Stop Time, "),
            if (robot.nTaskAllocated > 0):
                for j in (range (0, robot.nTaskAllocated-1, 1)):
                    print >> logFile, ("%.4f, " %(robot.taskEndTime[j][0] - robot.timeInit)),
                print >> logFile, ("%.4f" %(robot.taskEndTime[robot.nTaskAllocated-1][0] - robot.timeInit))
            else:
                print >> logFile, ("")

            print >> logFile, ("Execution of Task: Start Time, "),
            if (robot.nTaskAllocated > 0):
                for j in (range (0, robot.nTaskAllocated-1, 1)):
                    print >> logFile, ("%.4f, " %(robot.taskStartTime[j][1] - robot.timeInit)),
                print >> logFile, ("%.4f" %(robot.taskStartTime[robot.nTaskAllocated-1][1] - robot.timeInit))
            else:
                print >> logFile, ("")

            print >> logFile, ("Execution of Task: Stop Time, "),
            if (robot.nTaskAllocated > 0):
                for j in (range (0, robot.nTaskAllocated-1, 1)):
                    print >> logFile, ("%.4f, " %(robot.taskEndTime[j][1] - robot.timeInit)),
                print >> logFile, ("%.4f" %(robot.taskEndTime[robot.nTaskAllocated-1][1] - robot.timeInit))
            else:
                print >> logFile, ("")

            eTaskAllocated = 0
            emergencyTasks = []
            emergencyTaskOn = []
            emergencyTaskStart = []
            emergencyTaskStop = []

            if (robot.nTaskAllocated != 0):
                for j in (range (0, robot.nTaskAllocated, 1)):
                    if (robot.taskList[robot.taskBidOrder[j]].stcPrio == 0):
                        eTaskAllocated += 1
                        emergencyTasks.append(robot.taskBidOrder[j])
                        emergencyTaskOn.append(robot.taskList[robot.taskBidOrder[j]].timeOn)
                        emergencyTaskStart.append(robot.taskStartTime[j][0] - robot.taskList[j].timeInit)
                        emergencyTaskStop.append(robot.taskStartTime[j][1] - robot.taskList[j].timeInit)

            print >> logFile, ("Emergency Tasks Allocated, %d" %(eTaskAllocated))

            print >> logFile, ("Emergency Tasks, "),
            if (eTaskAllocated > 0):
                for j in (range (0, eTaskAllocated-1, 1)):
                    print >> logFile, ("%d, " %(emergencyTasks[j])),
                print >> logFile, ("%d" %(emergencyTasks[eTaskAllocated-1]))
            else:
                print >> logFile, ("")

            print >> logFile, ("Emergency Task On-Time, "),
            if (eTaskAllocated > 0):
                for j in (range (0, eTaskAllocated-1, 1)):
                    print >> logFile, ("%.4f, " %(emergencyTaskOn[j])),
                print >> logFile, ("%.4f" %(emergencyTaskOn[eTaskAllocated-1]))
            else:
                print >> logFile, ("")

            print >> logFile, ("Emergency Task Start Time, "),
            if (eTaskAllocated > 0):
                for j in (range (0, eTaskAllocated-1, 1)):
                    print >> logFile, ("%.4f, " %(emergencyTaskStart[j])),
                print >> logFile, ("%.4f" %(emergencyTaskStart[eTaskAllocated-1]))
            else:
                print >> logFile, ("")

            print >> logFile, ("Emergency Task Stop Time, "),
            if (eTaskAllocated > 0):
                for j in (range (0, eTaskAllocated-1, 1)):
                    print >> logFile, ("%.4f, " %(emergencyTaskStop[j])),
                print >> logFile, ("%.4f" %(emergencyTaskStop[eTaskAllocated-1]))
            else:
                print >> logFile, ("")

            print >> logFile, ("Distance traveled, %0.3f " %(rDist))

            # robot coordinates
            for idx in (range (0,len(robot.x))):
                print >> logFile, ("%0.3f, %0.3f" %(robot.x[idx], robot.y[idx]))

            logFile.close()


#        print ("r[%d] starting closing processes" %(rIndex))
        #===============================================================
        # ::after execution communication phase::
        # sends messages to all communicating robots
        # the robot uses a lock for writing messages to queue for each robot
        # the robot reads the messages from its message queue
        #===============================================================
        # communicate-write
        timeNow = time.time()
        while (misc.lt((time.time() - timeNow),5)):
            if (robot.rClient == None):
                break
            time.sleep(0.100)
            for rId in (neighbourList):
                lockStatus = False
                if (rId != rIndex):
                    try:
                        lockStatus = iRMsg[rId][1].acquire(0)
                    except:
                        print ("error in acquiring iRMsgLock[%d] by robot[%d]" %(rId, rIndex))
                        pass
                    else:
                        if (lockStatus):
                            try:
                                iRMsg[rId][0].put_nowait([robot.index, robot.taskAlloc, robot.taskBidVal, robot.taskExec, robot.taskBidTime, robot.taskDropTime])
                            except Queue.Full:
                                print ("inter-robot Queue full")
                                iRMsg[rId][1].release()
                            except:
                                iRMsg[rId][1].release()
                            else:
                                iRMsg[rId][1].release()
        # to avoid problems with data in queue
        iRMsg[rIndex][0].cancel_join_thread()
        robotCmd.cancel_join_thread()
        robotInfo.cancel_join_thread()

        print ("r[%d] diconnecting clients now" %(rIndex))
        # disconnect all proxies and client
        if (robot.rClient == None):
            print ("r[%d] player-client not connected" %(rIndex))
        elif (robot.rClient == "dummy variable"):
            print ("r[%d] player-client not connected" %(rIndex))
        else:
            robot.discClient()

        print ("robot [%d]: loopCount = %d" %(rIndex, loopCount))

        # putting finished status
        timeNow = time.time()
        while (misc.lt((time.time() - timeNow),5)):
            while (True):
                try:
                    lockStat = robotStat[1].acquire(0)
                except:
                    pass
                else:
                    if (lockStat):
                        robotStat[0].value = 1
                        robotStat[1].release()
                        break
        return

    def plotData(self, logsDir, plotsDir, fName):
        """"read all logs. merge them to a single one. plot all data in single plot"""
        print (">>> plotting data")
        
        fNameSmall = fName.split("/")[-1].split("\\")[-1]

        class Robot():
            def __init__(self, xOrg, yOrg):
                self.xOrg = 0.0 + xOrg
                self.yOrg = 0.0 + yOrg

        class Task():
            def __init__(self, xOrg, yOrg, xFinish, yFinish):
                self.xOrg = 0.0 + xOrg
                self.yOrg  = 0.0 + yOrg
                self.xFinish = 0.0 + xFinish
                self.yFinish = 0.0 + yFinish

        robotList = []
        taskList = []
        rId = []

        logFile = open(os.path.join(logsDir, fNameSmall + "_cbpae.log"), "r")
        logFile.readline()
        (xMin, xMax, yMin, yMax) = (float(x) for x in (logFile.readline().strip().split(",")))

        nRobot = int ((logFile.readline().strip().split(","))[1],10)
        for i in (range (nRobot)):
            robotData = logFile.readline().strip().split(",")
            rId.append(int(robotData[0],10))
            robotList.append(Robot(float(robotData[1]), float(robotData[2])))

        nTask = int ((logFile.readline().strip().split(","))[1],10)
        for j in (range (nTask)):
            taskData = logFile.readline().strip().split(",")
            taskList.append(Task(float(taskData[1]), float(taskData[2]), float(taskData[3]), float(taskData[4])))
        logFile.close()

        timeInit = []
        startTime = []
        stopTime = []
        execTime = []
        nTaskAlloc = []
        rTasks = []
        nRAct = []
        bidTime = []
        bidVal = []
        tNavStart = []
        tNavStop = []
        tExecStart = []
        tExecStop = []
        eTasks = []
        eTaskOn = []
        eTaskStart = []
        eTaskStop = []
        rDist = []
        x = []
        y = []

        for i in (range (nRobot)):
            fHandle = open(os.path.join(logsDir, string.join((fNameSmall, str(i), "cbpae.log"), "_")), "r")
            rId.append(int((fHandle.readline().strip().split(","))[1],10))

            timeInit.append(float((fHandle.readline().strip().split(","))[1]))
            startTime.append(float((fHandle.readline().strip().split(","))[1]))
            stopTime.append(float((fHandle.readline().strip().split(","))[1]))
            execTime.append(float((fHandle.readline().strip().split(","))[1]))

            nTaskAlloc.append(int((fHandle.readline().strip().split(","))[1],10))

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
                tasks = fHandle.readline().strip().split(",")
                for j in (range (1, nTaskAlloc[i]+1)):
                    rTasks[-1].append(int(tasks[j], 10))

                bTime = fHandle.readline().strip().split(",")
                for j in (range (1, nTaskAlloc[i]+1)):
                    bidTime[-1].append(float(bTime[j]))

                bVal = fHandle.readline().strip().split(",")
                for j in (range (1, nTaskAlloc[i]+1)):
                    bidVal[-1].append(float(bVal[j]))

                start = fHandle.readline().strip().split(",")
                for j in (range (1, nTaskAlloc[i]+1)):
                    tNavStart[-1].append(float(start[j]))

                stop = fHandle.readline().strip().split(",")
                for j in (range (1, nTaskAlloc[i]+1)):
                    tNavStop[-1].append(float(stop[j]))

                start = fHandle.readline().strip().split(",")
                for j in (range (1, nTaskAlloc[i]+1)):
                    tExecStart[-1].append(float(start[j]))

                stop = fHandle.readline().strip().split(",")
                for j in (range (1, nTaskAlloc[i]+1)):
                    tExecStop[-1].append(float(stop[j]))

                numETasks = int(fHandle.readline().strip().split(",")[1],10)
                if (numETasks > 0):
                    eTaskList = fHandle.readline().strip().split(",")
                    for j in (range (1, numETasks+1)):
                        eTasks.append(int(eTaskList[j],10))

                    eTaskInitList = fHandle.readline().strip().split(",")
                    for j in (range (1, numETasks+1)):
                        eTaskOn.append(float(eTaskInitList[j]))

                    eTaskStartList = fHandle.readline().strip().split(",")
                    for j in (range (1, numETasks+1)):
                        eTaskStart.append(float(eTaskStartList[j]))

                    eTaskStopList = fHandle.readline().strip().split(",")
                    for j in (range (1, numETasks+1)):
                        eTaskStop.append(float(eTaskStopList[j]))
                else:
                    fHandle.readline()
                    fHandle.readline()
                    fHandle.readline()
                    fHandle.readline()

                rDist.append(float(fHandle.readline().strip().split(",")[1]))
                while (True):
                    try:
                        xy = fHandle.readline().strip().split(",")
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
            fHandle.close()

        eTaskResponseTime = []
        eTaskExecutionTime = []
        for idx in (range (len(eTasks))):
            eTaskResponseTime.append(eTaskStart[idx] - eTaskOn[idx])
            eTaskExecutionTime.append(eTaskStop[idx] - eTaskStart[idx])

        totalDist = sum(rDist)
        avgDist = totalDist/nRobot
        if (sum(nRAct) != 0):
            avgDistAct = totalDist/sum(nRAct)
        else:
            avgDistAct = 0

        logFile = open(os.path.join(logsDir, string.join([fNameSmall,"cbpae.log"], "_")), "a")
        print >> logFile, ("--summary: start--")
        print >> logFile, ("Number of active robots = %d" %sum(nRAct))
        print >> logFile, ("Tasks assigned to each robot:")
        for i in (range (0, nRobot, 1)):
            print >> logFile, ("Robot[%d]:"%rId[i])
            print >>logFile, ("Tasks: ["),
            for j in (range (0, nTaskAlloc[i], 1)):
                print >> logFile, ("%d, " %(rTasks[i][j])),
            print >> logFile, ("]")
            print >>logFile, ("BidTime: ["),
            for j in (range (0, nTaskAlloc[i], 1)):
                print >> logFile, ("%.4f, "%(bidTime[i][j])),
            print >> logFile, ("]")
            print >>logFile, ("BidVal: ["),
            for j in (range (0, nTaskAlloc[i], 1)):
                print >> logFile, ("%.4f, "%(bidVal[i][j])),
            print >> logFile, ("]")
            print >>logFile, ("NavStart: ["),
            for j in (range (0, nTaskAlloc[i], 1)):
                print >> logFile, ("%.4f, "%(tNavStart[i][j])),
            print >> logFile, ("]")
            print >>logFile, ("NavEnd: ["),
            for j in (range (0, nTaskAlloc[i], 1)):
                print >> logFile, ("%.4f, "%(tNavStop[i][j])),
            print >> logFile, ("]")
            print >>logFile, ("ExecStart: ["),
            for j in (range (0, nTaskAlloc[i], 1)):
                print >> logFile, ("%.4f, "%(tExecStart[i][j])),
            print >> logFile, ("]")
            print >>logFile, ("ExecEnd: ["),
            for j in (range (0, nTaskAlloc[i], 1)):
                print >> logFile, ("%.4f, "%(tExecStop[i][j])),
            print >> logFile, ("]")
            print >> logFile, ("Distance travelled = %0.4f" %(rDist[i]))
            print >> logFile, ("Tasks Allocated = %d"  %(nTaskAlloc[i]))
        print >> logFile, ("totalDist = %0.3f" %(totalDist))
        print >> logFile, ("avgDist = %0.3f" %(avgDist))
        print >> logFile, ("avgDistAct = %0.3f" %(avgDistAct))
        print >>logFile, ("Total time of execution = %0.3f" %(max(execTime)))
        if (len(eTasks) != 0):
            print >> logFile, ("Emergency Tasks = %d" %(len(eTasks)))
            print >> logFile, ("Minimum Response Time = %0.3f" %(min(eTaskResponseTime)))
            print >> logFile, ("Maximum Response Time = %0.3f" %(max(eTaskResponseTime)))
            print >> logFile, ("Average Response Time = %0.3f" %(sum(eTaskResponseTime)/len(eTaskResponseTime)))
            print >> logFile, ("Minimum Execution Time = %0.3f" %(min(eTaskExecutionTime)))
            print >> logFile, ("Maximum Execution Time = %0.3f" %(max(eTaskExecutionTime)))
            print >> logFile, ("Average Execution Time = %0.3f" %(sum(eTaskExecutionTime)/len(eTaskExecutionTime)))
        print >> logFile, ("--summary: end--")
        print >> logFile, ("--coordinates of robot paths: start--")
        for i in (range (0, nRobot, 1)):
            print >> logFile, ("--robot: %d--" %rId[i])
            for idx in (range (0,len(x[i]))):
                print >> logFile, ("%0.3f,%0.3f" %(x[i][idx], y[i][idx]))
        print >> logFile, ("--coordinates of robot paths: end--")
        logFile.close()

        ## plotting the paths
        maxXY = max([xMax-xMin, yMax-yMin])
        figLen = misc.rescale(0, maxXY, 0, 8, xMax-xMin)
        figWid = misc.rescale(0, maxXY, 0, 8, yMax-yMin)
        fig1 = matplotlib.pyplot.figure(figsize=(figLen, figWid))
        spfig1 = fig1.add_subplot(111)

        for j in (range (0, nTask)):
            spfig1.plot(taskList[j].xOrg, taskList[j].yOrg,'yo', markersize=10)
            spfig1.annotate('T-%d'%j, (taskList[j].xOrg, taskList[j].yOrg+0.5),size=10, )

        for j in (range (0, nTask)):
            if (not ((misc.eq(taskList[j].xFinish, taskList[j].xOrg)) and misc.eq(taskList[j].yFinish, taskList[j].yOrg))):
                spfig1.plot(taskList[j].xFinish, taskList[j].yFinish,'rs', markersize=10)
                spfig1.annotate('T-%d'%j, (taskList[j].xFinish, taskList[j].yFinish+0.5),size=10, )

        for i in (range (0, nRobot)):
            spfig1.plot(robotList[i].xOrg, robotList[i].yOrg,'b*', markersize=10)
            spfig1.annotate('R-%d'%i, (robotList[i].xOrg+0.5, robotList[i].yOrg-0.5), size=10, )

        for i in (range (0, nRobot)):
            spfig1.plot(x[i],y[i], "-", linewidth=3)
        spfig1.set_xlim(xMin-1,xMax+1)
        spfig1.set_ylim(yMin-1,yMax+1)
        spfig1.set_title("Robot Paths")
        spfig1.grid(False)

        fig2 = matplotlib.pyplot.figure(figsize=(8,6))
        spfig2 = fig2.add_subplot(111)
        for i in (range (0, nRobot)):
            for j in (range (0, nTaskAlloc[i])):
                spfig2.broken_barh([(tExecStart[i][j] ,(tExecStop[i][j] - tExecStart[i][j]))],(rId[i]-0.125,.25),facecolors=("darkolivegreen"), edgecolor=("black"), linewidth=0.5)#facecolors=((1.0/nRobot)*i, (1.0/nRobot)*i, (1.0/nRobot)*i))
                spfig2.broken_barh([(tNavStart[i][j] ,(tNavStop[i][j] - tNavStart[i][j]))],(rId[i]-0.125,.25),facecolors=("darkkhaki"), edgecolor=("black"), linewidth=0.5)#facecolors=((1.0/nRobot)*i, (1.0/nRobot)*i, (1.0/nRobot)*i))
                spfig2.annotate('T-%d'%(rTasks[i][j]), (tNavStart[i][j] + (tExecStop[i][j] - tNavStart[i][j])/2.0-5, rId[i]+.3),weight='bold',size=14,)

        spfig2.set_xlim(-1,round(max(execTime))+1)
        spfig2.set_xlim(-1,((int(max(execTime))/50)+1)*50)
        spfig2.set_ylim(-0.5, nRobot)
        spfig2.set_xlabel("Time (s)")
        spfig2.set_ylabel("Robot")
        spfig2.set_title("Robot Schedules (Navigation & Execution)")
        spfig2.grid(False)

        fig3 = matplotlib.pyplot.figure(figsize=(8,6))
        spfig3 = fig3.add_subplot(111)
        for i in (range (0, nRobot)):
            for j in (range (0, nTaskAlloc[i])):
                spfig3.broken_barh([(tExecStart[i][j] ,(tExecStop[i][j] - tExecStart[i][j]))],(rId[i]-0.125,.25),facecolors=("darkolivegreen"), edgecolor=("black"), linewidth=0.5)#facecolors=((1.0/nRobot)*i, (1.0/nRobot)*i, (1.0/nRobot)*i))
                spfig3.broken_barh([(tNavStart[i][j] ,(tNavStop[i][j] - tNavStart[i][j]))],(rId[i]-0.125,.25),facecolors=("darkkhaki"), edgecolor=("black"), linewidth=0.5)#facecolors=((1.0/nRobot)*i, (1.0/nRobot)*i, (1.0/nRobot)*i))
                spfig3.plot(bidTime[i][j],rId[i],'b*', markersize=10)
                spfig3.annotate('%d\n%0.1f\n%0.1f'%(rTasks[i][j], bidVal[i][j], bidTime[i][j]), (bidTime[i][j]-5, rId[i]+.25),weight='bold',size=7,)
#                spfig3.annotate('%0.1f'%(rTasks[i][j], bidVal[i][j], bidTime[i][j]), (bidTime[i][j]-5, rId[i]-.25),weight='bold',size=8,)

        spfig3.set_xlim(-10,round(max(execTime))+1)
        spfig3.set_xlim(-10,((int(max(execTime))/50)+1)*50)
        spfig3.set_ylim(-0.5, nRobot)
        spfig3.set_xlabel("Time (s)")
        spfig3.set_ylabel("Robot")
        spfig3.set_title("Robot Bid Time and Bid Value")
        spfig3.grid(False)

        fig1.savefig(os.path.join(self.plots, string.join((fNameSmall, "path", "cbpae", self.version+".png"), "_")))
        fig2.savefig(os.path.join(self.plots, string.join((fNameSmall, "exec_time", "cbpae", self.version+".png"), "_")))
        fig3.savefig(os.path.join(self.plots, string.join((fNameSmall, "bid_time_val", "cbpae", self.version+".png"), "_")))

#        matplotlib.pyplot.show()

        matplotlib.pyplot.close(fig1)
        matplotlib.pyplot.close(fig2)
        matplotlib.pyplot.close(fig3)

        print ("Plotting Finished!!!")

if (__name__ == "__main__"):

    print sys.argv
    if len(sys.argv) <= 1:
        print "usage: cbpae.py <path_to_data_file>"
    else:
        fName     = sys.argv[1]
        
        main_dir = os.path.join(os.environ["HOME"], "cbpae")

        logsDir = os.path.join(main_dir, "logs")
        plotsDir = os.path.join(main_dir, "plots")
        
        if not os.path.exists(main_dir):
            os.mkdir(main_dir)
        
        if not os.path.exists(logsDir):
            os.mkdir(logsDir)
        
        if not os.path.exists(plotsDir):
            os.mkdir(plotsDir)
        
        test = Cbpae(fName, logsDir)
        test.run()
        if (LOGGING):
            test.plotData(logsDir, plotsDir, fName)






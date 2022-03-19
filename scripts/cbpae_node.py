#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on: Day Mon DD HH:MM:SS YYYY

@author: gpdas
"""
LOGGING = True

import rospy
import cbpae
import sys
import os
import actionlib
import move_base_msgs.msg
import multiprocessing
import time
import misc


class RosCbpae(cbpae.Cbpae):
    def __init__(self, fName, logsDir):

        super(RosCbpae, self).__init__(fName, logsDir)

        self.rosRIds = []
        self.mbClients = {}
        self.mbGoals = {}
        self.mbStates = {}
        self.mbGoalsActive = {}

        for rId in self.robotIds:
            if self.robotList[rId].rType == "ros":
                self.rosRIds.append(rId)
                self.mbClients[rId] = actionlib.SimpleActionClient(self.robotList[rId].ns+"/move_base", move_base_msgs.msg.MoveBaseAction)
                self.mbClients[rId].wait_for_server()
                self.mbGoals[rId] = multiprocessing.Queue()
                self.mbStates[rId] = multiprocessing.Queue()
                self.mbGoalsActive[rId] = False

    def startRobots(self, robotProc, iRMsg, bcMsg, robotInfo, robotCmd, robotStat):
        # create robotProcesses
        for rId in (self.robotIds):
            if self.robotList[rId].rType == "ros":
                robotProc[rId] = multiprocessing.Process(target=self.robotProcess, args=(self.robotList[rId], iRMsg, bcMsg[rId], robotInfo[rId], robotCmd[rId], robotStat[rId], self.mbGoals[rId], self.mbStates[rId]))
            else:
                robotProc[rId] = multiprocessing.Process(target=self.robotProcess, args=(self.robotList[rId], iRMsg, bcMsg[rId], robotInfo[rId], robotCmd[rId], robotStat[rId]))
            # start the robotProcesses
            robotProc[rId].start()
            print ("robot[%d] pid = %d" %(rId, robotProc[rId].pid))
        return robotProc

    def checkRJoinable(self, robotProc, robotStat):
        # looking for robotProcesses which have finished execution
        rJoinable = []
        goal = move_base_msgs.msg.MoveBaseGoal()
        # Possible States Are: PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST (if no goal)
        stateTxt = {0:"PENDING", 1:"ACTIVE", 2:"PREEMPTED",
                      3:"SUCCEEDED", 4:"ABORTED", 5:"REJECTED",
                      6:"PREEMPTING", 7:"RECALLING", 8:"RECALLED",
                      9:"LOST"}

        while (True):
            # check status of each robot process
            time.sleep(.1)
            # check for mbGoals from each robot
            for rId in self.rosRIds:
                if not self.mbGoals[rId].empty():
                    (toDo, x, y) = self.mbGoals[rId].get_nowait()
                    if toDo == "goal":
                        goal.target_pose.header.frame_id = "map"
                        goal.target_pose.pose.position.x = x
                        goal.target_pose.pose.position.y = y
                        goal.target_pose.pose.orientation.w = 1
                        self.mbClients[rId].send_goal(goal)
                        self.mbGoalsActive[rId] = True
                    elif toDo == "cancel":
                        self.mbClients[rId].cancel_all_goals()

            # check for mbClient status
            for rId in self.rosRIds:
                if self.mbGoalsActive[rId]:
                    state = self.mbClients[rId].get_state()

                    if stateTxt[state] == "SUCCEEDED":
                        self.mbStates[rId].put_nowait(stateTxt[state])
                        self.mbGoalsActive[rId] = False
#                        print (rId, stateTxt[state])

            # checking for robot processes can be joined (are they completed?)
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

        return rJoinable

    def run(self):
        """ generates a new process to handle all robot processes.
            checks for keyboard interrupt to exit gracefully"""

        (robotProc, iRMsg, robotStat, robotInfo, robotCmd, bcMsg, cbpaeRun, wsInfo) = self.prepVars()

        broadcasterProc = self.startBroadcaster(cbpaeRun, bcMsg, iRMsg)

# =============================================================================
#         # pass additional queues to the robot processes by overloading this method
# =============================================================================
        robotProc = self.startRobots(robotProc, iRMsg, bcMsg, robotInfo, robotCmd, robotStat)

        guiProc = self.startGui(wsInfo, robotInfo, robotCmd)

# =============================================================================
#         # This is the main loop checking robotProcs
# =============================================================================
        rJoinable = self.checkRJoinable(robotProc, robotStat)

        self.stopBroadcaster(cbpaeRun)

        self.clearQueues(iRMsg, robotCmd, robotInfo)

        self.joinRobotProc(robotProc)

        self.logBasicInfo()

        print ("CBPAE Trial Finished!!!")

#    def safeRobotStop(self, robot, robotCmd, stopRecvd, mbGoal):
#        #===============================================================
#        # reading commands for safe exit from execution
#        #===============================================================
#        cmds = misc.recvMsg(robotCmd)
#        for cmd in (cmds):
#            if (cmd == 1):
#                stopRecvd = 1
#                break
#
#        if (stopRecvd == 1):
#            if (robot.taskProcess != None):
##                print ("got stop command at robotProcess")
#                robot.stopExecution()
#                # wait until the robot process is finished
#                while (True):
#                    time.sleep(0.1)
#                    if (not(robot.taskProcess.is_alive())):
#                        break
#                return (stopRecvd, True)
#            else:
#                return (stopRecvd, True)
#        else:
#            return (stopRecvd, False)

    def robotProcess(self, robot, iRMsg, bcMsg, robotInfo, robotCmd, robotStat, mbGoal=None, mbState=None):
        """"robot process: handles the process of a single robot"""
        # guiButton events
        startRecvd = 0
        stopRecvd = 0

        # for limiting the communication frequency
        waitBeats = 1.0 # time between two inter-robot beats message multicasts
        waitInit = 2.0 # wait time for initialisation
        waitHs = 5.0 # wait between hs message broadcasts

        # main loop vars
        loopCount = 0
        timePrevBeats = {} # time at which the previous beats from this robot was sent
        prevIdx = 0

        rIndex = robot.index
        print ("r[%d] starting the CBPAE" %(rIndex))

        # if gui is enabled, wait for the start command.
        (startRecvd, stopRecvd) = self.waitForStartButton(robotCmd, startRecvd, stopRecvd)

        # progress from here only if the robot is active
        self.checkRobotActive(robot, iRMsg)

        # update active status all tasks
        robot.checkActiveTasks()

        # Create the initial NL - neighbour list
        (neighbourList, tempBeats, prevBeats, stopRecvd, timeStart, timeNow, timePrevHs) = self.populateNeighbours(robot, iRMsg, bcMsg, robotCmd, waitInit, waitHs, stopRecvd)

        freeTasks = self.getInitFreeTasks(robot, neighbourList)

        while (freeTasks > 0):
            # update task active status in every loop
            robot.checkActiveTasks()

            # update NL - remove neighbours not sending beats from NL
            neighbourList = robot.updateNeighbourList(neighbourList, prevBeats)
            robot.nNeighbour = len(neighbourList)

            prevIdx = self.updatePoseGui(robot, robotInfo, prevIdx)

            time.sleep(.1)
            if (robot.active == 1):
#                (stopRecvd, breakTrue) = self.safeRobotStop(robot, robotCmd, stopRecvd, mbGoal)
                (stopRecvd, breakTrue) = self.safeRobotStop(robot, robotCmd, stopRecvd)
                if breakTrue:
                    break

                #===============================================================
                # ::communication phase::
                # sends messages to all robots in NL
                # the robot uses a lock for writing messages to queue for each robot
                # the robot reads the messages from its message queue
                #===============================================================
                # communicate-write
                (timeNow, timePrevBeats, timePrevHs) = self.comWrite(robot, iRMsg, bcMsg, neighbourList, waitBeats, waitHs, timePrevBeats, timePrevHs)

                # communicate-read
                (neighbourList, tempBeats, prevBeats) = self.comRead(robot, iRMsg, neighbourList, prevBeats)

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
                self.bidConsensus(robot, tempBeats)

                #===============================================================
                # ::task assignment phase::
                # allocateTasks():
                #    assigns tasks based on the msgCount and current task status
                #===============================================================
                if robot.rType == "ros":
                    robot.allocateTask(mbGoal, mbState)
                else:
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

            # if all tasks are finished, continue communication process for a few times - to retain network links
            (neighbourList, stopRecvd) = self.comAfterAllTasks(robot, iRMsg, robotCmd, bcMsg, freeTasks, stopRecvd, neighbourList, timePrevBeats, timePrevHs, waitInit, waitBeats, waitHs)

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
        self.logRobotData(robot, timeStart, timeStop, timeExec)

        self.finishRobotProc(robot, iRMsg, robotCmd, robotInfo, robotStat, neighbourList, loopCount)

        return

if __name__ == "__main__":
    rospy.init_node("cbpae_node")

    print (sys.argv)
    if len(sys.argv) <= 1:
        print ("usage: cbpae.py <path_to_data_file>")
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

        cbpae = RosCbpae(fName, logsDir)
        cbpae.run()
        if (LOGGING):
            cbpae.plotData(logsDir, plotsDir, fName)
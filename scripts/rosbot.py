#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on: Day Mon DD HH:MM:SS YYYY

@author: gpdas
"""
import time
import Queue
import copy
import multiprocessing
import task
import misc
import robot
import rospy
import move_base_msgs.msg
import nav_msgs.msg
import actionlib
import std_msgs.msg

class Robot(robot.Robot):
    """
    """
    def __init__(self, rIndex, xCord=0.0, yCord=0.0, 
                 heading=0.0, nTaskLimit=1,  rOn=0.0, 
                 rOff=float("inf"), ipAdd="localhost", portNum=6665, doTask=False):
        """
        """
        super(Robot, self).__init__(rIndex, xCord, yCord, 
                 heading, nTaskLimit,  rOn, 
                 rOff, ipAdd, portNum, doTask)
        self.ns = "robot_%02d" %(rIndex)
        self.mbClient = actionlib.SimpleActionClient(self.ns+"/move_base", move_base_msgs.msg.MoveBaseAction)
        self.mbClient.wait_for_server()

        self.mbFinished = False
        self.goalSent = False
        
        self.xPos = xCord
        self.yPos = yCord
        self.odomSub = rospy.Subscriber(self.ns+"/odometry/base_raw", nav_msgs.msg.Odometry, self.odomCB)
        self.pub = rospy.Publisher(self.ns+"/chatter", std_msgs.msg.String)

    def odomCB(self, msg):
        """
        """
        self.xPos = msg.pose.pose.position.x
        self.yPos = msg.pose.pose.position.y

    def checkTaskProgress(self):
        """checkTaskProgress
        Input arguments:
            None
        Returns:
            None
        Description:
            this method is called from the main bidding/consensus/execution loop
            to check the progress of execution by reading the messages from the taskNavigation process"""
        # if taskFinish = 1 or 2 ignore stage
        # if taskFinish = 0, check for stage
        #     if stage is changed, make necessary modification and
        #         send command 1 in stgQ
        xNew = yNew = stat = 0
        if (self.taskProcess == None):
            return -1
        else:
            if (self.taskProcess.is_alive()):
                stat = 0
                while (True):
                    try:
                        msgNew = self.msgQ.get_nowait()
                        xNew = msgNew[0]
                        yNew = msgNew[1]
                        stage = msgNew[2]
                        stat = msgNew[3]
                    except Queue.Empty:
                        break
                    else:
                        self.x.append(xNew)
                        self.y.append(yNew)

                        if (stage > self.stage):
                            print ("r[%d] stage:%d prevStage:%d"%(self.index, stage, self.stage))

                        # normal task completion
                        if (stat == 1):
                            break
                        # task stopped as per command
                        elif (stat == 2):
                            break
                        elif (stat == 0):
                            if (stage > self.stage):
                                print ("r[%d] got a stage change" %self.index)
                                timeNow = time.time()
                                self.taskEndTime[self.nTaskAllocated-1][0] = timeNow
                                self.taskStartTime[self.nTaskAllocated-1][1] = timeNow
                                # progress to next stage
                                self.stage = copy.copy(stage)
                                # send command 2: OK to progress to next level
                                misc.sendMsg(self.cmdQ, None, 2)
                                print ("robot:checkTaskProgress-1: r[%d] sent stage change ack" %self.index)

            else: # process finished. now reading all data from queue
                while (True):
                    try:
                        msgNew = self.msgQ.get_nowait()
                        xNew = msgNew[0]
                        yNew = msgNew[1]
                        stage = msgNew[2]
                        stat = msgNew[3]
                    except Queue.Empty:
                        break
                    else:
                        self.x.append(xNew)
                        self.y.append(yNew)

                        if (stage > self.stage):
                            print ("r[%d] stage:%d prevStage:%d"%(self.index, stage, self.stage))

                        # normal task completion
                        if (stat == 1):
                            break
                        # task stopped as per command
                        elif (stat == 2):
                            break
                        elif (stat == 0):
                            if (stage > self.stage):
                                # progressed to next stage
                                self.stage     = copy.copy(stage)
                                # send command 2: OK to progress to next level
                                misc.sendMsg(self.cmdQ, None, 2)
                                print ("robot:checkTaskProgress-2: r[%d] sent stage change ack" %self.index)

            if (stat == 1):
                self.taskProcess.join()
                self.completeTask(time.time())
#                print ("robot[%d] navigation to task[%d] completed: %d" %(self.index, self.taskBidOrder[self.nTaskAllocated-1],time.time()))
                return 1
            elif (stat == 2):
                self.taskProcess.join()
#                print ("robot[%d] navigation to task[%d] stopped as per command: %d" %(self.index, self.taskBidOrder[self.nTaskAllocated-1], time.time()))
                return 2
            else:
                return 0
            
    def startTaskProcess(self, tIndex):
        """startTaskProcess
        Input arguments:
            tIndex -> index of task being assigned to this robot
        Returns:
            None
        Description:
            start the task process with the required arguments"""
        # creating new process - this varies based on the controlling software used
        # arguments to be passed:
        #    selfLoc -> x[-1], y[-1]
        #     queues    -> msgQ, cmdQ
        #    task    -> taskList[tIndex]
        # starting coordinates -> self.x[-1], self.y[-1]
        self.taskExec[tIndex]  = -3
        self.nTaskAllocated += 1
        timeNow = time.time()
        self.taskStartTime[self.nTaskBid-1][0] = timeNow
        self.stage = 0
        # creating new process
        self.taskProcess = multiprocessing.Process(target=self.taskExecution, args=(self.msgQ, self.cmdQ, self.taskList[tIndex], self.x[-1], self.y[-1]))
        self.taskProcess.start()

    def taskExecution(self, msgQ, cmdQ, currTask, xPrev, yPrev):
        """taskExecution
        Input arguments:
            msgQ -> message queue back to the main robot process
            cmdQ -> command queue to task execution process from main robot process
            currTask -> the current task as object
            xPrev -> previous x coordinate
            yPrev -> previous y coordinate
        Returns:
            None
        Description:
            method to be run as parallel process for execution of a task"""
        tIndex = currTask.index
        # navigate to org location of task
        # stage 0
        tX = currTask.xOrg
        tY = currTask.yOrg
        stage = 0

        (xNew, yNew, taskStat) = self.taskNavigation(msgQ, cmdQ, xPrev, yPrev, tX, tY, stage)

        if (taskStat == 1):
            #     => navigation to task location successful
            #     => a task may not be dropped after this stage

            print ("r[%d] finished stage 0 of t[%d]" %(self.index, currTask.index))
            if (currTask.taskType == "delivery"):
                print ("r[%d] going to (%0.2f,%0.2f) for finishing" %(self.index, currTask.xFinish, currTask.yFinish))

            stage = 1
            # even if self.stage is set as 1, it will not be reflected in the other side (main robot process) as both are separate processes
            # send the status change back to the main process
            # msgQ - (xNew, yNew, stage, taskFinish)
            misc.sendMsg(msgQ, None, [xNew, yNew, stage, 0])
            print ("r[%d] sent the stage update" %self.index)

            # wait for stage change ack & stop command
            # cmdQ - command 2: acknowledge state change, 1- stop execution, 0 - start
            stageAck = 0
            while (True):
                cmds = misc.recvMsg(cmdQ)
                for cmd in (cmds):
                    if (cmd == 1):
                        print ("1 r[%d] received stop command" %self.index)
                        taskStat = 2
                        return 2
                    elif (cmd == 2):
                        print ("1 r[%d] received stage change ack" %self.index)
                        stageAck = 1
                    else:
                        pass
                if (stageAck == 1):
                    break

            if (taskStat != 2):
                # actual execution based on the task type
                if (currTask.taskType == "fall"): # fall detection
                    taskStat = self.fall(cmdQ, tIndex)

                elif (currTask.taskType == "delivery"): # food / drink / medicine delivery
                    (xNew, yNew, taskStat) = self.delivery(msgQ, cmdQ, xPrev, yPrev, tX, tY, tIndex)

                elif (currTask.taskType == "door"): # door opening
                    taskStat = self.door(cmdQ, tIndex)

                elif (currTask.taskType == "medicine"): # reminder to take medicine
                    taskStat = self.medicine(cmdQ, tIndex)

                elif (currTask.taskType == "clean"): # cleaning the floor area
                    taskStat = self.clean(cmdQ, tIndex)

                elif (currTask.taskType == "surveillance"): # surveillance
                    taskStat = self.surveillance(cmdQ, tIndex)

                else:
                    taskStat = self.mockTask(cmdQ, tIndex)

            # whether taskStat == 1 or 2, send the current msg with the taskStat
            misc.sendMsg(msgQ, None, [xNew, yNew, stage, taskStat])
            return taskStat

        elif (taskStat == 2):
            #     => task has been stopped by command
            #     return the status
            misc.sendMsg(msgQ, None, [xNew, yNew, stage, taskStat])
            return taskStat
        return

    def taskNavigation(self, msgQ, cmdQ, xPrev, yPrev, xFinish, yFinish, stage):
        """taskNavigation
        Input arguments:
            msgQ -> message queue back to the main robot process
            cmdQ -> command queue to the task process from the main robot process
            xPrev -> previous x coordinate of the robot
            yPrev -> previous y coordinate of the robot
            xFinish -> target x coodinate
            yFinish -> target y coordinate
            stage -> current execution stage, to send it back to the main robot process
        Returns:
            None
        Description:
            method to control the navigation of a robot to the target location"""
        # task is allocated and the robot has to travel to the task location
        # robot moves in the direction of the task from the current location
        # keep on track: regularly check the position and orientation with respect to the task location
        # obstacle avoidance: may have to move away from planned path to avoid an obstacle
        # stop when it reaches the task location
        # this is a new process. all methods accessed from this used variables from this function.
        taskZone = self.taskZone
        print ("r[%d] started navigation to (%0.2f, %0.2f) from (%0.2f, %0.2f)" %(self.index, xFinish, yFinish, xPrev, yPrev))

        taskFinish = 0
        com = 0

        prevMsg = None
        # wait for some time.
        time.sleep(0.1)

        xNew, yNew = xPrev+0, yPrev+0

        taskFinish = self.checkTaskFinish(xNew, yNew, xFinish, yFinish, taskZone) # check task finish status: 1-finished; 0-still running

        misc.sendMsg(msgQ, None, [xNew, yNew, stage, taskFinish])
        prevMsg = [] + [xNew, yNew, stage, taskFinish]

        # get the path to navigate
        xyPath = self.getPath(xNew, yNew, xFinish, yFinish)
        # (tX,tY) are modified as the coordinates of the nodes in the point in the following loop
        # for (tX, tY) in xyPath:
        while (len(xyPath) != 0):
            # xyPath can be modified with in the loop
            # always the first waypoint in the path is the next waypoint
            # when a way point is reached that is removed from the path
            tX = xyPath[0][0]
            tY = xyPath[0][1]

            currRoom = self.mapInfo.getRoom(xNew, yNew)
            nextRoom = self.mapInfo.getRoom(tX, tY)

            if (misc.eq(xFinish, tX) and misc.eq(yFinish, tY)):
                taskZone = self.taskZone #*1.5 # not to detect the task object as obstacle
            else:
                taskZone = self.taskZone

            if (taskFinish == 2):
                break

            self.goalSent = False
            self.mbFinished = False

            taskFinish = 0
            while (taskFinish == 0):     # when task not finished
                time.sleep(0.1)
#                rospy.loginfo ("%s here: tf- %d, gs - %s" %(self.index, taskFinish, self.goalSent))
                if not self.goalSent:
                    goal = move_base_msgs.msg.MoveBaseGoal()
                    goal.target_pose.header.frame_id = "map"
                    goal.target_pose.pose.position.x = tX
                    goal.target_pose.pose.position.y = tY
                    goal.target_pose.pose.orientation.w = 1
                    self.mbClient.send_goal(goal, done_cb=self.doneCB)
                    time.sleep(0.1)
                    self.goalSent = True

                xNew = self.xPos
                yNew = self.yPos

                if (prevMsg != [xNew, yNew, stage, taskFinish]):
                    # send current coordination back
                    misc.sendMsg(msgQ, None, [xNew, yNew, stage, taskFinish])
                    prevMsg = [] + [xNew, yNew, stage, taskFinish]

                taskFinish = self.checkTaskFinish(xNew, yNew, tX, tY, taskZone) # check task finish status
                if (taskFinish == 1):
                    xyPath.pop(0)
                    break

                # if a stop task command is received from the main process,
                # stop the current execution with a stat return of 2
                try:
                    com = cmdQ.get_nowait() # command from main process. 1-> stop current task
                except Queue.Empty:
                    pass
                else:
                    if (com == 1):
                        taskFinish = 2
                        break

                # check the room
                tempRoom = self.mapInfo.getRoom(xNew, yNew)
                if (tempRoom == None):
                    tempRoom     = -1
                if ((tempRoom != currRoom) and (tempRoom != nextRoom) and (tempRoom != -1)):
                    taskFinish = 3
                    break

            # replan the path here
            if (taskFinish == 3):
                print ("r[%d] changed xyPath"%(self.index))
#                print ("r[%d] was moving from room[%d] to room[%d]. but now room is %d" %(self.index, currRoom, nextRoom, tempRoom))
                xyPath = self.getPath(xNew, yNew, xFinish, yFinish)

        if (taskFinish == 1):
            #     => navigation to task location successful
            misc.sendMsg(msgQ, None, [xNew, yNew, stage, 0])
            return (xNew, yNew, taskFinish)

        elif (taskFinish == 2):
            #     => task has been stopped by command
            #     return the status
            self.mbClient.cancel_all_goals()
            misc.sendMsg(msgQ, None, [xPrev, yPrev, stage, 0])
            return (xNew, yNew, taskFinish)

        # add detailed controls here for longer tasks after reaching the task location.
        print ("Error!!!, taskFinish = ", taskFinish)
        exit()
        return 0

    def doneCB(self, status, result):
        """done callback
        """
        status = status
        result = result
        self.mbFinished = True
        self.goalSent = False

    def stopExecution(self):
        if (self.taskProcess != None):
            if (self.taskProcess.is_alive()):
                misc.sendMsg(self.cmdQ, None, 1)
                while (True):
                    if not(self.taskProcess.is_alive()):
                        break

    def checkTaskFinish(self, rX, rY, tX, tY, taskZone):
        """ checks whether the robot has reached E-near the task location"""
        if self.goalSent and self.mbFinished:
            return 1
        else:
            return 0
#        
#        dist = misc.getDist(rX, rY, tX, tY)
#        if (misc.lt(dist, taskZone)):
#        #if (dist < self.taskZone):
#            return 1
#        else:
#            return 0

    def goToOrg(self):
        """similar to allocateTask this method creates a child process to go
        to the original location after completion of all tasks"""
        self.goTo(self.xOrg, self.yOrg)

    def goTo(self, x, y):
        """similar to allocateTask this method creates a child process to go
        to the given location"""
        dummyTask = task.Task(-1, x, y, x, y)
        dummyTask.stages = 1
        self.taskProcess = multiprocessing.Process(target=self.taskExecution, args=(self.msgQ, self.cmdQ, dummyTask, self.x[-1], self.y[-1]))
        self.taskProcess.start()
        print ("robot[%d] started navigation to (%0.2f, %0.2f)" %(self.index, x, y))

    def delivery(self, msgQ, cmdQ, xPrev, yPrev, tX, tY, tIndex):
        stage = 1
        xNew = xPrev + 0
        yNew = yPrev + 0
        currTask = self.taskList[tIndex]
        # pick the object
        taskStat = self.mockTask(cmdQ, tIndex)

        if (taskStat != 2):
            # move to target location
            if (not misc.eq(currTask.xOrg, currTask.xFinish) or not misc.eq(currTask.yOrg, currTask.yFinish)):
                tX = currTask.xFinish
                tY = currTask.yFinish
                (xNew, yNew, taskStat) = self.taskNavigation(msgQ, cmdQ, xPrev, yPrev, tX, tY, stage)

        if (taskStat != 2):
            # drop the object
            taskStat = self.mockTask(cmdQ, tIndex)

        return (xNew, yNew, taskStat)

    def fall(self, cmdQ, tIndex):
        return self.mockTask(cmdQ, tIndex)

    def door(self, cmdQ, tIndex):
        return self.mockTask(cmdQ, tIndex)

    def surveillance(self, cmdQ, tIndex):
        return self.mockTask(cmdQ, tIndex)

    def medicine(self, cmdQ, tIndex):
        return self.mockTask(cmdQ, tIndex)

    def clean(self, cmdQ, tIndex):
        return self.mockTask(cmdQ, tIndex)

    def mockTask(self, cmdQ, tIndex):
        currTask = self.taskList[tIndex]
        taskStat = 1
        timeNow = time.time()
        while (misc.lte((time.time() - timeNow), currTask.timeMax)):
            time.sleep(0.1)
            try:
                cmd = cmdQ.get_nowait()
            except Queue.Empty:
                pass
            else:
                if (cmd == 1):
                    taskStat = 2
                    break
                else:
                    pass
        return taskStat
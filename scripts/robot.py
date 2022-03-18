## ---------------------------------------------------- ##
# file: robot.py
# desc: single robot controls of a pioneer using simple distance calculations using constant velocity and time.
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

import copy
import time
import misc
import math
import multiprocessing
import queue
import task

class Robot(object):
    """ Description of the robot class """
    def __init__(self, rIndex, xCord=0.0, yCord=0.0, heading=0.0, nTaskLimit=1,  rOn=0.0, rOff=float("inf"), ipAdd="localhost", portNum=6665, doTask=False):
        """ Robot class constructor
        Input arguments:
            xCord -> x coordinate of the robot, float
            yCord -> y coordinate of the robot, float
            heading -> initial heading of the robot, float
            nTaskLimit -> maximum number of tasks the robot can be allocated, int
            rOn -> robot on time, float
            rOff -> robot off time, float
            ipAdd -> robot hostname, string
            portNum -> port number, int
            doTask -> do actions corresponding to the task / spent corresponding time, boolean
        Returns:
            None
        Description:
            General initialization of attributes.
        """
        self.index = rIndex
        self.xOrg = xCord
        self.yOrg = yCord

        self.rType = "test"
        self.nTaskLimit = nTaskLimit
        self.vel = 0.2
        self.timeOn = rOn
        self.timeOff = rOff
        self.timeInit = 0 # program starting time
        self.active = 0

        self.ipAdd = copy.copy(ipAdd)
        self.portNum = copy.copy(portNum)

        self.bidUpdateThreshold = 0.1
        self.rClient = "dummy variable"
        self.taskProcess = None

        self.x = []
        self.y = []
        self.x.append(xCord)
        self.y.append(yCord)

        self.doTask = doTask # a flag to set or reset to do the actual execution of the task
        self.maxDist = 0.0

#        self.expertise = {"navigation":0.8, "vision":0.8, "gripper":0.8, "manipulator":0.0, "audition":0.0}
#        self.skills = ["navigation", "vision", "gripper", "manipulator", "audition"]
        self.skills = []
        self.expertise = {}

        ## taskList contains all the tasks
        ## when a robot finishes a task, the taskIndex will be passed to other connected robots and
        ##    the completed task will be given a completed status.
        ##     nTask will not be updated. but uses the completed status to identify the completed tasks
        ##    task value of completed task will be given a completed status in taskBidVal
        ##    task biddable status will be given a completed status.
        ##    taskAlloc
        self.taskIds = []
        self.taskList = {} # local copy of global task list
        self.nTask = 0 # number of tasks (globally)
        self.nTaskAllocated = 0 # number of tasks already allcoated. will be updated when a atask is bid or updated or removed
        self.taskBidVal = {} # local list of winning bids from all robots
        self.taskBidTime = {} # local list of time at which the bid was placed on the task
        self.taskDropTime = {} # local list of time at which execution of the task is dropped. bid time alone is not sufficient to break the conflict
        self.taskFinTime = {} # local list of task finish times / time at which the finish status of the task is received. need not be accurate. (used internally only)
        self.taskBiddable = {} # local list of task biddablilty by this robot
        self.taskExecutable = {} # local list of task execution capability of this robot
        self.taskVal = {} # local task value/bid value for this robot
        self.taskAlloc  = {} # local info abt who was allocated each task, -1 => free, others => robotIndex
        self.taskExec = {} # task execution status. -2=> completed, -1 => free, -3 => busy, already allocated, -4 => task dropped
        self.taskBidOrder = [] # task bidding order
        self.taskReward = [] # task value for no tasks in the taskBidOrder / taskExecorder

        self.taskStartTime = [] # task starting time
        self.taskEndTime = [] # task ending time

        self.nRobot = 0 # number of robots. this is required to set the size of the rLoc list
        self.nNeighbour = 0 #  number of neighbour robots
        self.robotLoc = [] # list of global coordinates (x,y) of all robots. len = nRobot
                           # this info is used to identify whether an obstacle is another robot.
                           # if the obstacle is a robot, appropriate obstacle avoidance action needs
                           # to be taken based on the priority of the task the other robot is currently executing.
        self.robotLocTime = [] # timestamp at which the location info is sent.
                               # when 'a' gets coord info of 'b' from 'b', 'a' timestamps the rLoc data
                               # when 'a' gets  coord info of 'b' from 'c', 'a' updates the info based
                               # on the local timestamp of rLoc info of 'b'
                               # this time stamp may be used as a heartbeat signal. the last time someone heard something from 'b'

        self.nTaskBid = 0 # number of tasks bid, max value = nTaskAllocated + 1
        self.msgCount = 0 # a count of messages processed without change in the bid

        self.stage = 0 # execution stage of the current task if any
                       #     stage 0: moving to the task org location
                       #     stage 1: actual execution of the task
        self.mapInfo = None
        self.zone = {0:0.8, 1:1.0, 2:1.2}
        self.initialBeats = 0.0     # first copy of beats received status. 0-> not received. this is used as a flag for requesting beats with hsack while sending hs

#        self.taskZone            = 1.5*self.zone[2] # closeness to the task location where in which the task is considered to be executed
        # closeness to the task location where in which the task is considered to be executed
        if (self.rType == "test"):
            self.taskZone = 0.1
        else:
            self.taskZone = self.zone[0]

        # queues for inter process communication (robot process and execution process).
        # execution process handles the movement of the robot to the task location
        self.msgQ = multiprocessing.Queue() # Queue connection for transferring messages as tuple (x,y,stat)
        self.cmdQ = multiprocessing.Queue() # Queue connection for transferring stop command from main process to task execution process

    def setMapData(self, mapInfo):
        """setMapData
        Input arguments:
            mapInfo -> mapInfo from the input file, mapInfo
        Returns:
            None
        Description:
            sets the robot map data attribute to the data read from the input file
        """
        self.mapInfo = copy.deepcopy(mapInfo)
        self.maxDist = 4 * self.mapInfo.maxDist

    def setExpertise(self, skills=[], expertise=[]):
        """setExpertise
        Input arguments:
            skills -> list of skills, list of string
            expertise -> expertise value corresponding to the skills, list of floats
        Returns:
            None
        Description:
            sets the robot's skills and expertise attributes using the data read from the input file. The skills are
                navigation    : a function of the speed of the robot and its possible movements
                vision        : a function of the resolution and other features such as auto-focus, if applicable
                audition    :
                gripper        :
                manipulator    : a function of DOF, approachable range
                cleaner        :
        """
        # navigation=0.8, vision=0.8, gripper=0.8, manipulator=0.0, audition=0.0
        for idx in (range (len(skills))):
            if (misc.gt(expertise[idx], 0.0)):
                self.expertise[skills[idx]] = expertise[idx]
                self.skills.append(skills[idx])

    def setInitTime(self, timeInit):
        """setInitTime
        Input arguments:
            timeInit -> robot initialisation time
        Returns:
            None
        Description:
            sets the robot's and its tasks' initTime. called from cbpae
        """
        self.timeInit = copy.copy(timeInit)
        for tId in (self.taskIds):
            self.taskList[tId].timeInit = copy.copy(timeInit)

    def initTaskVectors(self):
        """initTaskVectors:
        Input arguments:
            None
        Returns:
            None
        Description:
            intialises all task attributes of the robot with default values.
            taskBid, taskBidVal, taskBiddable, taskVal, taskValOrder, taskAlloc are resized with default values
        """
        for tId in (self.taskIds):
            self.taskBidVal[tId] = float("inf") # initialize with inf => no approved bids from any robots
            self.taskBidTime[tId] = -1.0 # initialize with -1 => no approved bids from any robots; biding time otherwise
            self.taskDropTime[tId] = -1.0 # initialize with -1 => no execution drop; execution drop-time otherwise
            self.taskBiddable[tId] = 0 # initialize with 1 => biddable, will be updated before considering for bidding
            self.taskExecutable[tId] = 1 # initialize with 1 => executable, will be updated only at the start and when new tasks are introduced.
            self.taskVal[tId] = float("inf") # initialize with inf => no initial value
            self.taskAlloc[tId] = -1 # initialize with -1. not allocated to any robot
            self.taskExec[tId] = -1 # initialize with -1. not allocated to any robot

        self.taskBidOrder = [-1 for tC in (range (self.nTaskLimit))] # task bidding order, default = -1 => nothing in that position
        self.taskReward = [-1.0 for tC in (range (self.nTaskLimit))] # task value for number of  tasks in the taskBidOrder / taskExecorder
        self.taskStartTime = [[-1, -1] for tId in (range (self.nTaskLimit))] # task starting time
        self.taskEndTime = [[-1, -1] for tId in (range (self.nTaskLimit))] # task ending time

        self.robotLoc = [[0.0, 0.0, 0.0] for rC in (range (self.nRobot))] # Location of all robots
        self.robotLocTime = [0.0 for rC in (range (self.nRobot))] # Location info  update time of all robots

    def setRobotList(self, nRobot):
        """setRobotList
        Input arguments:
            nRobot -> number of robots, int
        Returns:
            None
        Description:
            sets the number of robots.
            not currently used.
        """
        self.nRobot    = copy.copy(nRobot)

    def setTaskList(self, taskIds, taskList):
        """setTaskList:
        Input arguments:
            taskIds -> list,
            taskList -> dict(tid: taskObject)
        Returns:
            None
        Description:
            Sets the list of available tasks to the robot.
            Calls addTask to accommodate each task in the taskList
        """
        self.taskIds = [] + taskIds
        self.taskList = copy.deepcopy(taskList)
        self.nTask = len(self.taskIds)
        self.initTaskVectors()
        self.updateTaskExecutable()

    def updateTaskExecutable(self):
        """updateTaskExecutable
        Input arguments:
            None
        Returns:
            None
        Description:
            Checks which all tasks are executable based on the skills required and the robot's skills
        """
        for tId in (self.taskIds):
            if (self.taskExec[tId] == -2):
                self.taskExecutable[tId]     = 0             # finished task
            else:
                skillsReq = self.taskList[tId].skillsReq     # skills required for the task
                for skill in (skillsReq):
                    if (skill not in self.expertise.keys()):
#                        print ("r[%d] doesn't have the skills for t[%d]" %(self.index, tId))
                        self.taskExecutable[tId]     = 0     # robot doesn't have the required skill
                        break

    def updateRobotLoc(self, nbrId, nbrRobotLoc, nbrRobotLocTime):
        """updateRobotLoc
        Input arguments:
            nbrId -> neighbour Robot's id, int
            nbrRobotLoc -> neighbour's location, [x, y]
            nbrRobotLocTime -> time at which the location is obtained
        Returns:
            None
        Description:
            Updates the rLoc info all all robots based on the local copy and the neighbours info"""
        # set the value of neighbour
        self.robotLoc[nbrId] = copy.deepcopy(nbrRobotLoc[nbrId])
        self.robotLocTime[nbrId] = copy.deepcopy(nbrRobotLocTime[nbrId])

        self.robotLoc[self.index] = copy.deepcopy([self.x[-1], self.y[-1]])
        self.robotLocTime[self.index] = time.time() - self.timeInit

        for i in (range (self.nRobot)):
            if ((i != self.index) and (i != nbrId)):
                if (misc.gt(nbrRobotLocTime[i], self.robotLocTime[i])):
                    self.robotLoc[i] = copy.deepcopy(nbrRobotLoc[i])
                    self.robotLocTime[i] = copy.deepcopy(nbrRobotLocTime[i])
        return

    def calcVal(self):
        """calcVal
        Input arguments:
            None
        Returns:
            None
        Description:
            Calculates task value for each task considering the task will be executed after the current task (if any) and updates in taskVal.
        """
        ## Find number of tasks already allocated to this robot.
        ## The tasks already in the taskExecOrder are counted
        ## get the taskReward only for those robots which are not reached nTaskLimit
        if (self.nTaskAllocated == 0):
            currTaskIdx = None
        else:
            currTaskIdx     = self.taskBidOrder[self.nTaskAllocated-1]

        # if currTaskIdx != None: find the remCurrBid

        currTaskSkills = []
        currTaskEffort = {}
        if (currTaskIdx != None) and (self.taskExec[currTaskIdx] == -3): # there is a task and it is being executed
            currTaskSkills = self.taskList[currTaskIdx].skillsReq
            currTaskEffort = self.getEffort(currTaskSkills, currTaskIdx) # gives the scaled efforts of all required skills as dict(skill:effort)
            for skill in (currTaskSkills):
                if (currTaskEffort[skill] == float("inf")):
#                    print ("Error: r[%d] effort[%s]=%0.3f"%(self.index, skill, currTaskEffort[skill]))
                    raise("")

        # for all other executable tasks find absNextTask
        if (self.nTaskBid <= self.nTaskLimit):
            for tId in (self.taskIds):
                if ((currTaskIdx == None) or
                    ((currTaskIdx != None) and (currTaskIdx != tId))):
                    if (self.taskExecutable[tId] == 1): # the robot has all the skills
                        # if the task is active
                        if (self.taskList[tId].active == 1):
                            if (self.taskExec[tId] == -2): # the task execution is finished
                                self.taskVal[tId] = float("inf")
                            elif (self.taskExec[tId] == -3): # task is being executed by some other robots
                                self.taskVal[tId] = float("inf")
                            else:
                                nextTaskSkills = self.taskList[tId].skillsReq
                                nextTaskEffort = self.getEffort(nextTaskSkills, tId) # gives the scaled efforts of all required skills as dict(skill:effort)
                                self.taskVal[tId] = 0.0
                                taskVal = 0.0

                                for skill in (nextTaskSkills):
                                    if (nextTaskEffort[skill] == float("inf")):
                                        taskVal = float("inf")
                                        break

                                # find the intersection of currTaskSkill and nextTaskSkill
                                # a = find the bid components for the skills common in currTaskSkill and nextTaskSkill
                                # b = find the bid components of skill differences (in currTaskSkill and not in nextTaskSkill)
                                # c = find the bid components of skill differences (in nextTaskSkill and not in currTaskSkill)
                                # m = cardinality of union of skills (currTaskSkill U nextTaskSkill)
                                # bid = (1/m)(a+b+c)
                                commonSkills = []
                                currNotNext = []
                                nextNotCurr = []
                                for skill in (currTaskSkills):
                                    if skill in nextTaskEffort.keys():
                                        commonSkills.append(skill)
                                    else:
                                        currNotNext.append(skill)
                                for skill in (nextTaskSkills):
                                    if skill not in currTaskEffort.keys():
                                        nextNotCurr.append(skill)

                                for skill in (commonSkills):
                                    taskVal += (currTaskEffort[skill] + nextTaskEffort[skill]) / self.expertise[skill]
                                for skill in (currNotNext):
                                    taskVal += currTaskEffort[skill] / self.expertise[skill]
                                for skill in (nextNotCurr):
                                    taskVal += nextTaskEffort[skill] / self.expertise[skill]

                                self.taskVal[tId] = taskVal
                                # taking average wrt to the number of skills creates problem.
                                # assume one robot has a bid 40 with only navigation => avg = 40
                                # another has bid 70 for navigation (curr+next) and 5 for vision (curr) => avg = 37.5

                                #TODO: check the effort reduction as the execution progresses, properly

                    else: # if a task is not executable, set the value as inf
                        self.taskVal[tId] = float("inf")

    def updateBiddable(self):
        """updateBiddable
        Input arguments:
            None
        Returns:
            None
        Description:
            Sets the task as biddable if
                The task is not bid already
                The tasks current taskBidVal < taskVal
            Resets for all other cases
        """
        # self.updateBattery()
        # check the possible distance that can be traveled with in timeOff
        timeNow = time.time()
        timeRem = self.timeOff - timeNow - 2*60.0 # extra 2 minutes margin
        if (misc.lte(timeRem, 0.0)):
            timeRem = 0.0

        for tId in (self.taskIds):
            if (self.taskList[tId].active == 1):
                if (self.taskExecutable[tId] == 0):
                    self.taskBiddable[tId] = 0
                elif ((self.taskExec[tId] == -1) or (self.taskExec[tId] == -4)):
                    if (self.taskVal[tId] == float("inf")):
                        self.taskBiddable[tId] = 0
                    else:
#                        if (misc.lt(self.taskVal[tId], 0.0)):
#                            print (">>>>>>>>>\nNegative BidValue\n<<<<<<<")

                        if (misc.lt(self.taskVal[tId], self.taskBidVal[tId])):
                            self.taskBiddable[tId] = 1
                        elif ((self.nTaskBid > 0) and
                            (self.nTaskBid > self.nTaskAllocated) and
                            (tId == self.taskBidOrder[self.nTaskBid-1])):
                            self.taskBiddable[tId] = 1 # the currently bid task (not assigned yet) is also set as biddable so that the value is updated eventhough the bid becomes lower because of obstacle
                        else:
                            self.taskBiddable[tId] = 0
                else:
                    self.taskBiddable[tId] = 0

    def getMaxValTask(self):
        """getMaxValTask
        Input arguments:
            None
        Returns:
            maxValTaskIndex -> the index of biddable task, giving maximum taskVal, int
            -1 -> If there are no biddable tasks
        Description:
            Returns taskIndex of the task with maximum taskVal  from the biddable tasks
        """
        self.calcVal() # calculate value of all tasks and sets in
        self.updateBiddable() # find the biddable tasks

        biddableMaxVal = float("inf")
        biddableMaxIndex = -1
        biddableMaxStcPrio = 6
        if (sum(self.taskBiddable) > 0):             # there is at least one biddable task
            for tId in (self.taskIds):
                if (self.taskList[tId].active == 1):
                    if (self.taskBiddable[tId] == 1):
                        # task is biddable and of higher priority than the biddableMaxIndex task
                        if (biddableMaxStcPrio > self.taskList[tId].stcPrio):
                            biddableMaxVal = self.taskBiddable[tId] * self.taskVal[tId]
                            biddableMaxIndex = copy.copy(tId)
                            biddableMaxStcPrio = copy.copy(self.taskList[tId].stcPrio)
                        # task is biddable and of equal priority as the biddableMaxIndex task, but with a higher bid value.
                        elif ((misc.lt(self.taskVal[tId], biddableMaxVal)) and (biddableMaxStcPrio == self.taskList[tId].stcPrio)):
                            biddableMaxVal = self.taskBiddable[tId] * self.taskVal[tId]
                            biddableMaxIndex = copy.copy(tId)
                            biddableMaxStcPrio = copy.copy(self.taskList[tId].stcPrio)
                        # if the currently bid task has a new value (lower bid bcz of obstacle covering distance) use this
                        elif ((self.nTaskBid > 0) and (self.nTaskBid > self.nTaskAllocated) and (tId == self.taskBidOrder[self.nTaskBid-1])):
                            if (misc.gt(abs(self.taskVal[tId] - self.taskBidVal[tId]), self.bidUpdateThreshold) and
                                ((biddableMaxIndex     == -1) or
                                    (biddableMaxStcPrio > self.taskList[tId].stcPrio) or
                                    ((biddableMaxStcPrio == self.taskList[tId].stcPrio) and (misc.lt(self.taskVal[tId], biddableMaxVal))))):
                                # if the biddableMaxIndex is not the currently bid task, that task will be released.
                                biddableMaxVal = self.taskBiddable[tId] * self.taskVal[tId]
                                biddableMaxIndex = copy.copy(tId)
                                biddableMaxStcPrio = copy.copy(self.taskList[tId].stcPrio)
            return biddableMaxIndex
        else:
            return -1

    def bidTask(self, tIndex):
        """bidTask
        Input arguments:
            taskIndex -> index of task to be acquired, int
        Returns:
            None
        Description:
            The task with index taskIndex is acquired to be executed after finding maxValTask
                taskBidVal  is set as taskVal, the maxReward for task
                taskBidOrder is augmented with taskIndex
                taskAlloc[taskIndex] is set as self index
                taskReward is augmented with cumulative taskReward through the taskExecOrder
        """
        self.taskBidVal[tIndex] = self.taskVal[tIndex]
        self.taskBidTime[tIndex] = time.time() - self.timeInit
        self.taskAlloc[tIndex] = self.index

        if (self.nTaskBid == 0):
            self.taskReward[self.nTaskBid] = self.taskVal[tIndex]
        else:
            self.taskReward[self.nTaskBid] = self.taskReward[self.nTaskBid - 1] + self.taskVal[tIndex]

        self.taskBidOrder[self.nTaskBid] = tIndex
        self.nTaskBid += 1

    def createBundle(self):
        """createBundle
        Input arguments:
            None
        Returns:
            None
        Description: Bundle is created (if there is no existing bid)
                The maxValTask has been identified by calling getMaxValTask()
                The maxValTask is bid by calling bidTask(taskIndex)
        """
        # if there are no bids ever placed or
        # (there is no current extra bid (nBid=nAlloc) and nTaskLimit has not reached)
        if ((self.nTaskBid == 0) or
            ((self.nTaskBid > 0) and (self.nTaskBid == self.nTaskAllocated) and
                (self.nTaskBid < self.nTaskLimit))):
            # find the task with max value and having a better value than current bid
            maxValTaskIndex = self.getMaxValTask()
            if (maxValTaskIndex != -1):
#                print ("robot[%d]: task[%d]" %(self.index, maxValTaskIndex))
                self.bidTask(maxValTaskIndex)

    def updateBundle(self):
        """updateBundle
        Input arguments:
            None
        Returns:
            None
        Description: updates current bundle
            if there is bidtask, get the maxValueTask.
            if it is the same as the current bid task update the bid value, else remove the bid and place a new bid.
            if there are no bidtask, remove the existing bid
        """
        if (self.nTaskBid > self.nTaskAllocated):
            tIndex = self.taskBidOrder[self.nTaskBid-1]
            # find the task with max value and having a better value than current bid
            maxValTaskIndex = self.getMaxValTask()
            # if there is a task for which this robot has the the maximum bid value, check it is the same as bid task
            if (maxValTaskIndex != -1):
                # the robot has the highest bid for another task other than the current one, release current bid and place a new bid
                if (maxValTaskIndex != tIndex):
#                    print ("r[%d] releasing t[%d] as current high bid is for t[%d]" %(self.index, tIndex, maxValTaskIndex))
                    self.releaseTask(tIndex)
                    self.msgCount = 0
                    self.createBundle()
                else: # maxValTaskIndex == tIndex
                    if (misc.gt(abs(self.taskBidVal[tIndex] - self.taskVal[tIndex]), self.bidUpdateThreshold)):
#                        print ("r[%d] updating bid on t[%d] from %0.3f to %0.3f" %(self.index, tIndex, self.taskBidVal[tIndex], self.taskVal[tIndex]))
                        self.taskReward[tIndex] = self.taskReward[tIndex] - self.taskBidVal[tIndex] + self.taskVal[tIndex]
                        self.taskBidVal[tIndex] = self.taskVal[tIndex]
                        self.taskBidTime[tIndex] = time.time() - self.timeInit
            else:
#                print ("r[%d] releasing t[%d] as there are no bidable tasks" %(self.index, tIndex))
                self.releaseTask(tIndex)
                self.msgCount = 0

    def releaseTask(self, tIndex):
        """releaseTask
        Input arguments:
            taskIndex -> index of task to be released, int
        Returns:
            None
        Description:
            The task with taskIndex is removed from the BidOrder / bid on the task is removed
            taskVal, taskValOrder, taskBid, taskBiddable, taskAlloc and taskBidVal of the task set to default
        """
        # reset: taskVal, taskValOrder, taskBid, taskBiddable
        self.taskVal[tIndex] = float("inf")
        self.taskBidVal[tIndex] = float("inf")
        self.taskBidTime[tIndex] = time.time() - self.timeInit # if the task is being released as some other robot has a better bid or some other robot started executing the task, that bidtime will be replaced after calling releaseTask()

        self.taskBiddable[tIndex] = 0
        self.taskAlloc[tIndex] = -1

        self.taskBidOrder[self.nTaskBid-1] = -1
        self.taskReward[self.nTaskBid-1] = float("inf")

        self.nTaskBid -= 1
        self.msgCount = 0
        print ("robot[%d] lost bid on task[%d]" %(self.index, tIndex))

    def dropTask(self, tIndex):
        """dropTask
        Input arguments:
            taskIndex -> index of task to be released, int
        Returns:
            None
        Description: task is currently being executed by the robot, but it has to be dropped from execution
            any bid placed after this is removed as that bid is calculated based on this task
            The task with taskIndex is removed from the BidOrder
            taskVal, taskValOrder, taskBid, taskBiddable, taskAlloc and taskBidVal of the task set to default
        """
        # reset: taskVal, taskValOrder, taskBid, taskBiddable
        self.taskVal[tIndex] = float("inf")
        self.taskBidVal[tIndex] = float("inf")
        self.taskBidTime[tIndex] = time.time() - self.timeInit
        self.taskBiddable[tIndex] = 0
        self.taskAlloc[tIndex] = -1

        self.taskExec[tIndex] = -4
        self.taskDropTime[tIndex] = time.time() - self.timeInit

        self.taskBidOrder[self.nTaskBid-1] = -1
        self.taskReward[self.nTaskBid-1] = float("inf")

        self.nTaskBid -= 1
        self.nTaskAllocated -= 1

        self.msgCount = 0
        self.stage = 0
        print ("robot[%d] dropping execution of task[%d]" %(self.index, tIndex))

    def updateTask(self, tIndex, taskBidVal, taskBidTime, taskDropTime, taskAlloc, taskExec):
        """updateTask
        Input arguments:
            tIndex -> index of the task to be modified
            taskBidVal -> bid value of the task from the neighbour
            taskBidTime -> bid time of the task from the neighbour
            taskDropTim -> drop time of the task from the neighbour
            taskAlloc -> id of the robot to which the task is allocated, as per neighbour
            taskExec -> execution status of the task, as per the neighbour
        Returns:
            None
        Description:
            sets the different task vectors correspinding to the t[tIndex] with the values from the neighbour
        """
        self.taskBidVal[tIndex] = taskBidVal
        self.taskBidTime[tIndex] = taskBidTime
        self.taskDropTime[tIndex] = taskDropTime
        self.taskAlloc[tIndex] = taskAlloc
        self.taskExec[tIndex] = taskExec

    def updateTaskBidTime(self, tIndex):
        """updateTaskBidTime
        Input arguments:
            tIdex -> index of the task
        Returns:
            None
        Description:
            update the bid time of the task to the current time
        """
        self.taskBidTime[tIndex] = time.time() - self.timeInit

    def updateTaskTime(self, tIndex, nbrTaskDropTime):
        """updateTaskTime
        Input arguments:
            tIndex -> index of the task
            nbrTaskDropTime -> drop time of the task from neighbour
        Returns:
            None
        Description:
            update the task's drop time with the value from the neighbour
            update the task's bidtime to current time
        """
        self.taskBidTime[tIndex] = time.time() - self.timeInit
        self.taskDropTime[tIndex] = nbrTaskDropTime

    def checkActiveTasks(self):
        """checkActiveTasks
        Input arguments:
            None
        Returns:
            None
        Description:
            check the active status of all tasks"""
        for tId in (self.taskIds):
            if (self.taskList[tId].active == 0):
                self.taskList[tId].checkStatus()

    def consensusUnbidTask(self, nbrId, nbrTaskIds, nbrTaskBidVal, nbrTaskExec, nbrTaskAlloc, nbrTaskBidTime, nbrTaskDropTime, bidUpdate):
        """consensusUnbidTask
        Input arguments:
            nbrId -> id of the neighbour robot, int
            nbrTaskIds -> list of task ids from the neighbour in the beats, list
            nbrTaskBidVal -> dict of bidvalues of the tasks from the neighbour, corresponding to nbrIds
            nbrTaskExec -> dict of execution status of the tasks from the neighbour, corresponding to nbrIds
            nbrTaskAlloc -> dict of allocated robots of the tasks from the neighbour, corresponding to nbrIds
            nbrTaskBidTime -> dict of bid times of the tasks from the neighbour, corresponding to nbrIds
            nbrTaskDropTime -> dict of drop time of tasks from the neighbour, corresponding to nbrIds
            bidUpdate -> bidUpdates
        Returns:
            bidUpdate -> if there is a bidUpdate, that many changes are added to the counter passed in
        Description:
            consensus process for all tasks other than the one bid for by the robot"""
        # nbr, taskIds, taskBid, taskExec, taskAlloc, taskBidTime, taskDropTime
        timeNow = time.time()
        tIndex = [-1, -1]
        if (self.nTaskBid > 0): # don't check for the currently bid task and currevtly allocated task
            tIndex[0] = self.taskBidOrder[self.nTaskBid-1]
        if (self.nTaskAllocated > 0): # don't check for currently executing task
            tIndex[1] = self.taskBidOrder[self.nTaskAllocated-1]

        for tId in (nbrTaskIds):
            if (self.taskList[tId].active == 1):
                if (tId != tIndex[0]) and (tId != tIndex[1]): # don't check for curently executing and bid tasks
                    if (nbrTaskExec[tId] == -4):
                        if (self.taskExec[tId] == -4):
                            if (misc.gt(nbrTaskDropTime[tId], self.taskDropTime[tId])):
                                # if drop time is newer in neighbour, update
                                bidUpdate += 1
                                self.updateTask(tId, nbrTaskBidVal[tId], nbrTaskBidTime[tId], nbrTaskDropTime[tId], nbrTaskAlloc[tId], nbrTaskExec[tId])
                            elif (misc.eq(nbrTaskDropTime[tId], self.taskDropTime[tId])):
                                # drop times are equal. now the chances are for changes in bids
                                if ((nbrTaskAlloc[tId] != -1) and
                                    (nbrTaskAlloc[tId] != self.index)):
                                    if (misc.gt(nbrTaskDropTime[tId], self.taskDropTime[tId])):
                                        self.updateTask(tId, nbrTaskBidVal[tId], nbrTaskBidTime[tId], nbrTaskDropTime[tId], nbrTaskAlloc[tId], nbrTaskExec[tId])
                                        bidUpdate += 1
                                    elif (misc.eq(nbrTaskDropTime[tId], self.taskDropTime[tId]) and
                                        misc.gt(nbrTaskBidTime[tId], self.taskBidTime[tId])):
                                        self.updateTask(tId, nbrTaskBidVal[tId], nbrTaskBidTime[tId], nbrTaskDropTime[tId], nbrTaskAlloc[tId], nbrTaskExec[tId])
                                        bidUpdate += 1
                                elif (nbrTaskAlloc[tId] != -1):
                                    if (misc.gt(nbrTaskDropTime[tId], self.taskDropTime[tId])):
                                        self.updateTask(tId, nbrTaskBidVal[tId], nbrTaskBidTime[tId], nbrTaskDropTime[tId], nbrTaskAlloc[tId], nbrTaskExec[tId])
                                        bidUpdate += 1
                        elif (self.taskExec[tId] == -3):
                            if (misc.gte(nbrTaskDropTime[tId], self.taskDropTime[tId])):
                                bidUpdate += 1
                                self.updateTask(tId, nbrTaskBidVal[tId], nbrTaskBidTime[tId], nbrTaskDropTime[tId], nbrTaskAlloc[tId], nbrTaskExec[tId])
                            else:
                                pass
                        elif (self.taskExec[tId] == -2):
                            # task has already been finished. neighbour has older info
                            pass
                        elif (self.taskExec[tId] == -1):
                            # DROP is newer info than NALC
                            bidUpdate += 1
                            self.updateTask(tId, nbrTaskBidVal[tId], nbrTaskBidTime[tId], nbrTaskDropTime[tId], nbrTaskAlloc[tId], nbrTaskExec[tId])

                    elif (nbrTaskExec[tId] == -3):
                        if (self.taskExec[tId] == -4):
                            if (misc.gte(nbrTaskDropTime[tId], self.taskDropTime[tId])):
                                # newer bid and allocation
                                bidUpdate += 1
                                self.updateTask(tId, nbrTaskBidVal[tId], nbrTaskBidTime[tId], nbrTaskDropTime[tId], nbrTaskAlloc[tId], nbrTaskExec[tId])
                            else:
                                pass
                        elif (self.taskExec[tId] == -3):
                            if (self.taskAlloc[tId] == nbrTaskAlloc[tId]):
                                if (misc.gt(nbrTaskBidTime[tId], self.taskBidTime[tId]) or
                                    misc.gt(nbrTaskDropTime[tId], self.taskDropTime[tId])):
                                    bidUpdate += 1
                                    self.updateTask(tId, nbrTaskBidVal[tId], nbrTaskBidTime[tId], nbrTaskDropTime[tId], nbrTaskAlloc[tId], nbrTaskExec[tId])
                                else:
                                    pass
                            elif (nbrTaskAlloc[tId] < self.taskAlloc[tId]):
                                bidUpdate += 1
                                self.updateTask(tId, nbrTaskBidVal[tId], nbrTaskBidTime[tId], nbrTaskDropTime[tId], nbrTaskAlloc[tId], nbrTaskExec[tId])
                        elif (self.taskExec[tId] == -2):
                            pass
                        elif (self.taskExec[tId] == -1):
                            bidUpdate += 1
                            self.updateTask(tId, nbrTaskBidVal[tId], nbrTaskBidTime[tId], nbrTaskDropTime[tId], nbrTaskAlloc[tId], nbrTaskExec[tId])

                    elif (nbrTaskExec[tId] == -2):
                        if (self.taskExec[tId] == -4):
                            bidUpdate += 1
                            self.updateTask(tId, nbrTaskBidVal[tId], nbrTaskBidTime[tId], nbrTaskDropTime[tId], nbrTaskAlloc[tId], nbrTaskExec[tId])
                            self.taskFinTime[tId] = timeNow
                        elif (self.taskExec[tId] == -3):
                            bidUpdate += 1
                            self.updateTask(tId, nbrTaskBidVal[tId], nbrTaskBidTime[tId], nbrTaskDropTime[tId], nbrTaskAlloc[tId], nbrTaskExec[tId])
                            self.taskFinTime[tId] = timeNow
                        elif (self.taskExec[tId] == -2):
                            pass
                        elif (self.taskExec[tId] == -1):
                            bidUpdate += 1
                            self.updateTask(tId, nbrTaskBidVal[tId], nbrTaskBidTime[tId], nbrTaskDropTime[tId], nbrTaskAlloc[tId], nbrTaskExec[tId])
                            self.taskFinTime[tId] = timeNow

                    elif (nbrTaskExec[tId] == -1):
                        if (self.taskExec[tId] == -4):
                            # NALC is older info compared to DROP
                            pass
                        elif (self.taskExec[tId] == -3):
                            # EXEC is newer info compared to NALC
                            pass
                        elif (self.taskExec[tId] == -2):
                            # FNSH is newer info compared to NALC
                            pass
                        elif (self.taskExec[tId] == -1):
                            if ((nbrTaskAlloc[tId] != -1) and
                                (nbrTaskAlloc[tId] != self.index)):
                                # In cases where there are higher bids with older time and lower bids higher time,
                                # the higher bidder has to take action by updating bidTime. The consensus rules are
                                # defined for the info to be passed towards the higher bidder assuming single link.
                                if (misc.gt(nbrTaskBidTime[tId], self.taskBidTime[tId])):
                                    bidUpdate += 1
                                    self.updateTask(tId, nbrTaskBidVal[tId], nbrTaskBidTime[tId], nbrTaskDropTime[tId], nbrTaskAlloc[tId], nbrTaskExec[tId])
                            elif (nbrTaskAlloc[tId] == -1):
                                if (misc.gt(nbrTaskBidTime[tId], self.taskBidTime[tId])):
                                    bidUpdate += 1
                                    self.updateTask(tId, nbrTaskBidVal[tId], nbrTaskBidTime[tId], nbrTaskDropTime[tId], nbrTaskAlloc[tId], nbrTaskExec[tId])
        return bidUpdate

    def consensusBidTask(self, nbrId, nbrTaskIds, nbrTaskBidVal, nbrTaskExec, nbrTaskAlloc, nbrTaskBidTime, nbrTaskDropTime):
        """consensusBidTask
        Input arguments:
            nbrId -> id of the neighbour robot, int
            nbrTaskIds -> list of task ids from the neighbour in the beats, list
            nbrTaskBidVal -> dict of bidvalues of the tasks from the neighbour, corresponding to nbrIds
            nbrTaskExec -> dict of execution status of the tasks from the neighbour, corresponding to nbrIds
            nbrTaskAlloc -> dict of allocated robots of the tasks from the neighbour, corresponding to nbrIds
            nbrTaskBidTime -> dict of bid times of the tasks from the neighbour, corresponding to nbrIds
            nbrTaskDropTime -> dict of drop time of tasks from the neighbour, corresponding to nbrIds
        Returns:
            bidUpdate -> if there is a bidUpdate
        Description:
            reach consensus on the task on which the current bid is placed"""
        # nbr, taskIds, taskBid, taskExec, taskAlloc, taskBidTime, taskDropTime
        timeNow = time.time()
        bidUpdate = 0
        # reach consensus only for the currently bid and unallocated task
        if ((self.nTaskBid > 0) and (self.nTaskBid > self.nTaskAllocated)):
            tIndex = self.taskBidOrder[self.nTaskBid-1]
            # check only if the task is in the nbrTaskIds
            if (tIndex in (nbrTaskIds)):
                # not allocated, not completed
                if (self.taskExec[tIndex] == -1):
                    if (nbrTaskExec[tIndex] == -3):
                        # this is a new bid, with out knowing executing status
                        # EXEC is newer info than NALC
                        self.releaseTask(tIndex)
                        self.msgCount = 0
                        self.updateTask(tIndex, nbrTaskBidVal[tIndex], nbrTaskBidTime[tIndex], nbrTaskDropTime[tIndex], nbrTaskAlloc[tIndex], nbrTaskExec[tIndex])
                        bidUpdate = 1

                    elif (nbrTaskExec[tIndex] == -2):
                        self.releaseTask(tIndex)
                        self.msgCount = 0
                        self.updateTask(tIndex, nbrTaskBidVal[tIndex], nbrTaskBidTime[tIndex], nbrTaskDropTime[tIndex], nbrTaskAlloc[tIndex], nbrTaskExec[tIndex])
                        self.taskFinTime[tIndex] = timeNow
                        bidUpdate = 1

                    elif (nbrTaskExec[tIndex] == -1):
                        if ((nbrTaskAlloc[tIndex] != -1) and
                            (nbrTaskAlloc[tIndex] != self.index)):
                            # bidTime is of high priority than bidVal
                            # higher bidder has to update the time for a lower bidder to withdraw its bid
                            if ((misc.lt(nbrTaskBidVal[tIndex], self.taskBidVal[tIndex])) and
                                (not misc.eq(nbrTaskBidVal[tIndex], float("inf"))) and
                                (misc.gte(nbrTaskBidTime[tIndex], self.taskBidTime[tIndex]))):
                                print ("robot[%d] (%0.3f) lost task[%d] to robot[%d] (%0.3f)" %(self.index,  self.taskBidVal[tIndex], tIndex, nbrTaskAlloc[tIndex], nbrTaskBidVal[tIndex]))
                                self.releaseTask(tIndex)
                                self.msgCount = 0
                                self.updateTask(tIndex, nbrTaskBidVal[tIndex], nbrTaskBidTime[tIndex], nbrTaskDropTime[tIndex], nbrTaskAlloc[tIndex], nbrTaskExec[tIndex])
                                bidUpdate = 1

                            elif ((misc.eq(nbrTaskBidVal[tIndex], self.taskBidVal[tIndex])) and
                                (not misc.eq(nbrTaskBidVal[tIndex], float("inf"))) and
                                (misc.gte(nbrTaskBidTime[tIndex], self.taskBidTime[tIndex])) and
                                (nbrTaskAlloc[tIndex] < self.taskAlloc[tIndex])):
                                print ("robot[%d] (%0.3f) lost task[%d] to robot[%d] (%0.3f)" %(self.index,  self.taskBidVal[tIndex], tIndex, nbrTaskAlloc[tIndex], nbrTaskBidVal[tIndex]))
                                self.releaseTask(tIndex)
                                self.msgCount = 0
                                self.updateTask(tIndex, nbrTaskBidVal[tIndex], nbrTaskBidTime[tIndex], nbrTaskDropTime[tIndex], nbrTaskAlloc[tIndex], nbrTaskExec[tIndex])
                                bidUpdate = 1

                            elif ((misc.eq(nbrTaskBidVal[tIndex], self.taskBidVal[tIndex])) and
                                (not misc.eq(nbrTaskBidVal[tIndex], float("inf"))) and
                                (misc.lt(nbrTaskBidTime[tIndex], self.taskBidTime[tIndex])) and
                                (nbrTaskAlloc[tIndex] > self.taskAlloc[tIndex])):
                                self.msgCount = 0
                                self.updateTaskBidTime(tIndex) # updates only bid time
                                bidUpdate = 1 # for the time being keeping this as 1. only change is the bidtime.

                            elif ((misc.gte(nbrTaskBidVal[tIndex], self.taskBidVal[tIndex])) and
                                (not misc.eq(nbrTaskBidVal[tIndex], float("inf"))) and
                                (misc.gte(nbrTaskBidTime[tIndex], self.taskBidTime[tIndex]))):
                                self.msgCount = 0
                                self.updateTaskBidTime(tIndex) # updates only bid time. keeps the self bid
                                bidUpdate = 1 # for the time being keeping this as 1. only change is the bidtime.

                        elif (nbrTaskAlloc[tIndex] == -1):
                            if (misc.gt(nbrTaskBidTime[tIndex], self.taskBidTime[tIndex])):
                                self.msgCount = 0
                                self.updateTaskBidTime(tIndex)         # updates only bid time
                                bidUpdate = 1 # for the time being keeping this as 1. only change is the bidtime.

                    elif (nbrTaskExec[tIndex] == -4):
                        # DROP is newer than NALC
                        # to avoid partial data update, release the task, update with incoming
                        self.releaseTask(tIndex)
                        self.updateTask(tIndex, nbrTaskBidVal[tIndex], nbrTaskBidTime[tIndex], nbrTaskDropTime[tIndex], nbrTaskAlloc[tIndex], nbrTaskExec[tIndex])
                        self.msgCount = 0
                        bidUpdate = 1

                elif (self.taskExec[tIndex] == -4):
                    if (nbrTaskExec[tIndex] == -3):
                        # this is a new bid, with out knowing executing status
                        if (misc.gt(nbrTaskBidTime[tIndex], self.taskDropTime[tIndex])):
                            # the other robot's info is newer and another robot is executing the task
                            # the bid was placed after the task drop time (as known to robot)
                            self.releaseTask(tIndex)
                            self.msgCount = 0
                            self.updateTask(tIndex, nbrTaskBidVal[tIndex], nbrTaskBidTime[tIndex], nbrTaskDropTime[tIndex], nbrTaskAlloc[tIndex], nbrTaskExec[tIndex])
                            bidUpdate = 1

                    elif (nbrTaskExec[tIndex] == -2):
                        self.releaseTask(tIndex)
                        self.msgCount = 0
                        self.updateTask(tIndex, nbrTaskBidVal[tIndex], nbrTaskBidTime[tIndex], nbrTaskDropTime[tIndex], nbrTaskAlloc[tIndex], nbrTaskExec[tIndex])
                        self.taskFinTime[tIndex] = timeNow
                        bidUpdate = 1

                    elif (nbrTaskExec[tIndex] == -1):
                        # DROP is newer than NALC
                        pass
                    elif (nbrTaskExec[tIndex] == -4):
                        if ((nbrTaskAlloc[tIndex] != -1) and
                            (nbrTaskAlloc[tIndex] != self.index)):
                            # bidTime is of high priority than bidVal
                            # higher bidder has to update the time for a lower bidder to withdraw its bid
                            if ((misc.lt(nbrTaskBidVal[tIndex], self.taskBidVal[tIndex])) and
                                (not misc.eq(nbrTaskBidVal[tIndex], float("inf"))) and
                                (misc.gte(nbrTaskBidTime[tIndex], self.taskBidTime[tIndex]))):
                                print ("robot[%d] (%0.3f) lost task[%d] to robot[%d] (%0.3f)" %(self.index,  self.taskBidVal[tIndex], tIndex, nbrTaskAlloc[tIndex], nbrTaskBidVal[tIndex]))
                                self.releaseTask(tIndex)
                                self.msgCount = 0
                                self.updateTask(tIndex, nbrTaskBidVal[tIndex], nbrTaskBidTime[tIndex], nbrTaskDropTime[tIndex], nbrTaskAlloc[tIndex], nbrTaskExec[tIndex])
                                bidUpdate = 1

                            elif ((misc.eq(nbrTaskBidVal[tIndex], self.taskBidVal[tIndex])) and
                                (not misc.eq(nbrTaskBidVal[tIndex], float("inf"))) and
                                (misc.gte(nbrTaskBidTime[tIndex], self.taskBidTime[tIndex])) and
                                (nbrTaskAlloc[tIndex] < self.taskAlloc[tIndex])):
                                print ("robot[%d] (%0.3f) lost task[%d] to robot[%d] (%0.3f)" %(self.index,  self.taskBidVal[tIndex], tIndex, nbrTaskAlloc[tIndex], nbrTaskBidVal[tIndex]))
                                self.releaseTask(tIndex)
                                self.msgCount = 0
                                self.updateTask(tIndex, nbrTaskBidVal[tIndex], nbrTaskBidTime[tIndex], nbrTaskDropTime[tIndex], nbrTaskAlloc[tIndex], nbrTaskExec[tIndex])
                                bidUpdate = 1

                            elif ((misc.eq(nbrTaskBidVal[tIndex], self.taskBidVal[tIndex])) and
                                (not misc.eq(nbrTaskBidVal[tIndex], float("inf"))) and
                                (misc.gte(nbrTaskBidTime[tIndex], self.taskBidTime[tIndex])) and
                                (nbrTaskAlloc[tIndex] > self.taskAlloc[tIndex]) and
                                (misc.gt(nbrTaskDropTime[tIndex], self.taskDropTime[tIndex]))):
                                self.msgCount = 0
                                self.updateTaskTime(tIndex, nbrTaskDropTime[tIndex]) # updates bidtime and copies drop time from neighbour. bid is still valid
                                bidUpdate = 1

                            elif ((misc.eq(nbrTaskBidVal[tIndex], self.taskBidVal[tIndex])) and
                                (not misc.eq(nbrTaskBidVal[tIndex], float("inf"))) and
                                (misc.gte(nbrTaskBidTime[tIndex], self.taskBidTime[tIndex])) and
                                (nbrTaskAlloc[tIndex] > self.taskAlloc[tIndex])):
                                self.msgCount= 0
                                self.updateTaskBidTime(tIndex) # updates only bid time
                                bidUpdate = 1

                            elif ((misc.gte(nbrTaskBidVal[tIndex], self.taskBidVal[tIndex])) and
                                (not misc.eq(nbrTaskBidVal[tIndex], float("inf"))) and
                                (misc.gte(nbrTaskBidTime[tIndex], self.taskBidTime[tIndex])) and
                                (misc.gt(nbrTaskDropTime[tIndex], self.taskDropTime[tIndex]))):
                                self.msgCount = 0
                                self.updateTaskTime(tIndex, nbrTaskDropTime[tIndex]) # updates bidtime and copies drop time from neighbour. bid is still valid
                                bidUpdate = 1 # for the time being keeping this as 1. only change is the bidtime.

                            elif ((misc.gte(nbrTaskBidVal[tIndex], self.taskBidVal[tIndex])) and
                                (not misc.eq(nbrTaskBidVal[tIndex], float("inf"))) and
                                (misc.gte(nbrTaskBidTime[tIndex], self.taskBidTime[tIndex]))):
                                self.msgCount = 0
                                self.updateTaskBidTime(tIndex) # updates only bid time
                                bidUpdate = 1 # for the time being keeping this as 1. only change is the bidtime.

                        elif (nbrTaskAlloc[tIndex] == -1):
                            if (misc.gt(nbrTaskBidTime[tIndex], self.taskBidTime[tIndex])):
                                self.msgCount = 0
                                self.updateTaskBidTime(tIndex) # updates only bid time
                                bidUpdate = 1 # for the time being keeping this as 1. only change is the bidtime.

                            # this condition/rule is applicable to all other conditions after applying those rules
                            if (misc.gt(nbrTaskDropTime[tIndex], self.taskDropTime[tIndex])):
                                self.msgCount = 0
                                self.updateTaskTime(tIndex, nbrTaskDropTime[tIndex]) # updates bidtime and copies drop time from neighbour. bid is still valid
                                bidUpdate = 1 # for the time being keeping this as 1. only change is the bidtime.
        return bidUpdate

    def completeTask(self, endTime):
        """completeTask
        Input arguments:
            endTime -> time at which the execution is finished
        Returns:
            None
        Description:
            set task finish time and task finish status in the task vectors"""
        tIndex = self.taskBidOrder[self.nTaskAllocated-1]
        self.taskEndTime[self.nTaskAllocated-1][1] = copy.copy(endTime)

        self.taskVal[tIndex] = float("inf")
        self.taskExec[tIndex] = -2
        self.taskFinTime[tIndex] = endTime
        self.taskBiddable[tIndex] = 0

        self.stage = 0
        print ("r[%d] finished t[%d]" %(self.index, tIndex))

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
                    except queue.Empty:
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
                    except queue.Empty:
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

    def allocateTask(self):
        """allocateTask
        Input arguments:
            None
        Returns:
            None
        Description:
            allocate the task and call startTaskProcess, if the following constraints are satisfied
                msgCount > 4*neighbours
                there is a bid task
                if there is a current task, it is finished"""
#        # workaround to avoid single robot situation
#        if (self.nRobot == 1):
#            self.msgCount = 4

        if ((self.nTaskBid > self.nTaskAllocated) and (self.msgCount>=4*self.nNeighbour)):
            tIndex = self.taskBidOrder[self.nTaskBid-1]

            if ((self.taskExec[tIndex] == -1) or (self.taskExec[tIndex] == -4)): # NOAL or EDRP
                # if no prior allocated tasks or prior task is completed, allocate the new task
                if ((self.nTaskBid == 1) or
                    ((self.nTaskBid > 1) and (self.taskExec[self.taskBidOrder[self.nTaskAllocated - 1]] == -2))):
                    self.startTaskProcess(tIndex)
                    print ("robot[%d] navigation started to task [%d]: %d" %(self.index, tIndex, time.time()))

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

            xOrg, yOrg     = xNew+0, yNew+0
            DX = tX - xOrg
            DY = tY - yOrg
            D = misc.getDist(xOrg, yOrg, tX, tY)

            currRoom = self.mapInfo.getRoom(xNew, yNew)
            nextRoom = self.mapInfo.getRoom(tX, tY)

            if (misc.eq(xFinish, tX) and misc.eq(yFinish, tY)):
                taskZone = self.taskZone #*1.5 # not to detect the task object as obstacle
            else:
                taskZone = self.taskZone

            if (taskFinish == 2):
                break

            timePrev = time.time()
            taskFinish = 0
            while (taskFinish == 0):     # when task not finished
                time.sleep(0.1)
                timeNow = time.time()
                timeDelta = timeNow - timePrev
                distDelta = timeDelta * self.vel

                xNew = (distDelta/D)*DX + xOrg
                yNew = (distDelta/D)*DY + yOrg

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
                    com     = cmdQ.get_nowait() # command from main process. 1-> stop current task
                except queue.Empty:
                    pass
                else:
                    if (com == 1):
                        taskFinish = 2
                        break

                # check the room
                tempRoom     = self.mapInfo.getRoom(xNew, yNew)
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
            misc.sendMsg(msgQ, None, [xPrev, yPrev, stage, 0])
            return (xNew, yNew, taskFinish)

        # add detailed controls here for longer tasks after reaching the task location.
        print ("Error!!!, taskFinish = ", taskFinish)
        exit()
        return 0

    def detectObstacle(self, sectorData):
        """detectObstacle
        Input arguments:
            sectorData -> dict of sensor data (sonar/laser) for obstacle identification
        Returns:
            zone -> zon eof the closest obstacle
        Description:
            method to detect the closest obstacle and its zone from the sensor readings"""
        # circular zones around the robot
        # 3 zones.
        # zone 1: reduce speed as a safety measure
        # zone 2: to detect near task, reduce speed further
        # zone 3: deviate to avoid collision
        # sectorData is either sonar or laser.
        zone     = -1
        for s in (['L', 'LF', 'F', 'RF', 'R']):
            if (s in sectorData.keys()):
                if ((zone == -1) or (zone > 0)) and misc.lt(sectorData[s], self.zone[0]):
                    zone = 0
                elif ((zone == -1) or (zone > 1)) and misc.lt(sectorData[s], self.zone[1]):
                    zone = 1
                elif (zone == -1) and misc.lt(sectorData[s], self.zone[2]):
                    zone = 2
            else:
#                print ("sectorData has no key %s" %(s))
                pass

#        # there can be obstacles in more than one zone at the same distance.
#        if (zone != -1):
#            minVal         = min(sonSectorData.values())
#            minIndex     = []
#            for idx in (range (0, 8)):
#                if (abs(sonData[idx] - minVal) < self.epsilon): minIndex.append(idx)
#        else:
#            minVal         = -1.0
#            minIndex     = []
#        return (zone, minVal, minIndex)    # returns zone info: -1: no obstacle a-on any side, 0:<0.4, 1:0.4-0.5, 2:0.5-1.0

        return zone    # returns zone info: -1: no obstacle a-on any side, 0:<0.4, 1:0.4-0.5, 2:0.5-1.0

    def getDirections(self, rX, rY, rA, tX, tY):
        """getDirections
        Input arguments:
            rX -> robot's x
            rY -> robot's y
            rA -> robot's heading
            tX -> target's x
            tY -> tasrget's y
        Returns:
            dist -> distance to the target from current location
            theta -. heading of the target from current location based on the robot's heading
        Description:
            coordinates and orientation is passed to the method.
        decision on the direction and distance remaining is taken here"""
        dX = (tX - rX)
        dY = (tY - rY)
        dist = math.sqrt(dX**2 + dY**2)
        if (misc.gt(dY, 0.0)):
            if (misc.eq(dX, 0.0)): # divide by zero. angle is pi/2
                theta = math.pi/2 - rA
            elif (misc.gt(dX, 0.0)): # Q1
                theta = math.atan(dY/dX) - rA
            elif (misc.lt(dX, 0.0)): # Q2
                theta =  math.pi + math.atan(dY/dX) - rA
                if (misc.gt(abs(theta), math.pi)):
                    theta = theta - 2 * math.pi
        elif (misc.lt(dY, 0.0)):
            if (misc.eq(dX, 0.0)): # divide by zero. angle is pi/2
                theta = - math.pi/2 - rA
            elif (misc.gt(dX, 0.0)): # Q4
                theta = math.atan(dY/dX) - rA
            elif (misc.lt(dX, 0.0)): # Q3
                theta =  - math.pi + math.atan(dY/dX) - rA
        elif (misc.eq(dY, 0.0)):
            if (misc.eq(dX, 0.0)): # divide by zero. angle is pi/2
                theta = 0.0
            elif (misc.gt(dX, 0.0)):
                theta = - rA
            elif (misc.lt(dX, 0.0)):
                if (misc.lt(rA, 0.0)):
                    theta = - math.pi - rA
                else:
                    theta = - math.pi - rA
        if (misc.gt(abs(theta), 2*math.pi)):
            theta = (abs(theta)%(2*math.pi))*(theta/abs(theta))

        if (misc.gt(theta, math.pi)):
            theta = theta - 2 * math.pi
        elif (misc.lt(theta, -math.pi)):
            theta = theta + 2 * math.pi

        return (dist, theta)

    def checkStatus(self):
        """checkStatus
        Input arguments:
            None
        Returns:
            None
        Description:
            check the robot's status (active/inactive) based on the rOn and rOff"""
        timeNow = time.time()
        if (misc.gte(timeNow - self.timeInit, self.timeOn)):
            self.active = 1
        if (misc.gte(timeNow - self.timeInit, self.timeOff)):
            self.active = 0

    def checkFreeTasks(self, getAll=False):
        """checkFreeTasks
        Input arguments:
            getAll -> flag to get info about all tasks, boolean
        Returns:
            if getAll is set:
                taskFree -> number of free tasks
                taskIdx -> index of tasks which are free
                taskExec -> execution status of the free tasks
            else:
                taskFree -> 0 or 1, if there is atleast one free task, this loop breaks
        Description:
            check if there are any free tasks / get the list of all free tasks"""
        # returns freeTasks:
        #  0     : robot has not reached nTaskLimit and there are no free tasks.
        # +1    : robot has not reached nTaskLimit and there are unallocated tasks
        taskFree = 0
        taskIdx = -1
        taskExec = 0
        for tId in (self.taskIds):
            if ((self.taskExec[tId] == -1) or
                (self.taskExec[tId] == -3) or
                (self.taskExec[tId] == -4)):
                taskFree += 1
                taskIdx = tId
                taskExec = self.taskExec[tId]
                if (taskFree > 0):
                    break
        if (getAll):
            return (taskFree, taskIdx, taskExec)
        else:
            return taskFree

    def checkParallel (self, nbrTaskIds, nbrTaskBidVal, nbrTaskExec, nbrTaskAlloc, nbrTaskBidTime, nbrTaskDropTime):
        #taskAlloc, taskBid, taskExec, taskBidTime
        """cehckParallel
        Input arguments:
            nbrTaskIds -> task ids in the beats
            nbrTaskBidVal -> bid values in the beats corresponding to task ids
            nbrTaskExec -> execution status in the beats corresponding to task ids
            nbrTaskAlloc -> allocated robot ids in the beats corresponding to task ids
            nbrTaskBidTime -> bid time in the beats corresponding to task ids
            nbrTaskDropTime -> drop time in the beats corresponding to task ids
        Returns:
            None
        Description:
            This method checks if another robot is also executing the same task this robot is doing.
            The action for this scenario as of now is the robot with higher index release the task letting the other do the task.
            This would require the message to be passed in such a way that for communicating nodes should pass the info
            in the direction of higher index robot. ie when a msg (taskExec[tId] == -3, taskAlloc[tId]==x) is received and
            the current info is (taskExec[tId] == -3, taskAlloc[tId]==y), modify the info based on (x < y).
            Alternative approaches exist, but not implemented now. A decision could be made based on the
            bid value and the bid time.
            If there is task already bid for after starting the execution of the task, remove that as well as the bid will be invalid.
            This method is run during the execution of a task, may be from checkTaskProgress in general and as a method call
            during the bidding phase.
        """
        bidUpdate = 0
        timeNow = time.time()
        if (self.nTaskAllocated > 0):
            # check parallel execution of current task
            # if the other robot has a lower index or has finished the task, drop it
            tIndex = self.taskBidOrder[self.nTaskAllocated-1]
            # check only if tIndex is in nbrTaskIds
            if (tIndex in (nbrTaskIds)):
                if ((self.taskExec[tIndex] == -3) and
                    (nbrTaskExec[tIndex] == -3) and
                    (nbrTaskAlloc[tIndex] != self.index)):

                    # consider stopping execution when the index of the other robot is lower
                    if (nbrTaskAlloc[tIndex] < self.index):
                        # cannot consider timing info for consideration of the dropping the task.
                        if (misc.gt(nbrTaskDropTime[tIndex], self.taskDropTime[tIndex])):
                            # in case the robot has a wrong drop time info, just update that.
                            self.taskDropTime[tIndex] = nbrTaskDropTime[tIndex]
                        # any bid placed after starting the current task is invalid as the distance will vary
                        if (self.nTaskBid > self.nTaskAllocated):
                            self.releaseTask(self.taskBidOrder[self.nTaskBid-1])

                        print ("robot[%d] decides to stop task[%d] - parallel with robot[%d]"%(self.index, tIndex, nbrTaskAlloc[tIndex]))
                        # send stop command to taskNavigation
                        misc.sendMsg(self.cmdQ, None, 1)

                        # releasing the task and waiting for the graceful release
                        release     = 0
                        while (release == 0):
                            release = self.checkTaskProgress()
                        # roll back changes in task vectors
                        # the count for the bid placed on the task must also be removed
                        self.releaseTask(tIndex)
                        self.taskBidVal[tIndex] = nbrTaskBidVal[tIndex]
                        self.taskBidTime[tIndex] = nbrTaskBidTime[tIndex]
                        self.taskAlloc[tIndex] = nbrTaskAlloc[tIndex]
                        self.taskExec[tIndex] = nbrTaskExec[tIndex]
                        bidUpdate = 1
                        self.nTaskAllocated -= 1

                elif ((self.taskExec[tIndex] == -3) and
                    (nbrTaskExec[tIndex] == -2)):
                    # if a new bid is placed considering the current task, remove that bid as the current task is going to be released
                    if (self.nTaskBid > self.nTaskAllocated):
                        self.releaseTask(self.taskBidOrder[self.nTaskBid-1])

                    print ("robot[%d] decides to stop task[%d] - parallel with robot[%d]"%(self.index, tIndex, nbrTaskAlloc[tIndex]))
                    # send stop command to taskNavigation
                    misc.sendMsg(self.cmdQ, None, 1)

                    # releasing the task and waiting for the graceful release
                    release = 0
                    count = 0
                    while (release == 0):
                        count += 1
                        if (count%100): print ("robot[%d] stuck in releasing the task for %d loops" %(self.index, count))
                        release = self.checkTaskProgress()
                    # roll back changes in task vectors
                    # the count for the bid placed on the task must also be removed
                    # this release is different from dropping a task.
                    self.releaseTask(tIndex)
                    self.taskBidVal[tIndex] = nbrTaskBidVal[tIndex]
                    self.taskBidTime[tIndex] = nbrTaskBidTime[tIndex]
                    self.taskAlloc[tIndex] = nbrTaskAlloc[tIndex]
                    self.taskExec[tIndex] = nbrTaskExec[tIndex]
                    self.taskFinTime[tIndex] = timeNow
                    bidUpdate = 1
                    self.nTaskAllocated -= 1

                elif ((self.taskExec[tIndex] == -3) and
                    (nbrTaskExec[tIndex] == -3) and
                    (nbrTaskAlloc[tIndex] != self.index)):
                    print (self.index, nbrTaskAlloc[tIndex], self.taskDropTime[tIndex], nbrTaskDropTime[tIndex], self.taskBidTime[tIndex], nbrTaskBidTime[tIndex])
                    raise Exception("Unknown error in taskExec!!!")

        return bidUpdate

    def checkDrop(self):
        """checkFreeTasks
        Input arguments:
            None
        Returns:
            None
        Description:
            method to check whether the currently task to be dropped or not.
            conditions to drop current task are
            1. presence of a high priority task
            2. presence of any equal priority task with a better bid - now disabled
                a. with higher bid value from current position
                b. DROP status(?)
        """
        if ((self.nTaskAllocated > 0) and (self.nTaskAllocated <= self.nTaskLimit)):
            tIndex     = self.taskBidOrder[self.nTaskAllocated - 1]
            if (self.taskExec[tIndex] == -3):
                # find all high priority tasks
                # find all equal priority tasks
                hiPrio = [] # high priority task indices
                eqPrio = [] # equal priority task indices

                tVal = []
                ctVal = [0.0]

                maxValTask = -1
                maxValHiPrioTask = -1
                maxValEqPrioTask = -1

                # dropChoice: drop the current task based on this choice
                # 0 - only for emergency tasks
                # 1 - for all higher priority tasks
                # 2 - all higher priority tasks and equal priority tasks from current location
                dropChoice = 0

                # remaining distance and current task value to the current task
                ctVal = self.checkVal([tIndex])

                for tId in (self.taskIds):
                    if (tId != tIndex):

                        if (dropChoice == 0):
                            # for dropping in case of any tasks with emergency priority
                            if ((self.taskList[tId].stcPrio == 0) and (self.taskList[tIndex].stcPrio != 0) and
                                ((self.taskExec[tId] == -1) or (self.taskExec[tId] == -4))):
                                hiPrio.append(tId)
                        elif (dropChoice > 0):
                            # for dropping in case of any tasks with higher priority
                            if ((self.taskList[tId].stcPrio < self.taskList[tIndex].stcPrio) and
                                ((self.taskExec[tId] == -1) or (self.taskExec[tId] == -4))):
                                hiPrio.append(tId)
                            elif ((self.taskList[tId].stcPrio == self.taskList[tIndex].stcPrio) and
                                ((self.taskExec[tId] == -1) or (self.taskExec[tId] == -4))):
                                eqPrio.append(tId)

                # at this point hiPrio and eqPrio contains the indices of high priority tasks and equal priority tasks
                if (len(hiPrio) > 0): # if there is at least one task with more priority
                    # this task must be biddable
                    # the bid value is calculated from the current position rather than through the current task location
                    tVal = self.checkVal(hiPrio)
                    tBiddable = self.checkBiddable(hiPrio, tVal, ctVal[0], "hi")
                    maxValHiPrioTask = self.checkMaxValTask(hiPrio, tBiddable, tVal)

#                if (maxValHiPrioTask != -1):
#                    print ("robot["+str(self.index)+"] hiprio:", hiPrio, "tVal: ", tVal, "tBiddable: ", tBiddable)

                if (dropChoice > 1):
                    # when dropping of tasks are allowed for equal priority tasks but closer than the current task
                    if ((len(eqPrio) > 0) and
                        (maxValHiPrioTask == -1)): # if there is no high priority task identified and at least one task with equal priority
                        # this task must be biddable
                        # the bid value is calculated from the current position rather than through the current task location
                        tVal = self.checkVal(eqPrio)
                        tBiddable = self.checkBiddable(eqPrio, tVal, ctVal[0], "eq")
                        maxValEqPrioTask = self.checkMaxValTask(eqPrio, tBiddable, tVal)

#                if (maxValEqPrioTask != -1):
#                    print ("robot["+str(self.index)+"] eqprio:", eqPrio, "tVal: ", tVal, "tBiddable: ", tBiddable)

                if (maxValHiPrioTask != -1):
                    maxValTask = maxValHiPrioTask
                elif (maxValEqPrioTask != -1):
                    maxValTask = maxValEqPrioTask
                else:
                    maxValTask = -1

                # check the feasibility of current task
                dropTask = 0

                # drop the task and any further bids
                if (maxValTask != -1) or (dropTask == 1):
                    print ("robot[%d]: currTask: %d; maxValTask:%d; dropTask=%d" %(self.index, tIndex, maxValTask, dropTask))
                    # any bid placed after starting the current task is invalid as the distance will vary
                    if (self.nTaskBid > self.nTaskAllocated):
                        taskIndex = self.taskBidOrder[self.nTaskBid-1]
                        self.releaseTask(taskIndex)

                    # send stop command to taskNavigation
                    misc.sendMsg(self.cmdQ, None, 1)

#                    # releasing the task and waiting for the graceful release
                    release     = 0
                    while (release == 0):
                        release = self.checkTaskProgress()
                        if (release != 0):
                            if not(self.taskProcess.is_alive()):
                                release = 2
                                break

#                    if (release == 2):
#                        print ("r[%d] successfully dropped t[%d]" %(self.index, tIndex))
#                    else:
#                        print ("r[%d] release status for t[%d] = %d" %(self.index, tIndex, release))

                    # roll back changes in task vectors
                    # the count for the bid placed on the task must also be removed
                    self.dropTask(tIndex)
        return

    def checkVal(self, tIdxList):
        """checkVal
        Input arguments:
            taskList     - indices of tasks
        Returns:
            tVal     - value of all tasks in taskList based on skills and expertise
        Description:
            Calculates task value for each task in taskList considering the current task (if any) is dropped before the task is executed
        """
        ## Find number of tasks already allocated to this robot.
        ## The tasks already in the taskExecOrder are counted
        ## get the taskReward only for those robots which are not reached nTaskLimit

        tVal = []
        for tId in (tIdxList):
            if (self.taskList[tId].active == 1):
                if (self.taskExec[tId] == -2):
                    tVal.append(float("inf"))            # same as execution status
                elif (self.taskExec[tId] == -3):
                    tVal.append(float("inf"))
                else:
                    if (self.taskExecutable[tId] == 1): # the robot has all the skills
                        tVal.append(0.0)

                        nextTaskSkills = self.taskList[tId].skillsReq
                        nextTaskEffort = self.getEffort(nextTaskSkills, tId) # gives the scaled efforts of all required skills as dict(skill:effort)

                        for skill in (nextTaskSkills):
                            if (nextTaskEffort[skill] == float("inf")):
                                taskVal = float("inf")
                                break

                        taskVal = 0.0
                        for skill in (nextTaskSkills):
                            taskVal += nextTaskEffort[skill] / self.expertise[skill]

                        tVal[-1] = taskVal

                        #TODO: check the effort reduction as the execution progresses, properly

                    else: # if a task is not executable, set the value as inf
                        tVal.append(float("inf"))

        return tVal

    def checkBiddable(self, tIdxList, tVal, ctVal, prio):
        """cehkBiddable
        Input arguments:
            tIdxList     - list of tasks
            tVal         - task values considering normalised tDist
            ctVal         - current task value (remaining distance)
            prio         - "hi" / "eq"
        Returns:
            tBiddable     - status whether a task is biddable or not
        Description:
            Sets the task as biddable when looking for task dropping conditions, if
                The task is not bid already
                The tasks current taskBidVal < tVal
            Resets for all other cases
        """
        # check the possible distance that can be traveled with in timeOff
        tBiddable = []
        for j in (range (len(tIdxList))):
            tIndex = tIdxList[j]
            if (self.taskList[tIndex].active == 0):
                self.taskList[tIndex].checkStatus()

            if (self.taskList[tIndex].active == 1):
                if ((self.taskExec[tIndex] == -1) or (self.taskExec[tIndex] == -4)):
                    if ((misc.lt(tVal[j], self.taskBidVal[tIndex])) and
                        (not misc.eq(tVal[j], float("inf"))) and
                        (prio == "hi")):
                        tBiddable.append(1)

                    elif ((misc.lt(tVal[j], self.taskBidVal[tIndex])) and
                        (not misc.eq(tVal[j], float("inf"))) and
                        (prio == "eq") and
                        (misc.lt(tVal[j], ctVal))):
                        tBiddable.append(1)
                    else:
                        tBiddable.append(0)

                else:
                    tBiddable.append(0)

            else:
                tBiddable.append(0)
        return tBiddable

    def checkMaxValTask(self, tIdxList, tBiddable, tVal):
        """checkMaxValTask
        Input arguments:
            tIdxList     - list of tasks
            tBiddable     - whether the tasks in tIdxList are biddable
            tVal         - values of tasks in tIdxList from the current location
        Returns:
            maxValTaskIndex -> the index of biddable task, giving maximum taskVal, int
            -1 -> If there are no biddable tasks
        Description:
            checks for a task with maximum bid value based on the given input arguments
            Returns index of the task with maximum tVal  from the biddable tasks
        """
        biddableMaxVal = float("inf")
        biddableMaxIndex = -1

        if (sum(tBiddable) > 0):             # there is at least one biddable task
            for j in (range (len(tIdxList))):
                tIndex = tIdxList[j]
                # check the status of the task
                if (self.taskList[tIndex].active == 0):
                    self.taskList[tIndex].checkStatus()

                if (self.taskList[tIndex].active == 1):
                    # task is biddable and
                    # task has higher priority than the biddableMaxIndex task
                    # task's value is greater than the current bid
                    if ((tBiddable[j] == 1) and
                        (misc.lt(tVal[j], biddableMaxVal)) and
                        (not misc.eq(tVal[j], float("inf"))) and
                        (misc.lt(tVal[j], self.taskBidVal[tIndex]))):
                        biddableMaxVal = tBiddable[j] * tVal[j]
                        biddableMaxIndex = copy.copy(tIndex)
        return biddableMaxIndex

    def getDist(self, tIndex):
        """getDist
        Input arguments:
            tIndex ->index of task
        Returns:
            dist -> Distance from the current position to the next task
        Description:
            return the distance from the current position to the task with the given tIndex, through the current task, if there is any
                get the path to the target
                get the path distance
        """
        # gets the total distance from the current location/current task finish location to the finishing of the task (tIndex)
        # to be used in calcVal()

        # distance to be travelled to finish the task
        dist = 0.0
        # path for the task
        path = []
        # any current task ?
        if (self.nTaskAllocated == 0):
            currTaskIdx = None
        else:
            currTaskIdx = self.taskBidOrder[self.nTaskAllocated-1]

        # if currTask == None: return the distance to the tIndex from current location
        # if curreTask != None and currTask == tIndex: return the remaining distance to finish the task
        # if curreTask != None and currTask != tIndex: return the distance to the tIndex from finish location of currTask
        if (currTaskIdx == None):
            xOrg = self.x[-1]
            yOrg = self.y[-1]
        elif (currTaskIdx != None) and (currTaskIdx == tIndex):
            xOrg = self.x[-1]
            yOrg = self.y[-1]
        elif (currTaskIdx != None) and (currTaskIdx != tIndex):
            xOrg = self.taskList[currTaskIdx].xFinish
            yOrg = self.taskList[currTaskIdx].yFinish

        # task corresponding to tIndex
        task = self.taskList[tIndex]

        # if the task given is the current task, the distance has to be
        # calculated based on the stage of execution of the task. else
        # the distance can be calculated by finding the complete path from
        # the xOrg, yOrg (eq to fin loc of current task if there is one)
        # to the fin loc of the task through the task loc
        if (currTaskIdx == tIndex):
            tXOrg = task.xOrg
            tYOrg = task.yOrg
            tXFin = task.xFinish
            tYFin = task.yFinish

            # navigation to the task location
            if (self.stage == 0):
                path = path + self.getPath(xOrg, yOrg, tXOrg, tYOrg)

            # navigation to the final location/execution
            # find the path for the travel from the task org loc to task finish loc
            if (self.stage == 0):
                path = path + self.getPath(tXOrg, tYOrg, tXFin, tYFin)
            elif (self.stage == 1):
                path = path + self.getPath(xOrg, yOrg, tXFin, tYFin)
            # total distance of the path
            dist = self.getPathDist([(xOrg, yOrg)]+path)
        else:
            tXOrg = task.xOrg
            tYOrg = task.yOrg
            tXFin = task.xFinish
            tYFin = task.yFinish

            # navigation to the task location
            path = path + self.getPath(xOrg, yOrg, tXOrg, tYOrg)
            if (len(path) == 0):
                dist = float("inf")
            else:
                # navigation to the final location/execution
                # find the path for the travel from the task org loc to task finish loc
                path = path + self.getPath(tXOrg, tYOrg, tXFin, tYFin)
                # total distance of the path
                dist = self.getPathDist([(xOrg, yOrg)]+path)
        return dist

    def getPathDist(self, path):
        """getPathDist
        Input arguments:
            path -> list of waypoint coordinates
        Returns:
            dist -> Distance from the first node to the last node in the given path
        Description:
            returns the distance from the first node to the last node in the given path
        """
        # given a path find the distance
        dist = float("inf")
        for idx in (range (len(path)-1)):
            if (dist == float("inf")):
                dist = misc.getDist(path[idx][0], path[idx][1], path[idx+1][0], path[idx+1][1])
            else:
                dist += misc.getDist(path[idx][0], path[idx][1], path[idx+1][0], path[idx+1][1])
        return dist

    def getPath(self, xOrg, yOrg, xFinish, yFinish):
        """getPath
        Input arguments:
            xOrg -> starting x coordinate
            yOrg -> starting y coordinate
            xFinish -> target x coordinate
            yFinish -> target y coordinate
        Returns:
            path -> list of way point coordinates from start location to target location
        Description:
            returns the path list of way point coordinates from start location to target location, based on the mapInfo
        """
        # returns the xy coordinates of the nodes in the path as a list of tuples
        if (self.mapInfo.nNodes > 0):
            (startRoomId, startNode)     = self.mapInfo.getClosestNode(xOrg, yOrg)
            (goalRoomId, goalNode)         = self.mapInfo.getClosestNode(xFinish, yFinish)
            if (startRoomId == goalRoomId): # same room go directly
                path = []
#                path     = self.mapInfo.aStar(startNode, goalNode)
            elif (startNode != -1) and (goalNode != -1): # separate rooms - find path using graph
                path = self.mapInfo.aStar(startNode, goalNode)
                newPath = []
                for node in (path):
                    x = self.mapInfo.nodes[node].x
                    y = self.mapInfo.nodes[node].y

                    if (startRoomId == self.mapInfo.getRoom(x, y)):
                        for k in (range (len(newPath))):
                            newPath.pop()
                    elif (goalRoomId == self.mapInfo.getRoom(x, y)):
                        newPath.append(node)
                        break
                    newPath.append(node)
                path = newPath
#                print ("r[%d] path: "%(self.index), path)
            else: # no closest startNode or goalNode. travel may not be possible
                path     = None
        else: # no graph present
            path = []
        xy        = []
        if (path != None):
            for item in (path):
                xy.append((self.mapInfo.nodes[item].x, self.mapInfo.nodes[item].y))
            xy.append((xFinish, yFinish))
        return xy

    def getEffort(self, skillsReq, tIndex):
        # given the set of skills required and the task Index, find the effort for each of them

        effortsReq = {}
        # assuming for each skill the robot has, there is a function corresponding to each skill to get the effrort
        for skill in (skillsReq):
            if (skill == "navigation"):
                effortsReq[skill] = self.getDist(tIndex)
            elif (skill == "vision"):
                # effort calculating / function calling here
                effortsReq[skill] = self.getRemTime(tIndex, skill)
            elif (skill == "audition"):
                # effort calculating / function calling here
                effortsReq[skill] = self.getRemTime(tIndex, skill)
            elif (skill == "gripper"):
                # effort calculating / function calling here
                effortsReq[skill] = self.getRemTime(tIndex, skill)
            elif (skill == "manipulator"):
                # effort calculating / function calling here
                effortsReq[skill] = self.getRemTime(tIndex, skill)
            elif (skill == "cleaner"):
                # effort calculating / function calling here
                effortsReq[skill] = self.getRemTime(tIndex, skill)
        return effortsReq

    def getRemTime(self, tIndex, skill):
        currTask = None
        stage = 0

        if (self.nTaskAllocated > 0):
            currTask = self.taskBidOrder[self.nTaskAllocated-1]

        if (currTask != None) and (currTask == tIndex):
            stage = self.stage

        if (stage == 0):
            return self.taskList[tIndex].timeExec[skill]
        else:
            # time calculation based on skill and task type
            if (self.taskList[tIndex].taskType == "fall"):
                # fall: navigation -> vision -> audition
                if (skill == "vision"):
                    timeStart = self.taskStartTime[self.nTaskAllocated-1][1]
                    
                elif (skill == "audition"):
                    timeStart = self.taskStartTime[self.nTaskAllocated-1][1] + self.taskList[tIndex].timeExec["vision"]
                    

            elif (self.taskList[tIndex].taskType == "surveillance"):
                # surveillance: navigation -> vision -> audition
                if (skill == "vision"):
                    timeStart = self.taskStartTime[self.nTaskAllocated-1][1]
                    
                elif (skill == "audition"):
                    timeStart = self.taskStartTime[self.nTaskAllocated-1][1] + self.taskList[tIndex].timeExec["vision"]
                    

            elif (self.taskList[tIndex].taskType == "medicine"):
                # medicine: navigation -> audition
                if (skill == "audition"):
                    timeStart = self.taskStartTime[self.nTaskAllocated-1][1]
                    

            elif (self.taskList[tIndex].taskType == "clean"):
                # clean: navigation -> cleaner
                if (skill == "cleaner"):
                    timeStart = self.taskStartTime[self.nTaskAllocated-1][1]
                    

            elif (self.taskList[tIndex].taskType == "door"):
                # door: navigation -> vision -> manipulator -> audition
                if (skill == "vision"):
                    timeStart = self.taskStartTime[self.nTaskAllocated-1][1]
                    
                elif (skill == "manipulator"):
                    timeStart = self.taskStartTime[self.nTaskAllocated-1][1] + self.taskList[tIndex].timeExec["vision"]
                    
                elif (skill == "audition"):
                    timeStart = self.taskStartTime[self.nTaskAllocated-1][1] + self.taskList[tIndex].timeExec["vision"] + self.taskList[tIndex].timeExec["manipulator"]
                    

            elif (self.taskList[tIndex].taskType == "delivery"):
                # delivery: navigation -> gripper -> navigation -> gripper
                if (skill == "gripper"):
                    timeStart = self.taskStartTime[self.nTaskAllocated-1][1]
                    

            timeNow = time.time()
            timeDiff = (timeNow - timeStart)
            if (misc.gte(timeDiff, self.taskList[tIndex].timeExec[skill])):
                return 0
            elif (misc.gt(timeDiff, 0)):
                return timeDiff
            else:
                return self.taskList[tIndex].timeExec[skill]

    def detectRobot(self, zone, rX, rY, rA):
        """detects whether the obstacle detected is a robot
        sends the list of all possible robots in the nearby area"""
        sinA = math.sin(rA)
        cosA = math.cos(rA)
        delta = self.zone[zone]
        obRobots = []

        for rC in (range (self.nRobot)):
            # rV and rO are coordinate transformation in terms of the robot axis
            # robot is assumed to be moving in the local y coordinate direction (myA => O axis)
            if (misc.lt(time.time() - self.robotLocTime[rC], 30)):
                pX = self.robotLoc[rC][0] - rX
                pY = self.robotLoc[rC][1] - rY

                rN = pX*sinA - pY*cosA
                rO = pX*cosA + pY*sinA

                if (misc.lte(abs(rN), delta) and  misc.gt(rO, 0.0) and misc.lt(rO, delta)):
                    obRobots.append([copy.copy(rC), rN, rO]) # appending the index and rLoc of the robot which is an obstacle
                else:
                    pass
            else:
                pass

        return obRobots

    def getTaskDetails(self, rIndex):
        """returns the index of the task if a robot with rIndex is executing any task currently"""
        for tC in (range (self.nTask)):
            if (rIndex == self.taskAlloc[tC]) and (self.taskExec[tC] == -3):
                return tC
        return None

    def robotObstacle(self, datLock, zone, rX, rY, rA):
        """checks whether there are any robots and returns the suggested actions
        0 -> go ahead with normal obstacle avoidance actions
        1 -> stop and wait """
        # get indices of all robots in the zone
        obRobots = self.detectRobot(zone, rX, rY, rA)
        numObRobots = len(obRobots)
        if (numObRobots == 0):
            return 0

        # there could be more than one robot
        # get the index, priority and execution status of the task of the other robots
        obRobotTasks = []
        for rC in (obRobots):
            print (rC[0])
            if (rC[0] != self.index):
                tIndex = self.getTaskDetails(rC[0])
                if (tIndex != None):
                    obRobotTasks.append(copy.copy(tIndex))

        numObRobotTasks = len(obRobotTasks)
        if (numObRobotTasks == 0): # no tasks is being executed, same as no robots
            return 0
        elif (numObRobotTasks >= 1): # one or more robots with tasks
            readStat = -1
            while (readStat == -1):
                lockStat = datLock.acquire(False) # for reading taskExecStatus
                #===================================================================
                # TODO: modify areas where taskExec, nTaskAllocated are modified
                #===================================================================
                if (lockStat):
                    if (self.taskExec[self.taskBidOrder[self.nTaskAllocated-1]] == -3):
                        stIndex = self.taskBidOrder[self.nTaskAllocated-1]
                        stPrio = self.taskList[stIndex].stcPrio # priority of self task
                        readStat = 0
                        datLock.release()
                    else:
                        datLock.release()
                        return 1

            lPrio = 0 # count of tasks with higher priority
            ePrio = 0 # count of tasks with equal priority
            hPrio = 0 # count of tasks with higher priority
            ePrioTasks = []
            for tC in (range (numObRobotTasks)):
                ortPrio    = self.taskList[obRobotTasks[tC]].stcPrio # obsRobotTask Priority
                if (stPrio > ortPrio):
                    hPrio += 1
                    break
                elif (stPrio == ortPrio):
                    ePrio += 1
                    ePrioTasks.append(copy.copy(tC))
                else:
                    lPrio += 1
            if (hPrio > 0): # if there is atleast one task with high priority wait
                print ("robot[%d]: robot obstacles; action 1" %(self.index))
                return 1
            elif  (lPrio == numObRobotTasks): # if all other tasks are of lower priority continue
                print ("robot[%d]: robot obstacles; action 0" %(self.index))
                return 0
            elif (stIndex < min (ePrioTasks)): # reaches here if there are some ePrio and no hPrio tasks. if self task index is less than all other ePrio tasks, continue
                print ("robot[%d]: robot obstacles; action 0" %(self.index))
                return 0
            else: # there are some ePrio and lPrio tasks. self task index is higher than other ePrio tasks. so wait
                print ("robot[%d]: robot obstacles; action 0" %(self.index))
                return 0

    def stopExecution(self):
        if (self.taskProcess != None):
            if (self.taskProcess.is_alive()):
                misc.sendMsg(self.cmdQ, None, 1)
                while (True):
                    if not(self.taskProcess.is_alive()):
                        break

    def checkTaskFinish(self, rX, rY, tX, tY, taskZone):
        """ checks whether the robot has reached E-near the task location"""
        dist = misc.getDist(rX, rY, tX, tY)
        if (misc.lt(dist, taskZone)):
        #if (dist < self.taskZone):
            return 1
        else:
            return 0

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

    def checkGoToOrg(self):
        return self.checkGoTo()

    def checkGoTo(self):
        """this checks the completion of the childprocess handling the traversal to the org location"""
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
                        stat = msgNew[2]
                    except queue.Empty:
                        break
                    else:
                        self.x.append(xNew)
                        self.y.append(yNew)

                        # normal task completion
                        if (stat == 1):
                            break
                        # task stopped as per command
                        elif (stat == 2):
                            break

            else: # process finished. now reading all data from queue
                while (True):
                    try:
                        msgNew = self.msgQ.get_nowait()
                        xNew = msgNew[0]
                        yNew = msgNew[1]
                        stat = msgNew[2]
                    except queue.Empty:
                        break
                    else:
                        self.x.append(xNew)
                        self.y.append(yNew)
                        # normal task completion
                        if (stat == 1):
                            break
                        # task stopped as per command
                        elif (stat == 2):
                            break

            if (stat == 1):
                self.taskProcess.join()
                print ("robot[%d] navigation to original robot location completed: %d" %(self.index, time.time()))
                return 1
            elif (stat == 2):
                self.taskProcess.join()
                print ("robot[%d] navigation to original robot location is stopped as per command: %d" %(self.index, time.time()))
                return 2
            else:
                return 0

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
            except queue.Empty:
                pass
            else:
                if (cmd == 1):
                    taskStat = 2
                    break
                else:
                    pass
        return taskStat

    def updateNeighbourList(self, neighbourList, prevBeats):
        """remove rId from NL if no beats is received for some time"""
        waitRemove = 30.0  # wait for 30 seconds since last beats from a robot before it is removed from NL
        timeNow = time.time()

        for rId in (neighbourList):
            if (rId in prevBeats.keys()):
                if misc.gt(timeNow-prevBeats[rId][1], waitRemove):
                    neighbourList.remove(rId)

        return neighbourList

    def encodeBEATs(self, neighbourList):
        """pack the BEATs and return it"""
        timeNow = time.time()
        waitFinish = 30.0 # time upto which a finished task's info is packed in beats
        beats = [] # beats packed as a list
        # msg id: msgType + msgTime + senderIndex
        beats.append("beats") # msg type. beats, hs, hsack, esr, esrack, esq, esqack, edr
        beats.append(timeNow) # msg-timeStamp
        beats.append(self.index) # sending robot's index
        # msg receivers: msgMode + numReceivers + receivers
        beats.append("m") # u-unicast, m-multicast, b-braodcast
        beats.append(len(neighbourList)) # no of receivers, -1 for broadcast, 1 for unicast and n for multicast -> this value needs to be modified in robotProcess
        beats.append([]+neighbourList) # receiver robot indices, empty list for broadcast -> this value needs to be modified in robotProcess
        # msg content: taskIdx, bidVal, execStat, taskAlloc, bidTime, dropTime
        taskIdx = [] # index = 6
        taskBidVal = {} # index = 7
        taskExec = {} # index = 8
        taskAlloc = {} # index = 9
        taskBidTime = {} # index = 10
        taskDropTime = {} # index = 11

        for tId in (self.taskIds):
            addTask = 0
            if (self.taskExec[tId] == -1):
                if (not(misc.eq(self.taskBidTime[tId], -1.0))):
                    addTask = 1
                
            elif (self.taskExec[tId] == -2):
                if (misc.gt(timeNow - self.taskFinTime[tId], waitFinish)): # tasks finished 30 seconds before are not parsed in beats
                    pass
                else:
                    addTask = 1
            else:
                addTask = 1

            # if the task has to be added, add the details to beats
            if (addTask == 1):
                taskIdx.append (tId)
                taskBidVal [tId] = self.taskBidVal[tId]
                taskExec [tId] = self.taskExec[tId]
                taskAlloc [tId] = self.taskAlloc[tId]
                taskBidTime [tId] = self.taskBidTime[tId]
                taskDropTime [tId] = self.taskDropTime[tId]

        beats = beats + [taskIdx, taskBidVal, taskExec, taskAlloc, taskBidTime, taskDropTime]
#        print ("r[%d]'s beats" %(self.index)),
#        print (beats)
        return beats

    def decodeBEATs(self, tempBeats, neighbourList, prevBeats):
        """beats decode"""
        beats = []
        for msg in (tempBeats):
            rId = msg[2]
            # ignore all beats from robots not in NL
            if (rId in (neighbourList)):
                if (rId not in prevBeats.keys()):
                    prevBeats[rId] = []
                if (prevBeats[rId] != msg):
                    prevBeats[rId] = [] + msg
                    taskIdx = msg[6]     # lsit of tIds
                    taskBidVal = msg[7]     # dict of B of tIds
                    taskExec = msg[8]     # dict of E of tIds
                    taskAlloc = msg[9]     # dict of A of tIds
                    taskBidTime = msg[10]     # dict of TB of tIds
                    taskDropTime = msg[11]     # dict of TD of tIds
                    beats.append([rId, taskIdx, taskBidVal, taskExec, taskAlloc, taskBidTime, taskDropTime])
        return (beats, prevBeats)

    def encodeHS(self):
        """"hand shake encode"""
        timeNow = time.time()

        hs = [] # hs packed as a list
        hs.append("hs")
        hs.append(timeNow)
        hs.append(self.index) # index of the sender robot
        hs.append("b") # broadcasting
        hs.append(-1) # number of receivers
        hs.append([]) # receiving robot indices
        hs.append(self.ipAdd) # ip address of the sender robot
        hs.append(6660) # communication port through which other robots can communicate to this robot
        # a robot joins later must get the status of finished tasks to avoid trying to do them again
        # as the beats are now restricted to a smaller set of tasks, even if the robot processes the beats just after
        # joining, these status will not be obtained. set this value as 1 to get the complete beats at the start with hsack
        if (self.initialBeats != 0):
            hs.append(0)
        else:
            hs.append(1)
        return hs

    def decodeHS(self, hs):
        """hand shake decode"""
        timeHs = hs[1]
        rId = hs[2]
        fullBeats = hs[8]
        return (timeHs, rId, fullBeats)

    def encodeHSACK(self, hs, neighbourList):
        """hand shake acknowledgement encode"""
        (timeHs, rId, fullBeats) = self.decodeHS(hs)

        if (rId in neighbourList):
            return (None, None) # rId already in NL
        elif (rId == self.index):
            return (None, None) # rId is self.index
        else:
            timeNow = time.time()
            hsAck = [] # hsack packed as a list

            hsAck.append("hsack") # msg type
            hsAck.append(timeNow) # msg time
            hsAck.append(self.index) # sender index
            hsAck.append(timeHs) # response to hs with this ts
            hsAck.append("u") # unicast
            hsAck.append(1) # number of receivers
            hsAck.append([rId]) # receiving robot indices
            hsAck.append(self.ipAdd) # ip address of the sender robot
            hsAck.append(6660) # communication port through which other robots can communicate to this robot
            beats = [] # partial beats - of the finished tasks and currently active (being bid and executed) tasks
            if (fullBeats == 1): # if the request for full beats is received, fill up beats
                beats.append("beats") # msg type. beats, hs, hsack, esr, esrack, esq, esqack, edr
                beats.append(timeNow) # msg-timeStamp
                beats.append(self.index) # sending robot's index
                # msg receivers: msgMode + numReceivers + receivers
                beats.append("u") # u-unicast, m-multicast, b-braodcast
                beats.append(1) # no of receivers, -1 for broadcast, 1 for unicast and n for multicast -> this value needs to be modified in robotProcess
                beats.append([rId]) # receiver robot indices, empty list for broadcast -> this value needs to be modified in robotProcess
                # msg content: taskIdx, bidVal, execStat, taskAlloc, bidTime, dropTime
                taskIdx = [] # index = 6
                taskBidVal = {} # index = 7
                taskExec = {} # index = 8
                taskAlloc = {} # index = 9
                taskBidTime = {} # index = 10
                taskDropTime = {} # index = 11

                for tId in (self.taskIds):
                    addTask = 0
                    if (self.taskExec[tId] == -1):
                        if (not(misc.eq(self.taskBidTime[tId], -1.0))):
                            addTask = 1
                    elif (self.taskExec[tId] == -2):
                        addTask = 1
                    else:
                        addTask = 1
                    # if the task has to be added, add the details to beats
                    if (addTask == 1):
                        taskIdx.append (tId)
                        taskBidVal [tId] = self.taskBidVal[tId]
                        taskExec [tId] = self.taskExec[tId]
                        taskAlloc [tId] = self.taskAlloc[tId]
                        taskBidTime [tId] = self.taskBidTime[tId]
                        taskDropTime [tId] = self.taskDropTime[tId]
                beats = beats + [taskIdx, taskBidVal, taskExec, taskAlloc, taskBidTime, taskDropTime]
            hsAck.append(beats)
            return (rId, hsAck) # rId not in NL. add to NL and send HSACK

    def decodeHSACK(self, hsAck, neighbourList):
        """hand shake acknowledgement decode"""
        waitHsack = 4 # wait time for hsack from hs

        rId = hsAck[2]
        beats = hsAck[9]
        if (rId in neighbourList): # rId already in NL, beats could have been obtained already
            return (None, [])
        else:
            timeNow = time.time()
            timeAckFor = hsAck[3]
            # delayed HSACk
            if (misc.gt((timeAckFor-timeNow), waitHsack)):
                return (None, [])
            else:
                return (rId, beats) # rId not in NL. add rId to NL


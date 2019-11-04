## ---------------------------------------------------- ##
# file: task.py
# desc: defines a task and the basic methods
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

class Task:
    def __init__(self, tIndex, xOrg=0.0, yOrg=0.0, xFinish=0.0, yFinish=0.0, onTime = 0.0, offTime=float("+inf"), stcPrio=5, taskType = 2):
        """
        Description of the task class
        """
        self.index = copy.copy(tIndex)

        self.xOrg = copy.copy(xOrg)
        self.yOrg = copy.copy(yOrg)
        self.xFinish = copy.copy(xFinish)
        self.yFinish = copy.copy(yFinish)

        self.timeOn = copy.copy(onTime)
        self.timeOff = copy.copy(offTime)

        self.active = 0
        self.timeInit = 0

        self.timeExec = {} # skill based execution time for the task (seconds). For multi location tasks, it repeats at each location
        self.timeMax = 0.0 # maximum time of observation; used for scaling tExec for bid calculation

        self.taskType = taskType # 0 -> transportation, 1 -> pushing, 2 -> surveillance

        # skills = ["navigation", "audition", "vision", "gripper", "manipulator", "cleaner"]
        if (self.taskType == "fall"):     # pushing
            self.skillsReq = ["navigation", "vision", "audition"]
            self.stcPrio = 0
            self.timeExec["vision"] = 8 * 10.0
            self.timeExec["audition"] = 10.0
            self.timeMax = self.timeExec["vision"] + self.timeExec["audition"]

        elif (self.taskType == "delivery"):     # transportation/delivery
            self.skillsReq = ["navigation", "gripper"]
            self.stcPrio = 2
            self.timeExec["gripper"] = 60.0
            self.timeMax = self.timeExec["gripper"]

        elif (self.taskType == "door"):
            self.skillsReq = ["navigation", "vision", "manipulator", "audition"]
            self.stcPrio = 1
            self.timeExec["vision"] = 2 * 10.0
            self.timeExec["manipulator"] = 60.0
            self.timeExec["audition"] = 10.0
            self.timeMax = self.timeExec["vision"] + self.timeExec["manipulator"] + self.timeExec["audition"]

        elif (self.taskType == "medicine"):
            self.skillsReq = ["navigation", "audition"]
            self.stcPrio = 2
            self.timeExec["audition"] = 10.0
            self.timeMax = self.timeExec["audition"]

        elif (self.taskType == "clean"):
            self.skillsReq = ["navigation", "cleaner"]
            self.stcPrio = 3
            self.timeExec["cleaner"] = 60.0
            self.timeMax = self.timeExec["cleaner"]

        elif (self.taskType == "surveillance"): # surveillance
            self.skillsReq = ["navigation", "vision", "audition"]
            self.stcPrio = 3
            self.timeExec["vision"] = 8 * 10.0
            self.timeExec["audition"] = 10.0
            self.timeMax = self.timeExec["vision"] + self.timeExec["audition"]

        self.stages     = 2

    def setStaticPriority(self, staticPriority):
        self.stcPrio = staticPriority

    def checkStatus(self):
        timeNow = time.time()
        if (misc.gte(timeNow - self.timeInit, self.timeOn) and (self.active == 0)):
            self.active = 1
        if (misc.gte(timeNow - self.timeInit, self.timeOff) and (self.active == 1)):
            self.active = 0



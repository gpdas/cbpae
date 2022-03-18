## ---------------------------------------------------- ##
# file: gui.py
# desc: a simple gui for stopping all robots. initialised from cbpae.py
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

import wx
import wx.lib.agw.flatnotebook as fnb

import matplotlib.pyplot
#matplotlib.use('WXAgg')

import matplotlib.backends.backend_wxagg
import matplotlib.backends.backend_wx
import matplotlib.cm

import time
import misc
#---------------------------------------------------------------------------

class TabPanel(wx.Panel):
    def __init__(self, parent):
        wx.Panel.__init__(self, parent=parent, id=wx.ID_ANY)

class MainTabPanel(wx.Panel):
    def __init__(self, parent, robotIds, robotCmd):
        wx.Panel.__init__(self, parent, id=wx.ID_ANY, size=(650,520))

        self.startButton = wx.Button(self, wx.ID_ANY, "Start CBPAE")
        self.startButton.SetPosition((10, 10))

        self.stopButton = wx.Button(self, wx.ID_ANY, "Stop CBPAE")
        self.stopButton.SetPosition((110, 10))

        self.Bind(wx.EVT_BUTTON, self.OnStop, self.stopButton)
        self.Bind(wx.EVT_BUTTON, self.OnStart, self.startButton)

        self.robotIds = robotIds
        # robotCmdQ: message: 0 = start, 1 = normal stop
        self.robotCmd = robotCmd

    def OnStop(self, event):
        for rId in (self.robotIds):
            misc.sendMsg(self.robotCmd[rId], None, 1)
            time.sleep(0.1)

    def OnStart(self, event):
        for rId in (self.robotIds):
            misc.sendMsg(self.robotCmd[rId], None, 0)
            time.sleep(0.1)

class PlotTabPanel(wx.Panel):
    def __init__(self, parent, wsInfo, robotInfo):
        wx.Panel.__init__(self, parent, id=wx.ID_ANY, size=(650,520))
        self.wsInfo = wsInfo
        self.robotInfo = robotInfo

        self.xMin = self.wsInfo[1][0]
        self.xMax = self.wsInfo[1][1]
        self.yMin = self.wsInfo[1][2]
        self.yMax = self.wsInfo[1][3]

        figLen = misc.rescale(self.xMin, self.xMax, 0.0, 8.0, (self.xMax - self.xMin))
        figWid = misc.rescale(self.yMin, self.yMax, 0.0, 8.0, (self.yMax - self.yMin))
        self.fig = matplotlib.pyplot.Figure((figLen*0.75, figWid*0.75), frameon=True, tight_layout=True)
        self.canvas = matplotlib.backends.backend_wxagg.FigureCanvasWxAgg(self, -1, self.fig)

        self.nRobot = self.wsInfo[0][0]
        self.robotIds = [self.wsInfo[2][i][0] for i in (range(self.nRobot))]
        self.nTask = self.wsInfo[0][1]
        self.taskIds = [self.wsInfo[3][j][0] for j in (range(self.nTask))]

        self.robotX = {} # list of x positions of robots
        self.robotY = {} # list of y positions of robots

        self.robotPos = {} # list of lines corresponding to robot positions -> marked with a circle
        self.robotPath = {} # list of lines corresponding to robot paths -> marked as line

        self.taskXOrg = {}
        self.taskYOrg = {}
        self.taskXFinish = {}
        self.taskYFinish = {}

        self.taskOrg = {}
        self.taskFinish = {}

        for rId in (self.robotIds):
            self.robotX[rId] = [self.wsInfo[2][rId][1]]
            self.robotY[rId] = [self.wsInfo[2][rId][2]]
            self.robotPos[rId] = None
            self.robotPath[rId] = None

        for tId in (self.taskIds):
            self.taskXOrg[tId] = [self.wsInfo[3][tId][1]]
            self.taskYOrg[tId]  = [self.wsInfo[3][tId][2]]
            self.taskXFinish[tId] = [self.wsInfo[3][tId][3]]
            self.taskYFinish[tId] = [self.wsInfo[3][tId][4]]
            self.taskOrg[tId] = None
            self.taskFinish[tId] = None

        self.initPlotData()

        TIMER_ID = wx.NewId()
        self.t = wx.Timer(self, TIMER_ID)
        self.Bind(wx.EVT_TIMER, self.onTimer) # call onTimer when timer event occurs
#        wx.EVT_TIMER(self, TIMER_ID, self.onTimer) # call onTimer when timer event occurs
        self.t.Start(1000)

    def initPlotData(self):
        self.ax = self.fig.add_subplot(111)
        for rId in (self.robotIds):
            self.robotPath[rId], = self.ax.plot(self.robotX[rId], self.robotY[rId])
            self.robotPos[rId], = self.ax.plot(self.robotX[rId][-1:], self.robotY[rId][-1:], "o")

        for tId in (self.taskIds):
            self.taskOrg[tId], = self.ax.plot(self.taskXOrg[tId], self.taskYOrg[tId], "^")
            if (self.wsInfo[3][tId][5] == "delivery"):
                self.taskFinish[tId], = self.ax.plot(self.taskXFinish[tId], self.taskYFinish[tId], "s")
            else:
                self.taskFinish[tId], = [None]

        self.ax.set_xlim(self.xMin, self.xMax)
        self.ax.set_ylim(self.yMin, self.yMax)

    def GetToolBar(self):
        # You will need to override GetToolBar if you are using an
        # unmanaged toolbar in your frame
        return self.toolbar

    def onTimer(self, evt):
        for rId in (self.robotIds):
            time.sleep(0.01)
            infos = misc.recvMsg(self.robotInfo[rId])

            updates = 0
            for info in (infos):
                self.robotX[rId].append(info[0])
                self.robotY[rId].append(info[1])
                updates = 1

            if (updates == 1):
                self.robotPath[rId].set_ydata(self.robotY[rId])  # update the data
                self.robotPath[rId].set_xdata(self.robotX[rId])  # update the data
                self.robotPos[rId].set_ydata(self.robotY[rId][-1:])  # update the data
                self.robotPos[rId].set_xdata(self.robotX[rId][-1:])  # update the data

#        print ("updating")
        self.canvas.draw()

    def onEraseBackground(self, evt):
        # this is supposed to prevent redraw flicker on some X servers...
        pass

class MyFrame(wx.Frame):
    def __init__(self, parent, ID, title, wsInfo, robotInfo, robotCmd):
        self.xMin = wsInfo[1][0]
        self.xMax = wsInfo[1][1]
        self.yMin = wsInfo[1][2]
        self.yMax = wsInfo[1][3]

        frameLen = misc.rescale(self.xMin, self.xMax, 0, 600, (self.xMax-self.xMin)) + 50
        frameWid = misc.rescale(self.yMin, self.yMax, 0, 600, (self.yMax-self.yMin)) + 80

        myStyle     = (wx.MINIMIZE_BOX | wx.SYSTEM_MENU | wx.CAPTION | wx.CLOSE_BOX | wx.CLIP_CHILDREN)
        wx.Frame.__init__(self, parent, ID, title, pos=wx.DefaultPosition, size=(frameLen, frameWid), style=myStyle)

        nRobot = wsInfo[0][0]
        robotIds = [wsInfo[2][i][0] for i in (range(nRobot))]

        panel = wx.Panel(self, -1)
        noteBookStyle = fnb.FNB_VC8|fnb.FNB_NO_X_BUTTON|fnb.FNB_NO_NAV_BUTTONS|fnb.FNB_NODRAG|fnb.FNB_SMART_TABS
        self.noteBook = fnb.FlatNotebook(panel, id=wx.ID_ANY)#, agwStyle=noteBookStyle)
        mainTab     = MainTabPanel(self.noteBook, robotIds, robotCmd)

        self.noteBook.AddPage(mainTab, " Main ")
        plotTab = PlotTabPanel(self.noteBook, wsInfo, robotInfo)
        self.noteBook.AddPage(plotTab, " XY Plot ")

        sizer = wx.BoxSizer(wx.VERTICAL)
        # add the widgets to the sizers
        sizer.Add(self.noteBook, 1, wx.ALL|wx.EXPAND, 5)

        panel.SetSizer(sizer)
        self.Layout()


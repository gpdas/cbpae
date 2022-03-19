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

from base64 import b64decode
from zlib import decompress
from io import BytesIO
#---------------------------------------------------------------------------

data = decompress(b64decode('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'))
stream = BytesIO(bytearray(data)) # just bytes() for py3
image = wx.Image(stream, wx.BITMAP_TYPE_ANY) # wx.ImageFromStream for legacy wx


class TabPanel(wx.Panel):
    def __init__(self, parent):
        wx.Panel.__init__(self, parent=parent, id=wx.ID_ANY)

class MainTabPanel(wx.Panel):
    def __init__(self, parent, robotIds, robotCmd):
        wx.Panel.__init__(self, parent, id=wx.ID_ANY, size=(650,520))

        self.startButton = wx.Button(self, wx.ID_ANY, "Start CBPAE")
        self.startButton.SetPosition((50, 50))

        self.stopButton = wx.Button(self, wx.ID_ANY, "Stop CBPAE")
        self.stopButton.SetPosition((200, 50))

        self.Bind(wx.EVT_BUTTON, self.OnStop, self.stopButton)
        self.Bind(wx.EVT_BUTTON, self.OnStart, self.startButton)

        bitmap = wx.Bitmap(image) # wx.BitmapFromImage for legacy wx
        self.staticBitmap = wx.StaticBitmap(self, wx.ID_ANY, bitmap=bitmap, pos=wx.Point(160, 100))

        self.text = wx.StaticText(self, wx.ID_ANY, label="Consensus Based Parallel Allocation and Execution (CBPAE) Algorithm\n", pos=wx.Point(10,180))

        self.citeTxt = """\n(C) Gautham Das"""
        self.text = wx.StaticText(self, wx.ID_ANY, label=self.citeTxt, pos=wx.Point(10,200))

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

        self.icon = wx.Icon()
        bitmap = wx.Bitmap(image) # wx.BitmapFromImage for legacy wx
        self.icon.CopyFromBitmap(bitmap)
        self.SetIcon(self.icon)

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


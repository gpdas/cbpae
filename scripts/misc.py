## ---------------------------------------------------- ##
# file: misc.py
# desc: miscellaneous functions
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

import math
import Queue

epsilon = 0.000001

def eq(a,b):
    aInf = (a == float("inf"))
    bInf = (b == float("inf"))
    if (aInf and bInf):
        return True
    elif not (aInf or bInf): # (not aInf) and (not bInf)
        if (abs(a-b) <= epsilon):
            return True
        else:
            return False
    else: # aInf or bInf
        return False

def gt(a, b):
    if ((a-b) > epsilon):
        return True
    else:
        return False

def lt(a, b):
    if ((b-a) > epsilon):
        return True
    else:
        return False

def gte(a, b):
    if (gt(a,b) or eq(a,b)):
        return True
    else:
        return False

def lte(a, b):
    if (lt(a,b) or eq(a,b)):
        return True
    else:
        return False

def rescale(a, b, A, B, x):
    # rescale x from range [a,b] to [A,B]
    if ((B-A) != 0):
        return ((x-a)*float((B-A))/(b-a))+A
    else:
        return float("+inf")

def getDist(x1, y1, x2, y2):
    return math.sqrt((x1-x2)**2+(y1-y2)**2)

def sign(x):
    # takes a value x and returns -1, 0, +1
    if eq(x,0): return 0
    elif lt(x,0): return -1
    elif gt(x,0): return 1

def sendMsg(msgQ, msgLock, msg):
    lockStat     = False
    while (True):
        if (msgLock != None):
            try:
                lockStat = msgLock.acquire(0)
            except:
                pass
            else:
                if (lockStat):
                    break
        else:
            break

    while (True):
        try:
            msgQ.put_nowait(msg)
        except:
            pass
        else:
            break

    if (lockStat):
        msgLock.release()

def recvMsg(msgQ):
    msgData = []
    while (True):
        try:
            msg = msgQ.get_nowait()
        except Queue.Empty:
            break
        else:
            msgData.append(msg)
    return msgData

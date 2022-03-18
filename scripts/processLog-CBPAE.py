#!/usr/bin/env python2
## ---------------------------------------------------- ##
# file: processLog-CBPAE.py
# desc: reads log files and extracts some metrics from those
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

import os

dirs = []
totDist = []
avgDist = []
avgActDist = []
timeExec = []
minResponse = []
maxResponse = []
avgResponse = []
minExec = []
maxExec = []
avgExec = []

#sets = [1,2,3,4]
sets = [1, 2, 3]
for idx in range(len(sets)):
    currSet = sets[idx]
    currDir = os.getcwd()
    dirs.append(currDir+"/set %d/" %(currSet))

r = [3]
t = [3, 6, 9]
#r = [10, 20, 30]
#t = [10, 20, 30, 40, 50, 60]

nR = len(r)
nT = len(t)

for rIdx in range(nR):
    totDist.append([])
    avgDist.append([])
    avgActDist.append([])
    timeExec.append([])
    minResponse.append([])
    maxResponse.append([])
    avgResponse.append([])
    minExec.append([])
    maxExec.append([])
    avgExec.append([])
    for tIdx in range(nT):
        totDist[-1].append([])
        avgDist[-1].append([])
        avgActDist[-1].append([])
        timeExec[-1].append([])
        minResponse[-1].append([])
        maxResponse[-1].append([])
        avgResponse[-1].append([])
        minExec[-1].append([])
        maxExec[-1].append([])
        avgExec[-1].append([])
        fName = "%d_%d_rooms_hetero_diffprio_randmstart_cbpae.log" %(r[rIdx], t[tIdx])
        for idx in range (len(sets)):
            setDir = dirs[idx]
            fHandle = open(setDir+fName, "r")
            while True:
                try:
                    data = fHandle.readline().strip().strip(" ").split(",")
                except:
                    break
                else:
                    if (len(data) == 1) and (data[0] == ""):
                        break
                    if (len(data) == 1):
                        data = data[0].split("=")
                        if (len(data) != 1):
                            if (data[0].strip() == "totalDist"):
                                totDist[-1][-1].append(float(data[1].strip()))
                            elif (data[0].strip() == "avgDist"):
                                avgDist[-1][-1].append(float(data[1].strip()))
                            elif (data[0].strip() == "avgDistAct"):
                                avgActDist[-1][-1].append(float(data[1].strip()))
                            elif (data[0].strip() == "Total time of execution"):
                                timeExec[-1][-1].append(float(data[1].strip()))
                            elif (data[0].strip() == "Minimum Response Time"):
                                minResponse[-1][-1].append(float(data[1].strip()))
                            elif (data[0].strip() == "Maximum Response Time"):
                                maxResponse[-1][-1].append(float(data[1].strip()))
                            elif (data[0].strip() == "Average Response Time"):
                                avgResponse[-1][-1].append(float(data[1].strip()))
                            elif (data[0].strip() == "Minimum Execution Time"):
                                minExec[-1][-1].append(float(data[1].strip()))
                            elif (data[0].strip() == "Maximum Execution Time"):
                                maxExec[-1][-1].append(float(data[1].strip()))
                            elif (data[0].strip() == "Average Execution Time"):
                                avgExec[-1][-1].append(float(data[1].strip()))

            fHandle.close()
            print ("done r=%d t=%d set=%d" %(r[rIdx], t[tIdx], sets[idx]))
#print totDist
#print avgDist
#print avgActDist
#print timeExec

fDist = open("cbpae_dist.csv","w")
fTime = open("cbpae_time.csv","w")
fETime = open("cbpae_emergency_time.csv","w")
newStr1 = "nr, nt, "
for idx in range(len(sets)):
    newStr1 += "set %d, " %(sets[idx])
newStr1 += "min, max, avg, "
print (newStr1[:-2], file=fTime)

for idx in range(len(sets)):
    newStr1 += "set %d, " %(sets[idx])
newStr1 += "min, max, avg, "
for idx in range(len(sets)):
    newStr1 += "set %d, " %(sets[idx])
newStr1 += "min, max, avg"
print (newStr1, file=fDist)

newStr11 = "nr, nt, min, max, avg, min, max, avg"
print (newStr11, file=fETime)

for rIdx in range (nR):
    for tIdx in range (nT):
        newStr2 = "%d, %d, " %(r[rIdx], t[tIdx])
        for idx in range(len(sets)):
            newStr2 += "%0.3f, " %(totDist[rIdx][tIdx][idx])
        newStr2 += "%0.3f, " %(min(totDist[rIdx][tIdx]))
        newStr2 += "%0.3f, " %(max(totDist[rIdx][tIdx]))
        newStr2 += "%0.3f, " %(sum(totDist[rIdx][tIdx])/len(sets))
        for idx in range(len(sets)):
            newStr2 += "%0.3f, " %(avgDist[rIdx][tIdx][idx])
        newStr2 += "%0.3f, " %(min(avgDist[rIdx][tIdx]))
        newStr2 += "%0.3f, " %(max(avgDist[rIdx][tIdx]))
        newStr2 += "%0.3f, " %(sum(avgDist[rIdx][tIdx])/len(sets))
        for idx in range(len(sets)):
            newStr2 += "%0.3f, " %(avgActDist[rIdx][tIdx][idx])
        newStr2 += "%0.3f, " %(min(avgActDist[rIdx][tIdx]))
        newStr2 += "%0.3f, " %(max(avgActDist[rIdx][tIdx]))
        newStr2 += "%0.3f " %(sum(avgActDist[rIdx][tIdx])/len(sets))
        print (newStr2, file=fDist)

        newStr3 = "%d, %d, " %(r[rIdx], t[tIdx])
        for idx in range(len(sets)):
            newStr3 += "%0.3f, " %(timeExec[rIdx][tIdx][idx])
        newStr3 += "%0.3f, " %(min(timeExec[rIdx][tIdx]))
        newStr3 += "%0.3f, " %(max(timeExec[rIdx][tIdx]))
        newStr3 += "%0.3f" %(sum(timeExec[rIdx][tIdx])/len(sets))
        print (newStr3, file=fTime)

        newStr4 = "%d, %d, " %(r[rIdx], t[tIdx])
        newStr4 += "%0.3f, " %(min(minResponse[rIdx][tIdx]))
        newStr4 += "%0.3f, " %(max(maxResponse[rIdx][tIdx]))
        newStr4 += "%0.3f, " %(sum(avgResponse[rIdx][tIdx])/len(sets))
        newStr4 += "%0.3f, " %(min(minExec[rIdx][tIdx]))
        newStr4 += "%0.3f, " %(max(maxExec[rIdx][tIdx]))
        newStr4 += "%0.3f" %(sum(avgExec[rIdx][tIdx])/len(sets))
        print (newStr4, file=fETime)

fDist.close()
fTime.close()
fETime.close()

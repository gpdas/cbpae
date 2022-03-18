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
        
        test = cbpae.Cbpae(fName, logsDir)
        test.run()
        if (LOGGING):
            test.plotData(logsDir, plotsDir, fName)
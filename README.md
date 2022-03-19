# Consensus Based Parallel Allocation and Execution (CBPAE)

## Version 6.00.00

A multi-robot task allocation (MRTA) algorithm for ST-SR-IA with
  * Heterogeneous robots and tasks
  * Prioritised tasks
  * Task drop functionality to attend a new emergency-priority task
  * Simple inter-robot communication protocol
  * Skills and expertise based bidding
  * Parallel allocation and execution (PAE) - suitable for dynamic environments
  * Only Python3 is supported
  * ROS Noetic and above 

Each robot is contained in a separate process and the robots communicate using inter-process communication (implemented with a centralised communciation manager).

## Types of robots:
  * abstract: These robots are defined as an abstract type for Monte-Carlo simulations
  * ros: These are now assumed to be turtlebot3 burger robots (either simulated or physical) with ros communications to drive them

## Input data:
Some sample input data are available.
  * data/abstract_openspace.txt: An open environment with two abstract robots and some tasks.
  * data/abstract_rooms.txt: An indoor environment with two abstract robots and some tasks.
  * data/ros_openspace.txt: An open environment with two ROS robots (assumed to be turtlebots3-Burger) and some tasks.

More such input data files can be create using `scripts/createData.py`

## Log of trials

Logs are stored in `$HOME/cbpae/logs/`.

Plots are generated in `$HOME/cbpae/plots/` from the logs.

## How to run examples
Python script: 
In a terminal go to the cbpae directory and run the following
```
python scripts/cbpae.py data/abstract_openspace.txt
```

ROS node:
Clone the repository to a catkin workspace and `catkin_make` it. In a terminal run the following :
```
python -m pip install tmule
roscd cbpae/config/tmule
tmule -c cbpae_turtles.yaml launch
rosrun cbpae cbpae_node.py $(rospack find cbpae)/data/ros_openspace.txt
```

To close, click `Stop CBPAE` button in the GUI and close the application window.

## Citation

To cite this algorithm use the citation below.
```
@article{Das2015,
author = {Das, Gautham P. and Mcginnity, Thomas M. and Coleman, Sonya A. and Behera, Laxmidhar},
isbn = {4428713754},
journal = {Journal of Intelligent and Robotic Systems},
keywords = {ambient assisted living,distributed task allocation,market based task,multi-robot,multi-robot task allocation,robots in healthcare facilities,systems},
number = {1},
pages = {33--58},
title = {{A Distributed Task Allocation Algorithm for a Multi-Robot System in Healthcare Facilities}},
volume = {80},
year = {2015}
}
```


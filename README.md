# Consensus Based Parallel Allocation and Execution (CBPAE)

Version 5.00.02

A multi-robot task allocation (MRTA) algorithm for ST-SR-IA with
  * task priorities
  * task drop to attend an emergency priority task
  * simple inter-robot communication

Each robot is contained in a separate process and the robots communicate using inter-process communication (implemented with a centralised communciation manager).

The robots are defined as an abstract type for Monte-Carlo simulations

Although not included, this abstract class was originally extended to work with  
  * Pioneer3-Dx robot simulated in Stage controlled using Player
  * Physical Pioneer3-Dx robot controlled using Player 
  * Physical Pioneer3-Dx robot controlled using ROS

A sample input data file for the experiment in `data/testing_inData.dat` (this is a simple text file).

Create more data files using `scripts/createData.py`

Modify `cbpae.py` with the input data file details and run using `python2`.

Logs are stored in `logs/`.

Plots are generated in `plots/` from the logs.

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


# Automatic-control-of-passenger-ferry-with-risk-contingencies
This repository contains the Matlab scripts that make up the proposed automatic control system in the Master thesis of Jon Magnus Moen, written in the spring of 2020. It also contains the scripts for running the simulations to test the system. 


# Dependencies
Written and tested in Matlab R2021a. Some of the functions used are copied from the [mss toolbox](https://github.com/cybergalactic/MSS), and some are copied from the master thesis of Elias Gauslaa. The following Matlab toolboxes are used:
- Optimization toolbox
- Mapping toolbox
- Image processing toolbox

# Structure
Each subsystem (guidance, control, supervisor) has its own folder containing the functions. Each simulation has its own folder, where running the *initsimX.m* script will start the simulation and plot the results. 

# Contributors
Jon Magnus Moen

### This repository contains web-based GUI for Coverage Path Planning alongside with some scropts for automatic testing
#### IMPORTANT: the file ```templates/map.html``` contains a private Google API key, so it should be removed if repo is made public or so

## Installation
For proper installation, do following steps:
* Ensure you have ```Python 3``` and ```ROS``` installed
* Install requirements by running ```$ pip install -r requirements.txt```
* Install [```mrs_msgs```](https://github.com/ctu-mrs/mrs_msgs) ROS package
* Install ROS package from [here](https://github.com/ddatsko/thesis_ccp_grnerator_nodelet) as a normal package
* Clone ros package for trajectory generation from [here](https://github.com/ddatsko/toppra_trajectory_generation) to somewhere (it may be your workspace if you plan 
to use it as a ROS package) and make a symlink to it by running ```$ ln -s <global_path_to_package> scripts/toppra_trajectory_generation```
* If you want to use planner from [here](https://github.com/savvas-ap/mCPP-optimized-DARP), download the ```mCPP-optimized-DARP.jar``` flie from their repository and 
place it into ```scripts/trajectory_planners``` directory
* If you want to use [GTSP planner from here](https://github.com/ethz-asl/polygon_coverage_planning), install that ROS package by yourself 
and run the ```$ roslaunch polygon_coverage_ros coverage_planner.launch``` together with each run of the GUI.

## Planners and their usage
The GUI supports woking with 4 different planners. Each has some differences in the process of working with them:
* ```Own``` planner installed from [here](https://github.com/ddatsko/thesis_ccp_grnerator_nodelet). This one ```MUST``` always be present  for the GUI to work properly.
All the values in GUI will be passed to this planner as it was mainly made for it.
* ```mCPP optimized``` from [here](https://github.com/savvas-ap/mCPP-optimized-DARP). For this one you just need to downlaod a file described above.
Sweeping step and number of UAVs together will all the Area Of Interest will be passed to it.
* ```POPCORN```. Planner that uses python ```wadl-planner``` package included into ```requirements.txt```. Only sweeping step and fly-zone will be passed to it.
No no-fly-zones or predefined number of UAVs are supported by it.
* ```GTSP``` planner from [here](https://github.com/ethz-asl/polygon_coverage_planning). It plans for only one UAV and sweeping step is passed to ROS node as a
parameter, not through the GUI


## Usage
To run the GUI, follow these steps:
* [optinal] Run ```$ roscore```
* Launch own planner ```$ roslaunch thesis_path_generator thesis_path_generator.launch``` (and change parameters here or in config files)
* [optinal] run GTSP planner ```$ roslaunch polygon_coverage_ros coverage_planner.launch```
* Run the GUI ```$ python3 app.py```

This will start the server on ```localhost:5000```

## Working instructions
### Drawing on map
To draw on map, there are 3 buttons in the top left corner of the map, two colorful squares in the top right corner and 3 buttons below them.
Green color means fly-zone and red one means no-fly-zone.

To draw a polygon, click on polygon button in the left top corner, then select color (optinally) and place points on map.
You can then adjust each point of it or add more points. 
If you want to change the color of a polygon, just select it by clicking on it and select the needed color in the top right corner.

To place the UAVs initial position, select the pin button in the left top corner of the map and place the pin on map.

For deleting current (selected) or all polygons, just use buttond in the top right corner. 

### Path planning
To plan some paths, these conditions must be satisfied:
* ```Save config``` tick is ***NOT ticked***
* There is one (and only one) fly-zone on the map
* There is one (and only one) starting point for UAVs on the map
* All the fields on top of the right panel (above special boxes) are filled in
* If a tick is ticked in any special box, each value insied is filled in

If it is true, click the ```Generate trajectories``` button and wait until some error or paths are displayed.


## Configuration saving and loading
The application supports storing configurations and loading them afterwards.
To store a configuration, just type its name into ```Experiment name``` field and click ```Generate trajectories button```. 
This currently shows an error, but the configuration is actually stored in the disk.

To load a configuration, just write its name into ```Directory path``` field and click ```Load polygons``` button.

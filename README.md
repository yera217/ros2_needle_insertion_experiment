# Needle Insertion Experiment Setup in ROS 2
System for ROS 2 needle insertion experiment at JHU for automatic shape-sensing FBG Needle insertion

For System architecture, go to the [System Architecture](docs/index.md) page.

## Package Contents
* `sm130_interrogator_cpp`: C++ implementation of sm130 Micron Optics FBG Interrogator (Not in use and not fully implemented)
* `sm130_interrogator_py`: Python implementatin of sm130 Micron Optics FBG Interrogator (In use and functional)
* `needle_insertion_robot`: Galil Motion Controller Robot for Hardware implementation of 4-axis needle insertion robot
* `needle_insertion_robot_translation`: `needle_insertion_robot` ROS 2 wrapper for state publishing and implementation of action server for out-of-plane control.
* `needle_shape_publisher`: Python implementation of needle shape-sensing package
* `insertion_point_publisher`: Python publisher for static publishing of needle insertion point. This is for system integration, primarily.

## Launching System
### Entire System
To launch the entire system, use:
```bash
  ros2 launch needle_insertion_experiment system.launch.py \
       sim_level_needle_sensing:=<1 - demo sensing, 2 - real hardware sensing, Default=1> \
       interrogator_ipAddress:=<IP address of FBG Interrogator, Default=192.168.1.11> \
       robot_ipAddress:=<IP Address of Galil controller, Default=192.168.1.201> \
       needle_needleParamFile:=<Needle param JSON file w.r.t. needle_shape_publisher share/config file, Default=3CH-4AA best calibration JSON file> \
```
Launching will include:
1. Needle Insertino Robot
2. Needle Insertion Robot Translation Node
3. Needle Insertion Robot Translation Action Server
4. FBG Interrogator
5. Sensorized Shape-Sensing Needle
6. Insertion Point Publisher

### Robotic Insertion Stage
Launching the robotic insertion stage is as follows:
```bash
  ros2 launch needle_insertion_experiment stage.launch.py robot_ipAddress:=<IP Address of Galil Controller>
```
This will launch the following in the same ROS 2 namespace (by default: `/stage`)
1. Needle Insertion Robot
2. Needle Insertion Robot Translation Node (For system integration)
3. Needle Insertion Robot Translation Action Server (For system Integration) 

### Needle and FBG Interrogator
Launching the shape-sensing needle can be performed without using any arguments with default arguments. 
Default needle used is a 3 CH, 4 AA needle primarily used needle insertion experiments at this point. 
```bash
  ros2 launch needle_insertion_experiment needle.launch.py
```
With arguments: 
```
  ros2 launch needle_insertion_experiment needle.launch.py \
      sim_level_needle_sensing:=<1,2, Default=1> \
      needleParamFile:=<JSON parameter file in share lib, Default=3CH-4AA-0004 best calibration> \
      numSignals:=<Value greater than 0, Default=200> \ 
      optimMaxIteationsr:=< Value greater than 1, Default=15> \
      interrogatorIP:=<IP address of interrogator, Default=192.168.1.11> \
      interrogatorParamFile:=<YAML param file in shapre lib, default=None>
```
Arguments are as follows
* `sim_level_needle_sensing`: 1 for demo needle shape publishing, 2 for real hardware interfaces. This path is relative to `needle_shape_publisher` `share` folder.
* `needleParamFile`: JSON parameter file for needle shape-sensing package to load shape-sensing needle.
* `numSignals`: Number of signals to perform calibration of signals and to keep on a moving average for curvature windowing.
* `optimMaxIteations`: Number of iterations for optimization to stop optimizing. Lower is faster, higher is more accurate
* `interrogatorIP`: IP address of the sm130 interrogator. 
* `interrogatorParamFile`: ROS 2 parameter file to override other ROS 2 parameters for sm130 interrogator. This path is relative to `sm130_interrogator_py` `share` folder.

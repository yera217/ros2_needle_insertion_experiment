# Needle Insertion Experiment Setup in ROS 2
System for ROS 2 needle insertion experiment at JHU for automatic shape-sensing FBG Needle insertion

## Package Contents
* `sm130_interrogator_cpp`: C++ implementation of sm130 Micron Optics FBG Interrogator (Not in use and not fully implemented)
* `sm130_interrogator_py`: Python implementatin of sm130 Micron Optics FBG Interrogator (In use and functional)
* `needle_insertion_robot`: Galil Motion Controller Robot for Hardware implementation of 4-axis needle insertion robot
* `needle_shape_publisher`: Python implementation of needle shape-sensing package

## Launching System
### Entire System
***TODO***

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

# IDMindUR5

This package contains ROS (Kinetic) drivers for the UR5 arm electronic solution built by IDMind

## Getting Started

Copy the package to an existing catkin workspace and run
```
catkin_make
```

After sourcing the environment, edit the ur5's device name in the configuration file
```
roscd ur5_driver/cfg
nano ur5.yaml
```

Replace /dev/ttyUSB0 by your device name

Launch the program
```
roslaunch ur5_driver idmind_ur5.launch
```

You can inspect the system status and power status via ROS Topics
```
rostopic echo /idmind_ur5/system_power_supply
rostopic echo /idmind_ur5/system_status
```

You can turn the robot pc on / off via a call to a ROS Service
```
rosservice call /idmind_ur5/enable_ur5 true
```

## Project Overview

Under the ur5_driver folder you will find all configuration, launch and source files.

The source files include node.py, which defines the ROS API, hw_api.py which defines the functions used to communicate with the board's firmware and idmind_serial.py, an auxiliary library for serial communication in python.

The node itself, publishes board status at a rate of 20hz, but only if communication with the board was successfull. If you see the rate of communication dropping too far below 20, you may use the ROS Service /idmind_ur5/reset_connection [std_srvs/Trigger] to attemp to reset the connection with the board.

# ATRV-Jr Setup
Last updated: November 15th, 2018

Author: David Dias, david.miguel.dias@tecnico.ulisboa.pt

## Dependecies
- Install [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) in Ubuntu 16.04.1

- Install ros-kinetic-joy
```
sudo apt-get install ros-kinetic-joy
```

- Install [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/installing.html).

## Setup
- Download the source code from [here](https://ulisboa-my.sharepoint.com/:f:/g/personal/ist427819_tecnico_ulisboa_pt/EtZ25H6jws9AoIFnfIbiAOUBGHngC-VyANbBK3PJDl6fqw?e=ky9hN2).

- Extract it.
```
tar xf ros_ws.tar.gz -C ~/
```

- Compile it.
```
cd ~/ros_ws/
catkin build
```

- Source it.
```
source ~/ros_ws/devel/setup.bash
```

- Add MPU-6050 udev rules.
```
sudo cp ~/ros_ws/src/atrvjr/drivers/mpu6050_serial_to_imu/udev/10-local.rules /etc/udev/rules.d
sudo udevadm control --reload-rules && udevadm trigger
```

## Running
1. Plug the USB hub cable to the PC.

2. Plug the remote controller to the PC.

3. Give the permitions to the serial ports.
```
sudo chmod 666 /dev/ttyUSB*
```

- Inside (without GPS)
```
roslaunch atrvjr atrvjr_hw_no_gps.launch
```

- Outside (with GPS)
```
roslaunch atrvjr atrvjr_hw.launch
```

Good luck

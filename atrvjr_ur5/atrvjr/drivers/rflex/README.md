# rflex
Drivers for ATRV-Jr rflex board. This is a catkinized version of [uml-robotics](https://github.com/uml-robotics/atrvjr_robot) rflex drivers for the ATRV-Jr developed by David V. Lu and Mikhail Medvedev. Tested on ROS Kinetic.

## Getting Started

Communication with the rflex board is done through RS-232, so you will need a compatible serial adaptor.

Clone rflex package to your catkin workspace and compile it.

Connect the adaptor and check the correspondent serial port, it usualy is /dev/ttyUSB0. You can check serial ports by running:
```
dmesg | grep tty
```
If it is a different one change the serialPort argument in rflex/launch/rflex.launch.

Give the following permissions to your serial port by running:
```
sudo chmod 666 /dev/ttyUSB0
```
As a permanent alternative it is possible to set udev rules for your serial adaptor.
## Authors

* **David V. Lu, Mikhail Medvedev**

## License

This project is licensed under the GPLv2 License - see the [LICENSE.md](LICENSE.md) file for details

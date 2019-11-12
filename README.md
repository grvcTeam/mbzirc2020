# mbzirc2020

Catkin repository to store ROS packages for the MBZIRC2020

## Dependencies

A list of required ROS default packages are provided in the root of this repo ([requirements.txt](requirements.txt)). For installing them:
```
sudo apt-get install $(cat requirements.txt)
```

Additionally:
```
sudo pip install crcmod
```

Be sure of having an updated **master**-branch clone of:
* [grvc-ual](https://github.com/grvcTeam/grvc-ual)
* [grvc-utils](https://github.com/grvcTeam/grvc-utils)
* [atrv_ur5e](https://github.com/joaocabogon/atrv_ur5e)


### PCL library
Since Kinetic default PCL (v1.7) library has a bug for converting XYZRGB to XYZHSV pointclouds, **bricks_detection** package requires **PCL 1.9.1** as well as **'pcl_conversions'** and **'pcl_ros'** compiled against that library. 

**WARNING: If you have 'pcl_ros' and 'pcl_conversions' installed by default with ros-kinetic-desktop-full, please remove them from the ROS instalation path ('/opt/ros/kinetic/..') but keep 'pcl_msgs'**

PCL compilation and installation:
```
git clone https://github.com/PointCloudLibrary/pcl.git
cd pcl
git checkout pcl-1.9.1
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/opt/pcl
make
sudo make install
echo "export PCL=/opt/pcl/share/pcl-1.9/" >> ~/.bashrc
source ~/.bashrc
```

Then remove 'build' and 'devel' folders and compile the workspace. 'pcl_conversions' and 'pcl_ros' included manually in the repo will compile against the new PCL library.

## Compilation

For improving code performance and running speed, remember compiling in RELEASE mode:

```
catkin_make -DCMAKE_BUILD_TYPE=Release
```

## Run instructions

### Challenge 1 [UNCOMPLETED]:
```
roslaunch mbzirc_launchers c1.launch
```

For visualization:
```
roslaunch mbzirc_visualization c1.launch
```

---
### Challenge 2
```
roslaunch mbzirc_launchers c2.launch
rosrun robot_tasks c2_dispatcher_coop.py
```  

For visualization:
```
roslaunch mbzirc_visualization c2.launch
```

#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# for your packages to be recognized by python
d = generate_distutils_setup(
 packages=['mbot_moveit_planning_scene_ros'],
 package_dir={'mbot_moveit_planning_scene_ros': 'ros/src/mbot_moveit_planning_scene_ros'}
)

setup(**d)

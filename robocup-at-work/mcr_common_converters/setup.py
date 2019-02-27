#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['mcr_common_converters_ros'],
    package_dir={'mcr_common_converters_ros': 'ros/src/mcr_common_converters_ros'}
)

setup(**d)

cmake_minimum_required(VERSION 2.8)
project(pcl_ros)

add_compile_options(-std=c++14)

## Find catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  message_filters
  nodelet
  nodelet_topic_tools
  pcl_conversions
  pcl_msgs
  pluginlib
  rosbag
  rosconsole
  roscpp
  roslib
  sensor_msgs
  std_msgs
  tf
  tf2
  tf2_eigen
  tf2_ros
)

## Find system dependencies
find_package(cmake_modules REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)
find_package(Eigen3 REQUIRED)

if(DEFINED ENV{PCL})
   set(CMAKE_PREFIX_PATH   ${CMAKE_PREFIX_PATH} $ENV{PCL})
endif()

find_package(PCL 1.9.1 REQUIRED COMPONENTS core features filters io segmentation surface)

if(NOT "${PCL_LIBRARIES}" STREQUAL "")
  # For debian: https://github.com/ros-perception/perception_pcl/issues/139
  list(REMOVE_ITEM PCL_LIBRARIES
    "vtkGUISupportQt"
    "vtkGUISupportQtOpenGL"
    "vtkGUISupportQtSQL"
    "vtkGUISupportQtWebkit"
    "vtkViewsQt"
    "vtkRenderingQt")
endif()

# There is a bug in the Ubuntu Artful (17.10) version of the VTK package,
# where it includes /usr/include/*-linux-gnu/freetype2 in the include
# directories (which doesn't exist).  This filters down to the PCL_INCLUDE_DIRS,
# and causes downstream projects trying to use these libraries to fail to
# configure properly.  Here we remove those bogus entries so that downstream
# consumers of this package succeed.
if(NOT "${PCL_INCLUDE_DIRS}" STREQUAL "")
  foreach(item ${PCL_INCLUDE_DIRS})
    string(REGEX MATCH "/usr/include/.*-linux-gnu/freetype2" item ${item})
    if(item)
      list(REMOVE_ITEM PCL_INCLUDE_DIRS ${item})
    endif()
  endforeach()
endif()

## Add include directories
include_directories(include)

include_directories(
  SYSTEM
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

## Declare the catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    pcl_ros_tf
  CATKIN_DEPENDS
    dynamic_reconfigure
    geometry_msgs
    message_filters
    nodelet
    nodelet_topic_tools
    pcl_conversions
    pcl_msgs
    rosbag
    roscpp
    sensor_msgs
    std_msgs
    tf
    tf2
    tf2_eigen
    tf2_ros
  DEPENDS
    Boost
    EIGEN3
    PCL
)

## Declare the pcl_ros_tf library
add_library(pcl_ros_tf src/transforms.cpp)
target_link_libraries(pcl_ros_tf ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(pcl_ros_tf ${catkin_EXPORTED_TARGETS})
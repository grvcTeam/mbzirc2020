cmake_minimum_required(VERSION 2.8.3)
project(mbzirc_gazebo_plugins)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  rospy
  std_msgs
  mbzirc_comm_objs
  sensor_msgs
  geometry_msgs
)

find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gazebo_magnetic_gripper
  CATKIN_DEPENDS gazebo_ros roscpp mbzirc_comm_objs sensor_msgs geometry_msgs
  DEPENDS gazebo
)

###########
## Build ##
###########
# check c++11 / c++0x
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler. Suggested solution: update the pkg build-essential ")
endif()

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${GAZEBO_LIBRARY_DIRS}
)

## Declare a cpp library
add_library(magnetic_gripper_plugin SHARED src/magnetic_gripper.cpp)
add_library(fake_object_detection_plugin SHARED src/fake_object_detection.cpp)
add_library(droplet_factory SHARED src/droplet_factory.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(gazebo_grasp_plugin_node gazebo_grasp_plugin_generate_messages_cpp)
add_dependencies(magnetic_gripper_plugin ${catkin_EXPORTED_TARGETS})
add_dependencies(fake_object_detection_plugin ${catkin_EXPORTED_TARGETS})
add_dependencies(droplet_factory ${catkin_EXPORTED_TARGETS})

add_executable(target_drone_trajectory src/target_drone_trajectory.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(magnetic_gripper_plugin
	${catkin_LIBRARIES}
	${GAZEBO_LIBRARIES}
	${Boost_LIBRARIES}
)

target_link_libraries(fake_object_detection_plugin
	${catkin_LIBRARIES}
	${GAZEBO_LIBRARIES}
	${Boost_LIBRARIES}
  RayPlugin
)

target_link_libraries(droplet_factory
	${catkin_LIBRARIES}
	${GAZEBO_LIBRARIES}
	${Boost_LIBRARIES}
)

 target_link_libraries(target_drone_trajectory
   ${catkin_LIBRARIES}
 )

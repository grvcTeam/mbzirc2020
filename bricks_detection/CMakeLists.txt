project(bricks_detection)
cmake_minimum_required(VERSION 3.5.1)

#####################
## Project Options ##
#####################

## Force some variables that could be defined in the command line to be written to cache
option(WARNINGS_ARE_ERRORS "Treat warnings as errors"                                  OFF)
option(WARNINGS_ANSI_ISO   "Issue all the mandatory diagnostics listed in C standard"  ON)
option(WARNINGS_EFFCPP     "Issue all the warnings listed in the book of Scot Meyers"  OFF)

add_compile_options(-std=c++11)
add_compile_options(-Wall)

if(${WARNINGS_ANSI_ISO})
   add_compile_options(-Wcast-align)
   add_compile_options(-Wpedantic)
   add_compile_options(-Wextra)
else()
   add_compile_options(-Wno-narrowing)
   add_compile_options(-Wno-delete-non-virtual-dtor)
   add_compile_options(-Wno-unnamed-type-template-args)
endif()

if(${WARNINGS_ARE_ERRORS})
   add_compile_options(-Werror)
endif()

if(${WARNINGS_EFFCPP})
   add_compile_options(-Weffc++)
endif()

##################################
## Configure CATKIN dependecies ##
##################################

find_package(catkin REQUIRED 
   COMPONENTS
   roscpp
   rospy
   cv_bridge
   sensor_msgs
   std_msgs
   message_filters
   tf
   dynamic_reconfigure
   pcl_conversions
   pcl_ros
   mbzirc_comm_objs)

catkin_python_setup()

#####################
## Other libraries ##
#####################

if(DEFINED ENV{PCL})
   set(CMAKE_PREFIX_PATH   ${CMAKE_PREFIX_PATH} $ENV{PCL})
endif()

find_package(PCL 1.9.1 REQUIRED)

find_package(Boost 1.58.0 REQUIRED)

find_package(OpenCV REQUIRED)

##################################################################
## Declare ROS messages, services, actions and dynamic reconfig ##
##################################################################


generate_dynamic_reconfigure_options(
   cfg/reconfig_filters.cfg
)

###################################
## CATKIN specific configuration ##
###################################

catkin_package(
   INCLUDE_DIRS   include
   LIBRARIES      ${PROJECT_NAME}
   CATKIN_DEPENDS rospy roscpp cv_bridge 
                  tf  
                  dynamic_reconfigure
                  pcl_conversions
                  pcl_ros
                  mbzirc_comm_objs)

###########
## Build ##
###########

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(SYSTEM ${Eigen3_INCLUDE_DIRS})
include_directories(SYSTEM ${PCL_INCLUDE_DIRS})
include_directories(SYSTEM ${BOOST_INCLUDE_DIRS})
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})

## Build a C++ library
file(GLOB_RECURSE SRC_INCS "include/${PROJECT_NAME}/*.h*")
file(GLOB_RECURSE SRC_HDRS "src/*.h*")

add_library(${PROJECT_NAME}_lib
  ${SRC_HDRS} 
  ${SRC_INCS}
  src/bricks_detection_handler.cpp
  src/bricks_detection.cpp
  src/color_filtering.cpp
  src/distance_filtering.cpp
  src/ransac_plane_detection.cpp
  src/shape_detection.cpp
  )
add_dependencies(${PROJECT_NAME}_lib       ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_lib  ${catkin_LIBRARIES} ${Eigen3_LIBRARIES} ${PCL_LIBRARIES} ${BOOST_LIBRARYDIR} ${OpenCV_LIBS})

## Build a C++ executable
add_executable(bricks_detection_node          src/bricks_detection_node.cpp)
target_link_libraries(bricks_detection_node   ${PROJECT_NAME}_lib)

#########################
## Print configuration ##
#########################

message(STATUS)
message(STATUS "------------ General configuration for - ${PROJECT_NAME} ------------")
message(STATUS)
message(STATUS "WARNINGS_ANSI_ISO   = ${WARNINGS_ANSI_ISO}")
message(STATUS "WARNINGS_ARE_ERRORS = ${WARNINGS_ARE_ERRORS}")
message(STATUS "WARNINGS_EFFCPP     = ${WARNINGS_EFFCPP}")
message(STATUS)
message(STATUS "BUILD_SHARED_LIBS   = ${BUILD_SHARED_LIBS}")
message(STATUS)
message(STATUS "-------------------------- Dependencies --------------------------")
message(STATUS)
message(STATUS "PCL                 = ${PCL_VERSION} - ${PCL_LIBRARIES}")
message(STATUS)
message(STATUS "Boost               = ${BOOST_VERSION} - ${BOOST_LIBRARIES}")
message(STATUS)
message(STATUS "OpenCV              = ${OpenCV_VERSION} - ${OpenCV_LIBS}")
message(STATUS)
message(STATUS "Change a value with: cmake -D<Variable>=<Value>")
message(STATUS)

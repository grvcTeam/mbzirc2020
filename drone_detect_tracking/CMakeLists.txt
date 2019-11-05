project(drone_detector_tracking)
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
   cv_bridge
   geometry_msgs
   sensor_msgs
   std_msgs
   message_generation
   message_filters
   roscpp
   image_transport
   dynamic_reconfigure
   eigen_conversions
   tf2_ros)

catkin_python_setup()

find_package(Eigen3)

find_package(OpenCV REQUIRED)

##################################################################
## Declare ROS messages, services, actions and dynamic reconfig ##
##################################################################

generate_dynamic_reconfigure_options(
   cfg/drone_detector_reconfigure.cfg
   cfg/color_detector_reconfigure.cfg
   cfg/moving_object_detector_reconfigure.cfg
 )

###################################
## CATKIN specific configuration ##
###################################

catkin_package(
   INCLUDE_DIRS   include
   LIBRARIES      ${PROJECT_NAME}
   CATKIN_DEPENDS 
   cv_bridge
   geometry_msgs
   sensor_msgs
   std_msgs
   message_generation
   message_filters
   roscpp
   image_transport
   dynamic_reconfigure
   eigen_conversions
   tf2_ros
   DEPENDS eigen3)

###########
## Build ##
###########

include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

## Build a C++ library
file(GLOB_RECURSE SRC_INCS "include/${PROJECT_NAME}/*.h*")
file(GLOB_RECURSE SRC_SRCS "src/*.c*")
file(GLOB_RECURSE SRC_HDRS "src/*.h*")

add_library(${PROJECT_NAME}            ${SRC_SRCS} ${SRC_HDRS} ${SRC_INCS})
add_dependencies(${PROJECT_NAME}       ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES} ${OpenCV_LIBS}) #${Eigen3_LIBRARIES}

## Build a C++ executable
add_executable(${PROJECT_NAME}_node          "nodes/drone_detector_node.cpp")
target_link_libraries(${PROJECT_NAME}_node   ${PROJECT_NAME} ${Eigen3_LIBRARIES})

# add_executable(moving_object_detector_node          "nodes/moving_object_detector_node.cpp")
# target_link_libraries(moving_object_detector_node   ${PROJECT_NAME} ${Eigen3_LIBRARIES})

add_executable(color_detector_node          "nodes/color_detector_node.cpp")
target_link_libraries(color_detector_node   ${PROJECT_NAME} ${Eigen3_LIBRARIES})

add_executable(group_test_node          "nodes/group_test.cpp")
target_link_libraries(group_test_node   ${PROJECT_NAME} ${Eigen3_LIBRARIES} ${OpenCV_LIBS})

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
message(STATUS "Change a value with: cmake -D<Variable>=<Value>")
message(STATUS)
message(STATUS "srcs: ${SRC_SRCS}" )



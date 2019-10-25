cmake_minimum_required(VERSION 2.8)
project(pcl_conversions)

find_package(catkin REQUIRED COMPONENTS)

if(DEFINED ENV{PCL})
   set(CMAKE_PREFIX_PATH   ${CMAKE_PREFIX_PATH} $ENV{PCL})
endif()

find_package(PCL 1.9.1 REQUIRED COMPONENTS common io)
find_package(Eigen3 REQUIRED)

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

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS pcl_msgs roscpp sensor_msgs std_msgs
  DEPENDS EIGEN3 PCL
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

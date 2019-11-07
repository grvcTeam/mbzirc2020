cmake_minimum_required(VERSION 2.8.3)
project(rflex)

find_package(catkin REQUIRED
    roscpp
    tf
    dynamic_reconfigure
)

find_library(USB_LIB usb-1.0)
# Enable C++11
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

generate_dynamic_reconfigure_options(
  cfg/AtrvjrParams.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rflex
  CATKIN_DEPENDS tf
)

## Additional include folders
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  include/rflex
)

## Declare a cpp library
add_library(rflex
  src/rflex_driver.cpp
  src/atrvjr_driver.cpp
  src/b21_driver.cpp
)

## Specify libraries to link a library or executable target against
#target_link_libraries(rflex
#  ${USB_LIB}
#)

### executables
add_executable(atrvjr_node src/atrvjr_node.cpp)

target_link_libraries(atrvjr_node rflex ${catkin_LIBRARIES})

### executables
add_executable(b21_node src/b21_node.cpp)

target_link_libraries(b21_node rflex ${catkin_LIBRARIES})

# make sure configure headers are built before any node using them
add_dependencies(atrvjr_node ${PROJECT_NAME}_gencfg)

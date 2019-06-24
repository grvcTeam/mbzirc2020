cmake_minimum_required(VERSION 2.8.3)
project(host_console)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    serial
    rosgraph_msgs
)

catkin_package(
    LIBRARIES host_console
    CATKIN_DEPENDS roscpp sensor_msgs serial rosgraph_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
)

add_executable(host_console_node src/host_console_node.cpp)

target_link_libraries(host_console_node
    ${catkin_LIBRARIES}
)

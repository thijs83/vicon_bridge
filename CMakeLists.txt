cmake_minimum_required(VERSION 3.3)
project(vicon_bridge)

set(CMAKE_CXX_STANDARD 14)

# Enable compile optimizations
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wno-dev -Wno-deprecated-declarations")
# For debugging use below
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall")

find_package(catkin QUIET)
find_package(ament_cmake QUIET)
if (catkin_FOUND)
    message(STATUS "\n ${PROJECT_NAME} => ROS1 found, building pkg for ROS1 \n")
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ROS1.cmake)
elseif (ament_cmake_FOUND)
    message(STATUS "\n ${PROJECT_NAME} => ROS2 found, building pkg for ROS2 \n")
    include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ROS2.cmake)
else ()
    message("No ROS versions found!!! \n First setup a ROS environment. \n NOT BUILDING, EXITING...")
endif ()



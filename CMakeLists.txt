cmake_minimum_required(VERSION 2.8.3)
project(gpicp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  roslaunch
  cv_bridge
  message_generation
  laser_geometry
  sensor_msgs
  geometry_msgs
  pcl_conversions
  pcl_ros
)


find_package(PCL 1.7 REQUIRED)
find_package(Boost 1.54 REQUIRED)
find_package(OpenCV)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES beginner_tutorials
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

#SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")
SET(CMAKE_CXX_FLAGS "-std=c++11")


set(gpicp_test_source
    gpicp_test/src/gpicp/gpicp.h
    gpicp_test/src/gpicp/gpicp.hpp
    gpicp_test/src/gpicp/gpicp.cpp
)

add_executable(gpicp_test gpicp_test/gpicp_test.cpp ${gpicp_test_source})
target_link_libraries(gpicp_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})


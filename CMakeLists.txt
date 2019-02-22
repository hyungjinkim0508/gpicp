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

SET(CMAKE_CXX_FLAGS "-std=c++11 -O3 ${CMAKE_CXX_FLAGS}")
#SET(CMAKE_CXX_FLAGS "-std=c++11")

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


set(basic_source
    src/gpicp/gpicp.h
    src/gpicp/gpicp.hpp
    src/gpicp/gpicp.cpp
    src/gpicp/gpicp_omp.cpp
    src/gpicp/gpicp_omp.h
    src/gpicp/gpicp_omp_impl.hpp
)

include_directories(src)
include_directories(src/pcl_omp/include)

add_executable(gpicp_test gpicp_test/gpicp_test.cpp ${basic_source})
target_link_libraries(gpicp_test ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

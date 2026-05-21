cmake_minimum_required(VERSION 2.8.3)
project(livo_backend)

add_definitions(-DNDEBUG)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fexceptions")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -march=native -DNDEBUG")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  sensor_msgs
  roscpp
  rospy
  std_msgs
  tf
  message_generation
)

find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
set(GTSAM_INCLUDE_DIRS ${GTSAM_INCLUDE_DIR})

catkin_package(
  CATKIN_DEPENDS cv_bridge geometry_msgs nav_msgs pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs message_runtime
  DEPENDS EIGEN3 GTSAM PCL OpenCV
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${GTSAM_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
)

add_definitions(-DROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/")

add_executable(livo_backend_node
  src/livo_backend_node.cpp
)

target_link_libraries(livo_backend_node
  ${catkin_LIBRARIES}
  ${GTSAM_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  gtsam
  gtsam_unstable
)

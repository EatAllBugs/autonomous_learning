cmake_minimum_required(VERSION 2.8.3)
project(Lidar_process)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")


find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  roscpp
  rosbag
  std_msgs
  pcl_ros
  jsk_recognition_msgs
)

find_package(Eigen3 REQUIRED)

find_package(PCL REQUIRED)
message(STATUS "***** PCL version: ${PCL_VERSION} *****")
####

include_directories(
  include
  ${catkin_INCLUDE_DIRS} 
  ${PCL_INCLUDE_DIRS}
  )

catkin_package(
  CATKIN_DEPENDS  roscpp  std_msgs sensor_msgs rosbag pcl_ros jsk_recognition_msgs
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS include
)

add_executable(cluster_node src/euclidean_cluster_node.cpp src/euclidean_cluster_core.cpp)
add_executable(ground_filter_node src/plane_ground_filter_node.cpp src/plane_ground_filter_core.cpp)


target_link_libraries(cluster_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(ground_filter_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})


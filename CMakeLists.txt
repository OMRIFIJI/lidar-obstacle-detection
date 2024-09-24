cmake_minimum_required(VERSION 3.0.2)
project(lidar_obstacle_detection)

add_compile_options(-std=c++14 -Wall)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    pcl_ros
    std_msgs
)

catkin_package(
    INCLUDE_DIRS include
)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(include)    
include_directories(${catkin_INCLUDE_DIRS})

add_executable(
    lidar_obstacle_detection_node
    src/lidar_obstacle_detection/lidar_obstacle_detection_node.cpp
    src/lidar_obstacle_detection/lidar_obstacle_detection.cpp
)
target_link_libraries(lidar_obstacle_detection_node ${catkin_LIBRARIES})

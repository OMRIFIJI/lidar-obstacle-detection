cmake_minimum_required(VERSION 3.0.2)
project(lidar_obstacle_detection)

add_compile_options(-std=c++11 -Wall)

include_directories(include)    
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    lidar_obstacle_detection
)
catkin_package(
    INCLUDE_DIRS include
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(
    lidar_obstacle_detection_node
    src/lidar_obstacle_detection/lidar_obstacle_detection_node.cpp
    src/lidar_obstacle_detection/lidar_obstacle_detection.cpp
)
target_link_libraries(lidar_obstacle_detection_node ${catkin_LIBRARIES})

install(DIRECTORY include/
    DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
)

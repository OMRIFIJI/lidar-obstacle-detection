# Angle based obstacle detection.

## Overview
This package provides 360-lidar obstacle detection algorithm for ROS noetic.
This algorithm checks local curvature of a surface to detect obstacles in lidar point cloud.
The algorithm considers all slopes on the point cloud and returns points of 
steep hills as obstacle points. Thus this algorithm works bad if there is noise in lidar data. 
This package provides publisher of all obstacles
and publisher of *dangerous obstacles* which are in the sector of circle in front of lidar. 
In the launch file you can define:
* minimal angle of slope required to call it an obstacle `obstacle_angle_thrsh`
* angle of sector in which obstacles will be called *dangerous* `obstacle_sector_angle`
* radius of sector in which obstacles will be called *dangerous* `obstacle_sector_radius`

Rock on the Mars                                                                                     |  Segmented rock
:---------------------------------------------------------------------------------------------------:|:--------------------------------------------------------------------------------------------------:
![Rock on the Mars](https://github.com/user-attachments/assets/72dad9b5-9377-4118-a258-15085c2e03bd) |  ![Segmented rock](https://github.com/user-attachments/assets/f4dce6fe-5031-436b-932b-1a591a3e7606)


## Dependencies.
* Depends on [PCL](https://github.com/PointCloudLibrary/pcl) for point cloud manipulations
* Depends on [perception_pcl](https://github.com/ros-perception/perception_pcl) for conversions 
between ROS and PCL point cloud classes

## Subscribers and publishers.
* This node subscribes to `velodyne_points` for lidar point cloud.
* `obstacle_pub` publishes cloud of detected obstacles. 
* `dangerous_obstacle_pub` publishes subcloud of obstacles defined by rosparams (check launch file). 
* `avoid_obstacles` publishes `True` if *dangerous_obstacle* cloud is not empty. 

## Quick start
* Clone this repository inside your `catkin_ws`
* Run `catkin_make` inside your workspace then source it
* Start your `velodyne_points` node
* Launch this package with `roslaunch --wait lidar_obstacle_detection obstacle_detection.launch`

## Package optimization
Current version was made for demonstration purposes.
If you want to use it to detect obstacles fast in real data.
You probably want to slice segment of lidar cloud in which
obstacles are expected and then pass it to `velodyne_points`.
Thus you can get rid of looking for *dangerous obstacles in sector* which is done
after searching for obstacles in the entirety of point cloud.

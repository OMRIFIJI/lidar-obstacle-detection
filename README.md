# Angle based obstacle detection.

## Dependencies.
* [PCL](https://github.com/PointCloudLibrary/pcl)
* [perception_pcl](https://github.com/ros-perception/perception_pcl)

## Subscribers and publishers.
* This node subscribes to `velodyne_points` for lidar point cloud.
* `obstacle_pub` publishes cloud of detected obstacles. 
* `dangerous_obstacle_pub` publishes subcloud of obstacles defined by rosparams (check launch file). 
* `avoid_obstacles` publishes `True` if *dangerous_obstacle* cloud is not empty. 

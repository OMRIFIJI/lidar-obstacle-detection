#ifndef LIDAR_OBSTACLE_DETECTION_HPP
#define LIDAR_OBSTACLE_DETECTION_HPP

#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>

#include <sensor_msgs/PointCloud2.h>

typedef pcl::PointXYZ PointT;

class ObstacleDetectionNode {
  public:
	ObstacleDetectionNode(int argc, char **argv);
  
  private:
    ros::Publisher _obstacle_pub;
    ros::Subscriber _lidar_sub;
    void _obstacle_detection_callback(const pcl::PointCloud<PointT>& src_cloud);
};

#endif

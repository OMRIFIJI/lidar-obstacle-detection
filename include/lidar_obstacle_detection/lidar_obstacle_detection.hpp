#ifndef LIDAR_OBSTACLE_DETECTION_HPP
#define LIDAR_OBSTACLE_DETECTION_HPP

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

#include <lidar_obstacle_detection/common.hpp>
#include <lidar_obstacle_detection/cloud_geometry.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <sensor_msgs/PointCloud2.h>


class ObstacleDetectionNode {
  public:
	ObstacleDetectionNode(int argc, char **argv);

  private:
	ros::Publisher _obstacle_pub;
	ros::Subscriber _lidar_sub;

    cloud_geometry::CloudAngleHandler _cloud_angle_handler;
    float _suspect_point_angle;
	float _suspect_point_angle_max;

	void _obstacle_detection_callback(const pcl::PointCloud<PointT> &src_cloud);
	void _find_obstacle_points(const pcl::PointCloud<PointT> &src_cloud);
};

#endif

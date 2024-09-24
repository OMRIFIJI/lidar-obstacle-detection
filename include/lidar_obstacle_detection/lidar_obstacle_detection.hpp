#ifndef LIDAR_OBSTACLE_DETECTION_HPP
#define LIDAR_OBSTACLE_DETECTION_HPP

#include <cstdint>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>

#include <lidar_obstacle_detection/cloud_geometry.hpp>
#include <lidar_obstacle_detection/common.hpp>

#include <pcl/kdtree/kdtree_flann.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>

class ObstacleDetectionNode {
  public:
	ObstacleDetectionNode(int argc, char **argv);

  private:
	ros::Publisher _obstacle_pub;
	ros::Subscriber _lidar_sub;

	float _angle_thrsh;
	cloud_geometry::CloudAngleHandler _cloud_angle_handler;

	std::uint64_t _cloud_ts;
    std::string _lidar_frame_id;
	pcl::PointCloud<PointT>::Ptr _obstacle_points;

	void _obstacle_detection_callback(const pcl::PointCloud<PointT> &src_cloud);
	void
	_find_obstacle_points(const pcl::PointCloud<PointT>::ConstPtr src_cloud);
	void _publish_obstacles();
};

#endif

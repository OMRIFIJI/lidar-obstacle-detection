#ifndef LIDAR_OBSTACLE_DETECTION_HPP
#define LIDAR_OBSTACLE_DETECTION_HPP

#include <cstdint>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <string>

#include <lidar_obstacle_detection/cloud_geometry.hpp>
#include <lidar_obstacle_detection/common.hpp>

#include <pcl/kdtree/kdtree_flann.h>

class ObstacleDetectionNode {
  public:
	ObstacleDetectionNode(ros::NodeHandle node_handle, float angle_thrsh);

  private:
	ros::Subscriber m_lidar_sub;
	ros::Publisher m_obstacle_pub;
	ros::Publisher m_dangerous_obstacle_pub;
	ros::Publisher m_avoid_obstacles_pub;

	// ros params
	float m_obstacle_sector_angle;
	float m_obstacle_sector_radius;

	cloud_geometry::CloudAngleHandler m_cloud_angle_handler;

	std::uint64_t m_cloud_ts;
	std::string m_lidar_frame_id;
	pcl::PointCloud<PointT>::Ptr m_obstacle_points;
	pcl::PointCloud<PointT>::Ptr m_dangerous_obstacle_points;

	pcl::KdTreeFLANN<PointT> m_kdtree;
	std::vector<int> m_pointIdxRadiusSearch;
	std::vector<float> m_pointRadiusSquaredDistance;

	void m_obstacle_detection_callback(const pcl::PointCloud<PointT> &src_cloud);
	void m_find_obstacle_points(const pcl::PointCloud<PointT>::ConstPtr src_cloud);
	void m_filter_dangerous_obstacle_points();
	void m_publish_obstacles();
};

#endif

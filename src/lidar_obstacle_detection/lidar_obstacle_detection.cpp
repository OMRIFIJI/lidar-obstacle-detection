#include <lidar_obstacle_detection/cloud_geometry.hpp>
#include <lidar_obstacle_detection/lidar_obstacle_detection.hpp>
#include <pcl/point_cloud.h>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>

ObstacleDetectionNode::ObstacleDetectionNode(ros::NodeHandle node_handle, float angle_thrsh)
	: m_cloud_angle_handler(angle_thrsh), m_lidar_frame_id("velodyne_base_link"),
	  m_obstacle_points(new pcl::PointCloud<PointT>),
	  m_dangerous_obstacle_points(new pcl::PointCloud<PointT>) {
	m_obstacle_points->header.frame_id = m_lidar_frame_id;
	m_dangerous_obstacle_points->header.frame_id = m_lidar_frame_id;

	node_handle.param<float>("/lidar_obstacle_detection_node/obstacle_sector_angle", m_obstacle_sector_angle, 0.523598);
    node_handle.param<float>("/lidar_obstacle_detection_node/obstacle_sector_radius", m_obstacle_sector_radius, 9);
	std::cout << "Param angle thrsh:" << angle_thrsh << std::endl;
	std::cout << "Param sector angle:" << m_obstacle_sector_angle << std::endl;
	std::cout << "Param sector radius:" << m_obstacle_sector_radius << std::endl;

	m_obstacle_pub = node_handle.advertise<sensor_msgs::PointCloud2>("obstacle_pub", 2);
	m_dangerous_obstacle_pub =
		node_handle.advertise<sensor_msgs::PointCloud2>("dangerous_obstacle_pub", 2);
	m_avoid_obstacles_pub = node_handle.advertise<std_msgs::Bool>("avoid_obstacles", 2);
	m_lidar_sub = node_handle.subscribe("velodyne_points", 2,
									   &ObstacleDetectionNode::m_obstacle_detection_callback, this);
}

void ObstacleDetectionNode::m_obstacle_detection_callback(const pcl::PointCloud<PointT> &src_cloud) {
	pcl::PointCloud<PointT>::ConstPtr src_cloud_shared = src_cloud.makeShared();
	m_cloud_ts = src_cloud.header.stamp;

	this->m_find_obstacle_points(src_cloud_shared);
	this->m_filter_dangerous_obstacle_points();
	std::cout << "All obstacle points detected. Total: " << m_obstacle_points->size() << std::endl;
	std::cout << "Dangerous obstacles: " << m_dangerous_obstacle_points->size() << std::endl;

	this->m_publish_obstacles();

	m_obstacle_points->clear();
	m_dangerous_obstacle_points->clear();
}

void ObstacleDetectionNode::m_find_obstacle_points(pcl::PointCloud<PointT>::ConstPtr src_cloud) {

	m_kdtree.setInputCloud(src_cloud);

	// Neighbour search init
	// Angle values init
	float radius = 2;
	std::cout << "Neighbors search with radius=" << radius << std::endl;

	// Check if point is obstacle
	for (auto &point : *src_cloud) {
		PointT obstacle_suspect = point;

		// Neighbour search and angles calculations
		m_cloud_angle_handler.reset_max_angle();
		if (m_kdtree.radiusSearch(obstacle_suspect, radius, m_pointIdxRadiusSearch,
								 m_pointRadiusSquaredDistance) > 0) {
			for (std::size_t i = 0; i < m_pointIdxRadiusSearch.size(); i++) {
				m_cloud_angle_handler.calculate_angle(obstacle_suspect,
													 src_cloud->points[m_pointIdxRadiusSearch[i]]);
			}
		}

		m_pointIdxRadiusSearch.clear();
		m_pointRadiusSquaredDistance.clear();

		// Check obstacle criteria (angle threshold)
		if (m_cloud_angle_handler.is_above_thrsh()) {
			m_obstacle_points->push_back(obstacle_suspect);
		}
	}
}

void ObstacleDetectionNode::m_filter_dangerous_obstacle_points() {
	// Extracting small segment in front of lidar
	for (auto &point : *m_obstacle_points) {
		if (cloud_geometry::is_inside_sector(point, m_obstacle_sector_angle,
											 m_obstacle_sector_radius)) {
			m_dangerous_obstacle_points->push_back(point);
		}
	}
}

void ObstacleDetectionNode::m_publish_obstacles() {
	std_msgs::Bool should_avoid;
	m_obstacle_points->header.stamp = m_cloud_ts;
	m_dangerous_obstacle_points->header.stamp = m_cloud_ts;

	std::cout << "Publishing obstacles..." << std::endl;
	m_obstacle_pub.publish(m_obstacle_points);
	m_dangerous_obstacle_pub.publish(m_dangerous_obstacle_points);

	should_avoid.data = m_dangerous_obstacle_points->size() > 0 ? true : false;
	std::cout << "Should avoid obstacle? " << (should_avoid.data ? "Yes" : "No") << std::endl
			  << std::endl;
	m_avoid_obstacles_pub.publish(should_avoid);
}

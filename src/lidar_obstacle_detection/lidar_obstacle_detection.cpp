#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include <lidar_obstacle_detection/lidar_obstacle_detection.hpp>

ObstacleDetectionNode::ObstacleDetectionNode(int argc, char **argv)
	: _cloud_angle_handler(0.785398), _lidar_frame_id("velodyne_base_link"),
	  _obstacle_points(new pcl::PointCloud<PointT>) {
	_angle_thrsh = 0.785398;
	ros::init(argc, argv, "lidar_obstacle_detection");
	ros::NodeHandle node_handle;
	_obstacle_pub =
		node_handle.advertise<sensor_msgs::PointCloud2>("obstacle_pub", 2);
	_lidar_sub = node_handle.subscribe(
		"velodyne_points", 2,
		&ObstacleDetectionNode::_obstacle_detection_callback, this);
	ros::spin();
}

void ObstacleDetectionNode::_obstacle_detection_callback(
	const pcl::PointCloud<PointT> &src_cloud) {
	pcl::PointCloud<PointT>::ConstPtr src_cloud_shared = src_cloud.makeShared();
	_cloud_ts = src_cloud.header.stamp;

	this->_find_obstacle_points(src_cloud_shared);

	this->_publish_obstacles();
	_obstacle_points->clear();
}

void ObstacleDetectionNode::_find_obstacle_points(
	pcl::PointCloud<PointT>::ConstPtr src_cloud) {

	pcl::KdTreeFLANN<PointT> kdtree;
	kdtree.setInputCloud(src_cloud);

	// Neighbour search init
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	// Angle values init
	float radius = 2;
	std::cout << "Neighbors search with radius=" << radius << std::endl;

	// Check if point is obstacle
	for (auto &point : *src_cloud) {
		PointT obstacle_suspect = point;

		// Neighbour search and angles calculations
		_cloud_angle_handler.reset_max_angle();
		if (kdtree.radiusSearch(obstacle_suspect, radius, pointIdxRadiusSearch,
								pointRadiusSquaredDistance) > 0) {
			for (std::size_t i = 0; i < pointIdxRadiusSearch.size(); i++) {
				_cloud_angle_handler.calculate_angle(
					obstacle_suspect,
					src_cloud->points[pointIdxRadiusSearch[i]]);
			}
		}

		pointIdxRadiusSearch.clear();
		pointRadiusSquaredDistance.clear();

		// Check obstacle criteria (angle threshold)
		if (_cloud_angle_handler.is_above_thrsh()) {
			_obstacle_points->push_back(obstacle_suspect);
		}
	}
	std::cout << "All obstacle points detected. Total: " << _obstacle_points->size() << std::endl;
}

void ObstacleDetectionNode::_publish_obstacles() {
	_obstacle_points->header.stamp = _cloud_ts;
	_obstacle_points->header.frame_id = _lidar_frame_id;


	std::cout << "Publishing obstacles..." << std::endl << std::endl;
	_obstacle_pub.publish(_obstacle_points);
}

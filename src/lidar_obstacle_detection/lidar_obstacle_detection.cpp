#include <lidar_obstacle_detection/lidar_obstacle_detection.hpp>
#include <memory>

ObstacleDetectionNode::ObstacleDetectionNode(int argc, char **argv) {
	_cloud_angle_handler = cloud_geometry::CloudAngleHandler(0.785398);
	ros::init(argc, argv, "lidar_obstacle_detection");
	ros::NodeHandle node_handle;
	_obstacle_pub =
		node_handle.advertise<sensor_msgs::PointCloud2>("obstacle_pub", 10);
	_lidar_sub = node_handle.subscribe(
		"velodyne_points", 10,
		&ObstacleDetectionNode::_obstacle_detection_callback, this);
	ros::spin();
}

void ObstacleDetectionNode::_obstacle_detection_callback(
	const pcl::PointCloud<PointT> &src_cloud) {
	this->_find_obstacle_points(src_cloud);
}

void _obstacle_detection_callback(const pcl::PointCloud<PointT> &src_cloud) {
	pcl::KdTreeFLANN<PointT> kdtree;
	kdtree.setInputCloud(src_cloud);

	// Neighbour search init
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	// Angle values init

	float radius = 2;
	std::cout << "Neighbors search with radius=" << radius << std::endl;

	// Check if point is obstacle
	for (auto &point : src_cloud) {
		PointT obstacle_suspect = point;

		// Neighbour search
		_cloud_angle_handler.reset_max_angle();
		if (kdtree.radiusSearch(obstacle_suspect, radius, pointIdxRadiusSearch,
								pointRadiusSquaredDistance) > 0) {
			for (std::size_t i = 0; i < pointIdxRadiusSearch.size(); i++) {
				_cloud_angle_handler.
			}
		}

		// Check thrsh
		pointIdxRadiusSearch.clear();
		pointRadiusSquaredDistance.clear();
	}
}

#include <lidar_obstacle_detection/lidar_obstacle_detection.hpp>
#include <memory>

ObstacleDetectionNode::ObstacleDetectionNode(int argc, char **argv) {
	ros::init(argc, argv, "lidar_obstacle_detection");
	ros::NodeHandle node_handle;
	_obstacle_pub = node_handle.advertise<sensor_msgs::PointCloud2>("obstacle_pub", 10);
	_lidar_sub = node_handle.subscribe(
		"velodyne_points", 10,
		&ObstacleDetectionNode::_obstacle_detection_callback, this);
	ros::spin();
}


void ObstacleDetectionNode::_obstacle_detection_callback(const pcl::PointCloud<PointT>& src_cloud){
    
}

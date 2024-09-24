#include <lidar_obstacle_detection/lidar_obstacle_detection.hpp>

ObstacleDetectionNode::ObstacleDetectionNode(int argc, char **argv) {
	ros::init(argc, argv, "lidar_obstacle_detection");
	ros::NodeHandle node_handle;
	ros::Publisher chatter_pub =
		node_handle.advertise<std_msgs::String>("obstacle_pub", 1000);
	ros::spin();
}

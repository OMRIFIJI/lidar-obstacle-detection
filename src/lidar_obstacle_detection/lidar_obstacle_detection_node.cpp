#include <lidar_obstacle_detection/lidar_obstacle_detection.hpp>

int main(int argc, char **argv) {
	ros::init(argc, argv, "lidar_obstacle_detection");
	ros::NodeHandle node_handle;

	float angle_thrsh;
	node_handle.param<float>("/lidar_obstacle_detection_node/obstacle_angle_thrsh", angle_thrsh,
							 0.785398);
	ObstacleDetectionNode obstacle_detection_node = ObstacleDetectionNode(node_handle, angle_thrsh);

	ros::spin();
	return 0;
}

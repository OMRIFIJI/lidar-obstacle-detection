#include <lidar_obstacle_detection/cloud_geometry.hpp>
#include <lidar_obstacle_detection/lidar_obstacle_detection.hpp>
#include <pcl/point_cloud.h>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>

ObstacleDetectionNode::ObstacleDetectionNode(ros::NodeHandle node_handle, float angle_thrsh)
	: _cloud_angle_handler(angle_thrsh), _lidar_frame_id("velodyne_base_link"),
	  _obstacle_points(new pcl::PointCloud<PointT>),
	  _dangerous_obstacle_points(new pcl::PointCloud<PointT>) {
	_obstacle_points->header.frame_id = _lidar_frame_id;
	_dangerous_obstacle_points->header.frame_id = _lidar_frame_id;

    node_handle.param<float>("obstacle_sector_angle",_obstacle_sector_angle, 0.523598);
    node_handle.param<float>("obstacle_sector_radius",_obstacle_sector_radius, 3);
	std::cout << "Param angle thrsh:" << angle_thrsh << std::endl;
	std::cout << "Param sector angle:" << _obstacle_sector_angle << std::endl;
	std::cout << "Param sector radius:" << _obstacle_sector_radius << std::endl;

	_obstacle_pub = node_handle.advertise<sensor_msgs::PointCloud2>("obstacle_pub", 2);
	_dangerous_obstacle_pub = node_handle.advertise<sensor_msgs::PointCloud2>("dangerous_obstacle_pub", 2);
	_avoid_obstacles_pub = node_handle.advertise<std_msgs::Bool>("avoid_obstacles", 2);
	_lidar_sub = node_handle.subscribe("velodyne_points", 2,
									   &ObstacleDetectionNode::_obstacle_detection_callback, this);
}

void ObstacleDetectionNode::_obstacle_detection_callback(const pcl::PointCloud<PointT> &src_cloud) {
	pcl::PointCloud<PointT>::ConstPtr src_cloud_shared = src_cloud.makeShared();
	_cloud_ts = src_cloud.header.stamp;

	this->_find_obstacle_points(src_cloud_shared);
	this->_filter_dangerous_obstacle_points();
	std::cout << "All obstacle points detected. Total: " << _obstacle_points->size() << std::endl;
	std::cout << "Dangerous obstacles: " << _dangerous_obstacle_points->size() << std::endl;

	this->_publish_obstacles();

	_obstacle_points->clear();
	_dangerous_obstacle_points->clear();
}

void ObstacleDetectionNode::_find_obstacle_points(pcl::PointCloud<PointT>::ConstPtr src_cloud) {

	_kdtree.setInputCloud(src_cloud);

	// Neighbour search init

	// Angle values init
	float radius = 2;
	std::cout << "Neighbors search with radius=" << radius << std::endl;

	// Check if point is obstacle
	for (auto &point : *src_cloud) {
		PointT obstacle_suspect = point;

		// Neighbour search and angles calculations
		_cloud_angle_handler.reset_max_angle();
		if (_kdtree.radiusSearch(obstacle_suspect, radius, _pointIdxRadiusSearch,
								 _pointRadiusSquaredDistance) > 0) {
			for (std::size_t i = 0; i < _pointIdxRadiusSearch.size(); i++) {
				_cloud_angle_handler.calculate_angle(obstacle_suspect,
													 src_cloud->points[_pointIdxRadiusSearch[i]]);
			}
		}

		_pointIdxRadiusSearch.clear();
		_pointRadiusSquaredDistance.clear();

		// Check obstacle criteria (angle threshold)
		if (_cloud_angle_handler.is_above_thrsh()) {
			_obstacle_points->push_back(obstacle_suspect);
		}
	}
}

void ObstacleDetectionNode::_filter_dangerous_obstacle_points(){
    // Extracting small segment in front of lidar
    for (auto &point : *_obstacle_points) {
        if(cloud_geometry::is_inside_sector(point, _obstacle_sector_angle, _obstacle_sector_radius)){
            _dangerous_obstacle_points->push_back(point);
        }
    }

}

void ObstacleDetectionNode::_publish_obstacles() {
    std_msgs::Bool should_avoid;
	_obstacle_points->header.stamp = _cloud_ts;
	_dangerous_obstacle_points->header.stamp = _cloud_ts;

	std::cout << "Publishing obstacles..." << std::endl;
	_obstacle_pub.publish(_obstacle_points);
	_dangerous_obstacle_pub.publish(_dangerous_obstacle_points);

    should_avoid.data = _dangerous_obstacle_points->size() > 0 ? true : false;
	std::cout << "Should avoid obstacle? " << (should_avoid.data ? "Yes" : "No") << std::endl << std::endl;
    _avoid_obstacles_pub.publish(should_avoid);
}

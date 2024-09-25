#ifndef LIDAR_OBSTACLE_DETECTION_HPP
#define LIDAR_OBSTACLE_DETECTION_HPP

#include <cstdint>
#include <string>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>

#include <lidar_obstacle_detection/cloud_geometry.hpp>
#include <lidar_obstacle_detection/common.hpp>

#include <pcl/kdtree/kdtree_flann.h>

class ObstacleDetectionNode {
  public:
	ObstacleDetectionNode(ros::NodeHandle node_handle, float angle_thrsh);

  private:
	ros::Subscriber _lidar_sub;
	ros::Publisher _obstacle_pub;
    ros::Publisher _dangerous_obstacle_pub;
	ros::Publisher _avoid_obstacles_pub;
    
    // ros params
    float _obstacle_sector_angle;
    float _obstacle_sector_radius;


	cloud_geometry::CloudAngleHandler _cloud_angle_handler;

	std::uint64_t _cloud_ts;
    std::string _lidar_frame_id;
	pcl::PointCloud<PointT>::Ptr _obstacle_points;
	pcl::PointCloud<PointT>::Ptr _dangerous_obstacle_points;

    
	pcl::KdTreeFLANN<PointT> _kdtree;
	std::vector<int> _pointIdxRadiusSearch;
	std::vector<float> _pointRadiusSquaredDistance;

	void _obstacle_detection_callback(const pcl::PointCloud<PointT> &src_cloud);
	void _find_obstacle_points(const pcl::PointCloud<PointT>::ConstPtr src_cloud);
    void _filter_dangerous_obstacle_points();
	void _publish_obstacles();
};

#endif

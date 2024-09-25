#ifndef CLOUD_GEOMETRY_HPP
#define CLOUD_GEOMETRY_HPP

#include <lidar_obstacle_detection/common.hpp>
#include <pcl/common/distances.h>

namespace cloud_geometry {
bool is_inside_sector(PointT point, float sector_angle, float sector_radius);

class CloudAngleHandler {
  public:
	CloudAngleHandler(float angle_thrsh);
	void reset_max_angle();
	void calculate_angle(PointT suspect_point, PointT neighbour_point);
	bool is_above_thrsh();

  private:
	float _suspect_point_angle;
	float _suspect_point_angle_max;
	float _angle_thrsh;
};

} // namespace cloud_geometry

#endif

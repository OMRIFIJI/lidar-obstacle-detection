#ifndef CLOUD_GEOMETRY_HPP
#define CLOUD_GEOMETRY_HPP

#include <lidar_obstacle_detection/common.hpp>
#include <pcl/common/distances.h>

namespace cloud_geometry {
bool is_inside_sector(const PointT point, float sector_angle, float sector_radius);

class CloudAngleHandler {
  public:
	CloudAngleHandler(float angle_thrsh);
	void reset_max_angle();
	void calculate_angle(const PointT suspect_point, const PointT neighbour_point);
	bool is_above_thrsh();

  private:
	float m_suspect_point_angle;
	float m_suspect_point_angle_max = 0;
	float m_angle_thrsh;
};

} // namespace cloud_geometry

#endif

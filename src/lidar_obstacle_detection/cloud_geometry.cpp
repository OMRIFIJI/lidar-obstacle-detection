#include <cmath>
#include <lidar_obstacle_detection/cloud_geometry.hpp>

namespace cloud_geometry {

CloudAngleHandler::CloudAngleHandler(float angle_thrsh) { m_angle_thrsh = angle_thrsh; }

void CloudAngleHandler::reset_max_angle() { m_suspect_point_angle_max = 0; }

void CloudAngleHandler::calculate_angle(const PointT suspect_point,
										const PointT neighbour_point) {
	// Point to form right triangle
	PointT orthogonal_point = PointT(neighbour_point.x, neighbour_point.y, suspect_point.z);
	float cathetus1 = pcl::euclideanDistance(suspect_point, orthogonal_point);
	float cathetus2 = pcl::euclideanDistance(neighbour_point, orthogonal_point);

	// Points too close (prevents 0 division)
	if (cathetus1 < 1e-3) {
		return;
	}
	m_suspect_point_angle = atan(cathetus2 / cathetus1);

	if (m_suspect_point_angle_max < m_suspect_point_angle)
		m_suspect_point_angle_max = m_suspect_point_angle;
}

bool CloudAngleHandler::is_above_thrsh() { return m_suspect_point_angle_max > m_angle_thrsh; }

float radius_vec_2d_norm(const PointT point) {
	return sqrt(point.x * point.x + point.y * point.y);
}

float project_to_x(const PointT point) { return point.x; }

bool is_inside_sector(const PointT point, float sector_angle, float sector_radius) {
	float norm = radius_vec_2d_norm(point);
	float angle;
	// Also prevents zero division
	if (norm > sector_radius)
		return 0;

	angle = acos(project_to_x(point) / norm);
	return fabs(angle) < sector_angle;
}

} // namespace cloud_geometry

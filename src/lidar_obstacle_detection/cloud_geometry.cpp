#include <cmath>
#include <lidar_obstacle_detection/cloud_geometry.hpp>

namespace cloud_geometry {

CloudAngleHandler::CloudAngleHandler(float angle_thrsh) {
	_angle_thrsh = angle_thrsh;
	_suspect_point_angle_max = 0;
}

void CloudAngleHandler::reset_max_angle() { _suspect_point_angle_max = 0; }

void CloudAngleHandler::calculate_angle(PointT suspect_point, PointT neighbour_point) {
	// Point to form right triangle
	PointT orthogonal_point = PointT(neighbour_point.x, neighbour_point.y, suspect_point.z);
	float cathetus1 = pcl::euclideanDistance(suspect_point, orthogonal_point);
	float cathetus2 = pcl::euclideanDistance(neighbour_point, orthogonal_point);

	// Points too close (prevents 0 division)
	if (cathetus1 < 1e-3) {
		return;
	}
	_suspect_point_angle = atan(cathetus2 / cathetus1);

	if (_suspect_point_angle_max < _suspect_point_angle)
		_suspect_point_angle_max = _suspect_point_angle;
}

bool CloudAngleHandler::is_above_thrsh() { return _suspect_point_angle_max > _angle_thrsh; }

float radius_vec_2d_norm(PointT point) { return sqrt(point.x * point.x + point.y * point.y); }

float project_to_x(PointT point) { return point.x; }

bool is_inside_sector(PointT point, float sector_angle, float sector_radius) {
	float norm = radius_vec_2d_norm(point);
	float angle;
	// Also prevents zero division
	if (norm < sector_radius)
		return 0;

	angle = acos(project_to_x(point) / norm);
	return fabs(angle) < sector_angle;
}

} // namespace cloud_geometry

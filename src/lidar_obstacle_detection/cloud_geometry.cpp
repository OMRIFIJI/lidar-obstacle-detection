#include <cmath>
#include <lidar_obstacle_detection/cloud_geometry.hpp>

using namespace cloud_geometry;

CloudAngleHandler::CloudAngleHandler(float angle_thrsh) {
	_angle_thrsh = angle_thrsh;
	_suspect_point_angle_max = 0;
}

void CloudAngleHandler::reset_max_angle() { _suspect_point_angle_max = 0; }

void CloudAngleHandler::calculate_angle(PointT suspect_point,
										PointT neighbour_point) {
	// Point to form right triangle
	PointT orthogonal_point =
		PointT(neighbour_point.x, neighbour_point.y, suspect_point.z);
	float cathetus1 = pcl::euclideanDistance(suspect_point, orthogonal_point);
	float cathetus2 = pcl::euclideanDistance(neighbour_point, orthogonal_point);

    // Points too close (prevents 0 division)
    if (cathetus1 < 1e-3){
        return;
    }
	_suspect_point_angle = atan(cathetus2 / cathetus1);

	if (_suspect_point_angle_max < _suspect_point_angle)
		_suspect_point_angle_max = _suspect_point_angle;
}

bool CloudAngleHandler::is_above_thrsh() {

	return _suspect_point_angle_max > _angle_thrsh;
}

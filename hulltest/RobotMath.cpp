#include "RobotMath.h"

using namespace std;

RobotMath::RobotMath()
{
	cout << "Constructor\n";
}

double RobotMath::GetDistance()
{
	dist_flat_left = ((k_diff_height * (cos(angle_elevation_left)) / sin(angle_elevation_left)));
	dist_flat_right = ((k_diff_height * (cos(angle_elevation_right)) / sin(angle_elevation_right)));

	angle_transition_inner = acos((4 + pow(dist_flat_left, 2) - pow(dist_flat_right, 2)) / (4 * dist_flat_left));
	angle_transition_outer = 180 - angle_transition_inner;

	dist_bkbd_perpendicular = dist_flat_left * sin(angle_transition_outer);
	
	dist_perpendicular_to_center = ((dist_bkbd_perpendicular * sin(angle_transition_outer)) / sin(angle_transition_inner)) + k_bkbd_halfwidth;

	dist_reflect_perpendicular = dist_bkbd_perpendicular + 1.25;

	/* "RETURN" */
	dist_output = sqrt(pow(dist_perpendicular_to_center, 2) + (pow(dist_reflect_perpendicular, 2)));

	return dist_output; // Yes, this could be done on the previous line, but we may not return at all.
}

double RobotMath::GetAngle()
{
	angle_offset_left = atan((480 - left.y) / (240 / tan(21.5)) + left.x);
	angle_offset_right = atan((480 - right.y) / (240 / tan(21.5)) + right.x);

	angle_offset_mid = (angle_offset_left + angle_offset_right) / 2;

	angle_reflect_correction = atan(dist_perpendicular_to_center / dist_bkbd_perpendicular) - atan(dist_perpendicular_to_center / dist_reflect_perpendicular);

	angle_output = angle_offset_mid - angle_reflect_correction;

	return angle_output;
}

#include "RobotMath.h"
using namespace std;

RobotMath::RobotMath()
{
	// Empt
}

// For now, robot needs to be on the left side of the field.
double RobotMath::GetDistance(CvPoint leftpt, CvPoint rightpt)
{
	angle_elevation_left = atan((240 - leftpt.y) / k_zeroy) + k_cameraoffset;
	angle_elevation_right = atan((240 - rightpt.y) / k_zeroy) + k_cameraoffset;
	
	dist_flat_left = (k_diff_height * (cos(angle_elevation_left)) / sin(angle_elevation_left));
	dist_flat_right = (k_diff_height * (cos(angle_elevation_right)) / sin(angle_elevation_right));
	
	if (dist_flat_left < dist_flat_right) // On left side of field
	{
		// acos may return negative
		angle_transition_inner = acos((pow(k_bkbd_width, 2) + pow(dist_flat_left, 2) - pow(dist_flat_right, 2)) / (2 * k_bkbd_width * dist_flat_left));
		angle_transition_outer = PI - angle_transition_inner;

		dist_bkbd_perpendicular = dist_flat_left * sin(angle_transition_outer);

		printf("dist_bkbd_perpendicular: %f\n", dist_bkbd_perpendicular);

		dist_perpendicular_to_center = ((dist_bkbd_perpendicular * sin(angle_transition_outer)) / cos(angle_transition_outer)) + (k_bkbd_width / 2);

		printf("dist_perpendicular_to_center: %f\n", dist_perpendicular_to_center);

		dist_reflect_perpendicular = dist_bkbd_perpendicular + 1.25;

		dist_output = sqrt(pow(dist_perpendicular_to_center, 2) + (pow(dist_reflect_perpendicular, 2)));

		return dist_output; // Yes, this could be done on the previous line, but we may not return at all.
	}
	else if (dist_flat_left > dist_flat_right) // On right side of field
	{
		angle_transition_inner = acos((pow(k_bkbd_width, 2) + pow(dist_flat_right, 2) - pow(dist_flat_left, 2)) / (2 * k_bkbd_width * dist_flat_right));
		angle_transition_outer = PI - angle_transition_inner;

		printf("angle_transition_outer: %f\n", angle_transition_outer);

		dist_bkbd_perpendicular = dist_flat_right * sin(angle_transition_outer);

		printf("dist_bkbd_perpendicular: %f\n", dist_bkbd_perpendicular);

		dist_perpendicular_to_center = ((dist_bkbd_perpendicular * sin(angle_transition_outer)) / cos(angle_transition_outer)) + (k_bkbd_width / 2);

		printf("dist_perpendicular_to_center: %f\n", dist_perpendicular_to_center);

		dist_reflect_perpendicular = dist_bkbd_perpendicular + 1.25;

		dist_output = sqrt(pow(dist_perpendicular_to_center, 2) + (pow(dist_reflect_perpendicular, 2)));

		return dist_output; 
	}
	else if (dist_flat_left == dist_flat_right)
	{
		dist_output = sqrt(pow(dist_flat_left, 2) - pow(k_bkbd_width / 2, 2)) + (15.0 / 12.0); // Dist. between bkbd and hoop center
	}
	return dist_output;
}

// Should be doable with only viewpicture info and nothing from GetDistance.
double RobotMath::GetAngle(CvPoint leftpt, CvPoint rightpt)
{
	angle_offset_left = atan((320 - leftpt.x) / k_zerox);
	angle_offset_right = atan((320 - rightpt.x) / k_zerox);

	angle_offset_mid = (angle_offset_left + angle_offset_right) / 2;

	angle_reflect_correction = atan(dist_perpendicular_to_center / dist_bkbd_perpendicular) - atan(dist_perpendicular_to_center / dist_reflect_perpendicular);

	if (dist_flat_left < dist_flat_right && angle_offset_mid <= 0)
	{
		angle_output = angle_reflect_correction + angle_offset_mid;
	}
	else if (dist_flat_left < dist_flat_right && angle_offset_mid > 0)
	{
		angle_output = angle_offset_mid - angle_reflect_correction;
	}
	else if (dist_flat_left > dist_flat_right && angle_offset_mid <= 0)
	{
		angle_output = angle_offset_mid + angle_reflect_correction;
	}
	else if (dist_flat_left > dist_flat_right && angle_offset_mid > 0)
	{
		angle_output = angle_reflect_correction - angle_offset_mid;
	}
	else
	{
		angle_output = angle_offset_mid;
	}

	return angle_output; // As above.
}

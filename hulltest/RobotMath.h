#ifndef ROBOTMATH_H
#define ROBOTMATH_H

#include <iostream>
#include <cmath>
#include <cv.h>

#define PI 3.1415926535

class RobotMath
{
public:
	RobotMath();
	double GetDistance(CvPoint leftpt, CvPoint rightpt);
	double GetAngle(CvPoint leftpt, CvPoint rightpt);
private:
	/* DISTANCE FINDING */
	/* GIVEN */
	// Diff. betwen camera  height and backboard height
	static const double k_diff_height = 85.5 / 12.0;

	// Real distance between top left and top right points. Should be 2ft.
	static const double k_bkbd_width = 2.0;

	// Constant for conversion between pixels and angle
	static const double k_zeroy = 609.275;
	static const double k_zerox = 589.366;

	// Angle above horizontal
	static const double k_cameraoffset = 0.22411;

	// Apparent angles to left and right points from camera horizontal zero line
	// Actually useless placeholders at the moment.
	double angle_elevation_left;
	double angle_elevation_right;

	/* CALCULATED */
	// Flat-ground distance to ground under left/right points
	double dist_flat_left;
	double dist_flat_right;

	// Used to find dist_bkbd_perpendicular
	double angle_transition_inner;
	double angle_transition_outer;

	// Orthogonal distance to backboard 
	double dist_bkbd_perpendicular;

	// Orthogonal distance to plane parallel with backboard and going through basket reflection
	double dist_reflect_perpendicular;

	// Distance from endpoint of dist_bkbd_perpendicular on backboard to point of basket attachment
	double dist_perpendicular_to_center;

	// Distance between robot and reflected basket behind backboard
	double dist_output;


	/* ANGLE FINDING */
	// Apparent angle offset between camera vertical center line and points on backboard
	double angle_offset_left;
	double angle_offset_right;
	double angle_offset_mid;

	// Angle correction to point at reflection of basket
	double angle_reflect_correction;

	// Angle offset between robot facing and reflected basket
	double angle_output;
};
#endif

	

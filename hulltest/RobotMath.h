#ifndef ROBOTMATH_H
#define ROBOTMATH_H

#include <iostream>
#include <cmath>
#include <cv.h>

class RobotMath
{
public:
	RobotMath();
	double GetDistance();
	double GetAngle();
private:
	/* DISTANCE FINDING */
	// Diff. betwen camera  height and backboard height
	double k_diff_height;

	// One-half real distance between top left and top right points. Should be 1.5ft.
	double k_bkbd_halfwidth;

	// Apparent angles to left and right points from camera flatline
	double angle_elevation_left;
	double angle_elevation_right;

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
	// Placeholder variable for left/right points. Will be replaced with reference to input from hulltest.
	CvPoint left;
	CvPoint right;

	// Apparent angle offset between camera center line and points on backboard
	double angle_offset_left;
	double angle_offset_right;
	double angle_offset_mid;

	// Angle correction to point at reflection of basket
	double angle_reflect_correction;

	// Angle offset between robot facing and reflected basket
	double angle_output;
};
#endif

	

#include "helper.h"

void init(double& width1, double& height1, 
	int& N_obs, double&D, double&Lx, 
	double&Ly, double&Ax, double&Ay, double&alpha_max)
{
	width1 = 640;
	height1 = 480;

	// number of obstacles (not currently implemented in the library)
	N_obs = 0;

	// set robot model parameters ////////

	D = 121.0; // distance between front wheels (pixels)

	// position of laser in local robot coordinates (pixels)
	// note for Lx, Ly we assume in local coord the robot
	// is pointing in the x direction		
	Lx = 31.0;
	Ly = 0.0;

	// position of robot axis of rotation halfway between wheels (pixels)
	// relative to the robot image center in local coordinates
	Ax = 37.0;
	Ay = 0.0;

	alpha_max = 3.14159; // max range of laser / gripper (rad)
}

void turn(int pwm_offset, int& pw_l, int& pw_r)
{
	pw_l = PWM_STATIONARY + pwm_offset;
	pw_r = PWM_STATIONARY + pwm_offset;
}
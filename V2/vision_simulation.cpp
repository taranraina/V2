
#include <cstdio>
#include <iostream>
#include <fstream>

#include <Windows.h>

using namespace std;

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

#include "image_transfer5.h"

// include this header file for computer vision functions
#include "vision.h"

#include "robot.h"

#include "vision_simulation.h"

#include "timer.h"

// global variables ///////////

robot_system *S1;
image rgb_robot, rgb_opponent, rgb_background;
image rgb_robot_r, rgb_opponent_r;
image rgb_obstacle, rgb_obstacle_r;

///////////////////////////////

int activate_simulation(double width, double height,
	double x_obs[], double y_obs[], double size_obs[], int N_obs,
	char robot_file[], char opponent_file[], char background_file[],
	char obstacle_file[], double D, double Lx, double Ly,
	double Ax, double Ay, double alpha_max)
{
	int i;

	S1 = new robot_system(D, Lx, Ly, Ax, Ay, alpha_max);

	if (S1 == NULL) {
		cout << "\nmemory allocation error in activate_simulation()";
		return 1;
	}

	S1->width = width;
	S1->height = height;

	S1->N_obs = N_obs;

	for (i = 1; i <= N_obs; i++) {
		S1->x_obs[i] = x_obs[i];
		S1->y_obs[i] = y_obs[i];
		S1->size_obs[i] = size_obs[i];
	}

	// get image size, dynamically allocate images, and load from file

	set_rgb_image(robot_file, rgb_robot);
	set_rgb_image(robot_file, rgb_robot_r);

	set_rgb_image(opponent_file, rgb_opponent);
	set_rgb_image(opponent_file, rgb_opponent_r);

	set_rgb_image(background_file, rgb_background);

	set_rgb_image(obstacle_file, rgb_obstacle);
	set_rgb_image(obstacle_file, rgb_obstacle_r);

	return 0;
}


int deactivate_simulation()
{
	// free the image memory before the program completes
	free_image(rgb_robot);
	free_image(rgb_opponent);
	free_image(rgb_background);
	free_image(rgb_robot_r);
	free_image(rgb_opponent_r);
	free_image(rgb_obstacle);
	free_image(rgb_obstacle_r);

	// safe delete
	if (S1 != NULL) {
		delete S1;
		S1 = NULL;
	}
	else {
		cout << "\nerror: NULL pointer in deactivate_simulation()";
	}

	return 0;
}

// TODO: interpolate inputs if large delays ?

int set_inputs(int pw_l, int pw_r, int pw_laser, int laser,
	double light, double light_gradient, double light_dir,
	double image_noise, double max_speed, double opponent_max_speed)
{
	// * note it's the responsibility of the user to call this function
	// or the inputs will stay constant in the simulation

	// set inputs for the simulation -- no smoothing / interpolation
	// since actual system doesn't do that

	// TODO: check mode and also set opponent inputs if required
	// -- *** need to hide this part since students could use it
	// maybe include simple strategies
	// let students have the option to program their own opponent too

	S1->light = light;
	S1->light_gradient = light_gradient;
	S1->light_dir = light_dir;
	S1->image_noise = image_noise;

	// need to set max_speed before setting other inputs
	// since inputs depend on that parameter
	S1->P[1]->v_max = max_speed;

	// set robot inputs
	S1->P[1]->set_inputs(pw_l, pw_r, pw_laser, laser);

	if (S1->N_robot > 1) {
		S1->P[2]->v_max = opponent_max_speed;
	}

	// note the opponent input will be set automatically later

	return 0;
}


// TODO:
// 1. precompute -- in activate to save time
// eg image rotations, etc.

// 2. draw laser gun turrent and laser blast
// precomputed laser blasts at different angles 
// use bmp and rotate for lasers -- how to shorten ?
// -- just draw multiple rgb points ?

int acquire_image_sim(image &rgb)
// assume this function is called frequenlty since it performs
// real-time simulation of the robots
{
	int i, j;
	double x, y, theta;
	int ic, jc;

	static int init = 0;
	static double tc0;
	double tc;
	double dt = 1.0e-4; // simulation time step

	// laser start time
	static double t_laser_start = 0.0;

	// previous state of laser input
	static int laser_previous = 0;
	static int laser_fired = 0;

	// laser time duration (s)
	double t_laser;
	double laser_duration = 3.0;

	// TODO: use subpixel rendering -- ie double for (i,j), etc.

	// TODO: time stamp result with simulation and or clock time

	// store initial clock time
	if (!init) {
		tc0 = high_resolution_time();
		init = 1;
	}

	// read current clock time
	tc = high_resolution_time() - tc0;

	// real-time simulation of robots
	// -- simulate robots with time step dt until simulation time = clock time
	while (S1->t < tc) S1->sim_step(dt);

	// note that theta adjustment assumes the robot image points upwards
	// -- save the robot bmp file with the robot pointing upwards
	theta = S1->P[1]->x[1] - 3.14159 / 2;
	x = S1->P[1]->x[2] - S1->P[1]->xa;
	y = S1->P[1]->x[3] - S1->P[1]->ya;

	// get pixel location of robot center
	i = (int)x;
	j = (int)y;

	// construct image from simulation results

	// copy background into result
	copy(rgb_background, rgb);

	// rotate robot image around image center point by theta	
	rotate(rgb_robot, rgb_robot_r, theta);

	// image center point
	ic = rgb_robot.width / 2;
	jc = rgb_robot.height / 2;

	// calculate bottom left corner of robot image in global coord
	i -= ic;
	j -= jc;

	// place robot at point (i,j)
	append(rgb, rgb_robot_r, i, j);

	// check if laser was fired (laser went from 0 to 1)
	// -- ie rising edge on laser
	if (S1->P[1]->laser && !laser_previous) {
		t_laser_start = S1->t; // record laser start time
		laser_fired = 1;
		cout << "\nlaser fired !";
	}

	// update previous laser state
	laser_previous = S1->P[1]->laser;

	// draw laser if fired
	if (laser_fired) {
		draw_laser(S1->P[1], rgb);
	}

	// turn laser off after duration
	t_laser = S1->t - t_laser_start;
	if (t_laser > laser_duration) {
		laser_fired = 0;
	}

	return 0;
}


int rotate(image &a, image &b, double theta)
{
	int i, j, k, R, G, B;
	int width, height;
	ibyte *pa, *pb;
	double i_l, j_l; // local coord for i,j
	double ic, jc;
	double cos_th, sin_th;
	int i1, j1, j2, i2;

	//	ibyte *p1, *p2, *p3, *p4;
	//	int B1, G1, R1, B2, G2, R2, B3, G3, R3, B4, G4, R4;	
	//	double Ba, Ga, Ra, Bb, Gb, Rb, ip, jp;

	// TODO: add #define DEBUG which checks bounds

	cos_th = cos(theta);
	sin_th = sin(theta);

	pa = a.pdata;
	pb = b.pdata;

	width = b.width;
	height = b.height;

	// erase output image
	R = 0; G = 0; B = 0;
	for (j = 0; j<height; j++) {
		for (i = 0; i<width; i++) {
			*pb = B;
			*(pb + 1) = G;
			*(pb + 2) = R;
			pb += 3; // move to next pixel in output image
		}
	}

	// initialize pb for next step
	pb = b.pdata;

	// calculate each pixel in output image by linear interpolation
	// with respect to original image
	for (j = 0; j<height; j++) {

		for (i = 0; i<width; i++) {

			// calculate local coord il, jl
			// pg = R*pl,  pg = [i,j], pl = [il,jl]	
			// pl = inv(R)*pg = Rt*pg
			// R  = [cos(th) -sin(th)]
			//      [sin(th)  cos(th)]
			// Rt = [ cos(th) sin(th)]
			//      [-sin(th) cos(th)]

			// center coords
			ic = i - 0.5*width;
			jc = j - 0.5*height;

			i_l = cos_th*ic + sin_th*jc;
			j_l = -sin_th*ic + cos_th*jc;

			i1 = (int)(i_l + 0.5*width);
			i2 = i1 + 1;

			j1 = (int)(j_l + 0.5*height);
			j2 = j1 + 1;

			// check if (i1,j1), etc. are within range
			if ((i1 > 3) && (i1 < width - 3) &&
				(j1 > 3) && (j1 < height - 3)) {

				// simple interpolation -- use (i1,j1) ///////////

				// byte k for pixel (i1,j1)
				k = 3 * (j1*width + i1);

				B = pa[k];
				G = pa[k + 1];
				R = pa[k + 2];

				////////////////////////////////////////////////

				/*
				// bilinear interpolation ///////////////////////

				// set neighborhood pointers to interpolation points
				// p3 p4
				// p1 p2

				// TODO: remove i,j calcs and use small increments like conv
				p1 = pa + 3*( j1*width + i1 ); // (i1,j1)
				p2 = pa + 3*( j1*width + i2 ); // (i2,j1)
				p3 = pa + 3*( j2*width + i1 ); // (i1,j2)
				p4 = pa + 3*( j2*width + i2 ); // (i2,j2)

				B1 = *p1; G1 = *(p1+1); R1 = *(p1+2);
				B2 = *p2; G2 = *(p2+1); R2 = *(p2+2);
				B3 = *p3; G3 = *(p3+1); R3 = *(p3+2);
				B4 = *p4; G4 = *(p4+1); R4 = *(p4+2);

				// find RGB for point a, b using linear interpolation, eg
				// p3 Rb p4
				// p1 Ra p2

				// interpolation point
				ip = i_l + 0.5*width;
				jp = j_l + 0.5*height;

				Ra = R1 + (R2 - R1)*(ip - i1);
				Rb = R3 + (R4 - R3)*(ip - i1);

				Ga = G1 + (G2 - G1)*(ip - i1);
				Gb = G3 + (G4 - G3)*(ip - i1);

				Ba = B1 + (B2 - B1)*(ip - i1);
				Bb = B3 + (B4 - B3)*(ip - i1);

				// interpolate from point a to point b
				// p3 Rb p4
				// p1 Ra p2

				R = Ra + (Rb - Ra)*(jp - j1);
				G = Ga + (Gb - Ga)*(jp - j1);
				B = Ba + (Bb - Ba)*(jp - j1);

				////////////////////////////////////////////////////
				*/

				if (!((R < 1) && (G < 1) && (B < 1))) {
					*pb = B;
					*(pb + 1) = G;
					*(pb + 2) = R;
				}

			} // end if in range

			// move to next pixel in output image
			pb += 3;

		} // end for i

	} // end for j

	return 0;
}


int append(image &a, image &b, int ip, int jp)
{
	int i, j, ia, ja;
	int width_a, height_a, width_b, height_b;
	ibyte *pa, *pb, R, G, B;

	pa = a.pdata;
	width_a = a.width;
	height_a = a.height;

	pb = b.pdata;
	width_b = b.width;
	height_b = b.height;

	// set pointer pa to beginning of the window for image b
	pa += 3 * (jp*width_a + ip);

	for (j = 0; j<height_b; j++) {

		for (i = 0; i<width_b; i++) {

			B = *pb;
			G = *(pb + 1);
			R = *(pb + 2);

			// calculate equivalent coordinates (ia,ja) in image a
			ia = ip + i;
			ja = jp + j;

			// only write if (ia,ja) is in range
			if ((ia >= 0) && (ia < width_a) && (ja >= 0) && (ja < height_a)) {

				if (!((R < 1) && (G < 1) && (B < 1))) {
					*pa = B;
					*(pa + 1) = G;
					*(pa + 2) = R;
				}

			} // end if in range

			// move to next pixel
			pa += 3;
			pb += 3;

		}

		// advance to the next line
		pa += 3 * (width_a - width_b);

	} // end for j

	return 0;
}


// 1 player, 2 player, practice, strategies, speed / difficulty level
int set_simulation_mode(int mode, int level)
{
	S1->mode = mode;
	S1->level = level;

	return 0;
}

int set_robot_position(double x, double y, double theta)
{
	S1->P[1]->x[1] = theta;
	S1->P[1]->x[2] = x;
	S1->P[1]->x[3] = y;

	// calculate robot output
	S1->P[1]->calculate_outputs();

	return 0;
}


int set_opponent_position(double x, double y, double theta)
{
	if (S1->N_robot > 1) {
		S1->P[2]->x[1] = theta;
		S1->P[2]->x[2] = x;
		S1->P[2]->x[3] = y;

		// calculate robot output
		S1->P[2]->calculate_outputs();
	}

	return 0;
}


robot_system::robot_system(double D, double Lx, double Ly,
	double Ax, double Ay, double alpha_max)
{
	int i;
	double x0, y0, theta0, vmax;
	double max_speed, opponent_max_speed;
	int pw_l, pw_r, pw_laser, laser;

	// initial time
	t = 0.0;

	N_robot = 1;

	width = 640;
	height = 480;

	N_obs = 0;

	// robot max wheel speed (pixels/s)
	max_speed = 75.0;

	// opponent max wheel speed (pixels/s)
	opponent_max_speed = 75.0;

	// array of pointers to robot objects
	for (i = 1; i <= N_robot; i++) {

		if (i == 1) { // robot
			x0 = 0.3*width;
			y0 = 0.5*height;
			theta0 = 0.0;
			vmax = max_speed;
		}
		else if (i == 2) { // opponent
			x0 = 0.7*width;
			y0 = 0.5*height;
			theta0 = 3.14159;
			vmax = opponent_max_speed;
		}
		else { // default / other robots
			x0 = 0.5*width;
			y0 = 0.5*height;
			theta0 = 3.14159 / 2;
			vmax = opponent_max_speed;
		}

		P[i] = new robot(x0, y0, theta0, vmax);

		if (P[i] == NULL) {
			cout << "\nmemory allocation error in robot_system()";
			return;
		}

	}

	// default actuator values
	pw_l = 1500; // us
	pw_r = 1500; // us
	pw_laser = 1500; // us
	laser = 0; // 0 or 1

	// set robot initial inputs and parameters
	for (i = 1; i <= N_robot; i++) {
		P[i]->set_inputs(pw_l, pw_r, pw_laser, laser);

		P[i]->D = 100.0; // distance between wheels (pixels)

		// position of laser in local robot coordinates
		// note for Lx, Ly we assume in local coord the robot
		// is pointing in the x direction	
		P[i]->Lx = 100.0; // laser / gripper position (pixels)
		P[i]->Ly = 0.0; // laser / gripper position (pixels)
		P[i]->Ax = 0.0; // position of robot center relative to image center
		P[i]->Ay = 0.0; // position of robot center relative to image center		
		P[i]->alpha_max; // max range of laser / gripper (rad)
	}

	// take robot parameters from activate_simulation function
	P[1]->D = D; // distance between wheels (pixels)
	P[1]->Lx = Lx; // laser / gripper position (pixels)
	P[1]->Ly = Ly; // laser / gripper position (pixels)
	P[1]->Ax = Ax; // position of robot center relative to image center
	P[1]->Ay = Ay; // position of robot center relative to image center	
	P[1]->alpha_max; // max range of laser / gripper (rad)

	// calculate robot outputs
	for (i = 1; i <= N_robot; i++) {
		P[i]->calculate_outputs();
	}

	// default lighting condition parameters (TODO: add shadow effect, etc.)
	light = 1.0;
	light_gradient = 0.0;
	light_dir = 0.0;
	image_noise = 0.0;

	// 1 player, 2 player, practice, strategies, speed / difficulty level
	mode = 1;
	level = 1;

}


robot_system::~robot_system()
// class destructor
// free dynamic memory, etc.
{
	int i;

	// array of pointers to robot objects
	for (i = 1; i <= N_robot; i++) {
		// safe delete
		if (P[i] != NULL) {
			delete P[i];
			P[i] = NULL;
		}
		else {
			cout << "\nerror: NULL pointer in ~robot_system()";
		}
	}

}


void robot_system::sim_step(double dt)
{
	int i;

	// * assume the inputs have already been set for each robot

	// simulate each robot in the system for one time step	
	for (i = 1; i <= N_robot; i++) {
		P[i]->sim_step(dt);
	}

	// increment robot_system time
	t += dt;

}


int draw_laser(robot *P, image &rgb)
{
	double x0, y0, theta, r, dr;
	int i, j, R, G, B;

	dr = 1;

	// set laser colour
	R = 0;
	G = 255;
	B = 0;

	// get start point of the laser
	x0 = P->xg;
	y0 = P->yg;

	// robot theta + laser alpha
	theta = P->x[1] + P->x[4];

	for (r = 0; r<5000; r += dr) {

		i = (int)(x0 + r*cos(theta));
		j = (int)(y0 + r*sin(theta));

		// stop loop when (i,j) goes out of range / off screen
		if (i < 3) break;
		if (i > rgb.width - 3) break;
		if (j < 3) break;
		if (j > rgb.height - 3) break;

		draw_point_rgb_laser(rgb, i, j, R, G, B);

	}

	return 0;
}


int draw_point_rgb_laser(image &rgb, int ip, int jp, int R, int G, int B)
{
	ibyte *p;
	int i, j, w = 1, pixel;

	// initialize pointer
	p = rgb.pdata;

	if (rgb.type != RGB_IMAGE) {
		cout << "\nerror in draw_point_RGB: input type not valid!\n";
		return 1;
	}

	// limit out of range (i,j) values
	// NOTE: this part is important to avoid wild pointers
	if (ip < w) ip = w;
	if (ip > rgb.width - w - 1) ip = rgb.width - w - 1;
	if (jp < w) jp = w;
	if (jp > rgb.height - w - 1) jp = rgb.height - w - 1;

	for (i = -w; i <= w; i++) {
		for (j = -w; j <= w; j++) {
			pixel = rgb.width*(jp + j) + (ip + i);
			p[3 * pixel] = B;
			p[3 * pixel + 1] = G;
			p[3 * pixel + 2] = R;
		}
	}

	return 0;
}


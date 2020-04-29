
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
#include "helper.h"

extern robot_system S1;

int main()
{
	double x0, y0, theta0, max_speed, opponent_max_speed;
	int pw_l, pw_r, pw_laser, laser;
	double light, light_gradient, light_dir, image_noise;
	double width1, height1;
	int N_obs;
	double x_obs[50], y_obs[50], size_obs[50];
	double D, Lx, Ly, Ax, Ay, alpha_max;
	int ic_c[20], jc_c[20];
	int nlabel, thresh1, thresh2;
	double Ravg[20], Gavg[20], Bavg[20];
	double tc, tc0; // clock time

	init( width1,  height1,
		 N_obs, D, Lx,
		Ly, Ax, Ay, alpha_max);

	cout << "\npress space key to begin program.";
	pause();

	// you need to activate the regular vision library before 
	// activating the vision simulation library
	activate_vision();

	// note it's assumed that the robot points upware in its bmp file

	// however, Lx, Ly, Ax, Ay assume robot image has already been
	// rotated 90 deg so that the robot is pointing in the x-direction
	// -- ie when specifying these parameters assume the robot
	// is pointing in the x-direction.

	// note that the robot opponent is not currently implemented in 
	// the library, but it will be implemented soon.

	activate_simulation(width1, height1, x_obs, y_obs, size_obs, N_obs,
		"robot_A.bmp", "robot_A.bmp", "background.bmp", "obstacle.bmp", D, Lx, Ly, Ax, Ay, alpha_max);

	// open an output file if needed for testing or plotting
	//	ofstream fout("sim1.txt");
	//	fout << scientific;

	// set robot initial position (pixels) and angle (rad)
	x0 = 320;
	y0 = 200;
	theta0 = 0;
	set_robot_position(x0, y0, theta0);

	// set initial inputs / on-line adjustable parameters /////////

	// inputs
	turn(-50, pw_l, pw_r);
	pw_laser = 1500; // pulse width for laser servo (us)
	laser = 0; // laser input (0 - off, 1 - fire)

	// paramaters
	max_speed = 100; // max wheel speed of robot (pixels/s)
	opponent_max_speed = 100; // max wheel speed of opponent (pixels/s)

	// lighting parameters (not currently implemented in the library)
	light = 1.0;
	light_gradient = 1.0;
	light_dir = 1.0;
	image_noise = 1.0;

	// set initial inputs
	set_inputs(pw_l, pw_r, pw_laser, laser,
		light, light_gradient, light_dir, image_noise,
		max_speed, opponent_max_speed);

	// regular vision program ////////////////////////////////

	// note that at this point you can write your vision program
	// exactly as before.

	// in addition, you can set the robot inputs to move it around
	// the image and fire the laser.

	image rgb, rgb0, grey1, grey2, labels;
	int height, width;

	// note that the vision simulation library currently
	// assumes an image size of 640x480
	width = 640;
	height = 480;

	init_image(rgb, width, height, RGB_IMAGE);
	init_image(rgb0, width, height, RGB_IMAGE);
	init_image(grey1, width, height, GREY_IMAGE);
	init_image(grey2, width, height, GREY_IMAGE);
	init_image(labels, width, height, LABEL_IMAGE);

	allocate_image(rgb);
	allocate_image(rgb0);
	allocate_image(grey1);
	allocate_image(grey2);
	allocate_image(labels);

	// allocate memory for the images
	

	// measure initial clock time
	tc0 = high_resolution_time();
	thresh1 = 50;

	while (1) {

		// simulates the robots and acquires the image from simulation
		acquire_image_sim(rgb);
		copy(rgb, rgb0);

		tc = high_resolution_time() - tc0;

		// TODO: Make this a function called centroid tracking

		get_labels(rgb0, grey1, grey2, labels, nlabel, thresh1);
		copy(rgb, rgb0);
		for (int i = 0; i <= nlabel; i++)
		{
			greyscale_centroid(rgb0, grey1, grey2, labels, ic_c[i], jc_c[i], i);
			copy(rgb, rgb0);
			view_rgb_image(rgb);
			calculate_average_RGB(rgb0, labels, i, Ravg[i], Gavg[i], Bavg[i]);
		}
		copy(rgb, rgb0);

		int x, y;
		double theta;
		calculate_robot_position(x, y, ic_c, jc_c, Ravg, Gavg, Bavg, nlabel, theta);

		// change the inputs to move the robot around
		// or change some additional parameters (lighting, etc.)

		// only the following inputs work so far
		// pw_l -- pulse width of left servo (us) (from 1000 to 2000)
		// pw_r -- pulse width of right servo (us) (from 1000 to 2000)
		// pw_laser -- pulse width of laser servo (us) (from 1000 to 2000)
		// -- 1000 -> -90 deg
		// -- 1500 -> 0 deg
		// -- 2000 -> 90 deg
		// laser -- (0 - laser off, 1 - fire laser for 3 s)
		// max_speed -- pixels/s for right and left wheels
		set_inputs(pw_l, pw_r, pw_laser, laser,
			light, light_gradient, light_dir, image_noise,
			max_speed, opponent_max_speed);

		view_rgb_image(rgb);

		// don't need to simulate too fast
		Sleep(10); // 100 fps max
	}

	// free the image memory before the program completes
	free_image(rgb);

	deactivate_vision();

	deactivate_simulation();

	cout << "\ndone.\n";

	return 0;
}

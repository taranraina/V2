#include "helper.h"

const double PI = atan(1) * 4;
extern robot_system *S1;
#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

void init(double& width1, double& height1,
	int& N_obs, double&D, double&Lx,
	double&Ly, double&Ax, double&Ay,
	double&alpha_max, double* x_obs,
	double* y_obs, double* size_obs, int obstacle_xlocation, int obstacle_ylocation)
{
	width1 = 640;
	height1 = 480;

	N_obs = 1;

	x_obs[1] = obstacle_xlocation; // pixels
	y_obs[1] = obstacle_ylocation; // pixels
	size_obs[1] = 1;

	//x_obs[2] = 135; // pixels
	//y_obs[2] = 135; // pixels
	//size_obs[2] = 1;
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

	alpha_max = 3.14159 / 2; // max range of laser / gripper (rad)
}

void turn(int pwm_offset, int& pw_l, int& pw_r)
{
	pw_l = PWM_STATIONARY + pwm_offset;
	pw_r = PWM_STATIONARY + pwm_offset;
}

int get_labels(image rgb, image grey1, image grey2, image label, int &nlabels, int thresh)
{
	//Get the labels in an image within a certain threshold. Note that this function doesn't use image inversion
	//To get labels for dark objects, use the other function "get_labels_inverted".

	int width, height;

	width = rgb.width;
	height = rgb.height;

	if (RGB_IMAGE != rgb.type)
	{
		cout << "\nError, input rgb isn't a RGB_IMAGE type.";
		return 1;
	}

	if (width != grey1.width || height != grey1.height)
	{
		cout << "\nError, input grey1 doesn't have the pixel dimensions.";
		return 1;
	}

	if (width != grey2.width || height != grey2.height)
	{
		cout << "\nError, input grey2 doesn't have the pixel dimensions.";
		return 1;
	}

	if (width != label.width || height != label.height)
	{
		cout << "\nError, input label doesn't have the pixel dimensions.";
		return 1;
	}

	if (GREY_IMAGE != grey1.type)
	{
		cout << "\nError, input grey1 isn't a GREY_IMAGE type.";
		return 1;
	}

	if (GREY_IMAGE != grey2.type)
	{
		cout << "\nError, input grey2 isn't a GREY_IMAGE type.";
		return 1;
	}

	if (LABEL_IMAGE != label.type)
	{
		cout << "\nError, input label isn't a GREY_IMAGE type.";
		return 1;
	}

	//Get the greyscale image
	copy(rgb, grey1);
	copy(grey1, rgb);

	//Apply filter to greyscale image
	lowpass_filter(grey1, grey2);
	copy(grey2, grey1);
	copy(grey1, rgb);

	//Scale the image to improve contrast
	scale(grey1, grey2);
	copy(grey2, grey1);
	copy(grey1, rgb);

	//Apply a threshold to the image
	threshold(grey1, grey2, thresh);
	copy(grey2, grey1);
	copy(grey1, rgb);

	//Invert the image
	//invert(grey1, grey2);
	//copy(grey2, grey1);
	//copy(grey1, rgb);

	//Erode the image
	erode(grey1, grey2);
	copy(grey2, grey1);
	copy(grey1, rgb);

	//Dialate the image
	dialate(grey1, grey2);
	copy(grey2, grey1);
	copy(grey1, rgb);

	//Label the objects in the image
	copy(rgb, grey1);
	label_image(grey1, label, nlabels);

	return 0;
}

int calculate_average_RGB(image rgb, image label, int nlabel, double &R, double &G, double &B)
{
	//Calculate the average normalized RGB value of a labeled image,.
	int k, i, j, height, width, size, m;
	ibyte *prgb;
	i2byte *plabel;

	height = rgb.height;
	width = rgb.width;

	if (width != label.width || height != label.height)
	{
		cout << "\nError, input label doesn't have the correct pixel dimensions.";
		return 1;
	}

	if (RGB_IMAGE != rgb.type)
	{
		cout << "\nError, input grey isn't a RGB_IMAGE type.";
		return 1;
	}

	if (LABEL_IMAGE != label.type)
	{
		cout << "\nError, input label isn't a LABEL_IMAGE type.";
		return 1;
	}

	size = height * width;
	R = G = B = 0;
	m = 0;
	for (k = 0; k < size; k++)
	{
		prgb = rgb.pdata + k * 3;
		plabel = (i2byte*)label.pdata + k;

		if (nlabel == (int)*plabel)
		{
			m += 1;
			B += *prgb;
			G += *(prgb + 1);
			R += *(prgb + 2);
		}
	}

	//This gives the average values for R G B
	R = R / m;
	G = G / m;
	B = B / m;
	//If we want to scale it from 0 and 1, we divide by 255.0

	R = R / 255.0;
	G = G / 255.0;
	B = B / 255.0;


	//cout << "\nR=" << R << "\tG=" << G << "\tB=" << B;
	return 0;
}

int greyscale_centroid(image rgb, image grey, image grey1, image label, int &ic, int &jc, short int nlabel)
{
	//Find the centroid of a labeled image in greyscale.
	//This uses the rgb image and converts everything to greyscale before doing the necessary calculations.

	int i, j, k;
	ibyte *pa;
	i2byte *pl;
	double m = 0.0;
	double rho;
	int width, height, size;

	ic = 0;
	jc = 0;

	width = rgb.width;
	height = rgb.height;

	if (RGB_IMAGE != rgb.type)
	{
		cout << "\nError, input rgb isn't a RGB_IMAGE type.";
		return 1;
	}

	if (width != grey.width || height != grey.height)
	{
		cout << "\nError, input grey doesn't have the pixel dimensions.";
		return 1;
	}

	if (width != grey1.width || height != grey1.height)
	{
		cout << "\nError, input grey1 doesn't have the pixel dimensions.";
		return 1;
	}

	if (width != label.width || height != label.height)
	{
		cout << "\nError, input label doesn't have the pixel dimensions.";
		return 1;
	}

	if (GREY_IMAGE != grey.type)
	{
		cout << "\nError, input grey1 isn't a GREY_IMAGE type.";
		return 1;
	}

	if (GREY_IMAGE != grey1.type)
	{
		cout << "\nError, input grey2 isn't a GREY_IMAGE type.";
		return 1;
	}

	if (LABEL_IMAGE != label.type)
	{
		cout << "\nError, input label isn't a GREY_IMAGE type.";
		return 1;
	}

	copy(rgb, grey);
	copy(grey, grey1);

	gaussian_filter(grey1, grey);
	copy(grey, grey1);
	scale(grey1, grey);

	size = width * height;

	pa = grey.pdata;
	pl = (i2byte *)label.pdata;

	for (k = 0; k < size; k++)
	{
		pa = grey.pdata + k;
		pl = (i2byte *)label.pdata + k;

		if (nlabel == *pl)
		{
			//cout << endl << "*pl=" << *pl;
			i = k % width;
			j = (k - i) / width;

			//cout << endl << "i=" << i;
			//cout << endl << "j=" << j;
			rho = (double)*pa;
			m += rho;
			ic += rho * i;
			jc += rho * j;

		}
	}

	ic = ic / m;
	jc = jc / m;

	return 0;
}

void init_image(image& img, int width, int height, int img_type)
{
	img.type = img_type;
	img.width = width;
	img.height = height;
}

int calculate_robot_position(int &x, int &y, int ic[],
	int jc[], double Ravg[], double Gavg[],
	double Bavg[], int nlabels, double &theta, int &ig, int&jg, int& ir, int& jr)
{
	//robot is straight if the centroid of the green and red objects are in the horizontal plane]
	//Assume that the only red and green color in the background is our robot.
	//Assume that the 0 degree angle is robot having the green circle at the right and red at the left.
	//note that the Ravg, Gavg and Bavg values range between 0 and 1. The thresholds in the if statement should be changed
	//depending on the lighting, camera settings, etc.

	//the variable x and y represent the position of the centroid of the robot (located between our two targets)
	//The angle theta is the orientation of the robot.

	int nl, flag = 1;
	static int ir_p = 0, jr_p = 0, ig_p = 0, jg_p = 0;
	int width = 640;
	int height = 480;
	ig = 0;
	jg = 0;
	ir = 0;
	jr = 0;

	for (nl = 0; nl <= nlabels; nl++)
	{
		//cout << endl << Ravg[nl];
		//Red target
		if (Ravg[nl] > 0.6 && Gavg[nl] < 0.4 && Bavg[nl] < 0.4)
		{
			ir = ic[nl];
			jr = jc[nl];
		}

		//Green target
		if (Ravg[nl] < 0.5 && Gavg[nl] > 0.6 && Bavg[nl] < 0.6)
		{
			ig = ic[nl];
			jg = jc[nl];
			cout << "\ngreen centroid is " << ig << "\t" << jg << endl;
		}

	}

	//assume that the green target is on the left and the red target is on the 
	//the angle the robot has turned is (jr-jg)/(ir-ig)
	//the center of the robot is the average position between the red and green target

	//cout << "\nig=" << ig << "\tjg=" << jg;
	//cout << "\nir=" << ir << "\tjr=" << jr;

	//if the robot went partially out of bounds, then use the previous value for an approximation based on the remaining color still in bound
	//and send a command to either move it back in bounds
	//
	//Returning 1 means to reverse
	//Returning 2 means to go forward
	if (ig == 0 && jg == 0)
	{
		cout << "\nGreen marker is out of bounds.";
		ig = ig_p;

		x = (ig + ir) / 2;
		y = (jg + jr) / 2;

		theta = atan2((jg - jr), (ig - ir));

		//if the green indicator left from the right side, then the robot is facing forward, need to reverse it
		return 1;

	}
	else if (ir == 0 && jr == 0)
	{
		cout << "\nRed marker is out of bounds.";
		ir = ir_p;

		x = (ig + ir) / 2;
		y = (jg + jr) / 2;

		theta = atan2((jg - jr), (ig - ir));

		return 2;
	}


	ir_p = ir;
	jr_p = jr;
	ig_p = ig;
	jg_p = jg;

	x = ig;
	y = jg;

	cout << "\nrobot centroid is " << x << "\t" << y << endl;
	theta = atan2((jg - jr), (ig - ir));

	//cout << "\nTheta=" << theta;

	return 0;
}

void robot_circle(image& img, double PI, int x, int y)
{
	int robot_r = 70;
	float num_particles = 100;
	for (int i = 0; i < num_particles; i++) {
		int X = x + robot_r * cos(((i * 360) / num_particles) * PI / 180.0);
		int Y = y + robot_r * sin(((i * 360) / num_particles) * PI / 180.0);

		draw_point_rgb(img, X, Y, 255, 0, 255);
	}
}

void convert_theta_positive(double& theta)
{
	double PI = atan(1) * 4;

	if (theta < 0) {
		while (theta < 0) theta = theta + 2 * PI;
	}
}

int rotate_robot(int &pw_r, int &pw_l, double theta_current, double theta_desired)
{
	//this function sets the pwm to rotate the robot to the desired angle.
	//don't worry about tolerance, already taken care of in the function where this is called.


	//cout << "\ntheta_desired=" << theta_desired << "\ttheta_current=" << theta_current;

	int pw_cw = 1450;
	int pw_ccw = 1550;
	int slope = 250;

	//if the desired angle is in the first quadrant.
	if (theta_desired >= 0 && theta_desired <= PI / 2)
	{
		//cout << "\ntheta_desired is in the first quadrant.";

		//current angle is between between the second quadrant and the desired angle in the first quadrant (turn it cw)
		if (theta_current > theta_desired && theta_current <= PI)
		{
			//rotate cw
			cout << "\nCW";
			pw_r = pw_cw - (theta_current - theta_desired)*slope;
			pw_l = pw_r;
			return 0;
		}
		//current angle is between on the third quadrant and the angle is less than the continous line made from the desired angle (theta_desired-PI).
		else if (theta_current < 0 && abs(theta_current) >= (PI - theta_desired))
		{
			//rotate cw
			cout << "\nCW";
			pw_r = pw_cw - ((2 * PI + theta_current) - theta_desired)*slope;
			pw_l = pw_r;
			return 0;
		}
		//current angle is on the first quadrant to the right of where the desired angle is
		else if (theta_current >= 0 && theta_current < theta_desired)
		{
			//rotate ccw
			cout << "\nCCW";
			pw_r = pw_ccw + (theta_desired - theta_current)*slope;
			pw_l = pw_r;
			return 0;
		}
		else if (theta_current < 0 && abs(theta_current) < (PI - theta_desired))
		{
			//rotate ccw
			cout << "\nCCW";
			pw_r = pw_ccw + (theta_desired - theta_current)*slope;
			pw_l = pw_r;
			return 0;
		}
	}
	//second quadrant
	else if (theta_desired > PI / 2 && theta_desired <= PI)
	{
		//cout << "\ntheta_desired is in the second quadrant.";

		if (theta_current > theta_desired && theta_current <= PI)
		{
			//rotate cw
			cout << "\nCW";
			pw_r = pw_cw - (theta_current - theta_desired)*slope;
			pw_l = pw_r;
			return 0;
		}
		else if (theta_current < 0 && abs(theta_current) >= (PI - theta_desired))
		{
			//rotate cw
			cout << "\nCW";
			pw_r = pw_cw - ((2 * PI + theta_current) - theta_desired)*slope;
			pw_l = pw_r;
			return 0;
		}
		else if (theta_current >= 0 && theta_current < theta_desired)
		{
			cout << "\nCCW";
			pw_r = pw_ccw + (theta_desired - theta_current)*slope;
			pw_l = pw_r;
			return 0;
		}
		else if (theta_current < 0 && abs(theta_current) <= (PI - theta_desired))
		{
			cout << "\nCCW";
			pw_r = pw_ccw + (theta_desired - theta_current)*slope;
			pw_l = pw_r;
			return 0;
		}
	}
	//third quadrant
	else if (theta_desired >= -PI && theta_desired <= -PI / 2)
	{
		//cout << "\ntheta_desired is in the third quadrant.";

		if (theta_current < theta_desired && theta_current > -PI)
		{
			//rotate ccw
			cout << "\nCCW";
			pw_r = pw_ccw - (theta_current - theta_desired)*slope;
			pw_l = pw_r;
			return 0;
		}

		else if (theta_current > 0 && theta_current > (PI + theta_desired))
		{
			//rotate ccw
			cout << "\nCCW";
			pw_r = pw_ccw - ((theta_current - 2 * PI) - theta_desired)*slope;
			pw_l = pw_r;
			return 0;
		}
		else if (theta_current >= 0 && theta_current < (PI + theta_desired))
		{
			cout << "\nCW";
			pw_r = pw_cw - (-theta_desired + theta_current)*slope;
			pw_l = pw_r;
			return 0;
		}
		else if (theta_current < 0 && (theta_current) >(theta_desired))
		{
			cout << "\nCW";
			pw_r = pw_cw - (-theta_desired + theta_current)*slope;
			pw_l = pw_r;
			return 0;
		}
	}
	//fourth quadrant
	else if (theta_desired > -PI / 2 && theta_desired < 0)
	{
		//cout << "\ntheta_desired is in the fourth quadrant.";

		if (theta_current < theta_desired && theta_current > -PI)
		{
			//rotate ccw
			cout << "\nCCW";
			pw_r = pw_ccw - (theta_current - theta_desired)*slope;
			pw_l = pw_r;
			return 0;
		}

		else if (theta_current > 0 && theta_current > (PI + theta_desired))
		{
			//rotate ccw
			cout << "\nCCW";
			pw_r = pw_ccw - ((theta_current - 2 * PI) - theta_desired)*slope;
			pw_l = pw_r;
			return 0;
		}
		else if (theta_current >= 0 && theta_current < (PI + theta_desired))
		{
			cout << "\nCW";
			pw_r = pw_cw - (-theta_desired + theta_current)*slope;
			pw_l = pw_r;
			return 0;
		}
		else if (theta_current < 0 && (theta_current) >(theta_desired))
		{
			cout << "\nCW";
			pw_r = pw_cw - (-theta_desired + theta_current)*slope;
			pw_l = pw_r;
			return 0;
		}

	}

	return 0;
}

void controller(int* mini_destinationx, int* mini_destinationy, double theta, int& pw_l, int& pw_r)
{
	double PI = atan(1) * 4;
	double theta_expected = atan2(mini_destinationy[1] - mini_destinationy[0], mini_destinationx[1] - mini_destinationx[0]);

	if (abs(theta - theta_expected) > 30 * (PI / 180)) // Tolerance
	{
		rotate_robot(pw_r, pw_l, theta, theta_expected);

		if (pw_r > 2000) pw_r = 2000;
		if (pw_l > 2000) pw_l = 2000;
		if (pw_r < 1000) pw_r = 1000;
		if (pw_l < 1000) pw_l = 1000;
	}
	else {
		int forward = 150;
		pw_l = 1500 - forward;
		pw_r = 1500 + forward;
	}
}

int calculate_opponent_position(int &x, int &y, int ic[], int jc[], double Ravg[], double Gavg[], double Bavg[], int nlabels, double &theta, int &io, int &jo)
{
	int nl, ir, jr, ig, jg, flag = 1;
	static int ir_p = 0, jr_p = 0, ig_p = 0, jg_p = 0;
	int width = 640;
	int height = 480;
	ir = 0;
	jr = 0;
	ig = 0;
	jg = 0;

	for (nl = 0; nl <= nlabels; nl++)
	{
		//Orange target
		if (Ravg[nl] > 0.8 && Gavg[nl] > 0.6 && Bavg[nl] < 0.5)
		{
			ir = ic[nl];
			jr = jc[nl];

		}

		//Blue target
		if (Ravg[nl] < 0.2 && Gavg[nl] > 0.5 && Bavg[nl] > 0.75)
		{
			ig = ic[nl];
			jg = jc[nl];
		}

	}

	//assume that the green target is on the left and the red target is on the 
	//the angle the robot has turned is (jr-jg)/(ir-ig)
	//the center of the robot is the average position between the red and green target

	//cout << "\nig=" << ig << "\tjg=" << jg;
	//cout << "\nir=" << ir << "\tjr=" << jr;

	//if the robot went partially out of bounds, then use the previous value for an approximation based on the remaining color still in bound
	//and send a command to either move it back in bounds
	//
	//Returning 1 means to reverse
	//Returning 2 means to go forward
	if (ig == 0 && jg == 0)
	{
		cout << "\Blue marker is out of bounds.";
		ig = ig_p;

		x = (ig + ir) / 2;
		y = (jg + jr) / 2;

		theta = atan2((jg - jr), (ig - ir));

		//if the green indicator left from the right side, then the robot is facing forward, need to reverse it
		return 1;

	}
	else if (ir == 0 && jr == 0)
	{
		cout << "\nOrange marker is out of bounds.";
		ir = ir_p;

		x = (ig + ir) / 2;
		y = (jg + jr) / 2;

		theta = atan2((jg - jr), (ig - ir));

		return 2;
	}


	ir_p = ir;
	jr_p = jr;
	ig_p = ig;
	jg_p = jg;

	x = ir;
	y = jr;

	io = ir;//centroid of orange marker
	jo = jr;//centroid of orange marker


	theta = atan2((jg - jr), (ig - ir));

	return 0;
}

bool are_robots_close(int x, int y, int xo, int yo)
{
	const double distance_threshold = 150;

	double distance = sqrt(pow(x - xo, 2) + pow(y - yo, 2));

	cout << distance << endl;

	return distance <= distance_threshold;
}

int position_laser(int &pw_laser, double theta, int io, int jo, int ig, int jg) {
	double lower_bound, upper_bound, theta_desired;
	double pw_rad = 500 / (PI / 2);
	double il, jl;
	il = S1->P[1]->xg;
	jl = S1->P[1]->yg;

	theta_desired = atan2(jo - jl, io - il);

	//first quadrant
	if (theta >= 0 && theta <= PI / 2) {

		upper_bound = theta + PI / 2;
		lower_bound = theta - PI / 2;

		//laser is pointing towards target, leave it pointing ahead
		if (theta_desired == theta)
			pw_laser = 1500;

		//rotate laser CW to point at theta desired between 1000-1500
		else if (theta_desired >= lower_bound && theta_desired < theta)
			pw_laser = 1500 - ((theta - theta_desired)*pw_rad);

		//rotate laser CCW to point at theta desired 1500-2000	
		else if (theta_desired <= upper_bound && theta_desired > theta)
			pw_laser = 1500 + ((theta_desired - theta)*pw_rad);

	}

	//second quadrant
	else if (theta > PI / 2 && theta <= PI) {

		upper_bound = -PI + (PI / 2 - (PI - theta));
		lower_bound = theta - PI / 2;

		if (theta_desired == theta)
			pw_laser = 1500;

		//rotate laser CW to point at theta desired between 1000-1500
		else if (theta_desired >= lower_bound && theta_desired < theta)
			pw_laser = 1500 - ((theta - theta_desired)*pw_rad);

		//rotate laser CCW to point at theta desired 1500-2000	
		///between PI and theta
		else if (theta_desired <= PI && theta_desired > theta)
			pw_laser = 1500 + ((theta_desired - theta)*pw_rad);
		//between -PI and upper_bound
		else if (theta_desired >= -PI && theta_desired <= upper_bound)
			pw_laser = 1500 + ((abs(-PI - theta_desired) + (PI - theta))*pw_rad);
	}

	//third quadrant
	else if (theta >= -PI && theta <= -PI / 2) {

		upper_bound = theta + PI / 2;
		lower_bound = PI - (PI / 2 - (-PI - theta));

		if (theta_desired == theta)
			pw_laser = 1500;

		//rotate laser CCW to point at theta desired between 1500-2000
		else if (theta_desired <= upper_bound && theta_desired > theta)
			pw_laser = 1500 + ((theta_desired - theta)*pw_rad);

		//rotate laser CW to point at theta desired 1500-1000	
		///between -PI and theta
		else if (theta_desired >= -PI && theta_desired < theta)
			pw_laser = 1500 - ((theta - theta_desired)*pw_rad);
		//between PI and lower_bound
		else if (theta_desired <= PI && theta_desired >= lower_bound)
			pw_laser = 1500 - ((abs(-PI - theta) + (PI - theta_desired))*pw_rad);
	}

	//fourth quadrant
	else if (theta > -PI / 2 && theta < 0) {

		upper_bound = theta + PI / 2;
		lower_bound = theta - PI / 2;

		//laser is pointing towards target, leave it pointing ahead
		if (theta_desired == theta)
			pw_laser = 1500;

		//rotate laser CW to point at theta desired between 1000-1500
		else if (theta_desired >= lower_bound && theta_desired < theta)
			pw_laser = 1500 - ((theta - theta_desired)*pw_rad);

		//rotate laser CCW to point at theta desired 1500-2000	
		else if (theta_desired <= upper_bound && theta_desired > theta)
			pw_laser = 1500 + ((theta_desired - theta)*pw_rad);

	}
	return 0;
}

bool isvalid_shot(int x, int y, image obstacle_laser, int width)
{
	int k;

	k = x + y * width;

	ibyte *po;

	po = obstacle_laser.pdata + k;
	//check if the point is out of bounds or is located on an obstacle.

	if (*po)
	{
		cout << "\nPoint is located on an obstacle.";
		return false;
	}
	else
	{
		return true;
	}
}

int shoot_laser(int& pw_laser, double& thetar, int & io, int & jo, int& ig, int& jg, int height, int width, image obstacle_laser, int &laser)
{
	double x0, y0, theta, r;
	int i, j;
	int previous;
	int current = pw_laser;
	static int count = 0;
	// get start point of the laser
	x0 = S1->P[1]->xg;
	y0 = S1->P[1]->yg;

	r = sqrt((x0 - io)*(x0 - io) + (y0 - jo)*(y0 - jo));

	// robot theta + laser alpha
	theta = S1->P[1]->x[1] + S1->P[1]->x[4];


	for (int n = 0; n < r; n++) {

		i = (int)(x0 + n * cos(theta));
		j = (int)(y0 + n * sin(theta));

		if (i < 3) return 0;
		if (i > width - 3) return 0;
		if (j < 3) return 0;
		if (j > height - 3) return 0;

		if (!isvalid_shot(i + 10, j + 10, obstacle_laser, width)) {
			cout << "\nShot is not valid";
			laser = 0;
			count = 0;
			return 0;
		}

		if (!isvalid_shot(i - 10, j - 10, obstacle_laser, width)) {
			cout << "\nShot is not valid";
			laser = 0;
			count = 0;
			return 0;
		}

	}
	count++;
	cout << "\nShot is valid, waiting on Count > 20. Current count : " << count;
	if (count > 30 && pw_laser > 1050 && pw_laser < 1950 ) {
		cout << "\nLaser fired!";
		position_laser(pw_laser, thetar, io, jo, ig, jg);
		laser = 1;
	}
	return 0;
}

int draw_circle(image grey, int rmin, int rmax, int io, int jo)
{

	//this function draws a circle in a binary image.
	int i, r, k;
	double theta;
	int width, height;
	int x, y;
	width = grey.width;
	height = grey.height;

	ibyte *pg;

	/*
	for (r = rmin; r < rmax; r++)
	{
	for (theta = -PI; theta < PI; theta += PI / 200)
	{
	x = int(r*cos_taylor(theta)) + io;
	y = int(r*sin_taylor(theta)) + jo;

	if (x > width || y > height || x < 0 || y < 0)
	{
	continue;
	}
	else{
	k = x + y*width;

	//cout << "\nx=" << x << "\ty=" << y;
	//_getch();
	pg = (grey.pdata + k);
	*pg = 255;
	}
	}
	}
	*/

	for (r = rmin; r <= rmax; r++)
	{
		for (theta = -PI / 2; theta <= PI / 2; theta += PI / 500)
		{
			//x = int(r*cos_taylor(theta)) + io;
			//y = int(r*sin_taylor(theta)) + jo;
			x = int(r*cos(theta)) + io;
			y = int(r*sin(theta)) + jo;

			if (x > width || y > height || x < 0 || y < 0)
			{
				continue;
			}
			else {
				k = x + y * width;

				//cout << "\nx=" << x << "\ty=" << y;
				//_getch();
				pg = (grey.pdata + k);
				*pg = 255;
			}

			mirror_point(grey, io, jo, x, y, true);
		}
	}

	return 0;
}

int mirror_point(image grey, int xc, int yc, int x, int y, bool vertical)
{
	int i, j, k;
	int im, jm, km;
	int size;
	int width, height;

	width = grey.width;
	height = grey.height;

	size = width * height;

	if (xc<0 || xc>width || yc<0 || yc>height)
	{
		return 1;
	}

	ibyte *pg;

	im = 0;
	jm = 0;

	if (vertical)
	{
		//mirror along a vertical plane
		if (x > xc) //if the point located to the right of the mirror
		{
			//mirrored point will be located the same number of pixels away from the mirror, but in the opposite direction;
			im = xc - (x - xc);
			jm = y;

			if (im<0 || im>width || jm<0 || jm>height)
			{
				return 1;
			}
			else
			{
				km = im + jm * width;
				pg = grey.pdata + km;
				*pg = 255;
			}
		}
		else if (x < xc)
		{
			im = xc + (xc - x);
			jm = y;

			if (im<0 || im>width || jm<0 || jm>height)
			{
				return 1;
			}
			else
			{
				km = im + jm * width;
				pg = grey.pdata + km;
				*pg = 255;
			}
		}
	}
	else
	{
		//mirror along a horizontal plane
		if (y > yc) //if the point located to the right of the mirror
		{
			//mirrored point will be located the same number of pixels away from the mirror, but in the opposite direction;
			im = x;
			jm = yc + (y - yc);

			if (im<0 || im>width || jm<0 || jm>height)
			{
				return 1;
			}
			else
			{
				km = im + jm * width;
				pg = grey.pdata + km;
				*pg = 255;
			}
		}
		else if (y < yc)
		{
			im = x;
			jm = yc - (yc - y);

			if (im<0 || im>width || jm<0 || jm>height)
			{
				return 1;
			}
			else
			{
				km = im + jm * width;
				pg = grey.pdata + km;
				*pg = 255;
			}
		}
	}

	return 0;
}

int escape_point(int & io, int & jo, int & ig, int & jg, int height, int width, image obstacle_laser, int ib[10], int jb[10], int &ie, int &je)
{
	int min = 0;
	double min_distance;
	double x0, y0, theta, r;
	int i, j;
	static int count = 0;

	for (int i = 1; i < 10; i++) {
		//		cout <<"++++++++++++++++++++++\ndistnace from obstacle " << i << " to robot centorid " <<sqrt((ig - ib[i])*(ig - ib[i]) + (jg - jb[i])*(jg - jb[i]));

		if (i == 1) {
			min = 1;
			min_distance = sqrt((ig - ib[i])*(ig - ib[i]) + (jg - jb[i])*(jg - jb[i]));
		}
		else if (sqrt((ig - ib[i])*(ig - ib[i]) + (jg - jb[i])*(jg - jb[i])) < min_distance) {
			min = i;
			min_distance = sqrt((ig - ib[i])*(ig - ib[i]) + (jg - jb[i])*(jg - jb[i]));
		}
	}

	//cout << "-------------------------\nmin distance from our robot to obsatcle is " << min_distance;


	theta = atan2(jb[min] - jo, ib[min] - io);
	r = sqrt((io - ib[min])*(io - ib[min]) + (jo - jb[min])*(jo - jb[min]));



	i = (int)(io + (r + 100) * cos(theta));
	j = (int)(jo + (r + 100) * sin(theta));


	if (i < 3) return 0;
	if (i > width - 3) return 0;
	if (j < 3) return 0;
	if (j > height - 3) return 0;

	ie = i;
	je = j;
	cout << "\n ie is " << ie;
	cout << "\n je is " << je;



	return 0;
}


int create_obstacle_image(image rgb, image &obstacle, image &obstacle_laser, image labels,
	int& nlabels, int ic[], int jc[], double Ravg[], double Gavg[], double Bavg[],
	int thresh, int ib[3], int jb[3], int r_obstacles[3], int*& obstaclesx, int*& obstaclesy, int& num_obstacles)
{
	int io, jo;
	int height, width;

	double thetao;
	int n;
	height = rgb.height;
	width = rgb.width;

	//find the obstacle course
	find_obstacle(rgb, obstacle, thresh);
	copy(obstacle, obstacle_laser);

	binary_centroid(obstacle, ib, jb, num_obstacles, nlabels); //ib and jb contain the centroids of the obstacles

	int num_obs = 0;

	for (int i = 1; i < 3; i++) {
		if (ib[i] != 0) num_obs++;
		else break;
	}

	for (int i = 1; i <= num_obs; i++) {
		draw_circle(obstacle, 27, 75, ib[i], jb[i]);

		copy(obstacle, rgb);
		view_rgb_image(rgb);
	}

	binary_centroid(obstacle, ib, jb, obstaclesx, obstaclesy, num_obstacles); //ib and jb contain the centroids of the obstacles

	for (int i = 1; i <= num_obs; i++) {
		r_obstacles[i] = get_radius(obstacle, ib[i], jb[i]);
	}

	return 0;
}

int collision(int* R, int ir, int jr, int* io, int* jo, int threshold, int num_obstacles)
{


	for (int i = 1; i <= num_obstacles; i++) {
		double dist;

		dist = sqrt((io[i] - ir)*(io[i] - ir) + (jo[i] - jr)*(jo[i] - jr));

		int r = R[i];

		if (dist < (R[i] + threshold))
		{
			return 1;
		}
	}

	return 0;
}

int find_obstacle(image rgb, image &obstacle, int thresh)
{
	//this function will determine whether the object is considered an obstacle and will 
	//create a binary image of obstacles.
	image grey1, grey2, label;

	int width, height, nlabels;
	int i, k, size;

	width = rgb.width;
	height = rgb.height;
	size = width * height;

	if (obstacle.width != width || obstacle.height != height)
	{
		cout << "\nError, the obstacle image is not the same size as the rgb image.";
		return 1;
	}

	if (obstacle.type != GREY_IMAGE)
	{
		cout << "\nError, the obstacle image should be a grey image.";
		return 1;
	}

	//reset the obstacle function so it is completely black.
	copy(rgb, obstacle);
	ibyte *pg;

	for (k = 0; k < size; k++)
	{
		pg = obstacle.pdata + k;
		*pg = 0;
	}

	grey1.type = GREY_IMAGE;
	grey1.width = width;
	grey1.height = height;
	allocate_image(grey1);

	grey2.type = GREY_IMAGE;
	grey2.width = width;
	grey2.height = height;
	allocate_image(grey2);

	label.type = LABEL_IMAGE;
	label.width = width;
	label.height = height;
	allocate_image(label);

	//Obstacles will be considered to be dark objects (mostly black) and large number of pixels
	//This function should be adjusted to accomodate more obstacles (smaller obstacles, different colors, etc).
	//We should consider using the edge detection software 
	//Because we have black objects as obstacles, then we want to invert the image as those will become white.
	get_labels_inverted(rgb, grey1, grey2, label, nlabels, thresh);
	//note that because the background is white, the first label is not the background. Therefore, label 0 could be an obstacle

	//cout << "\nnlabels=" << nlabels;
	//copy(grey1, rgb);
	//view_rgb_image(rgb);
	//_getch();

	i2byte *pl;

	for (i = 1; i < nlabels; i++)
	{
		//cout << "\nis_obstacle(grey1, label, i)" << is_obstacle(grey1, label, i);
		if (is_obstacle(grey1, label, i)) //if the obstacle is found, then make add it to the binary obstacle image.
		{
			for (k = 0; k < size; k++)
			{
				pl = (i2byte *)label.pdata + k;
				if (*pl == i)
				{
					pg = obstacle.pdata + k;
					*pg = 255;
				}
			}
		}
	}

	//copy(obstacle, rgb);
	//view_rgb_image(rgb);
	//_getch();

	free_image(grey1);
	free_image(grey2);
	free_image(label);

	return 0;
}



int binary_centroid(image grey, int ic[10], int jc[10], int*& obstaclesx, int*& obstaclesy, int& num_obstacles)
{
	//Find the centroid of a labeled image using a binary image (black & white).
	//the input image should already be in binary.

	static bool init = true;
	int i, j, k, n;
	ibyte *pa;
	i2byte *pl;
	double m = 0.0;
	double rho = 1.0;
	int width, height, size;
	image label;
	int nlabel;

	width = grey.width;
	height = grey.height;
	size = width * height;

	label.width = width;
	label.height = height;
	label.type = LABEL_IMAGE;

	allocate_image(label);

	if (width != label.width || height != label.height)
	{
		cout << "\nError, input label doesn't have the correct pixel dimensions.";
		return 1;
	}

	if (LABEL_IMAGE != label.type)
	{
		cout << "\nError, input label isn't a LABEL_IMAGE type.";
		return 1;
	}

	label_image(grey, label, nlabel);

	pa = grey.pdata;
	pl = (i2byte *)label.pdata;

	vector<int> ox;
	vector<int> oy;

	num_obstacles = 0;

	for (n = 1; n <= nlabel; n++)
	{
		for (k = 0; k < size; k++)
		{
			//pa = grey.pdata + k * 3;
			pa = grey.pdata + k;
			pl = (i2byte *)label.pdata + k;

			if (n == *pl)
			{
				i = k % width;
				j = (k - i) / width;

				ox.push_back(i);
				oy.push_back(j);

				m += rho;
				ic[n] += i;
				jc[n] += j;
			}
		}
		ic[n] = ic[n] / m;
		jc[n] = jc[n] / m;

		num_obstacles += m;
		m = 0;
	}

	obstaclesx = new int[num_obstacles];
	obstaclesy = new int[num_obstacles];

	std::copy(ox.begin(), ox.end(), obstaclesx);
	std::copy(oy.begin(), oy.end(), obstaclesy);

	free_image(label);

	init = true;

	return 0;
}

int binary_centroid(image grey, int ic[10], int jc[10], int& num_obstacles, int& nlabel)
{
	//Find the centroid of a labeled image using a binary image (black & white).
	//the input image should already be in binary.

	static bool init = true;
	int i, j, k, n;
	ibyte *pa;
	i2byte *pl;
	double m = 0.0;
	double rho = 1.0;
	int width, height, size;
	image label;

	width = grey.width;
	height = grey.height;
	size = width * height;

	label.width = width;
	label.height = height;
	label.type = LABEL_IMAGE;

	allocate_image(label);

	if (width != label.width || height != label.height)
	{
		cout << "\nError, input label doesn't have the correct pixel dimensions.";
		return 1;
	}

	if (LABEL_IMAGE != label.type)
	{
		cout << "\nError, input label isn't a LABEL_IMAGE type.";
		return 1;
	}

	label_image(grey, label, nlabel);

	pa = grey.pdata;
	pl = (i2byte *)label.pdata;

	for (n = 1; n <= nlabel; n++)
	{
		for (k = 0; k < size; k++)
		{
			//pa = grey.pdata + k * 3;
			pa = grey.pdata + k;
			pl = (i2byte *)label.pdata + k;

			if (n == *pl)
			{
				i = k % width;
				j = (k - i) / width;

				m += rho;
				ic[n] += i;
				jc[n] += j;
			}
		}
		ic[n] = ic[n] / m;
		jc[n] = jc[n] / m;

		num_obstacles += m;
		m = 0;
	}

	free_image(label);

	init = true;

	return 0;
}

int get_labels_inverted(image rgb, image grey1, image grey2, image label, int &nlabels, int thresh)
{
	//Get the labels in an image within a certain threshold.
	int width, height;

	width = rgb.width;
	height = rgb.height;

	if (RGB_IMAGE != rgb.type)
	{
		cout << "\nError, input rgb isn't a RGB_IMAGE type.";
		return 1;
	}

	if (width != grey1.width || height != grey1.height)
	{
		cout << "\nError, input grey1 doesn't have the pixel dimensions.";
		return 1;
	}

	if (width != grey2.width || height != grey2.height)
	{
		cout << "\nError, input grey2 doesn't have the pixel dimensions.";
		return 1;
	}

	if (width != label.width || height != label.height)
	{
		cout << "\nError, input label doesn't have the pixel dimensions.";
		return 1;
	}

	if (GREY_IMAGE != grey1.type)
	{
		cout << "\nError, input grey1 isn't a GREY_IMAGE type.";
		return 1;
	}

	if (GREY_IMAGE != grey2.type)
	{
		cout << "\nError, input grey2 isn't a GREY_IMAGE type.";
		return 1;
	}

	if (LABEL_IMAGE != label.type)
	{
		cout << "\nError, input label isn't a GREY_IMAGE type.";
		return 1;
	}

	//Get the greyscale image
	copy(rgb, grey1);
	copy(grey1, rgb);

	//Apply filter to greyscale image
	lowpass_filter(grey1, grey2);
	copy(grey2, grey1);
	copy(grey1, rgb);

	//Scale the image to improve contrast
	scale(grey1, grey2);
	copy(grey2, grey1);
	copy(grey1, rgb);

	//Apply a threshold to the image
	threshold(grey1, grey2, thresh);
	copy(grey2, grey1);
	copy(grey1, rgb);

	//Invert the image
	invert(grey1, grey2);
	copy(grey2, grey1);
	copy(grey1, rgb);

	//Erode the image
	erode(grey1, grey2);
	copy(grey2, grey1);
	copy(grey1, rgb);

	//Dialate the image
	dialate(grey1, grey2);
	copy(grey2, grey1);
	copy(grey1, rgb);

	//Label the objects in the image
	copy(rgb, grey1);
	label_image(grey1, label, nlabels);

	return 0;

}

bool is_obstacle(image grey, image label, int n)
{
	//this will return whether the label for the image is considered an obstacle or not.
	int m, k, size;
	int width, height;

	width = grey.width;
	height = grey.height;

	size = width * height;

	if (label.width != width || label.height != height)
	{
		cout << "\nError, the label image is not the same size as the grey image.";
		return 1;
	}

	m = 0;
	//ibyte *pg;
	i2byte *pl;

	for (k = 0; k < size; k++)
	{
		pl = (i2byte *)label.pdata + k;

		if (*pl == n)
		{
			m++;
		}

	}

	//cout << "\nm=" << m;

	int pixel_thresh = 2000;
	//if the object has more than 100 pixels and is below the threshold from the obstacle function, then it is an obstacle.
	if (m > pixel_thresh)
	{
		return true;
	}
	else
	{
		return false;
	}

}

int move_opponent(int &pw_l, int &pw_r)
{

	if (KEY(VK_UP))
	{
		pw_l = pw_l - 50;
		pw_r = pw_r + 50;
	}
	else if (KEY(VK_DOWN))
	{
		pw_l = pw_l + 50;
		pw_r = pw_r - 50;
	}
	else if (KEY(VK_RIGHT))
	{
		pw_l = pw_l - 50;
		pw_r = pw_r - 50;
	}
	else if (KEY(VK_LEFT))
	{
		pw_l = pw_l + 50;
		pw_r = pw_r + 50;
	}

	if (pw_l > 2000)
	{
		pw_l = 2000;
	}

	if (pw_l < 1000)
	{
		pw_l = 1000;
	}

	if (pw_r > 2000)
	{
		pw_r = 2000;
	}

	if (pw_l < 1000)
	{
		pw_l = 1000;
	}

	return 0;
}

int get_radius(image& img, int x, int y)
{
	int r = 0;

	ibyte* pl;

	for (int k = x + y * img.width;;) {
		pl = img.pdata + k;

		if (*pl == 0) {
			break;
		}
		r++;
		x++;

		k = x + y * img.width;

		if (x == img.width) break;
	}

	return r;
}
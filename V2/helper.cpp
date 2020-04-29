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

	size = height*width;
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

	size = width*height;

	pa = grey.pdata;
	pl = (i2byte *)label.pdata;

	for (k = 0; k < size; k++)
	{
		pa = grey.pdata + k;
		pl = (i2byte *)label.pdata + k;

		if (nlabel == *pl)
		{
			//cout << endl << "*pl=" << *pl;
			i = k%width;
			j = (k - i) / width;

			//cout << endl << "i=" << i;
			//cout << endl << "j=" << j;
			rho = (double)*pa;
			m += rho;
			ic += rho*i;
			jc += rho*j;

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

int calculate_robot_position(int &x, int &y, int ic[], int jc[], double Ravg[], double Gavg[], double Bavg[], int nlabels, double &theta)
{
	//robot is straight if the centroid of the green and red objects are in the horizontal plane]
	//Assume that the only red and green color in the background is our robot.
	//Assume that the 0 degree angle is robot having the green circle at the right and red at the left.
	//note that the Ravg, Gavg and Bavg values range between 0 and 1. The thresholds in the if statement should be changed
	//depending on the lighting, camera settings, etc.

	//the variable x and y represent the position of the centroid of the robot (located between our two targets)
	//The angle theta is the orientation of the robot.

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

	x = (ig + ir) / 2;
	y = (jg + jr) / 2;

	theta = atan2((jg - jr), (ig - ir));

	//cout << "\nTheta=" << theta;

	return 0;
}
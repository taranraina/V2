#include <iostream>
#include "image_transfer5.h"
#include "vision.h"

using namespace std;

const int PWM_STATIONARY = 1500;
void init(double& width1, double& height1, int& N_obs, 
	double&D, double&Lx, double&Ly, 
	double&Ax, double&Ay, double&alpha_max,
	double* x_obs, double* y_obs);
void turn(int pwm_offset,int& pw_l,int& pw_r);
int get_labels(image rgb, image grey1, image grey2, image label, int &nlabels, int thresh);
int greyscale_centroid(image rgb, image grey, image grey1, image label, int &ic, int &jc, short int nlabel);
int calculate_average_RGB(image rgb, image label, int nlabel, double &R, double &G, double &B);
void init_image(image& img, int width, int height, int img_type);
int calculate_robot_position(int &x, int &y, int ic[], int jc[], double Ravg[], double Gavg[], double Bavg[], int nlabels, double &theta);
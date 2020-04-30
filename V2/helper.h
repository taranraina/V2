#include <iostream>
#include <cmath>
#include "image_transfer5.h"
#include "vision.h"
#include "robot.h"
#include "vision_simulation.h"
#include <Windows.h>

using namespace std;


const int PWM_STATIONARY = 1500;
void init(double& width1, double& height1, int& N_obs,
	double&D, double&Lx, double&Ly,
	double&Ax, double&Ay, double&alpha_max,
	double* x_obs, double* y_obs, double* size_obs);
void turn(int pwm_offset, int& pw_l, int& pw_r);
int get_labels(image rgb, image grey1, image grey2, image label, int &nlabels, int thresh);
int greyscale_centroid(image rgb, image grey, image grey1, image label, int &ic, int &jc, short int nlabel);
int calculate_average_RGB(image rgb, image label, int nlabel, double &R, double &G, double &B);
void init_image(image& img, int width, int height, int img_type);
int calculate_robot_position(int &x, int &y, int ic[], int jc[], double Ravg[], double Gavg[], double Bavg[], int nlabels, double &theta, int &ig, int &jg);
int calculate_opponent_position(int &x, int &y, int ic[], int jc[], double Ravg[], double Gavg[], double Bavg[], int nlabels, double &theta, int &io, int &jo);
void robot_circle(image& img, double PI, int x, int y);
void controller(int* mini_destinationx, int* mini_destinationy, double theta, int& pw_l, int& pw_r);
bool are_robots_close(int x, int y, int xo, int yo);
void convert_theta_positive(double& theta);
int position_laser(int &pw_laser, double theta, int io, int jo, int ig, int jg);
bool isvalid_shot(int x, int y, image obstacle_laser, int width);
int shoot_laser(int &io, int &jo, int height, int width, image obstacle_laser, int &laser);
int escape_point(int &io, int &jo, int &ig, int &jg, int height, int width, image obstacle_laser, int ib[10], int jb[10], int &ie, int &je);
int create_obstacle_image(image rgb, image &obstacle, image &obstacle_laser, image labels, int nlabels, int ic[], int jc[], double Ravg[], double Gavg[], double Bavg[], int thresh, int ib[10], int jb[10]);
int find_obstacle(image rgb, image &obstacle, int thresh);
int binary_centroid(image grey, int ic[10], int jc[10]);
int get_labels_inverted(image rgb, image grey1, image grey2, image label, int &nlabels, int thresh);
bool is_obstacle(image grey, image label, int n);
int move_opponent(int &pw_l, int &pw_r);
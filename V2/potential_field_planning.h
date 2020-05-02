#include <iostream>
#include <algorithm>
#include "image_transfer5.h"
#include "vision.h"

using namespace std;

const double KP = 1.0; // Attractive pulse
const double ETA = 40000.0; // Repulsive pulse
const double AREA_WIDTH = 30.0; // Potential area width

void calculate_potential_field(image& img, double& minx, double& miny, double**& pmap, double gx, double gy,
	double reso, double rr, int& xw, int& yw,
	int r_obstacles[3], int* obstaclesx, int* obstaclesy, double*& repulsive, int num_obstacles, int width, int height);

double calc_attractive_potential(double x, double y, double gx, double gy);
double calc_repulsive_potential(double x, double y, int* ox, int* oy, int num_obstacles, double rr, double*& repulsive, int xw);

void potential_field_planning(image& img, int* mini_destinationx,
	int* mini_destinationy, double sx, double sy, double gx,
	double gy, double reso, double rr, int r_obstacles[3],
	int* obstaclesx, int* obstaclesy, double*& repulsive, int num_obstacles, int width, int height);

void get_motion_model(int**& motion);
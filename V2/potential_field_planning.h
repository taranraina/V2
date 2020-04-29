#include <iostream>
#include <algorithm>
#include "image_transfer5.h"
#include "vision.h"

using namespace std;

const double KP = 1.0; // Attractive pulse
const double ETA = 4000.0; // Repulsive pulse
const double AREA_WIDTH = 200.0; // Potential area width

void calculate_potential_field(image& img, double& minx, double& miny, double**& pmap, double gx, double gy,
	double* ox, double* oy, int num_obstacles,
	double reso, double rr, int& xw, int& yw);
double calc_attractive_potential(double x, double y, double gx, double gy);
double calc_repulsive_potential(double x, double y, double* ox, double* oy, int num_obstacles, double rr);
void potential_field_planning(image& img, int* mini_destinationx, int* mini_destinationy, double sx, double sy, double gx,
	double gy, double* ox, double* oy, 
	int num_obstacles, double reso, double rr);

void get_motion_model(int**& motion);
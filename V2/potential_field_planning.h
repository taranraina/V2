#include <iostream>
#include <algorithm>
#include "image_transfer5.h"

using namespace std;

const double KP = 5.0; // Attractive pulse
const double ETA = 100.0; // Repulsive pulse
const double AREA_WIDTH = 30.0; // Potential area width

void calculate_potential_field(image& img, double gx, double gy, double* ox, double* oy, int num_obstacles, double reso, double rr);
double calc_attractive_potential(double x, double y, double gx, double gy);
double calc_repulsive_potential(double x, double y, double* ox, double* oy, int num_obstacles, double rr);
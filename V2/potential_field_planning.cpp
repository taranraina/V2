#include "potential_field_planning.h"

void calculate_potential_field(image& img, double gx, double gy, 
	double* ox, double* oy, int num_obstacles, 
	double reso, double rr)
{
	// These determine the closest / furthest obstacles on the map (from origin)
	double minx = *std::min_element(ox+1, ox + num_obstacles+1) - AREA_WIDTH / 2.0;
	double miny = *std::min_element(oy+1, oy + num_obstacles+1) - AREA_WIDTH / 2.0;
	double maxx = *std::max_element(ox+1, ox + num_obstacles+1) + AREA_WIDTH / 2.0;
	double maxy = *std::max_element(oy+1, oy + num_obstacles+1) + AREA_WIDTH / 2.0;

	// Determine the width and height of the new map
	const int xw = int(round((maxx - minx) / reso));
	const int yw = int(round((maxy - miny) / reso));

	// Creating our pmap
	double** pmap = new double*[xw];

	for (int i = 0; i < xw; i++) {
		pmap[i] = new double[yw];
	}

	for (int ix = 0; ix < xw; ix++) {
		int x = ix * reso + minx;
		for (int iy = 0; iy < yw; iy++){
			int y = iy * reso + miny;
			double ug = calc_attractive_potential(x, y, gx, gy);
			double uo = calc_repulsive_potential(x, y, ox, oy,num_obstacles, rr);
			double uf = ug + uo;
			pmap[ix][iy] = uf;
		}
	}
}

double calc_attractive_potential(double x, double y, double gx, double gy)
{
	double dist = sqrt(pow(x - gx, 2) + pow(y - gy, 2));
	return 0.5 * KP * dist;
}

double calc_repulsive_potential(double x, double y, double* ox, double* oy, int num_obstacles, double rr)
{
	int minid = -1;
	double dmin = DBL_MAX;

	// Determine the closest obstacle
	for (int i = 0; i < num_obstacles; i++) {
		double dist = sqrt(pow(x - ox[i], 2) + pow(y - oy[i], 2));
		if (dmin >= dist){
			dmin = dist;
			minid = i;
		}
	}

	// Calculate repulsive potential
	double dq = sqrt(pow(x - ox[minid], 2) + pow(y - oy[minid], 2));

	if (dq <= rr) {
		if (dq <= 0.1)
			dq = 0.1;

		return 0.5 * ETA * pow((1.0 / dq - 1.0 / rr), 2);
	}
	else return 0.0;
}
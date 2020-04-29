#include "potential_field_planning.h"

void calculate_potential_field(image& img, double& minx, double& miny, double**& pmap, double gx, double gy, 
	double* ox, double* oy, int num_obstacles, 
	double reso, double rr, int& xw, int& yw)
{
	const int num_particles = 360;
	const int radius_padding = 60;
	const double PI = atan(1) * 4;

	// Convert centroid to edges
	double* edgesx = new double[num_obstacles * num_particles];
	double* edgesy = new double[num_obstacles * num_particles];

	for (int i = 1; i <= num_obstacles; i++) {
		for (int j = 0; j < num_particles; j++) {
			edgesx[(i - 1) * num_particles + j] = ox[i] + radius_padding * cos(j * PI / 180.0);
			edgesy[(i - 1) * num_particles + j] = oy[i] + radius_padding * sin(j * PI / 180.0);
		}
	}

	for (int i = 0; i < num_obstacles * num_particles; i++) {
		draw_point_rgb(img, edgesx[i], edgesy[i], 255, 255, 255);
	}

	view_rgb_image(img);

	// These determine the closest / furthest obstacles on the map (from origin)
	minx = *std::min_element(edgesx, edgesx + num_obstacles * num_particles) - AREA_WIDTH / 2.0;
	miny = *std::min_element(edgesy, edgesy + num_obstacles * num_particles) - AREA_WIDTH / 2.0;
	double maxx = *std::max_element(edgesx, edgesx + num_obstacles * num_particles) + AREA_WIDTH / 2.0;
	double maxy = *std::max_element(edgesy, edgesy + num_obstacles * num_particles) + AREA_WIDTH / 2.0;

	// Determine the width and height of the new map
	xw = int(round((maxx - minx) / reso));
	yw = int(round((maxy - miny) / reso));

	// Creating our pmap
	pmap = new double*[xw];

	for (int i = 0; i < xw; i++) {
		pmap[i] = new double[yw];
	}

	for (int ix = 0; ix < xw; ix++) {
		int x = ix * reso + minx;
		for (int iy = 0; iy < yw; iy++) {
			int y = iy * reso + miny;
			double ug = calc_attractive_potential(x, y, gx, gy);
			double uo = calc_repulsive_potential(x, y, edgesx, edgesy, num_obstacles * num_particles, rr);
			double uf = ug + uo;
			pmap[ix][iy] = uf;
		}
	}

	delete[] edgesx;
	delete[] edgesy;
}

void free_pmap(double** pmap, int xw, int yw) {
	for (int i = 0; i < xw; i++) {
		delete[] pmap[i];
	}

	delete[] pmap;
}

void potential_field_planning(image& img, int* mini_destinationx, int* mini_destinationy, 
	double sx, double sy,
	double gx, double gy, double* ox, 
	double* oy, int num_obstacles, double reso, double rr)
{
	double** pmap = nullptr;
	double minx, miny;
	int xw, yw;

	calculate_potential_field(img, minx, miny, pmap,
		gx, gy, ox, oy, num_obstacles,
		reso, rr, xw, yw);

	// search path
	double d = sqrt(pow(sx - gx, 2) + pow(sy - gy, 2));
	int ix = int((sx - minx) / reso);
	int iy = int((sy - miny) / reso);
	int gix = int((gx - minx) / reso);
	int giy = int((gy - miny) / reso);

	int rx = int(sx);
	int ry = int(sy);

	int** motion;

	get_motion_model(motion);
	int counter = 0;

	while (d >= reso + 50) {
		double minp = DBL_MAX;
		int minix = -1, miniy = -1;

		// Iterating over each possible command
		for (int i = 0; i < 8; i++) {
			int inx = int(ix + motion[i][0]);
			int	iny = int(iy + motion[i][1]);

			double p;
			if (inx >= xw || iny >= yw) p = DBL_MAX;  // outside area
			else {
				int px = inx, py = iny;
				if (inx < 0) px = xw + inx;
				if (iny < 0) py = yw + iny;
				p = pmap[px][py];
			}
			
			if (minp > p) {
				minp = p;
				minix = inx;
				miniy = iny;
			}
		}

		ix = minix;
		iy = miniy;
		int xp = ix * reso + minx;
		int yp = iy * reso + miny;
		d = sqrt(pow(gx - xp, 2) + pow(gy - yp, 2));

		// Display the xp and yp on the image
		draw_point_rgb(img, xp, yp, 0, 0, 255);

		if (counter < 2) {
			mini_destinationx[counter] = xp;
			mini_destinationy[counter] = yp;
		}

		counter++;
	}

	free_pmap(pmap, xw, yw);

	for (int i = 0; i < 8; i++) {
		delete[] motion[i];
	}

	delete[] motion;

	view_rgb_image(img);
}

void get_motion_model(int**& motion) {
	motion = new int*[8];

	for (int i = 0; i < 8; i++) {
		motion[i] = new int[2];
	}

	motion[0][0] = 1;
	motion[0][1] = 0;
	motion[1][0] = 0;
	motion[1][1] = 1;
	motion[2][0] = -1;
	motion[2][1] = 0;
	motion[3][0] = 0;
	motion[3][1] = -1;
	motion[4][0] = -1;
	motion[4][1] = -1;
	motion[5][0] = -1;
	motion[5][1] = 1;
	motion[6][0] = 1;
	motion[6][1] = -1;
	motion[7][0] = 1;
	motion[7][1] = 1;
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
		return 0.5 * ETA * pow((1.0 / dq - 1.0 / rr), 2);
	}
	else return 0.0;
}
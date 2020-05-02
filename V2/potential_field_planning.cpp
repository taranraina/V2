#include "potential_field_planning.h"

void calculate_potential_field(image& img, double& minx,
	double& miny, double**& pmap, double gx, double gy,
	double reso, double rr, int& xw, int& yw,
	int r_obstacles[3], int* obstaclesx, int* obstaclesy, double*& repulsive, int size, int height, int width)
{
	const double PI = atan(1) * 4;

	// These determine the closest / furthest obstacles on the map (from origin)
	//minx = *std::min_element(obstaclesx, obstaclesx + size) - AREA_WIDTH / 2.0;
	//miny = *std::min_element(obstaclesy, obstaclesy + size) - AREA_WIDTH / 2.0;
	//double maxx = *std::max_element(obstaclesx, obstaclesx + size) + AREA_WIDTH / 2.0;
	//double maxy = *std::max_element(obstaclesy, obstaclesy + size) + AREA_WIDTH / 2.0;

	minx = 0;
	miny = 0;
	double maxx = width;
	double maxy = height;

	// Determine the width and height of the new map
	xw = int(round((maxx-minx) / reso));
	yw = int(round((maxy-miny) / reso));

	// Creating our pmap
	pmap = new double*[xw];

	for (int i = 0; i < xw; i++) {
		pmap[i] = new double[yw];
	}

	if (repulsive == nullptr) {
		repulsive = new double[height*width];

		for (int i = 0; i < height*width; i++) {
			repulsive[i] = -1;
		}
	}

	for (int ix = 0; ix < xw; ix++) {
		int x = ix * reso + minx;
		for (int iy = 0; iy < yw; iy++) {
			int y = iy * reso + miny;
			double ug = calc_attractive_potential(x, y, gx, gy);
			double uo = calc_repulsive_potential(x, y, obstaclesx, obstaclesy, size, rr, repulsive, width);
			double uf = ug + uo;
			pmap[ix][iy] = uf;
		}
	}
}

void free_pmap(double** pmap, int xw, int yw) {
	for (int i = 0; i < xw; i++) {
		delete[] pmap[i];
	}

	delete[] pmap;
}

void potential_field_planning(image& img, int* mini_destinationx,
	int* mini_destinationy, double sx, double sy, double gx,
	double gy, double reso, double rr, int r_obstacles[3],
	int* obstaclesx, int* obstaclesy, double*& repulsive, int num_obstacles, int width, int height)
{
	double** pmap = nullptr;
	double minx, miny;
	int xw, yw;

	calculate_potential_field(img, minx, miny,
		pmap, gx, gy,
		reso, rr, xw, yw,
		r_obstacles, obstaclesx, obstaclesy, repulsive, num_obstacles, width, height);

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
		if (counter == 2) break;

		counter++;
	}

	free_pmap(pmap, xw, yw);

	for (int i = 0; i < 8; i++) {
		delete[] motion[i];
	}

	delete[] motion;

	//view_rgb_image(img);
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

double calc_repulsive_potential(double x, double y, int* ox, int* oy, int num_obstacles, double rr, double*& repulsive, int width)
{
	int ix = x;
	int iy = y;
	int idx = ix + iy * width;
	if (repulsive[idx] == -1) {
		int minid = -1;
		double dmin = DBL_MAX;

		// Determine the closest obstacle
		for (int i = 0; i < num_obstacles; i++) {
			double dist = sqrt(pow(x - ox[i], 2) + pow(y - oy[i], 2));
			if (dmin >= dist) {
				dmin = dist;
				minid = i;
			}
		}

		// Calculate repulsive potential
		double dq = sqrt(pow(x - ox[minid], 2) + pow(y - oy[minid], 2));

		if (dq <= rr) {
			repulsive[idx] = 0.5 * ETA * pow((1.0 / dq - 1.0 / rr), 2);
			return repulsive[idx];
		}
		else {
			repulsive[idx] = 0;
			return repulsive[idx];
		}
	}
	else {
		return repulsive[idx];
	}
}
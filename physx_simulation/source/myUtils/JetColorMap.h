#ifndef _JET_COLOR_MAP_H
#define _JET_COLOR_MAP_H

#include <Eigen/Dense>


Eigen::Vector3f GetColour(double v, double vmin, double vmax)
{
	Eigen::Vector3f c;
	c << 1.0, 1.0, 1.0; // white
	float dv;

	if (v < vmin)
		v = vmin;
	if (v > vmax)
		v = vmax;
	dv = vmax - vmin;

	if (v < (vmin + 0.25 * dv)) {
		c[0] = 0;
		c[1] = 4 * (v - vmin) / dv;
	}
	else if (v < (vmin + 0.5 * dv)) {
		c[0] = 0;
		c[2] = 1 + 4 * (vmin + 0.25 * dv - v) / dv;
	}
	else if (v < (vmin + 0.75 * dv)) {
		c[0] = 4 * (v - vmin - 0.5 * dv) / dv;
		c[2] = 0;
	}
	else {
		c[1] = 1 + 4 * (vmin + 0.75 * dv - v) / dv;
		c[2] = 0;
	}

	return(c);
}

#endif
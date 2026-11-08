/*
 * Bridge between the C firmware (main.c) and Sofia's C++ trilateration
 * (../../../trilateration/trilateration.cpp). Her code is used as is.
 */

#include "trilat_bridge.h"
#include "trilateration.hpp"

extern "C" bool il_trilaterate_2d(const float anchor_xy_m[][2], const float dist_m[], int n,
				  float *x_m, float *y_m)
{
	Anchor anchors[IL_TRILAT_MAX_ANCHORS];

	if (n < 3 || n > IL_TRILAT_MAX_ANCHORS) {
		return false;
	}

	for (int i = 0; i < n; i++) {
		anchors[i].pos.x = anchor_xy_m[i][0];
		anchors[i].pos.y = anchor_xy_m[i][1];
		anchors[i].pos.z = 0.0f;
		anchors[i].distance = dist_m[i];
	}

	Point3D pos;

	if (!Trilateration::calculate_2d_position(anchors, (size_t)n, pos)) {
		return false;
	}

	*x_m = pos.x;
	*y_m = pos.y;
	return true;
}

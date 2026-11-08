#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IL_TRILAT_MAX_ANCHORS 8

/**
 * @brief 2D position from distances to >= 3 beacons (calls Trilateration::calculate_2d_position).
 *
 * @param anchor_xy_m  Beacon coordinates {x, y} in meters, n entries.
 * @param dist_m       Horizontal distances to those beacons in meters, n entries.
 * @param n            Number of beacons (3..IL_TRILAT_MAX_ANCHORS).
 * @return false if there are too few beacons or they are on one line.
 */
bool il_trilaterate_2d(const float anchor_xy_m[][2], const float dist_m[], int n,
		       float *x_m, float *y_m);

#ifdef __cplusplus
}
#endif

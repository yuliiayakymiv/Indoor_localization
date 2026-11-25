#pragma once

/*
 * C interface of the position code in ../../../trilateration (C++):
 *   - il_trilaterate_2d(): one-shot 2D trilateration (Trilateration::calculate_2d_position);
 *   - il_track_*(): the position tracker (Tracker::RangeTracker, see tracker.hpp) and the
 *     laser height logic (HeightFusion::Estimator, see height_fusion.hpp).
 *
 * The tracker is used from the main thread (distances) and from the UART thread (laser), so
 * every il_track_* call is protected by a mutex.
 */

#include <stdbool.h>
#include <stdint.h>

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

enum il_mode {
	IL_MODE_CAR = 0,   /* 2D, B1..B3 only, z = car antenna height */
	IL_MODE_DRONE = 1, /* 3D, all beacons, z with the laser */
};

/* Result of il_track_add_range() (same order as Tracker::RangeResult) */
enum il_range_result {
	IL_RANGE_ACCEPTED = 0,
	IL_RANGE_REJECTED = 1, /* spike / reflection, not used */
	IL_RANGE_RESTARTED = 2,
	IL_RANGE_IGNORED = 3,  /* beacon not used in this mode */
};

struct il_fix {
	bool valid;
	float x_m, y_m, z_m;
	float residual_m; /* RMS of the distance residuals */
	int used;         /* beacons used */
	float max_age_s;  /* age of the oldest distance used */
	bool laser_z;     /* z was corrected by the laser within the last second */
};

enum il_surface {
	IL_SURFACE_UNKNOWN = 0,
	IL_SURFACE_FLOOR = 1,
	IL_SURFACE_OBSTACLE = 2,
};

struct il_laser_result {
	bool valid;           /* range and tilt in limits, position known */
	float laser_h_m;      /* range * cos(roll) * cos(pitch) */
	float surface_m;      /* z - laser_h: height of what is below */
	enum il_surface surface;
	bool used_for_z;      /* the laser corrected z (floor below) */
	bool new_obstacle;    /* just flew over the edge of an obstacle */
	float obstacle_h_m;   /* height of the obstacle below */
	float x_m, y_m, z_m;  /* position at that moment */
};

void il_track_init(float car_z_m);
void il_track_set_anchor(int idx, float x_m, float y_m, float z_m);
void il_track_set_mode(enum il_mode mode);
enum il_mode il_track_mode(void);
bool il_track_anchor_used(int idx);

enum il_range_result il_track_add_range(int idx, float dist_m, int64_t t_ms);
void il_track_fix(int64_t t_ms, struct il_fix *out);

/** One laser rangefinder sample (drone mode). @return false if not used at all. */
bool il_track_laser(float range_m, float roll_rad, float pitch_rad, int64_t t_ms,
		    struct il_laser_result *out);

#ifdef __cplusplus
}
#endif

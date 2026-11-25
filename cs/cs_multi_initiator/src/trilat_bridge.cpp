/*
 * Bridge between the C firmware (main.c) and the C++ position code in ../../../trilateration
 * (Sofia's trilateration, the tracker and the laser height logic). See trilat_bridge.h.
 */

#include "trilat_bridge.h"
#include "trilateration.hpp"
#include "tracker.hpp"
#include "height_fusion.hpp"

#include <zephyr/kernel.h>

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

namespace {

K_MUTEX_DEFINE(track_lock);
Tracker::RangeTracker tracker;
HeightFusion::Estimator height;
int64_t last_laser_z_ms = -100000;

double seconds(int64_t t_ms)
{
	return (double)t_ms / 1000.0;
}

struct Lock {
	Lock() { k_mutex_lock(&track_lock, K_FOREVER); }
	~Lock() { k_mutex_unlock(&track_lock); }
};

} // namespace

extern "C" void il_track_init(float car_z_m)
{
	Lock l;
	Tracker::Config cfg;
	cfg.car_z_m = car_z_m;
	tracker.configure(cfg);
	height.reset();
}

extern "C" void il_track_set_anchor(int idx, float x_m, float y_m, float z_m)
{
	Lock l;
	tracker.set_anchor(idx, {x_m, y_m, z_m});
}

extern "C" void il_track_set_mode(enum il_mode mode)
{
	Lock l;
	tracker.set_mode(mode == IL_MODE_DRONE ? Tracker::Mode::Drone : Tracker::Mode::Car);
	height.reset();
}

extern "C" enum il_mode il_track_mode(void)
{
	Lock l;
	return tracker.mode() == Tracker::Mode::Drone ? IL_MODE_DRONE : IL_MODE_CAR;
}

extern "C" bool il_track_anchor_used(int idx)
{
	Lock l;
	return tracker.anchor_used(idx);
}

extern "C" enum il_range_result il_track_add_range(int idx, float dist_m, int64_t t_ms)
{
	Lock l;
	switch (tracker.add_range(idx, dist_m, seconds(t_ms))) {
	case Tracker::RangeResult::Accepted:
		return IL_RANGE_ACCEPTED;
	case Tracker::RangeResult::Rejected:
		return IL_RANGE_REJECTED;
	case Tracker::RangeResult::Restarted:
		return IL_RANGE_RESTARTED;
	default:
		return IL_RANGE_IGNORED;
	}
}

extern "C" void il_track_fix(int64_t t_ms, struct il_fix *out)
{
	Lock l;
	const Tracker::Fix f = tracker.fix(seconds(t_ms));

	out->valid = f.valid;
	out->x_m = f.pos.x;
	out->y_m = f.pos.y;
	out->z_m = f.pos.z;
	out->residual_m = f.residual_m;
	out->used = f.used;
	out->max_age_s = f.max_age_s;
	out->laser_z = tracker.mode() == Tracker::Mode::Drone && t_ms - last_laser_z_ms < 1000;
}

extern "C" bool il_track_laser(float range_m, float roll_rad, float pitch_rad, int64_t t_ms,
			       struct il_laser_result *out)
{
	Lock l;

	*out = {};
	if (tracker.mode() != Tracker::Mode::Drone) {
		return false;
	}

	// position (and z) now, from the latest distances and the previous laser samples
	const Tracker::Fix f = tracker.fix(seconds(t_ms));

	if (!f.valid) {
		return false;
	}

	const HeightFusion::Result r =
		height.update(f.pos.z, range_m, roll_rad, pitch_rad, seconds(t_ms));

	out->valid = r.valid;
	out->laser_h_m = r.laser_h_m;
	out->surface_m = r.surface_m;
	out->surface = (enum il_surface)r.surface;
	out->new_obstacle = r.new_obstacle;
	out->obstacle_h_m = r.obstacle_h_m;
	out->x_m = f.pos.x;
	out->y_m = f.pos.y;
	out->z_m = f.pos.z;

	if (r.valid && r.use_for_z &&
	    tracker.add_height(r.laser_h_m, height.config().laser_sigma_m, seconds(t_ms))) {
		out->used_for_z = true;
		out->z_m = r.laser_h_m;
		last_laser_z_ms = t_ms;
	}
	return r.valid;
}

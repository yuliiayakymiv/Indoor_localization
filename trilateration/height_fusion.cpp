#include "height_fusion.hpp"

#include <math.h>

namespace HeightFusion {

Estimator::Estimator() { reset(); }

void Estimator::reset() {
    state_ = Surface::Unknown;
    have_prev_ = false;
    prev_h_ = 0.0f;
    prev_z_ = 0.0f;
    prev_t_ = 0.0;
    obstacle_h_ = 0.0f;
}

Result Estimator::update(float z_ble_m, float range_m, float roll_rad, float pitch_rad,
                         double t_s) {
    Result r = {};
    r.surface = state_;

    const float tilt = cosf(roll_rad) * cosf(pitch_rad);
    const float min_tilt = cosf(cfg_.max_tilt_deg * 3.14159265f / 180.0f);

    if (!isfinite(z_ble_m) || !isfinite(range_m) || range_m < cfg_.min_range_m ||
        range_m > cfg_.max_range_m || !(tilt >= min_tilt)) {
        have_prev_ = false; // no jump detection across an unusable sample
        return r;
    }

    const float h = range_m * tilt;
    const float surface = z_ble_m - h;

    r.valid = true;
    r.laser_h_m = h;
    r.surface_m = surface;

    const bool recent = have_prev_ && (t_s - prev_t_) <= (double)cfg_.max_gap_s && t_s >= prev_t_;
    const float drop = recent ? prev_h_ - h : 0.0f;          // > 0: something came closer
    const float z_move = recent ? fabsf(z_ble_m - prev_z_) : 1e9f;

    if (surface < cfg_.floor_tol_m) {
        // the floor (a negative surface = BLE z is too low; the laser over the floor is right)
        state_ = Surface::Floor;
        r.use_for_z = true;
    } else if (surface >= cfg_.obstacle_min_m) {
        if (state_ == Surface::Floor && drop >= cfg_.jump_min_m && z_move < cfg_.ble_still_m) {
            // the laser distance dropped at once, the drone did not move down: an obstacle
            state_ = Surface::Obstacle;
            obstacle_h_ = surface;
            r.new_obstacle = true;
        } else if (state_ == Surface::Obstacle) {
            // still above it; follow its height slowly (the top may not be flat)
            obstacle_h_ += 0.3f * (surface - obstacle_h_);
        } else {
            // far from the floor without a jump (e.g. started above a table, or BLE z drifted):
            // we do not know what is below
            state_ = Surface::Unknown;
        }
    } else if (state_ == Surface::Floor) {
        // between the two thresholds: probably still the floor, but do not correct z with it
        r.use_for_z = false;
    }

    r.surface = state_;
    r.obstacle_h_m = (state_ == Surface::Obstacle) ? obstacle_h_ : 0.0f;

    have_prev_ = true;
    prev_h_ = h;
    prev_z_ = z_ble_m;
    prev_t_ = t_s;
    return r;
}

} // namespace HeightFusion

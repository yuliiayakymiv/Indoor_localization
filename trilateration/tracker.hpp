#pragma once

/*
 * Position tracker of the mobile node.
 *
 * The beacons are measured one after another (one beacon every ~1.5 s, a full round takes
 * several seconds), so the distances of one round belong to different moments. The old way
 * (median of the last 5 distances of every beacon, then trilateration once per round) is
 * precise while standing still but lags 10+ s and drops real motion as "outliers".
 *
 * Here:
 *  1. Every beacon keeps one filtered distance. How much it is filtered depends on the speed
 *     of the object (estimated from the positions of the last round):
 *       - standing still: a new distance is averaged in (precise, like the old median), and a
 *         distance more than 0.35 m off is a spike (reflection) and is dropped once;
 *       - moving: the new distance is taken as is (no lag), and the spike gate opens by the
 *         distance the object could have covered since that beacon was measured.
 *     Two "spikes" in a row are a real change and are taken.
 *     (Tested on a PC against the old median filter and plain least squares, see
 *     test_tracker.cpp: as precise as the median standing still, no lag when moving.)
 *  2. Position = least squares + Gauss-Newton from the latest filtered distance of every
 *     beacon (Trilateration::*), after every new distance, not once per round.
 *  3. Modes:
 *       CAR    2D, only beacons 0..car_beacons-1 (B1..B3), the 4th beacon is ignored.
 *              z = antenna height of the car; slant distances are reduced to the floor.
 *       DRONE  3D, all beacons with a known position. z goes through its own small Kalman
 *              filter, which also takes the height from a laser rangefinder (add_height()),
 *              much more precise than z from BLE. With only 3 beacons in one plane the
 *              solution is taken above the plane.
 *
 * Portable C++ (no Zephyr): tested on a PC, see test_tracker.cpp.
 */

#include <stdint.h>
#include "trilateration.hpp"

namespace Tracker {

constexpr int MAX_ANCHORS = 8;

enum class Mode : uint8_t { Car = 0, Drone = 1 };

enum class RangeResult : uint8_t {
    Accepted,     // used by the beacon's distance filter
    Rejected,     // spike / reflection, not used
    Restarted,    // the beacon's distance filter restarted at this distance (real jump)
    Ignored,      // beacon not used in this mode, unknown position, or invalid distance
};

struct Config {
    float range_sigma_m = 0.15f;      // 1-sigma noise of one CS distance
    // Spike gate: a new distance is rejected (once) if it differs from the beacon's current
    // distance by more than gate_min_m + gate_speed_factor * speed * time since the last one.
    // Standing still that is 0.35 m (a reflection of +0.9 m is dropped); moving, the gate opens
    // by the distance the object could have covered.
    float gate_min_m = 0.35f;
    float gate_speed_factor = 2.5f;
    // Averaging: standing still a new distance is mixed in with this weight (like a median of
    // the last few); moving (speed * dt > still_motion_m) the new distance is taken as is, so
    // there is no lag.
    float alpha_still = 0.35f;
    float still_motion_m = 0.15f;
    float max_age_s = 10.0f;          // a beacon not measured for longer is not used
    float car_z_m = 0.0f;             // antenna height of the car above the floor
    int car_beacons = 3;              // CAR mode uses beacons 0..car_beacons-1
    float max_range_m = 30.0f;
    // DRONE height filter
    float drone_default_height_m = 1.0f; // start height above the beacon plane (no laser yet)
    float z_accel = 0.5f;             // process noise of the z filter, (m/s^2)^2/Hz
    float z_ble_sigma_m = 0.30f;      // noise of z from BLE trilateration
};

struct Fix {
    bool valid;
    Point3D pos;
    float residual_m; // RMS of the distance residuals (quality: small = consistent)
    int used;         // beacons used
    float max_age_s;  // age of the oldest distance used
};

/** Kalman filter on one value and its rate: [v, dv/dt]. */
class Filter1D {
public:
    void reset() { init_ = false; }
    bool initialized() const { return init_; }
    void init(float v, double t, float var = 0.04f, float rate_var = 0.25f);
    void predict_to(double t, float q);
    /** Gate: rejected (state unchanged) if |z - v| > gate. @return true if used */
    bool update(float z, float r_var, float gate);
    float value() const { return v_; }
    float rate() const { return r_; }
    float var() const { return P_[0][0]; }
    double time() const { return t_; }
    /** value extrapolated to t (the state is not changed), at most max_dt ahead */
    float value_at(double t, float max_dt) const;
    void clamp_rate(float max_rate);

private:
    bool init_ = false;
    float v_ = 0.0f, r_ = 0.0f;
    float P_[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};
    double t_ = 0.0;
};

class RangeTracker {
public:
    RangeTracker();

    void configure(const Config& cfg);
    const Config& config() const { return cfg_; }

    void set_anchor(int idx, const Point3D& pos);
    void clear_anchor(int idx);
    bool anchor_known(int idx) const;

    void set_mode(Mode mode);
    Mode mode() const { return mode_; }
    bool anchor_used(int idx) const;

    void reset();

    /** One distance to beacon idx (slant, as measured), at time t_s (seconds, monotonic). */
    RangeResult add_range(int idx, float dist_m, double t_s);

    /**
     * Height of the antenna above the floor from a laser rangefinder (DRONE mode).
     * @return true if it was used.
     */
    bool add_height(float z_m, float sigma_m, double t_s);

    /** Position at time t_s from the distances predicted for t_s (also stored as last_fix). */
    Fix fix(double t_s);

    /** Last computed position (from the last fix() call). */
    const Fix& last_fix() const { return last_; }

    /** Filtered distance of beacon idx; false if none / older than max_age_s at t_s. */
    bool distance(int idx, double t_s, float& d) const;

    /** Estimated horizontal speed, m/s (from the positions of the last round). */
    float speed() const { return speed_; }

private:
    float plane_z() const;

    Config cfg_;
    Mode mode_;
    Point3D anchors_[MAX_ANCHORS];
    bool known_[MAX_ANCHORS];
    float dist_[MAX_ANCHORS];      // filtered distance, < 0 = none
    double dist_t_[MAX_ANCHORS];   // time of the last accepted distance
    bool rejected_[MAX_ANCHORS];   // the previous distance of this beacon was rejected
    float speed_;
    static constexpr int HIST = 4;
    Point3D hist_p_[HIST];
    double hist_t_[HIST];
    int hist_n_;
    void push_history(const Point3D& p, double t);
    Filter1D z_;
    bool new_ble_ = false;
    int z_bad_ = 0;
    double last_laser_t_ = -1e9;
    Fix last_;
};

} // namespace Tracker

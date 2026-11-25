#pragma once

/*
 * Height of the drone from BLE + a downward laser rangefinder.
 *
 *   laser_h         = range * cos(roll) * cos(pitch)     vertical distance to what is below
 *   surface_height  = Z_BLE - laser_h                    height of that surface above the floor
 *
 *  - surface_height ~ 0  -> the floor is below. laser_h is then the height of the drone above
 *                           the floor, much less noisy than Z from BLE: use_for_z = true, the
 *                           caller feeds it into the position filter as a z measurement.
 *  - laser_h suddenly drops (e.g. by 0.7 m) while Z_BLE stays -> an obstacle is below
 *                           (a box, a table). The drone height is NOT changed, and the
 *                           obstacle (its height = surface_height) is reported, to be put on a
 *                           map at the current x, y.
 *  - in between / no floor seen yet / large tilt -> nothing is trusted (state Unknown).
 *
 * Z_BLE is the z of the position filter (above the floor). Over the floor it is corrected by
 * the laser, so it is stable; over an obstacle the laser is not used and z is carried by BLE
 * and the filter alone.
 *
 * Portable C++ (no Zephyr), tested on a PC, see test_tracker.cpp.
 */

#include <stdint.h>

namespace HeightFusion {

enum class Surface : uint8_t {
    Unknown = 0,  // not sure what is below (start, tilt, gradual change)
    Floor = 1,    // the floor is below: laser_h = height above the floor
    Obstacle = 2, // something stands on the floor below the drone
};

struct Config {
    float floor_tol_m = 0.25f;     // |surface_height| below this = floor
    float obstacle_min_m = 0.25f;  // surface_height above this can be an obstacle
    float jump_min_m = 0.20f;      // laser_h must drop at least this much between two samples
    float ble_still_m = 0.15f;     // ... while Z_BLE moves less than this
    float max_tilt_deg = 30.0f;    // above this the beam is too oblique: sample ignored
    float min_range_m = 0.03f;
    float max_range_m = 8.0f;
    float max_gap_s = 1.0f;        // a jump is only detected between samples closer than this
    float laser_sigma_m = 0.03f;   // noise of laser_h, for the position filter
};

struct Result {
    bool valid;            // the sample was usable (range and tilt in limits)
    float laser_h_m;       // range * cos(roll) * cos(pitch)
    float surface_m;       // Z_BLE - laser_h
    Surface surface;       // what is below after this sample
    bool use_for_z;        // feed laser_h into the position filter as z (floor below)
    bool new_obstacle;     // just flew over the edge of an obstacle
    float obstacle_h_m;    // height of the obstacle (when surface == Obstacle)
};

class Estimator {
public:
    Estimator();
    void configure(const Config& cfg) { cfg_ = cfg; reset(); }
    const Config& config() const { return cfg_; }
    void reset();

    /**
     * @param z_ble_m   z of the drone above the floor from the position filter
     * @param range_m   laser distance along the beam
     * @param roll_rad, pitch_rad   attitude of the drone (beam fixed to the frame, pointing down)
     * @param t_s       time, seconds (monotonic)
     */
    Result update(float z_ble_m, float range_m, float roll_rad, float pitch_rad, double t_s);

    Surface surface() const { return state_; }

private:
    Config cfg_;
    Surface state_;
    bool have_prev_;
    float prev_h_;
    float prev_z_;
    double prev_t_;
    float obstacle_h_;
};

} // namespace HeightFusion

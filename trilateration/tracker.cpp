#include "tracker.hpp"

#include <math.h>

namespace Tracker {

// ---------------------------------------------------------------------------------------
// Filter1D
// ---------------------------------------------------------------------------------------

void Filter1D::init(float v, double t, float var, float rate_var) {
    v_ = v;
    r_ = 0.0f;
    P_[0][0] = var;
    P_[0][1] = P_[1][0] = 0.0f;
    P_[1][1] = rate_var;
    t_ = t;
    init_ = true;
}

void Filter1D::predict_to(double t, float q) {
    float dt = (float)(t - t_);
    if (!init_ || !(dt > 0.0f)) {
        return;
    }
    if (dt > 30.0f) {
        dt = 30.0f;
    }
    v_ += r_ * dt;
    // P = F P F^T + Q, F = [1 dt; 0 1], Q of a white-noise acceleration
    const float p00 = P_[0][0] + dt * (P_[1][0] + P_[0][1]) + dt * dt * P_[1][1];
    const float p01 = P_[0][1] + dt * P_[1][1];
    const float p11 = P_[1][1];
    P_[0][0] = p00 + q * dt * dt * dt / 3.0f;
    P_[0][1] = P_[1][0] = p01 + q * dt * dt / 2.0f;
    P_[1][1] = p11 + q * dt;
    t_ = t;
}

bool Filter1D::update(float z, float r_var, float gate) {
    if (!init_ || !isfinite(z)) {
        return false;
    }
    const float y = z - v_;
    if (fabsf(y) > gate) {
        return false;
    }
    const float S = P_[0][0] + r_var;
    const float k0 = P_[0][0] / S;
    const float k1 = P_[1][0] / S;
    v_ += k0 * y;
    r_ += k1 * y;
    const float p00 = P_[0][0] - k0 * k0 * S;
    const float p01 = P_[0][1] - k0 * k1 * S;
    const float p11 = P_[1][1] - k1 * k1 * S;
    P_[0][0] = p00 > 1e-6f ? p00 : 1e-6f;
    P_[0][1] = P_[1][0] = p01;
    P_[1][1] = p11 > 1e-6f ? p11 : 1e-6f;
    return true;
}

float Filter1D::value_at(double t, float max_dt) const {
    float dt = (float)(t - t_);
    if (dt < 0.0f) {
        dt = 0.0f;
    }
    if (dt > max_dt) {
        dt = max_dt;
    }
    return v_ + r_ * dt;
}

void Filter1D::clamp_rate(float max_rate) {
    if (r_ > max_rate) {
        r_ = max_rate;
    } else if (r_ < -max_rate) {
        r_ = -max_rate;
    }
}

// ---------------------------------------------------------------------------------------
// RangeTracker
// ---------------------------------------------------------------------------------------

RangeTracker::RangeTracker() : mode_(Mode::Car) {
    for (int i = 0; i < MAX_ANCHORS; ++i) {
        anchors_[i] = {0.0f, 0.0f, 0.0f};
        known_[i] = false;
    }
    reset();
}

void RangeTracker::configure(const Config& cfg) {
    cfg_ = cfg;
    reset();
}

void RangeTracker::set_anchor(int idx, const Point3D& pos) {
    if (idx < 0 || idx >= MAX_ANCHORS) {
        return;
    }
    anchors_[idx] = pos;
    known_[idx] = true;
}

void RangeTracker::clear_anchor(int idx) {
    if (idx >= 0 && idx < MAX_ANCHORS) {
        known_[idx] = false;
    }
}

bool RangeTracker::anchor_known(int idx) const {
    return idx >= 0 && idx < MAX_ANCHORS && known_[idx];
}

void RangeTracker::set_mode(Mode mode) {
    if (mode == mode_) {
        return;
    }
    mode_ = mode;
    z_.reset();       // the distances are kept: they do not depend on the mode
    hist_n_ = 0;
    last_laser_t_ = -1e9;
    last_ = {};
}

bool RangeTracker::anchor_used(int idx) const {
    if (!anchor_known(idx)) {
        return false;
    }
    return mode_ == Mode::Drone || idx < cfg_.car_beacons;
}

void RangeTracker::reset() {
    for (int i = 0; i < MAX_ANCHORS; ++i) {
        dist_[i] = -1.0f;
        dist_t_[i] = -1e9;
        rejected_[i] = false;
    }
    speed_ = 0.0f;
    hist_n_ = 0;
    z_.reset();
    z_bad_ = 0;
    new_ble_ = false;
    last_laser_t_ = -1e9;
    last_ = {};
}

float RangeTracker::plane_z() const {
    float sum = 0.0f;
    int n = 0;
    for (int i = 0; i < MAX_ANCHORS && i < cfg_.car_beacons; ++i) {
        if (known_[i]) {
            sum += anchors_[i].z;
            n++;
        }
    }
    return n ? sum / n : 0.0f;
}

RangeResult RangeTracker::add_range(int idx, float dist_m, double t_s) {
    if (idx < 0 || idx >= MAX_ANCHORS || !known_[idx] || !(dist_m > 0.05f) ||
        dist_m > cfg_.max_range_m) {
        return RangeResult::Ignored;
    }

    // The distance of every known beacon is kept in both modes, so switching the mode does
    // not lose anything; only the beacons used by the mode count as "used".
    RangeResult res;

    if (dist_[idx] < 0.0f || t_s - dist_t_[idx] > (double)cfg_.max_age_s) {
        dist_[idx] = dist_m;
        rejected_[idx] = false;
        res = RangeResult::Accepted;
    } else {
        const float dt = (float)(t_s - dist_t_[idx]);
        const float motion = speed_ * (dt > 0.0f ? dt : 0.0f);
        const float gate = cfg_.gate_min_m + cfg_.gate_speed_factor * motion;
        const float y = dist_m - dist_[idx];

        if (fabsf(y) <= gate) {
            const float alpha = motion < cfg_.still_motion_m ? cfg_.alpha_still : 1.0f;
            dist_[idx] += alpha * y;
            rejected_[idx] = false;
            res = RangeResult::Accepted;
        } else if (rejected_[idx]) {
            dist_[idx] = dist_m; // second "spike" in a row: it really changed
            rejected_[idx] = false;
            res = RangeResult::Restarted;
        } else {
            rejected_[idx] = true; // a single spike (reflection): not used
            return anchor_used(idx) ? RangeResult::Rejected : RangeResult::Ignored;
        }
    }
    dist_t_[idx] = t_s;

    if (!anchor_used(idx)) {
        return RangeResult::Ignored;
    }
    new_ble_ = true;
    return res;
}

bool RangeTracker::distance(int idx, double t_s, float& d) const {
    if (idx < 0 || idx >= MAX_ANCHORS || dist_[idx] < 0.0f || t_s - dist_t_[idx] > (double)cfg_.max_age_s) {
        return false;
    }
    d = dist_[idx];
    return true;
}

void RangeTracker::push_history(const Point3D& p, double t) {
    for (int i = HIST - 1; i > 0; --i) {
        hist_p_[i] = hist_p_[i - 1];
        hist_t_[i] = hist_t_[i - 1];
    }
    hist_p_[0] = p;
    hist_t_[0] = t;
    if (hist_n_ < HIST) {
        hist_n_++;
    }
    // speed over the last round (oldest stored position), smoothed
    const int k = hist_n_ - 1;
    const double dt = hist_t_[0] - hist_t_[k];
    if (k > 0 && dt > 0.5) {
        const float dx = hist_p_[0].x - hist_p_[k].x, dy = hist_p_[0].y - hist_p_[k].y;
        const float v = sqrtf(dx * dx + dy * dy) / (float)dt;
        speed_ += 0.5f * (v - speed_);
    }
}

bool RangeTracker::add_height(float z_m, float sigma_m, double t_s) {
    if (mode_ != Mode::Drone || !isfinite(z_m)) {
        return false;
    }
    const float r = sigma_m * sigma_m;
    if (!z_.initialized()) {
        z_.init(z_m, t_s, r, 0.25f);
    } else {
        z_.predict_to(t_s, cfg_.z_accel);
        if (!z_.update(z_m, r, 1.0f)) {
            return false; // more than 1 m from the filter: a glitch
        }
        z_.clamp_rate(1.0f);
    }
    last_laser_t_ = t_s;
    return true;
}

Fix RangeTracker::fix(double t_s) {
    Fix fx = {};
    Anchor a[MAX_ANCHORS];
    int n = 0;
    float zmin = 1e9f, zmax = -1e9f;

    for (int i = 0; i < MAX_ANCHORS; ++i) {
        float d;
        if (!anchor_used(i) || !distance(i, t_s, d)) {
            continue;
        }
        a[n].pos = anchors_[i];
        a[n].distance = d;
        const float age = (float)(t_s - dist_t_[i]);
        if (age > fx.max_age_s) {
            fx.max_age_s = age;
        }
        zmin = fminf(zmin, anchors_[i].z);
        zmax = fmaxf(zmax, anchors_[i].z);
        n++;
    }
    fx.used = n;
    if (n < 3) {
        last_ = fx;
        return fx;
    }

    Anchor h[MAX_ANCHORS]; // horizontal distances at a known height
    auto horizontals = [&](float z) {
        for (int i = 0; i < n; ++i) {
            h[i] = a[i];
            const float dz = a[i].pos.z - z;
            const float h2 = a[i].distance * a[i].distance - dz * dz;
            h[i].distance = h2 > 0.0025f ? sqrtf(h2) : 0.05f;
        }
    };

    Point3D p = {0.0f, 0.0f, 0.0f};

    if (mode_ == Mode::Car) {
        horizontals(cfg_.car_z_m);
        if (!Trilateration::calculate_2d_position(h, n, p)) {
            last_ = fx;
            return fx;
        }
        p.z = cfg_.car_z_m;
        fx.residual_m = Trilateration::rms_residual(h, n, p, false);
    } else {
        const bool laser = z_.initialized() && t_s - last_laser_t_ < 1.0;
        const float z_hint = z_.initialized() ? z_.value_at(t_s, 1.0f)
                                              : plane_z() + cfg_.drone_default_height_m;
        bool ok = false;

        if (laser) {
            // the height is known precisely: solve x, y on that height
            horizontals(z_hint);
            ok = Trilateration::calculate_2d_position(h, n, p);
            p.z = z_hint;
        } else {
            if (n >= 4 && zmax - zmin > 0.3f) {
                ok = Trilateration::calculate_3d_position(a, n, p);
            }
            if (!ok) {
                // beacons in (nearly) one plane: start at the expected height and stay on that
                // side of the plane
                horizontals(z_hint);
                ok = Trilateration::calculate_2d_position(h, n, p);
                p.z = z_hint;
                Point3D r = p;
                if (ok && Trilateration::refine_3d(a, n, r, 10) && r.z > plane_z()) {
                    p = r;
                }
            }
        }
        if (!ok) {
            last_ = fx;
            return fx;
        }

        // z through its filter (BLE z is noisy; the laser, if any, already corrected it)
        if (new_ble_ && !laser) {
            const float r = cfg_.z_ble_sigma_m * cfg_.z_ble_sigma_m;
            if (!z_.initialized()) {
                z_.init(p.z, t_s, r, 0.25f);
            } else {
                z_.predict_to(t_s, cfg_.z_accel);
                float gate = 3.0f * sqrtf(z_.var() + r);
                if (gate < 1.0f) {
                    gate = 1.0f;
                }
                if (z_.update(p.z, r, gate)) {
                    z_bad_ = 0;
                } else if (++z_bad_ >= 3) {
                    z_.init(p.z, t_s, r, 0.25f); // the z filter is lost: restart from BLE
                    z_bad_ = 0;
                }
                z_.clamp_rate(1.0f);
            }
        }
        if (z_.initialized()) {
            p.z = z_.value_at(t_s, 1.0f);
        }
        fx.residual_m = Trilateration::rms_residual(a, n, p, true);
    }
    fx.valid = isfinite(p.x) && isfinite(p.y) && isfinite(p.z);
    fx.pos = p;
    if (fx.valid && new_ble_) {
        push_history(p, t_s);
    }
    new_ble_ = false;
    last_ = fx;
    return fx;
}

} // namespace Tracker

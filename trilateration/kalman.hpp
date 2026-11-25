#ifndef KALMAN_HPP
#define KALMAN_HPP

#include <math.h>
#include <stddef.h>

/*
 * Constant-velocity Kalman filter, state [x y z vx vy vz].
 *
 *  - predict(dt): white-noise acceleration model, the process noise grows with dt
 *    (q_accel = acceleration noise density, (m/s^2)^2 / Hz), so a long gap between two
 *    measurements correctly makes the filter less sure about the position.
 *  - update_range(): one distance to one beacon (non-linear, EKF). Every distance is used
 *    at the moment it was measured, so beacons measured one after another (round-robin)
 *    are not mixed as if they were measured at the same time.
 *  - update_axis(): direct measurement of one coordinate (e.g. z from a laser rangefinder).
 *  - Every update has a gate: a measurement whose normalized innovation squared (NIS) is
 *    above gate_chi2 is rejected and does not touch the state (reflections, spikes).
 *  - lock_z: 2D mode (car). z and vz are kept at their initial values and never updated.
 */
class KalmanFilter3D {
public:
    struct UpdateResult {
        bool accepted;
        float innovation; // measured - predicted
        float nis;        // innovation^2 / S
    };

    explicit KalmanFilter3D(float q_accel = 0.5f, float r_pos = 0.05f, float r_range = 0.0225f)
        : q_accel_(q_accel), r_pos_(r_pos), r_range_(r_range), vel_tau_(0.0f),
          max_speed_(0.0f), lock_z_(false), is_initialized_(false) {
        reset();
    }

    void reset() {
        for (int i = 0; i < 6; ++i) {
            x_[i] = 0.0f;
            for (int j = 0; j < 6; ++j) {
                P_[i][j] = (i == j) ? 1.0f : 0.0f;
            }
        }
        is_initialized_ = false;
    }

    void set_process_noise(float q_accel) { q_accel_ = q_accel; }
    /** Velocity decays to 0 with this time constant (s), 0 = constant velocity model.
     *  Without it one range every 1.5 s lets the velocity run away on a turning car. */
    void set_velocity_tau(float tau_s) { vel_tau_ = tau_s; }
    /** Speed limit (m/s, 0 = none), applied after every update. */
    void set_max_speed(float v) { max_speed_ = v; }
    void set_lock_z(bool lock) { lock_z_ = lock; }
    bool lock_z() const { return lock_z_; }

    void init(float x, float y, float z = 0.0f, float pos_var = 0.1f, float vel_var = 0.25f) {
        x_[0] = x;
        x_[1] = y;
        x_[2] = z;
        x_[3] = x_[4] = x_[5] = 0.0f;
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                P_[i][j] = 0.0f;
            }
        }
        for (int i = 0; i < 3; ++i) {
            P_[i][i] = pos_var;
            P_[i + 3][i + 3] = vel_var;
        }
        if (lock_z_) {
            P_[2][2] = 0.0f;
            P_[5][5] = 0.0f;
        }
        is_initialized_ = true;
    }

    void predict(float dt) {
        if (!is_initialized_ || !(dt > 0.0f)) {
            return;
        }
        if (dt > 30.0f) {
            dt = 30.0f; // keep the numbers sane after a very long gap
        }

        const int axes = lock_z_ ? 2 : 3;

        // F = [I  g*I; 0  e*I]: g = dt, e = 1 (constant velocity), or with velocity decay
        // (time constant tau) g = tau * (1 - exp(-dt/tau)), e = exp(-dt/tau)
        float e = 1.0f, g = dt;
        if (vel_tau_ > 0.0f) {
            e = expf(-dt / vel_tau_);
            g = vel_tau_ * (1.0f - e);
        }

        for (int a = 0; a < axes; ++a) {
            x_[a] += x_[a + 3] * g;
            x_[a + 3] *= e;
        }

        // P = F P F^T
        for (int i = 0; i < axes; ++i) {          // rows: F * P
            for (int j = 0; j < 6; ++j) {
                P_[i][j] += g * P_[i + 3][j];
                P_[i + 3][j] *= e;
            }
        }
        for (int i = 0; i < 6; ++i) {             // columns: (F P) * F^T
            for (int j = 0; j < axes; ++j) {
                P_[i][j] += g * P_[i][j + 3];
                P_[i][j + 3] *= e;
            }
        }

        // Q of a white-noise acceleration (continuous): q*[dt^3/3 dt^2/2; dt^2/2 dt]
        const float q = q_accel_;
        const float q_pp = q * dt * dt * dt / 3.0f;
        const float q_pv = q * dt * dt / 2.0f;
        const float q_vv = q * dt;
        for (int a = 0; a < axes; ++a) {
            P_[a][a] += q_pp;
            P_[a][a + 3] += q_pv;
            P_[a + 3][a] += q_pv;
            P_[a + 3][a + 3] += q_vv;
        }
        symmetrize();
    }

    /**
     * One distance to a beacon at (ax, ay, az). In lock_z mode (or with horizontal = true)
     * the distance must be the horizontal one (the caller removes the height difference).
     */
    UpdateResult update_range(float ax, float ay, float az, float measured_dist,
                              float r_var, float gate_chi2, bool horizontal = false) {
        UpdateResult res = {false, 0.0f, 0.0f};
        if (!is_initialized_) {
            return res;
        }

        const bool flat = horizontal || lock_z_;

        // Iterated EKF: re-linearize around the updated estimate. With one distance at a time
        // and a large uncertainty a single linearization can overshoot by meters.
        float x0[6], P0[6][6];
        for (int i = 0; i < 6; ++i) {
            x0[i] = x_[i];
            for (int j = 0; j < 6; ++j) {
                P0[i][j] = P_[i][j];
            }
        }
        float xi[6];
        for (int i = 0; i < 6; ++i) {
            xi[i] = x0[i];
        }

        for (int it = 0; it < 3; ++it) {
            float d[3] = {xi[0] - ax, xi[1] - ay, flat ? 0.0f : xi[2] - az};
            float pred = sqrtf(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
            if (pred < 1e-3f) {
                pred = 1e-3f;
            }
            float H[6] = {d[0] / pred, d[1] / pred, d[2] / pred, 0.0f, 0.0f, 0.0f};
            // innovation of the linearization around xi, referred to the prior x0
            float hx = 0.0f;
            for (int k = 0; k < 6; ++k) {
                hx += H[k] * (x0[k] - xi[k]);
            }
            float innov = measured_dist - pred - hx;

            for (int i = 0; i < 6; ++i) {
                x_[i] = x0[i];
                for (int j = 0; j < 6; ++j) {
                    P_[i][j] = P0[i][j];
                }
            }
            res.innovation = measured_dist - pred;
            res.accepted = false;
            apply_scalar_update(H, innov, r_var, it == 0 ? gate_chi2 : INFINITY, res);
            if (!res.accepted) {
                // rejected by the gate (first pass) or numerically bad: keep the prior
                for (int i = 0; i < 6; ++i) {
                    x_[i] = x0[i];
                    for (int j = 0; j < 6; ++j) {
                        P_[i][j] = P0[i][j];
                    }
                }
                return res;
            }
            float step = 0.0f;
            for (int i = 0; i < 3; ++i) {
                step += (x_[i] - xi[i]) * (x_[i] - xi[i]);
                xi[i] = x_[i];
            }
            for (int i = 3; i < 6; ++i) {
                xi[i] = x_[i];
            }
            if (step < 1e-6f) {
                break;
            }
        }
        clamp_speed();
        return res;
    }

    /** Direct measurement of x (axis 0), y (1) or z (2). */
    UpdateResult update_axis(int axis, float value, float r_var, float gate_chi2) {
        UpdateResult res = {false, 0.0f, 0.0f};
        if (!is_initialized_ || axis < 0 || axis > 2 || (axis == 2 && lock_z_)) {
            return res;
        }
        float H[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        H[axis] = 1.0f;
        res.innovation = value - x_[axis];
        apply_scalar_update(H, res.innovation, r_var, gate_chi2, res);
        clamp_speed();
        return res;
    }

    // ---- Older interface (kept for the code that already uses it, no gate) ----

    /** 2D range update in the XY plane (z ignored). */
    void update_range_ekf_2d(float anchor_x, float anchor_y, float measured_dist) {
        update_range(anchor_x, anchor_y, 0.0f, measured_dist, r_range_, INFINITY, true);
    }

    void update_range_ekf(float anchor_x, float anchor_y, float anchor_z, float measured_dist) {
        update_range(anchor_x, anchor_y, anchor_z, measured_dist, r_range_, INFINITY);
    }

    /** Position measurement (x, y, z), e.g. from trilateration. */
    void update(float x_m, float y_m, float z_m) {
        if (!is_initialized_) {
            init(x_m, y_m, z_m);
            return;
        }
        update_axis(0, x_m, r_pos_, INFINITY);
        update_axis(1, y_m, r_pos_, INFINITY);
        update_axis(2, z_m, r_pos_, INFINITY);
    }

    float getX() const { return x_[0]; }
    float getY() const { return x_[1]; }
    float getZ() const { return x_[2]; }
    float getVx() const { return x_[3]; }
    float getVy() const { return x_[4]; }
    float getVz() const { return x_[5]; }
    float var(int i) const { return P_[i][i]; }
    float cov(int i, int j) const { return P_[i][j]; }

    bool isInitialized() const { return is_initialized_; }

private:
    UpdateResult &apply_scalar_update(const float H[6], float innovation, float r_var,
                                      float gate_chi2, UpdateResult &res) {
        float PHt[6];
        for (int i = 0; i < 6; ++i) {
            PHt[i] = 0.0f;
            for (int k = 0; k < 6; ++k) {
                PHt[i] += P_[i][k] * H[k];
            }
        }
        float S = r_var;
        for (int k = 0; k < 6; ++k) {
            S += H[k] * PHt[k];
        }
        if (!(S > 0.0f) || !isfinite(innovation)) {
            return res;
        }

        res.nis = innovation * innovation / S;
        if (res.nis > gate_chi2) {
            return res; // outlier: rejected, the state is not changed
        }

        float K[6];
        for (int i = 0; i < 6; ++i) {
            K[i] = PHt[i] / S;
        }
        if (lock_z_) {
            K[2] = 0.0f;
            K[5] = 0.0f;
        }
        for (int i = 0; i < 6; ++i) {
            x_[i] += K[i] * innovation;
        }
        // P = P - K S K^T  (= P - K H P for the optimal gain), then symmetrize
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                P_[i][j] -= K[i] * K[j] * S;
            }
        }
        if (lock_z_) {
            for (int i = 0; i < 6; ++i) {
                P_[2][i] = P_[i][2] = 0.0f;
                P_[5][i] = P_[i][5] = 0.0f;
            }
        }
        for (int i = 0; i < 6; ++i) {
            if (P_[i][i] < 1e-6f && !(lock_z_ && (i == 2 || i == 5))) {
                P_[i][i] = 1e-6f; // numerical floor, keeps P positive
            }
        }
        symmetrize();
        res.accepted = true;
        return res;
    }

    void clamp_speed() {
        if (max_speed_ <= 0.0f) {
            return;
        }
        float v = sqrtf(x_[3] * x_[3] + x_[4] * x_[4] + x_[5] * x_[5]);
        if (v > max_speed_) {
            float k = max_speed_ / v;
            x_[3] *= k;
            x_[4] *= k;
            x_[5] *= k;
        }
    }

    void symmetrize() {
        for (int i = 0; i < 6; ++i) {
            for (int j = i + 1; j < 6; ++j) {
                float m = 0.5f * (P_[i][j] + P_[j][i]);
                P_[i][j] = P_[j][i] = m;
            }
        }
    }

    float x_[6];
    float P_[6][6];
    float q_accel_;
    float r_pos_;
    float r_range_;
    float vel_tau_;
    float max_speed_;
    bool lock_z_;
    bool is_initialized_;
};

#endif // KALMAN_HPP

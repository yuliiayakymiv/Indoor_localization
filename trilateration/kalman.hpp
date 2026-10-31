#ifndef KALMAN_HPP
#define KALMAN_HPP

#include <math.h>
#include <stddef.h>

/**
 * @brief 6-State Extended Kalman Filter (EKF) for 3D Indoor Localization
 *        Designed for nRF54L15 (Cortex-M33 with hardware FP32 FPU).
 *        Zero external dependencies (no Eigen, no dynamic memory allocation).
 *
 * State vector x = [x, y, z, vx, vy, vz]^T (6x1)
 */
class KalmanFilter3D {
public:
    KalmanFilter3D(float q_pos = 0.01f, float q_vel = 0.05f,
                   float r_pos = 0.05f, float r_range = 0.0225f)
        : q_pos_(q_pos), q_vel_(q_vel), r_pos_(r_pos), r_range_(r_range),
          is_initialized_(false) {
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

    void init(float x, float y, float z) {
        x_[0] = x;
        x_[1] = y;
        x_[2] = z;
        x_[3] = 0.0f;
        x_[4] = 0.0f;
        x_[5] = 0.0f;

        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                P_[i][j] = (i == j) ? 0.1f : 0.0f;
            }
        }
        is_initialized_ = true;
    }

    /**
     * @brief EKF Prediction step using Constant Velocity model (x = F*x, P = F*P*F^T + Q)
     *        Exploits sparsity of F without temporary 6x6 matrices.
     */
    void predict(float dt) {
        if (!is_initialized_) return;

        // 1. State prediction: x_pos += v * dt
        x_[0] += x_[3] * dt;
        x_[1] += x_[4] * dt;
        x_[2] += x_[5] * dt;

        // 2. Covariance prediction: P = F * P * F^T + Q
        // Step 2a: P = F * P (rows 0..2 += dt * rows 3..5)
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 6; ++j) {
                P_[i][j] += dt * P_[i + 3][j];
            }
        }
        // Step 2b: P = P * F^T (cols 0..2 += dt * cols 3..5)
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 3; ++j) {
                P_[i][j] += P_[i][j + 3] * dt;
            }
        }
        // Step 2c: Add process noise Q
        for (int i = 0; i < 3; ++i) {
            P_[i][i] += q_pos_;
            P_[i + 3][i + 3] += q_vel_;
        }
    }

    /**
     * @brief Non-linear EKF Update using measured distance to a single anchor.
     *        Observation model: h(x) = sqrt((x - ax)^2 + (y - ay)^2 + (z - az)^2)
     *        Jacobian row H = [(x - ax)/d, (y - ay)/d, (z - az)/d, 0, 0, 0]
     */
    void update_range_ekf(float anchor_x, float anchor_y, float anchor_z, float measured_dist) {
        if (!is_initialized_) return;

        float dx = x_[0] - anchor_x;
        float dy = x_[1] - anchor_y;
        float dz = x_[2] - anchor_z;

        // Predicted range h(x)
        float pred_dist = sqrtf(dx * dx + dy * dy + dz * dz);
        if (pred_dist < 1e-4f) {
            pred_dist = 1e-4f; // Prevent division by zero
        }

        // Jacobian H (only first 3 elements are non-zero)
        float inv_dist = 1.0f / pred_dist;
        float H[3] = {
            dx * inv_dist,
            dy * inv_dist,
            dz * inv_dist
        };

        // Compute PHt = P * H^T (6x1)
        float PHt[6];
        for (int i = 0; i < 6; ++i) {
            PHt[i] = P_[i][0] * H[0] + P_[i][1] * H[1] + P_[i][2] * H[2];
        }

        // Innovation covariance S = H * P * H^T + R_range (scalar)
        float S = H[0] * PHt[0] + H[1] * PHt[1] + H[2] * PHt[2] + r_range_;
        float inv_S = 1.0f / S;

        // Kalman gain K = PHt / S (6x1)
        float K[6];
        for (int i = 0; i < 6; ++i) {
            K[i] = PHt[i] * inv_S;
        }

        // Innovation (measurement residual)
        float y = measured_dist - pred_dist;

        // Update state vector x = x + K * y
        for (int i = 0; i < 6; ++i) {
            x_[i] += K[i] * y;
        }

        // Update covariance matrix P = (I - K * H) * P = P - K * (H * P)
        float HP[6];
        for (int j = 0; j < 6; ++j) {
            HP[j] = H[0] * P_[0][j] + H[1] * P_[1][j] + H[2] * P_[2][j];
        }
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                P_[i][j] -= K[i] * HP[j];
            }
        }
    }

    /**
     * @brief Linear position update (Loosely-coupled mode after WLS trilateration)
     */
    void update(float x_m, float y_m, float z_m) {
        if (!is_initialized_) {
            init(x_m, y_m, z_m);
            return;
        }
        const float z[3] = {x_m, y_m, z_m};
        for (int axis = 0; axis < 3; ++axis) {
            float S = P_[axis][axis] + r_pos_;
            float inv_S = 1.0f / S;
            float K[6];
            for (int i = 0; i < 6; ++i) {
                K[i] = P_[i][axis] * inv_S;
            }
            float y = z[axis] - x_[axis];
            for (int i = 0; i < 6; ++i) {
                x_[i] += K[i] * y;
            }
            float P_row[6];
            for (int j = 0; j < 6; ++j) {
                P_row[j] = P_[axis][j];
            }
            for (int i = 0; i < 6; ++i) {
                for (int j = 0; j < 6; ++j) {
                    P_[i][j] -= K[i] * P_row[j];
                }
            }
        }
    }

    float getX() const { return x_[0]; }
    float getY() const { return x_[1]; }
    float getZ() const { return x_[2]; }
    float getVx() const { return x_[3]; }
    float getVy() const { return x_[4]; }
    float getVz() const { return x_[5]; }

    bool isInitialized() const { return is_initialized_; }

private:
    float x_[6];
    float P_[6][6];
    float q_pos_;
    float q_vel_;
    float r_pos_;
    float r_range_;
    bool is_initialized_;
};

#endif // KALMAN_HPP

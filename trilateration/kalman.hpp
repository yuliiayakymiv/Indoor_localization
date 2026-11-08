#ifndef KALMAN_HPP
#define KALMAN_HPP

#include <math.h>
#include <stddef.h>

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

    void init(float x, float y, float z = 0.0f) {
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

    void predict(float dt) {
        if (!is_initialized_) return;

        x_[0] += x_[3] * dt;
        x_[1] += x_[4] * dt;
        x_[2] += x_[5] * dt;

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 6; ++j) {
                P_[i][j] += dt * P_[i + 3][j];
            }
        }
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 3; ++j) {
                P_[i][j] += P_[i][j + 3] * dt;
            }
        }
        for (int i = 0; i < 3; ++i) {
            P_[i][i] += q_pos_;
            P_[i + 3][i + 3] += q_vel_;
        }
    }

    /**
     * @brief 2D Non-linear EKF Update in XY plane (locks Z = 0, Vz = 0)
     */
    void update_range_ekf_2d(float anchor_x, float anchor_y, float measured_dist) {
        if (!is_initialized_) return;

        float dx = x_[0] - anchor_x;
        float dy = x_[1] - anchor_y;

        float pred_dist = sqrtf(dx * dx + dy * dy);
        if (pred_dist < 1e-4f) {
            pred_dist = 1e-4f;
        }

        float inv_dist = 1.0f / pred_dist;
        float H0 = dx * inv_dist;
        float H1 = dy * inv_dist;

        float PHt[6];
        for (int i = 0; i < 6; ++i) {
            PHt[i] = P_[i][0] * H0 + P_[i][1] * H1;
        }

        float S = H0 * PHt[0] + H1 * PHt[1] + r_range_;
        float inv_S = 1.0f / S;

        float K[6];
        for (int i = 0; i < 6; ++i) {
            K[i] = PHt[i] * inv_S;
        }

        float y = measured_dist - pred_dist;
        for (int i = 0; i < 6; ++i) {
            x_[i] += K[i] * y;
        }
        x_[2] = 0.0f;
        x_[5] = 0.0f;

        float HP[6];
        for (int j = 0; j < 6; ++j) {
            HP[j] = H0 * P_[0][j] + H1 * P_[1][j];
        }
        for (int i = 0; i < 6; ++i) {
            for (int j = 0; j < 6; ++j) {
                P_[i][j] -= K[i] * HP[j];
            }
        }
    }

    void update_range_ekf(float anchor_x, float anchor_y, float anchor_z, float measured_dist) {
        if (!is_initialized_) return;

        float dx = x_[0] - anchor_x;
        float dy = x_[1] - anchor_y;
        float dz = x_[2] - anchor_z;

        float pred_dist = sqrtf(dx * dx + dy * dy + dz * dz);
        if (pred_dist < 1e-4f) {
            pred_dist = 1e-4f;
        }

        float inv_dist = 1.0f / pred_dist;
        float H[3] = {
            dx * inv_dist,
            dy * inv_dist,
            dz * inv_dist
        };

        float PHt[6];
        for (int i = 0; i < 6; ++i) {
            PHt[i] = P_[i][0] * H[0] + P_[i][1] * H[1] + P_[i][2] * H[2];
        }

        float S = H[0] * PHt[0] + H[1] * PHt[1] + H[2] * PHt[2] + r_range_;
        float inv_S = 1.0f / S;

        float K[6];
        for (int i = 0; i < 6; ++i) {
            K[i] = PHt[i] * inv_S;
        }

        float y = measured_dist - pred_dist;
        for (int i = 0; i < 6; ++i) {
            x_[i] += K[i] * y;
        }

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

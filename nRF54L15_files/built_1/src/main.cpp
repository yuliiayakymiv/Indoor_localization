#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

struct Point3D {
    float x;
    float y;
    float z;
};

struct Anchor {
    Point3D pos;     // Coordinates of anchor in space (meters)
    float distance;  // Distance to anchor (meters)
};

class Trilateration {
public:
    static bool calculate_3d_position(const Anchor* anchors, size_t num_anchors, Point3D& out_pos) {
        if (anchors == nullptr || num_anchors < 4) {
            return false;
        }

        size_t ref_idx = 0;
        float min_dist = anchors[0].distance;
        for (size_t i = 1; i < num_anchors; ++i) {
            if (anchors[i].distance < min_dist) {
                min_dist = anchors[i].distance;
                ref_idx = i;
            }
        }

        const Point3D& p1 = anchors[ref_idx].pos;
        float d1 = anchors[ref_idx].distance;

        float AtA[3][3] = {0};
        float AtB[3] = {0};

        for (size_t i = 0; i < num_anchors; ++i) {
            if (i == ref_idx) {
                continue;
            }

            const Point3D& pi = anchors[i].pos;
            float di = anchors[i].distance;

            float A_row[3] = {
                2.0f * (pi.x - p1.x),
                2.0f * (pi.y - p1.y),
                2.0f * (pi.z - p1.z)
            };

            float b_i = (pi.x * pi.x + pi.y * pi.y + pi.z * pi.z - di * di) -
                        (p1.x * p1.x + p1.y * p1.y + p1.z * p1.z - d1 * d1);

            float w = 1.0f / (di + d1 + 1e-3f);

            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) {
                    AtA[r][c] += A_row[r] * A_row[c] * w;
                }
                AtB[r] += A_row[r] * b_i * w;
            }
        }

        float det = AtA[0][0] * (AtA[1][1] * AtA[2][2] - AtA[1][2] * AtA[2][1])
                  - AtA[0][1] * (AtA[1][0] * AtA[2][2] - AtA[1][2] * AtA[2][0])
                  + AtA[0][2] * (AtA[1][0] * AtA[2][1] - AtA[1][1] * AtA[2][0]);

        if (fabsf(det) < 1e-5f) {
            return false;
        }

        float inv_det = 1.0f / det;

        float inv[3][3];
        inv[0][0] =  (AtA[1][1] * AtA[2][2] - AtA[1][2] * AtA[2][1]) * inv_det;
        inv[0][1] =  (AtA[0][2] * AtA[2][1] - AtA[0][1] * AtA[2][2]) * inv_det;
        inv[0][2] =  (AtA[0][1] * AtA[1][2] - AtA[0][2] * AtA[1][1]) * inv_det;

        inv[1][0] =  (AtA[1][2] * AtA[2][0] - AtA[1][0] * AtA[2][2]) * inv_det;
        inv[1][1] =  (AtA[0][0] * AtA[2][2] - AtA[0][2] * AtA[2][0]) * inv_det;
        inv[1][2] =  (AtA[0][2] * AtA[1][0] - AtA[0][0] * AtA[1][2]) * inv_det;

        inv[2][0] =  (AtA[1][0] * AtA[2][1] - AtA[1][1] * AtA[2][0]) * inv_det;
        inv[2][1] =  (AtA[0][1] * AtA[2][0] - AtA[0][0] * AtA[2][1]) * inv_det;
        inv[2][2] =  (AtA[0][0] * AtA[1][1] - AtA[0][1] * AtA[1][0]) * inv_det;

        out_pos.x = inv[0][0] * AtB[0] + inv[0][1] * AtB[1] + inv[0][2] * AtB[2];
        out_pos.y = inv[1][0] * AtB[0] + inv[1][1] * AtB[1] + inv[1][2] * AtB[2];
        out_pos.z = inv[2][0] * AtB[0] + inv[2][1] * AtB[1] + inv[2][2] * AtB[2];

        return true;
    }
};

/**
 * @brief 6-State Extended Kalman Filter (EKF) without Eigen for nRF54L15
 *        State vector: x = [x, y, z, vx, vy, vz]^T
 */
class ExtendedKalmanFilter3D {
private:
    float x_[6];
    float P_[6][6];
    float q_pos_;
    float q_vel_;
    float r_range_;
    bool is_initialized_;

public:
    ExtendedKalmanFilter3D(float q_pos = 0.01f, float q_vel = 0.05f, float r_range = 0.0225f)
        : q_pos_(q_pos), q_vel_(q_vel), r_range_(r_range), is_initialized_(false) {
        for (int i = 0; i < 6; ++i) {
            x_[i] = 0.0f;
            for (int j = 0; j < 6; ++j) {
                P_[i][j] = (i == j) ? 1.0f : 0.0f;
            }
        }
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

    bool isInitialized() const { return is_initialized_; }

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

    Point3D get_position() const {
        Point3D p = {x_[0], x_[1], x_[2]};
        return p;
    }
};

static ExtendedKalmanFilter3D ekf(0.01f, 0.05f, 0.0225f);

Point3D apply_ekf_filter(const Anchor* anchors, size_t num_anchors, const Point3D& raw_pos, float dt) {
    if (!ekf.isInitialized()) {
        ekf.init(raw_pos.x, raw_pos.y, raw_pos.z);
    } else {
        ekf.predict(dt);
        for (size_t i = 0; i < num_anchors; ++i) {
            ekf.update_range_ekf(
                anchors[i].pos.x,
                anchors[i].pos.y,
                anchors[i].pos.z,
                anchors[i].distance
            );
        }
    }
    return ekf.get_position();
}

static float get_simulated_noise_m(void) {
    static uint32_t seed = 123456789;
    seed = seed * 1103515245 + 12345;
    uint32_t val = (seed / 65536) % 30;
    return ((float)val - 15.0f) / 100.0f;
}

int main(void) {
    Anchor anchors[4] = {
        {{0.0f, 0.0f, 2.5f}, 0.0f}, // Anchor 1 (Top-Left)
        {{5.0f, 0.0f, 0.5f}, 0.0f}, // Anchor 2 (Bottom-Right)
        {{0.0f, 5.0f, 0.5f}, 0.0f}, // Anchor 3 (Bottom-Left)
        {{5.0f, 5.0f, 2.5f}, 0.0f}  // Anchor 4 (Top-Right)
    };

    Point3D raw_tag_pos;
    float time_step = 0.0f;

    printk("=== Starting Drone Positioning Simulation with 6-State EKF (No Eigen) ===\n");

    while (1) {
        Point3D true_pos;
        true_pos.x = 2.5f + 1.5f * cosf(time_step);
        true_pos.y = 2.5f + 1.5f * sinf(time_step);
        true_pos.z = 1.5f + 0.5f * sinf(time_step * 0.5f);

        for (size_t i = 0; i < 4; ++i) {
            float dx = true_pos.x - anchors[i].pos.x;
            float dy = true_pos.y - anchors[i].pos.y;
            float dz = true_pos.z - anchors[i].pos.z;
            float true_dist = sqrtf(dx * dx + dy * dy + dz * dz);

            float noise = get_simulated_noise_m();
            anchors[i].distance = true_dist + noise;
        }

        if (Trilateration::calculate_3d_position(anchors, 4, raw_tag_pos)) {
            Point3D filtered_tag_pos = apply_ekf_filter(anchors, 4, raw_tag_pos, 0.15f);

            printk("TRUE     -> X: %5.2f | Y: %5.2f | Z: %5.2f\n",
                   (double)true_pos.x, (double)true_pos.y, (double)true_pos.z);
            printk("RAW      -> X: %5.2f | Y: %5.2f | Z: %5.2f\n",
                   (double)raw_tag_pos.x, (double)raw_tag_pos.y, (double)raw_tag_pos.z);
            printk("EKF      -> X: %5.2f | Y: %5.2f | Z: %5.2f\n",
                   (double)filtered_tag_pos.x, (double)filtered_tag_pos.y, (double)filtered_tag_pos.z);
            printk("--------------------------------------------------\n");
        } else {
            printk("Trilateration failed: Degenerate matrix configuration.\n");
        }

        time_step += 0.15f;
        k_msleep(500);
    }

    return 0;
}

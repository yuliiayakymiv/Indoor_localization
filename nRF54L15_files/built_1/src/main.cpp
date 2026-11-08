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
    /**
     * @brief LOC-02: 2D Localization (X, Y) using >= 3 beacons in the horizontal plane.
     */
    static bool calculate_2d_position(const Anchor* anchors, size_t num_anchors, Point3D& out_pos) {
        if (anchors == nullptr || num_anchors < 3) {
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

        float AtA[2][2] = {0};
        float AtB[2] = {0};

        for (size_t i = 0; i < num_anchors; ++i) {
            if (i == ref_idx) {
                continue;
            }

            const Point3D& pi = anchors[i].pos;
            float di = anchors[i].distance;

            float A_row[2] = {
                2.0f * (pi.x - p1.x),
                2.0f * (pi.y - p1.y)
            };

            float b_i = (pi.x * pi.x + pi.y * pi.y - di * di) -
                        (p1.x * p1.x + p1.y * p1.y - d1 * d1);

            float w = 1.0f / (di + d1 + 1e-3f);

            for (int r = 0; r < 2; ++r) {
                for (int c = 0; c < 2; ++c) {
                    AtA[r][c] += A_row[r] * A_row[c] * w;
                }
                AtB[r] += A_row[r] * b_i * w;
            }
        }

        float det = AtA[0][0] * AtA[1][1] - AtA[0][1] * AtA[1][0];
        if (fabsf(det) < 1e-5f) {
            return false;
        }

        float inv_det = 1.0f / det;
        out_pos.x = ( AtA[1][1] * AtB[0] - AtA[0][1] * AtB[1]) * inv_det;
        out_pos.y = (-AtA[1][0] * AtB[0] + AtA[0][0] * AtB[1]) * inv_det;
        out_pos.z = 0.0f;

        return true;
    }
};

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

    Point3D get_position() const {
        Point3D p = {x_[0], x_[1], x_[2]};
        return p;
    }
};

static ExtendedKalmanFilter3D ekf_2d(0.01f, 0.05f, 0.0225f);

Point3D apply_ekf_2d(const Anchor* anchors, size_t num_anchors, const Point3D& raw_2d_pos, float dt) {
    if (!ekf_2d.isInitialized()) {
        ekf_2d.init(raw_2d_pos.x, raw_2d_pos.y, 0.0f);
    } else {
        ekf_2d.predict(dt);
        for (size_t i = 0; i < num_anchors; ++i) {
            ekf_2d.update_range_ekf_2d(
                anchors[i].pos.x,
                anchors[i].pos.y,
                anchors[i].distance
            );
        }
    }
    return ekf_2d.get_position();
}

static float get_simulated_noise_m(void) {
    static uint32_t seed = 123456789;
    seed = seed * 1103515245 + 12345;
    uint32_t val = (seed / 65536) % 30;
    return ((float)val - 15.0f) / 100.0f;
}

/**
 * @brief LOC-02 Verification: Test 2D localization with 3 beacons on known test coordinates first
 */
static void verify_2d_on_known_checkpoints(Anchor anchors[3]) {
    const Point3D test_points[4] = {
        {2.50f, 2.50f, 0.0f}, // CP-1: Center of room
        {1.00f, 1.50f, 0.0f}, // CP-2: Near Beacon 1
        {3.50f, 1.00f, 0.0f}, // CP-3: Near Beacon 2
        {2.00f, 4.00f, 0.0f}  // CP-4: Near Beacon 3
    };

    printk("=== LOC-02: Verifying 2D Localization (3 Beacons) on Known Coordinates ===\n");
    for (int p = 0; p < 4; ++p) {
        for (size_t i = 0; i < 3; ++i) {
            float dx = test_points[p].x - anchors[i].pos.x;
            float dy = test_points[p].y - anchors[i].pos.y;
            anchors[i].distance = sqrtf(dx * dx + dy * dy);
        }

        Point3D calc_pos;
        if (Trilateration::calculate_2d_position(anchors, 3, calc_pos)) {
            float err = sqrtf((calc_pos.x - test_points[p].x) * (calc_pos.x - test_points[p].x) +
                              (calc_pos.y - test_points[p].y) * (calc_pos.y - test_points[p].y));
            printk("CP-%d Expected: (%4.2f, %4.2f) | Calc 2D: (%4.2f, %4.2f) | Err: %5.4f m [%s]\n",
                   p + 1,
                   (double)test_points[p].x, (double)test_points[p].y,
                   (double)calc_pos.x, (double)calc_pos.y,
                   (double)err,
                   (err < 0.01f) ? "OK" : "FAIL");
        }
    }
    printk("==========================================================================\n\n");
}

int main(void) {
    // 3 Beacons placed in 2D plane (5m x 5m room) for LOC-02
    Anchor anchors_2d[3] = {
        {{0.0f, 0.0f, 0.0f}, 0.0f}, // Beacon 1 (Origin)
        {{5.0f, 0.0f, 0.0f}, 0.0f}, // Beacon 2 (X-axis corner)
        {{0.0f, 5.0f, 0.0f}, 0.0f}  // Beacon 3 (Y-axis corner)
    };

    // Step 1 (LOC-02): Verify 2D localization on known static coordinates first
    verify_2d_on_known_checkpoints(anchors_2d);

    Point3D raw_2d_pos;
    float time_step = 0.0f;

    printk("=== Starting 2D Localization (3 Beacons) + 2D EKF Simulation ===\n");

    while (1) {
        Point3D true_pos = {
            2.5f + 1.5f * cosf(time_step),
            2.5f + 1.5f * sinf(time_step),
            0.0f
        };

        for (size_t i = 0; i < 3; ++i) {
            float dx = true_pos.x - anchors_2d[i].pos.x;
            float dy = true_pos.y - anchors_2d[i].pos.y;
            float true_dist = sqrtf(dx * dx + dy * dy);
            anchors_2d[i].distance = true_dist + get_simulated_noise_m();
        }

        // Base 2D Localization from 3 beacons (LOC-02)
        if (Trilateration::calculate_2d_position(anchors_2d, 3, raw_2d_pos)) {
            // EKF added after stable 2D X/Y (LOC-03)
            Point3D ekf_pos = apply_ekf_2d(anchors_2d, 3, raw_2d_pos, 0.15f);

            printk("TRUE   -> X: %5.2f | Y: %5.2f\n",
                   (double)true_pos.x, (double)true_pos.y);
            printk("RAW_2D -> X: %5.2f | Y: %5.2f (3 Beacons WLS)\n",
                   (double)raw_2d_pos.x, (double)raw_2d_pos.y);
            printk("EKF_2D -> X: %5.2f | Y: %5.2f (Smoothed EKF)\n",
                   (double)ekf_pos.x, (double)ekf_pos.y);
            printk("--------------------------------------------------\n");
        } else {
            printk("2D Trilateration failed: Collinear beacons.\n");
        }

        time_step += 0.15f;
        k_msleep(500);
    }

    return 0;
}

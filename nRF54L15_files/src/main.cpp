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
     * @brief Calculates the 3D position based on distance measurements to anchors
     *        using Weighted Least Squares (WLS) and dynamic reference selection.
     * @param anchors Array of anchors with known coordinates and distances
     * @param num_anchors Number of anchors (must be >= 4 for 3D)
     * @param out_pos Calculated output coordinates (X, Y, Z)
     * @return true if the calculation is successful; false if data is invalid or matrix is singular
     */
    static bool calculate_3d_position(const Anchor* anchors, size_t num_anchors, Point3D& out_pos) {
        if (anchors == nullptr || num_anchors < 4) {
            return false; // For 3D location we need at least 4 anchors
        }

        // Dynamically select the nearest anchor as reference (p1) to reduce d^2 noise propagation
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

        // Initialization
        float AtA[3][3] = {0};
        float AtB[3] = {0};

        // Constructing AtA and AtB matrices with Weighted Least Squares (WLS)
        for (size_t i = 0; i < num_anchors; ++i) {
            if (i == ref_idx) {
                continue; // Skip the reference anchor itself
            }

            const Point3D& pi = anchors[i].pos;
            float di = anchors[i].distance;

            // Row of matrix A
            float A_row[3] = {
                2.0f * (pi.x - p1.x),
                2.0f * (pi.y - p1.y),
                2.0f * (pi.z - p1.z)
            };

            // Element of matrix B
            float b_i = (pi.x * pi.x + pi.y * pi.y + pi.z * pi.z - di * di) -
                        (p1.x * p1.x + p1.y * p1.y + p1.z * p1.z - d1 * d1);

            // Weighting factor: closer anchors get higher weights (1 / distance)
            float w = 1.0f / (di + d1 + 1e-3f);

            // Accumulating AtA (3x3) and AtB (3x1) using weight w
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) {
                    AtA[r][c] += A_row[r] * A_row[c] * w;
                }
                AtB[r] += A_row[r] * b_i * w;
            }
        }

        // Calculating determinant of matrix AtA
        float det = AtA[0][0] * (AtA[1][1] * AtA[2][2] - AtA[1][2] * AtA[2][1])
                  - AtA[0][1] * (AtA[1][0] * AtA[2][2] - AtA[1][2] * AtA[2][0])
                  + AtA[0][2] * (AtA[1][0] * AtA[2][1] - AtA[1][1] * AtA[2][0]);

        // Check for singularity (if the determinant is close to 0)
        if (fabsf(det) < 1e-5f) {
            return false; // Anchors are placed incorrectly (on the same line or plane)
        }

        float inv_det = 1.0f / det;

        // Calculation of the inverse matrix (AtA)^-1
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

        // Multiplication of an inverse matrix by vector AtB: X = (AtA)^-1 * AtB
        out_pos.x = inv[0][0] * AtB[0] + inv[0][1] * AtB[1] + inv[0][2] * AtB[2];
        out_pos.y = inv[1][0] * AtB[0] + inv[1][1] * AtB[1] + inv[1][2] * AtB[2];
        out_pos.z = inv[2][0] * AtB[0] + inv[2][1] * AtB[1] + inv[2][2] * AtB[2];

        return true;
    }
};

class KalmanFilter1D {
private:
    float q; // Process noise
    float r; // Measurement noise
    float x; // Estimated value
    float p; // Estimation error
    float k; // Kalman gain

public:
    // Fixed initialization order (q, r, x, p, k) to match class member declarations
    KalmanFilter1D(float process_noise = 0.01f, float measurement_noise = 0.25f, float estimation_error = 1.0f)
        : q(process_noise), r(measurement_noise), x(0.0f), p(estimation_error), k(0.0f) {}

    void init(float initial_value) {
        x = initial_value;
    }

    float update(float measurement) {
        p = p + q;
        k = p / (p + r);
        x = x + k * (measurement - x);
        p = (1.0f - k) * p;
        return x;
    }

    float get_value() const { return x; }
};

static KalmanFilter1D kalmanX(0.01f, 0.2f);
static KalmanFilter1D kalmanY(0.01f, 0.2f);
static KalmanFilter1D kalmanZ(0.01f, 0.2f);
static bool is_kalman_initialized = false;

// Function for processing and smoothing noisy coordinates
Point3D apply_kalman_filter(float raw_x, float raw_y, float raw_z) {
    if (!is_kalman_initialized) {
        kalmanX.init(raw_x);
        kalmanY.init(raw_y);
        kalmanZ.init(raw_z);
        is_kalman_initialized = true;
    }

    Point3D filtered_pos;
    filtered_pos.x = kalmanX.update(raw_x);
    filtered_pos.y = kalmanY.update(raw_y);
    filtered_pos.z = kalmanZ.update(raw_z);

    return filtered_pos;
}

// Simple embedded pseudo-random generator (LCG) to simulate measurement noise (-0.15m to +0.15m)
static float get_simulated_noise_m(void) {
    static uint32_t seed = 123456789;
    seed = seed * 1103515245 + 12345;
    uint32_t val = (seed / 65536) % 30; // 0..29
    return ((float)val - 15.0f) / 100.0f; // -0.15f .. +0.14f
}

int main(void) {
    // Anchor coordinates in 3D room (5m x 5m x 3m)
    Anchor anchors[4] = {
        {{0.0f, 0.0f, 2.5f}, 0.0f}, // Anchor 1 (Top-Left)
        {{5.0f, 0.0f, 0.5f}, 0.0f}, // Anchor 2 (Bottom-Right)
        {{0.0f, 5.0f, 0.5f}, 0.0f}, // Anchor 3 (Bottom-Left)
        {{5.0f, 5.0f, 2.5f}, 0.0f}  // Anchor 4 (Top-Right)
    };

    Point3D raw_tag_pos;
    float time_step = 0.0f;

    printk("=== Starting Drone Positioning Simulation with Kalman Filtering ===\n");

    while (1) {
        // 1. SIMULATION: Calculate "True Drone Position" moving in a 3D circle
        Point3D true_pos;
        true_pos.x = 2.5f + 1.5f * cosf(time_step);
        true_pos.y = 2.5f + 1.5f * sinf(time_step);
        true_pos.z = 1.5f + 0.5f * sinf(time_step * 0.5f);

        // 2. SIMULATION: Compute real distances to anchors + add random measurement noise (+- 15cm)
        for (size_t i = 0; i < 4; ++i) {
            float dx = true_pos.x - anchors[i].pos.x;
            float dy = true_pos.y - anchors[i].pos.y;
            float dz = true_pos.z - anchors[i].pos.z;
            float true_dist = sqrtf(dx * dx + dy * dy + dz * dz);

            float noise = get_simulated_noise_m();
            anchors[i].distance = true_dist + noise;
        }

        // 3. TRILATERATION: Calculate raw noisy position
        if (Trilateration::calculate_3d_position(anchors, 4, raw_tag_pos)) {

            // 4. KALMAN FILTER: Smooth the raw calculated coordinates
            Point3D filtered_tag_pos = apply_kalman_filter(raw_tag_pos.x, raw_tag_pos.y, raw_tag_pos.z);

            // Print True, Raw, and Filtered values
            printk("TRUE     -> X: %5.2f | Y: %5.2f | Z: %5.2f\n",
                   (double)true_pos.x, (double)true_pos.y, (double)true_pos.z);
            printk("RAW      -> X: %5.2f | Y: %5.2f | Z: %5.2f\n",
                   (double)raw_tag_pos.x, (double)raw_tag_pos.y, (double)raw_tag_pos.z);
            printk("FILTERED -> X: %5.2f | Y: %5.2f | Z: %5.2f\n",
                   (double)filtered_tag_pos.x, (double)filtered_tag_pos.y, (double)filtered_tag_pos.z);
            printk("--------------------------------------------------\n");

        } else {
            printk("Trilateration failed: Degenerate matrix configuration.\n");
        }

        // Advance simulation time step (~10 degrees per tick)
        time_step += 0.15f;
        k_msleep(500); // 2Hz update rate for demo
    }

    return 0;
}

#include <math.h>
#include <stddef.h>
#include "trilateration.hpp"
#include "kalman.hpp"

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

static KalmanFilter1D kalmanX(0.01f, 0.25f);
static KalmanFilter1D kalmanY(0.01f, 0.25f);
static KalmanFilter1D kalmanZ(0.01f, 0.25f);
static bool is_kalman_initialized = false;

//function for processing and smoothing noisy coordinates
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

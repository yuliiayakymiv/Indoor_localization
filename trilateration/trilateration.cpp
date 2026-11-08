#include <math.h>
#include <stddef.h>
#include "trilateration.hpp"
#include "kalman.hpp"

namespace Trilateration {

/**
 * @brief Calculates the 2D position (X, Y) based on distance measurements to >= 3 anchors
 *        using Weighted Least Squares (WLS) and dynamic reference selection (LOC-02).
 * @param anchors Array of anchors with known coordinates (X, Y) and 2D distances
 * @param num_anchors Number of anchors (must be >= 3 for 2D)
 * @param out_pos Calculated output coordinates (X, Y, Z = 0.0f)
 * @return true if the calculation is successful; false if data is invalid or anchors are collinear
 */
bool calculate_2d_position(const Anchor* anchors, size_t num_anchors, Point3D& out_pos) {
    if (anchors == nullptr || num_anchors < 3) {
        return false; // For 2D location we need at least 3 anchors
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

    float AtA[2][2] = {0};
    float AtB[2] = {0};

    // Constructing 2x2 AtA and 2x1 AtB matrices with Weighted Least Squares (WLS)
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

    // Determinant of 2x2 matrix AtA
    float det = AtA[0][0] * AtA[1][1] - AtA[0][1] * AtA[1][0];
    if (fabsf(det) < 1e-5f) {
        return false; // Anchors are collinear (on the same line)
    }

    float inv_det = 1.0f / det;

    out_pos.x = ( AtA[1][1] * AtB[0] - AtA[0][1] * AtB[1]) * inv_det;
    out_pos.y = (-AtA[1][0] * AtB[0] + AtA[0][0] * AtB[1]) * inv_det;
    out_pos.z = 0.0f;

    return true;
}

/**
 * @brief Calculates the 3D position based on distance measurements to anchors
 *        using Weighted Least Squares (WLS) and dynamic reference selection.
 */
bool calculate_3d_position(const Anchor* anchors, size_t num_anchors, Point3D& out_pos) {
    if (anchors == nullptr || num_anchors < 4) {
        return false; // For 3D location we need at least 4 anchors
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

} // namespace Trilateration

static KalmanFilter3D kalman;

Point3D apply_kalman_filter(float raw_x, float raw_y, float raw_z, float dt) {
    kalman.predict(dt);
    kalman.update(raw_x, raw_y, raw_z);

    Point3D filtered_pos = {
        kalman.getX(),
        kalman.getY(),
        kalman.getZ()
    };
    return filtered_pos;
}

/**
 * @brief 2D EKF added on top of stable 2D X/Y localization from >= 3 beacons (LOC-03)
 */
Point3D apply_ekf_2d(const Anchor* anchors, size_t num_anchors, float dt) {
    if (!kalman.isInitialized()) {
        Point3D init_pos;
        if (Trilateration::calculate_2d_position(anchors, num_anchors, init_pos)) {
            kalman.init(init_pos.x, init_pos.y, 0.0f);
        }
    } else {
        kalman.predict(dt);
        for (size_t i = 0; i < num_anchors; ++i) {
            kalman.update_range_ekf_2d(
                anchors[i].pos.x,
                anchors[i].pos.y,
                anchors[i].distance
            );
        }
    }

    Point3D filtered_pos = {
        kalman.getX(),
        kalman.getY(),
        0.0f
    };
    return filtered_pos;
}

/**
 * @brief 3D EKF using >= 4 non-coplanar beacons
 */
Point3D apply_ekf_filter(const Anchor* anchors, size_t num_anchors, float dt) {
    if (!kalman.isInitialized()) {
        Point3D init_pos;
        if (Trilateration::calculate_3d_position(anchors, num_anchors, init_pos)) {
            kalman.init(init_pos.x, init_pos.y, init_pos.z);
        }
    } else {
        kalman.predict(dt);
        for (size_t i = 0; i < num_anchors; ++i) {
            kalman.update_range_ekf(
                anchors[i].pos.x,
                anchors[i].pos.y,
                anchors[i].pos.z,
                anchors[i].distance
            );
        }
    }

    Point3D filtered_pos = {
        kalman.getX(),
        kalman.getY(),
        kalman.getZ()
    };
    return filtered_pos;
}

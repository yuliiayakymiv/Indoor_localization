#include <math.h>
#include <stddef.h>
#include "trilateration.hpp"
#include "kalman.hpp"

namespace {

// Solves the symmetric system A x = b (n = 2 or 3). Returns false if A is (nearly)
// singular, judged relative to its scale, so the check does not depend on the units.
bool solve_sym(int n, const float A[3][3], const float b[3], float x[3]) {
    if (n == 2) {
        float det = A[0][0] * A[1][1] - A[0][1] * A[1][0];
        float tr = A[0][0] + A[1][1];
        if (!(tr > 0.0f) || fabsf(det) < 1e-5f * tr * tr) {
            return false;
        }
        x[0] = ( A[1][1] * b[0] - A[0][1] * b[1]) / det;
        x[1] = (-A[1][0] * b[0] + A[0][0] * b[1]) / det;
        return true;
    }

    float det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
              - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
              + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
    float tr = A[0][0] + A[1][1] + A[2][2];
    if (!(tr > 0.0f) || fabsf(det) < 1e-6f * tr * tr * tr) {
        return false;
    }
    float inv[3][3];
    inv[0][0] =  (A[1][1] * A[2][2] - A[1][2] * A[2][1]) / det;
    inv[0][1] =  (A[0][2] * A[2][1] - A[0][1] * A[2][2]) / det;
    inv[0][2] =  (A[0][1] * A[1][2] - A[0][2] * A[1][1]) / det;
    inv[1][0] =  (A[1][2] * A[2][0] - A[1][0] * A[2][2]) / det;
    inv[1][1] =  (A[0][0] * A[2][2] - A[0][2] * A[2][0]) / det;
    inv[1][2] =  (A[0][2] * A[1][0] - A[0][0] * A[1][2]) / det;
    inv[2][0] =  (A[1][0] * A[2][1] - A[1][1] * A[2][0]) / det;
    inv[2][1] =  (A[0][1] * A[2][0] - A[0][0] * A[2][1]) / det;
    inv[2][2] =  (A[0][0] * A[1][1] - A[0][1] * A[1][0]) / det;
    for (int r = 0; r < 3; ++r) {
        x[r] = inv[r][0] * b[0] + inv[r][1] * b[1] + inv[r][2] * b[2];
    }
    return true;
}

// Linear weighted least squares (subtract the equation of the nearest beacon).
// n = 2: x, y; n = 3: x, y, z.
bool linear_ls(const Anchor* anchors, size_t num_anchors, int n, Point3D& out) {
    size_t ref = 0;
    for (size_t i = 1; i < num_anchors; ++i) {
        if (anchors[i].distance < anchors[ref].distance) {
            ref = i;
        }
    }
    const Point3D& p1 = anchors[ref].pos;
    const float d1 = anchors[ref].distance;
    const float p1sq = p1.x * p1.x + p1.y * p1.y + (n == 3 ? p1.z * p1.z : 0.0f);

    float AtA[3][3] = {{0}};
    float AtB[3] = {0};

    for (size_t i = 0; i < num_anchors; ++i) {
        if (i == ref) {
            continue;
        }
        const Point3D& pi = anchors[i].pos;
        const float di = anchors[i].distance;
        const float row[3] = {2.0f * (pi.x - p1.x), 2.0f * (pi.y - p1.y),
                              n == 3 ? 2.0f * (pi.z - p1.z) : 0.0f};
        const float pisq = pi.x * pi.x + pi.y * pi.y + (n == 3 ? pi.z * pi.z : 0.0f);
        const float b = (pisq - di * di) - (p1sq - d1 * d1);

        // The noise of d^2 grows with d (var(d^2) ~ 4 d^2 sigma^2): weight 1 / (di^2 + d1^2)
        const float w = 1.0f / (di * di + d1 * d1 + 1e-3f);

        for (int r = 0; r < n; ++r) {
            for (int c = 0; c < n; ++c) {
                AtA[r][c] += row[r] * row[c] * w;
            }
            AtB[r] += row[r] * b * w;
        }
    }

    float x[3] = {0.0f, 0.0f, 0.0f};
    if (!solve_sym(n, AtA, AtB, x)) {
        return false;
    }
    out.x = x[0];
    out.y = x[1];
    out.z = (n == 3) ? x[2] : 0.0f;
    return isfinite(out.x) && isfinite(out.y) && isfinite(out.z);
}

bool gauss_newton(const Anchor* anchors, size_t num_anchors, int n, Point3D& p, int iterations) {
    float x[3] = {p.x, p.y, p.z};

    for (int it = 0; it < iterations; ++it) {
        float JtJ[3][3] = {{0}};
        float Jtr[3] = {0};

        for (size_t i = 0; i < num_anchors; ++i) {
            float d[3] = {x[0] - anchors[i].pos.x, x[1] - anchors[i].pos.y,
                          n == 3 ? x[2] - anchors[i].pos.z : 0.0f};
            float dist = sqrtf(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
            if (dist < 1e-3f) {
                dist = 1e-3f;
            }
            float J[3] = {d[0] / dist, d[1] / dist, d[2] / dist};
            float r = dist - anchors[i].distance;
            for (int a = 0; a < n; ++a) {
                for (int c = 0; c < n; ++c) {
                    JtJ[a][c] += J[a] * J[c];
                }
                Jtr[a] += J[a] * r;
            }
        }
        // small damping (Levenberg) keeps the step sane near a degenerate geometry
        float tr = 0.0f;
        for (int a = 0; a < n; ++a) {
            tr += JtJ[a][a];
        }
        for (int a = 0; a < n; ++a) {
            JtJ[a][a] += 1e-3f * tr;
        }

        float step[3] = {0.0f, 0.0f, 0.0f};
        if (!solve_sym(n, JtJ, Jtr, step)) {
            return false;
        }
        float len = 0.0f;
        for (int a = 0; a < n; ++a) {
            len += step[a] * step[a];
        }
        len = sqrtf(len);
        const float max_step = 2.0f;
        const float scale = len > max_step ? max_step / len : 1.0f;
        for (int a = 0; a < n; ++a) {
            x[a] -= step[a] * scale;
        }
        if (len < 1e-4f) {
            break;
        }
    }

    if (!isfinite(x[0]) || !isfinite(x[1]) || !isfinite(x[2])) {
        return false;
    }
    p.x = x[0];
    p.y = x[1];
    if (n == 3) {
        p.z = x[2];
    }
    return true;
}

} // namespace

namespace Trilateration {

/**
 * @brief 2D position (X, Y) from distances to >= 3 anchors: linear weighted least squares
 *        (nearest anchor as reference) + Gauss-Newton refinement (LOC-02).
 * @return false if the data is invalid or the anchors are (nearly) on one line.
 */
bool calculate_2d_position(const Anchor* anchors, size_t num_anchors, Point3D& out_pos) {
    if (anchors == nullptr || num_anchors < 3) {
        return false;
    }
    Point3D p = {0.0f, 0.0f, 0.0f};
    if (!linear_ls(anchors, num_anchors, 2, p)) {
        return false;
    }
    Point3D refined = p;
    if (gauss_newton(anchors, num_anchors, 2, refined, 8)) {
        p = refined;
    }
    out_pos = {p.x, p.y, 0.0f};
    return true;
}

/**
 * @brief 3D position from distances to >= 4 non-coplanar anchors: linear weighted least
 *        squares + Gauss-Newton refinement.
 */
bool calculate_3d_position(const Anchor* anchors, size_t num_anchors, Point3D& out_pos) {
    if (anchors == nullptr || num_anchors < 4) {
        return false;
    }
    Point3D p = {0.0f, 0.0f, 0.0f};
    if (!linear_ls(anchors, num_anchors, 3, p)) {
        return false;
    }
    Point3D refined = p;
    if (gauss_newton(anchors, num_anchors, 3, refined, 10)) {
        p = refined;
    }
    out_pos = p;
    return true;
}

bool refine_2d(const Anchor* anchors, size_t num_anchors, Point3D& io_pos, int iterations) {
    if (anchors == nullptr || num_anchors < 2) {
        return false;
    }
    return gauss_newton(anchors, num_anchors, 2, io_pos, iterations);
}

bool refine_3d(const Anchor* anchors, size_t num_anchors, Point3D& io_pos, int iterations) {
    if (anchors == nullptr || num_anchors < 3) {
        return false;
    }
    return gauss_newton(anchors, num_anchors, 3, io_pos, iterations);
}

float rms_residual(const Anchor* anchors, size_t num_anchors, const Point3D& pos, bool use_z) {
    if (anchors == nullptr || num_anchors == 0) {
        return NAN;
    }
    float ss = 0.0f;
    for (size_t i = 0; i < num_anchors; ++i) {
        float dx = pos.x - anchors[i].pos.x;
        float dy = pos.y - anchors[i].pos.y;
        float dz = use_z ? pos.z - anchors[i].pos.z : 0.0f;
        float e = sqrtf(dx * dx + dy * dy + dz * dz) - anchors[i].distance;
        ss += e * e;
    }
    return sqrtf(ss / num_anchors);
}

} // namespace Trilateration

// ---- Stand-alone filters (each keeps its own state) ----

static KalmanFilter3D kalman_pos;
static KalmanFilter3D kalman_2d;
static KalmanFilter3D kalman_3d;

Point3D apply_kalman_filter(float raw_x, float raw_y, float raw_z, float dt) {
    kalman_pos.predict(dt);
    kalman_pos.update(raw_x, raw_y, raw_z);
    return {kalman_pos.getX(), kalman_pos.getY(), kalman_pos.getZ()};
}

/**
 * @brief 2D EKF on top of 2D localization from >= 3 beacons (LOC-03). Distances must be
 *        horizontal.
 */
Point3D apply_ekf_2d(const Anchor* anchors, size_t num_anchors, float dt) {
    kalman_2d.set_lock_z(true);
    if (!kalman_2d.isInitialized()) {
        Point3D init_pos;
        if (Trilateration::calculate_2d_position(anchors, num_anchors, init_pos)) {
            kalman_2d.init(init_pos.x, init_pos.y, 0.0f);
        }
    } else {
        kalman_2d.predict(dt);
        for (size_t i = 0; i < num_anchors; ++i) {
            kalman_2d.update_range_ekf_2d(anchors[i].pos.x, anchors[i].pos.y,
                                          anchors[i].distance);
        }
    }
    return {kalman_2d.getX(), kalman_2d.getY(), 0.0f};
}

/**
 * @brief 3D EKF using >= 4 non-coplanar beacons
 */
Point3D apply_ekf_filter(const Anchor* anchors, size_t num_anchors, float dt) {
    if (!kalman_3d.isInitialized()) {
        Point3D init_pos;
        if (Trilateration::calculate_3d_position(anchors, num_anchors, init_pos)) {
            kalman_3d.init(init_pos.x, init_pos.y, init_pos.z);
        }
    } else {
        kalman_3d.predict(dt);
        for (size_t i = 0; i < num_anchors; ++i) {
            kalman_3d.update_range_ekf(anchors[i].pos.x, anchors[i].pos.y, anchors[i].pos.z,
                                       anchors[i].distance);
        }
    }
    return {kalman_3d.getX(), kalman_3d.getY(), kalman_3d.getZ()};
}

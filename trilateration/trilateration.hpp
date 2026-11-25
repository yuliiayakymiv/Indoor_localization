#pragma once

#include <stddef.h>

struct Point3D {
    float x;
    float y;
    float z;
};

struct Anchor {
    Point3D pos;     // Coordinates of anchor in space (meters)
    float distance;  // Distance to anchor (meters)
};

namespace Trilateration {
    // 2D localization (X, Y, with Z = 0) using >= 3 beacons (LOC-02).
    // Distances must be horizontal (in the plane of the beacons).
    bool calculate_2d_position(const Anchor* anchors, size_t num_anchors, Point3D& out_pos);

    // 3D localization (X, Y, Z) using >= 4 non-coplanar beacons
    bool calculate_3d_position(const Anchor* anchors, size_t num_anchors, Point3D& out_pos);

    // Gauss-Newton refinement on the true (non-linearised) distances, starting from
    // io_pos. 2D uses x, y only (z is left as is), 3D uses x, y, z. With only 3 beacons in
    // one plane the 3D version converges to the solution on the same side of the plane as
    // the starting point. Returns false if the geometry is degenerate.
    bool refine_2d(const Anchor* anchors, size_t num_anchors, Point3D& io_pos, int iterations = 8);
    bool refine_3d(const Anchor* anchors, size_t num_anchors, Point3D& io_pos, int iterations = 10);

    // RMS of (|pos - anchor| - distance), meters. 2D ignores z.
    float rms_residual(const Anchor* anchors, size_t num_anchors, const Point3D& pos, bool use_z);
}

// Stand-alone filters (each has its own state). dt in seconds since the previous call.
Point3D apply_kalman_filter(float raw_x, float raw_y, float raw_z, float dt = 0.05f);
Point3D apply_ekf_2d(const Anchor* anchors, size_t num_anchors, float dt = 0.05f);
Point3D apply_ekf_filter(const Anchor* anchors, size_t num_anchors, float dt = 0.05f);

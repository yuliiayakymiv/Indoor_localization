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
    bool calculate_3d_position(const Anchor* anchors, size_t num_anchors, Point3D& out_pos);
}

Point3D apply_kalman_filter(float raw_x, float raw_y, float raw_z, float dt = 0.05f);
Point3D apply_ekf_filter(const Anchor* anchors, size_t num_anchors, float dt = 0.05f);

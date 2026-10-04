#pragma once

struct Point3D {
    float x;
    float y;
    float z;
};

struct Anchor {
    Point3D pos;     // Coordinates of ancher in space (meters)
    float distance;  // Distance to ancher (meters)
};

namespace Trilateration {
    bool calculate_3d_position(const Anchor* anchors, int count, Point3D& result);
}

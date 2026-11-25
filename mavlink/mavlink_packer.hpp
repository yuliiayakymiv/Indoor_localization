#pragma once

#include <stdint.h>
#include "trilateration.hpp"

// Input: our beacon frame (x from IL_B1 to IL_B2, y counter-clockwise, z up = right-handed).
// Output: MAVLink local NED-style frame, also right-handed: (x, -y, -z).
// (Only negating z would give a left-handed frame, which the flight controller's EKF cannot
// align with its own axes.)
namespace MavlinkPacker {

constexpr uint16_t MAVLINK_PACKET_MAX = 280;

uint16_t pack_heartbeat(uint8_t* buf);
uint16_t pack_vision_position(const Point3D& p, uint64_t time_usec, uint8_t* buf);
// Same, with the position uncertainty (1-sigma, meters) in the covariance; the attitude part
// is marked as unknown (we do not measure it).
uint16_t pack_vision_position(const Point3D& p, uint64_t time_usec, float sigma_xy_m,
                              float sigma_z_m, uint8_t* buf);
uint16_t pack_local_position(const Point3D& p, uint32_t time_boot_ms, uint8_t* buf);

// Our frame -> NED
inline Point3D to_ned(const Point3D& p) { return {p.x, -p.y, -p.z}; }

}

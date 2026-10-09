#pragma once

#include <stdint.h>
#include "trilateration.hpp"

// Input: z up. Output: MAVLink NED (z down).
namespace MavlinkPacker {

constexpr uint16_t MAVLINK_PACKET_MAX = 280;

uint16_t pack_heartbeat(uint8_t* buf);
uint16_t pack_vision_position(const Point3D& p, uint64_t time_usec, uint8_t* buf);
uint16_t pack_local_position(const Point3D& p, uint32_t time_boot_ms, uint8_t* buf);

}

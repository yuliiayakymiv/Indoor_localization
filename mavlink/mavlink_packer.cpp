#include "mavlink_packer.hpp"

#include <math.h>
#include <common/mavlink.h>

namespace {

// For a real flight controller: COMP_ID = MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY,
// AUTOPILOT = MAV_AUTOPILOT_INVALID, SYS_ID = FC's system id.
constexpr uint8_t SYS_ID    = 1;
constexpr uint8_t COMP_ID   = MAV_COMP_ID_AUTOPILOT1;
constexpr uint8_t AUTOPILOT = MAV_AUTOPILOT_GENERIC;

}

namespace MavlinkPacker {

uint16_t pack_heartbeat(uint8_t* buf) {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(SYS_ID, COMP_ID, &msg,
                               MAV_TYPE_QUADROTOR, AUTOPILOT,
                               0, 0, MAV_STATE_ACTIVE);
    return mavlink_msg_to_send_buffer(buf, &msg);
}

uint16_t pack_vision_position(const Point3D& p, uint64_t time_usec, uint8_t* buf) {
    float covariance[21] = {NAN};
    mavlink_message_t msg;
    mavlink_msg_vision_position_estimate_pack(SYS_ID, COMP_ID, &msg,
                                              time_usec,
                                              p.x, p.y, -p.z,
                                              0.0f, 0.0f, 0.0f,
                                              covariance, 0);
    return mavlink_msg_to_send_buffer(buf, &msg);
}

uint16_t pack_local_position(const Point3D& p, uint32_t time_boot_ms, uint8_t* buf) {
    mavlink_message_t msg;
    mavlink_msg_local_position_ned_pack(SYS_ID, COMP_ID, &msg,
                                        time_boot_ms,
                                        p.x, p.y, -p.z,
                                        0.0f, 0.0f, 0.0f);
    return mavlink_msg_to_send_buffer(buf, &msg);
}

}

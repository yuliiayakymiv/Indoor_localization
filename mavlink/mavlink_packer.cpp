#include "mavlink_packer.hpp"

#include <math.h>
#include <common/mavlink.h>

namespace {

// A position sensor next to the flight controller: same system id as the FC (1 by default),
// its own component id, and a heartbeat of a companion device (not a second autopilot).
constexpr uint8_t SYS_ID    = 1;
constexpr uint8_t COMP_ID   = MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY;
constexpr uint8_t AUTOPILOT = MAV_AUTOPILOT_INVALID;
constexpr uint8_t TYPE      = MAV_TYPE_ONBOARD_CONTROLLER;

}

namespace MavlinkPacker {

uint16_t pack_heartbeat(uint8_t* buf) {
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(SYS_ID, COMP_ID, &msg,
                               TYPE, AUTOPILOT,
                               0, 0, MAV_STATE_ACTIVE);
    return mavlink_msg_to_send_buffer(buf, &msg);
}

uint16_t pack_vision_position(const Point3D& p, uint64_t time_usec, uint8_t* buf) {
    float covariance[21];
    for (int i = 0; i < 21; ++i) {
        covariance[i] = 0.0f;
    }
    covariance[0] = NAN; // "unknown" (MAVLink: NaN in the first element)
    const Point3D n = to_ned(p);
    mavlink_message_t msg;
    mavlink_msg_vision_position_estimate_pack(SYS_ID, COMP_ID, &msg,
                                              time_usec,
                                              n.x, n.y, n.z,
                                              0.0f, 0.0f, 0.0f,
                                              covariance, 0);
    return mavlink_msg_to_send_buffer(buf, &msg);
}

uint16_t pack_vision_position(const Point3D& p, uint64_t time_usec, float sigma_xy_m,
                              float sigma_z_m, uint8_t* buf) {
    // upper triangle of the 6x6 covariance (x y z roll pitch yaw), row by row:
    // diagonal elements at 0, 6, 11, 15, 18, 20
    float covariance[21];
    for (int i = 0; i < 21; ++i) {
        covariance[i] = 0.0f;
    }
    covariance[0] = sigma_xy_m * sigma_xy_m;
    covariance[6] = sigma_xy_m * sigma_xy_m;
    covariance[11] = sigma_z_m * sigma_z_m;
    covariance[15] = 10.0f; // attitude not measured: very uncertain
    covariance[18] = 10.0f;
    covariance[20] = 10.0f;
    const Point3D n = to_ned(p);
    mavlink_message_t msg;
    mavlink_msg_vision_position_estimate_pack(SYS_ID, COMP_ID, &msg,
                                              time_usec,
                                              n.x, n.y, n.z,
                                              0.0f, 0.0f, 0.0f,
                                              covariance, 0);
    return mavlink_msg_to_send_buffer(buf, &msg);
}

uint16_t pack_local_position(const Point3D& p, uint32_t time_boot_ms, uint8_t* buf) {
    const Point3D n = to_ned(p);
    mavlink_message_t msg;
    mavlink_msg_local_position_ned_pack(SYS_ID, COMP_ID, &msg,
                                        time_boot_ms,
                                        n.x, n.y, n.z,
                                        0.0f, 0.0f, 0.0f);
    return mavlink_msg_to_send_buffer(buf, &msg);
}

}

// g++ -Itrilateration -Imavlink -Ilib/mavlink -Wno-address-of-packed-member mavlink/test_mavlink_packer.cpp mavlink/mavlink_packer.cpp -o test_mavlink
#include <stdio.h>
#include <math.h>
#include <common/mavlink.h>
#include "mavlink_packer.hpp"

static int failures = 0;

static void check(bool ok, const char* what) {
    printf("[%s] %s\n", ok ? " OK " : "FAIL", what);
    if (!ok) failures++;
}

static bool near(float a, float b) { return fabsf(a - b) < 1e-5f; }

static bool parse(const uint8_t* buf, uint16_t len, mavlink_message_t& out) {
    mavlink_status_t status;
    for (uint16_t i = 0; i < len; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_1, buf[i], &out, &status)) return true;
    }
    return false;
}

int main() {
    uint8_t buf[MavlinkPacker::MAVLINK_PACKET_MAX];
    mavlink_message_t msg;
    const Point3D p = {1.25f, -2.5f, 0.8f};

    uint16_t len = MavlinkPacker::pack_heartbeat(buf);
    check(len > 0 && parse(buf, len, msg), "heartbeat parses (CRC valid)");
    check(msg.msgid == MAVLINK_MSG_ID_HEARTBEAT, "heartbeat msgid == 0");
    check(buf[0] == 0xFD, "MAVLink v2 start byte 0xFD");

    len = MavlinkPacker::pack_vision_position(p, 123456789ULL, buf);
    check(len > 0 && parse(buf, len, msg), "vision estimate parses");
    check(msg.msgid == MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE, "vision msgid == 102");
    mavlink_vision_position_estimate_t v;
    mavlink_msg_vision_position_estimate_decode(&msg, &v);
    check(v.usec == 123456789ULL, "vision timestamp");
    check(near(v.x, p.x) && near(v.y, p.y), "vision x, y");
    check(near(v.z, -p.z), "vision z converted to NED (down)");

    len = MavlinkPacker::pack_local_position(p, 4242, buf);
    check(len > 0 && parse(buf, len, msg), "local position parses");
    check(msg.msgid == MAVLINK_MSG_ID_LOCAL_POSITION_NED, "local position msgid == 32");
    mavlink_local_position_ned_t l;
    mavlink_msg_local_position_ned_decode(&msg, &l);
    check(l.time_boot_ms == 4242, "local position timestamp");
    check(near(l.x, p.x) && near(l.y, p.y) && near(l.z, -p.z), "local position x, y, z (NED)");

    printf("\n%s (%d failed)\n", failures ? "TESTS FAILED" : "ALL TESTS PASSED", failures);
    return failures ? 1 : 0;
}

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include "trilateration.hpp"

int main(void) {
    // Anchor coordinates
    Anchor anchors[4] = {
        {{0.0f, 0.0f, 2.5f}, 0.0f}, // Anchor 1
        {{5.0f, 0.0f, 0.5f}, 0.0f}, // Anchor 2
        {{0.0f, 5.0f, 0.5f}, 0.0f}, // Anchor 3
        {{5.0f, 5.0f, 2.5f}, 0.0f}  // Anchor 4
    };

    // Simulated distances from BLE Channel Sounding
    anchors[0].distance = 2.91f;
    anchors[1].distance = 3.64f;
    anchors[2].distance = 3.64f;
    anchors[3].distance = 2.91f;

    Point3D calculated_tag_pos;

    while (1) {
        if (Trilateration::calculate_3d_position(anchors, 4, calculated_tag_pos)) {
            printk("Calculated Tag Positioуn: X=%.2f, Y=%.2f, Z=%.2f\n",
                (double)calculated_tag_pos.x,
                (double)calculated_tag_pos.y,
                (double)calculated_tag_pos.z);
        } else {
            printk("Trilateration failed: Invalid matrix or bad anchor setup.\n");
        }

        k_msleep(1000);
    }

    return 0;
}

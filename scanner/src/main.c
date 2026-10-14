#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include "ble_scanner.h"

#define REPORT_INTERVAL_MS 500

static void on_beacon(uint8_t beacon_id, int8_t rssi, uint32_t time_ms)
{
	char line[32];

	ble_scanner_format_line(line, sizeof(line), beacon_id, rssi, time_ms);
	printk("%s", line);
}

int main(void)
{
	if (ble_scanner_start(on_beacon, REPORT_INTERVAL_MS) == 0) {
		printk("Scanner started\n");
	}
	return 0;
}

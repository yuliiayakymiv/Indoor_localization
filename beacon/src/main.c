#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>

#include "beacon_format.h"

/* Change for each beacon board: 1, 2, 3, ... */
#define BEACON_ID 1

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
	BT_DATA_BYTES(BT_DATA_MANUFACTURER_DATA,
		      IL_COMPANY_ID_LO, IL_COMPANY_ID_HI,
		      IL_MARKER_0, IL_MARKER_1,
		      BEACON_ID),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
		sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

int main(void)
{
	int err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (%d)\n", err);
		return 0;
	}

	/* Non-connectable advertising, interval 100-150 ms */
	err = bt_le_adv_start(BT_LE_ADV_NCONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		printk("Advertising failed to start (%d)\n", err);
		return 0;
	}

	printk("Beacon %d started\n", BEACON_ID);
	return 0;
}

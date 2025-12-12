/*
 * bq24296.cpp
 *
 *  Created on: 3 Dec 2025
 *      Author: pavloha
 */

#include <errno.h>

#include "bq24296.hpp"
#include "driver/i2c_master.h"


#include "esp_log.h"
static const char *TAG = "modem";


#define DEFAULT_I2C_TOUT_MS 50


namespace TI {


BQ24296::BQ24296(i2c_master_dev_handle_t dev): _dev(dev) {

}

void BQ24296::dump() {
	uint8_t regs[11];
	if (read(regs, 9)) {
		ESP_LOGE(TAG, "read fail");
		return;
	}
	regs[9] = read(9);
	regs[10] = read(10);

	ESP_LOG_BUFFER_HEXDUMP("vals", regs, sizeof(regs), ESP_LOG_INFO);

	static const uint16_t ilim[] = {
		100, 150, 500, 900, 1000, 1500, 2000, 3000,
	};

	static const uint8_t wdttime[] = {
		0, 40, 80, 160
	};
	static const uint8_t safetime[] = {
		5, 8, 12, 20,
	};
	static const char *vbus[] = {
		"Unknown", "Usb-host", "Adapter-Port", "OTG",
	};
	static const char *charge[] = {
		"Not-Charging", "Pre-Charge", "Fast-Charging", "Termination-Done",
	};

	printf("%s In %4dmV %4dmA\n", regs[0] & 0x80 ? "HiZ" : "Ok ", ((regs[0] >> 3) & 0x0F) * 80 + 3880, ilim[regs[0] & 0x07]);
	printf("Sys %4dmV %4dmA\n", ((regs[1] >> 1) & 0x7) * 100 + 3000, 1000 + (regs[1] & 1) * 500);
	printf("Charge %4dmA Cold -%d0C %3d%%\n", ((regs[2] >> 2) & 0x3F) * 64 + 512, 1 + (regs[2] & 2), regs[2] & 1 ? 20 : 100);
	printf("Pre %4dmA Cut %4dmA \n", ((regs[3] >> 4) & 0x0F) * 128 + 128, (regs[3] & 0x0F) * 128 + 128);
	printf("Vmax %4dmV vMin %sV Thershold %3dmV\n", ((regs[4] >> 2) & 0x3F) * 16 + 3504, regs[4] & 2 ? "3.0" : "2.8", regs[4] & 1 ? 300 : 100);
	printf("Termination %s Watchdog %3dS Safety Timer %s %2dH\n", regs[5] & 0x80 ? "Ena" : "Dis", wdttime[(regs[5] >> 4) & 0x03],
			regs[5] & 0x08 ? "Ena" : "Dis", safetime[(regs[5] >> 1) & 0x03]);

	printf("Boost %4dmV \n", ((regs[6] >> 4) & 0x0F) * 64 + 4550); // temp?
	printf("%s %s %s %s %s\n", vbus[(regs[8] >> 6) & 3], charge[(regs[8] >> 4) & 3], regs[8] & 4 ? "Good" : "fault",
			regs[8] & 2 ? "Hot" : "Cold", regs[8] & 1 ? "Low" : "");
}

int BQ24296::read(uint8_t addr) {
	if (addr > 0x0A) {
		ESP_LOGE(TAG, "invalid reg 0x%X", addr);
		return -EINVAL;
	}
	uint8_t reg_data = 0;
	int rv = i2c_master_transmit_receive(_dev, &addr, 1, &reg_data, 1, DEFAULT_I2C_TOUT_MS);
	if (rv) {
		ESP_LOGE(TAG, "read 0x%X %d", addr, rv);
		return -EIO;
	}
	return reg_data;
}


int BQ24296::read(uint8_t *regs, size_t len) {

	if (len > 9) {
		ESP_LOGE(TAG, "invalid len %d", len);
		return -EINVAL;
	}

	uint8_t addr = 0;
	int rv = i2c_master_transmit_receive(_dev, &addr, 1, regs, len, DEFAULT_I2C_TOUT_MS);
	if (rv) {
		ESP_LOGE(TAG, "read 0x%X %d", addr, rv);
		return -EIO;
	}
	return 0;
}

};




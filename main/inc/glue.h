
#pragma once

#ifdef __cplusplus
extern "C" {
#endif


#include <time.h>

#define I2C_TOOL_TIMEOUT_VALUE_MS 50

int WiFi_init(void);
int WiFi_connected(void);

int bt_init(void);
int bt_wifiScanResult(size_t count, void *arg);
int bt_connected(void);


int SNTP_request(void);
int SNTP_request_sync(uint32_t tout_s);
time_t SNTP_last(void);


#ifdef __cplusplus
}
#endif


#pragma once

#ifdef __cplusplus
extern "C" {
#endif


#include "driver/i2c_master.h"

#define I2C_TOOL_TIMEOUT_VALUE_MS 50

int register_i2c_cmd(i2c_master_bus_handle_t *pbus);

int usb_can_init(void);

int oled_lcd_init(i2c_master_bus_handle_t *i2c_bus);


#ifdef __cplusplus
}
#endif

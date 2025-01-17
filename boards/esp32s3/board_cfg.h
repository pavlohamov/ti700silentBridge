/*
 * board_cfg.h
 *
 *  Created on: Jan 12, 2024
 *      Author: pavlo.hamov@sigan.tech
 */

#pragma once

#define BOARD_CFG_ID "esp32s3 rc1"
#define BOARD_CFG_REVISION 0

#define BOARD_CFG_GPIO_MODEM_RX ((gpio_num_t)1)
#define BOARD_CFG_GPIO_MODEM_TX ((gpio_num_t)47)

#define BOARD_CFG_GPIO_TWAI_RX ((gpio_num_t)13)
#define BOARD_CFG_GPIO_TWAI_TX ((gpio_num_t)14)

#define BOARD_CFG_GPIO_I2C_SDA ((gpio_num_t)18)
#define BOARD_CFG_GPIO_I2C_SCL ((gpio_num_t)17)
#define BOARD_CFG_GPIO_BQ_INT ((gpio_num_t)21)

#define BOARD_CFG_GPIO_LED_1 ((gpio_num_t)15)
#define BOARD_CFG_GPIO_LED_2 ((gpio_num_t)16)
#define BOARD_CFG_GPIO_LED_3 ((gpio_num_t)39)
#define BOARD_CFG_GPIO_LED_4 ((gpio_num_t)40)

#define BOARD_CFG_GPIO_BUCK_2PWM ((gpio_num_t)3) // must be on when CPU is running: force PWM instead of PFM

#define BOARD_CFG_GPIO_BOOST_ENA ((gpio_num_t)46)
#define BOARD_CFG_GPIO_BOOST_2PWM ((gpio_num_t)9) // force PWM instead of PFM

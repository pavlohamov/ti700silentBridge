/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "sdkconfig.h"
#include "board_cfg.h"

#include "esp_system.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_fat.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "cmd_system.h"
#include "cmd_wifi.h"
#include "cmd_nvs.h"

#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "driver/gpio.h"


#include "modem_pwm.hpp"
#include "bq24296.hpp"

#include "glue.h"

#include "esp_log.h"
static const char *TAG = "main";

#ifndef MIN
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#endif

typedef struct {
	const gpio_num_t g;
	const int state;
} gpio_initer_t;

static int gpio_init(void) {

    static const gpio_initer_t leds[] = {
		{ BOARD_CFG_GPIO_BUCK_2PWM, 1 },

		{ BOARD_CFG_GPIO_BOOST_2PWM, 1 },
		{ BOARD_CFG_GPIO_BOOST_ENA, 0 },

		{ BOARD_CFG_GPIO_LED_1, 0 },
		{ BOARD_CFG_GPIO_LED_2, 0 },
		{ BOARD_CFG_GPIO_LED_3, 0 },
		{ BOARD_CFG_GPIO_LED_4, 0 },
    };

    for (size_t i = 0; i < sizeof(leds) / sizeof(*leds); ++i) {
        gpio_config_t cfg = {
            .pin_bit_mask = BIT64(leds[i].g),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_set_level(leds[i].g, leds[i].state));
        ESP_ERROR_CHECK(gpio_config(&cfg));
    }

    return 0;
}


#include "driver/i2s_pdm.h"
#include <math.h>

#define EXAMPLE_PDM_TX_FREQ_HZ   32000
//#define EXAMPLE_PDM_TX_FREQ_HZ 48000
#define TONE_TIME2SAMPLES(ms) ((uint32_t)(ms * EXAMPLE_PDM_TX_FREQ_HZ / 1000UL))
#define CONST_2_PI 6.2832f

typedef struct {
	i2s_chan_handle_t chan;
	int16_t *const buf;
	const size_t blen;
	const size_t slen;
	int16_t inbuff;
	float lp;
} pdm_player_t;


static i2s_chan_handle_t i2s_example_init_pdm_tx(void) {

    i2s_chan_config_t tx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    tx_chan_cfg.auto_clear = true;
    i2s_chan_handle_t tx_chan;
    ESP_ERROR_CHECK(i2s_new_channel(&tx_chan_cfg, &tx_chan, NULL));

    i2s_pdm_tx_config_t pdm_tx_cfg = {
        .clk_cfg = I2S_PDM_TX_CLK_DEFAULT_CONFIG(EXAMPLE_PDM_TX_FREQ_HZ),
        .slot_cfg = I2S_PDM_TX_SLOT_DAC_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .clk = I2S_GPIO_UNUSED,
            .dout = GPIO_NUM_3,
            .invert_flags = {
                .clk_inv = false,
            },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_pdm_tx_mode(tx_chan, &pdm_tx_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));
    return tx_chan;
}

static void clear_tone(pdm_player_t *pl) {
	assert(pl);
	pl->lp = 0;
	pl->inbuff = 0;
}

static int append_tone(pdm_player_t *pl, uint16_t freq, float time_ms, float ampl) {
	assert(pl);
	size_t len = TONE_TIME2SAMPLES(time_ms);
	const size_t space = pl->slen - pl->inbuff;
	if (space < len) {
		ESP_LOGE(TAG, "No space left");
		return -EIO;
	}

	const size_t points = EXAMPLE_PDM_TX_FREQ_HZ / freq;
	const size_t points_ha = points / 2;
	const int addi = !!(len % points_ha);
	len = (len / points_ha) * (points_ha + addi);
	ampl *= 32768;
	float pos = 0;
	for (size_t i = !!pl->inbuff; i < len; ++i) {
		pos = CONST_2_PI * i / (float)points + pl->lp;
		pl->buf[pl->inbuff++] = sinf(pos) * ampl;
	}
	while (pos >= CONST_2_PI)
		pos -= CONST_2_PI;
	pl->lp = pos;

	ESP_LOGI(TAG, "%zu/%zu %f %zu %zu %ld", pl->inbuff, pl->slen, pos, points, points_ha, TONE_TIME2SAMPLES(time_ms));
	return 0;
}

static int append_tone_zero(pdm_player_t *pl, uint16_t freq, float ampl) {
	assert(pl);

	const float points = EXAMPLE_PDM_TX_FREQ_HZ / freq;
	ampl *= 32768;
	float pos = pl->lp + CONST_2_PI / points;
	int prev = sinf(pos) * ampl >= 0;
	while (pl->inbuff < pl->slen) {
		const float val = sinf(pos) * ampl;
		pl->buf[pl->inbuff++] = val;
		pos += CONST_2_PI / points;
		const int sign = val >= 0;
		if (sign && prev != sign)
			break;
		prev = sign;
	}
	while (pos >= CONST_2_PI)
		pos -= CONST_2_PI;
	pl->lp = pos;

	ESP_LOGI(TAG, "%zu/%zu %f", pl->inbuff, pl->slen, pos);
	return 0;
}

static int play_it(pdm_player_t *pl) {
	const size_t blen = pl->inbuff * sizeof(int16_t);
	size_t w_bytes = 0;
	int rv = 0;

//	ESP_ERROR_CHECK(i2s_channel_preload_data(pl->chan, pl->buf, blen, &w_bytes));
//    ESP_ERROR_CHECK(i2s_channel_enable(pl->chan));
	for (size_t tot_bytes = w_bytes; tot_bytes < blen; tot_bytes += w_bytes) {
		rv = i2s_channel_write(pl->chan, (uint8_t*)pl->buf + tot_bytes, blen - tot_bytes, &w_bytes, 1000);
		if (rv) {
			ESP_LOGE(TAG, "Write %d @%zu/%zu", rv, tot_bytes, blen);
			break;
		}
		ESP_LOGI(TAG, "%d Write @%zu/%zu", rv, w_bytes, tot_bytes);
	}
//	i2s_channel_disable(pl->chan);
	return rv;
}

static int i2s_test() {
    vTaskDelay(pdMS_TO_TICKS(1000));

	const size_t len = TONE_TIME2SAMPLES(2000);
	const size_t blen = len * sizeof(int16_t);

	pdm_player_t player = {
		.chan = i2s_example_init_pdm_tx(),
		.buf = (int16_t*)calloc(1, blen),
		.blen = blen,
		.slen = len,
		.inbuff = 0,
		.lp = 0,
	};
	assert(player.buf);
	static const uint32_t s_freqLut[][2] = { // call tone 2100
		{ 1070, 1270 },
		{ 2025, 2225 },
	};
	clear_tone(&player);
	float ampl = 0.9f;
	float freq = 1070.0f;
//	for (int i = 0; i < 10; ++i) {
//		append_tone(&player, freq, 30.3f, ampl);
//		freq -= 10;
//	}
//	for (int i = 0; i < 10; ++i) {
//		freq += 10;
//		append_tone(&player, freq, 30.3f, ampl);
//	}

	for (int i = 0; i < 100; ++i) {
		append_tone(&player, 2225, 3.3f, ampl);
		append_tone(&player, 2025, 3.3f, ampl);
	}
//	append_tone(&player, 1270, 3.3f, ampl);
//	append_tone(&player, 2025, 3.3f, ampl);
//	append_tone(&player, 2225, 3.3f, ampl);

	int start = player.inbuff;
//	append_tone_zero(&player, freq, ampl);

	while (1) {
		play_it(&player);
	    vTaskDelay(pdMS_TO_TICKS(1));
	}


	player.inbuff = 0;
	player.lp = 0;
#if 0
	for (int i = 0; i < sizeof(s_freqLut) / sizeof(*s_freqLut); ++i)
		append_tone(&player, s_freqLut[i], 3.3f, 0.3);
#else
	static const char text[] = "Lorem ipsum dolor sit amet consectetur adipiscing elit quisque faucibus ex sapien vitae pellentesque sem placerat";
	int bandpass = 0;
	for (size_t i = 0; i < sizeof(text); ++i) {
		const uint8_t byte = text[i];
		int rv = 0;
		rv = append_tone(&player, s_freqLut[bandpass][0], 3.3f, 0.3); // start
		for (int x = 0x80; x && !rv; x >>= 1)
			rv = append_tone(&player, s_freqLut[bandpass][!!(byte & x)], 3.3f, 0.3);
		if (rv) {
			ESP_LOGI(TAG, "encoded %zu of %zu", i, sizeof(text));
			break;
		}
	}
#endif


	while (1) {
		play_it(&player);
		ESP_LOGI(TAG, "ok");
	}
	free(player.buf);


	return 0;
}

static int pwm_test(V11::Modem *mod) {

	static const char text[] =
			"Lorem ipsum dolor sit amet, consetetur sadipscing elitr, sed diam nonumy eirmod tempor invidunt ut labore et dolore magna aliquyam erat, "
			"sed diam voluptua. At vero eos et accusam et justo duo dolores et ea rebum. Stet clita kasd gubergren, no sea takimata sanctus est "
			"Lorem ipsum dolor sit amet. Lorem ipsum dolor sit amet, consetetur sadipscing elitr, sed diam nonumy eirmod tempor invidunt ut labore et "
			"dolore magna aliquyam erat, sed diam voluptua. At vero eos et accusam et justo duo dolores et ea rebum. Stet clita kasd gubergren, no sea "
			"takimata sanctus est Lorem ipsum dolor sit amet. Lorem ipsum dolor sit amet, consetetur sadipscing elitr, sed diam nonumy eirmod "
			"tempor invidunt ut labore et dolore magna aliquyam erat, sed diam voluptua. At vero eos et accusam et justo duo dolores et ea rebum. "
			"Stet clita kasd gubergren, no sea takimata sanctus est Lorem ipsum dolor sit amet. "

			"Duis autem vel eum iriure dolor in hendrerit in vulputate velit esse molestie consequat, vel illum dolore eu feugiat nulla facilisis "
			"at vero eros et accumsan et iusto odio dignissim qui blandit praesent luptatum zzril delenit augue duis dolore te feugait nulla facilisi. "
			"Lorem ipsum dolor sit amet, consectetuer adipiscing elit, sed diam nonummy nibh euismod tincidunt ut laoreet dolore magna aliquam erat volutpat. "
			;

    vTaskDelay(pdMS_TO_TICKS(500));
    vTaskDelay(pdMS_TO_TICKS(3000));

//    gpio_set_level(BOARD_CFG_GPIO_BOOST_ENA, 1);

//	mod->write("\n\r", 2);
//    uint32_t val = 0x08040201;
//    mod->write(&val, !!sizeof(val));

//	mod->write(text, 10);
//	return 0;

    for (int i = 0x40; i; i >>= 1) {
    	i = 'a';
    	mod->write(&i, 1);
        vTaskDelay(pdMS_TO_TICKS(90));
    	return 0;
    }
	return 0;
    int val = 0xAA;
	mod->write(&val, 1);
    vTaskDelay(pdMS_TO_TICKS(90));

    val = 0x55;
	mod->write(&val, 1);
    vTaskDelay(pdMS_TO_TICKS(90));


	mod->write(text, 10);
//    for (int i = 0; i < 10; i) {
//    	mod->write(&i, 1);
//        vTaskDelay(pdMS_TO_TICKS(90));
//    }

return 0;
	int sent = 0;
    while (1) {
		ESP_LOGI(TAG, "sending");
		for (int i = 0; i < sizeof(text); ++i) {

			mod->write(text + i, 1);
			if (++sent >= 80) {
				mod->write("\n\r", 2);
				sent = 0;
			}
//			ESP_LOGI(TAG, "%6d/%d %2d%%", i, sizeof(text), i * 100 / sizeof(text));

			const int mode = (i / 30) & 1;
	//	    gpio_set_level(BOARD_CFG_GPIO_BOOST_ENA, mode);
			gpio_set_level(BOARD_CFG_GPIO_LED_1, mode);
		}

		ESP_LOGI(TAG, "done");
    }

	return 0;
}

int LogUtil_init();

extern "C" void app_main(void) {

    gpio_init();
    LogUtil_init();

	static const  i2c_master_bus_config_t i2c_bus_config = {
		.i2c_port = -1,
        .sda_io_num = BOARD_CFG_GPIO_I2C_SDA,
        .scl_io_num = BOARD_CFG_GPIO_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
		.intr_priority = 0,
		.trans_queue_depth = 0,
		.flags = {
			.enable_internal_pullup = 1,
			.allow_pd = 0,
		},
    };

    static const i2c_device_config_t i2c_bq_conf = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x6b,
        .scl_speed_hz = 400 * 1000,
		.scl_wait_us = 0,
		.flags = {
			.disable_ack_check = 0,
		}
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

    i2c_master_dev_handle_t i2c_bq;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &i2c_bq_conf, &i2c_bq));

    TI::BQ24296 *bq = new TI::BQ24296(i2c_bq);
    bq->dump();


	V11::Modem *mod = new V11::Modem(BOARD_CFG_GPIO_MODEM_RX, BOARD_CFG_GPIO_MODEM_TX, nullptr);
    pwm_test(mod);
    mod->stoptone();
//    delete mod;
//    i2s_test();
}

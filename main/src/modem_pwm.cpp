/*
 * modem_pwm.cpp
 *
 *  Created on: 29 Nov 2025
 *      Author: pavloha
 */

#include "modem_pwm.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gptimer.h"
#include "driver/ledc.h"
#include "hal/ledc_hal.h"


#include "esp_log.h"
static const char *TAG = "modem";


namespace V11 {


#define MHZ                             (1000000)
#define EXAMPLE_TIMER_RESOLUTION        (1  * MHZ)          // 1 MHz timer counting resolution
#define EXAMPLE_CALLBACK_INTERVAL_US    (3333)               // 100 us interval of each timer callback
#define EXAMPLE_ALARM_COUNT             (EXAMPLE_CALLBACK_INTERVAL_US * (EXAMPLE_TIMER_RESOLUTION / MHZ))

// old Bell 103/202
static const uint16_t s_freq[][2] = {
	{ 2025, 2225 },
	{ 1070, 1270},
};


static const ledc_timer_config_t s_ledTimer = {
	.speed_mode = LEDC_LOW_SPEED_MODE,
	.duty_resolution = LEDC_TIMER_10_BIT,
	.timer_num = LEDC_TIMER_0,
	.freq_hz = 2225,
	.clk_cfg = LEDC_AUTO_CLK,
	.deconfigure = false,
};

struct SendArg {
	int pos;
	int byte;
	SemaphoreHandle_t done;
};

static bool IRAM_ATTR data_tx(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *arg) {
	SendArg_t sa = (SendArg_t)arg;
	int bit = 1;
	BaseType_t taskAwoken = pdFALSE;
	switch (sa->pos) {
		case 0: // start
			sa->pos += 1;
			bit = 0;
			break;
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
			bit = !!(sa->byte & (1 << (sa->pos - 1)));
			sa->pos += 1;
			break;
		case 8: {
			int ones = __builtin_popcount(sa->byte & 0x7F);
			bit = (ones % 2) ? 1 : 0;
			sa->pos += 1;
		} break;
		case 9: // stop
			bit = 1;
			sa->pos += 1;
			break;
		case 10: // done
			sa->pos += 1;
			xSemaphoreGiveFromISR(sa->done, &taskAwoken);
			break;
	}
	static const uint32_t freq[] = { 2025, 2225 };
	ledc_set_freq(s_ledTimer.speed_mode, s_ledTimer.timer_num, freq[bit]);
	return taskAwoken;
}


static gptimer_handle_t allocateTimer(void* args) {

    static const gptimer_config_t timer_cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = EXAMPLE_TIMER_RESOLUTION,
    };
    gptimer_handle_t timer_handle;
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_cfg, &timer_handle));

    gptimer_alarm_config_t alarm_cfg = {
        .alarm_count = EXAMPLE_ALARM_COUNT,
        .reload_count = 0,
        .flags = {
            .auto_reload_on_alarm = true,
        },
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(timer_handle, &alarm_cfg));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = data_tx,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer_handle, &cbs, args));

    ESP_ERROR_CHECK(gptimer_set_raw_count(timer_handle, 0));
    ESP_ERROR_CHECK(gptimer_enable(timer_handle));

    ESP_LOGV(TAG, "Timer resolution %d Hz, interval %d us", timer_cfg.resolution_hz, EXAMPLE_CALLBACK_INTERVAL_US);
    return timer_handle;
}

Modem::Modem(int gpio_rx, int gpio_tx, ReceiveCb onRx) : _onRx(onRx), _lock(nullptr), _rxed(nullptr) {
	_lock = xSemaphoreCreateBinary();
	_rxed = xSemaphoreCreateBinary();

	ESP_ERROR_CHECK(ledc_timer_config(&s_ledTimer));
	ESP_ERROR_CHECK(ledc_fade_func_install(0));

	const ledc_channel_config_t s_ledCfg = {
		.gpio_num   = gpio_tx,
		.speed_mode = s_ledTimer.speed_mode,
		.channel    = LEDC_CHANNEL_0,
		.intr_type  = LEDC_INTR_DISABLE,
		.timer_sel  = s_ledTimer.timer_num,
		.duty       = 512,
		.hpoint     = 0,
		.sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
		.flags = {
			.output_invert = 0,
		},
	};
	ESP_ERROR_CHECK(ledc_channel_config(&s_ledCfg));

	_send = (SendArg_t)heap_caps_aligned_calloc(1, 1, sizeof(SendArg), (MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
	_send->pos = -1;
	_send->done = xSemaphoreCreateBinary();

	_timer = allocateTimer(_send);

	xSemaphoreGive(_lock);
}

Modem::~Modem() {
	vSemaphoreDelete(_rxed);
	vSemaphoreDelete(_lock);
}

int Modem::write(const void *data, size_t size) {

	xSemaphoreTake(_lock, portMAX_DELAY);
	xSemaphoreTake(_send->done, 0);

	const uint8_t *ptr = static_cast<const uint8_t*>(data);
	for (size_t i = 0; i < size; ++i) {
		_send->pos = 0;
		_send->byte = ptr[i];
		if (!i)
			ESP_ERROR_CHECK(gptimer_start(_timer));
		xSemaphoreTake(_send->done, portMAX_DELAY);
	}

	ESP_ERROR_CHECK(gptimer_stop(_timer));
	xSemaphoreGive(_lock);
	return 0;
}


};

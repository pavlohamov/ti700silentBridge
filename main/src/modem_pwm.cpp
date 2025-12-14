/*
 * modem_pwm.cpp
 *
 *  Created on: 29 Nov 2025
 *      Author: pavloha
 */

#include <math.h>

#include "modem_pwm.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "driver/gptimer.h"
#include "driver/ledc.h"
#include "hal/ledc_hal.h"


#include "esp_log.h"
static const char *TAG = "modem";


#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof(*x))
#endif



namespace V11 {



#ifdef CONFIG_SPIRAM
#define STACK_FLAGS (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
#else
#define STACK_FLAGS (MALLOC_CAP_DEFAULT | MALLOC_CAP_8BIT)
#endif

#define RMT_COUNT 128

#define MHZ                             (1000000)
#define RMT_RES                             (MHZ * 10)

#define BELL_BAUDRATE 300
#define BIT_TIME_US (MHZ / BELL_BAUDRATE)


#define FSK_LO_L 1070
#define FSK_LO_H 1270

#define FSK_HI_L 2025
#define FSK_HI_H 2225

#define FSK_CENTRAL ((FSK_LO_L + FSK_HI_H) / 2)

// old Bell 103/202
static const uint16_t s_freq[][2] = {
//	{ FSK_LO_L, FSK_HI_H },
	{ FSK_LO_L, FSK_LO_H },
	{ FSK_HI_L, FSK_HI_H },
};

#define TX_FREQ s_freq[1][1]

static const rmt_symbol_word_t bell_one = {
#if 01
	.duration0 = RMT_RES / TX_FREQ / 2,
    .level0 = 0,
    .duration1 = RMT_RES / TX_FREQ / 2,
    .level1 = 1,
#else
	.duration0 = RMT_RES / TX_FREQ * 9 / 10,
    .level0 = 0,
    .duration1 = RMT_RES / TX_FREQ / 10,
    .level1 = 1,
#endif
};


static size_t idle_callback(const void *data, size_t data_size, size_t symbols_written, size_t symbols_free, rmt_symbol_word_t *symbols, bool *done, void *arg) {
	if (!symbols_free)
		return 0;
    symbols[0] = bell_one;
    *done = 1;
    return 1;
}

typedef struct {
	int bit;
	int phase;
} saved_t;

IRAM_ATTR static int append(rmt_symbol_word_t *symbols, int32_t space, int bit, saved_t& sav) {

	const uint32_t freq = s_freq[1][bit];
	const uint32_t count_phase = freq * 100UL / BELL_BAUDRATE;
	const uint16_t count = count_phase / 100 ;
	int16_t partial = count_phase % 100;
	if ((count + 1) > space) {
		ESP_LOGE(TAG, "won't fit %.2f in %lu", count, space);
		return 0;
	}

	const uint32_t dur = RMT_RES / freq / 2;

	sav.bit = 0;
	sav.phase = 50;

	int pos = 0;
	symbols[pos].level0 = 0;
	symbols[pos].level1 = 1;
	if (sav.bit) {
		symbols[pos].duration0 = 1;
		symbols[pos].duration1 = 1 + dur * sav.phase / 50;
	} else {
		symbols[pos].duration0 = 1 + dur * sav.phase / 50;
		symbols[pos].duration1 = dur;
	}
	++pos;

	for (int i = 1; i < (int)count; ++i) {
		symbols[pos].duration0 = dur;
		symbols[pos].level0 = 0;
		symbols[pos].duration1 = dur;
		symbols[pos].level1 = 1;
#if 0 // for scope observation
	#if 01
		if (bit) {
			symbols[pos].duration0 = symbols[pos].duration0 / 5;
			symbols[pos].duration1 = symbols[pos].duration1 * 9 / 5;
		} else {
			symbols[pos].duration0 = symbols[pos].duration0 * 9 / 5;
			symbols[pos].duration1 = symbols[pos].duration1 / 5;
		}
	#else
		if (bit) {
			symbols[pos].duration0 = 0;
			symbols[pos].duration1 = dur * 2;
		} else {
			symbols[pos].duration0 = dur * 2;
			symbols[pos].duration1 = 0;
		}
	#endif
#endif
		++pos;
	}
	ESP_LOGE(TAG, "%d f %4d dur %d: %d.%d [%d/%d] %d", bit, freq, dur, count, partial, count + 1, space, pos);
	return pos;

	int16_t dur_1 = partial - 50;
	sav.bit = partial >= 50;
	if (sav.bit)
		partial = 50;
	else
		dur_1 = 0;
	sav.phase = 50 - partial;
// duration cant be 0
	symbols[pos].duration0 = 1 + dur * partial / 50;
	symbols[pos].duration1 = 1 + dur * dur_1 / 50;
	symbols[pos].level0 = 0;
	symbols[pos].level1 = 1;

	return pos + 1;
}

IRAM_ATTR static size_t encoder_callback(const void *data, size_t data_size, size_t symbols_written, size_t symbols_free, rmt_symbol_word_t *symbols, bool *done, void *arg) {

    if (symbols_free < 90) {
        return 0;
    }
	size_t *tx_pos = (size_t*)arg;
	if (!symbols_written)
		*tx_pos = 0;
    const size_t data_pos = *tx_pos;
    const uint8_t *data_bytes = (uint8_t*)data;

    if (data_pos >= data_size) {
        *done = 1;
        return 0;
    }
    const uint8_t byte = data_bytes[data_pos];
    ESP_LOGI(TAG, "TX: '%c' %2X@%zu sz %zu/%zu free %zu", byte, byte, data_pos, symbols_written, data_size, symbols_free);


	saved_t last = {
		.bit = 0,
		.phase = 50,
	};
	// Encode a byte
	size_t symbol_pos = 0;
	symbol_pos += append(symbols + symbol_pos, symbols_free - symbol_pos, 0, last);  // start bit
	unsigned ones = 0;
	for (uint8_t bitmask = 0x1; bitmask != 0x80; bitmask <<= 1) {
		const int bit = !!(byte & bitmask);
		ones += bit;
		symbol_pos += append(symbols + symbol_pos, symbols_free - symbol_pos, bit, last);
	}
	symbol_pos += append(symbols + symbol_pos, symbols_free - symbol_pos, ones & 1, last); // parity
	symbol_pos += append(symbols + symbol_pos, symbols_free - symbol_pos, 1, last); // stop
//	symbol_pos += append(symbols + symbol_pos, symbols_free - symbol_pos, 1, last); // stop ?
	*tx_pos += 1;

//	ESP_LOGD(TAG, "TX: inbuf %zu", symbol_pos);
	return symbol_pos;
}


class BellWriter {
public:
	BellWriter(int gpio_tx): _tx(gpio_tx) {}
	virtual ~BellWriter() {}
	virtual int write(const void *data, size_t size) = 0;
	virtual void tone(bool play) = 0;
private:
	const int _tx;
};


static const rmt_transmit_config_t s_idle_config = {
    .loop_count = -1,
};


class RmtWriter: public BellWriter {
public:
	RmtWriter(int gpio_tx): BellWriter(gpio_tx) {
	    rmt_tx_channel_config_t tx_channel_cfg = {
			.gpio_num = (gpio_num_t)gpio_tx,
	        .clk_src = RMT_CLK_SRC_DEFAULT,
	        .resolution_hz = RMT_RES,
	        .mem_block_symbols = RMT_COUNT,
	        .trans_queue_depth = 4,
			.intr_priority = 0,
			.flags = {
				.invert_out = 0,
				.with_dma = 1,
				.allow_pd = 0,
				.init_level = 0,
			},
	    };
	    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_channel_cfg, &_rmtx));

	    _tx_pos = 0;
	    const rmt_simple_encoder_config_t simple_encoder_cfg = {
	        .callback = encoder_callback,
			.arg = &_tx_pos,
			.min_chunk_size = 90,
	    };
	    const rmt_simple_encoder_config_t idle_encoder_cfg = {
	        .callback = idle_callback,
	//		.arg = &_idlePresent,
	    };

	    ESP_ERROR_CHECK(rmt_new_simple_encoder(&simple_encoder_cfg, &_data_encoder));
	    ESP_ERROR_CHECK(rmt_new_simple_encoder(&idle_encoder_cfg, &_idle_encoder));

	    ESP_ERROR_CHECK(rmt_enable(_rmtx));
	    ESP_ERROR_CHECK(rmt_transmit(_rmtx, _idle_encoder, _rmtx, 1, &s_idle_config));
	}
	virtual ~RmtWriter() {
		rmt_disable(_rmtx);
		rmt_del_encoder(_data_encoder);
		rmt_del_encoder(_idle_encoder);
		rmt_del_channel(_rmtx);
	}

	virtual int write(const void *data, size_t size) {
		static const rmt_transmit_config_t transmit_config = {
			.loop_count = 0,
		};
		ESP_ERROR_CHECK(rmt_disable(_rmtx));
		ESP_ERROR_CHECK(rmt_enable(_rmtx));
		const uint8_t *ptr = (const uint8_t *)data;
		while (size) {
			uint8_t buff[16];
			const int toTx = size > sizeof(buff) ? sizeof(buff) : size;
			memcpy(buff, ptr, toTx);
			ESP_ERROR_CHECK(rmt_transmit(_rmtx, _data_encoder, buff, toTx, &transmit_config));
			ESP_ERROR_CHECK(rmt_tx_wait_all_done(_rmtx, toTx * 133));
			ptr += toTx;
			size -= toTx;
		}

		ESP_ERROR_CHECK(rmt_transmit(_rmtx, _idle_encoder, _rmtx, 1, &s_idle_config));
		return 0;
	}
	virtual void tone(bool play) {
		if (play) {
			ESP_ERROR_CHECK(rmt_enable(_rmtx));
			ESP_ERROR_CHECK(rmt_transmit(_rmtx, _idle_encoder, _rmtx, 1, &s_idle_config));
		} else
			ESP_ERROR_CHECK(rmt_disable(_rmtx));
	}
private:
	rmt_channel_handle_t _rmtx;
	rmt_encoder_handle_t _data_encoder;
	rmt_encoder_handle_t _idle_encoder;
	size_t _tx_pos;
};

typedef struct {
	int val;
	int pos;
	SemaphoreHandle_t done;
} SendParam_t;

static bool IRAM_ATTR onPwmTimer(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
	SendParam_t *sp = (SendParam_t*)user_ctx;

	const uint16_t *lut = s_freq[1];
	uint32_t newF = lut[1];
	BaseType_t wake = 0;
	switch (sp->pos) {
		case 0:
			newF = lut[0];
			break;
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7: {
			const uint8_t mask = 0x01 << (sp->pos - 1);
			newF = lut[!!(sp->val & mask)];
		} break;
		case 8:
			newF = lut[!(__builtin_popcount(sp->val) & 1)];
			break;
		case 9:
			newF = lut[1];
			break;
		case 10:
			xSemaphoreGiveFromISR(sp->done, &wake);
			break;
		default:
			sp->pos = 100;
	}
//	ESP_LOGI(TAG, "tim %d %u %lld %lld", sp->pos, newF, edata->count_value, edata->alarm_value);
	sp->pos += 1;

    ledc_set_freq(LEDC_LOW_SPEED_MODE, LEDC_TIMER_0, newF);
    return wake;
}

class PwmWriter: public BellWriter {
public:
	PwmWriter(int gpio_tx): BellWriter(gpio_tx), _tim(nullptr), sp(nullptr) {

		static const gptimer_config_t timer_cfg = {
			.clk_src = GPTIMER_CLK_SRC_DEFAULT,
			.direction = GPTIMER_COUNT_UP,
			.resolution_hz = RMT_RES,
		};

		static const gptimer_alarm_config_t alarm_cfg = {
			.alarm_count = BIT_TIME_US * (timer_cfg.resolution_hz / MHZ),
			.reload_count = 0,
			.flags = {
				.auto_reload_on_alarm = true,
			},
		};

		static const gptimer_event_callbacks_t cbs = {
			.on_alarm = onPwmTimer,
		};

		sp = (SendParam_t*)heap_caps_aligned_calloc(sizeof(void*), 1, sizeof(SendParam_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
		sp->done = xSemaphoreCreateBinary();
		sp->pos = 99;

		ESP_ERROR_CHECK(gptimer_new_timer(&timer_cfg, &_tim));
		ESP_ERROR_CHECK(gptimer_set_alarm_action(_tim, &alarm_cfg));

		ESP_ERROR_CHECK(gptimer_register_event_callbacks(_tim, &cbs, sp));
		ESP_ERROR_CHECK(gptimer_set_raw_count(_tim, 0));

		// init led
		static const ledc_timer_config_t ledc_timer = {
	        .speed_mode       = LEDC_LOW_SPEED_MODE,
	        .duty_resolution  = LEDC_TIMER_10_BIT,
	        .timer_num        = LEDC_TIMER_0,
	        .freq_hz          = FSK_HI_H,
	        .clk_cfg          = LEDC_AUTO_CLK,
	    };

	    const ledc_channel_config_t ledc_channel = {
			.gpio_num       = gpio_tx,
	        .speed_mode     = ledc_timer.speed_mode,
	        .channel        = LEDC_CHANNEL_0,
	        .timer_sel      = ledc_timer.timer_num,
	        .duty           = 0, // Set duty to 0%
	        .hpoint         = 0,
	#if CONFIG_PM_ENABLE
	        .sleep_mode     = LEDC_SLEEP_MODE_KEEP_ALIVE,
	#endif
	    };

	    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
	    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

	    ESP_ERROR_CHECK(ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, 1 << (ledc_timer.duty_resolution - 1)));
	    ESP_ERROR_CHECK(ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel));

		ESP_ERROR_CHECK(gptimer_enable(_tim));
	}

	virtual ~PwmWriter() {
		gptimer_stop(_tim);
		gptimer_disable(_tim);
	}

	virtual int write(const void *data, size_t size) {

//		ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 512);
		ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
		int rv = -1;
		const uint8_t *ptr = static_cast<const uint8_t*>(data);
		for (size_t i = 0; i < size; ++i) {
			sp->pos = 0;
			sp->val = ptr[i];
			if (!i) {
				xSemaphoreTake(sp->done, 0);
				gptimer_start(_tim);
			}
			rv = !xSemaphoreTake(sp->done, pdMS_TO_TICKS(1000 * 13 / BELL_BAUDRATE));
			if (rv) {
				ESP_LOGE(TAG, "tx %d tout %zu/%zu", rv, i, size);
				break;
			}
		}
		ESP_ERROR_CHECK(gptimer_stop(_tim));
//		ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 1025);
//		ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
		return rv;
	}

	virtual void tone(bool play) {
		ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, play ? 512 : 1025));
		ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
	}
private:
	gptimer_handle_t _tim;
	SendParam_t *sp;
};




static bool IRAM_ATTR data_rx(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *arg) {
    QueueHandle_t q = (QueueHandle_t)arg;
    BaseType_t high_task_wakeup = 0;
    xQueueSendFromISR(q, edata, &high_task_wakeup);
    return high_task_wakeup;
}

void Modem::routine(void *arg) {
	Modem *thiz = (Modem*)arg;

	thiz->_rxq = xQueueCreate(5, sizeof(rmt_rx_done_event_data_t));

	rmt_rx_event_callbacks_t cbs = {
		.on_recv_done = data_rx,
	};
	ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(thiz->_rmrx, &cbs, thiz->_rxq));

	rmt_receive_config_t receive_config = {
		.signal_range_min_ns = 3187,
		.signal_range_max_ns = 2000000ULL,
		.flags = {
			.en_partial_rx = 1,
		},
	};

	rmt_symbol_word_t raw_symbols[RMT_COUNT];
	rmt_rx_done_event_data_t rx_data;

	ESP_ERROR_CHECK(rmt_enable(thiz->_rmrx));
	ESP_ERROR_CHECK(rmt_receive(thiz->_rmrx, raw_symbols, sizeof(raw_symbols), &receive_config));

	thiz->_carrier = 0;
	int8_t rx[RMT_COUNT * 2];
	int inbuf = 0;
	while (1) {
		if (!xQueueReceive(thiz->_rxq, &rx_data, pdMS_TO_TICKS(100))) {
			inbuf = 0;
			if (thiz->_carrier)
				ESP_LOGI(TAG, "rxtout");
			thiz->_carrier = 0;
			continue;
		}
		uint32_t avg = 0;
		bool allones = true;
		int rxed = 0;
		int8_t bits[RMT_COUNT];
		for (size_t i = 0; i < rx_data.num_symbols; ++i) {
			rmt_symbol_word_t *sy = rx_data.received_symbols + i;
			const uint16_t f = RMT_RES / (sy->duration0 + sy->duration1 + 1);
			avg += f;
			const uint16_t *freq = s_freq[f > FSK_CENTRAL];
			const int away[] = { abs(f - freq[0]), abs(f - freq[1]) };
			const int bit = away[1] < away[0];
			if (inbuf || rxed) {
				bits[rxed++] = bit;
			} else if (!bit) {
				bits[rxed++] = bit;
				allones = false;
			}
//			ESP_LOGI(TAG, "%3zu {%d:%5d},{%d:%5d}", i, sy->level0, sy->duration0, sy->level1, sy->duration1);
//			ESP_LOGI(TAG, "%3zu %d   %d %d %d", i, f, away[0], away[1], bit);
		}
		if (!thiz->_carrier && rx_data.num_symbols > 30) {
			thiz->_carrier = avg / rx_data.num_symbols;
			ESP_LOGI(TAG, "got carrier %d %d", thiz->_carrier, rx_data.num_symbols);
		}
		if (allones && !inbuf) {
			ESP_LOGV(TAG, "RX: ones %zu", rx_data.num_symbols);
			inbuf = 0;
		} else {
			for (size_t i = 0; i < rxed; ++i) {
				if (inbuf > ARRAY_SIZE(rx)) {
					ESP_LOGE(TAG, "overrun %zu/%d", i, rxed);
					inbuf = ARRAY_SIZE(rx);
					break;
				}
				rx[inbuf++] = bits[i];
			}
//			ESP_LOGI(TAG, "RX: %zu %d inbuf %d", rx_data.num_symbols, avg / rx_data.num_symbols, inbuf);
//			if (inbuf >= 61)
			{
				char *text = (char*)malloc(inbuf + 16);
				int occ = 0;
				int accu = 0;
				int pos = 0;
				// start + 7bit + parity + stop
				for (int i = 0; i < inbuf; ++i) {
					const int next = i / 4;
//					ESP_LOGW(TAG, "n %d p %d a %d", next, pos, accu);
					if (next != pos) {
						bits[pos++] = accu >= (inbuf / 10 / 2);
						accu = 0;
					}
					accu += rx[i];
					occ += snprintf(text + occ, inbuf + 16 - occ, "%d", rx[i]);
				}
				ESP_LOGI(TAG, "RX: %zu %d last %d %d %s", rx_data.num_symbols, avg / rx_data.num_symbols, rx_data.flags.is_last, inbuf, text);

				if (pos >= 10 && !bits[0] && bits[9]) {
					occ = 0;
					text[0] = '\0';
					int received = 0;
					for (int i = 0; i < pos; ++i) {
						if (i)
							received |= bits[i] ? (0x1 << (i-1)) : 0;
						occ += snprintf(text + occ, inbuf + 16 - occ, "%d", bits[i]);
					}
					received &= 0x7f;
					ESP_LOGI(TAG, "RX: %d %s %x '%c'", pos, text, received, received);
					inbuf = 0;
				}
				free(text);
			}
		}

		if (rx_data.flags.is_last) {
			ESP_LOGD(TAG, "RX: last");
			ESP_ERROR_CHECK(rmt_receive(thiz->_rmrx, raw_symbols, sizeof(raw_symbols), &receive_config));
		}
	}
}

// 0000000 0000000 0000000 1111111 1111111 0000000 0000000 0111111 0000000 011111111111111111111111111111111
// 0000000 0000000 0111111 0000000 0111111 0000000 0111111 0000000 0000000 01111111

Modem::Modem(int gpio_rx, int gpio_tx, ReceiveCb onRx): _carrier(0), _onRx(onRx), _lock(nullptr), _rxed(nullptr) {

//	_writer = new RmtWriter(gpio_tx);
	_writer = new PwmWriter(gpio_tx);

	_lock = xSemaphoreCreateBinary();
	_rxed = xSemaphoreCreateBinary();

	const rmt_rx_channel_config_t rx_channel_cfg = {
		.gpio_num = (gpio_num_t)gpio_rx,
		.clk_src = RMT_CLK_SRC_DEFAULT,
		.resolution_hz = RMT_RES,
		.mem_block_symbols = RMT_COUNT,
		.intr_priority = 0,
		.flags = {
			.invert_in = 0,
			.with_dma = 1,
			.allow_pd = 0,
		},
	};
	ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_channel_cfg, &_rmrx));

	static const size_t stack_size = 4 * 1024;
	_stack = heap_caps_aligned_calloc(16, 1, stack_size, STACK_FLAGS);
	if (!_stack) {
		ESP_LOGE(TAG, "Can't alloc stack");
		return;
	}

	_thread = (StaticTask_t*)heap_caps_aligned_calloc(sizeof(void*), 1, sizeof(StaticTask_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
	if (!_thread) {
		free(_stack);
		ESP_LOGE(TAG, "Can't alloc thread");
		return;
	}

	_tid = xTaskCreateStatic(routine, TAG, stack_size, this, 5, (StackType_t*)_stack, _thread);

	xSemaphoreGive(_lock);
}

Modem::~Modem() {
	tone(0);
	vSemaphoreDelete(_rxed);
	vSemaphoreDelete(_lock);
}

int Modem::write(const void *data, size_t size) {
	xSemaphoreTake(_lock, portMAX_DELAY);
	int rv = _writer->write(data, size);
	xSemaphoreGive(_lock);
	return rv;
}

int Modem::tone(bool play) {
	xSemaphoreTake(_lock, portMAX_DELAY);
	_writer->tone(play);
	xSemaphoreGive(_lock);
	return 0;
}


};

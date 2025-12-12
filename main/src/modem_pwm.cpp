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


static bool IRAM_ATTR data_rx(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *arg) {
    QueueHandle_t q = (QueueHandle_t)arg;
    BaseType_t high_task_wakeup = 0;
    xQueueSendFromISR(q, edata, &high_task_wakeup);
    return high_task_wakeup;
}

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

void Modem::routine(void *arg) {
	Modem *thiz = (Modem*)arg;

	thiz->_rxq = xQueueCreate(5, sizeof(rmt_rx_done_event_data_t));

	rmt_rx_event_callbacks_t cbs = {
		.on_recv_done = data_rx,
	};
	ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(thiz->_rmrx, &cbs, thiz->_rxq));

	rmt_receive_config_t receive_config = {
		.signal_range_min_ns = 3187,
		.signal_range_max_ns = 1000000ULL,
		.flags = {
			.en_partial_rx = 1,
		},
	};

	rmt_symbol_word_t raw_symbols[RMT_COUNT];
	rmt_rx_done_event_data_t rx_data;

	ESP_ERROR_CHECK(rmt_enable(thiz->_rmrx));
	ESP_ERROR_CHECK(rmt_receive(thiz->_rmrx, raw_symbols, sizeof(raw_symbols), &receive_config));

	int8_t rx[RMT_COUNT];
	int rxed = 0;
	bool carrier = false;
	while (1) {
		if (!xQueueReceive(thiz->_rxq, &rx_data, pdMS_TO_TICKS(100))) {
			rxed = 0;
			if (carrier)
				ESP_LOGI(TAG, "rxtout");
			carrier = false;
			continue;
		}
		uint32_t avg = 0;
		bool allones = true;
		for (size_t i = 0; i < rx_data.num_symbols; ++i) {
			rmt_symbol_word_t *sy = rx_data.received_symbols + i;
			const uint16_t f = RMT_RES / (sy->duration0 + sy->duration1 + 1);
			avg += f;
			const uint16_t *freq = s_freq[f > FSK_CENTRAL];
			const int away[] = { abs(f - freq[0]), abs(f - freq[1]) };
			const int bit = away[1] < away[0];
			if (rxed) {
				rx[rxed++] = bit;
			} else if (!bit) {
				rx[rxed++] = bit;
				allones = false;
			}
//			ESP_LOGI(TAG, "%3zu {%d:%5d},{%d:%5d}", i, sy->level0, sy->duration0, sy->level1, sy->duration1);
//			ESP_LOGI(TAG, "%3zu %d   %d %d %d", i, f, away[0], away[1], bit);
		}
		if (!carrier) {
			ESP_LOGI(TAG, "got carrier %d", avg / rx_data.num_symbols);
			carrier = true;
		}
		if (allones) {
			ESP_LOGV(TAG, "RX: ones %zu", rx_data.num_symbols);
			rxed = 0;
		} else {
//			ESP_LOGI(TAG, "RX: %zu %d rxed %d", rx_data.num_symbols, avg / rx_data.num_symbols, rxed);
//			if (rxed >= 61)
			{

				char *text = (char*)malloc(rxed + 16);
				int occ = 0;
				for (int i = 0; i < rxed; ++i)
					occ += snprintf(text + occ, rxed + 16 - occ, "%d", rx[i]);

				ESP_LOGI(TAG, "RX: %zu %d last %d %d %s", rx_data.num_symbols, avg / rx_data.num_symbols, rx_data.flags.is_last, rxed, text);
				free(text);
			}
		}

		if (rx_data.flags.is_last) {
			ESP_LOGD(TAG, "RX: last");
			ESP_ERROR_CHECK(rmt_receive(thiz->_rmrx, raw_symbols, sizeof(raw_symbols), &receive_config));
		}
	}
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
#if 01 // for scope observation
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

//	ESP_LOGE(TAG, "%d f %4d dur %d: %d.%d [%d/%d] %d", bit, freq, dur, count, partial, count + 1, space, pos);
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
	symbol_pos += append(symbols + symbol_pos, symbols_free - symbol_pos, 1, last);
	symbol_pos += append(symbols + symbol_pos, symbols_free - symbol_pos, 0, last);  // start bit
	unsigned ones = 0;
	for (uint8_t bitmask = 0x40; bitmask; bitmask >>= 1) {
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

static const rmt_transmit_config_t s_idle_config = {
    .loop_count = -1,
};

Modem::Modem(int gpio_rx, int gpio_tx, ReceiveCb onRx) : _onRx(onRx), _lock(nullptr), _rxed(nullptr) {
	_lock = xSemaphoreCreateBinary();
	_rxed = xSemaphoreCreateBinary();

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
//    ESP_ERROR_CHECK(rmt_transmit(_rmtx, _idle_encoder, _rmtx, 1, &s_idle_config));

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
	stoptone();
	vSemaphoreDelete(_rxed);
	vSemaphoreDelete(_lock);
}

int Modem::write(const void *data, size_t size) {

	xSemaphoreTake(_lock, portMAX_DELAY);

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

//    ESP_ERROR_CHECK(rmt_transmit(_rmtx, _idle_encoder, _rmtx, 1, &s_idle_config));
	xSemaphoreGive(_lock);
	return 0;
}


int Modem::stoptone() {
	xSemaphoreTake(_lock, portMAX_DELAY);
	ESP_ERROR_CHECK(rmt_disable(_rmtx));
	ESP_ERROR_CHECK(rmt_enable(_rmtx));
	xSemaphoreGive(_lock);
	return 0;
}


};

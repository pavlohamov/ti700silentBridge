/*
 * modem_pwm.hpp
 *
 *  Created on: 29 Nov 2025
 *      Author: pavloha
 */

#pragma once

#include <stddef.h>
#include <functional>


typedef struct QueueDefinition *QueueHandle_t;
typedef QueueHandle_t SemaphoreHandle_t;
typedef struct gptimer_t *gptimer_handle_t;

typedef struct xSTATIC_TCB StaticTask_t;
typedef struct tskTaskControlBlock* TaskHandle_t;

typedef struct rmt_channel_t *rmt_channel_handle_t;
typedef struct rmt_encoder_t *rmt_encoder_handle_t;

namespace V11 {

// negative - errno
using ReceiveCb = std::function<void(int code)>;


class Modem {
public:
	Modem(int gpio_rx, int gpio_tx, ReceiveCb onRx);
	virtual ~Modem();

	int write(const void *data, size_t size);

	int stoptone();

private:
	const ReceiveCb _onRx;
	SemaphoreHandle_t _lock;
	SemaphoreHandle_t _rxed;
	rmt_channel_handle_t _rmtx;
	rmt_channel_handle_t _rmrx;

	size_t _tx_pos;
	rmt_encoder_handle_t _data_encoder;
	rmt_encoder_handle_t _idle_encoder;
	QueueHandle_t _rxq;

	void *_stack;
	StaticTask_t *_thread;
	TaskHandle_t _tid;

	static void routine(void *arg);
};

};

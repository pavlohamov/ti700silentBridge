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

class Modem;

// negative - errno
using ReceiveCb = std::function<void(Modem& mod, int code)>;

class BellWriter;

class Modem {
public:
	Modem(int gpio_rx, int gpio_tx, ReceiveCb onRx);
	virtual ~Modem();

	uint32_t carrier() const { return _carrier; }
	int write(const void *data, size_t size);

	int tone(bool play);

private:
	uint32_t _carrier;
	BellWriter *_writer;

	const ReceiveCb _onRx;
	SemaphoreHandle_t _lock;
	SemaphoreHandle_t _rxed;

	rmt_channel_handle_t _rmrx;
	QueueHandle_t _rxq;

	void *_stack;
	StaticTask_t *_thread;
	TaskHandle_t _tid;

	void onCarrierChange(uint32_t cur);

	static void routine(void *arg);
};

};

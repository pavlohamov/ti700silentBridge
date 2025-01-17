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

namespace V11 {

typedef struct SendArg *SendArg_t;

// negative - errno
using ReceiveCb = std::function<void(int code)>;


class Modem {
public:
	Modem(int gpio_rx, int gpio_tx, ReceiveCb onRx);
	virtual ~Modem();

	int write(const void *data, size_t size);

private:
	const ReceiveCb _onRx;
	SemaphoreHandle_t _lock;
	SemaphoreHandle_t _rxed;
	gptimer_handle_t _timer;
	SendArg_t _send;
};

};

/*
 * logUtil.cpp
 *
 *  Created on: 12 Jan 2024
 *      Author: pavloha
 */

#include <unistd.h>
#include <errno.h>

#include "esp_random.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"


#include "esp_log.h"
static const char *TAG = "asylog";


#define RB_SIZE (16 * 1024)

static struct {
	vprintf_like_t org;

	SemaphoreHandle_t sem;
	RingbufHandle_t rb;
	QueueHandle_t q;
} s_int;


static int my_print(const char *fmt, va_list args) {
	static const int size = 2048;
//	char *buff = (char*)malloc(size);
//	if (!buff)
//		return 0;

	static char buff[size];

	int occ = vsnprintf(buff, size, fmt, args);
	int rv = xRingbufferSend(s_int.rb, buff, occ, 0);
//	free(buff);
	if (rv)
		xSemaphoreGive(s_int.sem);
	return rv ? occ : 0;

}
static void log_writer(void *arg) {
	while (1) {
		xSemaphoreTake(s_int.sem, pdMS_TO_TICKS(2000));
		size_t size = 0;
		void *data = xRingbufferReceiveUpTo(s_int.rb, &size, portMAX_DELAY, RB_SIZE);
		if (!size || !data)
			continue;
		write(1, data, size);
		vRingbufferReturnItem(s_int.rb, data);
	}
}


int LogUtil_init() {

	s_int.sem = xSemaphoreCreateBinary();
	if (!s_int.sem) {
		ESP_LOGE(TAG, "Can't create sem");
		return -ENOMEM;
	}

	s_int.rb = xRingbufferCreate(RB_SIZE, RINGBUF_TYPE_BYTEBUF);
	if (!s_int.rb) {
		vSemaphoreDelete(s_int.sem);
		s_int.sem = NULL;
		ESP_LOGE(TAG, "Can't alloc %d bytes", RINGBUF_TYPE_BYTEBUF);
		return -ENOMEM;
	}
	if (!xTaskCreate(log_writer, "logger", 2048, NULL, 12, NULL)) {
		ESP_LOGE(TAG, "Can't start thread");
		free(s_int.rb);
		s_int.rb = NULL;
		vSemaphoreDelete(s_int.sem);
		s_int.sem = NULL;
		return -ENOMEM;
	}

	s_int.org = esp_log_set_vprintf(my_print);
	return 0;
}

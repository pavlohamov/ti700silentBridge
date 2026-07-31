/*
 * sntp.c
 *
 *  Created on: 7 Apr 2025
 *      Author: pavloha
 */

#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_netif_sntp.h"
#include "lwip/ip_addr.h"
#include "esp_sntp.h"

static const char *TAG = "sntp";

#ifndef CONFIG_SNTP_TIME_SERVER
#define CONFIG_SNTP_TIME_SERVER "pool.ntp.org"
#endif

#ifndef INET6_ADDRSTRLEN
#define INET6_ADDRSTRLEN 48
#endif

static TaskHandle_t s_rqThread;

static time_t s_lastSyncAt = 0xFFFF;

static void time_sync_notification_cb(struct timeval *tv) {
    ESP_LOGI(TAG, "Notification of a time synchronization event %llu", tv->tv_sec);
}

static void print_servers(void) {
    ESP_LOGI(TAG, "List of configured NTP servers:");

    for (size_t i = 0; i < SNTP_MAX_SERVERS; ++i) {
        if (esp_sntp_getservername(i)){
            ESP_LOGI(TAG, "server %d: %s", i, esp_sntp_getservername(i));
            continue;
        }

        char buff[INET6_ADDRSTRLEN];
        ip_addr_t const *ip = esp_sntp_getserver(i);
        if (ipaddr_ntoa_r(ip, buff, INET6_ADDRSTRLEN))
            ESP_LOGI(TAG, "server %d: %s", i, buff);
    }
}


static void sntp_routine(void *arg) {

	const uint32_t tout_s = (uint32_t)arg;

#if LWIP_DHCP_GET_NTP_SRV
    ESP_LOGI(TAG, "Initializing SNTP");
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG(CONFIG_SNTP_TIME_SERVER);
    config.start = true;                       // start SNTP service explicitly (after connecting)
    config.server_from_dhcp = true;             // accept NTP offers from DHCP server, if any (need to enable *before* connecting)
    config.renew_servers_after_new_IP = true;   // let esp-netif update configured SNTP server(s) after receiving DHCP lease
    config.index_of_first_server = 0;           // updates from server num 1, leaving server 0 (from DHCP) intact
    // configure the event on which we renew servers
    config.ip_event_to_renew = IP_EVENT_STA_GOT_IP;
    config.sync_cb = time_sync_notification_cb; // only if we need the notification function
    esp_netif_sntp_init(&config);
#endif


#if LWIP_DHCP_GET_NTP_SRV
    ESP_LOGI(TAG, "Starting SNTP");
    esp_netif_sntp_start();
#if LWIP_IPV6 && SNTP_MAX_SERVERS > 2
    ip_addr_t ip6;
    if (ipaddr_aton("2a01:3f7::1", &ip6)) {    // ipv6 ntp source "ntp.netnod.se"
        esp_sntp_setserver(2, &ip6);
    }
#endif

#else
    ESP_LOGI(TAG, "Initializing and starting SNTP");
#if CONFIG_LWIP_SNTP_MAX_SERVERS > 1
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG_MULTIPLE(2,
                               ESP_SNTP_SERVER_LIST(CONFIG_SNTP_TIME_SERVER, "pool.ntp.org" ) );
#else
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG(CONFIG_SNTP_TIME_SERVER);
#endif
    config.sync_cb = time_sync_notification_cb;     // Note: This is only needed if we want
    esp_netif_sntp_init(&config);
#endif

    print_servers();

    const int64_t end = esp_timer_get_time() + tout_s * 1000ULL * 1000ULL;
    unsigned retry = 0;
    while (esp_netif_sntp_sync_wait(pdMS_TO_TICKS(2000)) == ESP_ERR_TIMEOUT) {
        ESP_LOGI(TAG, "Waiting for system time to be set... %u", ++retry);
        if (tout_s && end < esp_timer_get_time()) {
			ESP_LOGE(TAG, "timeout");
			break;
        }
    }
    time_t now = 0;
    time(&now);
    s_lastSyncAt = now;
    struct tm timeinfo = { 0 };
    localtime_r(&now, &timeinfo);

    char timeText[256];
    strftime(timeText, sizeof(timeText), "%a, %d %b %Y %T %Z%z", &timeinfo);
    ESP_LOGI(TAG, "%s", timeText);

    if (s_rqThread)
    	vTaskSuspend(s_rqThread);
    s_rqThread = NULL;
}


int SNTP_request(void) {

	if (s_rqThread) {
		ESP_LOGI(TAG, "Already running");
		return 0;
	}
	int rv = xTaskCreate(sntp_routine, TAG, 3 * 1024, NULL, 12, &s_rqThread);
	if (!rv) {
		ESP_LOGE(TAG, "Can't create thread");
	}
	return 0;
}

int SNTP_request_sync(uint32_t tout_s) {

	if (s_rqThread) {
		ESP_LOGI(TAG, "Already running");
		return 0;
	}
	sntp_routine((void*)tout_s);
	return 0;
}

time_t SNTP_last(void) {
	return s_lastSyncAt;
}


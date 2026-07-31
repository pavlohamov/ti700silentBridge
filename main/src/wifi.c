/*
 * wifi.c
 *
 *  Created on: 17 May 2023
 *      Author: Pavlo
 */


#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdarg.h>

#include "glue.h"

#include "nvs_flash.h"

#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "esp_event.h"
#include "esp_crc.h"
#include "esp_random.h"
#include "esp_timer.h"


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/ringbuf.h"
#include "freertos/semphr.h"

#include "esp_log.h"
static const char *TAG = "mWiFi";

//#include "oled_lcd.h"

typedef struct {
	const char *ssid;
	const char *psk;
} WiFiCred_t;

static const WiFiCred_t s_creds[] = {
//		{ "SiGaN-Guest", "Mission99A" },
//		{ "SiGaN", "SiGaN-Mission99-2021" },
};

static int getConnection(wifi_ap_record_t *ap) {
	for (size_t i = 0; i < sizeof(s_creds) / sizeof(*s_creds); ++i) {
		if (!strcmp((char*)ap->ssid, s_creds[i].ssid))
			return i;
	}
	return -1;
}

static int s_ipDone;

static void onEventWifi(void* event_handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {

	switch (event_id) {
		case WIFI_EVENT_WIFI_READY: {
			ESP_LOGD(TAG, "WiFi Ready");
//			//oled_lcd_set_header("WiFi Ready");
		} break;
		case WIFI_EVENT_SCAN_DONE: {
		    uint16_t count = 0;
		    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&count));
			ESP_LOGI(TAG, "Scan Done. Found %d", count);
			int tryConnect = -1;
			if (count) {
	//			//oled_lcd_set_header("WiFi STA Scan Done");
				wifi_ap_record_t *aps = malloc(count * sizeof(wifi_ap_record_t));
				if (esp_wifi_scan_get_ap_records(&count, aps)) {
					ESP_LOGI(TAG, "failed to get scan results");
					esp_wifi_clear_ap_list();
					free(aps);
					break;
				}

				for (uint16_t i = 0; i < count; ++i) {
					wifi_ap_record_t *ap = aps + i;
					ESP_LOGI(TAG, "%d/%d '%.32s' %d %ddbm %.2s %d%d", i + 1, count, ap->ssid, ap->authmode, ap->rssi, ap->country.cc, ap->ftm_initiator, ap->ftm_responder);
					if (tryConnect == -1)
						tryConnect = getConnection(ap);
				}
				bt_wifiScanResult(count, aps);
				free(aps);
			}

			if (tryConnect != -1) {
				ESP_LOGI(TAG, "Connecting to %.64s", s_creds[tryConnect].ssid);
				//oled_lcd_set_header(buff);

			    wifi_config_t cfg;
			    int rv = esp_wifi_get_config(WIFI_IF_STA, &cfg);
			    if (rv) {
			    	ESP_LOGE(TAG, "Can't get cfg. %X", rv);
			    	break;
			    }
			    memcpy(cfg.sta.ssid, s_creds[tryConnect].ssid, strlen(s_creds[tryConnect].ssid));
			    memcpy(cfg.sta.password, s_creds[tryConnect].psk, strlen(s_creds[tryConnect].psk));
			    esp_wifi_clear_fast_connect();
			    rv = esp_wifi_set_config(WIFI_IF_STA, &cfg);
			    if (rv) {
			    	ESP_LOGE(TAG, "Can't set cfg. %X", rv);
					break;
				}
			    rv = esp_wifi_connect();
			    if (rv) {
					ESP_LOGE(TAG, "Can't connect cfg. %X", rv);
			    }
				break;
			}
		} break;

		case WIFI_EVENT_STA_START: {
			ESP_LOGI(TAG, "WiFi STA Ready. Scanning");
			//oled_lcd_set_header("WiFi STA Ready. Scanning");
		    esp_wifi_scan_start(NULL, false);
		} break;
		case WIFI_EVENT_STA_STOP: {
			ESP_LOGI(TAG, "Station stop");
		} break;
		case WIFI_EVENT_STA_CONNECTED: {
	        wifi_event_sta_connected_t *event = (wifi_event_sta_connected_t *)event_data;
			ESP_LOGI(TAG, "Station connected to AP");
			//oled_lcd_set_header("WiFi STA Connected");
		    wifi_ftm_initiator_cfg_t ftmi_cfg = {
		        .frm_count = 32,
		        .burst_period = 2,
		    };
	        memcpy(ftmi_cfg.resp_mac, event->bssid, sizeof(ftmi_cfg.resp_mac));
	        ftmi_cfg.channel = event->channel;
	        esp_wifi_ftm_initiate_session(&ftmi_cfg);
		} break;
		case WIFI_EVENT_STA_DISCONNECTED: {
			wifi_event_sta_disconnected_t *event = (wifi_event_sta_disconnected_t*)event_data;
			//oled_lcd_set_header("WiFi STA Disconnected. Starting AP");
			ESP_LOGI(TAG, "Station disconnected from AP %d", event->reason);
			esp_wifi_scan_start(NULL, false);
			s_ipDone = 0;
		} break;
		case WIFI_EVENT_STA_AUTHMODE_CHANGE: {
			ESP_LOGI(TAG, "the auth mode of AP connected by device's station changed");
		} break;
		case WIFI_EVENT_AP_START: {
			ESP_LOGI(TAG, "Soft-AP start");
		} break;
		case WIFI_EVENT_AP_STOP: {
			ESP_LOGI(TAG, "Soft-AP stop");
		} break;
		case WIFI_EVENT_AP_STACONNECTED: {
			ESP_LOGI(TAG, "a station connected to Soft-AP");
			//oled_lcd_set_wifi("WiFi AP: Clien Connected");
		} break;
		case WIFI_EVENT_AP_STADISCONNECTED: {
			ESP_LOGI(TAG, "a station disconnected from Soft-AP");
			//oled_lcd_set_wifi("");
		} break;
		case WIFI_EVENT_AP_PROBEREQRECVED: {
			ESP_LOGI(TAG, "Receive probe request packet in soft-AP interface");
		} break;
		case WIFI_EVENT_FTM_REPORT: {
	        wifi_event_ftm_report_t *event = (wifi_event_ftm_report_t *)event_data;
			ESP_LOGI(TAG, "FTM: %" PRId32 " nSec, est Distance - %" PRId32 ".%02" PRId32 " m", event->rtt_est, event->dist_est / 100, event->dist_est % 100);
		} break;
		default: {
			ESP_LOGD(TAG, "event %ld", event_id);
		} break;
	}
}

static void onEventIp(void* event_handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
	switch (event_id) {
		case IP_EVENT_STA_GOT_IP: {
			ip_event_got_ip_t *event = (ip_event_got_ip_t*)event_data;
			char buff[32];
			snprintf(buff, sizeof(buff), "STA " IPSTR, IP2STR(&event->ip_info.ip));
			ESP_LOGI(TAG, "Got ip %s", buff);
			s_ipDone = 1;
			//oled_lcd_set_wifi(buff);
		} break;
		case IP_EVENT_ASSIGNED_IP_TO_CLIENT: {
			ip_event_assigned_ip_to_client_t *event = (ip_event_assigned_ip_to_client_t*)event_data;
			char buff[32];
			snprintf(buff, sizeof(buff), "AP " IPSTR, IP2STR(&event->ip));
			ESP_LOGI(TAG, "Got ip %s", buff);
			//oled_lcd_set_wifi(buff);
		} break;
		case IP_EVENT_GOT_IP6:
		case IP_EVENT_ETH_GOT_IP:
		case IP_EVENT_PPP_GOT_IP:{
			ESP_LOGI(TAG, "Got ip");
		} break;
		case IP_EVENT_STA_LOST_IP:
		case IP_EVENT_ETH_LOST_IP:
		case IP_EVENT_PPP_LOST_IP: {
			//oled_lcd_set_wifi("");
			ESP_LOGI(TAG, "lost ip");
			s_ipDone = 0;
		} break;
	}
}


int WiFi_init(void) {

	static bool done;
	if (done)
		return 0;

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_netif_t *netif_sta = esp_netif_create_default_wifi_sta();
    esp_netif_t *netif_ap = esp_netif_create_default_wifi_ap();
    assert(netif_sta);
    assert(netif_ap);

    esp_netif_set_hostname(netif_sta, "my name");
    esp_netif_set_hostname(netif_ap, "my name");

    esp_netif_attach_wifi_station(netif_sta);
    esp_netif_attach_wifi_ap(netif_ap);

//    esp_set_default_wifi_handers();


    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, onEventWifi, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, onEventIp, NULL));

    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_connect();

	esp_wifi_set_ps(WIFI_PS_NONE); // disable power save - speedup wifi
    done = 1;
    return 0;
}

int WiFi_connected(void) {
	return s_ipDone;
}

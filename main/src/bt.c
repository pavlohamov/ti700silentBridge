/*
 * bt.c
 *
 *  Created on: Aug 12, 2025
 *      Author: pavlo
 */

#include "esp_wifi.h"
#include "glue.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_ble_api.h"

#include "esp_blufi.h"
#include "esp_blufi_api.h"

#include "esp_efuse.h"
#include "esp_efuse_table.h"

#include "blufi_example.h"

#include "esp_log.h"
static const char *TAG = "mBT";

esp_err_t esp_blufi_host_init(void) {
	uint8_t seed[6] = {};
	ESP_ERROR_CHECK(esp_efuse_read_field_blob(ESP_EFUSE_MAC_FACTORY, seed, sizeof(seed) * 8));
	char name[128];
	int occ = snprintf(name, sizeof(name), "ti707-");
	for (int i = 0; i < sizeof(seed); ++i) {
		occ += snprintf(name + occ, sizeof(name) - occ, "%2X", seed[i]);
	}

	ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_ble_gap_set_device_name(name));
	return ESP_OK;
}

esp_err_t esp_blufi_host_deinit(void) {
	ESP_ERROR_CHECK(esp_blufi_profile_deinit());
    ESP_ERROR_CHECK(esp_bluedroid_disable());
    ESP_ERROR_CHECK(esp_bluedroid_deinit());
	return ESP_OK;
}

esp_err_t esp_blufi_host_and_cb_init(esp_blufi_callbacks_t *example_callbacks) {
    esp_blufi_host_init();
    ESP_ERROR_CHECK(esp_blufi_register_callbacks(example_callbacks));
	ESP_ERROR_CHECK(esp_ble_gap_register_callback(esp_blufi_gap_event_handler));
	ESP_ERROR_CHECK(esp_blufi_profile_init());
    return ESP_OK;
}

static void example_event_callback(esp_blufi_cb_event_t event, esp_blufi_cb_param_t *param);

int bt_init(void) {
    static int inited;
	if (inited)
		return 0;
	inited = 1;

#if CONFIG_IDF_TARGET_ESP32
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
#endif
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

	static esp_blufi_callbacks_t example_callbacks = {
	    .event_cb = example_event_callback,
	    .negotiate_data_handler = blufi_dh_negotiate_data_handler,
	    .encrypt_func = blufi_aes_encrypt,
	    .decrypt_func = blufi_aes_decrypt,
	    .checksum_func = blufi_crc_checksum,
	};

	ESP_ERROR_CHECK(esp_blufi_host_and_cb_init(&example_callbacks));

    ESP_LOGI(TAG, "BT version %X", esp_blufi_get_version());
	return 0;
}

esp_err_t esp_blufi_controller_deinit() {
	ESP_ERROR_CHECK(esp_bt_controller_disable());
	ESP_ERROR_CHECK(esp_bt_controller_deinit());
	return ESP_OK;
}
static bool ble_is_connected;
static bool ble_scan_requested;

int bt_wifiScanResult(size_t count, void *arg) {

	if (!ble_is_connected || !ble_scan_requested)
		return 0;
	ble_scan_requested = false;

	esp_blufi_ap_record_t *bleps = (esp_blufi_ap_record_t*)malloc(count * sizeof(esp_blufi_ap_record_t));
	if (!bleps)
		return 0;

	wifi_ap_record_t *aps = (wifi_ap_record_t*)arg;
	for (size_t i = 0; i < count; ++i) {
		bleps[i].rssi = aps[i].rssi;
		memcpy(bleps[i].ssid, aps[i].ssid, sizeof(aps[i].ssid));
	}

    esp_blufi_send_wifi_list(count, bleps);
    free(bleps);
    return 0;
}

int bt_connected(void) {
	return ble_is_connected;
}

static void example_event_callback(esp_blufi_cb_event_t event, esp_blufi_cb_param_t *param) {

	static wifi_config_t sta_config;
    switch (event) {
    case ESP_BLUFI_EVENT_INIT_FINISH:
        ESP_LOGI(TAG, "BLUFI init finish");
        esp_blufi_adv_start();
        break;
    case ESP_BLUFI_EVENT_DEINIT_FINISH:
        ESP_LOGI(TAG, "BLUFI deinit finish");
        break;
    case ESP_BLUFI_EVENT_BLE_CONNECT:
        ESP_LOGI(TAG, "BLUFI ble connect");
        ble_is_connected = true;
        esp_blufi_adv_stop();
        blufi_security_init();
        break;
    case ESP_BLUFI_EVENT_BLE_DISCONNECT:
        ESP_LOGI(TAG, "BLUFI ble disconnect");
        ble_is_connected = false;
        blufi_security_deinit();
        esp_blufi_adv_start();
        break;
    case ESP_BLUFI_EVENT_SET_WIFI_OPMODE:
        ESP_LOGI(TAG, "BLUFI Set WIFI opmode %d", param->wifi_mode.op_mode);
//        ESP_ERROR_CHECK(esp_wifi_set_mode(param->wifi_mode.op_mode));
        break;
    case ESP_BLUFI_EVENT_REQ_CONNECT_TO_AP: {
        ESP_LOGI(TAG, "BLUFI request wifi connect to AP");
        esp_wifi_disconnect();
        esp_wifi_clear_fast_connect();
        ESP_LOGW(TAG, "Connect '%s'/'%s'", sta_config.sta.ssid, sta_config.sta.password);
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
        ESP_ERROR_CHECK(esp_wifi_connect());
    } break;
    case ESP_BLUFI_EVENT_REQ_DISCONNECT_FROM_AP:
        ESP_LOGI(TAG, "BLUFI request wifi disconnect from AP");
        esp_wifi_disconnect();
        break;
    case ESP_BLUFI_EVENT_REPORT_ERROR:
        BLUFI_ERROR("BLUFI report error, error code %d", param->report_error.state);
        esp_blufi_send_error_info(param->report_error.state);
        break;
    case ESP_BLUFI_EVENT_GET_WIFI_STATUS: {

    	wifi_sta_list_t list = {};
    	esp_wifi_ap_get_sta_list(&list);

	    wifi_config_t cfg;
	    ESP_ERROR_CHECK(esp_wifi_get_config(WIFI_IF_STA, &cfg));

        esp_blufi_extra_info_t info = {};
        memcpy(info.sta_bssid, cfg.sta.bssid, 6);
        info.sta_bssid_set = true;
        info.sta_ssid = cfg.sta.ssid;
        info.sta_ssid_len = strlen((char*)cfg.sta.ssid);

        int rssi = 0;
        info.sta_conn_rssi_set = !esp_wifi_sta_get_rssi(&rssi);
        info.sta_conn_rssi = rssi;

        wifi_mode_t mode;
        esp_wifi_get_mode(&mode);
        if (WiFi_connected()) {
//gl_sta_got_ip ? ESP_BLUFI_STA_CONN_SUCCESS : ESP_BLUFI_STA_NO_IP
            esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_SUCCESS, list.num, &info);
        } /*else if (gl_sta_is_connecting) {
            esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONNECTING, list.num, &info);
        } */else {
            esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_FAIL, list.num, &info);
        }
        ESP_LOGI(TAG, "BLUFI get wifi status from AP");

        break;
    }
    case ESP_BLUFI_EVENT_RECV_SLAVE_DISCONNECT_BLE:
        ESP_LOGI(TAG, "blufi close a gatt connection");
        esp_blufi_disconnect();
        break;
    case ESP_BLUFI_EVENT_DEAUTHENTICATE_STA:
        /* TODO */
        break;
	case ESP_BLUFI_EVENT_RECV_STA_BSSID: {
        memcpy(sta_config.sta.bssid, param->sta_bssid.bssid, 6);
        sta_config.sta.bssid_set = 1;
	} break;
	case ESP_BLUFI_EVENT_RECV_STA_SSID:
        if (param->sta_ssid.ssid_len >= sizeof(sta_config.sta.ssid)/sizeof(sta_config.sta.ssid[0])) {
            esp_blufi_send_error_info(ESP_BLUFI_DATA_FORMAT_ERROR);
            ESP_LOGI(TAG, "Invalid STA SSID %d", param->sta_ssid.ssid_len);
            break;
        }
        strncpy((char *)sta_config.sta.ssid, (char *)param->sta_ssid.ssid, param->sta_ssid.ssid_len);
        sta_config.sta.ssid[param->sta_ssid.ssid_len] = '\0';
        break;
	case ESP_BLUFI_EVENT_RECV_STA_PASSWD:
        if (param->sta_passwd.passwd_len >= sizeof(sta_config.sta.password)/sizeof(sta_config.sta.password[0])) {
            esp_blufi_send_error_info(ESP_BLUFI_DATA_FORMAT_ERROR);
            ESP_LOGI(TAG, "Invalid STA PASSWORD %d", param->sta_passwd.passwd_len);
            break;
        }
        strncpy((char *)sta_config.sta.password, (char *)param->sta_passwd.passwd, param->sta_passwd.passwd_len);
        sta_config.sta.password[param->sta_passwd.passwd_len] = '\0';
//        sta_config.sta.threshold.authmode = EXAMPLE_WIFI_SCAN_AUTH_MODE_THRESHOLD;
        break;
	case ESP_BLUFI_EVENT_RECV_SOFTAP_SSID:
        ESP_LOGE(TAG, "SOFTAP SSID");
        break;
	case ESP_BLUFI_EVENT_RECV_SOFTAP_PASSWD:
        ESP_LOGE(TAG, "SOFTAP pass");
        break;
	case ESP_BLUFI_EVENT_RECV_SOFTAP_MAX_CONN_NUM:
        ESP_LOGE(TAG, "SOFTAP max conn");
        break;
	case ESP_BLUFI_EVENT_RECV_SOFTAP_AUTH_MODE:
        ESP_LOGE(TAG, "SOFTAP auth");
        break;
	case ESP_BLUFI_EVENT_RECV_SOFTAP_CHANNEL:
        ESP_LOGE(TAG, "SOFTAP ch");
        break;
    case ESP_BLUFI_EVENT_GET_WIFI_LIST:{
        ESP_LOGI(TAG, "SCAN wifi");
    	ble_scan_requested = true;
    	esp_wifi_scan_start(NULL, false);
//        esp_wifi_scan_stop();
//        int rv = esp_wifi_scan_start(NULL, false);
//        if (rv) {
//            ESP_LOGI(TAG, "SCAN fail %X", rv);
//            esp_blufi_send_error_info(ESP_BLUFI_WIFI_SCAN_FAIL);
//        } else {
//        }
        break;
    }
    case ESP_BLUFI_EVENT_RECV_CUSTOM_DATA:
        ESP_LOGI(TAG, "Recv Custom Data %" PRIu32 "", param->custom_data.data_len);
        ESP_LOG_BUFFER_HEX("Custom Data", param->custom_data.data, param->custom_data.data_len);
        break;
	case ESP_BLUFI_EVENT_RECV_USERNAME:
        /* Not handle currently */
        break;
	case ESP_BLUFI_EVENT_RECV_CA_CERT:
        /* Not handle currently */
        break;
	case ESP_BLUFI_EVENT_RECV_CLIENT_CERT:
        /* Not handle currently */
        break;
	case ESP_BLUFI_EVENT_RECV_SERVER_CERT:
        /* Not handle currently */
        break;
	case ESP_BLUFI_EVENT_RECV_CLIENT_PRIV_KEY:
        /* Not handle currently */
        break;;
	case ESP_BLUFI_EVENT_RECV_SERVER_PRIV_KEY:
        /* Not handle currently */
        break;
    default:
        break;
    }
}

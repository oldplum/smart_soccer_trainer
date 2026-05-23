/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "wifi_client_connector.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "lwip/inet.h"
#include "wifi_data_sender.h"

#define TAG "wifi_client"

/* WiFi event group bits */
#define WIFI_CONNECTED_BIT    BIT0
#define WIFI_DISCONNECTED_BIT BIT1

static EventGroupHandle_t s_wifi_event_group = NULL;
static bool s_wifi_initialized = false;

/**
 * @brief WiFi event handler
 */
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        xEventGroupSetBits(s_wifi_event_group, WIFI_DISCONNECTED_BIT);
        ESP_LOGI(TAG, "WiFi disconnected, attempting reconnect");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "WiFi connected with IP: " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "WiFi gateway: " IPSTR, IP2STR(&event->ip_info.gw));
        wifi_data_sender_update_destination_ip(event->ip_info.gw.addr);
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        xEventGroupClearBits(s_wifi_event_group, WIFI_DISCONNECTED_BIT);
    }
}

/**
 * @brief Initialize WiFi and connect to station
 */
bool wifi_client_init(const char *ssid, const char *password)
{
    if (s_wifi_initialized) {
        ESP_LOGW(TAG, "WiFi already initialized");
        return true;
    }

    if (!ssid || !password) {
        ESP_LOGE(TAG, "SSID and password cannot be NULL");
        return false;
    }

    /* Create event group */
    s_wifi_event_group = xEventGroupCreate();
    if (!s_wifi_event_group) {
        ESP_LOGE(TAG, "Failed to create WiFi event group");
        return false;
    }

    /* Initialize NVS */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS flash init failed: %s", esp_err_to_name(ret));
        return false;
    }

    /* Initialize WiFi stack */
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    /* Register event handlers */
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    /* Configure WiFi */
    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,  /* Allow common phone hotspots */
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
            .sae_h2e_identifier = "",
            .pmf_cfg = {
                .capable = true,
                .required = false,
            },
        },
    };

    /* Copy SSID and password */
    strlcpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    strlcpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    
    /* Set WiFi power save mode to none for stability */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    
    ESP_ERROR_CHECK(esp_wifi_start());

    s_wifi_initialized = true;
    ESP_LOGI(TAG, "WiFi client initialized, connecting to %s", ssid);

    /* Wait for connection (with timeout) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           pdMS_TO_TICKS(10000));

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "WiFi connection established");
        return true;
    } else {
        ESP_LOGW(TAG, "WiFi connection timeout, will keep trying");
        return true;  /* Return true anyway, it will reconnect automatically */
    }
}

/**
 * @brief Get WiFi connection status
 */
bool wifi_client_is_connected(void)
{
    if (!s_wifi_initialized || !s_wifi_event_group) {
        return false;
    }

    EventBits_t bits = xEventGroupGetBits(s_wifi_event_group);
    return (bits & WIFI_CONNECTED_BIT) != 0;
}

/**
 * @brief Get assigned IP address
 */
bool wifi_client_get_ip(char *ip_str, size_t len)
{
    if (!ip_str || len < 16) {
        ESP_LOGE(TAG, "Invalid buffer for IP string");
        return false;
    }

    if (!wifi_client_is_connected()) {
        return false;
    }

    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (!netif) {
        return false;
    }

    esp_netif_ip_info_t ip_info;
    if (esp_netif_get_ip_info(netif, &ip_info) != ESP_OK) {
        return false;
    }

    ip4addr_ntoa_r((const ip4_addr_t *)&ip_info.ip, ip_str, len);
    return true;
}

/**
 * @brief Deinitialize WiFi
 */
void wifi_client_deinit(void)
{
    if (!s_wifi_initialized) {
        return;
    }

    ESP_ERROR_CHECK(esp_wifi_stop());
    ESP_ERROR_CHECK(esp_wifi_deinit());

    if (s_wifi_event_group) {
        vEventGroupDelete(s_wifi_event_group);
        s_wifi_event_group = NULL;
    }

    s_wifi_initialized = false;
    ESP_LOGI(TAG, "WiFi deinitialized");
}

/*
 * SPDX-FileCopyrightText: 2023-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "brookesia/system_phone/phone.hpp"
#include "esp_brookesia_app_temperature.hpp"
#include "esp_brookesia_app_compass.hpp"
#include "boost/thread.hpp"
#include <cstdio>
#include <cstring>
#include <new>
#include <vector>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#ifdef ESP_UTILS_LOG_TAG
#   undef ESP_UTILS_LOG_TAG
#endif
#define ESP_UTILS_LOG_TAG "Main"
#include "esp_lib_utils.h"
#include "esp_board_manager.h"
#include "soccer_data_sync.h"
#include "bmi270_collector.h"
#include "wifi_client_connector.h"
#include "wifi_data_sender.h"

using namespace esp_brookesia;
using namespace esp_brookesia::systems::phone;

/* Global sensor data buffer - updated by sensor threads */
SoccerSensorData g_soccer_sensor_data = {};

/* WiFi configuration */
static const char *WIFI_SSID = "oldplum's HONOR 400";
static const char *WIFI_PASSWORD = "liyuhan061218";
static const char *DATA_SERVER_IP = "10.132.164.189";  /* Hotspot/receiver IP address */
static const uint16_t DATA_SERVER_PORT = 5000;

/* Debug serial: environmental data only (motion CSV is printed by bmi270_collector) */
static const int DEBUG_SERIAL_PERIOD_ENV_MS = 3000;
static uint32_t last_env_print_ms = 0;
static uint32_t last_printed_env_ts = UINT32_MAX;

static const size_t UDP_BATCH_MAX_BYTES = 500;
static const size_t UDP_BATCH_BUFFER_SIZE = 1024;
static const uint32_t UDP_BATCH_MAX_COUNT = 10;
static const int64_t UDP_BATCH_MAX_INTERVAL_US = 100000;

static char s_udp_buffer[UDP_BATCH_BUFFER_SIZE];
static size_t s_udp_len = 0;
static uint32_t s_udp_count = 0;
static int64_t s_udp_batch_start_us = 0;
static SemaphoreHandle_t s_udp_buffer_mutex = nullptr;

static void udp_reset_buffer_locked(void)
{
    s_udp_len = 0;
    s_udp_count = 0;
    s_udp_batch_start_us = 0;
}

static void udp_flush_buffer_locked(void)
{
    if (s_udp_len == 0) {
        return;
    }

    wifi_data_sender_send_csv_line(s_udp_buffer, s_udp_len);
    udp_reset_buffer_locked();
}

static void udp_flush_buffer(void)
{
    if (s_udp_buffer_mutex == nullptr) {
        return;
    }

    if (xSemaphoreTake(s_udp_buffer_mutex, pdMS_TO_TICKS(20)) != pdTRUE) {
        return;
    }

    udp_flush_buffer_locked();

    xSemaphoreGive(s_udp_buffer_mutex);
}

static void udp_batch_flush_task(void *arg)
{
    (void)arg;

    while (true) {
        if (s_udp_len > 0 && s_udp_batch_start_us != 0) {
            int64_t now_us = esp_timer_get_time();
            if ((now_us - s_udp_batch_start_us) >= UDP_BATCH_MAX_INTERVAL_US) {
                udp_flush_buffer();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static bool udp_csv_sink(const char *line, size_t len, void *user_data)
{
    (void)user_data;

    if (s_udp_buffer_mutex == nullptr) {
        return false;
    }

    if (!wifi_client_is_connected()) {
        if (xSemaphoreTake(s_udp_buffer_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            udp_reset_buffer_locked();
            xSemaphoreGive(s_udp_buffer_mutex);
        }
        return false;
    }

    if (line == nullptr || len == 0) {
        return true;
    }

    if (len > UDP_BATCH_BUFFER_SIZE) {
        return wifi_data_sender_send_csv_line(line, len);
    }

    if (xSemaphoreTake(s_udp_buffer_mutex, pdMS_TO_TICKS(20)) != pdTRUE) {
        return false;
    }

    int64_t now_us = esp_timer_get_time();
    if (s_udp_len == 0) {
        s_udp_batch_start_us = now_us;
    }

    if (s_udp_len + len > UDP_BATCH_BUFFER_SIZE) {
        udp_flush_buffer_locked();
        s_udp_batch_start_us = now_us;
    }

    memcpy(s_udp_buffer + s_udp_len, line, len);
    s_udp_len += len;
    s_udp_count++;

    if (s_udp_count >= UDP_BATCH_MAX_COUNT ||
        s_udp_len >= UDP_BATCH_MAX_BYTES ||
        (now_us - s_udp_batch_start_us) >= UDP_BATCH_MAX_INTERVAL_US) {
        udp_flush_buffer_locked();
    }

    xSemaphoreGive(s_udp_buffer_mutex);

    return true;
}

/**
 * @brief Debug serial output task
 *
 * Environmental sensor CSV only. Motion data @ 100 Hz is printed by bmi270_collector.
 */
void debug_serial_task(void *arg)
{
    ESP_UTILS_LOGI("Debug serial task started");

    uint32_t start_time_ms = (uint32_t)(esp_timer_get_time() / 1000);

    while (1) {
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000) - start_time_ms;
        auto temp_app = esp_brookesia::apps::Temperature::requestInstance();

        /* Environmental data every 3 seconds */
        if (temp_app && temp_app->isSensorOnline() &&
            (now_ms - last_env_print_ms) >= DEBUG_SERIAL_PERIOD_ENV_MS) {
            uint32_t curr_ts = g_soccer_sensor_data.timestamp;
            if (curr_ts != last_printed_env_ts) {
                printf("%lu,%.2f,%.2f,%.1f,%.1f\n",
                       g_soccer_sensor_data.timestamp,
                       g_soccer_sensor_data.temp,
                       g_soccer_sensor_data.humidity,
                       g_soccer_sensor_data.pressure,
                       g_soccer_sensor_data.iaq);
                last_printed_env_ts = curr_ts;
            }
            last_env_print_ms = now_ms;
        }

        vTaskDelay(pdMS_TO_TICKS(5));  // 5ms delay to prevent blocking
    }
}

extern "C" void app_main(void)
{
    ESP_UTILS_LOGI("Sensor data collection demo");
    esp_board_manager_print_board_info();
    int ret = esp_board_manager_init();
    assert((ret == 0) && "Board manager initialization failed");

    /* Initialize sensor data buffer */
    soccer_data_init(&g_soccer_sensor_data);

    /* 100 Hz GPTimer-driven BMI270 acquisition (no vTaskDelay sampling) */
    if (!bmi270_collector_start()) {
        ESP_LOGE("main", "BMI270 collector start failed");
    }

    /* Start debug serial task for environmental CSV output */
    {
        esp_utils::thread_config_guard debug_thread_config({
            .name = "debug_serial",
            .stack_size = 4096,
        });
        boost::thread(debug_serial_task, nullptr).detach();
    }

    /* Create a phone object */
    Phone *phone = new (std::nothrow) Phone();
    ESP_UTILS_CHECK_NULL_EXIT(phone, "Create phone failed");

    {
        /* Begin the phone */
        bool phone_begin_ok = phone->begin();
        if (!phone_begin_ok) {
            ESP_LOGW("main", "Phone begin failed, continuing anyway");
        }

        /* Init and install apps from registry */
        std::vector<systems::base::Manager::RegistryAppInfo> inited_apps;
        if (!phone->initAppFromRegistry(inited_apps)) {
            ESP_LOGE("main", "Init app registry failed");
        } else {
            std::vector<std::string> ordered_app_names = {"Environment", "Gesture Detect", "Compass"};
            bool install_ok = phone->installAppFromRegistry(inited_apps, &ordered_app_names);
            if (!install_ok) {
                ESP_LOGW("main", "Install app registry failed (display issue?), but continuing with direct sensor startup");
            } else {
                ESP_LOGI("main", "All apps installed successfully");
            }
        }
    }

    /* Always try to start sensor collection directly via app singletons,
       regardless of whether phone/display initialization succeeded */
    ESP_LOGI("main", "Starting sensor collection via direct app access");
    
    auto temp_app = esp_brookesia::apps::Temperature::requestInstance();
    if (temp_app) {
        ESP_LOGI("main", "Starting Temperature sensor collection");
        if (!temp_app->startSensorCollection()) {
            ESP_LOGW("main", "Failed to start Temperature sensor collection");
        }
    }
    
    auto compass_app = esp_brookesia::apps::Compass::requestInstance();
    if (compass_app) {
        ESP_LOGI("main", "Starting Compass sensor collection");
        if (!compass_app->startSensorCollection()) {
            ESP_LOGW("main", "Failed to start Compass sensor collection");
        }
    }

    /* Initialize WiFi client connection */
    ESP_LOGI("main", "Initializing WiFi client...");
    s_udp_buffer_mutex = xSemaphoreCreateMutex();
    if (s_udp_buffer_mutex == nullptr) {
        ESP_LOGE("main", "Failed to create UDP batch mutex");
    }

    if (s_udp_buffer_mutex != nullptr && wifi_data_sender_init(DATA_SERVER_IP, DATA_SERVER_PORT)) {
        bmi270_collector_set_csv_sink(udp_csv_sink, nullptr);
        ESP_LOGI("main", "UDP data sender initialized, target: %s:%d", DATA_SERVER_IP, DATA_SERVER_PORT);

        BaseType_t task_created = xTaskCreate(
            udp_batch_flush_task,
            "udp_batch_flush",
            3072,
            nullptr,
            5,
            nullptr
        );
        if (task_created != pdPASS) {
            ESP_LOGE("main", "Failed to create UDP batch flush task");
        }
    } else {
        ESP_LOGE("main", "Failed to initialize UDP data sender");
    }

    if (wifi_client_init(WIFI_SSID, WIFI_PASSWORD)) {
        vTaskDelay(pdMS_TO_TICKS(2000));  /* Wait for WiFi to stabilize */

        char ip_str[16] = {0};
        if (wifi_client_get_ip(ip_str, sizeof(ip_str))) {
            ESP_LOGI("main", "WiFi connected, IP: %s", ip_str);
        } else {
            ESP_LOGW("main", "Failed to get IP address");
        }
    } else {
        ESP_LOGW("main", "WiFi initialization failed");
    }

}

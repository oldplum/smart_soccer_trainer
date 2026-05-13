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
#include <new>
#include <vector>
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#ifdef ESP_UTILS_LOG_TAG
#   undef ESP_UTILS_LOG_TAG
#endif
#define ESP_UTILS_LOG_TAG "Main"
#include "esp_lib_utils.h"
#include "esp_board_manager.h"
#include "soccer_data_sync.h"

using namespace esp_brookesia;
using namespace esp_brookesia::systems::phone;

/* Global sensor data buffer - updated by sensor threads */
SoccerSensorData g_soccer_sensor_data = {};

/* Debug serial output parameters */
static const int DEBUG_SERIAL_PERIOD_MOTION_MS = 10;    // Motion data: 10ms
static const int DEBUG_SERIAL_PERIOD_ENV_MS = 3000;     // Environmental data: 3 seconds
static uint32_t last_motion_print_ms = 0;
static uint32_t last_env_print_ms = 0;
/* Last printed timestamps to avoid duplicate prints when data hasn't changed */
static uint32_t last_printed_motion_ts = UINT32_MAX;
static uint32_t last_printed_env_ts = UINT32_MAX;

/**
 * @brief Debug serial output task
 *
 * Periodically outputs sensor data in CSV format:
 * - Motion data (acc, gyro, pitch, roll, heading) at 10ms intervals
 * - Environmental data (temp, humidity, pressure, iaq) at 3s intervals
 */
void debug_serial_task(void *arg)
{
    ESP_UTILS_LOGI("Debug serial task started");

    uint32_t start_time_ms = (uint32_t)(esp_timer_get_time() / 1000);

    while (1) {
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000) - start_time_ms;
        auto temp_app = esp_brookesia::apps::Temperature::requestInstance();
        auto compass_app = esp_brookesia::apps::Compass::requestInstance();

        /* Output motion data every 10ms, but only if timestamp changed */
        if (compass_app && compass_app->isSensorOnline() &&
            (now_ms - last_motion_print_ms) >= DEBUG_SERIAL_PERIOD_MOTION_MS) {
            uint32_t curr_ts = g_soccer_sensor_data.timestamp;
            if (curr_ts != last_printed_motion_ts) {
                printf("%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f\n",
                       g_soccer_sensor_data.timestamp,
                       g_soccer_sensor_data.acc[0],
                       g_soccer_sensor_data.acc[1],
                       g_soccer_sensor_data.acc[2],
                       g_soccer_sensor_data.gyro[0],
                       g_soccer_sensor_data.gyro[1],
                       g_soccer_sensor_data.gyro[2],
                       g_soccer_sensor_data.pitch,
                       g_soccer_sensor_data.roll,
                       g_soccer_sensor_data.heading);
                last_printed_motion_ts = curr_ts;
            }
            last_motion_print_ms = now_ms;
        }

        /* Output environmental data every 3 seconds, but only if timestamp changed */
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

    /* Start debug serial task for sensor data output early so it runs
       even if phone/display initialization fails */
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

}

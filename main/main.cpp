/*
 * SPDX-FileCopyrightText: 2023-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <cstdio>
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#ifdef ESP_UTILS_LOG_TAG
#   undef ESP_UTILS_LOG_TAG
#endif
#define ESP_UTILS_LOG_TAG "Main"
#include "environment_sensor.h"
#include "soccer_data_sync.h"

/* Global sensor data buffer - updated by sensor threads */
SoccerSensorData g_soccer_sensor_data = {};

static constexpr TickType_t kJsonPrintPeriod = pdMS_TO_TICKS(3000);
static constexpr TickType_t kImuSamplePeriod = pdMS_TO_TICKS(20);
#define PRINT_RAW_IMU 0

static inline void refresh_timestamp(SoccerSensorData *data)
{
    data->timestamp = static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
}

/**
 * @brief Periodic reserved-data output task
 *
 * Every 3 seconds, print a single JSON line that packs the reserved
 * higher-level fields so the interface exists even before algorithms
 * and hardware integration are complete.
 */
static void reserved_data_json_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        refresh_timestamp(&g_soccer_sensor_data);
        const SoccerSensorData snapshot = g_soccer_sensor_data;
        char json[256];
        int len = std::snprintf(
            json,
            sizeof(json),
            "{\"timestamp\":%lu,\"shoot_count\":%lu,\"pass_count\":%lu,\"heart_rate\":%u,\"temp\":%.2f,\"humidity\":%.2f,\"pressure\":%.2f,\"iaq\":%.2f,\"uwb_d0\":%.2f,\"uwb_d1\":%.2f,\"uwb_d2\":%.2f,\"uwb_d3\":%.2f}\n",
            static_cast<unsigned long>(snapshot.timestamp),
            static_cast<unsigned long>(snapshot.shoot_count),
            static_cast<unsigned long>(snapshot.pass_count),
            static_cast<unsigned int>(snapshot.heart_rate),
            snapshot.temp,
            snapshot.humidity,
            snapshot.pressure,
            snapshot.iaq,
            snapshot.uwb_d0,
            snapshot.uwb_d1,
            snapshot.uwb_d2,
            snapshot.uwb_d3);

        if (len > 0 && len < static_cast<int>(sizeof(json))) {
            printf("%s", json);
        }

        vTaskDelayUntil(&last_wake, kJsonPrintPeriod);
    }
}

/**
 * @brief Raw IMU sampling task
 *
 * Samples at 50Hz. CSV output is disabled by default through PRINT_RAW_IMU.
 * The task still runs in the background so the sampling hook is reserved.
 */
static void raw_imu_sampling_task(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
#if PRINT_RAW_IMU
        refresh_timestamp(&g_soccer_sensor_data);
        const SoccerSensorData snapshot = g_soccer_sensor_data;
        printf("%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
               static_cast<unsigned long>(snapshot.timestamp),
               snapshot.acc[0],
               snapshot.acc[1],
               snapshot.acc[2],
               snapshot.gyro[0],
               snapshot.gyro[1],
               snapshot.gyro[2]);
#endif
        vTaskDelayUntil(&last_wake, kImuSamplePeriod);
    }
}

extern "C" void app_main(void)
{
    ESP_LOGI("Main", "Sensor data collection demo");

    /* Initialize sensor data buffer */
    soccer_data_init(&g_soccer_sensor_data);

    if (!start_environment_sensor_task()) {
        ESP_LOGW("Main", "Environment sensor task is not running; defaults will remain until a sensor comes online");
    }

    /* Start reserved-data JSON task and silent IMU sampling task. */
    if (xTaskCreate(reserved_data_json_task,
                    "reserved_json",
                    4096,
                    nullptr,
                    tskIDLE_PRIORITY + 1,
                    nullptr) != pdPASS) {
        ESP_LOGE("Main", "Failed to create reserved data JSON task");
    }

    if (xTaskCreate(raw_imu_sampling_task,
                    "raw_imu",
                    4096,
                    nullptr,
                    tskIDLE_PRIORITY + 1,
                    nullptr) != pdPASS) {
        ESP_LOGE("Main", "Failed to create raw IMU sampling task");
    }
}

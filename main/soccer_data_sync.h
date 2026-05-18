/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SOCCER_DATA_SYNC_H
#define SOCCER_DATA_SYNC_H

#include <stdint.h>
#include <cstring>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Unified sensor data structure for soccer training
 *
 * This structure aggregates sensor data from multiple sources:
 * - Environmental sensors (BME690): temperature, humidity, pressure, IAQ
 * - Motion sensors (BMI270): accelerometer, gyroscope
 * - Orientation sensors (BMI270 + BMM350): pitch, roll, heading
 *
 * The structure is marked with __attribute__((packed)) for efficient
 * binary serialization and ESP-NOW wireless transmission.
 */
typedef struct {
    /* Environmental Data (from BME690 + BSEC) */
    float temp;           /**< Temperature in °C */
    float humidity;       /**< Relative humidity in % RH */
    float pressure;       /**< Atmospheric pressure in Pa */
    float iaq;            /**< Indoor Air Quality index (0-500+) */

    /* Motion Data (from BMI270) */
    float acc[3];         /**< Accelerometer data [X, Y, Z] in g */
    float gyro[3];        /**< Gyroscope data [X, Y, Z] in °/s */

    /* Orientation Data (from complementary filter + magnetometer) */
    float pitch;          /**< Pitch angle in degrees */
    float roll;           /**< Roll angle in degrees */
    float heading;        /**< Compass heading in degrees (0-360) */

    /* System Data */
    uint32_t timestamp;   /**< Timestamp in milliseconds since boot */

    /* Placeholder / Reserved Data (reserved for higher-level features)
     * - Action counters (to be computed by shot/pass detection model)
     * - Vitals (heart rate)
     * - UWB distance channels
     * These are populated with fixed default values so the rest of the
     * system can read a stable interface while algorithms / hardware
     * are still under development.
     */
    uint32_t shoot_count; /**< Placeholder: number of detected shots (default 0) */
    uint32_t pass_count;  /**< Placeholder: number of detected passes (default 0) */

    /* Heart rate (beats per minute) */
    uint16_t heart_rate;  /**< Placeholder heart rate (default 80 bpm) */

    /* UWB distance channels (meters) */
    float uwb_d0;
    float uwb_d1;
    float uwb_d2;
    float uwb_d3;
} __attribute__((packed)) SoccerSensorData;

/**
 * @brief Global sensor data instance
 *
 * This is the unified data buffer that is continuously updated by
 * various sensor reading threads:
 * - bmeDataThread (Temperature app) - updates environmental data
 * - updateSensorData (Compass app) - updates motion and orientation data
 *
 * Access to this buffer should be thread-safe (use mutex if needed).
 */
extern SoccerSensorData g_soccer_sensor_data;

/**
 * @brief Initialize the sensor data structure
 *
 * Zeros out all fields and sets initial values.
 */
static inline void soccer_data_init(SoccerSensorData *data)
{
    std::memset(data, 0, sizeof(SoccerSensorData));

    /* Set sensible defaults for reserved placeholder fields */
    data->shoot_count = 0;
    data->pass_count = 0;
    data->heart_rate = 80;
    data->uwb_d0 = 10.0f;
    data->uwb_d1 = 20.0f;
    data->uwb_d2 = 30.0f;
    data->uwb_d3 = 40.0f;
}

#ifdef __cplusplus
}
#endif

#endif // SOCCER_DATA_SYNC_H

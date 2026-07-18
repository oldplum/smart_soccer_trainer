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
 * - GNSS positioning (H802 GPS+Beidou): latitude, longitude, altitude,
 *   speed, course, satellites, fix quality
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

    /* GNSS Positioning Data (from H802 GPS+Beidou) */
    double latitude;      /**< Latitude in decimal degrees (+North, -South) */
    double longitude;     /**< Longitude in decimal degrees (+East, -West) */
    float altitude;       /**< Altitude above MSL in meters */
    float speed;          /**< Ground speed in km/h */
    float course;         /**< Course over ground in degrees (0-360) */
    uint8_t satellites;   /**< Number of tracked satellites */
    uint8_t fix_quality;  /**< Fix quality: 0=invalid, 1=GPS, 2=DGPS, 4=RTK fixed, 5=RTK float */
    uint8_t gnss_valid;   /**< GNSS data validity flag (boolean) */

    /* System Data */
    uint32_t timestamp;   /**< Timestamp in milliseconds since boot */
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
}

#ifdef __cplusplus
}
#endif

#endif // SOCCER_DATA_SYNC_H

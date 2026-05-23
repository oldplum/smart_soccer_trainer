/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef WIFI_DATA_SENDER_H
#define WIFI_DATA_SENDER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize WiFi data sender
 * @param server_ip Target PC IP address
 * @param server_port Target PC port
 * @return true if sender initialized, false otherwise
 */
bool wifi_data_sender_init(const char *server_ip, uint16_t server_port);

/**
 * @brief Send sensor data packet to server
 * @param timestamp Sensor timestamp (ms)
 * @param acc Accelerometer data [x, y, z] in g
 * @param gyro Gyroscope data [x, y, z] in °/s
 * @param pitch Pitch angle in degrees
 * @param roll Roll angle in degrees
 * @return true if sent successfully, false otherwise
 */
bool wifi_data_sender_send_motion_data(
    uint32_t timestamp,
    const float acc[3],
    const float gyro[3],
    float pitch,
    float roll
);

/**
 * @brief Send a preformatted CSV line using UDP
 * @param line CSV line buffer
 * @param len Length of the CSV line in bytes
 * @return true if sent successfully, false otherwise
 */
bool wifi_data_sender_send_csv_line(const char *line, size_t len);

/**
 * @brief Update the UDP destination address.
 * @param gateway_addr Gateway IPv4 address in network byte order.
 * @return true if updated successfully, false otherwise.
 */
bool wifi_data_sender_update_destination_ip(uint32_t gateway_addr);

/**
 * @brief Check if sender is connected
 * @return true if connected to server, false otherwise
 */
bool wifi_data_sender_is_connected(void);

/**
 * @brief Deinitialize sender and close connection
 */
void wifi_data_sender_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // WIFI_DATA_SENDER_H

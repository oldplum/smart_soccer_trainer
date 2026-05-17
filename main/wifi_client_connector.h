/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef WIFI_CLIENT_CONNECTOR_H
#define WIFI_CLIENT_CONNECTOR_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize WiFi and connect to station
 * @param ssid WiFi network name
 * @param password WiFi password
 * @return true if successfully connected, false otherwise
 */
bool wifi_client_init(const char *ssid, const char *password);

/**
 * @brief Get current WiFi connection status
 * @return true if connected, false otherwise
 */
bool wifi_client_is_connected(void);

/**
 * @brief Get assigned IP address
 * @param ip_str Buffer to store IP address string (min 16 bytes)
 * @return true if IP obtained, false otherwise
 */
bool wifi_client_get_ip(char *ip_str, size_t len);

/**
 * @brief Deinitialize WiFi
 */
void wifi_client_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // WIFI_CLIENT_CONNECTOR_H

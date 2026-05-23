/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "wifi_data_sender.h"

#include "esp_log.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define TAG "wifi_sender"
#define SEND_BUFFER_SIZE 512

static int s_socket_fd = -1;
static SemaphoreHandle_t s_socket_mutex = NULL;
static char s_server_ip[16] = {0};
static uint16_t s_server_port = 0;
static struct sockaddr_in s_server_addr;
static bool s_server_addr_valid = false;

/**
 * @brief Create UDP socket for the configured server
 */
static bool _connect_to_server(void)
{
    if (s_socket_fd >= 0) {
        return true;
    }

    if (!s_server_addr_valid) {
        ESP_LOGE(TAG, "Server address is not configured");
        return false;
    }

    s_socket_fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (s_socket_fd < 0) {
        ESP_LOGE(TAG, "Failed to create UDP socket: errno=%d", errno);
        return false;
    }

    struct timeval timeout = {.tv_sec = 2, .tv_usec = 0};
    setsockopt(s_socket_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
    setsockopt(s_socket_fd, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));

    ESP_LOGI(TAG, "UDP sender ready for %s:%d (fd=%d)", s_server_ip, s_server_port, s_socket_fd);
    return true;
}

static bool _send_udp_line(const char *line, size_t len)
{
    if (!line || len == 0) {
        return false;
    }

    if (!_connect_to_server()) {
        return false;
    }

    int sent = sendto(s_socket_fd, line, len, 0,
                      (struct sockaddr *)&s_server_addr,
                      sizeof(s_server_addr));
    if (sent < 0) {
        ESP_LOGW(TAG, "UDP sendto failed: errno=%d (%s)", errno, strerror(errno));
        close(s_socket_fd);
        s_socket_fd = -1;
        return false;
    }

    if (sent != (int)len) {
        ESP_LOGW(TAG, "Partial UDP send: %d/%u bytes", sent, (unsigned)len);
    }

    return true;
}

/**
 * @brief Initialize WiFi data sender
 */
bool wifi_data_sender_init(const char *server_ip, uint16_t server_port)
{
    if (!server_ip || server_port == 0) {
        ESP_LOGE(TAG, "Invalid server IP or port");
        return false;
    }

    if (!s_socket_mutex) {
        s_socket_mutex = xSemaphoreCreateMutex();
        if (!s_socket_mutex) {
            ESP_LOGE(TAG, "Failed to create mutex");
            return false;
        }
    }

    strncpy(s_server_ip, server_ip, sizeof(s_server_ip) - 1);
    s_server_ip[sizeof(s_server_ip) - 1] = '\0';
    s_server_port = server_port;

    memset(&s_server_addr, 0, sizeof(s_server_addr));
    s_server_addr.sin_family = AF_INET;
    s_server_addr.sin_port = htons(s_server_port);
    if (inet_pton(AF_INET, s_server_ip, &s_server_addr.sin_addr) <= 0) {
        ESP_LOGE(TAG, "Invalid IP address: %s", s_server_ip);
        return false;
    }
    s_server_addr_valid = true;

    ESP_LOGI(TAG, "WiFi UDP data sender initialized for %s:%d", server_ip, server_port);
    return true;
}

/**
 * @brief Send motion data as CSV line
 */
bool wifi_data_sender_send_motion_data(
    uint32_t timestamp,
    const float acc[3],
    const float gyro[3],
    float pitch,
    float roll
)
{
    if (!s_socket_mutex) {
        return false;
    }

    if (xSemaphoreTake(s_socket_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire socket mutex");
        return false;
    }

    /* Calculate total acceleration magnitude */
    float acc_total = sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);

    /* Format data as CSV: timestamp,acc_x,acc_y,acc_z,acc_total,gyro_x,gyro_y,gyro_z,pitch,roll */
    char buffer[SEND_BUFFER_SIZE];
    int len = snprintf(buffer, sizeof(buffer),
                      "%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f\n",
                      (unsigned long)timestamp,
                      acc[0], acc[1], acc[2], acc_total,
                      gyro[0], gyro[1], gyro[2],
                      pitch, roll);

    if (len < 0 || len >= (int)sizeof(buffer)) {
        ESP_LOGE(TAG, "Buffer overflow when formatting data");
        xSemaphoreGive(s_socket_mutex);
        return false;
    }

    bool sent_ok = _send_udp_line(buffer, (size_t)len);

    xSemaphoreGive(s_socket_mutex);
    return sent_ok;
}

bool wifi_data_sender_send_csv_line(const char *line, size_t len)
{
    if (!s_socket_mutex) {
        return false;
    }

    if (xSemaphoreTake(s_socket_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire socket mutex");
        return false;
    }

    bool sent_ok = _send_udp_line(line, len);
    xSemaphoreGive(s_socket_mutex);
    return sent_ok;
}

bool wifi_data_sender_update_destination_ip(uint32_t gateway_addr)
{
    if (!s_socket_mutex) {
        return false;
    }

    if (xSemaphoreTake(s_socket_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Failed to acquire socket mutex");
        return false;
    }

    s_server_addr.sin_family = AF_INET;
    s_server_addr.sin_port = htons(s_server_port);
    s_server_addr.sin_addr.s_addr = gateway_addr;
    s_server_addr_valid = true;

    char ip_str[16] = {0};
    ip4_addr_t gateway_ip = {.addr = gateway_addr};
    ip4addr_ntoa_r(&gateway_ip, ip_str, sizeof(ip_str));
    ESP_LOGI(TAG, "UDP destination updated to %s:%d", ip_str, s_server_port);

    xSemaphoreGive(s_socket_mutex);
    return true;
}

/**
 * @brief Check if sender is connected
 */
bool wifi_data_sender_is_connected(void)
{
    if (!s_socket_mutex) {
        return false;
    }

    if (xSemaphoreTake(s_socket_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return false;
    }

    bool connected = (s_socket_fd >= 0);

    xSemaphoreGive(s_socket_mutex);
    return connected;
}

/**
 * @brief Deinitialize sender
 */
void wifi_data_sender_deinit(void)
{
    if (s_socket_mutex) {
        xSemaphoreTake(s_socket_mutex, portMAX_DELAY);

        if (s_socket_fd >= 0) {
            shutdown(s_socket_fd, SHUT_RDWR);
            close(s_socket_fd);
            s_socket_fd = -1;
        }

        xSemaphoreGive(s_socket_mutex);
    }

    ESP_LOGI(TAG, "WiFi data sender deinitialized");
}

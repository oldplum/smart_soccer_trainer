/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 Wi-Fi 客户端并连接到指定 AP
 * 
 * @param ssid      Wi-Fi 热点名称
 * @param password  Wi-Fi 密码
 * @return true     初始化成功并开始连接
 * @return false    初始化失败
 */
bool wifi_manager_init(const char *ssid, const char *password);

/**
 * @brief 检查 Wi-Fi 当前是否已连接并获取 IP 地址
 * 
 * @return true     已连接
 * @return false    未连接
 */
bool wifi_manager_is_connected(void);

/**
 * @brief 获取当前的 IP 地址字符串
 * 
 * @param ip_str    用于存放 IP 的缓冲区
 * @param len       缓冲区长度
 * @return true     成功
 * @return false    失败
 */
bool wifi_manager_get_ip(char *ip_str, size_t len);

/**
 * @brief 获取网关（路由器/热点主机）的 IP 地址字符串
 * 
 * @param ip_str    用于存放网关 IP 的缓冲区
 * @param len       缓冲区长度
 * @return true     成功
 * @return false    失败
 */
bool wifi_manager_get_gateway(char *ip_str, size_t len);

/**
 * @brief 去初始化并停止 Wi-Fi
 */
void wifi_manager_deinit(void);

#ifdef __cplusplus
}
#endif

#endif /* WIFI_MANAGER_H */

/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef HTTP_UPLOADER_H
#define HTTP_UPLOADER_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化并启动定期向 FastAPI 后端上传传感器数据的 HTTP 任务
 * 
 * @param backend_url   后端的完整 HTTP 接口 URL
 *                      （若包含 "AUTO" 字样，例如 http://AUTO:8000/api/device/upload，
 *                      程序将自动把 AUTO 替换为当前 Wi-Fi 热点的网关 IP 地址，方便免焊免配置开发调试）
 * @param period_ms     上报周期（毫秒，建议 2000 即每 2 秒上传一次）
 * @return true         任务启动成功
 * @return false        任务启动失败
 */
bool http_uploader_start(const char *backend_url, int period_ms);

/**
 * @brief 停止上报任务
 */
void http_uploader_stop(void);

#ifdef __cplusplus
}
#endif

#endif /* HTTP_UPLOADER_H */

/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "http_uploader.h"
#include "wifi_manager.h"
#include "soccer_data_sync.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <string>

#define TAG "http_uploader"
#define DEG_TO_RAD(deg) ((deg) * M_PI / 180.0)

static TaskHandle_t s_upload_task_handle = NULL;
static bool s_task_running = false;
static char s_backend_url[256] = {0};
static int s_period_ms = 2000;

// GPS 距离微积分积分
static bool s_prev_gps_valid = false;
static double s_prev_lat = 0.0;
static double s_prev_lon = 0.0;
static float s_total_distance_m = 0.0f;

// 足球动作识别统计（射门和传球计数）
static int s_shoot_count = 0;
static int s_pass_count = 0;
static int64_t s_last_action_time_us = 0;

// GPS 仿真变量（无物理 GPS 时模拟跑步轨迹）
static double s_sim_lat = 39.904216; // 默认北京天安门/球场坐标
static double s_sim_lon = 116.407428;
static float s_sim_distance_m = 0.0f;

// 心率仿真变量（无物理心率模块时模拟跳动）
static int s_sim_heart_rate = 120;

/**
 * @brief 使用哈弗辛公式（Haversine）计算两个 GPS 坐标之间的实际距离（米）
 */
static double calculate_gps_distance(double lat1, double lon1, double lat2, double lon2)
{
    double R = 6371000.0; // 地球半径（米）
    double phi1 = DEG_TO_RAD(lat1);
    double phi2 = DEG_TO_RAD(lat2);
    double delta_phi = DEG_TO_RAD(lat2 - lat1);
    double delta_lambda = DEG_TO_RAD(lon2 - lon1);

    double a = sin(delta_phi / 2.0) * sin(delta_phi / 2.0) +
               cos(phi1) * cos(phi2) *
               sin(delta_lambda / 2.0) * sin(delta_lambda / 2.0);
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

    return R * c;
}

/**
 * @brief 足球动作识别算法入口（射门与传球）
 * @note  【核心提示】用户可以在这里修改识别逻辑、阈值、添加窗滤波或多帧峰值检测
 */
static void detect_football_action(const SoccerSensorData &data)
{
    float acc_x = data.acc[0];
    float acc_y = data.acc[1];
    float acc_z = data.acc[2];
    
    // 1. 计算三轴加速度的合矢量幅值（单位：g）
    float acc_mag = sqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z);
    int64_t now_us = esp_timer_get_time();

    // 2. 动作判定主逻辑：合加速度突变超过阈值（如 3.0g），且与上次触发间隔 > 1.5 秒（简单消抖，防连击）
    if (acc_mag > 3.0f && (now_us - s_last_action_time_us) > 1500000) {
        s_last_action_time_us = now_us;
        
        // 3. 计算三轴角速度的合矢量幅值（单位：度/秒）
        float gyro_mag = sqrt(data.gyro[0]*data.gyro[0] + data.gyro[1]*data.gyro[1] + data.gyro[2]*data.gyro[2]);
        
        // 4. 简易特征识别：角速度非常高（身体随之剧烈摆动旋转）判定为射门，较低则判定为传球
        if (gyro_mag > 300.0f) {
            s_shoot_count++;
            ESP_LOGW(TAG, "⚽ 检测到【射门】动作! 累计射门: %d (合加速度: %.2fg, 角速度: %.1f deg/s)", 
                     s_shoot_count, acc_mag, gyro_mag);
        } else {
            s_pass_count++;
            ESP_LOGI(TAG, "👟 检测到【传球】动作! 累计传球: %d (合加速度: %.2fg, 角速度: %.1f deg/s)", 
                     s_pass_count, acc_mag, gyro_mag);
        }
    }
}

/**
 * @brief HTTP 客户端事件回调
 */
static esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    // 调试用，一般不需要打印
    return ESP_OK;
}

static TaskHandle_t s_detect_task_handle = NULL;

static void action_detect_task(void *pvParameters)
{
    ESP_LOGI(TAG, "实时动作检测任务已启动 (周期: 20毫秒)");
    while (s_task_running) {
        // 只有当传感器数据有效时，才进行实时动作检测
        if (g_soccer_sensor_data.timestamp > 0) {
            SoccerSensorData data;
            memcpy(&data, &g_soccer_sensor_data, sizeof(SoccerSensorData));
            detect_football_action(data);
        }
        vTaskDelay(pdMS_TO_TICKS(20)); // 每 20ms 检测一次（50Hz 采样率）
    }
    s_detect_task_handle = NULL;
    vTaskDelete(NULL);
}

/**
 * @brief 数据上报主线程
 */
static void http_upload_task(void *pvParameters)
{
    ESP_LOGI(TAG, "HTTP 智能上报任务已启动 (周期: %d 毫秒, 目标 URL: %s)", s_period_ms, s_backend_url);
    s_task_running = true;

    // 初始化随机种子
    srand(esp_timer_get_time());

    while (s_task_running) {
        if (wifi_manager_is_connected()) {
            // 线程安全：局部复制全局传感器数据
            SoccerSensorData data;
            memcpy(&data, &g_soccer_sensor_data, sizeof(SoccerSensorData));

            /* ===== 1. 心率与血氧数据获取 (若无硬件则进行仿真) ===== */
            int heart_rate = 0;
            int blood_oxygen = 0;
            if (data.hr_valid && data.heart_rate > 0) {
                // 有真实传感器数据
                heart_rate = data.heart_rate;
                blood_oxygen = data.spo2;
            } else {
                // 无心率传感器：仿真生成一段心跳波形（在 120 - 160bpm 之间平滑起伏）
                s_sim_heart_rate += (rand() % 7) - 3;
                if (s_sim_heart_rate < 115) s_sim_heart_rate = 115;
                if (s_sim_heart_rate > 165) s_sim_heart_rate = 165;
                
                heart_rate = s_sim_heart_rate;
                blood_oxygen = 97 + (rand() % 3); // 模拟 97% ~ 99% 的血氧
            }

            // 心率或血氧异常触发负荷报警
            bool overload_warning = (heart_rate > 160) || (blood_oxygen < 90);

            /* ===== 2. GPS 经纬度与跑动距离获取 (若无硬件则进行仿真) ===== */
            double latitude = 0.0;
            double longitude = 0.0;
            float speed_kmh = 0.0f;
            float distance_m = 0.0f;

            if (data.gnss_valid && data.latitude != 0.0) {
                // 有真实 GPS 信号
                latitude = data.latitude;
                longitude = data.longitude;
                speed_kmh = data.speed;

                // 运动距离积分
                if (s_prev_gps_valid) {
                    if (speed_kmh > 1.5f) { // 过滤原地漂移
                        double dist = calculate_gps_distance(s_prev_lat, s_prev_lon, latitude, longitude);
                        if (dist > 0.1 && dist < 50.0) { // 剔除异常跳转
                            s_total_distance_m += dist;
                        }
                    }
                }
                s_prev_lat = latitude;
                s_prev_lon = longitude;
                s_prev_gps_valid = true;
                distance_m = s_total_distance_m;
            } else {
                // 无 GPS 传感器：仿真跑动轨迹（绕足球场中圈弧跑动）
                s_prev_gps_valid = false;
                double elapsed_sec = (double)esp_timer_get_time() / 1000000.0;
                
                // 经纬度做简易圆周摆动，模拟慢跑轨迹
                latitude = s_sim_lat + 0.0003 * sin(elapsed_sec / 20.0);
                longitude = s_sim_lon + 0.0004 * cos(elapsed_sec / 20.0);
                speed_kmh = 12.0f + 4.0f * sin(elapsed_sec / 5.0) + (rand() % 3); // 模拟速度 10-18 km/h
                
                // 距离积分：速度(m/s) * 周期(秒)
                s_sim_distance_m += (speed_kmh / 3.6f) * ((float)s_period_ms / 1000.0f);
                distance_m = s_sim_distance_m;
            }

            /* ===== 3. 构建 JSON 数据包 ===== */
            char json_payload[512];
            int json_len = snprintf(json_payload, sizeof(json_payload),
                "{"
                  "\"player_id\":\"player_001\","
                  "\"timestamp_ms\":%llu,"
                  "\"football_stats\":{"
                    "\"shoot_count\":%d,"
                    "\"pass_count\":%d"
                  "},"
                  "\"health_monitor\":{"
                    "\"heart_rate\":%d,"
                    "\"blood_oxygen\":%d,"
                    "\"overload_warning\":%s"
                  "},"
                  "\"gps_tracking\":{"
                    "\"latitude\":%.7f,"
                    "\"longitude\":%.7f,"
                    "\"speed_kmh\":%.2f,"
                    "\"total_distance_m\":%.2f"
                  "}"
                "}",
                (unsigned long long)(data.timestamp ? data.timestamp : (esp_timer_get_time() / 1000)),
                s_shoot_count,
                s_pass_count,
                heart_rate,
                blood_oxygen,
                overload_warning ? "true" : "false",
                latitude,
                longitude,
                speed_kmh,
                distance_m
            );

            if (json_len < 0 || json_len >= (int)sizeof(json_payload)) {
                ESP_LOGE(TAG, "JSON 数据包构建失败");
            } else {
                /* ===== 4. 自动解析 AUTO 关键字（热点极速联调） ===== */
                char resolved_url[256];
                if (strstr(s_backend_url, "AUTO") != NULL) {
                    char gw_ip[16] = {0};
                    if (wifi_manager_get_gateway(gw_ip, sizeof(gw_ip))) {
                        std::string url_str(s_backend_url);
                        size_t pos = url_str.find("AUTO");
                        url_str.replace(pos, 4, gw_ip);
                        strncpy(resolved_url, url_str.c_str(), sizeof(resolved_url) - 1);
                        resolved_url[sizeof(resolved_url) - 1] = '\0';
                    } else {
                        strncpy(resolved_url, s_backend_url, sizeof(resolved_url) - 1);
                        resolved_url[sizeof(resolved_url) - 1] = '\0';
                    }
                } else {
                    strncpy(resolved_url, s_backend_url, sizeof(resolved_url) - 1);
                    resolved_url[sizeof(resolved_url) - 1] = '\0';
                }

                /* ===== 5. 发送 HTTP POST 请求 ===== */
                esp_http_client_config_t config;
                memset(&config, 0, sizeof(config));
                config.url = resolved_url;
                config.event_handler = http_event_handler;
                config.timeout_ms = 4000;
                esp_http_client_handle_t client = esp_http_client_init(&config);
                if (client) {
                    esp_http_client_set_method(client, HTTP_METHOD_POST);
                    esp_http_client_set_header(client, "Content-Type", "application/json");
                    esp_http_client_set_post_field(client, json_payload, json_len);

                    esp_err_t err = esp_http_client_perform(client);
                    if (err == ESP_OK) {
                        int status_code = esp_http_client_get_status_code(client);
                        ESP_LOGI(TAG, "上报成功! HTTP 状态码: %d | 目标地址: %s", status_code, resolved_url);
                    } else {
                        ESP_LOGE(TAG, "上报失败! 目标 %s 原因: %s", resolved_url, esp_err_to_name(err));
                    }
                    esp_http_client_cleanup(client);
                } else {
                    ESP_LOGE(TAG, "HTTP 客户端初始化失败");
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(s_period_ms));
    }

    s_upload_task_handle = NULL;
    vTaskDelete(NULL);
}

bool http_uploader_start(const char *backend_url, int period_ms)
{
    if (s_upload_task_handle != NULL) {
        ESP_LOGW(TAG, "上报任务已在运行中");
        return true;
    }

    if (!backend_url) {
        ESP_LOGE(TAG, "URL 不能为空");
        return false;
    }

    strncpy(s_backend_url, backend_url, sizeof(s_backend_url) - 1);
    s_backend_url[sizeof(s_backend_url) - 1] = '\0';
    s_period_ms = period_ms;

    BaseType_t ret = xTaskCreate(
        http_upload_task,
        "http_upload",
        8192,  // 预留足够栈空间用于 JSON 解析和 TLS 握手
        NULL,
        5,
        &s_upload_task_handle
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "上报任务创建失败");
        return false;
    }

    // 创建实时的运动动作检测任务
    xTaskCreate(
        action_detect_task,
        "action_detect",
        4096,
        NULL,
        4,
        &s_detect_task_handle
    );

    return true;
}

void http_uploader_stop(void)
{
    s_task_running = false;
    ESP_LOGI(TAG, "停止上报任务...");
}

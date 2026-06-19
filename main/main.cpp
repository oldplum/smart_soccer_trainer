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
 * @brief 调试串口任务 - 把传感器数据打印到串口
 *
 * 这个函数就像一个"数据打印员"，它会：
 *   - 每 10 毫秒打印一次运动数据（加速度、陀螺仪、角度）
 *   - 每 3 秒打印一次环境数据（温度、湿度、气压、空气质量）
 *
 * 数据格式：CSV 格式（用逗号分隔），方便后续导入 Excel 分析
 *
 * @param arg  FreeRTOS 任务参数（这里用不到）
 */
void debug_serial_task(void *arg)
{
    ESP_UTILS_LOGI("Debug serial task started");

    // 记录任务开始时刻（单位：毫秒），用它算"相对时间"
    // 就像用秒表的"开始计时"按钮
    uint32_t start_time_ms = (uint32_t)(esp_timer_get_time() / 1000);

    // 死循环：嵌入式系统任务永远不会结束
    // 每隔 5ms 检查一次，看是否需要打印数据
    while (1) {
        // now_ms = 当前时刻 - 开始时刻，即"任务运行了多少毫秒"
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000) - start_time_ms;

        // 获取温度应用和指南针应用的"实例"（单例模式：整个程序只有一个对象）
        // 相当于去拿已经存在的遥控器，而不是重新买一个
        auto temp_app = esp_brookesia::apps::Temperature::requestInstance();
        auto compass_app = esp_brookesia::apps::Compass::requestInstance();

        /* ============================================================
         * 第一部分：打印运动数据（每 10ms 一次）
         * 包括：加速度(XYZ)、陀螺仪(XYZ)、俯仰角、横滚角、航向角
         * ============================================================ */
        // 三个条件：指南针应用存在、传感器在线、距离上次打印已过 10ms
        if (compass_app && compass_app->isSensorOnline() &&
            (now_ms - last_motion_print_ms) >= DEBUG_SERIAL_PERIOD_MOTION_MS) {
            // 检查数据是否是"新的"（时间戳变了）
            // 如果数据没变就不重复打印，避免刷屏
            uint32_t curr_ts = g_soccer_sensor_data.timestamp;
            if (curr_ts != last_printed_motion_ts) {
                // CSV 格式输出：时间戳,加速度X,加速度Y,加速度Z,陀螺仪X,陀螺仪Y,陀螺仪Z,pitch,roll,heading
                printf("%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%.1f,%.1f\n",
                       g_soccer_sensor_data.timestamp,  // 时间戳（毫秒）
                       g_soccer_sensor_data.acc[0],     // 加速度 X 轴
                       g_soccer_sensor_data.acc[1],     // 加速度 Y 轴
                       g_soccer_sensor_data.acc[2],     // 加速度 Z 轴
                       g_soccer_sensor_data.gyro[0],    // 陀螺仪 X 轴
                       g_soccer_sensor_data.gyro[1],    // 陀螺仪 Y 轴
                       g_soccer_sensor_data.gyro[2],    // 陀螺仪 Z 轴
                       g_soccer_sensor_data.pitch,      // 俯仰角（上下倾斜）
                       g_soccer_sensor_data.roll,       // 横滚角（左右翻滚）
                       g_soccer_sensor_data.heading);   // 航向角（指南针方向）
                last_printed_motion_ts = curr_ts;  // 记住这次打印的时间戳
            }
            last_motion_print_ms = now_ms;  // 更新"上次打印时刻"
        }

        /* ============================================================
         * 第二部分：打印环境数据（每 3 秒一次）
         * 包括：温度、湿度、气压、空气质量指数
         * ============================================================ */
        // 三个条件：温度应用存在、传感器在线、距离上次打印已过 3 秒
        if (temp_app && temp_app->isSensorOnline() &&
            (now_ms - last_env_print_ms) >= DEBUG_SERIAL_PERIOD_ENV_MS) {
            // 同样检查数据是否是"新的"
            uint32_t curr_ts = g_soccer_sensor_data.timestamp;
            if (curr_ts != last_printed_env_ts) {
                // CSV 格式输出：时间戳,温度,湿度,气压,空气质量
                printf("%lu,%.2f,%.2f,%.1f,%.1f\n",
                       g_soccer_sensor_data.timestamp,  // 时间戳
                       g_soccer_sensor_data.temp,       // 温度（摄氏度）
                       g_soccer_sensor_data.humidity,   // 相对湿度（%）
                       g_soccer_sensor_data.pressure,   // 大气压（百帕 hPa）
                       g_soccer_sensor_data.iaq);       // 空气质量指数 IAQ
                last_printed_env_ts = curr_ts;
            }
            last_env_print_ms = now_ms;
        }

        // 延时 5 毫秒，把 CPU 让给其他任务
        // 如果不休眠，这个循环会一直霸占 CPU，其他程序就没法运行了
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

/**
 * @brief 程序主入口 - ESP32 上电后第一个执行的函数
 *
 * 这个函数就像"开机启动流程"，按顺序做以下几件事：
 *   1. 初始化硬件（开发板管理）
 *   2. 准备数据缓冲区
 *   3. 启动调试串口打印（最关键，始终运行）
 *   4. 创建 Phone 对象（管理显示屏和应用）
 *   5. 尝试从注册表安装应用
 *   6. ★ 兜底方案：直接启动传感器采集（即使屏幕挂了也能收集数据）
 *
 * 设计哲学："传感器数据是核心，UI 只是锦上添花"——即使显示屏/手机模块初始化失败，
 *          传感器仍然照常采集数据，保证实验数据不会丢失。
 */
extern "C" void app_main(void)
{
    // 打印程序启动标志
    ESP_UTILS_LOGI("Sensor data collection demo");

    /* ========== 第 1 步：初始化开发板硬件 ========== */
    // 打印板子型号信息（方便调试，看是不是跑在正确的硬件上）
    esp_board_manager_print_board_info();
    // 初始化板级管理器（配置 GPIO、电源、外设等）
    int ret = esp_board_manager_init();
    // assert = "断言"：如果初始化失败（ret != 0），程序直接死机报错
    // 这一步必须成功，否则后面啥也干不了
    assert((ret == 0) && "Board manager initialization failed");

    /* ========== 第 2 步：初始化传感器数据缓冲区 ========== */
    // g_soccer_sensor_data 是一个全局变量，用来存放所有传感器的数据
    // 这一步就像"开箱并整理好盒子，准备往里装数据"
    soccer_data_init(&g_soccer_sensor_data);

    /* ========== 第 3 步：启动调试串口打印任务（最重要！）========== */
    // 为什么放这么早？——即使后面手机/显示屏初始化失败了，
    // 这个任务也已经在运行，数据仍然可以从串口看到
    // 这就叫"保证核心功能优先"
    {
        // 线程配置：名字叫 debug_serial，栈大小 4KB
        // thread_config_guard 是个 C++ "守卫"对象，离开大括号自动释放配置
        esp_utils::thread_config_guard debug_thread_config({
            .name = "debug_serial",
            .stack_size = 4096,
        });
        // 创建一个新线程来运行 debug_serial_task
        // detach() 表示"让这个线程独立运行，主程序不用等它"
        boost::thread(debug_serial_task, nullptr).detach();
    }

    /* ========== 第 4 步：创建 Phone 对象（显示屏 + 应用管理）========== */
    // Phone 类管理：显示屏 + 应用（类似手机的界面）
    // new (std::nothrow) 的意思：如果内存不够，返回空指针而不是崩溃
    Phone *phone = new (std::nothrow) Phone();
    // 如果创建失败（内存不够），直接退出
    ESP_UTILS_CHECK_NULL_EXIT(phone, "Create phone failed");

    {
        /* 启动 Phone（初始化显示屏、触屏等）*/
        // 注意：即使失败了也要继续！因为后面有兜底方案
        bool phone_begin_ok = phone->begin();
        if (!phone_begin_ok) {
            ESP_LOGW("main", "Phone begin failed, continuing anyway");
        }

        /* 从注册表初始化应用列表（就像先看一眼应用商店有哪些 APP）*/
        std::vector<systems::base::Manager::RegistryAppInfo> inited_apps;
        if (!phone->initAppFromRegistry(inited_apps)) {
            ESP_LOGE("main", "Init app registry failed");
        } else {
            // 指定安装顺序：环境监测 → 手势检测 → 指南针
            std::vector<std::string> ordered_app_names = {"Environment", "Gesture Detect", "Compass"};
            // 按顺序安装应用
            bool install_ok = phone->installAppFromRegistry(inited_apps, &ordered_app_names);
            if (!install_ok) {
                // 可能是显示屏排线松了之类的硬件问题
                // 不要紧，继续走，后面还有兜底方案
                ESP_LOGW("main", "Install app registry failed (display issue?), but continuing with direct sensor startup");
            } else {
                ESP_LOGI("main", "All apps installed successfully");
            }
        }
    }

    /* ========== 第 5 步：兜底方案 - 直接启动传感器采集（★关键！）========== */
    // 即使上面 Phone 初始化失败了（比如显示屏坏了），这里也要直接启动传感器
    // 这就是"容错设计"：核心功能（数据采集）不能被 UI 故障拖垮
    ESP_LOGI("main", "Starting sensor collection via direct app access");

    // 启动温度传感器（从单例拿实例）
    auto temp_app = esp_brookesia::apps::Temperature::requestInstance();
    if (temp_app) {
        ESP_LOGI("main", "Starting Temperature sensor collection");
        if (!temp_app->startSensorCollection()) {
            ESP_LOGW("main", "Failed to start Temperature sensor collection");
        }
    }

    // 启动指南针/运动传感器（从单例拿实例）
    auto compass_app = esp_brookesia::apps::Compass::requestInstance();
    if (compass_app) {
        ESP_LOGI("main", "Starting Compass sensor collection");
        if (!compass_app->startSensorCollection()) {
            ESP_LOGW("main", "Failed to start Compass sensor collection");
        }
    }

    // 到此，主函数就执行完了。
    // 注意：ESP32 的 app_main 结束后，系统不会停止，
    // 前面启动的那些 FreeRTOS 任务（比如 debug_serial_task）会在后台继续运行
}
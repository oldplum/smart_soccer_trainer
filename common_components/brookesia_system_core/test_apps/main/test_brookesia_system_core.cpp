/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "unity.h"
#include "unity_test_runner.h"
#include "unity_test_utils_memory.h"
#include "lvgl.h"
#include "lvgl_private.h"
#include "brookesia/system_core.hpp"

#define TEST_LVGL_RESOLUTION_WIDTH  CONFIG_TEST_LVGL_RESOLUTION_WIDTH
#define TEST_LVGL_RESOLUTION_HEIGHT CONFIG_TEST_LVGL_RESOLUTION_HEIGHT

// using namespace esp_brookesia;

static const char *TAG = "test_brookesia_system_core";

static void test_lvgl_init(lv_display_t **disp_out, lv_indev_t **tp_out);
static void test_lvgl_deinit(lv_display_t *disp, lv_indev_t *indev);

TEST_CASE("test esp-brookesia to begin and delete", "[esp-brookesia][phone][begin_del]")
{
    lv_display_t *disp = nullptr;
    lv_indev_t *tp = nullptr;

    test_lvgl_init(&disp, &tp);

    test_lvgl_deinit(disp, tp);
}

static void test_lvgl_init(lv_display_t **disp_out, lv_indev_t **tp_out)
{
    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();

    ESP_LOGI(TAG, "Register display driver to LVGL(%dx%d)", TEST_LVGL_RESOLUTION_WIDTH, TEST_LVGL_RESOLUTION_HEIGHT);
    int buf_bytes = TEST_LVGL_RESOLUTION_WIDTH * 10 * lv_color_format_get_size(LV_COLOR_FORMAT_RGB565);
    void *buf = malloc(buf_bytes);
    TEST_ASSERT_NOT_NULL_MESSAGE(buf, "Failed to alloc buffer");

    lv_display_t *disp = lv_display_create(TEST_LVGL_RESOLUTION_WIDTH, TEST_LVGL_RESOLUTION_HEIGHT);
    TEST_ASSERT_NOT_NULL_MESSAGE(disp, "Failed to create display");
    lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565);
    lv_display_set_buffers(disp, buf, nullptr, buf_bytes, LV_DISPLAY_RENDER_MODE_PARTIAL);
    lv_display_set_flush_cb(disp, [](lv_display_t *disp, const lv_area_t *area, uint8_t *color_p) {});

    lv_indev_t *indev = lv_indev_create();
    TEST_ASSERT_NOT_NULL_MESSAGE(indev, "Failed to create input device");
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_mode(indev, LV_INDEV_MODE_EVENT);
    lv_indev_set_read_cb(indev, [](lv_indev_t *indev, lv_indev_data_t *data) {});

    if (disp_out) {
        *disp_out = disp;
    }
    if (tp_out) {
        *tp_out = indev;
    }
}

static void test_lvgl_deinit(lv_display_t *disp, lv_indev_t *indev)
{
    ESP_LOGI(TAG, "Deinitialize LVGL library");
    free(disp->buf_1->data);
    lv_display_delete(disp);
    lv_indev_delete(indev);
    lv_deinit();
}

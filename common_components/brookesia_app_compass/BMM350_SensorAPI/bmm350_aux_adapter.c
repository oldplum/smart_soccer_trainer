/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "bmi270_api.h"
#include "bmm350.h"
#include "bmm350_defs.h"
#include "bmm350_aux_adapter.h"

static const char *TAG = "BMM350_AUX_ADAPTER";

static int8_t bmm350_aux_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr)
{
    struct bmi2_dev *bmi2 = (struct bmi2_dev *)intf_ptr;
    return bmi2_read_aux_man_mode(reg_addr, data, len, bmi2);
}

static int8_t bmm350_aux_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr)
{
    struct bmi2_dev *bmi2 = (struct bmi2_dev *)intf_ptr;
    return bmi2_write_aux_man_mode(reg_addr, data, len, bmi2);
}

static void bmm350_delay_us_cb(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    for (volatile uint32_t i = 0; i < period; i++) {
        __asm__ __volatile__("nop");
    }
}

int8_t bmm350_aux_adapter_init(const bmm350_aux_config_t *config, bmm350_aux_handle_t *handle)
{
    if (!config || !handle || !config->bmi2_dev) {
        return BMM350_E_NULL_PTR;
    }

    memset(handle, 0, sizeof(*handle));
    handle->bmi2_dev = config->bmi2_dev;

    memset(&handle->bmm350_dev, 0, sizeof(handle->bmm350_dev));
    handle->bmm350_dev.intf_ptr = handle->bmi2_dev;
    handle->bmm350_dev.read = bmm350_aux_read;
    handle->bmm350_dev.write = bmm350_aux_write;
    handle->bmm350_dev.delay_us = bmm350_delay_us_cb;

    int8_t rslt = bmm350_init(&handle->bmm350_dev);
    if (rslt != BMM350_OK) {
        ESP_LOGE(TAG, "bmm350_init failed: %d", rslt);
        return rslt;
    }

    rslt = bmm350_set_odr_performance(BMM350_DATA_RATE_12_5HZ, BMM350_REGULARPOWER, &handle->bmm350_dev);
    if (rslt != BMM350_OK) {
        ESP_LOGE(TAG, "set_odr_performance failed: %d", rslt);
        return rslt;
    }

    rslt = bmm350_enable_axes(BMM350_X_EN, BMM350_Y_EN, BMM350_Z_EN, &handle->bmm350_dev);
    if (rslt != BMM350_OK) {
        ESP_LOGE(TAG, "enable_axes failed: %d", rslt);
        return rslt;
    }

    rslt = bmm350_set_powermode(BMM350_NORMAL_MODE, &handle->bmm350_dev);
    if (rslt != BMM350_OK) {
        ESP_LOGE(TAG, "set_powermode failed: %d", rslt);
        return rslt;
    }

    handle->is_initialized = true;
    ESP_LOGI(TAG, "BMM350 AUX adapter initialized successfully");
    return BMM350_OK;
}

int8_t bmm350_aux_adapter_deinit(bmm350_aux_handle_t *handle)
{
    if (!handle) {
        return BMM350_E_NULL_PTR;
    }
    handle->is_initialized = false;
    return BMM350_OK;
}

int8_t bmm350_aux_adapter_read_mag_temp(bmm350_aux_handle_t *handle, struct bmm350_mag_temp_data *mag_temp)
{
    if (!handle || !mag_temp || !handle->is_initialized) {
        return BMM350_E_NULL_PTR;
    }
    return bmm350_get_compensated_mag_xyz_temp_data(mag_temp, &handle->bmm350_dev);
}

int8_t bmm350_aux_adapter_read_regs(bmm350_aux_handle_t *handle, uint8_t reg_addr, uint8_t *data, uint32_t len)
{
    if (!handle || !data || !handle->is_initialized) {
        return BMM350_E_NULL_PTR;
    }
    return bmm350_get_regs(reg_addr, data, len, &handle->bmm350_dev);
}

int8_t bmm350_aux_adapter_write_regs(bmm350_aux_handle_t *handle, uint8_t reg_addr, const uint8_t *data, uint32_t len)
{
    if (!handle || !data || !handle->is_initialized) {
        return BMM350_E_NULL_PTR;
    }
    return bmm350_set_regs(reg_addr, data, len, &handle->bmm350_dev);
}

float bmm350_aux_adapter_calculate_heading(float mag_x, float mag_y)
{
    float heading = atan2f(mag_y, mag_x) * 180.0f / (float)M_PI;
    if (heading < 0.0f) {
        heading += 360.0f;
    }
    return heading;
}

float bmm350_aux_adapter_calculate_strength(float mag_x, float mag_y, float mag_z)
{
    return sqrtf(mag_x * mag_x + mag_y * mag_y + mag_z * mag_z);
}

bool bmm350_aux_adapter_is_field_normal(float strength_ut)
{
    return (strength_ut >= 20.0f && strength_ut <= 100.0f);
}

const char *bmm350_aux_adapter_get_direction(float heading)
{
    if (heading >= 315.0f || heading < 45.0f) {
        return "North";
    }
    if (heading < 135.0f) {
        return "East";
    }
    if (heading < 225.0f) {
        return "South";
    }
    return "West";
}

int8_t bmm350_aux_adapter_magnetic_reset_and_wait(bmm350_aux_handle_t *handle)
{
    if (!handle || !handle->is_initialized) {
        return BMM350_E_NULL_PTR;
    }
    return bmm350_magnetic_reset_and_wait(&handle->bmm350_dev);
}

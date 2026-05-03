/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdbool.h>
#include "bmi270_api.h"
#include "bmm350.h"

typedef struct {
    struct bmi2_dev *bmi2_dev;
} bmm350_aux_config_t;

typedef struct {
    struct bmi2_dev *bmi2_dev;
    struct bmm350_dev bmm350_dev;
    bool is_initialized;
} bmm350_aux_handle_t;

int8_t bmm350_aux_adapter_init(const bmm350_aux_config_t *config, bmm350_aux_handle_t *handle);
int8_t bmm350_aux_adapter_deinit(bmm350_aux_handle_t *handle);

int8_t bmm350_aux_adapter_read_mag_temp(bmm350_aux_handle_t *handle, struct bmm350_mag_temp_data *mag_temp);
int8_t bmm350_aux_adapter_read_regs(bmm350_aux_handle_t *handle, uint8_t reg_addr, uint8_t *data, uint32_t len);
int8_t bmm350_aux_adapter_write_regs(bmm350_aux_handle_t *handle, uint8_t reg_addr, const uint8_t *data, uint32_t len);

float bmm350_aux_adapter_calculate_heading(float mag_x, float mag_y);
float bmm350_aux_adapter_calculate_strength(float mag_x, float mag_y, float mag_z);
bool  bmm350_aux_adapter_is_field_normal(float strength_ut);
const char *bmm350_aux_adapter_get_direction(float heading_deg);

/* Trigger magnetic reset per datasheet guidance. Safe to call on wake from long suspend (>2s). */
int8_t bmm350_aux_adapter_magnetic_reset_and_wait(bmm350_aux_handle_t *handle);

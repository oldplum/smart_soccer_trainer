/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef COMPASS_WRAPPER_HPP
#define COMPASS_WRAPPER_HPP

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Return value: true for success, false for failure
bool compass_back(void);
bool compass_set_correct(bool correct);

// NVS storage functions
bool compass_save_calibration_to_nvs(void);
bool compass_load_calibration_from_nvs(void);
bool compass_clear_calibration_from_nvs(void);
bool compass_is_calibrated(void);

// Async NVS functions (recommended for UI thread)
void compass_save_calibration_to_nvs_async(void);
void compass_load_calibration_from_nvs_async(void);

#ifdef __cplusplus
}
#endif

#endif

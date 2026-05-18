/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef BMI270_COLLECTOR_H
#define BMI270_COLLECTOR_H

#include <stdbool.h>
#include "i2c_bus.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Hardware GPTimer sample rate (Hz). Period = 1_000_000 / BMI270_COLLECT_HZ microseconds. */
#define BMI270_COLLECT_HZ           100
#define BMI270_COLLECT_PERIOD_US    10000   /* 10 ms */

bool bmi270_collector_start(void);
void bmi270_collector_stop(void);
bool bmi270_collector_is_active(void);
i2c_bus_handle_t bmi270_collector_get_i2c_bus(void);
void bmi270_collector_i2c_lock(void);
void bmi270_collector_i2c_unlock(void);

#ifdef __cplusplus
}
#endif

#endif /* BMI270_COLLECTOR_H */

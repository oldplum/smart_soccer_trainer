/**
* Copyright (c) 2023 Bosch Sensortec GmbH. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file  common.c
*
*/

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "bmm350.h"
#include "common.h"
#include "i2c_bus.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"
#include "sdkconfig.h"

/******************************************************************************/
/*!                Structure definition                                       */

#define TAG "BMM350_ESP32"

/******************************************************************************/
/*!                Static variable definition                                 */

/*! Variable that holds the I2C device address selection */
static uint8_t dev_addr;

/*! I2C bus/device handles */
static i2c_bus_handle_t s_i2c_bus = NULL;
static i2c_bus_device_handle_t s_i2c_dev = NULL;

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * I2C read function map to COINES platform
 */
BMM350_INTF_RET_TYPE bmm350_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t *)intf_ptr;
    if (s_i2c_dev == NULL || device_addr != dev_addr) {
        if (s_i2c_dev) {
            i2c_bus_device_delete(&s_i2c_dev);
            s_i2c_dev = NULL;
        }
        s_i2c_dev = i2c_bus_device_create(s_i2c_bus, device_addr, 0);
        if (s_i2c_dev == NULL) {
            ESP_LOGE(TAG, "i2c_bus_device_create failed for 0x%02X", device_addr);
            return BMM350_E_COM_FAIL;
        }
        dev_addr = device_addr;
    }

    esp_err_t ret = i2c_bus_read_bytes(s_i2c_dev, reg_addr, (uint16_t)length, reg_data);
    return (ret == ESP_OK) ? BMM350_INTF_RET_SUCCESS : BMM350_E_COM_FAIL;
}

/*!
 * I2C write function map to COINES platform
 */
BMM350_INTF_RET_TYPE bmm350_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t *)intf_ptr;
    if (s_i2c_dev == NULL || device_addr != dev_addr) {
        if (s_i2c_dev) {
            i2c_bus_device_delete(&s_i2c_dev);
            s_i2c_dev = NULL;
        }
        s_i2c_dev = i2c_bus_device_create(s_i2c_bus, device_addr, 0);
        if (s_i2c_dev == NULL) {
            ESP_LOGE(TAG, "i2c_bus_device_create failed for 0x%02X", device_addr);
            return BMM350_E_COM_FAIL;
        }
        dev_addr = device_addr;
    }

    esp_err_t ret = i2c_bus_write_bytes(s_i2c_dev, reg_addr, (uint16_t)length, reg_data);
    return (ret == ESP_OK) ? BMM350_INTF_RET_SUCCESS : BMM350_E_COM_FAIL;
}

/*!
 * Delay function map to COINES platform
 */
void bmm350_delay(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    if (period < 1000) {
        esp_rom_delay_us(period);
    } else {
        vTaskDelay(pdMS_TO_TICKS((period + 999) / 1000));
    }
}

/*!
 *  @brief Prints the execution status of the APIs.
 */
void bmm350_error_codes_print_result(const char api_name[], int8_t rslt)
{
    switch (rslt) {
    case BMM350_OK:
        break;

    case BMM350_E_NULL_PTR:
        printf("%s Error [%d] : Null pointer\r\n", api_name, rslt);
        break;
    case BMM350_E_COM_FAIL:
        printf("%s Error [%d] : Communication fail\r\n", api_name, rslt);
        break;
    case BMM350_E_DEV_NOT_FOUND:
        printf("%s Error [%d] : Device not found\r\n", api_name, rslt);
        break;
    case BMM350_E_INVALID_CONFIG:
        printf("%s Error [%d] : Invalid configuration\r\n", api_name, rslt);
        break;
    case BMM350_E_BAD_PAD_DRIVE:
        printf("%s Error [%d] : Bad pad drive\r\n", api_name, rslt);
        break;
    case BMM350_E_RESET_UNFINISHED:
        printf("%s Error [%d] : Reset unfinished\r\n", api_name, rslt);
        break;
    case BMM350_E_INVALID_INPUT:
        printf("%s Error [%d] : Invalid input\r\n", api_name, rslt);
        break;
    case BMM350_E_SELF_TEST_INVALID_AXIS:
        printf("%s Error [%d] : Self-test invalid axis selection\r\n", api_name, rslt);
        break;
    case BMM350_E_OTP_BOOT:
        printf("%s Error [%d] : OTP boot\r\n", api_name, rslt);
        break;
    case BMM350_E_OTP_PAGE_RD:
        printf("%s Error [%d] : OTP page read\r\n", api_name, rslt);
        break;
    case BMM350_E_OTP_PAGE_PRG:
        printf("%s Error [%d] : OTP page prog\r\n", api_name, rslt);
        break;
    case BMM350_E_OTP_SIGN:
        printf("%s Error [%d] : OTP sign\r\n", api_name, rslt);
        break;
    case BMM350_E_OTP_INV_CMD:
        printf("%s Error [%d] : OTP invalid command\r\n", api_name, rslt);
        break;
    case BMM350_E_OTP_UNDEFINED:
        printf("%s Error [%d] : OTP undefined\r\n", api_name, rslt);
        break;
    case BMM350_E_ALL_AXIS_DISABLED:
        printf("%s Error [%d] : All axis are disabled\r\n", api_name, rslt);
        break;
    case BMM350_E_PMU_CMD_VALUE:
        printf("%s Error [%d] : Unexpected PMU CMD value\r\n", api_name, rslt);
        break;
    default:
        printf("%s Error [%d] : Unknown error code\r\n", api_name, rslt);
        break;
    }
}

/*!
 *  @brief Function to select the interface.
 */
int8_t bmm350_interface_init(struct bmm350_dev *dev, i2c_bus_handle_t i2c_bus)
{
    int8_t rslt = BMM350_OK;

    if (dev == NULL) {
        return BMM350_E_NULL_PTR;
    }

    if (i2c_bus == NULL) {
        ESP_LOGE(TAG, "i2c_bus_create failed");
        return BMM350_E_COM_FAIL;
    }

    s_i2c_bus = i2c_bus;

    dev_addr = BMM350_I2C_ADSEL_SET_LOW; /* default 0x14; caller can change via dev->intf_ptr */
    dev->intf_ptr = &dev_addr;
    dev->read = bmm350_i2c_read;
    dev->write = bmm350_i2c_write;
    dev->delay_us = bmm350_delay;

    return rslt;
}

/* COINES-specific button state helper removed */

void bmm350_coines_deinit(void)
{
    if (s_i2c_dev) {
        i2c_bus_device_delete(&s_i2c_dev);
        s_i2c_dev = NULL;
    }
}

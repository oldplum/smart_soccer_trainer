/**
 * Copyright (C) 2024 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom_sys.h"

#include "bme69x_defs.h"
#include "bme69x.h"

/******************************************************************************/
/*!                         Macro definitions                                 */

/*! BME69X shuttle id */
#define BME69X_SHUTTLE_ID_PRIM  UINT16_C(0x93)
#define BME69X_SHUTTLE_ID_SEC   UINT16_C(0x93)

// I2C timeout configuration in milliseconds
#define I2C_TOOL_TIMEOUT_VALUE_MS (50)

static const char *TAG = "BME69X_ESP32";

/******************************************************************************/
/*!                Static variable definition                                 */

/*! Variable that holds the I2C device address or SPI chip selection */
static uint8_t dev_addr;
static i2c_master_bus_handle_t i2c_bus = NULL;
static i2c_master_dev_handle_t i2c_dev = NULL;

/******************************************************************************/
/*!                User interface functions                                   */

/*!
 * I2C read function map to ESP32 platform
 */
BME69X_INTF_RET_TYPE bme69x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    esp_err_t ret;
    (void)intf_ptr;

    if (i2c_dev == NULL) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return BME69X_E_COM_FAIL;
    }

    ESP_LOGD(TAG, "I2C read reg 0x%02X len %u", reg_addr, len);
    // Write register address and then read data (Repeated START)
    ret = i2c_master_transmit_receive(i2c_dev, &reg_addr, 1, reg_data, len, I2C_TOOL_TIMEOUT_VALUE_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read data failed: %s", esp_err_to_name(ret));
        return BME69X_E_COM_FAIL;
    }
    return BME69X_OK;
}

/*!
 * I2C write function map to ESP32 platform
 */
BME69X_INTF_RET_TYPE bme69x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    esp_err_t ret;
    (void)intf_ptr;

    if (i2c_dev == NULL) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return BME69X_E_COM_FAIL;
    }

    ESP_LOGD(TAG, "I2C write reg 0x%02X len %u", reg_addr, len);
    // Allocate a buffer that contains register address followed by data
    uint8_t *write_buf = malloc(len + 1);
    if (write_buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate write buffer");
        return BME69X_E_COM_FAIL;
    }

    write_buf[0] = reg_addr;
    memcpy(&write_buf[1], reg_data, len);

    // Write data to device
    ret = i2c_master_transmit(i2c_dev, write_buf, len + 1, I2C_TOOL_TIMEOUT_VALUE_MS);
    free(write_buf);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C write data failed: %s", esp_err_to_name(ret));
        return BME69X_E_COM_FAIL;
    }

    return BME69X_OK;
}

/*!
 * SPI read function map to ESP32 platform (not implemented)
 */
BME69X_INTF_RET_TYPE bme69x_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    ESP_LOGE(TAG, "SPI interface not implemented for ESP32");
    return BME69X_E_COM_FAIL;
}

/*!
 * SPI write function map to ESP32 platform (not implemented)
 */
BME69X_INTF_RET_TYPE bme69x_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    ESP_LOGE(TAG, "SPI interface not implemented for ESP32");
    return BME69X_E_COM_FAIL;
}

/*!
 * Delay function map to ESP32 platform
 */
void bme69x_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;

    if (period < 1000) {
        /* Busy wait for periods <1 ms to keep micro-second accuracy */
        esp_rom_delay_us(period);
    } else {
        /* Use RTOS delay for periods ≥1 ms (rounded up) */
        vTaskDelay(pdMS_TO_TICKS((period + 999) / 1000));
    }
}

/*!
 *  @brief Function to select the interface between SPI and I2C.
 */
int8_t bme69x_interface_init(struct bme69x_dev *bme, uint8_t intf)
{
    int8_t rslt = BME69X_OK;

    if (bme != NULL) {
        /* Bus configuration : I2C */
        if (intf == BME69X_I2C_INTF) {
            ESP_LOGI(TAG, "I2C Interface, dev_addr=0x%02X", BME69X_I2C_ADDR_LOW);

            // Note: I2C bus must be initialized externally; set only address and callbacks here
            if (i2c_bus == NULL) {
                ESP_LOGE(TAG, "I2C bus handle is NULL, please call bme69x_set_i2c_bus_handle first");
                return BME69X_E_COM_FAIL;
            }

            /* Use fixed I2C address 0x76; verify by post-add probe */
            esp_err_t esp_rslt;
            dev_addr = BME69X_I2C_ADDR_LOW;
            bme->read = bme69x_i2c_read;
            bme->write = bme69x_i2c_write;
            bme->intf = BME69X_I2C_INTF;

            // Create I2C device handle
            i2c_device_config_t dev_cfg = {
                .dev_addr_length = I2C_ADDR_BIT_7,
                .device_address  = dev_addr,
                .scl_speed_hz    = 100000,
                .flags.disable_ack_check = 0   /* Enable ACK check */
            };

            // tiny settle delay before add
            vTaskDelay(pdMS_TO_TICKS(10));

            esp_rslt = i2c_master_bus_add_device(i2c_bus, &dev_cfg, &i2c_dev);
            if (esp_rslt != ESP_OK) {
                ESP_LOGE(TAG, "i2c_master_bus_add_device failed: %s", esp_err_to_name(esp_rslt));
                return BME69X_E_COM_FAIL;
            }
            ESP_LOGI(TAG, "i2c_dev handle created: %p", i2c_dev);

            /* add after probe */
            esp_rslt = i2c_master_probe(i2c_bus, dev_addr, 200);
            ESP_LOGI(TAG, "post-add probe(0x%02X) -> %s", dev_addr, esp_err_to_name(esp_rslt));
            if (esp_rslt != ESP_OK) {
                return BME69X_E_DEV_NOT_FOUND;
            }
        }
        /* Bus configuration : SPI */
        else if (intf == BME69X_SPI_INTF) {
            ESP_LOGI(TAG, "SPI Interface (not implemented for ESP32)");
            return BME69X_E_COM_FAIL;
        }

        /* Holds the I2C device addr or SPI chip selection */
        bme->intf_ptr = &dev_addr;

        /* Configure delay in microseconds */
        bme->delay_us = bme69x_delay_us;

        /* Configure max read/write length (in bytes) ( Supported length depends on target machine) */
        // bme->read_write_len = 8;  // Not available in BME69X

        /* Assign to NULL to load the default config file. */
        // bme->config_file_ptr = NULL;  // Not available in BME69X

        /* Set ambient temperature */
        bme->amb_temp = 25; /* The ambient temperature in deg C is used for defining the heater temperature */
    } else {
        rslt = BME69X_E_NULL_PTR;
    }

    return rslt;
}

/*!
 *  @brief Prints the execution status of the APIs.
 */
void bme69x_check_rslt(const char api_name[], int8_t rslt)
{
    switch (rslt) {
    case BME69X_OK:
        /* Do nothing */
        break;
    case BME69X_E_NULL_PTR:
        ESP_LOGE(TAG, "API [%s] Error [%d] : Null pointer", api_name, rslt);
        break;
    case BME69X_E_COM_FAIL:
        ESP_LOGE(TAG, "API [%s] Error [%d] : Communication failure", api_name, rslt);
        break;
    case BME69X_E_INVALID_LENGTH:
        ESP_LOGE(TAG, "API [%s] Error [%d] : Incorrect length parameter", api_name, rslt);
        break;
    case BME69X_E_DEV_NOT_FOUND:
        ESP_LOGE(TAG, "API [%s] Error [%d] : Device not found", api_name, rslt);
        break;
    case BME69X_E_SELF_TEST:
        ESP_LOGE(TAG, "API [%s] Error [%d] : Self test error", api_name, rslt);
        break;
    case BME69X_W_NO_NEW_DATA:
        ESP_LOGW(TAG, "API [%s] Warning [%d] : No new data found", api_name, rslt);
        break;
    default:
        ESP_LOGE(TAG, "API [%s] Error [%d] : Unknown error code", api_name, rslt);
        break;
    }
}

/*!
 * @brief Set I2C bus handle for ESP32 platform
 */
void bme69x_set_i2c_bus_handle(i2c_master_bus_handle_t bus_handle)
{
    i2c_bus = bus_handle;
    ESP_LOGI(TAG, "I2C bus handle set");
}

/*!
 *  @brief Deinitializes coines platform
 */
void bme69x_coines_deinit(void)
{
    ESP_LOGI(TAG, "BME69X ESP32 deinit");

    // clean up I2C bus
    if (i2c_dev) {
        i2c_master_bus_rm_device(i2c_dev);
        i2c_dev = NULL;
    }
}

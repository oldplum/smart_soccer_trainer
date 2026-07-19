/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "esp_brookesia_app_max30102.hpp"
#include "esp_err.h"
#include "esp_lib_utils.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "i2c_bus.h"
#include "esp_board_manager.h"
#include "soccer_data_sync.h"

static const char *TAG = "Max30102";

using namespace esp_brookesia::systems::phone;

/* MAX30102 I2C Configuration */
#define MAX30102_I2C_ADDR       0x57
#define MAX30102_I2C_PORT       I2C_NUM_0

/* MAX30102 Register Addresses */
#define MAX30102_REG_INTR_STATUS_1  0x00
#define MAX30102_REG_INTR_STATUS_2  0x01
#define MAX30102_REG_INTR_ENABLE_1  0x02
#define MAX30102_REG_INTR_ENABLE_2  0x03
#define MAX30102_REG_FIFO_WR_PTR    0x04
#define MAX30102_REG_FIFO_OVERFLOW  0x05
#define MAX30102_REG_FIFO_RD_PTR    0x06
#define MAX30102_REG_FIFO_DATA      0x07
#define MAX30102_REG_FIFO_CONFIG    0x08
#define MAX30102_REG_MODE_CONFIG    0x09
#define MAX30102_REG_SPO2_CONFIG    0x0A
#define MAX30102_REG_LED1_PA        0x0C
#define MAX30102_REG_LED2_PA        0x0D
#define MAX30102_REG_PILOT_PA       0x10
#define MAX30102_REG_MULTI_LED1     0x11
#define MAX30102_REG_MULTI_LED2     0x12
#define MAX30102_REG_TINT           0x1F
#define MAX30102_REG_TFRAC          0x20
#define MAX30102_REG_TEMP_CONFIG    0x21
#define MAX30102_REG_REV_ID         0xFE
#define MAX30102_REG_PART_ID        0xFF

/* Mode Configuration Bits */
#define MAX30102_MODE_SHDN_BIT      (1 << 7)
#define MAX30102_MODE_RESET_BIT     (1 << 6)
#define MAX30102_MODE_SPO2          0x03
#define MAX30102_MODE_HR_ONLY       0x02

/* FIFO Configuration */
#define MAX30102_FIFO_SMP_AVE_1     (0 << 5)
#define MAX30102_FIFO_SMP_AVE_2     (1 << 5)
#define MAX30102_FIFO_SMP_AVE_4     (2 << 5)
#define MAX30102_FIFO_SMP_AVE_8     (3 << 5)
#define MAX30102_FIFO_SMP_AVE_16    (4 << 5)
#define MAX30102_FIFO_SMP_AVE_32    (5 << 5)
#define MAX30102_FIFO_ROLLOVER_EN   (1 << 4)
#define MAX30102_FIFO_A_FULL_17     (0 << 0)

/* SPO2 Configuration */
#define MAX30102_SPO2_ADC_RANGE_4096   (0 << 5)
#define MAX30102_SPO2_SR_50           (1 << 2)
#define MAX30102_SPO2_LED_PW_411      (0x3 << 0)

namespace esp_brookesia::apps {

/* --------------------------------------------------------------------------
 * Brookesia App metadata (no UI - data collection only)
 * -------------------------------------------------------------------------- */
constexpr systems::base::App::Config MAX30102_CORE_DATA = {
    .name = "HeartRate",
    .launcher_icon = gui::StyleImage::IMAGE(nullptr),
    .screen_size = gui::StyleSize::RECT_PERCENT(100, 100),
    .flags = {
        .enable_default_screen = 1,
        .enable_recycle_resource = 0,
        .enable_resize_visual_area = 1,
    },
};
constexpr App::Config MAX30102_APP_DATA = {
    .app_launcher_page_index = 0,
    .flags = {
        .enable_navigation_gesture = 1,
    },
};

/* --------------------------------------------------------------------------
 * Constructor / Destructor
 * -------------------------------------------------------------------------- */
Max30102::Max30102()
    : App(MAX30102_CORE_DATA, MAX30102_APP_DATA) {
    ESP_LOGI(TAG, "Max30102 app constructor");
}

Max30102::~Max30102() {
    ESP_UTILS_LOG_TRACE_GUARD_WITH_THIS();
    ESP_LOGI(TAG, "Max30102 app destructor");
    deinit();
}

Max30102 *Max30102::requestInstance() {
    if (_instance == nullptr) {
        _instance = new Max30102();
    }
    return _instance;
}

/* --------------------------------------------------------------------------
 * App lifecycle (Brookesia framework callbacks)
 * -------------------------------------------------------------------------- */
bool Max30102::init() {
    ESP_LOGI(TAG, "Initializing Max30102 app");
    return true;
}

bool Max30102::deinit() {
    ESP_LOGI(TAG, "Deinitializing Max30102 app");
    stopSensorCollection();
    return true;
}

bool Max30102::run() {
    ESP_LOGI(TAG, "Running Max30102 app");
    return true;
}

bool Max30102::back() {
    ESP_LOGI(TAG, "Max30102 app back");
    return close();
}

bool Max30102::close() {
    ESP_LOGI(TAG, "Closing Max30102 app");
    stopSensorCollection();
    return true;
}

bool Max30102::pause() {
    ESP_LOGI(TAG, "Max30102 app pause");
    return true;
}

bool Max30102::resume() {
    ESP_LOGI(TAG, "Max30102 app resume");
    return true;
}

/* --------------------------------------------------------------------------
 * Sensor operations
 * -------------------------------------------------------------------------- */
bool Max30102::writeRegister(uint8_t reg, uint8_t data) {
    if (i2c_bus_ == nullptr) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return false;
    }

    if (i2c_dev_ == nullptr) {
        i2c_dev_ = i2c_bus_device_create(i2c_bus_, MAX30102_I2C_ADDR, 0);
        if (i2c_dev_ == nullptr) {
            ESP_LOGE(TAG, "Failed to create I2C device");
            return false;
        }
    }

    esp_err_t ret = i2c_bus_write_bytes(i2c_dev_, reg, 1, &data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write register 0x%02X: %s", reg, esp_err_to_name(ret));
        return false;
    }

    return true;
}

bool Max30102::readRegister(uint8_t reg, uint8_t *data) {
    if (i2c_bus_ == nullptr) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return false;
    }

    if (i2c_dev_ == nullptr) {
        i2c_dev_ = i2c_bus_device_create(i2c_bus_, MAX30102_I2C_ADDR, 0);
        if (i2c_dev_ == nullptr) {
            ESP_LOGE(TAG, "Failed to create I2C device");
            return false;
        }
    }

    esp_err_t ret = i2c_bus_read_bytes(i2c_dev_, reg, 1, data);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read register 0x%02X: %s", reg, esp_err_to_name(ret));
        return false;
    }

    return true;
}

bool Max30102::readFIFO(uint32_t *red, uint32_t *ir) {
    if (i2c_bus_ == nullptr) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return false;
    }

    if (i2c_dev_ == nullptr) {
        i2c_dev_ = i2c_bus_device_create(i2c_bus_, MAX30102_I2C_ADDR, 0);
        if (i2c_dev_ == nullptr) {
            ESP_LOGE(TAG, "Failed to create I2C device");
            return false;
        }
    }

    uint8_t data[6];
    esp_err_t ret = i2c_bus_read_bytes(i2c_dev_, MAX30102_REG_FIFO_DATA, 6, data);
    if (ret != ESP_OK) {
        return false;
    }

    *red = ((uint32_t)data[0] << 16) | ((uint32_t)data[1] << 8) | data[2];
    *red &= 0x03FFFF; /* 18-bit */
    
    *ir = ((uint32_t)data[3] << 16) | ((uint32_t)data[4] << 8) | data[5];
    *ir &= 0x03FFFF;

    return true;
}

/* --------------------------------------------------------------------------
 * Sensor initialization
 * -------------------------------------------------------------------------- */
bool Max30102::initSensors() {
    ESP_LOGI(TAG, "Initializing MAX30102 sensors...");

    vTaskDelay(10 / portTICK_PERIOD_MS);

    // Get I2C pin configuration from Board Manager
    i2c_master_bus_config_t *i2c_config = nullptr;
    esp_err_t ret = esp_board_manager_get_periph_config("i2c_master", (void **)&i2c_config);
    if (ret != ESP_OK || i2c_config == nullptr) {
        ESP_LOGE(TAG, "Failed to get I2C peripheral config from Board Manager: %s", esp_err_to_name(ret));
        return false;
    }

    gpio_num_t sda_pin = i2c_config->sda_io_num;
    gpio_num_t scl_pin = i2c_config->scl_io_num;

    const i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {.clk_speed = 400000},
        .clk_flags = 0,
    };

    i2c_bus_ = i2c_bus_create(I2C_NUM_0, &i2c_conf);
    if (!i2c_bus_) {
        ESP_LOGE(TAG, "I2C bus create failed");
        return false;
    }
    ESP_LOGI(TAG, "I2C bus handle: %p", i2c_bus_);

    vTaskDelay(10 / portTICK_PERIOD_MS);

    /* Reset sensor */
    if (!writeRegister(MAX30102_REG_MODE_CONFIG, MAX30102_MODE_RESET_BIT)) {
        return false;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Verify Part ID */
    uint8_t part_id;
    if (!readRegister(MAX30102_REG_PART_ID, &part_id)) {
        return false;
    }
    
    ESP_LOGI(TAG, "MAX30102 Part ID: 0x%02X", part_id);

    /* Configure FIFO */
    uint8_t fifo_config = MAX30102_FIFO_SMP_AVE_4 | 
                          MAX30102_FIFO_ROLLOVER_EN | 
                          MAX30102_FIFO_A_FULL_17;
    if (!writeRegister(MAX30102_REG_FIFO_CONFIG, fifo_config)) {
        return false;
    }

    /* Configure SpO2 settings - must be done BEFORE entering SpO2 mode */
    uint8_t spo2_config = MAX30102_SPO2_ADC_RANGE_4096 | 
                          MAX30102_SPO2_SR_50 | 
                          MAX30102_SPO2_LED_PW_411;
    if (!writeRegister(MAX30102_REG_SPO2_CONFIG, spo2_config)) {
        return false;
    }

    /* Set LED current - 0x3F = ~12.6mA (visible glow) */
    if (!writeRegister(MAX30102_REG_LED1_PA, 0x3F)) { /* Red LED */
        return false;
    }
    if (!writeRegister(MAX30102_REG_LED2_PA, 0x3F)) { /* IR LED */
        return false;
    }

    /* Clear FIFO pointers */
    if (!writeRegister(MAX30102_REG_FIFO_WR_PTR, 0) ||
        !writeRegister(MAX30102_REG_FIFO_RD_PTR, 0) ||
        !writeRegister(MAX30102_REG_FIFO_OVERFLOW, 0)) {
        return false;
    }

    /* Configure Mode - SpO2 mode (both LEDs active) - MUST be last step */
    if (!writeRegister(MAX30102_REG_MODE_CONFIG, MAX30102_MODE_SPO2)) {
        return false;
    }

    ESP_LOGI(TAG, "MAX30102 sensor initialized successfully");
    return true;
}

bool Max30102::deinitSensors() {
    if (i2c_dev_) {
        i2c_bus_device_delete(&i2c_dev_);
        i2c_dev_ = nullptr;
    }
    if (i2c_bus_) {
        i2c_bus_delete(&i2c_bus_);
        i2c_bus_ = nullptr;
    }
    return true;
}

/* --------------------------------------------------------------------------
 * Public sensor control API
 * -------------------------------------------------------------------------- */
bool Max30102::startSensorCollection() {
    ESP_LOGI(TAG, "Starting MAX30102 sensor collection...");

    sensor_online_ = false;

    if (!initSensors()) {
        ESP_LOGE(TAG, "Failed to initialize sensors");
        return false;
    }

    running_ = true;
    data_thread_ = boost::thread(&Max30102::max30102DataThread, this);
    sensor_online_ = true;

    ESP_LOGI(TAG, "MAX30102 sensor collection started");
    return true;
}

void Max30102::stopSensorCollection() {
    ESP_LOGI(TAG, "Stopping MAX30102 sensor collection");
    sensor_online_ = false;
    running_ = false;
    if (data_thread_.joinable()) {
        data_thread_.join();
    }
    deinitSensors();
    ESP_LOGI(TAG, "MAX30102 sensor collection stopped");
}

bool Max30102::isSensorOnline() const {
    return sensor_online_.load(std::memory_order_relaxed);
}

/* --------------------------------------------------------------------------
 * Data acquisition thread
 * -------------------------------------------------------------------------- */
void Max30102::max30102DataThread() {
    ESP_LOGI(TAG, "MAX30102 data thread started");

    static uint32_t last_status_log = 0;
    const uint32_t STATUS_LOG_INTERVAL = 5000; /* ms */

    while (running_) {
        uint32_t red, ir;
        
        if (readFIFO(&red, &ir)) {
            /* Simple heart rate and SpO2 calculation placeholder */
            /* In a real implementation, you would use an algorithm to process raw data */
            
            heart_rate_ = 72; /* Dummy value */
            spo2_ = 98;      /* Dummy value */

            /* Update global sensor data buffer */
            g_soccer_sensor_data.heart_rate = heart_rate_;
            g_soccer_sensor_data.spo2 = spo2_;
            g_soccer_sensor_data.hr_valid = 1;
            g_soccer_sensor_data.timestamp = (uint32_t)(esp_timer_get_time() / 1000);

            /* Periodic status logging */
            uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
            if (now - last_status_log > STATUS_LOG_INTERVAL) {
                ESP_LOGI(TAG, "Heart Rate: %d bpm, SpO2: %d%%, Raw Red: %lu, Raw IR: %lu", 
                         heart_rate_, spo2_, (unsigned long)red, (unsigned long)ir);
                last_status_log = now;
            }
        } else {
            if (sensor_online_.load()) {
                ESP_LOGW(TAG, "Failed to read FIFO data");
                sensor_online_ = false;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    ESP_LOGI(TAG, "MAX30102 data thread stopped");
}

/* --------------------------------------------------------------------------
 * Plugin registration (Brookesia framework)
 * -------------------------------------------------------------------------- */
ESP_UTILS_REGISTER_PLUGIN_WITH_CONSTRUCTOR(systems::base::App, Max30102, "HeartRate", []() {
    return std::shared_ptr<Max30102>(Max30102::requestInstance(), [](Max30102 *p) {});
});

} // namespace esp_brookesia::apps
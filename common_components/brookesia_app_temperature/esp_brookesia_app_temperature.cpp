/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "bme69x_defs.h"
#include "bme69x.h"
#include "bsec_datatypes.h"
#include "bsec_interface.h"
#include "common.h"
#include "esp_brookesia_app_temperature.hpp"
#include "esp_err.h"
#include "esp_lib_utils.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "math.h"
#include "ui/ui.h"
#include "esp_board_manager.h"

static const char *TAG = "TemperatureApp";

using namespace esp_brookesia::systems::phone;

#define APP_NAME "Environment"

#define I2C_MASTER_NUM                  (I2C_NUM_0)
#define I2C_MASTER_FREQ_HZ              (100 * 1000)
#define BME690_SDO_PIN                  (GPIO_NUM_9)

// BSEC configuration
// Available sample rates:
//   BSEC_SAMPLE_RATE_ULP  (0.0033333f) = Ultra Low Power, once every 300 seconds (5 minutes)
//   BSEC_SAMPLE_RATE_LP   (0.33333f)   = Low Power, once every 3 seconds
//   BSEC_SAMPLE_RATE_CONT (1.0f)       = Continuous, once every 1 second
#define BSEC_SAMPLE_RATE            BSEC_SAMPLE_RATE_LP  // Low Power mode (once every 3 seconds)
#define TEMP_OFFSET                 5.0f                 // Temperature offset for self-heating compensation
#define STATE_SAVE_PERIOD           UINT32_C(360 * 60 * 1000000)  // 6 hours in microseconds

LV_IMG_DECLARE(img_app_temperature);

namespace esp_brookesia::apps {

constexpr systems::base::App::Config CORE_DATA = {
    .name = APP_NAME,
    .launcher_icon = gui::StyleImage::IMAGE(&img_app_temperature),
    .screen_size = gui::StyleSize::RECT_PERCENT(100, 100),
    .flags = {
        .enable_default_screen = 1,
        .enable_recycle_resource = 0,
        .enable_resize_visual_area = 1,
    },
};
constexpr App::Config APP_DATA = {
    .app_launcher_page_index = 0,
    .flags = {
        .enable_navigation_gesture = 1,
    },
};

Temperature::Temperature()
    : App(CORE_DATA, APP_DATA)
{
    ESP_LOGI(TAG, "Temperature app constructor");
    // Initialize humidity history array
    for (int i = 0; i < HUMIDITY_AVG_COUNT; i++) {
        humidity_history[i] = 0.0f;
    }
    humidity_history_index = 0;
    humidity_history_count = 0;
}

Temperature::~Temperature()
{
    ESP_UTILS_LOG_TRACE_GUARD_WITH_THIS();
    ESP_LOGI(TAG, "Temperature app destructor");
    deinit();
}

Temperature *Temperature::requestInstance()
{
    if (_instance == nullptr) {
        _instance = new Temperature();
    }
    return _instance;
}

bool Temperature::init()
{
    ESP_LOGI(TAG, "Initializing Temperature app");
    // Sensor initialization is done in run() method
    return true;
}

bool Temperature::deinit()
{
    ESP_LOGI(TAG, "Deinitializing Temperature app");

    destroyTemperatureUI();
    deinitSensors();

    _bme_running = false;
    if (_bme_thread.joinable()) {
        _bme_thread.join();
    }
    i2c_del_master_bus(i2c_bus_);
    i2c_bus_ = nullptr;
    return true;
}

bool Temperature::run()
{
    ESP_LOGI(TAG, "Running Temperature app");

    if (!initSensors()) {
        ESP_LOGE(TAG, "Failed to initialize sensors");
        is_initialized_ = false;
    } else {
        if (init_bsec() != BSEC_OK) {
            ESP_LOGE(TAG, "Failed to initialize BSEC");
            is_initialized_ = false;
        } else {
            is_initialized_ = true;
        }
    }

    createTemperatureUI();

    if (!is_initialized_) {
        return true;
    }

    _bme_running = true;
    boost::thread::attributes thread_attrs;
    thread_attrs.set_stack_size(4 * 1024);
    _bme_thread = boost::thread(thread_attrs, boost::bind(&Temperature::bmeDataThread, this));

    return true;
}

bool Temperature::back()
{
    ESP_LOGI(TAG, "Temperature app back");
    return close();
}

bool Temperature::close()
{
    ESP_LOGI(TAG, "Closing Temperature app");
    _bme_running = false;
    if (_bme_thread.joinable()) {
        _bme_thread.join();
    }
    deinitSensors();

    return true;
}

void Temperature::createTemperatureUI()
{
    ESP_LOGI(TAG, "Creating Temperature UI");
    Temperature_ui_init(is_initialized_);
    ESP_LOGI(TAG, "Temperature UI created successfully");
}

void Temperature::destroyTemperatureUI()
{
    Temperature_ui_destroy();
    ESP_LOGI(TAG, "Temperature UI destroyed");
}

esp_err_t Temperature::init_hardware()
{
    esp_err_t ret;
    gpio_config_t sdo_conf = {
        .pin_bit_mask = (1ULL << BME690_SDO_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ret = gpio_config(&sdo_conf);
    if (ret != ESP_OK) {
        return ret;
    }
    gpio_set_level((gpio_num_t)BME690_SDO_PIN, 0);

    ret = i2c_master_get_bus_handle(I2C_MASTER_NUM, &i2c_bus_);
    if (ret != ESP_OK) {
        return ret;
    }
    ESP_LOGI(TAG, "I2C bus handle: %p", i2c_bus_);
    if (i2c_bus_) {
        ESP_LOGI(TAG, "I2C bus get success");
        return ESP_OK;
    }

    /* SDO low -> 0x76 */

    // Get I2C pin configuration from Board Manager
    i2c_master_bus_config_t *i2c_config = nullptr;
    ret = esp_board_manager_get_periph_config("i2c_master", (void **)&i2c_config);
    if (ret != ESP_OK || i2c_config == nullptr) {
        ESP_LOGE(TAG, "Failed to get I2C peripheral config from Board Manager: %s", esp_err_to_name(ret));
        return ESP_FAIL;
    }

    gpio_num_t sda_pin = i2c_config->sda_io_num;
    gpio_num_t scl_pin = i2c_config->scl_io_num;

    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true
        }
    };
    ret = i2c_new_master_bus(&i2c_bus_config, &i2c_bus_);
    if (ret != ESP_OK) {
        return ret;
    }
    ESP_LOGI(TAG, "I2C bus initialized - SCL: GPIO%d, SDA: GPIO%d", scl_pin, sda_pin);
    return ESP_OK;
}

esp_err_t Temperature::init_bme69x_sensor()
{
    ESP_LOGI(TAG, "Initializing BME69x...");
    int8_t rslt;
    struct bme69x_conf conf;
    struct bme69x_heatr_conf heatr_conf;

    bme69x_set_i2c_bus_handle(i2c_bus_);

    rslt = bme69x_interface_init(&bme_, BME69X_I2C_INTF);
    bme69x_check_rslt("bme69x_interface_init", rslt);
    if (rslt != BME69X_OK) {
        ESP_LOGE(TAG, "BME690 interface initialization failed: %d", rslt);
        return ESP_FAIL;
    }

    rslt = bme69x_init(&bme_);
    bme69x_check_rslt("bme69x_init", rslt);
    if (rslt != BME69X_OK) {
        ESP_LOGE(TAG, "BME690 sensor initialization failed: %d", rslt);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Chip ID: 0x%02x", bme_.chip_id);

    conf.filter = BME69X_FILTER_OFF;
    conf.odr = BME69X_ODR_NONE;
    conf.os_hum = BME69X_OS_16X;
    conf.os_pres = BME69X_OS_16X;
    conf.os_temp = BME69X_OS_16X;
    rslt = bme69x_set_conf(&conf, &bme_);
    bme69x_check_rslt("bme69x_set_conf", rslt);
    if (rslt != BME69X_OK) {
        ESP_LOGE(TAG, "Failed to set sensor configuration: %d", rslt);
        return ESP_FAIL;
    }

    heatr_conf.enable = BME69X_ENABLE;
    heatr_conf.heatr_temp = 300;
    heatr_conf.heatr_dur = 100;
    rslt = bme69x_set_heatr_conf(BME69X_FORCED_MODE, &heatr_conf, &bme_);
    bme69x_check_rslt("bme69x_set_heatr_conf", rslt);
    if (rslt != BME69X_OK) {
        ESP_LOGE(TAG, "Failed to set heater configuration: %d", rslt);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Sample, Temperature(deg C), Pressure(Pa), Humidity(%%), Gas "
             "resistance(ohm)");
    return ESP_OK;
}

bool Temperature::initSensors()
{
    if (init_hardware() != ESP_OK) {
        ESP_LOGE(TAG, "Hardware initialization failed");
        return false;
    }

    if (init_bme69x_sensor() != ESP_OK) {
        ESP_LOGE(TAG, "BME69x sensor initialization failed");
        return false;
    }

    return true;
}

bsec_library_return_t Temperature::init_bsec(void)
{
    bsec_library_return_t bsec_status;

    bsec_status = bsec_init();
    if (bsec_status != BSEC_OK) {
        ESP_LOGE(TAG, "BSEC initialization failed: %d", bsec_status);
        return bsec_status;
    }

    bsec_sensor_configuration_t requested_virtual_sensors[8];
    uint8_t n_requested_virtual_sensors = 8;

    requested_virtual_sensors[0].sensor_id = BSEC_OUTPUT_IAQ;
    requested_virtual_sensors[0].sample_rate = BSEC_SAMPLE_RATE;

    requested_virtual_sensors[1].sensor_id = BSEC_OUTPUT_STATIC_IAQ;
    requested_virtual_sensors[1].sample_rate = BSEC_SAMPLE_RATE;

    requested_virtual_sensors[2].sensor_id = BSEC_OUTPUT_CO2_EQUIVALENT;
    requested_virtual_sensors[2].sample_rate = BSEC_SAMPLE_RATE;

    requested_virtual_sensors[3].sensor_id = BSEC_OUTPUT_BREATH_VOC_EQUIVALENT;
    requested_virtual_sensors[3].sample_rate = BSEC_SAMPLE_RATE;

    requested_virtual_sensors[4].sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE;
    requested_virtual_sensors[4].sample_rate = BSEC_SAMPLE_RATE;

    requested_virtual_sensors[5].sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY;
    requested_virtual_sensors[5].sample_rate = BSEC_SAMPLE_RATE;

    requested_virtual_sensors[6].sensor_id = BSEC_OUTPUT_RAW_PRESSURE;
    requested_virtual_sensors[6].sample_rate = BSEC_SAMPLE_RATE;

    requested_virtual_sensors[7].sensor_id = BSEC_OUTPUT_RAW_GAS;
    requested_virtual_sensors[7].sample_rate = BSEC_SAMPLE_RATE;

    bsec_sensor_configuration_t required_sensor_settings[BSEC_MAX_PHYSICAL_SENSOR];
    uint8_t n_required_sensor_settings = BSEC_MAX_PHYSICAL_SENSOR;

    bsec_status = bsec_update_subscription(requested_virtual_sensors, n_requested_virtual_sensors,
                                           required_sensor_settings, &n_required_sensor_settings);

    if (bsec_status == BSEC_W_SU_SAMPLERATEMISMATCH) {
        ESP_LOGW(TAG, "BSEC sample rate mismatch warning (code 14)");
        ESP_LOGW(TAG, "Using LP mode (3s interval) with default config - BSEC will adapt automatically");
        // This is just a warning, continue execution
    } else if (bsec_status != BSEC_OK && bsec_status > 0) {
        // Positive values are warnings, continue
        ESP_LOGW(TAG, "BSEC subscription warning: %d", bsec_status);
    } else if (bsec_status < 0) {
        // Negative values are errors, stop
        ESP_LOGE(TAG, "BSEC subscription failed with error: %d", bsec_status);
        return bsec_status;
    }

    ESP_LOGI(TAG, "BSEC initialized successfully");
    ESP_LOGI(TAG, "Required sensor settings: %d", n_required_sensor_settings);

    return BSEC_OK;
}

bool Temperature::deinitSensors()
{
    ESP_LOGI(TAG, "Deinitializing sensors...");
    bme69x_coines_deinit();
    i2c_del_master_bus(i2c_bus_);
    i2c_bus_ = nullptr;
    return true;
}

void Temperature::output_ready(int64_t timestamp, float iaq, uint8_t iaq_accuracy,
                               float temperature, float humidity, float pressure,
                               float raw_temperature, float raw_humidity,
                               float gas, bsec_library_return_t bsec_status,
                               float static_iaq, float co2_equivalent,
                               float breath_voc_equivalent)
{
    this->iaq = iaq;
    this->iaq_accuracy = iaq_accuracy;
    this->co2_equivalent = co2_equivalent;
    this->breath_voc_equivalent = breath_voc_equivalent;
    this->bsec_temperature = temperature;
    this->bsec_humidity = humidity;
    this->bsec_pressure = pressure;
}

void Temperature::bmeDataThread()
{
    ESP_LOGI(TAG, "BME69x sensor data thread started with BSEC integration");

    int8_t bme_rslt;
    bsec_library_return_t bsec_status;

    bsec_bme_settings_t sensor_settings;
    bsec_input_t bsec_inputs[BSEC_MAX_PHYSICAL_SENSOR];
    uint8_t n_bsec_inputs = 0;
    bsec_output_t bsec_outputs[BSEC_NUMBER_OUTPUTS];
    uint8_t n_bsec_outputs = 0;

    time_stamp_last_state_save = get_timestamp_us();

    while (_bme_running) {
        int64_t curr_time_ns = get_timestamp_us() * 1000;

        bsec_status = bsec_sensor_control(curr_time_ns, &sensor_settings);
        if (bsec_status != BSEC_OK) {
            ESP_LOGW(TAG, "BSEC sensor control failed: %d", bsec_status);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        if (sensor_settings.trigger_measurement) {
            // Configure BME68x sensor based on BSEC requirements
            struct bme69x_conf conf;
            conf.filter = BME69X_FILTER_OFF;
            conf.odr = BME69X_ODR_NONE;
            conf.os_hum = sensor_settings.humidity_oversampling;
            conf.os_pres = sensor_settings.pressure_oversampling;
            conf.os_temp = sensor_settings.temperature_oversampling;

            bme_rslt = bme69x_set_conf(&conf, &bme_);
            if (bme_rslt != BME69X_OK) {
                ESP_LOGE(TAG, "bme69x_set_conf failed: %d", bme_rslt);
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }

            struct bme69x_heatr_conf heatr_conf;
            heatr_conf.enable = BME69X_ENABLE;
            heatr_conf.heatr_temp = sensor_settings.heater_temperature;
            heatr_conf.heatr_dur = sensor_settings.heater_duration;

            bme_rslt = bme69x_set_heatr_conf(BME69X_FORCED_MODE, &heatr_conf, &bme_);
            if (bme_rslt != BME69X_OK) {
                ESP_LOGE(TAG, "bme68x_set_heatr_conf failed: %d", bme_rslt);
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }

            bme_rslt = bme69x_set_op_mode(BME69X_FORCED_MODE, &bme_);
            if (bme_rslt != BME69X_OK) {
                ESP_LOGE(TAG, "bme69x_set_op_mode failed: %d", bme_rslt);
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }

            uint32_t del_period = bme69x_get_meas_dur(BME69X_FORCED_MODE, &conf, &bme_) + (heatr_conf.heatr_dur * 1000);
            bme_.delay_us(del_period, bme_.intf_ptr);
            uint8_t n_data = 0;
            bme_rslt = bme69x_get_data(BME69X_FORCED_MODE, &data, &n_data, &bme_);
            if (bme_rslt != BME69X_OK || n_data == 0) {
                ESP_LOGW(TAG, "bme68x_get_data failed or no data: %d, n_data: %d", bme_rslt, n_data);
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }

            if (!(data.status & BME69X_GASM_VALID_MSK)) {
                ESP_LOGW(TAG, "Gas measurement not valid, skipping");
                vTaskDelay(pdMS_TO_TICKS(100));
                continue;
            }

            n_bsec_inputs = 0;
            int64_t time_stamp_ns = get_timestamp_us() * 1000;

            if (sensor_settings.process_data & BSEC_PROCESS_TEMPERATURE) {
                bsec_inputs[n_bsec_inputs].sensor_id = BSEC_INPUT_TEMPERATURE;
                bsec_inputs[n_bsec_inputs].signal = data.temperature;
                bsec_inputs[n_bsec_inputs].time_stamp = time_stamp_ns;
                n_bsec_inputs++;

                // Add heatsource input for temperature offset
                bsec_inputs[n_bsec_inputs].sensor_id = BSEC_INPUT_HEATSOURCE;
                bsec_inputs[n_bsec_inputs].signal = TEMP_OFFSET;
                bsec_inputs[n_bsec_inputs].time_stamp = time_stamp_ns;
                n_bsec_inputs++;
            }

            if (sensor_settings.process_data & BSEC_PROCESS_HUMIDITY) {
                bsec_inputs[n_bsec_inputs].sensor_id = BSEC_INPUT_HUMIDITY;
                bsec_inputs[n_bsec_inputs].signal = data.humidity;
                bsec_inputs[n_bsec_inputs].time_stamp = time_stamp_ns;
                n_bsec_inputs++;
            }

            if (sensor_settings.process_data & BSEC_PROCESS_PRESSURE) {
                bsec_inputs[n_bsec_inputs].sensor_id = BSEC_INPUT_PRESSURE;
                bsec_inputs[n_bsec_inputs].signal = data.pressure;
                bsec_inputs[n_bsec_inputs].time_stamp = time_stamp_ns;
                n_bsec_inputs++;
            }

            if (sensor_settings.process_data & BSEC_PROCESS_GAS) {
                bsec_inputs[n_bsec_inputs].sensor_id = BSEC_INPUT_GASRESISTOR;
                bsec_inputs[n_bsec_inputs].signal = data.gas_resistance;
                bsec_inputs[n_bsec_inputs].time_stamp = time_stamp_ns;
                n_bsec_inputs++;
            }

            n_bsec_outputs = BSEC_NUMBER_OUTPUTS;
            bsec_status = bsec_do_steps(bsec_inputs, n_bsec_inputs, bsec_outputs, &n_bsec_outputs);

            if (bsec_status != BSEC_OK) {
                ESP_LOGW(TAG, "bsec_do_steps failed: %d", bsec_status);
            } else if (n_bsec_outputs > 0) {
                float iaq = 0, static_iaq = 0, co2_eq = 0, breath_voc = 0;
                float temperature = 0, humidity = 0, pressure = 0;
                float raw_temp = 0, raw_hum = 0, gas = 0;
                uint8_t iaq_acc = 0;

                for (uint8_t i = 0; i < n_bsec_outputs; i++) {
                    switch (bsec_outputs[i].sensor_id) {
                    case BSEC_OUTPUT_IAQ:
                        iaq = bsec_outputs[i].signal;
                        iaq_acc = bsec_outputs[i].accuracy;
                        break;
                    case BSEC_OUTPUT_STATIC_IAQ:
                        static_iaq = bsec_outputs[i].signal;
                        break;
                    case BSEC_OUTPUT_CO2_EQUIVALENT:
                        co2_eq = bsec_outputs[i].signal;
                        break;
                    case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
                        breath_voc = bsec_outputs[i].signal;
                        break;
                    case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
                        temperature = bsec_outputs[i].signal;
                        break;
                    case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
                        humidity = bsec_outputs[i].signal;
                        break;
                    case BSEC_OUTPUT_RAW_PRESSURE:
                        pressure = bsec_outputs[i].signal;
                        break;
                    case BSEC_OUTPUT_RAW_GAS:
                        gas = bsec_outputs[i].signal;
                        break;
                    case BSEC_OUTPUT_RAW_TEMPERATURE:
                        raw_temp = bsec_outputs[i].signal;
                        break;
                    case BSEC_OUTPUT_RAW_HUMIDITY:
                        raw_hum = bsec_outputs[i].signal;
                        break;
                    }
                }

                output_ready(time_stamp_ns / 1000, iaq, iaq_acc, temperature, humidity,
                             pressure, raw_temp, raw_hum, gas, bsec_status,
                             static_iaq, co2_eq, breath_voc);

                if (_bme_running) {
                    lv_async_call([](void *user_data) {
                        Temperature *self = static_cast<Temperature *>(user_data);
                        if (!self->_bme_running) {
                            return;
                        }
                        self->updateTemperatureDisplay();
                    }, this);
                }
            }

            int64_t curr_time_us = get_timestamp_us();
            if ((curr_time_us - time_stamp_last_state_save) >= STATE_SAVE_PERIOD) {
                uint8_t work_buffer[BSEC_MAX_PROPERTY_BLOB_SIZE];
                uint32_t n_serialized_state = 0;

                bsec_status = bsec_get_state(0, this->bsec_state, sizeof(this->bsec_state),
                                             work_buffer, sizeof(work_buffer),
                                             &n_serialized_state);

                if (bsec_status == BSEC_OK) {
                    time_stamp_last_state_save = curr_time_us;
                }
            }
        }

        int64_t next_call_ns = sensor_settings.next_call;
        int64_t curr_time_ns_end = get_timestamp_us() * 1000;
        int64_t sleep_duration_ms = (next_call_ns - curr_time_ns_end) / 1000000;

        if (sleep_duration_ms > 0 && sleep_duration_ms < 10000) {
            vTaskDelay(pdMS_TO_TICKS(sleep_duration_ms));
        } else {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    ESP_LOGI(TAG, "BME69x sensor data thread stopped");
}

const char *Temperature::getIAQLevel(float iaq_value)
{
    if (iaq_value <= 50) {
        return "Excellent";
    } else if (iaq_value <= 100) {
        return "Good";
    } else if (iaq_value <= 150) {
        return "Fair";
    } else if (iaq_value <= 200) {
        return "Poor";
    } else if (iaq_value <= 300) {
        return "Bad";
    } else {
        return "Very Bad";
    }
}

lv_color_t Temperature::getIAQColor(float iaq_value)
{
    if (iaq_value <= 50) {
        return lv_color_hex(0x00C853);      // Green - Excellent
    } else if (iaq_value <= 100) {
        return lv_color_hex(0x64DD17);       // Light green - Good
    } else if (iaq_value <= 150) {
        return lv_color_hex(0xFFD600);       // Yellow - Fair
    } else if (iaq_value <= 200) {
        return lv_color_hex(0xFF6D00);       // Orange - Poor
    } else if (iaq_value <= 300) {
        return lv_color_hex(0xD50000);      // Red - Unhealthy
    } else {
        return lv_color_hex(0x880E4F);      // Dark red - Very unhealthy
    }
}

void Temperature::updateTemperatureDisplay()
{
    char s[32];

    // Update temperature display
    snprintf(s, sizeof(s), "%.2f", bsec_temperature);
    lv_label_set_text(ui_TemperatureNumber, s);

    // Update humidity display with sliding average (last 5 values)
    // Add current humidity to history
    humidity_history[humidity_history_index] = bsec_humidity;
    humidity_history_index = (humidity_history_index + 1) % HUMIDITY_AVG_COUNT;
    if (humidity_history_count < HUMIDITY_AVG_COUNT) {
        humidity_history_count++;
    }

    // Calculate average
    float humidity_sum = 0.0f;
    for (int i = 0; i < humidity_history_count; i++) {
        humidity_sum += humidity_history[i];
    }
    float humidity_avg = humidity_sum / humidity_history_count;

    // Display with 2 decimal places (percentage sign is separate label)
    snprintf(s, sizeof(s), "%.2f", humidity_avg);
    lv_label_set_text(ui_HumidityNumber, s);

    // Update humidity evaluation with ASCII characters and color based on comfort range
    const char *face_char;
    lv_color_t color;
    if (humidity_avg >= 30.0f && humidity_avg <= 60.0f) {
        // Comfortable range: 30-60%
        face_char = "^_^";
        color = lv_color_hex(0x4CAF50);  // Green
    } else if ((humidity_avg >= 20.0f && humidity_avg < 30.0f) ||
               (humidity_avg > 60.0f && humidity_avg <= 70.0f)) {
        // Moderate range: 20-30% or 60-70%
        face_char = "0_0";
        color = lv_color_hex(0xFFD600);  // Yellow
    } else {
        // Uncomfortable range: <20% or >70%
        face_char = "T_T";
        color = lv_color_hex(0xFF6D00);  // Orange
    }
    lv_label_set_text(ui_HumidityEvaluation, face_char);
    lv_obj_set_style_text_color(ui_HumidityEvaluation, color, LV_PART_MAIN | LV_STATE_DEFAULT);

    // Update pressure display - convert to hPa
    float pressure_hpa = bsec_pressure / 100.0f;
    snprintf(s, sizeof(s), "%.1f", pressure_hpa);
    lv_label_set_text(ui_PressureText, s);

    // Calculate difference from standard atmospheric pressure (1013.25 hPa)
    const float standard_pressure_hpa = 1013.25f;
    float pressure_diff_from_standard = pressure_hpa - standard_pressure_hpa;

    // Update pressure trend with weather-related text (without arrow)
    char trend_text[32];
    if (pressure_diff_from_standard > 5.0f) {
        snprintf(trend_text, sizeof(trend_text), "Maybe Sunny");
        lv_obj_set_style_text_color(ui_PressureTrend, lv_color_hex(0x4CAF50), (int)LV_PART_MAIN | (int)LV_STATE_DEFAULT);
    } else if (pressure_diff_from_standard < -5.0f) {
        snprintf(trend_text, sizeof(trend_text), "Maybe Rainy");
        lv_obj_set_style_text_color(ui_PressureTrend, lv_color_hex(0xF44336), (int)LV_PART_MAIN | (int)LV_STATE_DEFAULT);
    } else {
        snprintf(trend_text, sizeof(trend_text), "Good");
        lv_obj_set_style_text_color(ui_PressureTrend, lv_color_hex(0x757575), (int)LV_PART_MAIN | (int)LV_STATE_DEFAULT);
    }
    lv_label_set_text(ui_PressureTrend, trend_text);

    previous_pressure = bsec_pressure;

    // Update air quality display - CO2 concentration
    snprintf(s, sizeof(s), "%.0f", co2_equivalent);
    lv_label_set_text(ui_AirQualityNumber, s);

    // Update IAQ level text
    const char *iaq_level = getIAQLevel(iaq);
    lv_label_set_text(ui_AirQualityLevel, iaq_level);

    // Apply color coding - only number changes color, CO2 and unit stay black
    lv_color_t iaq_color = getIAQColor(iaq);
    lv_obj_set_style_text_color(ui_AirQualityCO2, lv_color_hex(0x000000), (int)LV_PART_MAIN | (int)LV_STATE_DEFAULT);  // Fixed black
    lv_obj_set_style_text_color(ui_AirQualityNumber, iaq_color, (int)LV_PART_MAIN | (int)LV_STATE_DEFAULT);  // Dynamic color
    lv_obj_set_style_text_color(ui_AirQualityLevel, iaq_color, (int)LV_PART_MAIN | (int)LV_STATE_DEFAULT);  // Dynamic color
    lv_obj_set_style_text_color(ui_AirQualitySignal, lv_color_hex(0x000000), (int)LV_PART_MAIN | (int)LV_STATE_DEFAULT);  // Fixed black
}

int64_t Temperature::get_timestamp_us()
{
    return esp_timer_get_time();
}

ESP_UTILS_REGISTER_PLUGIN_WITH_CONSTRUCTOR(systems::base::App, Temperature, "Environment", []()
{
    return std::shared_ptr<Temperature>(Temperature::requestInstance(), [](Temperature * p) {});
})

} // namespace esp_brookesia::apps

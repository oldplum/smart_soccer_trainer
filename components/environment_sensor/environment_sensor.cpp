#include "environment_sensor.h"

#include <atomic>
#include <cmath>
#include <cstdlib>

#include "bme69x.h"
#include "bme69x_defs.h"
#include "bsec_datatypes.h"
#include "bsec_interface.h"
#include "common.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soccer_data_sync.h"

static const char *TAG = "EnvSensor";

static constexpr i2c_port_num_t kI2cPort = I2C_NUM_0;
static constexpr gpio_num_t kSdaPin = GPIO_NUM_2;
static constexpr gpio_num_t kSclPin = GPIO_NUM_3;
static constexpr gpio_num_t kBme690SdoPin = GPIO_NUM_9;
static constexpr TickType_t kRetryDelay = pdMS_TO_TICKS(1000);
static constexpr float kTempOffset = 5.0f;
static constexpr int64_t kStateSavePeriodUs = INT64_C(5) * 60 * 1000000;
static constexpr float kSampleRate = BSEC_SAMPLE_RATE_LP;

static i2c_master_bus_handle_t s_i2c_bus = nullptr;
static struct bme69x_dev s_bme = {};
static std::atomic<bool> s_running{false};
static std::atomic<bool> s_sensor_online{false};
static std::atomic<bool> s_hardware_init_failed{false};
static uint8_t s_bsec_state[BSEC_MAX_STATE_BLOB_SIZE] = {};
static uint32_t s_bsec_state_len = 0;
static bool s_bsec_state_valid = false;
static int64_t s_last_state_save_us = 0;

static int64_t get_timestamp_us()
{
    return esp_timer_get_time();
}

static void sleep_until_ns(int64_t target_time_ns)
{
    int64_t now_ns = get_timestamp_us() * 1000;
    if (target_time_ns > now_ns) {
        int64_t wait_us = (target_time_ns - now_ns) / 1000;
        if (wait_us > 0) {
            vTaskDelay(pdMS_TO_TICKS((wait_us + 999) / 1000));
            return;
        }
    }

    vTaskDelay(pdMS_TO_TICKS(1));
}

static void save_bsec_state()
{
    uint32_t n_state = sizeof(s_bsec_state);
    uint8_t *work_buffer = static_cast<uint8_t *>(std::malloc(BSEC_MAX_WORKBUFFER_SIZE));
    if (work_buffer == nullptr) {
        ESP_LOGW(TAG, "save_bsec_state: malloc failed");
        return;
    }

    bsec_library_return_t bsec_status = bsec_get_state(0,
                                                        s_bsec_state,
                                                        n_state,
                                                        work_buffer,
                                                        BSEC_MAX_WORKBUFFER_SIZE,
                                                        &n_state);
    std::free(work_buffer);
    
    if (bsec_status == BSEC_OK && n_state > 0 && n_state <= sizeof(s_bsec_state)) {
        s_bsec_state_len = n_state;
        s_bsec_state_valid = true;
        
        nvs_handle_t nvs_handle;
        esp_err_t err = nvs_open("env_sensor", NVS_READWRITE, &nvs_handle);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to open NVS: %s", esp_err_to_name(err));
            return;
        }
        
        err = nvs_set_blob(nvs_handle, "bsec_state", s_bsec_state, s_bsec_state_len);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to save BSEC state: %s", esp_err_to_name(err));
            nvs_close(nvs_handle);
            return;
        }
        
        err = nvs_commit(nvs_handle);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Failed to commit NVS: %s", esp_err_to_name(err));
        }
        
        nvs_close(nvs_handle);
    } else {
        ESP_LOGW(TAG, "bsec_get_state failed: %d", bsec_status);
    }
}

static void restore_bsec_state()
{
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("env_sensor", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to open NVS for reading: %s", esp_err_to_name(err));
        return;
    }
    
    size_t required_size = sizeof(s_bsec_state);
    err = nvs_get_blob(nvs_handle, "bsec_state", s_bsec_state, &required_size);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to read BSEC state from NVS: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return;
    }
    nvs_close(nvs_handle);
    
    if (required_size == 0 || required_size > sizeof(s_bsec_state)) {
        ESP_LOGW(TAG, "Invalid BSEC state size from NVS: %lu", static_cast<unsigned long>(required_size));
        return;
    }
    
    s_bsec_state_len = required_size;
    s_bsec_state_valid = true;
    
    uint8_t *work_buffer = static_cast<uint8_t *>(std::malloc(BSEC_MAX_WORKBUFFER_SIZE));
    if (work_buffer == nullptr) {
        ESP_LOGW(TAG, "restore_bsec_state: malloc failed");
        return;
    }

    bsec_library_return_t bsec_status = bsec_set_state(s_bsec_state,
                                                       s_bsec_state_len,
                                                       work_buffer,
                                                       BSEC_MAX_WORKBUFFER_SIZE);
    std::free(work_buffer);
    
    if (bsec_status != BSEC_OK) {
        ESP_LOGW(TAG, "bsec_set_state failed: %d", bsec_status);
    } else {
        ESP_LOGI(TAG, "Restored BSEC state blob (%lu bytes) from NVS", static_cast<unsigned long>(s_bsec_state_len));
    }
}

static bool init_hardware()
{
    esp_err_t ret;

    if (s_i2c_bus != nullptr) {
        bme69x_set_i2c_bus_handle(s_i2c_bus);
        return true;
    }

    gpio_config_t sdo_conf = {
        .pin_bit_mask = (1ULL << kBme690SdoPin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ret = gpio_config(&sdo_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "gpio_config failed: %s", esp_err_to_name(ret));
        return false;
    }
    gpio_set_level(kBme690SdoPin, 0);

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = kI2cPort,
        .sda_io_num = kSdaPin,
        .scl_io_num = kSclPin,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true,
        },
    };
    ret = i2c_new_master_bus(&bus_cfg, &s_i2c_bus);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "i2c_new_master_bus failed (sensor may be disconnected): %s", esp_err_to_name(ret));
        s_hardware_init_failed = true;
        return false;
    }

    bme69x_set_i2c_bus_handle(s_i2c_bus);
    return true;
}

static bool init_bme69x_sensor()
{
    int8_t rslt;

    rslt = bme69x_interface_init(&s_bme, BME69X_I2C_INTF);
    if (rslt != BME69X_OK) {
        ESP_LOGE(TAG, "bme69x_interface_init failed: %d", rslt);
        return false;
    }

    rslt = bme69x_init(&s_bme);
    if (rslt != BME69X_OK) {
        ESP_LOGE(TAG, "bme69x_init failed: %d", rslt);
        return false;
    }

    s_bme.amb_temp = 25.0f;
    ESP_LOGI(TAG, "BME69x chip id: 0x%02x", s_bme.chip_id);
    return true;
}

static bool init_bsec()
{
    bsec_library_return_t bsec_status = bsec_init();
    if (bsec_status != BSEC_OK) {
        ESP_LOGE(TAG, "bsec_init failed: %d", bsec_status);
        return false;
    }

    restore_bsec_state();

    bsec_sensor_configuration_t requested_virtual_sensors[8];
    requested_virtual_sensors[0].sensor_id = BSEC_OUTPUT_IAQ;
    requested_virtual_sensors[0].sample_rate = kSampleRate;
    requested_virtual_sensors[1].sensor_id = BSEC_OUTPUT_STATIC_IAQ;
    requested_virtual_sensors[1].sample_rate = kSampleRate;
    requested_virtual_sensors[2].sensor_id = BSEC_OUTPUT_CO2_EQUIVALENT;
    requested_virtual_sensors[2].sample_rate = kSampleRate;
    requested_virtual_sensors[3].sensor_id = BSEC_OUTPUT_BREATH_VOC_EQUIVALENT;
    requested_virtual_sensors[3].sample_rate = kSampleRate;
    requested_virtual_sensors[4].sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE;
    requested_virtual_sensors[4].sample_rate = kSampleRate;
    requested_virtual_sensors[5].sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY;
    requested_virtual_sensors[5].sample_rate = kSampleRate;
    requested_virtual_sensors[6].sensor_id = BSEC_OUTPUT_RAW_PRESSURE;
    requested_virtual_sensors[6].sample_rate = kSampleRate;
    requested_virtual_sensors[7].sensor_id = BSEC_OUTPUT_RAW_GAS;
    requested_virtual_sensors[7].sample_rate = kSampleRate;

    bsec_sensor_configuration_t required_sensor_settings[BSEC_MAX_PHYSICAL_SENSOR];
    uint8_t n_required_sensor_settings = BSEC_MAX_PHYSICAL_SENSOR;
    bsec_status = bsec_update_subscription(requested_virtual_sensors,
                                           8,
                                           required_sensor_settings,
                                           &n_required_sensor_settings);
    if (bsec_status < 0) {
        ESP_LOGE(TAG, "bsec_update_subscription failed: %d", bsec_status);
        return false;
    }

    return true;
}

static bool init_sensor_stack()
{
    if (!init_hardware()) {
        return false;
    }

    if (!init_bme69x_sensor()) {
        return false;
    }

    if (!init_bsec()) {
        return false;
    }

    s_last_state_save_us = get_timestamp_us();
    s_sensor_online = true;
    return true;
}

static void deinit_sensor_stack()
{
    s_sensor_online = false;
    save_bsec_state();
    bme69x_coines_deinit();
    if (s_i2c_bus != nullptr) {
        i2c_del_master_bus(s_i2c_bus);
        s_i2c_bus = nullptr;
    }
}

static void update_global_environment(float temperature,
                                       float humidity,
                                       float pressure,
                                       float iaq,
                                       bool valid)
{
    g_soccer_sensor_data.temp = temperature;
    g_soccer_sensor_data.humidity = humidity;
    g_soccer_sensor_data.pressure = pressure;
    g_soccer_sensor_data.iaq = iaq;
    g_soccer_sensor_data.env_data_valid = valid ? 1 : 0;
    g_soccer_sensor_data.timestamp = static_cast<uint32_t>(get_timestamp_us() / 1000);
}

static void environment_sensor_task(void *)
{
    while (s_running.load(std::memory_order_relaxed)) {
        if (!s_sensor_online.load(std::memory_order_relaxed)) {
            if (s_hardware_init_failed.load(std::memory_order_relaxed)) {
                update_global_environment(g_soccer_sensor_data.temp,
                                           g_soccer_sensor_data.humidity,
                                           g_soccer_sensor_data.pressure,
                                           g_soccer_sensor_data.iaq,
                                           false);
                vTaskDelay(kRetryDelay);
                continue;
            }
            
            if (!init_sensor_stack()) {
                update_global_environment(g_soccer_sensor_data.temp,
                                           g_soccer_sensor_data.humidity,
                                           g_soccer_sensor_data.pressure,
                                           g_soccer_sensor_data.iaq,
                                           false);
                vTaskDelay(kRetryDelay);
                continue;
            }
        }

        int64_t curr_time_ns = get_timestamp_us() * 1000;
        bsec_bme_settings_t sensor_settings;
        bsec_library_return_t bsec_status = bsec_sensor_control(curr_time_ns, &sensor_settings);
        if (bsec_status < BSEC_OK) {
            ESP_LOGE(TAG, "bsec_sensor_control failed: %d", bsec_status);
            update_global_environment(g_soccer_sensor_data.temp,
                                       g_soccer_sensor_data.humidity,
                                       g_soccer_sensor_data.pressure,
                                       g_soccer_sensor_data.iaq,
                                       false);
            vTaskDelay(kRetryDelay);
            continue;
        } else if (bsec_status > BSEC_OK) {
            ESP_LOGW(TAG, "bsec_sensor_control warning: %d", bsec_status);
        }

        if (!sensor_settings.trigger_measurement) {
            sleep_until_ns(sensor_settings.next_call);
            continue;
        }

        bme69x_conf conf = {};
        conf.filter = BME69X_FILTER_OFF;
        conf.odr = BME69X_ODR_NONE;
        conf.os_hum = sensor_settings.humidity_oversampling;
        conf.os_pres = sensor_settings.pressure_oversampling;
        conf.os_temp = sensor_settings.temperature_oversampling;

        int8_t rslt = bme69x_set_conf(&conf, &s_bme);
        if (rslt != BME69X_OK) {
            ESP_LOGW(TAG, "bme69x_set_conf failed: %d", rslt);
            update_global_environment(g_soccer_sensor_data.temp,
                                       g_soccer_sensor_data.humidity,
                                       g_soccer_sensor_data.pressure,
                                       g_soccer_sensor_data.iaq,
                                       false);
            vTaskDelay(kRetryDelay);
            continue;
        }

        bme69x_heatr_conf heatr_conf = {};
        heatr_conf.enable = BME69X_ENABLE;
        heatr_conf.heatr_temp = sensor_settings.heater_temperature;
        heatr_conf.heatr_dur = sensor_settings.heater_duration;

        rslt = bme69x_set_heatr_conf(BME69X_FORCED_MODE, &heatr_conf, &s_bme);
        if (rslt != BME69X_OK) {
            ESP_LOGW(TAG, "bme69x_set_heatr_conf failed: %d", rslt);
            update_global_environment(g_soccer_sensor_data.temp,
                                       g_soccer_sensor_data.humidity,
                                       g_soccer_sensor_data.pressure,
                                       g_soccer_sensor_data.iaq,
                                       false);
            vTaskDelay(kRetryDelay);
            continue;
        }

        rslt = bme69x_set_op_mode(BME69X_FORCED_MODE, &s_bme);
        if (rslt != BME69X_OK) {
            ESP_LOGW(TAG, "bme69x_set_op_mode failed: %d", rslt);
            update_global_environment(g_soccer_sensor_data.temp,
                                       g_soccer_sensor_data.humidity,
                                       g_soccer_sensor_data.pressure,
                                       g_soccer_sensor_data.iaq,
                                       false);
            vTaskDelay(kRetryDelay);
            continue;
        }

        uint32_t meas_delay_us = bme69x_get_meas_dur(BME69X_FORCED_MODE, &conf, &s_bme) + (heatr_conf.heatr_dur * 1000);
        s_bme.delay_us(meas_delay_us, s_bme.intf_ptr);

        bme69x_data data = {};
        uint8_t n_data = 0;
        rslt = bme69x_get_data(BME69X_FORCED_MODE, &data, &n_data, &s_bme);
        if (rslt != BME69X_OK || n_data == 0 || !(data.status & BME69X_GASM_VALID_MSK)) {
            ESP_LOGW(TAG, "bme69x_get_data invalid: rslt=%d n_data=%u status=0x%02x", rslt, n_data, data.status);
            update_global_environment(g_soccer_sensor_data.temp,
                                       g_soccer_sensor_data.humidity,
                                       g_soccer_sensor_data.pressure,
                                       g_soccer_sensor_data.iaq,
                                       false);
            sleep_until_ns(sensor_settings.next_call);
            continue;
        }

        bsec_input_t bsec_inputs[BSEC_MAX_PHYSICAL_SENSOR];
        uint8_t n_bsec_inputs = 0;
        bsec_output_t bsec_outputs[BSEC_NUMBER_OUTPUTS];
        uint8_t n_bsec_outputs = BSEC_NUMBER_OUTPUTS;
        int64_t time_stamp_ns = get_timestamp_us() * 1000;

        if (sensor_settings.process_data & BSEC_PROCESS_TEMPERATURE) {
            bsec_inputs[n_bsec_inputs].sensor_id = BSEC_INPUT_TEMPERATURE;
            bsec_inputs[n_bsec_inputs].signal = data.temperature;
            bsec_inputs[n_bsec_inputs].time_stamp = time_stamp_ns;
            n_bsec_inputs++;

            bsec_inputs[n_bsec_inputs].sensor_id = BSEC_INPUT_HEATSOURCE;
            bsec_inputs[n_bsec_inputs].signal = kTempOffset;
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

        bsec_status = bsec_do_steps(bsec_inputs, n_bsec_inputs, bsec_outputs, &n_bsec_outputs);
        if (bsec_status < BSEC_OK) {
            ESP_LOGW(TAG, "bsec_do_steps failed: %d", bsec_status);
            update_global_environment(g_soccer_sensor_data.temp,
                                       g_soccer_sensor_data.humidity,
                                       g_soccer_sensor_data.pressure,
                                       g_soccer_sensor_data.iaq,
                                       false);
            sleep_until_ns(sensor_settings.next_call);
            continue;
        }

        float temperature = g_soccer_sensor_data.temp;
        float humidity = g_soccer_sensor_data.humidity;
        float pressure = g_soccer_sensor_data.pressure;
        float iaq = g_soccer_sensor_data.iaq;

        for (uint8_t i = 0; i < n_bsec_outputs; i++) {
            switch (bsec_outputs[i].sensor_id) {
            case BSEC_OUTPUT_IAQ:
                iaq = bsec_outputs[i].signal;
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
            default:
                break;
            }
        }

        update_global_environment(temperature, humidity, pressure, iaq, true);

        if ((get_timestamp_us() - s_last_state_save_us) >= kStateSavePeriodUs) {
            s_last_state_save_us = get_timestamp_us();
            save_bsec_state();
        }

        sleep_until_ns(sensor_settings.next_call);
    }

    deinit_sensor_stack();
    vTaskDelete(nullptr);
}

extern "C" bool start_environment_sensor_task(void)
{
    if (s_running.exchange(true, std::memory_order_relaxed)) {
        return true;
    }

    BaseType_t ret = xTaskCreate(environment_sensor_task,
                                 "env_sensor",
                                 8192,
                                 nullptr,
                                 tskIDLE_PRIORITY + 1,
                                 nullptr);
    if (ret != pdPASS) {
        s_running = false;
        ESP_LOGE(TAG, "Failed to create environment sensor task");
        return false;
    }

    return true;
}
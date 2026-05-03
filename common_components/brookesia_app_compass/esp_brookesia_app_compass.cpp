/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "BMM350_SensorAPI/bmm350_defs.h"
#include "BMM350_SensorAPI/bmm350.h"
#include "BMM350_SensorAPI/examples/common/common.h"
#include "driver/gpio.h"
#include "esp_brookesia_app_compass.hpp"
#include "esp_err.h"
#include "esp_lib_utils.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "math.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "ui/ui.h"
#include "brookesia/system_core/gui/lvgl/lock.hpp"
#include "esp_board_manager.h"
#include <cstdint>

static const char *TAG = "CompassApp";

using namespace esp_brookesia::systems::phone;
using namespace esp_brookesia::gui;

#define APP_NAME "Compass"

#ifndef M_PI
#define M_PI 3.1415926f
#endif

#define I2C_MASTER_SDO_IO GPIO_NUM_9
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ (100 * 1000)
#define SAMPLE_RATE_MS 10 // 10ms sampling interval

#define COMPLEMENTARY_FILTER_ALPHA 0.9f
#define HEADING_FILTER_ALPHA 0.85f  // Low-pass filter for heading smoothing (0.85 = 85% previous, 15% new)

// NVS storage configuration
#define NVS_NAMESPACE "compass_cal"
#define NVS_KEY_HARD_IRON "hard_iron"
#define NVS_KEY_SOFT_IRON "soft_iron"
#define NVS_KEY_CALIBRATED "calibrated"

LV_IMG_DECLARE(img_app_compass);

namespace esp_brookesia::apps {

constexpr systems::base::App::Config CORE_DATA = {
    .name = APP_NAME,
    .launcher_icon = gui::StyleImage::IMAGE(&img_app_compass),
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

Compass::Compass()
    : App(CORE_DATA, APP_DATA)
    , bmi_handle_(nullptr)
    , i2c_bus_(nullptr)
    , bmi2_dev_(nullptr)
    , mag_cal_{{0, 0, 0}, {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}, false}
, current_heading_(0.0f)
, current_mag_data_{0}
, bmm_running_(false)
{
    ESP_LOGI(TAG, "Compass app constructor");
}

Compass::~Compass()
{
    ESP_UTILS_LOG_TRACE_GUARD_WITH_THIS();
    ESP_LOGI(TAG, "Compass app destructor");
    deinit();
}

Compass *Compass::requestInstance()
{
    if (_instance == nullptr) {
        _instance = new Compass();
    }
    return _instance;
}

void Compass::setCorrect(bool correct)
{
    mag_cal_.calibrated = correct;
}

void Compass::externalBack()
{
    back();
}

bool Compass::isCalibrated() const
{
    return mag_cal_.calibrated;
}

bool Compass::init()
{
    ESP_LOGI(TAG, "Initializing Compass app");

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Load calibration data from NVS
    if (loadCalibrationFromNVS()) {
        ESP_LOGI(TAG, "Calibration data loaded from NVS");
    } else {
        ESP_LOGI(TAG, "No calibration data found in NVS, using defaults");
    }

    return true;
}

bool Compass::deinit()
{
    ESP_LOGI(TAG, "Deinitializing Compass app");

    destroyCompassUI();
    bmm_running_ = false;
    if (bmm_data_thread.joinable()) {
        bmm_data_thread.join();
    }

    if (update_compass_thread.joinable()) {
        update_compass_thread.join();
    }
    deinitSensors();

    return true;
}

bool Compass::run()
{
    ESP_LOGI(TAG, "Running Compass app");

    if (!initSensors()) {
        ESP_LOGE(TAG, "Failed to initialize sensors");
        is_initialized_ = false;
        is_app_running_ = false;
    } else {
        is_initialized_ = true;
        is_app_running_ = true;
    }

    createCompassUI();

    if (!is_initialized_) {
        return true;
    }

    // Start sensor reading task
    bmm_running_ = true;
    bmm_data_thread = boost::thread(&Compass::bmmDataThread, this);
    update_compass_thread = boost::thread(&Compass::updateCompassThread, this);

    return true;
}

bool Compass::back()
{
    ESP_LOGI(TAG, "Compass app back");
    ESP_UTILS_CHECK_FALSE_RETURN(notifyCoreClosed(), false, "Notify core closed failed");
    return true;
}

bool Compass::resume()
{
    ESP_LOGI(TAG, "Compass app resume");

    if (!initSensors()) {
        ESP_LOGE(TAG, "Failed to initialize sensors");
        is_initialized_ = false;
        is_app_running_ = false;
        {
            LvLockGuard gui_guard;
            lv_disp_load_scr(ui_CompassTipScreen);
        }
    } else {
        is_initialized_ = true;
        is_app_running_ = true;
        // lv_disp_load_scr(ui_CompassScreen);
    }

    return true;
}

bool Compass::pause()
{
    ESP_LOGI(TAG, "Compass app pause");

    is_app_running_ = false;
    if (!deinitSensors()) {
        ESP_LOGE(TAG, "Failed to deinitialize sensors");
        return true;
    } else {
        is_initialized_ = true;
    }

    return true;
}

bool Compass::close()
{
    ESP_LOGI(TAG, "Closing Compass app");
    bmm_running_ = false;
    if (bmm_data_thread.joinable()) {
        bmm_data_thread.join();
    }
    if (update_compass_thread.joinable()) {
        update_compass_thread.join();
    }
    return true;
}

void Compass::createCompassUI()
{
    ESP_LOGI(TAG, "Creating compass UI");
    compass_ui_init(is_initialized_);
    ESP_LOGI(TAG, "Compass UI created successfully");
}

void Compass::destroyCompassUI()
{
    compass_ui_destroy();
    ESP_LOGI(TAG, "Compass UI destroyed");
}

bool Compass::initSensors()
{
    ESP_LOGI(TAG, "Initializing BMM350 sensors...");

    vTaskDelay(10 / portTICK_PERIOD_MS);

    /* Configure SDO pin */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << I2C_MASTER_SDO_IO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SDO pin configuration failed: %s", esp_err_to_name(ret));
    }
    gpio_set_level(I2C_MASTER_SDO_IO, 0);

    vTaskDelay(10 / portTICK_PERIOD_MS);

    // Get I2C pin configuration from Board Manager
    i2c_master_bus_config_t *i2c_config = nullptr;
    ret = esp_board_manager_get_periph_config("i2c_master", (void **)&i2c_config);
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

    ret = bmi270_sensor_create(i2c_bus_, &bmi_handle_, bmi270_config_file, BMI2_GYRO_CROSS_SENS_ENABLE | BMI2_CRT_RTOSK_ENABLE);
    if (ret != ESP_OK || bmi_handle_ == NULL) {
        ESP_LOGE(TAG, "BMI270 creation failed");
    }

    bmi2_dev_ = (struct bmi2_dev *)bmi_handle_;
    ESP_LOGI(TAG, "BMI270 initialized successfully");

    struct bmi2_sens_config config[2];
    config[BMI2_ACCEL].type = BMI2_ACCEL;
    config[BMI2_GYRO].type = BMI2_GYRO;

    int8_t rslt = bmi2_get_sensor_config(config, 2, bmi2_dev_);
    if (rslt == BMI2_OK) {
        // Configure accelerometer
        config[BMI2_ACCEL].cfg.acc.odr = BMI2_ACC_ODR_100HZ;        // 100Hz sample rate
        config[BMI2_ACCEL].cfg.acc.range = BMI2_ACC_RANGE_4G;        // ±4G range
        config[BMI2_ACCEL].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;        // Normal averaging
        config[BMI2_ACCEL].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE; // Filter performance

        // Configure gyroscope
        config[BMI2_GYRO].cfg.gyr.odr = BMI2_GYR_ODR_100HZ;          // 100Hz sample rate
        config[BMI2_GYRO].cfg.gyr.range = BMI2_GYR_RANGE_2000;       // ±2000dps range
        config[BMI2_GYRO].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;       // Normal filter
        config[BMI2_GYRO].cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;  // Noise performance
        config[BMI2_GYRO].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE; // Filter performance

        rslt = bmi2_set_sensor_config(config, 2, bmi2_dev_);
        if (rslt != BMI2_OK) {
            ESP_LOGE(TAG, "BMI270 sensor config failed: %d", rslt);
            return false;
        }
    } else {
        ESP_LOGW(TAG, "BMI270 get sensor config failed: %d", rslt);
    }

    uint8_t sens_list[2] = {BMI2_ACCEL, BMI2_GYRO};
    rslt = bmi2_sensor_enable(sens_list, 2, bmi2_dev_);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "BMI270 sensor enable failed: %d", rslt);
        return false;
    }
    ESP_LOGI(TAG, "BMI270 sensors enabled");

    static uint8_t s_bmm350_addr = 0x14;

    if (bmm350_interface_init(&s_bmm350, i2c_bus_) != BMM350_OK) {
        ESP_LOGE(TAG, "BMM350 interface init failed");
        return false;
    }

    /* Try 0x14 first (ADSEL=GND). If it fails, try 0x15 (ADSEL=VDD). */
    const uint8_t addr_candidates[] = { 0x14, 0x15 };
    int8_t bmm350_rslt = BMM350_E_DEV_NOT_FOUND;

    for (size_t i = 0; i < sizeof(addr_candidates); ++i) {
        uint8_t addr = addr_candidates[i];
        /* Consistent with official example: intf_ptr passes address pointer, read/write are bmm350_i2c_read/write */
        s_bmm350_addr = addr;
        s_bmm350.read = bmm350_i2c_read;
        s_bmm350.write = bmm350_i2c_write;
        s_bmm350.delay_us = bmm350_delay;
        s_bmm350.intf_ptr = (void *)&s_bmm350_addr;

        /* Initialize BMM350 (reads CHIP_ID internally) */
        bmm350_rslt = bmm350_init(&s_bmm350);
        ESP_LOGI(TAG, "Init at 0x%02X -> rslt=%d, chip_id=0x%02X (expect 0x33)", addr, bmm350_rslt, s_bmm350.chip_id);

        /* Some boards may return -16(BMM350_E_PMU_CMD_VALUE) after power-on, but chip_id is already correct as 0x33.
         * In this case, perform a soft reset and continue. */
        if (s_bmm350.chip_id == BMM350_CHIP_ID) {
            if (bmm350_rslt != BMM350_OK) {
                (void)bmm350_soft_reset(&s_bmm350);
                bmm350_delay(BMM350_SOFT_RESET_DELAY + 10000, &s_bmm350); /* Extra delay for stability */
            }

            /* Wait for PMU busy to clear */
            struct bmm350_pmu_cmd_status_0 pmu0 = { 0 };
            for (int t = 0; t < 10; ++t) {
                (void)bmm350_get_pmu_cmd_status_0(&pmu0, &s_bmm350);
                if (pmu0.pmu_cmd_busy == 0) {
                    break;
                }
                bmm350_delay(5000, &s_bmm350);
            }

            uint8_t err_reg = 0;
            (void)bmm350_get_regs(BMM350_REG_ERR_REG, &err_reg, 1, &s_bmm350);
            ESP_LOGI(TAG, "PMU_ST0.busy=%u, ERR_REG=0x%02X", pmu0.pmu_cmd_busy, err_reg);

            /* Configure ODR/averaging, enable XYZ axes, and enter NORMAL mode */
            (void)bmm350_set_odr_performance(BMM350_DATA_RATE_100HZ, BMM350_AVERAGING_4, &s_bmm350);
            (void)bmm350_enable_axes(BMM350_X_EN, BMM350_Y_EN, BMM350_Z_EN, &s_bmm350);
            (void)bmm350_set_powermode(BMM350_NORMAL_MODE, &s_bmm350);
            ESP_LOGI(TAG, "BMM350 configured successfully");
            return true;
        }
    }

    return true;
}

bool Compass::deinitSensors()
{
    ESP_LOGI(TAG, "Deinitializing sensors...");

    if (bmi_handle_) {
        bmi270_sensor_del(&bmi_handle_);
        bmi_handle_ = nullptr;
    }

    bmm350_coines_deinit();

    return true;
}

bool Compass::isCalibrationSufficient(
    float mag_min[3], float mag_max[3],
    float pitch_min, float pitch_max,
    float roll_min, float roll_max,
    uint8_t octant_coverage,
    int sample_count)
{
    if (sample_count < 300) {
        return false;
    }

    float pitch_range = pitch_max - pitch_min;
    float roll_range = roll_max - roll_min;
    bool attitude_ok = (pitch_range > 90.0f && roll_range > 120.0f);

    if (!attitude_ok) {
        return false;
    }

    float mag_range[3];
    for (int i = 0; i < 3; i++) {
        mag_range[i] = mag_max[i] - mag_min[i];
    }
    float avg_range = (mag_range[0] + mag_range[1] + mag_range[2]) / 3.0f;
    float min_range = fminf(fminf(mag_range[0], mag_range[1]), mag_range[2]);

    bool mag_ok = (min_range > avg_range * 0.5f && avg_range > 50.0f);

    if (!mag_ok) {
        ESP_LOGW(TAG, "Magnetic range insufficient: X=%.1f, Y=%.1f, Z=%.1f (avg=%.1f)",
                 mag_range[0], mag_range[1], mag_range[2], avg_range);
        return false;
    }

    int covered_octants = __builtin_popcount(octant_coverage);
    bool octant_ok = (covered_octants >= 4);

    if (!octant_ok) {
        ESP_LOGW(TAG, "Octant coverage low: %d/8 (minimum 4 required)", covered_octants);
    }

    float max_range = fmaxf(fmaxf(mag_range[0], mag_range[1]), mag_range[2]);
    float ellipsoid_ratio = min_range / max_range;

    if (ellipsoid_ratio < 0.25f) {
        ESP_LOGW(TAG, "Ellipsoid too flat: ratio=%.2f (need >0.25)", ellipsoid_ratio);
        return false;
    }

    if (!octant_ok && ellipsoid_ratio < 0.6f) {
        ESP_LOGW(TAG, "Both octant coverage and ellipsoid quality are insufficient");
        return false;
    }

    ESP_LOGI(TAG, "  Samples: %d, Pitch: %.1f°, Roll: %.1f°",
             sample_count, pitch_range, roll_range);
    ESP_LOGI(TAG, "  Octants: %d/8, Ellipsoid: %.2f, Mag ranges: [%.1f, %.1f, %.1f]",
             covered_octants, ellipsoid_ratio, mag_range[0], mag_range[1], mag_range[2]);

    return true;
}

void Compass::calibrateMagnetometer()
{
    if (mag_cal_.calibrated) {
        return;
    }

    ESP_LOGI(TAG, "=== Magnetometer Calibration Started ===");
    ESP_LOGI(TAG, "Please rotate the device in all directions (figure-8 pattern)");

    float mag_min[3] = {10000, 10000, 10000};
    float mag_max[3] = {-10000, -10000, -10000};
    float pitch_min = 10000.0f;
    float pitch_max = -10000.0f;
    float roll_min = 10000.0f;
    float roll_max = -10000.0f;
    uint8_t octant_coverage = 0;
    int sample_count = 0;

    uint32_t last_progress_time = 0;

    const int INITIAL_SAMPLES = 100;
    float mag_center[3] = {0, 0, 0};
    bool center_initialized = false;

    while (bmm_running_) {
        struct bmm350_mag_temp_data mag_data = {0};
        int8_t rs = bmm350_get_compensated_mag_xyz_temp_data(&mag_data, &s_bmm350);

        if (rs == BMM350_OK) {
            float mag_x = (float)mag_data.x;
            float mag_y = (float)mag_data.y;
            float mag_z = (float)mag_data.z;

            if (mag_x < mag_min[0]) {
                mag_min[0] = mag_x;
            }
            if (mag_y < mag_min[1]) {
                mag_min[1] = mag_y;
            }
            if (mag_z < mag_min[2]) {
                mag_min[2] = mag_z;
            }
            if (mag_x > mag_max[0]) {
                mag_max[0] = mag_x;
            }
            if (mag_y > mag_max[1]) {
                mag_max[1] = mag_y;
            }
            if (mag_z > mag_max[2]) {
                mag_max[2] = mag_z;
            }

            sample_count++;

            if (sample_count >= INITIAL_SAMPLES && !center_initialized) {
                mag_center[0] = (mag_max[0] + mag_min[0]) / 2.0f;
                mag_center[1] = (mag_max[1] + mag_min[1]) / 2.0f;
                mag_center[2] = (mag_max[2] + mag_min[2]) / 2.0f;
                center_initialized = true;
                ESP_LOGI(TAG, "Magnetic field center estimated: [%.1f, %.1f, %.1f]",
                         mag_center[0], mag_center[1], mag_center[2]);
            }

            if (center_initialized) {
                mag_center[0] = (mag_max[0] + mag_min[0]) / 2.0f;
                mag_center[1] = (mag_max[1] + mag_min[1]) / 2.0f;
                mag_center[2] = (mag_max[2] + mag_min[2]) / 2.0f;

                uint8_t octant = 0;
                if (mag_x > mag_center[0]) {
                    octant |= (1 << 0);
                }
                if (mag_y > mag_center[1]) {
                    octant |= (1 << 1);
                }
                if (mag_z > mag_center[2]) {
                    octant |= (1 << 2);
                }

                octant_coverage |= (1 << octant);
            }

            struct bmi2_sens_data sensor_data = {0};
            int8_t rslt = bmi2_get_sensor_data(&sensor_data, bmi2_dev_);
            if (rslt == BMI2_OK) {
                /* Parse accelerometer data */
                float acc_x_mg = (float)sensor_data.acc.x / 16384.0f;
                float acc_y_mg = -(float)sensor_data.acc.y / 16384.0f;
                float acc_z_mg = -(float)sensor_data.acc.z / 16384.0f;

                /* Parse gyroscope data */
                float gyro_x_dps = (float)sensor_data.gyr.x / 16.4f;
                float gyro_y_dps = -(float)sensor_data.gyr.y / 16.4f;
                float gyro_z_dps = -(float)sensor_data.gyr.z / 16.4f;

                /* Update complementary filter with new sensor data */
                update_complementary_filter(acc_x_mg, acc_y_mg, acc_z_mg, gyro_x_dps,
                                            gyro_y_dps, gyro_z_dps, comp_filter.dt);

                if (comp_filter.euler.pitch < pitch_min) {
                    pitch_min = comp_filter.euler.pitch;
                }
                if (comp_filter.euler.pitch > pitch_max) {
                    pitch_max = comp_filter.euler.pitch;
                }
                if (comp_filter.euler.roll < roll_min) {
                    roll_min = comp_filter.euler.roll;
                }
                if (comp_filter.euler.roll > roll_max) {
                    roll_max = comp_filter.euler.roll;
                }
            }

            uint32_t now = esp_timer_get_time() / 1000000;
            if (now - last_progress_time >= 1) {
                int covered = __builtin_popcount(octant_coverage);
                float pitch_range = pitch_max - pitch_min;
                float roll_range = roll_max - roll_min;
                float mag_ranges[3] = {
                    mag_max[0] - mag_min[0],
                    mag_max[1] - mag_min[1],
                    mag_max[2] - mag_min[2]
                };

                ESP_LOGI(TAG, "Progress: Samples=%d, Octants=%d/8, Pitch=%.1f°, Roll=%.1f°",
                         sample_count, covered, pitch_range, roll_range);
                ESP_LOGI(TAG, "  Mag ranges: X=%.1f, Y=%.1f, Z=%.1f",
                         mag_ranges[0], mag_ranges[1], mag_ranges[2]);

                if (pitch_range < 90.0f) {
                    ESP_LOGI(TAG, "  → Rotate device forward/backward more");
                    lv_async_call([](void *user_data) {
                        Compass *self = static_cast<Compass *>(user_data);
                        if (!self->bmm_running_) {
                            return;
                        }
                        lv_label_set_text(
                            ui_CorrectTips,
                            "Calibrating...\nRotate device\nforward/backward more");
                    }, this);
                } else if (roll_range < 120.0f) {
                    ESP_LOGI(TAG, "  → Rotate device left/right more");
                    lv_async_call([](void *user_data) {
                        Compass *self = static_cast<Compass *>(user_data);
                        if (!self->bmm_running_) {
                            return;
                        }
                        lv_label_set_text(
                            ui_CorrectTips,
                            "Calibrating...\nRotate device\nleft/right more");
                    }, this);
                } else if (covered < 6) {
                    ESP_LOGI(TAG, "  → Move device in figure-8 pattern");
                    lv_async_call([](void *user_data) {
                        Compass *self = static_cast<Compass *>(user_data);
                        if (!self->bmm_running_) {
                            return;
                        }
                        lv_label_set_text(
                            ui_CorrectTips,
                            "Calibrating...\nMove device in\nfigure-8 pattern");
                    }, this);
                } else if (sample_count < 500) {
                    lv_async_call([](void *user_data) {
                        Compass *self = static_cast<Compass *>(user_data);
                        if (!self->bmm_running_) {
                            return;
                        }
                        lv_label_set_text(
                            ui_CorrectTips,
                            "Calibrating...\nKeep moving device\nin figure-8 pattern");
                    }, this);
                }

                last_progress_time = now;
            }

            if (sample_count > 500 &&
                    isCalibrationSufficient(mag_min, mag_max, pitch_min, pitch_max,
                                            roll_min, roll_max, octant_coverage,
                                            sample_count)) {
                break;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (!bmm_running_) {
        mag_cal_.calibrated = false;
        return;
    }

    // Calculate hard iron offset
    mag_cal_.hard_iron[0] = (mag_max[0] + mag_min[0]) / 2.0f;
    mag_cal_.hard_iron[1] = (mag_max[1] + mag_min[1]) / 2.0f;
    mag_cal_.hard_iron[2] = (mag_max[2] + mag_min[2]) / 2.0f;

    // Calculate soft iron correction
    float avg_delta[3];
    avg_delta[0] = (mag_max[0] - mag_min[0]) / 2.0f;
    avg_delta[1] = (mag_max[1] - mag_min[1]) / 2.0f;
    avg_delta[2] = (mag_max[2] - mag_min[2]) / 2.0f;
    float avg_radius = (avg_delta[0] + avg_delta[1] + avg_delta[2]) / 3.0f;

    if (avg_radius > 0) {
        mag_cal_.soft_iron[0][0] = avg_radius / avg_delta[0];
        mag_cal_.soft_iron[1][1] = avg_radius / avg_delta[1];
        mag_cal_.soft_iron[2][2] = avg_radius / avg_delta[2];
    }

    mag_cal_.calibrated = true;

    // Save calibration data to NVS
    if (saveCalibrationToNVS()) {
        ESP_LOGI(TAG, "Calibration data saved to NVS successfully");
    } else {
        ESP_LOGE(TAG, "Failed to save calibration data to NVS");
    }

    ESP_LOGI(TAG, "=== Calibration Complete! ===");
    ESP_LOGI(TAG, "Hard Iron: X=%.2f, Y=%.2f, Z=%.2f",
             mag_cal_.hard_iron[0], mag_cal_.hard_iron[1], mag_cal_.hard_iron[2]);
    ESP_LOGI(TAG, "Soft Iron: X=%.3f, Y=%.3f, Z=%.3f",
             mag_cal_.soft_iron[0][0], mag_cal_.soft_iron[1][1], mag_cal_.soft_iron[2][2]);
}

euler_angles_t Compass::calculate_euler_from_accel(float acc_x, float acc_y, float acc_z)
{
    euler_angles_t angles;

    angles.pitch = atan2(-acc_x, sqrt(acc_y * acc_y + acc_z * acc_z)) * 180.0f / M_PI;

    angles.roll = atan2(acc_y, acc_z) * 180.0f / M_PI;

    angles.yaw = 0.0f;

    return angles;
}

void Compass::applyMagCalibration(const float mag_raw[3], float mag_calibrated[3])
{
    if (!mag_cal_.calibrated) {
        mag_calibrated[0] = mag_raw[0];
        mag_calibrated[1] = mag_raw[1];
        mag_calibrated[2] = mag_raw[2];
        return;
    }

    // Apply hard iron correction
    float mag_hi_corrected[3];
    mag_hi_corrected[0] = mag_raw[0] - mag_cal_.hard_iron[0];
    mag_hi_corrected[1] = mag_raw[1] - mag_cal_.hard_iron[1];
    mag_hi_corrected[2] = mag_raw[2] - mag_cal_.hard_iron[2];

    // Apply soft iron correction
    mag_calibrated[0] = mag_cal_.soft_iron[0][0] * mag_hi_corrected[0] +
                        mag_cal_.soft_iron[0][1] * mag_hi_corrected[1] +
                        mag_cal_.soft_iron[0][2] * mag_hi_corrected[2];
    mag_calibrated[1] = mag_cal_.soft_iron[1][0] * mag_hi_corrected[0] +
                        mag_cal_.soft_iron[1][1] * mag_hi_corrected[1] +
                        mag_cal_.soft_iron[1][2] * mag_hi_corrected[2];
    mag_calibrated[2] = mag_cal_.soft_iron[2][0] * mag_hi_corrected[0] +
                        mag_cal_.soft_iron[2][1] * mag_hi_corrected[1] +
                        mag_cal_.soft_iron[2][2] * mag_hi_corrected[2];
}

void Compass::update_complementary_filter(float acc_x, float acc_y, float acc_z,
                                          float gyro_x, float gyro_y, float gyro_z,
                                          float dt)
{
    comp_filter.euler_accel = calculate_euler_from_accel(acc_x, acc_y, acc_z);

    if (!comp_filter.initialized) {
        comp_filter.euler = comp_filter.euler_accel;
        comp_filter.euler_gyro = comp_filter.euler_accel;
        comp_filter.initialized = true;
        return;
    }

    comp_filter.euler_gyro.pitch += gyro_y * dt;
    comp_filter.euler_gyro.roll += gyro_x * dt;
    comp_filter.euler_gyro.yaw += gyro_z * dt;

    if (comp_filter.euler_gyro.pitch > 180.0f) {
        comp_filter.euler_gyro.pitch -= 360.0f;
    }
    if (comp_filter.euler_gyro.pitch < -180.0f) {
        comp_filter.euler_gyro.pitch += 360.0f;
    }
    if (comp_filter.euler_gyro.roll > 180.0f) {
        comp_filter.euler_gyro.roll -= 360.0f;
    }
    if (comp_filter.euler_gyro.roll < -180.0f) {
        comp_filter.euler_gyro.roll += 360.0f;
    }
    if (comp_filter.euler_gyro.yaw > 180.0f) {
        comp_filter.euler_gyro.yaw -= 360.0f;
    }
    if (comp_filter.euler_gyro.yaw < -180.0f) {
        comp_filter.euler_gyro.yaw += 360.0f;
    }

    float alpha = COMPLEMENTARY_FILTER_ALPHA;
    comp_filter.euler.pitch = alpha * (comp_filter.euler.pitch + gyro_y * dt) +
                              (1.0f - alpha) * comp_filter.euler_accel.pitch;
    comp_filter.euler.roll = alpha * (comp_filter.euler.roll + gyro_x * dt) +
                             (1.0f - alpha) * comp_filter.euler_accel.roll;
    comp_filter.euler.yaw = comp_filter.euler_gyro.yaw;

    if (comp_filter.euler.pitch > 180.0f) {
        comp_filter.euler.pitch -= 360.0f;
    }
    if (comp_filter.euler.pitch < -180.0f) {
        comp_filter.euler.pitch += 360.0f;
    }
    if (comp_filter.euler.roll > 180.0f) {
        comp_filter.euler.roll -= 360.0f;
    }
    if (comp_filter.euler.roll < -180.0f) {
        comp_filter.euler.roll += 360.0f;
    }
    if (comp_filter.euler.yaw > 180.0f) {
        comp_filter.euler.yaw -= 360.0f;
    }
    if (comp_filter.euler.yaw < -180.0f) {
        comp_filter.euler.yaw += 360.0f;
    }
}

void Compass::apply_tilt_compensation(float mag_x, float mag_y, float mag_z, float pitch,
                                      float roll, float *mag_x_corrected,
                                      float *mag_y_corrected, float *mag_z_corrected)
{
    float pitch_rad = pitch * M_PI / 180.0f;
    float roll_rad = roll * M_PI / 180.0f;

    float cos_pitch = cosf(pitch_rad);
    float sin_pitch = sinf(pitch_rad);
    float cos_roll = cosf(roll_rad);
    float sin_roll = sinf(roll_rad);

    *mag_x_corrected = mag_x * cos_pitch + mag_y * sin_roll * sin_pitch +
                       mag_z * cos_roll * sin_pitch;

    *mag_y_corrected = mag_y * cos_roll - mag_z * sin_roll;

    *mag_z_corrected = -mag_x * sin_pitch + mag_y * sin_roll * cos_pitch +
                       mag_z * cos_roll * cos_pitch;
}

void Compass::updateSensorData()
{
    struct bmi2_sens_data sensor_data = {0};
    /* Read BMI270 accelerometer and gyroscope data */
    int8_t rslt = bmi2_get_sensor_data(&sensor_data, bmi2_dev_);
    if (rslt == BMI2_OK) {
        /* Parse accelerometer data */
        float acc_x_mg = (float)sensor_data.acc.x / 16384.0f;
        float acc_y_mg = -(float)sensor_data.acc.y / 16384.0f;
        float acc_z_mg = -(float)sensor_data.acc.z / 16384.0f;

        /* Parse gyroscope data */
        float gyro_x_dps = (float)sensor_data.gyr.x / 16.4f;
        float gyro_y_dps = -(float)sensor_data.gyr.y / 16.4f;
        float gyro_z_dps = -(float)sensor_data.gyr.z / 16.4f;

        /* Update complementary filter with new sensor data */
        update_complementary_filter(acc_x_mg, acc_y_mg, acc_z_mg,
                                    gyro_x_dps, gyro_y_dps, gyro_z_dps,
                                    comp_filter.dt);
    } else {
        ESP_LOGE(TAG, "BMI270 get sensor data failed: %d", rslt);
    }

    struct bmm350_mag_temp_data mag_data = {0};
    /* Read BMM350 magnetometer data */
    int8_t mag_rslt = bmm350_get_compensated_mag_xyz_temp_data(&mag_data, &s_bmm350);
    if (mag_rslt == BMM350_OK) {
        float mag_raw[3] = {(float)mag_data.x, (float)mag_data.y, (float)mag_data.z};
        float mag_calibrated[3];

        applyMagCalibration(mag_raw, mag_calibrated);

        float mag_x = mag_calibrated[0];
        float mag_y = mag_calibrated[1];
        float mag_z = mag_calibrated[2];

        // float mag_strength = sqrtf(mag_x * mag_x + mag_y * mag_y + mag_z * mag_z);

        float uncorrected_heading = atan2f(mag_y, mag_x) * 180.0f / M_PI;
        if (mag_z < 0) {
            uncorrected_heading += 180.0f;
        }
        while (uncorrected_heading < 0.0f) {
            uncorrected_heading += 360.0f;
        }
        while (uncorrected_heading > 360.0f) {
            uncorrected_heading -= 360.0f;
        }

        float pitch = comp_filter.euler.pitch;
        float roll = comp_filter.euler.roll;

        float mag_x_corrected, mag_y_corrected, mag_z_corrected;
        apply_tilt_compensation(mag_x, mag_y, mag_z, pitch, roll,
                                &mag_x_corrected, &mag_y_corrected,
                                &mag_z_corrected);

        float corrected_heading =
            atan2f(mag_y_corrected, mag_x_corrected) * 180.0f / M_PI;

        // Normalize to 0-360 range
        while (corrected_heading < 0.0f) {
            corrected_heading += 360.0f;
        }
        while (corrected_heading >= 360.0f) {
            corrected_heading -= 360.0f;
        }

        // Smooth angle transitions to avoid sudden 180-degree flips
        // Handle wrap-around when crossing 0/360 boundary using shortest path
        float previous_heading = current_heading_.load(std::memory_order_relaxed);
        float heading_diff = corrected_heading - previous_heading;

        // Calculate shortest path angle difference (handles 0/360 wrap-around)
        // This prevents 180-degree flips when crossing the boundary
        float shortest_diff = heading_diff;
        if (heading_diff > 180.0f) {
            shortest_diff = heading_diff - 360.0f;
        } else if (heading_diff < -180.0f) {
            shortest_diff = heading_diff + 360.0f;
        }

        // Apply low-pass filter to the angle difference to smooth changes and reduce noise
        // This helps reduce offset at specific angles while maintaining responsiveness
        // Filter the difference rather than the angle itself to handle wrap-around correctly
        static float filtered_diff_prev = 0.0f;
        float filtered_diff = HEADING_FILTER_ALPHA * filtered_diff_prev +
                              (1.0f - HEADING_FILTER_ALPHA) * shortest_diff;
        filtered_diff_prev = filtered_diff;

        // Update heading using filtered difference
        float final_heading = previous_heading + filtered_diff;

        // Normalize final heading to 0-360 range
        while (final_heading < 0.0f) {
            final_heading += 360.0f;
        }
        while (final_heading >= 360.0f) {
            final_heading -= 360.0f;
        }

        current_heading_.store(final_heading, std::memory_order_relaxed);

        /* print the raw and corrected data */
        // ESP_LOGI(TAG, "Uncorrected Heading: %7.2f°-> Corrected Heading: %7.2f°, Difference: %7.2f°",
        //      uncorrected_heading, corrected_heading, uncorrected_heading - corrected_heading);
    }

}

void Compass::bmmDataThread()
{
    ESP_LOGI(TAG, "Sensor read task started");

    if (!mag_cal_.calibrated) {
        // Perform magnetometer calibration
        ESP_LOGI(TAG, "Starting magnetometer calibration...");
        ESP_LOGI(TAG, "Please rotate the device slowly in ALL directions (figure-8 pattern)");

        LvLockGuard gui_guard;

        lv_disp_load_scr(ui_CompassCorrectScreen);
        vTaskDelay(pdMS_TO_TICKS(2000)); // Give user time to prepare
        calibrateMagnetometer();
    }

    {
        LvLockGuard gui_guard;
        lv_disp_load_scr(ui_CompassScreen);
    }

    comp_filter.dt = SAMPLE_RATE_MS / 1000.0f; // 10ms = 0.01s
    comp_filter.initialized = false;

    while (bmm_running_) {
        if (!is_app_running_) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        updateSensorData();
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_RATE_MS));
    }

    ESP_LOGI(TAG, "Sensor read task stopped");
}

void Compass::updateCompassDisplay()
{
    int16_t rotation = -(int16_t)(current_heading_.load(std::memory_order_relaxed) * 10);
    lv_image_set_rotation(ui_Pointer, rotation);
}

void Compass::updateCompassThread()
{
    ESP_LOGI(TAG, "Compass update thread started");

    while (bmm_running_) {
        if (!is_app_running_) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        if (bmm_running_) {
            lv_async_call([](void *user_data) {
                Compass *self = static_cast<Compass *>(user_data);
                if (!self->bmm_running_) {
                    return;
                }
                self->updateCompassDisplay();
            }, this);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    ESP_LOGI(TAG, "Compass update thread stopped");
}

// NVS storage methods implementation
bool Compass::saveCalibrationToNVS()
{
    ESP_LOGI(TAG, "Starting NVS save operation...");

    nvs_handle_t nvs_handle;
    esp_err_t err;

    // Check if NVS is initialized
    ESP_LOGI(TAG, "Opening NVS handle for namespace: %s", NVS_NAMESPACE);

    // Open NVS handle
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle: %s (0x%x)", esp_err_to_name(err), err);

        // Try to initialize NVS if it's not initialized
        if (err == ESP_ERR_NVS_NOT_INITIALIZED) {
            ESP_LOGI(TAG, "NVS not initialized, attempting to initialize...");
            err = nvs_flash_init();
            if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
                ESP_LOGI(TAG, "Erasing NVS flash and reinitializing...");
                ESP_ERROR_CHECK(nvs_flash_erase());
                err = nvs_flash_init();
            }
            ESP_ERROR_CHECK(err);

            // Try opening NVS again
            err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Error opening NVS handle after init: %s (0x%x)", esp_err_to_name(err), err);
                return false;
            }
        } else {
            return false;
        }
    }

    ESP_LOGI(TAG, "NVS handle opened successfully");

    // Save hard iron calibration
    ESP_LOGI(TAG, "Saving hard iron calibration data...");
    err = nvs_set_blob(nvs_handle, NVS_KEY_HARD_IRON, mag_cal_.hard_iron, sizeof(mag_cal_.hard_iron));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error saving hard iron calibration: %s (0x%x)", esp_err_to_name(err), err);
        nvs_close(nvs_handle);
        return false;
    }
    ESP_LOGI(TAG, "Hard iron calibration saved successfully");

    // Save soft iron calibration
    ESP_LOGI(TAG, "Saving soft iron calibration data...");
    err = nvs_set_blob(nvs_handle, NVS_KEY_SOFT_IRON, mag_cal_.soft_iron, sizeof(mag_cal_.soft_iron));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error saving soft iron calibration: %s (0x%x)", esp_err_to_name(err), err);
        nvs_close(nvs_handle);
        return false;
    }
    ESP_LOGI(TAG, "Soft iron calibration saved successfully");

    // Save calibration status
    ESP_LOGI(TAG, "Saving calibration status...");
    err = nvs_set_u8(nvs_handle, NVS_KEY_CALIBRATED, mag_cal_.calibrated ? 1 : 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error saving calibration status: %s (0x%x)", esp_err_to_name(err), err);
        nvs_close(nvs_handle);
        return false;
    }
    ESP_LOGI(TAG, "Calibration status saved successfully");

    // Commit changes
    ESP_LOGI(TAG, "Committing NVS changes...");
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error committing NVS changes: %s (0x%x)", esp_err_to_name(err), err);
        nvs_close(nvs_handle);
        return false;
    }
    ESP_LOGI(TAG, "NVS changes committed successfully");

    nvs_close(nvs_handle);
    ESP_LOGI(TAG, "Calibration data saved to NVS successfully");
    return true;
}

bool Compass::loadCalibrationFromNVS()
{
    nvs_handle_t nvs_handle;
    esp_err_t err;

    // Open NVS handle
    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGI(TAG, "NVS namespace not found, using default calibration");
        } else {
            ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(err));
        }
        return false;
    }

    // Load hard iron calibration
    size_t hard_iron_size = sizeof(mag_cal_.hard_iron);
    err = nvs_get_blob(nvs_handle, NVS_KEY_HARD_IRON, mag_cal_.hard_iron, &hard_iron_size);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Error loading hard iron calibration: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return false;
    }

    // Load soft iron calibration
    size_t soft_iron_size = sizeof(mag_cal_.soft_iron);
    err = nvs_get_blob(nvs_handle, NVS_KEY_SOFT_IRON, mag_cal_.soft_iron, &soft_iron_size);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Error loading soft iron calibration: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return false;
    }

    // Load calibration status
    uint8_t calibrated = 0;
    err = nvs_get_u8(nvs_handle, NVS_KEY_CALIBRATED, &calibrated);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Error loading calibration status: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return false;
    }

    mag_cal_.calibrated = (calibrated != 0);

    nvs_close(nvs_handle);

    ESP_LOGI(TAG, "Calibration data loaded from NVS:");
    ESP_LOGI(TAG, "  Hard Iron: X=%.2f, Y=%.2f, Z=%.2f",
             mag_cal_.hard_iron[0], mag_cal_.hard_iron[1], mag_cal_.hard_iron[2]);
    ESP_LOGI(TAG, "  Soft Iron: X=%.3f, Y=%.3f, Z=%.3f",
             mag_cal_.soft_iron[0][0], mag_cal_.soft_iron[1][1], mag_cal_.soft_iron[2][2]);
    ESP_LOGI(TAG, "  Calibrated: %s", mag_cal_.calibrated ? "Yes" : "No");

    return true;
}

void Compass::clearCalibrationFromNVS()
{
    nvs_handle_t nvs_handle;
    esp_err_t err;

    // Open NVS handle
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(err));
        return;
    }

    // Erase all calibration keys
    nvs_erase_key(nvs_handle, NVS_KEY_HARD_IRON);
    nvs_erase_key(nvs_handle, NVS_KEY_SOFT_IRON);
    nvs_erase_key(nvs_handle, NVS_KEY_CALIBRATED);

    // Commit changes
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error committing NVS changes: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI(TAG, "Calibration data cleared from NVS");
    }

    nvs_close(nvs_handle);
}

// Async NVS operations
void Compass::saveCalibrationToNVSAsync()
{
    ESP_LOGI(TAG, "Starting async NVS save operation...");

    // Create a task to handle NVS operations in background
    xTaskCreate([](void *param) {
        Compass *compass = static_cast<Compass *>(param);
        if (compass) {
            bool result = compass->saveCalibrationToNVS();
            ESP_LOGI(TAG, "Async NVS save completed with result: %s", result ? "success" : "failed");
        }
        vTaskDelete(NULL);
    }, "nvs_save_task", 4096, this, 5, NULL);
}

void Compass::loadCalibrationFromNVSAsync()
{
    ESP_LOGI(TAG, "Starting async NVS load operation...");

    // Create a task to handle NVS operations in background
    xTaskCreate([](void *param) {
        Compass *compass = static_cast<Compass *>(param);
        if (compass) {
            bool result = compass->loadCalibrationFromNVS();
            ESP_LOGI(TAG, "Async NVS load completed with result: %s", result ? "success" : "failed");
        }
        vTaskDelete(NULL);
    }, "nvs_load_task", 4096, this, 5, NULL);
}

ESP_UTILS_REGISTER_PLUGIN_WITH_CONSTRUCTOR(systems::base::App, Compass, "Compass", []()
{
    return std::shared_ptr<Compass>(Compass::requestInstance(), [](Compass * p) {});
})

} // namespace esp_brookesia::apps

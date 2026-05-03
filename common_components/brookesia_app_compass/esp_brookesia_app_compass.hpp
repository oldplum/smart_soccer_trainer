/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ESP_BROOKESIA_APP_COMPASS_HPP
#define ESP_BROOKESIA_APP_COMPASS_HPP

#include "bmi270_api.h"
#include "BMM350_SensorAPI/bmm350_defs.h"
#include "BMM350_SensorAPI/bmm350.h"
#include "BMM350_SensorAPI/examples/common/common.h"
#include "boost/signals2/signal.hpp"
#include "boost/thread.hpp"
#include "brookesia/system_phone/app.hpp"
#include "i2c_bus.h"
#include "lvgl.h"

namespace esp_brookesia::apps {

/* Magnetometer calibration structure */
typedef struct {
    float hard_iron[3];    // Hard iron offset [x, y, z]
    float soft_iron[3][3]; // Soft iron matrix (default identity matrix)
    bool calibrated;       // Calibration flag
} mag_calibration_t;

typedef struct {
    float pitch;    // Pitch angle (degrees) - rotation around Y axis
    float roll;     // Roll angle (degrees) - rotation around X axis
    float yaw;      // Yaw angle (degrees) - rotation around Z axis
} euler_angles_t;

/* Complementary filter state */
typedef struct {
    euler_angles_t euler;           // Current Euler angles
    euler_angles_t euler_gyro;      // Gyroscope integrated Euler angles
    euler_angles_t euler_accel;     // Accelerometer Euler angles
    float dt;                       // Time interval (seconds)
    bool initialized;               // Initialization flag
} complementary_filter_t;

class Compass: public systems::phone::App {
public:
    Compass(const Compass &) = delete;
    Compass(Compass &&) = delete;
    Compass &operator=(const Compass &) = delete;
    Compass &operator=(Compass &&) = delete;

    ~Compass();

    static Compass *requestInstance();
    void setCorrect(bool correct);
    void externalBack();
    bool isCalibrated() const;

    // NVS storage methods
    bool saveCalibrationToNVS();
    bool loadCalibrationFromNVS();
    void clearCalibrationFromNVS();

    // Async NVS operations (for UI thread safety)
    void saveCalibrationToNVSAsync();
    void loadCalibrationFromNVSAsync();

protected:
    bool run() override;
    bool back() override;
    bool pause() override;
    bool close() override;
    bool init() override;
    bool deinit() override;
    bool resume() override;
    void destroyCompassUI();
    void createCompassUI();

private:
    inline static Compass *_instance = nullptr;
    Compass();

    // BMM350 related methods
    bool initSensors();
    bool deinitSensors();
    void calibrateMagnetometer();
    bool isCalibrationSufficient(float mag_min[3], float mag_max[3],
                                 float pitch_min, float pitch_max,
                                 float roll_min, float roll_max,
                                 uint8_t octant_coverage, int sample_count);
    void applyMagCalibration(const float mag_raw[3], float mag_calibrated[3]);
    euler_angles_t calculate_euler_from_accel(float acc_x, float acc_y, float acc_z);
    void update_complementary_filter(float acc_x, float acc_y, float acc_z,
                                     float gyro_x, float gyro_y, float gyro_z,
                                     float dt);
    void apply_tilt_compensation(float mag_x, float mag_y, float mag_z, float pitch,
                                 float roll, float *mag_x_corrected,
                                 float *mag_y_corrected, float *mag_z_corrected);
    void updateSensorData();
    void bmmDataThread();
    void updateCompassDisplay();
    void updateCompassThread();

    // Complementary filter state
    complementary_filter_t comp_filter = {0};

    bool is_initialized_ = false;
    bool is_app_running_ = false;
    // BMM350 sensor handles
    bmi270_handle_t bmi_handle_;
    i2c_bus_handle_t i2c_bus_;
    struct bmi2_dev *bmi2_dev_;
    struct bmm350_dev s_bmm350 = {0};

    // Calibration data
    mag_calibration_t mag_cal_;

    // Current sensor data
    std::atomic<float> current_heading_;
    struct bmm350_mag_temp_data current_mag_data_;

    // Task handle for sensor reading
    bool bmm_running_ = false;
    boost::thread bmm_data_thread;
    boost::thread update_compass_thread;
};

} // namespace esp_brookesia::apps

#endif

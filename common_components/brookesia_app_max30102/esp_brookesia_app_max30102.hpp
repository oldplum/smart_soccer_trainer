/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ESP_BROOKESIA_APP_MAX30102_HPP
#define ESP_BROOKESIA_APP_MAX30102_HPP

#include "brookesia/system_phone/app.hpp"
#include "boost/thread.hpp"
#include <atomic>
#include <cstdint>
#include "i2c_bus.h"

namespace esp_brookesia::apps {

/**
 * @brief MAX30102 Heart Rate and SpO2 Sensor Application
 *
 * Reads heart rate and blood oxygen saturation data from MAX30102 sensor
 * via I2C, and syncs to the global sensor data buffer.
 *
 * Hardware: MAX30102 Heart Rate and SpO2 Sensor
 * Protocol: I2C
 * I2C Address: 0x57
 */
class Max30102 : public systems::phone::App {
public:
    Max30102(const Max30102 &) = delete;
    Max30102(Max30102 &&) = delete;
    Max30102 &operator=(const Max30102 &) = delete;
    Max30102 &operator=(Max30102 &&) = delete;

    ~Max30102();

    /** @brief Get or create the singleton instance */
    static Max30102 *requestInstance();

    /** @brief Start MAX30102 sensor data collection */
    bool startSensorCollection();

    /** @brief Stop MAX30102 sensor data collection */
    void stopSensorCollection();

    /** @brief Check if MAX30102 sensor is online and producing valid data */
    bool isSensorOnline() const;

protected:
    bool run() override;
    bool back() override;
    bool close() override;
    bool init() override;
    bool deinit() override;
    bool pause() override;
    bool resume() override;

private:
    inline static Max30102 *_instance = nullptr;
    Max30102();

    /* Hardware initialization */
    bool initSensors();
    bool deinitSensors();

    /* Data acquisition thread */
    void max30102DataThread();

    /* Sensor operations */
    bool writeRegister(uint8_t reg, uint8_t data);
    bool readRegister(uint8_t reg, uint8_t *data);
    bool readFIFO(uint32_t *red, uint32_t *ir);

    /* I2C bus handles */
    i2c_bus_handle_t i2c_bus_{nullptr};
    i2c_bus_device_handle_t i2c_dev_{nullptr};

    /* Sensor state */
    std::atomic<bool> sensor_online_{false};
    std::atomic<bool> running_{false};
    boost::thread data_thread_;

    /* Current readings */
    int heart_rate_ = 0;
    int spo2_ = 0;
};

} // namespace esp_brookesia::apps

#endif // ESP_BROOKESIA_APP_MAX30102_HPP

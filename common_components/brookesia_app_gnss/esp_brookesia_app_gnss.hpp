/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ESP_BROOKESIA_APP_GNSS_HPP
#define ESP_BROOKESIA_APP_GNSS_HPP

#include "brookesia/system_phone/app.hpp"
#include "boost/thread.hpp"
#include "driver/uart.h"
#include <atomic>
#include <cstdint>

namespace esp_brookesia::apps {

/**
 * @brief GNSS (GPS+Beidou) application using H802 module
 *
 * Reads NMEA-0183 sentences from the H802 GNSS module via UART,
 * parses position/velocity/time data, and syncs to the global
 * sensor data buffer (g_soccer_sensor_data).
 *
 * Hardware: H802 GPS+Beidou dual-mode module (GB/18FS)
 * Protocol: NMEA-0183 @ 9600 baud, 8-N-1
 * Interface: UART (TX→GPIO4 RX, RX→GPIO5 TX)
 */
class Gnss: public systems::phone::App {
public:
    Gnss(const Gnss &) = delete;
    Gnss(Gnss &&) = delete;
    Gnss &operator=(const Gnss &) = delete;
    Gnss &operator=(Gnss &&) = delete;

    ~Gnss();

    /** @brief Get or create the singleton instance */
    static Gnss *requestInstance();

    /** @brief Start GNSS data collection (UART + NMEA parsing thread) */
    bool startSensorCollection();

    /** @brief Stop GNSS data collection */
    void stopSensorCollection();

    /** @brief Check if GNSS module is online and producing valid data */
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
    inline static Gnss *_instance = nullptr;
    Gnss();

    /* Hardware initialization */
    bool initUart();
    bool deinitUart();

    /* Data acquisition thread */
    void gnssDataThread();

    /* NMEA sentence parsing */
    bool parseNmeaSentence(char *sentence);
    bool parseRMC(const char *fields[], int field_count);
    bool parseGGA(const char *fields[], int field_count);

    /** Convert NMEA coordinate format (ddmm.mmmm) to decimal degrees */
    static double nmeaToDecimalDegrees(double nmea_coord, char direction);

    /* UART configuration */
    static constexpr uart_port_t UART_PORT = UART_NUM_1;
    static constexpr int UART_BAUD_RATE = 9600;
    static constexpr int UART_RX_PIN = GPIO_NUM_4;
    static constexpr int UART_TX_PIN = GPIO_NUM_5;
    static constexpr int UART_RX_BUF_SIZE = 1024;

    /* NMEA parsing constants */
    static constexpr size_t NMEA_LINE_MAX = 256;
    static constexpr int NMEA_MAX_FIELDS = 20;

    /* Sensor online timeout: mark offline if no valid sentence within 5 seconds */
    static constexpr uint32_t SENSOR_TIMEOUT_MS = 5000;

    /* Thread control */
    std::atomic<bool> gnss_running_{false};
    boost::thread gnss_data_thread;

    /* State */
    std::atomic<bool> sensor_online_{false};
    uint32_t last_valid_data_ms_ = 0;
};

} // namespace esp_brookesia::apps

#endif // ESP_BROOKESIA_APP_GNSS_HPP

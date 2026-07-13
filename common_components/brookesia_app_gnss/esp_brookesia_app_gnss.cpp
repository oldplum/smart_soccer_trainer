/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "esp_brookesia_app_gnss.hpp"
#include "esp_err.h"
#include "esp_lib_utils.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include "../../../main/soccer_data_sync.h"

static const char *TAG = "GnssApp";

using namespace esp_brookesia::systems::phone;

#define APP_NAME "GNSS"

namespace esp_brookesia::apps {

/* --------------------------------------------------------------------------
 * Brookesia App metadata (no UI — data collection only)
 * -------------------------------------------------------------------------- */
constexpr systems::base::App::Config GNSS_CORE_DATA = {
    .name = APP_NAME,
    .launcher_icon = gui::StyleImage::IMAGE(nullptr),
    .screen_size = gui::StyleSize::RECT_PERCENT(100, 100),
    .flags = {
        .enable_default_screen = 1,
        .enable_recycle_resource = 0,
        .enable_resize_visual_area = 1,
    },
};
constexpr App::Config GNSS_APP_DATA = {
    .app_launcher_page_index = 0,
    .flags = {
        .enable_navigation_gesture = 1,
    },
};

/* --------------------------------------------------------------------------
 * Constructor / Destructor
 * -------------------------------------------------------------------------- */
Gnss::Gnss()
    : App(GNSS_CORE_DATA, GNSS_APP_DATA)
{
    ESP_LOGI(TAG, "Gnss app constructor");
}

Gnss::~Gnss()
{
    ESP_UTILS_LOG_TRACE_GUARD_WITH_THIS();
    ESP_LOGI(TAG, "Gnss app destructor");
    deinit();
}

Gnss *Gnss::requestInstance()
{
    if (_instance == nullptr) {
        _instance = new Gnss();
    }
    return _instance;
}

/* --------------------------------------------------------------------------
 * App lifecycle (Brookesia framework callbacks)
 * -------------------------------------------------------------------------- */
bool Gnss::init()
{
    ESP_LOGI(TAG, "Initializing Gnss app");
    return true;
}

bool Gnss::deinit()
{
    ESP_LOGI(TAG, "Deinitializing Gnss app");
    gnss_running_ = false;
    if (gnss_data_thread.joinable()) {
        gnss_data_thread.join();
    }
    deinitUart();
    return true;
}

bool Gnss::run()
{
    ESP_LOGI(TAG, "Running Gnss app");
    /* Sensor collection is started externally via startSensorCollection() */
    return true;
}

bool Gnss::back()
{
    ESP_LOGI(TAG, "Gnss app back");
    return close();
}

bool Gnss::close()
{
    ESP_LOGI(TAG, "Closing Gnss app");
    sensor_online_ = false;
    gnss_running_ = false;
    if (gnss_data_thread.joinable()) {
        gnss_data_thread.join();
    }
    return true;
}

bool Gnss::pause()
{
    ESP_LOGI(TAG, "Gnss app pause");
    return true;
}

bool Gnss::resume()
{
    ESP_LOGI(TAG, "Gnss app resume");
    return true;
}

/* --------------------------------------------------------------------------
 * Public sensor control API
 * -------------------------------------------------------------------------- */
bool Gnss::startSensorCollection()
{
    ESP_LOGI(TAG, "Starting GNSS sensor collection...");

    sensor_online_ = false;

    if (!initUart()) {
        ESP_LOGE(TAG, "Failed to initialize UART for GNSS module");
        return false;
    }

    gnss_running_ = true;
    gnss_data_thread = boost::thread(&Gnss::gnssDataThread, this);

    ESP_LOGI(TAG, "GNSS sensor collection started");
    return true;
}

void Gnss::stopSensorCollection()
{
    ESP_LOGI(TAG, "Stopping GNSS sensor collection");
    sensor_online_ = false;
    gnss_running_ = false;
    if (gnss_data_thread.joinable()) {
        gnss_data_thread.join();
    }
    deinitUart();
}

bool Gnss::isSensorOnline() const
{
    return sensor_online_.load(std::memory_order_relaxed);
}

/* --------------------------------------------------------------------------
 * UART initialization / deinitialization
 * -------------------------------------------------------------------------- */
bool Gnss::initUart()
{
    ESP_LOGI(TAG, "Initializing UART%u for H802 GNSS (baud=%d, rx=GPIO%d, tx=GPIO%d)",
             UART_PORT, UART_BAUD_RATE, UART_RX_PIN, UART_TX_PIN);

    const uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_driver_install(UART_PORT, UART_RX_BUF_SIZE, 0, 0, nullptr, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(ret));
        return false;
    }

    ret = uart_param_config(UART_PORT, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(ret));
        uart_driver_delete(UART_PORT);
        return false;
    }

    ret = uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART set pin failed: %s", esp_err_to_name(ret));
        uart_driver_delete(UART_PORT);
        return false;
    }

    /* Flush any stale data in the RX buffer */
    uart_flush_input(UART_PORT);

    ESP_LOGI(TAG, "UART initialized successfully");
    return true;
}

bool Gnss::deinitUart()
{
    ESP_LOGI(TAG, "Deinitializing UART");
    uart_driver_delete(UART_PORT);
    return true;
}

/* --------------------------------------------------------------------------
 * NMEA sentence parsing
 * -------------------------------------------------------------------------- */

/**
 * @brief Convert NMEA coordinate format to decimal degrees
 *
 * NMEA format: ddmm.mmmm for latitude, dddmm.mmmm for longitude
 * - Latitude:  dd = degrees (2 digits), mm.mmmm = minutes
 * - Longitude: ddd = degrees (3 digits), mm.mmmm = minutes
 *
 * @param nmea_coord  Coordinate in NMEA format (e.g. 3112.3456 = 31°12.3456')
 * @param direction   'N'/'S' for latitude, 'E'/'W' for longitude
 * @return            Decimal degrees (negative for South/West)
 */
double Gnss::nmeaToDecimalDegrees(double nmea_coord, char direction)
{
    if (nmea_coord < 1.0) {
        return 0.0;  // Invalid coordinate
    }

    /* Determine number of integer digits before the decimal-minutes part:
     * Latitude:  ddmm.mmmm → 2 degree digits + 2 minute digits = at least 4 digits
     * Longitude: dddmm.mmmm → 3 degree digits + 2 minute digits = at least 5 digits
     *
     * General approach: integer part / 100 gives degrees, remainder is minutes.
     * This works because NMEA always uses exactly 2 minute digits before the decimal. */
    double degrees = floor(nmea_coord / 100.0);
    double minutes = nmea_coord - (degrees * 100.0);
    double decimal = degrees + (minutes / 60.0);

    if (direction == 'S' || direction == 'W') {
        decimal = -decimal;
    }

    return decimal;
}

bool Gnss::parseRMC(const char *fields[], int field_count)
{
    /* $GNRMC field layout:
     * [0]  "$GNRMC"    Talker + sentence ID
     * [1]  time         UTC time HHMMSS.ss
     * [2]  status       A=valid, V=invalid
     * [3]  lat          Latitude  ddmm.mmmm
     * [4]  ns           N/S indicator
     * [5]  lon          Longitude dddmm.mmmm
     * [6]  ew           E/W indicator
     * [7]  speed        Speed over ground (knots)
     * [8]  course       Course over ground (degrees true)
     * [9]  date         Date DDMMYY
     * [10] mag_var      Magnetic variation (optional)
     * [11] mag_dir      E/W for magnetic variation
     * [12] mode         A=autonomous, D=DGPS, E=DR (NMEA 4.0+)
     *
     * Minimum required fields for a useful fix: 0–8 (through course)
     */
    if (field_count < 9) {
        return false;  // Not enough fields
    }

    /* Check validity */
    if (fields[2] == nullptr || fields[2][0] != 'A') {
        /* Status is 'V' (void) — data fields may be empty.
         * Still mark GNSS as receiving data, but data is invalid. */
        g_soccer_sensor_data.gnss_valid = 0;
        return true;  // Sentence was parsed, but data is not valid
    }

    /* Parse latitude */
    if (fields[3] == nullptr || fields[4] == nullptr || fields[3][0] == '\0') {
        g_soccer_sensor_data.gnss_valid = 0;
        return true;
    }
    double raw_lat = strtod(fields[3], nullptr);
    g_soccer_sensor_data.latitude = nmeaToDecimalDegrees(raw_lat, fields[4][0]);

    /* Parse longitude */
    if (fields[5] == nullptr || fields[6] == nullptr || fields[5][0] == '\0') {
        g_soccer_sensor_data.gnss_valid = 0;
        return true;
    }
    double raw_lon = strtod(fields[5], nullptr);
    g_soccer_sensor_data.longitude = nmeaToDecimalDegrees(raw_lon, fields[6][0]);

    /* Parse speed (knots → km/h) */
    if (fields[7] != nullptr && fields[7][0] != '\0') {
        float speed_knots = strtof(fields[7], nullptr);
        g_soccer_sensor_data.speed = speed_knots * 1.852f;  // knots to km/h
    } else {
        g_soccer_sensor_data.speed = 0.0f;
    }

    /* Parse course over ground */
    if (fields[8] != nullptr && fields[8][0] != '\0') {
        g_soccer_sensor_data.course = strtof(fields[8], nullptr);
    } else {
        g_soccer_sensor_data.course = 0.0f;
    }

    g_soccer_sensor_data.gnss_valid = 1;
    return true;
}

bool Gnss::parseGGA(const char *fields[], int field_count)
{
    /* $GNGGA field layout:
     * [0]  "$GNGGA"    Talker + sentence ID
     * [1]  time         UTC time HHMMSS.ss
     * [2]  lat          Latitude  ddmm.mmmm
     * [3]  ns           N/S indicator
     * [4]  lon          Longitude dddmm.mmmm
     * [5]  ew           E/W indicator
     * [6]  quality      Fix quality: 0=invalid, 1=GPS, 2=DGPS, 4=RTK fixed, 5=RTK float
     * [7]  satellites   Number of satellites in use
     * [8]  hdop         Horizontal dilution of precision
     * [9]  altitude     Altitude above MSL (meters)
     * [10] alt_unit     Always 'M' (meters)
     * [11] geoid_sep    Geoid separation (meters)
     * [12] geoid_unit   Always 'M'
     * [13] dgps_age     Age of DGPS data (seconds)
     * [14] dgps_id      DGPS station ID
     *
     * Minimum required: 0–9 (through altitude)
     */
    if (field_count < 10) {
        return false;
    }

    /* Parse fix quality */
    if (fields[6] != nullptr && fields[6][0] != '\0') {
        g_soccer_sensor_data.fix_quality = (uint8_t)atoi(fields[6]);
    }

    /* Parse satellite count */
    if (fields[7] != nullptr && fields[7][0] != '\0') {
        g_soccer_sensor_data.satellites = (uint8_t)atoi(fields[7]);
    } else {
        g_soccer_sensor_data.satellites = 0;
    }

    /* Parse altitude */
    if (fields[9] != nullptr && fields[9][0] != '\0') {
        g_soccer_sensor_data.altitude = strtof(fields[9], nullptr);
    } else {
        g_soccer_sensor_data.altitude = 0.0f;
    }

    return true;
}

/**
 * @brief Parse a single NMEA sentence
 *
 * Validates the sentence checksum (if present), then dispatches to the
 * appropriate sentence parser based on the talker+sentence ID.
 *
 * Supported sentences:
 * - $GNRMC / $GPRMC / $BDRMC — Recommended Minimum Navigation Information
 * - $GNGGA / $GPGGA / $BDGGA — Global Positioning System Fix Data
 */
bool Gnss::parseNmeaSentence(char *sentence)
{
    /* Must start with '$' */
    if (sentence[0] != '$') {
        return false;
    }

    /* Minimum length: "$GPGGA," = 7 chars */
    size_t len = strlen(sentence);
    if (len < 7) {
        return false;
    }

    /* --- Checksum verification ---
     * NMEA format: $<data>*<checksum>\r\n
     * Checksum is XOR of all characters between '$' and '*' (exclusive).
     */
    char *asterisk = strchr(sentence, '*');
    uint8_t expected_checksum = 0;

    if (asterisk != nullptr) {
        /* Parse expected checksum (2 hex digits) */
        char hex[3] = {0};
        hex[0] = asterisk[1];
        hex[1] = asterisk[2];
        expected_checksum = (uint8_t)strtol(hex, nullptr, 16);

        /* Compute actual checksum (XOR chars between $ and *) */
        uint8_t computed = 0;
        for (char *p = sentence + 1; p < asterisk; p++) {
            computed ^= (uint8_t)*p;
        }

        if (computed != expected_checksum) {
            /* Checksum mismatch — discard sentence */
            return false;
        }

        /* Terminate the data portion at the asterisk for field parsing */
        *asterisk = '\0';
    }
    /* If no '*' found, accept the sentence without checksum verification
     * (some modules omit checksum in debug/reduced modes). */

    /* --- Split into comma-separated fields --- */
    const char *fields[NMEA_MAX_FIELDS] = {nullptr};
    int field_count = 0;

    char *saveptr = nullptr;
    char *token = strtok_r(sentence + 1, ",", &saveptr);  // Skip leading '$'

    while (token != nullptr && field_count < NMEA_MAX_FIELDS) {
        /* Shift fields array to make room for talker+sentence ID at [0] */
        if (field_count == 0) {
            /* Reconstruct the full talker ID as "$<token>" */
            static char talker_id[16];
            snprintf(talker_id, sizeof(talker_id), "$%s", token);
            fields[field_count++] = talker_id;
        } else {
            fields[field_count++] = token;
        }
        token = strtok_r(nullptr, ",", &saveptr);
    }

    if (field_count < 2) {
        return false;
    }

    /* --- Dispatch based on sentence type ---
     * Check the last 3 characters of the talker ID to identify sentence type.
     * The talker prefix (first 2 chars after '$') varies:
     *   GP = GPS only,  BD = Beidou only,  GN = Multi-constellation
     * We accept all variants.
     */
    const char *sentence_id = fields[0];               // e.g. "$GNRMC"
    size_t id_len = strlen(sentence_id);

    if (id_len < 6) {
        return false;
    }

    /* Point to the 3-char sentence type (last 3 chars of talker ID) */
    const char *type = sentence_id + id_len - 3;

    if (strcmp(type, "RMC") == 0) {
        return parseRMC(fields, field_count);
    } else if (strcmp(type, "GGA") == 0) {
        return parseGGA(fields, field_count);
    }

    /* Other sentence types ($GNGLL, $GNVTG, $GNGSA, $GNGSV, etc.)
     * are received but not parsed — they don't add new information
     * beyond what RMC + GGA already provide. */
    return false;
}

/* --------------------------------------------------------------------------
 * GNSS data acquisition thread
 * -------------------------------------------------------------------------- */
void Gnss::gnssDataThread()
{
    ESP_LOGI(TAG, "GNSS data thread started");
    ESP_LOGI(TAG, "Waiting for H802 module to acquire satellite fix...");

    char line_buf[NMEA_LINE_MAX] = {0};
    size_t line_idx = 0;
    uint32_t last_status_log_ms = 0;

    /* Initialize GNSS fields in the global buffer */
    g_soccer_sensor_data.gnss_valid = 0;
    g_soccer_sensor_data.fix_quality = 0;
    g_soccer_sensor_data.satellites = 0;
    g_soccer_sensor_data.latitude = 0.0;
    g_soccer_sensor_data.longitude = 0.0;
    g_soccer_sensor_data.altitude = 0.0f;
    g_soccer_sensor_data.speed = 0.0f;
    g_soccer_sensor_data.course = 0.0f;

    while (gnss_running_) {
        uint8_t byte;
        int len = uart_read_bytes(UART_PORT, &byte, 1, pdMS_TO_TICKS(50));

        if (len < 0) {
            ESP_LOGE(TAG, "UART read error: %d", len);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        if (len == 0) {
            /* Timeout — no data available. Check if sensor has been silent too long. */
            uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
            if (sensor_online_.load(std::memory_order_relaxed) &&
                (now_ms - last_valid_data_ms_) > SENSOR_TIMEOUT_MS) {
                ESP_LOGW(TAG, "GNSS data timeout (no valid sentence for %lu ms)",
                         SENSOR_TIMEOUT_MS);
                sensor_online_ = false;
                g_soccer_sensor_data.gnss_valid = 0;
            }
            continue;
        }

        /* Process one byte */
        char c = (char)byte;

        if (c == '\n') {
            /* End of NMEA sentence */
            if (line_idx > 0) {
                line_buf[line_idx] = '\0';
                line_idx = 0;

                if (parseNmeaSentence(line_buf)) {
                    /* Update timestamp when we get valid data */
                    g_soccer_sensor_data.timestamp =
                        (uint32_t)(esp_timer_get_time() / 1000);
                    last_valid_data_ms_ = g_soccer_sensor_data.timestamp;

                    if (!sensor_online_.load(std::memory_order_relaxed)) {
                        sensor_online_ = true;
                        ESP_LOGI(TAG, "GNSS module online — receiving valid NMEA data");
                    }

                    /* Periodic status logging */
                    if (last_valid_data_ms_ - last_status_log_ms >= 5000) {
                        ESP_LOGI(TAG, "GNSS Fix: sat=%u q=%u lat=%.6f lon=%.6f alt=%.1fm speed=%.1fkm/h",
                                 g_soccer_sensor_data.satellites,
                                 g_soccer_sensor_data.fix_quality,
                                 g_soccer_sensor_data.latitude,
                                 g_soccer_sensor_data.longitude,
                                 g_soccer_sensor_data.altitude,
                                 g_soccer_sensor_data.speed);
                        last_status_log_ms = last_valid_data_ms_;
                    }
                }
            }
        } else if (c == '\r') {
            /* Skip carriage return */
            continue;
        } else if (c == '$' && line_idx > 0) {
            /* Unexpected start of new sentence — reset buffer */
            ESP_LOGW(TAG, "NMEA buffer overflow or corruption, resetting");
            line_buf[0] = '$';
            line_idx = 1;
        } else {
            /* Append to line buffer */
            if (line_idx < NMEA_LINE_MAX - 1) {
                line_buf[line_idx++] = c;
            } else {
                /* Buffer overflow protection */
                ESP_LOGW(TAG, "NMEA line too long, discarding buffer");
                line_idx = 0;
            }
        }
    }

    ESP_LOGI(TAG, "GNSS data thread stopped");
}

/* --------------------------------------------------------------------------
 * Plugin registration (Brookesia framework)
 * -------------------------------------------------------------------------- */
ESP_UTILS_REGISTER_PLUGIN_WITH_CONSTRUCTOR(systems::base::App, Gnss, "GNSS", []()
{
    return std::shared_ptr<Gnss>(Gnss::requestInstance(), [](Gnss *p) {});
})

} // namespace esp_brookesia::apps

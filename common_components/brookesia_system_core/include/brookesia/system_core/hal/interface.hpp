/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <string_view>
#include "boost/signals2/signal.hpp"

//----------------  Interface ----------------
/**
 * @brief IMU sensor interface.
 *
 * Consumers subscribe to `gesture_signal` to receive gesture events. The concrete
 * driver emits this signal when a gesture is recognized.
 */
struct IIMUSensor {
    static constexpr std::string_view interface_name = "IIMUSensor";

    /**
     * @brief Gesture types.
     */
    enum GestureType {
        UNKNOWN_GESTURE = 0,      ///< Unknown gesture
        ANY_MOTION,               ///< Any motion / small movement
        CIRCLE_CLOCKWISE,         ///< Circle clockwise
        CIRCLE_ANTICLOCKWISE,     ///< Circle counterclockwise
    };

    /**
     * @brief Gesture event signal.
     *
     * Subscribers receive the recognized gesture type. The callback execution context
     * depends on the implementation. Keep handlers short to avoid blocking.
     *
     * @param type Recognized gesture type
     */
    boost::signals2::signal<void(GestureType type)> gesture_signal;

    virtual bool read_accel(std::array<float, 3> &mps2, uint64_t &ts_us)
    {
        (void)mps2;
        (void)ts_us;
        return false;
    }
    virtual bool read_gyro(std::array<float, 3> &radps, uint64_t &ts_us)
    {
        (void)radps;
        (void)ts_us;
        return false;
    }
    virtual bool read_mag(std::array<float, 3> &uT, uint64_t &ts_us)
    {
        (void)uT;
        (void)ts_us;
        return false;
    }
    virtual bool read_temperature(float &temp_c, uint64_t &ts_us)
    {
        (void)temp_c;
        (void)ts_us;
        return false;
    }
};

/**
 * @brief Touch sensor interface.
 */
struct ITouchSensor {
    static constexpr std::string_view interface_name = "ITouchSensor";

    /**
     * @brief Get the touch button handle.
     *
     * The handle can be used with relevant APIs to register events, etc.
     *
     * @return button_handle_t Button handle (ownership not transferred)
     */
    virtual void *get_button_handle() = 0;

};

/**
 * @brief LED indicator interface.
 */
struct ILED_Indicator {
    static constexpr std::string_view interface_name = "ILED_Indicator";

    /**
     * @brief Indicator blink patterns.
     */
    enum BlinkType {
        BLINK_LOW_POWER = 0,      ///< Low battery indication
        BLINK_DEVELOP_MODE,       ///< Development/debug mode indication
        BLINK_TOUCH_PRESS_DOWN,   ///< Touch press indication
        BLINK_WIFI_CONNECTED,     ///< Wi‑Fi connected
        BLINK_WIFI_DISCONNECTED,  ///< Wi‑Fi disconnected
        BLINK_MAX,
    };

    /**
     * @brief Register Wi‑Fi event callbacks to update the indicator based on network state.
     */
    virtual void register_wifi_event(void) = 0;

    /**
     * @brief Start blinking with the specified pattern.
     * @param type Blink pattern
     */
    virtual void startBlink(BlinkType type) = 0;

    /**
     * @brief Stop blinking of the specified pattern.
     * @param type Blink pattern
     */
    virtual void stopBlink(BlinkType type) = 0;
};

/**
 * @brief File system interface.
 */
struct IFileSystem {
    static constexpr std::string_view interface_name = "IFileSystem";

    /**
     * @brief Get the mount base path of the SD card file system.
     * @return const char* Base path (e.g. "/sdcard"); may be nullptr if not mounted
     */
    virtual const char *file_system_get_sdcard_path() = 0;
};

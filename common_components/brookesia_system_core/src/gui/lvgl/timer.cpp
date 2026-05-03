/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "brookesia/system_core/macro_configs.h"
#if !ESP_BROOKESIA_LVGL_TIMER_ENABLE_DEBUG_LOG
#   define UTILS_DISABLE_DEBUG_LOG
#endif
#include "private/utils.hpp"
#include "brookesia/system_core/gui/lvgl/helper.hpp"
#include "brookesia/system_core/gui/lvgl/timer.hpp"

namespace esp_brookesia::gui {

LvTimer::LvTimer(TimerCallback callback, uint32_t period, void *user_data):
    _callback(callback),
    _user_data{this, user_data}
{
    ESP_UTILS_LOG_TRACE_ENTER_WITH_THIS();
    ESP_UTILS_LOGD("Param: period(%lu), user_data(0x%p)", period, user_data);

    ESP_UTILS_CHECK_NULL_EXIT(callback, "Invalid callback");
    _native_handle = lv_timer_create([](lv_timer_t *t) {
        ESP_UTILS_LOG_TRACE_ENTER();

        LvTimer *timer = (LvTimer *)t->user_data;
        ESP_UTILS_CHECK_NULL_EXIT(timer, "Invalid timer");

        timer->_callback(timer->_user_data.user_data);

        ESP_UTILS_LOG_TRACE_EXIT();
    }, period, this);

    ESP_UTILS_LOG_TRACE_EXIT_WITH_THIS();
}

LvTimer::~LvTimer()
{
    ESP_UTILS_LOG_TRACE_ENTER_WITH_THIS();

    if (isValid()) {
        lv_timer_delete(_native_handle);
    }

    ESP_UTILS_LOG_TRACE_EXIT_WITH_THIS();
}

bool LvTimer::pause()
{
    ESP_UTILS_LOG_TRACE_ENTER_WITH_THIS();

    ESP_UTILS_CHECK_FALSE_RETURN(isValid(), false, "Invalid timer");

    lv_timer_pause(_native_handle);

    ESP_UTILS_LOG_TRACE_EXIT_WITH_THIS();

    return true;
}

bool LvTimer::resume()
{
    ESP_UTILS_LOG_TRACE_ENTER_WITH_THIS();

    ESP_UTILS_CHECK_FALSE_RETURN(isValid(), false, "Invalid timer");

    lv_timer_resume(_native_handle);

    ESP_UTILS_LOG_TRACE_EXIT_WITH_THIS();

    return true;
}

bool LvTimer::restart()
{
    ESP_UTILS_LOG_TRACE_ENTER_WITH_THIS();

    ESP_UTILS_CHECK_FALSE_RETURN(isValid(), false, "Invalid timer");

    lv_timer_reset(_native_handle);
    lv_timer_resume(_native_handle);

    ESP_UTILS_LOG_TRACE_EXIT_WITH_THIS();

    return true;
}

bool LvTimer::reset()
{
    ESP_UTILS_LOG_TRACE_ENTER_WITH_THIS();

    ESP_UTILS_CHECK_FALSE_RETURN(isValid(), false, "Invalid timer");

    lv_timer_reset(_native_handle);

    ESP_UTILS_LOG_TRACE_EXIT_WITH_THIS();

    return true;
}

bool LvTimer::setInterval(uint32_t interval_ms)
{
    ESP_UTILS_LOG_TRACE_ENTER_WITH_THIS();

    ESP_UTILS_CHECK_FALSE_RETURN(isValid(), false, "Invalid timer");
    ESP_UTILS_LOGD("Param: interval_ms(%lu)", interval_ms);

    lv_timer_set_period(_native_handle, interval_ms);

    ESP_UTILS_LOG_TRACE_EXIT_WITH_THIS();

    return true;
}
} // namespace esp_brookesia::gui

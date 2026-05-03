/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "brookesia/system_core/macro_configs.h"
#if !ESP_BROOKESIA_LVGL_SCREEN_ENABLE_DEBUG_LOG
#   define UTILS_DISABLE_DEBUG_LOG
#endif
#include "private/utils.hpp"
#include "brookesia/system_core/gui/lvgl/screen.hpp"

namespace esp_brookesia::gui {

LvScreen::LvScreen():
    LvObject(lv_obj_create(nullptr))
{
    ESP_UTILS_LOG_TRACE_ENTER_WITH_THIS();

    ESP_UTILS_CHECK_FALSE_EXIT(isValid(), "Create screen failed");

    ESP_UTILS_CHECK_FALSE_EXIT(removeStyle(nullptr), "Remove style failed");
    ESP_UTILS_CHECK_FALSE_EXIT(
        setStyleAttribute(
            StyleFlag::STYLE_FLAG_CLICKABLE | StyleFlag::STYLE_FLAG_SCROLLABLE, false
        ), "Set style attribute failed"
    );

    ESP_UTILS_LOG_TRACE_EXIT_WITH_THIS();
}

bool LvScreen::load()
{
    ESP_UTILS_LOG_TRACE_ENTER_WITH_THIS();

    ESP_UTILS_CHECK_FALSE_RETURN(isValid(), false, "Invalid screen");

    lv_screen_load(getNativeHandle());

    ESP_UTILS_LOG_TRACE_EXIT_WITH_THIS();
    return true;
}

} // namespace esp_brookesia::gui

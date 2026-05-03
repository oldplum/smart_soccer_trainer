/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <memory>
#include <cstdlib>
#include "lvgl.h"
#include "brookesia/system_core/gui/style/properties.hpp"
#include "object.hpp"

namespace esp_brookesia::gui {

class LvScreen: public LvObject {
public:
    using LvObject::LvObject;

    LvScreen();

    bool load();
};

using LvScreenUniquePtr = std::unique_ptr<LvScreen>;

} // namespace esp_brookesia::gui

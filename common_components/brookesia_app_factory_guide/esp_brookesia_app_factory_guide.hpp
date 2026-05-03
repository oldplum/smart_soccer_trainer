/*
 * SPDX-FileCopyrightText: 2023-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include "lvgl.h"
#include "brookesia/system_phone/app.hpp"

namespace esp_brookesia::apps {

class FactoryGuide: public systems::phone::App {
public:
    FactoryGuide();
    ~FactoryGuide();

    bool run() override;
    bool back() override;
    bool close() override;

private:
};

} // namespace esp_brookesia::apps

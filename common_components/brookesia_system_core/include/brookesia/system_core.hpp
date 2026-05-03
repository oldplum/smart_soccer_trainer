/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

/* Assets */
#include "system_core/assets/assets.h"

/* Base */
#include "system_core/base/app.hpp"
#include "system_core/base/context.hpp"
#include "system_core/base/display.hpp"
#include "system_core/base/event.hpp"
#include "system_core/base/manager.hpp"

/* GUI */
// LVGL
#include "system_core/gui/lvgl/lvgl.hpp"
// Squareline
#include "system_core/gui/squareline/ui_helpers.h"
#include "system_core/gui/squareline/ui_comp.h"
// Style
#include "system_core/gui/style/properties.hpp"
#include "system_core/gui/style/stylesheet_manager.hpp"

/* HAL */
#include "system_core/hal/device.hpp"
#include "system_core/hal/interface.hpp"

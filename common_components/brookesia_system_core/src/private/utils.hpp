/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include "brookesia/system_core/macro_configs.h"

#if !ESP_BROOKESIA_BASE_ENABLE_DEBUG_LOG || defined(UTILS_DISABLE_DEBUG_LOG)
#   undef ESP_UTILS_LOGD_IMPL_FUNC
#   define ESP_UTILS_LOGD_IMPL_FUNC(...) {}
#endif
#undef ESP_UTILS_LOG_TAG
#define ESP_UTILS_LOG_TAG BROOKESIA_SYSTEM_CORE_LOG_TAG
#include "esp_lib_utils.h"

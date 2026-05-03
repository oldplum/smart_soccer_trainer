/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#if defined(ESP_PLATFORM)
#   include "sdkconfig.h"
#endif

/* Version */
#if !defined(BROOKESIA_SYSTEM_CORE_VER_MAJOR)
#   define BROOKESIA_SYSTEM_CORE_VER_MAJOR  (0)
#endif
#if !defined(BROOKESIA_SYSTEM_CORE_VER_MINOR)
#   define BROOKESIA_SYSTEM_CORE_VER_MINOR  (7)
#endif
#if !defined(BROOKESIA_SYSTEM_CORE_VER_PATCH)
#   define BROOKESIA_SYSTEM_CORE_VER_PATCH  (0)
#endif

/* Debug log */
#define BROOKESIA_SYSTEM_CORE_LOG_TAG "SysCore"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////// Base //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if !defined(ESP_BROOKESIA_BASE_ENABLE_DEBUG_LOG)
#   if defined(CONFIG_ESP_BROOKESIA_BASE_ENABLE_DEBUG_LOG)
#       define ESP_BROOKESIA_BASE_ENABLE_DEBUG_LOG  CONFIG_ESP_BROOKESIA_BASE_ENABLE_DEBUG_LOG
#   else
#       define ESP_BROOKESIA_BASE_ENABLE_DEBUG_LOG  (0)
#   endif
#endif

#if ESP_BROOKESIA_BASE_ENABLE_DEBUG_LOG
#   if !defined(ESP_BROOKESIA_BASE_APP_ENABLE_DEBUG_LOG)
#       if defined(CONFIG_ESP_BROOKESIA_BASE_APP_ENABLE_DEBUG_LOG)
#           define ESP_BROOKESIA_BASE_APP_ENABLE_DEBUG_LOG  CONFIG_ESP_BROOKESIA_BASE_APP_ENABLE_DEBUG_LOG
#       else
#           define ESP_BROOKESIA_BASE_APP_ENABLE_DEBUG_LOG  (0)
#       endif
#   endif
#   if !defined(ESP_BROOKESIA_BASE_DISPLAY_ENABLE_DEBUG_LOG)
#       if defined(CONFIG_ESP_BROOKESIA_BASE_DISPLAY_ENABLE_DEBUG_LOG)
#           define ESP_BROOKESIA_BASE_DISPLAY_ENABLE_DEBUG_LOG  CONFIG_ESP_BROOKESIA_BASE_DISPLAY_ENABLE_DEBUG_LOG
#       else
#           define ESP_BROOKESIA_BASE_DISPLAY_ENABLE_DEBUG_LOG  (0)
#       endif
#   endif
#   if !defined(ESP_BROOKESIA_BASE_EVENT_ENABLE_DEBUG_LOG)
#       if defined(CONFIG_ESP_BROOKESIA_BASE_EVENT_ENABLE_DEBUG_LOG)
#           define ESP_BROOKESIA_BASE_EVENT_ENABLE_DEBUG_LOG  CONFIG_ESP_BROOKESIA_BASE_EVENT_ENABLE_DEBUG_LOG
#       else
#           define ESP_BROOKESIA_BASE_EVENT_ENABLE_DEBUG_LOG  (0)
#       endif
#   endif
#   if !defined(ESP_BROOKESIA_BASE_MANAGER_ENABLE_DEBUG_LOG)
#       if defined(CONFIG_ESP_BROOKESIA_BASE_MANAGER_ENABLE_DEBUG_LOG)
#           define ESP_BROOKESIA_BASE_MANAGER_ENABLE_DEBUG_LOG  CONFIG_ESP_BROOKESIA_BASE_MANAGER_ENABLE_DEBUG_LOG
#       else
#           define ESP_BROOKESIA_BASE_MANAGER_ENABLE_DEBUG_LOG  (0)
#       endif
#   endif
#   if !defined(ESP_BROOKESIA_BASE_CORE_ENABLE_DEBUG_LOG)
#       if defined(CONFIG_ESP_BROOKESIA_BASE_CORE_ENABLE_DEBUG_LOG)
#           define ESP_BROOKESIA_BASE_CORE_ENABLE_DEBUG_LOG  CONFIG_ESP_BROOKESIA_BASE_CORE_ENABLE_DEBUG_LOG
#       else
#           define ESP_BROOKESIA_BASE_CORE_ENABLE_DEBUG_LOG  (0)
#       endif
#   endif
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////// GUI - LVGL /////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if !defined(ESP_BROOKESIA_LVGL_ENABLE_DEBUG_LOG)
#   if defined(CONFIG_ESP_BROOKESIA_LVGL_ENABLE_DEBUG_LOG)
#       define ESP_BROOKESIA_LVGL_ENABLE_DEBUG_LOG  CONFIG_ESP_BROOKESIA_LVGL_ENABLE_DEBUG_LOG
#   else
#       define ESP_BROOKESIA_LVGL_ENABLE_DEBUG_LOG  (0)
#   endif
#endif

#if ESP_BROOKESIA_LVGL_ENABLE_DEBUG_LOG
#   if !defined(ESP_BROOKESIA_LVGL_ANIMATION_ENABLE_DEBUG_LOG)
#       if defined(CONFIG_ESP_BROOKESIA_LVGL_ANIMATION_ENABLE_DEBUG_LOG)
#           define ESP_BROOKESIA_LVGL_ANIMATION_ENABLE_DEBUG_LOG  CONFIG_ESP_BROOKESIA_LVGL_ANIMATION_ENABLE_DEBUG_LOG
#       else
#           define ESP_BROOKESIA_LVGL_ANIMATION_ENABLE_DEBUG_LOG  (0)
#       endif
#   endif
#   if !defined(ESP_BROOKESIA_LVGL_CANVAS_ENABLE_DEBUG_LOG)
#       if defined(CONFIG_ESP_BROOKESIA_LVGL_CANVAS_ENABLE_DEBUG_LOG)
#           define ESP_BROOKESIA_LVGL_CANVAS_ENABLE_DEBUG_LOG  CONFIG_ESP_BROOKESIA_LVGL_CANVAS_ENABLE_DEBUG_LOG
#       else
#           define ESP_BROOKESIA_LVGL_CANVAS_ENABLE_DEBUG_LOG  (0)
#       endif
#   endif
#   if !defined(ESP_BROOKESIA_LVGL_CONTAINER_ENABLE_DEBUG_LOG)
#       if defined(CONFIG_ESP_BROOKESIA_LVGL_CONTAINER_ENABLE_DEBUG_LOG)
#           define ESP_BROOKESIA_LVGL_CONTAINER_ENABLE_DEBUG_LOG  CONFIG_ESP_BROOKESIA_LVGL_CONTAINER_ENABLE_DEBUG_LOG
#       else
#           define ESP_BROOKESIA_LVGL_CONTAINER_ENABLE_DEBUG_LOG  (0)
#       endif
#   endif
#   if !defined(ESP_BROOKESIA_LVGL_DISPLAY_ENABLE_DEBUG_LOG)
#       if defined(CONFIG_ESP_BROOKESIA_LVGL_DISPLAY_ENABLE_DEBUG_LOG)
#           define ESP_BROOKESIA_LVGL_DISPLAY_ENABLE_DEBUG_LOG  CONFIG_ESP_BROOKESIA_LVGL_DISPLAY_ENABLE_DEBUG_LOG
#       else
#           define ESP_BROOKESIA_LVGL_DISPLAY_ENABLE_DEBUG_LOG  (0)
#       endif
#   endif
#   if !defined(ESP_BROOKESIA_LVGL_HELPER_ENABLE_DEBUG_LOG)
#       if defined(CONFIG_ESP_BROOKESIA_LVGL_HELPER_ENABLE_DEBUG_LOG)
#           define ESP_BROOKESIA_LVGL_HELPER_ENABLE_DEBUG_LOG  CONFIG_ESP_BROOKESIA_LVGL_HELPER_ENABLE_DEBUG_LOG
#       else
#           define ESP_BROOKESIA_LVGL_HELPER_ENABLE_DEBUG_LOG  (0)
#       endif
#   endif
#   if !defined(ESP_BROOKESIA_LVGL_LOCK_ENABLE_DEBUG_LOG)
#       if defined(CONFIG_ESP_BROOKESIA_LVGL_LOCK_ENABLE_DEBUG_LOG)
#           define ESP_BROOKESIA_LVGL_LOCK_ENABLE_DEBUG_LOG  CONFIG_ESP_BROOKESIA_LVGL_LOCK_ENABLE_DEBUG_LOG
#       else
#           define ESP_BROOKESIA_LVGL_LOCK_ENABLE_DEBUG_LOG  (0)
#       endif
#   endif
#   if !defined(ESP_BROOKESIA_LVGL_OBJECT_ENABLE_DEBUG_LOG)
#       if defined(CONFIG_ESP_BROOKESIA_LVGL_OBJECT_ENABLE_DEBUG_LOG)
#           define ESP_BROOKESIA_LVGL_OBJECT_ENABLE_DEBUG_LOG  CONFIG_ESP_BROOKESIA_LVGL_OBJECT_ENABLE_DEBUG_LOG
#       else
#           define ESP_BROOKESIA_LVGL_OBJECT_ENABLE_DEBUG_LOG  (0)
#       endif
#   endif
#   if !defined(ESP_BROOKESIA_LVGL_SCREEN_ENABLE_DEBUG_LOG)
#       if defined(CONFIG_ESP_BROOKESIA_LVGL_SCREEN_ENABLE_DEBUG_LOG)
#           define ESP_BROOKESIA_LVGL_SCREEN_ENABLE_DEBUG_LOG  CONFIG_ESP_BROOKESIA_LVGL_SCREEN_ENABLE_DEBUG_LOG
#       else
#           define ESP_BROOKESIA_LVGL_SCREEN_ENABLE_DEBUG_LOG  (0)
#       endif
#   endif
#   if !defined(ESP_BROOKESIA_LVGL_TIMER_ENABLE_DEBUG_LOG)
#       if defined(CONFIG_ESP_BROOKESIA_LVGL_TIMER_ENABLE_DEBUG_LOG)
#           define ESP_BROOKESIA_LVGL_TIMER_ENABLE_DEBUG_LOG  CONFIG_ESP_BROOKESIA_LVGL_TIMER_ENABLE_DEBUG_LOG
#       else
#           define ESP_BROOKESIA_LVGL_TIMER_ENABLE_DEBUG_LOG  (0)
#       endif
#   endif
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////// GUI - Squareline ////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if !defined(ESP_BROOKESIA_GUI_ENABLE_SQUARELINE)
#   if defined(CONFIG_ESP_BROOKESIA_GUI_ENABLE_SQUARELINE)
#       define ESP_BROOKESIA_GUI_ENABLE_SQUARELINE  CONFIG_ESP_BROOKESIA_GUI_ENABLE_SQUARELINE
#   else
#       define ESP_BROOKESIA_GUI_ENABLE_SQUARELINE  (0)
#   endif
#endif

#if ESP_BROOKESIA_GUI_ENABLE_SQUARELINE
#   if !defined(ESP_BROOKESIA_SQUARELINE_ENABLE_UI_COMP)
#       if defined(CONFIG_ESP_BROOKESIA_SQUARELINE_ENABLE_UI_COMP)
#           define ESP_BROOKESIA_SQUARELINE_ENABLE_UI_COMP  CONFIG_ESP_BROOKESIA_SQUARELINE_ENABLE_UI_COMP
#       else
#           define ESP_BROOKESIA_EXPRESSION_ENABLE_DEBUG_LOG  (0)
#       endif
#   endif

#   if !defined(ESP_BROOKESIA_SQUARELINE_ENABLE_UI_HELPERS)
#       if defined(CONFIG_ESP_BROOKESIA_SQUARELINE_ENABLE_UI_HELPERS)
#           define ESP_BROOKESIA_SQUARELINE_ENABLE_UI_HELPERS  CONFIG_ESP_BROOKESIA_SQUARELINE_ENABLE_UI_HELPERS
#       else
#           define ESP_BROOKESIA_SQUARELINE_ENABLE_UI_HELPERS  (0)
#       endif
#   endif
#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////// GUI - Style ////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if !defined(ESP_BROOKESIA_STYLE_ENABLE_DEBUG_LOG)
#   if defined(CONFIG_ESP_BROOKESIA_STYLE_ENABLE_DEBUG_LOG)
#       define ESP_BROOKESIA_STYLE_ENABLE_DEBUG_LOG  CONFIG_ESP_BROOKESIA_STYLE_ENABLE_DEBUG_LOG
#   else
#       define ESP_BROOKESIA_STYLE_ENABLE_DEBUG_LOG  (0)
#   endif
#endif

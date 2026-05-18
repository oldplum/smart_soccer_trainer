# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(EXTRA_COMPONENT_DIRS "components" "common_components")

set(SDKCONFIG_DEFAULTS
    "${CMAKE_SOURCE_DIR}/sdkconfig.defaults"
    "${CMAKE_SOURCE_DIR}/components/gen_bmgr_codes/board_manager.defaults")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(smart_soccer_trainer)

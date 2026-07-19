# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

idf_build_set_property(COMPILE_DEFINITIONS "CONFIG_COMPILER_CXX_RTTI=1" APPEND)
idf_build_set_property(COMPILE_DEFINITIONS "CONFIG_COMPILER_CXX_EXCEPTIONS=1" APPEND)

project(smart_soccer_trainer)
# Force CMake Reconfigure

if(TARGET __idf_espressif__esp-boost)
    target_compile_definitions(__idf_espressif__esp-boost PUBLIC
        CONFIG_COMPILER_CXX_RTTI=1
        CONFIG_COMPILER_CXX_EXCEPTIONS=1
    )
endif()

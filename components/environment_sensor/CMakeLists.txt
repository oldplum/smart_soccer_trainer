set(BME690_API_DIR ${CMAKE_CURRENT_LIST_DIR}/../../common_components/brookesia_app_temperature/BME690_SensorAPI)
set(BSEC_INC_DIR ${CMAKE_CURRENT_LIST_DIR}/../../common_components/brookesia_app_temperature/bsec_IAQ/inc)
set(BSEC_LIB_DIR ${CMAKE_CURRENT_LIST_DIR}/../../common_components/brookesia_app_temperature/bsec_IAQ/bin/esp/esp32_risc_v)

idf_component_register(
    SRCS
        "environment_sensor.cpp"
        "${BME690_API_DIR}/bme69x.c"
        "${BME690_API_DIR}/examples/common/common.c"
    INCLUDE_DIRS
        "."
        "${CMAKE_CURRENT_LIST_DIR}/../../main"
        "${BME690_API_DIR}"
        "${BME690_API_DIR}/examples/common"
        "${BSEC_INC_DIR}"
    REQUIRES
        driver
        esp_timer
        nvs_flash
    PRIV_REQUIRES
        esp_common)

target_link_libraries(${COMPONENT_LIB} PUBLIC
    "${BSEC_LIB_DIR}/libalgobsec.a")

target_link_options(${COMPONENT_LIB} PUBLIC
    "-Wl,--allow-multiple-definition")
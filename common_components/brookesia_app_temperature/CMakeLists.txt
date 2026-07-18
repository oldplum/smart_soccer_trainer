set(PROJ_SRC ${CMAKE_CURRENT_LIST_DIR})

file(GLOB_RECURSE PROJ_SRCS_C ${PROJ_SRC}/*.c)
file(GLOB_RECURSE PROJ_SRCS_CPP ${PROJ_SRC}/*.cpp)

# Exclude UI source code
list(FILTER PROJ_SRCS_C EXCLUDE REGEX "ui/")
list(FILTER PROJ_SRCS_CPP EXCLUDE REGEX "ui/")

idf_component_register(
    SRCS  ${PROJ_SRCS_C} ${PROJ_SRCS_CPP}
    INCLUDE_DIRS 
        ${PROJ_SRC}
        ${PROJ_SRC}/BME690_SensorAPI
        ${PROJ_SRC}/BME690_SensorAPI/examples/common
        ${PROJ_SRC}/bsec_IAQ/inc
    WHOLE_ARCHIVE
    REQUIRES driver esp_common espressif__esp_board_manager
)

set_source_files_properties(
    ${PROJ_SRCS_CPP}
    PROPERTIES
        COMPILE_FLAGS "-Wno-missing-field-initializers"
)

# Link BSEC library
if(IDF_TARGET MATCHES "esp32c[0-9]" OR IDF_TARGET STREQUAL "esp32h2")
    set(BSEC_TARGET "esp32_risc_v")
else()
    set(BSEC_TARGET "${IDF_TARGET}")
endif()

target_link_libraries(${COMPONENT_LIB} PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/bsec_IAQ/bin/esp/${BSEC_TARGET}/libalgobsec.a"
)

# Add linker flags for the library
target_link_options(${COMPONENT_LIB} PUBLIC
    "-Wl,--allow-multiple-definition"
)
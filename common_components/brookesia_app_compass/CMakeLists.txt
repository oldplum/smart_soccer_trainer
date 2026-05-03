set(PROJ_SRC ${CMAKE_CURRENT_LIST_DIR})

file(GLOB_RECURSE PROJ_SRCS_C ${PROJ_SRC}/*.c)
file(GLOB_RECURSE PROJ_SRCS_CPP ${PROJ_SRC}/*.cpp)

idf_component_register(
    SRCS  ${PROJ_SRCS_C} ${PROJ_SRCS_CPP}
    INCLUDE_DIRS 
        ${PROJ_SRC}
        ${PROJ_SRC}/BMM350_SensorAPI
        ${PROJ_SRC}/BMM350_SensorAPI/examples/common
    WHOLE_ARCHIVE
    REQUIRES espressif__esp-boost
    REQUIRES espressif__i2c_bus
    REQUIRES nvs_flash
    REQUIRES espressif__esp_board_manager
)

set_source_files_properties(
    ${PROJ_SRCS_CPP}
    PROPERTIES
        COMPILE_FLAGS "-Wno-missing-field-initializers"
)

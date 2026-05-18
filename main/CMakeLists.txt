idf_component_register(
    SRC_DIRS "."
    INCLUDE_DIRS "."
    REQUIRES environment_sensor nvs_flash)

target_compile_options(${COMPONENT_LIB} INTERFACE -Wno-missing-field-initializers)


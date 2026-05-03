idf_component_register(
    SRCS "bmm350.c" "bmm350_aux_adapter.c"
    INCLUDE_DIRS "."
    REQUIRES espressif__i2c_bus 
    REQUIRES espressif__bmi270_sensor
)

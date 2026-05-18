idf_component_register(
    SRCS "bmi270_collector.c"
    INCLUDE_DIRS "include" "../../main"
    REQUIRES esp_board_manager espressif__bmi270_sensor i2c_bus esp_driver_gptimer driver esp_timer freertos)

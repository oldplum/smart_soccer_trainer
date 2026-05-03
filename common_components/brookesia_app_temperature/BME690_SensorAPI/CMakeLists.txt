idf_component_register(
    SRCS "bme69x.c" "examples/common/common.c"
    INCLUDE_DIRS "." "examples/common"
    REQUIRES driver esp_timer
    PRIV_REQUIRES esp_common
)

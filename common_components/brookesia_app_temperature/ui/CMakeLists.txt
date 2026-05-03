SET(SOURCES screens/ui_TemperatureScreen.c
    screens/ui_TemperatureTipScreen.c
    ui.c
    components/ui_comp_hook.c
    ui_helpers.c
    images/ui_img_temp_png.c
    images/ui_img_fire_png.c
    images/ui_img_water_png.c
    images/ui_img_pressure_png.c
    images/ui_img_leaf_png.c)

add_library(ui ${SOURCES})

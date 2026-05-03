SET(SOURCES screens/ui_GestureDetectTipScreen.c
    screens/ui_GestureDetectScreen.c
    ui.c
    ui_events.c
    ui_app.c
    images/ui_img_pickup_png.c
    images/ui_img_throw_png.c
    images/ui_img_shake_png.c
    images/ui_img_success_png.c)

add_library(ui ${SOURCES})

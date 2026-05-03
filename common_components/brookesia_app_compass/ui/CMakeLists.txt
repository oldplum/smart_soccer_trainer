SET(SOURCES screens/ui_CompassCorrectScreen.c
    screens/ui_CompassScreen.c
    screens/ui_CompassTipScreen.c
    ui.c
    images/ui_img_correct_screen_png.c
    images/ui_img_compass_base_png.c
    images/ui_img_compass_pointer_png.c)

add_library(ui ${SOURCES})

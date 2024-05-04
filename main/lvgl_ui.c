#include "lvgl.h"

static lv_obj_t *label;

void lvgl_ui(lv_disp_t *lv_disp)
{
    lv_obj_t *lv_screen = lv_disp_get_scr_act(lv_disp);

    label = lv_label_create(lv_screen);
    lv_label_set_text(label, "Hello World");
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
}

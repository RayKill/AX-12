#include "lvgl.h"
#include "lv_port_disp.h"

#define MY_HOR_RES 480
#define MY_VER_RES 272

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf1[MY_HOR_RES * 10];

void lv_port_disp_init(void) {
    lv_disp_draw_buf_init(&draw_buf, buf1, NULL, MY_HOR_RES * 10);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    disp_drv.hor_res = MY_HOR_RES;
    disp_drv.ver_res = MY_VER_RES;

    lv_disp_drv_register(&disp_drv);
}

void my_disp_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p) {
    // à implémenter pour afficher dans la framebuffer réelle
    lv_disp_flush_ready(drv);
}

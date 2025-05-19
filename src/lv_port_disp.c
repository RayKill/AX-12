#include "lvgl.h"
#include "lv_port_disp.h"

#define DISP_HOR_RES 480
#define DISP_VER_RES 272
#define FRAMEBUFFER_ADDR ((lv_color_t *)0xC0000000)  // RAM écran STM32F746G-DISCO

void my_disp_flush(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map)
{
    lv_coord_t x, y;
    lv_color_t * fb = FRAMEBUFFER_ADDR;

    for (y = area->y1; y <= area->y2; y++) {
        for (x = area->x1; x <= area->x2; x++) {
            fb[y * DISP_HOR_RES + x] = ((lv_color_t *)px_map)[(y - area->y1) * (area->x2 - area->x1 + 1) + (x - area->x1)];
        }
    }

    lv_display_flush_ready(disp);  // Signale à LVGL que le rendu est terminé
}

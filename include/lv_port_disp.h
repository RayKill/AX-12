#ifndef LV_PORT_DISP_H
#define LV_PORT_DISP_H

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

void my_disp_flush(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map);

#ifdef __cplusplus
}
#endif

#endif // LV_PORT_DISP_H

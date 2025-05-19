#include "stm32f7xx_hal.h"
#include "lvgl.h"
#include "lv_port_disp.h"  // contient my_disp_flush()
#include "ax12.h"
#include "usart.h"

#define DISP_HOR_RES 480
#define DISP_VER_RES 272

// Framebuffer (RAM LCD STM32F746G-DISCO)
#define FRAMEBUFFER_ADDR ((lv_color_t *)0xC0000000)

void SystemClock_Config(void);
void MX_GPIO_Init(void);

static void event_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_CLICKED) {
        // Test action sur servo
        ax12_ping(1);
        HAL_Delay(50);

        uint16_t pos = ax12_read_position(1);
        HAL_Delay(50);

        if (pos != 0xFFFF) {
            uint16_t newPos = pos + 50;
            if (newPos > 1023) newPos = 1023;
            ax12_move_to_position(1, newPos);
        }
    }
}

void testLvgl()
{
    lv_obj_t * label;

    lv_obj_t * btn1 = lv_button_create(lv_screen_active());
    lv_obj_add_event_cb(btn1, event_handler, LV_EVENT_ALL, NULL);
    lv_obj_align(btn1, LV_ALIGN_CENTER, 0, -40);
    lv_obj_remove_flag(btn1, LV_OBJ_FLAG_PRESS_LOCK);

    label = lv_label_create(btn1);
    lv_label_set_text(label, "Move AX-12");
    lv_obj_center(label);

    lv_obj_t * btn2 = lv_button_create(lv_screen_active());
    lv_obj_add_event_cb(btn2, event_handler, LV_EVENT_ALL, NULL);
    lv_obj_align(btn2, LV_ALIGN_CENTER, 0, 40);
    lv_obj_add_flag(btn2, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_set_height(btn2, LV_SIZE_CONTENT);

    label = lv_label_create(btn2);
    lv_label_set_text(label, "Toggle");
    lv_obj_center(label);
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    UART1_Init();

    lv_init();

    // Initialisation du display (v9)
    static lv_color_t buf1[DISP_HOR_RES * 10]; // buffer = 10 lignes
    static lv_draw_buf_t draw_buf;

    lv_display_t * disp = lv_display_create(DISP_HOR_RES, DISP_VER_RES);

    lv_draw_buf_init(&draw_buf, buf1, NULL, DISP_HOR_RES, 10, LV_COLOR_FORMAT_RGB565);
    lv_display_set_draw_buffers(disp, &draw_buf, NULL);
    lv_display_set_color_format(disp, LV_COLOR_FORMAT_RGB565);
    lv_display_set_flush_cb(disp, my_disp_flush);  // Définie dans lv_port_disp.c
    lv_display_set_default(disp);

    testLvgl();

    while (1) {
        lv_timer_handler();  // v9 = remplace lv_task_handler()
        HAL_Delay(5);
    }
}

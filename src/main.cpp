#include "lvgl.h"
#include "app_hal.h"
#include "ax12.h"
#include "usart.h"

#include <cstdio>

static uint8_t servoID = 1;

static void event_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if(code == LV_EVENT_CLICKED) {
        LV_LOG_USER("Clicked - Servo action");

        ax12_ping(servoID);

        uint16_t pos = ax12_read_position(servoID);
        if(pos != 0xFFFF) {
            LV_LOG_USER("Current position: %d", pos);
            uint16_t newPos = pos + 50;
            if(newPos > 1023) newPos = 1023;
            ax12_move_to_position(servoID, newPos);
        } else {
            LV_LOG_USER("Servo read failed");
        }
    }
    else if(code == LV_EVENT_VALUE_CHANGED) {
        LV_LOG_USER("Toggled");
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
    lv_label_set_text(label, "Move Servo");
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
    printf("LVGL + AX-12 Demo\n");
    fflush(stdout);

    HAL_Init();
    SystemClock_Config();       // Auto-généré par CubeMX
    MX_GPIO_Init();
    MX_USART1_UART_Init();     // UART1 @ 1 Mbps

    lv_init();
    hal_setup();

    testLvgl();

    hal_loop();
    return 0;
}

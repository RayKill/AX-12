#include "stm32f7xx_hal.h"
#include "lvgl.h"
#include "lv_port_disp.h"
#include "ax12.h"
#include "usart.h"

void SystemClock_Config(void);
void MX_GPIO_Init(void);

static void event_handler(lv_event_t * e) {
    lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_CLICKED) {
        // Logique de test AX-12 : ping, read, move
        ax12_ping(1);
        HAL_Delay(100);

        uint16_t pos = ax12_read_position(1);
        HAL_Delay(100);

        if (pos != 0xFFFF) {
            uint16_t newPos = pos + 50;
            if (newPos > 1023) newPos = 1023;
            ax12_move_to_position(1, newPos);
        }
    }
}

void create_ui(void) {
    lv_obj_t * btn = lv_btn_create(lv_scr_act());
    lv_obj_center(btn);
    lv_obj_add_event_cb(btn, event_handler, LV_EVENT_CLICKED, NULL);

    lv_obj_t * label = lv_label_create(btn);
    lv_label_set_text(label, "Move AX-12");
    lv_obj_center(label);
}

int main(void) {
    // Initialisation HAL et matériel
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    UART1_Init();

    // Initialisation de LVGL
    lv_init();
    lv_port_disp_init();     // Portage écran TFT

    create_ui();             // Affiche un bouton sur l’écran

    while (1) {
        lv_task_handler();   // Rafraîchit LVGL
        HAL_Delay(5);        // 5ms = fluide sans surcharger
    }
}

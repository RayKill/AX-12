#include "lvgl.h"
#include "TFT.h"
#include "ax12.h"  // Ton fichier AX-12
#include "usart.h" // UART1_Send

lv_obj_t *btn;
lv_obj_t *label;

void btn_event_cb(lv_event_t *e) {
    // Bouge le servo à une position aléatoire entre 300 et 700
    uint16_t pos = 512;
    ax12_move_to_position(1, pos);  // ID 1 pour le servo
}

extern "C" void app_main() {
    lv_init();
    tft_init(); // initialisation de l’écran
    UART1_Init(); // si nécessaire selon ta lib UART

    btn = lv_btn_create(lv_scr_act());
    lv_obj_center(btn);
    lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_CLICKED, NULL);

    label = lv_label_create(btn);
    lv_label_set_text(label, "Bouger AX-12");
    lv_obj_center(label);
}

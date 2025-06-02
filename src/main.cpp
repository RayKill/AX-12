#include "lvgl.h"
#include "Arduino.h"
#include "lvglDrivers.h"

#define SERVO_ID 1

static void event_handler(lv_event_t *e)
{
  lv_event_code_t code = lv_event_get_code(e);
  if (code == LV_EVENT_CLICKED) {
    LV_LOG_USER("Clicked");
  }
  else if (code == LV_EVENT_VALUE_CHANGED) {
    LV_LOG_USER("Toggled");
  }
}

void testLvgl()
{
  lv_obj_t *label;

  lv_obj_t *btn1 = lv_btn_create(lv_scr_act());
  lv_obj_add_event_cb(btn1, event_handler, LV_EVENT_ALL, NULL);
  lv_obj_align(btn1, LV_ALIGN_CENTER, 0, -40);
  lv_obj_remove_flag(btn1, LV_OBJ_FLAG_PRESS_LOCK);

  label = lv_label_create(btn1);
  lv_label_set_text(label, "Button");
  lv_obj_center(label);

  lv_obj_t *btn2 = lv_btn_create(lv_scr_act());
  lv_obj_add_event_cb(btn2, event_handler, LV_EVENT_ALL, NULL);
  lv_obj_align(btn2, LV_ALIGN_CENTER, 0, 40);
  lv_obj_add_flag(btn2, LV_OBJ_FLAG_CHECKABLE);
  lv_obj_set_height(btn2, LV_SIZE_CONTENT);

  label = lv_label_create(btn2);
  lv_label_set_text(label, "Toggle");
  lv_obj_center(label);
}

void mySetup()
{
  Serial.begin(115200);
  Serial1.begin(1000000); // AX-12A communication at 1 Mbps

  Serial.println("Initialisation terminée");

  testLvgl(); // Charge interface
}

void loop()
{
}

void myTask(void *pvParameters)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1)
  {
    // Crée et envoie un paquet PING vers le servo
    uint8_t packet[6] = {
      0xFF, 0xFF,
      SERVO_ID,
      0x02,     // length
      0x01,     // instruction: PING
      (uint8_t)(~(SERVO_ID + 0x02 + 0x01))  // checksum
    };

    Serial1.write(packet, 6);
    Serial.println("PING envoyé à AX-12A");

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2000)); // toutes les 2 secondes
  }
}

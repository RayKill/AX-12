#include "lvgl.h"
#include "Arduino.h"
#include "lvglDrivers.h"
#include "PeripheralPins.h"

#define SERVO_ID 0x01
HardwareSerial ax12(USART6);  // TX = PG14 (D1)

uint16_t g_vitesse = 512;

void enableTorque() {
  uint8_t packet[8] = {0xFF, 0xFF, SERVO_ID, 0x04, 0x03, 0x18, 0x01, (uint8_t)(~(SERVO_ID + 0x04 + 0x03 + 0x18 + 0x01))};
  ax12.write(packet, 8);
}

void setVitesse(uint16_t vitesse) {
  g_vitesse = vitesse;
  uint8_t velL = vitesse & 0xFF;
  uint8_t velH = (vitesse >> 8) & 0xFF;
  uint8_t packet[9] = {0xFF, 0xFF, SERVO_ID, 0x05, 0x03, 0x20, velL, velH, (uint8_t)(~(SERVO_ID + 0x05 + 0x03 + 0x20 + velL + velH))};
  ax12.write(packet, 9);
}

void moveTo(uint16_t position) {
  setVitesse(g_vitesse);
  uint8_t posL = position & 0xFF;
  uint8_t posH = (position >> 8) & 0xFF;
  uint8_t packet[9] = {0xFF, 0xFF, SERVO_ID, 0x05, 0x03, 0x1E, posL, posH, (uint8_t)(~(SERVO_ID + 0x05 + 0x03 + 0x1E + posL + posH))};
  ax12.write(packet, 9);
}

// --- CALLBACKS SLIDERS ---
static void slider_position_cb(lv_event_t *e)
{
  lv_obj_t *slider = (lv_obj_t *)lv_event_get_target(e);
  int pos = lv_slider_get_value(slider);
  lv_obj_t *label = (lv_obj_t *)lv_event_get_user_data(e);
  static char buf[32];
  snprintf(buf, sizeof(buf), "Position: %d", pos);
  lv_label_set_text(label, buf);
  if (lv_event_get_code(e) == LV_EVENT_VALUE_CHANGED) {
    moveTo(pos);
  }
}

static void slider_vitesse_cb(lv_event_t *e)
{
  lv_obj_t *slider = (lv_obj_t *)lv_event_get_target(e);
  int v = lv_slider_get_value(slider);
  lv_obj_t *label = (lv_obj_t *)lv_event_get_user_data(e);
  static char buf[32];
  snprintf(buf, sizeof(buf), "Vitesse: %d", v);
  lv_label_set_text(label, buf);
  if (lv_event_get_code(e) == LV_EVENT_VALUE_CHANGED) {
    setVitesse(v);
  }
}

void testLvgl()
{
  // Position Slider + Label
  lv_obj_t *slider_pos = lv_slider_create(lv_scr_act());
  lv_obj_set_width(slider_pos, 250);
  lv_obj_align(slider_pos, LV_ALIGN_CENTER, 0, -60);  // Positionne bien en haut
  lv_slider_set_range(slider_pos, 0, 1023);
  lv_slider_set_value(slider_pos, 512, LV_ANIM_OFF);

  lv_obj_t *label_pos = lv_label_create(lv_scr_act());
  lv_obj_align_to(label_pos, slider_pos, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

  lv_obj_add_event_cb(slider_pos, slider_position_cb, LV_EVENT_VALUE_CHANGED, label_pos);

  // Vitesse Slider + Label
  lv_obj_t *slider_vit = lv_slider_create(lv_scr_act());
  lv_obj_set_width(slider_vit, 250);
  lv_obj_align(slider_vit, LV_ALIGN_CENTER, 0, 60);   // Positionne bien en bas
  lv_slider_set_range(slider_vit, 0, 1023);
  lv_slider_set_value(slider_vit, 512, LV_ANIM_OFF);

  lv_obj_t *label_vit = lv_label_create(lv_scr_act());
  lv_obj_align_to(label_vit, slider_vit, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);

  lv_obj_add_event_cb(slider_vit, slider_vitesse_cb, LV_EVENT_VALUE_CHANGED, label_vit);
}

void mySetup()
{
  Serial.begin(115200);
  ax12.begin(1000000);
  pinmap_pinout(PG_14, PinMap_UART_TX);

  enableTorque();
  delay(20);

  setVitesse(g_vitesse);

  testLvgl();

  Serial.println("Interface 2 sliders prête !");
}

void loop() {}

void myTask(void *pvParameters) {
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

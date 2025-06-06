#include "lvgl.h"
#include "Arduino.h"
#include "lvglDrivers.h"
#include "PeripheralPins.h"

#define MAX_AX12_ID 0xFD
#define AX12_TIMEOUT_MS 10

HardwareSerial ax12(USART6); // TX: PG_14 (D1), RX: PC_7 (D0)
uint8_t current_servo_id = 0x01;
uint16_t g_vitesse = 100;
volatile bool runningSequence = false;

enum AppState {
  MENU_SCAN,
  MENU_CONTROL
};
AppState app_state = MENU_SCAN;

lv_obj_t *main_menu = NULL;
lv_obj_t *ctrl_menu = NULL;
lv_obj_t *label_scan = NULL;

static uint8_t detected_ids[50];
static uint8_t detected_count = 0;

// --- AX-12A communication ---
void enableTorque(uint8_t id) {
  uint8_t packet[8] = {0xFF, 0xFF, id, 0x04, 0x03, 0x18, 0x01, (uint8_t)(~(id + 0x04 + 0x03 + 0x18 + 0x01))};
  ax12.write(packet, 8);
}

void setVitesse(uint8_t id, uint16_t vitesse) {
  g_vitesse = vitesse;
  uint8_t velL = vitesse & 0xFF;
  uint8_t velH = (vitesse >> 8) & 0xFF;
  uint8_t packet[9] = {0xFF, 0xFF, id, 0x05, 0x03, 0x20, velL, velH, (uint8_t)(~(id + 0x05 + 0x03 + 0x20 + velL + velH))};
  ax12.write(packet, 9);
}

void moveTo(uint8_t id, uint16_t position) {
  setVitesse(id, g_vitesse);
  uint8_t posL = position & 0xFF;
  uint8_t posH = (position >> 8) & 0xFF;
  uint8_t packet[9] = {0xFF, 0xFF, id, 0x05, 0x03, 0x1E, posL, posH, (uint8_t)(~(id + 0x05 + 0x03 + 0x1E + posL + posH))};
  ax12.write(packet, 9);
}

// --- AX-12A PING avec reception ---
bool ax12_ping(uint8_t id) {
    uint8_t packet[6] = {0xFF, 0xFF, id, 0x02, 0x01, (uint8_t)(~(id + 0x02 + 0x01))};
    while (ax12.available()) ax12.read(); // Vide RX
    ax12.write(packet, 6);
    delayMicroseconds(400);
    while (ax12.available()) ax12.read(); // Vide RX

    unsigned long t0 = millis();
    while (millis() - t0 < AX12_TIMEOUT_MS) {
        if (ax12.available() >= 6) {
            uint8_t resp[6];
            for (uint8_t i=0; i<6; ++i) resp[i]=ax12.read();
            if (resp[0]==0xFF && resp[1]==0xFF && resp[2]==id) {
                Serial.print("Servo detecte ID ");
                Serial.println(id);
                return true;
            }
        }
    }
    Serial.print("Aucun retour ID ");
    Serial.println(id);
    return false;
}


// --- CALLBACKS UI ---
void show_ctrl_menu(uint8_t id);
void show_scan_menu();

static void id_btn_cb(lv_event_t *e) {
  uint8_t id = (uint8_t)(uintptr_t)lv_event_get_user_data(e);
  current_servo_id = id;
  show_ctrl_menu(id);
}

static void retour_cb(lv_event_t *e) {
  runningSequence = false;
  show_scan_menu();
}

// --- MENU SCAN ---
void show_scan_menu() {
  app_state = MENU_SCAN;
  runningSequence = false;
  detected_count = 0;
  if (ctrl_menu) {
    lv_obj_del(ctrl_menu);
    ctrl_menu = NULL;
  }
  if (main_menu) {
    lv_obj_del(main_menu);
    main_menu = NULL;
  }

  main_menu = lv_obj_create(lv_scr_act());
  lv_obj_set_size(main_menu, 480, 272);

  label_scan = lv_label_create(main_menu);
  lv_obj_align(label_scan, LV_ALIGN_TOP_MID, 0, 10);
  lv_label_set_text(label_scan, "Appuie sur Scanner pour detecter les servos AX-12A");

  // Bouton Scan
  lv_obj_t *btn_scan = lv_btn_create(main_menu);
  lv_obj_set_size(btn_scan, 120, 45);
  lv_obj_align(btn_scan, LV_ALIGN_TOP_MID, 0, 60);
  lv_obj_add_event_cb(btn_scan, [](lv_event_t *e){
      if (label_scan) lv_label_set_text(label_scan, "Scan en cours...");
      detected_count = 0;
      // On ping seulement 0 à 5 pour debug, mets 0xFD si tu veux tout
      for (uint8_t id = 0; id <= 254; ++id) {
          char info[40];
          snprintf(info, sizeof(info), "Scan ID %d...", id);
          if (label_scan) lv_label_set_text(label_scan, info);

          if (ax12_ping(id)) {
              detected_ids[detected_count++] = id;
              lv_obj_t *btn_id = lv_btn_create(main_menu);
              lv_obj_set_size(btn_id, 80, 40);
              lv_obj_align(btn_id, LV_ALIGN_TOP_LEFT, 10 + ((detected_count-1)%5)*90, 130 + ((detected_count-1)/5)*50);
              lv_obj_add_event_cb(btn_id, id_btn_cb, LV_EVENT_CLICKED, (void *)(uintptr_t)id);
              lv_obj_t *lbl = lv_label_create(btn_id);
              static char id_buf[8];
              snprintf(id_buf, sizeof(id_buf), "ID %d", id);
              lv_label_set_text(lbl, id_buf);
              lv_obj_center(lbl);
          }
      }
      if (detected_count == 0) {
          if (label_scan) lv_label_set_text(label_scan, "Aucun servo detecte !");
      } else {
          if (label_scan) lv_label_set_text(label_scan, "Selectionne un servo detecte :");
      }
  }, LV_EVENT_CLICKED, NULL);
  lv_obj_t *lbl = lv_label_create(btn_scan);
  lv_label_set_text(lbl, "Scanner");
  lv_obj_center(lbl);
}


// --- MENU CONTROLE ---
void show_ctrl_menu(uint8_t id) {
  app_state = MENU_CONTROL;
  if (main_menu) {
    lv_obj_del(main_menu);
    main_menu = NULL;
  }
  if (ctrl_menu) {
    lv_obj_del(ctrl_menu);
    ctrl_menu = NULL;
  }

  ctrl_menu = lv_obj_create(lv_scr_act());
  lv_obj_set_size(ctrl_menu, 480, 272);

  char titre[32];
  snprintf(titre, sizeof(titre), "AX-12A ID %d", id);
  lv_obj_t *label = lv_label_create(ctrl_menu);
  lv_label_set_text(label, titre);
  lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 8);

  // Slider position
  lv_obj_t *slider_pos = lv_slider_create(ctrl_menu);
  lv_obj_set_width(slider_pos, 250);
  lv_obj_align(slider_pos, LV_ALIGN_TOP_MID, 0, 40);
  lv_slider_set_range(slider_pos, 0, 1023);
  lv_slider_set_value(slider_pos, 512, LV_ANIM_OFF);
  lv_obj_t *label_pos = lv_label_create(ctrl_menu);
  lv_obj_align_to(label_pos, slider_pos, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);
  lv_obj_add_event_cb(slider_pos, [](lv_event_t *e){
    lv_obj_t *slider = (lv_obj_t *)lv_event_get_target(e);
    int pos = lv_slider_get_value(slider);
    lv_obj_t *label = (lv_obj_t *)lv_event_get_user_data(e);
    static char buf[32];
    snprintf(buf, sizeof(buf), "Position: %d", pos);
    lv_label_set_text(label, buf);
    if (lv_event_get_code(e) == LV_EVENT_VALUE_CHANGED && !runningSequence) {
      moveTo(current_servo_id, pos);
    }
  }, LV_EVENT_VALUE_CHANGED, label_pos);

  // Slider vitesse
  lv_obj_t *slider_vit = lv_slider_create(ctrl_menu);
  lv_obj_set_width(slider_vit, 250);
  lv_obj_align(slider_vit, LV_ALIGN_TOP_MID, 0, 110);
  lv_slider_set_range(slider_vit, 4, 330);
  lv_slider_set_value(slider_vit, 100, LV_ANIM_OFF);
  lv_obj_t *label_vit = lv_label_create(ctrl_menu);
  lv_obj_align_to(label_vit, slider_vit, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);
  lv_obj_add_event_cb(slider_vit, [](lv_event_t *e){
    lv_obj_t *slider = (lv_obj_t *)lv_event_get_target(e);
    int v = lv_slider_get_value(slider);
    lv_obj_t *label = (lv_obj_t *)lv_event_get_user_data(e);
    static char buf[32];
    snprintf(buf, sizeof(buf), "Vitesse: %d", v);
    lv_label_set_text(label, buf);
    if (lv_event_get_code(e) == LV_EVENT_VALUE_CHANGED) {
      setVitesse(current_servo_id, v);
    }
  }, LV_EVENT_VALUE_CHANGED, label_vit);

  // Toggle bouton (sequence random ON/OFF)
  lv_obj_t *toggle = lv_btn_create(ctrl_menu);
  lv_obj_set_size(toggle, 120, 45);
  lv_obj_align(toggle, LV_ALIGN_BOTTOM_MID, 0, -10);
  lv_obj_add_flag(toggle, LV_OBJ_FLAG_CHECKABLE);
  lv_obj_t *toggle_label = lv_label_create(toggle);
  lv_label_set_text(toggle_label, "Sequence");
  lv_obj_center(toggle_label);
  lv_obj_add_event_cb(toggle, [](lv_event_t *e){
    lv_obj_t *btn = (lv_obj_t *)lv_event_get_target(e);
    if (lv_obj_has_state(btn, LV_STATE_CHECKED)) {
      runningSequence = true;
    } else {
      runningSequence = false;
    }
  }, LV_EVENT_VALUE_CHANGED, NULL);

  // Bouton retour
  lv_obj_t *btn_retour = lv_btn_create(ctrl_menu);
  lv_obj_set_size(btn_retour, 70, 35);
  lv_obj_align(btn_retour, LV_ALIGN_TOP_LEFT, 10, 10);
  lv_obj_add_event_cb(btn_retour, retour_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *lbl_retour = lv_label_create(btn_retour);
  lv_label_set_text(lbl_retour, "Retour");
  lv_obj_center(lbl_retour);

  enableTorque(id);
  setVitesse(id, g_vitesse);
}

// --- LVGL + Setup ---
void testLvgl() { show_scan_menu(); }

void mySetup() {
  Serial.begin(115200);
  ax12.begin(1000000);
  pinmap_pinout(PG_14, PinMap_UART_TX); // TX
  pinmap_pinout(PC_7, PinMap_UART_RX);  // RX
  delay(20);

  Serial.println("Debut scan test rapide...");
  for (uint8_t id = 0; id <= 3; ++id) {
      if (ax12_ping(id)) {
          Serial.print("Servo trouve ID ");
          Serial.println(id);
      }
  }
  Serial.println("Fin scan test.");

  testLvgl();
  Serial.println("UI multi-servo auto prete !");
}

void loop() {}

void myTask(void *pvParameters) {
  while (1) {
    if (app_state == MENU_CONTROL && runningSequence) {
      uint16_t pos = random(50, 980);
      moveTo(current_servo_id, pos);
      vTaskDelay(pdMS_TO_TICKS(700 + abs((int)pos - 512) * 2));
    } else {
      vTaskDelay(pdMS_TO_TICKS(50));
    }
  }
}

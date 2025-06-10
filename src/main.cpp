#include "lvgl.h"
#include "Arduino.h"
#include "lvglDrivers.h"
#include "PeripheralPins.h"

#define MAX_AX12_ID 0xFD
#define AX12_TIMEOUT_MS 10
#define GRIPPER_LEFT_ID 1
#define GRIPPER_RIGHT_ID 23
#define GRIPPER_LEFT_CLOSED 950    // Beaucoup plus fermé
#define GRIPPER_RIGHT_CLOSED 380   // Beaucoup plus fermé
#define GRIPPER_LEFT_OPEN 724      // Position ouverte symétrique
#define GRIPPER_RIGHT_OPEN 620     // Position ouverte symétrique
#define GRIPPER_LEFT_INIT 830      // Position initiale gauche
#define GRIPPER_RIGHT_INIT 520     // Position initiale droite
#define GRIPPER_FORCE_THRESHOLD 20 // Seuil encore plus bas

HardwareSerial ax12(USART6); // TX: PG_14 (D1), RX: PC_7 (D0)
uint8_t current_servo_id = 0x01;
uint16_t g_vitesse = 100;
volatile bool runningSequence = false;
volatile bool gripperClosed = false;

enum AppState {
  MENU_MAIN,
  MENU_CONTROL
};
AppState app_state = MENU_MAIN;

lv_obj_t *main_menu = NULL;
lv_obj_t *ctrl_menu = NULL;
lv_obj_t *label_status = NULL;
lv_obj_t *servo_dropdown = NULL;

static uint8_t detected_ids[50];
static uint8_t detected_count = 0;

// --- AX-12A communication ---
void enableTorque(uint8_t id) {
  uint8_t packet[8] = {0xFF, 0xFF, id, 0x04, 0x03, 0x18, 0x01, (uint8_t)(~(id + 0x04 + 0x03 + 0x18 + 0x01))};
  ax12.write(packet, 8);
}

void disableTorque(uint8_t id) {
  uint8_t packet[8] = {0xFF, 0xFF, id, 0x04, 0x03, 0x18, 0x00, (uint8_t)(~(id + 0x04 + 0x03 + 0x18 + 0x00))};
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

// Lecture de la charge actuelle (pour détecter les contraintes)
uint16_t readLoad(uint8_t id) {
  uint8_t packet[8] = {0xFF, 0xFF, id, 0x04, 0x02, 0x28, 0x02, (uint8_t)(~(id + 0x04 + 0x02 + 0x28 + 0x02))};
  while (ax12.available()) ax12.read();
  ax12.write(packet, 8);
  
  unsigned long t0 = millis();
  while (millis() - t0 < AX12_TIMEOUT_MS) {
    if (ax12.available() >= 8) {
      uint8_t resp[8];
      for (uint8_t i = 0; i < 8; i++) resp[i] = ax12.read();
      if (resp[0] == 0xFF && resp[1] == 0xFF && resp[2] == id) {
        return (resp[6] << 8) | resp[5];
      }
    }
  }
  return 0;
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
    return false;
}

// --- Contrôle de la pince ---
void initGripper() {
  Serial.println("Initialisation de la pince...");
  if (label_status) lv_label_set_text(label_status, "Initialisation pince...");
  
  // Activer le couple et vitesse modérée
  enableTorque(GRIPPER_LEFT_ID);
  enableTorque(GRIPPER_RIGHT_ID);
  setVitesse(GRIPPER_LEFT_ID, 80);
  setVitesse(GRIPPER_RIGHT_ID, 80);
  
  // Position initiale avec les vraies valeurs
  moveTo(GRIPPER_LEFT_ID, GRIPPER_LEFT_INIT);   // Position initiale gauche
  moveTo(GRIPPER_RIGHT_ID, GRIPPER_RIGHT_INIT); // Position initiale droite
  
  delay(1000); // Attendre que les servos atteignent la position
  gripperClosed = false;
  
  if (label_status) lv_label_set_text(label_status, "Pince initialisee - Position neutre");
  Serial.println("Pince initialisee");
}

void controlGripper(bool close) {
  if (close && !gripperClosed) {
    // Fermeture intelligente avec adaptation à l'objet
    Serial.println("Fermeture intelligente de la pince...");
    if (label_status) lv_label_set_text(label_status, "Fermeture intelligente...");
    
    setVitesse(GRIPPER_LEFT_ID, 50);   // Vitesse modérée pour approche
    setVitesse(GRIPPER_RIGHT_ID, 50);
    
    // Étape 1: Approche lente vers la position de contact
    moveTo(GRIPPER_LEFT_ID, 888);
    moveTo(GRIPPER_RIGHT_ID, 440);
    delay(400);
    
    // Étape 2: Fermeture progressive avec détection d'objet
    Serial.println("Detection d'objet en cours...");
    setVitesse(GRIPPER_LEFT_ID, 15);   // Vitesse très lente pour détecter
    setVitesse(GRIPPER_RIGHT_ID, 15);
    
    uint16_t lastLoadLeft = 0, lastLoadRight = 0;
    bool objectDetected = false;
    
    // Fermeture progressive par petits pas
    for (int step = 0; step < 20 && !objectDetected; step++) {  // Plus d'étapes
      int leftPos = 888 + step;
      int rightPos = 440 - step;
      
      moveTo(GRIPPER_LEFT_ID, leftPos);
      moveTo(GRIPPER_RIGHT_ID, rightPos);
      delay(100); // Délai réduit pour plus de fluidité
      
      // Lire la charge actuelle
      uint16_t loadLeft = readLoad(GRIPPER_LEFT_ID);
      uint16_t loadRight = readLoad(GRIPPER_RIGHT_ID);
      
      // Détecter une augmentation significative de charge (objet touché)
      if ((loadLeft > lastLoadLeft + 15) || (loadRight > lastLoadRight + 15) ||
          (loadLeft > GRIPPER_FORCE_THRESHOLD) || (loadRight > GRIPPER_FORCE_THRESHOLD)) {
        
        Serial.print("Objet detecte ! Charge G:");
        Serial.print(loadLeft);
        Serial.print(" D:");
        Serial.println(loadRight);
        
        // Appliquer une pression importante pour sécuriser la prise
        moveTo(GRIPPER_LEFT_ID, leftPos + 8);  // +8 pour beaucoup plus de pression
        moveTo(GRIPPER_RIGHT_ID, rightPos - 8);
        delay(400);
        
        objectDetected = true;
        if (label_status) lv_label_set_text(label_status, "Objet saisi - Prise adaptee");
        break;
      }
      
      lastLoadLeft = loadLeft;
      lastLoadRight = loadRight;
    }
    
    if (!objectDetected) {
      // Aucun objet détecté, fermeture complète mais douce
      moveTo(GRIPPER_LEFT_ID, GRIPPER_LEFT_CLOSED);
      moveTo(GRIPPER_RIGHT_ID, GRIPPER_RIGHT_CLOSED);
      delay(300);
      if (label_status) lv_label_set_text(label_status, "Pince fermee - Aucun objet");
    }
    
    gripperClosed = true;
    
  } else if (!close && gripperClosed) {
    // Ouverture progressive pour éviter de faire tomber l'objet
    Serial.println("Ouverture progressive...");
    if (label_status) lv_label_set_text(label_status, "Liberation de l'objet...");
    
    setVitesse(GRIPPER_LEFT_ID, 60);   // Vitesse modérée
    setVitesse(GRIPPER_RIGHT_ID, 60);
    
    // Mouvement symétrique vers les positions ouvertes
    moveTo(GRIPPER_LEFT_ID, GRIPPER_LEFT_OPEN);
    moveTo(GRIPPER_RIGHT_ID, GRIPPER_RIGHT_OPEN);
    
    delay(600);
    gripperClosed = false;
    if (label_status) lv_label_set_text(label_status, "Pince ouverte - Objet libere");
  }
}

// --- CALLBACKS UI ---
void show_ctrl_menu(uint8_t id);
void show_main_menu();
void update_servo_dropdown();

static void dropdown_servo_cb(lv_event_t *e) {
  lv_obj_t *dropdown = (lv_obj_t *)lv_event_get_target(e);
  uint16_t selected = lv_dropdown_get_selected(dropdown);
  if (selected < detected_count) {
    current_servo_id = detected_ids[selected];
    show_ctrl_menu(current_servo_id);
  }
}

static void retour_cb(lv_event_t *e) {
  runningSequence = false;
  show_main_menu();
}

static void gripper_toggle_cb(lv_event_t *e) {
  lv_obj_t *btn = (lv_obj_t *)lv_event_get_target(e);
  bool close = lv_obj_has_state(btn, LV_STATE_CHECKED);
  controlGripper(close);
}

// --- MENU PRINCIPAL ---
void show_main_menu() {
  app_state = MENU_MAIN;
  runningSequence = false;
  
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
  lv_obj_set_style_bg_color(main_menu, lv_color_hex(0x1E1E2E), 0);

  // Titre principal
  lv_obj_t *title = lv_label_create(main_menu);
  lv_label_set_text(title, "CONTROLE AX-12A & PINCE ROBOTIQUE");
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);
  lv_obj_set_style_text_color(title, lv_color_hex(0x89B4FA), 0);

  // Status label
  label_status = lv_label_create(main_menu);
  lv_obj_align(label_status, LV_ALIGN_TOP_MID, 0, 35);
  lv_label_set_text(label_status, "Appuie sur Scanner pour detecter les servos");
  lv_obj_set_style_text_color(label_status, lv_color_hex(0xF9E2AF), 0);

  // Section SERVOMOTEURS
  lv_obj_t *servo_title = lv_label_create(main_menu);
  lv_label_set_text(servo_title, "CONTROLE SERVOMOTEURS");
  lv_obj_align(servo_title, LV_ALIGN_TOP_LEFT, 20, 70);
  lv_obj_set_style_text_color(servo_title, lv_color_hex(0x94E2D5), 0);

  // Bouton Scanner
  lv_obj_t *btn_scan = lv_btn_create(main_menu);
  lv_obj_set_size(btn_scan, 100, 35);
  lv_obj_align(btn_scan, LV_ALIGN_TOP_LEFT, 20, 95);
  lv_obj_set_style_bg_color(btn_scan, lv_color_hex(0x89B4FA), 0);
  lv_obj_add_event_cb(btn_scan, [](lv_event_t *e){
      if (label_status) lv_label_set_text(label_status, "Scan en cours...");
      detected_count = 0;
      
      for (uint8_t id = 0; id <= 254; ++id) {
          char info[40];
          snprintf(info, sizeof(info), "Scan ID %d...", id);
          if (label_status) lv_label_set_text(label_status, info);

          if (ax12_ping(id)) {
              detected_ids[detected_count++] = id;
          }
      }
      
      if (detected_count == 0) {
          if (label_status) lv_label_set_text(label_status, "Aucun servo detecte !");
      } else {
          char result[60];
          snprintf(result, sizeof(result), "%d servo(s) detecte(s). Selectionne dans la liste.", detected_count);
          if (label_status) lv_label_set_text(label_status, result);
          update_servo_dropdown();
      }
  }, LV_EVENT_CLICKED, NULL);
  
  lv_obj_t *lbl_scan = lv_label_create(btn_scan);
  lv_label_set_text(lbl_scan, "Scanner");
  lv_obj_center(lbl_scan);

  // Dropdown servos
  servo_dropdown = lv_dropdown_create(main_menu);
  lv_obj_set_width(servo_dropdown, 200);
  lv_obj_align(servo_dropdown, LV_ALIGN_TOP_LEFT, 140, 95);
  lv_dropdown_set_text(servo_dropdown, "Aucun servo detecte");
  lv_obj_add_flag(servo_dropdown, LV_OBJ_FLAG_HIDDEN);
  lv_obj_set_style_bg_color(servo_dropdown, lv_color_hex(0x45475A), 0);
  lv_obj_set_style_text_color(servo_dropdown, lv_color_hex(0xCDD6F4), 0);
  lv_obj_add_event_cb(servo_dropdown, dropdown_servo_cb, LV_EVENT_VALUE_CHANGED, NULL);

  // Section PINCE
  lv_obj_t *gripper_title = lv_label_create(main_menu);
  lv_label_set_text(gripper_title, "PINCE ROBOTIQUE INTELLIGENTE");
  lv_obj_align(gripper_title, LV_ALIGN_TOP_LEFT, 20, 150);
  lv_obj_set_style_text_color(gripper_title, lv_color_hex(0xF38BA8), 0);

  // Boutons pince en ligne
  lv_obj_t *btn_init = lv_btn_create(main_menu);
  lv_obj_set_size(btn_init, 90, 35);
  lv_obj_align(btn_init, LV_ALIGN_TOP_LEFT, 20, 175);
  lv_obj_set_style_bg_color(btn_init, lv_color_hex(0xA6E3A1), 0);
  lv_obj_add_event_cb(btn_init, [](lv_event_t *e){ initGripper(); }, LV_EVENT_CLICKED, NULL);
  lv_obj_t *lbl_init = lv_label_create(btn_init);
  lv_label_set_text(lbl_init, "Initialiser");
  lv_obj_center(lbl_init);

  lv_obj_t *btn_toggle = lv_btn_create(main_menu);
  lv_obj_set_size(btn_toggle, 120, 35);
  lv_obj_align(btn_toggle, LV_ALIGN_TOP_LEFT, 125, 175);
  lv_obj_add_flag(btn_toggle, LV_OBJ_FLAG_CHECKABLE);
  lv_obj_set_style_bg_color(btn_toggle, lv_color_hex(0xF38BA8), 0);
  lv_obj_set_style_bg_color(btn_toggle, lv_color_hex(0xF9E2AF), LV_STATE_CHECKED);
  lv_obj_add_event_cb(btn_toggle, gripper_toggle_cb, LV_EVENT_VALUE_CHANGED, NULL);
  lv_obj_t *lbl_toggle = lv_label_create(btn_toggle);
  lv_label_set_text(lbl_toggle, "Ouvrir/Fermer");
  lv_obj_center(lbl_toggle);

  lv_obj_t *btn_test = lv_btn_create(main_menu);
  lv_obj_set_size(btn_test, 80, 35);
  lv_obj_align(btn_test, LV_ALIGN_TOP_LEFT, 260, 175);
  lv_obj_set_style_bg_color(btn_test, lv_color_hex(0xCBA6F7), 0);
  lv_obj_add_event_cb(btn_test, [](lv_event_t *e){
      bool leftOk = ax12_ping(GRIPPER_LEFT_ID);
      bool rightOk = ax12_ping(GRIPPER_RIGHT_ID);
      
      char status[80];
      if (leftOk && rightOk) {
          snprintf(status, sizeof(status), "Pince OK - Gauche: ID%d, Droite: ID%d", GRIPPER_LEFT_ID, GRIPPER_RIGHT_ID);
          enableTorque(GRIPPER_LEFT_ID);
          enableTorque(GRIPPER_RIGHT_ID);
      } else {
          snprintf(status, sizeof(status), "Erreur pince - G:%s D:%s", leftOk ? "OK" : "KO", rightOk ? "OK" : "KO");
      }
      
      if (label_status) lv_label_set_text(label_status, status);
  }, LV_EVENT_CLICKED, NULL);
  lv_obj_t *lbl_test = lv_label_create(btn_test);
  lv_label_set_text(lbl_test, "Test");
  lv_obj_center(lbl_test);
}

void update_servo_dropdown() {
  if (!servo_dropdown || detected_count == 0) return;
  
  // Construire la liste des options
  static char options[500] = "";
  options[0] = '\0';
  
  for (uint8_t i = 0; i < detected_count; i++) {
    char item[20];
    snprintf(item, sizeof(item), "Servo ID %d", detected_ids[i]);
    if (i > 0) strcat(options, "\n");
    strcat(options, item);
  }
  
  lv_dropdown_set_options(servo_dropdown, options);
  lv_obj_clear_flag(servo_dropdown, LV_OBJ_FLAG_HIDDEN);
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
  snprintf(titre, sizeof(titre), "Controle AX-12A ID %d", id);
  lv_obj_t *label = lv_label_create(ctrl_menu);
  lv_label_set_text(label, titre);
  lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 8);

  // Dropdown pour changer de servo sans retourner au menu
  lv_obj_t *servo_select = lv_dropdown_create(ctrl_menu);
  lv_obj_set_width(servo_select, 150);
  lv_obj_align(servo_select, LV_ALIGN_TOP_RIGHT, -10, 35);
  
  // Remplir avec les servos détectés
  static char options[500] = "";
  options[0] = '\0';
  uint16_t current_index = 0;
  
  for (uint8_t i = 0; i < detected_count; i++) {
    char item[20];
    snprintf(item, sizeof(item), "ID %d", detected_ids[i]);
    if (i > 0) strcat(options, "\n");
    strcat(options, item);
    if (detected_ids[i] == id) current_index = i;
  }
  
  lv_dropdown_set_options(servo_select, options);
  lv_dropdown_set_selected(servo_select, current_index);
  lv_obj_add_event_cb(servo_select, [](lv_event_t *e){
    lv_obj_t *dropdown = (lv_obj_t *)lv_event_get_target(e);
    uint16_t selected = lv_dropdown_get_selected(dropdown);
    if (selected < detected_count) {
      current_servo_id = detected_ids[selected];
      show_ctrl_menu(current_servo_id);
    }
  }, LV_EVENT_VALUE_CHANGED, NULL);

  // Slider position
  lv_obj_t *slider_pos = lv_slider_create(ctrl_menu);
  lv_obj_set_width(slider_pos, 250);
  lv_obj_align(slider_pos, LV_ALIGN_TOP_MID, 0, 80);
  lv_slider_set_range(slider_pos, 0, 1023);
  lv_slider_set_value(slider_pos, 512, LV_ANIM_OFF);
  lv_obj_t *label_pos = lv_label_create(ctrl_menu);
  lv_label_set_text(label_pos, "Position: 512");
  lv_obj_align_to(label_pos, slider_pos, LV_ALIGN_OUT_BOTTOM_MID, 0, 5);
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
  lv_obj_align(slider_vit, LV_ALIGN_TOP_MID, 0, 140);
  lv_slider_set_range(slider_vit, 4, 330);
  lv_slider_set_value(slider_vit, 100, LV_ANIM_OFF);
  lv_obj_t *label_vit = lv_label_create(ctrl_menu);
  lv_label_set_text(label_vit, "Vitesse: 100");
  lv_obj_align_to(label_vit, slider_vit, LV_ALIGN_OUT_BOTTOM_MID, 0, 5);
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
  lv_obj_align(toggle, LV_ALIGN_BOTTOM_MID, 60, -10);
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
  lv_obj_set_size(btn_retour, 80, 35);
  lv_obj_align(btn_retour, LV_ALIGN_BOTTOM_LEFT, 10, -10);
  lv_obj_add_event_cb(btn_retour, retour_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *lbl_retour = lv_label_create(btn_retour);
  lv_label_set_text(lbl_retour, "Menu");
  lv_obj_center(lbl_retour);

  enableTorque(id);
  setVitesse(id, g_vitesse);
}

// --- LVGL + Setup ---
void testLvgl() { show_main_menu(); }

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
  Serial.println("Interface AX-12A et pince robotique prete !");
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
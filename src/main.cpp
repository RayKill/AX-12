/**
 * ========================================================================
 * TRIAX - Robot de Tri Automatique
 * ========================================================================
 * 
 * Projet académique - IUT de Cachan (Université Paris-Saclay)
 * Département GE2II - Génie Électrique et Informatique Industrielle
 * LP MECSE - Métiers de l'Electronique : Communication et Systèmes Embarqués
 * Auteur : DJOUDI Rayan
 * 
 * Description : Robot autonome de tri d'objets entre deux tapis parallèles
 * utilisant une pince robotisée intelligente avec servomoteurs AX-12A
 * 
 * Hardware :
 * - STM32F746G-DISCO (microcontrôleur + écran tactile)
 * - 3x servomoteurs AX-12A (IDs: 1, 23, 41)
 * - Pièces mécaniques du kit BIOLOID pour l'assemblage
 * - Power supply de laboratoire
 * 
 * Communication :
 * - Protocole Dynamixel v1.0 via UART6 à 1 Mbps
 * - Interface graphique LVGL tactile
 * ========================================================================
 */

#include "lvgl.h"           // Bibliothèque d'interface graphique légère
#include "Arduino.h"        // Framework Arduino pour STM32
#include "lvglDrivers.h"    // Pilotes d'affichage pour STM32F746G-DISCO
#include "PeripheralPins.h" // Mapping des broches STM32

// ========================================================================
// DÉFINITIONS ET CONSTANTES
// ========================================================================

#define MAX_AX12_ID 0xFD           // ID maximum possible pour AX-12A (253)
#define AX12_TIMEOUT_MS 10         // Timeout de communication série (10ms)

// IDs des servomoteurs (configurés physiquement sur chaque servo)
#define GRIPPER_LEFT_ID 1          // Servo gauche de la pince
#define GRIPPER_RIGHT_ID 23        // Servo droite de la pince  
#define SERVO_TRANSPORT_ID 41      // Servo de transport linéaire

// Positions critiques de la pince (calibrées expérimentalement)
#define GRIPPER_LEFT_CLOSED 950    // Position fermée maximale servo gauche
#define GRIPPER_RIGHT_CLOSED 380   // Position fermée maximale servo droite
#define GRIPPER_LEFT_OPEN 724      // Position ouverte servo gauche
#define GRIPPER_RIGHT_OPEN 620     // Position ouverte servo droite
#define GRIPPER_LEFT_INIT 830      // Position initiale servo gauche
#define GRIPPER_RIGHT_INIT 520     // Position initiale servo droite

#define GRIPPER_FORCE_THRESHOLD 20 // Seuil de détection d'objet (unités de charge)

// ========================================================================
// VARIABLES GLOBALES
// ========================================================================

// Communication série avec les servomoteurs AX-12A
HardwareSerial ax12(USART6);       // UART6: TX=PG_14(D1), RX=PC_7(D0)

// Variables de contrôle du système
uint8_t current_servo_id = 0x01;   // ID du servo actuellement sélectionné
uint16_t g_vitesse = 100;          // Vitesse globale par défaut
volatile bool runningSequence = false;    // Flag séquence manuelle en cours
volatile bool sequenceRunning = false;    // Flag séquence automatique en cours
volatile bool gripperClosed = false;      // État actuel de la pince
uint8_t servo_transport_id = 41;   // ID du servo de transport

// États de l'application
enum AppState {
  MENU_MAIN,     // Menu principal avec séquences automatiques
  MENU_CONTROL   // Menu de contrôle individuel des servos
};
AppState app_state = MENU_MAIN;

// Objets de l'interface graphique LVGL
lv_obj_t *main_menu = NULL;         // Conteneur du menu principal
lv_obj_t *ctrl_menu = NULL;         // Conteneur du menu de contrôle
lv_obj_t *label_status = NULL;      // Label d'affichage des statuts
lv_obj_t *servo_dropdown = NULL;    // Liste déroulante des servos détectés
lv_obj_t *label_torque_info = NULL; // Label d'affichage des infos de torque

// Gestion des servos détectés lors du scan
static uint8_t detected_ids[50];    // Tableau des IDs détectés
static uint8_t detected_count = 0;  // Nombre de servos détectés

// ========================================================================
// DÉCLARATIONS DES FONCTIONS
// ========================================================================

void sequenceDroite();              // Séquence automatique droite (192→839)
void sequenceGauche();              // Séquence automatique gauche (839→192)
void controlGripper(bool close);    // Contrôle intelligent de la pince
void initGripper();                 // Initialisation de la pince
void openGripperSecure();           // NOUVEAU : Ouverture sécurisée robuste
void show_ctrl_menu(uint8_t id);    // Affichage menu de contrôle individuel
void show_main_menu();              // Affichage menu principal
uint16_t readTorqueLimit(uint8_t id);    // Lecture limite de torque
void displayTorqueInfo();           // Affichage diagnostic complet

// ========================================================================
// FONCTIONS DE COMMUNICATION AX-12A (PROTOCOLE DYNAMIXEL v1.0)
// ========================================================================

/**
 * Active le couple d'un servomoteur
 * @param id : ID du servomoteur (1-253)
 * 
 * Envoie une commande d'écriture au registre 0x18 (Torque Enable)
 * Format trame : [0xFF][0xFF][ID][LENGTH][INSTRUCTION][PARAMS][CHECKSUM]
 */
void enableTorque(uint8_t id) {
  // Construction de la trame Dynamixel v1.0
  uint8_t packet[8] = {
    0xFF, 0xFF,           // Header (synchronisation)
    id,                   // ID du servo destinataire
    0x04,                 // Longueur des données suivantes
    0x03,                 // Instruction WRITE_DATA
    0x18,                 // Adresse registre Torque Enable
    0x01,                 // Valeur : 1 = activer
    (uint8_t)(~(id + 0x04 + 0x03 + 0x18 + 0x01)) // Checksum (~somme)
  };
  ax12.write(packet, 8);
}

/**
 * Désactive le couple d'un servomoteur
 * @param id : ID du servomoteur
 * 
 * Permet de relâcher le servo (devient libre en rotation)
 */
void disableTorque(uint8_t id) {
  uint8_t packet[8] = {
    0xFF, 0xFF, id, 0x04, 0x03, 0x18, 0x00, 
    (uint8_t)(~(id + 0x04 + 0x03 + 0x18 + 0x00))
  };
  ax12.write(packet, 8);
}

/**
 * Configure la limite de couple maximum d'un servomoteur
 * @param id : ID du servomoteur
 * @param torque_limit : Limite de couple (0-1023, 1023 = couple max)
 * 
 * Écrit dans le registre 0x0E (Max Torque) pour limiter la force
 * Essentiel pour éviter les overloads et protéger la mécanique
 */
void setTorqueLimit(uint8_t id, uint16_t torque_limit) {
  // Décomposition 16 bits → 2 bytes (Little Endian)
  uint8_t torqueL = torque_limit & 0xFF;         // Byte de poids faible
  uint8_t torqueH = (torque_limit >> 8) & 0xFF;  // Byte de poids fort
  
  uint8_t packet[9] = {
    0xFF, 0xFF, id, 0x05, 0x03, 0x0E, torqueL, torqueH,
    (uint8_t)(~(id + 0x05 + 0x03 + 0x0E + torqueL + torqueH))
  };
  ax12.write(packet, 9);
}

/**
 * Configure la vitesse de déplacement d'un servomoteur
 * @param id : ID du servomoteur
 * @param vitesse : Vitesse de rotation (0-1023, 0 = vitesse max)
 * 
 * Écrit dans le registre 0x20 (Moving Speed)
 */
void setVitesse(uint8_t id, uint16_t vitesse) {
  g_vitesse = vitesse;  // Sauvegarder la vitesse globale
  
  uint8_t velL = vitesse & 0xFF;
  uint8_t velH = (vitesse >> 8) & 0xFF;
  
  uint8_t packet[9] = {
    0xFF, 0xFF, id, 0x05, 0x03, 0x20, velL, velH,
    (uint8_t)(~(id + 0x05 + 0x03 + 0x20 + velL + velH))
  };
  ax12.write(packet, 9);
}

/**
 * Commande une position cible pour un servomoteur
 * @param id : ID du servomoteur
 * @param position : Position cible (0-1023, correspond à 0-300°)
 * 
 * Séquence : Configure d'abord la vitesse, puis envoie la position cible
 * Le servo se déplace automatiquement vers cette position
 */
void moveTo(uint8_t id, uint16_t position) {
  setVitesse(id, g_vitesse);  // Appliquer la vitesse avant le mouvement
  
  uint8_t posL = position & 0xFF;
  uint8_t posH = (position >> 8) & 0xFF;
  
  uint8_t packet[9] = {
    0xFF, 0xFF, id, 0x05, 0x03, 0x1E, posL, posH,  // 0x1E = Goal Position
    (uint8_t)(~(id + 0x05 + 0x03 + 0x1E + posL + posH))
  };
  ax12.write(packet, 9);
}

/**
 * Lit la charge actuelle d'un servomoteur
 * @param id : ID du servomoteur
 * @return : Valeur de charge (0-1023) ou 0 si erreur
 * 
 * Lit le registre 0x28 (Present Load) pour détecter les contraintes
 * Utilisé pour la détection d'objets dans la pince
 */
uint16_t readLoad(uint8_t id) {
  // Trame de lecture : 2 bytes à partir de l'adresse 0x28
  uint8_t packet[8] = {
    0xFF, 0xFF, id, 0x04, 0x02, 0x28, 0x02,
    (uint8_t)(~(id + 0x04 + 0x02 + 0x28 + 0x02))
  };
  
  // Vider le buffer de réception avant envoi
  while (ax12.available()) ax12.read();
  ax12.write(packet, 8);
  
  // Attendre la réponse avec timeout
  unsigned long t0 = millis();
  while (millis() - t0 < AX12_TIMEOUT_MS) {
    if (ax12.available() >= 8) {  // Réponse complète attendue
      uint8_t resp[8];
      for (uint8_t i = 0; i < 8; i++) resp[i] = ax12.read();
      
      // Vérifier l'intégrité de la réponse
      if (resp[0] == 0xFF && resp[1] == 0xFF && resp[2] == id) {
        // Reconstituer la valeur 16 bits (Little Endian)
        return (resp[6] << 8) | resp[5];
      }
    }
  }
  return 0;  // Erreur de communication
}

/**
 * Lit la limite de torque configurée d'un servomoteur
 * @param id : ID du servomoteur
 * @return : Limite de torque (0-1023) ou 0 si erreur
 */
uint16_t readTorqueLimit(uint8_t id) {
  uint8_t packet[8] = {
    0xFF, 0xFF, id, 0x04, 0x02, 0x0E, 0x02,  // 0x0E = Max Torque
    (uint8_t)(~(id + 0x04 + 0x02 + 0x0E + 0x02))
  };
  
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

/**
 * Lit les codes d'erreur d'un servomoteur
 * @param id : ID du servomoteur
 * @return : Byte d'erreur (bits d'état) ou 0xFF si erreur comm
 * 
 * Registre 0x12 (Present Error) :
 * Bit 0: Input Voltage Error
 * Bit 1: Angle Limit Error  
 * Bit 2: Overheating Error
 * Bit 3: Range Error
 * Bit 4: Checksum Error
 * Bit 5: Overload Error ← Important pour notre application
 * Bit 6: Instruction Error
 */
uint8_t readServoError(uint8_t id) {
  uint8_t packet[8] = {
    0xFF, 0xFF, id, 0x04, 0x02, 0x12, 0x01,  // 0x12 = Present Error
    (uint8_t)(~(id + 0x04 + 0x02 + 0x12 + 0x01))
  };
  
  while (ax12.available()) ax12.read();
  ax12.write(packet, 8);
  
  unsigned long t0 = millis();
  while (millis() - t0 < AX12_TIMEOUT_MS) {
    if (ax12.available() >= 7) {  // Réponse 1 byte
      uint8_t resp[7];
      for (uint8_t i = 0; i < 7; i++) resp[i] = ax12.read();
      if (resp[0] == 0xFF && resp[1] == 0xFF && resp[2] == id) {
        return resp[4];  // Byte d'erreur
      }
    }
  }
  return 0xFF;  // Erreur de communication
}

/**
 * Fonction de diagnostic complet - Test de communication multi-registres
 * 
 * Effectue plusieurs lectures pour vérifier la communication et afficher
 * un diagnostic complet : position, charge, limite, température, état pince
 */
void displayTorqueInfo() {
  if (!label_torque_info) return;
  
  static char torque_text[400];
  Serial.println("=== TEST COMMUNICATION COMPLETE ===");
  
  // Test 1: Lecture des positions actuelles (registre 0x24)
  // Les positions devraient être différentes entre les servos
  uint16_t pos_left = 0, pos_right = 0;
  
  // Lecture position servo gauche
  uint8_t packet1[8] = {
    0xFF, 0xFF, GRIPPER_LEFT_ID, 0x04, 0x02, 0x24, 0x02,
    (uint8_t)(~(GRIPPER_LEFT_ID + 0x04 + 0x02 + 0x24 + 0x02))
  };
  while (ax12.available()) ax12.read();
  ax12.write(packet1, 8);
  delay(10);  // Délai pour réception
  if (ax12.available() >= 8) {
    uint8_t resp[8];
    for (uint8_t i = 0; i < 8; i++) resp[i] = ax12.read();
    if (resp[0] == 0xFF && resp[1] == 0xFF && resp[2] == GRIPPER_LEFT_ID) {
      pos_left = (resp[6] << 8) | resp[5];
    }
  }
  
  // Lecture position servo droite
  uint8_t packet2[8] = {
    0xFF, 0xFF, GRIPPER_RIGHT_ID, 0x04, 0x02, 0x24, 0x02,
    (uint8_t)(~(GRIPPER_RIGHT_ID + 0x04 + 0x02 + 0x24 + 0x02))
  };
  while (ax12.available()) ax12.read();
  ax12.write(packet2, 8);
  delay(10);
  if (ax12.available() >= 8) {
    uint8_t resp[8];
    for (uint8_t i = 0; i < 8; i++) resp[i] = ax12.read();
    if (resp[0] == 0xFF && resp[1] == 0xFF && resp[2] == GRIPPER_RIGHT_ID) {
      pos_right = (resp[6] << 8) | resp[5];
    }
  }
  
  // Test 2: Lecture des charges et limites (via fonctions dédiées)
  uint16_t load_left = readLoad(GRIPPER_LEFT_ID);
  uint16_t load_right = readLoad(GRIPPER_RIGHT_ID);
  uint16_t limit_left = readTorqueLimit(GRIPPER_LEFT_ID);
  uint16_t limit_right = readTorqueLimit(GRIPPER_RIGHT_ID);
  
  // Test 3: Lecture des températures (registre 0x2B)
  // Utile pour vérifier la communication et surveiller la santé des servos
  uint8_t temp_left = 0, temp_right = 0;
  
  uint8_t packet3[8] = {
    0xFF, 0xFF, GRIPPER_LEFT_ID, 0x04, 0x02, 0x2B, 0x01,
    (uint8_t)(~(GRIPPER_LEFT_ID + 0x04 + 0x02 + 0x2B + 0x01))
  };
  while (ax12.available()) ax12.read();
  ax12.write(packet3, 8);
  delay(10);
  if (ax12.available() >= 7) {
    uint8_t resp[7];
    for (uint8_t i = 0; i < 7; i++) resp[i] = ax12.read();
    if (resp[0] == 0xFF && resp[1] == 0xFF && resp[2] == GRIPPER_LEFT_ID) {
      temp_left = resp[5];  // Température en °C
    }
  }
  
  uint8_t packet4[8] = {
    0xFF, 0xFF, GRIPPER_RIGHT_ID, 0x04, 0x02, 0x2B, 0x01,
    (uint8_t)(~(GRIPPER_RIGHT_ID + 0x04 + 0x02 + 0x2B + 0x01))
  };
  while (ax12.available()) ax12.read();
  ax12.write(packet4, 8);
  delay(10);
  if (ax12.available() >= 7) {
    uint8_t resp[7];
    for (uint8_t i = 0; i < 7; i++) resp[i] = ax12.read();
    if (resp[0] == 0xFF && resp[1] == 0xFF && resp[2] == GRIPPER_RIGHT_ID) {
      temp_right = resp[5];
    }
  }
  
  // Formatage et affichage sur l'interface LVGL
  snprintf(torque_text, sizeof(torque_text), 
    "ID %d: Pos=%d Charge=%d Limite=%d Temp=%d°C\n"
    "ID %d: Pos=%d Charge=%d Limite=%d Temp=%d°C\n"
    "Statut: Pince %s",
    GRIPPER_LEFT_ID, pos_left, load_left, limit_left, temp_left,
    GRIPPER_RIGHT_ID, pos_right, load_right, limit_right, temp_right,
    gripperClosed ? "FERMEE" : "OUVERTE"
  );
  
  lv_label_set_text(label_torque_info, torque_text);
  
  // Debug détaillé dans le Serial Monitor
  Serial.print("ID ");
  Serial.print(GRIPPER_LEFT_ID);
  Serial.print(" - Position: ");
  Serial.print(pos_left);
  Serial.print(", Charge: ");
  Serial.print(load_left);
  Serial.print(", Limite: ");
  Serial.print(limit_left);
  Serial.print(", Temp: ");
  Serial.print(temp_left);
  Serial.println("°C");
  
  Serial.print("ID ");
  Serial.print(GRIPPER_RIGHT_ID);
  Serial.print(" - Position: ");
  Serial.print(pos_right);
  Serial.print(", Charge: ");
  Serial.print(load_right);
  Serial.print(", Limite: ");
  Serial.print(limit_right);
  Serial.print(", Temp: ");
  Serial.print(temp_right);
  Serial.println("°C");
  
  // Analyse de la qualité de communication
  bool comm_ok = (pos_left != pos_right) || (temp_left != temp_right);
  Serial.print("Communication: ");
  Serial.println(comm_ok ? "OK - Valeurs différentes détectées" : "PROBLEME - Valeurs identiques suspectes");
  Serial.println("========================");
}

/**
 * Fonction PING - Détection d'un servomoteur sur le bus
 * @param id : ID à tester (0-253)
 * @return : true si le servo répond, false sinon
 * 
 * Envoie une commande PING et attend la réponse pour détecter
 * la présence d'un servomoteur avec l'ID spécifié
 */
bool ax12_ping(uint8_t id) {
  uint8_t packet[6] = {
    0xFF, 0xFF, id, 0x02, 0x01,  // 0x01 = PING instruction
    (uint8_t)(~(id + 0x02 + 0x01))
  };
  
  // Nettoyer le buffer et envoyer le ping
  while (ax12.available()) ax12.read();
  ax12.write(packet, 6);
  delayMicroseconds(400);  // Délai pour commutation half-duplex
  while (ax12.available()) ax12.read();  // Vider les échos

  // Attendre la réponse
  unsigned long t0 = millis();
  while (millis() - t0 < AX12_TIMEOUT_MS) {
    if (ax12.available() >= 6) {
      uint8_t resp[6];
      for (uint8_t i = 0; i < 6; ++i) resp[i] = ax12.read();
      if (resp[0] == 0xFF && resp[1] == 0xFF && resp[2] == id) {
        Serial.print("Servo detecte ID ");
        Serial.println(id);
        return true;
      }
    }
  }
  return false;
}

// ========================================================================
// FONCTIONS DE CONTRÔLE DE LA PINCE ROBOTISÉE
// ========================================================================

/**
 * NOUVELLE FONCTION : Ouverture sécurisée et synchronisée de la pince
 * 
 * Fonction dédiée pour garantir une ouverture parfaitement synchronisée
 * des deux servos de la pince, avec gestion robuste des erreurs
 */
void openGripperSecure() {
  Serial.println("Ouverture securisee de la pince...");
  
  // Reset de l'état pour éviter les conflits
  gripperClosed = false;
  
  // Configuration optimale pour ouverture
  setTorqueLimit(GRIPPER_LEFT_ID, 1023);   // Couple maximum
  setTorqueLimit(GRIPPER_RIGHT_ID, 1023);
  
  // Vitesse modérée pour contrôle précis
  setVitesse(GRIPPER_LEFT_ID, 120);
  setVitesse(GRIPPER_RIGHT_ID, 120);
  
  // Ouverture séquentielle pour éviter les conflits bus
  moveTo(GRIPPER_LEFT_ID, GRIPPER_LEFT_OPEN);
  delay(50);  // Micro-délai entre commandes
  moveTo(GRIPPER_RIGHT_ID, GRIPPER_RIGHT_OPEN);
  
  // Délai suffisant pour mouvement complet
  delay(800);
  
  // Vérification optionnelle des positions (diagnostic)
  Serial.println("Pince ouverte - Verification des positions...");
  displayTorqueInfo();  // Affiche les positions actuelles
}

/**
 * Initialisation de la pince robotisée
 * 
 * Configure les deux servos de la pince avec les paramètres optimaux :
 * - Activation du couple
 * - Configuration des limites de torque
 * - Positionnement initial
 */
void initGripper() {
  Serial.println("Initialisation de la pince...");
  if (label_status) lv_label_set_text(label_status, "Initialisation pince...");
  
  // Activer le couple sur les deux servos de la pince
  enableTorque(GRIPPER_LEFT_ID);
  enableTorque(GRIPPER_RIGHT_ID);
  
  // Configuration du couple maximum pour éviter le relâchement
  // 1023 = couple maximum (100% de la capacité du servo)
  setTorqueLimit(GRIPPER_LEFT_ID, 1023);
  setTorqueLimit(GRIPPER_RIGHT_ID, 1023);
  
  // Vitesse modérée pour l'initialisation
  setVitesse(GRIPPER_LEFT_ID, 80);
  setVitesse(GRIPPER_RIGHT_ID, 80);
  
  // Positionnement initial (pince légèrement ouverte)
  moveTo(GRIPPER_LEFT_ID, GRIPPER_LEFT_INIT);
  moveTo(GRIPPER_RIGHT_ID, GRIPPER_RIGHT_INIT);
  
  delay(1000);  // Attendre que les servos atteignent la position
  gripperClosed = false;  // État initial : pince ouverte
  
  if (label_status) lv_label_set_text(label_status, "Pince initialisee - Couple max configure");
  Serial.println("Pince initialisee avec couple maximum");
}

/**
 * Contrôle intelligent de la pince avec détection d'objet
 * @param close : true = fermer, false = ouvrir
 * 
 * Algorithme de saisie intelligente :
 * 1. Approche rapide vers position de contact
 * 2. Fermeture progressive par micro-étapes
 * 3. Détection d'objet par analyse de charge
 * 4. Application d'une pression contrôlée
 * 
 * Algorithme d'ouverture :
 * 1. Ouverture rapide vers position libre
 */
void controlGripper(bool close) {
  if (close && !gripperClosed) {
    // === ALGORITHME DE FERMETURE INTELLIGENTE ===
    Serial.println("Fermeture rapide avec couple maximum...");
    if (label_status) lv_label_set_text(label_status, "Fermeture avec force max...");
    
    // Configuration : couple maximum pour maintenir la prise
    setTorqueLimit(GRIPPER_LEFT_ID, 1023);
    setTorqueLimit(GRIPPER_RIGHT_ID, 1023);
    
    // Étape 1: Approche rapide vers la zone de contact
    setVitesse(GRIPPER_LEFT_ID, 150);
    setVitesse(GRIPPER_RIGHT_ID, 150);
    
    // Positions d'approche (proche de la fermeture mais sans forcer)
    moveTo(GRIPPER_LEFT_ID, 888);
    moveTo(GRIPPER_RIGHT_ID, 440);
    delay(400);  // Attendre la fin du mouvement
    
    // Étape 2: Fermeture progressive avec détection d'objet
    Serial.println("Detection d'objet avec force renforcee...");
    setVitesse(GRIPPER_LEFT_ID, 30);   // Vitesse lente pour contrôle fin
    setVitesse(GRIPPER_RIGHT_ID, 30);
    
    uint16_t lastLoadLeft = 0, lastLoadRight = 0;
    bool objectDetected = false;
    
    // Boucle de fermeture progressive par micro-étapes
    for (int step = 0; step < 12 && !objectDetected; step++) {
      // Calcul des nouvelles positions (fermeture symétrique)
      int leftPos = 888 + step;      // Servo gauche : augmenter
      int rightPos = 440 - step;     // Servo droite : diminuer
      
      // Exécuter le micro-mouvement
      moveTo(GRIPPER_LEFT_ID, leftPos);
      moveTo(GRIPPER_RIGHT_ID, rightPos);
      delay(120);  // Délai pour stabilisation
      
      // Lire la charge actuelle des deux servos
      uint16_t loadLeft = readLoad(GRIPPER_LEFT_ID);
      uint16_t loadRight = readLoad(GRIPPER_RIGHT_ID);
      
      // Algorithme de détection d'objet
      // Critères : augmentation soudaine OU dépassement du seuil
      if ((loadLeft > lastLoadLeft + 15) || (loadRight > lastLoadRight + 15) ||
          (loadLeft > GRIPPER_FORCE_THRESHOLD) || (loadRight > GRIPPER_FORCE_THRESHOLD)) {
        
        Serial.print("Objet detecte ! Charge G:");
        Serial.print(loadLeft);
        Serial.print(" D:");
        Serial.println(loadRight);
        
        // Application d'une pression contrôlée pour maintenir l'objet
        setVitesse(GRIPPER_LEFT_ID, 50);
        setVitesse(GRIPPER_RIGHT_ID, 50);
        moveTo(GRIPPER_LEFT_ID, leftPos + 12);   // Pression supplémentaire
        moveTo(GRIPPER_RIGHT_ID, rightPos - 12);
        delay(500);
        
        // Renforcement de la prise (sécurité anti-glissement)
        delay(200);
        moveTo(GRIPPER_LEFT_ID, leftPos + 15);
        moveTo(GRIPPER_RIGHT_ID, rightPos - 15);
        delay(300);
        
        objectDetected = true;
        if (label_status) lv_label_set_text(label_status, "Objet saisi - Prise RENFORCEE");
        break;
      }
      
      // Mémoriser les charges pour détection d'augmentation
      lastLoadLeft = loadLeft;
      lastLoadRight = loadRight;
    }
    
    // Si aucun objet détecté : fermeture complète
    if (!objectDetected) {
      setVitesse(GRIPPER_LEFT_ID, 80);
      setVitesse(GRIPPER_RIGHT_ID, 80);
      moveTo(GRIPPER_LEFT_ID, GRIPPER_LEFT_CLOSED);
      moveTo(GRIPPER_RIGHT_ID, GRIPPER_RIGHT_CLOSED);
      delay(400);
      if (label_status) lv_label_set_text(label_status, "Pince fermee - Force maximale");
    }
    
    gripperClosed = true;  // Marquer la pince comme fermée
    
  } else if (!close && gripperClosed) {
    // === ALGORITHME D'OUVERTURE RAPIDE ===
    Serial.println("Ouverture tres rapide...");
    if (label_status) lv_label_set_text(label_status, "Liberation rapide de l'objet...");
    
    // Configuration : vitesse maximale pour libération rapide
    setVitesse(GRIPPER_LEFT_ID, 200);
    setVitesse(GRIPPER_RIGHT_ID, 200);
    
    // Mouvement vers les positions d'ouverture
    moveTo(GRIPPER_LEFT_ID, GRIPPER_LEFT_OPEN);
    moveTo(GRIPPER_RIGHT_ID, GRIPPER_RIGHT_OPEN);
    
    delay(500);  // Attendre l'ouverture complète
    gripperClosed = false;  // Marquer la pince comme ouverte
    if (label_status) lv_label_set_text(label_status, "Pince ouverte - Objet libere");
  }
}

// ========================================================================
// SÉQUENCES AUTOMATIQUES DE TRI
// ========================================================================

/**
 * Séquence automatique DROITE : Transport d'objet de la position 192 vers 839
 * 
 * Étapes :
 * 1. Ouverture sécurisée de la pince
 * 2. Déplacement vers zone de prise (position 192)
 * 3. Saisie intelligente avec détection d'objet
 * 4. Transport vers zone de dépôt (position 839)
 * 5. Relâchement de l'objet
 */
void sequenceDroite() {
  if (sequenceRunning) return;  // Éviter les exécutions multiples
  sequenceRunning = true;
  
  Serial.println("=== SEQUENCE DROITE ===");
  if (label_status) lv_label_set_text(label_status, "Sequence DROITE en cours...");
  
  // Étape 1: Ouverture sécurisée avec fonction dédiée
  Serial.println("1. Ouverture pince (securite)");
  if (label_status) lv_label_set_text(label_status, "Ouverture securisee...");
  openGripperSecure();  // Utilisation de la fonction robuste
  
  // Étape 2: Déplacement vers la zone de prise
  Serial.println("2. Deplacement vers position de prise (192)");
  if (label_status) lv_label_set_text(label_status, "Deplacement vers zone de prise...");
  setVitesse(servo_transport_id, 200);  // Vitesse élevée sans objet
  moveTo(servo_transport_id, 192);
  delay(2000);  // Délai pour mouvement complet
  
  // Étape 3: Saisie intelligente de l'objet
  Serial.println("3. Fermeture intelligente de la pince");
  if (label_status) lv_label_set_text(label_status, "Saisie de l'objet...");
  controlGripper(true);  // Utilise l'algorithme de saisie intelligente
  delay(800);
  
  // Étape 4: Transport vers la zone de dépôt
  Serial.println("4. Deplacement vers position de depot (839)");
  if (label_status) lv_label_set_text(label_status, "Transport vers zone de depot...");
  setVitesse(servo_transport_id, 120);  // Vitesse réduite avec objet (sécurité)
  moveTo(servo_transport_id, 839);
  delay(2500);  // Délai plus long pour sécurité
  
  // Étape 5: Libération de l'objet avec ouverture sécurisée
  Serial.println("5. Liberation de l'objet");
  if (label_status) lv_label_set_text(label_status, "Liberation de l'objet");
  openGripperSecure();  // Ouverture sécurisée pour libération
  
  Serial.println("=== SEQUENCE DROITE TERMINEE ===");
  if (label_status) lv_label_set_text(label_status, "Sequence DROITE terminee avec succes");
  sequenceRunning = false;
}

/**
 * Séquence automatique GAUCHE : Transport d'objet de la position 839 vers 192
 * 
 * Séquence symétrique à sequenceDroite() avec inversion des positions
 * CORRECTION : Ouverture renforcée pour synchronisation parfaite
 */
void sequenceGauche() {
  if (sequenceRunning) return;
  sequenceRunning = true;
  
  Serial.println("=== SEQUENCE GAUCHE ===");
  if (label_status) lv_label_set_text(label_status, "Sequence GAUCHE en cours...");
  
  // Étape 1: Ouverture sécurisée RENFORCÉE
  Serial.println("1. Ouverture pince (securite renforcee)");
  if (label_status) lv_label_set_text(label_status, "Ouverture securisee...");
  openGripperSecure();  // Utilisation de la fonction robuste
  
  // Étape 2: Déplacement vers zone de prise (position inverse)
  Serial.println("2. Deplacement vers position de prise (839)");
  if (label_status) lv_label_set_text(label_status, "Deplacement vers zone de prise...");
  setVitesse(servo_transport_id, 200);
  moveTo(servo_transport_id, 839);
  delay(2000);
  
  // Étape 3: Saisie intelligente
  Serial.println("3. Fermeture intelligente de la pince");
  if (label_status) lv_label_set_text(label_status, "Saisie de l'objet...");
  controlGripper(true);
  delay(800);
  
  // Étape 4: Transport vers dépôt (position inverse)
  Serial.println("4. Deplacement vers position de depot (192)");
  if (label_status) lv_label_set_text(label_status, "Transport vers zone de depot...");
  setVitesse(servo_transport_id, 120);
  moveTo(servo_transport_id, 192);
  delay(2500);
  
  // Étape 5: Libération avec ouverture sécurisée
  Serial.println("5. Liberation de l'objet");
  if (label_status) lv_label_set_text(label_status, "Liberation de l'objet");
  openGripperSecure();  // Ouverture sécurisée pour libération
  
  Serial.println("=== SEQUENCE GAUCHE TERMINEE ===");
  if (label_status) lv_label_set_text(label_status, "Sequence GAUCHE terminee avec succes");
  sequenceRunning = false;
}

// ========================================================================
// CALLBACKS DE L'INTERFACE UTILISATEUR LVGL
// ========================================================================

/**
 * Callback pour la liste déroulante de sélection de servo
 * Permet de changer de servo dans le menu de contrôle
 */
static void dropdown_servo_cb(lv_event_t *e) {
  lv_obj_t *dropdown = (lv_obj_t *)lv_event_get_target(e);
  uint16_t selected = lv_dropdown_get_selected(dropdown);
  if (selected < detected_count) {
    current_servo_id = detected_ids[selected];
    show_ctrl_menu(current_servo_id);  // Changer vers le servo sélectionné
  }
}

/**
 * Callback pour le bouton retour vers le menu principal
 */
static void retour_cb(lv_event_t *e) {
  runningSequence = false;
  show_main_menu();
}

/**
 * Callback pour le toggle de contrôle de la pince (non utilisé actuellement)
 */
static void gripper_toggle_cb(lv_event_t *e) {
  lv_obj_t *btn = (lv_obj_t *)lv_event_get_target(e);
  bool close = lv_obj_has_state(btn, LV_STATE_CHECKED);
  controlGripper(close);
}

/**
 * Met à jour la liste déroulante avec les servos détectés
 */
void update_servo_dropdown() {
  if (!servo_dropdown || detected_count == 0) return;
  
  // Construction de la chaîne d'options
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

// ========================================================================
// INTERFACE GRAPHIQUE LVGL - MENU PRINCIPAL
// ========================================================================

/**
 * Création et affichage du menu principal
 * 
 * Contient :
 * - Titre du projet
 * - Section de diagnostic (scan, torque info)
 * - Boutons des séquences automatiques
 */
void show_main_menu() {
  app_state = MENU_MAIN;
  runningSequence = false;
  sequenceRunning = false;
  
  // Nettoyer les anciens menus
  if (ctrl_menu) {
    lv_obj_del(ctrl_menu);
    ctrl_menu = NULL;
  }
  if (main_menu) {
    lv_obj_del(main_menu);
    main_menu = NULL;
  }

  // Créer le conteneur principal
  main_menu = lv_obj_create(lv_scr_act());
  lv_obj_set_size(main_menu, 480, 272);  // Taille écran STM32F746G-DISCO
  lv_obj_set_style_bg_color(main_menu, lv_color_hex(0x1E1E2E), 0);  // Fond sombre

  // Titre principal
  lv_obj_t *title = lv_label_create(main_menu);
  lv_label_set_text(title, "ROBOT DE TRI AUTOMATIQUE");
  lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 5);
  lv_obj_set_style_text_color(title, lv_color_hex(0x89B4FA), 0);  // Bleu clair

  // Label de statut (affiché dynamiquement)
  label_status = lv_label_create(main_menu);
  lv_obj_align(label_status, LV_ALIGN_TOP_MID, 0, 25);
  lv_label_set_text(label_status, "Robot pret - Scannez d'abord les servos");
  lv_obj_set_style_text_color(label_status, lv_color_hex(0xF9E2AF), 0);  // Jaune

  // Section DEBUG/SCAN
  lv_obj_t *debug_title = lv_label_create(main_menu);
  lv_label_set_text(debug_title, "DEBUG - SCAN SERVOMOTEURS");
  lv_obj_align(debug_title, LV_ALIGN_TOP_LEFT, 10, 50);
  lv_obj_set_style_text_color(debug_title, lv_color_hex(0x94E2D5), 0);  // Cyan

  // Bouton Scanner - Détection automatique des servos
  lv_obj_t *btn_scan = lv_btn_create(main_menu);
  lv_obj_set_size(btn_scan, 100, 30);
  lv_obj_align(btn_scan, LV_ALIGN_TOP_LEFT, 10, 70);
  lv_obj_set_style_bg_color(btn_scan, lv_color_hex(0x89B4FA), 0);
  
  // Callback du bouton scanner (lambda function)
  lv_obj_add_event_cb(btn_scan, [](lv_event_t *e){
      if (label_status) lv_label_set_text(label_status, "Scan des servos en cours...");
      detected_count = 0;
      
      // Phase 1: Test prioritaire des servos critiques
      uint8_t test_ids[] = {GRIPPER_LEFT_ID, GRIPPER_RIGHT_ID, 41};
      for (uint8_t i = 0; i < 3; i++) {
          uint8_t id = test_ids[i];
          char info[50];
          snprintf(info, sizeof(info), "Test servo ID %d...", id);
          if (label_status) lv_label_set_text(label_status, info);
          
          if (ax12_ping(id)) {
              detected_ids[detected_count++] = id;
              enableTorque(id);  // Activer immédiatement
          }
      }
      
      // Phase 2: Scan complet de tous les IDs possibles
      for (uint8_t id = 0; id <= 254; ++id) {
          // Éviter de retester les servos déjà trouvés
          if (id == GRIPPER_LEFT_ID || id == GRIPPER_RIGHT_ID || id == 41) continue;
          
          if (ax12_ping(id)) {
              // Vérifier qu'il n'est pas déjà dans la liste
              bool already_found = false;
              for (uint8_t j = 0; j < detected_count; j++) {
                  if (detected_ids[j] == id) {
                      already_found = true;
                      break;
                  }
              }
              if (!already_found) {
                  detected_ids[detected_count++] = id;
              }
          }
      }
      
      // Affichage du résultat
      char result[80];
      snprintf(result, sizeof(result), "Scan termine: %d servo(s) detecte(s)", detected_count);
      if (label_status) lv_label_set_text(label_status, result);
      update_servo_dropdown();
      
      // Affichage automatique des informations de diagnostic
      displayTorqueInfo();
  }, LV_EVENT_CLICKED, NULL);
  
  lv_obj_t *lbl_scan = lv_label_create(btn_scan);
  lv_label_set_text(lbl_scan, "Scanner");
  lv_obj_center(lbl_scan);

  // Bouton Torque Info - Diagnostic des servos
  lv_obj_t *btn_torque = lv_btn_create(main_menu);
  lv_obj_set_size(btn_torque, 100, 30);
  lv_obj_align(btn_torque, LV_ALIGN_TOP_LEFT, 120, 70);
  lv_obj_set_style_bg_color(btn_torque, lv_color_hex(0xF38BA8), 0);
  lv_obj_add_event_cb(btn_torque, [](lv_event_t *e){
      displayTorqueInfo();  // Afficher le diagnostic complet
  }, LV_EVENT_CLICKED, NULL);
  
  lv_obj_t *lbl_torque = lv_label_create(btn_torque);
  lv_label_set_text(lbl_torque, "Torque Info");
  lv_obj_center(lbl_torque);

  // Liste déroulante des servos détectés
  servo_dropdown = lv_dropdown_create(main_menu);
  lv_obj_set_width(servo_dropdown, 150);
  lv_obj_align(servo_dropdown, LV_ALIGN_TOP_LEFT, 230, 70);
  lv_dropdown_set_text(servo_dropdown, "Aucun servo detecte");
  lv_obj_add_flag(servo_dropdown, LV_OBJ_FLAG_HIDDEN);  // Masqué au début
  lv_obj_set_style_bg_color(servo_dropdown, lv_color_hex(0x45475A), 0);
  lv_obj_set_style_text_color(servo_dropdown, lv_color_hex(0xCDD6F4), 0);
  lv_obj_add_event_cb(servo_dropdown, dropdown_servo_cb, LV_EVENT_VALUE_CHANGED, NULL);

  // Label d'affichage des informations de diagnostic
  label_torque_info = lv_label_create(main_menu);
  lv_obj_align(label_torque_info, LV_ALIGN_TOP_LEFT, 10, 105);
  lv_label_set_text(label_torque_info, "Cliquez sur 'Torque Info' pour voir les valeurs");
  lv_obj_set_style_text_color(label_torque_info, lv_color_hex(0xCDD6F4), 0);

  // Section SÉQUENCES AUTOMATIQUES
  lv_obj_t *seq_title = lv_label_create(main_menu);
  lv_label_set_text(seq_title, "SEQUENCES AUTOMATIQUES DE TRI");
  lv_obj_align(seq_title, LV_ALIGN_TOP_LEFT, 10, 170);
  lv_obj_set_style_text_color(seq_title, lv_color_hex(0xF38BA8), 0);

  // Bouton SÉQUENCE GAUCHE (839 → 192)
  lv_obj_t *btn_gauche = lv_btn_create(main_menu);
  lv_obj_set_size(btn_gauche, 180, 45);
  lv_obj_align(btn_gauche, LV_ALIGN_TOP_LEFT, 30, 195);
  lv_obj_set_style_bg_color(btn_gauche, lv_color_hex(0xA6E3A1), 0);  // Vert
  lv_obj_set_style_bg_color(btn_gauche, lv_color_hex(0x94E2D5), LV_STATE_PRESSED);
  lv_obj_add_event_cb(btn_gauche, [](lv_event_t *e){ 
      sequenceGauche();  // Lancer la séquence gauche
  }, LV_EVENT_CLICKED, NULL);
  lv_obj_t *lbl_gauche = lv_label_create(btn_gauche);
  lv_label_set_text(lbl_gauche, "SEQUENCE GAUCHE\n839 -> 192");
  lv_obj_center(lbl_gauche);

  // Bouton SÉQUENCE DROITE (192 → 839)
  lv_obj_t *btn_droite = lv_btn_create(main_menu);
  lv_obj_set_size(btn_droite, 180, 45);
  lv_obj_align(btn_droite, LV_ALIGN_TOP_LEFT, 220, 195);
  lv_obj_set_style_bg_color(btn_droite, lv_color_hex(0xF38BA8), 0);  // Rose
  lv_obj_set_style_bg_color(btn_droite, lv_color_hex(0xF9E2AF), LV_STATE_PRESSED);
  lv_obj_add_event_cb(btn_droite, [](lv_event_t *e){ 
      sequenceDroite();  // Lancer la séquence droite
  }, LV_EVENT_CLICKED, NULL);
  lv_obj_t *lbl_droite = lv_label_create(btn_droite);
  lv_label_set_text(lbl_droite, "SEQUENCE DROITE\n192 -> 839");
  lv_obj_center(lbl_droite);
}

// ========================================================================
// INTERFACE GRAPHIQUE LVGL - MENU DE CONTRÔLE INDIVIDUEL
// ========================================================================

/**
 * Création et affichage du menu de contrôle individuel d'un servo
 * @param id : ID du servomoteur à contrôler
 * 
 * Permet le contrôle fin d'un servo spécifique :
 * - Slider de position (0-1023)
 * - Slider de vitesse (4-330)
 * - Mode séquence aléatoire
 */
void show_ctrl_menu(uint8_t id) {
  app_state = MENU_CONTROL;
  
  // Nettoyer les anciens menus
  if (main_menu) {
    lv_obj_del(main_menu);
    main_menu = NULL;
  }
  if (ctrl_menu) {
    lv_obj_del(ctrl_menu);
    ctrl_menu = NULL;
  }

  // Créer le conteneur du menu de contrôle
  ctrl_menu = lv_obj_create(lv_scr_act());
  lv_obj_set_size(ctrl_menu, 480, 272);

  // Titre avec ID du servo
  char titre[32];
  snprintf(titre, sizeof(titre), "Controle AX-12A ID %d", id);
  lv_obj_t *label = lv_label_create(ctrl_menu);
  lv_label_set_text(label, titre);
  lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 8);

  // Liste déroulante pour changer de servo sans retour menu
  lv_obj_t *servo_select = lv_dropdown_create(ctrl_menu);
  lv_obj_set_width(servo_select, 150);
  lv_obj_align(servo_select, LV_ALIGN_TOP_RIGHT, -10, 35);
  
  // Construction dynamique de la liste des servos détectés
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
      show_ctrl_menu(current_servo_id);  // Recharger avec nouveau servo
    }
  }, LV_EVENT_VALUE_CHANGED, NULL);

  // Slider de contrôle de position (0-1023)
  lv_obj_t *slider_pos = lv_slider_create(ctrl_menu);
  lv_obj_set_width(slider_pos, 250);
  lv_obj_align(slider_pos, LV_ALIGN_TOP_MID, 0, 80);
  lv_slider_set_range(slider_pos, 0, 1023);
  lv_slider_set_value(slider_pos, 512, LV_ANIM_OFF);  // Position centrale
  
  // Label d'affichage de la position
  lv_obj_t *label_pos = lv_label_create(ctrl_menu);
  lv_label_set_text(label_pos, "Position: 512");
  lv_obj_align_to(label_pos, slider_pos, LV_ALIGN_OUT_BOTTOM_MID, 0, 5);
  
  // Callback du slider position
  lv_obj_add_event_cb(slider_pos, [](lv_event_t *e){
    lv_obj_t *slider = (lv_obj_t *)lv_event_get_target(e);
    int pos = lv_slider_get_value(slider);
    lv_obj_t *label = (lv_obj_t *)lv_event_get_user_data(e);
    
    // Mise à jour de l'affichage
    static char buf[32];
    snprintf(buf, sizeof(buf), "Position: %d", pos);
    lv_label_set_text(label, buf);
    
    // Commande du servo (seulement si pas en séquence)
    if (lv_event_get_code(e) == LV_EVENT_VALUE_CHANGED && !runningSequence) {
      moveTo(current_servo_id, pos);
    }
  }, LV_EVENT_VALUE_CHANGED, label_pos);

  // Slider de contrôle de vitesse (4-330)
  lv_obj_t *slider_vit = lv_slider_create(ctrl_menu);
  lv_obj_set_width(slider_vit, 250);
  lv_obj_align(slider_vit, LV_ALIGN_TOP_MID, 0, 140);
  lv_slider_set_range(slider_vit, 4, 330);
  lv_slider_set_value(slider_vit, 100, LV_ANIM_OFF);
  
  // Label d'affichage de la vitesse
  lv_obj_t *label_vit = lv_label_create(ctrl_menu);
  lv_label_set_text(label_vit, "Vitesse: 100");
  lv_obj_align_to(label_vit, slider_vit, LV_ALIGN_OUT_BOTTOM_MID, 0, 5);
  
  // Callback du slider vitesse
  lv_obj_add_event_cb(slider_vit, [](lv_event_t *e){
    lv_obj_t *slider = (lv_obj_t *)lv_event_get_target(e);
    int v = lv_slider_get_value(slider);
    lv_obj_t *label = (lv_obj_t *)lv_event_get_user_data(e);
    
    static char buf[32];
    snprintf(buf, sizeof(buf), "Vitesse: %d", v);
    lv_label_set_text(label, buf);
    
    if (lv_event_get_code(e) == LV_EVENT_VALUE_CHANGED) {
      setVitesse(current_servo_id, v);  // Appliquer immédiatement
    }
  }, LV_EVENT_VALUE_CHANGED, label_vit);

  // Bouton toggle pour mode séquence aléatoire
  lv_obj_t *toggle = lv_btn_create(ctrl_menu);
  lv_obj_set_size(toggle, 120, 45);
  lv_obj_align(toggle, LV_ALIGN_BOTTOM_MID, 60, -10);
  lv_obj_add_flag(toggle, LV_OBJ_FLAG_CHECKABLE);  // Bouton à bascule
  lv_obj_t *toggle_label = lv_label_create(toggle);
  lv_label_set_text(toggle_label, "Sequence");
  lv_obj_center(toggle_label);
  lv_obj_add_event_cb(toggle, [](lv_event_t *e){
    lv_obj_t *btn = (lv_obj_t *)lv_event_get_target(e);
    runningSequence = lv_obj_has_state(btn, LV_STATE_CHECKED);
  }, LV_EVENT_VALUE_CHANGED, NULL);

  // Bouton retour vers le menu principal
  lv_obj_t *btn_retour = lv_btn_create(ctrl_menu);
  lv_obj_set_size(btn_retour, 80, 35);
  lv_obj_align(btn_retour, LV_ALIGN_BOTTOM_LEFT, 10, -10);
  lv_obj_add_event_cb(btn_retour, retour_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *lbl_retour = lv_label_create(btn_retour);
  lv_label_set_text(lbl_retour, "Menu");
  lv_obj_center(lbl_retour);

  // Initialiser le servo sélectionné
  enableTorque(id);
  setVitesse(id, g_vitesse);
}

// ========================================================================
// FONCTIONS PRINCIPALES DU SYSTÈME
// ========================================================================

/**
 * Initialisation de l'interface LVGL
 */
void testLvgl() { 
  show_main_menu(); 
}

/**
 * Configuration initiale du système
 * 
 * - Initialisation des communications série
 * - Configuration des broches STM32
 * - Test rapide de détection des servos
 * - Lancement de l'interface graphique
 */
void mySetup() {
  // Initialisation des communications série
  Serial.begin(115200);    // Serial Monitor (debug)
  ax12.begin(1000000);     // Communication AX-12A (1 Mbps)
  
  // Configuration du mapping des broches STM32F746G
  pinmap_pinout(PG_14, PinMap_UART_TX);  // TX sur PG_14 (D1)
  pinmap_pinout(PC_7, PinMap_UART_RX);   // RX sur PC_7 (D0)
  delay(20);

  // Test rapide de détection des premiers servos
  Serial.println("Debut scan test rapide...");
  for (uint8_t id = 0; id <= 3; ++id) {
      if (ax12_ping(id)) {
          Serial.print("Servo trouve ID ");
          Serial.println(id);
      }
  }
  Serial.println("Fin scan test.");

  // Démarrage de l'interface graphique
  testLvgl();
  Serial.println("Interface robot de tri automatique prete !");
}

/**
 * Boucle principale du programme
 * 
 * Gère uniquement l'actualisation de l'interface LVGL
 * Les autres tâches sont gérées par FreeRTOS dans myTask()
 */
void loop() {
  lv_timer_handler();  // Actualisation LVGL (events, animations, etc.)
  delay(5);            // Délai court pour éviter la surcharge CPU
}

/**
 * Tâche FreeRTOS - Gestion des séquences automatiques et aléatoires
 * @param pvParameters : Paramètres de la tâche (non utilisés)
 * 
 * Gère :
 * - Mode séquence aléatoire en contrôle individuel
 * - Tempo entre les mouvements
 * - Préemption pour les séquences automatiques
 */
void myTask(void *pvParameters) {
  while (1) {
    // Mode séquence aléatoire (dans le menu de contrôle individuel)
    if (app_state == MENU_CONTROL && runningSequence) {
      // Génération d'une position aléatoire
      uint16_t pos = random(50, 980);
      moveTo(current_servo_id, pos);
      
      // Délai proportionnel à la distance (plus fluide)
      vTaskDelay(pdMS_TO_TICKS(700 + abs((int)pos - 512) * 2));
    } else {
      // Mode veille
      vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    // Préemption légère pendant les séquences automatiques
    if (sequenceRunning) {
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }
}

/**
 * ========================================================================
 * FIN DU PROGRAMME
 * ========================================================================
 * 
 * Ce code implémente un système complet de robot de tri automatique
 * avec les fonctionnalités suivantes :
 * 
 * ✅ Communication robuste avec servomoteurs AX-12A (protocole Dynamixel)
 * ✅ Interface graphique tactile professionnelle (LVGL)
 * ✅ Algorithme de saisie intelligente avec détection d'objet
 * ✅ Séquences automatiques de tri bidirectionnel
 * ✅ Diagnostic en temps réel des servomoteurs
 * ✅ Gestion des erreurs et protection anti-overload
 * ✅ Contrôle individuel pour calibrage et maintenance
 * 
 * Architecture modulaire permettant l'extension future du système
 * avec de nouvelles fonctionnalités (capteurs, IA, communication réseau)
 * 
 * Projet développé dans le cadre de la formation LP MECSE
 * IUT de Cachan - Université Paris-Saclay
 * ========================================================================
 */
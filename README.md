# 🤖 TRIAX - Robot de Tri Automatique

**Projet d'Instrumentation - LP MECSE**  
**IUT de Cachan (Université Paris-Saclay)**

Système robotique intelligent utilisant des **servomoteurs Dynamixel AX-12A** du kit **BIOLOID** pilotés par un **STM32F746G-DISCO** avec interface graphique **LVGL**.

---

## 🎯 Description du Projet

**TRIAX** est un robot autonome de tri d'objets entre deux zones parallèles, développé dans le cadre du module d'instrumentation de la LP MECSE. Le système intègre une **pince robotisée intelligente** avec détection d'objet et un **bras de transport linéaire**.

### 🎬 Démonstration en Action

![TRIAX en fonctionnement](triax-demo.gif)


*Séquence complète de tri : détection → saisie → transport → dépôt*

### 🎓 Objectifs Pédagogiques
- **Systèmes embarqués** : Programmation STM32 et gestion temps réel
- **Instrumentation** : Intégration capteurs et actionneurs intelligents
- **Protocoles industriels** : Maîtrise du bus Dynamixel et communications série
- **Interface utilisateur** : Développement d'IHM tactile professionnelle
- **Mécatronique** : Fusion hardware/software pour système robotique complet

### ✨ Fonctionnalités Principales

- 🔄 **Séquences automatiques** de tri bidirectionnel (Gauche ↔ Droite)
- 🦾 **Pince intelligente** avec détection d'objet par analyse de charge
- 📡 **Scan automatique** des servomoteurs sur le bus Dynamixel
- 🎛️ **Interface graphique** tactile complète avec LVGL
- 🔧 **Contrôle individuel** de chaque servo (position, vitesse, diagnostics)
- 📊 **Monitoring temps réel** (torque, température, erreurs)
- 🛡️ **Protection anti-surcharge** automatique avec recovery

---

## 🔩 Configuration Matérielle

### Plateforme de Développement
- **Microcontrôleur** : STM32F746G-DISCO (Cortex-M7 à 216 MHz)
- **Pièces mécaniques** : Kit robotique BIOLOID (assemblage structural)
- **Servomoteurs** : 3x AX-12A Dynamixel (couple 1.5 N⋅m, résolution 0.29°)
- **Support mécanique** : Planche de montage stabilisée
- **Alimentation** : Power supply de laboratoire (11-12V, 3A)

### Architecture Mécanique
- **Pince robotisée** : Assemblée avec pièces mécaniques du kit BIOLOID
- **Bras articulé** : Fixation rigide sur support pour stabilité optimale
- **Zones de tri** : Deux tapis parallèles pour transfert d'objets
- **Course linéaire** : Déplacement entre positions 192 et 839 (unités servo)

### Configuration des Servomoteurs
| ID | Fonction | Position Init | Position Fermée | Position Ouverte | Rôle |
|----|----------|---------------|-----------------|------------------|------|
| **1** | Pince Gauche | 830 | 950 | 724 | Saisie/Relâchement |
| **23** | Pince Droite | 520 | 380 | 620 | Saisie/Relâchement |
| **41** | Transport | Variable | - | - | Déplacement linéaire |

### Connexions Électriques
- **UART6** : Communication Dynamixel (1 Mbps, Half-duplex)
  - TX: **PG_14** (D1) - Transmission vers servos
  - RX: **PC_7** (D0) - Réception depuis servos
- **Écran tactile** : Interface LVGL intégrée (480×272 pixels)
- **Alimentation** : Power supply lab (tension ajustable, protection court-circuit)

---

## 🗂️ Structure du Projet

```
TRIAX/
├── src/
│   └── main.cpp              # Code principal commenté (1000+ lignes)
├── include/
│   ├── lvglDrivers.h         # Pilotes écran tactile STM32F746G
│   └── PeripheralPins.h      # Mapping broches microcontrôleur
├── docs/
│   ├── rapport_fin_module.md # Documentation technique complète
│   └── sequences.md          # Algorithmes de tri détaillés
├── schematics/
│   └── connexions.png        # Schémas de raccordement
├── platformio.ini            # Configuration PlatformIO
└── README.md                 # Ce fichier
```

---

## ⚙️ Configuration PlatformIO

```ini
[env:disco_f746ng]
platform = ststm32
board = disco_f746ng
framework = arduino
upload_protocol = stlink
build_flags = 
    -DLV_CONF_INCLUDE_SIMPLE
    -DHAL_UART_MODULE_ENABLED
lib_deps = 
    lvgl/lvgl@^8.3.0
```

---

## 🚀 Utilisation

### 1. Interface Principale

Au démarrage, **TRIAX** affiche son interface tactile avec :
- **Titre du projet** et statut système en temps réel
- **Section DEBUG** : Scan automatique et diagnostic des servos
- **Boutons de séquences** : GAUCHE (839→192) et DROITE (192→839)

### 2. Scan et Diagnostic

```cpp
// Cliquer sur "Scanner" pour détecter automatiquement :
// ✓ Servos critiques (ID 1, 23, 41) en priorité
// ✓ Scan complet bus Dynamixel (ID 0-254)
// ✓ Activation automatique du couple moteur
// ✓ Affichage dans dropdown pour contrôle individuel
```

**"Torque Info"** affiche le diagnostic complet :
- Position actuelle, charge instantanée, limite de torque
- Température des moteurs, codes d'erreur éventuels
- Statut de la pince (OUVERTE/FERMÉE)

### 3. Séquences Automatiques

#### 🟢 Séquence GAUCHE (839 → 192)
1. **Ouverture sécurisée** avec synchronisation renforcée
2. **Transport** vers zone de prise (position 839)
3. **Saisie intelligente** par analyse de charge temps réel
4. **Déplacement** vers zone de dépôt (position 192)
5. **Libération** contrôlée de l'objet

#### 🔴 Séquence DROITE (192 → 839)
Séquence symétrique avec inversion des positions de prise/dépôt.

### 4. Algorithme de Saisie Intelligente

```cpp
// Approche rapide → Fermeture progressive par micro-étapes
for (int step = 0; step < 12 && !objectDetected; step++) {
    // Mouvement coordonné des servos
    moveTo(GRIPPER_LEFT_ID, leftPos + step);
    moveTo(GRIPPER_RIGHT_ID, rightPos - step);
    
    // Analyse de la charge pour détection d'objet
    if (loadIncrease > THRESHOLD || loadAbsolute > LIMIT) {
        objectDetected = true;
        // Application pression contrôlée pour maintien
    }
}
```

### 5. Contrôle Individuel

Sélection d'un servo via dropdown → Menu de contrôle avancé :
- **Slider position** : 0-1023 (équivalent 0-300°)
- **Slider vitesse** : 4-330 (vitesse de rotation)
- **Mode séquence aléatoire** : Test automatique de mouvement
- **Retour menu principal** : Navigation fluide

---

## 🛡️ Sécurités et Robustesse

### Protection Anti-Overload
- **Détection temps réel** des surcharges par lecture registre d'erreur
- **Limites de couple** configurables (0-1023, défaut 1023 = couple max)
- **Recovery automatique** : reset + reconfiguration en cas d'erreur
- **Algorithme adaptatif** : réduction automatique du couple si nécessaire

### Communication Robuste
- **Protocole Dynamixel v1.0** avec checksum et détection d'erreurs
- **Timeouts optimisés** : 10ms par transaction, retry automatique
- **Gestion half-duplex** : délais calibrés pour commutation TX/RX
- **Scan intelligent** : priorité aux servos critiques, puis scan complet

### Interface Utilisateur
- **Feedback visuel** permanent (statuts, erreurs, progression)
- **Messages informatifs** pour guidage utilisateur
- **Protection logicielle** : impossibilité de lancer plusieurs séquences
- **Navigation intuitive** : retour menu, sélection servo sans redémarrage

---

## 📊 Performance et Métrologie

### Caractéristiques Mesurées
- **Temps de cycle complet** : 15-20 secondes par objet transporté
- **Précision de positionnement** : ±1 unité servo (≈ 0.3° angulaire)
- **Taux de réussite saisie** : >95% pour objets standards (20-50mm)
- **Vitesse de communication** : 1 Mbps stable sur bus Dynamixel

### Monitoring Temps Réel
- **Position** : Lecture registre 0x24 (Present Position)
- **Charge** : Registre 0x28 (Present Load) pour détection d'effort
- **Température** : Registre 0x2B pour surveillance thermique moteurs
- **Erreurs** : Registre 0x12 (flags overload, voltage, temperature)

---

## 🔧 Instrumentation et Protocoles

### Bus Dynamixel v1.0
**Format de trame** : `[0xFF][0xFF][ID][LENGTH][INSTRUCTION][PARAMS][CHECKSUM]`

**Instructions principales utilisées** :
- `PING (0x01)` : Détection présence servo
- `WRITE_DATA (0x03)` : Configuration registres (position, vitesse, couple)
- `READ_DATA (0x02)` : Lecture état (position, charge, température)

**Registres critiques** :
- `0x1E` Goal Position (0-1023)
- `0x20` Moving Speed (0-1023) 
- `0x0E` Max Torque (limitation couple)
- `0x18` Torque Enable (activation moteur)

### Architecture Logicielle
```cpp
// Couches logicielles
┌─────────────────────────────────────┐
│ Interface LVGL (IHM tactile)       │
├─────────────────────────────────────┤
│ Logique métier (séquences, algo)   │
├─────────────────────────────────────┤
│ Protocole Dynamixel (communication)│
├─────────────────────────────────────┤
│ HAL STM32 (UART, GPIO, timers)     │
└─────────────────────────────────────┘
```

---

## 🚧 Évolutions Futures

### Court terme
- [ ] **Calibrage automatique** des positions limites de pince
- [ ] **Sauvegarde EEPROM** des paramètres utilisateur
- [ ] **Mode maintenance** avec diagnostic approfondi
- [ ] **Optimisation vitesses** selon type d'objet détecté

### Moyen terme  
- [ ] **Capteur de vision** pour reconnaissance forme/couleur
- [ ] **Interface web** de monitoring à distance (WiFi/Ethernet)
- [ ] **Apprentissage adaptatif** des paramètres de saisie
- [ ] **Multi-robots** : coordination de plusieurs unités TRIAX

### Long terme
- [ ] **Intelligence artificielle** pour optimisation des trajectoires
- [ ] **Intégration IoT** avec supervision cloud
- [ ] **Modularité hardware** : adaptation différents effecteurs
- [ ] **Certification industrielle** pour environnements production

---

## 📝 Documentation Technique

### Registres AX-12A Utilisés
| Adresse | Nom | R/W | Description |
|---------|-----|-----|-------------|
| 0x0E | Max Torque | RW | Limite couple moteur (0-1023) |
| 0x12 | Present Error | R | Flags d'erreur (overload, voltage, etc.) |
| 0x18 | Torque Enable | RW | Activation/désactivation moteur |
| 0x1E | Goal Position | RW | Position cible (0-1023 = 0-300°) |
| 0x20 | Moving Speed | RW | Vitesse rotation (0-1023) |
| 0x24 | Present Position | R | Position actuelle lue |
| 0x28 | Present Load | R | Charge/effort instantané |
| 0x2B | Present Temperature | R | Température interne moteur |

### Calculs de Conversion
```cpp
// Position angulaire ↔ Unités servo
float angle_deg = (servo_units * 300.0) / 1023.0;
uint16_t servo_units = (angle_deg * 1023.0) / 300.0;

// Charge ↔ Pourcentage couple
float torque_percent = (load_value * 100.0) / torque_limit;
```

---

## 📚 Références et Standards

### Normes Appliquées
- **Dynamixel Protocol v1.0** (Robotis Inc.)
- **STM32F746xx Reference Manual** (STMicroelectronics)
- **LVGL Documentation v8.3** (Interface graphique)
- **Arduino Framework** pour développement rapid prototyping

### Documentation Projet
- **Code source** : 100% commenté avec explications détaillées
- **Rapport technique** : Architecture, algorithmes, résultats
- **Schémas électriques** : Connexions et interfaces
- **Guide utilisateur** : Procédures d'utilisation et maintenance

---

## 👥 Équipe Projet

**DJOUDI Rayan** - *Étudiant LP MECSE, IUT de Cachan (Université Paris-Saclay)*
- Conception et développement système complet
- Programmation embedded STM32 + interface LVGL  
- Assemblage mécanique avec composants BIOLOID
- Algorithmes de saisie intelligente et séquences automatiques
- Documentation technique et validation expérimentale

---

## 🎓 Contexte Académique

**Formation** : Licence Professionnelle MECSE (Mécatronique et Électronique des Systèmes Embarqués)  
**Établissement** : IUT de Cachan - Université Paris-Saclay  
**Module** : Projet d'Instrumentation  
**Année académique** : 2024-2025  

### Compétences Développées
- 🔧 **Programmation embarquée** : STM32, C++, framework Arduino
- 🤖 **Robotique industrielle** : Servomoteurs intelligents, protocoles série
- 🎨 **Interface homme-machine** : LVGL, design d'expérience utilisateur
- ⚡ **Instrumentation** : Capteurs/actionneurs, acquisition de données
- 🛠️ **Intégration système** : Mécanique + Électronique + Logiciel
- 📊 **Métrologie** : Mesures, validation, analyse de performance

### Livrables Académiques
- **Démonstration fonctionnelle** : Vendredi 20 juin 2025
- **Rapport technique** : Documentation complète du système
- **Code source commenté** : Repository GitHub public
- **Présentation orale** : Soutenance devant jury technique

---

## 🏫 Remerciements

- **Équipe pédagogique LP MECSE** - Encadrement technique et méthodologique
- **Laboratoire IUT de Cachan** - Mise à disposition matériel (BIOLOID, STM32, équipements)
- **Université Paris-Saclay** - Environnement de formation d'excellence
- **Robotis Inc.** - Documentation technique des servomoteurs Dynamixel

---

## 📜 Licence et Usage

**Projet académique** - IUT de Cachan (Université Paris-Saclay)  
Code source disponible à des fins éducatives et de démonstration.  
Utilisation commerciale soumise à autorisation de l'auteur.

---

## 📞 Contact

**Repository GitHub** : `https://github.com/RayKill/TRIAX`  
**Email étudiant** : `rayan.djou@gmail.com`  

---

*TRIAX - Projet d'instrumentation développé en LP MECSE à l'IUT de Cachan 🎓*
*"Quand la mécatronique rencontre l'intelligence artificielle" 🤖*

# 🤖 Robot de Tri Automatique - AX12A + LVGL + STM32F746G

**Projet académique IUT de Cachan (Université Paris-Saclay)**  
**Département GE2II - Génie Électrique et Informatique Industrielle**

Projet PlatformIO pour un **robot de tri automatique** utilisant des **servomoteurs Dynamixel AX-12A** du kit **BIOLOID** pilotés par un **STM32F746G-DISCO** avec interface graphique **LVGL**.

## 🎯 Description du Projet

Robot autonome de tri d'objets entre deux tapis parallèles, équipé d'une **pince robotisée intelligente** et d'un **système de transport linéaire**.

### 🎓 Objectifs Pédagogiques
- **Intégration système** : Combiner hardware BIOLOID et software embarqué
- **Programmation temps réel** : Gestion de séquences robotiques complexes
- **Interface utilisateur** : Développement d'IHM tactile avec LVGL
- **Protocoles industriels** : Maîtrise du bus Dynamixel et UART
- **Gestion d'erreurs** : Implémentation de sécurités anti-overload

### ✨ Fonctionnalités Principales

- 🔄 **Séquences automatiques** de tri (Gauche ↔ Droite)
- 🦾 **Pince intelligente** avec détection d'objet et gestion anti-overload
- 📡 **Scan automatique** des servomoteurs sur le bus Dynamixel
- 🎛️ **Interface graphique** complète avec LVGL
- 🔧 **Contrôle individuel** de chaque servo (position, vitesse)
- 📊 **Diagnostic en temps réel** (torque, température, erreurs)
- 🛡️ **Protection anti-surcharge** automatique

---

## 🔩 Configuration Matérielle

### Plateforme de Développement
- **Microcontrôleur** : STM32F746G-DISCO
- **Pièces mécaniques** : Kit robotique BIOLOID (assemblage uniquement)
- **Servomoteurs** : AX-12A Dynamixel
- **Support mécanique** : Planche de montage stabilisée
- **Alimentation** : Power supply de laboratoire

### Construction Mécanique
- **Pince robotisée** : Assemblée avec pièces mécaniques du kit BIOLOID
- **Bras articulé** : Fixé sur planche pour stabilité optimale
- **Tapis de tri** : Deux zones parallèles de transfert d'objets

### Servomoteurs AX-12A
| ID | Fonction | Position Initiale | Rôle |
|----|----------|------------------|------|
| **1** | Pince Gauche | 830 | Saisie/Relâchement |
| **23** | Pince Droite | 520 | Saisie/Relâchement |
| **41** | Transport | Variable | Déplacement linéaire |

### Connexions Électriques
- **UART6** : Communication Dynamixel (1 Mbps)
  - TX: **PG_14** (D1)
  - RX: **PC_7** (D0)
- **Écran tactile** : Interface LVGL intégrée
- **Alimentation** : Power supply de laboratoire (tension ajustable)

---

## 🗂️ Structure du Projet

```
robot-tri-automatique/
├── include/
│   ├── lvglDrivers.h
│   └── PeripheralPins.h
├── src/
│   └── main.cpp              # Code principal
├── platformio.ini            # Configuration PlatformIO
├── README.md
└── docs/
    └── sequences.md          # Documentation des séquences
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

Au démarrage, l'interface affiche :
- **Statut du robot** en temps réel
- **Section DEBUG** : Scan et diagnostic
- **Boutons de séquences** automatiques

### 2. Scan des Servomoteurs

```cpp
// Cliquer sur "Scanner" pour détecter automatiquement :
// - Servos de la pince (ID 1, 23)
// - Servo de transport (ID 41)  
// - Autres servos présents (ID 0-254)
```

### 3. Séquences Automatiques

#### 🟢 Séquence GAUCHE (839 → 192)
1. Ouverture sécurisée de la pince
2. Déplacement vers zone de prise (position 839)
3. **Saisie intelligente** avec détection d'objet
4. Transport vers zone de dépôt (position 192)
5. Relâchement de l'objet

#### 🔴 Séquence DROITE (192 → 839)
1. Ouverture sécurisée de la pince
2. Déplacement vers zone de prise (position 192)
3. **Saisie intelligente** avec détection d'objet
4. Transport vers zone de dépôt (position 839)
5. Relâchement de l'objet

### 4. Contrôle Manuel

Via le dropdown des servos détectés :
- **Contrôle position** (0-1023)
- **Réglage vitesse** (4-330)
- **Mode séquence aléatoire**

---

## 🛡️ Sécurités Intégrées

### Protection Anti-Overload
- **Détection automatique** des surcharges
- **Reset intelligent** des servos en erreur
- **Limites de couple** configurables
- **Fermeture progressive** pour éviter les blocages

### Gestion d'Erreurs
- **Diagnostic temps réel** des erreurs servo
- **Recovery automatique** en cas de problème
- **Affichage des codes d'erreur** en hexadécimal
- **Messages d'état** informatifs

---

## 📊 Diagnostic et Monitoring

### Informations Disponibles
- **Position actuelle** de chaque servo
- **Charge instantanée** vs limite configurée
- **Température** des moteurs
- **Codes d'erreur** détaillés
- **Pourcentage d'utilisation** du couple

### Bouton "Torque Info"
Affiche en temps réel pour chaque servo :
```
ID 1: Pos=830 Charge=45 Limite=800 Temp=32°C
ID 23: Pos=520 Charge=38 Limite=800 Temp=31°C
Statut: Pince OUVERTE
```

---

## 🔧 Configuration Avancée

### Paramètres de la Pince

```cpp
// Positions critiques
#define GRIPPER_LEFT_CLOSED 950    // Fermeture maximale gauche
#define GRIPPER_RIGHT_CLOSED 380   // Fermeture maximale droite
#define GRIPPER_LEFT_OPEN 724      // Position ouverte gauche
#define GRIPPER_RIGHT_OPEN 620     // Position ouverte droite

// Seuils de sécurité
#define GRIPPER_FORCE_THRESHOLD 20 // Détection d'objet
```

### Communication Série
- **Baudrate** : 1 000 000 bps (standard Dynamixel)
- **Protocole** : Dynamixel v1.0
- **Timeout** : 10ms par commande

---

## 🚧 Améliorations Futures

- [ ] **Capteurs de vision** pour reconnaissance d'objets
- [ ] **Interface web** de monitoring à distance
- [ ] **Apprentissage automatique** des séquences optimales
- [ ] **API REST** pour intégration système
- [ ] **Logs d'activité** avec horodatage
- [ ] **Mode maintenance** avec calibrage automatique

---

## 📝 Notes Techniques

### Problèmes Résolus
- ✅ **Overload servo ID 23** : Protection automatique implémentée
- ✅ **Relâchement intempestif** : Gestion intelligente du couple
- ✅ **Communication instable** : Timeouts et retry optimisés

### Performance
- **Temps de cycle** : ~15-20 secondes par objet
- **Précision positioning** : ±1 unité servo (0.3°)
- **Fiabilité saisie** : >95% avec objets standards

---

## 🤝 Contribution

1. Fork le projet
2. Créer une branche feature (`git checkout -b feature/nouvelle-fonctionnalite`)
3. Commit les modifications (`git commit -m 'Ajout nouvelle fonctionnalité'`)
4. Push vers la branche (`git push origin feature/nouvelle-fonctionnalite`)
5. Ouvrir une Pull Request

---

## 📜 Licence et Usage

**Projet académique** - IUT de Cachan (Université Paris-Saclay)  
Code disponible à des fins éducatives et de démonstration.

---

## 👥 Auteurs

**Rayan** - *Étudiant GE2II, IUT de Cachan (Université Paris-Saclay)*
- Conception et développement du système complet
- Assemblage mécanique avec pièces du kit BIOLOID
- Implémentation logicielle STM32 + LVGL

---

## 🎓 Contexte Académique

**Formation** : DUT Génie Électrique et Informatique Industrielle (GE2II)  
**Établissement** : IUT de Cachan - Université Paris-Saclay  
**Année** : 2024-2025  
**Type** : Projet de fin d'études / Mini-projet spécialisé

### Compétences Développées
- 🔧 **Programmation embarquée** (STM32, C++)
- 🤖 **Robotique** (Dynamixel, BIOLOID)
- 🎨 **Interface graphique** (LVGL)
- ⚡ **Protocoles de communication** (UART, bus série)
- 🛠️ **Intégration système** (hardware/software)

---

## 🏫 Remerciements

- **Équipe pédagogique GE2II** - Encadrement et ressources techniques
- **Laboratoire IUT de Cachan** - Mise à disposition du matériel (BIOLOID, power supply, STM32)
- **Université Paris-Saclay** - Cadre de formation d'excellence

---

*Projet robotique développé dans le cadre de la formation GE2II à l'IUT de Cachan 🎓*

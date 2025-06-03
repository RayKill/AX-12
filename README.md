# AX12 + LVGL + STM32F746G with PlatformIO

Projet PlatformIO pour piloter un **servo Dynamixel AX-12** à l'aide d'un **STM32F746G** (ex: STM32F746G-DISCO) avec une interface **graphique LVGL**.

## 🎯 Objectifs

- Contrôle d’un servo AX-12 via **UART1** à **1 Mbps** (protocole Dynamixel v1.0)
- Interface graphique avec **LVGL** (bouton pour le contrôler)
- Utilisation du framework **STM32Cube** dans PlatformIO
- Prêt à être flashé et étendu avec FreeRTOS ou autres interfaces

---

## 🗂️ Arborescence du projet

ax12_lvgl_project/
├── include/ # Fichiers d'en-tête (.h)
│ ├── ax12.h
│ ├── usart.h
│ └── lv_conf.h
├── lib/
│ └── lvgl/ # Cloné depuis https://github.com/lvgl/lvgl
├── src/
│ ├── main.c
│ ├── ax12.c
│ ├── usart.c
│ ├── lv_port_disp.c
│ └── lv_port_indev.c (optionnel)
├── platformio.ini
└── README.md

## ⚙️ Configuration `platformio.ini`

```ini
[env:disco_f746ng]
platform = ststm32
board = disco_f746ng
framework = stm32cube
upload_protocol = stlink
build_flags = 
    -DLV_CONF_INCLUDE_SIMPLE
    -Iinclude

## ⚙️ A modifier prochainement :

Modifier la resistance et mettre du 1K.
Rajouter une liste déroulante qui va vérifier tout les ax-12a présent sur le bus (boucle qui envoie un par un un ping à un id 0X00 to 0XFD) et les récupérer si présent (réponse de l'ax12).
Permettre de choisir un ax-12a afin de pouvoir le reconfigurer (changer id etc...)


# 2026_ESE_PROJET_TBD

## Procédure de test de la carte

### Tests d'alimentation

La 1ère étape pour tester le pcb est de souder les solder bridges. Une fois les soudures faites, une alimentation de laboratoire est utilisée pour alimenter la carte. Pour ne pas endommager la carte en cas de court circuit, le courant de l'alimentation est limité à 100mA. On peut ensuite venir tester un à un les différents points de test de la carte. Les différents points de test sont relié au 5v, 3v3, sys et GND. Lors des des tests d'alimentation, des led temoigneront du fonctionnement : 

<div align="center">
<img width="753" height="331" alt="image" src="https://github.com/user-attachments/assets/f3d6cfae-2d5c-4d91-8483-8c7ba309c1f6" />
</div>

| Test point alimenté | LED Temoin | LED allumée(s) en conséquence |Explication |
| :--- | :--- | :--- | :--- |
| 3v3 | Led rouge D1| X |X |
| 5v  | Led verte D2 | Led verte D2 orange D3 et rouge D4 | Le 5v alimente le régulateur 3v3 donc la led D2 s'allume. |
| SYS | Led rouge D4| X |X |

### Test du chargeur

Avant d'utiliser la batterie, les premiers essais du chargeur sont réalisés avec une alimentation 3v8 1A. Il faut programmer le chargeur via I2C: 
- Désactiver le timer interne (REG07, bits 5:4 = 00)
- Configurer la tension de sortie  boostée (REG0A)
- Couper la charge (REG03) (CHG_CONFIG = 0->Bit 4)
- Activer la sortie boostée (REG03) (OTG_CONFIG = 1 ->Bit 5)

Une fois tout cette partie validée, la batterie peut être branchée.

### Test du fuel gauge 

---

## Mise à jour OTA (PC → BLE → nRF52840 → UART → STM32G431)

### 1. Contexte et architecture

Le PCB ne donnant pas accès aux broches `NRST` et `BOOT0` de la STM32G431RBT6 depuis le nRF52840 (`nice!nano`), la reprogrammation passe exclusivement par la liaison UART reliant les deux microcontrôleurs :

| Signal | STM32G431RBT6 (`USART3`) | nRF52840 `nice!nano` (`uart0`) | Paramètres |
| :--- | :--- | :--- | :--- |
| TX → RX | `PC10` (`USART3_TX`) | `P0.08` (`UART_RX`, pull-up) | `115200` bauds, `8N1` |
| RX ← TX | `PC11` (`USART3_RX`) | `P0.06` (`UART_TX`) | Sans contrôle de flux |

Le nRF52840 ne disposant pas de contrôleur Wi-Fi intégré, la réception sans fil du firmware depuis le PC s'effectue en **Bluetooth Low Energy (BLE)** via une interface Web Bluetooth locale en glisser-déposer.

```text
PC (Chrome/Edge : stm32_ble_ota.html)
        │  BLE GATT ("Robot_STM32_OTA", paquets MTU jusqu'à 244 o)
        ▼
nRF52840 (Zephyr / NCS v2.9.3 : tampon RAM 112 Ko + contrôle CRC32)
        │  UART 115200 8N1 (P0.06 TX / P0.08 RX)
        ▼
STM32G431 (Bootloader permanent 0x08000000 → Flash App 0x08004000)
```

---

### 2. Cartographie mémoire STM32G431RBT6

La Flash interne (`128 Ko`, `64` pages de `2 Ko`, banque unique) et la SRAM (`32 Ko`) sont partitionnées ainsi :

| Zone | Plage d'adresses | Taille | Rôle |
| :--- | :--- | :--- | :--- |
| **Bootloader permanent** | `0x08000000` – `0x08003FFF` | `16 Ko` (pages 0..7) | Reçoit le firmware sur `USART3`, écrit la Flash App et saute vers l'application |
| **Application Robot** | `0x08004000` – `0x0801FFFF` | `112 Ko` (pages 8..63) | Firmware robot FreeRTOS (`SCB->VTOR = 0x08004000`) avec écoute OTA active |
| **SRAM applicative** | `0x20000000` – `0x20007FEF` | `32 Ko - 16 o` | Pile, heap et variables (`_estack = 0x20007FF0`) |
| **Mailbox OTA SRAM** | `0x20007FF0` – `0x20007FFF` | `16 octets` | Drapeau `0xB00710AD` posé avant `NVIC_SystemReset()` pour forcer l'entrée en bootloader |

---

### 3. Protocole série OTA et mécanisme de mise à jour

- **Format de trame (`Informatics/ota/ota_protocol.h`) :**
  `[0xAA, 0x55, CMD, LEN_LE (2 o), PAYLOAD (0..260 o), CRC32_LE (4 o)]` (CRC-32 IEEE 802.3).
  Réponses STM32 fixes sur `4` octets : `[0xAA, 0x55, RSP, ERR]` (`ACK=0x79`, `NACK=0x1F`, `READY=0x5A`, `REBOOTING=0x55`).
- **Auto-hébergement des mises à jour futures :**
  Chaque firmware applicatif initialise `ota_init(&huart3)` et appelle `ota_process()` dans la boucle FreeRTOS (`Core/Src/app_freertos.c`). Lorsqu'une trame `OTA_CMD_SYNC` (ou `"$$OTA$$"`, ou la commande shell `ota`) arrive :
  1. L'application coupe les moteurs (`motor_stop_all()`), répond `OTA_RSP_REBOOTING`, écrit `0xB00710AD` dans la mailbox SRAM et déclenche `NVIC_SystemReset()`.
  2. Le bootloader démarre, détecte la mailbox, efface `0` et répond `OTA_RSP_READY`. *(Au boot normal sans mailbox, il attend `350 ms` un `SYNC` éventuel avant de sauter vers l'app valide.)*
  3. **`OTA_CMD_BEGIN`** : transmet la taille totale et le CRC32 ; le bootloader efface uniquement les pages applicatives nécessaires (`>= page 8`).
  4. **`OTA_CMD_DATA`** : transfère le `.bin` par blocs de `256` octets max (écriture double-word `64 bits`, retry idempotent jusqu'à `3` tentatives).
  5. **`OTA_CMD_END`** : recalcule le CRC32 sur toute la zone écrite, vérifie le vecteur `MSP`/`Reset_Handler` à `0x08004000`, puis exécute `jump_to_app()`.
- **Cohabitation avec les `printf` :** `_write()` envoie aussi les sorties texte sur `USART3`. Le décodeur côté nRF52840 ignore les octets hors trame `[0xAA, 0x55, ...]` pendant l'OTA et relaie les lignes `[STM32] ...` sur la console USB de la `nice!nano` au repos.

---

### 4. Inventaire des fichiers ajoutés et modifiés

#### Côté STM32 & CMake (`Informatics/`)
- `ota/ota_protocol.h` : constantes, cartographie, CRC32 et format de trames partagés STM32 / nRF52840.
- `bootloader/boot_main.c` & `bootloader/STM32G431xx_BOOT.ld` : bootloader UART3 autonome (`16 Ko`, compilé en `-O2`).
- `app/ota.h` & `app/ota.c` : écoute `USART3` sous interruption, détection `SYNC`/`$$OTA$$`, arrêt moteurs et bascule vers le bootloader.
- `STM32G431xx_FLASH.ld` : décalage Flash de l'application à `0x08004000` (`112 Ko`) et réservation des `16` derniers octets de SRAM.
- `CMakeLists.txt` : ajout des sources OTA, génération automatique de `Informatics.bin` et de la cible `Informatics_Bootloader` (`Informatics_Bootloader.elf` / `.bin`).
- `Core/Src/main.c`, `Core/Src/app_freertos.c`, `Core/Src/stm32g4xx_it.c`, `Core/Src/stm32g4xx_hal_msp.c` : redirection `SCB->VTOR = OTA_APP_ADDR`, diffusion `printf` sur `USART2` + `USART3`, init/polling OTA et callback `USART3`.
- `bsp/uart_console.h`, `bsp/uart_console.c`, `app/shell.c` : routage des callbacks HAL secondaires vers `ota.c` et ajout de la commande shell `ota`.

#### Côté nRF52840 Zephyr (`Informatics/nrf52840/`)
- `app.overlay` : configuration Devicetree de `&uart0` (`stm32-uart`) à `115200 8N1` sur `P0.06` (TX) et `P0.08` (RX).
- `prj.conf` & `CMakeLists.txt` : activation UART par interruption, `ring_buf`, logs USB CDC ACM et stack BLE SoftDevice (`CONFIG_BT=y`, `CONFIG_BT_CTLR=y`, `CONFIG_BT_L2CAP_TX_MTU=247`).
- `../ota/nrf52840_stm32_ota.h` & `../ota/nrf52840_stm32_ota.c` : pilote portable maître OTA UART (`nrf_stm32_ota_flash()`).
- `src/stm32_ota_zephyr.h` & `src/stm32_ota_zephyr.c` : backend Zephyr (ISR UART, `ring_buf` 512 o, mutex OTA).
- `src/stm32_ble_ota.h` & `src/stm32_ble_ota.c` : service GATT BLE (`7b5a0001..0003-3c91-4d2e-9a4b-8f12c0e5d100`, périphérique `Robot_STM32_OTA`), réception en RAM (`112 Ko`), vérification CRC32 + en-tête STM32, et thread dédié de flashage UART avec notifications de progression.
- `src/stm32_fw_image.h` & `src/main.c` : point d'entrée Zephyr, option d'image embarquée pour test autonome, et relais des logs STM32.

#### Côté PC (`Informatics/ota/`)
- `stm32_ble_ota.html` : interface Web Bluetooth (glisser-déposer `.bin`, contrôle de l'adresse de reset `>= 0x08004000`, adaptation automatique au MTU BLE, barres de progression BLE et UART).
- `start_ble_ota_ui.bat` : lance un serveur HTTP local sur `http://127.0.0.1:8090/stm32_ble_ota.html` (requis par Chrome/Edge pour autoriser Web Bluetooth).

---

### 5. Procédure de déploiement et d'utilisation

1. **Première programmation du bootloader STM32 (1 seule fois via ST-Link) :**
   ```powershell
   & "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" -c port=SWD -w Informatics\build\Debug\Informatics_Bootloader.bin 0x08000000 -v -rst
   ```
   *(En cas de flashage ultérieur d'une appli via ST-Link/CubeIDE, ne pas activer « Full chip erase » afin de conserver les 16 premiers Ko du bootloader.)*

2. **Programmation du nRF52840 (`nice!nano`) :**
   Double-appuyer sur Reset et copier `Informatics/nrf52840/build/zephyr/zephyr.uf2` sur le lecteur USB `NICENANO`.

3. **Mise à jour sans fil d'un nouveau firmware STM32 :**
   - Compiler le projet STM32 (`Informatics.bin` généré dans `Informatics/build/Debug/`).
   - Lancer `Informatics/ota/start_ble_ota_ui.bat` et ouvrir la page dans Chrome ou Edge.
   - Cliquer sur **Connecter au nRF52840** (`Robot_STM32_OTA`), glisser-déposer `Informatics.bin`, puis cliquer sur **Flasher la STM32**.

> **Important pour tout futur firmware STM32 :** chaque binaire destiné à l'OTA doit conserver le décalage Flash à `0x08004000`, positionner `SCB->VTOR = 0x08004000UL`, et inclure `ota_init(&huart3)` + `ota_process()` afin de pouvoir être mis à jour à son tour.




#ifndef APP_OTA_H
#define APP_OTA_H

#include <stdbool.h>
#include "stm32g4xx_hal.h"

/**
 * @file ota.h
 * @brief Gestion cote Application de la mise a jour OTA depuis le nRF52840 via UART3.
 *        Ecoute la trame OTA_CMD_SYNC ou "$$OTA$$" sur UART3 et redemarre
 *        proprement vers le bootloader permanent (0x08000000).
 */

/** @brief Initialise l'ecoute OTA en interruption sur l'UART reliee au nRF52840 (ex: &huart3) */
bool ota_init(UART_HandleTypeDef *huart);

/** @brief Traite les octets recus sur UART3 (a appeler periodiquement, ex: dans RobotTask) */
void ota_process(void);

/** @brief Force le redemarrage immediat vers le bootloader UART3 */
void ota_enter_bootloader(void);

#endif /* APP_OTA_H */

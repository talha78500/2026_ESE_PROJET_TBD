#ifndef BSP_UART_CONSOLE_H
#define BSP_UART_CONSOLE_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "stm32g4xx_hal.h"

/**
 * @file uart_console.h
 * @brief Couche BSP pour la reception de la console UART (ex: USART2 / VCP ST-Link).
 *        Reception octet par octet par interruption, stockee dans un buffer circulaire.
 *        L'emission reste assuree par printf() (_write dans main.c).
 *
 *        PREREQUIS CubeMX : activer "USART2 global interrupt" dans NVIC Settings.
 */

#define UART_CONSOLE_RX_BUF_SIZE   128   // Doit etre une puissance de 2

/* Demarre la reception par interruption sur l'UART donnee */
bool uart_console_init(UART_HandleTypeDef *huart);

/* Lit un octet du buffer. Retourne false s'il est vide (non bloquant) */
bool uart_console_read_byte(uint8_t *byte);

/* Nombre d'octets perdus par debordement du buffer (diagnostic) */
uint32_t uart_console_get_overflow_count(void);

/* Hooks optionnels (weak) pour permettre a d'autres modules d'utiliser HAL_UART_Receive_IT */
void uart_console_ext_rx_callback(UART_HandleTypeDef *huart);
void uart_console_ext_err_callback(UART_HandleTypeDef *huart);

#endif /* BSP_UART_CONSOLE_H */

#ifndef NRF52840_STM32_OTA_H
#define NRF52840_STM32_OTA_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "ota_protocol.h"

/**
 * @file nrf52840_stm32_ota.h
 * @brief Bibliotheque portable cote nRF52840 (Zephyr / nRF Connect SDK / nRF5 SDK)
 *        pour programmer la STM32G431 via l'UART (115200 8N1).
 *
 *        Utilisation : copier ota_protocol.h, nrf52840_stm32_ota.h et
 *        nrf52840_stm32_ota.c dans le projet nRF52840, remplir les 4 callbacks
 *        de transport UART/temps dans nrf_ota_io_t, puis appeler
 *        nrf_stm32_ota_flash().
 */

typedef struct {
    /** Envoie len octets sur la liaison UART vers la STM32. Retourne true si OK. */
    bool     (*uart_tx)(const uint8_t *data, size_t len, void *ctx);
    /** Lit 1 octet non bloquant ou avec court timeout. Retourne true si un octet a ete lu. */
    bool     (*uart_rx_byte)(uint8_t *byte, uint32_t timeout_ms, void *ctx);
    /** Vide d'eventuels octets residuels (ex: printf) dans le buffer RX avant un envoi. Peut etre NULL. */
    void     (*uart_flush_rx)(void *ctx);
    /** Horloge monotone en millisecondes. */
    uint32_t (*get_ms)(void *ctx);
    /** Pause en millisecondes. */
    void     (*delay_ms)(uint32_t ms, void *ctx);
    void    *ctx;
} nrf_ota_io_t;

typedef enum {
    NRF_OTA_OK = 0,
    NRF_OTA_ERR_PARAM,
    NRF_OTA_ERR_SYNC,
    NRF_OTA_ERR_BEGIN,
    NRF_OTA_ERR_DATA,
    NRF_OTA_ERR_END,
    NRF_OTA_ERR_PROTO
} nrf_ota_status_t;

typedef void (*nrf_ota_progress_cb)(uint32_t sent_bytes, uint32_t total_bytes, void *user_ctx);

/**
 * @brief Flashe un binaire STM32 (.bin compile a 0x08004000) depuis le nRF52840.
 *        Fonctionne aussi bien si l'application STM32 tourne deja (elle reboote
 *        automatiquement dans le bootloader) que si la STM32 est deja en bootloader.
 */
nrf_ota_status_t nrf_stm32_ota_flash(const nrf_ota_io_t *io,
                                     const uint8_t *fw_bin,
                                     uint32_t fw_size,
                                     nrf_ota_progress_cb progress_cb,
                                     void *user_ctx);

#endif /* NRF52840_STM32_OTA_H */

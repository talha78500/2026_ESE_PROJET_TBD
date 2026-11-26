#ifndef OTA_PROTOCOL_H
#define OTA_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * @file ota_protocol.h
 * @brief Protocole et cartographie memoire partages entre le Bootloader STM32,
 *        l'Application STM32 et le nRF52840 (sur USART3 PC10/PC11 a 115200 8N1).
 *
 *        Ce fichier est portable (C standard uniquement) et peut etre copie
 *        tel quel dans le projet du nRF52840.
 */

/* Cartographie Flash STM32G431RBT6 (128 Ko = 64 pages de 2 Ko, single bank) */
#define OTA_BOOT_ADDR             0x08000000UL
#define OTA_BOOT_SIZE             (16U * 1024U)       /* Pages 0..7   */
#define OTA_APP_ADDR              0x08004000UL
#define OTA_APP_MAX_SIZE          (112U * 1024U)      /* Pages 8..63  */
#define OTA_APP_END_ADDR          (OTA_APP_ADDR + OTA_APP_MAX_SIZE)
#define OTA_FLASH_PAGE_SIZE       2048U
#define OTA_APP_FIRST_PAGE        8U
#define OTA_APP_MAX_PAGES         56U

/* Mailbox partagee dans les 16 derniers octets de la SRAM (hors pile/BSS) */
#define OTA_BOOT_MAILBOX_ADDR     0x20007FF0UL
#define OTA_MAGIC_ENTER_BOOT      0xB00710ADUL

/* Delai d'ecoute au demarrage normal avant de lancer l'application (ms) */
#define OTA_BOOT_WAIT_MS          350U

/* Parametres protocole UART3 */
#define OTA_UART_BAUDRATE         115200U
#define OTA_SOF0                  0xAAU
#define OTA_SOF1                  0x55U
#define OTA_DATA_CHUNK_MAX        256U                /* Multiple de 8 octets */
#define OTA_MAX_PAYLOAD           (4U + OTA_DATA_CHUNK_MAX)

/* Commande texte alternative (facultative) pour entrer en bootloader depuis UART3 */
#define OTA_ASCII_TRIGGER         "$$OTA$$"

/* Codes commande (nRF52840 -> STM32) */
typedef enum {
    OTA_CMD_SYNC  = 0x01,  /* Demande d'entree / verification mode bootloader (len = 0)        */
    OTA_CMD_BEGIN = 0x02,  /* Demarre OTA : uint32_t size + uint32_t crc32 (len = 8)           */
    OTA_CMD_DATA  = 0x03,  /* Ecrit bloc  : uint32_t offset + uint8_t data[1..256]             */
    OTA_CMD_END   = 0x04,  /* Verifie CRC32 global + table vecteurs puis lance l'app (len = 0) */
    OTA_CMD_ABORT = 0x05   /* Annule la session OTA en cours (len = 0)                         */
} ota_cmd_t;

/* Reponses STM32 -> nRF52840 : trame fixe 4 octets [0xAA, 0x55, RSP, ERR] */
typedef enum {
    OTA_RSP_ACK       = 0x79,
    OTA_RSP_NACK      = 0x1F,
    OTA_RSP_READY     = 0x5A,  /* Bootloader pret a recevoir OTA_CMD_BEGIN */
    OTA_RSP_REBOOTING = 0x55   /* App recu SYNC -> reboot vers bootloader  */
} ota_rsp_t;

typedef enum {
    OTA_ERR_NONE   = 0x00,
    OTA_ERR_CRC    = 0x01,
    OTA_ERR_SIZE   = 0x02,
    OTA_ERR_ERASE  = 0x03,
    OTA_ERR_WRITE  = 0x04,
    OTA_ERR_OFFSET = 0x05,
    OTA_ERR_VERIFY = 0x06,
    OTA_ERR_CMD    = 0x07,
    OTA_ERR_STATE  = 0x08
} ota_err_t;

/** @brief Mise a jour incrementale CRC-32 (polynome IEEE 802.3, identique a zlib.crc32) */
static inline uint32_t ota_crc32_update(uint32_t crc, const uint8_t *data, size_t len)
{
    crc = ~crc;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8U; b++) {
            uint32_t mask = (uint32_t)(-(int32_t)(crc & 1U));
            crc = (crc >> 1) ^ (0xEDB88320UL & mask);
        }
    }
    return ~crc;
}

static inline uint32_t ota_crc32(const uint8_t *data, size_t len)
{
    return ota_crc32_update(0U, data, len);
}

static inline uint16_t ota_get_u16_le(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static inline uint32_t ota_get_u32_le(const uint8_t *p)
{
    return (uint32_t)p[0] |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static inline void ota_put_u16_le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
}

static inline void ota_put_u32_le(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8) & 0xFFU);
    p[2] = (uint8_t)((v >> 16) & 0xFFU);
    p[3] = (uint8_t)((v >> 24) & 0xFFU);
}

#endif /* OTA_PROTOCOL_H */

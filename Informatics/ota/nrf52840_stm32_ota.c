#include "nrf52840_stm32_ota.h"
#include <string.h>

#define NRF_OTA_SYNC_ATTEMPTS     12U
#define NRF_OTA_DATA_RETRIES      3U
#define NRF_OTA_RSP_TIMEOUT_MS    500U
#define NRF_OTA_ERASE_TIMEOUT_MS  4000U
#define NRF_OTA_END_TIMEOUT_MS    5000U

static bool send_frame(const nrf_ota_io_t *io, uint8_t cmd, const uint8_t *payload, uint16_t len)
{
    uint8_t tx[2U + 3U + OTA_MAX_PAYLOAD + 4U];
    if (len > OTA_MAX_PAYLOAD) return false;

    tx[0] = OTA_SOF0;
    tx[1] = OTA_SOF1;
    tx[2] = cmd;
    ota_put_u16_le(&tx[3], len);
    if (len > 0U && payload != NULL) {
        memcpy(&tx[5], payload, len);
    }

    uint32_t crc = ota_crc32(&tx[2], (size_t)(3U + len));
    ota_put_u32_le(&tx[5U + len], crc);

    return io->uart_tx(tx, (size_t)(2U + 3U + len + 4U), io->ctx);
}

/* Ignore d'eventuels octets printf avant [0xAA, 0x55, RSP, ERR] */
static bool wait_rsp(const nrf_ota_io_t *io, uint32_t timeout_ms, uint8_t *rsp_out, uint8_t *err_out)
{
    uint32_t start = io->get_ms(io->ctx);
    uint8_t  prev  = 0U;

    while ((io->get_ms(io->ctx) - start) < timeout_ms) {
        uint8_t b = 0U;
        if (!io->uart_rx_byte(&b, 20U, io->ctx)) {
            continue;
        }
        if (prev == OTA_SOF0 && b == OTA_SOF1) {
            uint8_t rest[2] = {0};
            if (io->uart_rx_byte(&rest[0], 50U, io->ctx) &&
                io->uart_rx_byte(&rest[1], 50U, io->ctx)) {
                *rsp_out = rest[0];
                *err_out = rest[1];
                return true;
            }
        }
        prev = b;
    }
    return false;
}

static bool sync_bootloader(const nrf_ota_io_t *io)
{
    bool in_bootloader = false;

    for (uint32_t attempt = 0U; attempt < NRF_OTA_SYNC_ATTEMPTS; attempt++) {
        if (io->uart_flush_rx != NULL) {
            io->uart_flush_rx(io->ctx);
        }
        if (!send_frame(io, OTA_CMD_SYNC, NULL, 0U)) {
            return false;
        }

        uint8_t rsp = 0U, err = 0U;
        if (wait_rsp(io, 250U, &rsp, &err)) {
            if (rsp == OTA_RSP_REBOOTING) {
                /* L'app vient de lancer NVIC_SystemReset() : attendre READY du bootloader */
                if (wait_rsp(io, 500U, &rsp, &err) &&
                    (rsp == OTA_RSP_READY || rsp == OTA_RSP_ACK)) {
                    in_bootloader = true;
                    break;
                }
            } else if (rsp == OTA_RSP_READY || rsp == OTA_RSP_ACK) {
                in_bootloader = true;
                break;
            }
        }
        io->delay_ms(50U, io->ctx);
    }

    if (!in_bootloader) return false;

    /* Purge d'eventuelles reponses doubles READY/ACK puis verification d'un SYNC propre */
    io->delay_ms(20U, io->ctx);
    if (io->uart_flush_rx != NULL) {
        io->uart_flush_rx(io->ctx);
    }
    uint8_t rsp = 0U, err = 0U;
    return (send_frame(io, OTA_CMD_SYNC, NULL, 0U) &&
            wait_rsp(io, NRF_OTA_RSP_TIMEOUT_MS, &rsp, &err) &&
            rsp == OTA_RSP_ACK);
}

nrf_ota_status_t nrf_stm32_ota_flash(const nrf_ota_io_t *io,
                                     const uint8_t *fw_bin,
                                     uint32_t fw_size,
                                     nrf_ota_progress_cb progress_cb,
                                     void *user_ctx)
{
    if (io == NULL || io->uart_tx == NULL || io->uart_rx_byte == NULL ||
        io->get_ms == NULL || io->delay_ms == NULL ||
        fw_bin == NULL || fw_size == 0U || fw_size > OTA_APP_MAX_SIZE) {
        return NRF_OTA_ERR_PARAM;
    }

    /* 1. Bascule l'application en mode bootloader ou synchronise un bootloader deja actif */
    if (!sync_bootloader(io)) {
        return NRF_OTA_ERR_SYNC;
    }

    /* 2. Envoi taille + CRC32 total -> le bootloader efface les pages Flash necessaires */
    uint8_t  begin_pl[8];
    uint32_t total_crc = ota_crc32(fw_bin, fw_size);
    ota_put_u32_le(&begin_pl[0], fw_size);
    ota_put_u32_le(&begin_pl[4], total_crc);

    uint8_t rsp = 0U, err = 0U;
    if (!send_frame(io, OTA_CMD_BEGIN, begin_pl, sizeof(begin_pl)) ||
        !wait_rsp(io, NRF_OTA_ERASE_TIMEOUT_MS, &rsp, &err) ||
        rsp != OTA_RSP_ACK) {
        return NRF_OTA_ERR_BEGIN;
    }

    /* 3. Envoi des blocs de 256 octets max avec retry automatique */
    uint32_t offset = 0U;
    uint8_t  data_pl[4U + OTA_DATA_CHUNK_MAX];

    while (offset < fw_size) {
        uint32_t rem       = fw_size - offset;
        uint16_t chunk_len = (rem > OTA_DATA_CHUNK_MAX) ? OTA_DATA_CHUNK_MAX : (uint16_t)rem;

        ota_put_u32_le(&data_pl[0], offset);
        memcpy(&data_pl[4], &fw_bin[offset], chunk_len);

        bool chunk_ok = false;
        for (uint32_t tries = 0U; tries < NRF_OTA_DATA_RETRIES; tries++) {
            if (send_frame(io, OTA_CMD_DATA, data_pl, (uint16_t)(4U + chunk_len)) &&
                wait_rsp(io, NRF_OTA_RSP_TIMEOUT_MS, &rsp, &err) &&
                rsp == OTA_RSP_ACK) {
                chunk_ok = true;
                break;
            }
            io->delay_ms(20U, io->ctx);
        }

        if (!chunk_ok) {
            (void)send_frame(io, OTA_CMD_ABORT, NULL, 0U);
            return NRF_OTA_ERR_DATA;
        }

        offset += chunk_len;
        if (progress_cb != NULL) {
            progress_cb(offset, fw_size, user_ctx);
        }
    }

    /* 4. Verification CRC32 finale en Flash + saut automatique vers la nouvelle application */
    if (!send_frame(io, OTA_CMD_END, NULL, 0U) ||
        !wait_rsp(io, NRF_OTA_END_TIMEOUT_MS, &rsp, &err) ||
        rsp != OTA_RSP_ACK) {
        return NRF_OTA_ERR_END;
    }

    return NRF_OTA_OK;
}

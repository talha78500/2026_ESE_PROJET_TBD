#include "stm32g4xx_hal.h"
#include "ota_protocol.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/**
 * @file boot_main.c
 * @brief Bootloader permanent STM32G431RBT6 (0x08000000 .. 0x08003FFF, 16 Ko).
 *        Permet au nRF52840 de reprogrammer l'application (a partir de 0x08004000)
 *        uniquement via USART3 (PC10 TX / PC11 RX, 115200 8N1) sans broches RST/BOOT.
 */

static UART_HandleTypeDef huart3;

typedef struct {
    bool     active;
    uint32_t expected_size;
    uint32_t expected_crc32;
    uint32_t written_offset;
} ota_session_t;

void Error_Handler(void)
{
    __disable_irq();
    while (1) {
    }
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

static void boot_uart3_init(void)
{
    RCC_PeriphCLKInitTypeDef periph_clk = {0};
    periph_clk.PeriphClockSelection = RCC_PERIPHCLK_USART3;
    periph_clk.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&periph_clk) != HAL_OK) {
        Error_Handler();
    }

    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_10 | GPIO_PIN_11;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP; // Evite le bruit si nRF52840 eteint au demarrage
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOC, &gpio);

    huart3.Instance                    = USART3;
    huart3.Init.BaudRate               = OTA_UART_BAUDRATE;
    huart3.Init.WordLength             = UART_WORDLENGTH_8B;
    huart3.Init.StopBits               = UART_STOPBITS_1;
    huart3.Init.Parity                 = UART_PARITY_NONE;
    huart3.Init.Mode                   = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl              = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling           = UART_OVERSAMPLING_16;
    huart3.Init.OneBitSampling         = UART_ONE_BIT_SAMPLE_DISABLE;
    huart3.Init.ClockPrescaler         = UART_PRESCALER_DIV1;
    huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart3) != HAL_OK) {
        Error_Handler();
    }
    HAL_UARTEx_DisableFifoMode(&huart3);
}

static bool boot_uart_rx_byte(uint8_t *out, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    for (;;) {
        if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_ORE | UART_FLAG_FE | UART_FLAG_NE | UART_FLAG_PE)) {
            __HAL_UART_CLEAR_FLAG(&huart3, UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_PEF);
        }
        if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE)) {
            *out = (uint8_t)(USART3->RDR & 0xFFU);
            return true;
        }
        if (timeout_ms == 0U || (HAL_GetTick() - start) >= timeout_ms) {
            return false;
        }
    }
}

static bool boot_uart_rx_exact(uint8_t *buf, size_t len, uint32_t byte_timeout_ms)
{
    for (size_t i = 0; i < len; i++) {
        if (!boot_uart_rx_byte(&buf[i], byte_timeout_ms)) {
            return false;
        }
    }
    return true;
}

static void boot_send_rsp(ota_rsp_t rsp, ota_err_t err)
{
    uint8_t pkt[4] = { OTA_SOF0, OTA_SOF1, (uint8_t)rsp, (uint8_t)err };
    HAL_UART_Transmit(&huart3, pkt, sizeof(pkt), 100);
}

static bool is_app_valid(void)
{
    uint32_t msp = *(volatile uint32_t *)OTA_APP_ADDR;
    uint32_t rst = *(volatile uint32_t *)(OTA_APP_ADDR + 4U);

    if ((msp & 0x2FFE0000UL) != 0x20000000UL) return false;
    if (msp < 0x20000400UL || msp > 0x20008000UL) return false;
    if (rst < OTA_APP_ADDR || rst >= OTA_APP_END_ADDR) return false;
    if ((rst & 1UL) == 0UL) return false;
    return true;
}

static void jump_to_app(void)
{
    if (!is_app_valid()) return;

    uint32_t msp = *(volatile uint32_t *)OTA_APP_ADDR;
    uint32_t rst = *(volatile uint32_t *)(OTA_APP_ADDR + 4U);

    __disable_irq();

    while (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TC) == RESET) {
    }
    HAL_UART_DeInit(&huart3);
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10 | GPIO_PIN_11);
    __HAL_RCC_USART3_CLK_DISABLE();
    HAL_RCC_DeInit();
    HAL_DeInit();

    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;
    SCB->ICSR     = SCB_ICSR_PENDSTCLR_Msk;

    for (uint32_t i = 0; i < 8U; i++) {
        NVIC->ICER[i] = 0xFFFFFFFFUL;
        NVIC->ICPR[i] = 0xFFFFFFFFUL;
    }

    SCB->VTOR = OTA_APP_ADDR;
    __set_CONTROL(0);
    __set_MSP(msp);
    __DSB();
    __ISB();
    __enable_irq();

    ((void (*)(void))rst)();
}

static bool flash_erase_app(uint32_t image_size)
{
    if (image_size == 0U || image_size > OTA_APP_MAX_SIZE) {
        return false;
    }

    uint32_t nb_pages = (image_size + OTA_FLASH_PAGE_SIZE - 1U) / OTA_FLASH_PAGE_SIZE;
    if (nb_pages > OTA_APP_MAX_PAGES) {
        return false;
    }

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

    FLASH_EraseInitTypeDef erase = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .Banks     = FLASH_BANK_1,
        .Page      = OTA_APP_FIRST_PAGE,
        .NbPages   = nb_pages
    };
    uint32_t page_error = 0;
    HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&erase, &page_error);

    HAL_FLASH_Lock();
    return (st == HAL_OK);
}

static bool flash_write_chunk(uint32_t offset, const uint8_t *data, uint16_t len)
{
    if (len == 0U || len > OTA_DATA_CHUNK_MAX) return false;
    if ((offset % 8U) != 0U) return false;
    if ((offset + len) > OTA_APP_MAX_SIZE) return false;

    uint8_t aligned[OTA_DATA_CHUNK_MAX] __attribute__((aligned(8)));
    memcpy(aligned, data, len);

    uint16_t padded_len = len;
    while ((padded_len % 8U) != 0U) {
        aligned[padded_len++] = 0xFFU;
    }

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

    for (uint16_t i = 0; i < padded_len; i += 8U) {
        uint64_t dword = 0;
        memcpy(&dword, &aligned[i], 8U);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, OTA_APP_ADDR + offset + i, dword) != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
    }

    HAL_FLASH_Lock();
    return true;
}

static bool wait_startup_sync(uint32_t wait_ms)
{
    uint32_t start = HAL_GetTick();
    uint8_t  prev  = 0;
    uint8_t  ascii_idx = 0;
    const char *trigger = OTA_ASCII_TRIGGER;
    size_t trigger_len  = strlen(trigger);

    while ((HAL_GetTick() - start) < wait_ms) {
        uint8_t b = 0;
        if (!boot_uart_rx_byte(&b, 10U)) {
            continue;
        }

        /* 1. Detection trigger ASCII ("$$OTA$$") */
        if (b == (uint8_t)trigger[ascii_idx]) {
            ascii_idx++;
            if (ascii_idx == trigger_len) {
                return true;
            }
        } else {
            ascii_idx = (b == (uint8_t)trigger[0]) ? 1U : 0U;
        }

        /* 2. Detection debut de trame binaire [0xAA, 0x55, OTA_CMD_SYNC, ...] */
        if (prev == OTA_SOF0 && b == OTA_SOF1) {
            uint8_t hdr[3] = {0};
            uint8_t crc_buf[4] = {0};
            if (boot_uart_rx_exact(hdr, 3U, 50U) && boot_uart_rx_exact(crc_buf, 4U, 50U)) {
                uint16_t len = ota_get_u16_le(&hdr[1]);
                if (hdr[0] == OTA_CMD_SYNC && len == 0U && ota_crc32(hdr, 3U) == ota_get_u32_le(crc_buf)) {
                    boot_send_rsp(OTA_RSP_ACK, OTA_ERR_NONE);
                    return true;
                }
            }
        }
        prev = b;
    }

    return false;
}

static void handle_frame(ota_session_t *sess, uint8_t cmd, const uint8_t *payload, uint16_t len)
{
    switch (cmd) {
    case OTA_CMD_SYNC:
        boot_send_rsp(OTA_RSP_ACK, OTA_ERR_NONE);
        break;

    case OTA_CMD_BEGIN: {
        if (len != 8U) {
            boot_send_rsp(OTA_RSP_NACK, OTA_ERR_SIZE);
            break;
        }
        uint32_t img_size = ota_get_u32_le(&payload[0]);
        uint32_t img_crc  = ota_get_u32_le(&payload[4]);
        if (img_size == 0U || img_size > OTA_APP_MAX_SIZE) {
            boot_send_rsp(OTA_RSP_NACK, OTA_ERR_SIZE);
            break;
        }
        if (!flash_erase_app(img_size)) {
            boot_send_rsp(OTA_RSP_NACK, OTA_ERR_ERASE);
            break;
        }
        sess->active         = true;
        sess->expected_size  = img_size;
        sess->expected_crc32 = img_crc;
        sess->written_offset = 0U;
        boot_send_rsp(OTA_RSP_ACK, OTA_ERR_NONE);
        break;
    }

    case OTA_CMD_DATA: {
        if (!sess->active) {
            boot_send_rsp(OTA_RSP_NACK, OTA_ERR_STATE);
            break;
        }
        if (len <= 4U || (len - 4U) > OTA_DATA_CHUNK_MAX) {
            boot_send_rsp(OTA_RSP_NACK, OTA_ERR_SIZE);
            break;
        }
        uint32_t offset    = ota_get_u32_le(&payload[0]);
        uint16_t data_len  = (uint16_t)(len - 4U);
        const uint8_t *dat = &payload[4];

        if ((offset + data_len) > sess->expected_size) {
            boot_send_rsp(OTA_RSP_NACK, OTA_ERR_SIZE);
            break;
        }

        /* Retry idempotent si le nRF52840 n'a pas recu le dernier ACK */
        if (offset < sess->written_offset && (offset + data_len) <= sess->written_offset) {
            if (memcmp((const void *)(OTA_APP_ADDR + offset), dat, data_len) == 0) {
                boot_send_rsp(OTA_RSP_ACK, OTA_ERR_NONE);
            } else {
                boot_send_rsp(OTA_RSP_NACK, OTA_ERR_OFFSET);
            }
            break;
        }

        if (offset != sess->written_offset) {
            boot_send_rsp(OTA_RSP_NACK, OTA_ERR_OFFSET);
            break;
        }

        if (!flash_write_chunk(offset, dat, data_len)) {
            boot_send_rsp(OTA_RSP_NACK, OTA_ERR_WRITE);
            break;
        }

        sess->written_offset += data_len;
        boot_send_rsp(OTA_RSP_ACK, OTA_ERR_NONE);
        break;
    }

    case OTA_CMD_END: {
        if (!sess->active || sess->written_offset != sess->expected_size) {
            boot_send_rsp(OTA_RSP_NACK, OTA_ERR_STATE);
            break;
        }

        uint32_t actual_crc = ota_crc32((const uint8_t *)OTA_APP_ADDR, sess->expected_size);
        if (actual_crc != sess->expected_crc32 || !is_app_valid()) {
            sess->active = false;
            boot_send_rsp(OTA_RSP_NACK, OTA_ERR_VERIFY);
            break;
        }

        sess->active = false;
        boot_send_rsp(OTA_RSP_ACK, OTA_ERR_NONE);
        jump_to_app();
        break;
    }

    case OTA_CMD_ABORT:
        sess->active         = false;
        sess->written_offset = 0U;
        boot_send_rsp(OTA_RSP_ACK, OTA_ERR_NONE);
        break;

    default:
        boot_send_rsp(OTA_RSP_NACK, OTA_ERR_CMD);
        break;
    }
}

static void boot_ota_loop(void)
{
    ota_session_t sess = {0};
    uint8_t frame_hdr_and_payload[3U + OTA_MAX_PAYLOAD];
    uint8_t crc_buf[4];

    for (;;) {
        uint8_t b = 0;
        if (!boot_uart_rx_byte(&b, 1000U) || b != OTA_SOF0) {
            continue;
        }
        if (!boot_uart_rx_byte(&b, 50U) || b != OTA_SOF1) {
            continue;
        }
        if (!boot_uart_rx_exact(frame_hdr_and_payload, 3U, 50U)) {
            continue;
        }

        uint8_t  cmd = frame_hdr_and_payload[0];
        uint16_t len = ota_get_u16_le(&frame_hdr_and_payload[1]);
        if (len > OTA_MAX_PAYLOAD) {
            boot_send_rsp(OTA_RSP_NACK, OTA_ERR_SIZE);
            continue;
        }

        if (len > 0U && !boot_uart_rx_exact(&frame_hdr_and_payload[3], len, 100U)) {
            boot_send_rsp(OTA_RSP_NACK, OTA_ERR_SIZE);
            continue;
        }

        if (!boot_uart_rx_exact(crc_buf, 4U, 50U)) {
            boot_send_rsp(OTA_RSP_NACK, OTA_ERR_CRC);
            continue;
        }

        uint32_t rx_crc   = ota_get_u32_le(crc_buf);
        uint32_t calc_crc = ota_crc32(frame_hdr_and_payload, (size_t)(3U + len));
        if (rx_crc != calc_crc) {
            boot_send_rsp(OTA_RSP_NACK, OTA_ERR_CRC);
            continue;
        }

        handle_frame(&sess, cmd, &frame_hdr_and_payload[3], len);
    }
}

int main(void)
{
    HAL_Init();
    boot_uart3_init();

    volatile uint32_t *mailbox = (volatile uint32_t *)OTA_BOOT_MAILBOX_ADDR;
    bool forced_ota = (*mailbox == OTA_MAGIC_ENTER_BOOT);
    *mailbox = 0UL;

    if (!forced_ota && is_app_valid()) {
        if (!wait_startup_sync(OTA_BOOT_WAIT_MS)) {
            jump_to_app();
        }
    }

    /* Mode OTA actif : avertit le nRF52840 et attend les paquets */
    boot_send_rsp(OTA_RSP_READY, OTA_ERR_NONE);
    boot_ota_loop();

    return 0;
}

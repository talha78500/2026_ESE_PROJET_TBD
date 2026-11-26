#include "ota.h"
#include "ota_protocol.h"
#include "uart_console.h"
#include "motor.h"
#include <stdio.h>
#include <string.h>

#define OTA_RX_BUF_SIZE  128U
#define OTA_RX_MASK      (OTA_RX_BUF_SIZE - 1U)

static UART_HandleTypeDef *s_huart;
static uint8_t             s_rx_byte;
static volatile uint8_t    s_rx_buf[OTA_RX_BUF_SIZE];
static volatile uint16_t   s_head;
static volatile uint16_t   s_tail;

static bool rx_pop(uint8_t *out)
{
    if (s_tail == s_head) return false;
    *out   = s_rx_buf[s_tail];
    s_tail = (uint16_t)((s_tail + 1U) & OTA_RX_MASK);
    return true;
}

bool ota_init(UART_HandleTypeDef *huart)
{
    if (huart == NULL) return false;

    s_huart = huart;
    s_head  = 0U;
    s_tail  = 0U;

    if (huart->Instance == USART3) {
        HAL_NVIC_SetPriority(USART3_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
    }

    return (HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1) == HAL_OK);
}

void ota_enter_bootloader(void)
{
    printf("\r\n[OTA] Passage en mode bootloader sur UART3...\r\n");
    motor_stop_all();

    if (s_huart != NULL) {
        uint8_t rsp[4] = { OTA_SOF0, OTA_SOF1, OTA_RSP_REBOOTING, OTA_ERR_NONE };
        HAL_UART_Transmit(s_huart, rsp, sizeof(rsp), 50);
    }

    HAL_Delay(30);

    __disable_irq();
    *(volatile uint32_t *)OTA_BOOT_MAILBOX_ADDR = OTA_MAGIC_ENTER_BOOT;
    __DSB();
    NVIC_SystemReset();
}

void ota_process(void)
{
    static uint8_t win[9];
    static uint8_t win_len   = 0U;
    static uint8_t ascii_idx = 0U;
    const char    *trigger   = OTA_ASCII_TRIGGER;
    const size_t   trig_len  = strlen(trigger);
    uint8_t        b         = 0U;

    while (rx_pop(&b)) {
        /* 1. Trigger texte facultatif ("$$OTA$$") */
        if (b == (uint8_t)trigger[ascii_idx]) {
            ascii_idx++;
            if (ascii_idx == trig_len) {
                ascii_idx = 0U;
                ota_enter_bootloader();
                return;
            }
        } else {
            ascii_idx = (b == (uint8_t)trigger[0]) ? 1U : 0U;
        }

        /* 2. Fenetre glissante 9 octets pour la trame OTA_CMD_SYNC */
        if (win_len < 9U) {
            win[win_len++] = b;
        } else {
            memmove(win, &win[1], 8U);
            win[8] = b;
        }

        if (win_len == 9U &&
            win[0] == OTA_SOF0 &&
            win[1] == OTA_SOF1 &&
            win[2] == OTA_CMD_SYNC &&
            win[3] == 0U &&
            win[4] == 0U) {
            uint32_t rx_crc   = ota_get_u32_le(&win[5]);
            uint32_t calc_crc = ota_crc32(&win[2], 3U);
            if (rx_crc == calc_crc) {
                win_len = 0U;
                ota_enter_bootloader();
                return;
            }
        }
    }
}

void uart_console_ext_rx_callback(UART_HandleTypeDef *huart)
{
    if (s_huart == NULL || huart != s_huart) return;

    uint16_t next = (uint16_t)((s_head + 1U) & OTA_RX_MASK);
    if (next != s_tail) {
        s_rx_buf[s_head] = s_rx_byte;
        s_head = next;
    }

    HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1);
}

void uart_console_ext_err_callback(UART_HandleTypeDef *huart)
{
    if (s_huart == NULL || huart != s_huart) return;

    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);

    HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1);
}

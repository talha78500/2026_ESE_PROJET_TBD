#include "uart_console.h"

#define RX_MASK  (UART_CONSOLE_RX_BUF_SIZE - 1U)

static UART_HandleTypeDef *s_huart;
static uint8_t             s_rx_byte;                         // Octet en cours de reception (IT)
static volatile uint8_t    s_buf[UART_CONSOLE_RX_BUF_SIZE];
static volatile uint16_t   s_head;                            // Ecrit par l'ISR
static volatile uint16_t   s_tail;                            // Ecrit par le programme principal
static volatile uint32_t   s_overflow;

bool uart_console_init(UART_HandleTypeDef *huart)
{
    if (huart == NULL) return false;

    s_huart    = huart;
    s_head     = 0;
    s_tail     = 0;
    s_overflow = 0;

    return (HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1) == HAL_OK);
}

bool uart_console_read_byte(uint8_t *byte)
{
    if (byte == NULL || s_tail == s_head) return false;

    *byte  = s_buf[s_tail];
    s_tail = (uint16_t)((s_tail + 1U) & RX_MASK);
    return true;
}

uint32_t uart_console_get_overflow_count(void)
{
    return s_overflow;
}

/* ---- Callbacks HAL (appeles depuis l'interruption) ---- */

__weak void uart_console_ext_rx_callback(UART_HandleTypeDef *huart)
{
    (void)huart;
}

__weak void uart_console_ext_err_callback(UART_HandleTypeDef *huart)
{
    (void)huart;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != s_huart) {
        uart_console_ext_rx_callback(huart);
        return;
    }

    uint16_t next = (uint16_t)((s_head + 1U) & RX_MASK);
    if (next != s_tail) {
        s_buf[s_head] = s_rx_byte;
        s_head = next;
    } else {
        s_overflow++;                       // Buffer plein : l'octet est perdu
    }

    HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1);   // Relance la reception
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart != s_huart) {
        uart_console_ext_err_callback(huart);
        return;
    }

    // Overrun / framing / bruit : on efface les flags et on relance la reception
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);

    HAL_UART_Receive_IT(s_huart, &s_rx_byte, 1);
}

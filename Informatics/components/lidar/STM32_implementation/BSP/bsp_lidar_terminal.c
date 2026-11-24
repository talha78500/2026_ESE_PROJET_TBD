#include "bsp_lidar_terminal.h"
#include "stm32l4xx_hal.h"

static UART_HandleTypeDef terminal_uart;

int bsp_lidar_terminal_init(void)
{
    /* PA2 sends reports and PA3 receives keys from the ST-LINK virtual COM port. */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USART2_CONFIG(RCC_USART2CLKSOURCE_PCLK1);
    __HAL_RCC_USART2_CLK_ENABLE();

    GPIO_InitTypeDef pins = {
        .Pin = GPIO_PIN_2 | GPIO_PIN_3,
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_LOW,
        .Alternate = GPIO_AF7_USART2
    };
    HAL_GPIO_Init(GPIOA, &pins);

    terminal_uart.Instance = USART2;
    terminal_uart.Init = (UART_InitTypeDef){
        .BaudRate = 115200U,
        .WordLength = UART_WORDLENGTH_8B,
        .StopBits = UART_STOPBITS_1,
        .Parity = UART_PARITY_NONE,
        .Mode = UART_MODE_TX_RX,
        .HwFlowCtl = UART_HWCONTROL_NONE,
        .OverSampling = UART_OVERSAMPLING_16,
        .OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE
    };
    return HAL_UART_Init(&terminal_uart) == HAL_OK ? 0 : -1;
}

bool bsp_lidar_terminal_try_read(char *byte)
{
    /* A paste can overrun the single-byte receive register. Keep reading
     * subsequent keys without waiting for the terminal. */
    if (__HAL_UART_GET_FLAG(&terminal_uart, UART_FLAG_ORE)) {
        __HAL_UART_CLEAR_OREFLAG(&terminal_uart);
    }
    if (!__HAL_UART_GET_FLAG(&terminal_uart, UART_FLAG_RXNE)) {
        return false;
    }
    *byte = (char)(uint8_t)terminal_uart.Instance->RDR;
    return true;
}

bool bsp_lidar_terminal_try_write(char byte)
{
    if (!__HAL_UART_GET_FLAG(&terminal_uart, UART_FLAG_TXE)) {
        return false;
    }
    terminal_uart.Instance->TDR = (uint8_t)byte;
    return true;
}

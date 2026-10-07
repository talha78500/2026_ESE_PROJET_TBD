#include "bsp.h"

#include "stm32l4xx_hal.h"

#define BSP_RX_DMA_BUFFER_SIZE 1024U

/* CubeMX owns and initializes this handle in Core/Src/main.c. */
extern UART_HandleTypeDef huart1;

/* Private storage: DMA reception continues after bsp_init() returns. */
static uint8_t rx_dma_buffer[BSP_RX_DMA_BUFFER_SIZE];

int bsp_init(void)
{
    HAL_StatusTypeDef status;

    /* CubeMX must initialize USART1 and link its RX DMA before this call. */
    if (huart1.hdmarx == NULL) {
        return -1;
    }
    if (huart1.hdmarx->Init.Mode != DMA_CIRCULAR) {
        return -1;
    }

    /* Arm reception once. Circular DMA keeps running across RX events. */
    status = HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_dma_buffer,
                                        BSP_RX_DMA_BUFFER_SIZE);
    if (status != HAL_OK) {
        return -1;
    }

    return 0;
}

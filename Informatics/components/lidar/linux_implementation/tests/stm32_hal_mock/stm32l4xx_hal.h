#ifndef TEST_STM32L4XX_HAL_H
#define TEST_STM32L4XX_HAL_H

#include <stdint.h>

/* Only the hardware surface used by the STM32 LiDAR BSP is mocked. */
typedef struct {
    volatile uint32_t CNDTR;
} DMA_Channel_TypeDef;

typedef struct {
    DMA_Channel_TypeDef *Instance;
    struct { uint32_t Mode; } Init;
    uint32_t ErrorCode;
} DMA_HandleTypeDef;

typedef struct {
    DMA_HandleTypeDef *hdmarx;
    uint32_t ErrorCode;
} UART_HandleTypeDef;

typedef int HAL_StatusTypeDef;
typedef uint32_t HAL_UART_RxEventTypeTypeDef;

#define HAL_OK 0
#define HAL_ERROR 1
#define DMA_CIRCULAR 1U
#define HAL_UART_RXEVENT_TC 0U
#define HAL_UART_RXEVENT_HT 1U
#define HAL_UART_RXEVENT_IDLE 2U
#define TEST_DMA_TC_FLAG 0x20000U
#define HAL_UART_ERROR_ORE 0x08U
#define HAL_UART_ERROR_FE 0x04U
#define HAL_UART_ERROR_DMA 0x10U
#define UART_CLEAR_OREF 0x01U
#define UART_CLEAR_NEF 0x02U
#define UART_CLEAR_PEF 0x04U
#define UART_CLEAR_FEF 0x08U
#define UART_CLEAR_IDLEF 0x10U
#define UART_RXDATA_FLUSH_REQUEST 1U

#define __HAL_DMA_GET_TC_FLAG_INDEX(handle) TEST_DMA_TC_FLAG
#define __HAL_DMA_GET_FLAG(handle, flag) test_dma_flags(handle, flag)
#define __HAL_DMA_GET_COUNTER(handle) ((handle)->Instance->CNDTR)
#define __HAL_UART_CLEAR_FLAG(handle, flags) test_uart_clear_flags(handle, flags)
#define __HAL_UART_SEND_REQ(handle, request) test_uart_request(handle, request)

uint32_t test_dma_flags(DMA_HandleTypeDef *handle, uint32_t flag);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t mask);
void __DMB(void);
uint32_t HAL_GetTick(void);
void test_uart_clear_flags(UART_HandleTypeDef *handle, uint32_t flags);
void test_uart_request(UART_HandleTypeDef *handle, uint32_t request);

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *handle,
                                             uint8_t *buffer, uint16_t size);
HAL_UART_RxEventTypeTypeDef HAL_UARTEx_GetRxEventType(const UART_HandleTypeDef *handle);
HAL_StatusTypeDef HAL_UART_AbortReceive_IT(UART_HandleTypeDef *handle);

#endif

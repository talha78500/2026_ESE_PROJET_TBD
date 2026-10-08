#include "bsp_lidar.h"

#include "stm32l4xx_hal.h"
#include <stdbool.h>
#include <string.h>

#define BSP_RX_DMA_BUFFER_SIZE 1024U

/* CubeMX owns and initializes this handle in Core/Src/main.c. */
extern UART_HandleTypeDef huart1;

/* Private storage: DMA reception continues after bsp_lidar_init() returns. */
static uint8_t rx_dma_buffer[BSP_RX_DMA_BUFFER_SIZE];

/* Interrupt-written diagnostics. Position and type are meaningful only
 * after the first event; they describe the last notification, not unread data.
 * Volatile does not make these three fields a synchronized snapshot. */
static volatile uint32_t rx_event_count;
static volatile uint16_t rx_last_event_position;
static volatile HAL_UART_RxEventTypeTypeDef rx_last_event_type;

static uint16_t rx_read_position = 0;

static bool uart_reception_started = false;

int bsp_lidar_init(void)
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

    uart_reception_started = true;

    return 0;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    /* HAL shares this callback across UART instances. */
    if (huart != &huart1) {
        return;
    }

    /* Size is a buffer position (1..1024), not a count of new bytes.
     * Keep 1024 as the end position rather than a valid array index. */
    rx_last_event_position = Size;
    rx_last_event_type = HAL_UARTEx_GetRxEventType(huart);
    rx_event_count++;
}

int bsp_lidar_read(uint8_t *buffer, size_t capacity, size_t *received){

	if (received == NULL) {
		return -1;
	}

	*received = 0;

	if (buffer == NULL || capacity == 0){
		return -1;
	}

	if (huart1.hdmarx == NULL) {
		return -1;
	}

	if (uart_reception_started != true) {
		return -1;
	}

	uint16_t rx_remaining_buffer_bytes = __HAL_DMA_GET_COUNTER (huart1.hdmarx);
	uint16_t rx_write_position = BSP_RX_DMA_BUFFER_SIZE - rx_remaining_buffer_bytes;
	rx_write_position = (rx_write_position == BSP_RX_DMA_BUFFER_SIZE) ? 0 : rx_write_position;  // normalizes write position to 0 if no remaining buffer bytes
	uint16_t rx_available_bytes = 0;
	rx_available_bytes = (rx_read_position <= rx_write_position) ? (rx_write_position - rx_read_position) : (BSP_RX_DMA_BUFFER_SIZE - rx_read_position + rx_write_position);

	uint16_t rx_to_read = (rx_available_bytes < capacity) ? rx_available_bytes : capacity;
	uint16_t rx_bytes_before_end = BSP_RX_DMA_BUFFER_SIZE - rx_read_position;
	uint16_t rx_first_read = (rx_to_read < rx_bytes_before_end) ? rx_to_read : rx_bytes_before_end;
	uint16_t rx_second_read = rx_to_read - rx_first_read;

	memcpy(buffer, rx_dma_buffer + rx_read_position, rx_first_read);
	if (rx_second_read > 0) {
		memcpy(buffer + rx_first_read, rx_dma_buffer, rx_second_read);
	}

	rx_read_position = (rx_read_position + rx_to_read < BSP_RX_DMA_BUFFER_SIZE) ? (rx_read_position + rx_to_read) : (rx_read_position + rx_to_read - BSP_RX_DMA_BUFFER_SIZE);

	*received = rx_to_read;


	return 0;

}





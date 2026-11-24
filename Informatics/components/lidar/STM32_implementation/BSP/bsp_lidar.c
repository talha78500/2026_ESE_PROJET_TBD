#include "bsp_lidar.h"

#include "stm32l4xx_hal.h"
#include <stdbool.h>
#include <string.h>

#define BSP_RX_DMA_BUFFER_SIZE 1024U
#define BSP_RX_RETRY_INTERVAL_MS 100U
#define BSP_RX_ABORT_TIMEOUT_MS 100U

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

/* Counts completed laps reported by DMA interrupts, which must be serviced
 * promptly. Volatile does not synchronize a lap count with the DMA position. */
static volatile uint32_t rx_dma_wrap_count = 0;
/* Application-owned total: advanced only after bytes have been copied. */
static uint32_t rx_consumed_bytes = 0;
static uint32_t rx_discarded_bytes = 0;
/* Application-owned, volatile only to retain this diagnostic for debugging. */
static volatile uint32_t rx_overrun_count = 0;

/* ISR-shared fault/lifecycle flags; volatile ensures fresh accesses.
 * Critical sections below protect compound operations and snapshots. */
static volatile bool uart_reception_started; /* App sets, error ISR clears. */
static volatile bool rx_accept_events;       /* App/ISR write, callbacks read. */
static volatile bool rx_fault_pending;       /* Error ISR sets, app clears. */
static volatile bool rx_abort_pending;       /* App writes, abort callback reads. */
static volatile bool rx_abort_complete;      /* Callback sets, app consumes. */
static volatile uint32_t rx_error_generation; /* ISR increments, reader checks. */
/* Error callbacks write these counters/codes; app snapshots them. */
static volatile uint32_t rx_uart_errors;
static volatile uint32_t rx_dma_errors;
static volatile uint32_t rx_last_uart_error;
static volatile uint32_t rx_last_dma_error;

/* Application-owned lifecycle state and cumulative counters. */
static bsp_lidar_state_t rx_state = BSP_LIDAR_STOPPED;
static bool rx_want_running;
static bool rx_needs_abort;
static bool rx_ever_started;
static bool rx_discontinuity_pending;
static uint32_t rx_retry_tick;
static uint32_t rx_abort_tick;
static uint32_t rx_copied_total;
static uint32_t rx_discarded_total;
static uint32_t rx_restarts;
static uint32_t rx_start_failures;
static uint32_t rx_abort_failures;
static uint32_t last_read_at_ms;
static uint32_t max_read_gap_ms;
static uint32_t last_overwrite_read_gap_ms;
static uint32_t last_overwrite_unread_bytes;
static bool have_read_timestamp;

static void reset_rx_epoch(void)
{
    /* Call only while DMA is stopped, before accepting new callbacks. */
    rx_dma_wrap_count = 0;
    rx_consumed_bytes = 0;
    rx_discarded_bytes = 0;
    rx_read_position = 0;
    have_read_timestamp = false;
}

static int start_reception(void)
{
    HAL_StatusTypeDef status;
    uint32_t saved_mask;
    uint32_t generation;
    bool started;

    saved_mask = __get_PRIMASK();
    __disable_irq();
    reset_rx_epoch();
    rx_fault_pending = false;
    uart_reception_started = false;
    generation = rx_error_generation;
    __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_OREF | UART_CLEAR_NEF |
                         UART_CLEAR_PEF | UART_CLEAR_FEF | UART_CLEAR_IDLEF);
    __HAL_UART_SEND_REQ(&huart1, UART_RXDATA_FLUSH_REQUEST);
    rx_accept_events = true;
    __set_PRIMASK(saved_mask);

    status = HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_dma_buffer,
                                        BSP_RX_DMA_BUFFER_SIZE);

    /* An error callback can run inside/before the start API returns. */
    saved_mask = __get_PRIMASK();
    __disable_irq();
    started = status == HAL_OK && !rx_fault_pending &&
              generation == rx_error_generation;
    uart_reception_started = started;
    if (!started) {
        rx_accept_events = false;
    }
    __set_PRIMASK(saved_mask);

    if (!started) {
        rx_start_failures++;
        rx_needs_abort = true;
        rx_state = BSP_LIDAR_RECOVERING;
        return -1;
    }

    if (rx_ever_started) {
        rx_restarts++;
        rx_discontinuity_pending = true;
    }
    rx_ever_started = true;
    rx_state = BSP_LIDAR_RUNNING;
    return 0;
}

int bsp_lidar_init(void)
{
    if (rx_state != BSP_LIDAR_STOPPED) {
        return -1;
    }
    /* CubeMX owns peripheral setup. Reject a missing or incompatible DMA. */
    if (huart1.hdmarx == NULL || huart1.hdmarx->Instance == NULL ||
        huart1.hdmarx->Init.Mode != DMA_CIRCULAR) {
        rx_state = BSP_LIDAR_FAULTED;
        return -1;
    }

    rx_want_running = true;
    rx_needs_abort = false;
    rx_abort_pending = false;
    rx_abort_complete = false;
    rx_retry_tick = HAL_GetTick();
    return start_reception();
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    HAL_UART_RxEventTypeTypeDef event_type;

    /* HAL shares this callback across UART instances. */
    if (huart != &huart1 || !rx_accept_events) {
        return;
    }

    event_type = HAL_UARTEx_GetRxEventType(huart);

    /* Size is a buffer position (1..1024), not a count of new bytes.
     * Keep 1024 as the end position rather than a valid array index. */
    rx_last_event_position = Size;
    rx_last_event_type = event_type;
    rx_event_count++;

    if (event_type == HAL_UART_RXEVENT_TC) {
        rx_dma_wrap_count++;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    uint32_t error;

    if (huart != &huart1 || !rx_accept_events) {
        return;
    }
    error = huart->ErrorCode;
    rx_last_uart_error = error;
    if ((error & HAL_UART_ERROR_DMA) != 0) {
        rx_dma_errors++;
        rx_last_dma_error = huart->hdmarx != NULL ? huart->hdmarx->ErrorCode : 0;
    }
    if ((error & ~HAL_UART_ERROR_DMA) != 0 || error == 0) {
        rx_uart_errors++;
    }
    rx_error_generation++;
    rx_fault_pending = true;
    uart_reception_started = false;
    rx_accept_events = false;
    /* Recovery and restarting belong to application context, not this ISR. */
}

void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1 && rx_abort_pending) {
        rx_abort_complete = true;
    }
}

void bsp_lidar_deinit(void)
{
    rx_want_running = false;
    if (rx_state == BSP_LIDAR_STOPPED) {
        return;
    }
    uart_reception_started = false;
    rx_accept_events = false;
    rx_discontinuity_pending = true;
    rx_state = BSP_LIDAR_STOPPING;
    if (!rx_abort_pending) {
        rx_needs_abort = true;
    } else {
        rx_abort_tick = HAL_GetTick();
    }
}

void bsp_lidar_process(void)
{
    HAL_StatusTypeDef status;

    if (rx_state == BSP_LIDAR_RUNNING && rx_fault_pending) {
        rx_discontinuity_pending = true;
        rx_needs_abort = true;
        rx_state = BSP_LIDAR_RECOVERING;
    }
    if (rx_state != BSP_LIDAR_STOPPING && rx_state != BSP_LIDAR_RECOVERING) {
        return;
    }
    if (rx_abort_pending) {
        if (!rx_abort_complete) {
            if ((uint32_t)(HAL_GetTick() - rx_abort_tick) >= BSP_RX_ABORT_TIMEOUT_MS) {
                rx_abort_failures++;
                rx_want_running = false;
                rx_state = BSP_LIDAR_FAULTED;
            }
            return;
        }
        rx_abort_pending = false;
        rx_abort_complete = false;
        rx_fault_pending = false;
        reset_rx_epoch();
        rx_retry_tick = HAL_GetTick();
        rx_state = rx_want_running ? BSP_LIDAR_RECOVERING : BSP_LIDAR_STOPPED;
        return;
    }
    if (rx_needs_abort) {
        /* Arm before calling HAL: this HAL can complete synchronously. */
        rx_abort_pending = true;
        rx_abort_complete = false;
        rx_needs_abort = false;
        rx_abort_tick = HAL_GetTick();
        status = HAL_UART_AbortReceive_IT(&huart1);
        if (status != HAL_OK) {
            rx_abort_pending = false;
            rx_abort_failures++;
            rx_state = BSP_LIDAR_FAULTED;
        }
        return;
    }
    if (rx_want_running && (uint32_t)(HAL_GetTick() - rx_retry_tick) >=
        BSP_RX_RETRY_INTERVAL_MS) {
        rx_retry_tick = HAL_GetTick();
        (void)start_reception();
    }
}

bsp_lidar_stats_t bsp_lidar_get_stats(void)
{
    bsp_lidar_stats_t result;
    uint32_t saved_mask = __get_PRIMASK();

    __disable_irq();
    result.state = rx_state;
    if (rx_state == BSP_LIDAR_RUNNING && !uart_reception_started) {
        result.state = BSP_LIDAR_RECOVERING;
    }
    result.copied_bytes = rx_copied_total;
    result.discarded_bytes = rx_discarded_total;
    result.overwrite_events = rx_overrun_count;
    result.uart_errors = rx_uart_errors;
    result.dma_errors = rx_dma_errors;
    result.restarts = rx_restarts;
    result.start_failures = rx_start_failures;
    result.abort_failures = rx_abort_failures;
    result.last_uart_error = rx_last_uart_error;
    result.last_dma_error = rx_last_dma_error;
    result.max_read_gap_ms = max_read_gap_ms;
    result.last_overwrite_read_gap_ms = last_overwrite_read_gap_ms;
    result.last_overwrite_unread_bytes = last_overwrite_unread_bytes;
    __set_PRIMASK(saved_mask);
    return result;
}

/* Application context only. One attempt: never wait for a DMA boundary.
 * TC interrupts must be serviced at least once per buffer lap. A sticky
 * hardware flag cannot reveal multiple laps missed during a long halt. */
static bool snapshot_rx_progress(uint32_t *produced_bytes)
{
    DMA_HandleTypeDef *dma = huart1.hdmarx;
    uint32_t tc_flag = __HAL_DMA_GET_TC_FLAG_INDEX(dma);
    uint32_t saved_mask = __get_PRIMASK();
    uint32_t wraps;
    uint32_t remaining;
    bool pending_before;
    bool pending_after;

    /* Freeze callback accounting briefly; DMA itself continues running.
     * The barriers also order buffer reads relative to these snapshots. */
    __disable_irq();
    __DMB();
    if (!uart_reception_started) {
        __set_PRIMASK(saved_mask);
        return false;
    }
    wraps = rx_dma_wrap_count;
    pending_before = __HAL_DMA_GET_FLAG(dma, tc_flag) != 0;
    remaining = __HAL_DMA_GET_COUNTER(dma);
    pending_after = __HAL_DMA_GET_FLAG(dma, tc_flag) != 0;
    __DMB();
    __set_PRIMASK(saved_mask);

    /* A wrap crossed the samples, or the counter is reloading. Defer to
     * the next application call rather than combining different laps. */
    if (pending_before != pending_after || remaining == 0 ||
        remaining > BSP_RX_DMA_BUFFER_SIZE) {
        return false;
    }
    if (pending_before) {
        /* Account locally for an unserviced completion. Leave its flag
         * and the shared count for HAL/the callback to handle normally. */
        wraps++;
    }

    *produced_bytes = wraps * BSP_RX_DMA_BUFFER_SIZE +
                      (BSP_RX_DMA_BUFFER_SIZE - remaining);
    return true;
}

static void discard_unread_input(uint32_t produced_bytes, uint32_t read_gap_ms)
{
    uint32_t unread = produced_bytes - rx_consumed_bytes - rx_discarded_bytes;

    /* Drop the entire unreliable backlog, not just the overwritten part.
     * Consumed bytes continue to count successful copies only. */
    rx_discarded_bytes += unread;
    rx_discarded_total += unread;
    rx_read_position = (uint16_t)(produced_bytes % BSP_RX_DMA_BUFFER_SIZE);
    last_overwrite_read_gap_ms = read_gap_ms;
    last_overwrite_unread_bytes = unread;
    rx_overrun_count++;
}

int bsp_lidar_read(uint8_t *buffer, size_t capacity, size_t *received)
{
    uint32_t produced_before;
    uint32_t produced_after;
    uint32_t unread;
    size_t to_read;
    size_t before_end;
    size_t first_read;
    size_t second_read;
    uint32_t generation;
    uint32_t saved_mask;

    if (received == NULL) {
        return -1;
    }
    *received = 0;
    if (buffer == NULL || capacity == 0) {
        return -1;
    }
    if (huart1.hdmarx == NULL) {
        return -1;
    }
    if (!uart_reception_started) {
        return -1;
    }
    if (rx_discontinuity_pending) {
        rx_discontinuity_pending = false;
        return -1;
    }
    uint32_t now_ms = HAL_GetTick();
    uint32_t read_gap_ms = have_read_timestamp ? now_ms - last_read_at_ms : 0;
    last_read_at_ms = now_ms;
    have_read_timestamp = true;
    if (read_gap_ms > max_read_gap_ms) {
        max_read_gap_ms = read_gap_ms;
    }
    generation = rx_error_generation;
    if (!snapshot_rx_progress(&produced_before)) {
        return uart_reception_started ? 0 : -1;
    }

    /* Unsigned subtraction supports byte-counter rollover. The 1024-byte
     * ring also divides the uint32_t counter's range exactly. */
    unread = produced_before - rx_consumed_bytes - rx_discarded_bytes;
    if (unread > BSP_RX_DMA_BUFFER_SIZE) {
        discard_unread_input(produced_before, read_gap_ms);
        return -1;
    }
    if (unread == 0) {
        return 0;
    }

    to_read = unread < capacity ? unread : capacity;
    before_end = BSP_RX_DMA_BUFFER_SIZE - rx_read_position;
    first_read = to_read < before_end ? to_read : before_end;
    second_read = to_read - first_read;

    memcpy(buffer, rx_dma_buffer + rx_read_position, first_read);
    if (second_read > 0) {
        memcpy(buffer + first_read, rx_dma_buffer, second_read);
    }

    /* Interrupts stayed enabled during copying. Publish nothing if the
     * snapshot is unstable or DMA may have overwritten the source. */
    if (!snapshot_rx_progress(&produced_after)) {
        return uart_reception_started ? 0 : -1;
    }
    unread = produced_after - rx_consumed_bytes - rx_discarded_bytes;
    if (unread > BSP_RX_DMA_BUFFER_SIZE) {
        discard_unread_input(produced_after, read_gap_ms);
        return -1;
    }

    /* Do not publish bytes if an error interrupted either the copy or the
     * interval between the final hardware snapshot and this commit. */
    saved_mask = __get_PRIMASK();
    __disable_irq();
    if (!uart_reception_started || generation != rx_error_generation) {
        __set_PRIMASK(saved_mask);
        return -1;
    }
    rx_consumed_bytes += (uint32_t)to_read;
    rx_copied_total += (uint32_t)to_read;
    rx_read_position = (uint16_t)((rx_read_position + to_read) % BSP_RX_DMA_BUFFER_SIZE);
    __set_PRIMASK(saved_mask);
    *received = to_read;
    return 0;
}

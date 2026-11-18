#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "stm32l4xx_hal.h"
#include "lidar_test.h"

static DMA_Channel_TypeDef registers;
static DMA_HandleTypeDef dma = {
    .Instance = &registers, .Init = { .Mode = DMA_CIRCULAR }
};
UART_HandleTypeDef huart1 = { .hdmarx = &dma };

static uint32_t interrupt_mask;
static uint32_t pending_tc;
static unsigned flag_reads;
static unsigned wrap_on_flag_read;
static bool service_on_unmask;
static bool overrun_on_copy;
static bool error_on_copy;
static bool error_on_start;
static unsigned error_on_unmask;
static unsigned mask_restores;
static uint32_t tick;
static HAL_StatusTypeDef start_status;
static HAL_StatusTypeDef abort_status;
static bool abort_synchronous;
static unsigned start_calls;
static unsigned abort_calls;
static unsigned flush_calls;
static HAL_UART_RxEventTypeTypeDef event_type;

static void *copy_with_dma_progress(void *destination, const void *source, size_t count);

/* Compile the actual BSP in this test translation unit. The copy hook lets
 * tests advance hardware during a copy, without modifying production code. */
#define memcpy copy_with_dma_progress
#include "../../STM32_implementation/BSP/bsp_lidar.c"
#undef memcpy

uint32_t __get_PRIMASK(void) { return interrupt_mask; }
void __disable_irq(void) { interrupt_mask = 1; }
void __DMB(void) {}

void __set_PRIMASK(uint32_t mask)
{
    interrupt_mask = mask;
    mask_restores++;
    if (mask == 0 && error_on_unmask == mask_restores) {
        huart1.ErrorCode = HAL_UART_ERROR_ORE;
        HAL_UART_ErrorCallback(&huart1);
    }
    if (mask == 0 && service_on_unmask && pending_tc != 0) {
        pending_tc = 0; /* HAL clears the flag before invoking the callback. */
        event_type = HAL_UART_RXEVENT_TC;
        HAL_UARTEx_RxEventCallback(&huart1, 10);
    }
}

uint32_t HAL_GetTick(void) { return tick; }

void test_uart_clear_flags(UART_HandleTypeDef *handle, uint32_t flags)
{
    assert(handle == &huart1 && flags != 0);
}

void test_uart_request(UART_HandleTypeDef *handle, uint32_t request)
{
    assert(handle == &huart1 && request == UART_RXDATA_FLUSH_REQUEST);
    flush_calls++;
}

HAL_StatusTypeDef HAL_UART_AbortReceive_IT(UART_HandleTypeDef *handle)
{
    assert(handle == &huart1);
    abort_calls++;
    if (abort_status != HAL_OK) {
        return abort_status;
    }
    pending_tc = 0;
    if (abort_synchronous) {
        HAL_UART_AbortReceiveCpltCallback(handle);
    }
    return HAL_OK;
}

uint32_t test_dma_flags(DMA_HandleTypeDef *handle, uint32_t flag)
{
    uint32_t result = pending_tc;

    assert(handle == &dma && flag == TEST_DMA_TC_FLAG);
    assert(interrupt_mask == 1);
    flag_reads++;
    if (flag_reads == wrap_on_flag_read) {
        /* Hardware wraps between the flag and counter reads. */
        pending_tc = TEST_DMA_TC_FLAG;
        registers.CNDTR = 1024 - 9;
    }
    return result;
}

HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *handle,
                                             uint8_t *buffer, uint16_t size)
{
    assert(handle == &huart1 && buffer == rx_dma_buffer && size == 1024);
    assert(rx_dma_wrap_count == 0 && rx_consumed_bytes == 0);
    assert(rx_discarded_bytes == 0 && rx_read_position == 0);
    start_calls++;
    registers.CNDTR = size;
    pending_tc = 0;
    handle->ErrorCode = 0;
    dma.ErrorCode = 0;
    if (error_on_start) {
        error_on_start = false;
        handle->ErrorCode = HAL_UART_ERROR_FE;
        HAL_UART_ErrorCallback(handle);
    }
    return start_status;
}

HAL_UART_RxEventTypeTypeDef HAL_UARTEx_GetRxEventType(const UART_HandleTypeDef *handle)
{
    assert(handle == &huart1);
    return event_type;
}

static void *copy_with_dma_progress(void *destination, const void *source, size_t count)
{
    void *result = memcpy(destination, source, count);

    assert(interrupt_mask == 0); /* Bulk copies must not mask interrupts. */
    if (error_on_copy) {
        error_on_copy = false;
        huart1.ErrorCode = HAL_UART_ERROR_FE;
        HAL_UART_ErrorCallback(&huart1);
    }
    if (overrun_on_copy) {
        overrun_on_copy = false;
        pending_tc = TEST_DMA_TC_FLAG;
        registers.CNDTR = 1024 - 16;
        memset(rx_dma_buffer, 0xEE, sizeof rx_dma_buffer);
    }
    return result;
}

static void reset_fixture(void)
{
    uart_reception_started = false;
    rx_state = BSP_LIDAR_STOPPED;
    rx_accept_events = false;
    rx_fault_pending = false;
    rx_abort_pending = false;
    rx_abort_complete = false;
    rx_want_running = false;
    rx_needs_abort = false;
    rx_ever_started = false;
    rx_discontinuity_pending = false;
    rx_error_generation = 0;
    rx_uart_errors = 0;
    rx_dma_errors = 0;
    rx_last_uart_error = 0;
    rx_last_dma_error = 0;
    rx_copied_total = 0;
    rx_discarded_total = 0;
    rx_overrun_count = 0;
    rx_restarts = 0;
    rx_start_failures = 0;
    rx_abort_failures = 0;
    huart1.hdmarx = &dma;
    huart1.ErrorCode = 0;
    dma.ErrorCode = 0;
    interrupt_mask = 0;
    pending_tc = 0;
    flag_reads = 0;
    wrap_on_flag_read = 0;
    service_on_unmask = false;
    overrun_on_copy = false;
    error_on_copy = false;
    error_on_start = false;
    error_on_unmask = 0;
    mask_restores = 0;
    tick = 0;
    start_status = HAL_OK;
    abort_status = HAL_OK;
    abort_synchronous = true;
    start_calls = 0;
    abort_calls = 0;
    flush_calls = 0;
    rx_event_count = 0;
    assert(bsp_lidar_init() == 0);
    mask_restores = 0;
    for (size_t i = 0; i < sizeof rx_dma_buffer; i++) {
        rx_dma_buffer[i] = (uint8_t)(i % 251);
    }
}

static void set_progress(uint32_t serviced_laps, uint16_t position, bool pending)
{
    rx_dma_wrap_count = serviced_laps;
    registers.CNDTR = 1024 - position;
    pending_tc = pending ? TEST_DMA_TC_FLAG : 0;
}

static void set_consumer(uint32_t copied, uint32_t discarded)
{
    rx_consumed_bytes = copied;
    rx_discarded_bytes = discarded;
    rx_read_position = (uint16_t)((copied + discarded) % 1024);
}

static void test_copying_and_full_buffer(void)
{
    uint8_t destination[1026];
    size_t received;

    reset_fixture();
    set_progress(1, 100, false);
    set_consumer(900, 0);
    assert(bsp_lidar_read(destination, 200, &received) == 0 && received == 200);
    for (size_t i = 0; i < received; i++) {
        assert(destination[i] == rx_dma_buffer[(900 + i) % 1024]);
    }
    assert(rx_read_position == 76 && rx_consumed_bytes == 1100);
    assert(bsp_lidar_read(destination, 200, &received) == 0 && received == 24);
    assert(bsp_lidar_read(destination, 200, &received) == 0 && received == 0);

    reset_fixture();
    set_progress(1, 0, false);
    memset(destination, 0xA5, sizeof destination);
    assert(bsp_lidar_read(destination + 1, 1024, &received) == 0 && received == 1024);
    assert(destination[0] == 0xA5 && destination[1025] == 0xA5);
    assert(memcmp(destination + 1, rx_dma_buffer, 1024) == 0);
    assert(rx_read_position == 0 && rx_consumed_bytes == 1024);
    assert(bsp_lidar_read(destination, 1, &received) == 0 && received == 0);
}

static void test_snapshot_boundaries(void)
{
    uint8_t destination[256];
    size_t received;

    reset_fixture();
    set_consumer(1000, 0);
    set_progress(0, 10, true);
    service_on_unmask = true;
    assert(bsp_lidar_read(destination, 256, &received) == 0 && received == 34);
    assert(rx_dma_wrap_count == 1 && pending_tc == 0);
    assert(bsp_lidar_read(destination, 256, &received) == 0 && received == 0);

    reset_fixture();
    set_consumer(1000, 0);
    set_progress(0, 1010, false);
    wrap_on_flag_read = 1;
    assert(bsp_lidar_read(destination, 256, &received) == 0 && received == 0);
    assert(rx_consumed_bytes == 1000 && interrupt_mask == 0);
    assert(bsp_lidar_read(destination, 256, &received) == 0 && received == 33);

    /* A wrap during the post-copy snapshot must not publish that copy. */
    reset_fixture();
    set_consumer(1000, 0);
    set_progress(0, 1010, false);
    wrap_on_flag_read = 3;
    assert(bsp_lidar_read(destination, 256, &received) == 0 && received == 0);
    assert(rx_consumed_bytes == 1000 && rx_read_position == 1000);
    assert(bsp_lidar_read(destination, 256, &received) == 0 && received == 33);

    reset_fixture();
    interrupt_mask = 1;
    registers.CNDTR = 0;
    assert(bsp_lidar_read(destination, 256, &received) == 0 && received == 0);
    assert(interrupt_mask == 1 && rx_consumed_bytes == 0);
}

static void test_overwrite_and_recovery(void)
{
    uint8_t destination[256];
    size_t received;

    reset_fixture();
    set_progress(1, 100, false);
    assert(bsp_lidar_read(destination, 256, &received) == -1 && received == 0);
    assert(rx_overrun_count == 1 && rx_discarded_bytes == 1124);
    assert(rx_consumed_bytes == 0 && rx_read_position == 100);
    set_progress(1, 105, false);
    assert(bsp_lidar_read(destination, 256, &received) == 0 && received == 5);
    assert(memcmp(destination, rx_dma_buffer + 100, 5) == 0);

    reset_fixture();
    set_progress(0, 10, false);
    overrun_on_copy = true;
    assert(bsp_lidar_read(destination, 8, &received) == -1 && received == 0);
    assert(rx_overrun_count == 1 && rx_discarded_bytes == 1040);
    assert(rx_consumed_bytes == 0 && rx_read_position == 16);
    set_progress(1, 20, false);
    assert(bsp_lidar_read(destination, 256, &received) == 0 && received == 4);
    assert(rx_consumed_bytes == 4 && rx_discarded_bytes == 1040);
}

static void test_accounting_and_arguments(void)
{
    uint8_t destination[256];
    size_t received = 99;

    reset_fixture();
    event_type = HAL_UART_RXEVENT_HT;
    HAL_UARTEx_RxEventCallback(&huart1, 512);
    event_type = HAL_UART_RXEVENT_IDLE;
    HAL_UARTEx_RxEventCallback(&huart1, 1024);
    assert(rx_dma_wrap_count == 0);
    event_type = HAL_UART_RXEVENT_TC;
    HAL_UARTEx_RxEventCallback(&huart1, 100);
    assert(rx_dma_wrap_count == 1);
    assert(bsp_lidar_init() == -1 && rx_dma_wrap_count == 1);

    assert(bsp_lidar_read(NULL, 1, &received) == -1 && received == 0);
    assert(bsp_lidar_read(destination, 0, &received) == -1);
    assert(bsp_lidar_read(destination, 1, NULL) == -1);

    reset_fixture();
    set_consumer(UINT32_MAX - 15, 0);
    set_progress(UINT32_MAX / 1024 + 1, 16, false);
    assert(bsp_lidar_read(destination, 256, &received) == 0 && received == 32);
    assert(rx_consumed_bytes == 16 && rx_read_position == 16 && rx_overrun_count == 0);
}

static void test_fault_recovery(void)
{
    uint8_t destination[16];
    size_t received = 99;

    reset_fixture();
    huart1.ErrorCode = HAL_UART_ERROR_ORE;
    HAL_UART_ErrorCallback(&huart1);
    assert(!uart_reception_started);
    assert(bsp_lidar_get_stats().state == BSP_LIDAR_RECOVERING);
    assert(bsp_lidar_read(destination, sizeof destination, &received) == -1 && received == 0);
    bsp_lidar_process(); /* Request abort; callback completes synchronously. */
    assert(abort_calls == 1 && rx_abort_complete);
    bsp_lidar_process(); /* Collect completion, without starting immediately. */
    tick = 99;
    bsp_lidar_process();
    assert(start_calls == 1);
    tick = 100;
    bsp_lidar_process();
    assert(start_calls == 2 && uart_reception_started);
    assert(bsp_lidar_get_stats().uart_errors == 1 && bsp_lidar_get_stats().restarts == 1);
    assert(bsp_lidar_get_stats().last_uart_error == HAL_UART_ERROR_ORE);
    assert(bsp_lidar_read(destination, sizeof destination, &received) == -1);
    set_progress(0, 5, false);
    assert(bsp_lidar_read(destination, sizeof destination, &received) == 0 && received == 5);

    reset_fixture();
    huart1.ErrorCode = HAL_UART_ERROR_DMA;
    dma.ErrorCode = 0x20;
    HAL_UART_ErrorCallback(&huart1);
    assert(bsp_lidar_get_stats().dma_errors == 1 && bsp_lidar_get_stats().uart_errors == 0);
    assert(bsp_lidar_get_stats().last_dma_error == 0x20);
    bsp_lidar_deinit(); /* Stop must cancel automatic recovery. */
    bsp_lidar_process();
    bsp_lidar_process();
    tick = 1000;
    bsp_lidar_process();
    assert(bsp_lidar_get_stats().state == BSP_LIDAR_STOPPED && start_calls == 1);
    assert(bsp_lidar_init() == 0);
    assert(bsp_lidar_get_stats().dma_errors == 1 && bsp_lidar_get_stats().restarts == 1);
}

static void test_async_stop_and_failures(void)
{
    reset_fixture();
    abort_synchronous = false;
    bsp_lidar_deinit();
    assert(bsp_lidar_get_stats().state == BSP_LIDAR_STOPPING && !uart_reception_started);
    assert(bsp_lidar_init() == -1);
    bsp_lidar_process();
    bsp_lidar_process();
    assert(abort_calls == 1 && bsp_lidar_get_stats().state == BSP_LIDAR_STOPPING);
    UART_HandleTypeDef other = {0};
    HAL_UART_AbortReceiveCpltCallback(&other);
    assert(!rx_abort_complete);
    HAL_UART_AbortReceiveCpltCallback(&huart1);
    bsp_lidar_process();
    assert(bsp_lidar_get_stats().state == BSP_LIDAR_STOPPED);
    uint32_t events = rx_event_count;
    HAL_UARTEx_RxEventCallback(&huart1, 1024);
    assert(rx_event_count == events); /* Stale notifications cannot affect a stopped epoch. */
    bsp_lidar_deinit();
    bsp_lidar_process();
    assert(abort_calls == 1); /* Idempotent stop. */
    assert(bsp_lidar_init() == 0 && rx_consumed_bytes == 0 && rx_dma_wrap_count == 0);

    reset_fixture();
    abort_synchronous = false;
    bsp_lidar_deinit();
    bsp_lidar_process();
    tick = 100;
    bsp_lidar_process();
    assert(bsp_lidar_get_stats().state == BSP_LIDAR_FAULTED);
    assert(bsp_lidar_get_stats().abort_failures == 1 && !uart_reception_started);
    HAL_UART_AbortReceiveCpltCallback(&huart1); /* Late completion remains collectable. */
    bsp_lidar_deinit();
    bsp_lidar_process();
    assert(bsp_lidar_get_stats().state == BSP_LIDAR_STOPPED);

    reset_fixture();
    abort_status = HAL_ERROR;
    bsp_lidar_deinit();
    bsp_lidar_process();
    assert(bsp_lidar_get_stats().state == BSP_LIDAR_FAULTED);
    assert(bsp_lidar_get_stats().abort_failures == 1);
    abort_status = HAL_OK;
    bsp_lidar_deinit();
    bsp_lidar_process();
    bsp_lidar_process();
    assert(bsp_lidar_get_stats().state == BSP_LIDAR_STOPPED);
}

static void test_start_retry_and_error_during_copy(void)
{
    uint8_t destination[16];
    size_t received;

    reset_fixture();
    huart1.ErrorCode = HAL_UART_ERROR_ORE;
    HAL_UART_ErrorCallback(&huart1);
    bsp_lidar_process();
    bsp_lidar_process();
    start_status = HAL_ERROR;
    tick = 100;
    bsp_lidar_process();
    assert(bsp_lidar_get_stats().start_failures == 1 && !uart_reception_started);
    bsp_lidar_process();
    bsp_lidar_process();
    start_status = HAL_OK;
    tick = 199;
    bsp_lidar_process();
    assert(start_calls == 2);
    tick = 200;
    bsp_lidar_process();
    assert(start_calls == 3 && uart_reception_started);

    reset_fixture();
    bsp_lidar_deinit();
    bsp_lidar_process();
    bsp_lidar_process();
    error_on_start = true;
    assert(bsp_lidar_init() == -1);
    assert(!uart_reception_started && bsp_lidar_get_stats().state == BSP_LIDAR_RECOVERING);
    assert(bsp_lidar_get_stats().uart_errors == 1);

    reset_fixture();
    set_progress(0, 10, false);
    error_on_copy = true;
    assert(bsp_lidar_read(destination, 8, &received) == -1 && received == 0);
    assert(rx_consumed_bytes == 0 && rx_read_position == 0);

    reset_fixture();
    set_progress(0, 10, false);
    error_on_unmask = 2; /* Error immediately after post-copy snapshot. */
    assert(bsp_lidar_read(destination, 8, &received) == -1 && received == 0);
    assert(rx_consumed_bytes == 0 && rx_read_position == 0);
}

static void run_application(uint32_t now_ms)
{
    tick = now_ms;
    lidar_test_process(now_ms);
}

static void deliver_packet(uint16_t position)
{
    /* One rotation-start sample at 1000 mm: XOR checksum is 0x5B0B. */
    const uint8_t packet[12] = {0xAA, 0x55, 1, 1, 1, 0, 1, 0, 0x0B, 0x5B, 0xA0, 0x0F};

    memcpy(rx_dma_buffer + position, packet, sizeof packet);
    registers.CNDTR = 1024 - position - sizeof packet;
}

static void test_board_controls_with_parser(void)
{
    reset_fixture();
    /* Start the real application from a simulated boot state. */
    rx_state = BSP_LIDAR_STOPPED;
    rx_ever_started = false;
    uart_reception_started = false;
    rx_accept_events = false;
    lidar_test_diagnostics.initialized = false;
    assert(lidar_test_init() == 0);
    deliver_packet(0);
    run_application(0);
    assert(lidar_test_diagnostics.parser.valid_packets == 1);
    assert(lidar_test_diagnostics.latest_point.distance_mm == 1000.0f);

    lidar_test_controls.command = LIDAR_TEST_COMMAND_PAUSE_READER;
    run_application(100);
    set_progress(2, 100, false); /* DMA/interrupts run during the reader pause. */
    run_application(200);
    assert(lidar_test_diagnostics.reader_pause_active);
    assert(lidar_test_diagnostics.parser.valid_packets == 1);
    run_application(350);
    assert(!lidar_test_diagnostics.reader_pause_active);
    assert(lidar_test_diagnostics.bsp.overwrite_events == 1);
    assert(lidar_test_diagnostics.read_errors == 1);
    deliver_packet(100);
    run_application(351);
    assert(lidar_test_diagnostics.parser.valid_packets == 2);

    lidar_test_controls.command = LIDAR_TEST_COMMAND_STOP;
    run_application(352);
    run_application(353);
    assert(lidar_test_diagnostics.bsp.state == BSP_LIDAR_STOPPED);
    assert(!lidar_test_diagnostics.has_point);
    lidar_test_controls.command = LIDAR_TEST_COMMAND_START;
    run_application(354);
    assert(lidar_test_diagnostics.bsp.state == BSP_LIDAR_RUNNING);
    deliver_packet(0);
    run_application(355);
    assert(lidar_test_diagnostics.parser.valid_packets == 3);

    lidar_test_controls.command = LIDAR_TEST_COMMAND_RESTART;
    run_application(356);
    run_application(357);
    assert(lidar_test_diagnostics.bsp.state == BSP_LIDAR_RUNNING);
    deliver_packet(0);
    run_application(358);
    assert(lidar_test_diagnostics.parser.valid_packets == 4);
    assert(lidar_test_diagnostics.bsp.overwrite_events == 1);
    assert(lidar_test_diagnostics.bsp.copied_bytes == 48);
    assert(lidar_test_diagnostics.bsp.restarts == 2);

    lidar_test_controls.command = LIDAR_TEST_COMMAND_PAUSE_READER;
    run_application(UINT32_MAX - 100);
    run_application(0);
    assert(lidar_test_diagnostics.reader_pause_active);
    run_application(149);
    assert(!lidar_test_diagnostics.reader_pause_active);

    lidar_test_controls.reader_pause_ms = UINT32_MAX;
    lidar_test_controls.command = LIDAR_TEST_COMMAND_PAUSE_READER;
    run_application(1000);
    run_application(1499);
    assert(lidar_test_diagnostics.reader_pause_active);
    run_application(1500);
    assert(!lidar_test_diagnostics.reader_pause_active); /* Duration is capped at 500 ms. */

    lidar_test_controls.command = LIDAR_TEST_COMMAND_RESTART;
    run_application(1501);
    lidar_test_controls.command = LIDAR_TEST_COMMAND_STOP;
    run_application(1502);
    run_application(2000);
    assert(lidar_test_diagnostics.bsp.state == BSP_LIDAR_STOPPED);
    assert(lidar_test_diagnostics.bsp.restarts == 2);
}

static void test_application_initial_start_failure(void)
{
    reset_fixture();
    rx_state = BSP_LIDAR_STOPPED;
    rx_ever_started = false;
    uart_reception_started = false;
    rx_accept_events = false;
    lidar_test_diagnostics.initialized = false;
    start_status = HAL_ERROR;
    assert(lidar_test_init() == 0); /* Accepted recovery must not halt main. */
    assert(lidar_test_diagnostics.initialized);
    assert(lidar_test_diagnostics.bsp.state == BSP_LIDAR_RECOVERING);
    run_application(0);
    run_application(1);
    start_status = HAL_OK;
    run_application(100);
    assert(lidar_test_diagnostics.bsp.state == BSP_LIDAR_RECOVERING);
    run_application(101);
    assert(lidar_test_diagnostics.bsp.state == BSP_LIDAR_RUNNING);
    deliver_packet(0);
    run_application(102);
    assert(lidar_test_diagnostics.parser.valid_packets == 1);
}

int main(void)
{
    test_copying_and_full_buffer();
    test_snapshot_boundaries();
    test_overwrite_and_recovery();
    test_accounting_and_arguments();
    test_fault_recovery();
    test_async_stop_and_failures();
    test_start_retry_and_error_during_copy();
    test_board_controls_with_parser();
    test_application_initial_start_failure();
    puts("PASS: STM32 BSP full/empty rings, split copies, pending TC and snapshot races, interrupt-mask restoration, overwrite before/during copying, recovery, and counter rollover.");
    puts("PASS: UART/DMA faults, deferred recovery and retry timing, asynchronous/idempotent stop, abort failure/timeout, restart, error races, and real parser recovery through timed board-test commands.");
    return 0;
}

CC = cc
LIDAR_APP = ../STM32_implementation/App
LIDAR_BSP = ../STM32_implementation/BSP
CPPFLAGS += -I$(LIDAR_APP) -I$(LIDAR_BSP)
CFLAGS += -std=c11 -Wall -Wextra -Wpedantic -Werror -O2
LDLIBS += -lm
PC_MAP_SOURCES = src/pc_map.c src/can_detector.c
PC_MAP_HEADERS = src/pc_map.h src/can_detector.h

output/lidar-reader: src/main.c $(PC_MAP_SOURCES) $(PC_MAP_HEADERS) $(LIDAR_APP)/lidar.c bsp/bsp_lidar_linux.c $(LIDAR_APP)/lidar.h $(LIDAR_BSP)/bsp_lidar.h | output
	$(CC) $(CPPFLAGS) $(CFLAGS) src/main.c $(PC_MAP_SOURCES) $(LIDAR_APP)/lidar.c bsp/bsp_lidar_linux.c $(LDFLAGS) $(LDLIBS) -o $@

.PHONY: all test test-replay test-map test-stm32-bsp test-cans test-terminal
all: output/lidar-reader

output/test-lidar: tests/test_lidar.c $(LIDAR_APP)/lidar.c $(LIDAR_APP)/lidar.h $(LIDAR_BSP)/bsp_lidar.h | output
	$(CC) $(CPPFLAGS) $(CFLAGS) tests/test_lidar.c $(LIDAR_APP)/lidar.c $(LDFLAGS) $(LDLIBS) -o $@

test: output/test-lidar
	./output/test-lidar > output/parser-unit-test.txt
	cat output/parser-unit-test.txt

# Optional comparison using the saved hardware capture and its reference CSV.
test-replay: output/test-lidar
	./output/test-lidar output/lidar-raw-0deg.bin output/lidar-c-measurements.csv > output/parser-test.txt
	python3 tests/compare_capture.py

output:
	mkdir -p output

output/test-pc-map: tests/test_pc_map.c $(PC_MAP_SOURCES) $(PC_MAP_HEADERS) $(LIDAR_APP)/lidar.h | output
	$(CC) $(CPPFLAGS) -Isrc $(CFLAGS) tests/test_pc_map.c $(PC_MAP_SOURCES) $(LDFLAGS) -Wl,--wrap=clock_gettime $(LDLIBS) -o $@

test-map: output/test-pc-map
	LIDAR_MAP_RANGE_M=2 ./output/test-pc-map > output/map-unit-frames.txt
	python3 tests/check_pc_map.py

CAN_SOURCES = $(LIDAR_APP)/lidar_cans.c
CAN_HEADERS = $(LIDAR_APP)/lidar_cans.h $(LIDAR_APP)/lidar.h

output/test-stm32-bsp: tests/test_bsp_lidar_stm32.c tests/stm32_hal_mock/stm32l4xx_hal.h $(LIDAR_BSP)/bsp_lidar.c $(LIDAR_BSP)/bsp_lidar.h $(LIDAR_BSP)/bsp_lidar_motor.h $(LIDAR_APP)/lidar.c $(LIDAR_APP)/lidar_test.c $(LIDAR_APP)/lidar_test.h $(CAN_SOURCES) $(CAN_HEADERS) | output
	$(CC) $(CPPFLAGS) -Itests/stm32_hal_mock $(CFLAGS) tests/test_bsp_lidar_stm32.c $(LIDAR_APP)/lidar.c $(LIDAR_APP)/lidar_test.c $(CAN_SOURCES) $(LDFLAGS) $(LDLIBS) -o $@

test-stm32-bsp: output/test-stm32-bsp
	./output/test-stm32-bsp > output/stm32-bsp-test.txt
	cat output/stm32-bsp-test.txt

output/test-lidar-cans: tests/test_lidar_cans.c $(CAN_SOURCES) $(CAN_HEADERS) | output
	$(CC) $(CPPFLAGS) $(CFLAGS) tests/test_lidar_cans.c $(CAN_SOURCES) $(LDFLAGS) $(LDLIBS) -o $@

test-cans: output/test-lidar-cans
	./output/test-lidar-cans

output/test-lidar-terminal: tests/test_lidar_terminal.c $(LIDAR_APP)/lidar_terminal.c $(LIDAR_APP)/lidar_terminal.h $(LIDAR_APP)/lidar_test.h $(LIDAR_BSP)/bsp_lidar_terminal.h $(LIDAR_BSP)/bsp_lidar.h $(CAN_HEADERS) | output
	$(CC) $(CPPFLAGS) $(CFLAGS) tests/test_lidar_terminal.c $(LIDAR_APP)/lidar_terminal.c $(LDFLAGS) $(LDLIBS) -o $@

test-terminal: output/test-lidar-terminal
	./output/test-lidar-terminal

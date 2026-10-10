CC = cc
LIDAR_APP = ../STM32_implementation/App
LIDAR_BSP = ../STM32_implementation/BSP
CPPFLAGS += -I$(LIDAR_APP) -I$(LIDAR_BSP)
CFLAGS += -std=c11 -Wall -Wextra -Wpedantic -Werror -O2
LDLIBS += -lm

output/lidar-reader: src/main.c src/pc_map.c src/pc_map.h $(LIDAR_APP)/lidar.c bsp/bsp_lidar_linux.c $(LIDAR_APP)/lidar.h $(LIDAR_BSP)/bsp_lidar.h | output
	$(CC) $(CPPFLAGS) $(CFLAGS) src/main.c src/pc_map.c $(LIDAR_APP)/lidar.c bsp/bsp_lidar_linux.c $(LDFLAGS) $(LDLIBS) -o $@

.PHONY: all test test-replay test-map
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

output/test-pc-map: tests/test_pc_map.c src/pc_map.c src/pc_map.h $(LIDAR_APP)/lidar.h | output
	$(CC) $(CPPFLAGS) -Isrc $(CFLAGS) tests/test_pc_map.c src/pc_map.c $(LDFLAGS) -Wl,--wrap=clock_gettime $(LDLIBS) -o $@

test-map: output/test-pc-map
	LIDAR_MAP_RANGE_M=2 ./output/test-pc-map > output/map-unit-frames.txt
	python3 tests/check_pc_map.py

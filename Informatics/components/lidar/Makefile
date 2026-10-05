CC = cc
CPPFLAGS += -Iapp -Ibsp
CFLAGS += -std=c11 -Wall -Wextra -Wpedantic -Werror -O2
LDLIBS += -lm

output/lidar-reader: src/main.c src/pc_map.c src/pc_map.h app/lidar.c bsp/bsp.c app/lidar.h bsp/bsp.h | output
	$(CC) $(CPPFLAGS) $(CFLAGS) src/main.c src/pc_map.c app/lidar.c bsp/bsp.c $(LDFLAGS) $(LDLIBS) -o $@

.PHONY: all test test-replay test-map
all: output/lidar-reader

output/test-lidar: tests/test_lidar.c app/lidar.c app/lidar.h bsp/bsp.h | output
	$(CC) $(CPPFLAGS) $(CFLAGS) tests/test_lidar.c app/lidar.c $(LDFLAGS) $(LDLIBS) -o $@

test: output/test-lidar
	./output/test-lidar > output/parser-unit-test.txt
	cat output/parser-unit-test.txt

# Optional comparison using the saved hardware capture and its reference CSV.
test-replay: output/test-lidar
	./output/test-lidar output/lidar-raw-0deg.bin output/lidar-c-measurements.csv > output/parser-test.txt
	python3 tests/compare_capture.py

output:
	mkdir -p output

output/test-pc-map: tests/test_pc_map.c src/pc_map.c src/pc_map.h app/lidar.h | output
	$(CC) $(CPPFLAGS) -Isrc $(CFLAGS) tests/test_pc_map.c src/pc_map.c $(LDFLAGS) -Wl,--wrap=clock_gettime $(LDLIBS) -o $@

test-map: output/test-pc-map
	LIDAR_MAP_RANGE_M=2 ./output/test-pc-map > output/map-unit-frames.txt
	python3 tests/check_pc_map.py

# Host-side tests for the pure-C parts of the firmware.
#
#   make            run the parser tests for both targets
#   make test-stm32 NUCLEO-H753ZI parser tests only
#   make test-esp32 ESP32-S3 parser tests only
#
# The parsers do not touch any hardware, so the exact source that is compiled
# into the firmware is compiled and executed here on the development machine.
CC ?= cc
CFLAGS ?= -std=c11 -Wall -Wextra -Wshadow -Wundef -Wno-unused-parameter -O1

ESP32_SRC := ../firmware/main/position.c
STM32_SRC := ../firmware/stm32h753/src/position.c

ESP32_BIN := test_position
STM32_BIN := test_position_stm32

all: run

$(ESP32_BIN): $(ESP32_SRC) test_position.c ../firmware/main/position.h
	$(CC) $(CFLAGS) -I../firmware/main $(ESP32_SRC) test_position.c -o $(ESP32_BIN)

$(STM32_BIN): $(STM32_SRC) test_position_stm32.c ../firmware/stm32h753/src/position.h
	$(CC) $(CFLAGS) -I../firmware/stm32h753/src $(STM32_SRC) test_position_stm32.c -o $(STM32_BIN)

run: test-stm32 test-esp32

test-stm32: $(STM32_BIN)
	./$(STM32_BIN)

test-esp32: $(ESP32_BIN)
	./$(ESP32_BIN)

clean:
	rm -f $(ESP32_BIN) $(STM32_BIN)

.PHONY: all run test-stm32 test-esp32 clean

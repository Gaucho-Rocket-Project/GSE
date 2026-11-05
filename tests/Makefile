# Host-side tests for the pure-C parts of the firmware (no ESP32 needed).
CC ?= cc
CFLAGS ?= -std=c11 -Wall -Wextra -Wno-unused-parameter -O1

SRC := ../firmware/main/position.c
BIN := test_position

all: run

$(BIN): $(SRC) test_position.c ../firmware/main/position.h
	$(CC) $(CFLAGS) -I../firmware/main $(SRC) test_position.c -o $(BIN)

run: $(BIN)
	./$(BIN)

clean:
	rm -f $(BIN)

.PHONY: all run clean

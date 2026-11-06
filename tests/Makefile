CC      ?= gcc
CFLAGS  ?= -std=c11 -O2 -Wall -Wextra -Wpedantic -I../firmware/src/common
HDR      = ../firmware/src/common/gse_protocol.h
CORE     = ../firmware/src/common/gse_tx_core.c
COREHDR  = ../firmware/src/common/gse_tx_core.h ../firmware/src/common/gse_hal.h

all: test_protocol host_hal

test_protocol: test_protocol.c $(HDR)
	$(CC) $(CFLAGS) -o $@ test_protocol.c

# the TX core is compiled here exactly as the ESP32/STM32 builds compile it
host_hal: host_hal_stub.c $(CORE) $(COREHDR) $(HDR)
	$(CC) $(CFLAGS) -I../firmware/src/common -o $@ host_hal_stub.c $(CORE)

run: test_protocol host_hal
	./test_protocol
	./host_hal

clean:
	rm -f test_protocol host_hal
.PHONY: all run clean

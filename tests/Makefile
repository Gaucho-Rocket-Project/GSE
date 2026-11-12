CC      ?= gcc
CFLAGS  ?= -std=c11 -O2 -Wall -Wextra -Wpedantic -I../firmware/src/common
HDR      = ../firmware/src/common/gse_protocol.h
CORE     = ../firmware/src/common/gse_tx_core.c
COREHDR  = ../firmware/src/common/gse_tx_core.h ../firmware/src/common/gse_hal.h

all: test_protocol host_hal

# ESP32 board variants are compile-checked with the host compiler (no Espressif
# toolchain needed); see esp32_syntax_check.sh.
variants:
	sh esp32_syntax_check.sh

test_protocol: test_protocol.c $(HDR)
	$(CC) $(CFLAGS) -o $@ test_protocol.c

# the TX core is compiled here exactly as the ESP32/STM32 builds compile it
host_hal: host_hal_stub.c $(CORE) $(COREHDR) $(HDR)
	$(CC) $(CFLAGS) -I../firmware/src/common -o $@ host_hal_stub.c $(CORE)

# Keep transient integration binaries out of the tracked source tree.
integration:
	@set -e; tmp=$$(mktemp -d); trap 'rm -rf "$$tmp"' EXIT; \
	$(CC) $(CFLAGS) -c $(CORE) -o "$$tmp/core.o"; \
	for board in s3 esp32s placeholder; do \
	  flags=""; \
	  [ "$$board" != esp32s ] || flags="-D GSE_BOARD_ESP32S"; \
	  [ "$$board" != placeholder ] || flags="-D GSE_TEST_PLACEHOLDER"; \
	  echo "ESP-NOW pad/TX integration: $$board"; \
	  $(CXX) -std=gnu++17 -Wall -Wextra -Wno-unused-parameter -Wno-unused-variable \
	    -I esp32_stub -I ../firmware/src -I ../firmware/src/common $$flags \
	    test_espnow_controls.cpp "$$tmp/core.o" -o "$$tmp/test"; \
	  "$$tmp/test"; \
	done

run: test_protocol host_hal integration
	./test_protocol
	./host_hal
	$(MAKE) --no-print-directory variants

clean:
	rm -f test_protocol host_hal
.PHONY: all run clean variants integration

# ---------------------------------------------------------------------------
# ESP32-S3 dual SG90 servo test firmware - convenience wrapper around idf.py
#
#   make build        build (console on USB Serial/JTAG - default)
#   make build-uart   build with the console on UART0 (COM port) instead
#   make menuconfig   Kconfig UI for the default build
#   make flash        flash the default (USB) build
#   make flash-uart   flash the UART build
#   make monitor      serial monitor (Ctrl+] to quit)
#   make flash-monitor
#   make erase        erase the whole flash
#   make size         show binary sizes
#   make clean        remove build output
#   make distclean    remove build output and generated sdkconfig files
#
# Optional: PORT=/dev/ttyACM0  BAUD=460800
#
# You need ESP-IDF installed and loaded into your shell first, e.g.:
#   git clone -b v5.4.1 --recursive https://github.com/espressif/esp-idf.git
#   cd esp-idf && ./install.sh esp32s3 && . ./export.sh
# ---------------------------------------------------------------------------

IDF_PY  ?= idf.py
PORT    ?=
BAUD    ?= 460800

# Servo selection (see main/servo.h)
#   SERVO_TYPE=1  SG90 / micro analog            (default)
#   SERVO_TYPE=2  DS3225MG 25 kg digital
#   SERVO_TRAVEL_DEG=180 or 270 for the DS3225 variant you own
SERVO_TYPE       ?= 1
SERVO_TRAVEL_DEG ?=

BUILD_USB  := build
BUILD_UART := build-uart

CFG_USB    := $(CURDIR)/$(BUILD_USB)/sdkconfig
CFG_UART   := $(CURDIR)/$(BUILD_UART)/sdkconfig

# Each variant keeps its own sdkconfig inside its build directory, so the two
# builds never contaminate each other. (SDKCONFIG is a CMake cache variable,
# hence -D; an environment variable would be silently ignored.)
# CMake cache variables keep their first value, so force them on every run.
SERVO_ARGS := -D SERVO_TYPE=$(SERVO_TYPE) -D SERVO_TRAVEL_DEG=$(SERVO_TRAVEL_DEG)

USB_ARGS   := -B $(BUILD_USB)  -D SDKCONFIG=$(CFG_USB)  -D SDKCONFIG_DEFAULTS="sdkconfig.defaults" $(SERVO_ARGS)
UART_ARGS  := -B $(BUILD_UART) -D SDKCONFIG=$(CFG_UART) -D SDKCONFIG_DEFAULTS="sdkconfig.defaults.uart" $(SERVO_ARGS)

PORT_ARG    = $(if $(PORT),-p $(PORT),)

.PHONY: all build build-uart menuconfig flash flash-uart monitor flash-monitor \
        monitor-uart erase size servo-type test clean distclean help

all: build

build:
	$(IDF_PY) $(USB_ARGS) build

build-uart:
	$(IDF_PY) $(UART_ARGS) build

menuconfig:
	$(IDF_PY) $(USB_ARGS) menuconfig

flash:
	$(IDF_PY) $(USB_ARGS) $(PORT_ARG) -b $(BAUD) flash

flash-uart:
	$(IDF_PY) $(UART_ARGS) $(PORT_ARG) -b $(BAUD) flash

monitor:
	$(IDF_PY) -B $(BUILD_USB) $(PORT_ARG) monitor

monitor-uart:
	$(IDF_PY) -B $(BUILD_UART) $(PORT_ARG) monitor

flash-monitor: flash
	$(IDF_PY) -B $(BUILD_USB) $(PORT_ARG) monitor

erase:
	$(IDF_PY) $(USB_ARGS) $(PORT_ARG) erase-flash

size:
	$(IDF_PY) -B $(BUILD_USB) size

# Which servo type is compiled into this build?
servo-type:
	@echo "SERVO_TYPE=$(SERVO_TYPE)  SERVO_TRAVEL_DEG=$(if $(SERVO_TRAVEL_DEG),$(SERVO_TRAVEL_DEG),<type default>)"

test:
	$(MAKE) -C tests

clean:
	$(IDF_PY) -B $(BUILD_USB) clean
	$(IDF_PY) -B $(BUILD_UART) clean

distclean:
	rm -rf $(BUILD_USB) $(BUILD_UART) sdkconfig sdkconfig.old
	$(MAKE) -C tests clean

help:
	@sed -n '2,25p' Makefile

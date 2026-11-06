# ---------------------------------------------------------------------------
# NUCLEO-H753ZI dual servo firmware - build / verify / flash / monitor
#
#   make                 build out/servo.elf, out/servo.bin, out/servo.hex
#   make flash           program over ST-LINK (CubeProgrammer or OpenOCD)
#   make monitor         open the ST-LINK virtual COM port at 115200
#   make ports           list serial ports, marking the NUCLEO's ST-LINK one
#   make doctor          diagnose "the port will not open" / silent console
#   make test            run the position-parser unit tests on this machine
#   make clean
#
# Toolchain: arm-none-eabi-gcc, from any of
#   - Arm GNU Toolchain / xPack (arm-none-eabi-gcc-*)
#   - STM32CubeCLT or STM32CubeIDE (ST supplies arm-none-eabi-gcc)
#   - Debian/Ubuntu: sudo apt install gcc-arm-none-eabi
# Point at it explicitly if it is not on PATH:
#   make CROSS=/opt/arm/bin/arm-none-eabi-
#
# Options:
#   make TRAVEL=270              270 deg DS3225 variant (default 180)
#   make MIN_US=700 MAX_US=2300  narrower travel for small / buzzing servos
# ---------------------------------------------------------------------------

CROSS   ?= arm-none-eabi-

# Where the toolchain might be, in order of preference.
TOOLCHAIN_CANDIDATES := $(wildcard $(HOME)/.local/xpack-arm-none-eabi-gcc*/bin) \
                        $(wildcard /opt/xpack-arm-none-eabi-gcc*/bin) \
                        $(wildcard /opt/arm-gnu-toolchain*/bin) \
                        $(wildcard /tmp/tc/xpack-arm-none-eabi-gcc-*/bin)

ifneq ($(shell command -v $(CROSS)gcc 2>/dev/null),)
CC      := $(CROSS)gcc
OBJCOPY := $(CROSS)objcopy
SIZE    := $(CROSS)size
else ifneq ($(firstword $(TOOLCHAIN_CANDIDATES)),)
CC      := $(firstword $(TOOLCHAIN_CANDIDATES))/arm-none-eabi-gcc
OBJCOPY := $(firstword $(TOOLCHAIN_CANDIDATES))/arm-none-eabi-objcopy
SIZE    := $(firstword $(TOOLCHAIN_CANDIDATES))/arm-none-eabi-size
else
CC      := $(CROSS)gcc
OBJCOPY := $(CROSS)objcopy
SIZE    := $(CROSS)size
endif

TARGET  := servo
BUILD   := out
SRC     := src

TRAVEL  ?= 180
MIN_US  ?= 500
MAX_US  ?= 2500

CFLAGS  := -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard \
           -O2 -g -std=c11 -Wall -Wextra -Wshadow -Wundef \
           -ffunction-sections -fdata-sections -fno-common \
           -ffreestanding -fno-builtin -nostdlib \
           -DSERVO_TRAVEL_DEG=$(TRAVEL) \
           -DSERVO_MIN_PULSE_US=$(MIN_US) \
           -DSERVO_MAX_PULSE_US=$(MAX_US) \
           -I$(SRC)

LDFLAGS := -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard \
           -nostdlib -Wl,-T,link.ld -Wl,-Map,$(BUILD)/$(TARGET).map \
           -Wl,--gc-sections -Wl,--print-memory-usage -Wl,-e,Reset_Handler

SRCS := $(sort $(wildcard $(SRC)/*.c))
OBJS := $(patsubst $(SRC)/%.c,$(BUILD)/%.o,$(SRCS))
DEPS := $(OBJS:.o=.d)

# ---------------------------------------------------------------------------
.PHONY: all build clean flash flash-bin programmer probe erase monitor list-ports ports doctor sniff send test verify help

all: build

build: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).bin $(BUILD)/$(TARGET).hex

$(BUILD):
	@mkdir -p $(BUILD)

$(BUILD)/%.o: $(SRC)/%.c | $(BUILD)
	@echo "  CC  $<"
	@$(CC) $(CFLAGS) -MMD -MP -c $< -o $@

$(BUILD)/$(TARGET).elf: $(OBJS) link.ld | $(BUILD)
	@echo "  LD  $@"
	@$(CC) $(OBJS) $(LDFLAGS) -o $@
	@$(SIZE) $@

$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	@$(OBJCOPY) -O binary $< $@
	@echo "  BIN $@ ($$(stat -c%s $@ 2>/dev/null || wc -c <$@) bytes)"

$(BUILD)/$(TARGET).hex: $(BUILD)/$(TARGET).elf
	@$(OBJCOPY) -O ihex $< $@
	@echo "  HEX $@"

size: $(BUILD)/$(TARGET).elf
	@$(SIZE) -A -x $<

# --- flash ------------------------------------------------------------------
# tools/flash.sh picks whichever programmer is installed (STM32CubeProgrammer,
# OpenOCD, or the board's drag-and-drop drive) and explains itself if none is.
.PHONY: flash flash-bin programmer probe erase

flash:
	@./tools/flash.sh

flash-bin: $(BUILD)/$(TARGET).bin
	@./tools/flash.sh $(BUILD)/$(TARGET).bin

# Same thing with STM32CubeProgrammer explicitly.
programmer:
	STM32_Programmer_CLI -c port=SWD -w $(BUILD)/$(TARGET).elf -s -rst

# Talk to the ST-LINK without writing anything (sanity check).
probe:
	@./tools/flash.sh --probe

# Mass erase, then flash.
erase:
	@./tools/flash.sh --erase

# --- console ----------------------------------------------------------------
# The NUCLEO's ST-LINK exposes USART3 as a USB CDC-ACM port (ttyACM* / COMx).
# tools/monitor.py finds it by USB IDs (so it will not grab the ESP32-S3 board,
# which also shows up as a ttyACM port), checks that the board answers, and
# tells you what to do if it does not.
PORT ?=
monitor:
	@if [ -n "$(PORT)" ]; then ./tools/monitor.py $(PORT); else ./tools/monitor.py; fi

list-ports:
	@./tools/find_port.py --list

ports: list-ports

# Full diagnosis: terminal, permissions, brltty/ModemManager/stale screen,
# every serial port with its USB IDs, who holds it, and the exact errno.
doctor:
	@./tools/doctor.py $(PORT)

# Dump whatever arrives for N seconds (does not open an interactive session).
sniff:
	@./tools/monitor.py --sniff $(or $(SECS),2)

# Send one command and print the reply.
send:
	@./tools/monitor.py --send '$(CMD)\r' --sniff 1

# --- verification without an ARM toolchain ----------------------------------
# Compiles the whole firmware as 32-bit ARM with the host gcc if it has an
# arm cross target, else at least type-checks the portable parts and runs the
# parser unit tests.
test:
	$(MAKE) -C ../../tests test-stm32

verify:
	@$(MAKE) -C ../../tests test-stm32
	@echo
	@echo "Toolchain check:"
	@command -v $(CC) >/dev/null 2>&1 \
	    && echo "  $(CC) found - 'make build' will produce out/servo.bin" \
	    || echo "  $(CC) NOT found - install arm-none-eabi-gcc to build the image"

clean:
	rm -rf $(BUILD)

help:
	@sed -n '2,24p' Makefile

-include $(DEPS)

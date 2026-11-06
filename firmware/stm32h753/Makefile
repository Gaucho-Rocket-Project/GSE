# ---------------------------------------------------------------------------
# NUCLEO-H753ZI dual servo firmware - build / verify / flash / monitor
#
#   make                 build out/servo.elf, out/servo.bin, out/servo.hex
#   make flash           program over ST-LINK with OpenOCD
#   make monitor         open the ST-LINK virtual COM port at 115200
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
.PHONY: all build clean flash flash-bin programmer monitor list-ports test verify help

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
# OpenOCD ships with STM32CubeCLT, STM32CubeIDE and most distro packages, and
# talks to the on-board ST-LINK: no external programmer needed.
OPENOCD     ?= openocd
OOCD_ARGS   ?= -f interface/stlink.cfg -f target/stm32h7x.cfg

flash: $(BUILD)/$(TARGET).elf
	@echo "  FLASH $< -> STM32H753 (ST-LINK)"
	$(OPENOCD) $(OOCD_ARGS) -c "program $< verify reset exit"

flash-bin: $(BUILD)/$(TARGET).bin
	$(OPENOCD) $(OOCD_ARGS) -c "program $< 0x08000000 verify reset exit"

# Same thing with STM32CubeProgrammer instead of OpenOCD.
programmer:
	STM32_Programmer_CLI -c port=SWD -w $(BUILD)/$(TARGET).elf -s -rst

# --- console ----------------------------------------------------------------
# The NUCLEO's ST-LINK exposes USART3 as a USB CDC-ACM port (ttyACM* / COMx).
PORT ?= $(shell ls /dev/ttyACM* 2>/dev/null | head -1)
monitor:
	@test -n "$(PORT)" || { echo "No /dev/ttyACM* found - pass PORT=/dev/ttyACM0"; exit 1; }
	@echo "Opening $(PORT) @ 115200 (quit: Ctrl-A then K)"
	screen $(PORT) 115200

list-ports:
	@ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "no serial ports found"

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

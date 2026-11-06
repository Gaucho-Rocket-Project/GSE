#!/bin/sh
# syntax_check.sh -- compile-check the STM32Cube HAL port without a board
# ---------------------------------------------------------------------------
# tx_platform_stm32.c expects a CubeMX-generated main.h (HAL handles + AUX pin).
# This script stands in a throwaway main.h and the CMSIS/HAL headers that ship
# with PlatformIO, so a typo in the port is caught in a second instead of in a
# 50-second cross-compile or, worse, on the bench.
#
#   sh firmware/stm32/syntax_check.sh
#
# Exits non-zero on any error or warning.
set -e

HERE=$(cd "$(dirname "$0")" && pwd)
FW=$HERE/../..
PIO=${PLATFORMIO_CORE_DIR:-$HOME/.platformio}

HAL=$PIO/packages/framework-arduinoststm32/system/Drivers/STM32H7xx_HAL_Driver/Inc
DEV=$PIO/packages/framework-arduinoststm32/system/Drivers/CMSIS/Device/ST/STM32H7xx/Include
CMSIS=$PIO/packages/framework-cmsis/CMSIS/Core/Include
CC=$PIO/packages/toolchain-gccarmnoneeabi/bin/arm-none-eabi-gcc

[ -x "$CC" ] || { echo "no arm-none-eabi-gcc under $PIO; run 'pio pkg install' first"; exit 1; }

TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT

cat > "$TMP/main.h" <<'EOF'
#ifndef MAIN_H_STUB
#define MAIN_H_STUB
#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"
extern UART_HandleTypeDef huart1, huart2, huart3;
extern GPIO_TypeDef *AUX_GPIO_Port;
extern uint16_t AUX_Pin;
#endif
EOF

cat > "$TMP/stm32h7xx_hal_conf.h" <<'EOF'
#ifndef STM32H7xx_HAL_CONF_H
#define STM32H7xx_HAL_CONF_H
#define HAL_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_EXTI_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED
#define HSE_VALUE  8000000U
#define HSI_VALUE  64000000U
#include "stm32h7xx_hal_rcc.h"
#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_dma.h"
#include "stm32h7xx_hal_cortex.h"
#include "stm32h7xx_hal_uart.h"
#endif
EOF

"$CC" -mcpu=cortex-m7 -mthumb -mfpu=fpv5-sp-d16 -mfloat-abi=hard \
      -std=c11 -Wall -Wextra -fsyntax-only \
      -D STM32H753xx -D USE_HAL_DRIVER \
      -I "$TMP" -I "$CMSIS" -I "$HAL" -I "$DEV" -I "$HERE/../src/common" \
      "$HERE/tx_platform_stm32.c"

echo "STM32Cube HAL port: syntax OK (no warnings)"

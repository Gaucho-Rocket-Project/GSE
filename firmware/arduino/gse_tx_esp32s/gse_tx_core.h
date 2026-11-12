/*
 * gse_tx_core.h  --  API of the portable TX-node / laptop-bridge core
 *
 * The core never touches hardware.  A platform file supplies the five HAL
 * functions declared in gse_hal.h, calls gse_tx_init() once and gse_tx_tick()
 * from its main loop (100 Hz - 1 kHz is fine; the timing inside the core is
 * all millisecond based).
 *
 * Two complete platform implementations ship with this package:
 *      ESP32-S3 : firmware/src/tx/tx_platform_esp32.cpp  (Arduino)
 *      STM32H753: firmware/stm32/tx_platform_stm32.c     (STM32Cube HAL)
 *
 * Host-side unit testing is possible by linking this file against a stub HAL
 * (see tests/host_hal_stub.c) - the protocol tests already run that way.
 */
#ifndef GSE_TX_CORE_H
#define GSE_TX_CORE_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "gse_protocol.h"
#include "gse_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* The platform functions the core calls are declared in gse_hal.h. */

/* ---------------------------------------------------------------- stats */
typedef struct {
    uint32_t sent_frames;
    uint32_t safe_frames;
    uint32_t forwarded_cmd;
    uint32_t dropped_stale;
    uint32_t rx_state;
    uint32_t rx_event;
    uint32_t rx_frames;      /* every frame off the air, any type            */
    uint32_t bad_host_frames;
    bool     radio_present;
    uint8_t  level;
} gse_tx_stats_t;

/* ------------------------------------------------------------------ API */
void  gse_tx_init(void);
void  gse_tx_tick(void);
const gse_tx_stats_t *gse_tx_stats(void);

#ifdef __cplusplus
}
#endif
#endif /* GSE_TX_CORE_H */

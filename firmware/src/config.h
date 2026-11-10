/*
 * config.h  --  shared build configuration for both GSE radio nodes
 *
 *  Firmware:  ESP32-S3 (any devkit with native USB, e.g. ESP32-S3-DevKitC-1
 *             or the 8 MB "SuperMini"/N16R8 boards) + DX-LR41-24T12S
 *             (Semtech SX1281, 2.4 GHz LoRa/FLRC, UART "transparent" module)
 *
 *  Two environments live in this one PlatformIO project:
 *      pio run -e tx-node      # laptop-side radio bridge (USB-C -> LoRa)
 *      pio run -e pad-node     # launch-pad controller (LoRa -> servos + IGN)
 *
 *  Everything a builder is likely to change lives here.
 */
#ifndef GSE_CONFIG_H
#define GSE_CONFIG_H

#include "gse_protocol.h"

/* ------------------------------------------------------------- node identity */
//#define GSE_ROLE_TX         /* defined by platformio.ini for tx-node   */
//#define GSE_ROLE_PAD        /* defined by platformio.ini for pad-node  */

#ifndef GSE_ROLE_TX
#ifndef GSE_ROLE_PAD
#error "Define GSE_ROLE_TX or GSE_ROLE_PAD (platformio.ini does this)"
#endif
#endif

#define GSE_FW_MAJOR  1
#define GSE_FW_MINOR  0

/* ------------------------------------------------------------------- timings */
#define GSE_TICK_MS           10u     /* main loop period                  */
#ifdef GSE_ROLE_TX
#  define GSE_CMD_PERIOD_MS   50u     /* 20 Hz command frames              */
#  define GSE_STATE_TIMEOUT_MS 1500u  /* laptop warns after this           */
#else
#  define GSE_STATE_PERIOD_MS 100u    /* 10 Hz state frames back to laptop */
#endif

/* Serial link to the DX-LR41 module (its factory default is 9600 8N1)      */
#define DXLR41_BAUD           9600u

/* ------------------------------------------------------- radio provisioning */
/* Both nodes must agree; these are written into the modules at boot.        */
#define GSE_NODE_TX           1u
#define GSE_NODE_PAD          2u
#define GSE_ADDR_TX           0x0001u   /* fixed-point address of the TX end  */
#define GSE_ADDR_PAD          0x0002u   /* fixed-point address of the pad end */
/* Two-channel scheme (recommended).  Each module LISTENS on its own channel
 * and TRANSMITS on the peer's channel using the module's fixed-point mode
 * ("dst address + dst channel + data"), which removes the half-duplex
 * collision between the 20 Hz command stream and the state stream.
 * If the bench test shows your firmware revision ignores the destination
 * channel, build both nodes with -DGSE_SINGLE_CHANNEL and set the state rate
 * to 5 Hz (see docs/TEST_PLAN.md T-07).                                        */
#define GSE_CH_UP             0x0Cu     /* 2424 MHz : pad listens, TX sends   */
#define GSE_CH_DOWN           0x0Eu     /* 2428 MHz : TX listens, pad sends   */
#ifdef GSE_SINGLE_CHANNEL
#  define GSE_CH_SELF         GSE_CH_UP
#  define GSE_CH_PEER         GSE_CH_UP
#else
#  define GSE_CH_SELF(TX)     ((TX) ? GSE_CH_DOWN : GSE_CH_UP)
#  define GSE_CH_PEER(TX)     ((TX) ? GSE_CH_UP   : GSE_CH_DOWN)
#endif
#define GSE_LEVEL_DEFAULT     3u        /* 10 kbit/s LoRa, ~2.2 km LOS        */
#define GSE_LEVEL_FAST        8u        /* FLRC 1.04 Mbit/s, >=6 B payloads   */
#define GSE_KEY_WORD          51042u    /* AT+KEY (0..65535); not security    */

/* ------------------------------------------------------- pad PWM / relay pins */
/*
 * Servo channels replicate the FS-iA10B mapping documented in HCR-5100
 * OP 6 step 9 / 10 so that a stock GSE can be converted without rewiring:
 *      FILL valve      "Ch5"   (SwB)
 *      VEHICLE valves  "Ch6"   (SwD)
 *      PURGE valve     "Ch8"   (SwA)
 *      IGNITER relay   "Ch3"   (SwC + left stick lockout)
 */
#define PIN_SERVO_FILL        4
#define PIN_SERVO_VEH         5
#define PIN_SERVO_PRG         6
#define PIN_RELAY_IGN         7
#define PIN_HB                8       /* 50 Hz heartbeat -> hw keep-alive timer */
#define PIN_STATUS_RGB       48       /* WS2812 on most S3 devkits; -1 = none */
#define PIN_BUZZER           -1       /* optional active buzzer               */

/* DX-LR41 wiring on the pad node (UART1) */
#define PIN_LORA_RX          16       /* ESP32 RX  <- module TXD  */
#define PIN_LORA_TX          17       /* ESP32 TX  -> module RXD  */
#define PIN_LORA_AUX         18       /* module AUX (high = busy) */
#define PIN_LORA_M0          -1       /* tie low on the PCB, AT+SWITCH=0  */
#define PIN_LORA_M1          -1

/* Servo pulse limits (DS3225-class servo). 1000/2000 us = +/-45 deg = the
 * mechanical 90 deg throw of the ball valve handle.                          */
// #define SERVO_MIN_US          1000
// #define SERVO_MAX_US          2000
/* DS3225, 180-degree version: 500..2500 us = 0..180 deg, 1500 us = centre,
 * i.e. 500 us per 45 deg.  The scale is fixed here, separately from the clamp,
 * so that "travel" is real degrees each side of centre: 45 -> 1000/2000 us
 * (unchanged), 90 -> 500/2500 us (full 180-degree range).  The clamp is only a
 * safety limit at the servo's own pulse range.                               */
#define SERVO_US_PER_45DEG    500
#define SERVO_MIN_US          500
#define SERVO_MAX_US          2500
#define SERVO_NEUTRAL_US      1500
#define SERVO_REFRESH_MS      20      /* 50 Hz                              */
#define SERVO_DEFAULT_TRAVEL  45       /* deg, each side of neutral          */
#define SERVO_MIN_TRAVEL      5
#define SERVO_MAX_TRAVEL      90

/* ----------------------------------------------------------------- failsafe */
#define GSE_WDG_DS_DEFAULT        GSE_DEF_WDG_DS      /* 1.0 s  */
#define GSE_WDG_DS_MIN            3
#define GSE_WDG_DS_MAX            GSE_CFG_LIMIT_WDG_DS/* 3.0 s  */
#define GSE_IGN_MAX_DS_DEFAULT    GSE_DEF_IGN_MAX_DS  /* 0.5 s  */
#define GSE_IGN_MAX_DS_MIN        1
#define GSE_IGN_MAX_DS_MAX        GSE_CFG_LIMIT_IGN_DS/* 2.0 s  */

/* CO2 purge valve must never be opened while the rocket is connected
 * unless the user explicitly enables it in the field.                      */
#define GSE_ALLOW_PURGE_DEFAULT   1

/* Igniter "dead-man" hold: the GUI must re-assert ign=1 at >=5 Hz; anything
 * slower than this many consecutive missing assertions cuts the relay.      */
#define GSE_IGN_HOLD_FRAMES       3

/* ------------------------------------------------------------------ wiring */
#ifdef GSE_ROLE_TX
#  define PIN_LORA_RX   16
#  define PIN_LORA_TX   17
#  define PIN_LORA_AUX  18
#  define PIN_LORA_M0   -1
#  define PIN_LORA_M1   -1
#endif

/* Compile-time guard rails --------------------------------------------------*/
#ifndef GSE_ROLE_TX
static_assert(GSE_WDG_DS_MAX * 100u < 0xFFFFu, "age_ms must fit in uint16");
#endif

#endif /* GSE_CONFIG_H */

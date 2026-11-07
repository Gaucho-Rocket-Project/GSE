/*
 * config.h  --  shared build configuration for both GSE radio nodes
 *
 *  Firmware:  an ESP32 devkit + DX-LR41-24T12S
 *             (Semtech SX1281, 2.4 GHz LoRa/FLRC, UART "transparent" module)
 *
 *  Two board families are supported, selected by the build (platformio.ini
 *  does it for you -- see the "board variant" block below):
 *
 *      ESP32-S3   (default)  ESP32-S3-DevKitC-1 / N16R8 "SuperMini".
 *                            Native USB-CDC to the laptop, WS2812 status LED.
 *      ESP-32S               "ESP-32S Development Board 2.4 GHz Dual-Mode
 *                            WiFi+Bluetooth" -- the classic 30/38-pin
 *                            ESP32-WROOM-32 devkit.  CP2102/CH340 USB-UART
 *                            bridge instead of native USB, no addressable LED,
 *                            and GPIO6..11 do not exist (SPI flash).
 *
 *  Five environments live in this one PlatformIO project:
 *      pio run -e tx-node          # laptop-side radio bridge (USB -> LoRa)
 *      pio run -e pad-node         # launch-pad controller (LoRa -> servos+IGN)
 *      pio run -e tx-node-esp32s   # the same, on an ESP-32S board
 *      pio run -e pad-node-esp32s  # the same, on an ESP-32S board
 *      pio run -e tx-nucleo        # laptop-side bridge on a NUCLEO-H753ZI
 *
 *  Everything a builder is likely to change lives here.
 */
#ifndef GSE_CONFIG_H
#define GSE_CONFIG_H

#include <assert.h>          /* static_assert when this is compiled as C     */
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

/* ----------------------------------------------------------- board variant
 *
 * GSE_BOARD_ESP32_S3 and GSE_BOARD_ESP32S are *optional build flags*.  With
 * neither of them defined the firmware targets the ESP32-S3 devkit it has
 * always targeted, so every existing command keeps working unchanged:
 *
 *      pio run -e tx-node          # ESP32-S3   (default)
 *      pio run -e pad-node         # ESP32-S3   (default)
 *      pio run -e tx-node-esp32s   # ESP-32S    (-D GSE_BOARD_ESP32S)
 *      pio run -e pad-node-esp32s  # ESP-32S    (-D GSE_BOARD_ESP32S)
 *
 * You can also flip a single environment without editing platformio.ini:
 *
 *      pio run -e pad-node -e ... -D GSE_BOARD_ESP32S
 *      PLATFORMIO_BUILD_FLAGS="-D GSE_BOARD_ESP32S" pio run -e tx-node
 *
 * or, for the Arduino IDE, uncomment exactly one line below.
 *
 * What actually changes is only pin selection and the USB-CDC/LED plumbing --
 * the protocol, the failsafes, the watchdog and the GUI are identical, and a
 * pad node on either board talks to a TX node on the other one.
 */
//#define GSE_BOARD_ESP32_S3            /* default: ESP32-S3-DevKitC-1        */
//#define GSE_BOARD_ESP32S              /* ESP-32S / ESP32-WROOM-32 devkit    */

#if defined(GSE_BOARD_ESP32_S3) && defined(GSE_BOARD_ESP32S)
#error "Pick one board variant: GSE_BOARD_ESP32_S3 or GSE_BOARD_ESP32S, not both"
#endif
#if defined(GSE_BOARD_ESP32S) && defined(GSE_PLATFORM_STM32)
#error "GSE_BOARD_ESP32S is an ESP32 flag; the Nucleo build is GSE_PLATFORM_STM32"
#endif
#if !defined(GSE_BOARD_ESP32_S3) && !defined(GSE_BOARD_ESP32S) && \
    !defined(GSE_PLATFORM_STM32)
#define GSE_BOARD_ESP32_S3            /* neither flag given -> the default  */
#endif

#if defined(GSE_PLATFORM_STM32)
#  define GSE_BOARD_NAME        "stm32"
#  define GSE_HOST_IS_BRIDGE    1     /* ST-LINK virtual COM port (USART3)     */
#  define GSE_HAS_RGB_LED       0     /* the Nucleo has three plain LEDs       */
#elif defined(GSE_BOARD_ESP32S)
#  define GSE_BOARD_NAME        "esp32s"
   /* The "USB socket" is a CP2102/CH340 bridge on UART0, not the chip's own
    * USB peripheral, and there is no WS2812 on the board. */
#  define GSE_HOST_IS_BRIDGE    1
#  define GSE_HAS_RGB_LED       0
#else
#  define GSE_BOARD_NAME        "esp32s3"
#  define GSE_HOST_IS_BRIDGE    0
#  define GSE_HAS_RGB_LED       1
#endif

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
 *
 * The pin numbers are the *GPIO* numbers, not the silkscreen "D4/D5/..." of a
 * particular board, and they are deliberately identical on the two boards:
 *
 *      function          GPIO   ESP32-S3 devkit   ESP-32S devkit
 *      FILL servo         4     D4 / GPIO4         D4  (IO4)
 *      VEHICLE servo      5     D5 / GPIO5         D5  (IO5)
 *      PURGE servo        6     GPIO6              D15 (IO15)  <-- see below
 *      IGNITER relay      7     GPIO7              D13 (IO13)
 *      heartbeat          8     GPIO8              D33 (IO33)
 *      radio RX          16     GPIO16             RX2 (IO16)
 *      radio TX          17     GPIO17             TX2 (IO17)
 *      radio AUX         18     GPIO18             D18 (IO18)
 *
 * "See below": GPIO6..11 are the SPI flash lines on every ESP32-WROOM-32 and
 * are NOT available.  An S3 devkit happily drives a pad node from GPIO6, an
 * ESP-32S does not, so the ESP-32S build shifts IGN to D13, PURGE to D15 and
 * the keep-alive to D33.  None of the three is a flash pin, an input-only pin
 * or a strapping pin, and only D15 has a boot-time side effect (the ROM boot
 * log) -- which is why the relay is on D13 and D15 drives a servo.  The S3
 * build keeps its documented D4..D8 wiring, and the ESP-32S build uses nearby
 * header pins: relabel the harness, do not rewrite it.
 *
 * Which ESP-32S pins were deliberately NOT used, and why -- this matters most
 * for the igniter, and it is why the igniter is on D13 rather than the "next
 * pin along":
 *
 *   GPIO0, 2, 12, 15   strapping pins: pulled the wrong way during reset, and
 *                      GPIO15 also emits the ROM boot log on UART0.
 *   GPIO5, 14          emit a short PWM burst at boot (GPIO5 = strapping, the
 *                      14 burst is a known ESP32 reset artefact).  Harmless on
 *                      a servo (a DS3225 holds position without a valid pulse
 *                      train) -- never acceptable on the relay gate.
 *   GPIO1, 3           UART0 TX/RX, soldered to the USB bridge chip.
 *   GPIO6..11          SPI flash.
 *   GPIO34..39         input-only.
 *
 * D13 has no boot function, no strap and no reset artefact, which is exactly
 * what an igniter gate wants.  Even so, the relay can only fire with Q1
 * conducting (25 Hz keep-alive on D33) *and* SW2 closed, so a boot glitch on
 * any pin still cannot reach the e-match: that is hazard H4 in the ECR, and
 * the pin choice is the third layer of its defence, not the first.
 */
#if defined(GSE_BOARD_ESP32S)
 /* Names of the sibling pins *before* the flash-pin guard below, so that a
  * bad value is reported as "GPIO6..11 are flash" rather than as a redefinition
  * error that says nothing useful.  GSE_PIN_* are the values actually used. */
#  ifndef GSE_PIN_SERVO_FILL
#    define GSE_PIN_SERVO_FILL   4       /* D4  */
#  endif
#  ifndef GSE_PIN_SERVO_VEH
#    define GSE_PIN_SERVO_VEH    5       /* D5  */
#  endif
#  ifndef GSE_PIN_SERVO_PRG
#    define GSE_PIN_SERVO_PRG   15       /* D15 -- GPIO6 is flash on a WROOM-32 */
#  endif
#  ifndef GSE_PIN_RELAY_IGN
#    define GSE_PIN_RELAY_IGN   13       /* D13 -- GPIO7 is flash; D14 boots with
                                          * a PWM burst, D15 is a strapping pin
                                          * that also emits the boot log       */
#  endif
#  ifndef GSE_PIN_HB
#    define GSE_PIN_HB          33       /* D33 -- GPIO8 is flash on a WROOM-32 */
#  endif
#  define PIN_STATUS_RGB      -1         /* no WS2812 on the ESP-32S board      */
#  define PIN_STATUS_LED       2         /* on-board blue LED, D2               */
   /* a few ESP-32S clones wire that LED the other way round; if your LED is
    * on when it should be off, define GSE_STATUS_LED_ACTIVE_LOW (or -D it)  */
#  ifndef GSE_STATUS_LED_ACTIVE_LOW
#    define GSE_STATUS_LED_ACTIVE_LOW 0
#  endif
#else
#  ifndef GSE_PIN_SERVO_FILL
#    define GSE_PIN_SERVO_FILL   4
#  endif
#  ifndef GSE_PIN_SERVO_VEH
#    define GSE_PIN_SERVO_VEH    5
#  endif
#  ifndef GSE_PIN_SERVO_PRG
#    define GSE_PIN_SERVO_PRG    6
#  endif
#  ifndef GSE_PIN_RELAY_IGN
#    define GSE_PIN_RELAY_IGN    7
#  endif
#  ifndef GSE_PIN_HB
#    define GSE_PIN_HB           8       /* 50 Hz heartbeat -> hw keep-alive    */
#  endif
#  define PIN_STATUS_RGB      48         /* WS2812 on most S3 devkits; -1 = none*/
#  define PIN_STATUS_LED      -1         /* S3 devkits have no plain LED        */
#endif

/* The mistakes that would otherwise be found on the bench:
 *   - a servo / relay / heartbeat pin on GPIO6..11, which is the SPI flash on
 *     an ESP32-WROOM-32 (an ESP32-S3 devkit can drive GPIO6, which is why the
 *     S3 build keeps it -- this check is therefore ESP-32S-only);
 *   - a servo / relay / heartbeat pin on GPIO34..39, which are input-only;
 *   - the igniter relay on a strapping pin.  Those are pulled the "wrong" way
 *     during reset, and a glitch on the relay gate at power-up is hazard H4 in
 *     the ECR.  On the S3 GPIO0/3/45/46 strap; on the ESP32 GPIO0/2/12/15.
 *     PURGE may sit on D15 because a servo shrugs off a boot glitch; the relay
 *     may not, and the build enforces that.
 * The check fires on the pin you would actually wire, and GSE_PIN_* can be
 * overridden from the command line (that is what the syntax check does).   */
#if defined(GSE_ROLE_PAD) && defined(GSE_BOARD_ESP32S)
static_assert(!((GSE_PIN_SERVO_FILL >= 6 && GSE_PIN_SERVO_FILL <= 11)),
              "FILL servo on GPIO6..11: SPI flash on an ESP32-WROOM-32");
static_assert(!((GSE_PIN_SERVO_VEH  >= 6 && GSE_PIN_SERVO_VEH  <= 11)),
              "VEHICLE servo on GPIO6..11: SPI flash on an ESP32-WROOM-32");
static_assert(!((GSE_PIN_SERVO_PRG  >= 6 && GSE_PIN_SERVO_PRG  <= 11)),
              "PURGE servo on GPIO6..11: SPI flash on an ESP32-WROOM-32");
static_assert(!((GSE_PIN_RELAY_IGN  >= 6 && GSE_PIN_RELAY_IGN  <= 11)),
              "igniter relay on GPIO6..11: SPI flash on an ESP32-WROOM-32");
static_assert(!((GSE_PIN_HB         >= 6 && GSE_PIN_HB         <= 11)),
              "heartbeat on GPIO6..11: SPI flash on an ESP32-WROOM-32");
#endif
#if defined(GSE_ROLE_PAD)
static_assert(!(GSE_PIN_SERVO_FILL >= 34 && GSE_PIN_SERVO_FILL <= 39),
              "FILL servo on GPIO34..39: input-only pins");
static_assert(!(GSE_PIN_SERVO_VEH  >= 34 && GSE_PIN_SERVO_VEH  <= 39),
              "VEHICLE servo on GPIO34..39: input-only pins");
static_assert(!(GSE_PIN_SERVO_PRG  >= 34 && GSE_PIN_SERVO_PRG  <= 39),
              "PURGE servo on GPIO34..39: input-only pins");
static_assert(!(GSE_PIN_RELAY_IGN  >= 34 && GSE_PIN_RELAY_IGN  <= 39),
              "igniter relay on GPIO34..39: input-only pins");
static_assert(!(GSE_PIN_HB         >= 34 && GSE_PIN_HB         <= 39),
              "heartbeat on GPIO34..39: input-only pins");
static_assert(GSE_PIN_RELAY_IGN != 0 && GSE_PIN_RELAY_IGN != 2 &&
              GSE_PIN_RELAY_IGN != 12,
              "igniter relay on a strapping pin (GPIO0/2/12): glitches at reset");
#endif

/* The pin names the firmware uses, after the guard above has had its say.    */
#define PIN_SERVO_FILL    GSE_PIN_SERVO_FILL
#define PIN_SERVO_VEH     GSE_PIN_SERVO_VEH
#define PIN_SERVO_PRG     GSE_PIN_SERVO_PRG
#define PIN_RELAY_IGN     GSE_PIN_RELAY_IGN
#define PIN_HB            GSE_PIN_HB

#define PIN_BUZZER           -1       /* optional active buzzer               */

/* DX-LR41 wiring on the pad node (UART1) -- identical on both boards        */
#define PIN_LORA_RX          16       /* ESP32 RX  <- module TXD  */
#define PIN_LORA_TX          17       /* ESP32 TX  -> module RXD  */
#define PIN_LORA_AUX         18       /* module AUX (high = busy) */
#define PIN_LORA_M0          -1       /* tie low on the PCB, AT+SWITCH=0  */
#define PIN_LORA_M1          -1

/* ------------------------------------------------------------- host console */
/* The pad node talks to the laptop, and the TX node talks to the laptop, on
 * this port.  On an ESP32-S3 that is the chip's native USB-CDC ("Serial"); on
 * an ESP-32S "Serial" is the CP2102/CH340 bridge on UART0, which the same
 * laptop port name reaches.  The radio lives on UART1 on every ESP32 variant,
 * so the two never collide, and an ESP-32S keeps TX/RX on the default UART0
 * pins (GPIO1/GPIO3) because the bridge chip is soldered to them.
 */
#define GSE_HOST_BAUD     115200u

/* Servo pulse limits (DS3225-class servo). 1000/2000 us = +/-45 deg = the
 * mechanical 90 deg throw of the ball valve handle.                          */
#define SERVO_MIN_US          1000
#define SERVO_MAX_US          2000
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
/* (The DX-LR41 pins are selected per board variant above; PIN_LORA_* is the
 * single definition, on both roles.)                                        */

/* Compile-time guard rails --------------------------------------------------*/
#ifndef GSE_ROLE_TX
static_assert(GSE_WDG_DS_MAX * 100u < 0xFFFFu, "age_ms must fit in uint16");
#endif

#endif /* GSE_CONFIG_H */

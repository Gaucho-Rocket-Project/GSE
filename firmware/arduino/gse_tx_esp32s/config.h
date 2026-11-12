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

/* ------------------------------------------------------------ link transport
 *
 * TWO ways to get the frames from the TX node to the pad node.  Same protocol,
 * same frames, same safety logic -- only the pipe changes.
 *
 *   GSE_LINK_LORA            DX-LR42/LR41 on UART1.  Long range (hundreds of
 *                            metres), its own radio, no Wi-Fi, no pairing.
 *                            This is the original ECR configuration.
 *
 *   GSE_LINK_ESPNOW (default) ESP-NOW: the ESP32's own Wi-Fi radio, in
 *                            connectionless station mode.  No module, no
 *                            wiring, no AT commands, no SSID, no router, no
 *                            association, no pairing.  Each node sends to the
 *                            other's MAC address; there is no "connection" to
 *                            establish or lose.  Good for the bench, a spare
 *                            pad, and a short/medium-range link.
 *
 * ESP-NOW is the default on ESP32. Build both ends with -DGSE_LINK_LORA
 * (or use the -lora environments) to select the external DX-LR42/LR41.
 * The STM32 TX node remains LoRa-only. Both ends must use the same link.
 *
 * What does NOT change in ESP-NOW mode:
 *   - the frame format, CRC, sequence numbers and arm code;
 *   - the pad's 1.0 s watchdog and every failsafe rule;
 *   - the 20 Hz command pacing and the TX node's stale-frame rejection;
 *   - the igniter interlock, on-time limit and hardware keep-alive.
 * ESP-NOW is a transport, exactly like the LoRa module: if it goes away, the
 * pad does what it always does -- goes safe within the watchdog period.
 *
 * Why ESP-NOW and not Bluetooth: ESP-NOW is connectionless (nothing to pair,
 * nothing to reconnect, no GATT layer), it accepts a whole 250-byte frame in
 * one packet so the protocol never has to be split, it reports its own
 * link-layer delivery status per packet, and its encryption is a compiled-in
 * key rather than a pairing step.  It also keeps Wi-Fi association off
 * entirely, which matters on a launch site full of phones.
 *
 * Security note (read this before flying it): the GSE protocol has no
 * authentication.  Anyone in radio range who can speak it can talk to a
 * listening pad node.  That is why LoRa mode uses a private channel + KEY and
 * why the pad only accepts frames that carry the arm code.  ESP-NOW adds two
 * concrete controls, both compiled in rather than configured at runtime:
 *   - the pad only accepts datagrams whose source MAC is the TX node's, and
 *     vice versa (GSE_NOW_TX_MAC / GSE_NOW_PAD_MAC below) -- a stranger's
 *     ESP-NOW packet is dropped in the receive callback, before the protocol
 *     parser ever sees it;
 *   - CCMP encryption is turned on with a 16-byte local master key that both
 *     nodes share (GSE_NOW_KEY_TEXT).  It is a compiled-in secret, so treat it
 *     as obfuscation-plus rather than as a key-management scheme: change the
 *     text in both nodes before a launch you care about.
 * CCMP authenticates encrypted traffic only if the shared keys remain secret;
 * the published defaults are not secret. MAC addresses can be spoofed and the
 * fixed arm code is not authentication. Use independent physical interlocks.  See docs/LINK_OPTIONS.md and
 * test T-11.
 */
//#define GSE_LINK_LORA
//#define GSE_LINK_ESPNOW

#if defined(GSE_LINK_ESPNOW) && defined(GSE_LINK_LORA)
#error "Pick one link: GSE_LINK_ESPNOW or GSE_LINK_LORA, not both"
#endif
#if defined(GSE_PLATFORM_STM32) && defined(GSE_LINK_ESPNOW)
#error "GSE_LINK_ESPNOW is ESP32-only (the Nucleo TX node has no Wi-Fi)"
#endif
#if !defined(GSE_LINK_ESPNOW) && !defined(GSE_LINK_LORA)
#  if defined(GSE_PLATFORM_STM32)
#    define GSE_LINK_LORA             /* STM32 has no ESP-NOW radio */
#  else
#    define GSE_LINK_ESPNOW           /* ESP32 default */
#  endif
#endif

#if defined(GSE_LINK_ESPNOW)
#  define GSE_LINK_NAME        "espnow"
#else
#  define GSE_LINK_NAME        "lora"
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
#  define GSE_CH_SELF(TX)     GSE_CH_UP
#  define GSE_CH_PEER(TX)     GSE_CH_UP
#else
#  define GSE_CH_SELF(TX)     ((TX) ? GSE_CH_DOWN : GSE_CH_UP)
#  define GSE_CH_PEER(TX)     ((TX) ? GSE_CH_UP   : GSE_CH_DOWN)
#endif
#define GSE_LEVEL_DEFAULT     3u        /* 10 kbit/s LoRa, ~2.2 km LOS        */
#define GSE_LEVEL_FAST        8u        /* FLRC 1.04 Mbit/s, >=6 B payloads   */
#define GSE_KEY_WORD          51042u    /* AT+KEY (0..65535); not security    */

/* ------------------------------------------------------------- ESP-NOW link
 *
 * Two nodes, one pair of MAC addresses, compiled in at both ends.  Read them
 * off the boot banner (each node prints its own station MAC) and put them
 * here, or override per build with -D:
 *
 *   pio run -e tx-node-now  -t upload   ; prints "espnow local MAC 24:6f:...".
 *   board_flags = -D 'GSE_NOW_PAD_MAC={0xAA,0xBB,...}'
 *
 * Why compiled in: a pad that accepted "whoever called first" would be a pad a
 * stranger's ESP-NOW sender could command.  Fixing the peer MAC means every
 * other sender is dropped in the receive callback, before the protocol parser
 * runs.
 *
 * The placeholders below are for the FIRST flash only: the node comes up,
 * prints its own MAC on the boot banner and reports "peer MAC NOT CONFIGURED"
 * instead of pretending the link is ready.  Read the two MACs off the two
 * boards, type them in, rebuild.  (ESP-NOW does not need to know its own MAC,
 * only the peer's -- so one build cannot fill in the other's.)
 *
 * Channel: both nodes set GSE_NOW_CHANNEL at boot and stay there. Channel 1
 * is a starting point, not an interference-free guarantee.  It must
 * match at both ends or nothing is ever received.
 *
 * Encryption: ESP-NOW CCMP with a local master key.  The key material below is
 * only a default -- change the text once, in both nodes, and rebuild both.
 * Note that ESP-IDF derives the effective key from the *peer's* LMK and the
 * PMK, so both ends must use the same values; that is handled here.
 */
#define GSE_NOW_CHANNEL        1

/* The pad's station MAC: the ESP32-S3 vehicle board, 14:C1:9F:C9:E9:B8
 * (read with esptool read_mac).  Placeholder was 02:00:00:00:00:02. */
#ifndef GSE_NOW_PAD_MAC
#  define GSE_NOW_PAD_MAC      { 0x14, 0xC1, 0x9F, 0xC9, 0xE9, 0xB8 }
#endif
/* The TX node's station MAC: the ESP32S ground box, B0:CB:D8:89:4F:B8
 * (read with esptool read_mac).  Placeholder was 02:00:00:00:00:01. */
#ifndef GSE_NOW_TX_MAC
#  define GSE_NOW_TX_MAC       { 0xB0, 0xCB, 0xD8, 0x89, 0x4F, 0xB8 }
#endif
/* A real ESP32 station MAC carries the Espressif OUI, it is not the 02:00:...
 * locally-administered form above.  Booting with a placeholder is not a fault
 * -- it is how you learn the real one -- but it is not a working link either,
 * and link.h says exactly that in the boot banner instead of letting the pad
 * sit in NO-LINK looking healthy. */

/* CCMP local master key (16 bytes) and primary master key (16 bytes), shared
 * by both nodes.  Change the text and BOTH ends rebuild, or they cannot talk:
 * that is the intended behaviour, not a bug to work around. */
#ifndef GSE_NOW_KEY_TEXT
#  define GSE_NOW_KEY_TEXT     "HCR5100-GSE-LMK0"  /* exactly 16 characters */
#endif
#ifndef GSE_NOW_PMK_TEXT
#  define GSE_NOW_PMK_TEXT     "HCR5100-GSE-PMK0"  /* exactly 16 characters */
#endif
/* Both strings are strlen-checked at compile time in link.h: 16 characters, no
 * more and no fewer, because ESP-IDF's key types are fixed 16-byte arrays and a
 * silently-truncated key would fail at run time with "send failed" instead of
 * at build time with a message.  They are #ifndef-guarded so an operator who
 * does want a different key can pass -DGSE_NOW_KEY_TEXT='"..."' (build_flags)
 * instead of editing this file -- and if that string is not 16 characters the
 * build stops with the assert, which is the whole point of the guard. */

/* Delivery bookkeeping: ESP-NOW gives a per-packet send status, which is far
 * more useful than "it looked fine".  A pad that has not acked for this long
 * is reported as "no radio contact" in the TX node's 1 Hz line -- it does not
 * change any safety behaviour (the pad's own watchdog does that). */
#define GSE_NOW_QUIET_MS       1000u
#define GSE_NOW_TX_QUEUE       16

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

/* ------------------------------------------------ DX-LR42/LR41 pin wiring
 * Identical on both boards and on both roles.  The vendor's own reference
 * circuit (LR42 technical manual, Figure 6) is:
 *
 *      module RX  (3) <- MCU TXD          module M0 (1) <- MCU IO   (or GND)
 *      module TX  (4) -> MCU RXD          module M1 (2) <- MCU IO   (or GND)
 *      module AUX (5) -> MCU IO           module VCC (6) <- 5 V (LR42) / 3V3 (LR41)
 *      module GND (7) -- GND
 *
 * so on the ESP32 side the serial lines are crossed: module RX (3) is fed from
 * D17, and module TX (4) feeds D16.  AUX (5) is an INPUT to the ESP32.
 *
 * Not connected?  Each line can be left off in a pinch:
 *   AUX  -DGSE_LORA_AUX_NONE : define it (or set PIN_LORA_AUX to -1) and the
 *        firmware never reads GPIO18.  That pin then floats, which is fine --
 *        it is an unused input -- but you lose the "module is busy" flow
 *        control, so the 20 Hz command stream relies on pacing alone.  Fine at
 *        LEVEL 3 (one 22-byte frame is ~27 ms of a 50 ms slot), risky at
 *        LEVEL 8/FLRC.  AUX is a module OUTPUT; do not tie it to GND or 3V3.
 *   M0/M1  must be tied to GND at the module (see hardware/wiring.md §1.2);
 *        they are module outputs only while AT+SWITCH=0.
 */
#define PIN_LORA_RX          16       /* ESP32 D16 <- module TX pad (4)    */
#define PIN_LORA_TX          17       /* ESP32 D17 -> module RX pad (3)    */
#ifdef GSE_LORA_AUX_NONE
#  undef  PIN_LORA_AUX
#  define PIN_LORA_AUX       -1       /* AUX not wired: no busy-poll       */
#else
#  define PIN_LORA_AUX       18       /* module AUX (5), high = busy       */
#endif
#define PIN_LORA_AUX_PULLUP  0        /* 0 = INPUT (AUX is a push-pull out),
                                       * 1 = INPUT_PULLUP (harmless)       */
#define PIN_LORA_M0          -1       /* not driven: tie M0 to GND at the  */
#define PIN_LORA_M1          -1       /* module.  A GPIO here would also work
                                       * (driver drives both LOW) but is only
                                       * safe while the module is AT+SWITCH=0 */

/* ------------------------------------------------------------- host console */
/* The pad node talks to the laptop, and the TX node talks to the laptop, on
 * this port.  On an ESP32-S3 that is the chip's native USB-CDC ("Serial"); on
 * an ESP-32S "Serial" is the CP2102/CH340 bridge on UART0, which the same
 * laptop port name reaches.  The radio lives on UART1 on every ESP32 variant,
 * so the two never collide, and an ESP-32S keeps TX/RX on the default UART0
 * pins (GPIO1/GPIO3) because the bridge chip is soldered to them.
 */
#define GSE_HOST_BAUD     115200u

/* Servo pulses (DS3225, 180-degree version): 500..2500 us = 0..180 deg,
 * 1500 us = centre, i.e. 500 us per 45 deg.  The scale is fixed here,
 * separately from the clamp, so "travel" is real degrees each side of centre:
 * 45 -> 1000/2000 us (the default, unchanged), 90 -> 500/2500 us (full
 * +/-90 deg).  The clamp is only a safety limit at the servo's own range.    */
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
/* (The DX-LR41 pins are selected per board variant above; PIN_LORA_* is the
 * single definition, on both roles.)                                        */

/* Compile-time guard rails --------------------------------------------------*/
#ifndef GSE_ROLE_TX
static_assert(GSE_WDG_DS_MAX * 100u < 0xFFFFu, "age_ms must fit in uint16");
#endif

#endif /* GSE_CONFIG_H */

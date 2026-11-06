/*
 * servo.h - dual hobby-servo driver for the NUCLEO-H753ZI
 *
 * Both servos are driven from ONE hardware timer (TIM1) so they always share
 * the same 50 Hz frame: servo 1 on TIM1_CH1 (PE9), servo 2 on TIM1_CH2 (PE11).
 * Pulse widths are independent (separate compare registers), which is what
 * "separately or combined" needs.
 *
 * Timing: TIM1 runs at 200 MHz from the APB2 timer clock, prescaled to 1 MHz.
 * One timer tick is therefore exactly 1 us, the counter period is 20000 ticks
 * (20 ms, 50 Hz) and the compare value IS the pulse width in microseconds.
 * That is finer than the DS3225's own dead band (3 us).
 *
 * Defaults match the DSSERVO DS3225MG 25 kg digital servo (Amazon B09SB8RMTT):
 * 500-2500 us full travel, 1500 us centre. For SG90 / MG90S micro servos use
 * the same firmware - the 500-2500 us range is simply the outer limit, centre
 * is still 1500 us; narrow SERVO_MIN/MAX_PULSE_US if a small servo buzzes at
 * the extremes.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SERVO_COUNT 2

/* ------------------------------------------------------------------------- */
/* Servo profile - override on the compiler command line if you like:        */
/*   make TRAVEL=270            (270 deg DS3225 variant)                     */
/*   make MIN_US=700 MAX_US=2300                                            */
/* ------------------------------------------------------------------------- */
#ifndef SERVO_TRAVEL_DEG
#define SERVO_TRAVEL_DEG        180     /* DS3225MG listing is the 180 deg one */
#endif
#ifndef SERVO_MIN_PULSE_US
#define SERVO_MIN_PULSE_US      500     /* 0 deg   (full advertised range)     */
#endif
#ifndef SERVO_MAX_PULSE_US
#define SERVO_MAX_PULSE_US      2500    /* travel deg (full advertised range)  */
#endif
#define SERVO_DEFAULT_PULSE_US  1500    /* centre / neutral                    */

#define SERVO_FRAME_HZ          50
#define SERVO_FRAME_US          20000   /* 1 / 50 Hz                           */

/**
 * @brief Configure TIM1 for 50 Hz, enable both channels, leave both servos
 *        "off" (signal held low, so they are limp and draw nothing).
 */
void servo_init(void);

/**
 * @brief Start (or re-aim) servo @p idx at @p pulse_us. Enables the channel if
 *        it was off. Pulse width is clamped to the configured range.
 */
void servo_on(int idx, uint16_t pulse_us);

/** @brief Stop pulses for servo @p idx (signal low -> servo goes limp). */
void servo_off(int idx);

/** @brief Aim by angle in degrees (clamped to 0..SERVO_TRAVEL_DEG). */
void servo_set_angle(int idx, int angle_deg);

/** @brief True while the channel is generating pulses. */
bool servo_is_on(int idx);

/** @brief Last commanded pulse width, in microseconds. */
uint16_t servo_pulse_us(int idx);

/** @brief Last commanded angle in degrees (derived from the pulse width). */
int servo_angle(int idx);

/** @brief "servo1" / "servo2" (never NULL for a valid index). */
const char *servo_name(int idx);

/** @brief MCU pin description, e.g. "PE9" (never NULL). */
const char *servo_pin_name(int idx);

/** @brief Convert an angle to a pulse width, clamped to the safe range. */
uint16_t servo_angle_to_pulse_us(int angle_deg);

/** @brief Convert a pulse width to the nearest angle (for display). */
int servo_pulse_us_to_angle(uint16_t pulse_us);

/** @brief Clamp a pulse width into [SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US]. */
uint16_t servo_clamp_pulse(uint16_t pulse_us);

/**
 * @brief int-returning alias of servo_angle_to_pulse_us(), so it can be handed
 *        to position_parse_pulse_us() as a plain function pointer.
 */
int servo_angle_to_pulse_us_int(int angle_deg);

#ifdef __cplusplus
}
#endif

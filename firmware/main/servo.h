/*
 * servo.h - hobby-servo driver for ESP32-S3 (LEDC hardware PWM, 50 Hz)
 *
 * Works with any analog or digital servo that takes the standard 3-wire PWM
 * interface: SG90 / MG90S (micro), and the DSSERVO DS3225MG, MG996R, etc.
 * Only the pulse range and the mechanical travel angle differ - both are
 * configuration, not code changes.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SERVO_COUNT             2

/* ---------------------------------------------------------------------------
 * Servo type -> pulse range / travel
 *
 * Pick ONE of these before building. Everything else is automatic: the angle
 * commands, the sweep and the status output all use SERVO_TRAVEL_DEG.
 *
 *   SERVO_TYPE_SG90     micro analog, 1.0-2.0 ms for 0-180 (reduced range)
 *   SERVO_TYPE_DS3225   DSSERVO 25 kg digital: 500-2500 us, neutral 1500 us
 *
 * DS3225 note: the same servo is sold in 180 deg and 270 deg versions, and the
 * pulse range is 500-2500 us for BOTH. So on the 270 deg version 180 deg of
 * command travel would only use 2/3 of the mechanical range unless you tell the
 * firmware the mechanical travel is 270.
 * ------------------------------------------------------------------------- */
#define SERVO_TYPE_SG90       1
#define SERVO_TYPE_DS3225     2

#ifndef SERVO_TYPE
#define SERVO_TYPE            SERVO_TYPE_SG90
#endif

#if SERVO_TYPE == SERVO_TYPE_DS3225
  #define SERVO_TYPE_NAME       "DS3225MG (25 kg digital)"
  #define SERVO_MIN_PULSE_US    500     /* full advertised range        */
  #define SERVO_MAX_PULSE_US    2500
  #define SERVO_DEFAULT_PULSE_US 1500   /* neutral / centre             */
  #ifndef SERVO_TRAVEL_DEG
    #define SERVO_TRAVEL_DEG    270     /* 180 deg version? set it to 180 */
  #endif
#else
  #define SERVO_TYPE_NAME       "SG90 / micro analog"
  #define SERVO_MIN_PULSE_US    500     /* SG90 real-world safe range   */
  #define SERVO_MAX_PULSE_US    2500
  #define SERVO_DEFAULT_PULSE_US 1500
  #ifndef SERVO_TRAVEL_DEG
    #define SERVO_TRAVEL_DEG    180
  #endif
#endif

/* Clamp helper used by the command parser and status display */
#define SERVO_TRAVEL_MAX      SERVO_TRAVEL_DEG

/**
 * @brief Configure the 50 Hz LEDC timer and both servo channels.
 *        Servos start in the "off" state (no pulses, signal pin low).
 */
esp_err_t servo_init(void);

/**
 * @brief Start (or re-aim) servo @p idx : generate pulses at @p pulse_us.
 *        Also used to move a servo that is already on.
 */
esp_err_t servo_on(int idx, uint16_t pulse_us);

/**
 * @brief Stop pulses to servo @p idx (signal pin parked low, servo goes limp).
 */
esp_err_t servo_off(int idx);

/**
 * @brief Change the pulse width of servo @p idx (enables it if needed).
 */
esp_err_t servo_set_pulse(int idx, uint16_t pulse_us);

const char *servo_name(int idx);
bool        servo_is_on(int idx);
uint16_t    servo_pulse_us(int idx);
int         servo_gpio(int idx);
int         servo_angle_to_pulse_us(int angle_deg);

#ifdef __cplusplus
}
#endif

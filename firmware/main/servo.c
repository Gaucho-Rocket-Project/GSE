/*
 * servo.c - SG90 hobby-servo driver on top of the ESP32-S3 LEDC (PWM) peripheral
 *
 * An analog servo like the SG90 wants a 50 Hz pulse train:
 *   - pulse width 1.0 ms  ->   0 degrees
 *   - pulse width 1.5 ms  ->  90 degrees
 *   - pulse width 2.0 ms  -> 180 degrees
 * (a new pulse must arrive every 20 ms)
 *
 * The LEDC timer runs at 50 Hz with 16-bit resolution, so one "duty" tick is
 * 20000 us / 65536 = 0.3052 us and a pulse width in microseconds maps to
 * a duty value directly.
 */

#include "servo.h"

#include "driver/ledc.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "servo";

#define SERVO_LEDC_SPEED_MODE   LEDC_LOW_SPEED_MODE
#define SERVO_LEDC_TIMER        LEDC_TIMER_0
/* ESP32-S3 LEDC is 14-bit max -> 16384 steps per 20 ms period
 * (20000 us / 16384 = 1.22 us per step, i.e. ~0.1 degree). */
#define SERVO_LEDC_RES          LEDC_TIMER_14_BIT
#define SERVO_LEDC_STEPS        (1u << 14)
#define SERVO_PERIOD_US         20000.0f

typedef struct {
    const char      *name;
    gpio_num_t       gpio;
    ledc_channel_t   channel;
    bool             enabled;      /* pulses currently generated on the pin */
    uint16_t         pulse_us;     /* last commanded pulse width        */
} servo_t;

/*
 * Default pin assignment. Both servos share LEDC timer 0 (same 50 Hz period)
 * and run on separate channels. Timers/channels are a group resource of chips,
 * so every channel in this group must use the same frequency/resolution.
 *
 *                      +--------------+
 *  GPIO4  ------------>| Orange  Servo 1 (signal)
 *  GPIO5  ------------>| Orange  Servo 2 (signal)
 *  5V (or external) -->| Red     both   (5 V)
 *  GND    ------------>| Brown   both   (ground)
 *                      +--------------+
 */
static servo_t s_servos[SERVO_COUNT] = {
    [0] = { .name = "servo1", .gpio = GPIO_NUM_4, .channel = LEDC_CHANNEL_0, .enabled = false, .pulse_us = SERVO_DEFAULT_PULSE_US },
    [1] = { .name = "servo2", .gpio = GPIO_NUM_5, .channel = LEDC_CHANNEL_1, .enabled = false, .pulse_us = SERVO_DEFAULT_PULSE_US },
};

static uint32_t pulse_us_to_duty(uint16_t pulse_us)
{
    uint32_t max_duty = SERVO_LEDC_STEPS - 1u;                /* 16383 */
    uint64_t duty = ((uint64_t)pulse_us * SERVO_LEDC_STEPS) / (uint32_t)SERVO_PERIOD_US;
    if (duty > max_duty) {
        duty = max_duty;
    }
    return (uint32_t)duty;
}

static inline uint16_t pulse_us_clamped(uint16_t pulse_us)
{
    if (pulse_us < SERVO_MIN_PULSE_US) {
        return SERVO_MIN_PULSE_US;
    }
    if (pulse_us > SERVO_MAX_PULSE_US) {
        return SERVO_MAX_PULSE_US;
    }
    return pulse_us;
}

esp_err_t servo_init(void)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = SERVO_LEDC_SPEED_MODE,
        .timer_num       = SERVO_LEDC_TIMER,
        .duty_resolution = SERVO_LEDC_RES,
        .freq_hz         = 50,              /* 20 ms period */
        .clk_cfg         = LEDC_AUTO_CLK,   /* 80 MHz APB / 2^16 / 50 Hz -> tiny divider error */
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer_cfg), TAG, "LEDC timer config failed");

    for (int i = 0; i < SERVO_COUNT; i++) {
        servo_t *s = &s_servos[i];
        ledc_channel_config_t ch_cfg = {
            .gpio_num   = s->gpio,
            .speed_mode = SERVO_LEDC_SPEED_MODE,
            .channel    = s->channel,
            .intr_type  = LEDC_INTR_DISABLE,
            .timer_sel  = SERVO_LEDC_TIMER,
            .duty       = 0,
            .hpoint     = 0,
            .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
            .flags      = { .output_invert = 0 },
        };
        ESP_RETURN_ON_ERROR(ledc_channel_config(&ch_cfg), TAG, "LEDC channel %d config failed", i);
        s->enabled = false;
        ESP_LOGI(TAG, "%s ready on GPIO%d (idle / no pulses)", s->name, s->gpio);
    }
    return ESP_OK;
}

esp_err_t servo_on(int idx, uint16_t pulse_us)
{
    if (idx < 0 || idx >= SERVO_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    servo_t *s = &s_servos[idx];
    pulse_us = pulse_us_clamped(pulse_us);

    ESP_RETURN_ON_ERROR(ledc_set_duty(SERVO_LEDC_SPEED_MODE, s->channel, pulse_us_to_duty(pulse_us)),
                        TAG, "set duty failed");
    ESP_RETURN_ON_ERROR(ledc_update_duty(SERVO_LEDC_SPEED_MODE, s->channel),
                        TAG, "update duty failed");

    if (!s->enabled) {
        /* Bring the output back after a servo_off() (ledc_stop held the pin low). */
        ledc_channel_config_t ch_cfg = {
            .gpio_num   = s->gpio,
            .speed_mode = SERVO_LEDC_SPEED_MODE,
            .channel    = s->channel,
            .intr_type  = LEDC_INTR_DISABLE,
            .timer_sel  = SERVO_LEDC_TIMER,
            .duty       = pulse_us_to_duty(pulse_us),
            .hpoint     = 0,
            .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
            .flags      = { .output_invert = 0 },
        };
        ESP_RETURN_ON_ERROR(ledc_channel_config(&ch_cfg), TAG, "re-enable channel %d failed", idx);
        s->enabled = true;
    }

    s->pulse_us = pulse_us;
    return ESP_OK;
}

esp_err_t servo_off(int idx)
{
    if (idx < 0 || idx >= SERVO_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    servo_t *s = &s_servos[idx];
    /* Stop the pulse train and park the signal line low: the servo is unpowered
     * from a control point of view and stops holding torque (it goes limp, it
     * does not return to a park position). */
    ESP_RETURN_ON_ERROR(ledc_stop(SERVO_LEDC_SPEED_MODE, s->channel, 0), TAG, "stop failed");
    s->enabled = false;
    return ESP_OK;
}

esp_err_t servo_set_pulse(int idx, uint16_t pulse_us)
{
    if (idx < 0 || idx >= SERVO_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    return servo_on(idx, pulse_us);   /* same path; keeps it enabled */
}

const char *servo_name(int idx)
{
    return (idx >= 0 && idx < SERVO_COUNT) ? s_servos[idx].name : "?";
}

bool servo_is_on(int idx)
{
    return (idx >= 0 && idx < SERVO_COUNT) ? s_servos[idx].enabled : false;
}

uint16_t servo_pulse_us(int idx)
{
    return (idx >= 0 && idx < SERVO_COUNT) ? s_servos[idx].pulse_us : 0;
}

int servo_gpio(int idx)
{
    return (idx >= 0 && idx < SERVO_COUNT) ? (int)s_servos[idx].gpio : -1;
}

int servo_angle_to_pulse_us(int angle_deg)
{
    if (angle_deg < 0) {
        angle_deg = 0;
    } else if (angle_deg > SERVO_TRAVEL_DEG) {
        angle_deg = SERVO_TRAVEL_DEG;
    }
    return SERVO_MIN_PULSE_US +
           (angle_deg * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) / SERVO_TRAVEL_DEG;
}

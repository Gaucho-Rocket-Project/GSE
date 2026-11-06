/*
 * servo.c - TIM1 (advanced-control timer) based 50 Hz servo pulse generator
 *
 * Why TIM1 for both servos:
 *   - one timer, one time base -> the two 50 Hz frames are identical, so the
 *     two servos never fight over current at slightly different times;
 *   - CH1 and CH2 have independent compare registers -> fully independent
 *     pulse widths ("servo1", "servo2" or "both");
 *   - hardware does the work: no interrupts, no jitter, nothing to babysit.
 *
 * Register recipe (RM0433 TIM chapter):
 *   PSC  = (200 MHz / 1 MHz) - 1 = 199   -> 1 us per tick
 *   ARR  = 20000 - 1                     -> 20 ms period
 *   CCMR1: OC1M = 110 (PWM mode 1), OC1PE = 1, same for channel 2
 *   CCER : CC1E / CC2E enable the outputs
 *   BDTR : MOE = 1 - TIM1 is an advanced timer, without this the outputs stay
 *          inactive no matter what CCER says (classic gotcha)
 *   CCRx : pulse width in microseconds
 *   CR1  : ARPE = 1, CEN = 1
 */
#include "servo.h"

#include "board.h"

typedef struct {
    const char *name;
    const char *pin;
    uint32_t   cc_ch;      /* 1 or 2: which channel this servo is on */
    uint16_t   pulse_us;   /* last commanded pulse width             */
    bool       enabled;    /* output currently driven                 */
} servo_t;

static servo_t s_servos[SERVO_COUNT] = {
    [0] = { .name = "servo1", .pin = "PE9",  .cc_ch = 1,
            .pulse_us = SERVO_DEFAULT_PULSE_US, .enabled = false },
    [1] = { .name = "servo2", .pin = "PE11", .cc_ch = 2,
            .pulse_us = SERVO_DEFAULT_PULSE_US, .enabled = false },
};

/* --- small helpers ------------------------------------------------------- */

static inline volatile uint32_t *ccr_addr(int idx)
{
    return (idx == 0) ? &TIM1->CCR1 : &TIM1->CCR2;
}

static inline uint32_t ccx_enable_bit(int idx)
{
    /* CC1E = bit 0, CC2E = bit 4 */
    return (idx == 0) ? TIM_CCER_CCxE(1) : TIM_CCER_CCxE(2);
}

static void timer_setup_timebase(void)
{
    /* Reset-ish the timer: stop it, clear the control registers we use. */
    TIM1->CR1 = 0;
    TIM1->CCER = 0;
    TIM1->BDTR = 0;

    TIM1->PSC = (BOARD_TIM1CLK_HZ / 1000000UL) - 1UL;   /* 200 -> 1 MHz, 1 us */
    TIM1->ARR = SERVO_FRAME_US - 1UL;                   /* 20 ms              */
    TIM1->RCR = 0;

    /* PWM mode 1 on both channels, output preload on. */
    TIM1->CCMR1 = (TIM_CCMR_OCxM_PWM1 | TIM_CCMR_OCxPE) |            /* CH1 */
                  ((TIM_CCMR_OCxM_PWM1 | TIM_CCMR_OCxPE) << 8);      /* CH2 */

    /* 1500 us compare values before the outputs are enabled, so the first
     * pulse is already the centre position. */
    TIM1->CCR1 = SERVO_DEFAULT_PULSE_US;
    TIM1->CCR2 = SERVO_DEFAULT_PULSE_US;

    /* Load PSC/ARR/CCR from the preload registers and clear the counter. */
    TIM1->EGR = TIM_EGR_UG;

    /* Advanced timer: main output enable, or nothing comes out of the pins. */
    TIM1->BDTR = TIM_BDTR_MOE;

    /* Auto-reload preload + start counting. */
    TIM1->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
}

static void gpio_init(void)
{
    /* GPIOE clock, then PE9 = TIM1_CH1 (AF1) and PE11 = TIM1_CH2 (AF1).
     * Both are Arduino header pins: CN10-4 is D6 and CN10-6 is D5. */
    RCC_AHB4ENR |= RCC_AHB4ENR_GPIOEEN;
    (void)RCC_AHB4ENR;

    board_gpio_output(GPIOE, 9);          /* park low before handing to TIM1 */
    board_gpio_output(GPIOE, 11);
    GPIOE->BSRR = (1UL << (9 + 16)) | (1UL << (11 + 16));

    board_gpio_af(GPIOE, 9, AF1_TIM1);
    board_gpio_af(GPIOE, 11, AF1_TIM1);
}

void servo_init(void)
{
    RCC_APB2ENR |= RCC_APB2ENR_TIM1EN;
    (void)RCC_APB2ENR;

    gpio_init();
    timer_setup_timebase();

    for (int i = 0; i < SERVO_COUNT; i++) {
        s_servos[i].pulse_us = SERVO_DEFAULT_PULSE_US;
        s_servos[i].enabled  = false;
    }
    /* Channels stay disabled (CCER = 0) -> pins idle low, servos are limp. */
}

uint16_t servo_clamp_pulse(uint16_t pulse_us)
{
    if (pulse_us < SERVO_MIN_PULSE_US) {
        return SERVO_MIN_PULSE_US;
    }
    if (pulse_us > SERVO_MAX_PULSE_US) {
        return SERVO_MAX_PULSE_US;
    }
    return pulse_us;
}

void servo_on(int idx, uint16_t pulse_us)
{
    if (idx < 0 || idx >= SERVO_COUNT) {
        return;
    }
    servo_t *s = &s_servos[idx];
    pulse_us = servo_clamp_pulse(pulse_us);

    *ccr_addr(idx) = (uint32_t)pulse_us;
    s->pulse_us = pulse_us;

    if (!s->enabled) {
        TIM1->CCER |= ccx_enable_bit(idx);
        s->enabled = true;
    }
}

void servo_off(int idx)
{
    if (idx < 0 || idx >= SERVO_COUNT) {
        return;
    }
    servo_t *s = &s_servos[idx];

    /* Disable just this channel: the pin returns to its idle (low) level and
     * the other servo keeps its pulse train untouched. */
    TIM1->CCER &= ~ccx_enable_bit(idx);
    s->enabled = false;
}

void servo_set_angle(int idx, int angle_deg)
{
    servo_on(idx, servo_angle_to_pulse_us(angle_deg));
}

bool servo_is_on(int idx)
{
    return (idx >= 0 && idx < SERVO_COUNT) ? s_servos[idx].enabled : false;
}

uint16_t servo_pulse_us(int idx)
{
    return (idx >= 0 && idx < SERVO_COUNT) ? s_servos[idx].pulse_us : 0u;
}

int servo_angle(int idx)
{
    return servo_pulse_us_to_angle(servo_pulse_us(idx));
}

const char *servo_name(int idx)
{
    return (idx >= 0 && idx < SERVO_COUNT) ? s_servos[idx].name : "?";
}

const char *servo_pin_name(int idx)
{
    return (idx >= 0 && idx < SERVO_COUNT) ? s_servos[idx].pin : "?";
}

uint16_t servo_angle_to_pulse_us(int angle_deg)
{
    if (angle_deg < 0) {
        angle_deg = 0;
    } else if (angle_deg > SERVO_TRAVEL_DEG) {
        angle_deg = SERVO_TRAVEL_DEG;
    }
    return (uint16_t)(SERVO_MIN_PULSE_US +
                      (angle_deg * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) /
                          SERVO_TRAVEL_DEG);
}

int servo_angle_to_pulse_us_int(int angle_deg)
{
    return (int)servo_angle_to_pulse_us(angle_deg);
}

int servo_pulse_us_to_angle(uint16_t pulse_us)
{
    int span = SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US;
    int rel  = (int)servo_clamp_pulse(pulse_us) - SERVO_MIN_PULSE_US;
    return (rel * SERVO_TRAVEL_DEG + span / 2) / span;
}

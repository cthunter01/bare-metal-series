#include "core/timer.h"

#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>

#define PRESCALER (84)
#define ARR_VALUE (1000)

void timer_setup()
{
    // Always need to enable the clock
    rcc_periph_clock_enable(RCC_TIM2);

    // High level timer configuration
    timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

    // Set up PWM mode
    timer_set_oc_mode(TIM2, TIM_OC1, TIM_OCM_PWM1);

    // Enable the counter. It won't actually start counting without explicitly
    // turning it on
    timer_enable_counter(TIM2);
    timer_enable_oc_output(TIM2, TIM_OC1);

    // Set up the frequency. Since we haven't enabled any clock division,
    // it's running at 84 MHz. We should reduce that here, to say, 1 MHz
    timer_set_prescaler(TIM2, PRESCALER - 1);

    // Total ticks per cycle is 1000
    timer_set_period(TIM2, ARR_VALUE - 1);

    // overall frequency is then 84_000_000 / (84 * 1000) = 1 kHz
    // and duty cycle adjustable in increments of 0.1% (1000 points)


}

void timer_pwm_set_duty_cycle(float duty_cycle)
{
    // duty_cycle = (ccr / arr) * 100
    // ccr = duty_cycle * arr / 100
    if(duty_cycle < 0.0f)
    {
        duty_cycle = 0.0f;
    }
    if(duty_cycle > 100.0f)
    {
        duty_cycle = 100.0;
    }
    float raw_value = (float)ARR_VALUE * (duty_cycle / 100.0f);
    timer_set_oc_value(TIM2, TIM_OC1, (uint32_t)raw_value);
}
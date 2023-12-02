#include "timer.h"

#include <array>

#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>

namespace
{
std::array<stm32f4::PWMOutputConfig, stm32f4::PWM_TIM_USERCUSTOM> buildConfigs()
{
    std::array<stm32f4::PWMOutputConfig, stm32f4::PWM_TIM_USERCUSTOM> retVal;
    stm32f4::PWMOutputConfig tim2_1;
    tim2_1._clk = RCC_TIM2;
    tim2_1._timer = TIM2;
    tim2_1._clock_div = TIM_CR1_CKD_CK_INT;
    tim2_1._alignment = TIM_CR1_CMS_EDGE;
    tim2_1._direction = TIM_CR1_DIR_UP;
    tim2_1._oc_mode = TIM_OCM_PWM1;
    tim2_1._output_channel = TIM_OC1;

    tim2_1._prescale = 96U;
    tim2_1._period = 1000;

    retVal[stm32f4::PWM_TIM2_1] = tim2_1;

    return retVal;

}
}

namespace stm32f4
{

PWMOutput::PWMOutput(enum PWMOutputConfigEnum config)
  : PWMConfigs(buildConfigs()),
    _configEnum(config),
    _config(PWMConfigs[config]),
    _dutyCycle(0.0)
{
    // Always need to enable the clock
    rcc_periph_clock_enable(_config._clk);

    // High level timer configuration
    timer_set_mode(_config._timer,
                   _config._clock_div,
                   _config._alignment,
                   _config._direction);

    // Set up PWM mode
    timer_set_oc_mode(_config._timer,
                      _config._output_channel,
                      _config._oc_mode);

    // Enable the counter. It won't actually start counting without explicitly
    // turning it on
    timer_enable_counter(_config._timer);
    timer_enable_oc_output(_config._timer, _config._output_channel);

    // Set up the timer frequency. This scales from the main clock frequency
    timer_set_prescaler(_config._timer, _config._prescale - 1);

    // Total ticks per cycle
    timer_set_period(_config._timer, _config._period - 1);

    // TODO: enable interrupts for the timer. 
    //timer_enable_irq();
}

PWMOutput::~PWMOutput()
{

}

// TODO: This should probably be customizable. Not sure a single
// handler function makes sense in all cases
void PWMOutput::interrupt_handler()
{
    Interruptible::null_interrupt();
}

void PWMOutput::setDutyCycle(float duty_cycle)
{
    _dutyCycle = duty_cycle;
    if(_dutyCycle < 0.0f)
    {
        _dutyCycle = 0.0f;
    }
    if(_dutyCycle > 100.0f)
    {
        _dutyCycle = 100.0;
    }
    float raw_value = static_cast<float>(_config._period) * (_dutyCycle / 100.0f);
    timer_set_oc_value(_config._timer,
                       _config._output_channel,
                       static_cast<uint32_t>(raw_value));
}
}
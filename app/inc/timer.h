#ifndef _TIMER_H
#define _TIMER_H

#include <array>

#include "core/Interruptible.h"

#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rcc.h>

namespace stm32f4
{

struct PWMOutputConfig
{
    rcc_periph_clken _clk;
    tim_oc_mode _oc_mode;
    tim_oc_id _output_channel;
    uint32_t _timer;
    uint32_t _clock_div;
    uint32_t _alignment;
    uint32_t _direction;
    uint32_t _prescale;
    uint32_t _period;


};

enum PWMOutputConfigEnum
{
    PWM_TIM2_1= 0,
    PWM_TIM_USERCUSTOM = 1
};

class PWMOutput : public Interruptible
{
public:
    PWMOutput(enum PWMOutputConfigEnum config);
    virtual ~PWMOutput();
    void interrupt_handler() override;

    void setDutyCycle(float duty_cycle);
private:
    const std::array<PWMOutputConfig, PWM_TIM_USERCUSTOM> PWMConfigs;
    enum PWMOutputConfigEnum _configEnum;
    PWMOutputConfig _config;

    float _dutyCycle;
};

}

#endif // _TIMER_H

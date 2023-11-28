#ifndef _SYSTICK_H_
#define _SYSTICK_H_

#include <cstdint>

#include "core/Interruptible.h"

namespace stm32f401
{

class SysTick : public Interruptible
{
public:
    static SysTick& getInstance()
    {
        static SysTick inst;
        return inst;
    }
    
    void start();
    virtual ~SysTick() {}

    void interrupt_handler() override;

    uint32_t getTicks() { return ticks; }

    void setFrequency(uint32_t frequency = 1000U);

    void systemDelay(uint32_t delay);
private:
    SysTick();
    uint32_t ticks;
    uint32_t freq;
};

}
#endif // _SYSTICK_H_

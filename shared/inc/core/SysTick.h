#ifndef _SYSTICK_H_
#define _SYSTICK_H_

#include <cstdint>

#include "core/Interruptible.h"

namespace stm32f401
{

class SysTick : public Interruptible
{
public:
    SysTick();
    virtual ~SysTick() {}

    void interrupt_handler() override;

    uint32_t getTicks() { return ticks; }

    void systemDelay(uint32_t delay);
private:
    uint32_t ticks;
};

}
#endif // _SYSTICK_H_

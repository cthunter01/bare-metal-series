#include "core/SysTick.h"
#include "core/InterruptManager.h"

#include <libopencm3/cm3/systick.h>

namespace stm32f401
{

SysTick::SysTick()
  : Interruptible(),
    ticks{0}
{
    InterruptManager::set_systick_handler(&Interruptible::interrupt_handler, this);

    systick_set_frequency(1000, 84000000);

    systick_counter_enable();

    systick_interrupt_enable();

}

void SysTick::interrupt_handler()
{
    ticks++;
}

void SysTick::systemDelay(uint32_t delay)
{
    volatile uint32_t end_time = getTicks() + delay;
    while(getTicks() < end_time)
    {
        __asm__("nop");
    }
}

} // namespace
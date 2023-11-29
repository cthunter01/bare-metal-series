#include "core/SysTick.h"
#include "core/InterruptManager.h"

#include "core/system.h"
#include <libopencm3/cm3/systick.h>

namespace stm32f4
{

SysTick::SysTick()
  : Interruptible(),
    ticks{0}
{
    InterruptManager::set_systick_handler(&Interruptible::interrupt_handler, this);

    //systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_set_frequency(1000, CPU_FREQ);
    //systick_clear();

    systick_interrupt_enable();
    systick_counter_enable();


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
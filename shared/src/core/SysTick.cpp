#include "core/SysTick.h"
#include "core/InterruptManager.h"

#include <libopencm3/cm3/systick.h>

namespace stm32f401
{

SysTick::SysTick()
  : Interruptible(),
    ticks{0},
    freq{1000}
{
    InterruptManager::set_systick_isr_handler(&Interruptible::interrupt_handler, this);

}

void SysTick::setFrequency(uint32_t frequency)
{
    freq = frequency;
}

void SysTick::start()
{
    systick_set_frequency(freq, 84000000);

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
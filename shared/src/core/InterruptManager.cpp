#include "core/InterruptManager.h"
#include "core/Interruptible.h"

/*
stm32f401::Interruptible* InterruptManager::usart6Object;
void(InterruptManager::*usart6_handler)(void);

stm32f401::Interruptible* InterruptManager::sysTickObject;
void(InterruptManager::*systick_handler)(void);
*/
std::function<void(void)> InterruptManager::usart6_handler = InterruptManager::nullHandler;
std::function<void(void)> InterruptManager::systick_handler = InterruptManager::nullHandler;

InterruptManager::InterruptManager()
{
}

extern "C"
{
void sys_tick_handler(void)
{
    InterruptManager::systick_isr_handler();
}
}

/* NOT YET

void usart6_isr()
{
    InterruptManager::usart6_isr_handler();
}
*/
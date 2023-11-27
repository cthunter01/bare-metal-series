#include "core/InterruptManager.h"
#include "core/Interruptible.h"

stm32f401::Interruptible* InterruptManager::usart6Object;

void(InterruptManager::*usart6_handler)(void);
//void(&stm32f401::Interruptible::interrupt_handler)(void) InterruptManager::usart6_handler;

void usart6_isr()
{
    InterruptManager::usart6_isr_handler();
}
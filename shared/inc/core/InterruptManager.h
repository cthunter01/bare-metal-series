#ifndef SHARED_INTERRUPTMANAGER_H
#define SHARED_INTERRUPTMANAGER_H

#include "core/Interruptible.h"

class InterruptManager
{
public:

    static void set_usart6_isr_handler(void(stm32f401::Interruptible::*handler)(void), stm32f401::Interruptible* obj)
    {
        usart6Object = obj;
        usart6_handler = handler;
    }

    static void usart6_isr_handler()
    {
        (usart6Object->*usart6_handler)();
    }
private:
    // can't create one of these
    InterruptManager() {}

    static stm32f401::Interruptible* usart6Object;
    static void(stm32f401::Interruptible::*usart6_handler)(void);

};

// These have to have C linkage, no name mangling
// since they redefine functions declared weak elsewhere

#endif //  SHARED_INTERRUPT_MANAGER_H
#ifndef _INTERRUPTMANAGER_H
#define _INTERRUPTMANAGER_H

#include "core/Interruptible.h"

#include <functional>

class InterruptManager
{
public:
    InterruptManager();
    ~InterruptManager() {}

    static void set_usart6_isr_handler(void(stm32f401::Interruptible::*handler)(void), stm32f401::Interruptible* obj)
    {
        usart6_handler = std::bind(handler, obj);
        /*
        usart6Object = obj;
        usart6_handler = handler;
        */
    }

    static void set_systick_handler(void(stm32f401::Interruptible::*handler)(void), stm32f401::Interruptible* obj)
    {
        systick_handler = std::bind(handler, obj);
        /*
        sysTickObject = obj;
        systick_handler = handler;
        */
    }

    static void usart6_isr_handler()
    {
        usart6_handler();
    }

    static void systick_isr_handler()
    {
        systick_handler();
    }
private:

    static void nullHandler() { return; }

    //static stm32f401::Interruptible* usart6Object;
    //static void(stm32f401::Interruptible::*usart6_handler)(void);
    static std::function<void(void)> usart6_handler;

    //static stm32f401::Interruptible* sysTickObject;
    //static void(stm32f401::Interruptible::*systick_handler)(void);
    static std::function<void(void)> systick_handler;

};

#endif // _INTERRUPTMANAGER_H
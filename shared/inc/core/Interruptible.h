#ifndef CORE_INTERRUPTIBLE_H
#define CORE_INTERRUPTIBLE_H

namespace stm32f401
{
class Interruptible
{
public:
    Interruptible() {}
    virtual ~Interruptible() {}

    virtual void interrupt_handler() = 0;
};
}

#endif // CORE_INTERRUPTIBLE_H
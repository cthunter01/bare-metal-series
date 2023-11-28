#ifndef _INTERRUPTIBLE_H
#define _INTERRUPTIBLE_H

namespace stm32f401
{
class Interruptible
{
public:
    Interruptible() {}
    virtual ~Interruptible() {}

    virtual void interrupt_handler() = 0;

    static void null_interrupt(void) { return ; }
};

} // namespace

#endif // _INTERRUPTIBLE_H

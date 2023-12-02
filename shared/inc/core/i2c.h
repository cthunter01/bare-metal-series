#ifndef _CORE_I2C_H
#define _CORE_I2C_H

#include "core/Interruptible.h"

#include <libopencm3/stm32/i2c.h>

namespace stm32f1
{

struct I2CConfig
{
    
};

enum I2CConfigEnum
{
    I2C1_1 = 0,
    I2C2_1 = 1,
    I2C3_1 = 2,
    I2C_USERCUSTOM = 3
};

class I2C
{
public:

    I2C()

private:


};

} // namespace stm32f4

#endif // _CORE_I2C_H
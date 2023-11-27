#include "core/i2c.h"

#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/gpio.h>

namespace
{

std::array<stm32f401::I2CConfig, NUM_I2C_CONFIGS> buildConfigs()
{
    std::array<stm32f401::I2CConfig, NUM_I2C_CONFIGS> retVal;
    stm32f401::I2CConfig i2c1_1;
    i2c1_1._sclPin = GPIO6;
    i2c1_1._sclGPIOPort = GPIOB;
    i2c1_1._sclGPIO_AF = GPIO_AF4;
    i2c1_1._sclClock = RCC_GPIOB;
    i2c1_1._sdaPin = GPIO7;
    i2c1_1._sdaGPIOPort = GPIOB;
    i2c1_1._sdaGPIO_AF = GPIO_AF4;
    i2c1_1._sdaClock = RCC_GPIOB;
    i2c1_1._i2cClock = RCC_I2C1;
    i2c1_1._i2cNum  = I2C1;
    i2c1_1._rst  = RST_I2C1;
    retVal[stm32f401::I2CConfigEnum::I2C1_1] = i2c1_1;

    stm32f401::I2CConfig i2c1_2;
    i2c1_2._sclPin = GPIO8;
    i2c1_2._sclGPIOPort = GPIOB;
    i2c1_2._sclGPIO_AF = GPIO_AF4;
    i2c1_2._sclClock = RCC_GPIOB;
    i2c1_2._sdaPin = GPIO9;
    i2c1_2._sdaGPIOPort = GPIOB;
    i2c1_2._sdaGPIO_AF = GPIO_AF4;
    i2c1_2._sdaClock = RCC_GPIOB;
    i2c1_2._i2cClock = RCC_I2C1;
    i2c1_2._i2cNum  = I2C1;
    i2c1_2._rst  = RST_I2C1;
    retVal[stm32f401::I2CConfigEnum::I2C1_2] = i2c1_2;

    stm32f401::I2CConfig i2c2_1;
    i2c2_1._sclPin = GPIO10;
    i2c2_1._sclGPIOPort = GPIOB;
    i2c2_1._sclGPIO_AF = GPIO_AF4;
    i2c2_1._sclClock = RCC_GPIOB;
    i2c2_1._sdaPin = GPIO3;
    i2c2_1._sdaGPIOPort = GPIOB;
    i2c2_1._sdaGPIO_AF = GPIO_AF9;
    i2c2_1._sdaClock = RCC_GPIOB;
    i2c2_1._i2cClock = RCC_I2C2;
    i2c2_1._i2cNum  = I2C2;
    i2c2_1._rst  = RST_I2C2;
    retVal[stm32f401::I2CConfigEnum::I2C2_1] = i2c2_1;

    stm32f401::I2CConfig i2c3_1;
    i2c3_1._sclPin = GPIO8;
    i2c3_1._sclGPIOPort = GPIOA;
    i2c3_1._sclGPIO_AF = GPIO_AF4;
    i2c3_1._sclClock = RCC_GPIOA;
    i2c3_1._sdaPin = GPIO9;
    i2c3_1._sdaGPIOPort = GPIOC;
    i2c3_1._sdaGPIO_AF = GPIO_AF4;
    i2c3_1._sdaClock = RCC_GPIOC;
    i2c3_1._i2cClock = RCC_I2C3;
    i2c3_1._i2cNum  = I2C3;
    i2c3_1._rst  = RST_I2C3;
    retVal[stm32f401::I2CConfigEnum::I2C3_1] = i2c3_1;

    return retVal;
}
}

namespace stm32f401
{

//const std::array<I2CConfig, NUM_I2C_CONFIGS> I2C::I2CConfigs = buildConfigs();

I2C::I2C(enum I2CConfigEnum config)
    : I2CConfigs(buildConfigs()),
      _configEnum(config),
      _config(I2CConfigs[config])
{
}

I2C::I2C(const I2CConfig& config)
    : I2CConfigs(buildConfigs()),
      _configEnum(USERCUSTOM),
      _config(config)
{

}

void I2C::start()
{
    // start the hardware clock(s)
    rcc_periph_clock_enable(_config._i2cClock);
    rcc_periph_clock_enable(_config._sclClock);
    rcc_periph_clock_enable(_config._sdaClock);

    rcc_periph_reset_pulse(_config._rst);

    // GPIO mode setup
    // SCL pin
    gpio_mode_setup(_config._sclGPIOPort, GPIO_MODE_AF, GPIO_PUPD_NONE, _config._sclPin);
    gpio_set_af(_config._sclGPIOPort, _config._sclGPIO_AF, _config._sclPin);

    // SDA pin
    gpio_mode_setup(_config._sdaGPIOPort, GPIO_MODE_AF, GPIO_PUPD_NONE, _config._sdaPin);
    gpio_set_af(_config._sdaGPIOPort, _config._sdaGPIO_AF, _config._sdaPin);

    i2c_peripheral_disable(_config._i2cNum);

    //  HSI is 8 MHz
    i2c_set_speed(_config._i2cNum, i2c_speed_fm_400k, 8U);

    //i2c_set_clock_frequency(_config._i2cNum, 4U);
    i2c_peripheral_enable(_config._i2cNum);

}

void I2C::write(uint8_t address, uint8_t* cmd, size_t len)
{
    uint8_t data;
    i2c_transfer7(_config._i2cNum, address, cmd, len, &data, 1);
}

void I2C::read(uint8_t address, uint8_t* data, size_t dataLen, uint8_t* buf, size_t bufLen)
{
    i2c_transfer7(_config._i2cNum, address, data, dataLen, buf, bufLen);
}

void I2C::transfer(uint8_t address, uint8_t* cmd, size_t cmdLen, uint8_t* data, size_t dataLen)
{
    i2c_transfer7(_config._i2cNum, address, cmd, cmdLen, data, dataLen);
}
} // namespace stm32f401
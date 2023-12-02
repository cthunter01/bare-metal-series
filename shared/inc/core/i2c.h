#ifndef SHARED_I2C_H
#define SHARED_I2C_H

#include <cstdint>
#include <array>
#include <vector>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/gpio.h>

#define NUM_I2C1CONFIGS (2)
#define NUM_I2C2CONFIGS (1)
#define NUM_I2C3CONFIGS (1)
#define NUM_I2C_CONFIGS (NUM_I2C1CONFIGS + NUM_I2C2CONFIGS + NUM_I2C3CONFIGS)


namespace stm32f4
{
struct I2CConfig
{
    uint16_t _sclPin;
    uint16_t _sdaPin;
    uint32_t _sclGPIOPort;
    uint32_t _sdaGPIOPort;
    uint32_t _i2cNum;
    uint8_t _sclGPIO_AF;
    uint8_t _sdaGPIO_AF;
    enum rcc_periph_clken _i2cClock;
    enum rcc_periph_clken _sdaClock;
    enum rcc_periph_clken _sclClock;
    enum rcc_periph_rst _rst;
    enum i2c_speeds _speed;

};

enum I2CConfigEnum
{
    I2C1_1 = 0,
    I2C1_2 = 1,
    I2C2_1 = 2,
    I2C3_1 = 3,
    I2C_USERCUSTOM = 4
};

class I2C
{
public:
    I2C(enum I2CConfigEnum config);
    I2C(const I2CConfig& config);

    // nothing special
    ~I2C() { }

    const std::array<I2CConfig, I2C_USERCUSTOM> I2CConfigs;

    void start();

    void write(uint8_t address, uint8_t* reg, uint8_t* data, size_t len);
    void read(uint8_t address, uint8_t* reg, uint8_t* buf, size_t bufLen);

    // This writes and reads simultaneously. Send a command and retrieve the result
    void transfer(uint8_t address,
                  uint8_t* writeRegister,
                  uint8_t* writeBuffer,
                  size_t writeBufferLength,
                  uint8_t* readRegister,
                  uint8_t* readBuffer,
                  size_t readBufferLength);

private:

    enum I2CConfigEnum _configEnum;
    I2CConfig _config;

};

}
#endif // SHARED_I2C_H
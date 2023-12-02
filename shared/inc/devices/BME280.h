#ifndef _BME280_H
#define _BME280_H

#include <memory>
#include <utility>

#include "core/common-defines.h"
#include "core/i2c.h"

// The address of the BMP280 is either 0x76 or 0x77
// depending on the SDO pin pulled low or high (respectively)
//constexpr std::uint8_t addr{0x76};

enum class BME280_OSRS_P
{
    SKIPPED  = 0,
    ULPOWER  = 1,
    LPOWER   = 2,
    STANDARD = 3,
    HIRES    = 4,
    UHIRES   = 5
};

enum class BME280_OSRS_T
{
    SKIPPED  = 0,
    ULPOWER  = 1,
    LPOWER   = 2,
    STANDARD = 3,
    HIRES    = 4,
    UHIRES   = 5
};

enum class BME280_MODE
{
    SLEEP  = 0,
    FORCED = 1,
    NORMAL = 2
};

class BME280
{
public:
    BME280(uint8_t inAddr, std::unique_ptr<stm32f4::I2C>&& i2c);
    ~BME280();

    void start();
    void readCalibration();
    void writeConfig();
    void takeMeasurement();
    //double calculateTrueTemperatureDouble(long ut);
    float calculateTrueTemperature(int32_t ut);
    float calculateTruePressure(int32_t up);

    float getTemperature() { return _lastTemperature; }
    float getPressure() { return _lastPressure; }

    struct BME280CalibrationStruct
    {
    std::uint16_t T1;
	std::int16_t  T2;
	std::int16_t  T3;
	std::uint16_t P1;
	std::int16_t  P2;
	std::int16_t  P3;
	std::int16_t  P4;
	std::int16_t  P5;
	std::int16_t  P6;
	std::int16_t  P7;
	std::int16_t  P8;
	std::int16_t  P9;
    };

    BME280CalibrationStruct getCalibrationData() { return calData; }

    BME280CalibrationStruct calData;

    BME280_MODE mode;
    BME280_OSRS_P osrs_p;
    BME280_OSRS_T osrs_t;

    int32_t t_fine;

    std::unique_ptr<stm32f4::I2C> _i2c;
    uint8_t _addr;

    float _lastTemperature;
    float _lastPressure;

};

#endif // _BMP280_H

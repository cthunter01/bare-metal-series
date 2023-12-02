#include "core/system.h"
#include "core/USART.h"
#include "timer.h"

#include "core/SysTick.h"
#include "core/USART.h"

#include "devices/BME280.h"

#include <libopencm3/stm32/gpio.h>

#include <stdio.h>
#include <memory>

int main(void)
{
begin_main:
  system_setup();
  //timer_setup();
  //uart_setup();

  // Enable System Tick
  stm32f4::SysTick sysTickObject;

  // Enable and create a PWM output on TIMER2
  stm32f4::PWMOutput pwmOutput(stm32f4::PWM_TIM2_1);
  stm32f4::USART usartObject(stm32f4::USART6_2);
  usartObject.start();

  // Start up BME280 pressure sensor
  std::unique_ptr<stm32f4::I2C> i2c = std::make_unique<stm32f4::I2C>(stm32f4::I2C1_1);
  i2c->start();
  BME280 bme280(0x76, std::move(i2c));
  bme280.start();

  BME280::BME280CalibrationStruct calData = bme280.getCalibrationData();
  printf("T1: %u\n", calData.T1);
  printf("T2: %d\n", calData.T2);
  printf("T3: %d\n", calData.T3);
  printf("P1: %u\n", calData.P1);
  printf("P2: %d\n", calData.P2);
  printf("P3: %d\n", calData.P3);
  printf("P4: %d\n", calData.P4);
  printf("P5: %d\n", calData.P5);
  printf("P6: %d\n", calData.P6);
  printf("P7: %d\n", calData.P7);
  printf("P8: %d\n", calData.P8);
  printf("P9: %d\n", calData.P9);

  //uint32_t start_time = get_ticks();
  uint32_t start_time = sysTickObject.getTicks();
  uint32_t flasher_time = 0U;
  uint32_t bme280_time = 0U;
  float duty_cycle = 1.0f;
  float delta = 1.0f;

  while (1)
  {
    if(sysTickObject.getTicks() - start_time >= 10)
    {
      // flash 2x second
      if(sysTickObject.getTicks() - flasher_time >= 500)
      {
        gpio_toggle(LED_PORT, LED_PIN);
        flasher_time = sysTickObject.getTicks();
      }
      duty_cycle += delta;
      if(duty_cycle > 99.0f || duty_cycle < 1.0f)
      {
        delta *= -1.0f;
      }
      pwmOutput.setDutyCycle(duty_cycle);
      //start_time = get_ticks();
      start_time = sysTickObject.getTicks();
    }

    if(sysTickObject.getTicks() - bme280_time >= 1000)
    {
      bme280.takeMeasurement();
      printf("Temperature: %4.2f\n", bme280.getTemperature());
      printf("Pressure: %f\n", bme280.getPressure());
      bme280_time = sysTickObject.getTicks();
    }

    if(usartObject.dataAvailable())
    {
      uint8_t data = usartObject.readByte();

      printf("Recieved: %c\n", static_cast<char>(data));
    }
    // simulate some higher workload
    //sysTickObject.systemDelay(1000);

  }

// If for some reason we broke out of the while loop,
// go back to beginning of main().
goto begin_main;
  // Never return
  return 0;
}

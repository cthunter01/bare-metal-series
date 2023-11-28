#include "core/system.h"
#include "core/uart.h"
#include "timer.h"

#include "core/SysTick.h"

int main(void)
{
  system_setup();
  timer_setup();
  uart_setup();

  stm32f401::SysTick& sysTickObject = stm32f401::SysTick::getInstance();
  sysTickObject.setFrequency(2000U);
  sysTickObject.start();

  uint32_t start_time = sysTickObject.getTicks();
  float duty_cycle = 1.0f;
  float delta = 1.0f;

  while (1)
  {
    if(sysTickObject.getTicks() - start_time >= 10)
    {
      duty_cycle += delta;
      if(duty_cycle > 99.0f || duty_cycle < 1.0f)
      {
        delta *= -1.0f;
      }
      timer_pwm_set_duty_cycle(duty_cycle);
      start_time = sysTickObject.getTicks();
    }

    if(uart_data_available())
    {
      uint8_t data = uart_read_byte();
      uart_write_byte(data + 1);
    }

    // simulate some higher workload
    //system_delay(1000); // 1 second

  }

  // Never return
  return 0;
}

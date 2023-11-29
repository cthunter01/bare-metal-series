#include "core/system.h"
#include "core/uart.h"
#include "timer.h"

#include "core/SysTick.h"

#include <libopencm3/stm32/gpio.h>

int main(void)
{
begin_main:
  system_setup();
  timer_setup();
  uart_setup();

  stm32f4::SysTick sysTickObject;

  //uint32_t start_time = get_ticks();
  uint32_t start_time = sysTickObject.getTicks();
  uint32_t flasher_time = 0U;
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
      timer_pwm_set_duty_cycle(duty_cycle);
      //start_time = get_ticks();
      start_time = sysTickObject.getTicks();
    }

    if(uart_data_available())
    {
      uint8_t data = uart_read_byte();
      uart_write_byte(data + 1);
    }
    //sysTickObject.systemDelay(1000);

    // simulate some higher workload
    //system_delay(1000); // 1 second

  }

// If for some reason we broke out of the while loop,
// go back to beginning of main().
goto begin_main;
  // Never return
  return 0;
}

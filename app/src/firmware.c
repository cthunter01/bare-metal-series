#include <core/system.h>
#include <core/timer.h>

int main(void)
{
begin_main:
  system_setup();

  uint32_t start_time = get_ticks();
  float duty_cycle = 0.0f;

  while (1)
  {
    if(get_ticks() - start_time >= 10)
    {
      duty_cycle += 1.0f;
      if(duty_cycle > 100.0f)
      {
        duty_cycle = 0.0f;
      }
      timer_pwm_set_duty_cycle(duty_cycle);
      start_time = get_ticks();
    }
  }

// If for some reason we broke out of the while loop,
// go back to beginning of main().
goto begin_main;
  // Never return
  return 0;
}

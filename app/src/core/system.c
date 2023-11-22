#include "core/system.h"
#include "core/timer.h"

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/vector.h> // systick interrupt handler. Also
                                   // include nvic.h which declares sys_tick_handler


// volatile because only used in sys_tick_handler which the compiler
// might not see is ever called (it is, but it's an interrupt)
static volatile uint32_t ticks = 0;

// initially defined as a weak function in vector.c. We can redefine
// it for our purposes since the default is an alias for null_handler()
void sys_tick_handler(void)
{
  ticks++;
}

uint32_t get_ticks(void)
{
  return ticks;
}

static void rcc_setup(void)
{
  rcc_clock_setup_pll(&rcc_hsi_configs[RCC_CLOCK_3V3_84MHZ]);
  /*
  rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_180MHZ]);

  while(!rcc_osc_ready_int_flag(RCC_HSE))
  {
    // keep waiting until the clock is stabilized. Don't do anything
  }
  */
}

static void gpio_setup(void)
{

  rcc_periph_clock_enable(RCC_GPIOA);
  gpio_mode_setup(LED_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, LED_PIN);
  gpio_set_af(LED_PORT, GPIO_AF1, LED_PIN);
}

static void systick_setup(void)
{
  systick_set_frequency(SYSTICK_FREQ, CPU_FREQ);
  // need to actually enable
  systick_counter_enable();
  // set interrupt
  // will call sys_tick_handler() when interrupt arrives
  systick_interrupt_enable();
}

void system_setup()
{
    rcc_setup();
    gpio_setup();
    systick_setup();
    timer_setup();

    timer_pwm_set_duty_cycle(0.0f);
}
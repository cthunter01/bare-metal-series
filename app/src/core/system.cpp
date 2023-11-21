#include "core/system.h"

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/vector.h> // systick interrupt handler. Also
                                   // include nvic.h which declares sys_tick_handler


// volatile because only used in sys_tick_handler which the compiler
// might not see is ever called (it is, but it's an interrupt)
volatile uint32_t ticks = 0;

// initially defined as a weak function in vector.c. We can redefine
// it for our purposes since the default is an alias for null_handler()
void sys_tick_handler(void)
{
  ticks++;
  if(ticks % 500 == 0)
  {
    ticks = 0;
    gpio_toggle(LED_PORT, LED_PIN);
  }
}

uint32_t get_ticks(void)
{
  return ticks;
}

static void rcc_setup(void)
{
  rcc_clock_setup_pll(&rcc_hsi_configs[RCC_CLOCK_3V3_84MHZ]);
}

static void gpio_setup(void)
{

  rcc_periph_clock_enable(RCC_GPIOA);
  gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLDOWN, LED_PIN);
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
}
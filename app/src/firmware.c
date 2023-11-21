#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/systick.h>

#include <libopencm3/cm3/vector.h> // systick interrupt handler.
                                   // includes nvic.h which declares sys_tick_handler
                                

#define LED_PORT (GPIOA)
#define LED_PIN  (GPIO5)

#define CPU_FREQ     (84000000)
#define SYSTICK_FREQ (1000)


// volatile because only used in sys_tick_handler which the compiler
// might not see is ever called (it is, but it's an interrupt)
volatile uint32_t ticks = 0;

// initially defined as a weak function in vector.c. We can redefine
// it for our purposes since the default is an alias for null_handler()
void sys_tick_handler(void)
{
  ticks++;
  if(ticks % 100 == 0)
  {
    ticks = 0;
    gpio_toggle(LED_PORT, LED_PIN);
  }
}

/*
static uint64_t get_ticks(void)
{
  return ticks;
}
*/

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

int main(void)
{
begin_main:
  rcc_setup();
  gpio_setup();
  systick_setup();

  //uint64_t start_time = get_ticks();
  while (1)
  {
    /*
    if(get_ticks() - start_time > 1000)
    {
      gpio_toggle(LED_PORT, LED_PIN);
      start_time = get_ticks();
    }
    */
    // do useful work
  }

// If for some reason we broke out of the while loop,
// go back to beginning of main().
goto begin_main;

  // Never return
  return 0;
}

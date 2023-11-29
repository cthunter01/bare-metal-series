#include "core/system.h"

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/vector.h> // systick interrupt handler. Also
                                   // include nvic.h which declares sys_tick_handler


/*
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
*/
static void rcc_setup(void)
{
  //rcc_clock_setup_pll(&rcc_hsi_configs[RCC_CLOCK_3V3_96MHZ]);
  rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_96MHZ]);
  //rcc_clock_setup_pll(&rcc_hsi_configs[RCC_CLOCK_3V3_84MHZ]);
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

  rcc_periph_clock_enable(LED_PORT_CLOCK);

  // OMG don't forget this. enabling RCC_USART6 isn't enough if the
  // pins are on GPIOC. Also have to enable the GPIOC clock
  rcc_periph_clock_enable(USART_PORT_CLOCK);
  
  // LED setup
  // NUCLEO only set up for one LED
#ifdef NUCLEOF401RE
  gpio_mode_setup(LED_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, LED_PIN);
  gpio_set_af(LED_PORT, GPIO_AF1, LED_PIN);
#else
  rcc_periph_clock_enable(LED_PORT_CLOCK2);
  gpio_mode_setup(LED_PORT2, GPIO_MODE_AF, GPIO_PUPD_NONE, LED_PIN2);
  gpio_set_af(LED_PORT2, GPIO_AF1, LED_PIN2);
  gpio_mode_setup(LED_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);
#endif

  // UART setup
  // PC6 and PC7 are USART TX and RX respectively
  gpio_mode_setup(USART_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, USART_TX | USART_RX);
  // USART is AF8
  gpio_set_af(USART_PORT, GPIO_AF8, USART_TX | USART_RX);

}

/*
static void systick_setup(void)
{
  systick_set_frequency(SYSTICK_FREQ, CPU_FREQ);
  // need to actually enable
  systick_counter_enable();
  // set interrupt
  // will call sys_tick_handler() when interrupt arrives
  systick_interrupt_enable();
}
*/

void system_setup()
{
    rcc_setup();
    gpio_setup();
    //systick_setup();

}

/*
void system_delay(uint32_t milliseconds)
{
  uint32_t end_time = get_ticks() + milliseconds;
  while(get_ticks() < end_time)
  {
    // spin
  }
}
*/


#include <libopencm3/stm32/rcc.h>

#ifdef STM32F7

static void rcc_setup(void)
{
}


int main(void)
{
begin_main:



goto begin_main;
return 0;
}





#endif
#ifdef STM32H7

#define RCC_LED1  RCC_GPIOB
#define PORT_LED1 GPIOB
#define PIN_LED1  GPIO0

#define RCC_LED2  RCC_GPIOE
#define PORT_LED2 GPIOE
#define PIN_LED2  GPIO1


static void setup(void)
{
    rcc_periph_clock_enable(RCC_LED1);
    gpio_mode_setup(PORT_LED1, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN_LED1);
    gpio_set(PORT_LED1, PIN_LED1);
    rcc_periph_clock_enable(RCC_LED2);
    gpio_mode_setup(PORT_LED2, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN_LED2);

    gpio_toggle(PORT_LED1, PIN_LED1);
}

int main(void) {
    setup();
begin_main:
    /* wait a little bit */
    for (int i = 0; i < 20000000; i++)
    {
        __asm__("nop");
    }
    gpio_toggle(PORT_LED1, PIN_LED1);
    gpio_toggle(PORT_LED2, PIN_LED2);

goto begin_main;
    // never return
    return 0;
}

#endif
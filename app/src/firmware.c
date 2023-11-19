#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#define RCC_LED1  RCC_GPIOB
#define PORT_LED1 GPIOB
#define PIN_LED1  GPIO0

#define RCC_LED2  RCC_GPIOE
#define PORT_LED2 GPIOE
#define PIN_LED2  GPIO1


int main(void) {
    rcc_periph_clock_enable(RCC_LED1);
    gpio_mode_setup(PORT_LED1, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN_LED1);
    gpio_set(PORT_LED1, PIN_LED1);
    #if defined(RCC_LED2)
    rcc_periph_clock_enable(RCC_LED2);
    gpio_mode_setup(PORT_LED2, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN_LED2);
    #endif
    while(1)
    {
        /* wait a little bit */
        for (int i = 0; i < 10000000; i++)
	{
            __asm__("nop");
        }
    gpio_toggle(PORT_LED1, PIN_LED1);
    gpio_toggle(PORT_LED2, PIN_LED2);
}
}

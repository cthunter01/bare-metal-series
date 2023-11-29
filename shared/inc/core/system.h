#ifndef INC_CORE_SYSTEM_H
#define INC_CORE_SYSTEM_H

#include "common-defines.h"

#ifdef NUCLEOF401RE
#define LED_PORT (GPIOA)
#define LED_PORT_CLOCK (RCC_GPIOA)
#define LED_PIN  (GPIO5)
#define CPU_FREQ     (84000000)
#define USART_PORT (GPIOC)
#define USART_PORT_CLOCK (RCC_GPIOC)
#define USART_TX   (GPIO6)
#define USART_RX   (GPIO7)
#endif

#ifdef BLACKPILLF411
#define LED_PORT (GPIOC)
#define LED_PIN  (GPIO13)
#define LED_PORT_CLOCK (RCC_GPIOC)
#define LED_PORT2 (GPIOA)
#define LED_PIN2  (GPIO5)
#define LED_PORT_CLOCK2 (RCC_GPIOA)
#define CPU_FREQ     (96000000)
#define USART_PORT (GPIOA)
#define USART_PORT_CLOCK (RCC_GPIOA)
#define USART_TX   (GPIO11)
#define USART_RX   (GPIO12)
#endif

#define SYSTICK_FREQ (1000)
void system_setup(void);

//uint32_t get_ticks(void);

//void system_delay(uint32_t milliseconds);
#endif // INC_CORE_SYSTEM_H
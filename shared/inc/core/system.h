#ifndef INC_CORE_SYSTEM_H
#define INC_CORE_SYSTEM_H

#include "common-defines.h"

#define LED_PORT (GPIOA)
#define LED_PIN  (GPIO5)

#define USART_PORT (GPIOC)
#define USART_TX   (GPIO6)
#define USART_RX   (GPIO7)

#define CPU_FREQ     (84000000)
#define SYSTICK_FREQ (1000)
void system_setup(void);

uint32_t get_ticks(void);

void system_delay(uint32_t milliseconds);
#endif // INC_CORE_SYSTEM_H
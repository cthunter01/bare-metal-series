#include "core/uart.h"
#include "core/system.h"
#include "core/ring-buffer.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

#define RINGBUFFER_SIZE (128) // for maximum of ~10ms before updating rb

static uint8_t data_buffer[RINGBUFFER_SIZE] = {0U};

static ring_buffer_t rb;

// Found in vector_nvic.c. weak alias to blocking_handler,
// need to redefine
void usart6_isr(void)
{
    const bool overrun_occurred = usart_get_flag(USART6, USART_FLAG_ORE) == 1;
    const bool received_data = usart_get_flag(USART6, USART_FLAG_RXNE) == 1;

    if(received_data || overrun_occurred)
    {
        if(ring_buffer_write(&rb, static_cast<uint8_t>(usart_recv(USART6))))
        {
            // handle failure?
        }
    }
}

void uart_setup()
{
    // set up our ring_buffer. size must be power of 2
    ring_buffer_setup(&rb, data_buffer, RINGBUFFER_SIZE);
    // As always, enable the peripheral clock
    rcc_periph_clock_enable(RCC_USART6);
    // Transmit and recieve. Can just transmit or just recieve,
    // which might save some power
    usart_set_mode(USART6, USART_MODE_TX_RX);

    usart_set_flow_control(USART6, USART_FLOWCONTROL_NONE);
    usart_set_baudrate(USART6, 115200);
    
    // 8N1
    usart_set_databits(USART6, 8);
    usart_set_parity(USART6, USART_PARITY_NONE);
    usart_set_stopbits(USART6, 1);


    // set up interrupts on recieve
    usart_enable_rx_interrupt(USART6);
    nvic_enable_irq(NVIC_USART6_IRQ);

    // all of the previous was just configuration of the peripheral,
    // and turning on the hardware clock. This actually tells the peripheral
    // to start
    usart_enable(USART6);

}

void uart_write(uint8_t* data, const uint32_t len)
{
    for(uint32_t i = 0; i < len; ++i)
    {
        uart_write_byte(data[i]);
    }
}

void uart_write_byte(uint8_t data)
{
    usart_send_blocking(USART6, static_cast<uint16_t>(data));
}

uint32_t uart_read(uint8_t* data, const uint32_t len)
{
    if(len  <= 0)
    {
        return 0;
    }

    for(uint32_t bytes_read = 0; bytes_read < len; ++bytes_read)
    {
        if(!ring_buffer_read(&rb, &data[bytes_read]))
        {
            // we didn't read the full length requested
            return bytes_read;
        }
    }
    return len;
}

uint8_t uart_read_byte()
{
    uint8_t byte = 0;
    uart_read(&byte, 1);
    return byte;
}

bool uart_data_available()
{
    return !ring_buffer_empty(&rb);
}

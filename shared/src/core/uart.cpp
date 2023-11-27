#include "core/uart.h"
#include "core/system.h"
#include "core/ring-buffer.h"
#include "core/InterruptManager.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>

#include <functional>

namespace
{
std::array<stm32f401::USARTConfig, stm32f401::USERCUSTOM> buildConfigs()
{
    std::array<stm32f401::USARTConfig, stm32f401::USERCUSTOM> retVal;
    stm32f401::USARTConfig usart1_1;
    usart1_1.usartNum    = USART1;
    usart1_1.txPin      = GPIO9;
    usart1_1.txPinPort  = GPIOA;

    usart1_1.rxPin      = GPIO10;
    usart1_1.rxPinPort  = GPIOA;

    usart1_1.ckPin      = GPIO8;
    usart1_1.ckPinPort  = GPIOA;

    usart1_1.rtsPin     = GPIO12;
    usart1_1.rtsPinPort = GPIOA;

    usart1_1.ctsPin     = GPIO11;
    usart1_1.ctsPinPort = GPIOA;

    usart1_1.gpio_AF    = GPIO_AF7;

    usart1_1.irq         = NVIC_USART1_IRQ;
    usart1_1.usartMode   = USART_MODE_TX_RX;
    usart1_1.flowControl = USART_FLOWCONTROL_NONE;
    usart1_1.baudRate    = 115200U;
    usart1_1.parity      = USART_PARITY_NONE;
    usart1_1.stopBits    = 1U;
    usart1_1.dataBits    = 8U;
    usart1_1.clk         = RCC_USART1;

    usart1_1.hasCtsRts   = true;
    usart1_1.useCtsRts   = false;
    retVal[stm32f401::USART1_1] = usart1_1;

//// USART1_2
    stm32f401::USARTConfig usart1_2;
    usart1_2.usartNum    = USART1;
    usart1_2.txPin      = GPIO6;
    usart1_2.txPinPort  = GPIOB;

    usart1_2.rxPin      = GPIO7;
    usart1_2.rxPinPort  = GPIOB;

    usart1_2.ckPin      = GPIO8;
    usart1_2.ckPinPort  = GPIOA;

    usart1_2.rtsPin     = GPIO12;
    usart1_2.rtsPinPort = GPIOA;

    usart1_2.ctsPin     = GPIO11;
    usart1_2.ctsPinPort = GPIOA;

    usart1_2.gpio_AF    = GPIO_AF7;

    usart1_2.irq         = NVIC_USART1_IRQ;
    usart1_2.usartMode   = USART_MODE_TX_RX;
    usart1_2.flowControl = USART_FLOWCONTROL_NONE;
    usart1_2.baudRate    = 115200U;
    usart1_2.parity      = USART_PARITY_NONE;
    usart1_2.stopBits    = 1U;
    usart1_2.dataBits    = 8U;
    usart1_2.clk         = RCC_USART1;

    usart1_2.hasCtsRts   = true;
    usart1_2.useCtsRts   = false;
    retVal[stm32f401::USART1_2] = usart1_2;

//// USART2_1
    stm32f401::USARTConfig usart2_1;
    usart2_1.usartNum    = USART2;
    usart2_1.txPin      = GPIO5;
    usart2_1.txPinPort  = GPIOD;

    usart2_1.rxPin      = GPIO6;
    usart2_1.rxPinPort  = GPIOD;

    usart2_1.ckPin      = GPIO7;
    usart2_1.ckPinPort  = GPIOD;

    usart2_1.rtsPin     = GPIO4;
    usart2_1.rtsPinPort = GPIOD;

    usart2_1.ctsPin     = GPIO3;
    usart2_1.ctsPinPort = GPIOD;

    usart2_1.gpio_AF    = GPIO_AF7;

    usart2_1.irq         = NVIC_USART2_IRQ;
    usart2_1.usartMode   = USART_MODE_TX_RX;
    usart2_1.flowControl = USART_FLOWCONTROL_NONE;
    usart2_1.baudRate    = 115200U;
    usart2_1.parity      = USART_PARITY_NONE;
    usart2_1.stopBits    = 1U;
    usart2_1.dataBits    = 8U;
    usart2_1.clk         = RCC_USART2;

    usart2_1.hasCtsRts   = true;
    usart2_1.useCtsRts   = false;
    retVal[stm32f401::USART2_1] = usart2_1;

//// USART2_2
    stm32f401::USARTConfig usart2_2;
    usart2_2.usartNum    = USART2;
    usart2_2.txPin      = GPIO2;
    usart2_2.txPinPort  = GPIOA;

    usart2_2.rxPin      = GPIO3;
    usart2_2.rxPinPort  = GPIOA;

    usart2_2.ckPin      = GPIO4;
    usart2_2.ckPinPort  = GPIOA;

    usart2_2.rtsPin     = GPIO1;
    usart2_2.rtsPinPort = GPIOA;

    usart2_2.ctsPin     = GPIO0;
    usart2_2.ctsPinPort = GPIOA;

    usart2_2.gpio_AF    = GPIO_AF7;

    usart2_2.irq         = NVIC_USART2_IRQ;
    usart2_2.usartMode   = USART_MODE_TX_RX;
    usart2_2.flowControl = USART_FLOWCONTROL_NONE;
    usart2_2.baudRate    = 115200U;
    usart2_2.parity      = USART_PARITY_NONE;
    usart2_2.stopBits    = 1U;
    usart2_2.dataBits    = 8U;
    usart2_2.clk         = RCC_USART2;

    usart2_2.hasCtsRts   = true;
    usart2_2.useCtsRts   = false;
    retVal[stm32f401::USART2_2] = usart2_2;

//// USART6_1
    stm32f401::USARTConfig usart6_1;
    usart6_1.usartNum    = USART6;
    usart6_1.txPin      = GPIO6;
    usart6_1.txPinPort  = GPIOC;

    usart6_1.rxPin      = GPIO7;
    usart6_1.rxPinPort  = GPIOC;

    usart6_1.ckPin      = GPIO8;
    usart6_1.ckPinPort  = GPIOC;

    usart6_1.rtsPin     = 0;
    usart6_1.rtsPinPort = 0;

    usart6_1.ctsPin     = 0;
    usart6_1.ctsPinPort = 0;

    usart6_1.gpio_AF    = GPIO_AF8;

    usart6_1.irq         = NVIC_USART6_IRQ;
    usart6_1.usartMode   = USART_MODE_TX_RX;
    usart6_1.flowControl = USART_FLOWCONTROL_NONE;
    usart6_1.baudRate    = 115200U;
    usart6_1.parity      = USART_PARITY_NONE;
    usart6_1.stopBits    = 1U;
    usart6_1.dataBits    = 8U;
    usart6_1.clk         = RCC_USART6;

    usart6_1.hasCtsRts   = false;
    usart6_1.useCtsRts   = false;
    retVal[stm32f401::USART6_1] = usart6_1;

//// USART6_2
    stm32f401::USARTConfig usart6_2;
    usart6_2.usartNum    = USART6;
    usart6_2.txPin      = GPIO11;
    usart6_2.txPinPort  = GPIOA;

    usart6_2.rxPin      = GPIO12;
    usart6_2.rxPinPort  = GPIOA;

    usart6_2.ckPin      = GPIO8;
    usart6_2.ckPinPort  = GPIOC;

    usart6_2.rtsPin     = 0;
    usart6_2.rtsPinPort = 0;

    usart6_2.ctsPin     = 0;
    usart6_2.ctsPinPort = 0;

    usart6_2.gpio_AF    = GPIO_AF8;

    usart6_2.irq         = NVIC_USART6_IRQ;
    usart6_2.usartMode   = USART_MODE_TX_RX;
    usart6_2.flowControl = USART_FLOWCONTROL_NONE;
    usart6_2.baudRate    = 115200U;
    usart6_2.parity      = USART_PARITY_NONE;
    usart6_2.stopBits    = 1U;
    usart6_2.dataBits    = 8U;
    usart6_2.clk         = RCC_USART6;

    usart6_2.hasCtsRts   = false;
    usart6_2.useCtsRts   = false;
    retVal[stm32f401::USART6_2] = usart6_2;

    return retVal;
}

uint8_t data_buffer[128] = {0};

ring_buffer_t rb;
}

namespace stm32f401
{

USART::USART(enum USARTConfigEnum config)
    : UARTConfigs(buildConfigs()),
      _configEnum(config),
      _config(UARTConfigs[config]),
      _dataBuffer({0}),
      _rb()
{
    // set up our ring_buffer. size must be power of 2
    ring_buffer_setup(&_rb, _dataBuffer, 128);

    InterruptManager::set_usart6_isr_handler(&Interruptible::interrupt_handler, this);
    
}

void USART::start()
{
    // As always, enable the peripheral clock
    rcc_periph_clock_enable(_config.clk);
    // Transmit and recieve. Can just transmit or just recieve,
    // which might save some power
    usart_set_mode(_config.usartNum, _config.usartMode);

    usart_set_flow_control(_config.usartNum, _config.flowControl);
    usart_set_baudrate(_config.usartNum, _config.baudRate);
    
    // 8N1
    usart_set_databits(_config.usartNum, _config.dataBits);
    usart_set_parity(_config.usartNum, _config.parity);
    usart_set_stopbits(_config.usartNum, _config.stopBits);


    // set up interrupts on recieve
    usart_enable_rx_interrupt(_config.usartNum);
    nvic_enable_irq(_config.irq);

    // all of the previous was just configuration of the peripheral,
    // and turning on the hardware clock. This actually tells the peripheral
    // to start
    usart_enable(_config.usartNum);


}

void USART::interrupt_handler(void)
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


}

void uart_setup(void) {}

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

///////////////////////////////////
// **** VERY IMPORTANT TO REDEFINE THESE!!!

// Found in vector_nvic.c. weak alias to blocking_handler,
// need to redefine

extern "C"
{
}
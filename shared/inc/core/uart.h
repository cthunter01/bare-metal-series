#ifndef SHARED_UART_H
#define SHARED_UART_H

#include <array>

#include "common-defines.h"
#include "ring-buffer.h"
#include "core/Interruptible.h"

#include <libopencm3/stm32/rcc.h>

namespace stm32f401
{

struct USARTConfig
{
    uint32_t usartNum;
    uint16_t txPin;
    uint16_t rxPin;
    uint16_t ckPin;
    uint16_t rtsPin;
    uint16_t ctsPin;

    // USART GPIO AF seems consistent for any given USART, so
    // not split among different AF for different pins
    uint8_t gpio_AF;

    uint8_t irq;
    uint32_t txPinPort;
    uint32_t rxPinPort;
    uint32_t ckPinPort;
    uint32_t rtsPinPort;
    uint32_t ctsPinPort;
    uint32_t usartMode;
    uint32_t flowControl;
    uint32_t baudRate;
    uint32_t parity;
    uint32_t stopBits;
    uint8_t  dataBits;
    bool hasCtsRts;
    bool useCtsRts;
    enum rcc_periph_clken clk;

};

enum USARTConfigEnum
{
    USART1_1 = 0,
    USART1_2 = 1,
    USART2_1 = 2,
    USART2_2 = 3,
    USART6_1 = 4,
    USART6_2 = 5,
    USERCUSTOM = 6 // also used as # of predefined configs for convenience
};

class USART : public Interruptible
{
public:
    USART(enum USARTConfigEnum config);
    USART(const USARTConfig& config);
    virtual ~USART() {}

    const std::array<USARTConfig, USERCUSTOM> UARTConfigs;

    void start();

    void write();

    void read();

    void interrupt_handler() override;

private:

    enum USARTConfigEnum _configEnum;
    USARTConfig _config;

    uint8_t _dataBuffer[128];

    ring_buffer_t _rb;

};

} // namespace 

/**
 * @brief UART setup code. Enables clock, sets up parameters (8N1)
*/
void uart_setup();

/**
 * @brief Writes a data buffer to the UART port
 * @param data pointer to a uint8_t buffer containing data to write
 * @param len Number of bytes to write. Should be less than or equal
 *            to the length of the data array
 * 
*/
void uart_write(uint8_t* data, const uint32_t len);

/**
 * @brief Writes a single byte to the UART port
 * @param data data byte to write
*/
void uart_write_byte(uint8_t data);

/**
 * @brief Reads the UART recieve buffer
 * @param data pointer to a uint8_t array to populate with data read
 * @param len Number of bytes to read
 * @return number of bytes actually read
*/
uint32_t uart_read(uint8_t* data, const uint32_t len);

/**
 * @brief Reads a single byte
 * @return byte read
*/
uint8_t uart_read_byte();

/**
 * @brief Returns true if there is data to be read. False otherwise
 * @return Whether or not there is data in the UART buffer to be read
*/
bool uart_data_available();

#endif // SHARED_UART_H
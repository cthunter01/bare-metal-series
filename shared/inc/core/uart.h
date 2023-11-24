#ifndef SHARED_UART_H
#define SHARED_UART_H

#include "common-defines.h"

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
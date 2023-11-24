#include "comms.h"
#include "core/uart.h"
#include "core/crc8.h"
#include "core/ring-buffer.h"

#define PACKET_BUFFER_SIZE (8)

typedef enum comms_state_t
{
    CommsState_Length,
    CommsState_Data,
    CommsState_CRC,
} comms_state_t;

static comms_state_t state = comms_state_t::CommsState_Length;
static uint8_t data_byte_count = 0;

static comms_packet_t temporary_packet = 
{
    .len = 0,
    .data = {0xaa},
    .crc = 0
};
comms_packet_t retx_packet;
comms_packet_t ack_packet;
comms_packet_t last_xmit_packet = 
{
    .len = 0,
    .data = {0xaa},
    .crc = 0
};

static comms_packet_t packet_buffer[PACKET_BUFFER_SIZE];
static uint32_t packet_read_index = 0;
static uint32_t packet_write_index = 0;
static uint32_t packet_buffer_mask = PACKET_BUFFER_SIZE;

static void comms_packet_copy(const comms_packet_t* src, comms_packet_t* dest)
{
    dest->len = src->len;
    for(uint8_t i = 0; i < PACKET_DATA_LENGTH; ++i)
    {
        dest->data[i] = src->data[i];
    }
    dest->crc = src->crc;
}

// Assume valid crc
static bool comms_is_retx_packet(const comms_packet_t* packet)
{
    if(packet->len != 1)
    {
        return false;
    }
    if(packet->data[0] != PACKET_RETX_DATA0)
    {
        return false;
    }
    for(uint8_t i = 1; i < PACKET_DATA_LENGTH; ++i)
    {
        if(packet->data[i] != 0xaa)
        {
            return false;
        }
    }
    return true;
};

// Assume valid crc
static bool comms_is_ack_packet(const comms_packet_t* packet)
{
    if(packet->len != 1)
    {
        return false;
    }
    if(packet->data[0] != PACKET_ACK_DATA0)
    {
        return false;
    }
    for(uint8_t i = 1; i < PACKET_DATA_LENGTH; ++i)
    {
        if(packet->data[i] != 0xaa)
        {
            return false;
        }
    }
    return true;
}

void comms_setup()
{
    retx_packet.len = 1;
    retx_packet.data[0] = PACKET_RETX_DATA0;
    for(uint8_t i = 1; i < PACKET_DATA_LENGTH; ++i)
    {
        retx_packet.data[i] = 0xaa;
    }
    retx_packet.crc = comms_compute_crc(&retx_packet);

    ack_packet.len = 1;
    ack_packet.data[0] = PACKET_ACK_DATA0;
    for(uint8_t i = 1; i < PACKET_DATA_LENGTH; ++i)
    {
        ack_packet.data[i] = 0xaa;
    }
    ack_packet.crc = comms_compute_crc(&ack_packet);

}

void comms_update()
{
    while(uart_data_available())
    {
        switch(state)
        {
            case CommsState_Length:
            {
                temporary_packet.len = uart_read_byte();
                state = CommsState_Data;
                break;
            }
            case CommsState_Data:
            {
                temporary_packet.data[data_byte_count++] = uart_read_byte();
                if(data_byte_count >= PACKET_DATA_LENGTH)
                {
                    data_byte_count = 0;
                    state = CommsState_CRC;
                }
                break;
            }
            case CommsState_CRC:
            {
                temporary_packet.crc = uart_read_byte();

                if(temporary_packet.crc != comms_compute_crc(&temporary_packet))
                {
                    comms_write_packet(&retx_packet);
                    state = CommsState_Length;
                    break;
                }
                if(comms_is_retx_packet(&temporary_packet))
                {
                    comms_write_packet(&last_xmit_packet);
                    state = CommsState_Length;
                    break;
                }
                if(comms_is_ack_packet(&temporary_packet))
                {
                    // great, transition to Length
                    state = CommsState_Length;
                }

                uint32_t next_write_index = (packet_write_index + 1) & packet_buffer_mask;
                if(next_write_index == packet_write_index)
                {
                    // inserts an assembly instruction to cause
                    // a breakpoint. If debugging, we can hit this
                    __asm__("BKPT #0");
                }

                comms_packet_copy(&temporary_packet, &packet_buffer[packet_write_index]);
                packet_write_index = next_write_index;
                comms_write_packet(&ack_packet);
                // great, transition to Length
                state = CommsState_Length;

                break;
            }
            default:
                state = CommsState_Length;
        }
    }
}

bool comms_packets_available()
{
    return (packet_read_index != packet_write_index);
}

void comms_write_packet(comms_packet_t* packet)
{
    uart_write(reinterpret_cast<uint8_t*>(packet), PACKET_LENGTH);
}
void comms_read_packet(comms_packet_t* packet)
{
    if(comms_packets_available())
    {
        comms_packet_copy(&packet_buffer[packet_read_index], packet);
    }
}

uint8_t comms_compute_crc(comms_packet_t* packet)
{
    return crc8(reinterpret_cast<uint8_t*>(&packet), PACKET_LENGTH - PACKET_CRC_BYTES);
}
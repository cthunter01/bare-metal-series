#ifndef INC_COMMS_H
#define INC_COMMS_H

#include "core/common-defines.h"

#define PACKET_DATA_LENGTH 16
#define PACKET_LENGTH_BYTES (1)
#define PACKET_CRC_BYTES (1)
#define PACKET_LENGTH (PACKET_DATA_LENGTH + PACKET_LENGTH_BYTES + PACKET_CRC_BYTES)

#define PACKET_RETX_DATA0 (0x19)
#define PACKET_ACK_DATA0  (0x15)

typedef struct comms_packet_t
{
    uint8_t len;
    uint8_t data[PACKET_DATA_LENGTH];
    uint8_t crc;

} comms_packet_t;

// whole setup for packet state machine
void comms_setup();
void comms_update();

bool comms_packets_available();
void comms_write_packet(comms_packet_t* packet);
void comms_read_packet(comms_packet_t* packet);

uint8_t comms_compute_crc(comms_packet_t* packet);


#endif // INC_COMMS_H
#ifndef SHARED_RING_BUFFER_H
#define SHARED_RING_BUFFER_H

// for single producer/single consumer, this gives
// us certain guarantees

#include "core/common-defines.h"

typedef struct ring_buffer_t
{
    uint8_t* buffer;
    uint32_t mask; // for wrapping
    uint32_t read_index;
    uint32_t write_index;
} ring_buffer_t;

void ring_buffer_setup(ring_buffer_t* rb,
                       uint8_t* buffer,
                       uint32_t size);

bool ring_buffer_empty(ring_buffer_t* rb);

// returns whether successful or not. If the buffer is
// full, then this will be false. otherwise true
bool ring_buffer_write(ring_buffer_t* rb, uint8_t byte);

// returns true if successful. If unsuccessful, then buffer
// was empty
bool ring_buffer_read(ring_buffer_t* rb, uint8_t* byte);

#endif // SHARED_RING_BUFFER_H
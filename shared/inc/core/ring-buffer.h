#ifndef _RING_BUFFER_H
#define _RING_BUFFER_H

// for single producer/single consumer, this gives
// us certain guarantees

#include "core/common-defines.h"

struct ring_buffer_t
{
    uint8_t* buffer;
    uint32_t mask; // for wrapping
    uint32_t read_index;
    uint32_t write_index;

    ring_buffer_t(uint8_t size);
    ring_buffer_t(const ring_buffer_t&) = delete;
    ring_buffer_t(ring_buffer_t&&) = delete;

    ring_buffer_t& operator=(const ring_buffer_t&) = delete;
    ring_buffer_t& operator=(ring_buffer_t&&) = delete;
    ~ring_buffer_t();

    bool empty();

    // returns whether successful or not. If the buffer is
    // full, then this will be false. otherwise true
    bool write(uint8_t byte);

    // returns true if successful. If unsuccessful, then buffer
    // was empty
    bool read(uint8_t* byte);

};
#endif // _RING_BUFFER_H

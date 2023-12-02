#include "core/ring-buffer.h"

ring_buffer_t::ring_buffer_t(uint8_t size)
  : buffer{nullptr},
    mask{size - 1U},
    read_index{0},
    write_index{0}
{
    buffer = new uint8_t[size];
}

ring_buffer_t::~ring_buffer_t()
{
    delete[] buffer;
}

bool ring_buffer_t::empty()
{
    return read_index == write_index;
}

bool ring_buffer_t::write(uint8_t byte)
{
    // probably unecessary in case of write being
    // called in an interrupt
    uint32_t local_write_index = write_index;
    uint32_t local_read_index = read_index;
    
    uint32_t next_write_index = (local_write_index + 1) & mask;
    if(local_read_index == next_write_index)
    {
        // Lose this byte
        return false;
    }
    buffer[local_write_index] = byte;
    write_index = next_write_index;
    return true;

}

bool ring_buffer_t::read(uint8_t* byte)
{
    // Be careful to not just remove this local copy
    // In case of multiple readers, we want an immediate
    // copy of index to keep another reader from incrementing
    // under us
    uint32_t local_read_index = read_index;
    if(local_read_index == write_index)
    {
        return false;
    }

    *byte = buffer[local_read_index];
    local_read_index = (local_read_index + 1) & mask;
    read_index = local_read_index;
    return true;
}
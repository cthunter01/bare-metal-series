#include "core/syscalls.h"
#include <errno.h>

#include <libopencm3/stm32/usart.h>

int _getpid_r(void)
{
    return 1;
}

int _kill_r(int, int)
{
    return -1;
}

int _write(int file, char* ptr, int len)
{
    int i;

    if(file == 1)
    {
        for(i = 0; i < len; ++i)
        {
            if(ptr[i] == '\n')
            {
                usart_send_blocking(USART_STDIO, '\r');
            }
            usart_send_blocking(USART_STDIO, ptr[i]);
        }
        return i;
    }
    errno = EIO;
    return -1;
}

int _close(int)
{
    return 1;
}

int _fstat(int, struct stat*)
{
    return 1;
}

int _isatty(int)
{
    return 1;
}

off_t _lseek(int, off_t, int)
{
    return 1;
}

ssize_t _read(int, void*, size_t)
{
    return 1;
}
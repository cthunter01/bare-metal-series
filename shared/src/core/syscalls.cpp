#include "core/syscalls.h"

int _getpid_r(void)
{
    return 1;
}

int _kill_r(int, int)
{
    return -1;
}
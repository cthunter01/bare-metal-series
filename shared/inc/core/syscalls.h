#ifndef _SYSCALLS_H
#define _SYSCALLS_H

// Fake/stubbed system calls to keep libnosys happy. We don't actually need
// these to do anything, but they need to be present to keep the linker
// happy
extern "C"
{
int _getpid_r(void);

int _kill_r(int, int);
}
#endif // _SYSCALLS_H

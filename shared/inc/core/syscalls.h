#ifndef _SYSCALLS_H
#define _SYSCALLS_H

#include <sys/types.h>

// Fake/stubbed system calls to keep libnosys happy. We don't actually need
// these to do anything, but they need to be present to keep the linker
// happy
extern "C"
{
int _getpid_r(void);

int _kill_r(int, int);

int _write(int file, char *ptr, int len);

int _close(int);
int _fstat(int, struct stat* buf);
int _isatty(int);
off_t _lseek(int, off_t, int);
ssize_t _read(int, void*, size_t);
}
#endif // _SYSCALLS_H

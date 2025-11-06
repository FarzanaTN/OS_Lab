#ifndef __UNISTD_H
#define __UNISTD_H

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

#define STDIN_FILENO  0
#define STDOUT_FILENO 1
#define STDERR_FILENO 2

/* ---------- EXACT prototypes ---------- */
ssize_t write(int fd, const void *buf, size_t n);
ssize_t read(int fd, void *buf, size_t n);
int     getpid(void);
void    yield(void);
void    exitt(void);
int     reboot(void);
uint32_t getSysTickTime(void);

#endif /* __UNISTD_H */
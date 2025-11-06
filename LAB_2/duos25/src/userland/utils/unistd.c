#include <unistd.h>          /* <-- brings the prototypes above */
#include <syscall_def.h>
#include <kstdio.h>

/* ------------------------------------------------------------------ */
/* Inline SVC helper – keep it static inline so the compiler can see it */
/* ------------------------------------------------------------------ */
static inline __attribute__((always_inline))
int32_t svc_call(uint16_t svc_id, uint32_t a0, uint32_t a1, uint32_t a2)
{
    uint32_t ret;
    __asm volatile (
        "mov   r0, %1\n"
        "mov   r1, %2\n"
        "mov   r2, %3\n"
        "mov   r3, %4\n"
        "svc   #0\n"
        "mov   %0, r0\n"
        : "=r" (ret)
        : "r" ((uint32_t)svc_id), "r" (a0), "r" (a1), "r" (a2)
        : "r0","r1","r2","r3","r12","lr","memory","cc"
    );
    return (int32_t)ret;
}

/* ------------------------------------------------------------------ */
/* Implementations – **MUST MATCH the prototypes in unistd.h exactly**  */
/* ------------------------------------------------------------------ */

uint32_t getSysTickTime(void)
{
    return (uint32_t)svc_call(SYS___time, 0u, 0u, 0u);
}

/* ---- write ---- */
ssize_t write(int fd, const void *buf, size_t n)
{
    return (ssize_t)svc_call(SYS_write,
                            (uint32_t)fd,
                            (uint32_t)buf,
                            (uint32_t)n);
}

/* ---- read ---- */
ssize_t read(int fd, void *buf, size_t n)
{
    return (ssize_t)svc_call(SYS_read,
                            (uint32_t)fd,
                            (uint32_t)buf,
                            (uint32_t)n);
}

/* ---- others ---- */
int getpid(void)   { return (int)svc_call(SYS_getpid, 0u, 0u, 0u); }
void yield(void)   { (void)svc_call(SYS_yield, 0u, 0u, 0u); }

void exitt(void)
{
    (void)svc_call(SYS__exit, 0u, 0u, 0u);
    for (;;) {}               /* never returns */
}

int reboot(void)   { return (int)svc_call(SYS_reboot, 0u, 0u, 0u); }
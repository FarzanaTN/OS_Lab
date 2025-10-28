/*
 * Copyright (c) 2022 
 * Computer Science and Engineering, University of Dhaka
 * Credit: CSE Batch 25 (starter) and Prof. Mosaddek Tushar
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE UNIVERSITY AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE UNIVERSITY OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
#include <kunistd.h>
#include <syscall_def.h>
#include <stdint.h>
/* Add your functions here */

void k_exit(void) {
    __asm volatile ("MOV R12, R11");
    __asm volatile ("SVC #3");
    k_yield();
}

uint16_t  k_getpid(void) {
    //kprintf("In k_getpid\n");
    uint16_t pid = 0;
    __asm volatile ("MOV R12, R11");
    __asm volatile ("MOV R0, %0" : : "r" (&pid));
    __asm volatile ("SVC #5");
    return pid;
}

// attempts to read up to len bytes, returns the number of bytes read
int  k_read(int fd, unsigned char *s, int len) {
    kprintf("In k_read\n");
    int ret;
    __asm volatile ("MOV R1, %0" : : "r" (fd));
    __asm volatile ("MOV R2, %0" : : "r" (s));
    __asm volatile ("MOV R3, %0" : : "r" (len));
    __asm volatile ("MOV R12, %0" : : "r" (&ret));
    __asm volatile("SVC #50");
    return ret;
}

int  k_write(uint32_t fd, unsigned char *s, size_t len) {
    int ret;
    __asm volatile ("MOV R0, %0" : : "r" (fd));
    __asm volatile ("MOV R1, %0" : : "r" (s));
    __asm volatile ("MOV R2, %0" : : "r" (len));
    __asm volatile ("MOV R12, %0" : : "r" (&ret));
    __asm volatile("SVC #55");
    return ret;
}

int  k_get_time(void) {
    kprintf("In k_get_time\n");
    unsigned int time = 0;
    __asm volatile ("MOV R1, %0" : : "r" (&time));//(pointer pass to kernel) places the address of time into R1. This is the pointer where the kernel will write the result.
    __asm volatile ("SVC #113");    //trigger system call
    kprintf("Time from k_get_time: %d ms\n", time);
    return (int)(time);
}




void  k_reboot(void)  {
    __asm volatile ("SVC #119");
}

void  k_yield(void) {
    __asm volatile ("SVC #120");
}


int fopen(unsigned char *s, uint32_t fd)
{
    int ret;
    __asm volatile("MOV R1, %0": :  "r"(s)); 
    __asm volatile("MOV R2, %0": :  "r"(fd));
    __asm volatile("MOV R12, %0": :  "r"(&ret));
    //__asm volatile("SVC #45");
   // __asm volatile("SVC %[value]" : : [value] "I"(45));
     __asm volatile("SVC %[value]" : : [value] "I"(SYS_open));
    return ret;
}

int fclose(uint32_t fd)
{
    int ret;
    __asm volatile("MOV R1, %0": :  "r"(fd));
    __asm volatile("MOV R12, %0": :  "r"(&ret));
    __asm volatile("SVC #49");
    return ret;
}


int fprintf(int fd, char *format, ...)
{
    char *tr;
    uint32_t i;
    uint8_t *str;
    va_list list;
    double dval;
    va_start(list, format);
    unsigned char result[512];
    int index = 0;

    //kprintf("Device #%d output:\n", fd);

    for (tr = format; *tr != '\0'; tr++)
    {
        while (*tr != '%' && *tr != '\0')
        {
            result[index++] = (uint8_t)*tr;
            tr++;
        }
        if (*tr == '\0')
            break;
        tr++;
        switch (*tr)
        {
        case 'c':
            i = va_arg(list, int);
            result[index++] = (uint8_t)i;
            break;
        case 'd':
            i = va_arg(list, int);
            if (i < 0)
            {
                result[index++] = (uint8_t)'-';
                i = -i;
            }
            uint8_t *s1 = (uint8_t *)convert(i, 10);
            while (*s1)
            {
                result[index++] = *s1;
                s1++;
            }

            break;
        case 'o':
            i = va_arg(list, int);
            if (i < 0)
            {
                result[index++] = '-';
                i = -i;
            }
            s1 = (uint8_t *)convert(i, 8);
            while (*s1)
            {
                result[index++] = *s1;
                s1++;
            }
            break;
        case 'x':
            i = va_arg(list, int);
            if (i < 0)
            {
                result[index++] = '-';
                i = -i;
            }
            s1 = (uint8_t *)convert(i, 16);
            while (*s1)
            {
                result[index++] = *s1;
                s1++;
            }
            break;
        case 's':
            str = va_arg(list, uint8_t *);
            s1 = (uint8_t *)&str;
            while (*s1)
            {
                result[index++] = *s1;
                s1++;
            }
            break;
        case 'f':
            dval = va_arg(list, double);
            s1 = (uint8_t *)float2str(dval);
            while (*s1)
            {
                result[index++] = *s1;
                s1++;
            }
            break;
        default:
            break;
        }
    }
    va_end(list);
    result[index] = '\0';
    return k_write(STDOUT_FILENO, result, index);
}



/*

During SVC:

SP (MSP or PSP)
+---------------------+
| xPSR                | <- SP+28
| PC                  | <- SP+24
| LR                  | <- SP+20
| R12                 | <- SP+16
| R3                  | <- SP+12
| R2                  | <- SP+8
| R1                  | <- SP+4  <-- contains &time
| R0                  | <- SP+0
+---------------------+
SP points here (bottom of stack frame)

SVCall_Handler:
R0 = SP  <-- points to this frame
R1 = R0  <-- copy for convenience
B SVC_Handler_C

SVC_Handler_C(svc_args):
svc_args = R0
svc_args[1] = &time
*/
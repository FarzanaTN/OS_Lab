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
#include <sys_init.h>
#include <cm4.h>
#include <kmain.h>
#include <sys_usart.h>
#include <kstdio.h>
#include <sys_rtc.h>
#include <kstring.h>
#ifndef DEBUG
#define DEBUG 1
#endif
void kmain(void)
{
    __sys_init();

    kprintf("System initialized. Timer + syscall test...\n");
    uint16_t pid = k_getpid();
    kprintf("Running with PID: %d\n", pid);

    // // int last_time = k_get_time();
    // int elapsed_seconds = 0;

    // unsigned char buf[10]; // buffer to store input
    // int n;

    // kprintf("Please type a character: ");

    // // Call k_read for 1 byte from UART (fd = 0 for UART2, for example)
    // n = k_read(STDIN_FILENO, buf, 1);

    // if (n == 1)
    // {
    //     kprintf("\nYou typed: %c\n", buf[0]);
    // }
    // else
    // {
    //     kprintf("\nk_read failed or no input received\n");
    // }

    // kprintf("while loop starting...\n");
    while (1)
    {

        // uint32_t seconds = __get__Second();
        // kprintf("Time passed total: %d seconds\n", seconds);
        // ms_delay(1000);

        // kprintf("Reading RTC time:\n");

        uint32_t seconds = k_get_time();
        kprintf("Time passed total: %d seconds\n", seconds);
        ms_delay(1000);
        
    }
}

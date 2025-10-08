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

#include <sys_init.h>
#include <cm4.h>
#include <kmain.h>
#include <stdint.h>
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
    //uint32_t last_ms = 0;
    // uint32_t timeee = __get__Second();
    // kprintf("Time passed total: %d seconds\n", timeee);
    while (1)
    {
        
    //    uint32_t now_ms = __getTime();   // returns current time in ms
    // if (now_ms - last_ms >= 10)      // 30 ms elapsed
    //{
        //last_ms = now_ms;

        uint32_t seconds = __get__Second();
        kprintf("Time passed total: %d seconds\n", seconds);
        ms_delay(1000);
        //kprintf("lastms variable %d\n", last_ms++);
    //}
    }
}
/*
void kmain(void)
{
    __sys_init();  // make sure this calls __SysTick_init(SystemCoreClock/1000);

    while (1)
    {
        uint32_t hours   = __get__Hour();
        uint32_t minutes = __get__Minute() % 60;
        uint32_t seconds = __get__Second() % 60;

        kprintf("Stopwatch: %02d:%02d:%02d\n", hours, minutes, seconds);

        ms_delay(1000); // update every second
    }
}
*/


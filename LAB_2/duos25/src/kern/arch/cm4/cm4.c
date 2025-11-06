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

#include <cm4.h>
#include <sys_clock.h>
#include <syscall.h>

/************************************************************************************
 * __SysTick_init(uint32_t reload)
 * Function initialize the SysTick clock. The function with a weak attribute enables
 * redefining the function to change its characteristics whenever necessary.
 **************************************************************************************/

/*
initializes the ARM Cortex-M processor's SysTick timer to a desired rate,
typically for generating periodic interrupts.
*/

static volatile uint32_t __mscount = 0;
volatile static uint32_t _systick_count = 0;
volatile static uint32_t _ms_tick = 0;

void __SysTick_init(uint32_t reload)
{
    /*
    1.
    Stop the Timer during initialization
    to prevent unwanted interrupts from occurring
    while configuring the other registers.
    */
    SYSTICK->CTRL &= ~(1 << 0);
    /*
    2.
    Set the Reload Value, period of the timer
    */
    SYSTICK->LOAD = (reload - 1) & 0x00FFFFFF;
    //SYSTICK->LOAD = ;

    /*
    3.
    Clear the Counter:
    Write to the SysTick CURRENT (or VAL) register
    to clear the counter and reset it to its initial state
    */
    SYSTICK->VAL = 0;

    /*
    4.
    Configure the Control Register:
    Set the Clock Source.
    Enable the Interrupt.
    Enable the Timer
    */
    SYSTICK->CTRL |= (1 << 2) | // CLKSOURCE: Use processor clock (AHB)
                     (1 << 1) | // TICKINT: Enable interrupt
                     (1 << 0);  // ENABLE: Enable the counter
}
void SysTickIntDisable(void) // interrupt disable
{
    // if(!(SYSTICK->CTRL & ~(1<<1))){
    SYSTICK->CTRL &= ~(1 << 1); // Clear the TICKINT bit
    //}
}

void SysTickIntEnable(void)
{
    // if((SYSTICK->CTRL & ~(1<<1))){
    SYSTICK->CTRL |= (1 << 1); // Clear the TICKINT bit
    //}
}
/************************************************************************************
 * __sysTick_enable(void)
 * The function enables the SysTick clock if already not enabled.
 * redefining the function to change its characteristics whenever necessary.
 **************************************************************************************/
void __SysTick_enable(void)
{
    // if((SYSTICK->CTRL & ~(1<<0))){
    SYSTICK->CTRL |= (1 << 0); // Clear the TICKINT bit
    //}
}
void __sysTick_disable(void)
{
    // if(!(SYSTICK->CTRL & ~(1<<0))){
    SYSTICK->CTRL &= ~(1 << 0); // Clear the TICKINT bit
    //}
}
uint32_t __getSysTickCount(void)
{
    return _systick_count;
}
/************************************************************************************
 * __updateSysTick(uint32_t count)
 * Function reinitialize the SysTick clock. The function with a weak attribute enables
 * redefining the function to change its characteristics whenever necessary.
 **************************************************************************************/

void __updateSysTick(uint32_t count)
{
    __sysTick_disable();
    __SysTick_init(count);
    __SysTick_enable();
}

/************************************************************************************
 * __getTime(void)
 * Function return the SysTick elapsed time from the begining or reinitialing. The function with a weak attribute enables
 * redefining the function to change its characteristics whenever necessary.
 **************************************************************************************/

uint32_t __getTime(void)
{
    //return (_systick_count+(SYSTICK->LOAD-SYSTICK->VAL)/(PLL_N*1000));

    return _systick_count;
    //return (uint32_t)(_systick_count * SYSTICK->LOAD) + (SYSTICK->LOAD - SYSTICK->VAL);
}

uint32_t __get__Second(void)
{
    return _systick_count / 1000;
}
uint32_t __get__Minute(void)
{
    return __get__Second() / 60;
}
uint32_t __get__Hour(void)
{
    return __get__Minute() / 60;
}
void SysTick_Handler(void)
{
    _systick_count++;
   //_systick_count+=(SYSTICK->LOAD)/(PLL_N*1000);
}

void __enable_fpu()
{
    SCB->CPACR |= ((0xFUL << 20));
}

uint8_t ms_delay(uint32_t delay)
{
    uint32_t start_tick = _systick_count;
    while ((_systick_count - start_tick) < delay){
        __WFI();
    }
        
    return 0; // The return type suggests a status, 0 for success.
}

uint32_t getmsTick(void)
{
    return _systick_count;
}

uint32_t wait_until(uint32_t delay)
{
    // Returns the current tick count after a delay.
    // This is useful for scheduling events.
    return _systick_count + delay;
    /*cg
     uint32_t target = __mscount + delay;
    while (__mscount < target)
        ;
    return 1;
    */
}

void SYS_SLEEP_WFI(void)
{
    __WFI();
}

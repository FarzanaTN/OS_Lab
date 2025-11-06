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

#include <syscall.h>
#include <syscall_def.h>
#include <errno.h>
#include <errmsg.h>

#include <kstdio.h>
#include <sys_usart.h>
#include <cm4.h>
#include <types.h>
#include <kmain.h>
#include <system_config.h>
#include<kstring.h>
 #include <stdarg.h>
#include <kstdio.h>
#include <sys_usart.h>
#include <UsartRingBuffer.h>
#include <kstring.h>
#include <float.h>
void __sys_getpid(void)
{
    unsigned int *svc_args;
    // The pointer to the stacked register values (created when SVC occurred)
    // is passed in register R1 by the exception entry code.
    // Move the value of R1 into our C variable 'svc_args'.
    __asm volatile("MOV %0, R1" : "=r"(svc_args)); //
    // svc_args now points to the stack frame created by the CPU on SVC entry:
    //   svc_args[0] -> R0 (1st argument)
    //   svc_args[1] -> R1 (2nd argument)
    //   svc_args[2] -> R2 (3rd argument)
    //   svc_args[3] -> R3 (4th argument)
    //   svc_args[4] -> R12 (internal register, here used as pointer to current TCB)

    // Retrieve the task_id of the current task from its TCB (Task Control Block)
    // and write it to the user-provided memory address stored in R0.
    //
    // Equivalent in plain C:
    //    unsigned int *result_ptr = (unsigned int *)svc_args[0];   // from R0
    //    TCB_TypeDef *current_tcb = (TCB_TypeDef *)svc_args[4];    // from R12
    //    *result_ptr = current_tcb->task_id;
    *((unsigned int *)svc_args[0]) = (*((TCB_TypeDef *)svc_args[4])).task_id;
    // Nothing to return — the result has been written to user memory.
    return;
}

void __sys_exit(void)
{
    unsigned int *svc_args;

    // Retrieve the address of SVC (Supervisor Call) arguments.
    // In the ARM SVC handler convention, R1 contains a pointer to these arguments.
    __asm volatile("MOV %0, R1" : "=r"(svc_args));

    // svc_args[4] contains a pointer to the TCB (Task Control Block) of the calling task.
    // Cast it to a TCB_TypeDef pointer, dereference it, and set its status to 4 (exited).
    (*((TCB_TypeDef *)svc_args[4])).status = 4; // r12 = 4

    // Return from the system call.
    return;
}

void __sys_read(void)
{
    unsigned int *svc_args;

    // Move contents of register R1 into svc_args
    // R1 holds the address of the SVC argument array
    __asm volatile("MOV %0, R1" : "=r"(svc_args) :);

    // svc_args[2] = buffer pointer (R2)
    // svc_args[3] = length to read (R3)
    // svc_args[4] = address to store number of bytes read (R12)
    unsigned char *buff = (unsigned char *)svc_args[2]; // Read buffer
    int len = (int)svc_args[3];                         // Bytes requested
    int *bytes_read = (int *)svc_args[4];               // Pointer to result

    // If reading only one byte from UART2
    if (len == 1)
    {
        buff[0] = UART_GetChar(USART2); // Read single char from UART2
        *bytes_read = 1;                // Indicate 1 byte read
    }
    else
    {
        // Read up to 50 bytes into buffer using USART2
        *bytes_read = _USART_READ_STR(USART2, buff, 50);
    }

    return; // Exit system call
}

void __sys_write(void)
{
    unsigned char *s;       // Pointer to the string buffer to write
    unsigned int *svc_args; // Pointer to array of system call arguments

    // Move the value of register R1 into svc_args
    // R1 contains pointer to the arguments passed by user code for this syscall
    __asm volatile("MOV %0, R1" : "=r"(svc_args) :);

    // svc_args[1] holds pointer to the buffer/string to write
    s = (unsigned char *)svc_args[1]; // Cast to unsigned char*

    // Call USART write function to send string through USART2
    // Returns number of bytes written
    int len = _USART_WRITE(USART2, s);

    // Store return value (number of bytes written) at svc_args[4]
    // This is the standard location for the syscall return value
    *((int *)svc_args[4]) = len;

    return; // Exit syscall
}


// void __sys_write(void)
// {
//     unsigned int *svc_args;
//     unsigned char *s;
//     unsigned char buff[100];  // Temporary kernel buffer
//     int len = 0;

//     // R1 holds pointer to stacked syscall arguments
//     __asm volatile("MOV %0, R1" : "=r"(svc_args) :);

//     // svc_args[1] = user string pointer
//     s = (unsigned char *)svc_args[1];

//     // Copy from user string to local buffer (like your example)
//     len = __strlen(s);
//     for (int i = 0; i <= len; i++)
//         buff[i] = s[i];

//     // Print the copied buffer through USART2
//     int written = _USART_WRITE(USART2, buff);

//     // Return number of bytes written
//     *((int *)svc_args[4]) = written;

//     return;
// }


// void __sys_write(void)
// {
//     unsigned int *svc_args;
//     unsigned char *s;

//     // R1 holds pointer to stacked syscall arguments
//     __asm volatile("MOV %0, R1" : "=r"(svc_args) :);

//     // svc_args[1] = user string pointer
//     s = (unsigned char *)svc_args[1];

//     // Call the working UART function to send string
//     Uart_sendstring((char *)s, __CONSOLE);

//     // Return number of bytes written
//     // Assuming Uart_sendstring returns nothing, we just return string length
//     *((int *)svc_args[4]) = __strlen(s);

//     return;
// }




void __sys_close(void)
{
    unsigned int *svc_args;

    // Move the pointer to the stack frame of the SVC (supervisor call) arguments
    // into the variable svc_args. This allows us to access the arguments passed
    // to the syscall.
    __asm volatile("MOV %0, R1" : "=r"(svc_args) :);

    // File descriptor is passed as the first argument in the syscall
    // (usually in R0/R1 depending on your ABI). Here, svc_args[1] points to it.
    int fd = (int)svc_args[1]; // R1

    // Call the kernel function to actually close the file descriptor
    fclose(fd);

    return;
}

void __sys_gettime(void)
{
    unsigned int *svc_args;

    // Move the pointer to the SVC arguments from register R1 into svc_args
    // This is typical in ARM assembly for handling system calls
    __asm volatile("MOV %0, R1" : "=r"(svc_args) :); //r1 points to stack frame

    // svc_args[1] contains the address where the caller wants the time to be stored
    // We call __getTime() (your internal time function) and store the result at that address
    *((unsigned int *)svc_args[1]) = __get__Second(); // svc_args[1] = R1(time pointer)
}



void __sys_reboot(void)
{
    // Print a message to indicate that a reboot is being initiated
    kprintf("rebooting...");

    // Write to the Application Interrupt and Reset Control Register (AIRCR) of the System Control Block (SCB)
    // This triggers a system reset on Cortex-M microcontrollers
    // 0x05FA is the VECTKEY, required to allow writes to AIRCR
    // (1 << 2) sets the SYSRESETREQ bit, requesting a system reset
    SCB->AIRCR = (0x05FA << 16) | (1 << 2);

    // Wait indefinitely until the reset occurs
    // The CPU should reset before it ever exits this loop
    while (1)
        ;
}

/*

Writing SYSRESETREQ in AIRCR triggers the NVIC to reset the processor.

All registers, stack pointers, and memory are reset to their startup default values.

The CPU begins execution at the reset vector in  startup code (Reset_Handler).
*/

// Voluntarily yield the CPU to allow the scheduler to run another task.
// This triggers a context switch by setting the PendSV interrupt pending.
void __sys_yield(void)
{
    // Set PendSV set-pending bit (bit 28) in ICSR register
    // PendSV is a low-priority interrupt used by the RTOS for context switching
    SCB->ICSR |= (1 << 28);
}

void syscall(uint16_t callno)

{
    /* The SVC_Handler calls this function to evaluate and execute the actual function */
    /* Take care of return value or code */

    switch (callno)
    {
    /* Write your code to call actual function (kunistd.h/c or times.h/c and handle the return value(s) */
    case SYS_read:
        __sys_read();
        break;
    case SYS_write:
        __sys_write();
        break;
    case SYS_reboot:
        __sys_reboot();
        break;
    case SYS__exit:
        __sys_exit();
        break;
    case SYS_getpid:
        __sys_getpid();
        break;
    case SYS___time:
        __sys_gettime();
        break;
    case SYS_yield:
        __sys_yield();
        break;
    /* return error code see error.h and errmsg.h ENOSYS sys_errlist[ENOSYS]*/
    default:
        break;
    }

    __asm volatile("POP {LR}");
    /* Handle SVC return here */
}

// void SVC_Handler_C(unsigned int * svc_args) {
//     uint8_t svc_number;
//     uint32_t stacked_r[5], lr, pc, xpsr;

//     // Stack frame contains:
//     // r0, r1, r2, r3, r12, r14, the return address and xPSR
//     // __asm volatile ("BKPT 5");
//     stacked_r[0] = svc_args[0];
//     stacked_r[1] = svc_args[1];
//     stacked_r[2] = svc_args[2];
//     stacked_r[3] = svc_args[3];
//     stacked_r[4] = svc_args[4];

//     lr = svc_args[5];
//     pc = svc_args[6];
//     xpsr = svc_args[7];

//     svc_number = ((char *) pc)[-2]; //Memory[(Stacked PC)-2]
//     __asm volatile("PUSH {LR}");
//     syscall(svc_number);

//     __asm volatile("POP {LR}");
// }


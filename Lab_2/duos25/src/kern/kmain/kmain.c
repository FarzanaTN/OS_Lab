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
#define MAX_READ_LEN 64
void kmain(void)
{
    __sys_init();


    for(volatile int i = 0; i < 5000000; i++);

    // kprintf("System initialized. Timer + syscall test...\n");
    // uint16_t pid = k_getpid();
    // kprintf("Running with PID: %d\n", pid);

    // 1. Define the message and its length
    const char *message = "Hello from kmain via k_write syscall!\n";
    size_t message_len = 39; // Length of the string

    // 2. Define the file descriptor for Standard Output (usually 1)
    uint32_t stdout_fd = 1;

    // 3. Call k_write
    int bytes_written = k_write(
        stdout_fd,
        (unsigned char *)message, // Cast to unsigned char* as required by k_write
        message_len
    );

    // Optional: Print a status message (if another basic print function exists)
    // or halt the system.
    // if (bytes_written > 0) {
    //     // Success
    //     kprintf("yeee write done\n");
    // } else {
    //     // Handle error (e.g., if k_write returns a negative value)
    //     kprintf("sad\n");
    // }

// Test direct k_write call
    // unsigned char msg[] = "Hello";
    // int bytes_written = k_write(STDOUT_FILENO, msg, __strlen(msg));
    // kprintf("k_write returned: %d bytes\r\n", bytes_written);

    
    // 1. Prepare the buffer and length
    unsigned char input_buffer[MAX_READ_LEN];
    // Ensure the buffer is terminated if needed, or clear it.
    input_buffer[0] = '\0'; 

    // 2. Define the file descriptor for Standard Input (usually 0)
    int stdin_fd = 0;

    // 3. Call k_read
    int bytes_read = k_read(
        stdin_fd,
        input_buffer,
        MAX_READ_LEN - 1 // Read up to N-1 bytes to reserve space for the null terminator
    );

    // // 4. Process the result
    // if (bytes_read > 0)
    // {
    //     // Null-terminate the string for safe printing, in case the kernel didn't
    //     input_buffer[bytes_read] = '\0'; 
    //     kprintf("--- k_read Result ---");
    //     kprintf("\nReceived %d bytes:\n", bytes_read);
    //     kprintf("Data: %s\n", input_buffer);
    // }
    // else if (bytes_read == 0)
    // {
    //     kprintf("Read operation return 0\n");
    // }
    // else
    // {
    //     // Handle error (bytes_read < 0, typically)
    //     kprintf("Error during k_read: \n");
    // }

    
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

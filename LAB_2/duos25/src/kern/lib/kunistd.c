#include <kunistd.h>
#include <stddef.h>
#include <stdint.h>
#include <cm4.h>
#include <kstdio.h>
#include <errno.h>
#include <stdio.h>
#include <system_config.h>
#include <UsartRingBuffer.h>
#include<unistd.h>
#include <kstdio.h>
#include <thread.h>


// UART register definitions (same as your working code)
#define UART_BASE     0x40004400U   // USART2 base address
#define UART_SR       (*(volatile uint32_t*)(UART_BASE + 0x00))
#define UART_DR       (*(volatile uint32_t*)(UART_BASE + 0x04))
#define UART_TX_READY (1U << 7)
#define UART_RX_READY (1U << 5)

// External functions from cm4.c
extern uint32_t __getTime(void);
extern uint32_t __getSysTickCount(void);

// Use ring buffer USART helpers for I/O
// We avoid libc getchar/putchar due to -nostdlib build

/**
 * @brief Kernel implementation of getSysTickTime
 * Returns the current elapsed time of systick in milliseconds
 * @return Current time in milliseconds since system start
 */
uint32_t k_getSysTickTime(void)
{
    // Get the current systick count (incremented every millisecond)
    // This is safe to call from privileged mode (kernel)
    uint32_t time_ms = __getTime();
    
    return time_ms;
}

/**
 * @brief Kernel implementation of write
 * @param fd: File descriptor
 * @param buf: Buffer to write from
 * @param n: Number of bytes to write
 * @return Number of bytes written, or negative error code
 */
ssize_t k_write(int fd, const void *buf, size_t n)
{
    // Check for valid buffer pointer
    if (buf == NULL) {
        return -EINVAL;
    }
    
    // For STDOUT (fd=1) and STDERR (fd=2), write to debug console
    if (fd == 1 || fd == 2) {
        const uint8_t *str = (const uint8_t *)buf;
        putstr(str, n);
        return (ssize_t)n;
    }
    
    // For other file descriptors, not implemented yet
    return -ENOSYS;
}



static int uart_recv_char(void) 
{
    // Wait until data received
    // This is a blocking wait - it will wait until a character arrives
    while (!(UART_SR & UART_RX_READY));
    
    // Read and return the received byte
    return (int)(UART_DR & 0xFF);
}

/**
 * @brief Kernel implementation of read
 * @param fd: File descriptor
 * @param buf: Buffer to read into
 * @param n: Number of bytes to read (max 256 bytes)
 * @return Number of bytes read, or negative error code
 */
ssize_t k_read(int fd, void *buf, size_t n)
{
    // Validate parameters
    if (buf == NULL || n == 0) {
        return -EINVAL;  // Invalid argument
    }
    
    // Only handle STDIN (fd=0)
    if (fd != STDIN_FILENO && fd != 0) {
        return -ENOSYS;  // Not implemented for other file descriptors
    }
    
    // Cast buffer to char pointer
    char *cbuf = (char *)buf;
    int bytes_read = 0;
    
    // Limit maximum read size to 256 bytes
    int max_size = (n > 256) ? 256 : (int)n;
    
    // Read characters one by one
    for (int i = 0; i < max_size; i++) {
        // Receive one character from UART
        int c = uart_recv_char();
        
        // Check for error (though uart_recv_char blocks, this is for safety)
        if (c < 0) {
            break;
        }
        
        // Store character in buffer
        cbuf[i] = (char)c;
        bytes_read++;
        
        // Echo the character back to terminal
        kputchar(c);
        
        // Check for termination characters
        if (c == '\n' || c == '\r' || c == '\0') {
            break;
        }
    }
    
    return (ssize_t)bytes_read;
}


/**
 * @brief Kernel implementation of getpid
 * Returns the task ID of the current running thread
 * @return Current process/task ID
 */
int k_getpid(void)
{
    // Get current thread from thread subsystem
    TCB_TypeDef *current = thread_get_current();
    
    if (current != NULL && current->magic_number == 0xFECABAA0) {
        return (int)current->task_id;
    }
    
    // No thread running or thread system not initialized
    // Return a default PID for kernel/main context
    return 1000;
}
/**
 * @brief Kernel implementation of yield
 * Yields the CPU to other processes
 */
void k_yield(void)
{
    // TODO: Implement actual process scheduler
    // For now, this is a no-op
    thread_yield();
}

/**
 * @brief Kernel implementation of exit
 * Terminates the current process/thread and triggers scheduler
 * This function does not return
 */
void k_exit(void)
{
    TCB_TypeDef *current = thread_get_current();
    
    if (current != NULL) {
        kprintf("[KERNEL] Process %d exiting...\n", current->task_id);
        
        // Mark thread as terminated
        current->status = THREAD_TERMINATED;
        
        // Trigger scheduler via yield to switch to next thread
        thread_yield();
        
        // Should never reach here after yield
        while (1) {
            __WFI();
        }
    } else {
        // No thread context - halt system
        kprintf("[KERNEL] Exit called without thread context - halting\n");
        while (1) {
            __WFI();
        }
    }
}

/**
 * @brief Kernel implementation of reboot
 * @return 0 on success, negative error code on failure
 */
int k_reboot(void)
{
    // Perform software reset using ARM Cortex-M AIRCR register
    kprintf("System rebooting...\n");
    ms_delay(100);  // Give time for message to transmit
    
    // Request system reset
    SCB->AIRCR = (0x5FA << 16) |      // VECTKEY
                 (SCB->AIRCR & 0x700) | // Keep priority group
                 (1 << 2);             // SYSRESETREQ
    
    // Should never reach here
    while(1);
    
    return 0;
}

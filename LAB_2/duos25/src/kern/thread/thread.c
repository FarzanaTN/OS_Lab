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

#include <types.h> 
#include <thread.h>
#include <stdint.h>
#include <stddef.h>
#include <kstdio.h>
#include <cm4.h>

// SCB registers for PendSV and control
#define SCB_ICSR    (*(volatile uint32_t*)0xE000ED04)
#define SCB_SHPR3   (*(volatile uint32_t*)0xE000ED20)
#define PENDSVSET   (1 << 28)

// Thread pool and stacks
static TCB_TypeDef thread_pool[MAX_THREADS];
static uint32_t thread_stacks[MAX_THREADS][THREAD_STACK_SIZE / 4] __attribute__((aligned(8)));

// Scheduler state
static volatile int current_thread_idx = -1;
static volatile int next_thread_idx = -1;
static uint16_t next_task_id = 1000;
static volatile uint8_t scheduler_started = 0;

// Forward declarations
static void init_thread_stack(TCB_TypeDef *tcb, void (*entry_point)(void *), void *arg);
static void thread_wrapper(void);

/**
 * @brief Initialize thread control block
 */
static void tcb_init(TCB_TypeDef *tcb, uint16_t task_id, uint32_t *stack_base)
{
    tcb->magic_number = 0xFECABAA0;
    tcb->task_id = task_id;
    tcb->psp = NULL;
    tcb->status = THREAD_READY;
    tcb->execution_time = 0;
    tcb->waiting_time = 0;
    tcb->digital_signature = 0x00000001;
    tcb->priority = THREAD_PRIORITY_NORMAL;
    tcb->stack_base = stack_base;
    tcb->stack_size = THREAD_STACK_SIZE;
}

/**
 * @brief Initialize the threading subsystem
 */
void thread_init(void)
{
    // Clear all thread slots
    for (int i = 0; i < MAX_THREADS; i++) {
        thread_pool[i].magic_number = 0;
        thread_pool[i].task_id = 0;
        thread_pool[i].status = 0;
        thread_pool[i].psp = NULL;
        thread_pool[i].stack_base = NULL;
    }
    
    current_thread_idx = -1;
    next_thread_idx = -1;
    next_task_id = 1000;
    scheduler_started = 0;
    
    // Set PendSV to lowest priority (0xFF)
    SCB_SHPR3 |= (0xFF << 16);
    
    kprintf("[THREAD] Subsystem initialized, PendSV priority set\n");
}

/**
 * @brief Thread wrapper function
 * This is called when a thread first runs. It extracts the real
 * entry point and argument from the stack and calls it.
 */
static void thread_wrapper(void)
{
    TCB_TypeDef *current = thread_get_current();
    
    if (current != NULL && current->magic_number == 0xFECABAA0) {
        // The entry_point and arg were placed in R0 and R1 by init_thread_stack
        // They will be in registers when we enter here
        
        // Extract from the initial hardware stack frame
        uint32_t *hw_frame = (uint32_t *)current->psp;
        
        // In the hardware exception frame:
        // hw_frame[0-7] are R4-R11 (software saved)
        // hw_frame[8] = R0 (entry_point)
        // hw_frame[9] = R1 (arg)
        void (*entry_func)(void *) = (void (*)(void *))hw_frame[8];
        void *arg = (void *)hw_frame[9];
        
        // Call the actual thread function
        if (entry_func != NULL) {
            entry_func(arg);
        }
    }
    
    // If thread returns, exit it
    thread_exit();
    
    // Should never reach here
    while (1) {
        __WFI();
    }
}

/**
 * @brief Initialize thread stack frame for first run
 */
static void init_thread_stack(TCB_TypeDef *tcb, void (*entry_point)(void *), void *arg)
{
    // Stack pointer starts at top and grows down
    uint32_t *sp = tcb->stack_base + (THREAD_STACK_SIZE / 4);
    
    // Save initial SP for debugging
    uint32_t *sp_initial = sp;
    
    // Create exception stack frame (as if exception just occurred)
    // This is what hardware automatically saves/restores
    
    // xPSR - Thumb bit must be set (bit 24)
    *(--sp) = 0x01000000;
    
    // PC - where to return to (thread_wrapper)
    *(--sp) = (uint32_t)thread_wrapper;
    
    // LR - link register (return address)
    *(--sp) = 0xFFFFFFFD;  // EXC_RETURN: return to Thread mode, use PSP
    
    // R12
    *(--sp) = 0x12121212;
    
    // R3
    *(--sp) = 0x03030303;
    
    // R2
    *(--sp) = 0x02020202;
    
    // R1 - argument to thread function
    *(--sp) = (uint32_t)arg;
    
    // R0 - entry point function pointer
    *(--sp) = (uint32_t)entry_point;
    
    // Now the software-saved registers (saved by PendSV handler)
    // R11
    *(--sp) = 0x11111111;
    
    // R10
    *(--sp) = 0x10101010;
    
    // R9
    *(--sp) = 0x09090909;
    
    // R8
    *(--sp) = 0x08080808;
    
    // R7
    *(--sp) = 0x07070707;
    
    // R6
    *(--sp) = 0x06060606;
    
    // R5
    *(--sp) = 0x05050505;
    
    // R4
    *(--sp) = 0x04040404;
    
    // Save final stack pointer to TCB
    tcb->psp = (void *)sp;
    
    // Debug: verify stack setup
    uint32_t stack_used = (uint32_t)sp_initial - (uint32_t)sp;
    kprintf("[THREAD] Stack init: base=0x%08X, psp=0x%08X, used=%u bytes\n",
            (uint32_t)tcb->stack_base, (uint32_t)sp, stack_used);
}

/**
 * @brief Create a new thread
 */
int thread_create(void (*entry_point)(void *), void *arg, uint8_t priority)
{
    if (entry_point == NULL) {
        kprintf("[THREAD] Error: NULL entry point\n");
        return -1;
    }
    
    // Find free thread slot
    int free_idx = -1;
    for (int i = 0; i < MAX_THREADS; i++) {
        if (thread_pool[i].magic_number != 0xFECABAA0 || 
            thread_pool[i].status == THREAD_TERMINATED ||
            thread_pool[i].status == THREAD_KILLED) {
            free_idx = i;
            break;
        }
    }
    
    if (free_idx < 0) {
        kprintf("[THREAD] Error: No free slots (max=%d)\n", MAX_THREADS);
        return -1;
    }
    
    // Initialize TCB
    TCB_TypeDef *tcb = &thread_pool[free_idx];
    uint16_t tid = next_task_id++;
    
    tcb_init(tcb, tid, thread_stacks[free_idx]);
    tcb->priority = priority;
    tcb->status = THREAD_READY;
    
    // Setup initial stack frame
    init_thread_stack(tcb, entry_point, arg);
    
    kprintf("[THREAD] Created TID=%d (slot %d, psp=0x%08X)\n", 
            tid, free_idx, (uint32_t)tcb->psp);
    
    return (int)tid;
}

/**
 * @brief Get current running thread's TCB
 */
TCB_TypeDef* thread_get_current(void)
{
    if (current_thread_idx >= 0 && current_thread_idx < MAX_THREADS) {
        TCB_TypeDef *tcb = &thread_pool[current_thread_idx];
        if (tcb->magic_number == 0xFECABAA0) {
            return tcb;
        }
    }
    return NULL;
}

/**
 * @brief Get current thread's task ID
 */
uint16_t thread_get_current_id(void)
{
    TCB_TypeDef *tcb = thread_get_current();
    if (tcb != NULL) {
        return tcb->task_id;
    }
    return 0;  // No thread or invalid
}

/**
 * @brief Get thread by task ID
 */
TCB_TypeDef* thread_get_by_id(uint16_t task_id)
{
    for (int i = 0; i < MAX_THREADS; i++) {
        if (thread_pool[i].magic_number == 0xFECABAA0 &&
            thread_pool[i].task_id == task_id) {
            return &thread_pool[i];
        }
    }
    return NULL;
}

/**
 * @brief Terminate current thread
 */
void thread_exit(void)
{
    TCB_TypeDef *current = thread_get_current();
    
    if (current != NULL) {
        kprintf("[THREAD] TID=%d exiting\n", current->task_id);
        current->status = THREAD_TERMINATED;
        
        // Force context switch to next thread
        thread_yield();
    } else {
        kprintf("[THREAD] Exit called with no current thread\n");
    }
    
    // Should never return here after yield
    while (1) {
        __WFI();
    }
}

/**
 * @brief Yield CPU to next thread
 */
void thread_yield(void)
{
    if (!scheduler_started) {
        return;  // Scheduler not running yet
    }
    
    // Trigger PendSV exception
    SCB_ICSR |= PENDSVSET;
    
    // Memory barriers to ensure write completes
    __asm volatile("dsb");
    __asm volatile("isb");
}

/**
 * @brief Scheduler - select next thread to run
 * This is called from PendSV handler
 */
void scheduler(void)
{
    int start_idx = (current_thread_idx + 1) % MAX_THREADS;
    int search_idx = start_idx;
    
    // Search for next READY thread (round-robin)
    do {
        TCB_TypeDef *tcb = &thread_pool[search_idx];
        
        if (tcb->magic_number == 0xFECABAA0 && tcb->status == THREAD_READY) {
            next_thread_idx = search_idx;
            return;
        }
        
        search_idx = (search_idx + 1) % MAX_THREADS;
    } while (search_idx != start_idx);
    
    // No READY thread found - check if current is still valid
    if (current_thread_idx >= 0 && current_thread_idx < MAX_THREADS) {
        TCB_TypeDef *current = &thread_pool[current_thread_idx];
        if (current->magic_number == 0xFECABAA0 && 
            current->status == THREAD_RUNNING) {
            next_thread_idx = current_thread_idx;  // Keep running current
            return;
        }
    }
    
    // No runnable threads - idle
    next_thread_idx = -1;
}

/**
 * @brief Start the scheduler and jump to first thread
 */
void scheduler_start(void)
{
    kprintf("[THREAD] Starting scheduler...\n");
    
    // Find first READY thread
    int first_idx = -1;
    for (int i = 0; i < MAX_THREADS; i++) {
        if (thread_pool[i].magic_number == 0xFECABAA0 &&
            thread_pool[i].status == THREAD_READY) {
            first_idx = i;
            break;
        }
    }
    
    if (first_idx < 0) {
        kprintf("[THREAD] ERROR: No threads to run!\n");
        while (1) {
            __WFI();
        }
        return;
    }
    
    current_thread_idx = first_idx;
    thread_pool[first_idx].status = THREAD_RUNNING;
    
    kprintf("[THREAD] Starting TID=%d (psp=0x%08X)\n", 
            thread_pool[first_idx].task_id,
            (uint32_t)thread_pool[first_idx].psp);
    
    scheduler_started = 1;
    
    // Get first thread's PSP - make sure it's stored in a register
    register uint32_t psp_val = (uint32_t)thread_pool[first_idx].psp;
    
    // Start first thread with inline assembly
    __asm volatile(
        "mov r1, %[psp]\n"          // Load PSP into r1
        "ldmia r1!, {r4-r11}\n"     // Restore R4-R11 from stack
        "msr psp, r1\n"             // Set PSP register
        "mov r0, #3\n"              // CONTROL = 0x03 (use PSP, unprivileged)
        "msr control, r0\n"         // Apply control register
        "isb\n"                     // Instruction sync barrier
        "movw lr, #0xFFFD\n"        // Load EXC_RETURN lower half
        "movt lr, #0xFFFF\n"        // Load EXC_RETURN upper half
        "bx lr\n"                   // Exception return - hardware pops rest
        : /* no outputs */
        : [psp] "r" (psp_val)
        : "r0", "r1", "memory"
    );
    
    // Should never reach here
    while (1);
}

/**
 * @brief PendSV Handler - performs context switching
 * This is called when PendSV exception is triggered
 */
void __attribute__((naked)) PendSV_Handler(void)
{
    // Save current thread context
    __asm volatile(
        // Disable interrupts during context switch
        "cpsid i\n"
        
        // Check if we have a current thread to save
        "ldr r0, =current_thread_idx\n"
        "ldr r1, [r0]\n"
        "cmp r1, #0\n"
        "blt skip_save\n"  // Skip save if no current thread
        
        // Get current PSP
        "mrs r0, psp\n"
        
        // Save R4-R11 on thread's stack
        "stmdb r0!, {r4-r11}\n"
        
        // Calculate TCB address: &thread_pool[current_thread_idx]
        "ldr r2, =thread_pool\n"
        "ldr r3, =current_thread_idx\n"
        "ldr r3, [r3]\n"
        
        // TCB size calculation (sizeof(TCB_TypeDef) = 32 bytes)
        "mov r4, #32\n"
        "mul r3, r3, r4\n"
        "add r2, r2, r3\n"
        
        // Store PSP in TCB (offset 8 bytes from TCB start)
        "str r0, [r2, #8]\n"
        
        // Mark old thread as READY (if it was RUNNING)
        "ldrh r4, [r2, #10]\n"  // Load status
        "cmp r4, #2\n"          // Check if RUNNING (0x0002)
        "bne skip_save\n"
        "mov r4, #1\n"          // READY (0x0001)
        "strh r4, [r2, #10]\n"
        
        "skip_save:\n"
        
        // Call scheduler to pick next thread
        "push {lr}\n"
        "bl scheduler\n"
        "pop {lr}\n"
        
        // Update current_thread_idx = next_thread_idx
        "ldr r0, =next_thread_idx\n"
        "ldr r1, [r0]\n"
        "ldr r0, =current_thread_idx\n"
        "str r1, [r0]\n"
        
        // Check if we have a valid next thread
        "cmp r1, #0\n"
        "blt no_thread\n"
        
        // Calculate next TCB address
        "ldr r2, =thread_pool\n"
        "mov r3, #32\n"
        "mul r1, r1, r3\n"
        "add r2, r2, r1\n"
        
        // Mark new thread as RUNNING
        "mov r3, #2\n"  // THREAD_RUNNING
        "strh r3, [r2, #10]\n"
        
        // Load PSP from next TCB
        "ldr r0, [r2, #8]\n"
        
        // Restore R4-R11 from new thread's stack
        "ldmia r0!, {r4-r11}\n"
        
        // Update PSP
        "msr psp, r0\n"
        
        // Enable interrupts and return
        "cpsie i\n"
        "bx lr\n"
        
        "no_thread:\n"
        // No thread to run - just return
        "cpsie i\n"
        "bx lr\n"
    );
}
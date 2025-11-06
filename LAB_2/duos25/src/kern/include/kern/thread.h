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

#ifndef __THREAD_H
#define __THREAD_H

#include <types.h>
#include <stdint.h>

// Thread states
#define THREAD_READY        0x0001
#define THREAD_RUNNING      0x0002
#define THREAD_WAITING      0x0004
#define THREAD_TERMINATED   0x0008
#define THREAD_KILLED       0x0010

// Maximum number of threads
#define MAX_THREADS         8

// Thread stack size (in bytes)
#define THREAD_STACK_SIZE   1024

// Thread priority levels
#define THREAD_PRIORITY_HIGH    0
#define THREAD_PRIORITY_NORMAL  1
#define THREAD_PRIORITY_LOW     2

/**
 * @brief Thread Control Block (TCB) structure
 */
typedef struct task_tcb {
    uint32_t magic_number;      // Magic number for validation (0xFECABAA0)
    uint16_t task_id;           // Task ID (starting from 1000)
    void *psp;                  // Process Stack Pointer
    uint16_t status;            // Thread status
    uint32_t execution_time;    // Total execution time (in ms)
    uint32_t waiting_time;      // Total waiting time (in ms)
    uint32_t digital_signature; // Digital signature (0x00000001)
    uint8_t priority;           // Thread priority
    uint32_t *stack_base;       // Base address of thread stack
    uint32_t stack_size;        // Stack size in bytes
} TCB_TypeDef;

/**
 * @brief Initialize the threading subsystem
 */
void thread_init(void);

/**
 * @brief Create a new thread
 * @param entry_point: Function pointer to thread entry point
 * @param arg: Argument to pass to thread function
 * @param priority: Thread priority
 * @return Task ID on success, -1 on failure
 */
int thread_create(void (*entry_point)(void *), void *arg, uint8_t priority);

/**
 * @brief Get the current running thread's TCB
 * @return Pointer to current TCB, or NULL if none
 */
TCB_TypeDef* thread_get_current(void);

/**
 * @brief Get the task ID of current thread
 * @return Task ID
 */
uint16_t thread_get_current_id(void);

/**
 * @brief Terminate the current thread
 */
void thread_exit(void);

/**
 * @brief Yield CPU to another thread
 */
void thread_yield(void);

/**
 * @brief Get thread by ID
 * @param task_id: Task ID to search for
 * @return Pointer to TCB or NULL if not found
 */
TCB_TypeDef* thread_get_by_id(uint16_t task_id);

/**
 * @brief Scheduler - select next thread to run
 * Called by PendSV handler
 */
void scheduler(void);

/**
 * @brief Start the scheduler (called once from main)
 */
void scheduler_start(void);

/**
 * @brief PendSV Handler (context switch)
 */
void PendSV_Handler(void);

#endif /* __THREAD_H */
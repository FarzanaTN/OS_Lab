#include <unistd.h>
#include <kstdio.h>
#include <sys_init.h>
#include <cm4.h>
#include <kmain.h>
#include <thread.h>

extern void kmain(void);

void thread1_function(void *arg)
{
    kprintf("\n>>> Thread 1 STARTED! <<<\n");
    
    // Test getpid
    int pid = getpid();
    kprintf("[Thread 1] My PID is: %d\n", pid);
    
    // Do some work
    for (int i = 0; i < 3; i++) {
        kprintf("[Thread 1] Working... iteration %d\n", i);
        ms_delay(500);
        yield();  // Give other threads a chance
    }
    
    kprintf("[Thread 1] Done! Calling exit()...\n");
    exitt();  // This should terminate the thread
    
    // Should NEVER reach here
    kprintf("[Thread 1] ERROR: Still alive after exit()!\n");
    while (1);
}

/**
 * @brief Simple test thread 2
 */
void thread2_function(void *arg)
{
    kprintf("\n>>> Thread 2 STARTED! <<<\n");
    
    // Test getpid
    int pid = getpid();
    kprintf("[Thread 2] My PID is: %d\n", pid);
    
    // Do some work
    for (int i = 0; i < 5; i++) {
        kprintf("[Thread 2] Working... iteration %d\n", i);
        ms_delay(700);
        yield();
    }
    
    kprintf("[Thread 2] Done! Calling exit()...\n");
    exitt();
    
    kprintf("[Thread 2] ERROR: Still alive after exit()!\n");
    while (1);
}

/**
 * @brief Idle thread - never exits
 */
void idle_thread_function(void *arg)
{
    kprintf("\n>>> Idle Thread STARTED! <<<\n");
    
    int pid = getpid();
    kprintf("[Idle] My PID is: %d\n", pid);
    
    int count = 0;
    while (1) {
        kprintf("[Idle] Still alive... count=%d\n", count++);
        ms_delay(1000);
        yield();
        
        // Exit after other threads should be done
        if (count >= 10) {
            kprintf("[Idle] Exiting after %d iterations\n", count);
            exitt();
        }
    }
}

int main(void)

{
    /*
    kprintf("\n=== Simple read() Test ===\n\n");
    ms_delay(2000);

    // First verify other syscalls work
    kprintf("Verifying other syscalls...\n");

    ms_delay(2000);

    uint32_t t = getSysTickTime();

    kprintf("Time: %u ms ✓\n", t);
    ms_delay(2000);

    int pid = getpid();
    kprintf("PID: %d ✓\n\n", pid);
    ms_delay(2000);

    // Now test read
    kprintf("Now testing read()...\n");
    ms_delay(2000);

    kprintf("Type something and press Enter: ");
    ms_delay(2000);

    char buffer[32];
    ssize_t n = read(0, buffer, 31);

    kprintf("\n\nResult:\n");
    ms_delay(2000);

    kprintf("  Bytes read: %d\n", n);
    ms_delay(2000);

    if (n > 0)
    {
        buffer[n] = '\0';
        kprintf("  Data: '%s'\n", buffer);
        kprintf("  ✓ SUCCESS!\n");
    }
    else if (n == 0)
    {
        kprintf("  No data read\n");
    }
    else
    {
        kprintf("  Error code: %d\n", n);
    }

    write(1, "mainf", 5);

    kprintf("\n=== Test complete ===\n");
    */
     kprintf("\n");
    kprintf("========================================\n");
    kprintf("   Simple Thread Test: getpid & exit   \n");
    kprintf("========================================\n\n");
    
    ms_delay(500);
    
    // Initialize threading
    kprintf("[MAIN] Initializing thread subsystem...\n");
    thread_init();
    ms_delay(300);
    
    kprintf("[MAIN] Creating test threads...\n\n");
    
    // Create thread 1
    int tid1 = thread_create(thread1_function, NULL, THREAD_PRIORITY_NORMAL);
    if (tid1 < 0) {
        kprintf("[MAIN] ERROR: Failed to create thread 1\n");
        while(1);
    }
    kprintf("[MAIN] Thread 1 created: TID=%d\n", tid1);
    ms_delay(200);
    
    // Create thread 2
    int tid2 = thread_create(thread2_function, NULL, THREAD_PRIORITY_NORMAL);
    if (tid2 < 0) {
        kprintf("[MAIN] ERROR: Failed to create thread 2\n");
        while(1);
    }
    kprintf("[MAIN] Thread 2 created: TID=%d\n", tid2);
    ms_delay(200);
    
    // Create idle thread
    int tid3 = thread_create(idle_thread_function, NULL, THREAD_PRIORITY_LOW);
    if (tid3 < 0) {
        kprintf("[MAIN] ERROR: Failed to create idle thread\n");
        while(1);
    }
    kprintf("[MAIN] Idle thread created: TID=%d\n\n", tid3);
    ms_delay(500);
    
    kprintf("========================================\n");
    kprintf("[MAIN] All threads created successfully!\n");
    kprintf("[MAIN] Starting scheduler...\n");
    kprintf("========================================\n\n");
    ms_delay(500);
    
    // Start scheduler - this should not return
    scheduler_start();
    
    // Should NEVER reach here
    kprintf("\n[MAIN] ERROR: Returned from scheduler_start()!\n");
    
    while (1) {
        kprintf("[MAIN] Stuck in error loop\n");
        ms_delay(1000);
    }
    return 0;
}
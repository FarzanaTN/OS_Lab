#include <stdint.h>
#include <syscall_def.h>
#include "kunistd.h"
#include <sys_init.h>
#include <sys/types.h>
#include <cm4.h>
#include <kmain.h>
#include <stdint.h>
#include <sys_usart.h>
#include <kstdio.h>
#include <sys_rtc.h>
#include <kstring.h>

extern void main(void);

void kmain(void)
{
    __sys_init();
    

    main();
    
    
}
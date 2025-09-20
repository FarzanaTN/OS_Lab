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
 
#ifndef __CM4_H
#define __CM4_H
#include <stdint.h>
/*
* This file defines Cortex-M4 processor internal peripherals
* NVIC, SCB, FPU and so on
*/
#define SYSTICK ((SYSTICK_TypeDef*)0xE000E010)
#define NVIC ((NVIC_TypeDef*)0xE000E100)
#define SCB ((SCB_TypeDef*) 0xE000ED00)
/*
* Data structure for SCB
*/
typedef struct
{
volatile uint32_t CPUID; 	// CPUID Base Register 0x0
volatile uint32_t ICSR;		// Interrupt Control and State Register 0x4
volatile uint32_t VTOR;		// Vector Table Offset Register 0x8
volatile uint32_t AIRCR;	// Application Interrupt and Reset Control Register 0xC
volatile uint32_t SCR;		// System Control Register 0x10
volatile uint32_t CCR;		// Configuration and Control Register 0x14
volatile uint8_t SHPR[12]; 	// Exception priority setting for system exceptions
volatile uint32_t SHCSR;	// System Handler Control and State Register 0x24
volatile uint32_t CFSR;		// Configurable Fault Status Register combined of MemManage, Fault Status Register, BusFault Status Register, UsageFault Status Register 0x28
volatile uint32_t HFSR;		// HardFault Status Register 0x2C
volatile uint32_t DFSR;		// Hint information for causes of debug events
volatile uint32_t MMFAR;	// MemManage Fault Address Register 0x34
volatile uint32_t BFAR;		// BusFault Address Register 0x38
volatile uint32_t AFSR;		// Auxiliary Fault Status Register 0x3C: Information for device-specific fault status
volatile uint32_t PFR[2]; 	// Read only information on available processor features
volatile uint32_t DFR;		// Debug Feature: Read only information on available debug features
volatile uint32_t AFR;		// Auxiliary Feature: Read only information on available auxiliary features
volatile uint32_t MMFR[4]; 	// Memory Model	Feature Registers: Read only information
volatile uint32_t ISAR[5]; 	// Instruction Set Attributes Register: Register Read only information
uint32_t RESERVED1[5];		// Now it is reserved: Unknown 	
volatile uint32_t CPACR; 	// Coprocessor access control register 0x88
} SCB_TypeDef;
/*
* SysTick Data Structure
*/
typedef struct 
{
    //define systick register compenenets -- use volatile data type
    volatile uint32_t CTRL; //systick controller register
    volatile uint32_t LOAD; //systick reload value
    volatile uint32_t VAL; // systick down counter
    volatile uint32_t CALIB; //systick calibration register
}SYSTICK_TypeDef;

enum IRQn_TypeDef {
  NonMaskableInt_IRQn = -14,
  HardFault_IRQn = -13,
  MemoryManagement_IRQn = -12,
  BusFault_IRQn = -11,
  UsageFault_IRQn = -10,
  SecureFault_IRQn = -9,
  SVCall_IRQn = -5,
  DebugMonitor_IRQn = -4,
  PendSV_IRQn = -2,
  SysTick_IRQn = -1,
  WWDG_STM_IRQn = 0,
  PVD_STM_IRQn = 1
};

typedef struct
{
    //define NVIC register compenenets -- use volatile data type
  volatile uint32_t ISER[8];     /*!< Offset: 0x000  Interrupt Set Enable Register           */
  uint32_t RESERVED0[24];                               
  volatile uint32_t ICER[8];     /*!< Offset: 0x180  Interrupt Clear Enable Register         */
  uint32_t RSERVED1[24];                                    
  volatile uint32_t ISPR[8];     /*!< Offset: 0x200  Interrupt Set Pending Register          */
  uint32_t RESERVED2[24];                                   
  volatile uint32_t ICPR[8];     /*!< Offset: 0x280  Interrupt Clear Pending Register        */
  uint32_t RESERVED3[24];                                   
  volatile uint32_t IABR[8];     /*!< Offset: 0x300  Interrupt Active bit Register           */
  uint32_t RESERVED4[56];                                   
  volatile uint8_t  IP[240];     /*!< Offset: 0x400  Interrupt Priority Register (8Bit wide) */
  uint32_t RESERVED5[644];                                  
  volatile  uint32_t STIR;   	 /*!< Offset: 0xF00  Software Trigger Interrupt Register     */
}NVIC_TypeDef;

typedef struct
{
    //define FPU register compenenets -- use volatile data type
    

}FPU_TypeDef;
/**
* Function related to SysTick
*/
__attribute__((weak)) void __SysTick_init(uint32_t);
__attribute__((weak)) void __sysTick_enable(void);
__attribute__((weak)) void __sysTick_disable(void);
__attribute__((weak)) uint32_t __getSysTickCount(void);
__attribute__((weak)) void __updateSysTick(uint32_t) ;
__attribute__((weak)) uint32_t __getTime(void);
/**
* Functions on FPU
**/
void __enable_fpu(void);
#endif

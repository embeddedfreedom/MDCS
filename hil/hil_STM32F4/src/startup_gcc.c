/*
Copyright (c) 2026 Diptopal Basu (embeddedfreedom)
Licensed under the MIT License
*/

//*****************************************************************************
//
// startup_gcc.c - Startup code for use with GNU tools.
//
//
//
//*****************************************************************************
#include "memory_map_stm32f411xe.h"
#include "core_initialisation_STM32F411xe.h"

//*****************************************************************************
//
// Forward declaration of the default fault handlers.
//
//*****************************************************************************
void ResetISR();
static void NmiSR();
static void FaultISR();
static void IntDefaultHandler();
//*****************************************************************************
//
// Reserve space for the system stack.
//
//*****************************************************************************
static unsigned long pulStack[256];

//*****************************************************************************
//
// The vector table.  Note that the proper constructs must be placed on this to
// ensure that it ends up at physical address 0x0800.0000.
//
//*****************************************************************************
__attribute__ ((section(".isr_vector")))
void (* const g_pfnVectors[])() =
{
    (void (*)())((unsigned long)pulStack + sizeof(pulStack)),

	    ResetISR,                               // The reset handler
	    NmiSR,                                  // The NMI handler
	    FaultISR,                               // The hard fault handler
		IntDefaultHandler,
		IntDefaultHandler,
		IntDefaultHandler,
	    0,
	    0,
	    0,
	    0,
		IntDefaultHandler,
		IntDefaultHandler,
	    0,
		IntDefaultHandler,
		SysTick_Handler,                    /* SysTick Handler              */
		IntDefaultHandler,                  /* Window WatchDog              */
		IntDefaultHandler,                  /* PVD through EXTI Line detection */
		IntDefaultHandler,                  /* Tamper and TimeStamps through the EXTI line */
		IntDefaultHandler,                  /* RTC Wakeup through the EXTI line */
		IntDefaultHandler,                  /* FLASH                        */
		IntDefaultHandler,                  /* RCC                          */
		EXTI0_Handler,                      /* EXTI Line0                   */
		IntDefaultHandler,                  /* EXTI Line1                   */
		IntDefaultHandler,                  /* EXTI Line2                   */
		IntDefaultHandler,                  /* EXTI Line3                   */
		IntDefaultHandler,                  /* EXTI Line4                   */
		IntDefaultHandler,                  /* DMA1 Stream 0                */
		IntDefaultHandler,                  /* DMA1 Stream 1                */
		IntDefaultHandler,                  /* DMA1 Stream 2                */
		IntDefaultHandler,                  /* DMA1 Stream 3                */
		IntDefaultHandler,                  /* DMA1 Stream 4                */
		IntDefaultHandler,                  /* DMA1 Stream 5                */
		IntDefaultHandler,                  /* DMA1 Stream 6                */
		IntDefaultHandler,                  /* ADC1, ADC2 and ADC3s         */
	       0,               				    /* Reserved                      */
	       0,              					/* Reserved                     */
	       0,                                /* Reserved                     */
	       0,                                /* Reserved                     */
		IntDefaultHandler,                  /* External Line[9:5]s          */
		IntDefaultHandler,                  /* TIM1 Break and TIM9          */
		IntDefaultHandler,                  /* TIM1 Update and TIM10        */
		IntDefaultHandler,                  /* TIM1 Trigger and Commutation and TIM11 */
		IntDefaultHandler,                  /* TIM1 Capture Compare         */
		IntDefaultHandler,                  /* TIM2                         */
		IntDefaultHandler,                  /* TIM3                         */
		IntDefaultHandler,                  /* TIM4                         */
		IntDefaultHandler,                  /* I2C1 Event                   */
		IntDefaultHandler,                  /* I2C1 Error                   */
		IntDefaultHandler,                  /* I2C2 Event                   */
		IntDefaultHandler,                  /* I2C2 Error                   */
		IntDefaultHandler,                  /* SPI1                         */
		IntDefaultHandler,                  /* SPI2                         */
		IntDefaultHandler,                  /* USART1                       */
		IntDefaultHandler,                  /* USART2                       */
	       0,               				/* Reserved                       */
		IntDefaultHandler,                  /* External Line[15:10]s        */
		IntDefaultHandler,                  /* RTC Alarm (A and B) through EXTI Line */
		IntDefaultHandler,                  /* USB OTG FS Wakeup through EXTI line */
	       0,                               /* Reserved     				  */
	       0,                               /* Reserved       			  */
	       0,                               /* Reserved 					  */
	       0,                               /* Reserved                     */
	    IntDefaultHandler,                  /* DMA1 Stream7                 */
	       0,                               /* Reserved                     */
		IntDefaultHandler,                   /* SDIO                         */
		IntDefaultHandler,                   /* TIM5                         */
		IntDefaultHandler,                   /* SPI3                         */
	       0,                                 /* Reserved                     */
	       0,                                 /* Reserved                     */
	       0,                                 /* Reserved                     */
	       0,                                 /* Reserved                     */
		IntDefaultHandler,                   /* DMA2 Stream 0                */
		IntDefaultHandler,                   /* DMA2 Stream 1                */
		IntDefaultHandler,                   /* DMA2 Stream 2                */
		IntDefaultHandler,                   /* DMA2 Stream 3                */
		IntDefaultHandler,                   /* DMA2 Stream 4                */
	       0,                    			  /* Reserved                     */
	       0 ,             					  /* Reserved                     */
	       0,              					  /* Reserved                     */
	       0,             					  /* Reserved                     */
	       0,              					  /* Reserved                     */
	       0,              					  /* Reserved                     */
	    OTG_FS_IRQHandler,                    /* USB OTG FS                   */
		DMA2_Stream5_IRQHandler,              /* DMA2 Stream 5                */
		IntDefaultHandler,                    /* DMA2 Stream 6                */
		IntDefaultHandler,                    /* DMA2 Stream 7                */
		IntDefaultHandler,                    /* USART6                       */
		IntDefaultHandler,                    /* I2C3 event                   */
		IntDefaultHandler,                    /* I2C3 error                   */
	       0,                                 /* Reserved                     */
	       0,                                 /* Reserved                     */
	       0,                                 /* Reserved                     */
	       0,                                 /* Reserved                     */
	       0,                                 /* Reserved                     */
	       0,                                 /* Reserved                     */
	       0,                                 /* Reserved                     */
		IntDefaultHandler,                    /* FPU                          */
	       0,                                 /* Reserved                     */
	       0,                                 /* Reserved                     */
	    IntDefaultHandler,                    /* SPI4                         */
		IntDefaultHandler                     /* SPI5                         */
};

//*****************************************************************************
//
// The following are constructs created by the linker, indicating where the
// the "data" and "bss" segments reside in memory.  The initializers for the
// for the "data" segment resides immediately following the "text" segment.
//
//*****************************************************************************
extern unsigned char _etext;
extern unsigned char _data;
extern unsigned char _edata;
extern unsigned char _bss;
extern unsigned char _ebss;

// Need to put this in the data section so that the value
// persists on zeroing the .bss section
unsigned char *dst = &_bss;

//*****************************************************************************
//
// This is the code that gets called when the processor first starts execution
// following a reset event.  Only the absolutely necessary set is performed,
// after which the application supplied entry() routine is called.  Any fancy
// actions (such as making decisions based on the reset cause register, and
// resetting the bits in that register) are left solely in the hands of the
// application.
//
//*****************************************************************************
void ResetISR()
{
    unsigned char *pulSrc, *pulDest;
    //
	// Copy the data segment initializers from flash to SRAM.
	// Though the .bss zero fill for loop destroys the *pulSrc, *pulDest
    // variables as the stack is part of the .bss section, this
    // loop completes its job of moving the data out to the
    // SRAM by then, hence it's not an issue. These variables could
    //also be made global and initialised
	pulSrc = &_etext;
	for(pulDest = &_data; pulDest < &_edata; )
	{
		*pulDest++ = *pulSrc++;
	}

    //
    // Zero fill the bss segment.
    // The variable dst, if declared locally within ResetISR()
    // gets stomped when zeroing out the stack, because the for loop erases its
    // own data on the stack! Hence the dst variable is made global here, and forced into the
	// data section of RAM. Since dst is a globally initialised variable it exists outside
	// the .bss area and hence can zero the .bss section uninterrupted
    for (dst = &_bss; dst< &_ebss;)
    {
    		*dst++ = 0x0;
    	}

    //
    // Call the application's entry point.
    //
    main();
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives a NMI.  This
// simply enters an infinite loop, preserving the system state for examination
// by a debugger.
//
//*****************************************************************************
static void NmiSR()
{
    //
    // Enter an infinite loop.
    //
    while(1)
    {
    }
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives a fault
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger.
//
//*****************************************************************************
static void FaultISR()
{
    //
    // Enter an infinite loop.
    //
    while(1)
    {
    }
}

//*****************************************************************************
//
// This is the code that gets called when the processor receives an unexpected
// interrupt.  This simply enters an infinite loop, preserving the system state
// for examination by a debugger.
//
//*****************************************************************************
static void IntDefaultHandler()
{
    //
    // Go into an infinite loop.
    //
    while(1)
    {
    }
}

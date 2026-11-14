/*
Copyright (c) 2026 Diptopal Basu (embeddedfreedom)
Licensed under the MIT License
*/

#include "memory_map_stm32f411xe.h"
#include "core_initialisation_STM32F411xe.h"



void SysTick_Handler(){
	// 1. Check watchdog threshold
    if (hil_watchdog_counter >= 5) {
        /*
         * FAILSAGE TRIPPED: Host has been unresponsive for 5 ms!
         * Power down the compensator and force safe idle state.
         */
        compensator_enabled = 0;
        compensator_synced  = 0; // Require re-sync on resume to avoid velocity spike

        // Toggle an LED to indicate failsafe/timeout mode
        toggle_LED();
    } else {
        /* Link is alive - increment watchdog counter */
        hil_watchdog_counter++;
    }

    asm("isb"); // Pipeline flush
 }

 void EXTI0_Handler(){
	static uint32_t position = 0;
	position = position % 4;
	uint32_t LEDPINSCLR = ~(0xFF << 24); //These bits clear the mode of all 4 LEDs
	GPIOD->MODER &= LEDPINSCLR;           /* clear pin mode */
	uint32_t LEDPINSMODE = 0x1 << (24 + (position * 2)); //Set orange LED port as output port PD13
	                                        // So now the systick will glow the orange LED
	position++;
	GPIOD->MODER |= LEDPINSMODE;            /* set pins to output mode */
	uint32_t CLEARLINEPEND = 0x1 << 0;    //Clear pending on the external interrupt line
                                          // This is unique to the external interrupts that
	                                      // there is a pending bit even outside NVIC
	EXTI->PR |= CLEARLINEPEND;
 }




 /*
 * ===============================================================================
 * ISR: DMA2_Stream5_IRQHandler
 * PURPOSE: The 1 kHz Lockstep Heartbeat on USART1.
 *          Fires precisely when the 12th byte lands in rx_dma_buffer.
 * ===============================================================================
 */
void DMA2_Stream5_IRQHandler(void) {
    /* Bit 11 in DMA2->HISR indicates Stream 5 Transfer Complete */
    uint32_t TC_FLAG_STREAM5 = 0x1 << 11;

    if (DMA2->HISR & TC_FLAG_STREAM5) {
        /* MANDATORY: Clear the interrupt flag in DMA2->HIFCR! */
        DMA2->HIFCR |= TC_FLAG_STREAM5;

        /* Reset the 5 ms watchdog */
        extern volatile uint32_t inactivity_ms;
        

        float pos1 = 0.0f;
        float pos2 = 0.0f;
        uint8_t cmd = 0;


        /* Cast the buffer and take the 24 ns register snapshot */
        // Why do we even need the next commented line of code, passing the rx_dma_buffer reference directly
        // I do not see any side effects of this 
        //HostToMcuPacket *rx_pkt = (HostToMcuPacket *)&rx_dma_buffer[0];

        if(Extract_HostToMcuPacket(rx_dma_buffer, &pos1, &pos2, &cmd)) {
            hil_watchdog_counter = 0; // This has to go into the packet check as well.    
            Write_McuToHostPacket(tx_dma_buffer, pos1, pos2, 0x01);

            /* Step 3: Trigger one-shot DMA TX response */
            USART1_DMA_Send_Packet();
            /* 
             * RUN YOUR 1 KHZ COMPENSATOR CODE HERE!
             * (t1 and t2 are now safely isolated in hardware registers)
             * TBD the entire compensator logic should be yet another function preferably an inline function
             * As of now just checking the DMA functionality so this is blank
             */
        }
    }
}
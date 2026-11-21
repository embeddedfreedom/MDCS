/*
Copyright (c) 2026 Diptopal Basu (embeddedfreedom)
Licensed under the MIT License
*/

#ifndef __CORE_INITIALISATION_STM32F411XE_H__ //Include guard for this header
#define __CORE_INITIALISATION_STM32F411XE_H__


typedef unsigned int uint32_t;
typedef unsigned short uint16_t;
typedef unsigned char uint8_t;
typedef int int32_t;
typedef short int16_t;
typedef char int8_t;  
/* Custom boolean definitions */
typedef enum {
    false = 0,
    true  = 1
} bool;

/* Canonical 12-byte HIL frame (shared with the host compensators and the
   lockstep test). Self-contained: no libc includes. */
#include "hil_protocol.h"

/*
//*********************************************************************************
// These 3 macros will convert base and offset values to address.
// This is not done with structures as structures need to be accurately filled and padded
//*********************************************************************************
#define ADDRW(x) (*((volatile unsigned int *)(x)))
#define ADDRH(x) (*((volatile unsigned short *)(x)))
#define ADDRB(x) (*((volatile unsigned char *)(x)))
//**********************************************************************************
*/


/*Declations of Functions found in core_drivers.c*/
int main();
void PLL_Init_80MHZ();
void enable_systick();
static void enable_usb_interrupt();
void toggle_LED();
void SysTick_Handler();
void EXTI0_Handler();
void DMA2_Stream5_IRQHandler(void);
void initialise_FPU();
void initialise_portG2();
void initialise_usb0();
void delay(uint32_t);
void blink_LED();
int printm(const char *, ...);  
void USART1_init(void);
void USART2_init(void);
void double_word_padding_disable();
void PLL_Init_84MHZ();
void enable_LEDs();
void configure_user_button();
void configure_EXTI0_PortA0();
void TIM3_PWM_Init(uint32_t, uint32_t, uint32_t);
void TIM3_Set_DutyCycle(uint32_t);
void TIM3_Set_DutyCycle2(uint32_t);
void Write_McuToHostPacket(uint8_t *, float, float, uint8_t);
bool Extract_HostToMcuPacket(const uint8_t *, float *, float *, uint8_t *);
void USART1_DMA_TX_Init(void);
void USART1_DMA_RX_Init(void);
void USART1_DMA_Send_Packet(void);
void I2C1_init();
void OTG_FS_IRQHandler(void);
float run_compensator_cycle_HIL(float t1, float t2);
/* Optional dual-output interface, selected by -DHIL_TWO_OUTPUTS in the
   model's Makefile_HIL (e.g. furnace: two independent heater PWMs). */
void run_compensator_cycle_HIL_dual(float t1, float t2, float *out1, float *out2);


/* Buffers strictly aligned to 4-byte boundaries for the 32-bit DMA Bus Master */
extern uint8_t rx_dma_buffer[HIL_PACKET_SIZE] __attribute__((aligned(4)));
extern uint8_t tx_dma_buffer[HIL_PACKET_SIZE] __attribute__((aligned(4)));
#define OLED_I2C_ADDR_WRITE  0x78    /* 0x3C shifted left by 1 (Write address) */
#define OLED_TOTAL_DMA_SIZE  1025    /* 1 control byte (0x40) + 1024 pixel bytes */
/* 1025-byte framebuffer aligned to 4-byte boundary for 32-bit DMA bus master */
extern uint8_t oled_buffer[OLED_TOTAL_DMA_SIZE] __attribute__((aligned(4)));

/* Frame sync, command, status, and packet structs come from hil_protocol.h. */

/* Shared between SysTick and DMA ISR - must be volatile */
extern volatile int32_t  hil_watchdog_counter;
extern volatile uint8_t  compensator_enabled;  // 1 = Active, 0 = Powered Down
extern volatile uint8_t  compensator_synced;  // Prevents velocity spike on resume

#endif
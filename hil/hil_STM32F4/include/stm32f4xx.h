/*
Copyright (c) 2026 Diptopal Basu (embeddedfreedom)
Licensed under the MIT License
*/

#ifndef __STM32F4XX_H
#define __STM32F4XX_H

/*
 * This bridge satisfies TinyUSB's dwc2_stm32.h without pulling in ST HAL
 * or causing typedef collisions with memory_map_stm32f411xe.h.
 * uint32_t is already defined internally by TinyUSB prior to including this.
 */

/* 1. USB OTG FS Base Address (RM0383 Section 23 / memory_map_stm32f411xe.h) */
#define USB_OTG_FS_PERIPH_BASE        0x50000000UL
#define USB_OTG_FS_MAX_IN_ENDPOINTS   4U
#define OTG_FS_IRQn                   67

typedef int IRQn_Type;

/* 2. Bare-Metal Cortex-M4 NVIC Register Manipulation (SCS_BASE + 0x0100) */
#define NVIC_ISER_BASE                (0xE000E100UL)
#define NVIC_ICER_BASE                (0xE000E180UL)
#define NVIC_IP_BASE                  (0xE000E400UL)

static inline void NVIC_EnableIRQ(int irqn) {
    *((volatile uint32_t *)(NVIC_ISER_BASE + (((uint32_t)irqn >> 5) * 4))) = (1UL << ((uint32_t)irqn & 0x1F));
}

static inline void NVIC_DisableIRQ(int irqn) {
    *((volatile uint32_t *)(NVIC_ICER_BASE + (((uint32_t)irqn >> 5) * 4))) = (1UL << ((uint32_t)irqn & 0x1F));
}

static inline void NVIC_SetPriority(int irqn, uint32_t priority) {
    *((volatile uint8_t *)(NVIC_IP_BASE + (uint32_t)irqn)) = (uint8_t)((priority << 4) & 0xFF);
}

/* 3. System core clock variable for PHY turnaround calculation (TRDT) */
extern uint32_t SystemCoreClock;

#ifndef __NOP
#define __NOP() asm volatile("nop")
#endif

#endif /* __STM32F4XX_H */
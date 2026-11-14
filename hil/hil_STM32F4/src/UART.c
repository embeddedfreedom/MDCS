/*
Copyright (c) 2026 Diptopal Basu (embeddedfreedom)
Licensed under the MIT License
*/

#include "memory_map_stm32f411xe.h"
#include "core_initialisation_STM32F411xe.h"

/*
 * ===============================================================================
 * FUNCTION: USART1_init
 * PURPOSE : High-Speed HIL Port on APB2 (84 MHz) @ 921,600 Baud.
 *           Pins: PA9 (TX), PA10 (RX) in AF7.
 *           Configured with DMAR and DMAT enabled for DMA2 autonomous streaming.
 *           NO polling Tx/Rx functions needed here!
 * ===============================================================================
 */
void USART1_init(void) {
    // 1. Enable clocks for GPIOA (AHB1) and USART1 (APB2)
    uint32_t GPIOA_CLOCK_EN  = 0x1 << 0; // Bit 0 in AHB1ENR
    uint32_t USART1_CLOCK_EN = 0x1 << 4; // Bit 4 in APB2ENR
    RCC->AHB1ENR |= GPIOA_CLOCK_EN;
    RCC->APB2ENR |= USART1_CLOCK_EN;

    // 2. Configure PA9 (TX) and PA10 (RX) for Alternate Function (0b10)
    uint32_t PA9_PA10_MODER_CLR = ~((0x3 << (9 * 2)) | (0x3 << (10 * 2)));
    uint32_t PA9_PA10_AF_MODE   =  (0x2 << (9 * 2)) | (0x2 << (10 * 2));
    GPIOA->MODER &= PA9_PA10_MODER_CLR;
    GPIOA->MODER |= PA9_PA10_AF_MODE;

    // 3. High Output Speed on PA9 and PA10
    uint32_t SPEED_HIGH = (0x2 << (9 * 2)) | (0x2 << (10 * 2));
    GPIOA->OSPEEDR |= SPEED_HIGH;

    // 4. Set Alternate Function AF7 (0x7) in AFR[1] (PA9 = nibble 1, PA10 = nibble 2)
    uint32_t AFRH_CLR = ~((0xF << (1 * 4)) | (0xF << (2 * 4)));
    uint32_t AFRH_AF7 =  (0x7 << (1 * 4)) | (0x7 << (2 * 4)); // AF7 is USART1
    GPIOA->AFR[1] &= AFRH_CLR;
    GPIOA->AFR[1] |= AFRH_AF7;

    // 5. Baud Rate: 921,600 Baud @ 84 MHz APB2 (Mantissa = 5, Fraction = 11 -> 0x5B)
    uint32_t DIVMANTISSA = 5U  << 4; 
    uint32_t DIVFRACTION = 11U << 0;
    USART1->BRR = (DIVMANTISSA | DIVFRACTION);

    // 6. 1 Stop bit
    uint32_t STOP_1BIT_CLR = ~(0x3 << 12);
    USART1->CR2 &= STOP_1BIT_CLR;

    // 7. ENABLE DMA TRIGGERS FOR DMA2: DMAR (bit 6) and DMAT (bit 7)
    uint32_t DMAR_DMAT_ENABLE = (0x1 << 6) | (0x1 << 7);
    USART1->CR3 |= DMAR_DMAT_ENABLE;

    // 8. Enable Transmitter (TE), Receiver (RE), and USART1 (UE)
    uint32_t ENABLE_TX_RX_UE = (0x1 << 3) | (0x1 << 2) | (0x1 << 13);
    USART1->CR1 |= ENABLE_TX_RX_UE;
}

/*
 * ===============================================================================
 * FUNCTION: USART2_init
 * PURPOSE : Debug Console Port on APB1 (42 MHz) @ 115,200 Baud.
 *           Pins: PA2 (TX), PA3 (RX) in AF7.
 *           Dedicated strictly to printm() and human text.
 * ===============================================================================
 */
void USART2_init(void) {
    // 1. Enable clocks for GPIOA (AHB1) and USART2 (APB1)
    uint32_t GPIOA_CLOCK_EN  = 0x1 << 0;  // Bit 0 in AHB1ENR
    uint32_t USART2_CLOCK_EN = 0x1 << 17; // Bit 17 in APB1ENR
    RCC->AHB1ENR |= GPIOA_CLOCK_EN;
    RCC->APB1ENR |= USART2_CLOCK_EN;

    // 2. Configure PA2 (TX) and PA3 (RX) for Alternate Function (0b10)
    uint32_t PA2_PA3_MODER_CLR = ~((0x3 << (2 * 2)) | (0x3 << (3 * 2)));
    uint32_t PA2_PA3_AF_MODE   =  (0x2 << (2 * 2)) | (0x2 << (3 * 2));
    GPIOA->MODER &= PA2_PA3_MODER_CLR;
    GPIOA->MODER |= PA2_PA3_AF_MODE;

    // 3. Set Alternate Function AF7 in AFR[0] (PA2 = nibble 2, PA3 = nibble 3)
    uint32_t AFRL_CLR = ~((0xF << (2 * 4)) | (0xF << (3 * 4)));
    uint32_t AFRL_AF7 =  (0x7 << (2 * 4)) | (0x7 << (3 * 4)); // AF7 is USART2
    GPIOA->AFR[0] &= AFRL_CLR;
    GPIOA->AFR[0] |= AFRL_AF7;

    // 4. Baud Rate: 115,200 Baud @ 42 MHz APB1 (Mantissa = 22, Fraction = 13 -> 0x16D)
    uint32_t DIVMANTISSA = 22U << 4;
    uint32_t DIVFRACTION = 13U << 0;
    USART2->BRR = (DIVMANTISSA | DIVFRACTION);

    // 5. 1 Stop bit, No flow control
    USART2->CR2 &= ~(0x3 << 12);
    USART2->CR3 = 0x0000;

    // 6. Enable Transmitter, Receiver, and USART2
    uint32_t ENABLE_TX_RX_UE = (0x1 << 3) | (0x1 << 2) | (0x1 << 13);
    USART2->CR1 |= ENABLE_TX_RX_UE;
}

/*
 * ===============================================================================
 * ONLY USART2 HAS POLLING TX/RX FUNCTIONS (USED BY PRINTM)
 * ===============================================================================
 */
void UART_putchar(uint8_t data) {
    uint32_t TXE = 0x1 << 7;
    while (!(USART2->SR & TXE)) {} // Wait until USART2 Tx buffer empty
    USART2->DR = data;
}

uint8_t UART_getchar(void) {
    uint8_t data;
    uint32_t RXNE = 0x1 << 5;
    while (!(USART2->SR & RXNE)) {} // Wait until USART2 char arrives
    data = (uint8_t)USART2->DR;
    return data;
}
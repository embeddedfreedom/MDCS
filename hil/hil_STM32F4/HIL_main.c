/*
Copyright (c) 2026 Diptopal Basu (embeddedfreedom)
Licensed under the MIT License
*/

/*
	build_example_main.c - Demonstrate Interrupt handling
	Author - Diptopal
*/

#include "memory_map_stm32f411xe.h"
#include "core_initialisation_STM32F411xe.h"




// Control and watchdog flags (use volatile since they are modified in ISRs)
volatile int32_t hil_watchdog_counter = 0;
volatile uint8_t compensator_enabled  = 0;
volatile uint8_t compensator_synced   = 0;

// DMA Buffers (memory allocated here for the extern declarations in the header)
// With USB implementation these buffers are unused as the USB handles this logic
// in usb_cdc_adapter.c
uint8_t rx_dma_buffer[HIL_PACKET_SIZE] __attribute__((aligned(4)));
uint8_t tx_dma_buffer[HIL_PACKET_SIZE] __attribute__((aligned(4)));
uint8_t oled_buffer[OLED_TOTAL_DMA_SIZE] __attribute__((aligned(4)));

/* Bridge function declarations (no tusb.h needed) */
void usb_hardware_init(void);
void usb_cdc_init(void);
void usb_cdc_task(void);

/*Prescalar and Auto reload value in TIM3->ARR */
uint32_t prescalar = 83U;
uint32_t auto_reload_value = 999U;
uint32_t duty_cycle = 500U;

int main(){
	PLL_Init_84MHZ();
    initialise_FPU();
    USART2_init();              // 115200 Debug console
    TIM3_PWM_Init(prescalar, auto_reload_value , duty_cycle);
    //USART1_init();              // 921600 HIL Port
    //enable_LEDs();
    //USART1_DMA_RX_Init();       // Start listening on DMA2 Stream 5
    //USART1_DMA_TX_Init();       // Arm DMA2 Stream 7 for TX
    //enable_systick();
    //configure_user_button();
    //configure_EXTI0_PortA0();
    /* 1. Physical GPIO & RCC clock init */
    usb_hardware_init();

    /* 2. Boot TinyUSB CDC stack */
    usb_cdc_init();

    while (1) {
        /*
         * Core sleeps until an interrupt fires (OTG_FS_IRQHandler or SysTick).
         * Wakes up within ~140 ns upon packet arrival from Host.
         */
        asm volatile("wfi");

        /* Non-blocking event dispatch (< 2 microseconds overhead) */
        usb_cdc_task();
    }

    return 0;
}


/*
 * ===============================================================================
 * FUNCTION: TIM3_Set_DutyCycle
 * PURPOSE : Updates the capture compare register to adjust the PWM duty cycle.
 * Target  : STM32F411xE
 * The duty cycle in the current configuration varies between 0 and 999
 *  TIM3->ARR = 999U mentioned in src/core_initialisation_STM32F411xe.c
 * ===============================================================================
 */
void TIM3_Set_DutyCycle(uint32_t duty_cycle_value) {
    TIM3->CCR1 = duty_cycle_value;
}

void TIM3_Set_DutyCycle2(uint32_t duty_cycle_value) {
    TIM3->CCR2 = duty_cycle_value;
}

/*
 * ===============================================================================
 * FUNCTION: USART1_DMA_Send_Packet
 * PURPOSE : Triggers DMA2 Stream 7 Channel 4 to transmit the 12-byte packet
 *           in tx_dma_buffer to USART1.
 *           Called at the end of the compensator cycle.
 * ===============================================================================
 */
void USART1_DMA_Send_Packet(void) {
    uint32_t STREAM_EN = 0x1 << 0;
    /* ---------------------------------------------------------------------------
     * STEP 1: Ensure Stream 7 is disabled before reloading count
     * Silicon rule: NDTR cannot be updated while EN is 1.
     * --------------------------------------------------------------------------- */
    DMA2_Stream7->CR &= ~STREAM_EN;
    while (DMA2_Stream7->CR & STREAM_EN) {
        /* Wait until hardware confirms stream is stopped */
    }
    /* ---------------------------------------------------------------------------
     * STEP 2: Clear previous Transfer Complete flag for Stream 7 in HIFCR
     * Bit 27 in HIFCR clears TCIF7.
     * --------------------------------------------------------------------------- */
    uint32_t CLEAR_TCIF7 = 0x1 << 27;
    DMA2->HIFCR |= CLEAR_TCIF7;
    /* ---------------------------------------------------------------------------
     * STEP 3: Reload Number of Data Items (NDTR)
     * Exact packet size: 12 bytes.
     * --------------------------------------------------------------------------- */
    DMA2_Stream7->NDTR = HIL_PACKET_SIZE;
    /* ---------------------------------------------------------------------------
     * STEP 4: Enable Stream 7 to kick off autonomous transmission
     * --------------------------------------------------------------------------- */
    DMA2_Stream7->CR |= STREAM_EN;
}

/*
 * ===============================================================================
 * FUNCTION: I2C1_OLED_DMA_Flush
 * PURPOSE : Starts the background DMA streaming of the 1025-byte framebuffer.
 *           Called at human UI rates (20 to 30 Hz), or when the knob turns.
 * ===============================================================================
 */
void I2C1_OLED_DMA_Flush(void) {
    uint32_t STREAM_EN = 0x1 << 0;

    /* 1. If previous transfer is still in flight, do not interrupt */
    if (DMA1_Stream7->CR & STREAM_EN) {
        return;
    }

    /* 2. Clear previous Transfer Complete flag in HIFCR (Bit 27) */
    uint32_t CLEAR_TCIF7 = 0x1 << 27;
    DMA1->HIFCR |= CLEAR_TCIF7;

    /* 3. Reload byte count: 1025 bytes (1 control byte + 1024 pixel bytes) */
    DMA1_Stream7->NDTR = OLED_TOTAL_DMA_SIZE;

    /* 4. Enable DMA1 Stream 7 engine */
    DMA1_Stream7->CR |= STREAM_EN;

    /* ---------------------------------------------------------------------------
     * STEP 5: Generate I2C START Condition and Address Phase
     * I2C requires a START condition and the Slave Address before DMA takes over.
     * --------------------------------------------------------------------------- */
    // Generate START condition (Bit 8 in CR1)
    I2C1->CR1 |= (0x1 << 8);

    // Wait until START condition is generated (SB bit in SR1)
    while (!(I2C1->SR1 & (0x1 << 0))) {}

    // Send the 7-bit OLED Slave Address with Write bit (0x78)
    I2C1->DR = OLED_I2C_ADDR_WRITE;

    // Wait until Address is sent and ACK received (ADDR bit in SR1)
    while (!(I2C1->SR1 & (0x1 << 1))) {}

    // Clear ADDR flag by reading SR1 followed by reading SR2 (Silicon rule!)
    volatile uint32_t dummy_read;
    dummy_read = I2C1->SR1;
    dummy_read = I2C1->SR2;
    (void)dummy_read;

    /*
     * THE DMA IS NOW IN FULL CONTROL!
     * The DMA controller autonomously clocks out all 1025 bytes across the bus.
     * The CPU exits this function immediately and returns to the control loop!
     */
}


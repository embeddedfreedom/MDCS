/*
Copyright (c) 2026 Diptopal Basu (embeddedfreedom)
Licensed under the MIT License
*/

/*
  core_initialisation_STM32F411xe.c - Core initialisation for STM32F411xe
  Author - Diptopal
*/

#include "memory_map_stm32f411xe.h"
#include "core_initialisation_STM32F411xe.h"
      


/*
 * ===============================================================================
 * FUNCTION: PLL_Init_84MHZ_rectified
 * PURPOSE : Configures the STM32F411xC/E system clock to 84 MHz using an external 
 *           8 MHz crystal on HSE, with APB1 = 42 MHz and USB = 48 MHz.
 * Target  : STM32F411xE
 * Reference: RM0383 Section 6.3 (RCC) & Section 3.5 (Embedded Flash Memory)
 * ===============================================================================
 */
void PLL_Init_84MHZ(void) {
    /* ---------------------------------------------------------------------------
     * STEP 1: Configure Flash Latency and ART Accelerator
     * At 84 MHz and VDD = 3.0V - 3.6V, Flash requires 2 Wait States (WS).
     * We also enable Prefetch (PRFTEN), Instruction Cache (ICEN), and Data 
     * Cache (DCEN) in FLASH->ACR to achieve 0-wait-state equivalent performance.
     *   Bits [2:0] LATENCY = 0b010 (2 Wait States)
     *   Bit 8      PRFTEN  = 1 (Prefetch Enable)
     *   Bit 9      ICEN    = 1 (Instruction Cache Enable)
     *   Bit 10     DCEN    = 1 (Data Cache Enable)
     * --------------------------------------------------------------------------- */
    uint32_t FLASH_LATENCY_2WS = 0x2 << 0;
    uint32_t FLASH_PRFTEN      = 0x1 << 8;
    uint32_t FLASH_ICEN        = 0x1 << 9;
    uint32_t FLASH_DCEN        = 0x1 << 10;
    FLASH->ACR |= (FLASH_LATENCY_2WS | FLASH_PRFTEN | FLASH_ICEN | FLASH_DCEN);

    /* ---------------------------------------------------------------------------
     * STEP 2: Enable the High-Speed External (HSE) Oscillator
     * Discovery board provides an 8 MHz crystal oscillator on OSC_IN / OSC_OUT.
     * Bit 16 in RCC->CR is HSEON.
     * Bit 17 in RCC->CR is HSERDY (hardware status).
     * --------------------------------------------------------------------------- */
    uint32_t HSEON = 0x1 << 16;
    RCC->CR |= HSEON;

    uint32_t HSERDY = 0x1 << 17;
    while (!(RCC->CR & HSERDY)) {
        /* Wait until HSE hardware oscillator stabilizes */
    }

    /* ---------------------------------------------------------------------------
     * STEP 3: Configure PLL Multiplication & Division Parameters
     * Hardware Equations:
     *   f(VCO_in)  = f(PLL_in) / PLLM = 8 MHz / 4 = 2.0 MHz (ST spec: 1 to 2 MHz)
     *   f(VCO_out) = f(VCO_in) * PLLN = 2.0 MHz * 168 = 336.0 MHz (ST spec: 100 to 432 MHz)
     *   f(SYSCLK)  = f(VCO_out) / PLLP = 336 MHz / 4 = 84.0 MHz (Max 84 MHz)
     *   f(USB/SD)  = f(VCO_out) / PLLQ = 336 MHz / 7 = 48.0 MHz (Strict 48 MHz for USB)
     *
     * Bitfield encodings:
     *   PLLM[5:0]   : 4U  (Division by 4)
     *   PLLN[14:6]  : 168U (0xA8, Multiplier factor 168)
     *   PLLP[17:16] : 0x1 (0b01 encodes division factor /4: 00=/2, 01=/4, 10=/6, 11=/8)
     *   PLLSRC[22]  : 0x1 (1 = HSE selected as PLL source)
     *   PLLQ[27:24] : 7U  (Division by 7 for 48 MHz USB OTG FS)
     * --------------------------------------------------------------------------- */
    uint32_t PLLM   = 4U << 0;
    uint32_t PLLN   = 168U << 6;
    uint32_t PLLP   = 0x1 << 16;
    uint32_t PLLSRC = 0x1 << 22;
    uint32_t PLLQ   = 7U << 24;

    /* Mask-clear all PLL configuration bitfields (Bits 0-5, 6-14, 16-17, 22, 24-27) */
    uint32_t PLLCFGR_CLEAR_MASK = (0x3F << 0)  | 
                                  (0x1FF << 6) | 
                                  (0x3 << 16)  | 
                                  (0x1 << 22)  | 
                                  (0xF << 24);
    RCC->PLLCFGR &= ~PLLCFGR_CLEAR_MASK;
    RCC->PLLCFGR |= (PLLM | PLLN | PLLP | PLLSRC | PLLQ);

    /* ---------------------------------------------------------------------------
     * STEP 4: Configure Bus Prescalers on RCC->CFGR
     * Maximum Bus Frequencies:
     *   AHB  Max = 84 MHz  -> HPRE  = /1 (84 MHz)
     *   APB1 Max = 50 MHz  -> PPRE1 = /2 (42 MHz) -> Required for USART2, I2C1, TIM2/3/4
     *   APB2 Max = 100 MHz -> PPRE2 = /1 (84 MHz) -> SPI1, USART1/6, ADC1
     *
     * Encodings:
     *   HPRE[7:4]   : 0b0000 (/1)
     *   PPRE1[12:10]: 0b100  (/2)
     *   PPRE2[15:13]: 0b000  (/1)
     * --------------------------------------------------------------------------- */
    /* Clear HPRE [7:4], PPRE1 [12:10], and PPRE2 [15:13] before setting */
    uint32_t BUS_PRESCALER_CLEAR_MASK = (0xF << 4) | (0x7 << 10) | (0x7 << 13);
    RCC->CFGR &= ~BUS_PRESCALER_CLEAR_MASK;

    uint32_t HPRE  = 0b0000 << 4;
    uint32_t PPRE1 = 0b100 << 10;
    uint32_t PPRE2 = 0b000 << 13;
    RCC->CFGR |= (HPRE | PPRE1 | PPRE2);

    /* ---------------------------------------------------------------------------
     * STEP 5: Enable the Main PLL Engine
     * Bit 24 in RCC->CR is PLLON.
     * Bit 25 in RCC->CR is PLLRDY (hardware lock confirmation).
     * --------------------------------------------------------------------------- */
    uint32_t PLLON = 0x1 << 24;
    RCC->CR |= PLLON;

    uint32_t PLLRDY = 0x1 << 25;
    while (!(RCC->CR & PLLRDY)) {
        /* Wait until PLL frequency synthesizer is locked */
    }

    /* ---------------------------------------------------------------------------
     * STEP 6: Switch System Clock to Main PLL
     * Bits [1:0] in RCC->CFGR (SW):
     *   0b00 = HSI, 0b01 = HSE, 0b10 = PLL
     * Bits [3:2] in RCC->CFGR (SWS):
     *   System Clock Switch Status (Read-Only)
     * --------------------------------------------------------------------------- */
    /* Clear SW [1:0] before switching */
    uint32_t SW_CLEAR_MASK = 0x3 << 0;
    RCC->CFGR &= ~SW_CLEAR_MASK;

    uint32_t SW_PLL = 0b10 << 0;
    RCC->CFGR |= SW_PLL;

    /* Loop until hardware SWS field confirms PLL is active system clock source */
    uint32_t SWS_MASK       = 0x3 << 2;
    uint32_t SWS_PLL_ACTIVE = 0b10 << 2;
    while ((RCC->CFGR & SWS_MASK) != SWS_PLL_ACTIVE) {
        /* Wait for clock switchover */
    }

    /* ---------------------------------------------------------------------------
     * STEP 7: Disable Internal High-Speed RC Oscillator (HSI)
     * Now that the core runs deterministically on HSE+PLL, turn off HSI to 
     * eliminate noise and unnecessary power consumption.
     * Bit 0 in RCC->CR is HSION.
     * --------------------------------------------------------------------------- */
    uint32_t HSION = 0x1 << 0;
    RCC->CR &= ~HSION;
}


/*
 * ===============================================================================
 * FUNCTION: TIM3_PWM_Init
 * PURPOSE : Configures Timer 3 Channel 1 for PWM generation on Port A Pin 6.
 * Target  : STM32F411xE (Timer Clock = 84 MHz)
 * ===============================================================================
 */
void TIM3_PWM_Init(uint32_t prescalar, uint32_t auto_reload_value, uint32_t duty_cycle) {
    /* ---------------------------------------------------------------------------
     * STEP 1: Enable GPIOA (AHB1) and TIM3 (APB1) Clocks
     * --------------------------------------------------------------------------- */
    RCC->AHB1ENR |= (1 << 0);  // GPIOAEN
    RCC->APB1ENR |= (1 << 1);  // TIM3EN

    /* ---------------------------------------------------------------------------
     * STEP 2: Configure PA6 for Alternate Function (AF2 = TIM3_CH1)
     * --------------------------------------------------------------------------- */
    // Set MODER to Alternate Function (0b10) on Pin 6 (bits 12-13)
    GPIOA->MODER &= ~(0x3 << (6 * 2));
    GPIOA->MODER |=  (0x2 << (6 * 2));

    // Map AF2 (TIM3_CH1) in AFR[0] (AFRL) on Pin 6 (bits 24-27)
    GPIOA->AFR[0] &= ~(0xF << (6 * 4));
    GPIOA->AFR[0] |=  (0x2 << (6 * 4));

    // Set High Output Speed (0b10) for sharp PWM edges (bits 12-13)
    GPIOA->OSPEEDR &= ~(0x3 << (6 * 2));
    GPIOA->OSPEEDR |=  (0x2 << (6 * 2));

    // No pull-up, no pull-down
    GPIOA->PUPDR &= ~(0x3 << (6 * 2));

    /* ---------------------------------------------------------------------------
     * STEP 2b: Configure PA7 for Alternate Function (AF2 = TIM3_CH2)
     * --------------------------------------------------------------------------- */
    // Set MODER to Alternate Function (0b10) on Pin 7 (bits 14-15)
    GPIOA->MODER &= ~(0x3 << (7 * 2));
    GPIOA->MODER |=  (0x2 << (7 * 2));

    // Map AF2 (TIM3_CH2) in AFR[0] (AFRL) on Pin 7 (bits 28-31)
    GPIOA->AFR[0] &= ~(0xF << (7 * 4));
    GPIOA->AFR[0] |=  (0x2 << (7 * 4));

    // Set High Output Speed for sharp PWM edges (bits 14-15)
    GPIOA->OSPEEDR &= ~(0x3 << (7 * 2));
    GPIOA->OSPEEDR |=  (0x2 << (7 * 2));

    // No pull-up, no pull-down
    GPIOA->PUPDR &= ~(0x3 << (7 * 2));

    /* ---------------------------------------------------------------------------
     * STEP 3: Configure Timer Frequency Base & Initial Duty Cycle
     * --------------------------------------------------------------------------- */
    TIM3->PSC  = prescalar;          // Prescaler (Divides by PSC + 1)
    TIM3->ARR  = auto_reload_value;   // Period (Period = ARR + 1)
    TIM3->CCR1 = duty_cycle;          // Channel 1 Duty Cycle
    TIM3->CCR2 = duty_cycle;          // Channel 2 Duty Cycle

    /* ---------------------------------------------------------------------------
     * STEP 4: Configure Channel 1 for PWM Mode 1 with Preload
     * --------------------------------------------------------------------------- */
    // Clear CC1S [1:0] (00 = Output) and OC1M [6:4] before configuring
    TIM3->CCMR1 &= ~((0x3 << 0) | (0x7 << 4));
    // Set OC1M = 0b110 (PWM Mode 1) and OC1PE = 1 (Preload Enable)
    TIM3->CCMR1 |=  ((0x6 << 4) | (0x1 << 3));

    // Configure Channel 2 for PWM Mode 1 with Preload
    // OC2S [8:9] = 00 (Output), OC2M [10:12] = 110 (PWM Mode 1), OC2PE = bit 11
    TIM3->CCMR1 &= ~((0x3 << 8) | (0x7 << 12));
    TIM3->CCMR1 |=  ((0x6 << 12) | (0x1 << 11));

    // Enable Channel 1 Output on Pin (CC1E = 1)
    TIM3->CCER |= (1 << 0);
    // Enable Channel 2 Output on Pin (CC2E = 1)
    TIM3->CCER |= (1 << 4);

    // Enable Auto-Reload Preload (ARPE = 1) to prevent glitching on ARR updates
    TIM3->CR1 |= (1 << 7);

    // Force an update event to transfer PSC & ARR into the active shadow registers
    TIM3->EGR |= (1 << 0); // UG bit

    /* ---------------------------------------------------------------------------
     * STEP 5: Start Counter
     * --------------------------------------------------------------------------- */
    TIM3->CR1 |= (1 << 0); // CEN bit
}




void delay(uint32_t value) {
        while (value--){;}
  }

void enable_systick(){
	uint32_t CLK_SRC = 0x1 << 2;    // Set to 1 to use Processor Clock (84 MHz)
    uint32_t INTEN   = 0x1 << 1;    // Enable SysTick interrupt
    uint32_t ENABLE  = 0x1 << 0;    // Enable SysTick counter
    uint32_t RELOAD  = 83999;       // 84,000 cycles at 84 MHz = 1 ms (1 kHz)
    
    uint32_t SYSTICK_PRI = 0x4 << 4; // SysTick Priority (upper 4 bits relevant)
    
    asm volatile("cpsid i");         // Disabling all interrupts with configurable priority during config
    /*Enabling SysTick interrupt and setting a priority of 2
    The SYSTICK interrupt registers are different from the conventional interrupt config registers
    Enabling the interrupts here instead of a separate function since the interrupt registers are common with systick*/
    SCB->SHP[11] = SYSTICK_PRI; //This is a byte access
    SysTick->LOAD = RELOAD ; 
    SysTick->CTRL &= ~(0b111);
    SysTick->CTRL |= ENABLE | INTEN | CLK_SRC;
    asm volatile("cpsie i"); //Enabling all interrupts with configurable priority
 }

void configure_EXTI0_PortA0(){
	asm volatile("cpsid i"); //Disabling all interrupts with configurable priority
	uint32_t EXTICR1A0 = 0xF << 0; //PA0 is configured to accept EXTI0 interrupt
	SYSCFG->EXTICR[0] &= ~EXTICR1A0; // Clearing the INTI0 field of EXTICR1
	                                 // Nothing more to be done here PA0 is now
	                                 // linked to INTI0
	uint32_t ENINTL0 = 0x1 << 0;     // Configure interrupt line enable 0 bit
	EXTI->IMR |= ENINTL0;            // Enable interrupt line 0
    uint32_t RISTRIGINTI0 = 0x1 << 0; // Configure rising trigger variable for INTI0
                                      // the button is active high
    EXTI->RTSR |= RISTRIGINTI0;       // Enable interrupt on rising trigger
    uint32_t NVICINTI0EN = 0x1 << 6;  // Configure enable bit variable in NVIC for EXTII0
    NVIC->ISER[0] |= NVICINTI0EN;     // Enabling ISER0 in NVIC


    uint8_t INTI0PRIO = 0x3 << 4;     // Setting the INTI0 priority variable
                                      // left shift by 4 bytes as upper 4 bits are used
    NVIC->IP[6] &= (uint8_t)~0xFF;    // Clearing the NVIC INTI0 priority field
    NVIC->IP[6] |=  INTI0PRIO;        // Setting the interrupt priority here
    asm volatile("cpsie i");          //Enabling all interrupts with configurable priority
}

void configure_user_button(){
	uint32_t GPIOACLOCK = 0x1 << 0; //GPIO A position
	RCC->AHB1ENR |= GPIOACLOCK; /* enable GPIOA clock */
	uint32_t BUTTONPINSCLR = ~(0b11 << 0); //These bits clear the mode for the port A0 pin
	GPIOA->MODER &= BUTTONPINSCLR; /* clear pin mode */

	uint32_t BUTTONPINSMODE = 0b00 << 0; //Configuration to set Port A0 as input port
	                                     //Actually this is set by default but just keeping it here
	GPIOA->MODER |= BUTTONPINSMODE;      // Set Port A0 as input port
	uint32_t ACTIVELOW = 0x2;
	GPIOA->PUPDR |= ACTIVELOW << 0; // Setting pin to active low here
}

void enable_LEDs(){
   uint32_t GPIODCLOCK = 0x1 << 3 ; //GPIO D position	
   RCC->AHB1ENR |= GPIODCLOCK; /* enable GPIOD clock */
   uint32_t LEDPINSCLR = ~(0xFF << 24); //These bits clear the mode of all 4 LEDs 	
   GPIOD->MODER &= LEDPINSCLR; /* clear pin mode */
   uint32_t LEDPINSMODE = 0b01000000 << 24; //Set blue LED port as output port PD15
   GPIOD->MODER |= LEDPINSMODE; /* set pins to output mode */    		
}

void blink_LED(uint32_t duration){
    
}

 void toggle_LED(){
    //Toggles all LEDs
	uint32_t ALLLEDS = 0b1111 << 12; //Blue LED position in ODR
    GPIOD->ODR ^= ALLLEDS; // Toggle the LED   
 }


 
 void initialise_FPU(){
	 uint32_t ENABLECP10CP11 = 0xF << 20;
	 SCB->CPACR |= ENABLECP10CP11;
	 asm("DSB");
	 asm("ISB");

   }


/*
Initialize I2C1 for maximum speed (Fast Mode up to 400kHz) assuming a 42 MHz APB1 bus.
Uses PB8 (SCL) and PB9 (SDA) with Alternate Function 4 (AF4).
*/
void I2C1_init() {
    // 1. Enable GPIOB clock and I2C1 clock
    uint32_t GPIOBCLOCK = 0x1 << 1; 
    RCC->AHB1ENR |= GPIOBCLOCK; // Enable GPIOB clock
    
    uint32_t I2C1EN = 0x1 << 21;
    RCC->APB1ENR |= I2C1EN; // Enable I2C1 clock in APB1ENR

    // 2. Configure PB8 for I2C1_SCL and PB9 for I2C1_SDA (Alternate Function 4)
    // Configure PB8 Alternate Function
    GPIOB->AFR[1] &= ~0x0000000F; // Clear the field for PB8 alt function
    uint32_t GPIOPB8ALT = 0x4 << 0; 
    GPIOB->AFR[1] |= GPIOPB8ALT; // AF4 for I2C1 PB8
    GPIOB->MODER &= ~0x00030000; // Clear alt function enable pin for PB8
    uint32_t PB8ALTEN = 0x2 << 16;
    GPIOB->MODER |= PB8ALTEN;  // Enable alternate function for PB8

    // Configure PB9 Alternate Function
    GPIOB->AFR[1] &= ~0x000000F0; // Clear the field for PB9 alt function
    uint32_t GPIOPB9ALT = 0x4 << 4; 
    GPIOB->AFR[1] |= GPIOPB9ALT; // AF4 for I2C1 PB9
    GPIOB->MODER &= ~0x000C0000; // Clear alt function enable pin for PB9
    uint32_t PB9ALTEN = 0x2 << 18;
    GPIOB->MODER |= PB9ALTEN;  // Enable alternate function for PB9

    // Set PB8 and PB9 to Open-Drain (ODR) and Pull-Up for I2C bus compliance
    GPIOB->OTYPER |= (0x1 << 8) | (0x1 << 9); // Open-drain output type
    GPIOB->PUPDR  |= (0x1 << 16) | (0x1 << 18); // Pull-up enabled

    // 3. Peripheral Configuration
    // Ensure I2C is disabled before configuring parameters
    I2C1->CR1 &= ~(0x1 << 0); // Clear PE (Peripheral Enable) bit

    // Configure CR2 with APB1 frequency in MHz (42 MHz for 84MHz SYSCLK with APB1 prescaler = 2)
    uint32_t FREQ_42MHZ = 42U;
    I2C1->CR2 = FREQ_42MHZ; 

    // Configure CCR for Fast Mode (400 kHz)
    // Formula for Fast Mode (DUTY = 0): Thigh = 2 * Tlow, T_pclk1 = 1 / 42MHz
    // CCR = APB1_CLK / (350 * Target_Freq) -> 42,000,000 / (350 * 400,000) = 3 -> let's use standard calculation:
    // T_high = CCR * T_pclk1, T_low = 2 * CCR * T_pclk1 -> F_scl = APB1_CLK / (3 * CCR) => CCR = 42,000,000 / (3 * 400,000) = 35 (0x23)
    uint32_t FAST_MODE = 0x1 << 15; // F/S mode = 1 (Fast mode)
    uint32_t CCR_VAL = 35U;        // Calculated for 400kHz @ 42MHz APB1
    I2C1->CCR = FAST_MODE | CCR_VAL;

    // Configure TRISE for Fast Mode
    // Formula: TRISE = (Maximum SCL rise time in fast mode (300ns) / T_pclk1) + 1
    // (300ns * 42MHz) + 1 = 12.6 + 1 = 14 (0x0E)
    uint32_t TRISE_VAL = 14U;
    I2C1->TRISE = TRISE_VAL;

    // 4. Enable I2C1 Peripheral
    I2C1->CR1 |= (0x1 << 0); // Set PE bit to enable I2C1
}



///This segment configures all the DMAs for the application///

/*
 * ===============================================================================
 * FUNCTION: USART1_DMA_RX_Init
 * PURPOSE : Configures DMA2 Stream 5 Channel 4 for autonomous USART1 reception.
 *           Fires DMA2_Stream5_IRQHandler every 12 bytes (1 ms lockstep boundary).
 * ===============================================================================
 */
void USART1_DMA_RX_Init(void) {
    /* ---------------------------------------------------------------------------
     * STEP 1: Enable the DMA2 Clock on the AHB1 Bus
     * Bit 22 in RCC->AHB1ENR enables DMA2 clock.
     * --------------------------------------------------------------------------- */
    uint32_t DMA2_CLOCK_EN = 0x1 << 22;
    RCC->AHB1ENR |= DMA2_CLOCK_EN;

    /* ---------------------------------------------------------------------------
     * STEP 2: Disable the Stream before configuration
     * Silicon rule: Registers cannot be modified while EN (bit 0) is 1.
     * --------------------------------------------------------------------------- */
    uint32_t STREAM_EN = 0x1 << 0;
    DMA2_Stream5->CR &= ~STREAM_EN;
    while (DMA2_Stream5->CR & STREAM_EN) {
        /* Wait until hardware confirms the stream is completely halted */
    }

    /* ---------------------------------------------------------------------------
     * STEP 3: Clear any stale interrupt flags for Stream 5 in HIFCR
     * Stream 5 flags are in the High Interrupt Flag Clear Register (HIFCR).
     * Bits 6 to 11 represent Stream 5:
     *   Bit 11: CTCIF5 (Clear Transfer Complete flag)
     *   Bit 10: CHTIF5 (Clear Half Transfer flag)
     *   Bit 9 : CTEIF5 (Clear Transfer Error flag)
     *   Bit 8 : CDMEIF5 (Clear Direct Mode Error flag)
     *   Bit 6 : CFEIF5 (Clear FIFO Error flag)
     * --------------------------------------------------------------------------- */
    uint32_t CLEAR_STREAM5_FLAGS = (0x1 << 11) | (0x1 << 10) | (0x1 << 9) | 
                                   (0x1 << 8)  | (0x1 << 6);
    DMA2->HIFCR |= CLEAR_STREAM5_FLAGS;

    /* ---------------------------------------------------------------------------
     * STEP 4: Set the Source Peripheral Address (PAR)
     * The Data Register (DR) of USART1 sits at offset 0x04.
     * Address: USART1_BASE + 0x04 = 0x40011004
     * --------------------------------------------------------------------------- */
    DMA2_Stream5->PAR = (uint32_t)&(USART1->DR);

    /* ---------------------------------------------------------------------------
     * STEP 5: Set the Destination Memory Address (M0AR)
     * Address of our RAM buffer.
     * --------------------------------------------------------------------------- */
    DMA2_Stream5->M0AR = (uint32_t)&rx_dma_buffer[0];

    /* ---------------------------------------------------------------------------
     * STEP 6: Set Number of Data Items to Transfer (NDTR)
     * Exact packet size: 12 bytes.
     * --------------------------------------------------------------------------- */
    DMA2_Stream5->NDTR = HIL_PACKET_SIZE;

    /* ---------------------------------------------------------------------------
     * STEP 7: Configure Control Register (CR)
     * First clear all configurable fields (Bits 0 to 27)
     * --------------------------------------------------------------------------- */
    DMA2_Stream5->CR &= 0xF0000000; // Clear lower 28 bits

    uint32_t CHSEL_CH4   = 0x4 << 25; // Select Channel 4 (USART1_RX)
    uint32_t PL_VERYHIGH = 0x3 << 16; // Priority level: Very High (0b11)
    uint32_t MSIZE_8BIT  = 0x0 << 13; // Memory data size: 8-bit byte (0b00)
    uint32_t PSIZE_8BIT  = 0x0 << 11; // Peripheral data size: 8-bit byte (0b00)
    uint32_t MINC_ENABLE = 0x1 << 10; // Memory pointer increments after each byte
    uint32_t PINC_FIXED  = 0x0 << 9;  // Peripheral address stays fixed at USART1->DR
    uint32_t CIRC_MODE   = 0x1 << 8;  // Circular Mode: auto-reloads NDTR on completion
    uint32_t DIR_P_TO_M  = 0x0 << 6;  // Direction: Peripheral-to-Memory (0b00)
    uint32_t TCIE_ENABLE = 0x1 << 4;  // Transfer Complete Interrupt Enable

    DMA2_Stream5->CR |= (CHSEL_CH4 | PL_VERYHIGH | MSIZE_8BIT | PSIZE_8BIT |
                         MINC_ENABLE | PINC_FIXED | CIRC_MODE | DIR_P_TO_M | 
                         TCIE_ENABLE);

    /* ---------------------------------------------------------------------------
     * STEP 8: Configure NVIC for DMA2 Stream 5
     * On STM32F411, DMA2 Stream 5 is IRQ number 68.
     * IRQ 68 sits in ISER[2] at bit 4 (68 % 32 = 4).
     * --------------------------------------------------------------------------- */
    asm volatile("cpsid i");                  // Disable interrupts during NVIC setup
    uint32_t DMA2_STR5_IRQ_MASK = 0x1 << (68 % 32); // Bit 4 in ISER[2]
    NVIC->ISER[2] |= DMA2_STR5_IRQ_MASK;      // Enable IRQ 68 in NVIC

    uint8_t PRIORITY_LEVEL = 0x2 << 4;        // High priority (upper 4 bits used)
    NVIC->IP[68] = PRIORITY_LEVEL;
    asm volatile("cpsie i");                  // Re-enable interrupts

    /* ---------------------------------------------------------------------------
     * STEP 9: Enable USART1 DMA Mode Receiver (DMAR)
     * Bit 6 in USART1->CR3 activates DMA trigger on received bytes.
     * --------------------------------------------------------------------------- */
    uint32_t DMAR_ENABLE = 0x1 << 6;
    USART1->CR3 |= DMAR_ENABLE;

    /* ---------------------------------------------------------------------------
     * STEP 10: Enable DMA2 Stream 5 Engine
     * Set EN bit to start listening for incoming bytes.
     * --------------------------------------------------------------------------- */
    DMA2_Stream5->CR |= STREAM_EN;
}

 
/*
 * ===============================================================================
 * FUNCTION: USART1_DMA_TX_Init
 * PURPOSE : Pre-configures DMA2 Stream 7 Channel 4 for USART1 Transmission.
 *           Configured in Normal (one-shot) mode.
 * ===============================================================================
 */
void USART1_DMA_TX_Init(void) {
    /* ---------------------------------------------------------------------------
     * STEP 1: Enable the DMA2 Clock on the AHB1 Bus
     * Bit 22 in RCC->AHB1ENR enables DMA2 clock.
     * --------------------------------------------------------------------------- */
    RCC->AHB1ENR |= (0x1 << 22);
    /* ---------------------------------------------------------------------------
     * STEP 2: Disable the Stream before configuration
     * Silicon rule: Stream registers are write-protected while EN is 1.
     * --------------------------------------------------------------------------- */
    uint32_t STREAM_EN = 0x1 << 0;
    DMA2_Stream7->CR &= ~STREAM_EN;
    while (DMA2_Stream7->CR & STREAM_EN) {
        /* Wait until hardware confirms stream is completely disabled */
    }
    /* ---------------------------------------------------------------------------
     * STEP 3: Clear any stale interrupt flags for Stream 7 in HIFCR
     * Stream 7 flags are in the High Interrupt Flag Clear Register (HIFCR):
     *   Bit 27: CTCIF7 (Clear Transfer Complete flag)
     *   Bit 26: CHTIF7 (Clear Half Transfer flag)
     *   Bit 25: CTEIF7 (Clear Transfer Error flag)
     *   Bit 24: CDMEIF7 (Clear Direct Mode Error flag)
     *   Bit 22: CFEIF7 (Clear FIFO Error flag)
     * --------------------------------------------------------------------------- */
    uint32_t CLEAR_STREAM7_FLAGS = (0x1 << 27) | (0x1 << 26) | (0x1 << 25) | 
                                   (0x1 << 24) | (0x1 << 22);
    DMA2->HIFCR |= CLEAR_STREAM7_FLAGS;
    /* ---------------------------------------------------------------------------
     * STEP 4: Set the Destination Peripheral Address (PAR)
     * Destination is USART1 Data Register (DR).
     * Address: USART1_BASE + 0x04 = 0x40011004
     * --------------------------------------------------------------------------- */
    DMA2_Stream7->PAR = (uint32_t)&(USART1->DR);
    /* ---------------------------------------------------------------------------
     * STEP 5: Set the Source Memory Address (M0AR)
     * Source is our aligned transmit buffer in RAM.
     * --------------------------------------------------------------------------- */
    DMA2_Stream7->M0AR = (uint32_t)&tx_dma_buffer[0];
    /* ---------------------------------------------------------------------------
     * STEP 6: Configure Control Register (CR)
     * First clear all configurable fields (Bits 0 to 27)
     * --------------------------------------------------------------------------- */
    DMA2_Stream7->CR &= 0xF0000000; // Clear lower 28 bits
    uint32_t CHSEL_CH4   = 0x4 << 25; // Select Channel 4 (USART1_TX)
    uint32_t PL_HIGH     = 0x2 << 16; // Priority level: High (0b10)
    uint32_t MSIZE_8BIT  = 0x0 << 13; // Memory data size: 8-bit byte (0b00)
    uint32_t PSIZE_8BIT  = 0x0 << 11; // Peripheral data size: 8-bit byte (0b00)
    uint32_t MINC_ENABLE = 0x1 << 10; // Memory pointer increments after each byte
    uint32_t NORMAL_MODE = 0x0 << 8;  // Normal mode (0b0): Single-shot, no auto-reload
    uint32_t DIR_M_TO_P  = 0x1 << 6;  // Direction: Memory-to-Peripheral (0b01)
    DMA2_Stream7->CR |= (CHSEL_CH4 | PL_HIGH | MSIZE_8BIT | PSIZE_8BIT |
                         MINC_ENABLE | NORMAL_MODE | DIR_M_TO_P);
    /* ---------------------------------------------------------------------------
     * STEP 7: Enable USART1 DMA Transmitter mode (DMAT)
     * Bit 7 in USART1->CR3 activates DMA trigger on TX buffer empty.
     * --------------------------------------------------------------------------- */
    uint32_t DMAT_ENABLE = 0x1 << 7;
    USART1->CR3 |= DMAT_ENABLE;
}
/*
 * ===============================================================================
 * FUNCTION: I2C1_DMA_OLED_Init
 * PURPOSE : Configures DMA1 Stream 7 Channel 1 for I2C1 Transmit.
 * ===============================================================================
 */
void I2C1_DMA_OLED_Init(void) {
    // 1. Enable DMA1 clock
    RCC->AHB1ENR |= (0x1 << 21);

    // 2. Disable Stream 7 before configuration
    uint32_t STREAM_EN = 0x1 << 0;
    DMA1_Stream7->CR &= ~STREAM_EN;
    while (DMA1_Stream7->CR & STREAM_EN) {}

    // 3. Clear Stream 7 flags in HIFCR (Bits 22 to 27)
    uint32_t CLEAR_STREAM7_FLAGS = (0x1 << 27) | (0x1 << 26) | (0x1 << 25) | 
                                   (0x1 << 24) | (0x1 << 22);
    DMA1->HIFCR |= CLEAR_STREAM7_FLAGS;

    // 4. Peripheral address is I2C1 Data Register
    DMA1_Stream7->PAR = (uint32_t)&(I2C1->DR);

    // 5. Source is our framebuffer in RAM
    DMA1_Stream7->M0AR = (uint32_t)&oled_buffer[0];

    // 6. Configure CR: Channel 1, Memory Increment, Normal Mode, M-to-P
    DMA1_Stream7->CR &= 0xF0000000;
    uint32_t CHSEL_CH1   = 0x1 << 25; // Channel 1 (I2C1_TX)
    uint32_t PL_LOW      = 0x0 << 16; // Priority level: Low (UART takes priority)
    uint32_t MSIZE_8BIT  = 0x0 << 13;
    uint32_t PSIZE_8BIT  = 0x0 << 11;
    uint32_t MINC_ENABLE = 0x1 << 10;
    uint32_t DIR_M_TO_P  = 0x1 << 6;

    DMA1_Stream7->CR |= (CHSEL_CH1 | PL_LOW | MSIZE_8BIT | PSIZE_8BIT | 
                         MINC_ENABLE | DIR_M_TO_P);

    // 7. Enable DMA mode in I2C1 (DMAEN = 1 in CR2)
    I2C1->CR2 |= (0x1 << 11);
}

/* Global system clock variable required by TinyUSB turnaround calculations */
uint32_t SystemCoreClock = 84000000U;

/*
 * ===============================================================================
 * FUNCTION: usb_hardware_init
 * PURPOSE : Configures physical pins and clocks for USB OTG FS.
 *           - PA11: OTG_FS_DM (AF10)
 *           - PA12: OTG_FS_DP (AF10)
 *           - Clocks: GPIOA (AHB1) + USB OTG FS (AHB2)
 *           - NVIC: IRQ 67 (OTG_FS_IRQn) enabled at Priority 5
 * ===============================================================================
 */
void usb_hardware_init(void) {
    /* 1. Enable peripheral clocks */
    RCC->AHB1ENR |= (1 << 0);  /* Enable GPIOA clock */
    RCC->AHB2ENR |= (1 << 7);  /* Enable USB OTG FS peripheral clock */

    /* 2. Configure PA11 (DM) and PA12 (DP) as Alternate Function AF10 */
    // Clear MODER bits for PA11 (bits 22-23) and PA12 (bits 24-25)
    GPIOA->MODER &= ~((3U << 22) | (3U << 24));
    // Set to Alternate Function mode (0b10)
    GPIOA->MODER |=  ((2U << 22) | (2U << 24));

    // Clear and configure AFR[1] (Alternate Function High register)
    GPIOA->AFR[1] &= ~((0xFU << 12) | (0xFU << 16));  // Clear PA11 and PA12
    GPIOA->AFR[1] |=  ((10U  << 12) | (10U  << 16));  // Set AF10 (USB OTG FS)

    // Output Speed: Very High Speed (0b11)
    GPIOA->OSPEEDR |= ((3U << 22) | (3U << 24));

    // Pull-up / Pull-down: No pull (0b00)
    GPIOA->PUPDR &= ~((3U << 22) | (3U << 24));

    /* 3. Configure NVIC Priority for USB OTG FS (IRQ 67) */
    // Priority level 5 (upper 4 bits on STM32 Cortex-M4)
    NVIC->IP[67] = (5U << 4);

    // Enable IRQ 67 in NVIC ISER[2] (67 % 32 = 3)
    NVIC->ISER[2] = (1U << (67 % 32));
}
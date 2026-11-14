/**
  ******************************************************************************
  * @file    memory_map_stm32f411xe.h
  * @author  MCD Application Team
  * @brief   CMSIS STM32F411xE Device Peripheral Access Layer Header File.
  *
  *          This file contains:
  *           - Data structures and the address mapping for all peripherals
  *           - peripherals registers declarations and bits definition
  *           - Macros to access peripheral’s registers hardware
  *           
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
 /*
  Mofified and abbreviated 
 */

    
#ifndef __MEMORY_MAP_STM32F411xE_H
#define __MEMORY_MAP_STM32F411xE_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */



/*
  Configuration of the Cortex-M4 Processor and Core Peripherals 
*/
#define __CM4_REV                 0x0001U  /*!< Core revision r0p1                            */
#define __MPU_PRESENT             1U       /*!< STM32F4XX provides an MPU                     */
#define __NVIC_PRIO_BITS          4U       /*!< STM32F4XX uses 4 Bits for the Priority Levels */
#define __Vendor_SysTickConfig    0U       /*!< Set to 1 if different SysTick Config is used  */
#define __FPU_PRESENT             1U       /*!< FPU present                                   */

//Only for testing 	 
typedef unsigned int uint32_t;
typedef unsigned short uint16_t;
typedef unsigned char uint8_t;
typedef int int32_t;
typedef short int16_t;
typedef char int8_t;	 

  
/*
Commenting this section out and bringing in all necessary mapping within this file
#include "core_cm4.h"             // Cortex-M4 processor and core peripherals
#include "system_stm32f4xx.h"
#include <stdint.h>
*/
	 
/* 
Private Peripherals Memory Map 
Refer to dm00046982-stm32-cortex-m4-mcus-and-mpus-programming-manual-stmicroelectronics.pdf for
programming model and Internal peripherals.
*/
#define SCS_BASE            0xE000E000UL                            /*!< System Control Space Base Address */
#define ITM_BASE            0xE0000000UL                            /*!< ITM Base Address */
#define DWT_BASE            0xE0001000UL                            /*!< DWT Base Address */
#define TPI_BASE            0xE0040000UL                            /*!< TPI Base Address */
#define CoreDebug_BASE      0xE000EDF0UL                            /*!< Core Debug Base Address */
#define SysTick_BASE        (SCS_BASE +  0x0010UL)                    /*!< SysTick Base Address */
#define NVIC_BASE           (SCS_BASE +  0x0100UL)                    /*!< NVIC Base Address */
#define SCB_BASE            (SCS_BASE +  0x0D00UL)                    /*!< System Control Block Base Address */
#define MPU_BASE            (SCS_BASE +  0x0D90UL)                    /*!< Memory Protection Unit */	
#define FPU_BASE            (SCS_BASE +  0x0F30UL)                    /*!< Floating Point Unit */

	 
/**
  \brief  Structure type to access the System Control and ID Register not in the SCB.
 */
typedef struct
{
  uint32_t RESERVED0[1U];
  volatile const  uint32_t ICTR;            /*!< Offset: 0x004 (R/ )  Interrupt Controller Type Register */
  volatile uint32_t ACTLR;                  /*!< Offset: 0x008 (R/W)  Auxiliary Control Register */
} SCnSCB_Type;
	 
#define SCnSCB              ((SCnSCB_Type    *)     SCS_BASE      )   /*!< System control Register not in SCB */

/**
  \brief  Structure type to access the System Control Block (SCB).
 */
typedef struct
{
  volatile const  uint32_t CPUID;           /*!< Offset: 0x000 (R/ )  CPUID Base Register */
  volatile uint32_t ICSR;                   /*!< Offset: 0x004 (R/W)  Interrupt Control and State Register */
  volatile uint32_t VTOR;                   /*!< Offset: 0x008 (R/W)  Vector Table Offset Register */
  volatile uint32_t AIRCR;                  /*!< Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register */
  volatile uint32_t SCR;                    /*!< Offset: 0x010 (R/W)  System Control Register */
  volatile uint32_t CCR;                    /*!< Offset: 0x014 (R/W)  Configuration Control Register */
  volatile uint8_t  SHP[12U];               /*!< Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15) */
  volatile uint32_t SHCSR;                  /*!< Offset: 0x024 (R/W)  System Handler Control and State Register */
  volatile uint32_t CFSR;                   /*!< Offset: 0x028 (R/W)  Configurable Fault Status Register */
  volatile uint32_t HFSR;                   /*!< Offset: 0x02C (R/W)  HardFault Status Register */
  volatile uint32_t DFSR;                   /*!< Offset: 0x030 (R/W)  Debug Fault Status Register */
  volatile uint32_t MMFAR;                  /*!< Offset: 0x034 (R/W)  MemManage Fault Address Register */
  volatile uint32_t BFAR;                   /*!< Offset: 0x038 (R/W)  BusFault Address Register */
  volatile uint32_t AFSR;                   /*!< Offset: 0x03C (R/W)  Auxiliary Fault Status Register */
  volatile const  uint32_t PFR[2U];         /*!< Offset: 0x040 (R/ )  Processor Feature Register */
  volatile const  uint32_t DFR;             /*!< Offset: 0x048 (R/ )  Debug Feature Register */
  volatile const  uint32_t ADR;             /*!< Offset: 0x04C (R/ )  Auxiliary Feature Register */
  volatile const  uint32_t MMFR[4U];        /*!< Offset: 0x050 (R/ )  Memory Model Feature Register */
  volatile const  uint32_t ISAR[5U];        /*!< Offset: 0x060 (R/ )  Instruction Set Attributes Register */
  uint32_t RESERVED0[5U];
  volatile uint32_t CPACR;                  /*!< Offset: 0x088 (R/W)  Coprocessor Access Control Register */
} SCB_Type;	 
	 
#define SCB                 ((SCB_Type       *)     SCB_BASE      )   /*!< SCB configuration struct */

/**
  \brief  Structure type to access the System Timer (SysTick).
 */
typedef struct
{
  volatile uint32_t CTRL;                   /*!< Offset: 0x000 (R/W)  SysTick Control and Status Register */
  volatile uint32_t LOAD;                   /*!< Offset: 0x004 (R/W)  SysTick Reload Value Register */
  volatile uint32_t VAL;                    /*!< Offset: 0x008 (R/W)  SysTick Current Value Register */
  volatile const  uint32_t CALIB;                  /*!< Offset: 0x00C (R/ )  SysTick Calibration Register */
} SysTick_Type;	 
	 
#define SysTick             ((SysTick_Type   *)     SysTick_BASE  )   /*!< SysTick configuration struct */


/**
  \brief  Structure type to access the Nested Vectored Interrupt Controller (NVIC).
 */
typedef struct
{
  volatile uint32_t ISER[8U];               /*!< Offset: 0x000 (R/W)  Interrupt Set Enable Register */
  uint32_t RESERVED0[24U];
  volatile uint32_t ICER[8U];               /*!< Offset: 0x080 (R/W)  Interrupt Clear Enable Register */
  uint32_t RSERVED1[24U];
  volatile uint32_t ISPR[8U];               /*!< Offset: 0x100 (R/W)  Interrupt Set Pending Register */
  uint32_t RESERVED2[24U];
  volatile uint32_t ICPR[8U];               /*!< Offset: 0x180 (R/W)  Interrupt Clear Pending Register */
  uint32_t RESERVED3[24U];
  volatile uint32_t IABR[8U];               /*!< Offset: 0x200 (R/W)  Interrupt Active bit Register */
  uint32_t RESERVED4[56U];
  volatile uint8_t  IP[240U];               /*!< Offset: 0x300 (R/W)  Interrupt Priority Register (8Bit wide) */
  uint32_t RESERVED5[644U];
  volatile  uint32_t STIR;                  /*!< Offset: 0xE00 ( /W)  Software Trigger Interrupt Register */
}  NVIC_Type;	 
	 
#define NVIC                ((NVIC_Type      *)     NVIC_BASE     )   /*!< NVIC configuration struct */

/**
  \brief  Structure type to access the Floating Point Unit (FPU).
 */
typedef struct
{
  uint32_t RESERVED0[1U];
  volatile uint32_t FPCCR;                  /*!< Offset: 0x004 (R/W)  Floating-Point Context Control Register */
  volatile uint32_t FPCAR;                  /*!< Offset: 0x008 (R/W)  Floating-Point Context Address Register */
  volatile uint32_t FPDSCR;                 /*!< Offset: 0x00C (R/W)  Floating-Point Default Status Control Register */
  volatile const  uint32_t MVFR0;                  /*!< Offset: 0x010 (R/ )  Media and FP Feature Register 0 */
  volatile const  uint32_t MVFR1;                  /*!< Offset: 0x014 (R/ )  Media and FP Feature Register 1 */
} FPU_Type;	 
	 
#define FPU                 ((FPU_Type       *)     FPU_BASE      )   /*!< Floating Point Unit */

/**
  \brief  Structure type to access the Memory Protection Unit (MPU).
 */
typedef struct
{
  volatile const  uint32_t TYPE;                   /*!< Offset: 0x000 (R/ )  MPU Type Register */
  volatile uint32_t CTRL;                   /*!< Offset: 0x004 (R/W)  MPU Control Register */
  volatile uint32_t RNR;                    /*!< Offset: 0x008 (R/W)  MPU Region RNRber Register */
  volatile uint32_t RBAR;                   /*!< Offset: 0x00C (R/W)  MPU Region Base Address Register */
  volatile uint32_t RASR;                   /*!< Offset: 0x010 (R/W)  MPU Region Attribute and Size Register */
  volatile uint32_t RBAR_A1;                /*!< Offset: 0x014 (R/W)  MPU Alias 1 Region Base Address Register */
  volatile uint32_t RASR_A1;                /*!< Offset: 0x018 (R/W)  MPU Alias 1 Region Attribute and Size Register */
  volatile uint32_t RBAR_A2;                /*!< Offset: 0x01C (R/W)  MPU Alias 2 Region Base Address Register */
  volatile uint32_t RASR_A2;                /*!< Offset: 0x020 (R/W)  MPU Alias 2 Region Attribute and Size Register */
  volatile uint32_t RBAR_A3;                /*!< Offset: 0x024 (R/W)  MPU Alias 3 Region Base Address Register */
  volatile uint32_t RASR_A3;                /*!< Offset: 0x028 (R/W)  MPU Alias 3 Region Attribute and Size Register */
} MPU_Type;	 
	 
#define MPU                 ((MPU_Type       *)     MPU_BASE      )	 
	 
	 
	 
	 
/** 
    Peripheral_memory_map
    Refer to user manual STM32F411ve.pdf which has more data on memory map and alternate functions
    Page 47 for Alternate function Mapping and page 53 for memory mapping. 	
*/
#define FLASH_BASE            0x08000000UL /*!< FLASH(up to 1 MB) base address in the alias region                         */
#define SRAM1_BASE            0x20000000UL /*!< SRAM1(128 KB) base address in the alias region                             */
#define PERIPH_BASE           0x40000000UL /*!< Peripheral base address in the alias region                                */
#define SRAM1_BB_BASE         0x22000000UL /*!< SRAM1(128 KB) base address in the bit-band region                          */
#define PERIPH_BB_BASE        0x42000000UL /*!< Peripheral base address in the bit-band region                             */
#define BKPSRAM_BB_BASE       0x42480000UL /*!< Backup SRAM(4 KB) base address in the bit-band region                      */
#define FLASH_END             0x0807FFFFUL /*!< FLASH end address                                                          */
#define FLASH_OTP_BASE        0x1FFF7800UL /*!< Base address of : (up to 528 Bytes) embedded FLASH OTP Area                */
#define FLASH_OTP_END         0x1FFF7A0FUL /*!< End address of : (up to 528 Bytes) embedded FLASH OTP Area                 */

/* Legacy defines */
#define SRAM_BASE             SRAM1_BASE
#define SRAM_BB_BASE          SRAM1_BB_BASE

/*!< Peripheral memory map */
#define APB1PERIPH_BASE       0x40000000UL /*Same as PERIPH_BASE but mentioning the address here for clarity*/ 
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000UL)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000UL)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x10000000UL)

/*!< APB1 peripherals */
#define TIM2_BASE             (APB1PERIPH_BASE + 0x0000UL)
#define TIM3_BASE             (APB1PERIPH_BASE + 0x0400UL)
#define TIM4_BASE             (APB1PERIPH_BASE + 0x0800UL)
#define TIM5_BASE             (APB1PERIPH_BASE + 0x0C00UL)
#define RTC_BASE              (APB1PERIPH_BASE + 0x2800UL)
#define WWDG_BASE             (APB1PERIPH_BASE + 0x2C00UL)
#define IWDG_BASE             (APB1PERIPH_BASE + 0x3000UL)
#define I2S2ext_BASE          (APB1PERIPH_BASE + 0x3400UL)
#define SPI2_BASE             (APB1PERIPH_BASE + 0x3800UL)
#define SPI3_BASE             (APB1PERIPH_BASE + 0x3C00UL)
#define I2S3ext_BASE          (APB1PERIPH_BASE + 0x4000UL)
#define USART2_BASE           (APB1PERIPH_BASE + 0x4400UL)
#define I2C1_BASE             (APB1PERIPH_BASE + 0x5400UL)
#define I2C2_BASE             (APB1PERIPH_BASE + 0x5800UL)
#define I2C3_BASE             (APB1PERIPH_BASE + 0x5C00UL)
#define PWR_BASE              (APB1PERIPH_BASE + 0x7000UL)

/*!< APB2 peripherals */
#define TIM1_BASE             (APB2PERIPH_BASE + 0x0000UL)
#define USART1_BASE           (APB2PERIPH_BASE + 0x1000UL)
#define USART6_BASE           (APB2PERIPH_BASE + 0x1400UL)
#define ADC1_BASE             (APB2PERIPH_BASE + 0x2000UL)
#define ADC1_COMMON_BASE      (APB2PERIPH_BASE + 0x2300UL)
/* Legacy define */
#define ADC_BASE               ADC1_COMMON_BASE
#define SDIO_BASE             (APB2PERIPH_BASE + 0x2C00UL)
#define SPI1_BASE             (APB2PERIPH_BASE + 0x3000UL)
#define SPI4_BASE             (APB2PERIPH_BASE + 0x3400UL)
#define SYSCFG_BASE           (APB2PERIPH_BASE + 0x3800UL)
#define EXTI_BASE             (APB2PERIPH_BASE + 0x3C00UL)
#define TIM9_BASE             (APB2PERIPH_BASE + 0x4000UL)
#define TIM10_BASE            (APB2PERIPH_BASE + 0x4400UL)
#define TIM11_BASE            (APB2PERIPH_BASE + 0x4800UL)
#define SPI5_BASE             (APB2PERIPH_BASE + 0x5000UL)

/*!< AHB1 peripherals */
#define GPIOA_BASE            (AHB1PERIPH_BASE + 0x0000UL)
#define GPIOB_BASE            (AHB1PERIPH_BASE + 0x0400UL)
#define GPIOC_BASE            (AHB1PERIPH_BASE + 0x0800UL)
#define GPIOD_BASE            (AHB1PERIPH_BASE + 0x0C00UL)
#define GPIOE_BASE            (AHB1PERIPH_BASE + 0x1000UL)
#define GPIOH_BASE            (AHB1PERIPH_BASE + 0x1C00UL)
#define CRC_BASE              (AHB1PERIPH_BASE + 0x3000UL)
#define RCC_BASE              (AHB1PERIPH_BASE + 0x3800UL)
#define FLASH_R_BASE          (AHB1PERIPH_BASE + 0x3C00UL)
#define DMA1_BASE             (AHB1PERIPH_BASE + 0x6000UL)
#define DMA1_Stream0_BASE     (DMA1_BASE + 0x010UL)
#define DMA1_Stream1_BASE     (DMA1_BASE + 0x028UL)
#define DMA1_Stream2_BASE     (DMA1_BASE + 0x040UL)
#define DMA1_Stream3_BASE     (DMA1_BASE + 0x058UL)
#define DMA1_Stream4_BASE     (DMA1_BASE + 0x070UL)
#define DMA1_Stream5_BASE     (DMA1_BASE + 0x088UL)
#define DMA1_Stream6_BASE     (DMA1_BASE + 0x0A0UL)
#define DMA1_Stream7_BASE     (DMA1_BASE + 0x0B8UL)
#define DMA2_BASE             (AHB1PERIPH_BASE + 0x6400UL)
#define DMA2_Stream0_BASE     (DMA2_BASE + 0x010UL)
#define DMA2_Stream1_BASE     (DMA2_BASE + 0x028UL)
#define DMA2_Stream2_BASE     (DMA2_BASE + 0x040UL)
#define DMA2_Stream3_BASE     (DMA2_BASE + 0x058UL)
#define DMA2_Stream4_BASE     (DMA2_BASE + 0x070UL)
#define DMA2_Stream5_BASE     (DMA2_BASE + 0x088UL)
#define DMA2_Stream6_BASE     (DMA2_BASE + 0x0A0UL)
#define DMA2_Stream7_BASE     (DMA2_BASE + 0x0B8UL)


/*!< Debug MCU registers base address */
#define DBGMCU_BASE           0xE0042000UL
/*!< USB registers base address */
#define USB_OTG_FS_PERIPH_BASE               0x50000000UL

#define USB_OTG_GLOBAL_BASE                  0x000UL
#define USB_OTG_DEVICE_BASE                  0x800UL
#define USB_OTG_IN_ENDPOINT_BASE             0x900UL
#define USB_OTG_OUT_ENDPOINT_BASE            0xB00UL
#define USB_OTG_EP_REG_SIZE                  0x20UL
#define USB_OTG_HOST_BASE                    0x400UL
#define USB_OTG_HOST_PORT_BASE               0x440UL
#define USB_OTG_HOST_CHANNEL_BASE            0x500UL
#define USB_OTG_HOST_CHANNEL_SIZE            0x20UL
#define USB_OTG_PCGCCTL_BASE                 0xE00UL
#define USB_OTG_FIFO_BASE                    0x1000UL
#define USB_OTG_FIFO_SIZE                    0x1000UL

#define UID_BASE                     0x1FFF7A10UL           /*!< Unique device ID register base address */
#define FLASHSIZE_BASE               0x1FFF7A22UL           /*!< FLASH Size register base address       */
#define PACKAGE_BASE                 0x1FFF7BF0UL           /*!< Package size register base address     */
/**
  * @}
  */

	 


/** 
 Analog to Digital Converter  
*/

typedef struct
{
  volatile uint32_t SR;     /*!< ADC status register,                         Address offset: 0x00 */
  volatile uint32_t CR1;    /*!< ADC control register 1,                      Address offset: 0x04 */
  volatile uint32_t CR2;    /*!< ADC control register 2,                      Address offset: 0x08 */
  volatile uint32_t SMPR1;  /*!< ADC sample time register 1,                  Address offset: 0x0C */
  volatile uint32_t SMPR2;  /*!< ADC sample time register 2,                  Address offset: 0x10 */
  volatile uint32_t JOFR1;  /*!< ADC injected channel data offset register 1, Address offset: 0x14 */
  volatile uint32_t JOFR2;  /*!< ADC injected channel data offset register 2, Address offset: 0x18 */
  volatile uint32_t JOFR3;  /*!< ADC injected channel data offset register 3, Address offset: 0x1C */
  volatile uint32_t JOFR4;  /*!< ADC injected channel data offset register 4, Address offset: 0x20 */
  volatile uint32_t HTR;    /*!< ADC watchdog higher threshold register,      Address offset: 0x24 */
  volatile uint32_t LTR;    /*!< ADC watchdog lower threshold register,       Address offset: 0x28 */
  volatile uint32_t SQR1;   /*!< ADC regular sequence register 1,             Address offset: 0x2C */
  volatile uint32_t SQR2;   /*!< ADC regular sequence register 2,             Address offset: 0x30 */
  volatile uint32_t SQR3;   /*!< ADC regular sequence register 3,             Address offset: 0x34 */
  volatile uint32_t JSQR;   /*!< ADC injected sequence register,              Address offset: 0x38*/
  volatile uint32_t JDR1;   /*!< ADC injected data register 1,                Address offset: 0x3C */
  volatile uint32_t JDR2;   /*!< ADC injected data register 2,                Address offset: 0x40 */
  volatile uint32_t JDR3;   /*!< ADC injected data register 3,                Address offset: 0x44 */
  volatile uint32_t JDR4;   /*!< ADC injected data register 4,                Address offset: 0x48 */
  volatile uint32_t DR;     /*!< ADC regular data register,                   Address offset: 0x4C */
} ADC_TypeDef;
	 
#define ADC1                ((ADC_TypeDef *) ADC1_BASE)	 	 
	 
	 
	 
typedef struct
{
  volatile uint32_t CSR;    /*!< ADC Common status register,                  Address offset: ADC1 base address + 0x300 */
  volatile uint32_t CCR;    /*!< ADC common control register,                 Address offset: ADC1 base address + 0x304 */
  volatile uint32_t CDR;    /*!< ADC common regular data register for dual
                             AND triple modes,                            Address offset: ADC1 base address + 0x308 */
} ADC_Common_TypeDef;

#define ADC1_COMMON         ((ADC_Common_TypeDef *) ADC1_COMMON_BASE)	 
#define ADC                  ADC1_COMMON
	 
/** 
  *  CRC calculation unit 
  */

typedef struct
{
  volatile uint32_t DR;         /*!< CRC Data register,             Address offset: 0x00 */
  volatile uint8_t  IDR;        /*!< CRC Independent data register, Address offset: 0x04 */
  uint8_t       RESERVED0;  /*!< Reserved, 0x05                                      */
  uint16_t      RESERVED1;  /*!< Reserved, 0x06                                      */
  volatile uint32_t CR;         /*!< CRC Control register,          Address offset: 0x08 */
} CRC_TypeDef;

#define CRC                 ((CRC_TypeDef *) CRC_BASE)	 
	 
/** 
  *  Debug MCU
  */

typedef struct
{
  volatile uint32_t IDCODE;  /*!< MCU device ID code,               Address offset: 0x00 */
  volatile uint32_t CR;      /*!< Debug MCU configuration register, Address offset: 0x04 */
  volatile uint32_t APB1FZ;  /*!< Debug MCU APB1 freeze register,   Address offset: 0x08 */
  volatile uint32_t APB2FZ;  /*!< Debug MCU APB2 freeze register,   Address offset: 0x0C */
}DBGMCU_TypeDef;

#define DBGMCU              ((DBGMCU_TypeDef *) DBGMCU_BASE)
	 
/** 
  *  DMA Controller
  */

typedef struct
{
  volatile uint32_t CR;     /*!< DMA stream x configuration register      */
  volatile uint32_t NDTR;   /*!< DMA stream x number of data register     */
  volatile uint32_t PAR;    /*!< DMA stream x peripheral address register */
  volatile uint32_t M0AR;   /*!< DMA stream x memory 0 address register   */
  volatile uint32_t M1AR;   /*!< DMA stream x memory 1 address register   */
  volatile uint32_t FCR;    /*!< DMA stream x FIFO control register       */
} DMA_Stream_TypeDef;
	 

#define DMA1_Stream0        ((DMA_Stream_TypeDef *) DMA1_Stream0_BASE)
#define DMA1_Stream1        ((DMA_Stream_TypeDef *) DMA1_Stream1_BASE)
#define DMA1_Stream2        ((DMA_Stream_TypeDef *) DMA1_Stream2_BASE)
#define DMA1_Stream3        ((DMA_Stream_TypeDef *) DMA1_Stream3_BASE)
#define DMA1_Stream4        ((DMA_Stream_TypeDef *) DMA1_Stream4_BASE)
#define DMA1_Stream5        ((DMA_Stream_TypeDef *) DMA1_Stream5_BASE)
#define DMA1_Stream6        ((DMA_Stream_TypeDef *) DMA1_Stream6_BASE)
#define DMA1_Stream7        ((DMA_Stream_TypeDef *) DMA1_Stream7_BASE)

#define DMA2_Stream0        ((DMA_Stream_TypeDef *) DMA2_Stream0_BASE)
#define DMA2_Stream1        ((DMA_Stream_TypeDef *) DMA2_Stream1_BASE)
#define DMA2_Stream2        ((DMA_Stream_TypeDef *) DMA2_Stream2_BASE)
#define DMA2_Stream3        ((DMA_Stream_TypeDef *) DMA2_Stream3_BASE)
#define DMA2_Stream4        ((DMA_Stream_TypeDef *) DMA2_Stream4_BASE)
#define DMA2_Stream5        ((DMA_Stream_TypeDef *) DMA2_Stream5_BASE)
#define DMA2_Stream6        ((DMA_Stream_TypeDef *) DMA2_Stream6_BASE)
#define DMA2_Stream7        ((DMA_Stream_TypeDef *) DMA2_Stream7_BASE)
	 

typedef struct
{
  volatile uint32_t LISR;   /*!< DMA low interrupt status register,      Address offset: 0x00 */
  volatile uint32_t HISR;   /*!< DMA high interrupt status register,     Address offset: 0x04 */
  volatile uint32_t LIFCR;  /*!< DMA low interrupt flag clear register,  Address offset: 0x08 */
  volatile uint32_t HIFCR;  /*!< DMA high interrupt flag clear register, Address offset: 0x0C */
} DMA_TypeDef;

#define DMA1                ((DMA_TypeDef *) DMA1_BASE)
#define DMA2                ((DMA_TypeDef *) DMA2_BASE)	 
	 
	 
/** 
  *  External Interrupt/Event Controller
  */

typedef struct
{
  volatile uint32_t IMR;    /*!< EXTI Interrupt mask register,            Address offset: 0x00 */
  volatile uint32_t EMR;    /*!< EXTI Event mask register,                Address offset: 0x04 */
  volatile uint32_t RTSR;   /*!< EXTI Rising trigger selection register,  Address offset: 0x08 */
  volatile uint32_t FTSR;   /*!< EXTI Falling trigger selection register, Address offset: 0x0C */
  volatile uint32_t SWIER;  /*!< EXTI Software interrupt event register,  Address offset: 0x10 */
  volatile uint32_t PR;     /*!< EXTI Pending register,                   Address offset: 0x14 */
} EXTI_TypeDef;

	 
#define EXTI                ((EXTI_TypeDef *) EXTI_BASE)
	 
	 
/** 
  *  FLASH Registers
  */

typedef struct
{
  volatile uint32_t ACR;      /*!< FLASH access control register,   Address offset: 0x00 */
  volatile uint32_t KEYR;     /*!< FLASH key register,              Address offset: 0x04 */
  volatile uint32_t OPTKEYR;  /*!< FLASH option key register,       Address offset: 0x08 */
  volatile uint32_t SR;       /*!< FLASH status register,           Address offset: 0x0C */
  volatile uint32_t CR;       /*!< FLASH control register,          Address offset: 0x10 */
  volatile uint32_t OPTCR;    /*!< FLASH option control register ,  Address offset: 0x14 */
  volatile uint32_t OPTCR1;   /*!< FLASH option control register 1, Address offset: 0x18 */
} FLASH_TypeDef;
	 
#define FLASH               ((FLASH_TypeDef *) FLASH_R_BASE)	 

/** 
  *  General Purpose I/O
  */

typedef struct
{
  volatile uint32_t MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
  volatile uint32_t OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
  volatile uint32_t OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
  volatile uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
  volatile uint32_t IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
  volatile uint32_t ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */
  volatile uint32_t BSRR;     /*!< GPIO port bit set/reset register,      Address offset: 0x18      */
  volatile uint32_t LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
  volatile uint32_t AFR[2];   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
} GPIO_TypeDef;
	 
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOH               ((GPIO_TypeDef *) GPIOH_BASE)
	 

/** 
  *  System configuration controller
  */

typedef struct
{
  volatile uint32_t MEMRMP;       /*!< SYSCFG memory remap register,                      Address offset: 0x00      */
  volatile uint32_t PMC;          /*!< SYSCFG peripheral mode configuration register,     Address offset: 0x04      */
  volatile uint32_t EXTICR[4];    /*!< SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14 */
  uint32_t      RESERVED[2];  /*!< Reserved, 0x18-0x1C                                                          */
  volatile uint32_t CMPCR;        /*!< SYSCFG Compensation cell control register,         Address offset: 0x20      */
} SYSCFG_TypeDef;
	 
#define SYSCFG              ((SYSCFG_TypeDef *) SYSCFG_BASE)	 

/** 
  *  Inter-integrated Circuit Interface
  */

typedef struct
{
  volatile uint32_t CR1;        /*!< I2C Control register 1,     Address offset: 0x00 */
  volatile uint32_t CR2;        /*!< I2C Control register 2,     Address offset: 0x04 */
  volatile uint32_t OAR1;       /*!< I2C Own address register 1, Address offset: 0x08 */
  volatile uint32_t OAR2;       /*!< I2C Own address register 2, Address offset: 0x0C */
  volatile uint32_t DR;         /*!< I2C Data register,          Address offset: 0x10 */
  volatile uint32_t SR1;        /*!< I2C Status register 1,      Address offset: 0x14 */
  volatile uint32_t SR2;        /*!< I2C Status register 2,      Address offset: 0x18 */
  volatile uint32_t CCR;        /*!< I2C Clock control register, Address offset: 0x1C */
  volatile uint32_t TRISE;      /*!< I2C TRISE register,         Address offset: 0x20 */
  volatile uint32_t FLTR;       /*!< I2C FLTR register,          Address offset: 0x24 */
} I2C_TypeDef;
	 
#define I2C1                ((I2C_TypeDef *) I2C1_BASE)
#define I2C2                ((I2C_TypeDef *) I2C2_BASE)
#define I2C3                ((I2C_TypeDef *) I2C3_BASE)	 

/** 
  *  Independent WATCHDOG
  */

typedef struct
{
  volatile uint32_t KR;   /*!< IWDG Key register,       Address offset: 0x00 */
  volatile uint32_t PR;   /*!< IWDG Prescaler register, Address offset: 0x04 */
  volatile uint32_t RLR;  /*!< IWDG Reload register,    Address offset: 0x08 */
  volatile uint32_t SR;   /*!< IWDG Status register,    Address offset: 0x0C */
} IWDG_TypeDef;

#define IWDG                ((IWDG_TypeDef *) IWDG_BASE)

/** 
  *  Power Control
  */

typedef struct
{
  volatile uint32_t CR;   /*!< PWR power control register,        Address offset: 0x00 */
  volatile uint32_t CSR;  /*!< PWR power control/status register, Address offset: 0x04 */
} PWR_TypeDef;
	 
#define PWR                 ((PWR_TypeDef *) PWR_BASE)	 

/** 
  *  Reset and Clock Control
  */

typedef struct
{
  volatile uint32_t CR;            /*!< RCC clock control register,                                  Address offset: 0x00 */
  volatile uint32_t PLLCFGR;       /*!< RCC PLL configuration register,                              Address offset: 0x04 */
  volatile uint32_t CFGR;          /*!< RCC clock configuration register,                            Address offset: 0x08 */
  volatile uint32_t CIR;           /*!< RCC clock interrupt register,                                Address offset: 0x0C */
  volatile uint32_t AHB1RSTR;      /*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
  volatile uint32_t AHB2RSTR;      /*!< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
  volatile uint32_t AHB3RSTR;      /*!< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                                    */
  volatile uint32_t APB1RSTR;      /*!< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
  volatile uint32_t APB2RSTR;      /*!< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                               */
  volatile uint32_t AHB1ENR;       /*!< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
  volatile uint32_t AHB2ENR;       /*!< RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
  volatile uint32_t AHB3ENR;       /*!< RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                                    */
  volatile uint32_t APB1ENR;       /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
  volatile uint32_t APB2ENR;       /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                               */
  volatile uint32_t AHB1LPENR;     /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
  volatile uint32_t AHB2LPENR;     /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
  volatile uint32_t AHB3LPENR;     /*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                                    */
  volatile uint32_t APB1LPENR;     /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
  volatile uint32_t APB2LPENR;     /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                               */
  volatile uint32_t BDCR;          /*!< RCC Backup domain control register,                          Address offset: 0x70 */
  volatile uint32_t CSR;           /*!< RCC clock control & status register,                         Address offset: 0x74 */
  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                               */
  volatile uint32_t SSCGR;         /*!< RCC spread spectrum clock generation register,               Address offset: 0x80 */
  volatile uint32_t PLLI2SCFGR;    /*!< RCC PLLI2S configuration register,                           Address offset: 0x84 */
  uint32_t      RESERVED7[1];  /*!< Reserved, 0x88                                                                    */
  volatile uint32_t DCKCFGR;       /*!< RCC Dedicated Clocks configuration register,                 Address offset: 0x8C */
} RCC_TypeDef;
	 
#define RCC                 ((RCC_TypeDef *) RCC_BASE)	 

/** 
  *  Real-Time Clock
  */

typedef struct
{
  volatile uint32_t TR;      /*!< RTC time register,                                        Address offset: 0x00 */
  volatile uint32_t DR;      /*!< RTC date register,                                        Address offset: 0x04 */
  volatile uint32_t CR;      /*!< RTC control register,                                     Address offset: 0x08 */
  volatile uint32_t ISR;     /*!< RTC initialization and status register,                   Address offset: 0x0C */
  volatile uint32_t PRER;    /*!< RTC prescaler register,                                   Address offset: 0x10 */
  volatile uint32_t WUTR;    /*!< RTC wakeup timer register,                                Address offset: 0x14 */
  volatile uint32_t CALIBR;  /*!< RTC calibration register,                                 Address offset: 0x18 */
  volatile uint32_t ALRMAR;  /*!< RTC alarm A register,                                     Address offset: 0x1C */
  volatile uint32_t ALRMBR;  /*!< RTC alarm B register,                                     Address offset: 0x20 */
  volatile uint32_t WPR;     /*!< RTC write protection register,                            Address offset: 0x24 */
  volatile uint32_t SSR;     /*!< RTC sub second register,                                  Address offset: 0x28 */
  volatile uint32_t SHIFTR;  /*!< RTC shift control register,                               Address offset: 0x2C */
  volatile uint32_t TSTR;    /*!< RTC time stamp time register,                             Address offset: 0x30 */
  volatile uint32_t TSDR;    /*!< RTC time stamp date register,                             Address offset: 0x34 */
  volatile uint32_t TSSSR;   /*!< RTC time-stamp sub second register,                       Address offset: 0x38 */
  volatile uint32_t CALR;    /*!< RTC calibration register,                                 Address offset: 0x3C */
  volatile uint32_t TAFCR;   /*!< RTC tamper and alternate function configuration register, Address offset: 0x40 */
  volatile uint32_t ALRMASSR;/*!< RTC alarm A sub second register,                          Address offset: 0x44 */
  volatile uint32_t ALRMBSSR;/*!< RTC alarm B sub second register,                          Address offset: 0x48 */
  uint32_t RESERVED7;    /*!< Reserved, 0x4C                                                                 */
  volatile uint32_t BKP0R;   /*!< RTC backup register 1,                                    Address offset: 0x50 */
  volatile uint32_t BKP1R;   /*!< RTC backup register 1,                                    Address offset: 0x54 */
  volatile uint32_t BKP2R;   /*!< RTC backup register 2,                                    Address offset: 0x58 */
  volatile uint32_t BKP3R;   /*!< RTC backup register 3,                                    Address offset: 0x5C */
  volatile uint32_t BKP4R;   /*!< RTC backup register 4,                                    Address offset: 0x60 */
  volatile uint32_t BKP5R;   /*!< RTC backup register 5,                                    Address offset: 0x64 */
  volatile uint32_t BKP6R;   /*!< RTC backup register 6,                                    Address offset: 0x68 */
  volatile uint32_t BKP7R;   /*!< RTC backup register 7,                                    Address offset: 0x6C */
  volatile uint32_t BKP8R;   /*!< RTC backup register 8,                                    Address offset: 0x70 */
  volatile uint32_t BKP9R;   /*!< RTC backup register 9,                                    Address offset: 0x74 */
  volatile uint32_t BKP10R;  /*!< RTC backup register 10,                                   Address offset: 0x78 */
  volatile uint32_t BKP11R;  /*!< RTC backup register 11,                                   Address offset: 0x7C */
  volatile uint32_t BKP12R;  /*!< RTC backup register 12,                                   Address offset: 0x80 */
  volatile uint32_t BKP13R;  /*!< RTC backup register 13,                                   Address offset: 0x84 */
  volatile uint32_t BKP14R;  /*!< RTC backup register 14,                                   Address offset: 0x88 */
  volatile uint32_t BKP15R;  /*!< RTC backup register 15,                                   Address offset: 0x8C */
  volatile uint32_t BKP16R;  /*!< RTC backup register 16,                                   Address offset: 0x90 */
  volatile uint32_t BKP17R;  /*!< RTC backup register 17,                                   Address offset: 0x94 */
  volatile uint32_t BKP18R;  /*!< RTC backup register 18,                                   Address offset: 0x98 */
  volatile uint32_t BKP19R;  /*!< RTC backup register 19,                                   Address offset: 0x9C */
} RTC_TypeDef;
	 
#define RTC                 ((RTC_TypeDef *) RTC_BASE)	 

/** 
  *  SD host Interface
  */

typedef struct
{
  volatile uint32_t POWER;                 /*!< SDIO power control register,    Address offset: 0x00 */
  volatile uint32_t CLKCR;                 /*!< SDI clock control register,     Address offset: 0x04 */
  volatile uint32_t ARG;                   /*!< SDIO argument register,         Address offset: 0x08 */
  volatile uint32_t CMD;                   /*!< SDIO command register,          Address offset: 0x0C */
  volatile const uint32_t  RESPCMD;        /*!< SDIO command response register, Address offset: 0x10 */
  volatile const uint32_t  RESP1;          /*!< SDIO response 1 register,       Address offset: 0x14 */
  volatile const uint32_t  RESP2;          /*!< SDIO response 2 register,       Address offset: 0x18 */
  volatile const uint32_t  RESP3;          /*!< SDIO response 3 register,       Address offset: 0x1C */
  volatile const uint32_t  RESP4;          /*!< SDIO response 4 register,       Address offset: 0x20 */
  volatile uint32_t DTIMER;                /*!< SDIO data timer register,       Address offset: 0x24 */
  volatile uint32_t DLEN;                  /*!< SDIO data length register,      Address offset: 0x28 */
  volatile uint32_t DCTRL;                 /*!< SDIO data control register,     Address offset: 0x2C */
  volatile const uint32_t  DCOUNT;         /*!< SDIO data counter register,     Address offset: 0x30 */
  volatile const uint32_t  STA;            /*!< SDIO status register,           Address offset: 0x34 */
  volatile uint32_t ICR;                   /*!< SDIO interrupt clear register,  Address offset: 0x38 */
  volatile uint32_t MASK;                  /*!< SDIO mask register,             Address offset: 0x3C */
  uint32_t      RESERVED0[2];          /*!< Reserved, 0x40-0x44                                  */
  volatile const uint32_t  FIFOCNT;        /*!< SDIO FIFO counter register,     Address offset: 0x48 */
  uint32_t      RESERVED1[13];         /*!< Reserved, 0x4C-0x7C                                  */
  volatile uint32_t FIFO;                  /*!< SDIO data FIFO register,        Address offset: 0x80 */
} SDIO_TypeDef;
	 
#define SDIO                ((SDIO_TypeDef *) SDIO_BASE)	 

/** 
  *  Serial Peripheral Interface
  */

typedef struct
{
  volatile uint32_t CR1;        /*!< SPI control register 1 (not used in I2S mode),      Address offset: 0x00 */
  volatile uint32_t CR2;        /*!< SPI control register 2,                             Address offset: 0x04 */
  volatile uint32_t SR;         /*!< SPI status register,                                Address offset: 0x08 */
  volatile uint32_t DR;         /*!< SPI data register,                                  Address offset: 0x0C */
  volatile uint32_t CRCPR;      /*!< SPI CRC polynomial register (not used in I2S mode), Address offset: 0x10 */
  volatile uint32_t RXCRCR;     /*!< SPI RX CRC register (not used in I2S mode),         Address offset: 0x14 */
  volatile uint32_t TXCRCR;     /*!< SPI TX CRC register (not used in I2S mode),         Address offset: 0x18 */
  volatile uint32_t I2SCFGR;    /*!< SPI_I2S configuration register,                     Address offset: 0x1C */
  volatile uint32_t I2SPR;      /*!< SPI_I2S prescaler register,                         Address offset: 0x20 */
} SPI_TypeDef;
	 

#define SPI1                ((SPI_TypeDef *) SPI1_BASE)	 
#define SPI2                ((SPI_TypeDef *) SPI2_BASE)
#define SPI3                ((SPI_TypeDef *) SPI3_BASE)
#define SPI4                ((SPI_TypeDef *) SPI4_BASE)
#define SPI5                ((SPI_TypeDef *) SPI5_BASE)	 
#define I2S2ext             ((SPI_TypeDef *) I2S2ext_BASE)	 
#define I2S3ext             ((SPI_TypeDef *) I2S3ext_BASE)	 



/** 
  *  TIM
  */

typedef struct
{
  volatile uint32_t CR1;         /*!< TIM control register 1,              Address offset: 0x00 */
  volatile uint32_t CR2;         /*!< TIM control register 2,              Address offset: 0x04 */
  volatile uint32_t SMCR;        /*!< TIM slave mode control register,     Address offset: 0x08 */
  volatile uint32_t DIER;        /*!< TIM DMA/interrupt enable register,   Address offset: 0x0C */
  volatile uint32_t SR;          /*!< TIM status register,                 Address offset: 0x10 */
  volatile uint32_t EGR;         /*!< TIM event generation register,       Address offset: 0x14 */
  volatile uint32_t CCMR1;       /*!< TIM capture/compare mode register 1, Address offset: 0x18 */
  volatile uint32_t CCMR2;       /*!< TIM capture/compare mode register 2, Address offset: 0x1C */
  volatile uint32_t CCER;        /*!< TIM capture/compare enable register, Address offset: 0x20 */
  volatile uint32_t CNT;         /*!< TIM counter register,                Address offset: 0x24 */
  volatile uint32_t PSC;         /*!< TIM prescaler,                       Address offset: 0x28 */
  volatile uint32_t ARR;         /*!< TIM auto-reload register,            Address offset: 0x2C */
  volatile uint32_t RCR;         /*!< TIM repetition counter register,     Address offset: 0x30 */
  volatile uint32_t CCR1;        /*!< TIM capture/compare register 1,      Address offset: 0x34 */
  volatile uint32_t CCR2;        /*!< TIM capture/compare register 2,      Address offset: 0x38 */
  volatile uint32_t CCR3;        /*!< TIM capture/compare register 3,      Address offset: 0x3C */
  volatile uint32_t CCR4;        /*!< TIM capture/compare register 4,      Address offset: 0x40 */
  volatile uint32_t BDTR;        /*!< TIM break and dead-time register,    Address offset: 0x44 */
  volatile uint32_t DCR;         /*!< TIM DMA control register,            Address offset: 0x48 */
  volatile uint32_t DMAR;        /*!< TIM DMA address for full transfer,   Address offset: 0x4C */
  volatile uint32_t OR;          /*!< TIM option register,                 Address offset: 0x50 */
} TIM_TypeDef;

#define TIM1                ((TIM_TypeDef *) TIM1_BASE)	 	 
#define TIM2                ((TIM_TypeDef *) TIM2_BASE)
#define TIM3                ((TIM_TypeDef *) TIM3_BASE)
#define TIM4                ((TIM_TypeDef *) TIM4_BASE)
#define TIM5                ((TIM_TypeDef *) TIM5_BASE)
#define TIM9                ((TIM_TypeDef *) TIM9_BASE)
#define TIM10               ((TIM_TypeDef *) TIM10_BASE)
#define TIM11               ((TIM_TypeDef *) TIM11_BASE)

/** 
  *  Universal Synchronous Asynchronous Receiver Transmitter
  */
 
typedef struct
{
  volatile uint32_t SR;         /*!< USART Status register,                   Address offset: 0x00 */
  volatile uint32_t DR;         /*!< USART Data register,                     Address offset: 0x04 */
  volatile uint32_t BRR;        /*!< USART Baud rate register,                Address offset: 0x08 */
  volatile uint32_t CR1;        /*!< USART Control register 1,                Address offset: 0x0C */
  volatile uint32_t CR2;        /*!< USART Control register 2,                Address offset: 0x10 */
  volatile uint32_t CR3;        /*!< USART Control register 3,                Address offset: 0x14 */
  volatile uint32_t GTPR;       /*!< USART Guard time and prescaler register, Address offset: 0x18 */
} USART_TypeDef;
	 

#define USART1              ((USART_TypeDef *) USART1_BASE)
#define USART2              ((USART_TypeDef *) USART2_BASE)	 
#define USART6              ((USART_TypeDef *) USART6_BASE)	 
	 

/** 
  *  Window WATCHDOG
  */

typedef struct
{
  volatile uint32_t CR;   /*!< WWDG Control register,       Address offset: 0x00 */
  volatile uint32_t CFR;  /*!< WWDG Configuration register, Address offset: 0x04 */
  volatile uint32_t SR;   /*!< WWDG Status register,        Address offset: 0x08 */
} WWDG_TypeDef;
	 
#define WWDG                ((WWDG_TypeDef *) WWDG_BASE)
	 
/** 
  *  USB_OTG_Core_Registers
  */
typedef struct
{
  volatile uint32_t GOTGCTL;              /*!< USB_OTG Control and Status Register          000h */
  volatile uint32_t GOTGINT;              /*!< USB_OTG Interrupt Register                   004h */
  volatile uint32_t GAHBCFG;              /*!< Core AHB Configuration Register              008h */
  volatile uint32_t GUSBCFG;              /*!< Core USB Configuration Register              00Ch */
  volatile uint32_t GRSTCTL;              /*!< Core Reset Register                          010h */
  volatile uint32_t GINTSTS;              /*!< Core Interrupt Register                      014h */
  volatile uint32_t GINTMSK;              /*!< Core Interrupt Mask Register                 018h */
  volatile uint32_t GRXSTSR;              /*!< Receive Sts Q Read Register                  01Ch */
  volatile uint32_t GRXSTSP;              /*!< Receive Sts Q Read & POP Register            020h */
  volatile uint32_t GRXFSIZ;              /*!< Receive FIFO Size Register                   024h */
  volatile uint32_t DIEPTXF0_HNPTXFSIZ;   /*!< EP0 / Non Periodic Tx FIFO Size Register     028h */
  volatile uint32_t HNPTXSTS;             /*!< Non Periodic Tx FIFO/Queue Sts reg           02Ch */
  uint32_t Reserved30[2];             /*!< Reserved                                     030h */
  volatile uint32_t GCCFG;                /*!< General Purpose IO Register                  038h */
  volatile uint32_t CID;                  /*!< User ID Register                             03Ch */
  uint32_t  Reserved40[48];           /*!< Reserved                                0x40-0xFF */
  volatile uint32_t HPTXFSIZ;             /*!< Host Periodic Tx FIFO Size Reg               100h */
  volatile uint32_t DIEPTXF[0x0F];        /*!< dev Periodic Transmit FIFO                        */
} USB_OTG_GlobalTypeDef;
	 
#define USB_CORE          ((USB_OTG_GlobalTypeDef *) USB_OTG_FS_PERIPH_BASE)	 

/** 
  *  USB_OTG_device_Registers
  */
typedef struct 
{
  volatile uint32_t DCFG;            /*!< dev Configuration Register   800h */
  volatile uint32_t DCTL;            /*!< dev Control Register         804h */
  volatile uint32_t DSTS;            /*!< dev Status Register (RO)     808h */
  uint32_t Reserved0C;           /*!< Reserved                     80Ch */
  volatile uint32_t DIEPMSK;         /*!< dev IN Endpoint Mask         810h */
  volatile uint32_t DOEPMSK;         /*!< dev OUT Endpoint Mask        814h */
  volatile uint32_t DAINT;           /*!< dev All Endpoints Itr Reg    818h */
  volatile uint32_t DAINTMSK;        /*!< dev All Endpoints Itr Mask   81Ch */
  uint32_t  Reserved20;          /*!< Reserved                     820h */
  uint32_t Reserved9;            /*!< Reserved                     824h */
  volatile uint32_t DVBUSDIS;        /*!< dev VBUS discharge Register  828h */
  volatile uint32_t DVBUSPULSE;      /*!< dev VBUS Pulse Register      82Ch */
  volatile uint32_t DTHRCTL;         /*!< dev threshold                830h */
  volatile uint32_t DIEPEMPMSK;      /*!< dev empty msk                834h */
  volatile uint32_t DEACHINT;        /*!< dedicated EP interrupt       838h */
  volatile uint32_t DEACHMSK;        /*!< dedicated EP msk             83Ch */
  uint32_t Reserved40;           /*!< dedicated EP mask            840h */
  volatile uint32_t DINEP1MSK;       /*!< dedicated EP mask            844h */
  uint32_t  Reserved44[15];      /*!< Reserved                 844-87Ch */
  volatile uint32_t DOUTEP1MSK;      /*!< dedicated EP msk             884h */
} USB_OTG_DeviceTypeDef;
	 
#define USB_DEVICE          ((USB_OTG_DeviceTypeDef *) USB_OTG_FS_PERIPH_BASE + USB_OTG_DEVICE_BASE)	 
	 

/** 
  *  USB_OTG_IN_Endpoint-Specific_Register
  */
typedef struct 
{
  volatile uint32_t DIEPCTL;           /*!< dev IN Endpoint Control Reg    900h + (ep_num * 20h) + 00h */
  uint32_t Reserved04;             /*!< Reserved                       900h + (ep_num * 20h) + 04h */
  volatile uint32_t DIEPINT;           /*!< dev IN Endpoint Itr Reg        900h + (ep_num * 20h) + 08h */
  uint32_t Reserved0C;             /*!< Reserved                       900h + (ep_num * 20h) + 0Ch */
  volatile uint32_t DIEPTSIZ;          /*!< IN Endpoint Txfer Size         900h + (ep_num * 20h) + 10h */
  volatile uint32_t DIEPDMA;           /*!< IN Endpoint DMA Address Reg    900h + (ep_num * 20h) + 14h */
  volatile uint32_t DTXFSTS;           /*!< IN Endpoint Tx FIFO Status Reg 900h + (ep_num * 20h) + 18h */
  uint32_t Reserved18;             /*!< Reserved  900h+(ep_num*20h)+1Ch-900h+ (ep_num * 20h) + 1Ch */
} USB_OTG_INEndpointTypeDef;

//The IN endpoints are indexed here by ofsetting the EP by 0x20 to reach the next bank of registers	 
#define USB_EP_IN(x)      ((USB_OTG_INEndpointTypeDef *) USB_OTG_FS_PERIPH_BASE + USB_OTG_IN_ENDPOINT_BASE + (x * 0x20))	     	 
	 
	 

/** 
  *  USB_OTG_OUT_Endpoint-Specific_Registers
  */
typedef struct 
{
  volatile uint32_t DOEPCTL;       /*!< dev OUT Endpoint Control Reg           B00h + (ep_num * 20h) + 00h */
  uint32_t Reserved04;         /*!< Reserved                               B00h + (ep_num * 20h) + 04h */
  volatile uint32_t DOEPINT;       /*!< dev OUT Endpoint Itr Reg               B00h + (ep_num * 20h) + 08h */
  uint32_t Reserved0C;         /*!< Reserved                               B00h + (ep_num * 20h) + 0Ch */
  volatile uint32_t DOEPTSIZ;      /*!< dev OUT Endpoint Txfer Size            B00h + (ep_num * 20h) + 10h */
  volatile uint32_t DOEPDMA;       /*!< dev OUT Endpoint DMA Address           B00h + (ep_num * 20h) + 14h */
  uint32_t Reserved18[2];      /*!< Reserved B00h + (ep_num * 20h) + 18h - B00h + (ep_num * 20h) + 1Ch */
} USB_OTG_OUTEndpointTypeDef;

//The OUT endpoints are indexed here by ofsetting the EP by 0x20 to reach the next bank of registers	 
#define USB_EP_OUT(x)      ((USB_OTG_OUTEndpointTypeDef *) USB_OTG_FS_PERIPH_BASE + USB_OTG_OUT_ENDPOINT_BASE + (x * 0x20))	 

/** 
  *  USB_OTG_Host_Mode_Register_Structures
  */
typedef struct 
{
  volatile uint32_t HCFG;             /*!< Host Configuration Register          400h */
  volatile uint32_t HFIR;             /*!< Host Frame Interval Register         404h */
  volatile uint32_t HFNUM;            /*!< Host Frame Nbr/Frame Remaining       408h */
  uint32_t Reserved40C;           /*!< Reserved                             40Ch */
  volatile uint32_t HPTXSTS;          /*!< Host Periodic Tx FIFO/ Queue Status  410h */
  volatile uint32_t HAINT;            /*!< Host All Channels Interrupt Register 414h */
  volatile uint32_t HAINTMSK;         /*!< Host All Channels Interrupt Mask     418h */
} USB_OTG_HostTypeDef;
	 
#define USB_HOST             ((USB_OTG_HostTypeDef *) USB_OTG_FS_PERIPH_BASE + USB_OTG_HOST_BASE) 

/** 
  *  USB_OTG_Host_Channel_Specific_Registers
  */
typedef struct
{
  volatile uint32_t HCCHAR;           /*!< Host Channel Characteristics Register    500h */
  volatile uint32_t HCSPLT;           /*!< Host Channel Split Control Register      504h */
  volatile uint32_t HCINT;            /*!< Host Channel Interrupt Register          508h */
  volatile uint32_t HCINTMSK;         /*!< Host Channel Interrupt Mask Register     50Ch */
  volatile uint32_t HCTSIZ;           /*!< Host Channel Transfer Size Register      510h */
  volatile uint32_t HCDMA;            /*!< Host Channel DMA Address Register        514h */
  uint32_t Reserved[2];           /*!< Reserved                                      */
} USB_OTG_HostChannelTypeDef;
	 
#define USB_HOST_CHANNEL             ((USB_OTG_HostChannelTypeDef *)USB_OTG_FS_PERIPH_BASE + USB_OTG_HOST_CHANNEL_BASE) 
#define USB_PCGCCTL                 *((volatile uint32_t *) USB_OTG_FS_PERIPH_BASE +  USB_OTG_PCGCCTL_BASE)
#define USB_FIFO	                *((volatile uint32_t *) USB_OTG_FS_PERIPH_BASE +  USB_OTG_FIFO_BASE)


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* __MEMORY_MAP_STM32F411xE_H */



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

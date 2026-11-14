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


#define HIL_PACKET_SIZE 12

/* Buffers strictly aligned to 4-byte boundaries for the 32-bit DMA Bus Master */
extern uint8_t rx_dma_buffer[HIL_PACKET_SIZE] __attribute__((aligned(4)));
extern uint8_t tx_dma_buffer[HIL_PACKET_SIZE] __attribute__((aligned(4)));
#define OLED_I2C_ADDR_WRITE  0x78    /* 0x3C shifted left by 1 (Write address) */
#define OLED_TOTAL_DMA_SIZE  1025    /* 1 control byte (0x40) + 1024 pixel bytes */
/* 1025-byte framebuffer aligned to 4-byte boundary for 32-bit DMA bus master */
extern uint8_t oled_buffer[OLED_TOTAL_DMA_SIZE] __attribute__((aligned(4)));

#define HIL_HOST_SYNC_BYTE0    0xAA
#define HIL_HOST_SYNC_BYTE1    0x55

#define HIL_MCU_SYNC_BYTE0     0x55
#define HIL_MCU_SYNC_BYTE1     0xAA

#define HIL_PACKET_SIZE        12

/* Host Commands */
#define HIL_CMD_STOP           0x00
#define HIL_CMD_RUN            0x01
#define HIL_CMD_RESET          0x02

/* MCU Status Flags */
#define HIL_STATUS_RUNNING     (1 << 0)  /* PA0 Button State (1=Run, 0=Stop) */
#define HIL_STATUS_TIMEOUT     (1 << 1)  /* Watchdog tripped (1=Failsafe active) */

#pragma pack(push, 1)

/*
 * =========================================================================
 * 1. HOST -> MCU PACKET (Received via USB CDC)
 * =========================================================================
 * Total Size: Exactly 12 Bytes (Zero Padding)
 *
 * Byte Offsets:
 * [0..1]  sync[2]   (2 bytes) -> 0xAA, 0x55
 * [2..5]  pos1      (4 bytes) -> float (Primary angle/position, e.g., rad)
 * [6..9]  pos2      (4 bytes) -> float (Secondary angle/position, 0.0f if unused)
 * [10]    host_cmd  (1 byte)  -> 0x01 = RUN, 0x00 = STOP
 * [11]    checksum  (1 byte)  -> XOR of bytes 0..10
 * =========================================================================
 */
typedef struct __attribute__((packed)) {
    uint8_t  sync[2];       /* Offset 0..1  : Frame synchronization magic */
    float    pos1;          /* Offset 2..5  : Primary plant position/angle */
    float    pos2;          /* Offset 6..9  : Secondary plant position/angle */
    uint8_t  host_cmd;      /* Offset 10    : Run/Stop command */
    uint8_t  checksum;      /* Offset 11    : XOR of bytes 0..10 */
} HostToMcuPacket;

/* Compile-time verification: Build halts if compiler inserts padding */
_Static_assert(sizeof(HostToMcuPacket) == 12, "ERROR: HostToMcuPacket is not 12 bytes!");


/*
 * =========================================================================
 * 2. MCU -> HOST PACKET (Transmitted via USB CDC)
 * =========================================================================
 * Total Size: Exactly 12 Bytes (Zero Padding)
 *
 * Byte Offsets:
 * [0..1]  sync[2]     (2 bytes) -> 0x55, 0xAA
 * [2..5]  output1     (4 bytes) -> float (Primary control effort: e.g. signed duty -1.0 to +1.0)
 * [6..9]  output2     (4 bytes) -> float (Secondary control effort, or telemetry/gain value)
 * [10]    mcu_status  (1 byte)  -> Bit 0: PA0 Run/Stop state, Bit 1: Watchdog
 * [11]    checksum    (1 byte)  -> XOR of bytes 0..10
 * =========================================================================
 */
typedef struct __attribute__((packed)) {
    uint8_t  sync[2];       /* Offset 0..1  : Frame synchronization magic */
    float    output1;       /* Offset 2..5  : Primary control effort */
    float    output2;       /* Offset 6..9  : Secondary output / live telemetry */
    uint8_t  mcu_status;    /* Offset 10    : Status flags */
    uint8_t  checksum;      /* Offset 11    : XOR of bytes 0..10 */
} McuToHostPacket;

/* Compile-time verification: Build halts if compiler inserts padding */
_Static_assert(sizeof(McuToHostPacket) == 12, "ERROR: McuToHostPacket is not 12 bytes!");

#pragma pack(pop)

/* Shared between SysTick and DMA ISR - must be volatile */
extern volatile int32_t  hil_watchdog_counter;
extern volatile uint8_t  compensator_enabled;  // 1 = Active, 0 = Powered Down
extern volatile uint8_t  compensator_synced;  // Prevents velocity spike on resume

#endif
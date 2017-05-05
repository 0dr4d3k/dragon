/*******************************************************************************
  Filename:       hal_board_cfg_common.h
  Revised:        $Date: 2017-02-23 11:32:02 $
  Revision:       $Revision: 00001 $

  Description:    Board configuration for CC2541 Dragonfly. 
*******************************************************************************/

#ifndef HAL_BOARD_CFG_COMMON_H
#define HAL_BOARD_CFG_COMMON_H

#define ACTIVE_LOW        !
#define ACTIVE_HIGH       !!    /* double negation forces result to be '1' */

#ifdef __cplusplus
extern "C"
{
#endif

  
/* -----------------------------------------------------------------------------
 *                                   Includes
 * -----------------------------------------------------------------------------
 */
#include "hal_board.h"
#include "hal_mcu.h"
#include "hal_defs.h"
#include "hal_types.h"

  
/* -----------------------------------------------------------------------------
 *                              Common Definitions
 * -----------------------------------------------------------------------------
 */
/// role configuration /////////////////////////////////////////////////////////  
/* GAP Roles */
#define BROADCASTER_CFG  0x01
#define OBSERVER_CFG     0x02
#define PERIPHERAL_CFG   0x04
#define CENTRAL_CFG      0x08
  
/* BLE Controller Build Components */
#define ADV_NCONN_CFG    0x01
#define ADV_CONN_CFG     0x02
#define SCAN_CFG         0x04
#define INIT_CFG         0x08
  
  
/// clock speed ////////////////////////////////////////////////////////////////
/* Clock Speed */
#define HAL_CPU_CLOCK_MHZ     32

/* Sleep Clock */
#define EXTERNAL_CRYSTAL_OSC          0x00  // external 32kHz XOSC
#define INTERNAL_RC_OSC               0x80  // internal 32kHz RCOSC

/*
 * If not using power management, assume that the 32kHz crystal is not
 * installed. Even if a 32kHz crystal is present on the board, it will
 * never get used since device does not ever go to sleep. By forcing
 * OSC32K_CRYSTAL_INSTALLED to FALSE, we avoid mismatches between
 * libraries built with power management off, and applications in which
 * power management is not used.
 */
#if ( !defined ( POWER_SAVING ) ) && ( !defined ( OSC32K_CRYSTAL_INSTALLED ) )
  #define OSC32K_CRYSTAL_INSTALLED FALSE
#endif

/* 32 kHz clock source select in CLKCONCMD */
#if !defined (OSC32K_CRYSTAL_INSTALLED) || \
    (defined (OSC32K_CRYSTAL_INSTALLED) && (OSC32K_CRYSTAL_INSTALLED == TRUE))
  #define OSC_32KHZ  EXTERNAL_CRYSTAL_OSC /* external 32 KHz xosc */
#else
  #define OSC_32KHZ  INTERNAL_RC_OSC      /* internal 32 KHz rcosc */
#endif

// Minimum Time for Stable External 32kHz Clock (in ms)
#define MIN_TIME_TO_STABLE_32KHZ_XOSC 400


/// OSAL NV implemented by internal flash pages ////////////////////////////////
      
// Flash is partitioned into 8 banks of 16 pages of 2 KB. 32K by bank. 
#define HAL_FLASH_PAGE_PER_BANK    16
// Flash is constructed of 128 pages of 2 KB.
#define HAL_FLASH_PAGE_PHYS        2048
// SNV can use a larger logical page size to accomodate more or bigger items or 
// extend lifetime.
#define HAL_FLASH_PAGE_SIZE        HAL_FLASH_PAGE_PHYS
#define HAL_FLASH_WORD_SIZE        4

// CODE banks get mapped into the XDATA range 8000-FFFF.
#define HAL_FLASH_PAGE_MAP         0x8000

// The last 16 bytes of the last available page are reserved for flash lock bits.
// NV page definitions must coincide with segment declaration in project *.xcl file.
#if defined NON_BANKED
#define HAL_FLASH_LOCK_BITS        16
#define HAL_NV_PAGE_END            30
#else
#define HAL_FLASH_LOCK_BITS        16
#define HAL_NV_PAGE_END            126
#endif

// Re-defining Z_EXTADDR_LEN here so as not to include a Z-Stack .h file.
#define HAL_FLASH_IEEE_SIZE        8
#define HAL_FLASH_IEEE_PAGE       (HAL_NV_PAGE_END+1)
#define HAL_FLASH_IEEE_OSET       (HAL_FLASH_PAGE_SIZE - HAL_FLASH_LOCK_BITS - \
                                   HAL_FLASH_IEEE_SIZE)
#define HAL_INFOP_IEEE_OSET        0xC

#define HAL_NV_PAGE_CNT            2
#define HAL_NV_PAGE_BEG           (HAL_NV_PAGE_END-HAL_NV_PAGE_CNT+1)

// Used by DMA macros to shift 1 to create a mask for DMA registers.
#define HAL_NV_DMA_CH              0
#define HAL_DMA_CH_RX              3
#define HAL_DMA_CH_TX              4

#define HAL_NV_DMA_GET_DESC()      HAL_DMA_GET_DESC0()
#define HAL_NV_DMA_SET_ADDR(a)     HAL_DMA_SET_ADDR_DESC0((a))

/* Critical Vdd Monitoring to prevent flash damage or radio lockup. */
// Vdd/3 / Internal Reference X ENOB --> (Vdd / 3) / 1.15 X 127
#define VDD_2_0  74   // 2.0 V required to safely read/write internal flash.
#define VDD_2_7  100  // 2.7 V required for the Numonyx device.

#define VDD_MIN_RUN   VDD_2_0
#define VDD_MIN_NV   (VDD_2_0+4)  // 5% margin over minimum to survive a page erase and compaction.
#define VDD_MIN_XNV  (VDD_2_7+5)  // 5% margin over minimum to survive a page erase and compaction.
      

/* -----------------------------------------------------------------------------
 *                   Common Pin Definitions for all Dragonflys
 * -----------------------------------------------------------------------------
 */
/// interruptions definitions flags ////////////////////////////////////////////
//                                                    |DfG DfA DfP DfI DfT DfV |
#define HAL_EVNT1_ACC  0x0001 //                      | x   x   x   x   x   x  |
#define HAL_EVNT2_QI   0x0002 //                      |     x   x   x   x   x  |
#define HAL_EVNT3_PIR  0x0004 //                      |                 x      |
#define HAL_EVNT4_LUX  0x0008 //                      |                 x   x  |
#define HAL_EVNT5_AIR  0x0010 //                      |             x          |
#define HAL_EVNT5_UVI  0x0020 //                      |                     x  |
               
//                                                    |DfG DfA DfP DfI DfT DfV |
#define HAL_INT1 0x01  // INT1->P1_7->INT_1 Acc       | x   x   x   x   x   x  |
#define HAL_INT2 0x02  // INT2->P1_6->INT_2 Lux/Air   |             x   x      | 
#define HAL_INT3 0x04  // INT3->P1_3->INT_3 PIR       |                 x      |
#define HAL_INT4 0x08  // INT4->P0_6->INT_4 Qi        | x   x   x   x   x   x  |
#define HAL_INT5 0x10  // INT5->P1_0->INT_5 UVi       |                     x  |
#define HAL_INT6 0x20  // FREE
#define HAL_INT7 0x40  // FREE
#define HAL_INT8 0x80  // FREE

/// interruptions (Push Button Configuration) //////////////////////////////////

/*----------------------------- Common definitions ---------------------------*/      
/* Interrupt option - Enable or disable */
#define HAL_KEY_INTERRUPT_DISABLE    0x00
#define HAL_KEY_INTERRUPT_ENABLE     0x01

/* Interrupt option - ISR service attended */
#define HAL_KEY_ISR0     0x00
#define HAL_KEY_ISR1     0x01

/* Key state - shift or nornal */
#define HAL_KEY_STATE_NORMAL          0x00
#define HAL_KEY_STATE_SHIFT           0x01

/* Switches (keys) */
#define HAL_KEY_SW_1          0x01  // S1->P1_7->INT_1 Accelerometer
#define HAL_KEY_SW_2          0x02  // S2->P1_6->INT_2 Luxometer/Air
#define HAL_KEY_SW_3          0x04  // S3->P1_3->INT_3 PIR
#define HAL_KEY_SW_4          0x08  // S4->P0_6->INT_4 Qi Charger
#define HAL_KEY_SW_5          0x10  // S5->P1_0->INT_5 UVi
      
#define HAL_KEY_DEBOUNCE_VALUE  25 // Debounce Value

/*------------------------ I/O Port configuration ----------------------------*/
/* S1->INT_1 Accelerometer is at P1.7 falling edge*/
#define HAL_KEY_SW_1_PORT     P1
#define HAL_KEY_SW_1_BIT      BV(7)
#define HAL_KEY_SW_1_SEL      P1SEL
#define HAL_KEY_SW_1_DIR      P1DIR
   
/* S2->INT_2 Luxometer/Air is at P1.6 falling edge*/
#define HAL_KEY_SW_2_PORT     P1
#define HAL_KEY_SW_2_BIT      BV(6)
#define HAL_KEY_SW_2_SEL      P1SEL
#define HAL_KEY_SW_2_DIR      P1DIR

/* S3->INT_3 PIR is at P1.2 rising edge */
#define HAL_KEY_SW_3_PORT     P1
#define HAL_KEY_SW_3_BIT      BV(3)
#define HAL_KEY_SW_3_SEL      P1SEL
#define HAL_KEY_SW_3_DIR      P1DIR

/* S4->INT_4 Qi Charger is at P0.6 rising edge */
#define HAL_KEY_SW_4_PORT     P0
#define HAL_KEY_SW_4_BIT      BV(6)
#define HAL_KEY_SW_4_SEL      P0SEL
#define HAL_KEY_SW_4_DIR      P0DIR

/* S5->INT_5 UVi is at P1.0 rising edge */
#define HAL_KEY_SW_5_PORT     P1
#define HAL_KEY_SW_5_BIT      BV(0)
#define HAL_KEY_SW_5_SEL      P1SEL
#define HAL_KEY_SW_5_DIR      P0DIR

/*----------------------- Interruption configuration -------------------------*/
/* Interrupt pending in whole port */
#define HAL_KEY_CPU_PORT_0_IF P0IF /* MASK BV(5) in IRCON(0xC0) interrupt pending in port 0 */
#define HAL_KEY_CPU_PORT_1_IF P1IF /* MASK BV(3) in IRCON2(0xE8) interrupt pending in port 1 */
#define HAL_KEY_CPU_PORT_2_IF P2IF /* MASK BV(0) in IRCON2(0xE8) interrupt pending in port 2 */

/* Port Interrupt edge configuration */
#define HAL_QI_INT_PICTL    PICTL /* REGISTER (0x8C) port edge configuration */
      
/* S1->INT_1 Accelerometer is at P1.7 falling edge*/
#define HAL_KEY_SW_1_IEN      IEN2  /* REGISTER (0x9A) interrupt enable */
#define HAL_KEY_SW_1_IENBIT   BV(4) /* MASK in IEN2 for enable all Port_1 interrupts */
#define HAL_KEY_SW_1_ICTL     P1IEN /* REGISTER (0x8D) enable interrupts in specific Port_1 pin */
#define HAL_KEY_SW_1_ICTLBIT  BV(7) /* MASK in P1IEN - P1.7 enable/disable bit */
#define HAL_KEY_SW_1_PXIFG    P1IFG /* REGISTER (0x8A) Interrupt flag in specific Port_1 pin*/
#define HAL_KEY_SW_1_EDGEBIT  BV(2) /* MASK in PICTL - Port 1, inputs 7 to 4 interrupt 0/(1) rising/(falling) edge configuration */

/* S2->INT_2 Luxometer/Air is at P1.6 falling edge*/
#define HAL_KEY_SW_2_IEN      IEN2  /* REGISTER (0x9A) interrupt enable */
#define HAL_KEY_SW_2_IENBIT   BV(4) /* MASK in IEN2 for enable all Port_1 interrupts */
#define HAL_KEY_SW_2_ICTL     P1IEN /* REGISTER (0x8D) enable interrupts in specific Port_1 pin */
#define HAL_KEY_SW_2_ICTLBIT  BV(6) /* MASK in P1IEN - P1.6 enable/disable bit */
#define HAL_KEY_SW_2_PXIFG    P1IFG /* REGISTER (0x8A) Interrupt flag in specific Port_1 pin */
#define HAL_KEY_SW_2_EDGEBIT  BV(2) /* MASK in PICTL - Port 1, inputs 7 to 4 interrupt 0/(1) rising/(falling) edge configuration */

/* S3->INT_3 PIR is at P1.3 rising edge */
#define HAL_KEY_SW_3_IEN      IEN2  /* REGISTER (0x9A) interrupt enable */
#define HAL_KEY_SW_3_IENBIT   BV(4) /* MASK in IEN2 for enable all Port_1 interrupts */
#define HAL_KEY_SW_3_ICTL     P1IEN /* REGISTER (0x8D) enable interrupts in specific Port_1 pin */
#define HAL_KEY_SW_3_ICTLBIT  BV(3) /* MASK in P1IEN - P1.3 enable/disable bit */
#define HAL_KEY_SW_3_PXIFG    P1IFG /* REGISTER (0x8A) Interrupt flag in specific Port_1 pin */
#define HAL_KEY_SW_3_EDGEBIT  BV(1) /* MASK in PICTL - Port 1, inputs 3 to 0 interrupt 0/(1) rising/(falling) edge configuration */

/* S4->INT_4 Qi Charger is at P0.6 falling edge */
#define HAL_KEY_SW_4_IEN      IEN1  /* REGISTER (0xB8) interrupt enable */
#define HAL_KEY_SW_4_IENBIT   BV(5) /* MASK in IEN1 for enable all Port_0 interrupts */
#define HAL_KEY_SW_4_ICTL     P0IEN /* REGISTER (0xAB) enable interrupts in specific Port_0 pin */
#define HAL_KEY_SW_4_ICTLBIT  BV(6) /* MASK in P0IEN - P0.6 enable/disable bit */
#define HAL_KEY_SW_4_PXIFG    P0IFG /* REGISTER (0x89) Interrupt flag in specific Port_0 pin */
#define HAL_KEY_SW_4_EDGEBIT  BV(0) /* MASK in PICTL - Port 0, inputs 7 to 0 interrupt 0/(1) rising/(falling) edge configuration */

/* S5->INT_5 UVi is at P1.0 rising edge */
#define HAL_KEY_SW_5_IEN      IEN2  /* REGISTER (0x9A) interrupt enable */
#define HAL_KEY_SW_5_IENBIT   BV(4) /* MASK in IEN2 for enable all Port_1 interrupts */
#define HAL_KEY_SW_5_ICTL     P1IEN /* REGISTER (0x8D) enable interrupts in specific Port_1 pin */
#define HAL_KEY_SW_5_ICTLBIT  BV(0) /* MASK in P1IEN - P1.0 enable/disable bit */
#define HAL_KEY_SW_5_PXIFG    P1IFG /* REGISTER (0x8A) Interrupt flag in specific Port_1 pin */
#define HAL_KEY_SW_5_EDGEBIT  BV(1) /* MASK in PICTL - Port 1, inputs 3 to 0 interrupt 0/(1) rising/(falling) edge configuration */

/// buzzer configuration ///////////////////////////////////////////////////////


/// leds configuration /////////////////////////////////////////////////////////

#define HAL_NUM_LEDS                 3

/* 1 - Green. (P0_4-OUTPUT)*/
#define LED1_BV                      BV(4)
#define LED1_SBIT                    P0_4
#define LED1_DDR                     P0DIR
#define LED1_POLARITY                ACTIVE_LOW

/* 2 - Red. (P0_3-OUTPUT) */
#define LED2_BV                      BV(3)
#define LED2_SBIT                    P0_3
#define LED2_DDR                     P0DIR
#define LED2_POLARITY                ACTIVE_LOW

/* 3 - Blue. (P0_2-OUTPUT)*/
#define LED3_BV                      BV(2)
#define LED3_SBIT                    P0_2
#define LED3_DDR                     P0DIR
#define LED3_POLARITY                ACTIVE_LOW

/// macros - delays ////////////////////////////////////////////////////////////
/* Statementt ative delay: 125 cycles ~1 msec */
#define ST_HAL_DELAY(n)       st({volatile uint32 i; for (i=0; i<(n); i++) {};})

#define HAL_LED_BLINK_DELAY() ST_HAL_DELAY(0x5800)


/// charger Qi configuration ///////////////////////////////////////////////////

/* --- P0_6-CRGn signal-Input-Pull-UP-Interrupt falling edge -----------------*/
/* PORT: Qi interrupt. (P0_6-INPUT PULL-UP)*/
#define HAL_QI_INT_BIT        BV(6)
#define HAL_QI_INT_PORT       P0
#define HAL_QI_INT_DIR        P0DIR
#define HAL_QI_INT_SEL        P0SEL

/* INT ENABLE REG: CPU port interrupt */
#define HAL_QI_CPU_PORT_0_IF P0IF

/* INT CFG PORT: CPU port interrupt */
#define HAL_QI_INT_IEN      IEN1  /* REGISTER (0xB8) interrupt enable */
#define HAL_QI_INT_IENBIT   BV(5) /* MASK in IEN1 for enable all Port_0 interrupts */
#define HAL_QI_INT_ICTL     P0IEN /* REGISTER (0xAB) enable interrupts in specific Port_0 pin */
#define HAL_QI_INT_ICTLBIT  BV(6) /* MASK in P0IEN - P0.6 enable/disable bit */
#define HAL_QI_INT_PXIFG    P0IFG /* REGISTER (0x89) Interrupt flag in Port_0 */

#define HAL_QI_INT_PICTL    PICTL /* REGISTER (0x8C) port edge configuration */
#define HAL_QI_INT_EDGEBIT  BV(0) /* MASK in PICTL Port 0, interrupt 0(1)-rising(falling) edge */

/* ----P0_5-TS_CTRL signal-Output/(Input-Tristate)-------------------------------------------------------*/
/* QI control (P0_5-OUTPUT)*/
#define HAL_QI_CTRL_BIT       BV(5)
#define HAL_QI_CTRL_PORT      P0
#define HAL_QI_CTRL_SBIT      P0_5
#define HAL_QI_CTRL_DIR       P0DIR
#define HAL_QI_CTRL_SEL       P0SEL
   

/* -----------------------------------------------------------------------------
 *                               Macros
 * -----------------------------------------------------------------------------
 */

/// macro - default ports gpio initialization for all dragonfly ////////////////

//oJo, testear esto con cuidado que te cargas el conversor
//#define HAL_BOARD_INIT()                                                            \
//{                                                                                   \
//  /* Set to 16Mhz to set 32kHz OSC, then back to 32MHz */                           \
//  START_HSOSC_XOSC();                                                               \
//  SET_OSC_TO_HSOSC();                                                               \
//  SET_32KHZ_OSC();                                                                  \
//  SET_OSC_TO_XOSC();                                                                \
//  STOP_HSOSC();                                                                     \
//  /* Enable cache prefetch mode. */                                                 \
//  PREFETCH_ENABLE();                                                                \
//  /* set direction for GPIO outputs  */                                             \
//  P0DIR = 0x1E;     /* 0-Input(1-Output) 0b00011110*/                               \
//  P1DIR = 0x1F;     /* 0-Input(1-Output) 0b00011011*/                               \
//  P2DIR = 0x01;     /* 0-Input(1-Output) 0b00000001*/                               \
//  P0INP = 0x21;     /* 0-Pull-Up/Pull-Dn(1-Triestate)inputs 0b00100001*/            \
//  P1INP = 0x00;     /* 0-Pull-Up/Pull-Dn(1-Triestate)inputs 0b00000000*/            \
//  P2INP = 0x00;     /* Pull-Up/Pull-Dn for P0, P1 and PU/PD(3S) for P2 0b00000000*/ \
//}

#define HAL_BOARD_INIT()                                                            \
{                                                                                   \
  /* Set to 16Mhz to set 32kHz OSC, then back to 32MHz */                           \
  START_HSOSC_XOSC();                                                               \
  SET_OSC_TO_HSOSC();                                                               \
  SET_32KHZ_OSC();                                                                  \
  SET_OSC_TO_XOSC();                                                                \
  STOP_HSOSC();                                                                     \
  /* Enable cache prefetch mode. */                                                 \
  PREFETCH_ENABLE();                                                                \
  /* set direction for GPIO outputs  */                                             \
  P0DIR = 0x1C;     /* 0-Input(1-Output) 0b00011100*/                               \
  P1DIR = 0x07;     /* 0-Input(1-Output) 0b00000111*/                               \
  P2DIR = 0x01;     /* 0-Input(1-Output) 0b00000001*/                               \
  P0INP = 0x20;     /* 0-Pull-Up/Pull-Dn(1-Triestate)inputs 0b00100000*/            \
  P1INP = 0x08;     /* 0-Pull-Up/Pull-Dn(1-Triestate)inputs 0b00001000*/            \
  P2INP = 0x00;     /* Pull-Up/Pull-Dn for P0, P1 and PU/PD(3S) for P2 0b00000000*/ \
}
      
/// macros - system ////////////////////////////////////////////////////////////
/* Cache Prefetch Control */
#define PREFETCH_ENABLE()     st( FCTL = 0x08; )
#define PREFETCH_DISABLE()    st( FCTL = 0x04; )

/* Setting Clocks */

// switch to the 16MHz HSOSC and wait until it is stable
#define SET_OSC_TO_HSOSC()                                                     \
{                                                                              \
  CLKCONCMD = (CLKCONCMD & 0x80) | CLKCONCMD_16MHZ;                            \
  while ( (CLKCONSTA & ~0x80) != CLKCONCMD_16MHZ );                            \
}

// switch to the 32MHz XOSC and wait until it is stable
#define SET_OSC_TO_XOSC()                                                      \
{                                                                              \
  CLKCONCMD = (CLKCONCMD & 0x80) | CLKCONCMD_32MHZ;                            \
  while ( (CLKCONSTA & ~0x80) != CLKCONCMD_32MHZ );                            \
}

// set 32kHz OSC and wait until it is stable
#define SET_32KHZ_OSC()                                                        \
{                                                                              \
  CLKCONCMD = (CLKCONCMD & ~0x80) | OSC_32KHZ;                                 \
  while ( (CLKCONSTA & 0x80) != OSC_32KHZ );                                   \
}

#define START_HSOSC_XOSC()                                                     \
{                                                                              \
  SLEEPCMD &= ~OSC_PD;            /* start 16MHz RCOSC & 32MHz XOSC */         \
  while (!(SLEEPSTA & XOSC_STB)); /* wait for stable 32MHz XOSC */             \
}

#define STOP_HSOSC()                                                           \
{                                                                              \
  SLEEPCMD |= OSC_PD;             /* stop 16MHz RCOSC */                       \
}

/* Debounce */
#define HAL_DEBOUNCE(expr)    { int i; for (i=0; i<500; i++) { if (!(expr)) i = 0; } }


/// macros - leds //////////////////////////////////////////////////////////////
#define HAL_TURN_OFF_LED1()       st( LED1_SBIT = LED1_POLARITY (0); )
#define HAL_TURN_OFF_LED2()       st( LED2_SBIT = LED2_POLARITY (0); )
#define HAL_TURN_OFF_LED3()       st( LED3_SBIT = LED3_POLARITY (0); )
#define HAL_TURN_OFF_LED4()       HAL_TURN_OFF_LED1()

#define HAL_TURN_ON_LED1()        st( LED1_SBIT = LED1_POLARITY (1); )
#define HAL_TURN_ON_LED2()        st( LED2_SBIT = LED2_POLARITY (1); )
#define HAL_TURN_ON_LED3()        st( LED3_SBIT = LED3_POLARITY (1); )
#define HAL_TURN_ON_LED4()        HAL_TURN_ON_LED1()

#define HAL_TOGGLE_LED1()         st( if (LED1_SBIT) { LED1_SBIT = 0; }        \
                                      else { LED1_SBIT = 1;} )
#define HAL_TOGGLE_LED2()         st( if (LED2_SBIT) { LED2_SBIT = 0; }        \
                                      else { LED2_SBIT = 1;} )
#define HAL_TOGGLE_LED3()         st( if (LED3_SBIT) { LED3_SBIT = 0; }        \
                                      else { LED3_SBIT = 1;} )
#define HAL_TOGGLE_LED4()         HAL_TOGGLE_LED1()

#define HAL_STATE_LED1()          (LED1_POLARITY (LED1_SBIT))
#define HAL_STATE_LED2()          (LED2_POLARITY (LED2_SBIT))
#define HAL_STATE_LED3()          (LED3_POLARITY (LED3_SBIT))
#define HAL_STATE_LED4()          HAL_STATE_LED1()

/*******************************************************************************************************
*/
#endif // HAL_BOARD_CFG_COMMON_H

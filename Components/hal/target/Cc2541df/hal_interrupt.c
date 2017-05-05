/**************************************************************************************************
  Filename:       hal_interrupt.c
  Revised:        $Date: 2012-08-10 19:21:08 -0700 (Fri, 10 Aug 2012) $
  Revision:       $Revision: 31192 $

  Description:    Interrupt services routines for sensors


  Copyright 2012  Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */
#include "hal_board.h"
#include "hal_keys.h"
#include "hal_acc.h"
#include "hal_pir.h"
#include "hal_qi.h"
//#include "hal_mag.h"
#include "hal_i2c.h"
#include "hal_suart.h"

/***************************************************************************************************
*                                    INTERRUPT SERVICE ROUTINE
***************************************************************************************************/

/**************************************************************************************************
* @fn      halPirPort0Isr
*
* @brief   Port0 ISR
**************************************************************************************************/
#if (defined HAL_KEY) && (HAL_KEY == TRUE)
HAL_ISR_FUNCTION( halPirPort0Isr, P0INT_VECTOR )
{

  HAL_ENTER_ISR();

/* S4->INT_4 Qi Charger is at P0.6 falling edge */
#if (defined HAL_KEY) && (HAL_KEY == TRUE)
  #if (defined HAL_QI) && (HAL_QI == TRUE)  
    if (HAL_KEY_SW_4_PXIFG & HAL_KEY_SW_4_BIT) //
    {
      halProcessKeyInterrupt(HAL_KEY_ISR0);
    }
  #endif
#endif
       
  /*
  Clear the CPU interrupt flag for Port_0
  PxIFG has to be cleared before PxIF
  */
         
//  P0IFG = 0;
//  HAL_PIR_PXIFG = 0; //es lo mismo que lo anterior
  HAL_KEY_SW_4_PXIFG = 0;

//  P0IF = 0;
//  HAL_PIR_CPU_PORT_0_IF = 0; //es lo mismo que lo anterior
  HAL_KEY_CPU_PORT_0_IF = 0;
  
  HAL_EXIT_ISR();
}
#endif

/**************************************************************************************************
* @fn      halKeyPort1Isr
*
* @brief   Port1 ISR
**************************************************************************************************/
#if (defined HAL_KEY) && (HAL_KEY == TRUE)
HAL_ISR_FUNCTION( halKeyPort1Isr, P1INT_VECTOR )
{
    HAL_ENTER_ISR();
    
#if (defined HAL_KEY) && (HAL_KEY == TRUE)
//oJo sacar cada KEY a su respectiva board
  if ((HAL_KEY_SW_1_PXIFG & HAL_KEY_SW_1_BIT) || 
      (HAL_KEY_SW_2_PXIFG & HAL_KEY_SW_2_BIT) || 
      (HAL_KEY_SW_3_PXIFG & HAL_KEY_SW_3_BIT))// ||
 //     (HAL_KEY_SW_4_PXIFG & HAL_KEY_SW_4_BIT))
    {
        halProcessKeyInterrupt(HAL_KEY_ISR1);
    }
#endif
  
#if defined (HAL_BOARD_DRAGONFLY_v7E_UVI)
// oJo.- Future implementation
//  if (HAL_UVI_INT_PXIFG & HAL_UVI_INT_BIT)  
//  {
//    HAL_TOGGLE_LED3();
//    HalProcessUviInterrupt();
//  }
#endif  
  
    /*
    Clear the CPU interrupt flag for Port_1
    PxIFG has to be cleared before PxIF
    */
    P1IFG = 0;
    //Ojo queda más claro pero hace 4 veces lo mismo
    HAL_KEY_SW_1_PXIFG = 0;
    HAL_KEY_SW_2_PXIFG = 0;
    HAL_KEY_SW_3_PXIFG = 0;
    HAL_KEY_SW_4_PXIFG = 0;
    
    P1IF = 0;
    HAL_EXIT_ISR();
}
#endif  


/**************************************************************************************************
* @fn      halKeyPort2Isr
*
* @brief   Port2 ISR
**************************************************************************************************/

HAL_ISR_FUNCTION( halI2CIsr, I2C_VECTOR )
{
  HAL_ENTER_ISR();

  /*
    Clear the CPU interrupt flag for Port_2
    PxIFG has to be cleared before PxIF
    Notes: P2_1 and P2_2 are debug lines.
  */
  P2IFG = 0;
  P2IF  = 0;

  HAL_EXIT_ISR();
}

#if (defined HAL_SUART) && (HAL_SUART == TRUE)
// C language code:
// The UARTx TX ISR assumes that the code in Figure 16 has initialized the
// UART TX session, by sending the very first byte on the UARTx TX line.
// Then the UARTx TX ISR will send the remaining bytes based in interrupt
// request generation by the UART peripheral.
// The code implements the following steps:
// 1. Clear UARTx TX Interrupt Flag (IRCON2.UTXxIF = 0).
// 2. Read byte from the allocated UART TX source buffer and write to UxDBUF.
// 3. If no UART byte left to transmit, stop this UART TX session.
// Note that in order to start another UART TX session the application
// just needs to prepare the source buffer, and simply send the very first
// UARTx byte.

extern char uartTxBuffer[SIZE_OF_UART_TX_BUFFER];
extern uint16 uartTxIndex;
extern volatile uint16 uartTxWriting, uartTxLast;

HAL_ISR_FUNCTION( halSuart0TxIsr, UTX0_VECTOR )
{
  HAL_ENTER_ISR();

  UTX0IF = 0;

  if (uartTxIndex == uartTxLast)
  {
    uartTxWriting = 0;
    uartTxIndex = 0;
    uartTxLast = 0;
    IEN2 &= ~0x04;  
    return;
  }
  
  U0DBUF = uartTxBuffer[uartTxIndex % SIZE_OF_UART_TX_BUFFER];
  uartTxIndex++;
  
  HAL_EXIT_ISR();
}

HAL_ISR_FUNCTION( halSuart1TxIsr, UTX1_VECTOR )
{
  HAL_ENTER_ISR();

  UTX1IF = 0;

  if (uartTxIndex == uartTxLast)
  {
    uartTxWriting = 0;
    uartTxIndex = 0;
    uartTxLast = 0;
    IEN2 &= ~0x08; //oJo. comprobar que esta bien
    return;
  }
  
  U1DBUF = uartTxBuffer[uartTxIndex % SIZE_OF_UART_TX_BUFFER];
  uartTxIndex++;

  HAL_EXIT_ISR();
}
#endif

#if (defined HAL_SUART) && (HAL_SUART == TRUE)
// C language code:
// The UARTx RX ISR assumes that the code in Figure 18 has initialized the
// UART RX session, by enabling the UART RX interrupt. Then this UART RX ISR
// will receive the data based in interrupt request generation by the
// USART peripheral.
// The code implements the following steps:
// 1. Clear UARTx RX Interrupt Flag (TCON.URXxIF = 0)
// 2. Read UxDBUF and store the value in the allocated UART RX target buffer
// 3. If all UART data received, stop this UART RX session
// Note that in order to start another UART RX session the application
// just needs to re-enable the UART RX interrupt(IEN0.URXxIE = 1).

extern char uartRxBuffer[SIZE_OF_UART_RX_BUFFER];
extern uint16 uartRxIndex;
extern volatile uint16 uartRxLast;

HAL_ISR_FUNCTION( halSuart0RxIsr, URX0_VECTOR )
{
  HAL_ENTER_ISR();

  URX0IF = 0;

  uartRxBuffer[uartRxIndex++] = U0DBUF;

  if (uartRxIndex >= SIZE_OF_UART_RX_BUFFER) 
  {
    uartRxIndex = 0; 
    IEN0 &= ~0x04;
  }

  HAL_EXIT_ISR();
}

HAL_ISR_FUNCTION( halSuart1RxIsr, URX1_VECTOR )
{
  HAL_ENTER_ISR();

  URX1IF = 0;

  uartRxBuffer[uartRxIndex++] = U1DBUF;

  if (uartRxIndex >= SIZE_OF_UART_RX_BUFFER) 
  {
    uartRxIndex = 0; 
    IEN0 &= ~0x08;  
  }

  HAL_EXIT_ISR();
}
#endif

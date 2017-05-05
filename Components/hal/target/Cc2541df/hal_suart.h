/*******************************************************************************
  Filename:       hal_suart.h
  Revised:        $Date: 2016-09-17 16:28:12$
  Revision:       $Revision: 0 $

  Description:    This file contains the interface to control the simple uart.
*******************************************************************************/
#ifndef HAL_SUART_H
#define HAL_SUART_H

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */
#include "comdef.h"

/* ------------------------------------------------------------------------------------------------
 *                                          Constants
 * ------------------------------------------------------------------------------------------------
 */
/* Defines ports */
#define HAL_UART_PORT_ALT1   1 /* Port Map Alternative 1 */
#define HAL_UART_PORT_ALT2   2 /* Port Map Alternative 2 */

/* Defines UARTS */
#define HAL_UART0   0 /* Use UART0 */
#define HAL_UART1   1 /* Use UART1 */

/* Baud Rate Settings for 32MHz System Clock */
#define HAL_BAUD_M_2400      59 /* Mantissa for   2400bps */
#define HAL_BAUD_E_2400       6 /* Exponent for   2400bps */
#define HAL_BAUD_M_9600      59 /* Mantissa for   9600bps */
#define HAL_BAUD_E_9600       8 /* Exponent for   9600bps */
#define HAL_BAUD_M_57600    216 /* Mantissa for  57600bps */
#define HAL_BAUD_E_57600     10 /* Exponent for  57600bps */
#define HAL_BAUD_M_115200   216 /* Mantissa for 115200bps */
#define HAL_BAUD_E_115200    11 /* Exponent for 115200bps */
#define HAL_BAUD_M_230400   216 /* Mantissa for 230400bps */
#define HAL_BAUD_E_230400    12 /* Exponent for 230400bps */

// Define size of allocated UART RX/TX buffer (just an example)
#define SIZE_OF_UART_RX_BUFFER 128
#define SIZE_OF_UART_TX_BUFFER SIZE_OF_UART_RX_BUFFER  
/* ------------------------------------------------------------------------------------------------
 *                                           Typedefs
 * ------------------------------------------------------------------------------------------------
 */
// C language code:
// Define and allocate a setup structure for the UART protocol:
typedef struct 
{
  unsigned char uartNum : 1; // UART peripheral number (0 or 1)
  unsigned char START   : 1; // Start bit level (low/high)
  unsigned char STOP    : 1; // Stop bit level (low/high)
  unsigned char SPB     : 1; // Stop bits (0 => 1, 1 => 2)
  unsigned char PARITY  : 1; // Parity control (enable/disable)
  unsigned char BIT9    : 1; // 9 bit enable (8bit / 9bit)
  unsigned char D9      : 1; // 9th bit level or Parity type
  unsigned char FLOW    : 1; // HW Flow Control (enable/disable)
  unsigned char ORDER   : 1; // Data bit order(LSB/MSB first)
} UART_PROT_CONFIG;


/* ------------------------------------------------------------------------------------------------
 *                                          Functions
 * ------------------------------------------------------------------------------------------------
 */


extern void uartPrintf(char *format_string, ...);
extern void uartWrite(uint8 *data, int length);

extern void HalSuartInit(void);

void uartStartTxForIsr(unsigned char uartNum);
void uartStartRxForIsr(unsigned char uartNum);


#ifdef __cplusplus
};
#endif

#endif

/**************************************************************************************************
*/

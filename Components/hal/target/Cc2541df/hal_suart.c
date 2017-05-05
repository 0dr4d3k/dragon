/*******************************************************************************
  Filename:       hal_suart.c
  Revised:        $Date: 2016-09-17 16:28:12$
  Revision:       $Revision: 0 $

  Description:    Simple UART.
*******************************************************************************/

/*******************************************************************************
 *                                 INCLUDES
 ******************************************************************************/
#include "hal_board.h"
#include "hal_suart.h"
#include "hal_drivers.h"
#include "hal_sensor.h"
#include "osal.h"
#include "string.h"
#include <stdio.h>
#include <stdarg.h>

#if (defined HAL_SUART) && (HAL_SUART == TRUE)

/*******************************************************************************
 *                                 CONSTANTS
 ******************************************************************************/


/*******************************************************************************
 *                                   MACROS
 ******************************************************************************/

/*******************************************************************************
 *                                  TYPEDEFS
 ******************************************************************************/

/*******************************************************************************
 *                              GLOBAL VARIABLES
 ******************************************************************************/

UART_PROT_CONFIG uartProtConfig;

// Allocate buffer+index for UART RX/TX
char uartRxBuffer[SIZE_OF_UART_RX_BUFFER];
uint16 uartRxIndex;
uint16 uartRxLast;
char uartTxBuffer[SIZE_OF_UART_TX_BUFFER];
uint16 uartTxIndex;
uint16 volatile uartTxWriting, uartTxLast;
 
/*******************************************************************************
 *                               LOCAL VARIABLES
 ******************************************************************************/


/*******************************************************************************
 *                              FUNCTIONS - Local
 ******************************************************************************/
void uartMapPort(uint8 uartPortAlt, uint8 uartNum);
void uartInitBitrate(uint8 uartBaudM, uint8 uartBaudE);
void uartInitProtocol(UART_PROT_CONFIG* uartProtConfig);


// C language code:
// This function maps/connects the UART to the desired SoC I/O port.
// The application should call this function with "uartPortAlt" = 1 or 2,
// and "uartNum" = 0 or 1.
void uartMapPort(uint8 uartPortAlt, uint8 uartNum) 
{
  // If UART Port Alternative 1 desired
  if(uartPortAlt == 1) 
  {
    // If UART0 desired
    if (uartNum == 0) 
    {
      // Configure UART0 for Alternative 1 => Port P0 (PERCFG.U0CFG = 0)
      PERCFG &= ~0x01;
      // Configure relevant Port P0 pins for peripheral function:
      // P0SEL.SELP0_2/3/4/5 = 1 => RX = P0_2, TX = P0_3, CT = P0_4, RT = P0_5
//      P0SEL |= 0x3C;
      P0SEL |= 0x0C;
      // Configure relevant Port P1 pins back to GPIO function
//      P1SEL &= ~0x3C;
//      P1SEL &= ~0x0C;
    } 
    // Else (UART1 desired)
    else 
    {
      // Configure UART1 for Alternative 1 => Port P0 (PERCFG.U1CFG = 0)
      PERCFG &= ~0x02;
      // Configure relevant Port P0 pins for peripheral function:
      // P0SEL.SELP0_2/3/4/5 = 1 => CT = P0_2, RT = P0_3, TX = P0_4, RX = P0_5
//      P0SEL |= 0x3C;
      // Configure relevant Port P1 pins back to GPIO function
//      P1SEL &= ~0xF0;
    }
  } 
  // Else (UART Port Alternative 2 desired)
  else 
  {
    // If UART0 desired 
    if (uartNum == 0) 
    {
      // Configure UART0 for Alternative 2 => Port P1 (PERCFG.U0CFG = 1)
      PERCFG |= 0x01;
      // P1SEL.SELP1_2/3/4/5 = 1 => CT = P1_2, RT = P1_3, RX = P1_4, TX = P1_5
//      P1SEL |= 0x3C;
      P1SEL |= 0x30;
      // Configure relevant Port P0 pins back to GPIO function
//      P0SEL &= ~0x3C;
    } 
    // Else (UART1 desired)
    else 
    {
      // Configure UART1 for Alternative 2 => Port P1 (PERCFG.U1CFG = 1)
      PERCFG |= 0x02;
      // P1SEL.SELP1_4/5/6/7 = 1 => CT = P1_4, RT = P1_5, TX = P1_6, RX = P1_7
//      P1SEL |= 0xF0;
      // Configure relevant Port P0 pins back to GPIO function
//      P0SEL &= ~0x3C;
    }
  }
}

// C language code:
// This function initializes the UART bit rate.
void uartInitBitrate(unsigned char uartBaudM, unsigned char uartBaudE) 
{
  ///////////////////////////////////////////////////////////////
  // This initial code section ensures that the SoC system clock is driven
  // by the HS XOSC:
  // Clear CLKCON.OSC to make the SoC operate on the HS XOSC.
  // Set CLKCON.TICKSPD/CLKSPD = 000 => system clock speed = HS RCOSC speed.
// CLKCON &= 0x80;
   CLKCONCMD &= 0x80;
   
  // Monitor CLKCON.OSC to ensure that the HS XOSC is stable and actually
  // applied as system clock source before continuing code execution
//  while(CLKCON & 0x40);
   while(CLKCONCMD & 0x40);
  
  // Set SLEEP.OSC_PD to power down the HS RCOSC.
//  SLEEP |= 0x04;
   SLEEPCMD |= 0x04;

  ///////////////////////////////////////////////////////////////
  // Initialize bitrate (U0BAUD.BAUD_M, U0GCR.BAUD_E)
  U0BAUD = uartBaudM;
  U0GCR = (U0GCR&~0x1F) | uartBaudE;
}

// C language code:
// This function initializes the UART protocol (start/stop bit, data bits,
// parity, etc.). The application must call this function with an initialized
// data structure according to the code in Figure 12.
void uartInitProtocol(UART_PROT_CONFIG* uartProtConfig) 
{
  // Initialize UART protocol for desired UART (0 or 1)
  if (uartProtConfig->uartNum == 0) 
  {
    // USART mode = UART (U0CSR.MODE = 1)
    U0CSR |= 0x80;
    
    // Start bit level = low => Idle level = high (U0UCR.START = 0)
    // Start bit level = high => Idle level = low (U0UCR.START = 1)
    U0UCR = (U0UCR&~0x01) | uartProtConfig->START;
    
    // Stop bit level = high (U0UCR.STOP = 1)
    // Stop bit level = low (U0UCR.STOP = 0)
    U0UCR = (U0UCR&~0x02) | (uartProtConfig->STOP << 1);

    // Number of stop bits = 1 (U0UCR.SPB = 0) 
    // Number of stop bits = 2 (U0UCR.SPB = 1)
    U0UCR = (U0UCR&~0x04) | (uartProtConfig->SPB << 2);

    // Parity = disabled (U0UCR.PARITY = 0)
    // Parity = enabled (U0UCR.PARITY = 1)
    U0UCR = (U0UCR&~0x08) | (uartProtConfig->PARITY << 3);

    // 9-bit data disable = 8 bits transfer (U0UCR.BIT9 = 0)
    // 9-bit data enable = 9 bits transfer (U0UCR.BIT9 = 1)
    U0UCR = (U0UCR&~0x10) | (uartProtConfig->BIT9 << 4);

    // Level of bit 9 = 0 (U0UCR.D9 = 0), used when U0UCR.BIT9 = 1
    // Level of bit 9 = 1 (U0UCR.D9 = 1), used when U0UCR.BIT9 = 1
    // Parity = Even (U0UCR.D9 = 0), used when U0UCR.PARITY = 1
    // Parity = Odd (U0UCR.D9 = 1), used when U0UCR.PARITY = 1
    U0UCR = (U0UCR&~0x20) | (uartProtConfig->D9 << 5);

    // Flow control = disabled (U0UCR.FLOW = 0)
    // Flow control = enabled (U0UCR.FLOW = 1)
    U0UCR = (U0UCR&~0x40) | (uartProtConfig->FLOW << 6);

    // Bit order = MSB first (U0GCR.ORDER = 1)
    // Bit order = LSB first (U0GCR.ORDER = 0) => For PC/Hyperterminal
    U0GCR = (U0GCR&~0x20) | (uartProtConfig->ORDER << 5);
  } 
  else 
  {
    // USART mode = UART (U1CSR.MODE = 1)
    U1CSR |= 0x80;

    // Start bit level = low => Idle level = high (U1UCR.START = 0)
    // Start bit level = high => Idle level = low (U1UCR.START = 1)
    U1UCR = (U1UCR&~0x01) | uartProtConfig->START;

    // Stop bit level = high (U1UCR.STOP = 1)
    // Stop bit level = low (U1UCR.STOP = 0)
    U1UCR = (U1UCR&~0x02) | (uartProtConfig->STOP << 1);

    // Number of stop bits = 1 (U1UCR.SPB = 0)
    // Number of stop bits = 2 (U1UCR.SPB = 1)
    U1UCR = (U1UCR&~0x04) | (uartProtConfig->SPB << 2);

    // Parity = disabled (U1UCR.PARITY = 0)
    // Parity = enabled (U1UCR.PARITY = 1)
    U1UCR = (U1UCR&~0x08) | (uartProtConfig->PARITY << 3);

    // 9-bit data enable = 8 bits transfer (U1UCR.BIT9 = 0)
    // 9-bit data enable = 8 bits transfer (U1UCR.BIT9 = 1)
    U1UCR = (U1UCR&~0x10) | (uartProtConfig->BIT9 << 4);

    // Level of bit 9 = 0 (U1UCR.D9 = 0), used when U1UCR.BIT9 = 1
    // Level of bit 9 = 1 (U1UCR.D9 = 1), used when U1UCR.BIT9 = 1
    // Parity = Even (U1UCR.D9 = 0), used when U1UCR.PARITY = 1
    // Parity = Odd (U1UCR.D9 = 1), used when U1UCR.PARITY = 1
    U1UCR = (U1UCR&~0x20) | (uartProtConfig->D9 << 5);

    // Flow control = disabled (U1UCR.FLOW = 0)
    // Flow control = enabled (U1UCR.FLOW = 1)
    U1UCR = (U1UCR&~0x40) | (uartProtConfig->FLOW << 6);

    // Bit order = MSB first (U1GCR.ORDER = 1)
    // Bit order = LSB first (U1GCR.ORDER = 0) => For PC/Hyperterminal
    U1GCR = (U1GCR&~0x20) | (uartProtConfig->ORDER << 5);
  }
}

  
/*******************************************************************************
 *                               FUNCTIONS - API
 ******************************************************************************/

/*******************************************************************************
 * @fn      HalChaseInit
 *
 * @brief   Initilize chase hardware
 *
 * @param   none
 *
 * @return  None
 */
void HalSuartInit(void)
{  
  uartProtConfig.uartNum = 0;
  uartProtConfig.START   = 0; 
  uartProtConfig.STOP    = 1; 
  uartProtConfig.SPB     = 0;
  uartProtConfig.PARITY  = 0;
  uartProtConfig.BIT9    = 0;
  uartProtConfig.D9      = 0;
  uartProtConfig.FLOW    = 0;
  uartProtConfig.ORDER   = 0;

  uartRxIndex = uartRxLast = 0;
  uartTxIndex = uartTxLast = uartTxWriting = 0;

  // setting UART baud rate generator
  uartInitBitrate(HAL_BAUD_M_115200, HAL_BAUD_E_115200);
  ST_HAL_DELAY(125);

  // initialize the UART protocol
  uartInitProtocol(&uartProtConfig);  
  ST_HAL_DELAY(125);

  // define to use UART0 in ports P1.5(TX) P1.4(RX), this is the alternative 2
  uartMapPort(HAL_UART_PORT_ALT2, HAL_UART0);
  ST_HAL_DELAY(125);

  // sample message
  uartPrintf("[hal_suart] dragonfly terminal\r\n");
}

char uartPrintfBuff[SIZE_OF_UART_TX_BUFFER];
void uartPrintf(char *format_string, ...)
{
  int i, length;

  va_list args;
  va_start(args, format_string);
  length = vsprintf(uartPrintfBuff, format_string, args);
  va_end(args);

  for(i=0; i<length; i++)
  {
    uartTxBuffer[(uartTxLast + i) % SIZE_OF_UART_TX_BUFFER] = uartPrintfBuff[i];
  }
  uartTxLast += length;

  uartStartTxForIsr(HAL_UART0);
}

void uartWrite(uint8 *data, int length)
{
  int i;

  for(i=0; i<length; i++)
  {
    uartTxBuffer[(uartTxLast + i) % SIZE_OF_UART_TX_BUFFER] = data[i];
  }
  uartTxLast += length;

  uartStartTxForIsr(HAL_UART0);
}


// C language code:
// This function starts the UART TX session and leaves the transmission
// of the remaining bytes to the associated UART TX ISR in Figure 17.
// Before this function is called the application must initialize the
// UART peripheral according to the code shown in Figure 3, Figure 11,
// Figure 12, and Figure 13.
// The code implements the following steps:
// 1. Initialize the UART TX buffer index.
// 2. Clear UART TX Interrupt Flag (IRCON2.UTXxIF = 0.
// 3. Enable UART TX Interrupt (IEN2.UTXxIE = 1)
// 4. Send very first UART byte
// 5. Enable global interrupt (IEN0.EA = 1).
void uartStartTxForIsr(unsigned char uartNum) 
{
  if(uartTxWriting == 1)
  {
    return;
  }

  uartTxWriting = 1;

  if (uartNum == 0) 
  {
    UTX0IF = 0;
    U0DBUF = uartTxBuffer[uartTxIndex++];
    IEN2 |= 0x04;
  } 
  else 
  {
    UTX1IF = 0;
    U1DBUF = uartTxBuffer[uartTxIndex++];
    IEN2 |= 0x08;
  }
  
  IEN0 |= 0x80;
}

// C language code:
// This function initializes the UART RX session, by simply enabling the
// corresponding UART interrupt, and leave the sample reception to the
// UART ISR shown in Figure 19. Before this function is called the
// application must initialize the UART peripheral according to the
// code shown in Figure 3, Figure 11, Figure 12, and Figure 13.
// The code implements the following steps:
// 1. Initialize the UART RX buffer index.
// 2. Clear UART RX Interrupt Flag (TCON.URXxIF = 0)
// 3. Enable UART RX and Interrupt (IEN0.URXxIE = 1, UxCSR.RE = 1)
// 4. Enable global interrupt (IEN0.EA = 1)

void uartStartRxForIsr(unsigned char uartNum) 
{
  if (uartNum == 0) 
  {
    URX0IF = 0;
    U0CSR |= 0x40;
    IEN0 |= 0x04;
  } 
  else 
  {
    URX1IF = 0;
    U1CSR |= 0x40;
    IEN0 |= 0x08;
  }
  
  IEN0 |= 0x80;
}


/*******************************************************************************
*******************************************************************************/
#endif //(defined HAL_SUART) && (HAL_SUART == TRUE)
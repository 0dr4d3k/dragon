
/***********************************************************************************
  Filename:     i2c_master_receive.c

  Description:  This example uses a master to receive data from a slave using I2C.

  Comments:     To execute this example, use the IAR project i2c_slave_send
                as the slave. The slave's code must be executed before executing
                the master's code, since the slave is clocked by the Master. The
                bytes sent are simply numbers 0x00 upto BUFFER_SIZE. When bytes
                up to BUFFER_SIZE are sent, the example will end and LED1 will
                be set on the SmartRF05EB.

  Note:         On the SmartRF05EB, P0_6 and P0_7 from CC2543EM is shared by the 
                EM_JOY_LEVEL and EM_LCD_CS signals. These have to be disconnected 
                by removing jumpers 7-8 and 3-4 on P10 I/O Connector. Thus the I2C 
                and SRF05EB LCD can't be used simultaneously.

                The I2C pins also have external pullups on the EM, at the as 
                required by the I2C standard.

***********************************************************************************/

/***********************************************************************************
* INCLUDES
*/

#include <hal_types.h>
// Include Name definitions of individual bits and bit-fields in the CC254x device registers.
#include <ioCC254x_bitdef.h>
// Include device specific file
#include "ioCC2541.h"   


/***********************************************************************************
* CONSTANTS
*/

// Define size of buffer and number of bytes to send
#define BUFFER_SIZE 0xFF
#define SLAVE_ADDRESS 0x18    // 7-bit addressing

#define READ_FROM_SLAVE 0x01
#define WRITE_TO_SLAVE 0x00


/***********************************************************************************
* LOCAL VARIABLES
*/

// Masters's transmit buffer.
static uint8 buffer[BUFFER_SIZE];
static uint16 bufferIndex = 0;


/***********************************************************************************
* LOCAL FUNCTIONS
*/

/***********************************************************************************
* @fn          I2C_ISR
*
* @brief       Function which sends the next I2C data byte.
*
* @param       none
*
* @return      0
*/
#pragma vector = I2C_VECTOR

__interrupt void I2C_ISR(void)
{ 
    // Clear I2C CPU interrupt flag.
    P2IF = 0;
  
    // If a Start or Restart condition has been transmitted ...
    if (I2CSTAT == 0x08 || I2CSTAT == 0x10)
    {
        // Send Slave address and R/W access.
        I2CDATA = (SLAVE_ADDRESS << 1) | READ_FROM_SLAVE;
    
        // End Start condition.
        I2CCFG &= ~I2CCFG_STA;
    }

    // If a Data byte has been received and acknowledge has been returned...
    else if (I2CSTAT == 0x50)
    {
        // Read Data byte.
        buffer[bufferIndex++] = I2CDATA;
    }
  
    // If finished receiving...
    if (bufferIndex >= BUFFER_SIZE )
    {
        // Generate Stop condition.
        I2CCFG |= I2CCFG_STO;

        // Disable interrupt from I2C by setting [IEN2.I2CIE = 0].
        IEN2 &= ~IEN2_P2IE;
    
        // Set SRF05EB LED1.
        P0_3 = 1;  
    }

    // I2CCFG.SI flag must be cleared by software at the end of the ISR.
    I2CCFG &= ~I2CCFG_SI;
}


/***********************************************************************************
* @fn          main
*
* @brief       Send data to a single slave using I2C in Master mode
*
* @param       void
*
* @return      0
*/
int main(void)
{
    /****************************************************************************
    * Clock setup
    * See basic software example "clk_xosc_cc254x"
    */
  
    // Set system clock source to HS XOSC, with no pre-scaling.
    CLKCONCMD = (CLKCONCMD & ~(CLKCON_OSC | CLKCON_CLKSPD)) | CLKCON_CLKSPD_32M;
    // Wait until clock source has changed.
    while (CLKCONSTA & CLKCON_OSC);
  
    // Note the 32 kHz RCOSC starts calibrating, if not disabled.

    /***************************************************************************
    * Setup I/O ports
    *
    * CC2541 has dedicated I2C ports
    * I2C SCL:   Pin 2   (Debug Connector P18_5)
    * I2C SDA:   Pin 3   (Debug Connector P18_3)
    */

    // Enable I2C on CC2541.
    I2CWC &= ~0x80;

    // Configure P1_0 as GPIO output for LED1.
    P0SEL &= BIT3;      // GPIO.
    P0DIR |= BIT3;      // Output.
    P0_3 = 0;           // LED1 off.


  /***************************************************************************
  * Setup interrupt
  */

    // Clear I2C (P2) CPU interrupt flag.
    P2IF = 0;

    // Enable interrupt from I2C by setting [IEN2.P2IE = 1].
    IEN2 |= IEN2_P2IE;

    // Enable global interrupts.
    EA = 1;


    /***************************************************************************
    * Configure I2C
    */

    // Enable the I2C module with 33 kHz clock rate.
    // Enable Assert Acknowledge (AA bit).
    // The STArt bit signals a master.
    I2CCFG = (I2CCFG & ~I2CCFG_CR) | I2CCFG_CR_DIV_960 | I2CCFG_ENS1 | I2CCFG_AA | I2CCFG_STA;


    /* Main Loop, the transfers are interrupt handled. */
    while(1);

}



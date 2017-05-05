/*******************************************************************************
  Filename:       hal_sensor.c
  Revised:        $Date: 2017-04-14 11:44:11$
  Revision:       $Revision: 31581 $

  Description:    This file contains code that is common to all sensor drivers.
*******************************************************************************/

/* -----------------------------------------------------------------------------
 *                                   Includes
 * -----------------------------------------------------------------------------
 */
#include "hal_sensor.h"
#include "hal_i2c.h"
#include "hal_led.h"
#include "hal_debug.h"

/* -----------------------------------------------------------------------------
 *                             Macros and constants
 * -----------------------------------------------------------------------------
 */
#define N_TEST_RUNS 10

/* -----------------------------------------------------------------------------
 *                               Local Variables
 * -----------------------------------------------------------------------------
 */
static uint8 buffer[24];

/******************************************************************************
 * @fn          HalSensorReadReg
 * @brief       This function implements the I2C protocol to read from a sensor. 
 *              The sensor must be selected before this routine is called.
 * @param       reg - which register to read.
 * @param       pBuf - pointer to buffer to place data.
 * @param       nBytes - numbver of bytes to read.
 * @return      TRUE if the required number of bytes are reveived.
 ******************************************************************************/
bool HalSensorReadReg(uint8 reg, uint8 *pBuf, uint8 nBytes)
{
  uint8 i = 0;

  /* Send address we're reading from */
  if (HalI2CWrite(1,&reg) == 1)
  {
    /* Now read data */
    i = HalI2CRead(nBytes,pBuf);
  }

  if ( i != nBytes)
    HAL_TOGGLE_LED2();

  return i == nBytes;
}

/******************************************************************************
 * @fn          HalSensorWriteReg
 * @brief       This function implements the I2C protocol to write to a sensor. 
 *              The sensor must be selected before this routine is called.
 * @param       reg  - which register to write.
 * @param       pBuf - pointer to buffer containing data to be written.
 * @param       nBytes - number of bytes to write.
 * @return      TRUE if successful write.
 ******************************************************************************/
bool HalSensorWriteReg(uint8 reg, uint8 *pBuf, uint8 nBytes)
{
  uint8 i;
  uint8 *p = buffer;

  /* Copy register and data to local buffer for burst write */
  *p++ = reg;
  for (i = 0; i < nBytes; i++)
  {
    *p++ = *pBuf++;
  }
  nBytes++;

  /* Send register and data */
  i = HalI2CWrite(nBytes, buffer);
  if ( i!= nBytes)
    HAL_TOGGLE_LED2();

  return (i == nBytes);
}

/******************************************************************************
 * @fn      HalSensorTest
 * @brief   Run a self-test on all the sensors
 * @param   none
 * @return  bitmask of error flags
 ******************************************************************************/
uint16 HalSensorTest(void)
{
  uint16 i;
  uint8 selfTestResult;

  selfTestResult = 0;

  for  (i=0; i<N_TEST_RUNS; i++)
  {
    HalLedSet(HAL_LED_2,HAL_LED_MODE_TOGGLE);

    // 1. Temp sensor test

    // 2. Humidity  sensor test

    // 3. Magnetometer test

    // 4. Accelerometer test
//    if (HalAccTest())
//      selfTestResult |= ST_ACC;

    // 5. Barometer test

    // 6. Gyro test
  }

//////////////////////////////////////////////////// begin debug context ///////
  HAL_DEBUG_STATEMENT(                                                         \
  /* statement context */                                                      \
  DEBUG_FORCE_UART | DEBUG_SENSOR | DEBUG_DUMP,                                \
  {                                                                            \
    /* i2c scan*/                                                              \
    uint16 i; uint16 ii; uint16 iii;                                           \
    unsigned char data[2];                                                     \
    bool show = false;                                                         \
                                                                               \
    halDebugMsg(DEBUG_SENSOR | DEBUG_DUMP, "i2c_bus_scan:\n");                 \
                                                                               \
    /* read all registers*/                                                    \
    for (i = 0; i <= (uint8) 0x7F; i++)                                        \
    {                                                                          \
      HalI2CInit(i, i2cClock_267KHZ);                                          \
                                                                               \
      for (ii = 0; ii <= 0xFF; ii += 16 )                                      \
      {                                                                        \
        for (int z = 0; z < 16; z++) data[z] = 0x00;                           \
                                                                               \
        HalSensorReadReg(ii, data, 16);                                        \
                                                                               \
        for (int z = 0; z < 16; z++)                                           \
        {                                                                      \
          show = false;                                                        \
          if (data[z] != 0x00) show = true;                                    \
        }                                                                      \
        if (show)                                                              \
        {                                                                      \
           halDebugMsg(DEBUG_SENSOR | DEBUG_DUMP,                              \
                       "  addr:0x%02X, reg:0x%02X ->", i, ii);                 \
           for (iii = 0; iii < 16; iii++)                                      \
             halDebugMsg(SKIP_CONTEXT," %02X", data[iii]);                     \
           halDebugMsg(SKIP_CONTEXT,"\r\n");                                   \
        }                                                                      \
      }                                                                        \
    }                                                                          \
  });  
////////////////////////////////////////////////////// end debug context ///////
  
  
  
  return selfTestResult;
}

/******************************************************************************
 * Function: 	This function writes some bits via I2C.
 * Parameters:	devAddr: I2C address of the device.
 *              regAddr: I2C register.
 *              data: data to send.
 *              pos: first position of the bits to write [7|6|5|4|3|2|1|0].
 *              length: number of bits to write.
 * Return:      TRUE if the required number of bytes are reveived.
******************************************************************************/
uint8 HalSensorWriteBits(uint8 regAddr, uint8 data, uint8 pos, uint8 length)
{
  uint8 mask = 0;
  uint8 buffer = 0;
  uint8 success = 0;
    
  // Read the register
  success = HalSensorReadReg (regAddr, &buffer, sizeof(buffer));

  if (success !=0 )
  {
    //error handle
    return success;
  }
  
  // Mask to the read data and stores the value
  mask = (((1 << length) - 1) << pos);
  data = (data << pos) & mask;
  buffer &= ~mask;	
  buffer |= data;
  
  
  // Write the register
  HalSensorWriteReg(regAddr, &buffer, sizeof(buffer));

  return success;
} 

/******************************************************************************
 * Function: 	This function reads a bit via I2C.
 * Parameters:	devAddr: I2C address of the device.
 *              regAddr: I2C register.
 *              data: buffer to store the data.
 *              pos: position of the bit to read [7|6|5|4|3|2|1|0].
 * Return:      TRUE if the required number of bytes are reveived.
 ******************************************************************************/
uint8 HalSensorReadBit(uint8 regAddr, uint8 *data, uint8 pos)
{
  uint8 buffer, mask;
  uint8 answer;
	
  // Read the register
  answer = HalSensorReadReg (regAddr, &buffer, sizeof(buffer));
		
  mask = 1 << pos;	
  if ((mask & buffer) != 0) 
    data[0] = 1;
  else 
    data[0] = 0;
	
  return answer;	
}

/******************************************************************************
 * Function: 	This function reads a bit via I2C
 * Parameters:	devAddr: I2C address of the device
 *              regAddr: I2C register
 *              data: buffer to store the data
 *              pos: position of the bit to read [7|6|5|4|3|2|1|0]
 * Return:      TRUE if the required number of bytes are reveived
 ******************************************************************************/
uint8 HalSensorReadBits(uint8 devAddr, uint8 regAddr, uint8 *data, uint8 pos, uint8 length)
{
  uint8 buffer, mask;
  uint8 answer;
	
  // Read the register
  answer = HalSensorReadReg (regAddr, &buffer, sizeof(buffer));
	
  // Mask to the read data and stores the value
  buffer = buffer >> pos;
  mask = ((1 << length) - 1);
  buffer &= mask;
  data[0] = buffer;	
	
  return answer;	
}

/******************************************************************************
 ******************************************************************************/

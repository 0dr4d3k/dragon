/*******************************************************************************
  Filename:       hal_uvi.c
  Revised:        $Date: 2016-08-03 11:45:31$
  Revision:       $Revision: 0 $

  Description:    This file contains the declaration of SILICON LABs Si1133
                  Sun UV index readiation sensor abstraction layer.
*******************************************************************************/

/* -----------------------------------------------------------------------------
 *                                 Includes
 * -----------------------------------------------------------------------------
 */

#include "hal_board.h"
#include "hal_sensor.h"
#include "hal_suart.h"
#include "hal_i2c.h"
#include "hal_uvi.h"
#include "hal_debug.h"
#include "math.h"

#if (defined HAL_UVI) && (HAL_UVI == TRUE)

/* -----------------------------------------------------------------------------
 *                                 Constants
 * -----------------------------------------------------------------------------
 */


/* -----------------------------------------------------------------------------
 *                              Local Functions
 * -----------------------------------------------------------------------------
 */
// INTERNAL CONTROL FUNCTIONS
static void HalUViSelect(void);

static void HalUViWakeUp(void);
static void HalUViSleep(void);

/* no used in our code
static void   HalUViWrite8(uint8 reg, uint8 value);
static uint8  HalUViRead8(uint8 reg);
static uint16 HalUViRead16(uint8 reg);
*/
static void   HalUViReadData(void);

float uv_call_example(HANDLE si115x_handle, 
                      SI115X_SAMPLES *samples);

float lux_call_example(HANDLE si115x_handle, 
                      SI115X_SAMPLES *samples);

int16_t si115x_init (HANDLE si115x_handle);

/* -----------------------------------------------------------------------------
 *                               Local Variables
 * -----------------------------------------------------------------------------
 */

HANDLE si115x_handle;
SI115X_SAMPLES *samples;

/* -----------------------------------------------------------------------------
 *                              Private functions
 * -----------------------------------------------------------------------------
 */

// Move these functions to hal_sensor.c to enable to use for all sensors
int16_t Si115xWriteToRegister(  HANDLE   si115x_handle,
                                uint8_t  address,
                                uint8_t  value);

int16_t Si115xReadFromRegister( HANDLE   si115x_handle, 
                                uint8_t  address);

int16_t Si115xBlockWrite(       HANDLE   si115x_handle,
                                uint8_t  address,
                                uint8_t  length,
                                uint8_t* values);

int16_t Si115xBlockRead(        HANDLE   si115x_handle,
                                uint8_t  address,
                                uint8_t  length,
                                uint8_t* values);

// Delay function used by reset
void Si115xDelay_10ms(          void);


//----------------------------------------------------------------------------//
int16_t Si115xWriteToRegister(  HANDLE   si115x_handle,
                                uint8_t  address,
                                uint8_t  value)
{
//    return i2c_smbus_write_byte_data(si115x_handle, address, value);
  HalUViSelect();
  if (TRUE != HalSensorWriteReg(address, &value, sizeof(value)))
    return -1;
  return 0;
}
//............................................................................//
int16_t Si115xReadFromRegister( HANDLE   si115x_handle, 
                                uint8_t  address)
{
  uint8 buf;
//    return i2c_smbus_read_byte_data(si115x_handle, address);
  HalUViSelect();
  if (TRUE != HalSensorReadReg(address, &buf, sizeof(buf)))
    return -1;
  return buf;
}

//............................................................................//
int16_t Si115xBlockWrite(       HANDLE   si115x_handle,
                                uint8_t  address,
                                uint8_t  length,
                                uint8_t* values)
{
   uint8_t retval, counter;

   for ( counter = 0; counter < length; counter++)
   {
      retval = Si115xWriteToRegister(si115x_handle, address+counter, values[counter]);
   }

   return retval;    
}

//............................................................................//
int16_t Si115xBlockRead(        HANDLE   si115x_handle,
                                uint8_t  address,
                                uint8_t  length,
                                uint8_t* values)

{
//   return i2c_smbus_read_i2c_block_data ( si115x_handle, address, length, values );
  if (TRUE != HalSensorReadReg(address, values, length))
    return -1;
  return  length;

}

void Si115xDelay_10ms(void)
{
  ST_HAL_DELAY(1250); // delay ~10ms 
}


/******************************************************************************
 * Function:  This function select the SI1133 UVi sensor device on the I2C bus.
 * Returns:   Nothing
 ******************************************************************************/
static void HalUViSelect(void)
{
  //Set up I2C that is used to communicate with TSL2561
  HalI2CInit(SI1133_ADDR, i2cClock_267KHZ);
}

/******************************************************************************
 * Function:  This function enables the device
 * Returns:   Nothing
 ******************************************************************************/
void HalUViWakeUp(void)
{
  // To start forced measurements         
  Si115xForce( si115x_handle);
}

/******************************************************************************
 * Function:  Disables the device (putting it in lower power sleep mode)
 * Returns:   Nothing
 ******************************************************************************/
void HalUViSleep(void)
{
  /* Turn the device off to save power */
//  HalUViWrite8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWEROFF);
}

/******************************************************************************
 * Function:   Private function to read luminosity on both channels.
 * Parameters: pBB. Pointer to BroadBand data
 *             pIR. Pointer to Infra Red data              
 * Returns:    Nothing
 ******************************************************************************/
void HalUViReadData(void)
{
#if defined CALCULATE_LUX

    uint8_t buffer[13];                           
    Si115xBlockRead(  si115x_handle,               
                      SI115x_REG_IRQ_STATUS,             
                      13,                         
                      buffer);                    
    samples->irq_status = buffer[0];              
    samples->ch0  = (int32_t)buffer[1] << 16;
    samples->ch0 |= (int32_t)buffer[2] <<  8;
    samples->ch0 |= (int32_t)buffer[3];
    if( samples->ch0 & 0x800000 )   
        samples->ch0 |= 0xFF000000; 
    samples->ch1  = (int32_t)buffer[4] << 16;
    samples->ch1 |= (int32_t)buffer[5] <<  8;
    samples->ch1 |= (int32_t)buffer[6];
    if( samples->ch1 & 0x800000 )   
        samples->ch1 |= 0xFF000000; 
    samples->ch2  = (int32_t)buffer[7] << 16;
    samples->ch2 |= (int32_t)buffer[8] <<  8;
    samples->ch2 |= (int32_t)buffer[9];
    if( samples->ch2 & 0x800000 )   
        samples->ch2 |= 0xFF000000; 
    samples->ch3  = (int32_t)buffer[10] << 16;
    samples->ch3 |= (int32_t)buffer[11] <<  8;
    samples->ch3 |= (int32_t)buffer[12];
    if( samples->ch3 & 0x800000 )   
        samples->ch3 |= 0xFF000000; 
#elif defined CALCULATE_UV
    uint8_t buffer[4];                           
    Si115xBlockRead( si115x_handle,               
                      SI115x_REG_IRQ_STATUS,             
                      4,                         
                      buffer);                    
    samples->irq_status = buffer[0];              
    samples->ch0  = (int32_t)buffer[1] << 16;
    samples->ch0 |= (int32_t)buffer[2] <<  8;
    samples->ch0 |= (int32_t)buffer[3];
    if( samples->ch0 & 0x800000 )   
        samples->ch0 |= 0xFF000000;
#endif    
}


/******************************************************************************
 * Function:   Private function to write a 8bits register.
 * Parameters: reg. Destin resister to write to.
 *             value. Value to write.              
 * Returns:    Nothing
 ******************************************************************************/
/* no used in our code 
void HalUViWrite8(uint8 reg, uint8 value)
{
  //call to sensor functions
  HalSensorWriteReg(reg, &value, sizeof(value)); 
}
*/


/******************************************************************************
 * Function:   Private function to read a 8bits register.
 * Parameters: reg. Resister witch we will get data.
 * Returns:    value. uint8 readed value.
 ******************************************************************************/
/* no used in our code
uint8 HalUViRead8(uint8 reg)
{  
  uint8 buffer = 0x55;
  bool success = FALSE;
  
  //call to sensor functions
  success = HalSensorReadReg(reg, &buffer, sizeof(buffer));
  
  if (success) 
    return buffer;
  else 
    return 0x55; // error   
}
*/


/******************************************************************************
 * Function:   Private function to read a 16bits register.
 * Parameters: reg. Resister witch we will get data.
 * Returns:    value. uint16 readed value.
 ******************************************************************************/
uint16 HalUViRead16(uint8 reg)
{
  uint8 buffer[2] = {0x55, 0x55};
  bool success = FALSE;
  
  //call to sensor functions
  success = HalSensorReadReg(reg, buffer, sizeof(buffer));
  
  if (success) 
    return BUILD_UINT16(buffer[0], buffer[1]);
  else 
    return 0x5555; // error   
}


/* -----------------------------------------------------------------------------
 *                               Public Functions
 * -----------------------------------------------------------------------------
 */


/******************************************************************************
 * Function:   Execute measurement step.
 * Parameters: None.
 * Returns:    Success.
 ******************************************************************************/
bool HalUViExecMeasurementStep(uint8 state)
{
//  uint32 delay = 0x00000000;
  bool success = FALSE;
  
  HalUViSelect();

  switch (state)
  {
    case HAL_UVI_MEAS_STATE_0: // Start measure
      /* Start sensor. Forced mode. */
      HalUViWakeUp();
// in the future do some like this:      
//      /* Wait x ms for ADC to complete - In Application*/
//      switch (_IntegrationTime)
//      {
//        case TSL2561_INTEGRATIONTIME_13MS:
//          delay = TSL2561_DELAY_INTTIME_13MS;  // KTOWN: Was 14ms
//          break;
//        case TSL2561_INTEGRATIONTIME_101MS:
//          delay = TSL2561_DELAY_INTTIME_101MS; // KTOWN: Was 102ms
//          break;
//        default:
//          delay = TSL2561_DELAY_INTTIME_402MS; // KTOWN: Was 403ms
//          break;
//      }
//
//      /* Launch HAL timer with delay*/
//      osal_start_timerEx (Hal_TaskID, HAL_LUX_EVENT, delay);
      break;

    case HAL_UVI_MEAS_STATE_1: // Fill structure with values values 
      /* oJo put in the interuption handled for UVi if implemented INT */
      /* Read UVi from Sensor */
      HalUViReadData();
 
      /* Launch OSAL event to registered application */
      
      /* Stop and sleep: Set the control bits to 0x00 */ 
      HalUViSleep();
 
      break;
  }

  return success;
}


/******************************************************************************
 * Function:   Get UViometer data.
 * Parameters: None
 * Returns:    Success read
 ******************************************************************************/
void double2Ints(double f, uint8 p, uint8 *i, uint8 *d)
{ 
  // f = float, p=decimal precision, i=integer, d=decimal
  uint8   li; 
  uint8   prec = 1;

  for(int x=p; x>0; x--) 
  {
    prec*=10;
  };  // same as power(10,p)

  li = (uint8) f;              // get integer part
  *d = (uint8) ((f-li)*prec);  // get decimal part
  *i = li;
}


bool HalUViReadMeasurement(uint8 *pBuf)
{
  float UVi_f = 0.0;
  uint8 fractpart, intpart;

  UVi_f = uv_call_example(si115x_handle, 
                          samples);
  
//  printf("UVi : %f\n", UVi_f);

  double2Ints(UVi_f, 1, &intpart, &fractpart);
  pBuf[0] = fractpart;
  pBuf[1] = intpart;
  
//  UINT32_TO_BUF_LITTLE_ENDIAN(pBuf, UVi_f);
  return 0;
}


bool HalUVluxReadMeasurement(uint8 *pBuf)
{
  float lux_f = 0.0;
  uint8 fractpart, intpart;

  lux_f = lux_call_example(si115x_handle, 
                           samples);

  double2Ints(lux_f, 2, &intpart, &fractpart);
  pBuf[0] = fractpart;
  pBuf[1] = intpart;
  
  return 0;
}
// Conversion float to integer reference code used.

// // The simplest way is just to treat it as a buffer.
// float x;
// i2c.write(addr, (char*)&x, sizeof(x));
// ...
// i2c.read(addr, (char*)&x, sizeof(x));
//
// // If you need to access each byte separately, use a union:
// union float2bytes { float f; char b[sizeof(float)]; };
// 
// float2bytes f2b;
// 
// float x;
// f2b.f = x;
// for ( int i=0; i < sizeof(float); i++ )
//   send_byte(f2b.b[i]);
// ...
// for ( int i=0; i < sizeof(float); i++ )
//   f2b.b[i] = read_byte();
// 
// x = f2b.f;
// // Note that you need to make sure the sender and receiver both use the same 
// // endianness and floating-point format.
                          
// Split float in integer and fractional
///* modf example */
//#include <stdio.h>      /* printf */
//#include <math.h>       /* modf */
//
//int main ()
//{
//  double param, fractpart, intpart;
//
//  param = 3.14159265;
//  fractpart = modf (param , &intpart);
//  printf ("%f = %f + %f \n", param, intpart, fractpart);
//  return 0;
//}
//
//Output: 3.141593 = 3.000000 + 0.141593


/******************************************************************************
 * Function:   Set the mode of the Loxometer Sensor.
 *          
 * Returns:    Nothing
 ******************************************************************************/
void HalUViSetMode(uint8 mode)
{
}


/******************************************************************************
 * Function:   This function initialize the TSL2561 with default values.
 * Parameters: None.
 * Returns:    None.
 ******************************************************************************/
void HalUViInit(void) 
{  
  HalUViTest();
  si115x_init(si115x_handle);
  // To start forced measurements         
  Si115xForce(si115x_handle);
}


/******************************************************************************
 * Function:   This function Test that the device is pressent.
 * Parameters: None.
 * Returns:    TRUE if pressent.
 ******************************************************************************/
bool HalUViTest(void) 
{  
  
//////////////////////////////////////////////////// begin debug context ///////
  HAL_DEBUG_STATEMENT(                                                         \
  /* statement context */                                                      \
  DEBUG_HAL_UVI | DEBUG_ASSERT,                                                \
  {                                                                            \
    /* test if exist Si1133 */                                                 \
    HalUViSelect();                                                            \
    if (0x33 == Si115xReadFromRegister(si115x_handle, SI115x_REG_PART_ID))     \
    {                                                                          \
      halDebugMsg(DEBUG_HAL_UVI | DEBUG_ASSERT,                                \
                  " SI1133 detected in 0x%02X\r\n", SI1133_ADDR);              \
      return TRUE;                                                             \
    }                                                                          \
                                                                               \
    halDebugMsg(DEBUG_HAL_UVI | DEBUG_ERROR, " SI1133 no detected\r\n");       \
    return FALSE;                                                              \
  });
////////////////////////////////////////////////////// end debug context ///////

return TRUE;    
}


/***************************************************************************//**
 * @brief
 *   Waits until the Si115x is sleeping before proceeding
 ******************************************************************************/
static int16_t _waitUntilSleep(HANDLE si115x_handle)
{
  int16_t retval = -1;
  uint8_t count = 0;
  // This loops until the Si115x is known to be in its sleep state
  // or if an i2c error occurs
  while(count < 5)
  {
    retval = Si115xReadFromRegister(si115x_handle, SI115x_REG_RESPONSE0);
    if((retval&RSP0_CHIPSTAT_MASK) == RSP0_SLEEP)
      break;
    if(retval <  0)
      return retval;
    count++;
  }
  return 0;
}

/***************************************************************************//**
 * @brief
 *   Resets the Si115x/6x, clears any interrupts and initializes the HW_KEY
 *   register.
 * @param[in] si115x_handle
 *   The programmer's toolkit handle
 * @retval  0
 *   Success
 * @retval  <0
 *   Error
 ******************************************************************************/
int16_t Si115xReset(HANDLE si115x_handle)
{
  int16_t retval = 0;

  // Do not access the Si115x earlier than 25 ms from power-up.
  // Uncomment the following lines if Si115xReset() is the first
  // instruction encountered, and if your system MCU boots up too
  // quickly.
  Si115xDelay_10ms();
  Si115xDelay_10ms();
  Si115xDelay_10ms();
  
  // Perform the Reset Command
  retval += Si115xWriteToRegister(si115x_handle, SI115x_REG_COMMAND, 1);

  // Delay for 10 ms. This delay is needed to allow the Si115x
  // to perform internal reset sequence.
  Si115xDelay_10ms();

  return retval;
}

/***************************************************************************//**
 * @brief
 *   Helper function to send a command to the Si113x/4x
 ******************************************************************************/
static int16_t _sendCmd(HANDLE si115x_handle, uint8_t command)
{
  int16_t  response;
  int8_t   retval;
  uint8_t  count = 0;

  // Get the response register contents
  response = Si115xReadFromRegister(si115x_handle, SI115x_REG_RESPONSE0);
  if(response < 0)
  {
    return response;
  }

  response = response & RSP0_COUNTER_MASK;

  // Double-check the response register is consistent
  while(count < 5)
  {
    if((retval = _waitUntilSleep(si115x_handle)) != 0)
      return retval;

    if(command == 0)
      break; // Skip if the command is NOP

    retval = Si115xReadFromRegister(si115x_handle, SI115x_REG_RESPONSE0);

    if((retval&RSP0_COUNTER_MASK) == response)
      break;
    else if(retval < 0)
      return retval;
    else
      response = retval & RSP0_COUNTER_MASK;

    count++;
  } // end loop

  // Send the Command
  if((retval = (Si115xWriteToRegister(si115x_handle, SI115x_REG_COMMAND, command))
                != 0))
  {
    return retval;
  }

  count = 0;
  // Expect a change in the response register
  while(count < 5)
  {
    if(command == 0)
      break; // Skip if the command is NOP

    retval = Si115xReadFromRegister(si115x_handle, SI115x_REG_RESPONSE0);
    if((retval & RSP0_COUNTER_MASK) != response)
      break;
    else if(retval < 0)
      return retval;

    count++;
  } // end loop

  return 0;
}

/***************************************************************************//**
 * @brief
 *   Sends a NOP command to the Si115x/6x
 * @param[in] si115x_handle
 *   The programmer's toolkit handle
 * @retval  0
 *   Success
 * @retval  <0
 *   Error
 ******************************************************************************/
int16_t Si115xNop(HANDLE si115x_handle)
{
  return _sendCmd(si115x_handle, CMD_NOP);
}

/***************************************************************************//**
 * @brief
 *   Sends a FORCE command to the Si115x/6x
 * @param[in] si115x_handle
 *   The programmer's toolkit handle
 * @retval  0
 *   Success
 * @retval  <0
 *   Error
 ******************************************************************************/
int16_t Si115xForce(HANDLE si115x_handle)
{
  return _sendCmd(si115x_handle, CMD_FORCE_CH);
}

/***************************************************************************//**
 * @brief
 *   Sends a PSALSAUTO command to the Si113x/4x
 * @param[in] si115x_handle
 *   The programmer's toolkit handle
 * @retval  0
 *   Success
 * @retval  <0
 *   Error
 ******************************************************************************/
int16_t Si115xStart (HANDLE si115x_handle)
{
  return _sendCmd(si115x_handle, CMD_AUTO_CH);
}

/***************************************************************************//**
 * @brief
 *   Reads a Parameter from the Si115x/6x
 * @param[in] si115x_handle
 *   The programmer's toolkit handle
 * @param[in] address
 *   The address of the parameter.
 * @retval <0
 *   Error
 * @retval 0-255
 *   Parameter contents
 ******************************************************************************/
int16_t Si115xParamRead(HANDLE si115x_handle, uint8_t address)
{
  // returns Parameter[address]
  int16_t retval;
  uint8_t cmd = CMD_PARAM_QUERY + (address & 0x3F);

  retval=_sendCmd(si115x_handle, cmd);
  if( retval != 0 )
  {
    return retval;
  }
  retval = Si115xReadFromRegister(si115x_handle, SI115x_REG_RESPONSE1);
  return retval;
}

/***************************************************************************//**
 * @brief
 *   Writes a byte to an Si115x/6x Parameter
 * @param[in] si115x_handle
 *   The programmer's toolkit handle
 * @param[in] address
 *   The parameter address
 * @param[in] value
 *   The byte value to be written to the Si113x/4x parameter
 * @retval 0
 *   Success
 * @retval <0
 *   Error
 * @note This function ensures that the Si115x/6x is idle and ready to
 * receive a command before writing the parameter. Furthermore,
 * command completion is checked. If setting parameter is not done
 * properly, no measurements will occur. This is the most common
 * error. It is highly recommended that host code make use of this
 * function.
 ******************************************************************************/
int16_t Si115xParamSet(HANDLE si115x_handle, uint8_t address, uint8_t value)
{
  int16_t retval;
  uint8_t buffer[2];
  int16_t response_stored;
  int16_t response;

  if((retval = _waitUntilSleep(si115x_handle))!=0)
  {
    return retval;
  }

  response_stored = RSP0_COUNTER_MASK
                    & Si115xReadFromRegister(si115x_handle,
                                             SI115x_REG_RESPONSE0);

  buffer[0] = value;
  buffer[1] = CMD_PARAM_SET + (address & 0x3F);

  retval = Si115xBlockWrite(si115x_handle,
                            SI115x_REG_HOSTIN0,
                            2,
                            (uint8_t*) buffer);
  if(retval != 0)
    return retval;

  // Wait for command to finish
  response = Si115xReadFromRegister(si115x_handle, SI115x_REG_RESPONSE0);
  while((response & RSP0_COUNTER_MASK) == response_stored)
  {
    response = Si115xReadFromRegister(si115x_handle, SI115x_REG_RESPONSE0);
  }

  if(retval < 0)
    return retval;
  else
    return 0;
}

/***************************************************************************//**
 * @brief
 *   Pause measurement helper function
 ******************************************************************************/
static int16_t _Pause(HANDLE si115x_handle)
{
  return _sendCmd(si115x_handle, CMD_PAUSE_CH);
}

/***************************************************************************//**
 * @brief
 *   Pauses autonomous measurements
 * @param[in] si115x_handle
 *  The programmer's toolkit handle
 * @retval  0
 *   Success
 * @retval  <0
 *   Error
 ******************************************************************************/
int16_t Si115xPause(HANDLE si115x_handle)
{
  uint8_t countA, countB;
  int8_t  retval;

  //  After a RESET, if the Si115x receives a command (including NOP) before
  //  the Si115x has gone to sleep, the chip hangs. This first while loop
  //  avoids this.  The reading of the REG_RESPONS0 does not disturb
  //  the internal MCU.

  retval = 0; // initialize data so that we guarantee to enter the loop
  while((RSP0_CHIPSTAT_MASK & retval) != RSP0_SLEEP)
  {
    retval = Si115xReadFromRegister(si115x_handle, SI115x_REG_RESPONSE0);
  }

  countA = 0;
  while(countA < 5)
  {
    countB = 0;
    // Keep sending nops until the response is zero
    while(countB < 5)
    {
      retval = Si115xReadFromRegister(si115x_handle, SI115x_REG_RESPONSE0);
      if((retval & RSP0_COUNTER_MASK) == 0)
        break;
      else
      {
        // Send the NOP Command to clear any error...we cannot use
        // Si115xNop() because it first checks if REG_RESPONSE < 0 and
        // if so it does not perform the cmd. Since we have a saturation
        // REG_RESPONSE will be < 0
        Si115xWriteToRegister(si115x_handle, SI115x_REG_COMMAND, 0x00);
      }
      countB++;
    } // end inner loop

    // Pause the device
    _Pause(si115x_handle);

    countB = 0;
    // Wait for response
    while(countB < 5)
    {
      retval = Si115xReadFromRegister(si115x_handle, SI115x_REG_RESPONSE0);
      if((retval & RSP0_COUNTER_MASK) != 0)
        break;
      countB++;
    }

    // When the PsAlsPause() response is good, we expect it to be a '1'.
    retval = Si115xReadFromRegister(si115x_handle, SI115x_REG_RESPONSE0);
    if((retval&RSP0_COUNTER_MASK) == 1 )
      break;  // otherwise, start over.
    countA++;
  } // end outer loop
  return 0;
}

/***************************************************************************//**
 * @brief
 *   Writes a byte to an Si115x/6x Parameter
 * @param[in] si115x_handle
 *   The programmer's toolkit handle
 * @param[in] param_addr
 *   The parameter address
 * @param[in] param_value
 *   The byte value to be written to the Si115x/6x parameter
 * @retval 0
 *   Success
 * @retval <0
 *   Error
 * @note This function ensures that the Si115x/6x is idle and ready to
 * receive a command before writing the parameter. Furthermore,
 * command completion is checked. If setting parameter is not done
 * properly, no measurements will occur. This is the most common
 * error. It is highly recommended that host code make use of this
 * function.
 ******************************************************************************/
void SetParam(HANDLE si115x_handle, uint8_t param_addr, uint8_t param_value)
{
  uint8_t temp;

  temp = Si115xReadFromRegister(si115x_handle, SI115x_REG_RESPONSE0);
  while(!(temp & 0x20))
  {
    // wait for device to sleep
    temp = Si115xReadFromRegister(si115x_handle, SI115x_REG_RESPONSE0);
  }
  if(temp & 0x10)
  {
    // if error code is present, NOP to clear
    Si115xWriteToRegister(si115x_handle, SI115x_REG_COMMAND, CMD_NOP);
    while(temp & 0xDF)
    {
      // wait for device to sleep and clear
      temp = Si115xReadFromRegister(si115x_handle, SI115x_REG_RESPONSE0);
    } // end loop
  } // end if

  Si115xWriteToRegister(si115x_handle,
                        SI115x_REG_HOSTIN0,
                        param_value);
  Si115xWriteToRegister(si115x_handle,
                        SI115x_REG_COMMAND,
                        CMD_PARAM_SET
                        | param_addr);
  while( (temp & 0x1f)
         == (Si115xReadFromRegister(si115x_handle, SI115x_REG_RESPONSE0)
         & 0x1f) )
  {
    // Do Nothing 
    ;
  }
}

/***************************************************************************//**
 * @brief
 *   Reads a Parameter from the Si115x/6x
 * @param[in] si115x_handle
 *   The programmer's toolkit handle
 * @param[in] param_addr
 *   The address of the parameter.
 * @retval <0
 *   Error
 * @retval 0-255
 *   Parameter contents
 ******************************************************************************/
uint8_t QueryParam(HANDLE si115x_handle, uint8_t param_addr)
{
  uint8_t temp;

  temp = Si115xReadFromRegister(si115x_handle, SI115x_REG_RESPONSE0);
  while(!(temp & 0x20))
  {
    // wait for device to sleep
    temp = Si115xReadFromRegister(si115x_handle, SI115x_REG_RESPONSE0);
  }
  if(temp & 0x10)
  {
    // if error code is present, NOP to clear
    Si115xWriteToRegister(si115x_handle, SI115x_REG_COMMAND, CMD_NOP);
    while(temp & 0xDF)
    {
      // wait for device to sleep and clear
      temp = Si115xReadFromRegister(si115x_handle, SI115x_REG_RESPONSE0);
    }
  }
  Si115xWriteToRegister(si115x_handle,
                        SI115x_REG_COMMAND,
                        (CMD_PARAM_QUERY | param_addr));
  while((temp & 0x1f)
         == (Si115xReadFromRegister(si115x_handle, SI115x_REG_RESPONSE0)
             & 0x1f))
  {
    // Do Nothing 
    ;
  }
  return Si115xReadFromRegister(si115x_handle, SI115x_REG_HOSTOUT0);
}

/***************************************************************************//**
 * @brief
 *   Helper function to send a command to the Si113x/4x
 ******************************************************************************/
uint8_t SendCmd(HANDLE si115x_handle, uint8_t cmd)
{
  uint8_t temp;

  temp = Si115xReadFromRegister(si115x_handle, SI115x_REG_RESPONSE0);
  while(!(temp & 0x20))
  {
    // wait for device to sleep
    temp = Si115xReadFromRegister(si115x_handle, SI115x_REG_RESPONSE0);
  }
  if(temp & 0x10)
  {
    // if error code is present, NOP to clear
    Si115xWriteToRegister(si115x_handle, SI115x_REG_COMMAND, CMD_NOP);
    while(temp & 0xDF)
    {
      // wait for device to sleep and clear
      temp = Si115xReadFromRegister(si115x_handle, SI115x_REG_RESPONSE0);
    }
  }
  Si115xWriteToRegister(si115x_handle, SI115x_REG_COMMAND, cmd);
  while((temp & 0x1f)
         == (Si115xReadFromRegister(si115x_handle, SI115x_REG_RESPONSE0)
             & 0x1f));
  {
    // Do Nothing 
    ;
  }
  return Si115xReadFromRegister(si115x_handle, SI115x_REG_RESPONSE0);
}



////////////////////////////////////////////////////////////////////////////////
// UVi example with diffuser
////////////////////////////////////////////////////////////////////////////////


int16_t si115x_init( HANDLE si115x_handle )
{
    int16_t    retval;            

    retval  = Si115xReset( si115x_handle ); 
    Si115xDelay_10ms(); 

    // view meaning in "Si1133-780640.pdf" datasheet pag 27, and section 7.
    // intialization for forcedmode. for autonomous mode configure threshold
    // and activate interruptions.
#if defined CALCULATE_LUX
    retval += Si115xParamSet( si115x_handle, PARAM_CH_LIST, 0x0f); 
    retval += Si115xParamSet( si115x_handle, PARAM_ADCCONFIG0, 0x78); 
    retval += Si115xParamSet( si115x_handle, PARAM_ADCSENS0, 0x09); 
    retval += Si115xParamSet( si115x_handle, PARAM_ADCPOST0, 0x40); 
    retval += Si115xParamSet( si115x_handle, PARAM_ADCCONFIG1, 0x4d); 
    retval += Si115xParamSet( si115x_handle, PARAM_ADCSENS1, 0x52); 
    retval += Si115xParamSet( si115x_handle, PARAM_ADCPOST1, 0x40); 
    retval += Si115xParamSet( si115x_handle, PARAM_ADCCONFIG2, 0x41); 
    retval += Si115xParamSet( si115x_handle, PARAM_ADCSENS2, 0x61); 
    retval += Si115xParamSet( si115x_handle, PARAM_ADCPOST2, 0x50); 
    retval += Si115xParamSet( si115x_handle, PARAM_ADCCONFIG3, 0x4d); 
    retval += Si115xParamSet( si115x_handle, PARAM_ADCSENS3, 0x07); 
    retval += Si115xParamSet( si115x_handle, PARAM_ADCPOST3, 0x40); 
    retval += Si115xWriteToRegister( si115x_handle, SI115x_REG_IRQ_ENABLE, 0x0f);  
#elif defined CALCULATE_UV
    retval += Si115xParamSet( si115x_handle, PARAM_CH_LIST, 0x01); 
    retval += Si115xParamSet( si115x_handle, PARAM_ADCCONFIG0, 0x78); 
    retval += Si115xParamSet( si115x_handle, PARAM_ADCSENS0, 0x09); 
    retval += Si115xParamSet( si115x_handle, PARAM_ADCPOST0, 0x40); 
    retval += Si115xWriteToRegister( si115x_handle, SI115x_REG_IRQ_ENABLE, 0x01); 
#endif 
    return retval;
}

//
// To start forced measurements         
//     Si115xForce( si115x_handle)      
//
                              
                                                  
void si115x_handler(HANDLE si115x_handle,         
                    SI115X_SAMPLES *samples)      
{      
  // to HalUViReadData()
}
#define X_ORDER_MASK 0x0070
#define Y_ORDER_MASK 0x0007
#define SIGN_MASK    0x0080
#define get_x_order(m)   ( (m & X_ORDER_MASK) >> 4 )
#define get_y_order(m)   ( (m & Y_ORDER_MASK)      )
#define get_sign(m)      ( (m & SIGN_MASK   ) >> 7 )

typedef struct {
  int16_t     info;
  uint16_t    mag;
} COEFF;

int32_t poly_inner( int32_t input,
               int8_t  fraction,
               uint16_t mag,
               int8_t  shift)
{    
  if (shift < 0)
  {
    return ( ( input << fraction ) / mag ) >> -shift ;
  }
  else
  {
    return ( ( input << fraction ) / mag ) << shift  ;
  }    
}

int32_t eval_poly( int32_t x, 
               int32_t y, 
               uint8_t input_fraction, 
               uint8_t output_fraction,
               uint8_t num_coeff, 
               COEFF  *kp
             )
{
  uint8_t  info, x_order, y_order, counter;
  int8_t   sign, shift;
  uint16_t mag;
  int32_t  output=0, x1, x2, y1, y2;

  for(counter=0; counter < num_coeff; counter++) 
  {
    info    = kp->info;
    x_order = get_x_order(info);
    y_order = get_y_order(info);

    shift   = ((uint16_t)kp->info&0xff00)>>8;
    shift  ^= 0x00ff;
    shift  += 1;
    shift   = -shift;    
    
    mag     = kp->mag;
    
    if( get_sign(info) ) sign = -1;
    else                 sign = 1;

    if( (x_order==0) && (y_order==0) )
    {
      output += sign * mag << output_fraction;
    }
    else
    {
      if( x_order > 0 )
      {
        x1 = poly_inner( x, input_fraction, mag, shift);
        if ( x_order > 1 )
        {
          x2 = poly_inner( x, input_fraction, mag, shift);
        }
        else
          x2 = 1;
      }
      else { x1 = 1; x2 = 1; }

      if( y_order > 0 )
      {
        y1 = poly_inner( y, input_fraction, mag, shift);
        if ( y_order > 1 )
        {
          y2 = poly_inner( y, input_fraction, mag, shift);
        }
        else
          y2 = 1;
      }
      else
      { y1 = 1; y2 = 1; }

      output += sign * x1 * x2 * y1 * y2;
    }
    kp++;
  }
  if( output < 0 ) output = -output;
  return output;
}

//UV calculation///////////////////////////////////////////////////////////////
//
// Initialize UV coefficients
//
COEFF uk[2] = { {1537, 27440}, {2, 59952} };

#define UV_INPUT_FRACTION       15
#define UV_OUTPUT_FRACTION      12
#define UV_NUMCOEFF             2

//
// This is the main entry point for computing uv. The value returned by 
// get_uv is scaled by UV_OUTPUT_FRACTION
// 
// In order to get uvi as an integer, do this:
//
//   uvi = get_uv(uv, uk) / ( 1 << UV_OUTPUT_FRACTION )  
//
int32_t get_uv ( int32_t uv, 
                 COEFF *uk)
{
    int32_t uvi;

    uvi = eval_poly( 0, 
                     uv, 
                     UV_INPUT_FRACTION, 
                     UV_OUTPUT_FRACTION,
                     UV_NUMCOEFF,
                     uk );
    return uvi;
}

//
// General steps:
//
// 1. Initialize the Si115x
//
// 2. Initiate a conversion by using si115x_force()
//
// 3. The interrupt causes the interrupt handler to fill the
//    SI115X_SAMPLES structure
//
// 4. The example_calling_routine picks up data from the
//    SI115X_SAMPLES structure and calls the get_uv()
//    routine to compute the uvi
//
float uv_call_example( HANDLE si115x_handle, 
                       SI115X_SAMPLES *samples )
{
  float uvi;

  //
  // Example conversion to human-readable uvi values
  //
  uvi = (float) get_uv( samples->ch0, uk);
  uvi = uvi / ( 1 << UV_OUTPUT_FRACTION );

  return uvi;
}

//lux calculation//////////////////////////////////////////////////////////////
#if defined CALCULATE_LUX
typedef struct {
  COEFF   coeff_high[4];
  COEFF   coeff_low[9];
} LUX_COEFF; 

//
// Initialize coefficients
//
LUX_COEFF lk ={ { {0, 107},           // coeff_high[0]
                  {1665, 105},        // coeff_high[1]
                  {2064, 110},        // coeff_high[2]
                  {-1519, 168} },     // coeff_high[3]
                { {0, 0},             // coeff_low[0]
                  {1665, 17351},      // coeff_low[1]
                  {-1022, 54619},     // coeff_low[2]
                  {2064, 19277},      // coeff_low[3]
                  {-879, 37075},      // coeff_low[4]
                  {-1902, 53304},     // coeff_low[5]
                  {-864, 34111},      // coeff_low[6]
                  {-1759, 52507},     // coeff_low[7]
                  {-2398, 36626} } }; // coeff_low[8]

#define ADC_THRESHOLD           22000
#define INPUT_FRACTION_HIGH     7
#define INPUT_FRACTION_LOW      15
#define LUX_OUTPUT_FRACTION     12
#define NUMCOEFF_LOW            9
#define NUMCOEFF_HIGH           4

//
// This is the main entry point for computing lux. The value returned by 
// get_lux is scaled by LUX_OUTPUT_FRACTION
// 
// In order to get lux as an integer, do this:
//
//   lux = get_lux(vis_high, vis_low, ir, &lk) / ( 1 << LUX_OUTPUT_FRACTION )  
//
int32_t get_lux( int32_t vis_high, 
                 int32_t vis_low, 
                 int32_t ir, 
                 LUX_COEFF *lk)
{
  int32_t lux;

  if( (vis_high > ADC_THRESHOLD) || (ir > ADC_THRESHOLD) ) 
  {
    lux = eval_poly( vis_high, 
                     ir, 
                     INPUT_FRACTION_HIGH, 
                     LUX_OUTPUT_FRACTION,
                     NUMCOEFF_HIGH,
                     &(lk->coeff_high[0]) );
  }
  else
  {
    lux = eval_poly( vis_low, 
                     ir, 
                     INPUT_FRACTION_LOW, 
                     LUX_OUTPUT_FRACTION,
                     NUMCOEFF_LOW,
                     &(lk->coeff_low[0]) );
  }
  return lux;
}

//
// General steps:
//
// 1. Initialize the Si115x
//
// 2. Initiate a conversion by using si115x_force()
//
// 3. The interrupt causes the interrupt handler to fill the
//    SI115X_SAMPLES structure
//
// 4. The example_calling_routine picks up data from the
//    SI115X_SAMPLES structure and calls the get_lux()
//    routine to compute the lux
//
float lux_call_example( HANDLE si115x_handle, 
                        SI115X_SAMPLES *samples )
{
  float lux;

  //
  // Example conversion to human-readable lux values
  //
  lux = (float) get_lux( samples->ch1,
                         samples->ch3,
                         samples->ch2, 
                         &lk);
  lux = lux / ( 1 << LUX_OUTPUT_FRACTION );

  return lux;
}
#endif // CALCULATE_LUX

#endif // (defined HAL_UVI) && (HAL_UVI == TRUE)

/*******************************************************************************
  Filename:       hal_lux.c
  Revised:        $Date: 2016-03-30 11:45:31$
  Revision:       $Revision: 0 $

  Description:    This file contains the declaration of TSL2561 T package
                  light-to-digital sensor abstraction layer. Auto-Gain support, 
                  and added lux clipping check (returns 0 lux on saturation)
*******************************************************************************/

/* -----------------------------------------------------------------------------
 *                                 Includes
 * -----------------------------------------------------------------------------
 */

#include "hal_board.h"
#include "hal_weather.h"
#include "hal_sensor.h"
#include "hal_i2c.h"
#include "hal_lux.h"
#include "hal_debug.h"
   
#if (defined HAL_LUX) && (HAL_LUX == TRUE)

/* -----------------------------------------------------------------------------
 *                                 Constants
 * -----------------------------------------------------------------------------
 */

#define TSL2561_DELAY_INTTIME_13MS    (15)
#define TSL2561_DELAY_INTTIME_101MS   (120)
#define TSL2561_DELAY_INTTIME_402MS   (450)

/* -----------------------------------------------------------------------------
 *                              Local Functions
 * -----------------------------------------------------------------------------
 */
// INTERNAL CONTROL FUNCTIONS
static void HalLuxSelect(void);

static void HalLuxWakeUp(void);
static void HalLuxSleep(void);

//static void   enable  (void);
//static void   disable (void);
static void   HalLuxWrite8(uint8 reg, uint8 value);
//static uint8  HalLuxRead8(uint8 reg);
static uint16 HalLuxRead16(uint8 reg);
static void   HalLuxReadData(void);
#if defined (LUX_CALC)
static uint32 HalLuxCalculateLux(uint16 BB, uint16 IR);
#endif

// INTERNAL COMPENSATION FUNCTIONS
static void setGain(Gain_t gain);
static void setIntegrationTime(IntegrationTime_t time);
static void enableAutoRange(bool enable);

/* -----------------------------------------------------------------------------
 *                               Local Variables
 * -----------------------------------------------------------------------------
 */

//int8     _addr = TSL2561_ADDR_FLOAT;
bool  _Initialised = false;
bool  _AutoGain = false;
IntegrationTime_t  _IntegrationTime = TSL2561_INTEGRATIONTIME_13MS;
Gain_t   _Gain = TSL2561_GAIN_16X;
//  _tsl2561SensorID = sensorID;

//static uint8 usr;                         // Keeps user register value
//static uint8 buf[4] = {0x44, 0x44, 0x44, 0x44}; // Data buffer
static bool  success;
uint16 BB, IR;
#if defined (LUX_CALC)
uint32 LUX;
#endif



/* -----------------------------------------------------------------------------
 *                              Private functions
 * -----------------------------------------------------------------------------
 */

/******************************************************************************
 * Function:  This function select the TSL2561 lux sensor device on the I2C bus.
 * Returns:   Nothing
 ******************************************************************************/
static void HalLuxSelect(void)
{
  //Set up I2C that is used to communicate with TSL2561
  HalI2CInit(TSL2561_ADDR_FLOAT, i2cClock_267KHZ);
}

/******************************************************************************
 * Function:  This function enables the device
 * Returns:   Nothing
 ******************************************************************************/
void HalLuxWakeUp(void)
{
  /* Enable the device by setting the control bit to 0x03 */
  HalLuxWrite8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWERON);
}

/******************************************************************************
 * Function:  Disables the device (putting it in lower power sleep mode)
 * Returns:   Nothing
 ******************************************************************************/
void HalLuxSleep(void)
{
  /* Turn the device off to save power */
  HalLuxWrite8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_CONTROL, TSL2561_CONTROL_POWEROFF);
}

/******************************************************************************
 * Function:   Private function to read luminosity on both channels.
 * Parameters: pBB. Pointer to BroadBand data
 *             pIR. Pointer to Infra Red data              
 * Returns:    Nothing
 ******************************************************************************/
void HalLuxReadData(void)
{
//   uint16 BB, IR;
  /* Start temperature read: Set the control bits to 0x03 */
//  HalLuxWakeUp();//no es necesari aqui, lo hace en step
//      
//  /* Wait x ms for ADC to complete */
//  switch (_IntegrationTime)
//  {
//    case TSL2561_INTEGRATIONTIME_13MS:
//      delay = TSL2561_DELAY_INTTIME_13MS;  // KTOWN: Was 14ms
//      break;
//    case TSL2561_INTEGRATIONTIME_101MS:
//      delay = TSL2561_DELAY_INTTIME_101MS; // KTOWN: Was 102ms
//      break;
//    default:
//      delay = TSL2561_DELAY_INTTIME_402MS; // KTOWN: Was 403ms
//      break;
//  }
//
//  /* Launch HAL timer with delay*/
//  osal_start_timerEx(Hal_TaskID, HAL_LUX_EVENT, delay);
      
  /* Reads a two byte value from channel 0 (visible + infrared) */
//  *pBB = HalLuxRead16(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN0_LOW);
  BB = HalLuxRead16(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN0_LOW);

  /* Reads a two byte value from channel 1 (infrared) */
//  *pIR = HalLuxRead16(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN1_LOW);
  IR = HalLuxRead16(TSL2561_COMMAND_BIT | TSL2561_WORD_BIT | TSL2561_REGISTER_CHAN1_LOW);
  
//  buf[0]=LO_UINT16(BB);
//  buf[1]=HI_UINT16(BB);
//  buf[2]=LO_UINT16(IR);
//  buf[3]=HI_UINT16(IR);

  
  /* Turn the device off to save power */
//  HalLuxSleep();//no es necesari aqui, lo hace en step
}




/******************************************************************************
 * Function:   Private function to write a 8bits register.
 * Parameters: reg. Destin resister to write to.
 *             value. Value to write.              
 * Returns:    Nothing
 ******************************************************************************/
void HalLuxWrite8(uint8 reg, uint8 value)
{
  //call to sensor functions
  HalSensorWriteReg(reg, &value, sizeof(value)); 
}

/******************************************************************************
 * Function:   Private function to read a 8bits register.
 * Parameters: reg. Resister witch we will get data.
 * Returns:    value. uint8 readed value.
 ******************************************************************************/
uint8 HalLuxRead8(uint8 reg)
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

/******************************************************************************
 * Function:   Private function to read a 16bits register.
 * Parameters: reg. Resister witch we will get data.
 * Returns:    value. uint16 readed value.
 ******************************************************************************/
uint16 HalLuxRead16(uint8 reg)
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
 *                               Public unctions
 * -----------------------------------------------------------------------------
 */


/******************************************************************************
 * Function:   Execute measurement step.
 * Parameters: None.
 * Returns:    Success.
 ******************************************************************************/
bool HalLuxExecMeasurementStep(uint8 state)
{
//  uint32 delay = 0x00000000;
  
  HalLuxSelect();

  switch (state)
  {
    case HAL_LUX_MEAS_STATE_0: // Start measure
      /* Start temperature read: Set the control bits to 0x03 */
      HalLuxWakeUp();
      
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

    case HAL_LUX_MEAS_STATE_1: // Read and store BB, IR and LUX values 
      /* Read and store BB and IR values */
      HalLuxReadData();
      
#if defined (LUX_CALC)
      /* Calculate and store LUX value */
      LUX = HalLuxCalculateLux(BB, IR);
#endif
      /* Launch OSAL event to registered application */
      
      
      /* Stop and sleep: Set the control bits to 0x00 */ 
      HalLuxSleep();
 
      break;

  }

  return success;
}


/******************************************************************************
 * Function:   Get luxometer data.
 * Parameters: None
 * Returns:    Success read
 ******************************************************************************/
bool HalLuxReadMeasurement(uint8 *pBuf)
{
  // Broadband Data
//  pBuf[0] = buf[1];
//  pBuf[1] = buf[0];
  pBuf[0] = LO_UINT16(BB);
  pBuf[1] = HI_UINT16(BB);

  // InfraRed Data
//  pBuf[2] = buf[4];
//  pBuf[3] = buf[3];
  pBuf[2] = LO_UINT16(IR);
  pBuf[3] = HI_UINT16(IR);

#if defined (LUX_CALC)
  // Lux Data
  pBuf[4] = BREAK_UINT32(LUX, 0);
  pBuf[5] = BREAK_UINT32(LUX, 1);
  pBuf[6] = BREAK_UINT32(LUX, 2);
  pBuf[7] = BREAK_UINT32(LUX, 3);
#else  
  // Lux Data
  pBuf[4] = 0x44;
  pBuf[5] = 0x44;
  pBuf[6] = 0x44;
  pBuf[7] = 0x44;
#endif  

//  return success;
  return TRUE;
}

/******************************************************************************
 * Function:   Set the mode of the Loxometer Sensor.
 * Parameters: mode.- Loxometer data- 3 bits - bits(0-2)
 *                  - Luxometer mode- 1 bit  - bit(3)
 *                  - Luxometer gain- 2 bits - bits(4-5)
 *                  - Luxometer time- 2 bits - bits(6-7)
 *          
 * Returns:    Nothing
 ******************************************************************************/
void HalLuxSetMode(uint8 mode)
{
  uint8  luxMode = mode;
  
  switch (luxMode & ~(TSL2561_MODE_DATA_MASK << TSL2561_MODE_DATA_POS))
  {
  case TSL2561_MODE_EN_LUX << TSL2561_MODE_DATA_POS:
////////////////////////////
//Completa esto co    
    break;

  default:
    // Should not get here
    break;
  }
}

/******************************************************************************
 * Function:   This function initialize the TSL2561 with default values.
 * Parameters: None.
 * Returns:    None.
 ******************************************************************************/
void HalLuxInit(void) 
{
  
  // Set default values
  HalLuxSetMode( TSL2561_MODE_EN_LUX     << TSL2561_MODE_DATA_POS |
                 TSL2561_MODE_MONITORING << TSL2561_MODE_CTRL_POS |
                 TSL2561_MODE_AUTO_GAIN  << TSL2561_MODE_GAIN_POS |
                 TSL2561_MODE_TIME_101MS << TSL2561_MODE_TIME_POS);
  
  // I2C init for this sensor
  HalLuxSelect();

  // WakeUp the lux sensor
  HalLuxWakeUp();
  
//////////////////////////////////////////////////// begin debug context ///////
  HAL_DEBUG_STATEMENT(                                                         \
  /* statement context */                                                      \
  DEBUG_HAL_LUX | DEBUG_ASSERT,                                                \
  {                                                                            \
    /* test if exist TSL2561 */                                                \
    uint8 usr = HalLuxRead8(TSL2561_REGISTER_ID);                              \
    if (usr & 0x0A)                                                            \
      halDebugMsg(DEBUG_HAL_LUX | DEBUG_ASSERT,                                \
                  " TSL2561 detected in 0x%02X\r\n", TSL2561_ADDR_FLOAT);      \
    else                                                                       \
      halDebugMsg(DEBUG_HAL_LUX | DEBUG_ERROR, " TSL2561 no detected\r\n");    \
  });
////////////////////////////////////////////////////// end debug context ///////
  
  // *** Configure TSL2561 ***
  // Set integration time
  setIntegrationTime(_IntegrationTime);
  // Set gain
  setGain(_Gain);
  // Set auto range mode
  enableAutoRange(FALSE);
    
  _Initialised = true;
  
  /* Note: by default, the device is in power down mode on bootup */
  // Sleep the sensor
  HalLuxSleep();
    
}
  
/**************************************************************************/
/*!
    @brief  Enables or disables the auto-gain settings when reading
            data from the sensor
*/
/**************************************************************************/

void enableAutoRange(bool enable)
{
   _AutoGain = enable ? true : false;
}

/**************************************************************************/
/*!
    Sets the integration time for the TSL2561
*/
/**************************************************************************/
void setIntegrationTime(IntegrationTime_t time)
{
//podemos eliminarlo, redundante
//  if (!_Initialised) HalLuxInit();

  /* Enable the device by setting the control bit to 0x03 */
  HalLuxWakeUp();

  /* Update the timing register */
  HalLuxWrite8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_TIMING, (uint8)time | (uint8)_Gain);

  /* Update value placeholders */
  _IntegrationTime = time;

  /* Turn the device off to save power */
  HalLuxSleep();
}

/**************************************************************************/
/*!
    Adjusts the gain on the TSL2561 (adjusts the sensitivity to light)
*/
/**************************************************************************/
extern void setGain(Gain_t gain)
{
//  if (!_Initialised) HalLuxInit();

  /* Enable the device by setting the control bit to 0x03 */
  HalLuxWakeUp();
  
  /* Update the timing register */
  HalLuxWrite8(TSL2561_COMMAND_BIT | TSL2561_REGISTER_TIMING, (uint8)_IntegrationTime | (uint8)gain);

  /* Update value placeholders */
  _Gain = gain;

  /* Turn the device off to save power */
  HalLuxSleep();
}

/******************************************************************************
 * Function:   Gets the broadband (mixed lighting) and IR only values from 
 *             the TSL2561, adjusting gain if auto-gain is enabled.
 * Parameters: pBB. Pointer to buffer to store the broadband data.
 *             pIR. Pointer to buffer to store the broadband data.
 * Returns:    Nothing.
 ******************************************************************************/
//void HalLuxGetLuminosity (uint16_t *pBB, uint16_t *pIR)
//{
//  bool valid = false;
//
//    
//  
//  
//  
//
//
//
//  if (!_tsl2561Initialised) begin();
//
//  /* If Auto gain disabled get a single reading and continue */
//  if(!_tsl2561AutoGain)
//  {
//    getData (broadband, ir);
//    return;
//  }
//
//  /* Read data until we find a valid range */
//  bool _agcCheck = false;
//  do
//  {
//    uint16_t _b, _ir;
//    uint16_t _hi, _lo;
//    tsl2561IntegrationTime_t _it = _tsl2561IntegrationTime;
//
//    /* Get the hi/low threshold for the current integration time */
//    switch(_it)
//    {
//      case TSL2561_INTEGRATIONTIME_13MS:
//        _hi = TSL2561_AGC_THI_13MS;
//        _lo = TSL2561_AGC_TLO_13MS;
//        break;
//      case TSL2561_INTEGRATIONTIME_101MS:
//        _hi = TSL2561_AGC_THI_101MS;
//        _lo = TSL2561_AGC_TLO_101MS;
//        break;
//      default:
//        _hi = TSL2561_AGC_THI_402MS;
//        _lo = TSL2561_AGC_TLO_402MS;
//        break;
//    }
//
//    getData(&_b, &_ir);
//
//    /* Run an auto-gain check if we haven't already done so ... */
//    if (!_agcCheck)
//    {
//      if ((_b < _lo) && (_tsl2561Gain == TSL2561_GAIN_1X))
//      {
//        /* Increase the gain and try again */
//        setGain(TSL2561_GAIN_16X);
//        /* Drop the previous conversion results */
//        getData(&_b, &_ir);
//        /* Set a flag to indicate we've adjusted the gain */
//        _agcCheck = true;
//      }
//      else if ((_b > _hi) && (_tsl2561Gain == TSL2561_GAIN_16X))
//      {
//        /* Drop gain to 1x and try again */
//        setGain(TSL2561_GAIN_1X);
//        /* Drop the previous conversion results */
//        getData(&_b, &_ir);
//        /* Set a flag to indicate we've adjusted the gain */
//        _agcCheck = true;
//      }
//      else
//      {
//        /* Nothing to look at here, keep moving ....
//           Reading is either valid, or we're already at the chips limits */
//        *broadband = _b;
//        *ir = _ir;
//        valid = true;
//      }
//    }
//    else
//    {
//      /* If we've already adjusted the gain once, just return the new results.
//         This avoids endless loops where a value is at one extreme pre-gain,
//         and the the other extreme post-gain */
//      *broadband = _b;
//      *ir = _ir;
//      valid = true;
//    }
//  } while (!valid);
//}

/**************************************************************************/
/*!
    Converts the raw sensor values to the standard SI lux equivalent.
    Returns 0 if the sensor is saturated and the values are unreliable.
*/
/**************************************************************************/

/******************************************************************************
 * Function:   Converts the raw sensor values to the standard SI lux equivalent.
 * Parameters: BB.- BroadBand raw data.
 *             IR.- InfraRed raw data.
 * Returns:    Luxes in an uint32 variable.  Returns 0 if the sensor is 
 *             saturated and the values are unreliable.
 *             1. Error in comunication.
 ******************************************************************************/
#if defined (LUX_CALC)
uint32 HalLuxCalculateLux(uint16 BB, uint16 IR)
{
  unsigned long chScale;
  unsigned long channel1;
  unsigned long channel0;  
  
  /* Make sure the sensor isn't saturated! */
  uint16 clipThreshold;
  switch (_IntegrationTime)
  {
    case TSL2561_INTEGRATIONTIME_13MS:
      clipThreshold = TSL2561_CLIPPING_13MS;
      break;
    case TSL2561_INTEGRATIONTIME_101MS:
      clipThreshold = TSL2561_CLIPPING_101MS;
      break;
    default:
      clipThreshold = TSL2561_CLIPPING_402MS;
      break;
  }

  /* Return 0 lux if the sensor is saturated */
  if ((BB > clipThreshold) || (IR > clipThreshold))
  {
    return 0;
  }

  /* Get the correct scale depending on the intergration time */
  switch (_IntegrationTime)
  {
    case TSL2561_INTEGRATIONTIME_13MS:
      chScale = TSL2561_LUX_CHSCALE_TINT0;
      break;
    case TSL2561_INTEGRATIONTIME_101MS:
      chScale = TSL2561_LUX_CHSCALE_TINT1;
      break;
    default: /* No scaling ... integration time = 402ms */
      chScale = (1 << TSL2561_LUX_CHSCALE);
      break;
  }

  /* Scale for gain (1x or 16x) */
  if (!_Gain) chScale = chScale << 4;

  /* Scale the channel values */
  channel0 = (BB * chScale) >> TSL2561_LUX_CHSCALE;
  channel1 = (IR * chScale) >> TSL2561_LUX_CHSCALE;

  /* Find the ratio of the channel values (Channel1/Channel0) */
  unsigned long ratio1 = 0;
  if (channel0 != 0) ratio1 = (channel1 << (TSL2561_LUX_RATIOSCALE+1)) / channel0;

  /* round the ratio value */
  unsigned long ratio = (ratio1 + 1) >> 1;

  unsigned int b, m;

#ifdef TSL2561_PACKAGE_CS
  if (ratio <= TSL2561_LUX_K1C)
    {b=TSL2561_LUX_B1C; m=TSL2561_LUX_M1C;}
  else if (ratio <= TSL2561_LUX_K2C)
    {b=TSL2561_LUX_B2C; m=TSL2561_LUX_M2C;}
  else if (ratio <= TSL2561_LUX_K3C)
    {b=TSL2561_LUX_B3C; m=TSL2561_LUX_M3C;}
  else if (ratio <= TSL2561_LUX_K4C)
    {b=TSL2561_LUX_B4C; m=TSL2561_LUX_M4C;}
  else if (ratio <= TSL2561_LUX_K5C)
    {b=TSL2561_LUX_B5C; m=TSL2561_LUX_M5C;}
  else if (ratio <= TSL2561_LUX_K6C)
    {b=TSL2561_LUX_B6C; m=TSL2561_LUX_M6C;}
  else if (ratio <= TSL2561_LUX_K7C)
    {b=TSL2561_LUX_B7C; m=TSL2561_LUX_M7C;}
  else if (ratio > TSL2561_LUX_K8C)
    {b=TSL2561_LUX_B8C; m=TSL2561_LUX_M8C;}
#else
  if (ratio <= TSL2561_LUX_K1T)
    {b=TSL2561_LUX_B1T; m=TSL2561_LUX_M1T;}
  else if (ratio <= TSL2561_LUX_K2T)
    {b=TSL2561_LUX_B2T; m=TSL2561_LUX_M2T;}
  else if (ratio <= TSL2561_LUX_K3T)
    {b=TSL2561_LUX_B3T; m=TSL2561_LUX_M3T;}
  else if (ratio <= TSL2561_LUX_K4T)
    {b=TSL2561_LUX_B4T; m=TSL2561_LUX_M4T;}
  else if (ratio <= TSL2561_LUX_K5T)
    {b=TSL2561_LUX_B5T; m=TSL2561_LUX_M5T;}
  else if (ratio <= TSL2561_LUX_K6T)
    {b=TSL2561_LUX_B6T; m=TSL2561_LUX_M6T;}
  else if (ratio <= TSL2561_LUX_K7T)
    {b=TSL2561_LUX_B7T; m=TSL2561_LUX_M7T;}
  else if (ratio > TSL2561_LUX_K8T)
    {b=TSL2561_LUX_B8T; m=TSL2561_LUX_M8T;}
#endif

  unsigned long temp;
  temp = ((channel0 * b) - (channel1 * m));

  /* Do not allow negative lux value */
//  if (temp < 0) temp = 0;

  /* Round lsb (2^(LUX_SCALE-1)) */
  temp += (1 << (TSL2561_LUX_LUXSCALE-1));

  /* Strip off fractional portion */
  uint32 lux = temp >> TSL2561_LUX_LUXSCALE;

  /* Signal I2C had no errors */
  return lux;
}
#endif

#endif //(defined HAL_AIR) && (HAL_AIR == TRUE)

/**************************************************************************************************
  Filename:       hal_acc.c
  Revised:        $Date: 2016-03-23 11:45:31$
  Revision:       $Revision: 0 $

  Description:    This file contains the declaration to the Accelerometer HAL BMA250 
                  abstraction layer.
**************************************************************************************************/

/* ------------------------------------------------------------------------------------------------
*                                          Includes
* ------------------------------------------------------------------------------------------------
*/

#include "hal_board.h"
#include "hal_acc.h"
#include "hal_sensor.h"
#include "hal_i2c.h"
#include "hal_debug.h"

#if (defined HAL_ACC) && (HAL_ACC == TRUE)

/* ------------------------------------------------------------------------------------------------
*                                           Constants
* ------------------------------------------------------------------------------------------------
*/

/* ------------------------------------------------------------------------------------------------
*                                           Typedefs
* ------------------------------------------------------------------------------------------------
*/

/* ------------------------------------------------------------------------------------------------
*                                           Macros
* ------------------------------------------------------------------------------------------------
*/

/* ------------------------------------------------------------------------------------------------
*                                       Local Functions
* ------------------------------------------------------------------------------------------------
*/
static void HalI2CAccSelect(void);

static void HalAccWakeUp(void);
static void HalAccSleep(void);



/* ------------------------------------------------------------------------------------------------
*                                       Local Variables
* ------------------------------------------------------------------------------------------------
*/
// Default Accelerometer configuration values
static uint8 accSensorPM_Wakeup = ACC_PM_ON;
static uint8 accSensorPM_Sleep = ACC_PM_LP | ACC_PM_SLEEP_10MS;
static uint8 accSensorRange = HAL_ACC_RANGE_2G;
static uint8 accBandwidth = ACC_BW_250HZ;

// oJo estudiar esto, no funciona bien la interrupción INT2
//Electrical characteristics for INT1 and INT2 push-pull low level activated.
//static uint8 accIntElecBe = ((accIntElecBe & ~(ACC_INT1_LVL | ACC_INT2_LVL)) | (ACC_INT1_OD | ACC_INT2_OD)) ; 
static uint8 accIntElecBe = 0x0A; //0b00001010
static uint8 accIntTimeBe = ACC_INT_TEMP_1S; 
//static uint8 accIntScr = ACC_INT_SRC_TAP_UNFILT; //Scr for Tap unfiltered

//Interruption activation
static uint8 accInt_En0 = ACC_INT_D_TAP_EN | ACC_INT_ORIENT_EN ; // Bit in register 0x16
//static uint8 accInt_En0 = ACC_INT_D_TAP_EN; // Bit in register 0x16
static uint8 accInt_En1 = ACC_INT_DATA_EN; // Bit in register 0x17

//Interrupt Mapping 0x19-0x1A-0x1B registers
// INT1 -> P1.7 -> KEY_1 --> Simple Tap
static uint8 accIntMap0 = ACC_INT_MAP_D_TAP; // 0x19 register
// INT2 -> P1.6 -> KEY_2 --> New Data
//static uint8 accIntMap1 = ACC_INT2_MAP_DATA; // 0x1A register


//static uint8 accShadow = 0x40; 



/**************************************************************************************************
* @fn          HalAccInit
*
* @brief       This function initializes the HAL Accelerometer abstraction layer.
*
* @return      None.
*/
void HalAccInit(void)
{
//////////////////////////////////////////////////// begin debug context ///////
  HAL_DEBUG_STATEMENT(                                                         \
  /* statement context */                                                      \
  DEBUG_HAL_ACC | DEBUG_ASSERT,                                                \
  {                                                                            \
    uint8 usr = 0x00;                                                          \
    /* test if exist BME250 */                                                 \
    HalI2CReadDirect(HAL_BMA250_I2C_ADDRESS, ACC_CHIPID, &usr, 1);             \
    if (usr & 0x03)                                                            \
      halDebugMsg(DEBUG_HAL_ACC | DEBUG_ASSERT,                                \
             " BME250 detected in 0x%02X\r\n", HAL_BMA250_I2C_ADDRESS);        \
    else                                                                       \
    {                                                                          \
      halDebugMsg(DEBUG_HAL_ACC | DEBUG_ERROR, " BME250 not detected\r\n");     \
      HAL_DEBUG_ERROR();                                                       \
    }                                                                          \
  });
////////////////////////////////////////////////////// end debug context ///////
  
  // Set default values
  HalAccSetRange(HAL_ACC_RANGE_2G);
  HalAccSetPM(ACC_PM_LP, ACC_PM_SLEEP_10MS);
  
  // I2C init for this sensor
  HalI2CAccSelect();

  //*** Enable Accelerometer power ***
//  P2_0=1;
  // Wait 2ms for accelerometer to power up and settle
  ST_HAL_DELAY(250);
  
  // *** Configure accelerometer ***
  // Set range
  HalSensorWriteReg(ACC_RANGE, &accSensorRange, sizeof(accSensorRange));

  // Set Interrupt pin time behaviour
  HalSensorWriteReg(ACC_INT_TIME_BEHAVIOR, &accIntTimeBe, sizeof(accIntTimeBe));

  // Set Interrupt pin electrical behaviour
  HalSensorWriteReg(ACC_INT_PIN_BEHAVIOR, &accIntElecBe, sizeof(accIntElecBe));

  // Set Interrupt select source '0'('1') filtered(unfiltered)
//  HalSensorWriteReg(ACC_INT_SOURCE, &accIntScr, sizeof(accIntScr));

  // Set Interrupt mapping to INT1 INT2 pis
  HalSensorWriteReg(ACC_INT_MAPPING0, &accIntMap0, sizeof(accIntMap0));
//  HalSensorWriteReg(ACC_INT_MAPPING1, &accIntMap1, sizeof(accIntMap1));

  
  
  
  // Enable Interruptions
  HalSensorWriteReg(ACC_INT_ENABLE0, &accInt_En0, sizeof(accInt_En0));
  HalSensorWriteReg(ACC_INT_ENABLE1, &accInt_En1, sizeof(accInt_En1));

  
  
  
  
  
  
  // Filter detection bandwidth (Time between samples = 1/(2*filter_bw))
  HalSensorWriteReg(ACC_BW, &accBandwidth, sizeof(accBandwidth));

  // Disamble shadow mode. Permit read MSB without read LSB before.
//  HalSensorWriteReg(ACC_CONF_FILT_SHADOW, &accShadow, sizeof(accShadow));
  
  // Set PowerMode
  HalSensorWriteReg(ACC_PM, &accSensorPM_Sleep, sizeof(accSensorPM_Sleep));
}

/**************************************************************************************************
* @fn          HalAccWakeUp
*
* @brief       This function WakeUp I2C Accelerometer.
*
* @return      None.
*/
void HalAccWakeUp(void)
{
  // I2C init for this sensor
  HalI2CAccSelect();

  // *** Configure accelerometer ***
  
  // Set PowerMode
  HalSensorWriteReg(ACC_PM, &accSensorPM_Wakeup, sizeof(accSensorPM_Wakeup));

}

/**************************************************************************************************
* @fn          HalAccSleep
*
* @brief       This function Sleep I2C Accelerometer.
*
* @return      None.
*/
void HalAccSleep(void)
{
  // I2C init for this sensor it is no necessary, I2C for Accekerometer arlready selected
//  HalI2CAccSelect();

  // *** Configure accelerometer ***
  
  // Set PowerMode
  HalSensorWriteReg(ACC_PM, &accSensorPM_Sleep, sizeof(accSensorPM_Sleep));

}

/**************************************************************************************************
* @fn          HalAccSetRange
*
* @brief       Set the range of the accelerometer
*
* @param       range: HAL_ACC_RANGE_2G, HAL_ACC_RANGE_4G, HAL_ACC_RANGE_8G
*
* @return      None
*/
void HalAccSetRange(uint8 range)
{
  uint8 accRange = range;
  
  switch (accRange)
  {
  case HAL_ACC_RANGE_2G:
    accSensorRange = ACC_RANGE_2G; // in Register 0x0F
    break;
  case HAL_ACC_RANGE_4G:
    accSensorRange = ACC_RANGE_4G;
    break;
  case HAL_ACC_RANGE_8G:
    accSensorRange = ACC_RANGE_8G;
    break;
  case HAL_ACC_RANGE_16G:
    accSensorRange = ACC_RANGE_16G;
    break;
  default:
    // Should not get here
    break;
  }
}

/**************************************************************************************************
* @fn          HalAccSetPM
*
* @brief       Set the PowerMode of the accelerometer
*
* @param       mode: ACC_PM_SUSP, ACC_PM_LP
*              sleep_time: ACC_PM_SLEEP_10MS, ACC_PM_SLEEP_25MS, ACC_PM_SLEEP_50MS
*
* @return      None
*/
void HalAccSetPM(uint8 mode, uint8 sleep_time)
{
  uint8 accMode = mode | sleep_time;
//  accSleepTime = sleep_time;
  
  switch (accMode)
  {
  case ACC_PM_ON:
    accSensorPM_Sleep = ACC_PM_ON; //Never sleep
    break;
  case ACC_PM_SUSP:
    accSensorPM_Sleep = ACC_PM_SUSP; 
    break;
  case ACC_PM_LP | ACC_PM_SLEEP_10MS:
    accSensorPM_Sleep = ACC_PM_LP | ACC_PM_SLEEP_10MS;
    break;
  case ACC_PM_LP | ACC_PM_SLEEP_25MS:
    accSensorPM_Sleep = ACC_PM_LP | ACC_PM_SLEEP_25MS;
    break;
  case ACC_PM_LP | ACC_PM_SLEEP_50MS:
    accSensorPM_Sleep = ACC_PM_LP | ACC_PM_SLEEP_50MS;
    break;
  default:
    // Should not get here
    break;
  }
}

/**************************************************************************************************
* @fn          HalAccGetXYZ
*
* @brief       Read data from the accelerometer - X, Y, Z - 3 bytes
*
* @return      TRUE if valid data, FALSE if not
*/
bool HalAccGetXYZ(uint8 *pBuf )
{
  uint8 x;
  uint8 y;
  uint8 z;
  bool success;

  // Seled Address and Speed of I2C for Accelerometer
  HalI2CAccSelect();

  // WakeUp Accelerometer
  HalAccWakeUp();

  // Read the three registers
  success = HalSensorReadReg( ACC_X_MSB, &x, sizeof(x));
  if (success)
  {
    success = HalSensorReadReg( ACC_Y_MSB, &y, sizeof(y));
    if (success)
    {
       success = HalSensorReadReg( ACC_Z_MSB, &z, sizeof(z));
    }
  }

  if (success)
  {
    // Valid data
    pBuf[0] = x;
    pBuf[1] = y;
    pBuf[2] = z;
  }

  // Sleep Accelerometer 
  HalAccSleep();

  return success;
}

/**************************************************************************************************
* @fn          HalAccGetAll
*
* @brief       Read All data from the accelerometer - X, Y, Z, Temperature and Orientation - 5 bytes
*
* @return      TRUE if valid data, FALSE if not
*/
bool HalAccGetAll(uint8 *pBuf )
{
   
//  uint8 orientation;
  uint8 rData[11];
        
  bool success;

  // Seled Address and Speed of I2C for Accelerometer
  HalI2CAccSelect();

  // WakeUp Accelerometer
  HalAccWakeUp();

  // Wait for measurement ready (appx. 1.45 ms)
  ST_HAL_DELAY(180);
    
  // Read 7 cosecutive registers from ACC_X_LSB. 
  success = HalSensorReadReg( ACC_X_LSB, rData, sizeof(rData));

  if (success)
  {
    // Valid data
    pBuf[0] = rData[1]; //get MSB X data
    pBuf[1] = rData[3]; //get MSB Y data
    pBuf[2] = rData[5]; //get MSB Z data
    pBuf[3] = rData[6]; //get Temperature data
    pBuf[4] = rData[10]; //get Orientation data
  }
  
  // Sleep Accelerometer 
  HalAccSleep();

  return success;
}

/**************************************************************************************************
* @fn          HalAccGetOrientation
*
* @brief       Read Orientation from the accelerometer
*
* @return      TRUE if valid data, FALSE if not
*/
bool HalAccGetOrientation(uint8 *pBuf )
{
  uint8 orientation;
  bool success;

  // Seled Address and Speed of I2C for Accelerometer
  HalI2CAccSelect();

  // WakeUp Accelerometer
  HalAccWakeUp();

  // Read the three registers
  success = HalSensorReadReg( ACC_ORIENTATION, &orientation, sizeof(orientation));

  if (success)
  {
    // Valid data
    pBuf[0] = orientation;
    pBuf[1] = 0x00;
    pBuf[2] = 0x00;
    pBuf[3] = 0x00;
    pBuf[4] = 0x00;
  }

  // Sleep Accelerometer 
  HalAccSleep();

  return success;
}

/**************************************************************************************************
* @fn          HalAccReadX
*
* @brief       Read X data from the accelerometer
*
* @return      TRUE if valid data, FALSE if not
*/
bool HalAccGetReadX(uint8 *pBuf )
{
  uint8 x;
  bool success;

  // Seled Address and Speed of I2C for Accelerometer
  HalI2CAccSelect();

  // WakeUp Accelerometer
  HalAccWakeUp();

  // Read the three registers
  success = HalSensorReadReg( ACC_X_MSB, &x, sizeof(x));

  if (success)
  {
    // Valid data
    pBuf[0] = x;
    pBuf[1] = 1;
    pBuf[2] = 2;
  }

  // Sleep Accelerometer 
  HalAccSleep();

  return success;
}


/**************************************************************************************************
* @fn          HalAccGetID
*
* @brief       Read data ID register
*
* @return      TRUE if valid data, FALSE if not
*/
bool HalAccGetID(uint8 *pBuf )
{
  uint8 id;
  bool success;

  // Seled Address and Speed of I2C for Accelerometer
  HalI2CAccSelect();

  // WakeUp Accelerometer
  HalAccWakeUp();

  // Read ID register
  success = HalSensorReadReg( ACC_CHIPID, &id, sizeof(id));

  if (success)
  {
    // Valid data
    pBuf[0] = id;
    pBuf[1] = 0x0E;
    pBuf[2] = 0xFE;
  }

  // Sleep Accelerometer 
  HalAccSleep();

  return success;
}

/**************************************************************************************************
* @fn          HalAccGetTemp
*
* @brief       Read data Acc Internal Temperature
*
* @return      TRUE if valid data, FALSE if not
*
* (0x08) temp register is given in two`s complement. The slope of the temperature
* sensor are 0,5K/LSB, its center temperature is 24ºC. Therefore, the typical
* temperature measurement range is -40ºC up to 87,5ºC.
*/
bool HalAccGetTemp(uint8 *pBuf )
{
  uint8 temp;
  bool success;

  // Seled Address and Speed of I2C for Accelerometer
  HalI2CAccSelect();

  // WakeUp Accelerometer
  HalAccWakeUp();

  // Read ID register
  success = HalSensorReadReg( ACC_TEMP, &temp, sizeof(temp));

  if (success)
  {
    // Valid data
    pBuf[0] = temp;
    pBuf[1] = 0x0E;
    pBuf[2] = 0xFE;
  }
  
  // Sleep Accelerometer 
  HalAccSleep();

  return success;
}

/**************************************************************************************************
 * @fn          HalAccTest
 *
 * @brief       Run a sensor self-test
 *
 * @return      TRUE if passed, FALSE if failed
 */
bool HalAccTest(void)
{
//  uint8 val;
//
//  // Select this sensor on the I2C bus
//  HalAccSelect();
//
//  // Check the DCST_RESP (pattern 0x55)
//  ST_ASSERT(HalSensorReadReg(ACC_REG_ADDR_DCST_RESP, &val, 1));
//  ST_ASSERT(val==0x55);
//
//  // Check the DCST_RESP (pattern 0xAA)
//  val = 0x10;     // Sets the DCST bit
//  ST_ASSERT(HalSensorWriteReg(ACC_REG_ADDR_CTRL_REG3, &val, 1));
//  ST_ASSERT(HalSensorReadReg(ACC_REG_ADDR_DCST_RESP, &val, 1));
//  ST_ASSERT(val==0xAA);
//
//  // Check the WHO AM I register
//  ST_ASSERT(HalSensorReadReg(ACC_REG_ADDR_WHO_AM_I, &val, 1));
//  ST_ASSERT(val==REG_VAL_WHO_AM_I);

  return TRUE;
}

/* ------------------------------------------------------------------------------------------------
*                                           Private functions
* -------------------------------------------------------------------------------------------------
*/

/**************************************************************************************************
* @fn          HalAccSelect
*
* @brief       Select the accelerometer on the I2C-bus
*
* @return
*/
static void HalI2CAccSelect(void)
{
  //Set up I2C that is used to communicate
  HalI2CInit(HAL_BMA250_I2C_ADDRESS,i2cClock_267KHZ);
}

/*  Conversion algorithm for X, Y, Z
 *  ================================
 *
float calcAccel(int8 rawX, uint8 range)
{
    float v;

    switch (range)
    {
      case HAL_ACC_RANGE_2G:
      //-- calculate acceleration, unit G, range -2, +2
      v = (rawX * 1.0) / (256/4);
      break;

      case HAL_ACC_RANGE_4G:
      //-- calculate acceleration, unit G, range -4, +4
      v = (rawX * 1.0) / (256/8);
      break;

      case HAL_ACC_RANGE_4G:
      //-- calculate acceleration, unit G, range -8, +8
      v = (rawX * 1.0) / (256/16);
      break;    
    }
    return v;
}
*/


/*********************************************************************
*********************************************************************/

#endif // (defined HAL_ACC) && (HAL_ACC == TRUE)

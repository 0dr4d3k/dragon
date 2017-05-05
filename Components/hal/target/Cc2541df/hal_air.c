/*******************************************************************************
  Filename:       hal_air.h
  Revised:        $Date: 2016-03-14 11:45:31$
  Revision:       $Revision: 0 $

  Description:    This file contains the declaration Indoor Air Quality (IAQ) to 
                  the HAL CCS811 abstraction layer.
*******************************************************************************/

/* -----------------------------------------------------------------------------
*                                  Includes
* ------------------------------------------------------------------------------
*/

#include "hal_board.h"
#include "hal_air.h"
#include "hal_sensor.h"
#include "hal_i2c.h"
#include "hal_debug.h"

#if (defined HAL_AIR) && (HAL_AIR == TRUE)

/* -----------------------------------------------------------------------------
*                                  Constants
* ------------------------------------------------------------------------------
*/

/* -----------------------------------------------------------------------------
*                                  Typedefs
* ------------------------------------------------------------------------------
*/

/* -----------------------------------------------------------------------------
*                                   Macros
* ------------------------------------------------------------------------------
*/

/* -----------------------------------------------------------------------------
*                               Local Functions
* ------------------------------------------------------------------------------
*/
static void HalI2CAirSelect(void);

static void HalAirWakeUp(void);
static void HalAirSleep(void);



/* -----------------------------------------------------------------------------
*                               Local Variables
* ------------------------------------------------------------------------------
*/
// Default Accelerometer configuration values
static uint8 airSensorPM_WakeUp = AIR_MODE1; // Mode1: Constant power, IAQ every 1s. 
static uint8 airSensorPM_Sleep = AIR_MODE1;   
//static uint8 airSensorPM_Sleep = AIR_MODE0;   // Mode0: Idle (Measurements are disabled).
//                             | AIR_INTERRUPT //Activate physical INT interrupt.
//                             | AIR_THRESH;   //Activate physical thresold

static uint8 airSensor_ENV_DATA[4] = {0x00,0x64,0x00,0x64}; // oJo, comprobar si es little endian   
static uint8 airSensor_AIR_BASELINE[2] = {0x00,0x64};       // oJo, comprobar si es little endian
uint8 index=0;


/*******************************************************************************
* @fn          HalAirInit
*
* @brief       This function initializes the HAL CCS811 Indoor Air Quality (IAQ) 
*              abstraction layer.
*
* @return      None.
*/
void HalAirInit(void)
{
  //*** WakeUp Air Sensor ***
  // set as 0: General-purpose I/O; 1: Peripheral function     
  HAL_AIR_WAKE_SEL &= (uint8) ~HAL_AIR_WAKE_BV;
  
  // set as OUTPUT port
  HAL_AIR_WAKE_DDR |= (uint8) HAL_AIR_WAKE_BV;
   
  // and activate nWAKE
  HAL_AIR_WAKE_SBIT = 0;  

  // Wait 500us after I2C trasmission
  ST_HAL_DELAY(62);

//////////////////////////////////////////////////// begin debug context ///////
  HAL_DEBUG_STATEMENT(                                                         \
  /* statement context */                                                      \
  DEBUG_HAL_AIR | DEBUG_ASSERT,                                                \
  {                                                                            \
    uint8 usr = 0x00;                                                          \
    /* test if exist CCS811 */                                                 \
    HalI2CReadDirect(HAL_CCS811_I2C_ADDRESS, AIR_HW_ID, &usr, 1);              \
    if (usr & AIR_HW_ID_VALUE)                                                 \
      halDebugMsg(DEBUG_HAL_LUX | DEBUG_ASSERT,                                \
             " CCS811 detected in 0x%02X\r\n", HAL_CCS811_I2C_ADDRESS);        \
    else                                                                       \
      halDebugMsg(DEBUG_HAL_LUX | DEBUG_ERROR, " CCS811 not detected\r\n");    \
  });
////////////////////////////////////////////////////// end debug context ///////

  uint8 dummy=0; 
  
  // Microcontroler I2C init for this sensor
  HalI2CAirSelect();

  // Set default values
//  HalAirSetPM(AIR_MODE0); //Low power sleep no measures
 
  // *** Configure Air Sensor ***
  // Write ENV_DATA. Default value.
  HalSensorWriteReg(AIR_ENV_DATA, airSensor_ENV_DATA, sizeof(airSensor_ENV_DATA));

  // Write AIR_BASELINE. Default value. 
  HalSensorWriteReg(AIR_BASELINE, airSensor_AIR_BASELINE, sizeof(airSensor_AIR_BASELINE));

  // Set default mode
  HalSensorWriteReg(AIR_MEAS_MODE, &airSensorPM_Sleep, sizeof(airSensorPM_Sleep));
//  HalAirSleep();

  // Start the application
  HalSensorWriteReg(AIR_APP_START, &dummy, sizeof(dummy));

  HAL_AIR_WAKE_SBIT=1;
}


/*******************************************************************************
* @fn          HalAirWakeUp
*
* @brief       This function WakeUp I2C Indoor Air Quality (IAQ) sensor.
*
* @return      None.
*/
void HalAirWakeUp(void)
{
  // Activate nWAKE sensor pin mapped to HAL_AIR_WAKE_SBIT CC2541 pin
  //Arreglar esto en hal_borad_cfg.h

  //*** WakeUp Air Sensor ***
  HAL_AIR_WAKE_SBIT=0;
  // Wait 500us after I2C trasmission
  ST_HAL_DELAY(62);
  
  // I2C init for this sensor
  HalI2CAirSelect();

  // *** Configure AirSensor in Wakeup Mode ***
  // Set PowerMode
  HalSensorWriteReg(AIR_MEAS_MODE, &airSensorPM_WakeUp, sizeof(airSensorPM_WakeUp));
}


/*******************************************************************************
* @fn          HalAirSleep
*
* @brief       This function Sleep I2C Indoor Air Quality (IAQ) sensor.
*
* @return      None.
*/
void HalAirSleep(void)
{
// I2C init for this sensor it is no necessary, I2C for Accekerometer arlready selected
//  HalI2CAccSelect();

  //*** Ensure that Air Sensor is WakeUp ***
  HAL_AIR_WAKE_SBIT=0;
  // Wait 500us after I2C trasmission
  ST_HAL_DELAY(62);
  
  // *** Configure AirSensor in Sleep Mode ***
  // Set PowerMode
  HalSensorWriteReg(AIR_MEAS_MODE, &airSensorPM_Sleep, sizeof(airSensorPM_Sleep));

  //*** Sleep Air Sensor ***
  // Deactivate nWAKE sensor pin mapped to HAL_AIR_WAKE_SBIT CC2541 pin
  HAL_AIR_WAKE_SBIT=1;
}


/*******************************************************************************
* @fn          HalAirSetPM
*
* @brief       Set the PowerMode of the Air Sensor
*
* @param       mode: Power Mode
*
* @return      None
*/
void HalAirSetPM(uint8 mode)
{

  airSensorPM_WakeUp = (airSensorPM_WakeUp & AIR_MODE_MASK) | mode;
  airSensorPM_Sleep = (airSensorPM_Sleep & AIR_MODE_MASK) | AIR_MODE0;
}


/*******************************************************************************
* @fn          HalAirGetID
*
* @brief       Read data ID register
*
* @param       pBuf: buffer to allocate the data
*
* @return      TRUE if valid data, FALSE if not
*/
bool HalAirGetID(uint8 *pBuf)
{
//  uint8 data[16];
//  uint8 id=0,hw_id=0,error=0,mode=0,data=0;
  bool success=1;

  // WakeUp Sensor
//  HalAirWakeUp();
  //*** WakeUp Air Sensor ***
  HAL_AIR_WAKE_SBIT=0;
  // Wait 500us after I2C trasmission
  ST_HAL_DELAY(62);
  
  // I2C init for this sensor
  HalI2CAirSelect();

  // *** Configure AirSensor in Wakeup Mode ***
  // Set PowerMode
  HalSensorWriteReg(AIR_MEAS_MODE, &airSensorPM_WakeUp, sizeof(airSensorPM_WakeUp));
  
  // Read ID register 
  success &= HalSensorReadReg(index, pBuf+1, 8);
  pBuf[0]=index;
  index++;

  //  success &= HalSensorReadReg( AIR_HW_ID, raw, sizeof(raw));

  if (success)
  {
    // Valid data
//    pBuf[0] = id;
  }
  else
  {
    // Error data
    pBuf[0] = 0xAA;
    pBuf[1] = 0xAA;
    pBuf[2] = 0xAA;
    pBuf[3] = 0xAA;
//    pBuf[4] = 0x41;
//    pBuf[5] = 0x41;
  }

  // Sleep Accelerometer 
//  HalAirSleep();
  // *** Configure AirSensor in Sleep Mode ***
  // Set PowerMode
//  HalSensorWriteReg(AIR_MEAS_MODE, &airSensorPM_Sleep, sizeof(airSensorPM_Sleep));

  //*** Sleep Air Sensor ***
  // Deactivate nWAKE sensor pin mapped to HAL_AIR_WAKE_SBIT CC2541 pin
  HAL_AIR_WAKE_SBIT=1;

  return success;
}


/*******************************************************************************
* @fn          HalAirGetReg
*
* @brief       Read data from register
*
* @param       reg: regites to read
*              pBuf: pointer to allocate the data
*
* @return      TRUE if valid data, FALSE if not
*/
bool initialize=FALSE;
bool HalAirGetReg(uint8 reg, uint8 *pBuf )
{
  uint8 i2c_buff[8]={0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA};
  uint8 le=1;

//  uint8 thresholds_data[5]={0x05, 0xDC, 0x09, 0xC4, 0x32};
//  uint8 env_data[4]={0x64, 0x00, 0x64, 0x00};
//  uint8 baseline_data[2]={0x64, 0x00};
//  uint8 reset_data[4]={0x11, 0xE5, 0x72, 0x8A};
  bool success=TRUE;

  // WakeUp Sensor
//  HalAirWakeUp();
  //*** WakeUp Air Sensor ***
  HAL_AIR_WAKE_SBIT=0;
  // Wait 500us after I2C trasmission
  ST_HAL_DELAY(62);
  
  // I2C init for this sensor
  HalI2CAirSelect();

  //oJo, porque no llevar esto a la funcion de inicializacion?
  if (!initialize)
  {    
    // Read HD_ID. Check ID 
    HalSensorReadReg(AIR_HW_ID, i2c_buff, 1);
    if(!(i2c_buff[0] & AIR_HW_ID_VALUE)) 
      HAL_TURN_ON_LED2(); //Error  

    // Read STATUS. Check STATUS is App Valid
    HalSensorReadReg(AIR_STATUS, i2c_buff, 1);
    if(!(i2c_buff[0] & AIR_APP_VALID)) 
      HAL_TURN_ON_LED2(); //Error  
 
    // Write APP_START
    HalSensorWriteReg(AIR_APP_START, &success, sizeof(success));

    // Read STATUS. Check STATUS is App Valid & Firmarwe in App Mode
    HalSensorReadReg(AIR_STATUS, i2c_buff, 1);
    if(!(i2c_buff[0] & (AIR_APP_VALID | AIR_FW_MODE))) 
      HAL_TURN_ON_LED2(); //Error

    // Write MEAS_MODE. Write drive mode and interrupt
    HalSensorWriteReg(AIR_MEAS_MODE, &airSensorPM_WakeUp, sizeof(airSensorPM_WakeUp));

    // Write ENV_DATA. Default value.
    HalSensorWriteReg(AIR_ENV_DATA, airSensor_ENV_DATA, sizeof(airSensor_ENV_DATA));

    // Write AIR_BASELINE. Default value. 
    HalSensorWriteReg(AIR_BASELINE, airSensor_AIR_BASELINE, sizeof(airSensor_AIR_BASELINE));

    initialize=TRUE;
  }  
  
//  HalSensorWriteReg(AIR_SW_RESET, reset_data, sizeof(reset_data));
  
  // *** Configure AirSensor in Wakeup Mode ***
  // Set PowerMode
//  HalSensorWriteReg(AIR_APP_START, sizeof(success), &success);
//  HalSensorWriteReg(AIR_MEAS_MODE, sizeof(airSensorPM_WakeUp), &airSensorPM_WakeUp);
//  HalSensorWriteReg(AIR_ENV_DATA, sizeof(airSensor_ENV_DATA), airSensor_ENV_DATA);
//  HalSensorWriteReg(AIR_THRESHOLDS, sizeof(thresholds_data), thresholds_data);
//  HalSensorWriteReg(AIR_BASELINE, sizeof(airSensor_AIR_BASELINE), airSensor_AIR_BASELINE);
  
//  HalI2CWriteDirect(HAL_CCS811_I2C_ADDRESS, AIR_APP_START, i2c_buff, 0);  
//  ST_HAL_DELAY(62);

//HalI2CWriteDirect(HAL_CCS811_I2C_ADDRESS, AIR_STATUS, i2c_buff, 0);
//HalI2CWriteDirect(HAL_CCS811_I2C_ADDRESS, i2c_buff, 1);
//if(!(i2c_buff[0] & 0x10)) HAL_TOGGLE_LED2();
//HalI2CWriteDirect(HAL_CCS811_I2C_ADDRESS, AIR_APP_START, i2c_buff, 0);
//
//
//  i2c_buff[0] = AIR_MODE2;
//  HalI2CWriteDirect(HAL_CCS811_I2C_ADDRESS, AIR_MEAS_MODE, i2c_buff, 1);  
//
//HalI2CWriteDirect(HAL_CCS811_I2C_ADDRESS, AIR_STATUS, i2c_buff, 0);
//HalI2CReadDirect(HAL_CCS811_I2C_ADDRESS,i2c_buff, 1);
//if(i2c_buff[0] & 0x8) // check if data ready
//{
//HAL_TOGGLE_LED1();
//HalI2CWriteDirect(HAL_CCS811_I2C_ADDRESS, AIR_ALG_RESULT_DATA, i2c_buff, 0);
//HalI2CReadDirect(HAL_CCS811_I2C_ADDRESS, i2c_buff, 4);
//}

  success &= HalSensorReadReg(AIR_STATUS, i2c_buff, 1);
  if(i2c_buff[0] & AIR_DATA_READY) // check if data ready
  {  
    // Read ID register 
    if (reg == 0x02) le = 8;
    if (reg == 0x03) le = 2;
    if (reg == 0x04) le = 4;
    if (reg == 0x06) le = 4;
    if (reg == 0x10) le = 5;
    if (reg == 0x11) le = 2;
    if (reg == 0x23) le = 2;
    if (reg == 0x24) le = 2;
    if (reg == 0xF4) le = 0;
    if (reg == 0x41) reg = 0;

    success &= HalSensorReadReg(reg, i2c_buff, le);

    pBuf[0]=reg;
    for (int i = 0; i < le; i++) pBuf[i+1] = i2c_buff[i];
  }
  else 
  {
    success &= FALSE;
  }

  HAL_AIR_WAKE_SBIT=1;
  
  return success;       
}


/*******************************************************************************
* @fn          HalAirGetData
*
* @brief       Get CO2 and VOCs data from registers
*
* @param       pBuf: pointer to allocate the data
*
* @return      TRUE if valid data, FALSE if not
*/
bool HalAirGetData( uint8 *pBuf )
{
  uint8 i2c_buff[8]={0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA,0xAA};

  bool success=1;
  
  // WakeUp Sensor
//  HalAirWakeUp();
  //*** WakeUp Air Sensor ***
  HAL_AIR_WAKE_SBIT=0;
  // Wait 500us after I2C trasmission
  ST_HAL_DELAY(62);
  
  // I2C init for this sensor
  HalI2CAirSelect();

  if (!initialize) //oJo, esto deberia estar en la inicializacion?
  {    
    // Read HD_ID. Check ID 
    HalSensorReadReg(AIR_HW_ID, i2c_buff, 1);
    if(!(i2c_buff[0] & AIR_HW_ID_VALUE)) 
      HAL_TURN_ON_LED2(); //Error  

    // Read STATUS. Check STATUS is App Valid
    HalSensorReadReg(AIR_STATUS, i2c_buff, 1);
    if(!(i2c_buff[0] & AIR_APP_VALID)) 
      HAL_TURN_ON_LED2(); //Error  
 
    // Write APP_START
    HalSensorWriteReg(AIR_APP_START, &success, sizeof(success));

    // Read STATUS. Check STATUS is App Valid & Firmarwe in App Mode
    HalSensorReadReg(AIR_STATUS, i2c_buff, 1);
    if(!(i2c_buff[0] & (AIR_APP_VALID | AIR_FW_MODE))) 
      HAL_TURN_ON_LED2(); //Error

    // Write MEAS_MODE. Write drive mode and interrupt
    HalSensorWriteReg(AIR_MEAS_MODE, &airSensorPM_WakeUp, sizeof(airSensorPM_WakeUp));
    
    // Write ENV_DATA. Default value.
    HalSensorWriteReg(AIR_ENV_DATA, airSensor_ENV_DATA, sizeof(airSensor_ENV_DATA));

    // Write AIR_BASELINE. Default value. 
    HalSensorWriteReg(AIR_BASELINE, airSensor_AIR_BASELINE, sizeof(airSensor_AIR_BASELINE));

    initialize=TRUE;
  }  
  
  // WakeUp Sensor
//  HalAirWakeUp();
  //*** WakeUp Air Sensor ***
  HAL_AIR_WAKE_SBIT=0;
  // Wait 500us after I2C trasmission
  ST_HAL_DELAY(62);
  
  // Microcontroller I2C init for this sensor
  HalI2CAirSelect();
  
  // Read HD_ID. Check ID 
  HalI2CReadDirect(HAL_CCS811_I2C_ADDRESS, AIR_HW_ID, i2c_buff, 1);
  if(!(i2c_buff[0] & AIR_HW_ID_VALUE)) 
    HAL_TOGGLE_LED2(); //Error

  // Read STATUS. Check STATUS is App Valid
  HalI2CReadDirect(HAL_CCS811_I2C_ADDRESS, AIR_STATUS, i2c_buff, 1);
  if(!(i2c_buff[0] & AIR_APP_VALID)) 
    HAL_TOGGLE_LED2(); //Error

  // Write APP_START
  HalI2CWriteDirect(HAL_CCS811_I2C_ADDRESS, AIR_APP_START, i2c_buff, 0);
  // Read STATUS. Check STATUS is App Valid & Firmarwe in App Mode
  HalI2CReadDirect(HAL_CCS811_I2C_ADDRESS, AIR_STATUS, i2c_buff, 1);
  if(!(i2c_buff[0] & (AIR_APP_VALID | AIR_FW_MODE))) 
    HAL_TOGGLE_LED2(); //Error

  // Write MEAS_MODE. Write drive mode and interrupt
  i2c_buff[0] = AIR_MODE1;
  HalI2CWriteDirect(HAL_CCS811_I2C_ADDRESS, AIR_MEAS_MODE, i2c_buff, 1); 

  //Polling Read
  bool readed=FALSE;
  while (!(readed))
  {
    // Read STATUS. Check DATA_READY
    HalI2CReadDirect(HAL_CCS811_I2C_ADDRESS, AIR_STATUS, i2c_buff, 1);
    if(i2c_buff[0] & AIR_DATA_READY) // check if data ready
    {
      readed=TRUE;
      
      // Get data
      HalI2CReadDirect(HAL_CCS811_I2C_ADDRESS, AIR_ALG_RESULT_DATA, i2c_buff, 8);
     }
   }
  
//      VOID osal_memcpy( pBuf, i2c_buff, 4 );
  
  pBuf[0]=i2c_buff[0];
  pBuf[1]=i2c_buff[1];
  pBuf[2]=i2c_buff[2];
  pBuf[3]=i2c_buff[3];
  pBuf[4]=i2c_buff[4];
  pBuf[5]=i2c_buff[5];
  pBuf[6]=i2c_buff[6];
  pBuf[7]=i2c_buff[7];
  
  HAL_AIR_WAKE_SBIT=1;

  return success;       
}


bool HalAirPutReg(uint8 reg, uint8 value )
{
  bool success=1;

  // WakeUp Sensor
//  HalAirWakeUp();
  //*** WakeUp Air Sensor ***
  HAL_AIR_WAKE_SBIT=0;
  // Wait 500us after I2C trasmission
  ST_HAL_DELAY(62);
  
  // I2C init for this sensor
  HalI2CAirSelect( );

  // *** Configure AirSensor in Wakeup Mode ***
  // Set PowerMode
  HalSensorWriteReg(AIR_MEAS_MODE, &airSensorPM_WakeUp, sizeof(airSensorPM_WakeUp));

  //Write register
  HalSensorWriteReg(reg, &value, sizeof(value));

  HAL_AIR_WAKE_SBIT=1;

  return success;
}

/*******************************************************************************
 * @fn          HalAirTest
 *
 * @brief       Run a sensor self-test
 *
 * @return      TRUE if passed, FALSE if failed
 */
bool HalAirTest(void)
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

/* -----------------------------------------------------------------------------
*                             Private functions
* ------------------------------------------------------------------------------
*/

/*******************************************************************************
* @fn          HalAirSelect
*
* @brief       Select the Air Quality Sensor on the I2C-bus
*
* @return
*/
static void HalI2CAirSelect(void)
{
  //Set up I2C that is used to communicate
  HalI2CInit(HAL_CCS811_I2C_ADDRESS,i2cClock_267KHZ);
}

#else

void HalAirInit(void) {}
bool HalAirGetReg(uint8 reg, uint8 *pBuf ) {return TRUE;}

#endif /* HAL_AIR */
/*******************************************************************************
*******************************************************************************/

/**************************************************************************//**

    @file       hal_mag.c

    @brief      Functions for accessing magnetometer LIS3MDL on Dragonfly.

******************************************************************************/


/* ------------------------------------------------------------------------------------------------
*                                          Includes
* ------------------------------------------------------------------------------------------------
*/

#include <stdio.h>

#include "hal_mag.h"
#include "hal_sensor.h"
#include "hal_i2c.h"
#include "hal_board.h"

#include "hal_suart.h"

#if (defined HAL_MAG) && (HAL_MAG == TRUE)

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
static void HalI2CMagSelect(void);

static void HalMagWakeUp(void);
static void HalMagSleep(void);

static void HalMagSleep(void);

//static void HalMagSetPM(uint8 mode, uint8 sleep_time);
//static void HalMagSetRange(uint8 range);




/* ------------------------------------------------------------------------------------------------
*                                       Local Variables
* ------------------------------------------------------------------------------------------------
*/
//// Default Accelerometer configuration values
//static uint8 accSensorPM_Wakeup = ACC_PM_ON;
//static uint8 accSensorPM_Sleep  = ACC_PM_LP | ACC_PM_SLEEP_10MS;
//static uint8 accSensorRange     = HAL_ACC_RANGE_2G;
//static uint8 accBandwidth       = ACC_BW_250HZ;
//
////Electrical characteristics for INT1 and INT2 push-pull low level activated.
////static uint8 accIntElecBe = ((accIntElecBe & ~(ACC_INT1_LVL | ACC_INT2_LVL)) | (ACC_INT1_OD | ACC_INT2_OD)) ; 
//static uint8 accIntElecBe = 0x0A; //0b00001010
//static uint8 accIntTimeBe = ACC_INT_TEMP_1S; 
////static uint8 accIntScr = ACC_INT_SRC_TAP_UNFILT; //Scr for Tap unfiltered
//
////Interruption activation
//static uint8 accInt_En0 = ACC_INT_D_TAP_EN | ACC_INT_ORIENT_EN ; // Bit in register 0x16
////static uint8 accInt_En0 = ACC_INT_D_TAP_EN; // Bit in register 0x16
//static uint8 accInt_En1 = ACC_INT_DATA_EN; // Bit in register 0x17
//
////Interrupt Mapping 0x19-0x1A-0x1B registers
//// INT1 -> P1.7 -> KEY_1 --> Simple Tap
//static uint8 accIntMap0 = ACC_INT_MAP_D_TAP; // 0x19 register
//// INT2 -> P1.6 -> KEY_2 --> New Data
////static uint8 accIntMap1 = ACC_INT2_MAP_DATA; // 0x1A register
//
//
////static uint8 accShadow = 0x40; 

char host[] = "test.libelium.com";
char resource[] = "/test-get-post.php?Temperatura=%i";

void espSetup() 
{
  // put your setup code here, to run once:
  uartPrintf("GET ESP8266 Demo\n\r");

  // connect to network
  ST_HAL_DELAY((uint32)125*1000); //delay(1000);
  uartPrintf("AT+CWMODE=1\r\n");                               // (set)     Module as Client
  ST_HAL_DELAY((uint32)125*500); //  delay(500);
  uartPrintf("AT+CIPMUX=1\r\n");
  ST_HAL_DELAY((uint32)125*500); //  delay(500);
  uartPrintf("AT+CWLAP\r\n");                                  // (execute) List accesible AP
  ST_HAL_DELAY((uint32)125*1000); //  delay(1000);
  uartPrintf("AT+CWJAP=\"libelium_wsn2\",\"libelium.2012_\"\r\n"); // (execute) Join to AP. Connect to 
  ST_HAL_DELAY((uint32)125*10000); //  delay(10000);
  uartPrintf("AT+CWJAP?\r\n");                                 // (inquiry) View current network 
  ST_HAL_DELAY((uint32)125*1000); //  delay(1000);
  uartPrintf("AT+CIFSR\r\n");                                  // (execute) Show IP address
  ST_HAL_DELAY((uint32)125*1000); //  delay(1000);
}

void espSend(int temperatura) 
{
  char _buff[200];
  int lenght;

  // open socket 0: TCP 
  uartPrintf("AT+CIPSTART=1,\"TCP\",\"test.libelium.com\",80\r\n");
  ST_HAL_DELAY((uint32)125*7000); //  delay(7000);

  // compose GET peittion
  lenght = sprintf(_buff, "GET /test-get-post.php?Temperatura=%i HTTP/1.1\r\nHost: %s\r\n\r\n", temperatura, host);

  // send http GET request using socket 0
//  uartPrintf("AT+CIPSEND=1,%i\r\n", 25 + sizeof(resource) + sizeof(host));
  uartPrintf("AT+CIPSEND=1,%i\r\n", lenght);
  ST_HAL_DELAY((uint32)125*2000); //  delay(2000);
  
  uartPrintf(_buff);

  //  uartPrintf("GET "); // 4
//  uartPrintf(resource);
//  uartPrintf(" HTTP/1.1\r\nHost: "); // 17
//  uartPrintf(host);
//  uartPrintf("\r\n\r\n"); // 4

  ST_HAL_DELAY((uint32)125*500); //  delay(500);
}


/**************************************************************************************************
* @fn          HalMagInit
*
* @brief       This function initializes the HAL Magnetometer abstraction layer.
*
* @return      None.
*/
void HalMagInit(void)
{
  uint8 buf;
  // Set default values
//  HalMagSetRange(HAL_ACC_RANGE_2G);
//  HalMagSetPM(ACC_PM_LP, ACC_PM_SLEEP_10MS);
//  HalMagTest();
  
  // set as IO port 
  HAL_MAG_POWER_SEL |= (uint8) HAL_MAG_POWER_BV;
  
  // set as OUTPUT port
  HAL_MAG_POWER_DDR |= (uint8) HAL_MAG_POWER_BV;
  
  // and put P2_0 pin to 1
  HAL_MAG_POWER_SBIT = 1;
  
  if (!HalMagTest()) return;
  
  // I2C init for this sensor
  HalI2CMagSelect();
 
  //oJo mirar si es conveniente control de power *** Enable Accelerometer power ***
  
  // *** Configure magnetometer ***

    // 0x70 = 0b01110000
    // OM = 11 (ultra-high-performance mode for X and Y); DO = 100 (10 Hz ODR)
    buf = 0x70;
    buf |= 0x80; // enable temperature sensor
    HalSensorWriteReg(CTRL_REG1, &buf, sizeof(buf));

    // 0x00 = 0b00000000
    // FS = 00 (+/- 4 gauss full scale)
    buf = 0x00;
    HalSensorWriteReg(CTRL_REG2, &buf, sizeof(buf));

    // 0x00 = 0b00000000
    // MD = 00 (continuous-conversion mode)
    buf = 0x00;
    HalSensorWriteReg(CTRL_REG3, &buf, sizeof(buf));

    // 0x0C = 0b00001100
    // OMZ = 11 (ultra-high-performance mode for Z)
    buf = 0x0C;
    HalSensorWriteReg(CTRL_REG4, &buf, sizeof(buf));

    
/* ejemplo del magnetometro con el esp
================================================================================    
    // test mmagnetometer
    static uint8 buff[8] = {0,0,0,0,0,0,0,0};
 
    static int16 ta = 0;
    static int16 loop = 0;
//    static int16 mx, my, mz = 0;
    static int16 mz = 0;
        
    // oJo esp8266 single interaction (change suart to 115200baud)
    espSetup();
    
    while (TRUE)
    {
      HalMagGetTemp(buff);      
      ta = (((buff[1]<<8)+buff[0])>>3)+25;
      
      HalMagGetXYZ(buff);
//      mx = (buff[1]<<8)+buff[0];
//      my = (buff[3]<<8)+buff[2];
      mz = (buff[5]<<8)+buff[4];
      
      if ((mz > 6000) && (loop == 0))
      {
        // oJo Sigfox interaction (change suart to 9600baud)
        //uartPrintf("AT$SF=%i\r\n", ta);
        
        // oJo esp8266 single interaciton
        espSend(ta);
        loop++;
      }
      else if (mz < 3000)
        loop = 0;
        
//      uartPrintf("[hal_mag] mz:%i t:%i\n", mz, ta);
//      uartPrintf("[hal_mag] mx:%i my:%i mz:%i t:%i\n", mx, my, mz, ta);
//      uartPrintf("%i %i %i %i\n", mx, my, mz, ta);
      ST_HAL_DELAY(12500);      
    }
  // Set PowerMode
//  HalSensorWriteReg(ACC_PM, &accSensorPM_Sleep, sizeof(accSensorPM_Sleep));
================================================================================    
*/
}

/**************************************************************************************************
* @fn          HalMagWakeUp
*
* @brief       This function WakeUp I2C Magnetometer.
*
* @return      None.
*/
void HalMagWakeUp(void)
{
  // I2C init for this sensor
  HalI2CMagSelect();

  // *** Configure accelerometer ***
  
  // Set PowerMode
//  HalSensorWriteReg(ACC_PM, &accSensorPM_Wakeup, sizeof(accSensorPM_Wakeup));
}

/**************************************************************************************************
* @fn          HalMagSleep
*
* @brief       This function Sleep I2C Magnetometer.
*
* @return      None.
*/
void HalMagSleep(void)
{
  // I2C init for this sensor it is no necessary, I2C for Magnetometer arlready selected
  //HalI2CmagSelect();

  // *** Configure accelerometer ***
  
  // Set PowerMode
//  HalSensorWriteReg(ACC_PM, &accSensorPM_Sleep, sizeof(accSensorPM_Sleep));
}

/**************************************************************************************************
* @fn          HalMagSetRange
*
* @brief       Set the range of the Magnetometer
*
* @param       range: HAL_MAG_RANGE_4G, HAL_MAG_RANGE_8G, HAL_MAG_RANGE_16G
*
* @return      None
*/
void HalMagSetRange(uint8 range)
{
//  uint8 magRange = range;
//  
//  switch (magRange)
//  {
//  case HAL_MAG_RANGE_4G:
//    magSensorRange = MAG_RANGE_4G; 
//    break;
//  case HAL_MAG_RANGE_8G:
//    magSensorRange = ACC_RANGE_8G;
//    break;
//  case HAL_MAG_RANGE_16G:
//    magSensorRange = MAG_RANGE_16G;
//    break;
//  default:
//    // Should not get here
//    break;
//  }
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
void HalMagSetPM(uint8 mode, uint8 sleep_time)
{
//  uint8 accMode = mode | sleep_time;
//  
//  switch (accMode)
//  {
//  case ACC_PM_ON:
//    accSensorPM_Sleep = ACC_PM_ON; //Never sleep
//    break;
//  case ACC_PM_SUSP:
//    accSensorPM_Sleep = ACC_PM_SUSP; 
//    break;
//  case ACC_PM_LP | ACC_PM_SLEEP_10MS:
//    accSensorPM_Sleep = ACC_PM_LP | ACC_PM_SLEEP_10MS;
//    break;
//  case ACC_PM_LP | ACC_PM_SLEEP_25MS:
//    accSensorPM_Sleep = ACC_PM_LP | ACC_PM_SLEEP_25MS;
//    break;
//  case ACC_PM_LP | ACC_PM_SLEEP_50MS:
//    accSensorPM_Sleep = ACC_PM_LP | ACC_PM_SLEEP_50MS;
//    break;
//  default:
//    // Should not get here
//    break;
//  }
}

/**************************************************************************************************
* @fn          HalMagGetXYZ
*
* @brief       Read data from the magnetometer - X, Y, Z - 3 bytes
*
* @return      TRUE if valid data, FALSE if not
*/
bool HalMagGetXYZ(uint8 *pBuf )
{
  uint8 xlm;
  uint8 xhm;
  uint8 ylm;
  uint8 yhm;
  uint8 zlm;
  uint8 zhm;
  bool success;

  // Seled Address and Speed of I2C for Magnetometer
  HalI2CMagSelect();

  // WakeUp Magnetometer
  HalMagWakeUp();

  // Read the registers 
  // oJo utilizar brust
  success  = HalSensorReadReg(OUT_X_L, &xlm, sizeof(xlm));
  success |= HalSensorReadReg(OUT_X_H, &xhm, sizeof(xhm));
  success |= HalSensorReadReg(OUT_Y_L, &ylm, sizeof(ylm));
  success |= HalSensorReadReg(OUT_Y_H, &yhm, sizeof(yhm));
  success |= HalSensorReadReg(OUT_Z_L, &zlm, sizeof(zlm));
  success |= HalSensorReadReg(OUT_Z_H, &zhm, sizeof(zhm));

  if (success)
  {
    // Valid data
    pBuf[0] = xlm;
    pBuf[1] = xhm;
    pBuf[2] = ylm;
    pBuf[3] = yhm;
    pBuf[4] = zlm;
    pBuf[5] = zhm;
  }

  // Sleep Magnetometer 
  HalMagSleep();

  return success;
}


/**************************************************************************************************
* @fn          HalMagGetID
*
* @brief       Read data ID register
*
* @return      TRUE if valid data, FALSE if not
*/
bool HalMagGetID(uint8 *pBuf)
{
  uint8 id;
  bool success;

  // Seled Address and Speed of I2C for Magnetometer
  HalI2CMagSelect();

  // WakeUp Magnetometer
  HalMagWakeUp();

  // Read ID register
  success = HalSensorReadReg(WHO_AM_I, &id, sizeof(id));

  if (success)
  {
    // Valid data
    pBuf[0] = id;
  }

  // Sleep Magnetometer 
  HalMagSleep();

  return success;
}

/**************************************************************************************************
* @fn          HalMagGetTemp
*
* @brief       Read data mag Internal Temperature
*
* @return      TRUE if valid data, FALSE if not
*
*/
bool HalMagGetTemp(uint8 *pBuf )
{
  // do not forget to enable temperature sensor in CTRL_REG1
  uint8 tl;
  uint8 th;
  bool success;

  // Seled Address and Speed of I2C for Magnetometer
  HalI2CMagSelect();

  // WakeUp Magnetometer
  HalMagWakeUp();

  // Read ID register
  success  = HalSensorReadReg(TEMP_OUT_L, &tl, sizeof(tl));
  success |= HalSensorReadReg(TEMP_OUT_H, &th, sizeof(th));

  if (success)
  {
    // Valid data
    pBuf[0] = tl;
    pBuf[1] = th;
  }
  
  // Sleep Accelerometer 
  HalMagSleep();

  return success;
}

/**************************************************************************************************
 * @fn          HalAccTest
 *
 * @brief       Run a sensor self-test
 *
 * @return      TRUE if passed, FALSE if failed
 */
bool HalMagTest(void)
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
//  ST_ASSERT(HalSensorReadReg(WHO_AM_I, &val, 1));
//  ST_ASSERT(val==LIS3MDL_WHO_ID);
//
  
  
  uint8 data;
  
  // i2c scanner 
  for (uint8 addr = 0; addr < 0x7F; addr++)
  {
    HalI2CInit(addr, i2cClock_267KHZ);
    for (uint8 reg = 0x00; reg < 0x3F; reg++)
    {
      if (HalI2CWrite(1,&reg) == 1)
      /* Now read data */
      HalI2CRead(1,&data);
      
      if (data == 0x3D)
          printf("OK, addr: %x, reg: %x, data %x\n", addr, reg, data);
    }
  }

 // test if exist LIS3MDL
  HalI2CMagSelect();

  HalSensorReadReg(WHO_AM_I, &data, sizeof(data));
  
  if (0x3D != data)
  {
    return FALSE;
  }

#if (defined HAL_SUART) && (HAL_SUART == TRUE)
  uartPrintf("[hal_mag] LIS3MDL present\r\n");
#endif  
  
  return TRUE;
}


/* ------------------------------------------------------------------------------------------------
*                                           Private functions
* -------------------------------------------------------------------------------------------------
*/

/**************************************************************************************************
* @fn          HalMagSelect
*
* @brief       Select the magnetometer on the I2C-bus
*
* @return
*/
static void HalI2CMagSelect(void)
{
  //Set up I2C that is used to communicate
  HalI2CInit(HAL_LIS3MDL_I2C_ADDRESS,i2cClock_267KHZ);
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

#endif //(defined HAL_MAG) && (HAL_LUX == MAG)
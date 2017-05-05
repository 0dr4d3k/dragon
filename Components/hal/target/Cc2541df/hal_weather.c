/*******************************************************************************
  Filename:       hal_weather.c
  Revised:        $Date: 2016-03-14 11:45:31$
  Revision:       $Revision: 0 $

  Description:    This file contains the declaration Bosch BME280 Humidity, 
                  Presure and Temperature sensor abstraction layer.
 ******************************************************************************/

/* -----------------------------------------------------------------------------
 *                                 Includes
 * -----------------------------------------------------------------------------
 */

#include "hal_board.h"
#include "hal_weather.h"
#include "hal_sensor.h"
#include "hal_i2c.h"
#include "hal_debug.h"

#if (defined HAL_WEATHER) && (HAL_WEATHER == TRUE)

/* -----------------------------------------------------------------------------
 *                                 Constants
 * -----------------------------------------------------------------------------
 */

/* -----------------------------------------------------------------------------
 *                                 Typedefs
 * -----------------------------------------------------------------------------
 */

/* -----------------------------------------------------------------------------
 *                                  Macros
 * -----------------------------------------------------------------------------
 */

/* -----------------------------------------------------------------------------
 *                              Local Functions
 * -----------------------------------------------------------------------------
 */
// INTERNAL CONTROL FUNCTIONS
static void HalWeatherSelect(void);

static void HalWeatherWakeUp(void);


// INTERNAL COMPENSATION FUNCTIONS
//! This function compensates the temperature with the calibration data
static int32 compensateTemp(int32 uncompensated_temp);

//! This function compensates the pressure with the calibration data
static uint32 compensatePres(int32 uncompensated_pres);

//! This function compensates the humidity with the calibration data
static uint32 compensateHum(int32 uncompensated_hum);

//! This function reads the calibration registers
static void readCalibration();


/* -----------------------------------------------------------------------------
 *                               Local Variables
 * -----------------------------------------------------------------------------
 */
  
/// Calibration variables
static uint16 dig_T1;
static int16  dig_T2;
static int16  dig_T3;

static uint16 dig_P1;
static int16  dig_P2;
static int16  dig_P3;
static int16  dig_P4;
static int16  dig_P5;
static int16  dig_P6;
static int16  dig_P7;
static int16  dig_P8;
static int16  dig_P9;

static uint8  dig_H1;
static int16  dig_H2;
static uint8  dig_H3;
static int16  dig_H4;
static int16  dig_H5;
static int8   dig_H6;

static int32 t_fine;

// Configuration Registers
static uint8 ctrl_Filter = 0x00;  //Sets IIR filter value
static uint8 ctrl_Humi = 0x00;    //Set Over Sample value for Humidity
static uint8 ctrl_Measure = 0x00; //Sets Over Sample for Temperature and Pressure and Measure Mode

/* -----------------------------------------------------------------------------
 *                       Definition External Functions
 * -----------------------------------------------------------------------------
 */

/// PUBLIC FUNCTIONS
/// POWER AND CONFIGURATION FUNCTIONS


/*******************************************************************************
 * @fn          HalWeatherSetMode
 *
 * @brief       Set the mode of the Weather Sensor
 *
 * @param       mode: BME280_MODE_EN_T, BME280_MODE_EN_P, BME280_MODE_EN_H
 *              BME280_MODE_MONITORING
 *              BME280_MODE_AIR_QUALITY
 *              BME280_MODE_INDOOR_NAVIGATION
 *              BME280_MODE_INDOOR_GAMING
 *
 * @return      None
 ******************************************************************************/
void HalWeatherSetMode(uint8 mode)
{
  uint8  weatherMode = mode;
  
  switch (weatherMode & ~BME280_MODE_MASK)
  {
  case BME280_MODE_MONITORING:
    
    //////////////////////////////////////////////////////////////////////
    // Sensor mode forced mode, 1 sample / minute                       //
    // Oversampling settings pressure ×1, temperature ×1, humidity ×1   //
    // IIR filter settings filter off                                   //
    // Performance for suggested settings                               //
    // Current consumption 0.16 µA                                      //
    // RMS Noise 3.3 Pa / 30 cm, 0.07 %RH                               //
    // Data output rate 1/60 Hz                                         //
    //////////////////////////////////////////////////////////////////////

    //Set Oversampling values for Temperature
    if (weatherMode | BME280_MODE_EN_T) 
      ctrl_Measure = (ctrl_Measure & ~BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__MSK) | 
        (BME280_OVERSAMP_1X << BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__POS); 
    else 
      ctrl_Measure = (ctrl_Measure & ~BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__MSK) | 
        (BME280_OVERSAMP_SKIPPED << BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__POS); 

    //Set Oversampling values for Pressure
    if (weatherMode | BME280_MODE_EN_P) 
      ctrl_Measure = (ctrl_Measure & ~BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__MSK) | 
        (BME280_OVERSAMP_1X << BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__POS); 
    else 
      ctrl_Measure = (ctrl_Measure & ~BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__MSK) | 
        (BME280_OVERSAMP_SKIPPED << BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__POS); 
    
    //Set Oversampling values for Humidity
    if (weatherMode | BME280_MODE_EN_H) 
      ctrl_Humi = (ctrl_Humi & ~BME280_CTRL_HUMIDITY_REG_OVERSAM_HUMIDITY__MSK) | 
        (BME280_OVERSAMP_1X << BME280_CTRL_HUMIDITY_REG_OVERSAM_HUMIDITY__POS); 
    else 
      ctrl_Humi = (ctrl_Humi & ~BME280_CTRL_HUMIDITY_REG_OVERSAM_HUMIDITY__MSK) | 
        (BME280_OVERSAMP_SKIPPED << BME280_CTRL_HUMIDITY_REG_OVERSAM_HUMIDITY__POS); 
    
    //Set IIR filter to off
    ctrl_Filter = (ctrl_Filter & ~BME280_CONFIG_REG_FILTER__MSK) | 
      (BME280_FILTER_COEFF_OFF << BME280_CONFIG_REG_FILTER__POS); 
    
    //Set Measure Mode to Forced Mode
    ctrl_Measure = (ctrl_Measure & ~BME280_CTRL_MEAS_REG_POWER_MODE__MSK) | 
      (BME280_FORCED_MODE << BME280_CTRL_MEAS_REG_POWER_MODE__POS); 

    break;

  case BME280_MODE_AIR_QUALITY:
    
    //////////////////////////////////////////////////////////////////////
    // Suggested settings for weather monitoring                        //
    // Sensor mode forced mode, 1 sample / second                       //
    // Oversampling settings pressure ×0, temperature ×1, humidity ×1   //
    // IIR filter settings filter off                                   //
    // Performance for suggested settings                               //
    // Current consumption 2.9 µA                                       //
    // RMS Noise 0.07 %RH                                               //
    // Data output rate 1 Hz                                            //
    //////////////////////////////////////////////////////////////////////

    //Set Oversampling values for Temperature
    ctrl_Measure = (ctrl_Measure & ~BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__MSK) | 
      (BME280_OVERSAMP_1X << BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__POS); 

    //Set Oversampling values for Pressure
    ctrl_Measure = (ctrl_Measure & ~BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__MSK) | 
      (BME280_OVERSAMP_SKIPPED << BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__POS); 
    
    //Set Oversampling values for Humidity
    ctrl_Humi = (ctrl_Humi & ~BME280_CTRL_HUMIDITY_REG_OVERSAM_HUMIDITY__MSK) | 
      (BME280_OVERSAMP_1X << BME280_CTRL_HUMIDITY_REG_OVERSAM_HUMIDITY__POS); 
    
    //Set IIR filter to off
    ctrl_Filter = (ctrl_Filter & ~BME280_CONFIG_REG_FILTER__MSK) | 
      (BME280_FILTER_COEFF_OFF << BME280_CONFIG_REG_FILTER__POS); 
    
    //Set Measure Mode to Forced Mode
    ctrl_Measure = (ctrl_Measure & ~BME280_CTRL_MEAS_REG_POWER_MODE__MSK) | 
      (BME280_FORCED_MODE << BME280_CTRL_MEAS_REG_POWER_MODE__POS); 
    
    break;

  case BME280_MODE_INDOOR_NAVIGATION:

    //////////////////////////////////////////////////////////////////////
    // Suggested settings for indoor navigation                         //
    // Sensor mode normal mode, tstandby = 0.5 ms                       //
    // Oversampling settings pressure ×16, temperature ×2, humidity ×1  //
    // IIR filter settings filter coefficient 16                        //
    // Performance for suggested settings                               //
    // Current consumption 633 µA                                       //
    // RMS Noise 0.2 Pa / 1.7 cm                                        //
    // Data output rate 25Hz                                            //
    // Filter bandwidth 0.53 Hz                                         //
    // Response time (75%) 0.9 s                                        //
    //////////////////////////////////////////////////////////////////////
    
    //Set Oversampling values for Temperature
    if (weatherMode | BME280_MODE_EN_T) 
      ctrl_Measure = (ctrl_Measure & ~BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__MSK) | 
        (BME280_OVERSAMP_2X << BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__POS); 
    else 
      ctrl_Measure = (ctrl_Measure & ~BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__MSK) | 
        (BME280_OVERSAMP_SKIPPED << BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__POS); 

    //Set Oversampling values for Pressure
    if (weatherMode | BME280_MODE_EN_P) 
      ctrl_Measure = (ctrl_Measure & ~BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__MSK) | 
        (BME280_OVERSAMP_16X << BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__POS); 
    else 
      ctrl_Measure = (ctrl_Measure & ~BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__MSK) | 
        (BME280_OVERSAMP_SKIPPED << BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__POS); 
    
    //Set Oversampling values for Humidity
    if (weatherMode | BME280_MODE_EN_H) 
      ctrl_Humi = (ctrl_Humi & ~BME280_CTRL_HUMIDITY_REG_OVERSAM_HUMIDITY__MSK) | 
        (BME280_OVERSAMP_1X << BME280_CTRL_HUMIDITY_REG_OVERSAM_HUMIDITY__POS); 
    else 
      ctrl_Humi = (ctrl_Humi & ~BME280_CTRL_HUMIDITY_REG_OVERSAM_HUMIDITY__MSK) | 
        (BME280_OVERSAMP_SKIPPED << BME280_CTRL_HUMIDITY_REG_OVERSAM_HUMIDITY__POS); 
    
    //Set IIR filter to 16
    ctrl_Filter = (ctrl_Filter & ~BME280_CONFIG_REG_FILTER__MSK) | 
      (BME280_FILTER_COEFF_16 << BME280_CONFIG_REG_FILTER__POS); 
    
    //Set Measure Mode to Forced Mode
    ctrl_Measure = (ctrl_Measure & ~BME280_CTRL_MEAS_REG_POWER_MODE__MSK) | 
      (BME280_FORCED_MODE << BME280_CTRL_MEAS_REG_POWER_MODE__POS); 

    break;

  case BME280_MODE_INDOOR_GAMING:
    
    //////////////////////////////////////////////////////////////////////
    // Suggested settings for gaming                                    //
    // Sensor mode normal mode, tstandby = 0.5 ms                       //
    // Oversampling settings pressure ×4, temperature ×1, humidity ×0   //
    // IIR filter settings filter coefficient 16                        //
    // Performance for suggested settings                               //
    // Current consumption 581 µA                                       //
    // RMS Noise 0.3 Pa / 2.5 cm                                        //
    // Data output rate 83 Hz                                           //
    // Filter bandwidth 1.75 Hz                                         //
    // Response time (75%) 0.3 s                                        //   
    //////////////////////////////////////////////////////////////////////

    //Set Oversampling values for Temperature
    if (weatherMode | BME280_MODE_EN_T) 
      ctrl_Measure = (ctrl_Measure & ~BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__MSK) | 
        (BME280_OVERSAMP_1X << BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__POS); 
    else 
      ctrl_Measure = (ctrl_Measure & ~BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__MSK) | 
        (BME280_OVERSAMP_SKIPPED << BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__POS); 

    //Set Oversampling values for Pressure
    if (weatherMode | BME280_MODE_EN_P) 
      ctrl_Measure = (ctrl_Measure & ~BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__MSK) | 
        (BME280_OVERSAMP_4X << BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__POS); 
    else 
      ctrl_Measure = (ctrl_Measure & ~BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__MSK) | 
        (BME280_OVERSAMP_SKIPPED << BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__POS); 
    
    //Set Oversampling values for Humidity
    ctrl_Humi = (ctrl_Humi & ~BME280_CTRL_HUMIDITY_REG_OVERSAM_HUMIDITY__MSK) | 
      (BME280_OVERSAMP_SKIPPED << BME280_CTRL_HUMIDITY_REG_OVERSAM_HUMIDITY__POS); 
    
    //Set IIR filter to 16
    ctrl_Filter = (ctrl_Filter & ~BME280_CONFIG_REG_FILTER__MSK) | 
      (BME280_FILTER_COEFF_16 << BME280_CONFIG_REG_FILTER__POS); 
    
    //Set Measure Mode to Forced Mode
    ctrl_Measure = (ctrl_Measure & ~BME280_CTRL_MEAS_REG_POWER_MODE__MSK) | 
      (BME280_FORCED_MODE << BME280_CTRL_MEAS_REG_POWER_MODE__POS); 
    break;

  default:
    // Should not get here
    break;
  }
      
   ///////////////////////////////////////////////////////////////////////
   //                        TiME CaLCULATION                           //
   // meassure_time = 1 + (2 * (1 << (over_sample_value - 1)));         //
   // meassure_max_time = 1.25 + (2.3 * (1 << (over_sample_value - 1)));//
   ///////////////////////////////////////////////////////////////////////
}


/******************************************************************************
 * Function:   This function initialize the BM·280 with default values.
 * Parameters: None.
 * Returns:    None.
 ******************************************************************************/
void HalWeatherInit(void)
{
//////////////////////////////////////////////////// begin debug context ///////
  HAL_DEBUG_STATEMENT(                                                         \
  /* statement context */                                                      \
  DEBUG_HAL_WEATHER | DEBUG_ASSERT,                                            \
  {                                                                            \
    uint8 usr = 0x00;                                                          \
    /* test if exist BME280 */                                                 \
    HalI2CReadDirect(HAL_BME280_I2C_ADDRESS, BME280_CHIP_ID_REG, &usr, 1);     \
    if (usr & BME280_CHIP_ID_REG_CHIP_ID)                                      \
      halDebugMsg(DEBUG_HAL_WEATHER | DEBUG_ASSERT,                            \
             " BME280 detected in 0x%02X\r\n", HAL_BME280_I2C_ADDRESS);        \
    else                                                                       \
      halDebugMsg(DEBUG_HAL_WEATHER | DEBUG_ERROR, " BME280 no detected\r\n"); \
  });
////////////////////////////////////////////////////// end debug context ///////
  
  // Set default values
  HalWeatherSetMode(BME280_MODE_MONITORING | 
                    BME280_MODE_EN_T | 
                    BME280_MODE_EN_P | 
                    BME280_MODE_EN_H);
  
  // I2C init for this sensor
  HalWeatherSelect();

  // *** Configure BME280 ***
  // Set ctrl_Filter
  HalSensorWriteReg(BME280_CONFIG_REG, &ctrl_Filter, sizeof(ctrl_Filter));
  // Set ctrl_Measure
  HalSensorWriteReg(BME280_CTRL_MEAS_REG, &ctrl_Measure, sizeof(ctrl_Measure));
  // Set ctrl_Humidity
  HalSensorWriteReg(BME280_CTRL_HUMIDITY_REG, &ctrl_Humi, sizeof(ctrl_Humi));
  
  // Read the calibration registers
  readCalibration();
     
}

/******************************************************************************
 * Function:   This function checks if the module is accesible via I2C
 * Parameters: None.
 * Returns:    1 if is accesible, 0 if not
 ******************************************************************************/
bool HalWeatherTest(void)
{
  uint8 valueID;
  
  // I2C init for this sensor
  HalWeatherSelect();
  
  // Read ID register
  HalSensorReadReg(BME280_CHIP_ID_REG, &valueID, sizeof(valueID));
 
  // Compare with spected response
  if (valueID == BME280_CHIP_ID_REG_CHIP_ID) 
    return 1;
  else
    return 0;
}

/******************************************************************************
 * Function:   This function WakeUp the sensor.
 * Parameters: None.
 * Returns:    None.
 ******************************************************************************/
static void HalWeatherWakeUp(void)
{
  // Microcontroller I2C init for this sensor
  HalWeatherSelect();
  // *** Configure BME280 ***
  // Set ctrl_Filter
  HalSensorWriteReg(BME280_CONFIG_REG, &ctrl_Filter, sizeof(ctrl_Filter));
  // Set ctrl_Measure
  HalSensorWriteReg(BME280_CTRL_MEAS_REG, &ctrl_Measure, sizeof(ctrl_Measure));
  // Set ctrl_Humidity
  HalSensorWriteReg(BME280_CTRL_HUMIDITY_REG, &ctrl_Humi, sizeof(ctrl_Humi));
}

/******************************************************************************
 * Function:   Read all data in the sensor.
 * Parameters: pBuf. Pointer to buffer to store the data
 * Returns:    0. No errors.
 *             1. Error in comunication.
 *             2. Measure error.
 *             3. Brust data error.
 ******************************************************************************/
bool HalWeatherReadData(uint8 cfg, uint8 *pBuf)
{
  uint8 buffer[8];
  int32 Temp, Pres, Humi;
  int32 uTemp, uPres, uHumi;

  bool success;

  // Microcontroller I2C init for this sensor
  HalWeatherSelect();

  // Check communication
  if (HalWeatherTest() != 1)
    return 1; // comunication error
  
  // WakeUp the sensor
  HalWeatherWakeUp();

  // Check if the meassure is completed
  HalSensorReadBit(BME280_STAT_REG, 
             buffer, 
             BME280_STAT_REG_MEASURING__POS);
  
  if (buffer[0] == 1)
    return 2; // measure error

  //************************************************************************
  // Brust all data and compensate if it is selected
  //************************************************************************

  success = HalSensorReadReg(BME280_PRESSURE_MSB_REG, buffer, 8);
  // Compose Pressure
  uPres = 
    ((int32)(buffer[0]) << 16) + 
    ((int32)(buffer[1]) << 8)  + 
    ((int32)(buffer[2]));
  uPres >>= 4;

  // Compose Temperature
  uTemp = 
    ((int32)(buffer[3]) << 16) + 
    ((int32)(buffer[4]) << 8)  + 
    ((int32)(buffer[5]));
  uTemp >>= 4;

  // Compose Humidity
  uHumi = 
    ((int32)(buffer[6]) << 8)  + 
    ((int32)(buffer[7]));

  
  Pres = compensatePres(uPres);
//  Pres = uPres;
  Humi = compensateHum(uHumi);
//  Humi = uHumi;
  Temp = compensateTemp(uTemp);
//  Temp = uTemp;

  if (success)
  {
    // Valid data
    pBuf[0] = (uint8)((Pres >> 0) & 0x000000FF); 
    pBuf[1] = (uint8)((Pres >> 8) & 0x000000FF); 
    pBuf[2] = (uint8)((Pres >> 16) & 0x000000FF); 
    // Valid data
    pBuf[3] = (uint8)((Humi >> 0) & 0x000000FF); 
    pBuf[4] = (uint8)((Humi >> 8) & 0x000000FF); 
    pBuf[5] = (uint8)((Humi >> 16) & 0x000000FF); 
    // Valid data
    pBuf[6] = (uint8)((Temp >> 0) & 0x000000FF); 
    pBuf[7] = (uint8)((Temp >> 8) & 0x000000FF); 
    
    return 0; // no errors
  }
  else
    return 3; // brust data error
  
  // BME280 AutoSleep before forced reading data (in forced mode)
  // so no actions to sleep are need.
}

/* -----------------------------------------------------------------------------
 *                              Private functions
 * -----------------------------------------------------------------------------
 */

/******************************************************************************
 * Function:  This function reads the calibration registers
 * Returns:   Nothing
 ******************************************************************************/
void readCalibration()
{
  uint8 buffer[4];

  HalWeatherSelect();
 
  // dig_T1
  HalSensorReadReg(BME280_DIG_T1_LSB_REG, buffer, 2);
  dig_T1 = ((uint16)(buffer[1]) << 8) + buffer[0];
  
  // dig_T2
  HalSensorReadReg(BME280_DIG_T2_LSB_REG, buffer, 2);
  dig_T2 = ((int16)(buffer[1]) << 8) + buffer[0];
  
  // dig_T3
  HalSensorReadReg(BME280_DIG_T3_LSB_REG, buffer, 2);
  dig_T3 = ((int16)(buffer[1]) << 8) + buffer[0];
  
  // dig_P1
  HalSensorReadReg(BME280_DIG_P1_LSB_REG, buffer, 2);
  dig_P1 = ((uint16)(buffer[1]) << 8) + buffer[0];
  
  // dig_P2
  HalSensorReadReg(BME280_DIG_P2_LSB_REG, buffer, 2);
  dig_P2 = ((int16)(buffer[1]) << 8) + buffer[0];
  
  // dig_P3
  HalSensorReadReg(BME280_DIG_P3_LSB_REG, buffer, 2);
  dig_P3 = ((int16)(buffer[1]) << 8) + buffer[0];
  
  // dig_P4
  HalSensorReadReg(BME280_DIG_P4_LSB_REG, buffer, 2);
  dig_P4 = ((int16)(buffer[1]) << 8) + buffer[0];
  
  // dig_P5
  HalSensorReadReg(BME280_DIG_P5_LSB_REG, buffer, 2);
  dig_P5 = ((int16)(buffer[1]) << 8) + buffer[0];
  
  // dig_P6
  HalSensorReadReg(BME280_DIG_P6_LSB_REG, buffer, 2);
  dig_P6 = ((int16)(buffer[1]) << 8) + buffer[0];
  
  // dig_P7
  HalSensorReadReg(BME280_DIG_P7_LSB_REG, buffer, 2);
  dig_P7 = ((int16)(buffer[1]) << 8) + buffer[0];
  
  // dig_P8
  HalSensorReadReg(BME280_DIG_P8_LSB_REG, buffer, 2);
  dig_P8 = ((int16)(buffer[1]) << 8) + buffer[0];
  
  // dig_P9
  HalSensorReadReg(BME280_DIG_P9_LSB_REG, buffer, 2);
  dig_P9 = ((int16)(buffer[1]) << 8) + buffer[0];

  // dig_H1
  HalSensorReadReg(BME280_DIG_H1_REG, &dig_H1, 1);
  
  // dig_H2
  HalSensorReadReg(BME280_DIG_H2_LSB_REG, buffer, 2);
  dig_H2 = ((int16)(buffer[1]) << 8) + buffer[0];

  // dig_H3
  HalSensorReadReg(BME280_DIG_H3_REG, &dig_H3, 1);
  
  // dig_H4 and dig_H5
  HalSensorReadReg(BME280_DIG_H4_MSB_REG, buffer, 3);
  dig_H4 = ((int16)(buffer[0]) << 4) + (buffer[1] & 0x0F);
  
  dig_H5 = ((int16)(buffer[1] & 0xF0) << 4) + buffer[2];
  dig_H5 &= 0x0FFF; //?? maybe a sign loss

  // dig_H6
  HalSensorReadReg(BME280_DIG_T2_LSB_REG, buffer, 1);
  dig_H6 = buffer[0]; 
  
}

/******************************************************************************
 * Function:   Put OUT calibration information for external data compensation
 *             calculation.
 * Parameters: pBuf. Pointer to buffer to store the data
 *             data. Data to show in pBuf:
 *             BME280_GET_T, BME280_GET_Pa, BME280_GET_Pb and BME280_GET_H                
 * Returns:    Nothing
 ******************************************************************************/
void HalWeatherShowCalibration(uint8 data, uint8 *pBuf)
{

  switch (data)
  {
    case BME280_GET_T:
      pBuf[0] = (uint8)((dig_T1 >> 0) & 0x00FF); 
      pBuf[1] = (uint8)((dig_T1 >> 8) & 0x00FF); 
      pBuf[2] = (uint8)((dig_T2 >> 0) & 0x00FF); 
      pBuf[3] =  (int8)((dig_T2 >> 8) & 0x00FF); 
      pBuf[4] = (uint8)((dig_T3 >> 0) & 0x00FF); 
      pBuf[5] =  (int8)((dig_T3 >> 8) & 0x00FF); 
    break;
 
    case BME280_GET_Pa:
      pBuf[0] = (uint8)((dig_P1 >> 0) & 0x00FF); 
      pBuf[1] = (uint8)((dig_P1 >> 8) & 0x00FF); 
      pBuf[2] = (uint8)((dig_P2 >> 0) & 0x00FF); 
      pBuf[3] =  (int8)((dig_P2 >> 8) & 0x00FF); 
      pBuf[4] = (uint8)((dig_P3 >> 0) & 0x00FF); 
      pBuf[5] =  (int8)((dig_P3 >> 8) & 0x00FF); 
      pBuf[6] = (uint8)((dig_P4 >> 0) & 0x00FF); 
      pBuf[7] =  (int8)((dig_P4 >> 8) & 0x00FF); 
      pBuf[8] = (uint8)((dig_P5 >> 0) & 0x00FF); 
    break;

    case BME280_GET_Pb:
      pBuf[0] =  (int8)((dig_P5 >> 8) & 0x00FF); 
      pBuf[1] = (uint8)((dig_P6 >> 0) & 0x00FF); 
      pBuf[2] =  (int8)((dig_P6 >> 8) & 0x00FF); 
      pBuf[3] = (uint8)((dig_P7 >> 0) & 0x00FF); 
      pBuf[4] =  (int8)((dig_P7 >> 8) & 0x00FF); 
      pBuf[5] = (uint8)((dig_P8 >> 0) & 0x00FF); 
      pBuf[6] =  (int8)((dig_P8 >> 8) & 0x00FF); 
      pBuf[7] = (uint8)((dig_P9 >> 0) & 0x00FF); 
      pBuf[8] =  (int8)((dig_P8 >> 8) & 0x00FF); 
    break;
    
    case BME280_GET_H:
      pBuf[0] = (uint8)((dig_H1 >> 0) & 0x00FF); 
      pBuf[1] = (uint8)((dig_H2 >> 0) & 0x00FF); 
      pBuf[2] =  (int8)((dig_H2 >> 8) & 0x00FF); 
      pBuf[3] = (uint8)((dig_H3 >> 0) & 0x00FF); 
      pBuf[4] = (uint8)((dig_H4 >> 0) & 0x00FF); 
      pBuf[5] =  (int8)((dig_H4 >> 8) & 0x00FF); 
      pBuf[6] = (uint8)((dig_H5 >> 0) & 0x00FF); 
      pBuf[7] =  (int8)((dig_H5 >> 8) & 0x00FF); 
      pBuf[8] =  (int8)((dig_H6 >> 0) & 0x00FF); 
    break;    
  }
  
}

/*******************************************************************************
 * @fn     HalWeatherSelect.
 *
 * @brief  Select the temparature, humidity and pressure BME280 sensor on 
 *         the I2C-bus.
 *
 * @return None.
 ******************************************************************************/
static void HalWeatherSelect(void)
{
  //Set up I2C that is used to communicate with BME280
  HalI2CInit(HAL_BME280_I2C_ADDRESS,i2cClock_267KHZ);
}

/// INTERNAL COMPENSATION FUNCTIONS

/******************************************************************************
 * Function: This function compensates the temperature with calibration data.
 * Parameters: uncompensated_temp: uncompensated temperature value.
 * Returns: Returns temperature in DegC, resolution is 0.01 DegC. 
 * Example: Output value of “5123” equals 51.23 DegC.
 *          t_fine carries fine temperature as global value.
 * Source: Api de Bosch. int32_t BME280_compensate_T_int32(BME280_S32_t adc_T)
 ******************************************************************************/
int32 compensateTemp(int32 uncompensated_temp)
{
  
  int32 var1, var2, T;
  
  var1 = ((((uncompensated_temp >> 3) - ((int32)dig_T1 << 1))) * ((int32)dig_T2)) >> 11;
  var2 = (((((uncompensated_temp >> 4) - ((int32)dig_T1)) * ((uncompensated_temp >> 4) - 
         ((int32)dig_T1))) >> 12) * ((int32)dig_T3)) >> 14;
  t_fine = var1 + var2;
  T = ((t_fine * 5 + 128) >> 8);
   
  return (T);
}


/******************************************************************************
 * Function: This function compensates the pressure with the calibration data.
 * Parameters: uncompensated_pres: uncompensated pressure value.
 * Returns: Returns pressure in Pa as unsigned 32 bit integer. 
 * Example: Output value of “96386” equals 96386 Pa = 963.86 hPa.
 * Source: Api de Bosch. uint32_t BME280_compensate_P_int32(BME280_S32_t adc_P)
 ******************************************************************************/
uint32 compensatePres(int32 uncompensated_pres)
{
  
  int32 var1, var2;
  uint32 p;
    
  var1 = ((int32)t_fine >> 1) - (int32)64000;
  var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32) dig_P6);
  var2 = var2 + ((var1 * (int32)dig_P5) << 1);
  var2 = (var2 >> 2) + (((int32)dig_P4) << 16);
  var1 = (((dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + 
         ((((int32)dig_P2) * var1)>>1))>>18;
  var1 =((((32768+var1))*((int32)dig_P1))>>15);
  
  if (var1 == 0)
    return 0; // avoid exception caused by division by zero

  p = (((uint32)(((int32)1048576)-uncompensated_pres)-(var2>>12)))*3125;
  if (p < 0x80000000)
    p = (p << 1) / ((uint32)var1);
  else
    p = (p / (uint32)var1) * 2;

  var1 = (((int32)dig_P9) * ((int32)(((p>>3) * (p>>3))>>13)))>>12;
  var2 = (((int32)(p>>2)) * ((int32)dig_P8))>>13;
  p = (uint32)((int32)p + ((var1 + var2 + dig_P7) >> 4));
  return p;
}

/******************************************************************************
 * Function: This function compensates the humidity with the calibration data.
 * Parameters: uncompensated_hum: uncompensated humidity value.
 * Returns: Returns humidity in %RH as unsigned 32 bit integer in 
 *          Q22.10 format (22 integer and 10 fractional bits).
 * Example: Output value of “47445” represents 47445/1024 = 46.333 %RH
 * Source: Api de Bosch. uint32_t bme280_compensate_H_int32(BME280_S32_t adc_H)
 ******************************************************************************/
uint32 compensateHum (int32 uncompensated_hum)
{
  int32 v_x1_u32r;
		
  v_x1_u32r = (t_fine - ((int32)76800));
  v_x1_u32r = (((((uncompensated_hum << 14) - (((int32)dig_H4) << 20) - 
              (((int32)dig_H5) * v_x1_u32r)) + ((int32)16384)) >> 15) * 
              (((((((v_x1_u32r * ((int32)dig_H6)) >> 10) * (((v_x1_u32r * 
              ((int32)dig_H3)) >> 11) + ((int32)32768))) >> 10) + 
              ((int32)2097152)) * ((int32)dig_H2) + 8192) >> 14));
  v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * 
              ((int32)dig_H1)) >> 4));
	
  
  v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
  v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
  
  return (v_x1_u32r >> 12);
}

// For implement in Libelium Dragonfly API
///**************************************************************************/
///*!
//    Calculates the altitude (in meters) from the specified atmospheric
//    pressure (in hPa), and sea-level pressure (in hPa).
//
//    @param  seaLevel      Sea-level pressure in hPa
//    @param  atmospheric   Atmospheric pressure in hPa
//*/
///**************************************************************************/
//float readAltitude(float seaLevel)
//{
//  // Equation taken from BMP180 datasheet (page 16):
//  //  http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf
//
//  // Note that using the equation from wikipedia can give bad results
//  // at high altitude.  See this thread for more information:
//  //  http://forums.adafruit.com/viewtopic.php?f=22&t=58064
//
//  float atmospheric = readPressure() / 100.0F;
//  return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
//}


#endif // (defined HAL_WEATHER) && (HAL_WEATHER == TRUE)


/*******************************************************************************
  Filename:       hal_weather.h
  Revised:        $Date: 2016-03-14 11:45:31$
  Revision:       $Revision: 0 $

  Description:    This file contains the declaration Bosch BME280 Humidity, 
                  Presure and Temperature sensor abstraction layer.
*******************************************************************************/

#ifndef HAL_WEATHER_H
#define HAL_WEATHER_H

#ifdef __cplusplus
extern "C" {
#endif

/* -----------------------------------------------------------------------------
 *                                  Includes
 * -----------------------------------------------------------------------------
 */

#include "comdef.h"

/* -----------------------------------------------------------------------------
 *                              BME280 Constants
 * -----------------------------------------------------------------------------
 */

//#define HAL_BME280_I2C_ADDRESS          0x76    // I2C Address 0x01110110    
#define HAL_BME280_I2C_ADDRESS          0x77    // I2C Address 0x01110111    
    
// Register address
/*calibration registers */
#define BME280_DIG_T1_LSB_REG           0x88
#define BME280_DIG_T1_MSB_REG           0x89
#define BME280_DIG_T2_LSB_REG           0x8A
#define BME280_DIG_T2_MSB_REG           0x8B
#define BME280_DIG_T3_LSB_REG           0x8C
#define BME280_DIG_T3_MSB_REG           0x8D

#define BME280_DIG_P1_LSB_REG           0x8E
#define BME280_DIG_P1_MSB_REG           0x8F
#define BME280_DIG_P2_LSB_REG           0x90
#define BME280_DIG_P2_MSB_REG           0x91
#define BME280_DIG_P3_LSB_REG           0x92
#define BME280_DIG_P3_MSB_REG           0x93
#define BME280_DIG_P4_LSB_REG           0x94
#define BME280_DIG_P4_MSB_REG           0x95
#define BME280_DIG_P5_LSB_REG           0x96
#define BME280_DIG_P5_MSB_REG           0x97
#define BME280_DIG_P6_LSB_REG           0x98
#define BME280_DIG_P6_MSB_REG           0x99
#define BME280_DIG_P7_LSB_REG           0x9A
#define BME280_DIG_P7_MSB_REG           0x9B
#define BME280_DIG_P8_LSB_REG           0x9C
#define BME280_DIG_P8_MSB_REG           0x9D
#define BME280_DIG_P9_LSB_REG           0x9E
#define BME280_DIG_P9_MSB_REG           0x9F

#define BME280_DIG_H1_REG               0xA1
#define BME280_DIG_H2_LSB_REG           0xE1
#define BME280_DIG_H2_MSB_REG           0xE2
#define BME280_DIG_H3_REG               0xE3
#define BME280_DIG_H4_MSB_REG           0xE4
#define BME280_DIG_H5_LSB_H4_LSB_REG    0xE5
#define BME280_DIG_H5_MSB_REG           0xE6
#define BME280_DIG_H6_REG               0xE7

/*system registers */
#define BME280_CHIP_ID_REG              0xD0    // Chip ID Register
#define BME280_RST_REG                  0xE0    // Softreset Register
#define BME280_STAT_REG                 0xF3    // Status Register
#define BME280_CTRL_MEAS_REG            0xF4    // Ctrl Measure Register
#define BME280_CTRL_HUMIDITY_REG        0xF2    // Ctrl Humidity Register
#define BME280_CONFIG_REG               0xF5    // Configuration Register

/*data registers */
#define BME280_PRESSURE_MSB_REG         0xF7    // Pressure MSB Register
#define BME280_PRESSURE_LSB_REG         0xF8    // Pressure LSB Register
#define BME280_PRESSURE_XLSB_REG        0xF9    // Pressure XLSB Register
#define BME280_TEMPERATURE_MSB_REG      0xFA    // Temperature MSB Reg
#define BME280_TEMPERATURE_LSB_REG      0xFB    // Temperature LSB Reg
#define BME280_TEMPERATURE_XLSB_REG     0xFC    // Temperature XLSB Reg
#define BME280_HUMIDITY_MSB_REG         0xFD    // Humidity MSB Reg
#define BME280_HUMIDITY_LSB_REG         0xFE    // Humidity LSB Reg

   
// ID Register (0xD0)
#define BME280_CHIP_ID_REG_CHIP_ID      0x60

// Status Register (0xF3)
#define BME280_STAT_REG_MEASURING__POS      3
#define BME280_STAT_REG_MEASURING__MSK      0x08
#define BME280_STAT_REG_MEASURING__LEN      1

#define BME280_STAT_REG_IM_UPDATE__POS      0
#define BME280_STAT_REG_IM_UPDATE__MSK      0x01
#define BME280_STAT_REG_IM_UPDATE__LEN      1

// Control Measurement Register (0xF4)
#define BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__POS      5
#define BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__MSK      0xE0
#define BME280_CTRL_MEAS_REG_OVERSAMP_TEMPERATURE__LEN      3

#define BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__POS         2
#define BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__MSK         0x1C
#define BME280_CTRL_MEAS_REG_OVERSAMP_PRESSURE__LEN         3

#define BME280_CTRL_MEAS_REG_POWER_MODE__POS                0
#define BME280_CTRL_MEAS_REG_POWER_MODE__MSK                0x03
#define BME280_CTRL_MEAS_REG_POWER_MODE__LEN                2

// Control Humidity Register (0xF2)
#define BME280_CTRL_HUMIDITY_REG_OVERSAM_HUMIDITY__POS      0
#define BME280_CTRL_HUMIDITY_REG_OVERSAM_HUMIDITY__MSK      0x07
#define BME280_CTRL_HUMIDITY_REG_OVERSAM_HUMIDITY__LEN      3

// Configuration Register (0xF5)
#define BME280_CONFIG_REG_TSB__POS              5
#define BME280_CONFIG_REG_TSB__MSK              0xE0
#define BME280_CONFIG_REG_TSB__LEN              3

#define BME280_CONFIG_REG_FILTER__POS           2
#define BME280_CONFIG_REG_FILTER__MSK           0x1C
#define BME280_CONFIG_REG_FILTER__LEN           3

#define BME280_CONFIG_REG_SPI3_ENABLE__POS      0
#define BME280_CONFIG_REG_SPI3_ENABLE__MSK      0x01
#define BME280_CONFIG_REG_SPI3_ENABLE__LEN      1

// XLSB Data Register for Pressure and Temperature (0xF9 0xFC)
#define BME280_PRESSURE_XLSB_REG_DATA__POS          4
#define BME280_PRESSURE_XLSB_REG_DATA__MSK          0xF0
#define BME280_PRESSURE_XLSB_REG_DATA__LEN          4

#define BME280_TEMPERATURE_XLSB_REG_DATA__POS       4
#define BME280_TEMPERATURE_XLSB_REG_DATA__MSK       0xF0
#define BME280_TEMPERATURE_XLSB_REG_DATA__LEN       4


//! Sensor Specific constants for Control Measurement Register (0xF4)
#define BME280_SLEEP_MODE               0x00
#define BME280_FORCED_MODE              0x01
#define BME280_NORMAL_MODE              0x03
#define BME280_SOFT_RESET_CODE          0xB6

//! Sensor Specific constants for Configuration Register (0xF5)
#define BME280_STANDBY_TIME_1_MS        0x00
#define BME280_STANDBY_TIME_63_MS       0x01
#define BME280_STANDBY_TIME_125_MS      0x02
#define BME280_STANDBY_TIME_250_MS      0x03
#define BME280_STANDBY_TIME_500_MS      0x04
#define BME280_STANDBY_TIME_1000_MS     0x05
#define BME280_STANDBY_TIME_10_MS       0x06
#define BME280_STANDBY_TIME_20_MS       0x07

// Oversampling constants 
// for Control Humidity Register (0xF2) 
// and Control Measurement Register (0xF4)
#define BME280_OVERSAMP_SKIPPED         0x00
#define BME280_OVERSAMP_1X              0x01
#define BME280_OVERSAMP_2X              0x02
#define BME280_OVERSAMP_4X              0x03
#define BME280_OVERSAMP_8X              0x04
#define BME280_OVERSAMP_16X             0x05

// Filter constants for Configuration Register (0xF5)
#define BME280_FILTER_COEFF_OFF         0x00
#define BME280_FILTER_COEFF_2           0x01
#define BME280_FILTER_COEFF_4           0x02
#define BME280_FILTER_COEFF_8           0x03
#define BME280_FILTER_COEFF_16          0x04

// Calibrarion get data constants
#define BME280_GET_T                    0x00
#define BME280_GET_Pa                   0x01
#define BME280_GET_Pb                   0x02
#define BME280_GET_H                    0x03
   
// Weather MODES
#define BME280_MODE_EN_T                0x01
#define BME280_MODE_EN_P                0x02
#define BME280_MODE_EN_H                0x04
#define BME280_MODE_MONITORING          0x08
#define BME280_MODE_AIR_QUALITY         0x10
#define BME280_MODE_INDOOR_NAVIGATION   0x18
#define BME280_MODE_INDOOR_GAMING       0x20

#define BME280_MODE_MASK                0x07
 
// Add here new MODES


/* -----------------------------------------------------------------------------
 *                                 Typedefs
 * -----------------------------------------------------------------------------
 */


/* -----------------------------------------------------------------------------
 *                                 Functions
 * -----------------------------------------------------------------------------
 */

// CONFIGURATION AND TEST FUNCTIONS    
void HalWeatherInit(void);
bool HalWeatherTest(void);
void HalWeatherSetMode(uint8 cfg_mode);

void HalWeatherShowCalibration(uint8 data, uint8 *pBuf);

// MEASURE FUNCTIONS 
bool HalWeatherReadData(uint8 cfg, uint8 *pBuf);

   
/******************************************************************************/

#ifdef __cplusplus
};
#endif

#endif

/******************************************************************************/


/*******************************************************************************
  Filename:       hal_lux.h
  Revised:        $Date: 2016-03-30 11:45:31$
  Revision:       $Revision: 0 $

  Description:    This file contains the declaration of TSL2561 T package
                  light-to-digital sensor abstraction layer.
*******************************************************************************/

#ifndef HAL_LUX_H
#define HAL_LUX_H

#ifdef __cplusplus
extern "C" {
#endif

/* -----------------------------------------------------------------------------
 *                                  Includes
 * -----------------------------------------------------------------------------
 */

#include "comdef.h"

/* -----------------------------------------------------------------------------
 *                             TSL2561 Constants
 * -----------------------------------------------------------------------------
 */
  
#define TSL2561_VISIBLE 2                   // channel 0 - channel 1
#define TSL2561_INFRARED 1                  // channel 1
#define TSL2561_FULLSPECTRUM 0              // channel 0

// I2C address options
#define TSL2561_ADDR_LOW          (0x29)
#define TSL2561_ADDR_FLOAT        (0x39)    // Default address (pin left floating)
#define TSL2561_ADDR_HIGH         (0x49)

// Lux calculations Enable
#define LUX_CALC
   
// Lux calculations differ slightly for CS package
//#define TSL2561_PACKAGE_CS
#define TSL2561_PACKAGE_T_FN_CL

#define TSL2561_COMMAND_BIT       (0x80)    // Must be 1
#define TSL2561_CLEAR_BIT         (0x40)    // Clears any pending interrupt (write 1 to clear)
#define TSL2561_WORD_BIT          (0x20)    // 1 = read/write word (rather than byte)
#define TSL2561_BLOCK_BIT         (0x10)    // 1 = using block read/write

#define TSL2561_CONTROL_POWERON   (0x03)
#define TSL2561_CONTROL_POWEROFF  (0x00)

#define TSL2561_LUX_LUXSCALE      (14)      // Scale by 2^14
#define TSL2561_LUX_RATIOSCALE    (9)       // Scale ratio by 2^9
#define TSL2561_LUX_CHSCALE       (10)      // Scale channel values by 2^10
#define TSL2561_LUX_CHSCALE_TINT0 (0x7517)  // 322/11 * 2^TSL2561_LUX_CHSCALE
#define TSL2561_LUX_CHSCALE_TINT1 (0x0FE7)  // 322/81 * 2^TSL2561_LUX_CHSCALE

// T, FN and CL package values
#define TSL2561_LUX_K1T           (0x0040)  // 0.125 * 2^RATIO_SCALE
#define TSL2561_LUX_B1T           (0x01f2)  // 0.0304 * 2^LUX_SCALE
#define TSL2561_LUX_M1T           (0x01be)  // 0.0272 * 2^LUX_SCALE
#define TSL2561_LUX_K2T           (0x0080)  // 0.250 * 2^RATIO_SCALE
#define TSL2561_LUX_B2T           (0x0214)  // 0.0325 * 2^LUX_SCALE
#define TSL2561_LUX_M2T           (0x02d1)  // 0.0440 * 2^LUX_SCALE
#define TSL2561_LUX_K3T           (0x00c0)  // 0.375 * 2^RATIO_SCALE
#define TSL2561_LUX_B3T           (0x023f)  // 0.0351 * 2^LUX_SCALE
#define TSL2561_LUX_M3T           (0x037b)  // 0.0544 * 2^LUX_SCALE
#define TSL2561_LUX_K4T           (0x0100)  // 0.50 * 2^RATIO_SCALE
#define TSL2561_LUX_B4T           (0x0270)  // 0.0381 * 2^LUX_SCALE
#define TSL2561_LUX_M4T           (0x03fe)  // 0.0624 * 2^LUX_SCALE
#define TSL2561_LUX_K5T           (0x0138)  // 0.61 * 2^RATIO_SCALE
#define TSL2561_LUX_B5T           (0x016f)  // 0.0224 * 2^LUX_SCALE
#define TSL2561_LUX_M5T           (0x01fc)  // 0.0310 * 2^LUX_SCALE
#define TSL2561_LUX_K6T           (0x019a)  // 0.80 * 2^RATIO_SCALE
#define TSL2561_LUX_B6T           (0x00d2)  // 0.0128 * 2^LUX_SCALE
#define TSL2561_LUX_M6T           (0x00fb)  // 0.0153 * 2^LUX_SCALE
#define TSL2561_LUX_K7T           (0x029a)  // 1.3 * 2^RATIO_SCALE
#define TSL2561_LUX_B7T           (0x0018)  // 0.00146 * 2^LUX_SCALE
#define TSL2561_LUX_M7T           (0x0012)  // 0.00112 * 2^LUX_SCALE
#define TSL2561_LUX_K8T           (0x029a)  // 1.3 * 2^RATIO_SCALE
#define TSL2561_LUX_B8T           (0x0000)  // 0.000 * 2^LUX_SCALE
#define TSL2561_LUX_M8T           (0x0000)  // 0.000 * 2^LUX_SCALE

// Auto-gain thresholds
#define TSL2561_AGC_THI_13MS      (4850)    // Max value at Ti 13ms = 5047
#define TSL2561_AGC_TLO_13MS      (100)
#define TSL2561_AGC_THI_101MS     (36000)   // Max value at Ti 101ms = 37177
#define TSL2561_AGC_TLO_101MS     (200)
#define TSL2561_AGC_THI_402MS     (63000)   // Max value at Ti 402ms = 65535
#define TSL2561_AGC_TLO_402MS     (500)

// Clipping thresholds
#define TSL2561_CLIPPING_13MS     (4900)
#define TSL2561_CLIPPING_101MS    (37000)
#define TSL2561_CLIPPING_402MS    (65000)

// Configuration Register /////////////////////////////////////////////////////
// Luxometer data
#define TSL2561_MODE_DATA_MASK               0x07
#define TSL2561_MODE_DATA_POS                0x00
#define TSL2561_MODE_EN_LUX                  0x01
#define TSL2561_MODE_EN_BB                   0x02
#define TSL2561_MODE_EN_IR                   0x04
// Luxometer mode
#define TSL2561_MODE_CTRL_MASK               0x01
#define TSL2561_MODE_CTRL_POS                0x03
#define TSL2561_MODE_MONITORING              0x00 
#define TSL2561_MODE_INTERRUPTION            0x01 
// Luxometer gain
#define TSL2561_MODE_GAIN_MASK               0x03
#define TSL2561_MODE_GAIN_POS                0x05
#define TSL2561_MODE_LOW_GAIN                0x00 
#define TSL2561_MODE_HIGH_GAIN               0x01
#define TSL2561_MODE_AUTO_GAIN               0x02
// Loxometer integration time
#define TSL2561_MODE_TIME_MASK               0x03
#define TSL2561_MODE_TIME_POS                0x06
#define TSL2561_MODE_TIME_13MS               0x00
#define TSL2561_MODE_TIME_101MS              0x01
#define TSL2561_MODE_TIME_402MS              0x02

// Register address
enum
{
  TSL2561_REGISTER_CONTROL          = 0x00,
  TSL2561_REGISTER_TIMING           = 0x01,
  TSL2561_REGISTER_THRESHHOLDL_LOW  = 0x02,
  TSL2561_REGISTER_THRESHHOLDL_HIGH = 0x03,
  TSL2561_REGISTER_THRESHHOLDH_LOW  = 0x04,
  TSL2561_REGISTER_THRESHHOLDH_HIGH = 0x05,
  TSL2561_REGISTER_INTERRUPT        = 0x06,
  TSL2561_REGISTER_CRC              = 0x08,
  TSL2561_REGISTER_ID               = 0x0A,
  TSL2561_REGISTER_CHAN0_LOW        = 0x0C,
  TSL2561_REGISTER_CHAN0_HIGH       = 0x0D,
  TSL2561_REGISTER_CHAN1_LOW        = 0x0E,
  TSL2561_REGISTER_CHAN1_HIGH       = 0x0F
};

// Luxometer State ////////////////////////////////////////////////////////////
#define HAL_LUX_MEAS_STATE_0         0    // Start lux. meas.
#define HAL_LUX_MEAS_STATE_1         1    // Read lux. data.

/* -----------------------------------------------------------------------------
 *                                 Typedefs
 * -----------------------------------------------------------------------------
 */

typedef enum
{
  TSL2561_INTEGRATIONTIME_13MS      = 0x00,    // 13.7ms
  TSL2561_INTEGRATIONTIME_101MS     = 0x01,    // 101ms
  TSL2561_INTEGRATIONTIME_402MS     = 0x02     // 402ms
}
IntegrationTime_t;

typedef enum
{
  TSL2561_GAIN_1X                   = 0x00,    // No gain
  TSL2561_GAIN_16X                  = 0x10,    // 16x gain
}
Gain_t;

/* -----------------------------------------------------------------------------
 *                                 Functions
 * -----------------------------------------------------------------------------
 */
// CONFIGURATION AND TEST FUNCTIONS   
void HalLuxInit(void);
bool HalLuxTest(void);
void HalLuxSetMode(uint8 cfg_mode);

// MEASURE FUNCTIONS   

extern bool HalLuxExecMeasurementStep(uint8 state);
extern bool HalLuxReadMeasurement(uint8 *pBuf);




//class Adafruit_TSL2561_Unified : public Adafruit_Sensor {
// public:
//  Adafruit_TSL2561_Unified(uint8_t addr, int32_t sensorID = -1);
//  boolean begin(void);
//  
//  /* TSL2561 Functions */
//  void enableAutoRange(bool enable);
//  void setIntegrationTime(tsl2561IntegrationTime_t time);
//  void setGain(tsl2561Gain_t gain);
//  void getLuminosity (uint16_t *broadband, uint16_t *ir);
//  uint32_t calculateLux(uint16_t broadband, uint16_t ir);
//  
//  /* Unified Sensor API Functions */  
//  bool getEvent(sensors_event_t*);
//  void getSensor(sensor_t*);
//
// private:
//  int8_t _addr;
//  boolean _tsl2561Initialised;
//  boolean _tsl2561AutoGain;
//  tsl2561IntegrationTime_t _tsl2561IntegrationTime;
//  tsl2561Gain_t _tsl2561Gain;
//  int32_t _tsl2561SensorID;
//  
//  void     enable (void);
//  void     disable (void);
//  void     write8 (uint8_t reg, uint32_t value);
//  uint8_t  read8 (uint8_t reg);
//  uint16_t read16 (uint8_t reg);
//  void     getData (uint16_t *broadband, uint16_t *ir);
//};
#endif

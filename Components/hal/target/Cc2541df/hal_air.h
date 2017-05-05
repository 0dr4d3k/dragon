/**************************************************************************************************
  Filename:       hal_air.h
  Revised:        $Date: 2016-03-14 11:45:31$
  Revision:       $Revision: 0 $

  Description:    This file contains the declaration Indoor Air Quality (IAQ) to the HAL CCS811 
                  abstraction layer.
**************************************************************************************************/

#ifndef HAL_AIR_H
#define HAL_AIR_H

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */

#include "comdef.h"

/* ------------------------------------------------------------------------------------------------
 *                                          Constants
 * ------------------------------------------------------------------------------------------------
 */
// Debug info 0 -> no debug info
//           >0 -> [debug][air] messages   
//           >1 -> [debug][scan] I2C scan devices and registers dump 
#define AIR_DEBUG  0    

/* ------------------------------------------------------------------------------------------------
*                                        CCS811 Constants
* ------------------------------------------------------------------------------------------------
*/

#define HAL_CCS811_I2C_ADDRESS      0x5A    // I2C Address pin I2C_ADDR=LOW    
//#define HAL_CCS811_I2C_ADDRESS      0x5B    // I2C Address pin I2C_ADDR=HIGH    
  
// CCS811 addressing space
#define AIR_STATUS                  0x00    // R   1_byte.  Satus register.
#define AIR_MEAS_MODE               0x01    // R/W 1_byte.  Measurement mode and condiction register.
#define AIR_ALG_RESULT_DATA         0x02    // R   8_bytes. Processed Data. eCO2_H:eCO2_L:TVOC_H:TVOC_L:STATUS:ERROR_ID:RAW_DATA_H:RAW_DATAA_L.
#define AIR_RAW_DATA                0x03    // R   2_bytes. Raw Data. (15-10):Source, (9-0): ADC Raw Data.
#define AIR_ENV_DATA                0x04    // R/W 4_bytes. Compensation data. HUM_H:HUM_L:T_H:T_L.
#define AIR_NTC                     0x06    // R   4_bytes. NTC compensation. vREF_H:vRED_L:vNTC_H:vNTC_L.
#define AIR_THRESHOLDS              0x10    // R/W 5_bytes. Thresholds for INT. L2M_H:L2M_L:M2H_H:M2H_L:HYS.
#define AIR_BASELINE                0x11    // R/W 2_bytes. Current baseline.
#define AIR_HW_ID                   0x20    // R   1_byte.  Hardware ID. The value is 0x81.
#define AIR_HW_VERSION              0x21    // R   1_byte.  Hardware Version. The value is 0x11.
#define AIR_FW_BOOT_VERSION         0x23    // R   2_bytes. Firmware Boot Version.
#define AIR_FW_BPP_VERSION          0x24    // R   2_bytes. Firmware Application Version.
#define AIR_ERROR_ID                0xE0    // R   1_byte.  Error ID.  
#define AIR_APP_START               0xF4    // W   0_bytes. Application start. A write with no data is required.
#define AIR_SW_RESET                0xFF    // W   4_bytes. Write 0x11:0xE5:0x72:0x8A reset and return to BOOT mode.   

// 0x00 (7:FW_MODE, 4:APP_VALID, 3:DATA_READY, 0:ERROR) register - Status Register
#define AIR_FW_MODE                 0x80    //  Bit 7: 0('1') boot(application) mode.
#define AIR_APP_VALID               0x10    //  Bit 4: 0('1') no_application(application) firmware loaded.
#define AIR_DATA_READY              0x08    //  Bit 3: 0('1') no_data(data) in AIR_ALG_RESULT_DATA. This bit is cleared when data readed.
#define AIR_ERROR                   0x01    //  Bit 0: 0('1') no_error(error) in sensor. AIR_ERROR_ID contains the error source.

// 0x01 (6:4 DRIVER_MODE, 3:INTERRUPT, 2:THRESH) register- Measurement and condictions.
#define AIR_MODE_MASK               0x8F    // Bits 6:4 (0x1'000'1111) Mask for insolate Mode bits in Register.    
#define AIR_MODE0                   0x00    // Bits 6:4 (0x0'000'0000) Mode0: Idle (Measurements are disabled).    
#define AIR_MODE1                   0x10    // Bits 6:4 (0x0'001'0000) Mode1: Constant power, IAQ every 1s.    
#define AIR_MODE2                   0x20    // Bits 6:4 (0x0'010'0000) Mode2: Pulse power, IAQ every 10s.    
#define AIR_MODE3                   0x30    // Bits 6:4 (0x0'011'0000) Mode3: Pulse power, IAQ every 60s.    
#define AIR_MODE4                   0x40    // Bits 6:4 (0x0'100'0000) Mode4: Constant power, RAW mesure 250ms.    
#define AIR_INTERRUPT               0x08    // Bit 4: 0('1') disable(enable) physucal interrupt generation. Cleared when read AIR_ALG_RESULT_DATA
#define AIR_THRESH                  0x04    // Bit 2: 0('1') disable(enable) thresold in interrupt generation.
 
// 0xE0(5:HEATER_SUPPLY, 4:HEATER_FAULT, 3:MAX_RESISTENCE, 2:MEASMOED_INVALID, 1:READ_REG_INVALID, 0:MSG_INVALID) register- Measurement and condictions.
#define AIR_HEATER_SUPPLY           0x20    // Bits 5. The Heater voltage is not being applied correctly. 
#define AIR_HEATER_FAULT            0x10    // Bits 4. The Heater current in the CCS811 is not in range. 
#define AIR_MAX_RESISTENCE          0x08    // Bits 3. The sensor resistance measurement has exceeded the maximum range. 
#define AIR_MEASMODE_INVALID        0x04    // Bits 2. Recibed an unsupported Mode.
#define AIR_READ_REG_INVALID        0x02    // Bits 1. Recibed an invalid I2C Read. 
#define AIR_MSG_INVALID             0x01    // Bits 0. Recibed an invalid I2C Write. 

// Perform soft reset
#define AIR_SW_RESET_KEY            0x8A72E511    // Soft reset by writing this key in AIR_SW_RESET register.

// 0x20 Hardware ID
#define AIR_HW_ID_VALUE             0x81    // Hardware ID value in AIR_HW_ID register.

// 0x21 Hardware Version
#define AIR_HW_VERSION_VALUE        0x11    // Hardware Version value in AIR_HW_VERSION register.

// Air State ///////////////////////////////////////////////////////////////////
#define HAL_AIR_MEAS_STATE_0         0    // Start air. meas.
#define HAL_AIR_MEAS_STATE_1         1    // Read air. data.
  
/* ------------------------------------------------------------------------------------------------
 *                                           Typedefs
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                          Functions
 * ------------------------------------------------------------------------------------------------
 */
extern void HalAirInit(void);
extern bool HalAirTest(void);

bool HalAirGetID(uint8 *pReg);
bool HalAirGetData(uint8 *pReg);
bool HalAirGetReg(uint8 reg, uint8 *pReg);
bool HalAirPutReg(uint8 reg, uint8 value);

void HalAirSetPM(uint8 range);
//void HalAirSetPM(uint8 mode, uint8 sleep_time);

/**************************************************************************************************
*/

#ifdef __cplusplus
};
#endif

#endif

/**************************************************************************************************
*/

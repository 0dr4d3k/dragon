/**************************************************************************************************
  Filename:       hal_acc.h
  Revised:        $Date: 2016-03-23 11:45:31$
  Revision:       $Revision: 0 $

  Description:    This file contains the declaration to the Accelerometer HAL BMA250 
                  abstraction layer.
**************************************************************************************************/

#ifndef HAL_ACC_H
#define HAL_ACC_H

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

/* ------------------------------------------------------------------------------------------------
*                                        BMA250 Constants
* ------------------------------------------------------------------------------------------------
*/

#define HAL_BMA250_I2C_ADDRESS      0x18    // I2C Address 0x00011000    

// BMA250 addressing space
#define ACC_CHIPID                  0x00    // Always 0x03
#define ACC_X_LSB                   0x02    // ACC_X_LSB[7:6] = 2 LSb of X acceleration data
#define ACC_X_MSB                   0x03    // ACC_X_MSB[7:0] = 8 MSb of X data
#define ACC_Y_LSB                   0x04
#define ACC_Y_MSB                   0x05
#define ACC_Z_LSB                   0x06
#define ACC_Z_MSB                   0x07
#define ACC_TEMP                    0x08    // Temperature data
#define ACC_INT_STATUS              0x09
#define ACC_DATA_INT_STATUS         0x0A
#define ACC_ORIENTATION             0x0C
#define ACC_RANGE                   0x0F    // 2/4/8/16 G range
#define ACC_BW                      0x10    // Filtered bandwidth
#define ACC_PM                      0x11    // Susp[7], Low_power[6], sleep_dur[4:1]
#define ACC_CONF_FILT_SHADOW        0x13
#define ACC_SOFTRESET               0x14
#define ACC_INT_ENABLE0             0x16
#define ACC_INT_ENABLE1             0x17
#define ACC_INT_MAPPING0            0x19    // Select '1' ('0'), maps(unmaps) interrupts in INT1 pin
#define ACC_INT_MAPPING1            0x1A    // Select '1' ('0'), maps(unmaps) interrupts in INT1/INT2 pin
#define ACC_INT_MAPPING2            0x1B    // Select '1' ('0'), maps(unmaps) interrupts in INT2 pin
#define ACC_INT_SOURCE              0x1E    // Select '0' ('1'), filtered (unfiltered) data
#define ACC_INT_PIN_BEHAVIOR        0x20    // Electrical INT1/INT2 pins behaviour
#define ACC_INT_TIME_BEHAVIOR       0x21    // Interrupt Time Behaviour latch_int[3:0]

// 0x0F(3:0) register - Range selection definitions 
#define ACC_RANGE_2G                0x03    //  3.91 mg/LSB (Default 0x03)
#define ACC_RANGE_4G                0x05    //  7.81 mg/LSB
#define ACC_RANGE_8G                0x08    // 15.62 mg/LSB
#define ACC_RANGE_16G               0x0C    // 31.25 mg/LSB

// 0x10(4:0) register - Filtered bandwidth selection (delta_t = time between successive acc samples)
#define ACC_BW_7_81HZ               0x08    // 7.81Hz bandwidth (delta_t = 64 ms)
#define ACC_BW_15_63HZ              0x09    // delta_t = 32   ms
#define ACC_BW_31_25HZ              0x0A    // delta_t = 16   ms
#define ACC_BW_62_5HZ               0x0B    // delta_t =  8   ms
#define ACC_BW_125HZ                0x0C    // delta_t =  4   ms
#define ACC_BW_250HZ                0x0D    // delta_t =  2   ms
#define ACC_BW_500HZ                0x0E    // delta_t =  1   ms
#define ACC_BW_1000HZ               0x0F    // delta_t =  0.5 ms (Default 0x1F)

  // 0x11(7 suspend, 6 low_power, 4:1 sleep_duration, 0 reserved) register - Power Mode
#define ACC_PM_ON                   0x00    // Power mode ON, bit 7 = 0
#define ACC_PM_SUSP                 0x80    // Power mode SUSPENDED, bit 7 = 1
#define ACC_PM_LP                   0x40    // Low power mode SLEEP - Sleep times allowed from:0,5ms to 1s
#define ACC_PM_SLEEP_10MS           0x14    // Power consumption: 16,4uA
#define ACC_PM_SLEEP_25MS           0x16    // Power consumption:  7,4uA
#define ACC_PM_SLEEP_50MS           0x18    // Power consumption:  4,0uA

// 0x16-0x17 registers - Interrupt enable bitmasks (for use with registers ACC_INT_ENABLEx [x=0,1] )
#define ACC_INT_FLAT_EN             0x80    // Bit in register 0x16
#define ACC_INT_ORIENT_EN           0x40    //          "
#define ACC_INT_S_TAP_EN            0x20    //          "
#define ACC_INT_D_TAP_EN            0x10    //          "
#define ACC_INT_SLOPE_Z_EN          0x04    //          "
#define ACC_INT_SLOPE_Y_EN          0x02    //          "
#define ACC_INT_SLOPE_X_EN          0x01    //          "
#define ACC_INT_DATA_EN             0x10    // Bit in register 0x17
#define ACC_INT_LOW_EN              0x08    //          "
#define ACC_INT_HIGH_Z_EN           0x04    //          "
#define ACC_INT_HIGH_Y_EN           0x02    //          "
#define ACC_INT_HIGH_X_EN           0x01    //          "

// 0x19-0x1A-0x1B registers - Interrupt mapping bitmasks (for use with registers ACC_INT_MAPPINGx [x=0,1,2] )
#define ACC_INT_MAP_FLAT            0x80    // For pin INT1 (INT2), bit in register 0x19 (0x1B)
#define ACC_INT_MAP_ORIENT          0x40    //                   "
#define ACC_INT_MAP_S_TAP           0x20    //                   "
#define ACC_INT_MAP_D_TAP           0x10    //                   "
#define ACC_INT_MAP_SLOPE           0x04    //                   "
#define ACC_INT_MAP_HIGH            0x02    //                   "
#define ACC_INT_MAP_LOW             0x01    //                   "
#define ACC_INT1_MAP_DATA           0x01    // New data IRQ to pin INT1, bit in register 0x1A
#define ACC_INT2_MAP_DATA           0x80    // New data IRQ to pin INT2, bit in register 0x1A

// 0x1E(5 int_scr_data: 4 int_scr_tap: 2 inst_scr_slope: 1 int_scr_high: 0 int_scr_low) register 
// Interrupt source bitmasks (for use with register ACC_INT_SOURCE) Default 0x00 all filtered
#define ACC_INT_SRC_DATA_UNFILT     0x20    
#define ACC_INT_SRC_TAP_UNFILT      0x10
#define ACC_INT_SRC_SLOPE_UNFILT    0x04
#define ACC_INT_SRC_HIGH_UNFILT     0x02
#define ACC_INT_SRC_LOW_UNFILT      0x01

  // 0x20(3:0) register - Interrupt pin behavior bitmasks (for use with register (Open drive/push-pull and active level 0/1)
#define ACC_INT2_OD                 0x08    // INT2 pin as open_drive(push-pull), '0'('1').
#define ACC_INT2_LVL                0x04    // INT2 pin active by low(high), '0'('1'). Default='1'.
#define ACC_INT1_OD                 0x02    // INT1 pin as open_drive(push-pull), '0'('1').
#define ACC_INT1_LVL                0x01    // INT1 pin active by low(high), '0'('1'). Default='1'.

// 0x21(3:0) register - Interrupt time behavior bitmasks (From 50us to 8s)
#define ACC_INT_NON_LATCHED         0x00    // Non Latched Interrupt (Default 0x00)
#define ACC_INT_TEMP_500MS          0x02    // Temporary Latched t=500ms
#define ACC_INT_TEMP_1S             0x03    // Temporary Latched t=1s
#define ACC_INT_TEMP_2S             0x04    // Temporary Latched t=2s
#define ACC_INT_TEMP_4S             0x05    // Temporary Latched t=4s
#define ACC_INT_TEMP_8S             0x06    // Temporary Latched t=8s
#define ACC_INT_LATCHED             0x0F    // Latched Interrupt
  
// Perform soft reset
#define ACC_SOFTRESET_EN            0xB6    // Soft reset by writing 0xB6 to softreset register

// Accelerometer Range
#define HAL_ACC_RANGE_16G      4
#define HAL_ACC_RANGE_8G       3
#define HAL_ACC_RANGE_4G       2
#define HAL_ACC_RANGE_2G       1

/* ------------------------------------------------------------------------------------------------
 *                                           Typedefs
 * ------------------------------------------------------------------------------------------------
 */


/* ------------------------------------------------------------------------------------------------
 *                                          Functions
 * ------------------------------------------------------------------------------------------------
 */
extern void HalAccInit(void);
extern bool HalAccTest(void);

bool HalAccGetXYZ(uint8 *pReg);
bool HalAccGetReadX(uint8 *pReg);
bool HalAccGetAll(uint8 *pReg);
bool HalAccGetID(uint8 *pReg);
bool HalAccGetTemp(uint8 *pReg);
bool HalAccGetOrientation(uint8 *pReg);

void HalAccSetRange(uint8 range);
void HalAccSetPM(uint8 mode, uint8 sleep_time);

/**************************************************************************************************
*/

#ifdef __cplusplus
};
#endif

#endif

/**************************************************************************************************
*/

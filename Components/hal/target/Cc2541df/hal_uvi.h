/*******************************************************************************
  Filename:       hal_uvi.h
  Revised:        $Date: 2016-08-03 11:45:31$
  Revision:       $Revision: 0 $

  Description:    This file contains the declaration of SILICON LABs Si1133
                  Sun UV index readiation sensor abstraction layer.
*******************************************************************************/

#ifndef HAL_UVI_H
#define HAL_UVI_H

#ifdef __cplusplus
extern "C" {
#endif

/* -----------------------------------------------------------------------------
 *                                  Includes
 * -----------------------------------------------------------------------------
 */

#include "comdef.h"

/* -----------------------------------------------------------------------------
 *                             SI1133 Constants
 * -----------------------------------------------------------------------------
 */
  
// Activate polinomical calculations
#define CALCULATE_UV
#define CALCULATE_LUX

// I2C address options
// datasheet indicate 0x55 and 0x52 as address
//#define SI1133_ADDR             0x29
#define SI1133_ADDR             0x55

// I2C Registers
#define SI115x_REG_PART_ID      0x00
#define SI115x_REG_REV_ID       0x01
#define SI115x_REG_MFR_ID       0x02
#define SI115x_REG_INFO0        0x03
#define SI115x_REG_INFO1        0x04
#define SI115x_REG_HOSTIN3      0x07
#define SI115x_REG_HOSTIN2      0x08
#define SI115x_REG_HOSTIN1      0x09
#define SI115x_REG_HOSTIN0      0x0A
#define SI115x_REG_COMMAND      0x0B
#define SI115x_REG_IRQ_ENABLE   0x0F
#define SI115x_REG_RESPONSE1    0x10
#define SI115x_REG_RESPONSE0    0x11
#define SI115x_REG_IRQ_STATUS   0x12
#define SI115x_REG_HOSTOUT0     0x13
#define SI115x_REG_HOSTOUT1     0x14
#define SI115x_REG_HOSTOUT2     0x15
#define SI115x_REG_HOSTOUT3     0x16
#define SI115x_REG_HOSTOUT4     0x17
#define SI115x_REG_HOSTOUT5     0x18
#define SI115x_REG_HOSTOUT6     0x19
#define SI115x_REG_HOSTOUT7     0x1A
#define SI115x_REG_HOSTOUT8     0x1B
#define SI115x_REG_HOSTOUT9     0x1C
#define SI115x_REG_HOSTOUT10    0x1D
#define SI115x_REG_HOSTOUT11    0x1E
#define SI115x_REG_HOSTOUT12    0x1F
#define SI115x_REG_HOSTOUT13    0x20
#define SI115x_REG_HOSTOUT14    0x21
#define SI115x_REG_HOSTOUT15    0x22
#define SI115x_REG_HOSTOUT16    0x23
#define SI115x_REG_HOSTOUT17    0x24
#define SI115x_REG_HOSTOUT18    0x25
#define SI115x_REG_HOSTOUT19    0x26
#define SI115x_REG_HOSTOUT20    0x27
#define SI115x_REG_HOSTOUT21    0x28
#define SI115x_REG_HOSTOUT22    0x29
#define SI115x_REG_HOSTOUT23    0x2A
#define SI115x_REG_HOSTOUT24    0x2B
#define SI115x_REG_HOSTOUT25    0x2C
#define SI115x_REG_OTP_CONTROL  0x2F
#define SI115x_REG_CHIP_STAT    0x30

// I2C Parameters
//#define PARAM_I2C_ADDR          0x00 //echo address
#define PARAM_I2C_ADDR          0x29
#define PARAM_CH_LIST           0x01
#define PARAM_ADCCONFIG0        0x02
#define PARAM_ADCSENS0          0x03
#define PARAM_ADCPOST0          0x04
#define PARAM_MEASCONFIG0       0x05
#define PARAM_ADCCONFIG1        0x06
#define PARAM_ADCSENS1          0x07
#define PARAM_ADCPOST1          0x08
#define PARAM_MEASCONFIG1       0x09
#define PARAM_ADCCONFIG2        0x0A
#define PARAM_ADCSENS2          0x0B
#define PARAM_ADCPOST2          0x0C
#define PARAM_MEASCONFIG2       0x0D
#define PARAM_ADCCONFIG3        0x0E
#define PARAM_ADCSENS3          0x0F
#define PARAM_ADCPOST3          0x10
#define PARAM_MEASCONFIG3       0x11
#define PARAM_ADCCONFIG4        0x12
#define PARAM_ADCSENS4          0x13
#define PARAM_ADCPOST4          0x14
#define PARAM_MEASCONFIG4       0x15
#define PARAM_ADCCONFIG5        0x16
#define PARAM_ADCSENS5          0x17
#define PARAM_ADCPOST5          0x18
#define PARAM_MEASCONFIG5       0x19
#define PARAM_MEASRATE_H        0x1A
#define PARAM_MEASRATE_L        0x1B
#define PARAM_MEASCOUNT0        0x1C
#define PARAM_MEASCOUNT1        0x1D
#define PARAM_MEASCOUNT2        0x1E
#define PARAM_LED1_A            0x1F
#define PARAM_LED1_B            0x20
#define PARAM_LED2_A            0x21
#define PARAM_LED2_B            0x22
#define PARAM_LED3_A            0x23
#define PARAM_LED3_B            0x24
#define PARAM_THRESHOLD0_H      0x25
#define PARAM_THRESHOLD0_L      0x26
#define PARAM_THRESHOLD1_H      0x27
#define PARAM_THRESHOLD1_L      0x28
#define PARAM_THRESHOLD2_H      0x29
#define PARAM_THRESHOLD2_L      0x2A
#define PARAM_BURST             0x2B

// Commands    
#define CMD_NOP                 0x00
#define CMD_RESET               0x01
#define CMD_NEW_ADDR            0x02
#define CMD_FORCE_CH            0x11
#define CMD_PAUSE_CH            0x12
#define CMD_AUTO_CH             0x13
#define CMD_PARAM_SET           0x80
#define CMD_PARAM_QUERY         0x40

// Register and Parameter Bit Definition
#define RSP0_CHIPSTAT_MASK      0xe0
#define RSP0_COUNTER_MASK       0x1f
#define RSP0_SLEEP              0x20    
    
// UVi State ///////////////////////////////////////////////////////////////////
#define HAL_UVI_MEAS_STATE_0         0    // Start uvi. meas.
#define HAL_UVI_MEAS_STATE_1         1    // Read uvi. data.

/* -----------------------------------------------------------------------------
 *                                 Typedefs
 * -----------------------------------------------------------------------------
 */
#define int32_t  int32 
#define int16_t  int16 
#define int8_t   int8 
#define uint32_t uint32 
#define uint16_t uint16 
#define uint8_t  uint8 

#ifndef HANDLE
typedef void *HANDLE;
#endif

typedef struct           
{                        
    uint8_t     irq_status;
#if defined CALCULATE_UV
    int32_t     ch0;
#endif
//#if defined CALCULATE_LUX
    int32_t     ch1;
    int32_t     ch2;
    int32_t     ch3;
//#endif
} SI115X_SAMPLES;   

/* -----------------------------------------------------------------------------
 *                                 Functions
 * -----------------------------------------------------------------------------
 */
// CONFIGURATION AND TEST FUNCTIONS   
void HalUViInit(void);
bool HalUViTest(void);
void HalUViSetMode(uint8 cfg_mode);

// MEASURE FUNCTIONS   
extern bool HalUViExecMeasurementStep(uint8 state);
extern bool HalUViReadMeasurement(uint8 *pBuf);
extern bool HalUVluxReadMeasurement(uint8 *pBuf);

// FUNCTIONS API
int16_t  Si115xReset(     HANDLE  si115x_handle);

int16_t  Si115xNop(       HANDLE  si115x_handle);

int16_t  Si115xForce(     HANDLE  si115x_handle);

int16_t  Si115xPause(     HANDLE  si115x_handle);

int16_t  Si115xStart(     HANDLE  si115x_handle);

int16_t  Si115xParamSet(  HANDLE  si115x_handle, 
                          uint8_t address, 
                          uint8_t value);

int16_t  Si115xParamRead( HANDLE  si115x_handle, 
                          uint8_t address);

uint8_t  SendCmd(         HANDLE  si115x_handle, 
                          uint8_t cmd );

uint8_t  QueryParam(      HANDLE  si115x_handle, 
                          uint8_t param_addr );

void     SetParam(        HANDLE si115x_handle, 
                          uint8_t param_addr, 
                          uint8_t param_value);

#endif

  /******************************************************************************
  Filename:       hal_sensor.h
  Revised:        $Date: 2013-03-26 07:47:25 -0700 (Tue, 26 Mar 2013) $
  Revision:       $Revision: 33597 $

  Description:    Interface to sensor driver shared code.
 ******************************************************************************/

#ifndef HAL_SENSOR_H
#define HAL_SENSOR_H

#ifdef __cplusplus
extern "C"
{
#endif

/******************************************************************************
 * INCLUDES
 ******************************************************************************/
#include "hal_types.h"

/******************************************************************************
 * CONSTANTS and MACROS
 ******************************************************************************/

/* Sensor bit values for self-test and DCDC control */
//oJo, esto no es utilizado en dragonfly  
#define ST_IRTEMP                             0x01
#define ST_HUMID                              0x02
#define ST_MAGN                               0x04
#define ST_ACC                                0x08
#define ST_PRESS                              0x10
#define ST_GYRO                               0x20

#define HIGH_SUPPLY_SENSOR_MAP                ( ST_IRTEMP | ST_HUMID | ST_GYRO)

/* Self test assertion; return FALSE (failed) if condition is not met */
#define ST_ASSERT(cond) st( if (!(cond)) return FALSE; )

/******************************************************************************
 * FUNCTIONS
 ******************************************************************************/
bool   HalSensorReadReg(uint8 reg, uint8 *pBuf, uint8 nBytes);
bool   HalSensorWriteReg(uint8 reg, uint8 *pBuf, uint8 nBytes);
uint16 HalSensorTest(void);

//New functions for bit handled
//uint8 I2CwriteBits(uint8 reg, uint8 data, uint8 pos, uint8 nbits);
//uint8 I2CreadBit(uint8 regAddr, uint8 *data, uint8 pos);
uint8 HalSensorWriteBits(uint8 reg, uint8 data, uint8 pos, uint8 nbits);
uint8 HalSensorReadBit(uint8 regAddr, uint8 *data, uint8 pos);
uint8 HalSensorReadBits(uint8 devAddr, uint8 regAddr, 
                        uint8 *data, uint8 pos, uint8 length);

/******************************************************************************
 ******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* HAL_SENSOR_H */

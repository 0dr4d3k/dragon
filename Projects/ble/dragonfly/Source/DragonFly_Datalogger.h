/**************************************************************************************************
  Filename:       DragonFly.h
  Revised:        $Date: 2013-03-25 07:58:08 -0700 (Mon, 25 Mar 2013) $
  Revision:       $Revision: 33575 $

  Description:    This file contains the Dragonfly sample application
                  definitions and prototypes.

  Copyright 2012-2013 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

#ifndef DRAGONFLY_H
#define DRAGONFLY_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

// System Events
#define DF_SYS_STAR_DEVICE_EVT                        0x0001
#define DF_SYS_RESET_EVT                              0x0002
#define DF_SYS_PERIODIC_EVT                           0x0040
#if defined PLUS_BROADCASTER
  #define DF_SYS_ADV_IN_CONNECTION_EVT                0x0080
#endif
  
// HAL Events
#define DF_HAL_ACCELEROMETER_EVT                      0x0010
#define DF_HAL_INTEMP_EVT                             0x0020
#define DF_HAL_BATTERY_EVT                            0x0100
  
// App Events
#define DF_APP_ALARM_EVT                              0x0004
#define DF_APP_DATALOGGER_EVT                         0x0008

// Sensor Events
#if defined FEATURE_DIGITAL 
  #define DF_SEN_DIGITAL_EVT                          0x0200
#endif

#if defined FEATURE_ANALOG 
  #define DF_SEN_ANALOG_EVT                           0x0400
#endif
  
#if defined FEATURE_WEATHER 
  #define DF_SEN_WEATHER_EVT                          0x0800
#endif
  
#if defined FEATURE_LUXOMETER 
  #define DF_SEN_LUXOMETER_EVT                        0x1000 /* oJo, overlayed */
#endif

#if defined FEATURE_MAGNETOMETER 
  #define DF_SEN_MAGNETOMETER_EVT                     0x1000 /* oJo, overlayed */
#endif

#if defined FEATURE_AIR 
  #define DF_SEN_AIR_EVT                              0x2000
#endif

#if defined FEATURE_UVI 
  #define DF_SEN_UVI_EVT                              0x4000
#endif
  
// Dragonfly Task Events
 
// Hardware Services

//#define ST_IRTEMPERATURE_READ_EVT                        0x0002
//#define ST_HUMIDITY_SENSOR_EVT                           0x0008
//#define ST_MAGNETOMETER_SENSOR_EVT                       0x0010
//#define ST_BAROMETER_SENSOR_EVT                          0x0020
//#define ST_GYROSCOPE_SENSOR_EVT                          0x0040

  
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS
 */

/*
 * Task Initialization for the BLE Application
 */
extern void DragonFly_Init( uint8 task_id );

/*
 * Task Event Processor for the BLE Application
 */
extern uint16 DragonFly_ProcessEvent( uint8 task_id, uint16 events );

/*
 * Power on self test
 */
extern uint16 DragonFly_Test(void);

// oJo solo para testerar SERIAL_INTERFACE, eliminar
uint8 Application_StartAdvertise(uint16 duration, uint16 interval);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* DRAGONFLY_H */

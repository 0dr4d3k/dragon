/**************************************************************************************************
  Filename:       magnetometerservice.h
  Revised:        $Date: 2017-04-06$
  Revision:       $Revision: 00001 $

  Description:    Magnetometer service definitions and prototypes
**************************************************************************************************/

#ifndef MAGNETOMETERSERVICE_H
#define MAGNETOMETERSERVICE_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "df_util.h"
  
/*********************************************************************
 * CONSTANTS
 */

// Service UUID
#define MAGNETOMETER_SERV_UUID              0xBB60  // F000BB60-0451-4000-B000-00000000-0000
#define MAGNETOMETER_DATA_UUID              0xBB61
#define MAGNETOMETER_CONF_UUID              0xBB62
#define MAGNETOMETER_PERI_UUID              0xBB63

// Sensor Profile Services bit fields
// Cidado!!, revisar en todos los servicios
#define MAGNETOMETER_SERVICE                0x00000010

// Length of sensor data in bytes
#define MAGNETOMETER_DATA_LEN               8

// Length of period data in bytes
#define MAGNETOMETER_PERI_LEN               4
  
/*********************************************************************
 * TYPEDEFS
 */


/*********************************************************************
 * MACROS
 */


/*********************************************************************
 * API FUNCTIONS
 */


/*
 * Magnetometer_AddService- Initializes the Sensor GATT Profile service by registering
 *          GATT attributes with the GATT server.
 */
extern bStatus_t Magnetometer_AddService( void );

/*
 * Magnetometer_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t Magnetometer_RegisterAppCBs( sensorCBs_t *appCallbacks );

/*
 * Magnetometer_SetParameter - Set a Sensor GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t Magnetometer_SetParameter( uint8 param, uint8 len, void *value );

/*
 * Magnetometer_GetParameter - Get a Sensor GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t Magnetometer_GetParameter( uint8 param, void *value );


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* MAGNETOMETERSERVICE_H */


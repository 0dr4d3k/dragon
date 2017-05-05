/**************************************************************************************************
  Filename:       eventservice.h
  Description:    Event service definitions and prototypes
**************************************************************************************************/

#ifndef EVENTSERVICE_H
#define EVENTSERVICE_H

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
#define EVENT_SERV_UUID              0xAA00  // F000AA00-0451-4000-B000-00000000-0000
#define EVENT_DATA_UUID              0xAA01

//oJo, cambiar esto
// Key Values
#define SK_KEY_INT1                   0x01
#define SK_KEY_INT2                   0x02
#define SK_KEY_AUX                    0x04
#define SK_KEY_QI                     0x08

// Sensor Profile Services bit fields
// oJo, revisar en todos los servicios
#define EVENT_SERVICE                0x08000000

// Length of sensor data in bytes
#define EVENT_DATA_LEN               1

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
 * Event_AddService- Initializes the Sensor GATT Profile service by registering
 *          GATT attributes with the GATT server.
 */
extern bStatus_t Event_AddService( void );

/*
 * Event_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t Event_RegisterAppCBs( sensorCBs_t *appCallbacks );

/*
 * Event_SetParameter - Set a Sensor GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t Event_SetParameter( uint8 param, uint8 len, void *value );

/*
 * Event_GetParameter - Get a Sensor GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t Event_GetParameter( uint8 param, void *value );


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* EVENTSERVICE_H */


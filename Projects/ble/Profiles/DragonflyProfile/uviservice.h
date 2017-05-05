/**************************************************************************************************
  Filename:       uviservice.c
  Revised:        $Date: 2016-08-02$
  Revision:       $Revision: 0     $

  Description:    UVi service definitions and prototypes
**************************************************************************************************/

#ifndef UVISERVICE_H
#define UVISERVICE_H

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
#define UVI_SERV_UUID              0xBB50  // F000BB50-0451-4000-B000-00000000-0000
#define UVI_DATA_UUID              0xBB51
#define UVI_CONF_UUID              0xBB52
#define UVI_PERI_UUID              0xBB53

// Sensor Profile Services bit fields
// Cidado!!, revisar en todos los servicios
#define UVI_SERVICE                0x00000020

// Length of sensor data in bytes
#define UVI_DATA_LEN               4

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
 * UVi_AddService- Initializes the Sensor GATT Profile service by registering
 *          GATT attributes with the GATT server.
 */
extern bStatus_t UVi_AddService( void );

/*
 * UVi_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t UVi_RegisterAppCBs( sensorCBs_t *appCallbacks );

/*
 * UVi_SetParameter - Set a Sensor GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t UVi_SetParameter( uint8 param, uint8 len, void *value );

/*
 * UVi_GetParameter - Get a Sensor GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t UVi_GetParameter( uint8 param, void *value );


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* UVISERVICE_H */


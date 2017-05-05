/*******************************************************************************
  Filename:       gateservice.h
  Revised:        $Date: 2017-04-17 11:11:11$
  Revision:       $Revision: 35100 $

  Description:    Gate service definitions and prototypes
*******************************************************************************/
#ifndef GATESERVICE_H
#define GATESERVICE_H

#ifdef __cplusplus
extern "C"
{
#endif

  
/*******************************************************************************
 * INCLUDES
 */
#include "df_util.h"
  
  
/*******************************************************************************
 * CONSTANTS
 */  
/* Service configuration values */
#define GATE_SERV_UUID              0xDD00  // F000DD00-0451-4000-B000-00000000-0000
#define GATE_DATA_UUID              0xDD01  // F000DD01-0451-4000-B000-00000000-0000
#define GATE_CONF_UUID              0xDD02  // F000DD02-0451-4000-B000-00000000-0000
#define GATE_PERI_UUID              0xDD03  // F000DD03-0451-4000-B000-00000000-0000

#define GATE_SERVICE                0x00000001 
#define GATE_DATA_LEN               18
#define GATE_CONF_LEN               3
#define GATE_PERI_LEN               4

#ifdef USER_DESCRIPTION
#define GATE_DATA_DESCR             "Gate. Data"
#define GATE_CONFIG_DESCR           "Gate. Conf."
#define GATE_PERIOD_DESCR           "Gate. Period"
#endif  

/* Profile Parameters */
#define GATE_DATA                   0
#define GATE_CONF                   1
#define GATE_PERI                   2

  
/*******************************************************************************
 * TYPEDEFS
 */
// Callback when a characteristic value has changed
typedef void (*gateChange_t)( uint8 paramID );
   
typedef struct
{
  gateChange_t        pfnGateChange;  // Called when characteristic value changes
} gateCBs_t;

/*******************************************************************************
 * MACROS
 */


/*******************************************************************************
 * API FUNCTIONS
 */
/*
 * Gate_AddService- Initializes the Gate GATT Profile service by registering
 *          GATT attributes with the GATT server.
 */
extern bStatus_t Gate_AddService( void );


/*
 * Gate_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t Gate_RegisterAppCBs( gateCBs_t *appCallbacks );


/*
 * Gate_SetParameter - Set a Gate GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t Gate_SetParameter( uint8 param, uint8 len, void *value );


/*
 * Gate_GetParameter - Get a Gate GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 */
extern bStatus_t Gate_GetParameter( uint8 param, void *value );


/*******************************************************************************
*******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* GATESERVICE_H */


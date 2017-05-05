/**************************************************************************************************
  Filename:       analogservice.c
  Revised:        $Date: 2013-08-23 11:45:31 -0700 (Fri, 23 Aug 2013) $
  Revision:       $Revision: 35100 $

  Description:    Analog service.


  Copyright 2012 - 2015 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED �AS IS� WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
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

/*********************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "linkdb.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"

#include "analogservice.h"
#include "df_util.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/* Service configuration values */
#define SENSOR_SERVICE_UUID     ANALOG_SERV_UUID
#define SENSOR_DATA_UUID        ANALOG_DATA_UUID
#define SENSOR_CONFIG_UUID      ANALOG_CONF_UUID
#define SENSOR_PERIOD_UUID      ANALOG_PERI_UUID

#define SENSOR_SERVICE          ANALOG_SERVICE
#define SENSOR_DATA_LEN         ANALOG_DATA_LEN
#define SENSOR_PERI_LEN         ANALOG_PERI_LEN

#ifdef USER_DESCRIPTION
#define SENSOR_DATA_DESCR       "Analog. Data"
#define SENSOR_CONFIG_DESCR     "Analog. Conf."
#define SENSOR_PERIOD_DESCR     "Analog. Period"
#endif

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Service UUID
static CONST uint8 sensorServiceUUID[TI_UUID_SIZE] =
{
  TI_UUID(SENSOR_SERVICE_UUID),
};

// Characteristic UUID: data
static CONST uint8 sensorDataUUID[TI_UUID_SIZE] =
{
  TI_UUID(SENSOR_DATA_UUID),
};

// Characteristic UUID: config
static CONST uint8 sensorCfgUUID[TI_UUID_SIZE] =
{
  TI_UUID(SENSOR_CONFIG_UUID),
};

// Characteristic UUID: period
static CONST uint8 sensorPeriodUUID[TI_UUID_SIZE] =
{
  TI_UUID(SENSOR_PERIOD_UUID),
};


/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

static sensorCBs_t *sensor_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// Profile Service attribute
static CONST gattAttrType_t sensorService = { TI_UUID_SIZE, sensorServiceUUID };

// Characteristic Value: data
static uint8 sensorData[SENSOR_DATA_LEN] = { 0, 0};

// Characteristic Properties: data
static uint8 sensorDataProps = GATT_PROP_READ | GATT_PROP_NOTIFY;

// Characteristic Configuration: data
static gattCharCfg_t *sensorDataConfig;

#ifdef USER_DESCRIPTION
// Characteristic User Description: data
static uint8 sensorDataUserDescr[] = SENSOR_DATA_DESCR;
#endif

// Characteristic Properties: configuration
static uint8 sensorCfgProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic Value: configuration
static uint8 sensorCfg = 0;

#ifdef USER_DESCRIPTION
// Characteristic User Description: configuration
static uint8 sensorCfgUserDescr[] = SENSOR_CONFIG_DESCR; 
#endif

// Characteristic Properties: period
static uint8 sensorPeriodProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic Value: period
//static uint8 sensorPeriod = SENSOR_MIN_UPDATE_PERIOD / SENSOR_PERIOD_RESOLUTION;
static uint8 sensorPeriod[SENSOR_PERI_LEN] = 
//        { (SENSOR_MIN_UPDATE_PERIOD / SENSOR_PERIOD_RESOLUTION), 0, 0, 0 };
        { 0xD0, 0x07, 0, 0 }; //0x000007D0 -> 2000 ms default period

#ifdef USER_DESCRIPTION
// Characteristic User Description: period
static uint8 sensorPeriodUserDescr[] = SENSOR_PERIOD_DESCR;
#endif

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t sensorAttrTable[] =
{
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&sensorService                   /* pValue */
  },

    // Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &sensorDataProps
    },

      // Characteristic Value "Data"
      {
        { TI_UUID_SIZE, sensorDataUUID },
        GATT_PERMIT_READ,
        0,
        sensorData
      },

      // Characteristic configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&sensorDataConfig
      },
#ifdef USER_DESCRIPTION
      // Characteristic User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        sensorDataUserDescr
      },
#endif
    // Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &sensorCfgProps
    },

      // Characteristic Value "Configuration"
      {
        { TI_UUID_SIZE, sensorCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        &sensorCfg
      },
#ifdef USER_DESCRIPTION
      // Characteristic User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        sensorCfgUserDescr
      },
#endif
     // Characteristic Declaration "Period"
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &sensorPeriodProps
    },

      // Characteristic Value "Period"
      {
        { TI_UUID_SIZE, sensorPeriodUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
//        &sensorPeriod
        sensorPeriod
      },
#ifdef USER_DESCRIPTION
      // Characteristic User Description "Period"
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        sensorPeriodUserDescr
      },
#endif
};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 sensor_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                            uint8 *pValue, uint8 *pLen, uint16 offset, 
                            uint8 maxLen, uint8 method );
static bStatus_t sensor_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset, 
                                 uint8 method );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
static CONST gattServiceCBs_t sensorCBs =
{
  sensor_ReadAttrCB,  // Read callback function pointer
  sensor_WriteAttrCB, // Write callback function pointer
  NULL                // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      Analog_AddService
 *
 * @brief   Initializes the Sensor Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 */
bStatus_t Analog_AddService( void )
{
  bStatus_t ret;

  ret = util_initCharacteristicConfig(&sensorDataConfig);
  if (ret != SUCCESS)
  {
    return ret;
  }

  // Register GATT attribute list and CBs with GATT Server App
  return GATTServApp_RegisterService( sensorAttrTable,
                                      GATT_NUM_ATTRS (sensorAttrTable),
                                      GATT_MAX_ENCRYPT_KEY_SIZE,
                                      &sensorCBs );
}


/*********************************************************************
 * @fn      Analog_RegisterAppCBs
 *
 * @brief   Registers the application callback function. Only call
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t Analog_RegisterAppCBs( sensorCBs_t *appCallbacks )
{
  if ( sensor_AppCBs == NULL)
  {
    if ( appCallbacks != NULL)
    {
      sensor_AppCBs = appCallbacks;
    }

    return ( SUCCESS );
  }

  return ( bleAlreadyInRequestedMode );
}

/*********************************************************************
 * @fn      Analog_SetParameter
 *
 * @brief   Set a parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to right
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t Analog_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;

  switch ( param )
  {
    case SENSOR_DATA:
    if ( len == SENSOR_DATA_LEN )
    {
      VOID osal_memcpy( sensorData, (uint8*)value, SENSOR_DATA_LEN );
      // See if Notification has been enabled
      GATTServApp_ProcessCharCfg( sensorDataConfig, sensorData, FALSE,
                                 sensorAttrTable, GATT_NUM_ATTRS( sensorAttrTable ),
                                 INVALID_TASK_ID, sensor_ReadAttrCB);
    }
    else
    {
      ret = bleInvalidRange;
    }
    break;

    case SENSOR_CONF:
      if ( len == sizeof ( uint8 ) )
      {
        sensorCfg = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case SENSOR_PERI:
//      if ( len == sizeof ( uint8 ) )
      if ( len == SENSOR_PERI_LEN )      
      {
//        sensorPeriod = *((uint8*)value);
        VOID osal_memcpy( sensorPeriod, value, SENSOR_PERI_LEN );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return ( ret );
}

/*********************************************************************
 * @fn      Analog_GetParameter
 *
 * @brief   Get a Sensor Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t Analog_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;

  switch ( param )
  {
    case SENSOR_DATA:
      VOID osal_memcpy( value, sensorData, SENSOR_DATA_LEN );
      break;

    case SENSOR_CONF:
      *((uint8*)value) = sensorCfg;
      break;

    case SENSOR_PERI:
//      *((uint8*)value) = sensorPeriod;
      VOID osal_memcpy( value, sensorPeriod, SENSOR_PERI_LEN );
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return ( ret );
}

/*********************************************************************
 * @fn          sensor_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 *
 * @return      Success or Failure
 */
static uint8 sensor_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen, uint8 method )
{
  uint16 uuid;
  bStatus_t status = SUCCESS;

  // If attribute permissions require authorization to read, return error
  if ( gattPermitAuthorRead( pAttr->permissions ) )
  {
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }

  // Make sure it's not a blob operation (no attributes in the profile are long)
  if ( offset > 0 )
  {
    return ( ATT_ERR_ATTR_NOT_LONG );
  }

  if (utilExtractUuid16(pAttr,&uuid) == FAILURE) {
    // Invalid handle
    *pLen = 0;
    return ATT_ERR_INVALID_HANDLE;
  }

  switch ( uuid )
  {
    // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
    // gattserverapp handles those reads
    case SENSOR_DATA_UUID:
      *pLen = SENSOR_DATA_LEN;
      VOID osal_memcpy( pValue, pAttr->pValue, SENSOR_DATA_LEN );
      break;

    case SENSOR_CONFIG_UUID:
      *pLen = 1;
      pValue[0] = *pAttr->pValue;
      break;
 
    case SENSOR_PERIOD_UUID:
//      *pLen = 1;
//      pValue[0] = *pAttr->pValue;
      *pLen = SENSOR_PERI_LEN;
      VOID osal_memcpy( pValue, pAttr->pValue, SENSOR_PERI_LEN );
      break;

    default:
      *pLen = 0;
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
    }

  return ( status );
}

/*********************************************************************
* @fn      sensor_WriteAttrCB
*
* @brief   Validate attribute data prior to a write operation
*
* @param   connHandle - connection message was received on
* @param   pAttr - pointer to attribute
* @param   pValue - pointer to data to be written
* @param   len - length of data
* @param   offset - offset of the first octet to be written
*
* @return  Success or Failure
*/
static bStatus_t sensor_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                           uint8 *pValue, uint8 len, uint16 offset, uint8 method)
{
  bStatus_t status = SUCCESS;
  uint8 notifyApp = 0xFF;
  uint16 uuid;

  // If attribute permissions require authorization to write, return error
  if ( gattPermitAuthorWrite( pAttr->permissions ) )
  {
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }

  if (utilExtractUuid16(pAttr,&uuid) == FAILURE) {
    // Invalid handle
    return ATT_ERR_INVALID_HANDLE;
  }

  switch ( uuid )
  {
    case SENSOR_DATA_UUID:
      // Should not get here
      break;

    case SENSOR_CONFIG_UUID:
      // Validate the value
      // Make sure it's not a blob oper
      if ( offset == 0 )
      {
        if ( len != 1 )
        {
          status = ATT_ERR_INVALID_VALUE_SIZE;
        }
      }
      else
      {
        status = ATT_ERR_ATTR_NOT_LONG;
      }

      // Write the value
      if ( status == SUCCESS )
      {
        uint8 *pCurValue = (uint8 *)pAttr->pValue;

        *pCurValue = pValue[0];

        if( pAttr->pValue == &sensorCfg )
        {
          notifyApp = SENSOR_CONF;
        }
      }
      break;

    case SENSOR_PERIOD_UUID:
//      // Validate the value
//      // Make sure it's not a blob oper
//      if ( offset == 0 )
//      {
//        if ( len != 1 )
//        {
//          status = ATT_ERR_INVALID_VALUE_SIZE;
//        }
//      }
//      else
//      {
//        status = ATT_ERR_ATTR_NOT_LONG;
//      }
//      // Write the value
//      if ( status == SUCCESS )
//      {
//        if (pValue[0]>=(SENSOR_MIN_UPDATE_PERIOD/SENSOR_PERIOD_RESOLUTION))
//        {
//          uint8 *pCurValue = (uint8 *)pAttr->pValue;
//          *pCurValue = pValue[0];
//
//          if( pAttr->pValue == &sensorPeriod )
//          {
//            notifyApp = SENSOR_PERI;
//          }
//        }
//        else
//        {
//           status = ATT_ERR_INVALID_VALUE;
//        }
//      }

//METER ESTA COMPROBACION SI ES NECESARIO
      // Validate the value
      // Make sure it's not a blob oper
//      if ( offset == 0 )
//      {
//        if ( len != 2 )
//        {
//          status = ATT_ERR_INVALID_VALUE_SIZE;
//        }
//      }
//      else
//      {
//        status = ATT_ERR_ATTR_NOT_LONG;
//      }
      // Write the value
      if ( status == SUCCESS )
      {
 //       if (pValue[0]>=(SENSOR_MIN_UPDATE_PERIOD/SENSOR_PERIOD_RESOLUTION))
 //       {

          uint8 *pCurValue = (uint8 *)pAttr->pValue;
          VOID osal_memcpy( pCurValue, pValue, SENSOR_PERI_LEN );

          if( pAttr->pValue == sensorPeriod )
          {
            notifyApp = SENSOR_PERI;
          }
          
//          if( pAttr->pValue == &sensorPeriod )
//          {
//            notifyApp = SENSOR_PERI;
//          }
 //       }
 //       else
 //       {
 //          status = ATT_ERR_INVALID_VALUE;
 //       }
      }


  
      break;

    case GATT_CLIENT_CHAR_CFG_UUID:
      status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                              offset, GATT_CLIENT_CFG_NOTIFY );
      break;

    default:
      // Should never get here!
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
  }

  // If a charactersitic value changed then callback function to notify application of change
  if ( (notifyApp != 0xFF ) && sensor_AppCBs && sensor_AppCBs->pfnSensorChange )
  {
    sensor_AppCBs->pfnSensorChange( notifyApp );
  }

  return ( status );
}


/*********************************************************************
*********************************************************************/
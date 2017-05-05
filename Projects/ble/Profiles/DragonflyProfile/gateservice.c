/*******************************************************************************
  Filename:       gateservice.c
  Revised:        $Date: 2017-04-17 11:11:11$
  Revision:       $Revision: 1 $

  Description:    Gate service.
*******************************************************************************/


/*******************************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "linkdb.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"

#include "gateservice.h"
#include "df_util.h"


/*******************************************************************************
 * MACROS
 */


/*******************************************************************************
 * CONSTANTS
 */

   
/*******************************************************************************
 * TYPEDEFS
 */


/*******************************************************************************
 * GLOBAL VARIABLES
 */
// Service UUID
static CONST uint8 gateServiceUUID[TI_UUID_SIZE] =
{
  TI_UUID(GATE_SERV_UUID),
};

// Characteristic UUID: data
static CONST uint8 gateDataUUID[TI_UUID_SIZE] =
{
  TI_UUID(GATE_DATA_UUID),
};

// Characteristic UUID: config
static CONST uint8 gateCfgUUID[TI_UUID_SIZE] =
{
  TI_UUID(GATE_CONF_UUID),
};

// Characteristic UUID: period
static CONST uint8 gatePeriodUUID[TI_UUID_SIZE] =
{
  TI_UUID(GATE_PERI_UUID),
};


/*******************************************************************************
 * EXTERNAL VARIABLES
 */


/*******************************************************************************
 * EXTERNAL FUNCTIONS
 */


/*******************************************************************************
 * LOCAL VARIABLES
 */
static gateCBs_t *gate_AppCBs = NULL;


/*******************************************************************************
 * Profile Attributes - variables
 */

// Profile Service attribute
static CONST gattAttrType_t gateService = {TI_UUID_SIZE, gateServiceUUID};

// Characteristic Value: data
static uint8 gateData[GATE_DATA_LEN] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

// Characteristic Properties: data
static uint8 gateDataProps = GATT_PROP_READ | GATT_PROP_WRITE | GATT_PROP_NOTIFY;

// Characteristic Configuration: data
static gattCharCfg_t *gateDataConfig;

#ifdef USER_DESCRIPTION
// Characteristic User Description: data
static uint8 gateDataUserDescr[] = GATE_DATA_DESCR;
#endif

// Characteristic Properties: configuration
static uint8 gateCfgProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic Value: configuration
static uint8 gateCfg[GATE_CONF_LEN] = {0};

#ifdef USER_DESCRIPTION
// Characteristic User Description: configuration
static uint8 gateCfgUserDescr[] = GATE_CONFIG_DESCR; 
#endif

// Characteristic Properties: period
static uint8 gatePeriodProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic Value: period
//static uint8 gatePeriod = GATE_MIN_UPDATE_PERIOD / GATE_PERIOD_RESOLUTION;
static uint8 gatePeriod[GATE_PERI_LEN] = 
//        { (GATE_MIN_UPDATE_PERIOD / GATE_PERIOD_RESOLUTION), 0, 0, 0 };
        { 0xD0, 0x07, 0, 0 }; //0x000007D0 -> 2000 ms default period

#ifdef USER_DESCRIPTION
// Characteristic User Description: period
static uint8 gatePeriodUserDescr[] = GATE_PERIOD_DESCR;
#endif


/*******************************************************************************
 * Profile Attributes - Table
 */
static gattAttribute_t gateAttrTable[] =
{
  {
    {ATT_BT_UUID_SIZE, primaryServiceUUID}, /* type */
    GATT_PERMIT_READ,                       /* permissions */
    0,                                      /* handle */
    (uint8 *)&gateService                   /* pValue */
  },

    // Characteristic Declaration
    {
      {ATT_BT_UUID_SIZE, characterUUID},
      GATT_PERMIT_READ,
      0,
      &gateDataProps
    },

      // Characteristic Value "Data"
      {
        {TI_UUID_SIZE, gateDataUUID},
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        gateData
      },

      // Characteristic configuration
      {
        {ATT_BT_UUID_SIZE, clientCharCfgUUID},
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&gateDataConfig
      },
#ifdef USER_DESCRIPTION
      // Characteristic User Description
      {
        {ATT_BT_UUID_SIZE, charUserDescUUID},
        GATT_PERMIT_READ,
        0,
        gateDataUserDescr
      },
#endif
    // Characteristic Declaration
    {
      {ATT_BT_UUID_SIZE, characterUUID},
      GATT_PERMIT_READ,
      0,
      &gateCfgProps
    },

      // Characteristic Value "Configuration"
      {
        {TI_UUID_SIZE, gateCfgUUID},
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        gateCfg
      },
#ifdef USER_DESCRIPTION
      // Characteristic User Description
      {
        {ATT_BT_UUID_SIZE, charUserDescUUID},
        GATT_PERMIT_READ,
        0,
        gateCfgUserDescr
      },
#endif
     // Characteristic Declaration "Period"
    {
      {ATT_BT_UUID_SIZE, characterUUID},
      GATT_PERMIT_READ,
      0,
      &gatePeriodProps
    },

      // Characteristic Value "Period"
      {
        {TI_UUID_SIZE, gatePeriodUUID},
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        gatePeriod
      },
#ifdef USER_DESCRIPTION
      // Characteristic User Description "Period"
      {
        {ATT_BT_UUID_SIZE, charUserDescUUID},
        GATT_PERMIT_READ,
        0,
        gatePeriodUserDescr
      },
#endif
};


/*******************************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 gate_ReadAttrCB(uint16 connHandle, gattAttribute_t *pAttr,
                            uint8 *pValue, uint8 *pLen, uint16 offset, 
                            uint8 maxLen, uint8 method);

static bStatus_t gate_WriteAttrCB(uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset, 
                                 uint8 method);


/*******************************************************************************
 * PROFILE CALLBACKS
 */
// Simple Profile Service Callbacks
static CONST gattServiceCBs_t gateCBs =
{
  gate_ReadAttrCB,  // Read callback function pointer
  gate_WriteAttrCB, // Write callback function pointer
  NULL              // Authorization callback function pointer
};


/*******************************************************************************
 * PUBLIC FUNCTIONS
 */
/*******************************************************************************
 * @fn      Gate_AddService
 *
 * @brief   Initializes the Gate Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @return  Success or Failure
 */
bStatus_t Gate_AddService(void)
{
  bStatus_t ret;

  ret = util_initCharacteristicConfig(&gateDataConfig);
  if (ret != SUCCESS)
  {
    return ret;
  }

  // Register GATT attribute list and CBs with GATT Server App
  return GATTServApp_RegisterService(gateAttrTable,
                                     GATT_NUM_ATTRS (gateAttrTable),
                                     GATT_MAX_ENCRYPT_KEY_SIZE,
                                     &gateCBs);
}


/*******************************************************************************
 * @fn      Gate_RegisterAppCBs
 *
 * @brief   Registers the application callback function. Only call
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t Gate_RegisterAppCBs(gateCBs_t *appCallbacks)
{
  if(gate_AppCBs == NULL)
  {
    if(appCallbacks != NULL)
    {
      gate_AppCBs = appCallbacks;
    }

    return (SUCCESS);
  }

  return (bleAlreadyInRequestedMode);
}


/*******************************************************************************
 * @fn      Gate_SetParameter
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
bStatus_t Gate_SetParameter(uint8 param, uint8 len, void *value)
{
  bStatus_t ret = SUCCESS;

  switch ( param )
  {
    case GATE_DATA:
    if(len == GATE_DATA_LEN)
    {
      VOID osal_memcpy(gateData, (uint8*)value, GATE_DATA_LEN);
      // See if Notification has been enabled
      GATTServApp_ProcessCharCfg(gateDataConfig, gateData, FALSE,
                                 gateAttrTable, GATT_NUM_ATTRS(gateAttrTable),
                                 INVALID_TASK_ID, gate_ReadAttrCB);
    }
    else
    {
      ret = bleInvalidRange;
    }
    break;

    case GATE_CONF:
      if(len == GATE_CONF_LEN)
      {
        VOID osal_memcpy(gateCfg, value, GATE_CONF_LEN);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case GATE_PERI:
      if ( len == GATE_PERI_LEN )
      {
        VOID osal_memcpy(gatePeriod, value, GATE_PERI_LEN);
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

  return (ret);
}


/*******************************************************************************
 * @fn      Gate_GetParameter
 *
 * @brief   Get a Gate Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate
 *          data type (example: data type of uint16 will be cast to
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t Gate_GetParameter(uint8 param, void *value)
{
  bStatus_t ret = SUCCESS;

  switch (param)
  {
    case GATE_DATA:
      VOID osal_memcpy(value, gateData, GATE_DATA_LEN);
      break;

    case GATE_CONF:
      VOID osal_memcpy(value, gateCfg, GATE_CONF_LEN);
      break;

    case GATE_PERI:
      VOID osal_memcpy(value, gatePeriod, GATE_PERI_LEN);
      break;

    default:
      ret = INVALIDPARAMETER;
      break;
  }

  return (ret);
}


/*******************************************************************************
 * @fn          gate_ReadAttrCB
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
static uint8 gate_ReadAttrCB(uint16 connHandle, gattAttribute_t *pAttr,
                            uint8 *pValue, uint8 *pLen, uint16 offset, 
                            uint8 maxLen, uint8 method)
{
  uint16 uuid;
  bStatus_t status = SUCCESS;

  // If attribute permissions require authorization to read, return error
  if(gattPermitAuthorRead(pAttr->permissions))
  {
    // Insufficient authorization
    return (ATT_ERR_INSUFFICIENT_AUTHOR);
  }

  // Make sure it's not a blob operation (no attributes in the profile are long)
  if(offset > 0)
  {
    return (ATT_ERR_ATTR_NOT_LONG);
  }

  if(utilExtractUuid16(pAttr,&uuid) == FAILURE) 
  {
    // Invalid handle
    *pLen = 0;
    return ATT_ERR_INVALID_HANDLE;
  }

  switch(uuid)
  {
    // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
    // gattserverapp handles those reads
    case GATE_DATA_UUID:
      *pLen = GATE_DATA_LEN;
      VOID osal_memcpy(pValue, pAttr->pValue, GATE_DATA_LEN);
      break;

    case GATE_CONF_UUID:
      *pLen = GATE_CONF_LEN;
      VOID osal_memcpy(pValue, pAttr->pValue, GATE_CONF_LEN);
      break;
 
    case GATE_PERI_UUID:
      *pLen = GATE_PERI_LEN;
      VOID osal_memcpy(pValue, pAttr->pValue, GATE_PERI_LEN);
      break;

    default:
      *pLen = 0;
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
    }

  return (status);
}


/*******************************************************************************
* @fn      gate_WriteAttrCB
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
static bStatus_t gate_WriteAttrCB(uint16 connHandle, gattAttribute_t *pAttr,
                                  uint8 *pValue, uint8 len, uint16 offset, uint8 method)
{
  bStatus_t status = SUCCESS;
  uint8 notifyApp = 0xFF;
  uint16 uuid;

  // If attribute permissions require authorization to write, return error
  if(gattPermitAuthorWrite(pAttr->permissions))
  {
    // Insufficient authorization
    return ATT_ERR_INSUFFICIENT_AUTHOR;
  }

  if(utilExtractUuid16(pAttr,&uuid) == FAILURE)
  {
    // Invalid handle
    return ATT_ERR_INVALID_HANDLE;
  }

  switch(uuid)
  {
    case GATE_DATA_UUID:
      // Validate the value
      // Make sure it's not a blob oper
      if ( offset == 0 )
      {
        if(len > GATE_DATA_LEN)
        {
          status = ATT_ERR_INVALID_VALUE_SIZE;
        }
      }
      else
      {
        status = ATT_ERR_ATTR_NOT_LONG;
      }

      // Write the value
      if(status == SUCCESS)
      {
          uint8 *pCurValue = (uint8 *)pAttr->pValue;
          VOID osal_memcpy(pCurValue, pValue, GATE_DATA_LEN);

          if(pAttr->pValue == gateData)
          {
            notifyApp = GATE_DATA;
          }
      }
      break;

    case GATE_CONF_UUID:
      // Validate the value
      // Make sure it's not a blob oper
      if(offset == 0)
      {
        if(len != GATE_CONF_LEN)
        {
          status = ATT_ERR_INVALID_VALUE_SIZE;
        }
      }
      else
      {
        status = ATT_ERR_ATTR_NOT_LONG;
      }

      // Write the value
      if(status == SUCCESS)
      {
          uint8 *pCurValue = (uint8 *)pAttr->pValue;
          VOID osal_memcpy(pCurValue, pValue, GATE_CONF_LEN);

          if(pAttr->pValue == gateCfg)
          {
            notifyApp = GATE_CONF;
          }
      }
      break;

    case GATE_PERI_UUID:
      // Write the value
      if(status == SUCCESS)
      {
          uint8 *pCurValue = (uint8 *)pAttr->pValue;
          VOID osal_memcpy(pCurValue, pValue, GATE_PERI_LEN);

          if(pAttr->pValue == gatePeriod)
          {
            notifyApp = GATE_PERI;
          }
      }
      break;

    case GATT_CLIENT_CHAR_CFG_UUID:
      status = GATTServApp_ProcessCCCWriteReq(connHandle, pAttr, pValue, len,
                                              offset, GATT_CLIENT_CFG_NOTIFY);
      break;

    default:
      // Should never get here!
      status = ATT_ERR_ATTR_NOT_FOUND;
      break;
  }

  // If a charactersitic value changed then callback function to notify application of change
  if((notifyApp != 0xFF) && gate_AppCBs && gate_AppCBs->pfnGateChange)
  {
    gate_AppCBs->pfnGateChange(notifyApp);
  }

  return (status);
}


/*******************************************************************************
*******************************************************************************/

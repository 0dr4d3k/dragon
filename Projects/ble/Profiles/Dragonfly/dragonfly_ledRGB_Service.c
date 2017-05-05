/**************************************************************************************************
  Filename:       ledRGBGATTprofile.c
  Revised:        $Date: 2015-03-24 09:19:15 -0700 (Tue, 24 Mar 2015) $
  Revision:       $Revision: 43274 $

  Description:    This file contains the ledRGB GATT profile sample GATT service 
                  profile for use with the BLE sample application.

  Copyright 2010 - 2015 Texas Instruments Incorporated. All rights reserved.

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

/*********************************************************************
 * INCLUDES
 */
#include <string.h>

#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"

#include "dragonfly_ledRGB_Service.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

#define SERVAPP_NUM_ATTR_SUPPORTED        17

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// ledRGB GATT Profile Service UUID: 0xFFF0
CONST uint8 ledRGBProfileServUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(LEDRGBPROFILE_SERV_UUID), HI_UINT16(LEDRGBPROFILE_SERV_UUID)
};

// Characteristic 1 UUID: 0xFFF1
CONST uint8 ledRGBProfilechar1UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(LEDRGBPROFILE_CHAR1_UUID), HI_UINT16(LEDRGBPROFILE_CHAR1_UUID)
};

// Characteristic 2 UUID: 0xFFF2
CONST uint8 ledRGBProfilechar2UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(LEDRGBPROFILE_CHAR2_UUID), HI_UINT16(LEDRGBPROFILE_CHAR2_UUID)
};

// Characteristic 3 UUID: 0xFFF3
CONST uint8 ledRGBProfilechar3UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(LEDRGBPROFILE_CHAR3_UUID), HI_UINT16(LEDRGBPROFILE_CHAR3_UUID)
};

// Characteristic 4 UUID: 0xFFF4
CONST uint8 ledRGBProfilechar4UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(LEDRGBPROFILE_CHAR4_UUID), HI_UINT16(LEDRGBPROFILE_CHAR4_UUID)
};

// Characteristic 5 UUID: 0xFFF5
CONST uint8 ledRGBProfilechar5UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(LEDRGBPROFILE_CHAR5_UUID), HI_UINT16(LEDRGBPROFILE_CHAR5_UUID)
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

static ledRGBProfileCBs_t *ledRGBProfile_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// ledRGB Profile Service attribute
static CONST gattAttrType_t ledRGBProfileService = { ATT_BT_UUID_SIZE, ledRGBProfileServUUID };


// ledRGB Profile Characteristic 1 Properties
static uint8 ledRGBProfileChar1Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic 1 Value
static uint8 ledRGBProfileChar1 = 0;

// ledRGB Profile Characteristic 1 User Description
static uint8 ledRGBProfileChar1UserDesp[17] = "Characteristic 1";


// ledRGB Profile Characteristic 2 Properties
static uint8 ledRGBProfileChar2Props = GATT_PROP_READ;

// Characteristic 2 Value
static uint8 ledRGBProfileChar2 = 0;

// ledRGB Profile Characteristic 2 User Description
static uint8 ledRGBProfileChar2UserDesp[17] = "Characteristic 2";


// ledRGB Profile Characteristic 3 Properties
static uint8 ledRGBProfileChar3Props = GATT_PROP_WRITE;

// Characteristic 3 Value
static uint8 ledRGBProfileChar3 = 0;

// ledRGB Profile Characteristic 3 User Description
static uint8 ledRGBProfileChar3UserDesp[17] = "Characteristic 3";


// ledRGB Profile Characteristic 4 Properties
static uint8 ledRGBProfileChar4Props = GATT_PROP_NOTIFY;

// Characteristic 4 Value
static uint8 ledRGBProfileChar4 = 0;

// ledRGB Profile Characteristic 4 Configuration Each client has its own
// instantiation of the Client Characteristic Configuration. Reads of the
// Client Characteristic Configuration only shows the configuration for
// that client and writes only affect the configuration of that client.
static gattCharCfg_t *ledRGBProfileChar4Config;
                                        
// ledRGB Profile Characteristic 4 User Description
static uint8 ledRGBProfileChar4UserDesp[17] = "Characteristic 4";


// ledRGB Profile Characteristic 5 Properties
static uint8 ledRGBProfileChar5Props = GATT_PROP_READ;

// Characteristic 5 Value
static uint8 ledRGBProfileChar5[LEDRGBPROFILE_CHAR5_LEN] = { 0, 0, 0, 0, 0 };

// ledRGB Profile Characteristic 5 User Description
static uint8 ledRGBProfileChar5UserDesp[17] = "Characteristic 5";

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t ledRGBProfileAttrTbl[SERVAPP_NUM_ATTR_SUPPORTED] = 
{
  // ledRGB Profile Service
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
//    GATT_PERMIT_AUTHEN_READ, 
    0,                                        /* handle */
    (uint8 *)&ledRGBProfileService            /* pValue */
  },

    // Characteristic 1 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &ledRGBProfileChar1Props 
    },

      // Characteristic Value 1
      { 
        { ATT_BT_UUID_SIZE, ledRGBProfilechar1UUID },
        GATT_PERMIT_READ, 
        0, 
        &ledRGBProfileChar1 
      },

      // Characteristic 1 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        ledRGBProfileChar1UserDesp 
      },      

    // Characteristic 2 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &ledRGBProfileChar2Props 
    },

      // Characteristic Value 2
      { 
        { ATT_BT_UUID_SIZE, ledRGBProfilechar2UUID },
        GATT_PERMIT_READ, 
        0, 
        &ledRGBProfileChar2 
      },

      // Characteristic 2 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        ledRGBProfileChar2UserDesp 
      },           
      
    // Characteristic 3 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &ledRGBProfileChar3Props 
    },

      // Characteristic Value 3
      { 
        { ATT_BT_UUID_SIZE, ledRGBProfilechar3UUID },
        GATT_PERMIT_WRITE, 
        0, 
        &ledRGBProfileChar3 
      },

      // Characteristic 3 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        ledRGBProfileChar3UserDesp 
      },

    // Characteristic 4 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &ledRGBProfileChar4Props 
    },

      // Characteristic Value 4
      { 
        { ATT_BT_UUID_SIZE, ledRGBProfilechar4UUID },
        0, 
        0, 
        &ledRGBProfileChar4 
      },

      // Characteristic 4 configuration
      { 
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)&ledRGBProfileChar4Config 
      },
      
      // Characteristic 4 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        ledRGBProfileChar4UserDesp 
      },
      
    // Characteristic 5 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &ledRGBProfileChar5Props 
    },

      // Characteristic Value 5
      { 
        { ATT_BT_UUID_SIZE, ledRGBProfilechar5UUID },
        GATT_PERMIT_AUTHEN_READ, 
        0, 
        ledRGBProfileChar5 
      },

      // Characteristic 5 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        ledRGBProfileChar5UserDesp 
      },
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static bStatus_t ledRGBProfile_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                                           uint8 *pValue, uint8 *pLen, uint16 offset,
                                           uint8 maxLen, uint8 method );
static bStatus_t ledRGBProfile_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                            uint8 *pValue, uint8 len, uint16 offset,
                                            uint8 method );

/*********************************************************************
 * PROFILE CALLBACKS
 */
// ledRGB Profile Service Callbacks
CONST gattServiceCBs_t ledRGBProfileCBs =
{
  ledRGBProfile_ReadAttrCB,  // Read callback function pointer
  ledRGBProfile_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      ledRGBProfile_AddService
 *
 * @brief   Initializes the ledRGB Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t ledRGBProfile_AddService( uint32 services )
{
  uint8 status;
  
  // Allocate Client Characteristic Configuration table
  ledRGBProfileChar4Config = (gattCharCfg_t *)osal_mem_alloc( sizeof(gattCharCfg_t) *
                                                              linkDBNumConns );
  if ( ledRGBProfileChar4Config == NULL )
  {     
    return ( bleMemAllocError );
  }
  
  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, ledRGBProfileChar4Config );
  
  if ( services & LEDRGBPROFILE_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( ledRGBProfileAttrTbl, 
                                          GATT_NUM_ATTRS( ledRGBProfileAttrTbl ),
                                          GATT_MAX_ENCRYPT_KEY_SIZE,
                                          &ledRGBProfileCBs );
  }
  else
  {
    status = SUCCESS;
  }
  
  return ( status );
}

/*********************************************************************
 * @fn      ledRGBProfile_RegisterAppCBs
 *
 * @brief   Registers the application callback function. Only call 
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t ledRGBProfile_RegisterAppCBs( ledRGBProfileCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    ledRGBProfile_AppCBs = appCallbacks;
    
    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}

/*********************************************************************
 * @fn      ledRGBProfile_SetParameter
 *
 * @brief   Set a ledRGB Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   len - length of data to write
 * @param   value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t ledRGBProfile_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case LEDRGBPROFILE_CHAR1:
      if ( len == sizeof ( uint8 ) ) 
      {
        ledRGBProfileChar1 = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case LEDRGBPROFILE_CHAR2:
      if ( len == sizeof ( uint8 ) ) 
      {
        ledRGBProfileChar2 = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case LEDRGBPROFILE_CHAR3:
      if ( len == sizeof ( uint8 ) ) 
      {
        ledRGBProfileChar3 = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case LEDRGBPROFILE_CHAR4:
      if ( len == sizeof ( uint8 ) ) 
      {
        ledRGBProfileChar4 = *((uint8*)value);
        
        // See if Notification has been enabled
        GATTServApp_ProcessCharCfg( ledRGBProfileChar4Config, &ledRGBProfileChar4, FALSE,
                                    ledRGBProfileAttrTbl, GATT_NUM_ATTRS( ledRGBProfileAttrTbl ),
                                    INVALID_TASK_ID, ledRGBProfile_ReadAttrCB );
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case LEDRGBPROFILE_CHAR5:
      if ( len == LEDRGBPROFILE_CHAR5_LEN ) 
      {
        VOID memcpy( ledRGBProfileChar5, value, LEDRGBPROFILE_CHAR5_LEN );
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
 * @fn      ledRGBProfile_GetParameter
 *
 * @brief   Get a ledRGB Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t ledRGBProfile_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case LEDRGBPROFILE_CHAR1:
      *((uint8*)value) = ledRGBProfileChar1;
      break;

    case LEDRGBPROFILE_CHAR2:
      *((uint8*)value) = ledRGBProfileChar2;
      break;      

    case LEDRGBPROFILE_CHAR3:
      *((uint8*)value) = ledRGBProfileChar3;
      break;  

    case LEDRGBPROFILE_CHAR4:
      *((uint8*)value) = ledRGBProfileChar4;
      break;

    case LEDRGBPROFILE_CHAR5:
      VOID memcpy( value, ledRGBProfileChar5, LEDRGBPROFILE_CHAR5_LEN );
      break;      
      
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/*********************************************************************
 * @fn          ledRGBProfile_ReadAttrCB
 *
 * @brief       Read an attribute.
 *
 * @param       connHandle - connection message was received on
 * @param       pAttr - pointer to attribute
 * @param       pValue - pointer to data to be read
 * @param       pLen - length of data to be read
 * @param       offset - offset of the first octet to be read
 * @param       maxLen - maximum length of data to be read
 * @param       method - type of read message
 *
 * @return      SUCCESS, blePending or Failure
 */
static bStatus_t ledRGBProfile_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                                           uint8 *pValue, uint8 *pLen, uint16 offset,
                                           uint8 maxLen, uint8 method )
{
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
 
  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      // No need for "GATT_SERVICE_UUID" or "GATT_CLIENT_CHAR_CFG_UUID" cases;
      // gattserverapp handles those reads

      // characteristics 1 and 2 have read permissions
      // characteritisc 3 does not have read permissions; therefore it is not
      //   included here
      // characteristic 4 does not have read permissions, but because it
      //   can be sent as a notification, it is included here
      case LEDRGBPROFILE_CHAR1_UUID:
      case LEDRGBPROFILE_CHAR2_UUID:
      case LEDRGBPROFILE_CHAR4_UUID:
        *pLen = 1;
        pValue[0] = *pAttr->pValue;
        break;

      case LEDRGBPROFILE_CHAR5_UUID:
        *pLen = LEDRGBPROFILE_CHAR5_LEN;
        VOID memcpy( pValue, pAttr->pValue, LEDRGBPROFILE_CHAR5_LEN );
        break;
        
      default:
        // Should never get here! (characteristics 3 and 4 do not have read permissions)
        *pLen = 0;
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // 128-bit UUID
    *pLen = 0;
    status = ATT_ERR_INVALID_HANDLE;
  }

  return ( status );
}

/*********************************************************************
 * @fn      ledRGBProfile_WriteAttrCB
 *
 * @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t ledRGBProfile_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                            uint8 *pValue, uint8 len, uint16 offset,
                                            uint8 method )
{
  bStatus_t status = SUCCESS;
  uint8 notifyApp = 0xFF;
  
  // If attribute permissions require authorization to write, return error
  if ( gattPermitAuthorWrite( pAttr->permissions ) )
  {
    // Insufficient authorization
    return ( ATT_ERR_INSUFFICIENT_AUTHOR );
  }
  
  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      case LEDRGBPROFILE_CHAR1_UUID:
      case LEDRGBPROFILE_CHAR3_UUID:

        //Validate the value
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
        
        //Write the value
        if ( status == SUCCESS )
        {
          uint8 *pCurValue = (uint8 *)pAttr->pValue;        
          *pCurValue = pValue[0];

          if( pAttr->pValue == &ledRGBProfileChar1 )
          {
            notifyApp = LEDRGBPROFILE_CHAR1;        
          }
          else
          {
            notifyApp = LEDRGBPROFILE_CHAR3;           
          }
        }
             
        break;

      case GATT_CLIENT_CHAR_CFG_UUID:
        status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                                 offset, GATT_CLIENT_CFG_NOTIFY );
        break;
        
      default:
        // Should never get here! (characteristics 2 and 4 do not have write permissions)
        status = ATT_ERR_ATTR_NOT_FOUND;
        break;
    }
  }
  else
  {
    // 128-bit UUID
    status = ATT_ERR_INVALID_HANDLE;
  }

  // If a charactersitic value changed then callback function to notify application of change
  if ( (notifyApp != 0xFF ) && ledRGBProfile_AppCBs && ledRGBProfile_AppCBs->pfnledRGBProfileChange )
  {
    ledRGBProfile_AppCBs->pfnledRGBProfileChange( notifyApp );  
  }
  
  return ( status );
}

/*********************************************************************
*********************************************************************/

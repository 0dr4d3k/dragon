/**************************************************************************************************
  Filename:       dragonfly_ledRGB_Service.c
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
// Dragonfly ledRGB Profile Service UUID: 0xFF00
CONST uint8 ledRGBProfileServUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(LEDRGB_PROFILE_SERV_UUID), HI_UINT16(LEDRGB_PROFILE_SERV_UUID)
};

// Red Characteristic UUID: 0xFF01
CONST uint8 ledRGBredUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(LEDRGB_RED_CHAR_UUID), HI_UINT16(LEDRGB_RED_CHAR_UUID)
};

// Green Characteristic  UUID: 0xFF02
CONST uint8 ledRGBgreenUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(LEDRGB_GREEN_CHAR_UUID), HI_UINT16(LEDRGB_GREEN_CHAR_UUID)
};

// Blue Characteristic 3 UUID: 0xFF03
CONST uint8 ledRGBblueUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(LEDRGB_BLUE_CHAR_UUID), HI_UINT16(LEDRGB_BLUE_CHAR_UUID)
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
// Key Pressed Characteristic Configs
static gattCharCfg_t *ledRGBConfig;


// ledRGB Profile Service attribute
static CONST gattAttrType_t ledRGBProfileService = { ATT_BT_UUID_SIZE, ledRGBProfileServUUID };

//LED RED

// Characteristic Red - Properties
static uint8 ledRGBredProfileCharProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic Red - Value
static uint8 ledRGBredProfileChar = 0;

// Characteristic Red - User Description
static uint8 ledRGBredProfileCharUserDesp[14] = "Led RGB - Red";

//LED GREEN

// Characteristic Green - Properties
static uint8 ledRGBgreenProfileCharProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic Green - Value
static uint8 ledRGBgreenProfileChar = 0;

// Characteristic Green - User Description
static uint8 ledRGBgreenProfileCharUserDesp[16] = "Led RGB - Green";

//LED BLUE

// Characteristic Blue - Properties
static uint8 ledRGBblueProfileCharProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic Blue - Value
static uint8 ledRGBblueProfileChar = 0;

// Characteristic Blue - User Description
static uint8 ledRGBblueProfileCharUserDesp[15] = "Led RGB - Blue";



/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t ledRGBProfileAttrTbl[SERVAPP_NUM_ATTR_SUPPORTED] = 
{
  // ledRGB Profile Service
  { 
    { ATT_BT_UUID_SIZE, ledRGBProfileServUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
//    GATT_PERMIT_AUTHEN_READ, 
    0,                                        /* handle */
    (uint8 *)&ledRGBProfileService            /* pValue */
  },

    // Characteristic Red Declaration
    { 
      { ATT_BT_UUID_SIZE, ledRGBredUUID },
      GATT_PERMIT_READ, 
      0,
      &ledRGBredProfileCharProps 
    },

      // Characteristic Red Value 
      { 
        { ATT_BT_UUID_SIZE, ledRGBredUUID },
         GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        &ledRGBredProfileChar 
      },

      // Characteristic Red User Description
      { 
        { ATT_BT_UUID_SIZE, ledRGBredUUID },
        GATT_PERMIT_READ, 
        0, 
        ledRGBredProfileCharUserDesp
      },      

    // Characteristic Green Declaration
    { 
      { ATT_BT_UUID_SIZE, ledRGBgreenUUID },
      GATT_PERMIT_READ, 
      0,
      &ledRGBgreenProfileCharProps 
    },

      // Characteristic Green Value 
      { 
        { ATT_BT_UUID_SIZE, ledRGBgreenUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0, 
        &ledRGBgreenProfileChar 
      },

      // Characteristic Green User Description
      { 
        { ATT_BT_UUID_SIZE, ledRGBgreenUUID },
        GATT_PERMIT_READ, 
        0, 
        ledRGBgreenProfileCharUserDesp
      },      


    // Characteristic Blue Declaration
    { 
      { ATT_BT_UUID_SIZE, ledRGBblueUUID },
      GATT_PERMIT_READ, 
      0,
      &ledRGBblueProfileCharProps 
    },

      // Characteristic Blue Value 
      { 
        { ATT_BT_UUID_SIZE, ledRGBblueUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0, 
        &ledRGBblueProfileChar 
      },

      // Characteristic Blue User Description
      { 
        { ATT_BT_UUID_SIZE, ledRGBblueUUID },
        GATT_PERMIT_READ, 
        0, 
        ledRGBblueProfileCharUserDesp
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
  ledRGBConfig = (gattCharCfg_t *)osal_mem_alloc( sizeof(gattCharCfg_t) *
                                                              linkDBNumConns );
  if ( ledRGBConfig == NULL )
  {     
    return ( bleMemAllocError );
  }
  
  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, ledRGBConfig );
  
  if ( services & LEDRGB_PROFILE_SERVICE )
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
    case LEDRGB_RED_CHAR:
      if ( len == sizeof ( uint8 ) ) 
      {
        ledRGBredProfileChar = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case LEDRGB_GREEN_CHAR:
      if ( len == sizeof ( uint8 ) ) 
      {
        ledRGBgreenProfileChar = *((uint8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case LEDRGB_BLUE_CHAR:
      if ( len == sizeof ( uint8 ) ) 
      {
        ledRGBblueProfileChar = *((uint8*)value);
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
    case LEDRGB_RED_CHAR:
      *((uint8*)value) = ledRGBredProfileChar;
      break;

    case LEDRGB_GREEN_CHAR:
      *((uint8*)value) = ledRGBgreenProfileChar;
      break;      

    case LEDRGB_BLUE_CHAR:
      *((uint8*)value) = ledRGBblueProfileChar;
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
      case LEDRGB_RED_CHAR_UUID:
      case LEDRGB_GREEN_CHAR_UUID:
      case LEDRGB_BLUE_CHAR_UUID:
        *pLen = 1;
        pValue[0] = *pAttr->pValue;
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
      switch ( uuid )
      {
      case LEDRGB_RED_CHAR_UUID:
        notifyApp = LEDRGB_RED_CHAR;
        break;
        
      case LEDRGB_GREEN_CHAR_UUID:
        notifyApp = LEDRGB_GREEN_CHAR; 
        break;
        
      case LEDRGB_BLUE_CHAR_UUID:
        notifyApp = LEDRGB_BLUE_CHAR;           
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
    }
  }
  else
  {
    // 128-bit UUID
    status = ATT_ERR_INVALID_HANDLE;
  }  
  
 /* 
  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    switch ( uuid )
    {
      case SIMPLEPROFILE_CHAR1_UUID:
      case SIMPLEPROFILE_CHAR3_UUID:

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

          if( pAttr->pValue == &simpleProfileChar1 )
          {
            notifyApp = SIMPLEPROFILE_CHAR1;        
          }
          else
          {
            notifyApp = SIMPLEPROFILE_CHAR3;           
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
*/
  // If a charactersitic value changed then callback function to notify application of change
  if ( (notifyApp != 0xFF ) && ledRGBProfile_AppCBs && ledRGBProfile_AppCBs->pfnledRGBProfileChange )
  {
    ledRGBProfile_AppCBs->pfnledRGBProfileChange( notifyApp );  
  }
  
  return ( status );
}

/*********************************************************************
*********************************************************************/

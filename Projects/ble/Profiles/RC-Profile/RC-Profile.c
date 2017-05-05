/**************************************************************************************************
  Filename:       simpleGATTprofile.c
  Revised:        $Date: 2012-08-09 $
  Revision:       $Revision: 1 $

  Description:    This file contains the RC Profile GATT service 
                  profile for use with the TI BLE Car application.

  Copyright 2010 Texas Instruments Incorporated. All rights reserved.

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
#include "bcomdef.h"
#include "OSAL.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "gapbondmgr.h"

#include "RC-Profile.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

//#define SERVAPP_NUM_ATTR_SUPPORTED        17

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
// RC Profile Service UUID: 0xACC0
CONST uint8 rcProfileServUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(RC_PROFILE_SERV_UUID), HI_UINT16(RC_PROFILE_SERV_UUID)
};

// Throttle characteristic UUID: 0xACC1
CONST uint8 rcProfileThrottleUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(RC_THROTTLE_CHAR_UUID), HI_UINT16(RC_THROTTLE_CHAR_UUID)
};

// Steering characteristic UUID: 0xACC2
CONST uint8 rcProfileSteeringUUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(RC_STEERING_CHAR_UUID), HI_UINT16(RC_STEERING_CHAR_UUID)
};

// Characteristic 3 UUID: 0xACC3
CONST uint8 rcProfilechar3UUID[ATT_BT_UUID_SIZE] =
{ 
  LO_UINT16(RC_Z_CHAR_UUID), HI_UINT16(RC_Z_CHAR_UUID)
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

static rcProfileCBs_t *rcProfile_AppCBs = NULL;

/*********************************************************************
 * Profile Attributes - variables
 */

// RC Profile Service attribute
static CONST gattAttrType_t rcProfileService = { ATT_BT_UUID_SIZE, rcProfileServUUID };

// Throttle characteristic Properties
static uint8 rcProfileThrottleProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Throttle characteristic Value
static int8 rcProfileThrottle = 0;

// Throttle characteristic User Description
static uint8 rcProfileThrottleUserDesp[17] = "Throttle\0";


// Steering characteristic Properties
static uint8 rcProfileSteeringProps = GATT_PROP_READ | GATT_PROP_WRITE;

// Steering characteristic Value
static int8 rcProfileSteering = 0;

// Steering characteristic User Description
static uint8 rcProfileSteeringUserDesp[17] = "Steering\0";


// Characteristic 3 Properties
static uint8 rcProfileChar3Props = GATT_PROP_READ | GATT_PROP_WRITE;

// Characteristic 3 Value
static uint8 rcProfileChar3 = 0;

// Characteristic 3 User Description
static uint8 rcProfileChar3UserDesp[17] = "Z\0";



/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t rcProfileAttrTbl[] = 
{
  // RC Profile Service
  { 
    { ATT_BT_UUID_SIZE, primaryServiceUUID }, /* type */
    GATT_PERMIT_READ,                         /* permissions */
    0,                                        /* handle */
    (uint8 *)&rcProfileService            /* pValue */
  },

    // Throttle characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &rcProfileThrottleProps 
    },

      // Throttle characteristic Value
      { 
        { ATT_BT_UUID_SIZE, rcProfileThrottleUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)&rcProfileThrottle 
      },

      // Throttle characteristic User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        rcProfileThrottleUserDesp 
      },      

    // Steering characteristic Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &rcProfileSteeringProps 
    },

      // Steering characteristic Value
      { 
        { ATT_BT_UUID_SIZE, rcProfileSteeringUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        (uint8 *)&rcProfileSteering 
      },

      // Steering characteristic User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        rcProfileSteeringUserDesp 
      },           
      
    // Characteristic 3 Declaration
    { 
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ, 
      0,
      &rcProfileChar3Props 
    },

      // Characteristic Value 3
      { 
        { ATT_BT_UUID_SIZE, rcProfilechar3UUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE, 
        0, 
        &rcProfileChar3 
      },

      // Characteristic 3 User Description
      { 
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ, 
        0, 
        rcProfileChar3UserDesp 
      },

};


/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8 rcProfile_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen );
static bStatus_t rcProfile_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset );


/*********************************************************************
 * PROFILE CALLBACKS
 */
// RC Profile Service Callbacks
CONST gattServiceCBs_t rcProfileCBs =
{
  rcProfile_ReadAttrCB,  // Read callback function pointer
  rcProfile_WriteAttrCB, // Write callback function pointer
  NULL                       // Authorization callback function pointer
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/** @brief   Initializes the RC Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 *
 * @return  Success or Failure
 */
bStatus_t rcProfile_AddService( uint32 services )
{
  uint8 status = SUCCESS;
  
  if ( services & RCROFILE_SERVICE )
  {
    // Register GATT attribute list and CBs with GATT Server App
    status = GATTServApp_RegisterService( rcProfileAttrTbl, 
                                          GATT_NUM_ATTRS( rcProfileAttrTbl ),
                                          &rcProfileCBs );
  }

  return ( status );
}


/** @brief   Registers the application callback function. Only call 
 *          this function once.
 *
 * @param   callbacks - pointer to application callbacks.
 *
 * @return  SUCCESS or bleAlreadyInRequestedMode
 */
bStatus_t rcProfile_RegisterAppCBs( rcProfileCBs_t *appCallbacks )
{
  if ( appCallbacks )
  {
    rcProfile_AppCBs = appCallbacks;
    
    return ( SUCCESS );
  }
  else
  {
    return ( bleAlreadyInRequestedMode );
  }
}
  

/** @brief   Set a RC Profile parameter.
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
bStatus_t rcProfile_SetParameter( uint8 param, uint8 len, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case RC_THROTTLE_CHAR:
      if ( len == sizeof ( int8 ) ) 
      {
        rcProfileThrottle = *((int8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case RC_STEERING_CHAR:
      if ( len == sizeof ( uint8 ) ) 
      {
        rcProfileSteering = *((int8*)value);
      }
      else
      {
        ret = bleInvalidRange;
      }
      break;

    case RC_Z_CHAR:
      if ( len == sizeof ( uint8 ) ) 
      {
        rcProfileChar3 = *((uint8*)value);
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

/** @brief   Get a RC Profile parameter.
 *
 * @param   param - Profile parameter ID
 * @param   value - pointer to data to put.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 *
 * @return  bStatus_t
 */
bStatus_t rcProfile_GetParameter( uint8 param, void *value )
{
  bStatus_t ret = SUCCESS;
  switch ( param )
  {
    case RC_THROTTLE_CHAR:
      *((int8*)value) = rcProfileThrottle;
      break;

    case RC_STEERING_CHAR:
      *((int8*)value) = rcProfileSteering;
      break;      

    case RC_Z_CHAR:
      *((uint8*)value) = rcProfileChar3;
      break;    
      
    default:
      ret = INVALIDPARAMETER;
      break;
  }
  
  return ( ret );
}

/** @brief       Read an attribute.
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
static uint8 rcProfile_ReadAttrCB( uint16 connHandle, gattAttribute_t *pAttr, 
                            uint8 *pValue, uint8 *pLen, uint16 offset, uint8 maxLen )
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
    
    *pLen = 1;
    pValue[0] = *pAttr->pValue;
    //TODO: put some error handling back from the original code?
    
  }
  else
  {
    // 128-bit UUID
    *pLen = 0;
    status = ATT_ERR_INVALID_HANDLE;
  }

  return ( status );
}

/** @brief   Validate attribute data prior to a write operation
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   complete - whether this is the last packet
 * @param   oper - whether to validate and/or write attribute value  
 *
 * @return  Success or Failure
 */
static bStatus_t rcProfile_WriteAttrCB( uint16 connHandle, gattAttribute_t *pAttr,
                                 uint8 *pValue, uint8 len, uint16 offset )
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
      case RC_THROTTLE_CHAR_UUID:
        notifyApp = RC_THROTTLE_CHAR;
        break;
        
      case RC_STEERING_CHAR_UUID:
        notifyApp = RC_STEERING_CHAR; 
        break;
        
      case RC_Z_CHAR_UUID:
        notifyApp = RC_Z_CHAR;           
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

  // If a charactersitic value changed then callback function to notify application of change
  if ( (notifyApp != 0xFF ) && rcProfile_AppCBs && rcProfile_AppCBs->pfnrcProfileChange )
  {
    rcProfile_AppCBs->pfnrcProfileChange( notifyApp );  
  }
  
  return ( status );
}


/*********************************************************************
*********************************************************************/

/**************************************************************************************************
  Filename:       dragonfly_ledRGB_Service.h
  Revised:        $Date: 2010-08-06 08:56:11 -0700 (Fri, 06 Aug 2010) $
  Revision:       $Revision: 23333 $

  Description:    This file contains the ledRGB GATT profile definitions and
                  prototypes.

  Copyright 2010 - 2013 Texas Instruments Incorporated. All rights reserved.

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

#ifndef DRAGONFLY_LEDRGB_SERVICE_H
#define DRAGONFLY_LEDRGB_SERVICE_H

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

// Profile Parameters
#define LEDRGB_RED_CHAR                 0  // RW uint8 - Dragonfly Profile ledRGB Red value 
#define LEDRGB_GREEN_CHAR               1  // RW uint8 - Dragonfly Profile ledRGB Green value 
#define LEDRGB_BLUE_CHAR                2  // RW uint8 - Dragonfly Profile ledRGB Blue value 
  
// Dragonfly ledRGB Profile Service UUID
#define LEDRGB_PROFILE_SERV_UUID        0xFF00
    
// Dragonfly ledRGB Profile Characteristics UUID
#define LEDRGB_RED_CHAR_UUID            0xFF01
#define LEDRGB_GREEN_CHAR_UUID          0xFF02
#define LEDRGB_BLUE_CHAR_UUID           0xFF03
  
// ledRGB Keys Profile Services bit fields
#define LEDRGB_PROFILE_SERVICE               0x00000001


/*********************************************************************
 * TYPEDEFS
 */

  
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * Profile Callbacks
 */

// Callback when a characteristic value has changed
typedef void (*ledRGBProfileChange_t)( uint8 paramID );

typedef struct
{
  ledRGBProfileChange_t        pfnledRGBProfileChange;  // Called when characteristic value changes
} ledRGBProfileCBs_t;

    

/*********************************************************************
 * API FUNCTIONS 
 */


/*
 * ledRGBProfile_AddService- Initializes the ledRGB GATT Profile service by registering
 *          GATT attributes with the GATT server.
 *
 * @param   services - services to add. This is a bit map and can
 *                     contain more than one service.
 */

extern bStatus_t ledRGBProfile_AddService( uint32 services );

/*
 * ledRGBProfile_RegisterAppCBs - Registers the application callback function.
 *                    Only call this function once.
 *
 *    appCallbacks - pointer to application callbacks.
 */
extern bStatus_t ledRGBProfile_RegisterAppCBs( ledRGBProfileCBs_t *appCallbacks );

/*
 * ledRGBProfile_SetParameter - Set a ledRGB GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    len - length of data to right
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t ledRGBProfile_SetParameter( uint8 param, uint8 len, void *value );
  
/*
 * ledRGBProfile_GetParameter - Get a ledRGB GATT Profile parameter.
 *
 *    param - Profile parameter ID
 *    value - pointer to data to write.  This is dependent on
 *          the parameter ID and WILL be cast to the appropriate 
 *          data type (example: data type of uint16 will be cast to 
 *          uint16 pointer).
 */
extern bStatus_t ledRGBProfile_GetParameter( uint8 param, void *value );


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* DRAGONFLY_LEDRGB_SERVICE_H */

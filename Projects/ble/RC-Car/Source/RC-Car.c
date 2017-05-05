/**************************************************************************************************
  Filename:       RC-Car.c
  Revised:        $Date: 2012-07-06 $
  Revision:       $Revision: 1 $

  Description:    This file contains the BLE RC Car sample application 
                  for use with the CC254x Bluetooth Low Energy Protocol Stack.

  Copyright 2010 - 2011 Texas Instruments Incorporated. All rights reserved.

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
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"

#if defined( HAL_LED ) && ( HAL_LED )
  #include "hal_led.h"
#endif
#if defined( HAL_LCD ) && ( HAL_LCD )
  #include "hal_lcd.h"
#endif

#include "gatt.h"
#include "hci.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "RC-Profile.h"


#if defined ( PLUS_BROADCASTER )
  #include "peripheralBroadcaster.h"
#else
  #include "peripheral.h"
#endif

#include "gapbondmgr.h"

#include "RC-Car.h"

#include "rc.h"


/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/** \brief How often to perform periodic event */
#define RC_PWM_PERIOD                         20

/** \brief What is the advertising interval when device is discoverable (units of 625us, 160=100ms) */
#define DEFAULT_ADVERTISING_INTERVAL          160

/** \brief Whether to enable automatic parameter update request when a connection is formed */
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

/** \brief Limited discoverable mode advertises for 30.72s, and then stops
* General discoverable mode advertises indefinitely  */
#define DEFAULT_DISCOVERABLE_MODE             GAP_ADTYPE_FLAGS_GENERAL

/** \brief Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled */
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL     32

/** \brief Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled */
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL     64

/** \brief Slave latency to use if automatic parameter update request is enabled */
#define DEFAULT_DESIRED_SLAVE_LATENCY         5

/** \brief Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled */
#define DEFAULT_DESIRED_CONN_TIMEOUT          50

/** \brief Company Identifier: Texas Instruments Inc. (13) */
#define TI_COMPANY_ID                         0x000D

#define INVALID_CONNHANDLE                    0xFFFF

/** \brief Length of bd addr as a string */
#define B_ADDR_STR_LEN                        15

#if defined ( PLUS_BROADCASTER )
  #define ADV_IN_CONN_WAIT                    500 // delay 500 ms
#endif

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8 RC_Car_TaskID;   // Task ID for internal task/event processing

/** \brief GAP - SCAN RSP data (max size = 31 bytes) */
static uint8 scanRspData[] =
{
  // complete name
  0x0B,   // length of this data
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  0x54,   // 'T'
  0x49,   // 'I'
  0x2D,   // '-'
  0x42,   // 'B'
  0x4C,   // 'L'
  0x45,   // 'E'
  0x2D,   // '-'
  0x43,   // 'C'
  0x61,   // 'a'
  0x72,   // 'r'
  
  // connection interval range
  0x05,   // length of this data
  GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
  LO_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),   // 100ms
  HI_UINT16( DEFAULT_DESIRED_MIN_CONN_INTERVAL ),  
  LO_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),   // 1s
  HI_UINT16( DEFAULT_DESIRED_MAX_CONN_INTERVAL ),  

  // Tx power level
  0x02,   // length of this data
  GAP_ADTYPE_POWER_LEVEL,
  0,       // 0dBm  
  
  0x03,   // Length of this data
  GAP_ADTYPE_MANUFACTURER_SPECIFIC,
  TI_COMPANY_ID,
  0xCA,   // It's a CAr 
};

/** \brief  GAP - Advertisement data (max size = 31 bytes, though this is
* best kept short to conserve power while advertisting) */
static uint8 advertData[] = 
{ 
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

  // service UUID, to notify central devices what services are included
  // in this peripheral
  0x03,   // length of this data
  GAP_ADTYPE_16BIT_MORE,      // some of the UUID's, but not all
  LO_UINT16( RC_PROFILE_SERV_UUID ),
  HI_UINT16( RC_PROFILE_SERV_UUID ),
  
  0x03,   // Length of this data
  GAP_ADTYPE_MANUFACTURER_SPECIFIC,
  TI_COMPANY_ID,
  0xCA,   // It's a CAr 
};

/** \brief  GAP GATT Attributes */
static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "Simple BLE Peripheral";

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void peripheralStateNotificationCB( gaprole_States_t newState );
static void simpleProfileChangeCB( uint8 paramID );
static void RC_Car_Stop();

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t simpleBLEPeripheral_PeripheralCBs =
{
  peripheralStateNotificationCB,  // Profile State Change Callbacks
  NULL                            // When a valid RSSI is read from controller (not used by application)
};

// GAP Bond Manager Callbacks
static gapBondCBs_t simpleBLEPeripheral_BondMgrCBs =
{
  NULL,                     // Passcode callback (not used by application)
  NULL                      // Pairing / Bonding state Callback (not used by application)
};

// Simple GATT Profile Callbacks
static rcProfileCBs_t simpleBLEPeripheral_SimpleProfileCBs =
{
  simpleProfileChangeCB    // Charactersitic value change callback
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/** @brief   Initialization function for the Simple BLE Peripheral App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void RC_Car_Init( uint8 task_id )
{
  RC_Car_TaskID = task_id;

  // Setup the GAP Peripheral Role Profile
  {
    uint8 initial_advertising_enable = TRUE;

    // By setting this to zero, the device will go into the waiting state after
    // being discoverable for 30.72 second, and will not being advertising again
    // until the enabler is set back to TRUE
    uint16 gapRole_AdvertOffTime = 0;
      
    uint8 enable_update_request = DEFAULT_ENABLE_UPDATE_REQUEST;
    uint16 desired_min_interval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
    uint16 desired_max_interval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
    uint16 desired_slave_latency = DEFAULT_DESIRED_SLAVE_LATENCY;
    uint16 desired_conn_timeout = DEFAULT_DESIRED_CONN_TIMEOUT;

    // Set the GAP Role Parameters
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );
    
    GAPRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
    GAPRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );
    
    GAPRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }
  
  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName );

  // Set advertising interval
  {
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
  }
  
  // Setup the GAP Bond Manager
  {
    uint32 passkey = 0; // passkey "000000"
    uint8 pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
    uint8 mitm = TRUE;
    uint8 ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
    uint8 bonding = TRUE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof ( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof ( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof ( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof ( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof ( uint8 ), &bonding );
  }  

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );            // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES );    // GATT attributes
  DevInfo_AddService();                           // Device Information Service
  rcProfile_AddService( GATT_ALL_SERVICES );  // Simple GATT Profile  
  
  // Make sure the characteristic values are initialized to zero
  RC_Car_Stop();

#if (defined HAL_LCD) && (HAL_LCD == TRUE)  

    HalLcdWriteString( "TI BLE Car", HAL_LCD_LINE_1 );
    HalLcdWriteString( "Monitor is:", HAL_LCD_LINE_2 );
  
#endif // (defined HAL_LCD) && (HAL_LCD == TRUE)  
  
  // Register callback with SimpleGATTprofile
  VOID rcProfile_RegisterAppCBs( &simpleBLEPeripheral_SimpleProfileCBs );
  
  // Enable clock divide on halt
  // This reduces active current while radio is active and CC254x MCU
  // is halted
  HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );
  
#if defined( CC2540_CAR )

  // Initialize rc car
  rcInit();
  
#endif // #if defined( CC2540_CAR )
  
  // Setup a delayed profile startup
  osal_set_event( RC_Car_TaskID, RC_START_DEVICE_EVT );
  
  // Set timer for first RC Pulse
  osal_start_timerEx( RC_Car_TaskID, RC_PWM_EVT, RC_PWM_PERIOD );

}

/** @brief   Simple BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 RC_Car_ProcessEvent( uint8 task_id, uint16 events )
{
  
  VOID task_id; // OSAL required parameter that isn't used in this function

  if ( events & RC_START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPRole_StartDevice( &simpleBLEPeripheral_PeripheralCBs );
    
    // Start Bond Manager
    
    VOID GAPBondMgr_Register( &simpleBLEPeripheral_BondMgrCBs );
    
    return ( events ^ RC_START_DEVICE_EVT );
  }
  
  if ( events & RC_PWM_EVT )
  {
    static int8 throttle, steering;
    
    //Get steering and throttle values
    rcProfile_GetParameter(RC_THROTTLE_CHAR, &throttle);
    rcProfile_GetParameter(RC_STEERING_CHAR, &steering);
    
    // Make steering and throttle control pulses
    rcPulse(throttle, steering);
    
    // Schedule the next pulses
    osal_start_timerEx( RC_Car_TaskID, RC_PWM_EVT, RC_PWM_PERIOD );
    
    return (events ^ RC_PWM_EVT);
  }  
     
#if defined ( PLUS_BROADCASTER )
  if ( events & RC_ADV_IN_CONNECTION_EVT )
  {
    uint8 turnOnAdv = TRUE;
    // Turn on advertising while in a connection
    GAPRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &turnOnAdv );

    return (events ^ RC_ADV_IN_CONNECTION_EVT);
  }  
#endif // PLUS_BROADCASTER
  
  // Discard unknown events
  return 0;
}


/** @brief   Notification from the profile of a state change.
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void peripheralStateNotificationCB( gaprole_States_t newState )
{
  #if (defined HAL_LCD) && (HAL_LCD == TRUE)
    HalLcdWriteString( "TI BLE Car", HAL_LCD_LINE_1 );
    HalLcdWriteString( "Monitor is:", HAL_LCD_LINE_2 );
    HalLcdWriteString( "", HAL_LCD_LINE_3 );
  #endif
    
  #if defined( CC2540_CAR )
    //Make sure the car is not moving when a state change occurs
    RC_Car_Stop();
  #endif
    
  switch ( newState )
  {
    case GAPROLE_STARTED:
      {    
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Initialized",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE) 
      }
      break;
      
    case GAPROLE_ADVERTISING:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Advertising",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE) 
      }
      break;

    case GAPROLE_CONNECTED:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Connected",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)  
      }
      break;      

    case GAPROLE_WAITING:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Disconnected",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)    
      }
      break;      

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Timed Out",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)     
      }
      break;      

    case GAPROLE_ERROR:
      {
        #if (defined HAL_LCD) && (HAL_LCD == TRUE)
          HalLcdWriteString( "Error",  HAL_LCD_LINE_3 );
        #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)
      }
      break;      
      
    default:
      {
        // Nothing
      }        
      break;
    
  }
  
}

/** @brief   Callback from SimpleBLEProfile indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void simpleProfileChangeCB( uint8 paramID )
{
  #if (defined HAL_LCD) && (HAL_LCD == TRUE)
  
    uint8 newValue;
    
    //hal_lcd.c does not handle negative numbers so here is a quick and dirty workaround...
    
    switch( paramID )
    {
    case RC_THROTTLE_CHAR:
      SimpleProfile_GetParameter( RC_THROTTLE_CHAR, &newValue );
      
      if ( newValue < 127 )
        HalLcdWriteStringValue( "Throttle:  ", (uint8)(newValue), 10,  HAL_LCD_LINE_1 );
      else
        HalLcdWriteStringValue( "Throttle: -", (uint8)(255-newValue), 10,  HAL_LCD_LINE_1 );
      
      break;
      
    case RC_STEERING_CHAR:
      SimpleProfile_GetParameter( RC_STEERING_CHAR, &newValue );
      
      if ( newValue < 127 )
        HalLcdWriteStringValue( "Steering:  ", (uint8)(newValue), 10,  HAL_LCD_LINE_2 );
      else
        HalLcdWriteStringValue( "Steering: -", (uint8)(255-newValue), 10,  HAL_LCD_LINE_2 );
      
      break;
      
//    case RC_Z_CHAR:
//      SimpleProfile_GetParameter( RC_Z_CHAR, &newValue );
//      
//      if ( newValue < 127 )
//        HalLcdWriteStringValue( "Z:  ", (uint8)(newValue), 10,  HAL_LCD_LINE_3 );
//      else
//        HalLcdWriteStringValue( "Z: -", (uint8)(255-newValue), 10,  HAL_LCD_LINE_3 );
//      
//      break;
      
    default:
      // should not reach here!
      break;
    }  
  #endif // (defined HAL_LCD) && (HAL_LCD == TRUE)  
}

/** @brief   Sets throttle and steering to zero
 *
 * @return  none
 */
static void RC_Car_Stop()
{
  int8 throttle = 0;
  int8 steering = 0;
  
  //Set steering and throttle values
  rcProfile_SetParameter(RC_THROTTLE_CHAR, sizeof(int8), &throttle);
  rcProfile_SetParameter(RC_STEERING_CHAR, sizeof(int8), &steering);
  
}
/*********************************************************************
*********************************************************************/

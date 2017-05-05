/**************************************************************************************************
  Filename:       RC-Keyfob.c
  Revised:        $Date: 2012-06-15 $
  Revision:       $Revision: 1 $

  Description:    This file contains the TI BLE Car sample application for the Keyfob controller.

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
#include "OSAL_PwrMgr.h"
#include "OnBoard.h"
#include "hal_led.h"
#include "hal_key.h"
#include "gatt.h"
#include "ll.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "gatt.h"
#include "central.h"
#include "gapbondmgr.h"
#include "RC-Profile.h"
#include "RC-Keyfob.h"

#if defined ( BUZZER )
  #include "buzzer.h"
#endif

#include "cma3000d.h"

/*********************************************************************
 * MACROS
 */

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

/*********************************************************************
 * CONSTANTS
 */

/** \brief Maximum number of scan responses */
#define DEFAULT_MAX_SCAN_RES                  8

/** \brief Scan duration in ms */
#define DEFAULT_SCAN_DURATION                 3000

/** \brief Discovey mode (limited, general, all) */
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

/** \brief TRUE to use active scan */
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         TRUE

/** \brief TRUE to use white list during discovery */
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

/** \brief TRUE to use high scan duty cycle when creating link */
#define DEFAULT_LINK_HIGH_DUTY_CYCLE          FALSE

/** \brief TRUE to use white list when creating link */
#define DEFAULT_LINK_WHITE_LIST               FALSE

/** \brief Default RSSI polling period in ms */
#define DEFAULT_RSSI_PERIOD                   1000

/** \brief Critical and alert RSSI levels */
#define DEFAULT_RSSI_ALERT_LEVEL              -85

/** \brief Whether to enable automatic parameter update request when a connection is formed */
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

/** \brief Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled */
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL      32

/** \brief Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled */
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL      64

/** \brief Slave latency to use if automatic parameter update request is enabled */
#define DEFAULT_UPDATE_SLAVE_LATENCY          2

/** \brief Supervision timeout value (units of 10ms) if automatic parameter update request is enabled */
#define DEFAULT_UPDATE_CONN_TIMEOUT           100

/** \brief Default passcode */
#define DEFAULT_PASSCODE                      19655

/** \brief Default GAP pairing mode */
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

/** \brief Default MITM mode (TRUE to require passcode or OOB when pairing) */
#define DEFAULT_MITM_MODE                     FALSE

/** \brief Default bonding mode, TRUE to bond */
#define DEFAULT_BONDING_MODE                  TRUE

/** \brief Default GAP bonding I/O capabilities */
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_DISPLAY_ONLY

/** \brief Default service discovery timer delay in ms */
#define DEFAULT_SVC_DISCOVERY_DELAY           50

/** \brief TRUE to filter discovery results on desired service UUID */
#define DEFAULT_DEV_DISC_BY_SVC_UUID          TRUE

/** \brief Application states */
enum
{
  BLE_STATE_IDLE,
  BLE_STATE_CONNECTING,
  BLE_STATE_CONNECTED,
  BLE_STATE_DISCONNECTING
};

/** \brief Discovery states */
enum
{
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_CHAR                 // Characteristic discovery
};

#if defined ( BUZZER )
/** \brief Buzzer beep tone frequency (in Hz) */
  #define BUZZER_ALERT_FREQ           800
#endif

//Accelerometer settings
/** \brief How often (in ms) to read the accelerometer */
#define ACCEL_READ_PERIOD             20

/** \brief Minimum change in accelerometer before sending a notification */
#define ACCEL_CHANGE_THRESHOLD        2

/** \brief Determines how much the keyfob must tilt before sending a value */
#define ACCEL_X_DEADZONE              10
#define ACCEL_Y_DEADZONE              10

/** \brief Time to hold button 1&2 to connect/disconnect */
#define RC_TOGGLE_CONNECT_PRESS       2000

/** \brief Time to hold button 2 to toggle accelerometer */
#define RC_TOGGLE_ACCEL_PRESS         1000

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

// Task ID for internal task/event processing
static uint8 rcKeyfobTaskId;

// GAP GATT Attributes
static const uint8 rcKeyfobDeviceName[GAP_DEVICE_NAME_LEN] = "TI BLE Car Keyfob";

// Number of scan results and scan result index
static uint8 rcKeyfobScanRes;
static uint8 rcKeyfobScanIdx;

// Scan result list
static gapDevRec_t rcKeyfobDevList[DEFAULT_MAX_SCAN_RES];

// Scanning state
static uint8 rcKeyfobScanning = FALSE;

// Connection handle of current connection 
static uint16 rcKeyfobConnHandle = GAP_CONNHANDLE_INIT;

// Application state
static uint8 rcKeyfobState = BLE_STATE_IDLE;

// Discovery state
static uint8 rcKeyfobDiscState = BLE_DISC_STATE_IDLE;

// Discovered service start and end handle
static uint16 rcKeyfobSvcStartHdl = 0;
static uint16 rcKeyfobSvcEndHdl = 0;

// Discovered characteristics handles
static uint16 throttleCharHdl;
static uint16 steeringCharHdl;
static uint16 zCharHdl;

// GATT read/write procedure state
static bool rcProcedureInProgress = FALSE;

// Accelerometer Profile Parameters
static uint8 accelEnabled = FALSE;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void rcKeyfobProcessGATTMsg( gattMsgEvent_t *pMsg );
static void rcKeyfobRssiCB( uint16 connHandle, int8  rssi );
static void rcKeyfobEventCB( gapCentralRoleEvent_t *pEvent );
static void rcKeyfobPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs );
static void rcKeyfobPairStateCB( uint16 connHandle, uint8 state, uint8 status );
static void rcKeyfob_HandleKeys( uint8 shift, uint8 keys );
static void rcKeyfob_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void rcKeyfobGATTDiscoveryEvent( gattMsgEvent_t *pMsg );
static void rcKeyfobStartDiscovery( void );
static bool rcFindCar( uint8 *pData, uint8 dataLen );
static void rcKeyfobAddDeviceInfo( uint8 *pAddr, uint8 addrType );
static void rcCarUpdateDirectives( void );
static void rcCarWrite(int8 val, uint16 handle);
static void rcCarStop( void );

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static const gapCentralRoleCB_t rcKeyfobRoleCB =
{
  rcKeyfobRssiCB,       // RSSI callback
  rcKeyfobEventCB       // Event callback
};

// Bond Manager Callbacks
static const gapBondCBs_t rcKeyfobBondCB =
{
  rcKeyfobPasscodeCB,
  rcKeyfobPairStateCB
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/** @brief   Initialization function for the Simple BLE Central App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void rcKeyfob_Init( uint8 task_id )
{
  rcKeyfobTaskId = task_id;

  // Setup Central Profile
  {
    uint8 scanRes = DEFAULT_MAX_SCAN_RES;
    GAPCentralRole_SetParameter ( GAPCENTRALROLE_MAX_SCAN_RES, sizeof( uint8 ), &scanRes );
  }
  
  // Setup GAP
  GAP_SetParamValue( TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION );
  GAP_SetParamValue( TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION );
  GAP_SetParamValue( TGAP_CONN_EST_INT_MIN, DEFAULT_UPDATE_MIN_CONN_INTERVAL );
  GAP_SetParamValue( TGAP_CONN_EST_INT_MAX, DEFAULT_UPDATE_MAX_CONN_INTERVAL );
  GAP_SetParamValue( TGAP_CONN_EST_LATENCY, DEFAULT_UPDATE_SLAVE_LATENCY );
  GAP_SetParamValue( TGAP_CONN_EST_SUPERV_TIMEOUT, DEFAULT_UPDATE_CONN_TIMEOUT );
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, (uint8 *) rcKeyfobDeviceName );

  // Setup the GAP Bond Manager
  {
    uint32 passkey = DEFAULT_PASSCODE;
    uint8 pairMode = DEFAULT_PAIRING_MODE;
    uint8 mitm = DEFAULT_MITM_MODE;
    uint8 ioCap = DEFAULT_IO_CAPABILITIES;
    uint8 bonding = DEFAULT_BONDING_MODE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof( uint8 ), &bonding );
  }  

  // Initialize GATT Client
  VOID GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd( rcKeyfobTaskId );

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );         // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES ); // GATT attributes
  
  // For keyfob board set GPIO pins into a power-optimized state
  // Note that there is still some leakage current from the buzzer,
  // accelerometer, LEDs, and buttons on the PCB.
  
  P0SEL = 0; // Configure Port 0 as GPIO
  P1SEL = 0x40; // Configure Port 1 as GPIO, except P1.6 for peripheral function for buzzer
  P2SEL = 0; // Configure Port 2 as GPIO

  P0DIR = 0xFC; // Port 0 pins P0.0 and P0.1 as input (buttons),
                // all others (P0.2-P0.7) as output
  P1DIR = 0xFF; // All port 1 pins (P1.0-P1.7) as output
  P2DIR = 0x1F; // All port 1 pins (P2.0-P2.4) as output
  
  P0 = 0x03; // All pins on port 0 to low except for P0.0 and P0.1 (buttons)
  P1 = 0;   // All pins on port 1 to low
  P2 = 0;   // All pins on port 2 to low  
  
  // Register for all key events - This app will handle all key events
  RegisterForKeys( rcKeyfobTaskId );  
  
  #if defined ( BUZZER )
    // initialize the buzzer
    buzzerInit();

    // make sure buzzer is off
    buzzerStop();
  #endif
    
  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );
  
  // Setup a delayed profile startup
  osal_start_timerEx( rcKeyfobTaskId, START_DEVICE_EVT, 100 );
}

/** @brief   Simple BLE Central Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 rcKeyfob_ProcessEvent( uint8 task_id, uint16 events )
{
  
  VOID task_id; // OSAL required parameter that isn't used in this function
  
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( rcKeyfobTaskId )) != NULL )
    {
      rcKeyfob_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPCentralRole_StartDevice( (gapCentralRoleCB_t *) &rcKeyfobRoleCB );

    
    // Register with bond manager after starting device
    GAPBondMgr_Register( (gapBondCBs_t *) &rcKeyfobBondCB );

    return ( events ^ START_DEVICE_EVT );
  }

  if ( events & START_DISCOVERY_EVT )
  {
    rcKeyfobStartDiscovery( );
    
    return ( events ^ START_DISCOVERY_EVT );
  }
  
  if ( events & RC_ACCEL_READ_EVT )
  {

    if ( accelEnabled )
    {
      // Restart timer
      if ( ACCEL_READ_PERIOD )
      {
        osal_start_timerEx( rcKeyfobTaskId, RC_ACCEL_READ_EVT, ACCEL_READ_PERIOD );
      }
      //Read accelerometers and update car directive
      rcCarUpdateDirectives();
    }
    else
    {
      // Stop the acceleromter
      osal_stop_timerEx( rcKeyfobTaskId, RC_ACCEL_READ_EVT);
    }
    return (events ^ RC_ACCEL_READ_EVT);
  }
  
    if ( events & RC_LEDS_STOP_EVT )
  {
    HalLedSet( HAL_LED_ALL, HAL_LED_MODE_OFF );
    
    return (events ^ RC_LEDS_STOP_EVT);
  }  
  
  #if defined ( BUZZER )
    if ( events & RC_BUZZER_STOP_EVT )
    {
      buzzerStop();
  
      #if defined ( POWER_SAVING )
        osal_pwrmgr_device( PWRMGR_BATTERY );
      #endif
    
      return (events ^ RC_BUZZER_STOP_EVT);
    }
  #endif    
  
  if ( events & RC_TOGGLE_CONNECT_EVT )
  {
    
    HalLedSet( HAL_LED_ALL, HAL_LED_MODE_OFF );
    
    // Start or stop discovery and connect
    if ( rcKeyfobState != BLE_STATE_CONNECTED )
    {

      rcKeyfobScanning = TRUE;
      rcKeyfobScanRes = 0;
      
      GAPCentralRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                    DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                    DEFAULT_DISCOVERY_WHITE_LIST );
      HalLedSet( HAL_LED_ALL, HAL_LED_MODE_FLASH );
      osal_start_timerEx( rcKeyfobTaskId, RC_LEDS_STOP_EVT, DEFAULT_SCAN_DURATION );
      
    }
    else if ( rcKeyfobState == BLE_STATE_CONNECTING ||
             rcKeyfobState == BLE_STATE_CONNECTED )
    {
      // Disconnect
      rcKeyfobState = BLE_STATE_DISCONNECTING;
      
      VOID GAPCentralRole_TerminateLink( rcKeyfobConnHandle );

      HalLedSet( HAL_LED_2, HAL_LED_MODE_BLINK );
    }
    
    return (events ^ RC_TOGGLE_CONNECT_EVT);
  }
  
  if ( events & RC_TOGGLE_ACCEL_EVT )
  {

    // Start or stop accelerometer
    if (rcKeyfobState == BLE_STATE_CONNECTED && accelEnabled == FALSE)
    {
      // Make sure the car is stopped
      rcCarStop();
      
      // Initialize accelerometer
      accInit();
      
      // Setup timer for accelerometer task
      osal_start_timerEx( rcKeyfobTaskId, RC_ACCEL_READ_EVT, ACCEL_READ_PERIOD );
      
      accelEnabled = TRUE;
      
      HalLedSet( HAL_LED_1, HAL_LED_MODE_BLINK );
      
    } else 
    {
      // Stop the car
      rcCarStop();
      
      // Stop the acceleromter
      osal_stop_timerEx( rcKeyfobTaskId, RC_ACCEL_READ_EVT);
      
      accelEnabled = FALSE;
      HalLedSet( HAL_LED_ALL, HAL_LED_MODE_BLINK );
    }
    
    
    return (events ^ RC_TOGGLE_ACCEL_EVT);
  }
  
  // Discard unknown events
  return 0;
}

/** @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void rcKeyfob_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
    case KEY_CHANGE:
      rcKeyfob_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;

    case GATT_MSG_EVENT:
      rcKeyfobProcessGATTMsg( (gattMsgEvent_t *) pMsg );
      break;
  }
}

/** @brief   Handles all key events for this device. Press and
 *          release both generates a call to this function.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
static void rcKeyfob_HandleKeys( uint8 shift, uint8 keys )
{
  (void)shift;  // Intentionally unreferenced parameter
  
  // New key event has occured, clear any previous ones
  osal_stop_timerEx( rcKeyfobTaskId, RC_TOGGLE_CONNECT_EVT );
  osal_stop_timerEx( rcKeyfobTaskId, RC_TOGGLE_ACCEL_EVT );
  
  if ( ( keys & HAL_KEY_SW_1 ) && ( keys & HAL_KEY_SW_2 ) )
  {
    // Hold (for 2s) or key release event will clear the task
    osal_start_timerEx( rcKeyfobTaskId, RC_TOGGLE_CONNECT_EVT, RC_TOGGLE_CONNECT_PRESS );
  }
  
  else if ( keys & HAL_KEY_SW_1 )
  {
    //Do cool stuff
  }

  else if ( keys & HAL_KEY_SW_2 )
  {
    // Hold (for 1s) or key release event will clear the task
    osal_start_timerEx( rcKeyfobTaskId, RC_TOGGLE_ACCEL_EVT, RC_TOGGLE_ACCEL_PRESS );
  }
}

/** @brief   Process GATT messages
 *
 * @return  none
 */
static void rcKeyfobProcessGATTMsg( gattMsgEvent_t *pMsg )
{
  if ( rcKeyfobState != BLE_STATE_CONNECTED )
  {
    // In case a GATT message came after a connection has dropped,
    // ignore the message
    return;
  }
  
  if ( ( pMsg->method == ATT_READ_RSP ) ||
       ( ( pMsg->method == ATT_ERROR_RSP ) &&
         ( pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ ) ) )
  {
    if ( pMsg->method == ATT_ERROR_RSP )
    {
      //uint8 status = pMsg->msg.errorRsp.errCode;
      //LCD_WRITE_STRING_VALUE( "Read Error", status, 10, HAL_LCD_LINE_1 );
    }
    else
    {
      // After a successful read, display the read value
      //uint8 valueRead = pMsg->msg.readRsp.value[0];
      //LCD_WRITE_STRING_VALUE( "Read rsp:", valueRead, 10, HAL_LCD_LINE_1 );
    }
    
    rcProcedureInProgress = FALSE;
    
  }
  else if ( ( pMsg->method == ATT_WRITE_RSP ) ||
       ( ( pMsg->method == ATT_ERROR_RSP ) &&
         ( pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ ) ) )
  {
    
    if ( pMsg->method == ATT_ERROR_RSP == ATT_ERROR_RSP )
    {
      //uint8 status = pMsg->msg.errorRsp.errCode;
      
      //LCD_WRITE_STRING_VALUE( "Write Error", status, 10, HAL_LCD_LINE_1 );
    }
    else
    {
      // After a succesful write, display the value that was written and increment value
      //LCD_WRITE_STRING_VALUE( "Write sent:", rcKeyfobCharVal++, 10, HAL_LCD_LINE_1 );      
    } 

    rcProcedureInProgress = FALSE;
    
  }
  else if ( rcKeyfobDiscState != BLE_DISC_STATE_IDLE )
  {
    rcKeyfobGATTDiscoveryEvent( pMsg );
  }
  
}

/** @brief   RSSI callback. Sounds the buzzer as a warning if the
 *          RSSI is below the alert level.
 *
 * @param   connHandle - connection handle
 * @param   rssi - RSSI
 *
 * @return  none
 */
static void rcKeyfobRssiCB( uint16 connHandle, int8 rssi )
{
  if ( rssi < DEFAULT_RSSI_ALERT_LEVEL )
  {
    //Alert level, warn!
    HalLedSet( HAL_LED_ALL, HAL_LED_MODE_FLASH );
    
    #if defined ( BUZZER )
      
      #if defined ( POWER_SAVING )
        osal_pwrmgr_device( PWRMGR_ALWAYS_ON );
      #endif
          
      buzzerStart( BUZZER_ALERT_FREQ );
      osal_start_timerEx( rcKeyfobTaskId, RC_BUZZER_STOP_EVT, 100);
    #endif

  }
  else
  {
    //All is fine, turn off leds
    HalLedSet( HAL_LED_ALL, HAL_LED_MODE_OFF );
  }
}

/** @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void rcKeyfobEventCB( gapCentralRoleEvent_t *pEvent )
{
  switch ( pEvent->gap.opcode )
  {
    case GAP_DEVICE_INIT_DONE_EVENT:  
      {

      }
      break;

    case GAP_DEVICE_INFO_EVENT:
      {
        // if filtering device discovery results based on advertising data
        if ( DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE )
        {
          if ( rcFindCar( pEvent->deviceInfo.pEvtData, pEvent->deviceInfo.dataLen ) )
          {
            rcKeyfobAddDeviceInfo( pEvent->deviceInfo.addr, pEvent->deviceInfo.addrType );
          }
        }
      }
      break;
      
    case GAP_DEVICE_DISCOVERY_EVENT:
      {
        // discovery complete
        rcKeyfobScanning = FALSE;

        // if not filtering device discovery results based on service UUID
        if ( DEFAULT_DEV_DISC_BY_SVC_UUID == FALSE )
        {
          // Copy results
          rcKeyfobScanRes = pEvent->discCmpl.numDevs;
          osal_memcpy( rcKeyfobDevList, pEvent->discCmpl.pDevList,
                       (sizeof( gapDevRec_t ) * pEvent->discCmpl.numDevs) );
          
        }
        

        if ( rcKeyfobScanRes > 0 )
        {
          //connect to the found device
          if ( !rcKeyfobScanning && rcKeyfobScanRes > 0 )
          {
            // Increment index of current result (with wraparound)
            rcKeyfobScanIdx++;
            if ( rcKeyfobScanIdx >= rcKeyfobScanRes )
            {
              rcKeyfobScanIdx = 0;
            }    
          }

          uint8 addrType;
          uint8 *peerAddr;
          
          // Connect or disconnect
          if ( rcKeyfobState == BLE_STATE_IDLE )
          {
            // if there is a scan result
            if ( rcKeyfobScanRes > 0 )
            {
              // connect to current device in scan result
              peerAddr = rcKeyfobDevList[rcKeyfobScanIdx].addr;
              addrType = rcKeyfobDevList[rcKeyfobScanIdx].addrType;
              
              rcKeyfobState = BLE_STATE_CONNECTING;
              GAPCentralRole_EstablishLink( DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                           DEFAULT_LINK_WHITE_LIST,
                                           addrType, peerAddr );
            }
          }
        }
        
        // initialize scan index to last device
        rcKeyfobScanIdx = rcKeyfobScanRes;

      }
      break;

    case GAP_LINK_ESTABLISHED_EVENT:
      {
        if ( pEvent->gap.hdr.status == SUCCESS )
        {          
          rcKeyfobState = BLE_STATE_CONNECTED;
          rcKeyfobConnHandle = pEvent->linkCmpl.connectionHandle;
          rcProcedureInProgress = TRUE;
          
          // If service discovery not performed initiate service discovery
          if ( throttleCharHdl == 0 || steeringCharHdl == 0 || zCharHdl == 0 )
          {
            osal_start_timerEx( rcKeyfobTaskId, START_DISCOVERY_EVT, DEFAULT_SVC_DISCOVERY_DELAY );
          }
          GAPCentralRole_StartRssi( rcKeyfobConnHandle, DEFAULT_RSSI_PERIOD );

          HalLedSet( HAL_LED_ALL, HAL_LED_MODE_OFF );
          HalLedSet( HAL_LED_1, HAL_LED_MODE_BLINK );
        }
        else
        {
          rcKeyfobState = BLE_STATE_IDLE;
          rcKeyfobConnHandle = GAP_CONNHANDLE_INIT;
          rcKeyfobDiscState = BLE_DISC_STATE_IDLE;

          osal_set_event(rcKeyfobTaskId, RC_TOGGLE_ACCEL_EVT);
          GAPCentralRole_CancelRssi( rcKeyfobConnHandle );
          
          HalLedSet( HAL_LED_ALL, HAL_LED_MODE_OFF );
          HalLedSet( HAL_LED_2, HAL_LED_MODE_BLINK );
        }
      }
      break;

    case GAP_LINK_TERMINATED_EVENT:
      {
        rcKeyfobState = BLE_STATE_IDLE;
        rcKeyfobConnHandle = GAP_CONNHANDLE_INIT;
        rcKeyfobDiscState = BLE_DISC_STATE_IDLE;
        throttleCharHdl = steeringCharHdl = zCharHdl = 0;
        rcProcedureInProgress = FALSE;
                
        osal_set_event(rcKeyfobTaskId, RC_TOGGLE_ACCEL_EVT);
        GAPCentralRole_CancelRssi( rcKeyfobConnHandle );
          
        //LCD_WRITE_STRING( "Disconnected", HAL_LCD_LINE_1 );
        //LCD_WRITE_STRING_VALUE( "Reason:", pEvent->linkTerminate.reason, 10, HAL_LCD_LINE_2 );
        HalLedSet( HAL_LED_ALL, HAL_LED_MODE_OFF ); 
        HalLedSet( HAL_LED_2, HAL_LED_MODE_BLINK );
      }
      break;

    case GAP_LINK_PARAM_UPDATE_EVENT:
      {
        
      }
      break;
      
    default:
      break;
  }
}

/** @brief   Pairing state callback.
 *
 * @return  none
 */
static void rcKeyfobPairStateCB( uint16 connHandle, uint8 state, uint8 status )
{
  if ( state == GAPBOND_PAIRING_STATE_STARTED )
  {
    //LCD_WRITE_STRING( "Pairing started", HAL_LCD_LINE_1 );
  }
  else if ( state == GAPBOND_PAIRING_STATE_COMPLETE )
  {
    if ( status == SUCCESS )
    {
      //LCD_WRITE_STRING( "Pairing success", HAL_LCD_LINE_1 );
    }
    else
    {
      //LCD_WRITE_STRING_VALUE( "Pairing fail", status, 10, HAL_LCD_LINE_1 );
    }
  }
  else if ( state == GAPBOND_PAIRING_STATE_BONDED )
  {
    if ( status == SUCCESS )
    {
      //LCD_WRITE_STRING( "Bonding success", HAL_LCD_LINE_1 );
    }
  }
}

/** @brief   Passcode callback.
 *
 * @return  none
 */
static void rcKeyfobPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs )
{
#if (HAL_LCD == TRUE)

  uint32  passcode;
  uint8   str[7];

  // Create random passcode
  LL_Rand( ((uint8 *) &passcode), sizeof( uint32 ));
  passcode %= 1000000;
  
  // Display passcode to user
  if ( uiOutputs != 0 )
  {
    LCD_WRITE_STRING( "Passcode:",  HAL_LCD_LINE_1 );
    LCD_WRITE_STRING( (char *) _ltoa(passcode, str, 10),  HAL_LCD_LINE_2 );
  }
  
  // Send passcode response
  GAPBondMgr_PasscodeRsp( connectionHandle, SUCCESS, passcode );
#endif
}

/** @brief   Start service discovery.
 *
 * @return  none
 */
static void rcKeyfobStartDiscovery( void )
{
  uint8 uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(RC_PROFILE_SERV_UUID),
                                   HI_UINT16(RC_PROFILE_SERV_UUID) };
  
  // Initialize cached handles
  rcKeyfobSvcStartHdl = rcKeyfobSvcEndHdl = throttleCharHdl = steeringCharHdl = zCharHdl = 0;

  rcKeyfobDiscState = BLE_DISC_STATE_SVC;
  
  // Discovery simple BLE service
  GATT_DiscPrimaryServiceByUUID( rcKeyfobConnHandle,
                                 uuid,
                                 ATT_BT_UUID_SIZE,
                                 rcKeyfobTaskId );
}

/** @brief   Process GATT discovery event
 *
 * @return  none
 */
static void rcKeyfobGATTDiscoveryEvent( gattMsgEvent_t *pMsg )
{
  //attReadByTypeReq_t req;
  
  if ( rcKeyfobDiscState == BLE_DISC_STATE_SVC )
  {
    // Service found, store handles
    if ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
         pMsg->msg.findByTypeValueRsp.numInfo > 0 )
    {
      rcKeyfobSvcStartHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].handle;
      rcKeyfobSvcEndHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].grpEndHandle;
    }
    
    // If procedure complete
    if ( ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP  && 
           pMsg->hdr.status == bleProcedureComplete ) ||
         ( pMsg->method == ATT_ERROR_RSP ) )
    {
      if ( rcKeyfobSvcStartHdl != 0 )
      {
        // Discover characteristic
        rcKeyfobDiscState = BLE_DISC_STATE_CHAR;
        
//        req.startHandle = rcKeyfobSvcStartHdl;
//        req.endHandle = rcKeyfobSvcEndHdl;
//        req.type.len = ATT_BT_UUID_SIZE;
//        req.type.uuid[0] = LO_UINT16(SIMPLEPROFILE_CHAR1_UUID);
//        req.type.uuid[1] = HI_UINT16(SIMPLEPROFILE_CHAR1_UUID);
//
//        GATT_ReadUsingCharUUID( rcKeyfobConnHandle, &req, rcKeyfobTaskId );
        
        GATT_DiscAllChars( rcKeyfobConnHandle,
                           rcKeyfobSvcStartHdl,
                           rcKeyfobSvcEndHdl,
                           rcKeyfobTaskId );
        
        
      }
    }
  }
  else if ( rcKeyfobDiscState == BLE_DISC_STATE_CHAR )
  {
    // Characteristic found, store handle
    if ( pMsg->method == ATT_READ_BY_TYPE_RSP && 
         pMsg->msg.readByTypeRsp.numPairs > 0 )
    {
      //TODO: do the following parsing in some slightly less stupid way
      uint8 len = pMsg->msg.readByTypeRsp.len;
      for (uint8 i = 0; i < pMsg->msg.readByTypeRsp.numPairs * len +1; i += 7)
      {
        
        uint16 foundUUID = BUILD_UINT16( pMsg->msg.readByTypeRsp.dataList[i+len-2],
                                         pMsg->msg.readByTypeRsp.dataList[i+len-1] );
        switch ( foundUUID )
        {
        case RC_THROTTLE_CHAR_UUID:
          throttleCharHdl = BUILD_UINT16( pMsg->msg.readByTypeRsp.dataList[i+len-4],
                                   pMsg->msg.readByTypeRsp.dataList[i+len-3] );
          break;
          
        case RC_STEERING_CHAR_UUID:
          steeringCharHdl = BUILD_UINT16( pMsg->msg.readByTypeRsp.dataList[i+len-4],
                                   pMsg->msg.readByTypeRsp.dataList[i+len-3] );
          break;
          
        case RC_Z_CHAR_UUID:
          zCharHdl = BUILD_UINT16( pMsg->msg.readByTypeRsp.dataList[i+len-4],
                                   pMsg->msg.readByTypeRsp.dataList[i+len-3] );
          break;  
          
        default:
          break;
        }
        
      }
      
      
    }
    if ( !( throttleCharHdl == 0 && steeringCharHdl == 0 && zCharHdl == 0 ) ) 
    {
      rcProcedureInProgress = FALSE;
      rcKeyfobDiscState = BLE_DISC_STATE_IDLE;
    }
    
  }    
}


/** @brief   Find a unit with given advertisment data.
 *
 * @return  TRUE if car is found
 */
static bool rcFindCar( uint8 *pData, uint8 dataLen )
{
  uint8 adLen;
  uint8 *pEnd;
  
  pEnd = pData + dataLen - 1;
  
  // Look for specific advertisment data of the RC car
  // While end of data not reached
  while ( pData < pEnd )
  {
    // Get length of next AD item
    adLen = *pData++;
    
    // Check for the car-specific AD-data
    if ( adLen > 0  &&
         pData[0] == GAP_ADTYPE_MANUFACTURER_SPECIFIC &&
         pData[1] == 0x000D &&    //TI Company ID
         pData[2] == 0xCA )       //It's a CAr
    {
      // Match found
      return TRUE;
    }
    else
    {
      // Go to next item
      pData += adLen;
    }
  }
  
  // Match not found
  return FALSE;
}

/** @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void rcKeyfobAddDeviceInfo( uint8 *pAddr, uint8 addrType )
{
  uint8 i;
  
  // If result count not at max
  if ( rcKeyfobScanRes < DEFAULT_MAX_SCAN_RES )
  {
    // Check if device is already in scan results
    for ( i = 0; i < rcKeyfobScanRes; i++ )
    {
      if ( osal_memcmp( pAddr, rcKeyfobDevList[i].addr , B_ADDR_LEN ) )
      {
        return;
      }
    }
    
    // Add addr to scan result list
    osal_memcpy( rcKeyfobDevList[rcKeyfobScanRes].addr, pAddr, B_ADDR_LEN );
    rcKeyfobDevList[rcKeyfobScanRes].addrType = addrType;
    
    // Increment scan result count
    rcKeyfobScanRes++;
  }
}

/** @brief   Called by the application to read accelerometer data
 *          and put data on the car server
 *
 * @param   none
 *
 * @return  none
 */
static void rcCarUpdateDirectives( void )
{
  static int8 x, y, z, throttle, steering;
  int8 new_x, new_y, new_z;
  
  (void)z;

  // Read data for each axis of the accelerometer
  accReadAcc(&new_x, &new_y, &new_z);

  // Check if x-axis value has changed by more than the threshold value
  if( (x < (new_x-ACCEL_CHANGE_THRESHOLD)) || (x > (new_x+ACCEL_CHANGE_THRESHOLD)) )
  {
    x = -new_x;
    
    throttle = 0;
    
    // Check if position is outside dead zone
    if ( x > ACCEL_X_DEADZONE )
    {
      throttle = x - ACCEL_X_DEADZONE;
    }
    else if ( x < -ACCEL_X_DEADZONE )
    {
      throttle = x + ACCEL_X_DEADZONE;
    }
   
    if ( rcKeyfobState == BLE_STATE_CONNECTED &&
              throttleCharHdl != 0 &&
              rcProcedureInProgress == FALSE )
    {
      //Send write command
      rcCarWrite(throttle, throttleCharHdl);
    }
  }
  
  // Check if y-axis value has changed by more than the threshold value and
  // send it to the server
  if( (y < (new_y-ACCEL_CHANGE_THRESHOLD)) || (y > (new_y+ACCEL_CHANGE_THRESHOLD)) )
  {
    y = new_y;
    
    steering = 0;
    
    // Check if position is outside dead zone
    if ( y > ACCEL_Y_DEADZONE )
    {
      steering = y - ACCEL_Y_DEADZONE;
    }
    else if ( y < -ACCEL_Y_DEADZONE )
    {
      steering = y + ACCEL_Y_DEADZONE;
    }
    
    if ( rcKeyfobState == BLE_STATE_CONNECTED &&
              steeringCharHdl != 0 &&
              rcProcedureInProgress == FALSE )
    {
      // Send write command
      rcCarWrite(steering, steeringCharHdl);
    }
  }
  
  
  // Check if z-axis value has changed by more than the threshold value and
  // send it to the server
  /*
  if( (z < (new_z-ACCEL_CHANGE_THRESHOLD)) || (z > (new_z+ACCEL_CHANGE_THRESHOLD)) )
  {
    z = new_z;
    
    if ( rcKeyfobState == BLE_STATE_CONNECTED &&
              zCharHdl != 0 &&
              rcProcedureInProgress == FALSE )
    {
      rcCarWrite(z, zCharHdl);

    }
  }
  */
}

/** @brief   Called by the application to send write commands to the car
 *
 * @param   val - Value to write
 * @param   handle - Connection handle to write to
 *
 * @return  none
 */
static void rcCarWrite(int8 val, uint16 handle)
{
  // Send write command
  attWriteReq_t req;
  req.len = 1;
  req.value[0] = val;
  req.sig = 0;
  req.cmd = 1;
  req.handle = handle;
  GATT_WriteNoRsp(rcKeyfobConnHandle, &req);
}

/** @brief  Writes zero to both steering and throttle
 *
 * @param   none
 *
 * @return  none
 */
static void rcCarStop()
{
  rcCarWrite(0, throttleCharHdl);
  rcCarWrite(0, steeringCharHdl);
//  rcCarWrite(0, zCharHdl); 
}
/*********************************************************************
*********************************************************************/

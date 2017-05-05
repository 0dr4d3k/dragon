/*******************************************************************************
  Filename:       DragonFly_Gate.c
  Revised:        $Date: 2017-04-17 07:07:07$
  Revision:       $Revision: 1 $

  Description:    This file contains the Dragonfly Gate sample application
                  definitions and prototypes. DragonFly Gate is the communication
                  path with the real world.
*******************************************************************************/

/*******************************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"

#include "gatt.h"

#include "hci.h"

#include "gattservapp.h"

#include "DragonFly_Gate.h"
#include "gateservice.h"

#if defined (SERIAL_INTERFACE)
#include "serialInterface.h"
#endif

#include "stdio.h"


/*******************************************************************************
 * MACROS
 */


/*******************************************************************************
 * CONSTANTS
 */
// How often to perform periodic event
#define DRAGONFLY_GATE_PERIODIC_EVT_PERIOD                   5000

#define INVALID_CONNHANDLE                                   0xFFFF


/*******************************************************************************
 * TYPEDEFS
 */


/*******************************************************************************
 * GLOBAL VARIABLES
 */

/*******************************************************************************
 * EXTERNAL VARIABLES
 */

/*******************************************************************************
 * EXTERNAL FUNCTIONS
 */

/*******************************************************************************
 * LOCAL VARIABLES
 */
static uint8 dragonflyGate_TaskID;   // Task ID for internal task/event processing

/*******************************************************************************
 * LOCAL FUNCTIONS
 */
static void dragonflyGate_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void dragonflyGate_ProcessGATTMsg( gattMsgEvent_t *pMsg );
static void performPeriodicTask( void );
static void dragonflyChangeCB( uint8 paramID );


/*******************************************************************************
 * PROFILE CALLBACKS
 */
static gateCBs_t DragonFly_GateCBs =
{
  dragonflyChangeCB,            // Characteristic value change callback
};


/*******************************************************************************
 * PUBLIC FUNCTIONS
 */
/*******************************************************************************
 * @fn      DragonFly_Gate_Init
 *
 * @brief   Initialization function for the Simple BLE Peripheral App Task.
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
void DragonFly_Gate_Init(uint8 task_id)
{
  dragonflyGate_TaskID = task_id;

  // Initialize GATT attributes
  Gate_AddService();  
  
  // Register callback with SimpleGATTprofile
  VOID Gate_RegisterAppCBs(&DragonFly_GateCBs);
  
  // Setup the SimpleProfile Characteristic Values
  {
    uint8 gateData[GATE_DATA_LEN] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    Gate_SetParameter(SENSOR_DATA, GATE_DATA_LEN, gateData);
  }

  // Setup a delayed profile startup
  osal_set_event(dragonflyGate_TaskID, DRAGONFLY_GATE_STAR_EVT);
}


/*******************************************************************************
 * @fn      SimpleBLEPeripheral_ProcessEvent
 *
 * @brief   Simple BLE Peripheral Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 DragonFly_Gate_ProcessEvent(uint8 task_id, uint16 events)
{

  VOID task_id; // OSAL required parameter that isn't used in this function

  if(events & SYS_EVENT_MSG)
  {
    uint8 *pMsg;

    if((pMsg = osal_msg_receive(dragonflyGate_TaskID)) != NULL)
    {
      dragonflyGate_ProcessOSALMsg((osal_event_hdr_t *)pMsg);

      // Release the OSAL message
      VOID osal_msg_deallocate(pMsg);
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if (events & DRAGONFLY_GATE_STAR_EVT)
  {
    // Set timer for first periodic event
    osal_start_timerEx(dragonflyGate_TaskID, 
                       DRAGONFLY_GATE_PERIODIC_EVT, 
                       DRAGONFLY_GATE_PERIODIC_EVT_PERIOD);

    return (events ^ DRAGONFLY_GATE_STAR_EVT);
  }

  if (events & DRAGONFLY_GATE_PERIODIC_EVT)
  {
    // Restart timer
    if (DRAGONFLY_GATE_PERIODIC_EVT_PERIOD)
    {
      osal_start_timerEx(dragonflyGate_TaskID, 
                         DRAGONFLY_GATE_PERIODIC_EVT, 
                         DRAGONFLY_GATE_PERIODIC_EVT_PERIOD);
    }
    
    // Perform periodic application task
    performPeriodicTask();

    return (events ^ DRAGONFLY_GATE_PERIODIC_EVT);
  }

  // Discard unknown events
  return 0;
}


/*******************************************************************************
 * @fn      dragonflyGate_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void dragonflyGate_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {     
    case KEY_CHANGE:
      break;
 
    case GATT_MSG_EVENT:
      // Process GATT message
      dragonflyGate_ProcessGATTMsg( (gattMsgEvent_t *)pMsg );
      break;
      
    default:
      // do nothing
      break;
  }
}


/*******************************************************************************
 * @fn      simpleBLEPeripheral_ProcessGATTMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
static void dragonflyGate_ProcessGATTMsg( gattMsgEvent_t *pMsg )
{  
  // deallocate GATT message
  GATT_bm_free( &pMsg->msg, pMsg->method );
}


/*******************************************************************************
 * @fn      performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets
 *          called every five seconds as a result of the SBP_PERIODIC_EVT
 *          OSAL event. In this example, the value of the third
 *          characteristic in the SimpleGATTProfile service is retrieved
 *          from the profile, and then copied into the value of the
 *          the fourth characteristic.
 *
 * @param   none
 *
 * @return  none
 */
static void performPeriodicTask( void )
{
  static uint8 newValue[GATE_DATA_LEN];
      Gate_GetParameter(SENSOR_DATA, newValue);
      newValue[0]++;
      Gate_SetParameter(SENSOR_DATA,GATE_DATA_LEN, newValue);
}


/*******************************************************************************
 * @fn      simpleProfileChangeCB
 *
 * @brief   Callback from SimpleBLEProfile indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void dragonflyChangeCB( uint8 paramID )
{
//  uint8 newValue[GATE_DATA_LEN];

  switch( paramID )
  {
    case SENSOR_DATA:
//      Gate_GetParameter(SENSOR_DATA, newValue);
      break;

    default:
      // should not reach here!
      break;
  }
}


/*******************************************************************************
*******************************************************************************/
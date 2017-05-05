/*******************************************************************************
  Filename:       DragonFly_Oad.c
  Revised:        $Date: 2017-04-17 07:07:07$
  Revision:       $Revision: 1 $

  Description:    This file contains the Dragonfly Oad sample application
                  definitions and prototypes. 
*******************************************************************************/


/*******************************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"

#include "gatt.h"

#include "gattservapp.h"

#include "DragonFly_Oad.h"

// System Services
#include "ccservice.h"       //UUID:0xCCC0
#include "oad.h"             //UUID:0xFFC0
#include "oad_target.h"   

#include "peripheral.h" //oJo, OAD->NetCore Parameters update a lo mejor tiene que estar el NetCore
 

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


/*******************************************************************************
 * EXTERNAL VARIABLES
 */


/*******************************************************************************
 * EXTERNAL FUNCTIONS
 */


/*******************************************************************************
 * LOCAL VARIABLES
 */
static uint8 dragonflyOad_TaskID;   // Task ID for internal task/event processing


/*******************************************************************************
 * LOCAL FUNCTIONS
 */
// called if change in connection control characteristic values
static void dragonFlyOad_ccCB(uint8 paramID);

// called if change in connection control parameters by any
static void dragonFlyOad_ccUpCB(uint16 connInterval, 
                                uint16 connSlaveLatency,
                                uint16 connTimeout);

// called when change in GAP role state
//static void dragonFlyOad_RoleCB(gaprole_States_t newState);

// called to update connection service with actual conection control parameters
static void dragonFlyOad_ccUp( void );


/*******************************************************************************
 * PROFILE CALLBACKS
 */
static ccCBs_t dragonFlyOad_ccCBs =
{
  dragonFlyOad_ccCB,            // Charactersitic value change callback
};

static gapRolesParamUpdateCB_t dragonFlyOad_ccUpCBs =
{
  dragonFlyOad_ccUpCB,          // Actualize Charactersitic value
};

// GAP Role Callbacks
//static gapPeripheralRoleCBs_t dragonFlyOad_RoleCBs =
//{
//  dragonFlyOad_RoleCB,      // Profile State Change Callbacks
//  NULL                      // When a valid RSSI is read from controller (not used by application)
//};


/*******************************************************************************
 * PUBLIC FUNCTIONS
 */


/*******************************************************************************
 * @fn      DragonFly_Oad_Init
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
void DragonFly_Oad_Init(uint8 task_id)
{
  dragonflyOad_TaskID = task_id;

/*... register applications ..................................................*/  
  // register callbacks for connection control service
  VOID CcService_RegisterAppCBs(&dragonFlyOad_ccCBs);

  // register callbacks for connection parameters update
/* this function is called when any internal or external change the connection*/
/* parameters. ccService update CHAR1 with actual data.                       */
  VOID GAPPeripheralRole_RegisterAppCBs(&dragonFlyOad_ccUpCBs); 
  
/*... GATT services ..........................................................*/  
// initialize services
  CcService_AddService();                         // Connection Control Service

  VOID OADTarget_AddService();                    // OAD Profile
    
/*............................................................................*/  
  // Setup a delayed profile startup
  osal_set_event(dragonflyOad_TaskID, DRAGONFLY_OAD_STAR_EVT);
}


/*******************************************************************************
 * @fn      DragonFly_Oad_ProcessEvent
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
uint16 DragonFly_Oad_ProcessEvent(uint8 task_id, uint16 events)
{

  VOID task_id; // OSAL required parameter that isn't used in this function

/*............................................................................*/
  if(events & SYS_EVENT_MSG)
  {
    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  
/*.........................................................start the device...*/
  if (events & DRAGONFLY_OAD_STAR_EVT)
  {
    // register GAP Role callbacks
//    VOID GAPPeripheralRole_StartDevice(&dragonFlyOad_RoleCBs);

    // return unprocessed events
    return (events ^ DRAGONFLY_OAD_STAR_EVT);
  }
    
/*..................................................................unknow....*/
  // Discard unknown events
  return 0;
}


/*******************************************************************************
 * @fn      dragonFlyOad_RoleCB
 *
 * @brief   Callback from GAP Role indicating change in role state
 *
 * @param   newState - new state
 *
 * @return  none
 */
//static void dragonFlyOad_RoleCB(gaprole_States_t newState)
//{
//  switch ( newState )
//  {
//    case GAPROLE_STARTED:
//    break;
//
//    case GAPROLE_ADVERTISING:
//    break;
//
//    case GAPROLE_ADVERTISING_NONCONN:
//      break;
//      
//    case GAPROLE_CONNECTED:
//      dragonFlyOad_ccUp(); //<- oJo, Para que hacemos esto?
//      break;
//
//    case GAPROLE_CONNECTED_ADV:
//      break;
//      
//    case GAPROLE_WAITING:
//      break;
//
//    case GAPROLE_WAITING_AFTER_TIMEOUT:
//    break;
//  }
//}


/**********************************************************************
 * @fn      dragonFlyOad_ccUp
 *
 * @brief   Update the Connection Control service with the current connection
 *          control settings
 *
 */
extern void dragonFlyOad_ccUp( void )
{
  uint8 buf[CCSERVICE_CHAR1_LEN];
  uint16 connInterval;
  uint16 connSlaveLatency;
  uint16 connTimeout; 
  
  // Get the connection control data
  GAPPeripheralRole_GetParameter(GAPROLE_CONN_INTERVAL, &connInterval);
  GAPPeripheralRole_GetParameter(GAPROLE_SLAVE_LATENCY, &connSlaveLatency);
  GAPPeripheralRole_GetParameter(GAPROLE_CONN_TIMEOUT, &connTimeout);

  buf[0] = LO_UINT16(connInterval);
  buf[1] = HI_UINT16(connInterval);
  buf[2] = LO_UINT16(connSlaveLatency);
  buf[3] = HI_UINT16(connSlaveLatency);
  buf[4] = LO_UINT16(connTimeout);
  buf[5] = HI_UINT16(connTimeout);

  CcService_SetParameter(CCSERVICE_CHAR1,sizeof(buf),buf);
}


/*********************************************************************
 * @fn      dragonFlyOad_ccCB
 *
 * @brief   Callback from Connection Control indicating a value change
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  none
 */
static void dragonFlyOad_ccCB( uint8 paramID )
{
  // CCSERVICE_CHAR1: read & notify only

  // CCSERVICE_CHAR: requested connection parameters
  if( paramID == CCSERVICE_CHAR2 )
  {
    uint8 buf[CCSERVICE_CHAR2_LEN];
    uint16 minConnInterval;
    uint16 maxConnInterval;
    uint16 slaveLatency;
    uint16 timeoutMultiplier;

    CcService_GetParameter( CCSERVICE_CHAR2, buf );

    minConnInterval = BUILD_UINT16(buf[0],buf[1]);
    maxConnInterval = BUILD_UINT16(buf[2],buf[3]);
    slaveLatency = BUILD_UINT16(buf[4],buf[5]);
    timeoutMultiplier = BUILD_UINT16(buf[6],buf[7]);

    // Update connection parameters
    GAPPeriperalRole_SendUpdateParam(minConnInterval, 
                                     maxConnInterval, 
                                     slaveLatency, 
                                     timeoutMultiplier, 
                                     GAPROLE_NO_ACTION);
  }

  // CCSERVICE_CHAR3: Disconnect request
  if( paramID == CCSERVICE_CHAR3 )
  {
    // Any change in the value will terminate the connection
    GAPPeripheralRole_TerminateConnection();
  }
}


/*********************************************************************
 * @fn      dragonFlyOad_ccUpCB
 *
 * @brief   Called when connection parameters are updates
 *
 * @param   connInterval - new connection interval
 *
 * @param   connSlaveLatency - new slave latency
 *
 * @param   connTimeout - new connection timeout
 *
 * @return  none
*/
static void dragonFlyOad_ccUpCB( uint16 connInterval, uint16 connSlaveLatency,
                                  uint16 connTimeout )
{
  uint8 buf[CCSERVICE_CHAR1_LEN];

  buf[0] = LO_UINT16(connInterval);
  buf[1] = HI_UINT16(connInterval);
  buf[2] = LO_UINT16(connSlaveLatency);
  buf[3] = HI_UINT16(connSlaveLatency);
  buf[4] = LO_UINT16(connTimeout);
  buf[5] = HI_UINT16(connTimeout);
  CcService_SetParameter(CCSERVICE_CHAR1,sizeof(buf),buf);
}


/*******************************************************************************
*******************************************************************************/


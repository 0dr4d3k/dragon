/*******************************************************************************
  Filename:       DragonFly_Net_Core.c
  Revised:        $Date: 2017-04-17 07:07:07$
  Revision:       $Revision: 1 $

  Description:    This file contains the Dragonfly Net_Core sample application
                  definitions and prototypes. DragonFly Net_Core is the communication
                  path with the real world.

Notas: echale un vistazo a la aplicacion Central-MultiSlave
*******************************************************************************/


/*******************************************************************************
 * INCLUDES
 */
#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"

#include "gap.h" //<- gap role added

//#if defined (GAP_ROLE_PERIPHERAL)
  #include "peripheral.h" //<- gap role added oJo, peripheral contiene ojetos comunes a central y peripheral. Habria que llevarlos a common_role
//#endif

#if defined (GAP_ROLE_CENTRAL)
  #include "central.h" //<- gap role added
  //#include "eventservice.h" 
#endif

#include "gatt.h"

#include "hci.h"

#include "gattservapp.h"

#include "DragonFly_Net_Core.h"

#if defined FEATURE_INFO
  #include "devinfoservice-df.h"
#endif   
   
#if defined (SERIAL_INTERFACE)
#include "serialInterface.h"
#endif

#include "hal_led.h"
#include "hal_buzzer.h"
#include "hal_uart.h" // <- for debug issues

#include "DragonFly.h" // oJo cambiar el nombre por DragonFly_Sensor
#include "DragonFly_Oad.h" 


/*******************************************************************************
 * MACROS
 */


/*******************************************************************************
 * CONSTANTS
 */
// How often to perform periodic event
#define DRAGONFLY_NETCORE_PERIODIC_EVT_PERIOD    2000
#define DRAGONFLY_NETCORE_KERNEL_EVT_PERIOD      1000

// What is the advertising interval when device is discoverable (units of 625us, 320=200ms)
#define DEFAULT_ADVERTISING_INTERVAL             320

// General discoverable mode advertises indefinitely
#if defined ADVERTISING_ALLTIME
  #define DEFAULT_DISCOVERABLE_MODE              GAP_ADTYPE_FLAGS_GENERAL
#else
  #define DEFAULT_DISCOVERABLE_MODE              GAP_ADTYPE_FLAGS_LIMITED
#endif
//Maximun time to reamain advertising, when in Limited Discoverable mode. In seconds.
#define DEFAULT_TGAP_LIM_ADV_TIMEOUT             14

// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL        80

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL        800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY            0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT             1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST            FALSE
//#define DEFAULT_ENABLE_UPDATE_REQUEST            TRUE

// Minimum time upon connection establishment before starts a connection update. In seconds
#define DEFAULT_CONN_PAUSE_PERIPHERAL            3

#if defined ( PLUS_BROADCASTER )
  #define ADV_IN_CONN_WAIT                       10000 //  500 ms
#endif

#if defined (GAP_ROLE_CENTRAL)
  // Central parameters
  // Company Identifier: Texas Instruments Inc. (13)
  //#define TI_COMPANY_ID                         0x000D

  //#define INVALID_CONNHANDLE                    0xFFFF

  // Length of bd addr as a string
  #define B_ADDR_STR_LEN                        15

  #define MAX_CONNECTIONS                       3

  // Maximum number of scan responses
  #define DEFAULT_MAX_SCAN_RES                  8

  // Scan duration in ms
  #define DEFAULT_SCAN_DURATION                 4000 // <- oJo, estudialo

  // Discovey mode (limited, general, all)
  #define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL // <- oJo, estudialo

  // TRUE to use active scan
  #define DEFAULT_DISCOVERY_ACTIVE_SCAN         TRUE   // <- oJo, estudialo

  // TRUE to use white list during discovery
  #define DEFAULT_DISCOVERY_WHITE_LIST          FALSE // <- oJo, estudialo

  // TRUE to use high scan duty cycle when creating link
  #define DEFAULT_LINK_HIGH_DUTY_CYCLE          FALSE

  // TRUE to use white list when creating link
  #define DEFAULT_LINK_WHITE_LIST               FALSE 

  // Default RSSI polling period in ms
  #define DEFAULT_RSSI_PERIOD                   1000

  // Default passcode
  #define DEFAULT_PASSCODE                      19655

  // Default GAP pairing mode
  #define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

  // Default MITM mode (TRUE to require passcode or OOB when pairing)
  #define DEFAULT_MITM_MODE                     FALSE

  // Default bonding mode, TRUE to bond
  //#define DEFAULT_BONDING_MODE                  TRUE
  #define DEFAULT_BONDING_MODE                  FALSE

  // Default GAP bonding I/O capabilities
  #define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_DISPLAY_ONLY

  // Default service discovery timer delay in ms
  #define DEFAULT_SVC_DISCOVERY_DELAY           1000

  // TRUE to filter discovery results on desired service UUID
  #define DEFAULT_DEV_DISC_BY_SVC_UUID          TRUE

#endif //(GAP_ROLE_CENTRAL)

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

// HAL Related Callback functions
#if (defined HAL_BUZZER) && (HAL_BUZZER == TRUE)
  extern void BuzzerRing( uint16 timeout, uint8 tone );
  extern void BuzzerCompleteCB( void );
#endif
  
/*******************************************************************************
 * LOCAL VARIABLES
 */
static uint8 dragonflyNetCore_TaskID;   // Task ID for internal task/event processing

extern gaprole_States_t gapProfileState = GAPROLE_INIT; // <- oJo, definir como global NET->ALL

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8 scanRspData[] =
{
  // complete name
  0x0c,   // length of this data (excluding length byte)
  GAP_ADTYPE_LOCAL_NAME_COMPLETE,
  '-', 'D', 'r', 'a', 'g', 'o', 'n', 'F', 'l', 'y',  '-',

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
  0       // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8 advertData[] =
{
  // Flags; this sets the device to use limited discoverable
  // mode (advertises for 30 seconds at a time) instead of general
  // discoverable mode (advertises indefinitely)
  0x02,   // length of this data
  GAP_ADTYPE_FLAGS,
  DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
};

// GAP GATT Attributes
#if defined FEATURE_GATTINFO
  static uint8 attDeviceName[GAP_DEVICE_NAME_LEN] = "DragonFly_v0_04";
#endif

/*******************************************************************************
 * CENTRAL DEFINITIONS
 */

#if defined (GAP_ROLE_PERIPHERAL) || defined (GAP_ROLE_CENTRAL)  //<-oJo, sobra?
  // Number of init counts, a flag to determine action on role change
  static bool initCount = 0;

  // Some initialization only do once
  static bool initDone = FALSE;
#endif

#if defined (GAP_ROLE_CENTRAL)
  // Device list handle
  typedef struct
  {
    uint8 state;
    uint8 connHandle;
    uint8 addr[B_ADDR_LEN]; 
  } deviceList_t;

  static deviceList_t deviceList[3];

  static uint8 currentDevice = 0;

  static void simpleBLECentralRssiCB( uint16 connHandle, int8  rssi );
  static uint8 simpleBLECentralEventCB( gapCentralRoleEvent_t *pEvent );

  // Periodic Task
  static void periodicCentralTask(void); 

  // Application state - initial state is advertising
  static uint8 masterSlave_State = BLE_STATE_CONNECTING;

  // Initial State
  static gaprole_Modes_t actualRole;
  static gaprole_Modes_t requestRole = GAPROLE_CENTRAL;
  
  // Flags
  static uint8 step = 0;
#endif

#if  defined (GAP_ROLE_PERIPHERAL) && !defined (GAP_ROLE_CENTRAL)
  // Application state - initial state is advertising
  static uint8 masterSlave_State = BLE_STATE_ADVERTISING;

  // Initial State
  static gaprole_Modes_t actualRole;  
  static gaprole_Modes_t requestRole = GAPROLE_PERIPHERAL; 

  // Periodic Task
  static void periodicPeripheralTask(void); 
#endif
  
/*******************************************************************************
 * LOCAL FUNCTIONS
 */
static void dragonflyNetCore_ProcessOSALMsg(osal_event_hdr_t *pMsg);
static void dragonflyNetCore_ProcessGATTMsg(gattMsgEvent_t *pMsg);

static void performPeriodicTask(void);

static void dragonflyNetCore_RoleCB(gaprole_States_t newState);

static void dragonflyNetCore_Kernel(void);

static void handleError(uint8 error, uint8 status);

/*******************************************************************************
 * PROFILE CALLBACKS
 */
#if defined (GAP_ROLE_PERIPHERAL)
  // GAP Peripheral Role Callbacks
  static gapPeripheralRoleCBs_t dragonflyNetCore_PeripheralRoleCBs =
  {
    dragonflyNetCore_RoleCB,  // Profile State Change Callbacks
    NULL                      // When a valid RSSI is read from controller (not used by application)
  };
#endif

#if defined (GAP_ROLE_CENTRAL)
  // GAP Central Role Callbacks
  static const gapCentralRoleCBs_t dragonflyNetCore_CentralRoleCBs =
  {
    simpleBLECentralRssiCB,       // RSSI callback
    simpleBLECentralEventCB       // Event callback
  };
#endif

#if defined GAP_BOND
  // GAP Bond Manager Callbacks
  static gapBondCBs_t dragonflyNetCore_BondMgrCBs =
  {
    NULL,                     // Passcode callback (not used by application)
    NULL                      // Pairing / Bonding state Callback (not used by application)
  };
#endif


/*******************************************************************************
 * PUBLIC FUNCTIONS
 */
/*******************************************************************************
 * @fn      DragonFly_Net_Core_Init
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
void DragonFly_Net_Core_Init(uint8 task_id)
{
  dragonflyNetCore_TaskID = task_id;

  // set my own external bluetooth address, oJo bajo test   
  #ifdef HAL_BOARD_DRAGONFLY_v8B_GATEWAY
    static uint8 bdAddress[6] = {0xff,0xff,0xff,0xff,0xff,HAL_BOARD_DRAGONFLY_v8B_GATEWAY};
  #endif
    
  #ifdef HAL_BOARD_DRAGONFLY_v8B_ACTUATOR
    static uint8 bdAddress[6] = {0xff,0xff,0xff,0xff,0xff,HAL_BOARD_DRAGONFLY_v8B_ACTUATOR}; 
  #endif
    
  #ifdef HAL_BOARD_DRAGONFLY_v8B_THERMOSTAT
    static uint8 bdAddress[6] = {0xff,0xff,0xff,0xff,0xff,HAL_BOARD_DRAGONFLY_v8B_THERMOSTAT}; 
  #endif

  #ifdef HAL_BOARD_DRAGONFLY_v8B_PLANTS
    static uint8 bdAddress[6] = {0xff,0xff,0xff,0xff,0xff,HAL_BOARD_DRAGONFLY_v8B_PLANTS}; 
  #endif

  #ifdef HAL_BOARD_DRAGONFLY_v8B_LIGHT
    static uint8 bdAddress[6] = {0xff,0xff,0xff,0xff,0xff,HAL_BOARD_DRAGONFLY_v8B_LIGHT}; 
  #endif

  #ifdef HAL_BOARD_DRAGONFLY_v8B_AIR
    static uint8 bdAddress[6] = {0xff,0xff,0xff,0xff,0xff,HAL_BOARD_DRAGONFLY_v8B_AIR}; 
  #endif
 
  #ifdef HAL_BOARD_DRAGONFLY_MINIMAL
    static uint8 bdAddress[6] = {0xff,0xff,0xff,0xff,0xff,HAL_BOARD_DRAGONFLY_MINIMAL}; 
  #endif
    
  HCI_EXT_SetBDADDRCmd(bdAddress);  
   
/*... GAP initialization .....................................................*/  
  // Setup the GAP
  // Minimum time upon connection establishment before the peripheral starts a 
  // connection update procedure. In seconds.
  VOID GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);

  // Setup the GAP Peripheral Role Profile
  {
    // Device starts advertising upon initialization
    #if defined ( AUTO_ADVERTISING )
      uint8 initial_advertising_enable = TRUE; 
    #else
      uint8 initial_advertising_enable = FALSE;
    #endif

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
    GAPPeripheralRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &initial_advertising_enable );
    GAPPeripheralRole_SetParameter( GAPROLE_ADVERT_OFF_TIME, sizeof( uint16 ), &gapRole_AdvertOffTime );

    GAPPeripheralRole_SetParameter( GAPROLE_SCAN_RSP_DATA, sizeof ( scanRspData ), scanRspData );
    GAPPeripheralRole_SetParameter( GAPROLE_ADVERT_DATA, sizeof( advertData ), advertData );

    GAPPeripheralRole_SetParameter( GAPROLE_PARAM_UPDATE_ENABLE, sizeof( uint8 ), &enable_update_request );
    GAPPeripheralRole_SetParameter( GAPROLE_MIN_CONN_INTERVAL, sizeof( uint16 ), &desired_min_interval );
    GAPPeripheralRole_SetParameter( GAPROLE_MAX_CONN_INTERVAL, sizeof( uint16 ), &desired_max_interval );
    GAPPeripheralRole_SetParameter( GAPROLE_SLAVE_LATENCY, sizeof( uint16 ), &desired_slave_latency );
    GAPPeripheralRole_SetParameter( GAPROLE_TIMEOUT_MULTIPLIER, sizeof( uint16 ), &desired_conn_timeout );
  }

#if defined FEATURE_GATTINFO
  // Set the GAP Characteristics
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, sizeof(attDeviceName), attDeviceName );
#endif
  
  // Set advertising interval
  {
    uint16 advInt = DEFAULT_ADVERTISING_INTERVAL;

    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_LIM_DISC_ADV_INT_MAX, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MIN, advInt );
    GAP_SetParamValue( TGAP_GEN_DISC_ADV_INT_MAX, advInt );
  }

  //Set time to remain advertising
  {
    uint16 default_tgap_lim_timeout = DEFAULT_TGAP_LIM_ADV_TIMEOUT;

    GAP_SetParamValue( TGAP_LIM_ADV_TIMEOUT, default_tgap_lim_timeout );
  }

  // Setup the GAP Bond Manager
#if defined GAP_BOND
  {
    uint32 passkey = 444444; // passkey "000000"
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
#endif
    
/*............................................................................*/  
#if defined (GAP_ROLE_PERIPHERAL)
  // Peripheral setup
  // Initialize GATT Server
  VOID GATT_InitServer();  // <- oJo, lo he puesto yo, comprueba que no esta en otro sitio
#endif
  
#if defined (GAP_ROLE_CENTRAL)
  // Central setup
  // Initialize GATT Client
  VOID GATT_InitClient();
  
  // Register to receive incoming ATT Indications/Notifications
//  GATT_RegisterForInd(dragonflyNetCore_TaskID); // <- mejor llevar a gate
  
  // CENTRAL APP STARTUP  
  //devices the central will try and connect to
  uint8 addr1[6] ={0xFF,0xFF,0xFF,0xFF,0xFF,0x05}; // light
  uint8 addr2[6] ={0xFF,0xFF,0xFF,0xFF,0xFF,0x02}; // actuator
  uint8 addr3[6] ={0xFF,0xFF,0xFF,0xFF,0xFF,0x06}; // air
  
  //uint8 addr3[6] ={0xFF,0xFF,0xFF,0xFF,0xFF,0x00}; //MASTER FOB
  
  VOID osal_memcpy( deviceList[0].addr, addr1, 6 );
  VOID osal_memcpy( deviceList[1].addr, addr2, 6 );
  VOID osal_memcpy( deviceList[2].addr, addr3, 6 );
  
  deviceList[0].state = BLE_STATE_IDLE;
  deviceList[1].state = BLE_STATE_IDLE;
  deviceList[2].state = BLE_STATE_IDLE;
  
  deviceList[0].connHandle = 1;         
  deviceList[1].connHandle = 2;
  deviceList[2].connHandle = 3;    

  // initialize role
#endif  

/*............................................................................*/  
  // Enable clock divide on halt
  // This reduces active current while radio is active and CC254x MCU
  // is halted
//  HCI_EXT_ClkDivOnHaltCmd(HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT);
  HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_DISABLE_CLK_DIVIDE_ON_HALT );

  // Setup a delayed profile startup
  osal_set_event(dragonflyNetCore_TaskID, DRAGONFLY_NETCORE_STAR_EVT);  
}


/*******************************************************************************
 * @fn      DragonFly_Net_Core_ProcessEvent
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
uint16 DragonFly_Net_Core_ProcessEvent(uint8 task_id, uint16 events)
{

  VOID task_id; // OSAL required parameter that isn't used in this function

/*............................................................................*/
  if(events & SYS_EVENT_MSG)
  {
    //oJo, aqui es donde proceso un menje que llega desde L2CAP:
    
    uint8 *pMsg;

    if((pMsg = osal_msg_receive(dragonflyNetCore_TaskID)) != NULL)
    {
      dragonflyNetCore_ProcessOSALMsg((osal_event_hdr_t *)pMsg);

      // Release the OSAL message
      VOID osal_msg_deallocate(pMsg);
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  
/*.........................................................start the device...*/
  if (events & DRAGONFLY_NETCORE_STAR_EVT)
  {
  //oJo solo para debug
  HalUARTPrintf("--------------------- DragonFly_Net_Core_ProcessEvent --\r\n");  
  HalUARTPrintf("--------------------- DRAGONFLY_NETCORE_STAR_EVT -------\r\n");  

    // Init device role 
    #if defined(GAP_ROLE_CENTRAL) // CENTRAL or CENTRAL + PERIPHERAL
    {
      if((requestRole == GAPROLE_CENTRAL) && (actualRole != GAPROLE_CENTRAL))
      {
/*
        // Start the Device
        if (GAPCentralRole_StartDevice((gapCentralRoleCBs_t *) &dragonflyNetCore_CentralRoleCBs ) != SUCCESS) 
        {
           // error
           HalUARTPrintf("ERROR statrt central role \r\n");
           // if error, do no actualize acutal Role
        }
        else
        {

          // initiate the first request to avoid error in first loop
          bStatus_t status = SUCCESS;
          status = GAPCentralRole_EstablishLink( DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                    DEFAULT_LINK_WHITE_LIST,
                                    ADDRTYPE_PUBLIC, deviceList[currentDevice].addr );

          if (status == SUCCESS) 
          {
            HalUARTPrintf("OK start central role. ");
            deviceList[currentDevice].state = BLE_STATE_CONNECTING;
          }
          else
            HalUARTPrintf("ERROR start central role, status: 0x02%x. ", status);

          // Always connecting or connected when central
          masterSlave_State = BLE_STATE_CONNECTING;
          // actualize role
          actualRole = requestRole;
        }
*/
          actualRole = requestRole; //oJo, lo pongo para testear la nueva maquina
      }
      else if((requestRole == GAPROLE_PERIPHERAL) && (actualRole != GAPROLE_PERIPHERAL))
      {
        // Register GAP Role callbacks
//        VOID GAPPeripheralRole_StartDevice(&dragonflyNetCore_PeripheralRoleCBs);
        if (GAPPeripheralRole_StartDevice((gapPeripheralRoleCBs_t *) &dragonflyNetCore_PeripheralRoleCBs ) != SUCCESS) 
        {
           // error
           HalUARTPrintf("ERROR start peripheral role. ");
           // if error, do no actualize acutal Role
        }
        else
        {
           // ok
           HalUARTPrintf("OK start peripheral role. ");
          // Always connecting or connected when central
          masterSlave_State = BLE_STATE_ADVERTISING;          
          // actualize role
          actualRole = requestRole;
        }
      }
    }
    #else // PERIPHERAL only
    {   
      if((requestRole == GAPROLE_PERIPHERAL) && (actualRole != GAPROLE_PERIPHERAL))
      {
        // Register GAP Role callbacks
//        VOID GAPPeripheralRole_StartDevice(&dragonflyNetCore_PeripheralRoleCBs);
        if (GAPPeripheralRole_StartDevice((gapPeripheralRoleCBs_t *) &dragonflyNetCore_PeripheralRoleCBs ) != SUCCESS) 
        {
           // error
           HalUARTPrintf("ERROR start peripheral role. ");
           // if error, do no actualize acutal Role
        }
        else
        {
           // ok
           HalUARTPrintf("OK start peripheral role. ");
          // Always connecting or connected when central
          masterSlave_State = BLE_STATE_ADVERTISING;          
          // actualize role
          actualRole = requestRole;
        }
      }
    }
    #endif
     
    //only first time
    if(initDone == FALSE)
    {

      // Start NetCore Moore Automate
      osal_start_timerEx(dragonflyNetCore_TaskID, 
                         DRAGONFLY_NETCORE_KERNEL_EVT, 
                         100);      
/*
      // Start first periodic event
      osal_start_timerEx(dragonflyNetCore_TaskID, 
                         DRAGONFLY_NETCORE_PERIODIC_EVT, 
                         200);
*/
      // Register GAP Bonds callbacks if enabled
      #if defined GAP_BOND
      VOID GAPBondMgr_Register(&dragonFlyNetCore_BondMgrCBs);
      #endif
      
      initDone = TRUE;
    }
    initCount ++;

    HalUARTPrintf("initCount: 0x%02x\r\n", initCount);  
    
    // return unprocessed events
    return (events ^ DRAGONFLY_NETCORE_STAR_EVT);
  }

/*..........................................................periodic event....*/
  if (events & DRAGONFLY_NETCORE_KERNEL_EVT)
  {
    // Restart timer
    if (DRAGONFLY_NETCORE_PERIODIC_EVT_PERIOD)
    {
      osal_start_timerEx(dragonflyNetCore_TaskID, 
                         DRAGONFLY_NETCORE_KERNEL_EVT, 
                         DRAGONFLY_NETCORE_PERIODIC_EVT_PERIOD);
    }
    
    // Perform periodic application task
    dragonflyNetCore_Kernel();

    // Clear event
    return (events ^ DRAGONFLY_NETCORE_KERNEL_EVT);
  }

/*..........................................................periodic event....*/
  if (events & DRAGONFLY_NETCORE_PERIODIC_EVT)
  {
    // Restart timer
    if (DRAGONFLY_NETCORE_PERIODIC_EVT_PERIOD)
    {
      osal_start_timerEx(dragonflyNetCore_TaskID, 
                         DRAGONFLY_NETCORE_PERIODIC_EVT, 
                         DRAGONFLY_NETCORE_PERIODIC_EVT_PERIOD);
    }
    
    // Perform periodic application task
    //if (gapProfileState == GAPROLE_CONNECTED) <- only if connected
      performPeriodicTask();

    #if defined (GAP_ROLE_CENTRAL)
      // Perform master slave periodic task
      periodicCentralTask();    
    #endif
    
    #if defined (GAP_ROLE_PERIPHERAL) && !defined (GAP_ROLE_CENTRAL)
      // Perform master slave periodic task
      periodicPeripheralTask();    
    #endif

    return (events ^ DRAGONFLY_NETCORE_PERIODIC_EVT);
  }

/*......................................................boardcaseter event....*/
// oJo, esto desaparecera para ser controlado por el core
#if defined ( PLUS_BROADCASTER )
  if ( events & DF_SYS_ADV_IN_CONNECTION_EVT )
  {
    uint8 turnOnAdv = TRUE;
    // Turn on advertising while in a connection
    GAPPeripheralRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &turnOnAdv );

    return (events ^ DF_SYS_ADV_IN_CONNECTION_EVT);
  }
#endif // PLUS_BROADCASTER
    
/*..................................................................unknow....*/
  // Discard unknown events
  return 0;
  }

          
          


/*******************************************************************************
 * @fn      dragonflyNetCore_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void dragonflyNetCore_ProcessOSALMsg(osal_event_hdr_t *pMsg)
{
  switch (pMsg->event)
  {     
    case KEY_CHANGE: //handle in sensor
      break;
 
    case GATT_MSG_EVENT:
      // Process GATT message
      dragonflyNetCore_ProcessGATTMsg((gattMsgEvent_t *)pMsg);
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
static void dragonflyNetCore_ProcessGATTMsg(gattMsgEvent_t *pMsg)
{  
  // deallocate GATT message
  GATT_bm_free(&pMsg->msg, pMsg->method); // <- oJo, es necesario?, no se ya con VOID osal_msg_deallocate(pMsg);
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
static void performPeriodicTask(void)
{
}


/*******************************************************************************
 * @fn      dragonflyNetCore_RoleCB
 *
 * @brief   Callback from GAP Role indicating change in role state
 *
 * @param   newState - new state
 *
 * @return  none
 */
static void dragonflyNetCore_RoleCB(gaprole_States_t newState)
{
  
#if defined PLUS_BROADCASTER //oJo, quitar esta feature; no nos sirve
  static uint8 first_conn_flag = 0;
  uint8 advertEnabled;
#endif

  switch(newState)
  {
    case GAPROLE_CONNECTED:
    {
      HalLedSet(HAL_LED_ALL, HAL_LED_MODE_OFF);     

      #ifdef PLUS_BROADCASTER
        // Only turn advertising on for this state when we first connect
        // otherwise, when we go from connected_advertising back to this state
        // we will be turning advertising back on.
        if ( first_conn_flag == 0 ) 
        {
            advertEnabled = FALSE; // Turn on Advertising

            // Disable connectable advertising.
            GAPPeripheralRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8),
                                 &advertEnabled);
            
            // Set to true for non-connectabel advertising.
            advertEnabled = TRUE;
            
            // Enable non-connectable advertising.
            GAPPeripheralRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8),
                                 &advertEnabled);
            
            first_conn_flag = 1;
        }
      #endif 

      //status 
      masterSlave_State = BLE_STATE_CONNECTED;      
    }
    break;

    case GAPROLE_STARTED:
    {
    #if defined FEATURE_INFO
      // oJo, es esto necesario si no esta activado DEVINFO feature?
      uint8 ownAddress[B_ADDR_LEN];
      uint8 systemId[DEVINFO_SYSTEM_ID_LEN];

      GAPPeripheralRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

      // use 6 bytes of device address for 8 bytes of system ID value
      systemId[0] = ownAddress[0];
      systemId[1] = ownAddress[1];
      systemId[2] = ownAddress[2];

      // set middle bytes to zero
      systemId[4] = 0x00;
      systemId[3] = 0x00;

      // shift three bytes up
      systemId[7] = ownAddress[5];
      systemId[6] = ownAddress[4];
      systemId[5] = ownAddress[3];

      DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);
    #endif
    
      if(initCount > 1)
      {
        uint8 turnOnAdv = TRUE;
        GAPPeripheralRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &turnOnAdv );
        //masterSlave_State = BLE_STATE_ADVERTISING; //<- oJo, lo pongo yo
      }
    }
    break;

    case GAPROLE_ADVERTISING:
    {
      #if (defined HAL_BUZZER) && (HAL_BUZZER == TRUE)
        BuzzerRing (100, HAL_BUZZER_HIGH_TONE); // short low 
      #endif
    }
    break;

    #if (defined PLUS_BROADCASTER)   // oJo, elimiar
    /* After a connection is dropped a device in PLUS_BROADCASTER will continue
     * sending non-connectable advertisements and shall sending this change of 
     * state to the application.  These are then disabled here so that sending 
     * connectable advertisements can resume.
     */
    case GAPROLE_ADVERTISING_NONCONN:
      {
        advertEnabled = FALSE;
      
        // Disable non-connectable advertising.
        GAPPeripheralRole_SetParameter(GAPROLE_ADV_NONCONN_ENABLED, sizeof(uint8),
                           &advertEnabled);
        
        // Reset flag for next connection.
        first_conn_flag = 0;
      }
      break;
    #endif 
      
    case GAPROLE_CONNECTED_ADV:
      {
        HalLedSet(HAL_LED_ALL, HAL_LED_MODE_OFF );
//      HalLedSet(HAL_LED_2, HAL_LED_MODE_ON );
      }
      break;
      
    case GAPROLE_WAITING:
      {
        // Link terminated intentionally: reset all sensors
        //oJo, por defecto resetea todos los sensores en waiting y a lo mejor no es necesario
        resetSensorSetup(); //oJo NET->SENSOR convertir en funcion externa en FragonFly_Sensor
        HalLedSet(HAL_LED_ALL, HAL_LED_MODE_OFF );          
      
        #ifdef PLUS_BROADCASTER                
          advertEnabled = TRUE;
      
          // Enabled connectable advertising.
          GAPPeripheralRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8),
                               &advertEnabled);
        #endif

        //status 
        masterSlave_State = BLE_STATE_ADVERTISING;      
      }
      break;

    case GAPROLE_WAITING_AFTER_TIMEOUT:
      {
        //Time Out
        HalLedSet(HAL_LED_ALL, HAL_LED_MODE_OFF );
//        HalLedSet(HAL_LED_2, HAL_LED_MODE_ON );
          
        #ifdef PLUS_BROADCASTER
          // Reset flag for next connection.
          first_conn_flag = 0;
        #endif 
        
        //status 
        masterSlave_State = BLE_STATE_ADVERTISING;      
      }
      break;
      
    case GAPROLE_ERROR:
      {
        // panic reset
        HalUARTPrintf("--------------------------------------- PANIC RESET ----\r\n");
        ST_HAL_DELAY(1250);
        HAL_SYSTEM_RESET();
      }
      break;
 
    default:
      {
        //status 
        masterSlave_State = BLE_STATE_ADVERTISING;      
      }
    break;
  }
  //oJo solo para debug
  HalUARTPrintf("----------------------------- dragonflyNetCore_RoleCB --\r\n");  
  HalUARTPrintf("gapProfileState: 0x%02x\r\n", gapProfileState);  
  HalUARTPrintf("newState: 0x%02x\r\n", newState);  
  HalUARTPrintf("masterSlave_State: 0x%02x\r\n", masterSlave_State);  
  HalUARTPrintf("initCount: 0x%02x\r\n", initCount);  

  gapProfileState = newState;
}

#if (defined(GAP_ROLE_CENTRAL) && defined(GAP_ROLE_PERIPHERAL)) ||             \
     defined(GAP_ROLE_CENTRAL)
/* @fn      simpleBLECentralRssiCB
 *
 * @brief   RSSI callback.
 *
 * @param   connHandle - connection handle
 * @param   rssi - RSSI
 *
 * @return  none
 */
static void simpleBLECentralRssiCB( uint16 connHandle, int8 rssi )
{
   //do something if desired
}

/*********************************************************************
 * @fn      simpleBLECentralEventCB
 *
 * @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static uint8 simpleBLECentralEventCB( gapCentralRoleEvent_t *pEvent )
{
  uint8 i;  
  //oJo solo para debug
  HalUARTPrintf("-> simpleBLECentralEventCB: ");  
  
  switch ( pEvent->gap.opcode )
  { 
    
  case GAP_DEVICE_INIT_DONE_EVENT:  // 0x00
    {
      HalUARTPrintf("GAP init done. "); 
    }
    break;

  case GAP_LINK_ESTABLISHED_EVENT:  // 0x05
    {
      if ( pEvent->gap.hdr.status == SUCCESS )
      {          
      for(i=0; i<MAX_CONNECTIONS;i++)
        {
          //Check against addresses we want to connect to
          if(deviceList[i].addr[5] == pEvent->linkCmpl.devAddr[5])
          {
            HalUARTPrintf("GAP link established. device:0x%02x, address: 0x%02x. ", i, deviceList[i].addr[5]); 
            deviceList[i].state = BLE_STATE_CONNECTED;
            deviceList[i].connHandle = pEvent->linkCmpl.connectionHandle;
          }
        }
      }
/*      
      // next step in connection sequence
      if (step == 0)
      {
        step++;
        osal_start_timerEx(dragonflyNetCore_TaskID, 
                           DRAGONFLY_NETCORE_PERIODIC_EVT, 
                           100);
      }
*/
      
      // debug
//      HalUARTPrintf("GAP link established\r\n"); 
//      for(i=0; i<MAX_CONNECTIONS;i++)
//      {
//         HalUARTPrintf("deviceList[%i].state: 0x%02x\r\n", i, deviceList[i].state);
//      }    
//      HalUARTPrintf("step: 0x%02x\r\n", step); 
    }
    break;
    
  case GAP_LINK_TERMINATED_EVENT:   // 0x06
    {
      for(i=0; i<MAX_CONNECTIONS;i++)
      {
        //Check against addresses we want to connect to
        if(deviceList[i].connHandle == pEvent->linkTerminate.connectionHandle)
        {
          deviceList[i].state = BLE_STATE_IDLE;
          // terminate any request pending to that handled oJo, hacer esto a ver si soluciona algo
          
          HalUARTPrintf("GAP link terminated. device:0x%02x. ", i); 
        }
      }     

      // debug
//      HalUARTPrintf("GAP link terminated\r\n"); 
//      for(i=0; i<MAX_CONNECTIONS;i++)
//      {
//         HalUARTPrintf("deviceList[%i].state: 0x%02x\r\n", i, deviceList[i].state);
//      }    
//      HalUARTPrintf("step: 0x%02x\r\n", step); 
    }
    break;
  }
  
  HalUARTPrintf("gap.opcode: 0x%02x\r\n", pEvent->gap.opcode);  
  
  return SUCCESS;
}

/*********************************************************************
 * @fn      periodicCentralTask
 *
 * @brief   Perform a periodic application task. 
 *
 * @param   none
 *
 * @return  none
 */
static void periodicCentralTask(void)
{
  volatile uint8 numDevices=1;
  uint8 i, error = 0;
  bStatus_t status;
  bool nextDevice = FALSE;
  
  //turn all LED's off
  HalLedSet(HAL_LED_ALL, HAL_LED_MODE_OFF); 

/// calculate the numbers of devices connected /////////////////////////////////  
  if(actualRole == GAPROLE_CENTRAL)
  {
    //Calc number of connections
    for(i=0;i<MAX_CONNECTIONS;i++)
    {
      if(deviceList[i].state == BLE_STATE_CONNECTED)
      {
        numDevices = numDevices + 1;
      }
    }
  }
  
/// NetCore State Machine //////////////////////////////////////////////////////  
        uint8 current_adv_enabled_status;
        uint8 new_adv_enabled_status;
  
  switch (masterSlave_State)
  {        
//......................................... connecting or connected state ....//
  case BLE_STATE_CONNECTING:
  case BLE_STATE_CONNECTED:    
    if(actualRole == GAPROLE_CENTRAL)
    {
      status = SUCCESS;
      
      if (deviceList[currentDevice].state != BLE_STATE_CONNECTED) 
      {
        if (deviceList[currentDevice].state == BLE_STATE_IDLE) 
        {
          // request connection
          if (step == 0)
          {
            // 0.- terminate initiated links request 
            status = GAPCentralRole_TerminateLink(GAP_CONNHANDLE_INIT);

            if (status != SUCCESS) 
            {
              error |= 0x01;
              if (status == 0x12)
              {
                HalUARTPrintf("MODO INCORRECTO, try to fix\r\n");
            // and then request a establish link
//            status = GAPCentralRole_EstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE,
//                                          DEFAULT_LINK_WHITE_LIST,
//                                          ADDRTYPE_PUBLIC, deviceList[currentDevice].addr);
//            status = GAPCentralRole_TerminateLink(GAP_CONNHANDLE_INIT);
            
//            status = GAPCentralRole_StartDevice(NULL);                
              }
            }
            
            
            
            
                       
            
          }
          else if (step == 1)
          {
            // 1.- wait to terminate initiated links grant

            // and then request a establish link
            status = GAPCentralRole_EstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                          DEFAULT_LINK_WHITE_LIST,
                                          ADDRTYPE_PUBLIC, deviceList[currentDevice].addr);

            if (status != SUCCESS) error |= 0x02;
            else
            {
              // connecting request intiated
//              deviceList[currentDevice].state = BLE_STATE_CONNECTING;

      for(i=0; i<MAX_CONNECTIONS;i++)
      {
        if (currentDevice == i) deviceList[i].state = BLE_STATE_CONNECTING;
        else 
          if (deviceList[i].state != BLE_STATE_CONNECTED) 
            deviceList[i].state = BLE_STATE_IDLE;
        
        
//        if (deviceList[i].state == BLE_STATE_CONNECTING)
//        {
//            // 0.- terminate initiated links request 
//            status = GAPCentralRole_TerminateLink(deviceList[i].connHandle);
//
//            if (status != SUCCESS) error |= 0x01;
//            else 
//            {
//              deviceList[i].state = BLE_STATE_IDLE;
//            }
//        }
      }


            }
            
            // reset connection step and round robbing to next device
            step = 0;
            nextDevice = TRUE;
          } 
        }
        else if (deviceList[currentDevice].state == BLE_STATE_CONNECTING)
        {
          // request connection already initiated, then next
          nextDevice = TRUE;
        }
      }
      else
      {
          nextDevice = TRUE;
      }

      // reset connection step and round robbing to next device
//      nextDevice = TRUE;
      
      // 2.- show the number of connected devices
      osal_pwrmgr_device( PWRMGR_BATTERY );
      HalLedBlink (HAL_LED_1, numDevices, 50, 500);
      osal_pwrmgr_device( PWRMGR_ALWAYS_ON );
    }                

    if(actualRole == GAPROLE_PERIPHERAL)
    {
      // green+blue when peripheral
      osal_pwrmgr_device( PWRMGR_BATTERY );
      HalLedBlink (HAL_LED_1 | HAL_LED_2 | HAL_LED_3, numDevices, 50, 500);
      osal_pwrmgr_device( PWRMGR_ALWAYS_ON );
    }                
    
    break;

    
//........................................... disconnecting or idle state ....//
  case BLE_STATE_DISCONNECTING:
  case BLE_STATE_IDLE:
    if(actualRole == GAPROLE_CENTRAL)
    {
      //are we finished disconnecting
      if(numDevices == 1)
      {  
        // set new role to peripheral           
        requestRole = GAPROLE_PERIPHERAL;
        
        masterSlave_State = BLE_STATE_ADVERTISING;
        
        //restet to peripheral and advertise
        osal_start_timerEx( dragonflyNetCore_TaskID, DRAGONFLY_NETCORE_STAR_EVT, 100 );
      }
      else  //continue to terminate links
      {
        if(deviceList[currentDevice].state == BLE_STATE_CONNECTED)
        {
          //terminate link
          status = GAPCentralRole_TerminateLink(deviceList[currentDevice].connHandle);
          
          if (status != SUCCESS) error |= 0x04;
        }
      }
      
      //red
      HalLedBlink (HAL_LED_2, 2, 10, 100);
      
      // nextDevice
      nextDevice = TRUE;
    }
    break;
    

//..................................................... advertesing state ....//
  case BLE_STATE_ADVERTISING: 
    // Check that it is in advertesing state

        // Find the current GAP advertising status
        status = GAPPeripheralRole_GetParameter( GAPROLE_ADVERT_ENABLED, &current_adv_enabled_status );
        
        if (status != SUCCESS) error |= 0x08;

        if( current_adv_enabled_status == FALSE )
        {
          new_adv_enabled_status = TRUE;
        
          // Change the GAP advertisement status to opposite of current status
          status = GAPPeripheralRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status );
          if (status != SUCCESS) error |= 0x10;
        }
    
    // HalLedBlink (2, 1, HAL_LED_DEFAULT_DUTY_CYCLE, HAL_LED_DEFAULT_FLASH_TIME);    
    HalLedBlink(HAL_LED_3, 4, 10, 100);
    
    // nextDevice
    nextDevice = TRUE;
    break;
    
  default:
    break;
  }

  
//... round robing to next device ............................................//
  if (nextDevice)
  {
    currentDevice = currentDevice +1;
  
    if(currentDevice > 2)  currentDevice = 0;
  }
  
  //oJo solo para debug
//  if (error)
  {
    HalUARTPrintf("-> periodicCentralTask: ");  
    HalUARTPrintf("numDevices: 0x%02x, currentDevice: 0x%02x, step: 0x%02x. ", numDevices, currentDevice, step);  
//  HalUARTPrintf("actualRole: 0x%02x\r\n", actualRole);  
//  HalUARTPrintf("requestRole: 0x%02x\r\n", requestRole);  
//  HalUARTPrintf("masterSlave_State: 0x%02x\r\n", masterSlave_State);  
//  HalUARTPrintf("currentDevice: 0x%02x\r\n", currentDevice);  
//  HalUARTPrintf("current_adv_enabled_sttus: 0x%02x\r\n", current_adv_enabled_status);  
  HalUARTPrintf("error: 0x%02x, sttus: 0x%02x.\r\n", error, status);
//  HalUARTPrintf("step: 0x%02x\r\n", step); 
  }
}
#endif // GAP_ROLE_CENTRAL


#if defined (GAP_ROLE_PERIPHERAL) && !defined (GAP_ROLE_CENTRAL)
//oJo, solo se ejecuta si solo esta seleccionado periphera... tiene sentido? quitar esta tarea y hacer una tara comun es lo mas logico
/*********************************************************************
 * @fn      periodicPeripheralTask
 *
 * @brief   Perform a periodic application task. 
 *
 * @param   none
 *
 * @return  none
 */
static void periodicPeripheralTask(void)
{  
  uint8 numDevices=1;
  
  //turn all LED's off
  HalLedSet(HAL_LED_ALL, HAL_LED_MODE_OFF); 
    
  switch (masterSlave_State)
  {
  case BLE_STATE_CONNECTING:
  case BLE_STATE_CONNECTED:
    if(actualRole == GAPROLE_CENTRAL)
    {
    }                
    //green+blue (peripheral)
    osal_pwrmgr_device( PWRMGR_BATTERY );
    HalLedBlink (HAL_LED_1 | HAL_LED_2 | HAL_LED_3, numDevices, 50, 500);
    osal_pwrmgr_device( PWRMGR_ALWAYS_ON );
    break;

  case BLE_STATE_DISCONNECTING:
  case BLE_STATE_IDLE:
    if(actualRole == GAPROLE_CENTRAL)
    {
      //are we finished disconnecting      
      //red
      HalLedBlink (HAL_LED_2, 2, 10, 100);
    }
    break;
    
  case BLE_STATE_ADVERTISING: 
    HalLedBlink(HAL_LED_3, 4, 10, 100);
    break;
    
  default:
    break;
  }
}
#endif

/*******************************************************************************
*******************************************************************************/



















//oJo, falta implementar esto ya que no tenfo handle_keys en este modulo

#if defined (GAP_ROLE_CENTRAL) || defined (GAP_ROLE_SWITCH)
/*********************************************************************
 * @fn      simpleBLESwitch_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
//static void simpleBLESwitch_HandleKeys( uint8 shift, uint8 keys )
//{
//  uint8 new_adv_enabled_status = TRUE;
//  
//  (void)shift;  // Intentionally unreferenced parameter
//  
//  // ROLE SWITCH  
//  if ( keys & HAL_KEY_SW_1 )
//  {
//    if( actualRole == ROLE_PERIPHERAL)
//    {
//      // PERIPHERAL --> CENTRAL
//      new_adv_enabled_status = FALSE;
//      
//      //change the GAP advertisement status to opposite of current status
//      GAPPeripheralRole_SetParameter( GAPROLE_ADVERT_ENABLED, sizeof( uint8 ), &new_adv_enabled_status ); 
//      
//      actualRole = ROLE_CENTRAL;
//      
//      osal_start_timerEx( dragonflyNetCore_TaskID, DRAGONFLY_NETCORE_STAR_EVT, 500 );
//    }
//    else
//    {
//      // CENTRAL --> PERIPHERAL
//      masterSlave_State = BLE_STATE_DISCONNECTING; 
//      
//      //cancel current link request 
//      GAPCentralRole_TerminateLink( 0xFFFF );
//      
//      //GAPCentralRole_TerminateLink( GAP_CONNHANDLE_ALL );
//      //GAPCentralRole_TerminateLink(0);
//      //GAPCentralRole_TerminateLink(1);
//    }
//  }
//}

extern uint8 DragonFly_Net_Core_RoleMode(gaprole_Modes_t role)
{
  uint8 new_adv_enabled_status = TRUE;
  gaprole_Modes_t newRole = GAPROLE_PERIPHERAL;
    
  // ROLE SWITCH  
  if (role == GAPROLE_SWITCH)
  {
    if(actualRole == GAPROLE_PERIPHERAL)
      newRole = GAPROLE_CENTRAL;
  }
  else if (role == GAPROLE_CENTRAL)
    newRole = GAPROLE_CENTRAL;
  
  // ROLE CENTRAL
  if (newRole == GAPROLE_CENTRAL)
  {
    if(actualRole == GAPROLE_PERIPHERAL)
    {
      // PERIPHERAL --> CENTRAL
      new_adv_enabled_status = FALSE;
      
      //change the GAP advertisement status to disabled
      GAPPeripheralRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8), &new_adv_enabled_status); 
      
      // Terminate any stablished connection
      GAPPeripheralRole_TerminateConnection();
      
      requestRole = GAPROLE_CENTRAL;
      
      // Panic change mode. oJo, hasta los huevos estoy
//      HalUARTPrintf("------------------------------ PERIPHERL TO CENTRAL ----\r\n");
//      ST_HAL_DELAY(1250);
//      HAL_SYSTEM_RESET();
      
      osal_start_timerEx(dragonflyNetCore_TaskID, DRAGONFLY_NETCORE_STAR_EVT, 500);
      return 0x00; // return OK
    }
    else
      // return error, already in this role
      return 0x02;
  }

  // ROLE PERIPHERRAL  
  if (newRole == GAPROLE_PERIPHERAL)
  {
    if(actualRole == GAPROLE_PERIPHERAL)
      // return error, already in this role
      return 0x02;
    else
    {
      // CENTRAL --> PERIPHERAL
      masterSlave_State = BLE_STATE_DISCONNECTING; 
      
      #if defined (GAP_ROLE_CENTRAL)
        //cancel current link request 
        GAPCentralRole_TerminateLink(0xFFFE);
        
      
//        GAPCentralRole_TerminateLink( GAP_CONNHANDLE_ALL );
//        GAPCentralRole_TerminateLink(0);
//        GAPCentralRole_TerminateLink(1);
      #endif
//      actualRole = GAPROLE_PERIPHERAL;
      
//      osal_start_timerEx(dragonflyNetCore_TaskID, DRAGONFLY_NETCORE_STAR_EVT, 500);

        
      return 0x00; // return OK
    }
  }
 
  // return error, unknow or unssupported request role
  return 0x03;
}

#endif 










//... netCore_Kernel..........................................................//

// ringled moore state initialization
static netCoreKernelState_t actualKernelState = KERNEL_STATE_INIT_ROLE;
static netCoreKernelState_t newKernelState = KERNEL_STATE_INIT_ROLE;

//static gaprole_Modes_t dragonflyNetCore_Mode = GAPROLE_CENTRAL;



/*******************************************************************************
 * @fn      dragonflyNetCore_Kernel
 *
 * @brief   Perform a periodic kernel polling.
 *
 * @param   none
 *
 * @return  none
 */
/*
Notas: 
-> este proceso se ejecuta isosincronamente cada DRAGONFLY_NETCORE_KERNEL_EVT_PERIOD

*/
static void dragonflyNetCore_Kernel(void)
{
  // internal variables
//  static uint16 neopixel_n_frames = 0;
  uint16 counter = 0;
  bStatus_t status = SUCCESS;
  uint8 error = 0;

  
//////////////////////// MOORE AUTOMATE IMPLEMENTATION /////////////////////////  
  switch(actualRole)
  {

    
/// idle role //////////////////////////////////////////////////////////////////
    case GAPROLE_IDLE:
      // asincronous actions
/*  -> en periodic event
        osal_start_timerEx(dragonflyNetCore_TaskID, 
                           DRAGONFLY_NETCORE_KERNEL_EVT, 
                           DRAGONFLY_NETCORE_KERNEL_EVT_PERIOD);      
*/      
      // callback to application
//      HalChaseStop();
      
      // clear machine register
//      machine_register &= ~HAL_NEOPIXEL_AUTOMATE;

      break;

      
/// central role - connecting machine///////////////////////////////////////////
    case GAPROLE_CENTRAL:

///-- STARTxxxxxx -----------------------------------------------------------///
      if (actualKernelState == KERNEL_STATE_INIT_ROLE)
      {
        // 1. set timers
        
        // 2.2. start role
        // register GAP role central callbacks
        status = GAPCentralRole_StartDevice((gapCentralRoleCBs_t *) 
                                  &dragonflyNetCore_CentralRoleCBs);
        
        if ( status != SUCCESS) 
        {
          // error
          handleError(ERROR_STAR_CENTRAL, status);
           
          // next state
          newKernelState = KERNEL_STATE_ERROR;
        }

        // 3. next state
        newKernelState = KERNEL_STATE_CONNECTING;

        
        break;        
      }

///-- KERNEL_xxxxxxxxxxxxxxxx -----------------------------------------------///
      else if (actualKernelState == KERNEL_STATE_CONNECTING)
      {
        // 1. set timers
        
        // 2. actions
        
        
        // 2.1. initiate the first request to avoid error in first loop
        status = GAPCentralRole_EstablishLink(DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                              DEFAULT_LINK_WHITE_LIST,
                                              ADDRTYPE_PUBLIC, 
                                              deviceList[currentDevice].addr );

        if (status != SUCCESS) 
        {
          // error
          handleError(BLE_STATE_CONNECTING, status);
           
          // 3. next state
          newKernelState = KERNEL_STATE_ERROR;
        }
        else
        {
          deviceList[currentDevice].state = BLE_STATE_CONNECTING;
        }
          
        // next state
        // waiting to timeout -> newStatee = KERNEL_STATE_NEXT_DEVICE
        // waiting to GAP callbacks notification  -> newStatee = KERNEL_STATE_NEXT_DEVICE
        newKernelState = KERNEL_STATE_WAITING;
        break;        
      }

///-- KERNEL_xxxxxxxxxxxxxxxx -----------------------------------------------///
      else if (actualKernelState == KERNEL_STATE_WAITING)
      {
        // 1. set timers

        
        // 2. actions
        if (deviceList[currentDevice].state == BLE_STATE_CONNECTED)
        {
          // switch to next device
          
          // and connect to this new devic
          newKernelState = KERNEL_STATE_NEXT_DEVICE;        
        }
          
        // next state
        newKernelState = KERNEL_STATE_WAITING;
        break;        
      }
      
///-- KERNEL_STATE_TERMINATE ------------------------------------------------///
      else if (actualKernelState == KERNEL_STATE_CONNECTING)
      {
        // 1. set timers
        
        // 2. actions
        
        
           // 2.1.- terminate initiated links request 
            status = GAPCentralRole_TerminateLink(GAP_CONNHANDLE_INIT);

            if (status != SUCCESS) 
            {
              error |= 0x01;
              HalUARTPrintf("terminate init link error\r\n");
            }        
              HalUARTPrintf("terminate init link ok\r\n");
        
            
            status = GAPCentralRole_TerminateLink(GAP_CONNHANDLE_ALL);

            if (status != SUCCESS) 
            {
              error |= 0x01;
              HalUARTPrintf("terminate all link error\r\n");
            }        
              HalUARTPrintf("terminate all link ok\r\n");

        // next state
        newKernelState = KERNEL_STATE_START;
        break;        
      }
      
///-- KERNEL_STATE_TERMINATE ------------------------------------------------///
      else if (actualKernelState == KERNEL_STATE_CONNECTED)
      {
        // 1. set timers
        
        // 2. actions
        
        
           // 2.1.- terminate initiated links request 
            status = GAPCentralRole_TerminateLink(GAP_CONNHANDLE_INIT);

            if (status != SUCCESS) 
            {
              error |= 0x01;
              HalUARTPrintf("terminate init link error\r\n");
            }        
              HalUARTPrintf("terminate init link ok\r\n");
        
            
            status = GAPCentralRole_TerminateLink(GAP_CONNHANDLE_ALL);

            if (status != SUCCESS) 
            {
              error |= 0x01;
              HalUARTPrintf("terminate all link error\r\n");
            }        
              HalUARTPrintf("terminate all link ok\r\n");

        // next state
        newKernelState = KERNEL_STATE_START;
        break;        
      }
            
///-- KERNEL_STATE_TERMINATE ------------------------------------------------///
      else if (actualKernelState == KERNEL_STATE_TERMINATING)
      {
        // 1. set timers
        
        // 2. actions
        
        
           // 2.1.- terminate initiated links request 
            status = GAPCentralRole_TerminateLink(GAP_CONNHANDLE_INIT);

            if (status != SUCCESS) 
            {
              error |= 0x01;
              HalUARTPrintf("terminate init link error\r\n");
            }        
              HalUARTPrintf("terminate init link ok\r\n");
        
            
            status = GAPCentralRole_TerminateLink(GAP_CONNHANDLE_ALL);

            if (status != SUCCESS) 
            {
              error |= 0x01;
              HalUARTPrintf("terminate all link error\r\n");
            }        
              HalUARTPrintf("terminate all link ok\r\n");

        // next state
        newKernelState = KERNEL_STATE_START;
        break;        
      }
      
///-- KERNEL_STATE_TERMINATE ------------------------------------------------///
      else if (actualKernelState == KERNEL_STATE_TERMINATED)
      {
        // 1. set timers
        
        // 2. actions
        
        
           // 2.1.- terminate initiated links request 
            status = GAPCentralRole_TerminateLink(GAP_CONNHANDLE_INIT);

            if (status != SUCCESS) 
            {
              error |= 0x01;
              HalUARTPrintf("terminate init link error\r\n");
            }        
              HalUARTPrintf("terminate init link ok\r\n");
        
            
            status = GAPCentralRole_TerminateLink(GAP_CONNHANDLE_ALL);

            if (status != SUCCESS) 
            {
              error |= 0x01;
              HalUARTPrintf("terminate all link error\r\n");
            }        
              HalUARTPrintf("terminate all link ok\r\n");

        // next state
        newKernelState = KERNEL_STATE_START;
        break;        
      }
      
///-- KERNEL_STATE_ERROR ----------------------------------------------------///
      else if (actualKernelState == KERNEL_STATE_ERROR)
      {
        // 1. set timers
        
        // 2. actions

        // next state
        newKernelState = KERNEL_STATE_ERROR;
        break;        
      }
      
/// other roles ////////////////////////////////////////////////////////////////
    default:
      break;
  }
  
  // actualize state
  actualKernelState = newKernelState;
}


void handleError(uint8 error, uint8 status)
{
  // print the error
  HalUARTPrintf("error_code, status: %d, %d\r\n", error, status);
};







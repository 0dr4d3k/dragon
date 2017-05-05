/**************************************************************************************************
  Filename:       DragonFly_Net_Core.h
  Revised:        $Date: 2013-03-25 07:58:08 -0700 (Mon, 25 Mar 2013) $
  Revision:       $Revision: 33575 $

  Description:    This file contains the Dragonfly sample application
                  definitions and prototypes.
**************************************************************************************************/

#ifndef DRAGONFLY_NET_CORE_H
#define DRAGONFLY_NET_CORE_H

#ifdef __cplusplus
extern "C"
{
#endif
  
//extern gaprole_States_t gapProfileState = GAPROLE_INIT; // <- oJo, definir como global NET->ALL
  

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */

// System Events
#define DRAGONFLY_NETCORE_STAR_EVT                        0x0001
#define DRAGONFLY_NETCORE_KERNEL_EVT                      0x0002
#define DRAGONFLY_NETCORE_PERIODIC_EVT                    0x0004

   
/*********************************************************************
 * MACROS
 */

  
/*********************************************************************
 * TYPEDEFS
 */
typedef enum
{
  GAPROLE_PERIPHERAL = 0,                 //!< Peripheral Role
  GAPROLE_CENTRAL,                        //!< Central Role
  GAPROLE_CENTRAK_PERIPHERAL,             //!< Central & Peripheral Role
  GAPROLE_SWITCH,                         //!< Switch Peripheral <-> Central
  GAPROLE_IDLE                            //!< Idle Role
} gaprole_Modes_t;  

// NetCore Moore Automate state definitions
typedef enum
{
  KERNEL_STATE_START = 0,
  KERNEL_STATE_SCAN,
  KERNEL_STATE_CONNECTING,
  KERNEL_STATE_CONNECTED,
  KERNEL_STATE_WAITING,
  KERNEL_STATE_TERMINATING,
  KERNEL_STATE_TERMINATED,
  KERNEL_STATE_DISCONNECTED,
  KERNEL_STATE_SEND_DATA,
  KERNEL_STATE_GET_DATA,          
  KERNEL_STATE_ERROR,          
  KERNEL_STATE_NEXT_DEVICE,
  KERNEL_STATE_INIT_ROLE,
  KERNEL_STATE_IDLE
} netCoreKernelState_t;  

// Application states
enum
{
  BLE_STATE_IDLE,           // 0x00
  BLE_STATE_CONNECTING,     // 0x01
  BLE_STATE_CONNECTED,      // 0x02
  BLE_STATE_DISCONNECTING,  // 0x03
  BLE_STATE_ADVERTISING     // 0x04
};

// Error codes
enum
{
  ERROR_STAR_ROLE,
  ERROR_STAR_CENTRAL,
  ERROR_CONNECTING,
  ERROR_TERMINATING
};

/*********************************************************************
 * FUNCTIONS
 */
/*
 * Task Initialization for the BLE Application
 */
extern void DragonFly_Net_Core_Init( uint8 task_id );

/*
 * Task Event Processor for the BLE Application
 */
extern uint16 DragonFly_Net_Core_ProcessEvent( uint8 task_id, uint16 events );

/*
 * Power on self test
 */
extern uint16 DragonFly_Net_Core_Test(void);

/*
 * Set Role Mode
 */
extern uint8 DragonFly_Net_Core_RoleMode(gaprole_Modes_t role);

// oJo solo para testerar SERIAL_INTERFACE, eliminar
uint8 Application_StartAdvertise(uint16 duration, uint16 interval);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* DRAGONFLY_H */

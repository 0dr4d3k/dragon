/**************************************************************************************************
  Filename:       OSAL_DragonFly.c
  Revised:        $Date: 2015-04-10 14:27:43 -0700 (Fri, 10 Apr 2015) $
  Revision:       $Revision: 43387 $

  Description:    This file contains function that allows user setup tasks
**************************************************************************************************/

/**************************************************************************************************
 *                                            INCLUDES
 **************************************************************************************************/
#include "hal_types.h"
#include "OSAL.h"
#include "OSAL_Tasks.h"

/* HAL */
#include "hal_drivers.h"

/* LL */
#include "ll.h"

/* HCI */
#include "hci_tl.h"

#if defined (OSAL_CBTIMER_NUM_TASKS)
  #include "osal_cbtimer.h"
#endif

/* L2CAP */
#include "l2cap.h"

/* gap */
#include "gap.h"
//#include "gapgattserver.h"
#include "gapbondmgr.h"

/* GATT */
#include "gatt.h"
#include "gattservapp.h"

/* Profiles */
#if defined (GAP_ROLE_PERIPHERAL)
  #include "peripheral.h"
#endif

#if defined (GAP_ROLE_CENTRAL)
  #include "central.h"
#endif

/* Application */
#if defined (DRAGONFLY_GATE)
  #include "gateservice.h"
  #include "DragonFly_Gate.h"
#endif 

#if defined (SERIAL_INTERFACE)
#include "serialInterface.h"
#endif 

#include "DragonFly_Oad.h"
#include "DragonFly_Net_Core.h"
#include "DragonFly.h"
   
/*********************************************************************
 * GLOBAL VARIABLES
 */

// The order in this table must be identical to the task initialization calls below in osalInitTask.
const pTaskEventHandlerFn tasksArr[] =
{
////////////////////////////////////////////////////////////////////////////////  
  LL_ProcessEvent,                                                  // task 0
////////////////////////////////////////////////////////////////////////////////  
  Hal_ProcessEvent,                                                 // task 1
////////////////////////////////////////////////////////////////////////////////  
  HCI_ProcessEvent,                                                 // task 2
////////////////////////////////////////////////////////////////////////////////  
#if defined (OSAL_CBTIMER_NUM_TASKS)
  OSAL_CBTIMER_PROCESS_EVENT(osal_CbTimerProcessEvent),             // task 3
#endif
////////////////////////////////////////////////////////////////////////////////  
  L2CAP_ProcessEvent,                                               // task 4
////////////////////////////////////////////////////////////////////////////////  
  GAP_ProcessEvent,                                                 // task 5
////////////////////////////////////////////////////////////////////////////////  
  SM_ProcessEvent,                                                  // task 6
////////////////////////////////////////////////////////////////////////////////  
  GATT_ProcessEvent,                                                // task 7
////////////////////////////////////////////////////////////////////////////////  
#if defined (GAP_ROLE_PERIPHERAL)
  GAPRole_ProcessEvent,                                             // task 8
#endif
////////////////////////////////////////////////////////////////////////////////  
#if defined (GAP_ROLE_CENTRAL)
  GAPCentralRole_ProcessEvent,                                      // task 9
#endif
////////////////////////////////////////////////////////////////////////////////  
#if defined (GAP_BOND)
  GAPBondMgr_ProcessEvent,                                          // task 10
#endif
////////////////////////////////////////////////////////////////////////////////  
  GATTServApp_ProcessEvent,                                         // task 11
////////////////////////////////////////////////////////////////////////////////  
#if defined (DRAGONFLY_GATE)
  DragonFly_Gate_ProcessEvent,
#endif
////////////////////////////////////////////////////////////////////////////////  
#if defined (SERIAL_INTERFACE)
  SerialInterface_ProcessEvent,
#endif 
////////////////////////////////////////////////////////////////////////////////  
  DragonFly_Net_Core_ProcessEvent,                                            
////////////////////////////////////////////////////////////////////////////////  
  DragonFly_Oad_ProcessEvent,                                            
////////////////////////////////////////////////////////////////////////////////  
  DragonFly_ProcessEvent                                            
////////////////////////////////////////////////////////////////////////////////  
};

const uint8 tasksCnt = sizeof(tasksArr) / sizeof(tasksArr[0]);
uint16 *tasksEvents;

/*********************************************************************
 * FUNCTIONS
 *********************************************************************/

/*********************************************************************
 * @fn      osalInitTasks
 *
 * @brief   This function invokes the initialization function for each task.
 *
 * @param   void
 *
 * @return  none
 */
void osalInitTasks(void)
{
  uint8 taskID = 0;

  tasksEvents = (uint16 *)osal_mem_alloc( sizeof( uint16 ) * tasksCnt);
  osal_memset(tasksEvents, 0, (sizeof( uint16 ) * tasksCnt));

  /* LL Task */
  LL_Init(taskID++);

  /* Hal Task */
  Hal_Init(taskID++);

  /* HCI Task */
  HCI_Init(taskID++);

#if defined (OSAL_CBTIMER_NUM_TASKS)
  /* Callback Timer Tasks */
  osal_CbTimerInit(taskID);
  taskID += OSAL_CBTIMER_NUM_TASKS;
#endif

  /* L2CAP Task */
  L2CAP_Init(taskID++);

  /* GAP Task */
  GAP_Init(taskID++);

  /* SM Task */
  SM_Init(taskID++);
  
  /* GATT Task */
  GATT_Init(taskID++);

  /* Profiles */
#if defined (GAP_ROLE_PERIPHERAL)
  GAPPeripheralRole_Init(taskID++);
#endif

#if defined (GAP_ROLE_CENTRAL)
  GAPCentralRole_Init(taskID++);
#endif

#if defined GAP_BOND
  GAPBondMgr_Init(taskID++);
#endif
  
  GATTServApp_Init(taskID++);

  /* DragonFly_Gate Task */
#if defined (DRAGONFLY_GATE)
  DragonFly_Gate_Init(taskID++);
#endif
  
  /* Application */
#if defined (SERIAL_INTERFACE)
  SerialInterface_Init(taskID++);
#endif 
  
  DragonFly_Net_Core_Init(taskID++);

  DragonFly_Oad_Init(taskID++);

  DragonFly_Init(taskID);
}

/*********************************************************************
*********************************************************************/

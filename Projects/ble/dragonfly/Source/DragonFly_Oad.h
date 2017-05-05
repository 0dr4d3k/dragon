/**************************************************************************************************
  Filename:       DragonFly_Oad.h
  Revised:        $Date: 2013-03-25 07:58:08 -0700 (Mon, 25 Mar 2013) $
  Revision:       $Revision: 33575 $

  Description:    This file contains the Dragonfly sample application
                  definitions and prototypes.
**************************************************************************************************/

#ifndef DRAGONFLY_OAD_H
#define DRAGONFLY_OAD_H

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
#define DRAGONFLY_OAD_STAR_EVT                        0x0001

   
/*********************************************************************
 * MACROS
 */

   
/*********************************************************************
 * FUNCTIONS
 */
/*
 * Task Initialization for the BLE Application
 */
extern void DragonFly_Oad_Init( uint8 task_id );

/*
 * Task Event Processor for the BLE Application
 */
extern uint16 DragonFly_Oad_ProcessEvent( uint8 task_id, uint16 events );

/*
 * Power on self test
 */
extern uint16 DragonFly_Oad_Test(void);


/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* DRAGONFLY_H */

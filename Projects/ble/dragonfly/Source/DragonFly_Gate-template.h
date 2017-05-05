/*******************************************************************************
  Filename:       DragonFly_Gate.h
  Revised:        $Date: 2017-04-17 07:07:07$
  Revision:       $Revision: 1 $

  Description:    This file contains the Dragonfly Gate sample application
                  definitions and prototypes. DragonFly Gate is the communication
                  path with the real world.
*******************************************************************************/
#ifndef DRAGONFLY_GATE_H
#define DRAGONFLY_GATE_H

#ifdef __cplusplus
extern "C"
{
#endif

  
/*******************************************************************************
 * INCLUDES
 */

  
/*******************************************************************************
 * CONSTANTS
 */
// System Events
#define DRAGONFLY_GATE_STAR_EVT                       0x0001
#define DRAGONFLY_GATE_RESET_EVT                      0x0002
#define DRAGONFLY_GATE_PERIODIC_EVT                   0x0040
    
// HAL Events
#define DRAGONFLY_GATE_HAL_UART_EVT                   0x0010
  
// Defined modules  
#if defined FEATURE_DIGITAL 
#endif

#if defined FEATURE_ANALOG 
#endif
  
#if defined FEATURE_WEATHER 
#endif
  
#if defined FEATURE_LUXOMETER 
#endif

#if defined FEATURE_MAGNETOMETER 
#endif

#if defined FEATURE_AIR 
#endif

#if defined FEATURE_UVI 
#endif
  

/*******************************************************************************
 * MACROS
 */

  
/*******************************************************************************
 * FUNCTIONS
 */
/*
 * Task Initialization for the Gate Application
 */
extern void DragonFly_Gate_Init(uint8 task_id);

/*
 * Task Event Processor for the Gate Application
 */
extern uint16 DragonFly_Gate_ProcessEvent(uint8 task_id, uint16 events);


/*******************************************************************************
*******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* DRAGONFLY_GATE_H */

/**************************************************************************************************
  Filename:       hal_drivers.h
  Revised:        $Date: 2012-07-09 13:23:30 -0700 (Mon, 09 Jul 2012) $
  Revision:       $Revision: 30873 $

  Description:    This file contains the interface to the Drivers service. Merged with 
                  hal_drivers.h air
**************************************************************************************************/
#ifndef HAL_DRIVER_H
#define HAL_DRIVER_H

#ifdef __cplusplus
extern "C"
{
#endif

/**************************************************************************************************
 * INCLUDES
 **************************************************************************************************/

#include "hal_board.h"
#include "hal_types.h"

/**************************************************************************************************
 * CONSTANTS
 **************************************************************************************************/

#define HAL_BUZZER_EVENT                    0x0080
#define PERIOD_RSSI_RESET_EVT               0x0040
#define HAL_LED_BLINK_EVENT                 0x0020
#define HAL_KEY_EVENT                       0x0010

#if (defined POWER_SAVING || DMA_PM == 1) 
#define HAL_SLEEP_TIMER_EVENT               0x0004
#define HAL_PWRMGR_HOLD_EVENT               0x0002
#define HAL_PWRMGR_CONSERVE_EVENT           0x0001
#endif

#define HAL_PWRMGR_CONSERVE_DELAY           10
#define PERIOD_RSSI_RESET_TIMEOUT           10

/**************************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************************/

extern uint8 Hal_TaskID;

/**************************************************************************************************
 * FUNCTIONS - API
 **************************************************************************************************/

extern void Hal_Init ( uint8 task_id );

/*
 * Process Hal Events
 */
extern uint16 Hal_ProcessEvent ( uint8 task_id, uint16 events );

/*
 * Process Polls
 */
extern void Hal_ProcessPoll (void);

/*
 * Initialize HW
 */
extern void HalDriverInit (void);

/**************************************************************************************************
 * @fn          halDriverBegPM
 *
 * @brief       This function is called before entering PM so that drivers can be put into their
 *              lowest power states.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void halDriverBegPM(void);

/**************************************************************************************************
 * @fn          halDriverEndPM
 *
 * @brief       This function is called after exiting PM so that drivers can be restored to their
 *              ready power states.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      None.
 **************************************************************************************************
 */
void halDriverEndPM(void);


#ifdef __cplusplus
}
#endif

#endif

/**************************************************************************************************
**************************************************************************************************/

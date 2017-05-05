/*******************************************************************************
  Filename:       SensorTag_Main.c
  Revised:        $Date: 2012-08-08 17:04:23 -0700 (Wed, 08 Aug 2012) $
  Revision:       $Revision: 31161 $

  Description:    This file contains the main and callback functions for
                  the Dragonfly sample application.
*******************************************************************************/
/*******************************************************************************
 *                                 Includes
 ******************************************************************************/
/* Hal Drivers */
#include "hal_types.h"
#include "hal_board.h"
#include "hal_keys.h"
#include "hal_timer.h"
#include "hal_drivers.h"
#include "hal_led.h"
#include "hal_assert.h"

/* OSAL */
#include "OSAL.h"
#include "OSAL_Tasks.h"
#include "OSAL_PwrMgr.h"
#include "osal_snv.h"
#include "OnBoard.h"

/* Application */
#include "DragonFly.h"


/*******************************************************************************
 * FUNCTIONS
 ******************************************************************************/
/*******************************************************************************
 * @fn          main
 * @brief       Start of application.
 * @param       none
 * @return      int 0
 *******************************************************************************
 */
int main(void)
{
  /* Initialize hardware */
  HAL_BOARD_INIT();

  // Initialize board I/O
  InitBoard(OB_COLD);

  /* Initialze the HAL driver */
  HalDriverInit();

  /* Initialize NV system */
#if defined NV_MEMORY
  osal_snv_init();
#endif
  
  /* Initialize the operating system */
  osal_init_system();

  /* Enable interrupts */
  HAL_ENABLE_INTERRUPTS();

  // Final board initialization
  InitBoard(OB_READY);

  #if defined(POWER_SAVING)
    osal_pwrmgr_device(PWRMGR_BATTERY);
  #endif

  /* Run power on self-test */
  DragonFly_Test();

  /* Start OSAL */
  osal_start_system(); // No Return from here

  return 0;
}


/*******************************************************************************
                                  CALL-BACKS
*******************************************************************************/
/*******************************************************************************
*******************************************************************************/

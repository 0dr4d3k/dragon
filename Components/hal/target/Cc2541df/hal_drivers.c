/**************************************************************************************************
  Filename:       hal_drivers.c
  Revised:        $Date: 2007-07-06 10:42:24 -0700 (Fri, 06 Jul 2007) $
  Revision:       $Revision: 13579 $

  Description:    This file contains the interface to the Drivers Service. Merged with
                  hal_drivers.c v30649 air

**************************************************************************************************/

/**************************************************************************************************
 *                                            INCLUDES
 **************************************************************************************************/

#include "OSAL.h"
#include "hal_board.h"
#include "hal_keys.h"
#include "hal_led.h"
#include "hal_sleep.h"
#include "hal_timer.h"
#include "hal_types.h"
#include "hal_uart.h"
#include "hal_drivers.h"
#include "hal_adc.h"
#include "hal_pir.h"
#include "hal_qi.h"
#include "hal_weather.h"
#include "hal_lux.h"
#include "hal_uvi.h"
#include "hal_suart.h"
#include "hal_acc.h"
#include "hal_mag.h"
#include "hal_air.h"
#include "hal_i2c.h"
#include "OSAL_PwrMgr.h"
#include "hal_dma.h"
#include "hal_aes.h"
#include "hal_buzzer.h"


/**************************************************************************************************
 *                                      GLOBAL VARIABLES
 **************************************************************************************************/
uint8 Hal_TaskID;

extern void HalLedUpdate( void ); /* Notes: This for internal only so it shouldn't be in hal_led.h */

/**************************************************************************************************
 *                                      FUNCTIONS - API
 **************************************************************************************************/

/**************************************************************************************************
 * @fn      Hal_Init
 *
 * @brief   Hal Initialization function.
 *
 * @param   task_id - Hal TaskId
 *
 * @return  None
 **************************************************************************************************/
void Hal_Init( uint8 task_id )
{
  /* Register task ID */
  Hal_TaskID = task_id;
}

/**************************************************************************************************
 * @fn      Hal_DriverInit
 *
 * @brief   Initialize HW - These need to be initialized before anyone.
 *
 * @param   task_id - Hal TaskId
 *
 * @return  None
 **************************************************************************************************/
void HalDriverInit (void)
{

  /* SUART */
#if (defined HAL_SUART) && (HAL_SUART == TRUE)
  HalSuartInit();
#endif

  /* I2C */
#if (defined HAL_I2C) && (HAL_I2C == TRUE)
  /* mandatory I2C running from start */
  HalI2CInit(HAL_BMA250_I2C_ADDRESS, i2cClock_267KHZ);
#endif
  
  /* Accelerometer */
#if (defined HAL_ACC) && (HAL_ACC == TRUE)
  HalAccInit();
#endif

  /* Magnetometer */
#if (defined HAL_MAG) && (HAL_MAG == TRUE)
  HalMagInit();
#endif

  /* ADC */
#if (defined HAL_ADC) && (HAL_ADC == TRUE)
  HalAdcInit();
#endif

  /* WEATHER */
#if (defined HAL_WEATHER) && (HAL_WEATHER == TRUE)
  HalWeatherInit();
#endif

  /* LUXOMETER */
#if (defined HAL_LUX) && (HAL_LUX == TRUE)
  HalLuxInit();
#endif
  
  /* UVI */
#if (defined HAL_UVI) && (HAL_UVI == TRUE)
  HalUViInit();
#endif

  /* AIR */
#if (defined HAL_AIR) && (HAL_AIR == TRUE)
  HalAirInit();
#endif
  
  /* PIR */
#if (defined HAL_PIR) && (HAL_PIR == TRUE)
  HalPirInit();
#endif

  /* QI */
#if (defined HAL_QI) && (HAL_QI == TRUE)
  HalQiInit();
#endif
  
  /* DMA */
#if (defined HAL_DMA) && (HAL_DMA == TRUE)
  // Must be called before the init call to any module that uses DMA.
  HalDmaInit();
#endif

  /* AES */
#if (defined HAL_AES_DMA) && (HAL_AES_DMA == TRUE)
  HalAesInit();
#endif

  /* LED */
#if (defined HAL_LED) && (HAL_LED == TRUE)
  HalLedInit();
#endif

  /* KEY */
#if (defined HAL_KEY) && (HAL_KEY == TRUE)
  HalKeyInit();
#endif

 /* UART */
#if (defined HAL_UART) && (HAL_UART == TRUE)
  HalUARTInit();
#endif
  
  /* HID */
#if (defined HAL_HID) && (HAL_HID == TRUE)
  usbHidInit();
#endif

  /* Buzzer */
#if (defined HAL_BUZZER) && (HAL_BUZZER == TRUE)
  HalBuzzerInit();
#endif
  
}
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------
//MIRAR IMPLEMENTAR ESTAS FUNCIONES
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
void halDriverBegPM(void)
{
#if ((defined HAL_LED) && (HAL_LED == TRUE))
  HalLedEnterSleep();
#endif
#if ((defined HAL_KEY) && (HAL_KEY == TRUE))
  HalKeyEnterSleep();
#endif
#if ((defined HAL_I2C) && (HAL_I2C == TRUE))
//  Mirar hal_i2c en AIR es una libreria mas completa y admite esta funcion 
//  HalI2CEnterSleep();
#endif
}

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
void halDriverEndPM(void)
{
#if ((defined HAL_LED) && (HAL_LED == TRUE))
  HalLedExitSleep();
#endif
#if ((defined HAL_KEY) && (HAL_KEY == TRUE))
  HalKeyExitSleep();
#endif
#if ((defined HAL_I2C) && (HAL_I2C == TRUE))
//  Mirar hal_i2c en AIR es una libreria mas completa y admite esta funcion 
//  HalI2CExitSleep();
#endif
}
//-----------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------






/**************************************************************************************************
 * @fn      Hal_ProcessEvent
 *
 * @brief   Hal Process Event
 *
 * @param   task_id - Hal TaskId
 *          events - events
 *
 * @return  None
 **************************************************************************************************/
uint16 Hal_ProcessEvent( uint8 task_id, uint16 events )
{
  uint8 *msgPtr;

  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    msgPtr = osal_msg_receive(Hal_TaskID);

    while (msgPtr)
    {
      /* Do something here - for now, just deallocate the msg and move on */

      /* De-allocate */
      osal_msg_deallocate( msgPtr );
      /* Next */
      msgPtr = osal_msg_receive( Hal_TaskID );
    }
    return events ^ SYS_EVENT_MSG;
  }

  if ( events & HAL_LED_BLINK_EVENT )
  {
#if (defined (BLINK_LEDS)) && (HAL_LED == TRUE)
    HalLedUpdate();
#endif /* BLINK_LEDS && HAL_LED */
    return events ^ HAL_LED_BLINK_EVENT;
  }

  if (events & HAL_KEY_EVENT)
  {
#if (defined HAL_KEY) && (HAL_KEY == TRUE)
    /* Check for keys */
    HalKeyPoll();

    /* if interrupt disabled, do next polling */
    if (!Hal_KeyIntEnable)
    {
      osal_start_timerEx( Hal_TaskID, HAL_KEY_EVENT, 100);
    }
#endif
    return events ^ HAL_KEY_EVENT;
  }

// handled in App layer
//  if (events & HAL_LUX_EVENT)
//  {
//#if (defined HAL_LUX) && (HAL_LUX == TRUE)
//    /* Data is now generated; recolect it */
//
//      HalLuxExecMeasurementStep(luxState);
//#endif
//    return events ^ HAL_LUX_EVENT;
//  }
  
#if defined POWER_SAVING
  if ( events & HAL_SLEEP_TIMER_EVENT )
  {
    halRestoreSleepLevel();
    return events ^ HAL_SLEEP_TIMER_EVENT;
  }

  if ( events & HAL_PWRMGR_HOLD_EVENT )
  {
    (void)osal_pwrmgr_task_state(Hal_TaskID, PWRMGR_HOLD);

    (void)osal_stop_timerEx(Hal_TaskID, HAL_PWRMGR_CONSERVE_EVENT);
    (void)osal_clear_event(Hal_TaskID, HAL_PWRMGR_CONSERVE_EVENT);

    return (events & ~(HAL_PWRMGR_HOLD_EVENT | HAL_PWRMGR_CONSERVE_EVENT));
  }

  if ( events & HAL_PWRMGR_CONSERVE_EVENT )
  {
    (void)osal_pwrmgr_task_state(Hal_TaskID, PWRMGR_CONSERVE);
    return events ^ HAL_PWRMGR_CONSERVE_EVENT;
  }
#endif

#if (defined HAL_BUZZER) && (HAL_BUZZER == TRUE)
  if (events & HAL_BUZZER_EVENT)
  {
    HalBuzzerStop();
    return events ^ HAL_BUZZER_EVENT;
  }
#endif


  return 0;
}

/**************************************************************************************************
 * @fn      Hal_ProcessPoll
 *
 * @brief   This routine will be called by OSAL to poll UART, TIMER...
 *
 * @param   task_id - Hal TaskId
 *
 * @return  None
 **************************************************************************************************/
void Hal_ProcessPoll ()
{
#if defined( POWER_SAVING )
  /* Allow sleep before the next OSAL event loop */
  ALLOW_SLEEP_MODE();
#endif
  
  /* UART Poll */
#if (defined HAL_UART) && (HAL_UART == TRUE)
  HalUARTPoll();
#endif
  
  /* HID poll */
#if (defined HAL_HID) && (HAL_HID == TRUE)
  usbHidProcessEvents();
#endif
 
}

/**************************************************************************************************
**************************************************************************************************/


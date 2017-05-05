/*******************************************************************************
  Filename:       hal_debug.c
  Revised:        $Date: 2017-04-12 11:22:22$
  Revision:       $Revision: 00001 $

  Description:    Debug context for Dragonfly.
*******************************************************************************/


/* -----------------------------------------------------------------------------
 *                                 Includes
 * -----------------------------------------------------------------------------
 */
#include "hal_board.h"
#include "hal_debug.h"
#include "hal_types.h"
#include "hal_defs.h"
#include "hal_mcu.h"


#if (defined HAL_DEBUG) && (HAL_DEBUG == TRUE)
/* -----------------------------------------------------------------------------
 *                                 Constants
 * -----------------------------------------------------------------------------
 */

   
/* -----------------------------------------------------------------------------
 *                         Protected Local Variables
 * -----------------------------------------------------------------------------
 */
#if (defined DEBUG_SYSTEM)
  static uint32 debug_context = DEBUG_SYSTEM;   
#else
  static uint32 debug_context = DEFAULT_DEBUG_SYSTEM;   
#endif 

  
/* -----------------------------------------------------------------------------
 *                              Local Functions
 * -----------------------------------------------------------------------------
 */
/*******************************************************************************
 * @fn          halDebugHandler
 * @brief       Logic to handle an assert or error.
 * @param       none
 * @return      none
 *******************************************************************************
 */
void halDebugHandler(void)
{
  /* execute code that handles asserts */
#if (defined DEBUG_ASSERT_RESET)
  HAL_SYSTEM_RESET();
#endif
  
#if (defined DEBUG_ASSERT_WHILE)
  while(1);
#endif

#if (defined DEBUG_ASSERT_LIGHTS)
  halDebugErrorLights();
#endif
}


#if defined DEBUG_ASSERT_LIGHTS
/*******************************************************************************
 * @fn          halDebugErrorLights
 * @brief       Blink LEDs to indicate an error.
 * @param       none
 * @return      none
 *******************************************************************************
 */
void halDebugErrorLights(void)
{

  /* disable all interrupts before anything else */
  HAL_DISABLE_INTERRUPTS();

  /* Master infinite loop */
  for (;;)
  {
#if (defined HAL_LED) && (HAL_LED == TRUE) 
    /* toggle LEDS */
    HAL_TOGGLE_LED1();
    ST_HAL_DELAY(0x1450); //~40ms
    
    HAL_TOGGLE_LED2();
    ST_HAL_DELAY(0x1450); //~40ms
    
    HAL_TOGGLE_LED3();
    ST_HAL_DELAY(0x1450); //~40ms
#endif
  }
}
#endif


/*******************************************************************************
 * @fn          halDebugAssertContext
 * @brief       Assert context before to handle
 * @param       context: context to assert
 * @return      TRUE if assert, FALSE if not
 *******************************************************************************
 */
bool halDebugAssertContext(uint32 context)
{
  /* if DEBUG_FORCE_UART and no UART enabled skip debug statement */
  if (((context & DEBUG_FORCE_MASK) == DEBUG_FORCE_UART) & 
       (DEBUG_OVER_UART == FALSE))
    return FALSE;
  
  /* if SKIP_CONTEXT request then assert incoditionaly */
  if ((context & DEBUG_LEVEL_MASK) == SKIP_CONTEXT)
    return TRUE;

  /* check if request context level is less or equal that the actual defined 
  debug_contex level*/
  if ((debug_context & DEBUG_LEVEL_MASK) >= (context & DEBUG_LEVEL_MASK))
  {  
    if ((debug_context & DEBUG_MODULE_MASK) & (context & DEBUG_MODULE_MASK))
      return TRUE;
    else
      return FALSE;
  }
  else
    return FALSE;
}


/* -----------------------------------------------------------------------------
 *                                Api Functions
 * -----------------------------------------------------------------------------
 */
/*******************************************************************************
 * @fn          halDebugMsg
 * @brief       Send a printf message to selected output if context asserted
 * @param       context: context to assert
 *              format_string: printf string and arguments
 * @return      TRUE if assert, FALSE if not
 *******************************************************************************
 */
void halDebugMsg(uint32 context, char *format_string, ...)
{
  /* assert context */
  if (halDebugAssertContext(context))
  {
    /* print debug code header message */
    if (DEBUG_CODE_DECODE)
    {
      /* print debug header */
      if (context != SKIP_CONTEXT )printf("[debug]");
      
      /* print module context */
      switch(context & DEBUG_MODULE_MASK)
      {
        case DEBUG_HAL_I2C:     printf("[hal_i2c]");        break;
        case DEBUG_HAL_ACC:     printf("[hal_acc]");        break;
        case DEBUG_HAL_LUX:     printf("[hal_lux]");        break;
        case DEBUG_HAL_MAG:     printf("[hal_mag]");        break;
        case DEBUG_HAL_UVI:     printf("[hal_uvi]");        break;
        case DEBUG_HAL_AIR:     printf("[hal_air]");        break;
        case DEBUG_HAL_WEATHER: printf("[hal_weather]");    break;
        case DEBUG_HAL_EVENTS:  printf("[hal_events]");     break;
        case DEBUG_HAL_UART:    printf("[hal_uart]");       break;
        case DEBUG_HAL_PIR:     printf("[hal_pir]");        break;
        case DEBUG_GAP:         printf("[hal_gap]");        break;
        case DEBUG_GATT:        printf("[hal_gatt]");       break;
        case DEBUG_GATE:        printf("[hal_gate]");       break;
        case DEBUG_NET:         printf("[hal_net]");        break;
        case DEBUG_SENSOR:      printf("[hal_sensor]");     break;
        case DEBUG_DATALOGGER:  printf("[hal_datalogger]"); break;
        case DEBUG_ALARMS:      printf("[hal_alarm]");      break;
        default:                                            break;
      }      

      /* print message level */
      switch(context & DEBUG_LEVEL_MASK)
      {
        case DEBUG_ERROR:       printf("[error]");          break;
        case DEBUG_ASSERT:      printf("[assert]");         break;
        case DEBUG_INFO:        printf("[info]");           break;
        case DEBUG_DUMP:        printf("[dump]");           break;
        default:                                            break;
      }      
    }
    else
    {
      printf("[debug][0x%02x%02x%02x%02x]", 
            BREAK_UINT32(context,3),
            BREAK_UINT32(context,2),
            BREAK_UINT32(context,1),
            BREAK_UINT32(context,0)); 
    }
    
    /* print format string */
    va_list args;
    va_start(args, format_string);
    vprintf(format_string, args);
    va_end(args);
  }
}


/*******************************************************************************
 * @fn          halGetDebugContext
 * @brief       Get actual protected debug_context variable 
 * @param       none
 * @return      uint32: actual debug_contex
 *******************************************************************************
 */
uint32 halGetDebugContext(){return debug_context;}


/*******************************************************************************
 * @fn          halSetDebugContext
 * @brief       Set inruntime the protected debug_context variable
 * @param       context: new context to set 
 * @return      none
 *******************************************************************************
 */
void halSetDebugContext(uint32 context){debug_context = context;}


#else 
  /* dummy definitions */
  void halDebugMsg(uint32 context, char *format_string, ...){}
  bool halDebugAssertContext(uint32 context){return FALSE;}
  uint32 halGetDebugContext(){return DEBUG_NOTHING;}
  void halSetDebugContext(uint32 context){}
#endif // (defined HAL_DEBUG) && (HAL_DEBUG == TRUE)


/*******************************************************************************
*/   
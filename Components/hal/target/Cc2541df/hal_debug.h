/*******************************************************************************
  Filename:       hal_debug.h
  Revised:        $Date: 2017-04-12 11:22:22$
  Revision:       $Revision: 00001 $

  Description:    Debug context for Dragonfly.
*******************************************************************************/
#ifndef HAL_DEBUG_H
#define HAL_DEBUG_H

#ifdef __cplusplus
extern "C"
{
#endif
/* -----------------------------------------------------------------------------
 *                                 Includes
 * -----------------------------------------------------------------------------
 */
#include "hal_types.h"
#include "hal_board.h"
#include "stdio.h"

  
/* -----------------------------------------------------------------------------
 *                             Debug Definitions
 * -----------------------------------------------------------------------------
 */
#define DEBUG_NOTHING          0x00000000

/// debug I2C devices ////////////////////////////// enable only the used ///////
#define DEBUG_HAL_I2C          0x00000001
#define DEBUG_HAL_ACC          0x00000002
#define DEBUG_HAL_LUX          0x00000004
#define DEBUG_HAL_MAG          0x00000008
#define DEBUG_HAL_UVI          0x00000010
#define DEBUG_HAL_AIR          0x00000020
#define DEBUG_HAL_WEATHER      0x00000040

/// debug system devices /////////////////////////// enable only the used //////
#define DEBUG_HAL_QI           0x00001000
#define DEBUG_HAL_EVENTS       0x00002000
#define DEBUG_HAL_UART         0x00004000
#define DEBUG_HAL_PIR          0x00008000
  
/// debug modules ////////////////////////////////// enable only the used //////
#define DEBUG_GAP              0x00100000
#define DEBUG_GATT             0x00200000
#define DEBUG_GATE             0x00400000
#define DEBUG_NET              0x00800000
#define DEBUG_SENSOR           0x01000000
#define DEBUG_DATALOGGER       0x02000000
#define DEBUG_ALARMS           0x04000000

/// debug levels //////////////////////////// cascade messages upon level //////
#define DEBUG_ERROR            0x10000000 // [error] messages
#define DEBUG_ASSERT           0x20000000 // [assert]ions and system check
#define DEBUG_INFO             0x30000000 // [info]  system no asking messages 
#define DEBUG_DUMP             0x40000000 // [dumps] scans and multiline messages
  
#define SKIP_CONTEXT           0x70000000 // skip level and context assertion 

/// force UART mode ////////////////////////////////////////////////////////////
#define DEBUG_FORCE_UART       0x80000000 // force UART messages when radio on
  
/// masks //////////////////////////////////////////////////////////////////////
#define DEBUG_FORCE_MASK       0x80000000
#define DEBUG_LEVEL_MASK       0x70000000
#define DEBUG_MODULE_MASK      0x0FFFFFFF
#define DEBUG_MASK             0xFFFFFFFF

/// debug configuration ////////////////////////////////////////////////////////
#define DEFAULT_DEBUG_SYSTEM   DEBUG_NOTHING
   
            
/* -----------------------------------------------------------------------------
 *                                  Macros
 * -----------------------------------------------------------------------------
 */
/*
 *  HAL_DEBUG_ASSERT(expression)-The given expression must evaluate as "true" or 
 *  else the debug handler is called.  From here, the call stack feature of the 
 *  debugger can pinpoint where the problem occurred.
 *
 *  HAL_DEBUG_ERROR()-If debug is in use, immediately calls the debug handler.
 *
 *  HAL_DEBUG_STATEMENT(statement)- Inserts the given C statement but only when 
 *  debug are in use.  This macro allows debug code that is not part of a 
 *  expression.
 *
 *  HAL_DEBUG_DECLARATION(declaration)- Inserts the given C declaration but 
 *  only when debug is in use.  This macros allows debug code that is not part 
 *  of an expression.
 *
 *  Debug can be disabled for optimum performance and minimum code size.  
 *  To disable debug, define the preprocessor symbol HAL_DEBUG to FALSE in 
 *  the "hal_board_cf.h" file.
 */
#if (defined HAL_DEBUG) && (HAL_DEBUG == FALSE)
  #define HAL_DEBUG_ASSERT(expr)
  #define HAL_DEBUG_ERROR()
  #define HAL_DEBUG_STATEMENT(context, statement)
  #define HAL_DEBUG_DECLARATION(declaration)
#else
  #define HAL_DEBUG_ASSERT(expr)                  st(if (!( expr )) halDebugHandler();)
  #define HAL_DEBUG_ERROR()                       halDebugHandler()
  #define HAL_DEBUG_STATEMENT(context, statement) st(if(halDebugAssertContext(context)) \
                                                    statement)
  #define HAL_DEBUG_DECLARATION(declaration)      declaration
#endif
   

/* -----------------------------------------------------------------------------
 *                            Internal Prototypes
 * -----------------------------------------------------------------------------
 */
void halDebugHandler(void);
bool halDebugAssertContext(uint32 context);


/*******************************************************************************
 *                              FUNCTIONS - API
 ******************************************************************************/
/// inruntime debug context change /////////////////////////////////////////////
extern uint32 halGetDebugContext();
extern void halSetDebugContext(uint32 context);

/// inruntime debug context change /////////////////////////////////////////////
extern void halDebugMsg(uint32 context, char *format_string, ...);            
extern void halDebugErrorLights(void);


/*******************************************************************************
*/
#endif // HAL_DEBUG_H

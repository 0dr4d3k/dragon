/*******************************************************************************
  Filename:       hal_board_cfg.h
  Revised:        $Date: 2017-04-17 11:22:11$
  Revision:       $Revision: 00001$

  Description:    Board configuration for CC2541 Dragonfly.
*******************************************************************************/

/*******************************************************************************
TAREAS PENDIENTES: <- oJo
  - Generacion de imgA. Hay que meter directivas de preprocessor en la configura
cion del precompiler para permitir el cambio automatico al cambiar de proyecto.
*******************************************************************************/



#ifndef HAL_BOARD_CFG_H
#define HAL_BOARD_CFG_H

/*
 *     =============================================================
 *     |            Libelium Dragonfly CC2541 Sensor Board         |
 *     | --------------------------------------------------------- |
 *     |  mcu   : 8051 core                                        |
 *     |  clock : 32MHz                                            |
 *     =============================================================
 */

#include "hal_types.h"


#ifdef __cplusplus
extern "C"
{
#endif

/* -----------------------------------------------------------------------------
 *                                   Includes
 * -----------------------------------------------------------------------------
 */
/* -----------------------------------------------------------------------------
 *                             Board Indentifier
 *
 *      Define the Board Identifier to :
 *  (DEPRECATED)           -> HAL_BOARD_DRAGONFLY_TRAINER_MAGETOMETER 
 *  (DEPRECATED)           -> HAL_BOARD_DRAGONFLY_v7E_CORE
 *  (DEPRECATED)           -> HAL_BOARD_DRAGONFLY_v7E_PLANTS
 *  (DEPRECATED)           -> HAL_BOARD_DRAGONFLY_v7E_AIR
 *  (DEPRECATED)           -> HAL_BOARD_DRAGONFLY_v7E_THERMOSTAT
 *  (DEPRECATED)           -> HAL_BOARD_DRAGONFLY_v7E_UVI
 *  (DEPRECATED)           -> HAL_BOARD_DRAGONFLY_v7_PLANTS_TEST_PIR
 *  (DEPRECATED)           -> HAL_BOARD_DRAGONFLY_v7_PLANTS
 *  (DEPRECATED)           -> HAL_BOARD_DRAGONFLY_v7
 *                         -> HAL_BOARD_DRAGONFLY_MINIMAL           0x01
 *                         -> HAL_BOARD_DRAGONFLY_v8B_THERMOSTAT    0x02
 *                         -> HAL_BOARD_DRAGONFLY_v8B_PLANTS        0x03
 *                         -> HAL_BOARD_DRAGONFLY_v8B_ACTUATIR      0x04
 *                         -> HAL_BOARD_DRAGONFLY_v8B_LIGHT         0x05
 *                         -> HAL_BOARD_DRAGONFLY_v8B_AIR           0x06
 *                         -> HAL_BOARD_DRAGONFLY_v8B_GATEWAY       0x07
 * -----------------------------------------------------------------------------
 */
//// device selection //////////////////////////////////////////////////////////
#define CC2541
  
//// board selection ///////////////////////////////////////////////////////////
#define HAL_BOARD_DRAGONFLY_v8B_GATEWAY       0x01 
//#define HAL_BOARD_DRAGONFLY_v8B_ACTUATOR      0x02
//#define HAL_BOARD_DRAGONFLY_v8B_THERMOSTAT    0x03
//#define HAL_BOARD_DRAGONFLY_v8B_PLANTS        0x04
//#define HAL_BOARD_DRAGONFLY_v8B_LIGHT         0x05
//#define HAL_BOARD_DRAGONFLY_v8B_AIR           0x06
//#define HAL_BOARD_DRAGONFLY_MINIMAL           0x07
       
/* -----------------------------------------------------------------------------
 *                         Includes upon HAL_BOARD
 * -----------------------------------------------------------------------------
 */
/* -- hal board configuration for HAL_BOARD_DRAGONFLY_MINIMAL --------------- */ 
#if (defined HAL_BOARD_DRAGONFLY_MINIMAL)
  #include "hal_board_cfg_minimal.h"
#endif 

/* -- hal board configuration for HAL_BOARD_DRAGONFLY_v8B_THERMOSTAT -------- */ 
#if (defined HAL_BOARD_DRAGONFLY_v8B_THERMOSTAT)
  #include "hal_board_cfg_thermostat.h"
#endif 
  
/* -- hal board configuration for HAL_BOARD_DRAGONFLY_v8B_PLANTS ------------ */ 
#if (defined HAL_BOARD_DRAGONFLY_v8B_PLANTS)
  #include "hal_board_cfg_plants.h"
#endif 

/* -- hal board configuration for HAL_BOARD_DRAGONFLY_v8B_ACTUATOR ---------- */ 
#if (defined HAL_BOARD_DRAGONFLY_v8B_ACTUATOR)
  #include "hal_board_cfg_actuator.h"
#endif 

/* -- hal board configuration for HAL_BOARD_DRAGONFLY_v8B_LIGHT ------------- */ 
#if (defined HAL_BOARD_DRAGONFLY_v8B_LIGHT)
  #include "hal_board_cfg_light.h"
#endif 

/* -- hal board configuration for HAL_BOARD_DRAGONFLY_v8B_AIR --------------- */ 
#if (defined HAL_BOARD_DRAGONFLY_v8B_AIR)
  #include "hal_board_cfg_air.h"
#endif 

/* -- hal board configuration for HAL_BOARD_DRAGONFLY_v8B_GATEWAY ----------- */ 
#if (defined HAL_BOARD_DRAGONFLY_v8B_GATEWAY)
  #include "hal_board_cfg_gateway.h"
#endif 

   
/* -----------------------------------------------------------------------------
 *                       Common includes for all boards
 * -----------------------------------------------------------------------------
 */
#include "hal_board_cfg_common.h"  // common definitions for all Dragonflys

   
/* -----------------------------------------------------------------------------
 *                       Default Driver Configuration
 * -----------------------------------------------------------------------------
 */
/* default driver configuration defined in specific dragonfly hal_board_cfg_XX.h
 configuration file */   
  

/* -----------------------------------------------------------------------------
 *                           Debug Configuration
 * -----------------------------------------------------------------------------
 */
#define HAL_DEBUG              FALSE     // warning!, this generates extra code

#define DEBUG_OVER_TERMINAL    TRUE
#define DEBUG_OVER_UART        TRUE    // no implemented

#define DEBUG_CODE_DECODE      TRUE

//#define DEBUG_ASSERT_RESET    // debug assertion generate hardware reset
//#define DEBUG_ASSERT_WHILE    // debug assertion generate while forever and ever
#define DEBUG_ASSERT_LIGHTS   // debug assertion loops ligh flashes 

#define DEBUG_SYSTEM           /* I2C modules to debug                   */  \
                               DEBUG_HAL_I2C       |                         \
                               DEBUG_HAL_ACC       |                         \
                               DEBUG_HAL_LUX       |                         \
                               DEBUG_HAL_UVI       |                         \
                               DEBUG_HAL_AIR       |                         \
                               DEBUG_HAL_WEATHER   |                         \
                               /* system modules to debug                */  \
                               DEBUG_HAL_QI        |                         \
                               DEBUG_HAL_EVENTS    |                         \
                               DEBUG_HAL_UART      |                         \
                               DEBUG_HAL_PIR       |                         \
                               /* application modules to debug           */  \
                               DEBUG_GAP           |                         \
                               DEBUG_GATT          |                         \
                               DEBUG_GATE          |                         \
                               DEBUG_NET           |                         \
                               DEBUG_SENSOR        |                         \
                               DEBUG_DATALOGGER    |                         \
                               DEBUG_ALARMS        |                         \
                               /* select only the major level messages   */  \
                              /* DEBUG_ERRO        |                     */  \
                              /* DEBUG_INFO        |                     */  \
                              /* DEBUG_ASSER       |                     */  \
                               DEBUG_DUMP        /*                      */  
   
/*******************************************************************************
*/
#endif // HAL_BOARD_CFG_H                                 
                                 
                                 
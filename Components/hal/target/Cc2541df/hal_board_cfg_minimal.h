/*******************************************************************************
  Filename:       hal_board_cfg_minimal.h
  Revised:        $Date: 2017-02-23 11:32:02 $
  Revision:       $Revision: 00001 $

  Description:    Board configuration for CC2541 Dragonfly Minimal. 
*******************************************************************************/
#ifndef HAL_BOARD_CFG_MINIMAL_H
#define HAL_BOARD_CFG_MINIMAL_H

#ifdef __cplusplus
extern "C"
{
#endif
/* -----------------------------------------------------------------------------
 *                                   Includes
 * -----------------------------------------------------------------------------
 */
  
  
/* -----------------------------------------------------------------------------
 *                   Preprocesor Directives upon HAL_BOARD
 * -----------------------------------------------------------------------------
 */  
/* -- preprocesor directives for HAL_BOARD_DRAGONFLY_MINIMAL ---------------- */ 
/// common definitions /////////////////////////////////////////////////////////
#define HAL_IMAGE_B
#define OAD_BIM
#define OAD_KEEP_NV_PAGES
#define OAD_IMAGE_VERSION      0x0000 // see "oad_target.c"
#define INT_HEAP_LEN           3000
#define HALNODEBUG
#define OSAL_CBTIMER_NUM_TASKS 1
  
/// configuration definitions //////////////////////////////////////////////////
#define POWER_SAVING 
#define AUTO_ADVERTISING
#define xGATT_TI_UUID_128_BIT
#define xUSER_DESCRIPTION
#define xPLUS_BROADCASTER
#define xGAP_BOND 
#define xNV_MEMORY 
#define xBLINK_LEDS  
  
/// HAL definitions ////////////////////////////////////////////////////////////
/// system ....
#define HAL_HID                FALSE 
#define HAL_AES_DMA            TRUE
#define HAL_DMA                TRUE
#define HAL_I2C                FALSE
#define HAL_LED                FALSE
#define HAL_KEY                FALSE
#define HAL_ADC                FALSE
#define HAL_QI                 FALSE
#define HAL_ACC                FALSE
#define HAL_SUART              FALSE

/// uart ....          see extra uart configuration in "hal_board_cfg_gateway.h"
#define xSERIAL_INTERFACE   
#define HAL_UART               FALSE 
    
/// sensors ....
#define HAL_BUZZER             FALSE
#define HAL_WEATHER            FALSE
#define HAL_PIR                FALSE
#define HAL_UVI                FALSE
#define HAL_LUX                FALSE
#define HAL_AIR                FALSE
#define HAL_MAG                FALSE

/// GAP features ///////////////////////////////////////////////////////////////
#define FEATURE_OAD           // TI 0xFFC0 TI OAD Service (128b UUID)
                              // TI 0xCCC0 TI Connection Control Service
#define xFEATURE_DL           // Deprecated Service (oJo, eliminar)
#define xFEATURE_DATALOGGER   // LIB 0xCC00 Application Datalogger Service
#define xFEATURE_ALARM        // LIB 0xCC20 Application Alarm Service

#define xFEATURE_GATTINFO     // SIG 0x1800 Generic Access Service
#define xFEATURE_ATTRIBUTE    // SIG 0x1801 Generic Attibute Serrvice
#define xFEATURE_DEVINFO      // SIG 0x180A Dvice Information Service
#define xFEATURE_BATTERY      // SIG 0x180F Battery Service

#define xFEATURE_KEY          // LIB 0xAA00 System Events Service
#define xFEATURE_ACC          // LIB 0xAA10 Accelerometer Service
#define xFEATURE_ITEMP        // LIB 0xAA20 Internal Temperature Service 
#define xFEATURE_TEST         // LIB 0xAA60 System Test Service

#define xFEATURE_REGISTER     // Deprecated Service (oJo, eliminar)
#define xFEATURE_DIGITAL      // LIB 0xBB00 Sensor Digital IO Service
#define xFEATURE_ANALOG       // LIB 0xBB10 Sensor Analog IO Service
#define xFEATURE_WEATHER      // LIB 0xBB20 Sensor Weather Service
#define xFEATURE_LUXOMETER    // LIB 0xBB30 Sensor Luxometer Service
#define xFEATURE_AIR          // LIB 0xBB40 Sensor Air Quality Service
#define xFEATURE_UVI          // LIB 0xBB50 Sensor UVI Service
#define xFEATURE_MAGNETOMETER // LIB 0xBB60 Sensor Magnetometer Service
   
/// App features ///////////////////////////////////////////////////////////////
#define xDRAGONFLY_GATE       // LIB 0xDD00 Dragonfly Gate Service        
     
#endif

  
/* -----------------------------------------------------------------------------
 *                             Sensor Configuration
 * -----------------------------------------------------------------------------
 */
    

/*******************************************************************************
*/
#endif // HAL_BOARD_CFG_MINIMAL_H

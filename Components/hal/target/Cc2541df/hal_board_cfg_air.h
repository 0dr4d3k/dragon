/*******************************************************************************
  Filename:       hal_board_cfg_air.h
  Revised:        $Date: 2017-04-1 11:55:11 $
  Revision:       $Revision: 00001 $

  Description:    Board configuration for CC2541 Dragonfly Air. 
*******************************************************************************/
#ifndef HAL_BOARD_CFG_AIR_H
#define HAL_BOARD_CFG_AIR_H

#ifdef __cplusplus
extern "C"
{
#endif  
/* -----------------------------------------------------------------------------
 *                                   Includes
 * -----------------------------------------------------------------------------
 */
  
  
/* -----------------------------------------------------------------------------
 *                     Register Hal Board and Interruptions
 * -----------------------------------------------------------------------------
 */
#define REGISTER_HAL_BOARD  HAL_BOARD_DRAGONFLY_v8B_AIR
#define REGISTER_HAL_INT    HAL_INT1 | HAL_INT4

                                  // HAL_KEY = TRUE mandatory, then:  
#define HAL_INT1_EN         TRUE  // ACC
#define HAL_INT2_EN         FALSE // LUX/AIR oJo, no activada por el momento
#define HAL_INT3_EN         FALSE // nc
#define HAL_INT4_EN         TRUE  // Qi
#define HAL_INT5_EN         FALSE // nc
#define HAL_INT6_EN         FALSE // nc
#define HAL_INT7_EN         FALSE // nc
#define HAL_INT8_EN         FALSE // nc

 // keys handled in hal_keys.c, OnBoard.c, hal_interruption.c and hal_drivers.c
   
   
/* -----------------------------------------------------------------------------
 *                   Preprocesor Directives upon HAL_BOARD
 * -----------------------------------------------------------------------------
 */
/* -- precompiler directives for HAL_BOARD_DRAGONFLY_v8B_THERMOSTAT --------- */ 
/// common definitions /////////////////////////////////////////////////////////
#define HAL_IMAGE_B
#define OAD_BIM
#define OAD_KEEP_NV_PAGES
#define OAD_IMAGE_VERSION         0x0000 // see "oad_target.c"
#define INT_HEAP_LEN              3000
#define HALNODEBUG
#define OSAL_CBTIMER_NUM_TASKS    1
  
/// configuration definitions //////////////////////////////////////////////////
#define POWER_SAVING 
#define AUTO_ADVERTISING
#define xGATT_TI_UUID_128_BIT
#define xUSER_DESCRIPTION
#define xPLUS_BROADCASTER
#define xGAP_BOND 
#define NV_MEMORY 
#define BLINK_LEDS  
  
/// HAL definitions ////////////////////////////////////////////////////////////
/// system ....
#define HAL_HID                FALSE // used only in cc2540 USB HID
#define HAL_AES_DMA            TRUE  
#define HAL_DMA                TRUE  
#define HAL_I2C                TRUE
#define HAL_LED                TRUE  
#define HAL_KEY                TRUE  // oJo, interfiere con HAL_UART
#define HAL_ADC                TRUE  
#define HAL_QI                 TRUE
#define HAL_ACC                TRUE  
#define HAL_SUART              FALSE

/// uart ....          see extra uart configuration in "hal_board_cfg_gateway.h"
#define xSERIAL_INTERFACE   
#define HAL_UART               FALSE 
    
/// sensors ....
#define HAL_BUZZER             FALSE 
#define HAL_WEATHER            TRUE
#define HAL_PIR                FALSE  
#define HAL_UVI                FALSE 
#define HAL_LUX                FALSE  
#define HAL_AIR                TRUE 
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

#define xFEATURE_KEY           // LIB 0xAA00 System Events Service
#define xFEATURE_ACC          // LIB 0xAA10 Accelerometer Service
#define xFEATURE_ITEMP        // LIB 0xAA20 Internal Temperature Service 
#define xFEATURE_TEST         // LIB 0xAA60 System Test Service

#define xFEATURE_REGISTER     // Deprecated Service (oJo, eliminar)
#define xFEATURE_DIGITAL      // LIB 0xBB00 Sensor Digital IO Service
#define xFEATURE_ANALOG       // LIB 0xBB10 Sensor Analog IO Service
#define xFEATURE_WEATHER       // LIB 0xBB20 Sensor Weather Service
#define xFEATURE_LUXOMETER    // LIB 0xBB30 Sensor Luxometer Service
#define xFEATURE_AIR           // LIB 0xBB40 Sensor Air Quality Service
#define xFEATURE_UVI          // LIB 0xBB50 Sensor UVI Service
#define xFEATURE_MAGNETOMETER // LIB 0xBB60 Sensor Magnetometer Service
   
/// App features ///////////////////////////////////////////////////////////////
#define DRAGONFLY_GATE       // LIB 0xDD00 Dragonfly Gate Service     
  
  
/* -----------------------------------------------------------------------------
 *                             Sensor Configuration
 * -----------------------------------------------------------------------------
 */
//// PORT: WAKE pin configuration. (P1_0-OUTPUT)
  /* PORT: AIR WAKE pin. (P1_0-OUTPUT)*/
  #define HAL_AIR_WAKE_BV      BV(0)
  #define HAL_AIR_WAKE_PORT    P1
  #define HAL_AIR_WAKE_SBIT    P1_0
  #define HAL_AIR_WAKE_DDR     P1DIR
  #define HAL_AIR_WAKE_SEL     P1SEL
   
//  /* PORT: AIR interrupt INT. (P1_6-INPUT)*/
//  #define HAL_AIR_INT_BIT        BV(6)
//  #define HAL_AIR_INT_PORT       P1
//  #define HAL_AIR_INT_DIR        P1DIR
//  #define HAL_AIR_INT_SEL        P1SEL
//
//   /* AIR INT ENABLE: Interrupt option - Enable or disable */
//  #define HAL_AIR_INT_DISABLE    0x00
//  #define HAL_AIR_INT_ENABLE     0x01
//
//  /* INT ENABLE REG: CPU port interrupt */
//  #define HAL_AIR_CPU_PORT_1_IF P1IF
//
//  /* INT CFG PORT: CPU port interrupt */
//  #define HAL_AIR_INT_IEN      IEN2  /* CPU interrupt mask register */
//  #define HAL_AIR_INT_IENBIT   BV(4) /* Mask bit for all of Port_1 */
//  #define HAL_AIR_INT_ICTL     P1IEN /* Port Interrupt Control register */
//  #define HAL_AIR_INT_ICTLBIT  BV(6) /* P1IEN - P1.3 enable/disable bit */
//  #define HAL_AIR_INT_PXIFG    P1IFG /* Interrupt flag at source */
//
//  #define HAL_PIR_INT_EDGEBIT  BV(1)  /* In PICTL Port 1, inputs 3 to 0 interrupt 0(1)-rising(falling) edge configuration */

    
/*******************************************************************************
*/
#endif // HAL_BOARD_CFG_THERMOSTAT_H

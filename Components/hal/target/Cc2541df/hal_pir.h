/*******************************************************************************
  Filename:       hal_pir.h
  Revised:        $Date: 2016-03-18 11:45:31$
  Revision:       $Revision: 0 $

  Description:    This file contains the declaration of Execelitas PYD 1698 PIR. 
                  Pyro-electric IR detector abstraction layer.
*******************************************************************************/

#ifndef HAL_PIR_H
#define HAL_PIR_H

#ifdef __cplusplus
extern "C" {
#endif

/* -----------------------------------------------------------------------------
 *                                  Includes
 * -----------------------------------------------------------------------------
 */

#include "comdef.h"

/* -----------------------------------------------------------------------------
 *                                 Constants
 * -----------------------------------------------------------------------------
 */

/* -----------------------------------------------------------------------------
 *                          Execelitas PYD1698 PIR
 * -----------------------------------------------------------------------------
 */
#define factory_Mask   0x0000001F  // 4_bits (4:0)
#define factory_Param  0x00000010  // Must be 0x10

#define source_Mask    0x00000060  // 2_bits (6:5) 01100000
#define source_PIR_BFP 0x00000000  // Band Pass Filter
#define source_PIR_LPF 0x00000020  // Low Pass Filter
#define source_TEMP    0x00000060  // Temperature

#define mode_Mask      0x00000180  // 2_bits (8:7)
#define mode_Forced    0x00000000  // Forced Read Out
#define mode_14ms      0x00000080  // 14ms InterruptRead Out
#define mode_WakeUp    0x00000100  // WakeUp 

#define window_Mask    0x00000600  // 2_bits (10:9)
#define window_4s      0x00000000  // 4s 
#define window_8s      0x00000200  // 8s 
#define window_12s     0x00000400  // 12s 
#define window_16s     0x00000600  // 16s 

#define counter_Mask   0x00001800  // 2_bits (12:11)
#define counter_1p     0x00000000  // 1 pulse
#define counter_2p     0x00000800  // 2 pulse
#define counter_3p     0x00001000  // 3 pulse
#define counter_4p     0x00001800  // 4 pulse

// Bind_time = 0.5s + [bind_value] * 0.5s
#define blind_Mask     0x0001E000  // 4_bits (16:13) No motion detection for this time

// Threshold[%] = [thersold_value] / 255 *100
#define threshold_Mask 0x01FE0000  // 8_bits (24:17) WakeUp Sensibility 


/* -----------------------------------------------------------------------------
 *                                  Typedefs
 * -----------------------------------------------------------------------------
 */


/* -----------------------------------------------------------------------------
 *                                 Functions
 * -----------------------------------------------------------------------------
 */
void HalPirInit(void);
bool HalPirTest(void);

void HalProcessPirInterrupt(void);

bool HalPirGetData(void *pData, void *pCfg);
bool HalPirGetData16(uint8 *pData);
bool HalPirPutCfg(uint32 value);

/*******************************************************************************
*/

#ifdef __cplusplus
};
#endif

#endif

/*******************************************************************************
*/

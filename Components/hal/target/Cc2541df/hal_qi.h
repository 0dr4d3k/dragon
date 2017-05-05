/*******************************************************************************
  Filename:       hal_qi.h
  Revised:        $Date: 2016-05-24 13:10:31$
  Revision:       $Revision: 0 $

  Description:    This file contains the declaration of Texas Instrument bq51051B
                  High-Efficiency Qi v1_1-Compilant Wireless Power Receiver and 
                  Battery Charger driver for DragonFly devices.
*******************************************************************************/

#ifndef HAL_QI_H
#define HAL_QI_H

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
 *                  Texas Instrument bq51051B Qi Charger
 * -----------------------------------------------------------------------------
 */

/* QI INT CONTROL: Interrupt option - Enable or disable */
#define QI_INT_DISABLE    0x00
#define QI_INT_ENABLE     0x01

/* QI TS_CTRL: Charger emissor comunication */
// Put to Imput Tristate in normal mode
#define QI_TS_CTRL_FAULT    0x00
#define QI_TS_CTRL_FINISH   0x01

// Qi Status
#define QI_SATE_IDLE        0x00
#define QI_SATE_CHARGING    0x01
#define QI_SATE_FAULT       0x02
#define QI_SATE_CHARGED     0x04
#define QI_SATE_PLUGGED     0x08


/* -----------------------------------------------------------------------------
 *                                  Typedefs
 * -----------------------------------------------------------------------------
 */


/* -----------------------------------------------------------------------------
 *                                 Functions
 * -----------------------------------------------------------------------------
 */
void HalQiInit(void);
bool HalQiTest(void);

void HalProcessQiInterrupt(void);

uint8 HalQiGetState(void);
bool HalQiGetBattery(uint8 *pData);

/*******************************************************************************
*/

#ifdef __cplusplus
};
#endif

#endif

/*******************************************************************************
*/

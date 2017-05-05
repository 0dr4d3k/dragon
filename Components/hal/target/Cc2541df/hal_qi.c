/*******************************************************************************
  Filename:       hal_qi.c
  Revised:        $Date: 2016-05-24 13:10:31$
  Revision:       $Revision: 0 $

  Description:    This file contains the declaration of Texas Instrument bq51051B
                  High-Efficiency Qi v1_1-Compilant Wireless Power Receiver and 
                  Battery Charger driver for DragonFly devices.
*******************************************************************************/

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */

#include "hal_qi.h"
#include "hal_sensor.h"
#include "hal_board.h"

#if (defined HAL_QI) && (HAL_QI == TRUE)

/* ------------------------------------------------------------------------------------------------
 *                                           Constants
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Typedefs
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Macros
 * ------------------------------------------------------------------------------------------------
 */

/* ------------------------------------------------------------------------------------------------
 *                                           Local Functions
 * ------------------------------------------------------------------------------------------------
 */
 
//static void HalI2CAirSelect(void);



/* ------------------------------------------------------------------------------------------------
 *                                           Local Variables
 * ------------------------------------------------------------------------------------------------
 */

uint8 qi_state;



/**************************************************************************************************
 * @fn          HalQiInit
 *
 * @brief       This function initializes the Texas Instrument bq51051B Qi Charger.
 *
 * @return      None.
 */

void HalQiInit(void)
{

  /* Initialize GPIO Port */
/* ----P0_6-CRGn signal-Input-Pull-UP-Interrupt falling edge-------------------------------------*/
  HAL_QI_INT_SEL &= (uint8)~(HAL_QI_INT_BIT);    /* Set pin function to GPIO */
  HAL_QI_INT_DIR &= (uint8)~(HAL_QI_INT_BIT);    /* Set pin direction to Input */

/* ----P0_5-TS_CTRL signal-Output/(Input-Tristate)-------------------------------------------------------*/
  HAL_QI_CTRL_SEL &= (uint8)~(HAL_QI_INT_BIT);    /* Set pin function to GPIO */
  HAL_QI_CTRL_DIR |= (uint8)(HAL_QI_INT_BIT);     /* Set pin direction to Output */

  
  /* Configure QI Interruption */
// oJo configurado en Key
//  HAL_QI_INT_PICTL |= (HAL_QI_INT_EDGEBIT);   /* Set the edge bit to set falling edge to give int */
//
//  HAL_QI_INT_ICTL |= HAL_QI_INT_ICTLBIT;  /* enable interrupt generation at port */
//  HAL_QI_INT_IEN  |= HAL_QI_INT_IENBIT;   /* enable CPU interrupt */
//  HAL_QI_INT_PXIFG = (uint8)~(HAL_QI_INT_IEN);  /* Clear any pending interrupt. Clear by w0 */
}

/**************************************************************************************************
 * @fn          HalQiTest
 *
 * @brief       Run a device self-test
 *
 * @return      TRUE if passed, FALSE if failed
 */
bool HalQiTest(void)
{
  return TRUE;
}

/**************************************************************************************************
 * @fn          HalProcessQiInterrupt
 *
 * @brief       Process the trigger interruption
 *
 * @return      none
 */
void HalProcessQiInterrupt(void)
{

  bool valid=FALSE;

  if( HAL_QI_INT_PXIFG & HAL_QI_INT_BIT) /* Interrupt Flag has been set by QI charger */
  {
    HAL_QI_INT_PXIFG = (uint8)~(HAL_QI_INT_BIT); /* Clear Interrupt Flag */
    valid = TRUE;
  }

  if (valid)
  {
      // Implement debounce if necessary
//    osal_start_timerEx (Hal_TaskID, HAL_KEY_EVENT, HAL_KEY_DEBOUNCE_VALUE);
//    Poner aqui lo que quiero que haga cuando salte la interupcion

    //Disable QI interrupt while reading
    HAL_QI_INT_ICTL &= ~HAL_QI_INT_ICTLBIT;  /* disable interrupt generation at port */
    HAL_QI_INT_IEN  &= ~HAL_QI_INT_IENBIT;   /* disable CPU interrupt */

    qi_state = QI_SATE_PLUGGED;
    //generate evet to handle in main loop. OjO hacer un evento directo
//    osal_start_timerEx (Hal_TaskID, HAL_QI_EVENT, HAL_QI_VALUE);


    //Enable QI interrupt
    HAL_QI_INT_ICTL |= HAL_QI_INT_ICTLBIT;  /* enable interrupt generation at port */
    HAL_QI_INT_IEN  |= HAL_QI_INT_IENBIT;   /* enable CPU interrupt */

  }

}

/**************************************************************************************************
 * @fn          HalQiGetState
 *
 * @brief       Gets the current state of the Meal Automate
 *
 * @param       None
 * 
 * @return      Current State
 */
uint8 HalQiGetState(void)
{
  return QI_SATE_IDLE;
}

/**************************************************************************************************
 * @fn          HalQiGetBattery
 *
 * @brief       Get battery level while charging 
 *
 * @param       pData - pointer to buffer to place data
 * 
 * @return      TRUE if passed, FALSE if failed
 */
bool HalQiGetBattery(uint8 *pData)
{

  uint16 data_value=0;



  pData[0]=data_value & 0x00FF; //LSB
  pData[1]=(data_value>>=8)& 0x00FF; //MSB
  
  return TRUE;
}

/* ------------------------------------------------------------------------------------------------
 *                                           Private functions
 * -------------------------------------------------------------------------------------------------
 */


/*************************************************************************************************/

#endif // (defined HAL_PIR) && (HAL_PIR == TRUE)

/**************************************************************************************************
  Filename:       hal_pir.h
  Revised:        $Date: 2016-03-18 11:45:31$
  Revision:       $Revision: 0 $

  Description:    This file contains the declaration of Execelitas PYD 1698 PIR. Pyro-electric IR 
                  detector abstraction layer.
**************************************************************************************************/

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */

#include "hal_board.h"
#include "hal_pir.h"
#include "hal_sensor.h"

#if (defined HAL_PIR) && (HAL_PIR == TRUE)

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





/**************************************************************************************************
 * @fn          HalPirInit
 *
 * @brief       This function initializes the HAL PYD1698 PIR  Pyro-electric IR detector.
 *
 * @return      None.
 */

void HalPirInit(void)
{
  uint32 configuration = 0x00000000;

  /* Initialize GPIO Port */
  HAL_PIR_DIRECTLINK_SEL &= (uint8)~(HAL_PIR_DIRECTLINK_BIT);    // Set pin function to GPIO 

  /*If in WakeUp it is necessary delete any pending interruption in DirectLink */
  HAL_PIR_DIRECTLINK_DIR |= (uint8) (HAL_PIR_DIRECTLINK_BIT);    // Configure DL as Output 
  HAL_PIR_DIRECTLINK_SBIT =0;                                    // Set DIRECTLINK_PIN = Low, duration must be > 200 ns (tL) 
  ST_HAL_DELAY(1);                                               // delay ~8us tS
  HAL_PIR_DIRECTLINK_DIR &= (uint8)~(HAL_PIR_DIRECTLINK_BIT);    // Configure DIRECTLINK_PIN as Input 

  /* Configure PIR Interruption */
// oJo configurado en Key
//  PICTL &= ~(HAL_PIR_DIRECTLINK_EDGEBIT);                        // Clear the edge bit to set rising edge to give int 
//
//  HAL_PIR_DIRECTLINK_ICTL |= HAL_PIR_DIRECTLINK_ICTLBIT;         // enable interrupt generation at port 
//  HAL_PIR_DIRECTLINK_PXIFG = (uint8)~(HAL_PIR_DIRECTLINK_BIT);   // Clear any pending interrupt. Clear by w0 
//  HAL_PIR_DIRECTLINK_IEN  |= HAL_PIR_DIRECTLINK_IENBIT;          // enable CPU interrupt 
  
  /* Configure PIR */  
  static uint32 blind_Value = 0x3;      // The blind time is [blind_Value] * 0.5s
  static uint32 threshold_Value = 0x80; // Threshold [%] = [threshold_Value] / 255 * 100
  configuration = 
   ((configuration & ~factory_Mask)   | factory_Param)        |
   ((configuration & ~source_Mask)    | source_PIR_BFP)       |
   ((configuration & ~mode_Mask)      | mode_WakeUp)          |
   ((configuration & ~window_Mask)    | window_8s)            |
   ((configuration & ~counter_Mask)   | counter_2p)           |
   ((configuration & ~blind_Mask)     | blind_Value << 13)    |  // Revisar estos desplazamientos
   ((configuration & ~threshold_Mask) | threshold_Value << 17);  // Revisar estos desplazamientos

  HalPirPutCfg((uint32)configuration);  
  

/*The Wake Up Operation Mode is inactive and positive edges on D/L aren’t generated.
The output of the PIR signal is only low pass filtered. The host microcontroller can 
initiate the readout anytime.*/
//  HalPirPutCfg((uint32)0x00000030);  
  
/*The Wake Up Operation Mode is inactive and positive edges on D/L aren’t generated.
The output of the PIR signal is band pass filtered. The host microcontroller can initiate 
the readout anytime.*/
//  HalPirPutCfg((uint32)0x00000010);  
  
/*The Wake Up Operation Mode is active and the LowPower DigiPyro generates a positive
edge on D/L when the PIR signal exceeds the threshold of 9.4% twice within 12 sec.
Another positive edge will not be generated for 1.5 sec. after D/L was set to Low and
being released (input of host microcontroller is high impedance).*/
//original  HalPirPutCfg((uint32)0x00306D10); //0b0000000-00011000-0011-01-10-10-00-10000 
//  HalPirPutCfg((uint32)0x00106D10); //0b0000000-00001000-0011-01-10-10-00-10000 

/*  The Wake Up Operation Mode is active and the LowPower DigiPyro generates a positive
edge on D/L when the PIR signal exceeds the threshold of 37.6% three times within 8
sec. Another positive edge will not be generated for 0.5 sec. after D/L was set to Low and
being released (input of host microcontroller is high impedance). */
  //  HalPirPutCfg((uint32)0x00C03310); //0b0000000-01100000-0001-10-01-10-00-10000 
}

/**************************************************************************************************
 * @fn          HalPirTest
 *
 * @brief       Run a sensor self-test
 *
 * @return      TRUE if passed, FALSE if failed
 */
bool HalPirTest(void)
{
  return TRUE;
}

/**************************************************************************************************
 * @fn          HalProcessPirInterrupt
 *
 * @brief       Process the trigger interruption
 *
 * @return      none
 */
void HalProcessPirInterrupt(void)
{

  bool valid=FALSE;
  uint8 pData[2] = {0x00, 0x00};

  if( HAL_PIR_DIRECTLINK_PXIFG & HAL_PIR_DIRECTLINK_BIT) /* Interrupt Flag has been set by PIR sensor */
  {
    HAL_PIR_DIRECTLINK_PXIFG = (uint8)~(HAL_PIR_DIRECTLINK_BIT); /* Clear Interrupt Flag */
    valid = TRUE;
  }

  if (valid)
  {
      // Implement debounce if necessary
//    osal_start_timerEx (Hal_TaskID, HAL_KEY_EVENT, HAL_KEY_DEBOUNCE_VALUE);
//    Poner aqui lo que quiero que haga cuando salte la interupcion

    //Disable PIR interrupt while reading
    HAL_PIR_DIRECTLINK_ICTL &= ~HAL_PIR_DIRECTLINK_ICTLBIT;  /* disable interrupt generation at port */
    HAL_PIR_DIRECTLINK_IEN  &= ~HAL_PIR_DIRECTLINK_IENBIT;   /* disable CPU interrupt */

    HalPirGetData16((uint8 *)&pData);

    //Enable PIR interrupt
    HAL_PIR_DIRECTLINK_ICTL |= HAL_PIR_DIRECTLINK_ICTLBIT;  /* enable interrupt generation at port */
    HAL_PIR_DIRECTLINK_IEN  |= HAL_PIR_DIRECTLINK_IENBIT;   /* enable CPU interrupt */

  }

}

/**************************************************************************************************
 * @fn          HalPirGetData
 *
 * @brief       Get all data (40_bits) from PIR sensor: data (39:25) 15_bits & cfg (24:0) 25_bits
 *
 * @param       pData - pointer to buffer to place data
 *              pCfg - pointer to buffer to place configuration register
 * 
 * @return      TRUE if passed, FALSE if failed
 */
bool HalPirGetData(void *pData, void *pCfg)
{
  return TRUE;
}

/**************************************************************************************************
 * @fn          HalPirGetData
 *
 * @brief       Get data from PIR sensor: data (39:25) 15_bits 
 *
 * @param       pData - pointer to buffer to place data
 * 
 * @return      TRUE if passed, FALSE if failed
 */
bool HalPirGetData16(uint8 *pData)
{

  uint8 i;
  uint16 data_value=0;


   /* Initialize GPIO Port as Input */
  HAL_PIR_DIRECTLINK_SEL &= (uint8)~(HAL_PIR_DIRECTLINK_BIT);    /* Set pin function to GPIO */
  HAL_PIR_DIRECTLINK_DIR &= (uint8)~(HAL_PIR_DIRECTLINK_BIT);    /* Set pin direction to Iutput */

  while (HAL_PIR_DIRECTLINK_SBIT == 0); //Wait for DPDL=HIGH
  ST_HAL_DELAY(5);         // delay ~40us tS

  for (i = 0; i < 15; i++)
  {
    // create low to high transition
    HAL_PIR_DIRECTLINK_DIR |= (uint8) (HAL_PIR_DIRECTLINK_BIT);    // Configure DL as Output
    HAL_PIR_DIRECTLINK_SBIT = 0;                                   // Set DIRECTLINK_PIN = Low, duration must be > 200 ns (tL)
    asm("nop");                                                    // number of nop depedant processor speed (200ns min.)
    HAL_PIR_DIRECTLINK_SBIT = 0;                                   // Set DIRECTLINK_PIN = High, duration must be > 200 ns (tH)
    asm("nop");
    HAL_PIR_DIRECTLINK_DIR &= (uint8)~(HAL_PIR_DIRECTLINK_BIT);    // Configure DIRECTLINK_PIN as Input
    ST_HAL_DELAY(1);                                               // delay ~8us. Wait for stable low signal

    data_value <<= 1;
    if (HAL_PIR_DIRECTLINK_SBIT == 1) data_value++; //Sample bit

  }

  HAL_PIR_DIRECTLINK_DIR |= (uint8) (HAL_PIR_DIRECTLINK_BIT);      // Configure DL as Output
  HAL_PIR_DIRECTLINK_SBIT = 0;                                     // Set DIRECTLINK_PIN = Low, duration must be > 200 ns (tL)
  HAL_PIR_DIRECTLINK_DIR &= (uint8)~(HAL_PIR_DIRECTLINK_BIT);      // Configure DIRECTLINK_PIN as Input

  pData[0]=data_value & 0x00FF; //LSB
  pData[1]=(data_value>>=8)& 0x00FF; //MSB
  
  return TRUE;
}

/**************************************************************************************************
 * @fn          HalPirPutCfg
 *
 * @brief       Put data into PIR sensor configuration register: cfg (24:0) 25_bits
 *
 * @param       value - data to place in configuation register.
 * 
 * @return      TRUE if passed, FALSE if failed
 */
bool HalPirPutCfg(uint32 value)
{
  uint8 i;
  uint8 nextbit;
  uint32 regmask = 0x01000000;

//  noInterrupts();

   /* Initialize GPIO Port */
  HAL_PIR_SERIN_SEL &= (uint8)~(HAL_PIR_SERIN_BIT);    /* Set pin function to GPIO */
  HAL_PIR_SERIN_DIR |= (uint8) (HAL_PIR_SERIN_BIT);    /* Set pin direction to Output */
  HAL_PIR_SERIN_SBIT = 0; 
  ST_HAL_DELAY(5); // delay ~40us tSL

  for (i = 0; i < 25; i++) 
  {
    nextbit = (value & regmask) != 0;
    regmask >>= 1;
    HAL_PIR_SERIN_SBIT = 0; 
    HAL_PIR_SERIN_SBIT = 1;
    HAL_PIR_SERIN_SBIT = nextbit;
    ST_HAL_DELAY(13); // delay ~100us tSHD
  }
  HAL_PIR_SERIN_SBIT = 0; 
  ST_HAL_DELAY(75); // delay ~600us tSLT
    
  return TRUE;
}
/* ------------------------------------------------------------------------------------------------
*                                           Private functions
* -------------------------------------------------------------------------------------------------
*/


/*************************************************************************************************/
#endif // (defined HAL_PIR) && (HAL_PIR == TRUE)

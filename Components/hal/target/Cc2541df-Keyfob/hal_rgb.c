/***********************************************************************************
* INCLUDES
*/

//#include <hal_types.h>
// Include Name definitions of individual bits and bit-fields in the CC254x device registers.
#include <ioCC254x_bitdef.h>
// Include device specific file
#include "ioCC2541.h"

#include "hal_rgb.h"
#include "hal_led.h"

#include "OnBoard.h"
#include <math.h>


/*********************************************************************
 * LOCAL VARIABLES
 */

static int red_current = 0x00;
static int red_target = 0x00; // de (0 a 15) 

static int green_current = 0x00;
static int green_target = 0x00; // de (0 a 15) 

static int blue_current = 0x00;
static int blue_target = 0x00; // de (0 a 15) 

//void LedSetRed(unsigned char duty_cycle);
//void LedSetGreen(unsigned char duty_cycle);
//void LedSetBlue(unsigned char duty_cycle);


/***************************************************************************************************
 * @fn      HalRGBInit
 *
 * @brief   Initialize LED Service
 *
 * @param   init - pointer to void that contains the initialized value
 *
 * @return  None
 ***************************************************************************************************/
void HalRGBInit (void)
{
  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1 | HAL_LED_2 | HAL_LED_3), HAL_LED_MODE_OFF );

  P0SEL = 0; // Configure Port 0 as GPIO
  P1SEL = 0; // Configure Port 1 as GPIO
  P2SEL = 0; // Configure Port 2 as GPIO

  P0DIR = 0xFF; // All port 0 pins (P0.0-P0.7) as output
  P1DIR = 0xFF; // All port 1 pins (P1.0-P1.7) as output
  P2DIR = 0x1F; // All port 2 pins (P2.0-P2.4) as output

  P0 = 0x07;   // Leds off
  P1 = 0;   // All pins on port 1 to low
  P2 = 0;   // All pins on port 2 to low
}


/***************************************************************************************************
 * @fn      HalRGBTest
 *
 * @brief   Test RGB Service
 *
 * @return  None
 ***************************************************************************************************/
void HalRGBTest (void)
{
  red_target++;
  if( red_target >= 15 )
  {
     red_target=0;
     green_target++;
  }
  if( green_target >= 15 )
  {
     red_target=0;
     green_target=0;
     blue_target++;
  }
  if( blue_target >= 15 )
  {
     red_target=0;
     green_target=0;
     blue_target=0;
  }
}

/***************************************************************************************************
 * @fn      HalRGBSet
 *
 * @brief   Set RGB Led
 *
 * @return  None
 ***************************************************************************************************/

void HalRGBSet(RGB_t led, unsigned char value)
{  
  if (led == LED_RED) {
    red_target = value;
//    red_speed = speed;
    
  } else if (led == LED_GREEN) {
    green_target = value;
//    green_speed = speed;   
    
  } else if (led == LED_BLUE) {
    blue_target = value;
//    blue_speed = speed;    
    
  }  
  HalRGBUpdate();
}



/***************************************************************************************************
 * @fn      HalRGBUpdate
 *
 * @brief   Update RGB Led
 *
 * @return  None
 ***************************************************************************************************/

void HalRGBUpdate(void) {      

  //Red Duty Clicle
  if ((red_target==0)||(red_current>red_target))
  {
      P0 |= 0x02; //OFF
  }
  else
  {
      P0 &= ~0x02; //ON   
  }
  //Green Duty Clicle
  if ((green_target == 0)||(green_current > green_target))
  {
      P0 |= 0x04; //OFF
  }
  else
  {
      P0 &= ~0x04; //ON   
  }
  //Blue Duty Clicle
  if ((blue_target==0)||(blue_current>blue_target))
  {
      P0 |= 0x01; //OFF
  }
  else
  {
      P0 &= ~0x01; //ON   
  }
  
  if( red_current >= 15 ) red_current = 0;
  else red_current++;
  if( green_current >= 15 ) green_current = 0;
  else red_current++;
  if( green_current >= 15 ) green_current = 0;
  else green_current++;
  if( blue_current >= 15 ) blue_current = 0;
  else blue_current++;
}


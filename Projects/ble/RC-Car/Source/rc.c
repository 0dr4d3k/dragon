/**************************************************************************************************
  Filename:       rc.c
  Revised:        $Date: 2012-07-06 $
  Revision:       $Revision: 1 $

  Description:    This file contains rc control interfacing for the TI BLE Car application
**************************************************************************************************/

#include <ioCC2540.h>
#include "rc.h"
#include "hci.h"
#include "OSAL_PwrMgr.h"
//***********************************************************************************
// Defines

/** \brief Timer steps corresponding to 1.5 ms pulse */
#define RC_CENTER                375

/** \brief Maximum number of timer steps for throttle pulse*/
#define RC_THROTTLE_MAX_STEPS    415
/** \brief Minimum number of timer steps for throttle pulse*/
#define RC_THROTTLE_MIN_STEPS    330
/** \brief Multiplier for throttle change*/
#define RC_THROTTLE_ACCELERATION 1/3

/** \brief Maximum number of timer steps for steering pulse*/
#define RC_STEERING_MAX_STEPS    500
/** \brief Minimum number of timer steps for steering pulse*/
#define RC_STEERING_MIN_STEPS    250
/** \brief Multiplier for steering change*/
#define RC_STEERING_ACCELERATION 1/5

//***********************************************************************************

/** \brief	Initialize RC control
*
* This will initialize the RC control.
* Timer 1 channel 0, 1 & 2 are used for timing.
* Ports P0_3 and P0_4 are output to RC servos
*
*/
void rcInit(void)
{
    // We will use Timer 1 Channel 0, 1 & 2 for timings
    // Ch1 for P0_3
    // Ch2 for P0_4
    
    // Configure IO's
#if defined ( CC2540_CAR )
    P0DIR |= 0x1C; //0x18;              // Data direction OUT for the PWM pins
    P0SEL |= 0x1C; //0x18;               // Choose peripheral mode for PWM pins
    PERCFG |= 0x03;             // Move USART1&2 to alternate2 location so that T1 is visible
#elif defined ( CC2540_MINIDK )
    // Keyfob testing /w LED!
    P1DIR |= 0x03;
    P1SEL |= 0x03;
    PERCFG |= 0x40;             //Move timer1 to LED pins
#endif
    
    // Initialize Timer 1
    T1CTL = 0x0C;               // Div = 128, CLR, MODE = Suspended
    T1CCTL0 = 0x4C;            // IM = 1, CMP = Clear output on compare; Mode = Compare
    T1CC0H = 0x02;              // Ticks = 600 (2.4ms)
    T1CC0L = 0x58;              // Ticks = 600 (2.4ms)
    T1CCTL1 = 0x0C;             // IM = 0; CMP = Clear output on compare; Mode = Compare
    T1CCTL2 = 0x0C;             // IM = 0; CMP = Clear output on compare; Mode = Compare
    T1CNTL = 0;                 // Reset timer to 0;

    IEN1 |= 0x02;               // Enable T1 cpu interrupt

    T1CC1H = 0x01;              // Ticks = 375 (1,5ms initial duty cycle)
    T1CC1L = 0x77;
    T1CC2H = 0x01;              // Ticks = 375 (1,5ms initial duty cycle)
    T1CC2L = 0x77;

}

/** \brief	Send RC control pulse
*
* This will send an RC control pulse.
* Value of 0 gives a pulse of 1.5ms
* Value of -50 gives a pulse of RC_CAR_***_MIN_STEPS
* Value of 127 gives a pulse of RC_CAR_***_MAX_STEPS
* Step size = 0.004ms
*
* \param[in]       throttle
*     Pulse to send on ch1, range -50>50 (int8)
* \param[in]       steering
*     Pulse to send on ch2, range -50>50 (int8)
*/

void rcPulse(int8 throttle, int8 steering)
{
  // Standstill and straight are at 1.5ms = 375 steps = 0x0177 
  static uint16 throttle_steps = RC_CENTER;
  static uint16 steering_steps = RC_CENTER;
  static bool braking = FALSE;
  static uint16 brake_duration = 0;
  int16 delta;
  
  
  // TODO: Generalize?
  
  // Throttle
  
  // Do a mean estimation: mean += eta * (sample - mean)
  delta = (RC_CENTER + (int16)(2 * throttle)) - throttle_steps;
  
  
  // Handle braking behaviour when trying to reverse with some ESCs
  if ( (throttle_steps >= RC_CENTER && throttle < 0) ||
      braking == TRUE )
  {
    braking = TRUE;
    brake_duration++;
    
    //Check that we are still braking
    if (throttle < 0)
    {
      //After a short while, increase the throttle to stop braking and enable reverse
      if (brake_duration > 60)
      {
        delta = RC_CENTER - throttle_steps;
      }
      //Reverse enabled, return to normal state
      if (brake_duration > 70)
      {
        braking = FALSE;
        brake_duration = 0;
      }
    }
    else
    {
      braking = FALSE;
      brake_duration = 0;
    }
  }
  
  throttle_steps += delta * RC_THROTTLE_ACCELERATION;
  
  
  if (throttle_steps > RC_THROTTLE_MAX_STEPS)
  {
    throttle_steps = RC_THROTTLE_MAX_STEPS;
  }
  else if (throttle_steps < RC_THROTTLE_MIN_STEPS)
  {
    throttle_steps = RC_THROTTLE_MIN_STEPS;
  }
  
  // Steering
  
  // Do a mean estimation: mean += eta * (sample - mean)
  delta = (0x0177 + (int16)(2 * steering)) - steering_steps;
  steering_steps += delta * RC_STEERING_ACCELERATION;
  
  if (steering_steps > RC_STEERING_MAX_STEPS)
  {
    steering_steps = RC_STEERING_MAX_STEPS;
  }    
  else if (steering_steps < RC_STEERING_MIN_STEPS)
  {
    steering_steps = RC_STEERING_MIN_STEPS;
  }

  // Set up the timer registers
  T1CC1L = (uint8)steering_steps;
  T1CC1H = (uint8)(steering_steps >> 8);
  T1CC2L = (uint8)throttle_steps;
  T1CC2H = (uint8)(throttle_steps >> 8);
  
  // Make sure timer will count continously
  HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_DISABLE_CLK_DIVIDE_ON_HALT );

  #if defined ( POWER_SAVING )
    osal_pwrmgr_device( PWRMGR_ALWAYS_ON );
  #endif
    
  // Reset timer
  T1CNTL = 0;
  
  // Start timer in modulo mode.
  T1CTL |= 0x02;

}


/** \brief	ISR to reset timer after RC control pulse
*
* 
*
*/

#pragma register_bank=2
#pragma vector = T1_VECTOR
__interrupt void rcISR (void) {
    uint8 flags = T1STAT;
    // T1 ch 0
    if (flags & 0x01){          
      
      // Stop Timer 1
      T1CTL ^= 0x02;
      
      //Allow clock division and re-enable power saving until next pulse
      HCI_EXT_ClkDivOnHaltCmd( HCI_EXT_ENABLE_CLK_DIVIDE_ON_HALT );
      
      #if defined ( POWER_SAVING )
        osal_pwrmgr_device( PWRMGR_BATTERY );
      #endif
      
    }
    T1STAT = ~ flags;
}


/*
+------------------------------------------------------------------------------
|  Copyright 2004-2007 Texas Instruments Incorporated. All rights reserved.
|
|  IMPORTANT: Your use of this Software is limited to those specific rights
|  granted under the terms of a software license agreement between the user who
|  downloaded the software, his/her employer (which must be your employer) and
|  Texas Instruments Incorporated (the "License"). You may not use this Software
|  unless you agree to abide by the terms of the License. The License limits
|  your use, and you acknowledge, that the Software may not be modified, copied
|  or distributed unless embedded on a Texas Instruments microcontroller or used
|  solely and exclusively in conjunction with a Texas Instruments radio
|  frequency transceiver, which is integrated into your product. Other than for
|  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
|  works of, modify, distribute, perform, display or sell this Software and/or
|  its documentation for any purpose.
|
|  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
|  PROVIDED “AS IS” WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
|  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
|  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
|  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
|  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
|  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING
|  BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
|  CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
|  SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
|  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
|
|  Should you have any questions regarding your right to use this Software,
|  contact Texas Instruments Incorporated at www.TI.com.
|
+------------------------------------------------------------------------------
*/

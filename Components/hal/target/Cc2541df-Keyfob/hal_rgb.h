/******************************************************************************
    Copyright (C) Studio Sophisti 2013
    Author: Tijn Kooijmans, tijn@studiosophisti.nl
******************************************************************************/

#ifndef HAL_RGB_H
#define HAL_RGB_H

//#include "hal_rgb.h"

/*===========================================================================
 * TYPES
 *=========================================================================*/

typedef enum
{
    LED_RED   = 1,
    LED_GREEN = 2,
    LED_BLUE  = 4
} RGB_t;

extern void HalRGBInit(void);
extern void HalRGBUpdate(void);
extern void HalRGBSet(RGB_t led, unsigned char value);
extern void HalRGBTest(void);
extern void HalRGBUpdate(void);

#endif


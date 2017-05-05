/**************************************************************************//**
    @file       hal_mag.h

    @brief      Header file for magnetometer LIS3MDL.
******************************************************************************/
#ifndef HAL_MAG_H
#define HAL_MAG_H


/******************************************************************************
 * INCLUDES
 */
#include "hal_types.h"
#include "comdef.h"


/******************************************************************************
 * DEFINES
 */
// 7-bit number for the address, and sets the last bit correctly based on 
// reads and writes
#define LIS3MDL_SA1_HIGH_ADDRESS  0b0011110 // 0x1E
#define LIS3MDL_SA1_LOW_ADDRESS   0b0011100 // 0x1C

#define HAL_LIS3MDL_I2C_ADDRESS      0x1C    // I2C Address    

#define LIS3MDL_WHO_ID  0x3D

// LIS30MDL registers
#define WHO_AM_I                    0x0F    

#define CTRL_REG1                   0x20    
#define CTRL_REG2                   0x21   
#define CTRL_REG3                   0x22
#define CTRL_REG4                   0x23
#define CTRL_REG5                   0x24

#define STATUS_REG                  0x27
#define OUT_X_L                     0x28
#define OUT_X_H                     0x29
#define OUT_Y_L                     0x2A
#define OUT_Y_H                     0x2B
#define OUT_Z_L                     0x2C
#define OUT_Z_H                     0x2D
#define TEMP_OUT_L                  0x2E
#define TEMP_OUT_H                  0x2F
#define INT_CFG                     0x30
#define INT_SRC                     0x31
#define INT_THS_L                   0x32
#define INT_THS_H                   0x33


/******************************************************************************
 * FUNCTION PROTOTYPES
 */
void HalMagInit(void);
bool HalMagTest(void);

bool HalMagRead(uint8 *pBuf);

bool HalMagGetID(uint8 *pBuf);
bool HalMagGetXYZ(uint8 *pBuf );
bool HalMagGetTemp(uint8 *pBuf );

void HalMagSetRange(uint8 range);
void HalMagSetThresold(uint16 thresold);

/******************************************************************************
******************************************************************************/


#endif

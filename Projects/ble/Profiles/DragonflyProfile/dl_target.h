/**************************************************************************************************
  Filename:       dl_target.h
  Revised:        $Date: 2012-11-16 18:39:26 -0800 (Fri, 16 Nov 2012) $
  Revision:       $Revision: 32218 $

  Description:    This file contains DL Target header file.

**************************************************************************************************/
#ifndef DL_TARGET_H
#define DL_TARGET_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

#include "hal_aes.h"
#include "hal_types.h"

/*********************************************************************
 * CONSTANTS
 */

#if !defined DL_IMG_A_PAGE
#define DL_IMG_A_PAGE        1
#define DL_IMG_A_AREA        62
#endif

#if !defined DL_IMG_B_PAGE
// Image-A/B can be very differently sized areas when implementing BIM vice DL boot loader.
#if defined FEATURE_DL_BIM
#define DL_IMG_B_PAGE        8
#else
#define DL_IMG_B_PAGE        63
#endif
#define DL_IMG_B_AREA       (124 - DL_IMG_A_AREA)
#endif

#if defined HAL_IMAGE_B
#define DL_IMG_D_PAGE        DL_IMG_A_PAGE
#define DL_IMG_D_AREA        DL_IMG_A_AREA
#define DL_IMG_R_PAGE        DL_IMG_B_PAGE
#define DL_IMG_R_AREA        DL_IMG_B_AREA
#else   //#elif defined HAL_IMAGE_A or a non-BIM-enabled DL Image-A w/ constants in Bank 1 vice 5.
#define DL_IMG_D_PAGE        DL_IMG_B_PAGE
#define DL_IMG_D_AREA        DL_IMG_B_AREA
#define DL_IMG_R_PAGE        DL_IMG_A_PAGE
#define DL_IMG_R_AREA        DL_IMG_A_AREA
#endif

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// DL Image Header
extern const __code dl_hdr_t _dlHdr;

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * @fn      DLTarget_AddService
 *
 * @brief   Initializes the DL Service by registering GATT attributes
 *          with the GATT server. Only call this function once.
 *
 * @return  Success or Failure
 */
bStatus_t DLTarget_AddService( void );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* DL_TARGET_H */

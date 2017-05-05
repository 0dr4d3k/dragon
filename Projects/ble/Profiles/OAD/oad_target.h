/*******************************************************************************
  Filename:       oad_target.h
  Revised:        $Date: 2017-04-14 18:18:18$
  Revision:       $Revision: 32218 $

  Description:    This file contains OAD Target header file.
*******************************************************************************/
#ifndef OAD_TARGET_H
#define OAD_TARGET_H

#ifdef __cplusplus
extern "C"
{
#endif

 
/*******************************************************************************
 * INCLUDES
 */
#include "hal_aes.h"
#include "hal_types.h"

  
/*******************************************************************************
 * CONSTANTS
 */
#if !defined OAD_IMG_A_PAGE
  #define OAD_IMG_A_PAGE        1

  // for linker file: "dz_cc254x_f256_imgA_c96K.xcl" do not work
  // and linker file: "dz_cc254x_f256_imgB_c152K.xcl"
//  #define OAD_IMG_A_AREA        48
  
  // for linker file: "dz_cc254x_f256_imgA_a100K.xcl"
  // and linker file: "dz_cc254x_f256_imgB_a148K.xcl"
  #define OAD_IMG_A_AREA        50

  // for linker file: "df_cc254x_f256_imgA_b104K.xcl"
  // and linker file: "df_cc254x_f256_imgB_b144K.xcl"
//  #define OAD_IMG_A_AREA        52

  // NOTE: do not forget modify also in BIM  
  // NOTE: do not forget change linker file in Options-Liker contextual menu
#endif

#if !defined OAD_IMG_B_PAGE
  // Image-A/B can be very differently sized areas when implementing BIM vice OAD boot loader.
  #if defined OAD_BIM
    #define OAD_IMG_B_PAGE        6
  #else
    #define OAD_IMG_B_PAGE        53
  #endif
  #define OAD_IMG_B_AREA       (124 - OAD_IMG_A_AREA)
#endif

#if defined HAL_IMAGE_B
#define OAD_IMG_D_PAGE        OAD_IMG_A_PAGE
#define OAD_IMG_D_AREA        OAD_IMG_A_AREA
#define OAD_IMG_R_PAGE        OAD_IMG_B_PAGE
#define OAD_IMG_R_AREA        OAD_IMG_B_AREA
#else   //#elif defined HAL_IMAGE_A or a non-BIM-enabled OAD Image-A w/ constants in Bank 1 vice 5.
#define OAD_IMG_D_PAGE        OAD_IMG_B_PAGE
#define OAD_IMG_D_AREA        OAD_IMG_B_AREA
#define OAD_IMG_R_PAGE        OAD_IMG_A_PAGE
#define OAD_IMG_R_AREA        OAD_IMG_A_AREA
#endif

  
/*******************************************************************************
 * MACROS
 */

  
/*******************************************************************************
 * GLOBAL VARIABLES
 */
// OAD Image Header
extern const __code img_hdr_t _imgHdr;


/*******************************************************************************
 * TYPEDEFS
 */


/*******************************************************************************
 * @fn      OADTarget_AddService
 * @brief   Initializes the OAD Service by registering GATT attributes
 *          with the GATT server. Only call this function once.
 * @return  Success or Failure
 */
bStatus_t OADTarget_AddService( void );

/*******************************************************************************
*******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* OAD_TARGET_H */

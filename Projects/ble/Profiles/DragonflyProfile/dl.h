/**************************************************************************************************
  Filename:       dl.h
  Revised:        $Date: 2012-12-06 13:16:36 -0800 (Thu, 06 Dec 2012) $
  Revision:       $Revision: 32472 $

  Description:    This file contains DL Profile header file.
**************************************************************************************************/
#ifndef DL_H
#define DL_H

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

#define DL_SERVICE_UUID      0xCC10
#define DL_IMG_IDENTIFY_UUID 0xCC11
#define DL_IMG_BLOCK_UUID    0xCC12

#define DL_IMG_CRC_OSET      0x0000
#if defined FEATURE_DL_SECURE
#define DL_IMG_HDR_OSET      0x0000
#else  // crc0 is calculated and placed by the IAR linker at 0x0, so img_hdr_t is 2 bytes offset.
#define DL_IMG_HDR_OSET      0x0002
#endif

#define DL_CHAR_CNT          2

// DL Characteristic Indices
#define DL_CHAR_IMG_IDENTIFY 0
#define DL_CHAR_IMG_BLOCK    1

// Image Identification size
#define DL_IMG_ID_SIZE       4

// Image header size (version + length + image id size)
#define DL_IMG_HDR_SIZE      ( 2 + 2 + DL_IMG_ID_SIZE )

// The Image is transporte in 16-byte blocks in order to avoid using blob operations.
#define DL_BLOCK_SIZE        16
#define DL_BLOCKS_PER_PAGE  (HAL_FLASH_PAGE_SIZE / DL_BLOCK_SIZE)
#define DL_BLOCK_MAX        (DL_BLOCKS_PER_PAGE * DL_IMG_D_AREA)

/*********************************************************************
 * MACROS
 */

// Macros to get Image ID (LSB) and Version Number
#define DL_IMG_ID( ver )    ( (ver) & 0x01 )
#define DL_VER_NUM( ver )   ( (ver) >> 0x01 )

// Macro to set Image Version
#if defined (HAL_IMAGE_A)
  #define DL_IMG_VER( ver ) ( (uint16)( (ver) << 0x01 ) )            // Clear LSB for Image A
#else
  #define DL_IMG_VER( ver ) ( (uint16)( ( (ver) << 0x01 ) | 0x01 ) ) // Set LSB for Imange B
#endif

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * TYPEDEFS
 */

// The Image Header will not be encrypted, but it will be included in a Signature.
typedef struct {
#if defined FEATURE_DL_SECURE
  // Secure DL uses the Signature for image validation instead of calculating a CRC, but the use
  // of CRC==CRC-Shadow for quick boot-up determination of a validated image is still used.
  uint16 crc0;       // CRC must not be 0x0000 or 0xFFFF.
#endif
  uint16 crc1;       // CRC-shadow must be 0xFFFF.
  // User-defined Image Version Number - default logic uses simple a '!=' comparison to start an DL.
  uint16 ver;
  uint16 len;        // Image length in 4-byte blocks (i.e. HAL_FLASH_WORD_SIZE blocks).
  uint8  uid[4];     // User-defined Image Identification bytes.
  uint8  res[4];     // Reserved space for future use.
} dl_hdr_t;

#if defined FEATURE_DL_SECURE
static_assert((sizeof(img_hdr_t) == 16), "Bad SBL_ADDR_AES_HDR definition.");
static_assert(((sizeof(img_hdr_t) % KEY_BLENGTH) == 0),
                      "img_hdr_t is not an even multiple of KEY_BLENGTH");
#endif

// The AES Header must be encrypted and the Signature must include the Image Header.
typedef struct {
  uint8 signature[KEY_BLENGTH];  // The AES-128 CBC-MAC signature.
  uint8 nonce12[12];             // The 12-byte Nonce for calculating the signature.
  uint8 spare[4];
} aes_dl_hdr_t;

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* DL_H */

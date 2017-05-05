/**************************************************************************************************
  Filename:       dl_target.c
  Revised:        $Date: 2012-11-16 18:39:26 -0800 (Fri, 16 Nov 2012) $
  Revision:       $Revision: 32218 $

  Description:    This file contains DL Target implementation.

**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "linkdb.h"
#include "att.h"
#include "gatt.h"
#include "gatt_uuid.h"
#include "gattservapp.h"
#include "hal_aes.h"
#include "hal_crc.h"
#include "hal_flash.h"
#include "hal_dma.h"
#include "hal_types.h"
#include "dl.h"
#include "dl_target.h"
#include "OSAL.h"
#include "df_util.h"

/*********************************************************************
 * CONSTANTS
 */
#define DL_FLASH_PAGE_MULT  ((uint16)(HAL_FLASH_PAGE_SIZE / HAL_FLASH_WORD_SIZE))

#if defined (FEATURE_DL_SECURE) && defined (HAL_IMAGE_A)
  // Enabled to ONLY build a BOOTSTRAP Encrypted Image-A (for programming over
  // BEM, not BIM). Comment line below to build a non-bootstrap Encrypted Image-A.
  #define BOOTP_E_IMAGE_A
#endif

#if !defined (DL_IMAGE_VERSION)
  #define DL_IMAGE_VERSION    0x0002
#endif

#if !defined (DL_IMAGE_A_USER_ID)
  #define DL_IMAGE_A_USER_ID  {'A', 'A', 'A', 'A'}
#endif

#if !defined (DL_IMAGE_B_USER_ID)
  #define DL_IMAGE_B_USER_ID  {'B', 'B', 'B', 'B'}
#endif

#define DL_IMG_BLK_NUM_SIZE   2

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// DL Service UUID

static CONST uint8 dlServUUID[ATT_UUID_SIZE] =
{
  LIBELIUM_BASE_UUID_128( DL_SERVICE_UUID )
};

static CONST uint8 dlCharUUID[DL_CHAR_CNT][ATT_UUID_SIZE] =
{
 // DL Image Identify UUID
 LIBELIUM_BASE_UUID_128( DL_IMG_IDENTIFY_UUID ),

 // DL Image Block Request/Response UUID
 LIBELIUM_BASE_UUID_128( DL_IMG_BLOCK_UUID )
};
   
/* comprobar que pasa con UUID 128 y 16
static CONST uint8 dlServUUID[TI_UUID_SIZE] =
{
  TI_UUID( DL_SERVICE_UUID )
};

static CONST uint8 dlCharUUID[DL_CHAR_CNT][TI_UUID_SIZE] =
{
 // DL Image Identify UUID
 TI_UUID( DL_IMG_IDENTIFY_UUID ),

 // DL Image Block Request/Response UUID
 TI_UUID( DL_IMG_BLOCK_UUID )
};
*/  

/*********************************************************************
 * Profile Attributes - variables
 */

// DL Service attribute
static CONST gattAttrType_t dlService = { ATT_UUID_SIZE, dlServUUID };

// Place holders for the GATT Server App to be able to lookup handles.
static uint8 dlCharVals[DL_CHAR_CNT];

// DL Characteristic Properties
static uint8 dlCharProps = GATT_PROP_WRITE_NO_RSP | GATT_PROP_WRITE | GATT_PROP_NOTIFY;

// DL Client Characteristic Configs
static gattCharCfg_t *dlImgIdentifyConfig;
static gattCharCfg_t *dlImgBlockConfig;

// DL Characteristic user descriptions
static CONST uint8 dlImgIdentifyDesc[] = "Datalogger Identify";
static CONST uint8 dlImgBlockDesc[] = "Datalogger Block";

/*********************************************************************
 * Profile Attributes - Table
 */

static gattAttribute_t dlAttrTbl[] =
{
  // DL Service
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID },
    GATT_PERMIT_READ,
    0,
    (uint8 *)&dlService
  },

    // DL Image Identify Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &dlCharProps
    },

      // DL Image Identify Characteristic Value
      {
        { ATT_UUID_SIZE, dlCharUUID[0] },
        GATT_PERMIT_WRITE,
        0,
        dlCharVals+0
      },

      // Characteristic configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&dlImgIdentifyConfig
      },

      // DL Image Identify User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        (uint8 *)dlImgIdentifyDesc
      },

    // DL Image Block Request/Response Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &dlCharProps
    },

      // DL Image Block Request/Response Characteristic Value
      {
        { ATT_UUID_SIZE, dlCharUUID[1] },
        GATT_PERMIT_WRITE,
        0,
        dlCharVals+1
      },

       // Characteristic configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&dlImgBlockConfig
      },

      // DL Image Block Request/Response User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        (uint8 *)dlImgBlockDesc
      }
};
/*Comprobar que pasa con UUID 128 y 16
static gattAttribute_t dlAttrTbl[] =
{
  // DL Service
  {
    { ATT_BT_UUID_SIZE, primaryServiceUUID },
    GATT_PERMIT_READ,
    0,
    (uint8 *)&dlService
  },

    // DL Image Identify Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &dlCharProps
    },

      // DL Image Identify Characteristic Value
      {
        { TI_UUID_SIZE, dlCharUUID[0] },
        GATT_PERMIT_WRITE,
        0,
        dlCharVals+0
      },

      // Characteristic configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&dlImgIdentifyConfig
      },

      // DL Image Identify User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        (uint8 *)dlImgIdentifyDesc
      },

    // DL Image Block Request/Response Characteristic Declaration
    {
      { ATT_BT_UUID_SIZE, characterUUID },
      GATT_PERMIT_READ,
      0,
      &dlCharProps
    },

      // DL Image Block Request/Response Characteristic Value
      {
        { TI_UUID_SIZE, dlCharUUID[1] },
        GATT_PERMIT_WRITE,
        0,
        dlCharVals+1
      },

       // Characteristic configuration
      {
        { ATT_BT_UUID_SIZE, clientCharCfgUUID },
        GATT_PERMIT_READ | GATT_PERMIT_WRITE,
        0,
        (uint8 *)&dlImgBlockConfig
      },

      // DL Image Block Request/Response User Description
      {
        { ATT_BT_UUID_SIZE, charUserDescUUID },
        GATT_PERMIT_READ,
        0,
        (uint8 *)dlImgBlockDesc
      }
};
*/

#pragma location="DL_HEADER"
const __code dl_hdr_t _dlHdr = {
#if defined FEATURE_DL_SECURE
  2012,                       // CRC must not be 0x0000 or 0xFFFF.
#endif
#if defined (BOOTP_E_IMAGE_A)
#warning "Forcing a CRC-shadow match with the BOOTP_E_IMAGE_A flag - is this bootstrap code?"
  2012,                       // CRC-shadow forced to match CRC for a bootstrap Encrypted Image-A
#else
  0xFFFF,                     // CRC-shadow must be 0xFFFF for everything else
#endif
  DL_IMG_VER( DL_IMAGE_VERSION ), // 15-bit Version #, left-shifted 1; OR with Image-B/Not-A bit.
  DL_IMG_R_AREA * DL_FLASH_PAGE_MULT,
#if defined HAL_IMAGE_A
  DL_IMAGE_A_USER_ID,        // User-Id
#else
  DL_IMAGE_B_USER_ID,        // User-Id
#endif
  { 0xFF, 0xFF, 0xFF, 0xFF }  // Reserved
};
#pragma required=_dlHdr

#pragma location="AES_DL_HEADER"
static const __code aes_dl_hdr_t _aesdlHdr = {
 { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
 { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B },  // Dummy Nonce
 { 0xFF, 0xFF, 0xFF, 0xFF }   // Spare
};
#pragma required=_aesdlHdr

/*********************************************************************
 * LOCAL VARIABLES
 */

static uint16 dlBlkNum = 0, dlBlkTot = 0xFFFF;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static bStatus_t dlReadAttrCB(uint16 connHandle, gattAttribute_t *pAttr,
                               uint8 *pValue, uint8 *pLen, uint16 offset,
                               uint8 maxLen, uint8 method);

static bStatus_t dlWriteAttrCB(uint16 connHandle, gattAttribute_t *pAttr,
                                uint8 *pValue, uint8 len, uint16 offset,
                                uint8 method);

CONST gattServiceCBs_t dlCBs =
{
  dlReadAttrCB,  // Read callback function pointer.
  dlWriteAttrCB, // Write callback function pointer.
  NULL            // Authorization callback function pointer.
};

static void dlImgBlockReq(uint16 connHandle, uint16 blkNum);

static void dlImgIdentifyReq(uint16 connHandle, dl_hdr_t *pImgHdr);

static bStatus_t dlImgIdentifyWrite( uint16 connHandle, uint8 *pValue );

static bStatus_t dlImgBlockWrite( uint16 connHandle, uint8 *pValue );

#if !defined FEATURE_DL_SECURE
static void DMAExecCrc(uint8 page, uint16 offset, uint16 len);
static uint8 checkDL(void);
#endif

/*********************************************************************
 * @fn      DLTarget_AddService
 *
 * @brief   Initializes the DL Service by registering GATT attributes
 *          with the GATT server. Only call this function once.
 *
 * @return  The return value of GATTServApp_RegisterForMsg().
 */
bStatus_t DLTarget_AddService(void)
{
  // Allocate Client Characteristic Configuration table
  dlImgIdentifyConfig = (gattCharCfg_t *)osal_mem_alloc( sizeof(gattCharCfg_t) *
                                                          linkDBNumConns);
  if (dlImgIdentifyConfig == NULL)
  {
    return ( bleMemAllocError );
  }
  
  // Allocate Client Characteristic Configuration table
  dlImgBlockConfig = (gattCharCfg_t *)osal_mem_alloc( sizeof(gattCharCfg_t) *
                                                       linkDBNumConns);
  
  if (dlImgBlockConfig == NULL)
  {
    // Free already allocated data
    osal_mem_free( dlImgIdentifyConfig );
    
    return ( bleMemAllocError );
  }
  
  // Initialize Client Characteristic Configuration attributes
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, dlImgIdentifyConfig );
  GATTServApp_InitCharCfg( INVALID_CONNHANDLE, dlImgBlockConfig );

  return GATTServApp_RegisterService(dlAttrTbl, GATT_NUM_ATTRS(dlAttrTbl),
                                     GATT_MAX_ENCRYPT_KEY_SIZE, &dlCBs);
}

/*********************************************************************
 * @fn      dlReadAttrCB
 *
 * @brief   Read an attribute.
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be read
 * @param   pLen - length of data to be read
 * @param   offset - offset of the first octet to be read
 * @param   maxLen - maximum length of data to be read
 * @param   method - type of read message 
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t dlReadAttrCB(uint16 connHandle, gattAttribute_t *pAttr,
                               uint8 *pValue, uint8 *pLen, uint16 offset,
                               uint8 maxLen, uint8 method)
{
  bStatus_t status = SUCCESS;

  // TBD: is there any use for supporting reads
  *pLen = 0;
  status = ATT_ERR_INVALID_HANDLE;

  return status;
}

/*********************************************************************
 * @fn      dlWriteAttrCB
 *
 * @brief   Validate and Write attribute data
 *
 * @param   connHandle - connection message was received on
 * @param   pAttr - pointer to attribute
 * @param   pValue - pointer to data to be written
 * @param   len - length of data
 * @param   offset - offset of the first octet to be written
 * @param   method - type of write message 
 *
 * @return  SUCCESS, blePending or Failure
 */
static bStatus_t dlWriteAttrCB(uint16 connHandle, gattAttribute_t *pAttr,
                                uint8 *pValue, uint8 len, uint16 offset,
                                uint8 method)
{
  bStatus_t status = SUCCESS;

  if ( pAttr->type.len == ATT_BT_UUID_SIZE )
  {
    // 16-bit UUID
    uint16 uuid = BUILD_UINT16( pAttr->type.uuid[0], pAttr->type.uuid[1]);
    if ( uuid == GATT_CLIENT_CHAR_CFG_UUID)
    {
      status = GATTServApp_ProcessCCCWriteReq( connHandle, pAttr, pValue, len,
                                               offset, GATT_CLIENT_CFG_NOTIFY );
    }
    else
    {
      status = ATT_ERR_ATTR_NOT_FOUND; // Should never get here!
    }
  }
  else
  {
    // 128-bit UUID
    if (osal_memcmp(pAttr->type.uuid, dlCharUUID[DL_CHAR_IMG_IDENTIFY], ATT_UUID_SIZE))
    {
      status = dlImgIdentifyWrite( connHandle, pValue );
    }
    else if (osal_memcmp(pAttr->type.uuid, dlCharUUID[DL_CHAR_IMG_BLOCK], ATT_UUID_SIZE))
    {
      status = dlImgBlockWrite( connHandle, pValue );
    }
    else
    {
      status = ATT_ERR_ATTR_NOT_FOUND; // Should never get here!
    }
  }

  return status;
}

/*********************************************************************
 * @fn      dlImgIdentifyWrite
 *
 * @brief   Process the Image Identify Write.
 *
 * @param   connHandle - connection message was received on
 * @param   pValue - pointer to data to be written
 *
 * @return  status
 */
static bStatus_t dlImgIdentifyWrite( uint16 connHandle, uint8 *pValue )
{
  dl_hdr_t rxHdr;
  dl_hdr_t ImgHdr;

  rxHdr.ver = BUILD_UINT16( pValue[0], pValue[1] );
  rxHdr.len = BUILD_UINT16( pValue[2], pValue[3] );

  (void)osal_memcpy(rxHdr.uid, pValue+4, sizeof(rxHdr.uid));

  HalFlashRead(DL_IMG_R_PAGE, DL_IMG_HDR_OSET, (uint8 *)&ImgHdr, sizeof(dl_hdr_t));

  dlBlkTot = rxHdr.len / (DL_BLOCK_SIZE / HAL_FLASH_WORD_SIZE);

  if ( (DL_IMG_ID( ImgHdr.ver ) != DL_IMG_ID( rxHdr.ver )) && // TBD: add customer criteria for initiating DL here.
       (dlBlkTot <= DL_BLOCK_MAX) &&
       (dlBlkTot != 0) )
  {
    dlBlkNum = 0;
    dlImgBlockReq(connHandle, 0);
  }
  else
  {
    dlImgIdentifyReq(connHandle, &ImgHdr);
  }

  return ( SUCCESS );
}

/*********************************************************************
 * @fn      dlImgBlockWrite
 *
 * @brief   Process the Image Block Write.
 *
 * @param   connHandle - connection message was received on
 * @param   pValue - pointer to data to be written
 *
 * @return  status
 */
static bStatus_t dlImgBlockWrite( uint16 connHandle, uint8 *pValue )
{
  uint16 blkNum = BUILD_UINT16( pValue[0], pValue[1] );

  // make sure this is the image we're expecting
  if ( blkNum == 0 )
  {
    dl_hdr_t ImgHdr;
    uint16 ver = BUILD_UINT16( pValue[6], pValue[7] );
    uint16 blkTot = BUILD_UINT16( pValue[8], pValue[9] ) / (DL_BLOCK_SIZE / HAL_FLASH_WORD_SIZE);

    HalFlashRead(DL_IMG_R_PAGE, DL_IMG_HDR_OSET, (uint8 *)&ImgHdr, sizeof(dl_hdr_t));

    if ( ( dlBlkNum != blkNum ) ||
         ( dlBlkTot != blkTot ) ||
         ( DL_IMG_ID( ImgHdr.ver ) == DL_IMG_ID( ver ) ) )
    {
      return ( ATT_ERR_WRITE_NOT_PERMITTED );
    }
  }

  if (dlBlkNum == blkNum)
  {
    uint16 addr = dlBlkNum * (DL_BLOCK_SIZE / HAL_FLASH_WORD_SIZE) +
                              (DL_IMG_D_PAGE * DL_FLASH_PAGE_MULT);
    dlBlkNum++;

#if defined FEATURE_DL_SECURE
    if (blkNum == 0)
    {
      // Stop attack with crc0==crc1 by forcing crc1=0xffff.
      pValue[4] = 0xFF;
      pValue[5] = 0xFF;
    }
#endif

#if defined HAL_IMAGE_B
    // Skip the Image-B area which lies between the lower & upper Image-A parts.
    if (addr >= (DL_IMG_B_PAGE * DL_FLASH_PAGE_MULT))
    {
      addr += DL_IMG_B_AREA * DL_FLASH_PAGE_MULT;
    }
#endif
    if ((addr % DL_FLASH_PAGE_MULT) == 0)
    {
      HalFlashErase(addr / DL_FLASH_PAGE_MULT);
    }

    HalFlashWrite(addr, pValue+2, (DL_BLOCK_SIZE / HAL_FLASH_WORD_SIZE));
  }

  if (dlBlkNum == dlBlkTot)  // If the DL Image is complete.
  {
#if defined FEATURE_DL_SECURE
    HAL_SYSTEM_RESET();  // Only the secure DL boot ldler has the security key to decrypt.
#else
    if (checkDL())
    {
#if !defined HAL_IMAGE_A
      // The BIM always checks for a valid Image-B before Image-A,
      // so Image-A never has to invalidate itself.
      uint16 crc[2] = { 0x0000, 0xFFFF };
      uint16 addr = DL_IMG_R_PAGE * DL_FLASH_PAGE_MULT + DL_IMG_CRC_OSET / HAL_FLASH_WORD_SIZE;
      HalFlashWrite(addr, (uint8 *)crc, 1);
#endif
      HAL_SYSTEM_RESET();
    }
#endif
  }
  else  // Request the next DL Image block.
  {
    dlImgBlockReq(connHandle, dlBlkNum);
  }

  return ( SUCCESS );
}

/*********************************************************************
 * @fn      dlImgIdentifyReq
 *
 * @brief   Process the Image Identify Request.
 *
 * @param   connHandle - connection message was received on
 * @param   pImgHdr - Pointer to the dl_hdr_t data to send.
 *
 * @return  None
 */
static void dlImgBlockReq(uint16 connHandle, uint16 blkNum)
{
  uint16 value = GATTServApp_ReadCharCfg( connHandle, dlImgBlockConfig );

  // If notifications enabled
  if ( value & GATT_CLIENT_CFG_NOTIFY )
  {
    gattAttribute_t *pAttr = GATTServApp_FindAttr(dlAttrTbl, GATT_NUM_ATTRS(dlAttrTbl),
                                                  dlCharVals+DL_CHAR_IMG_BLOCK);
    if ( pAttr != NULL )
    {
      attHandleValueNoti_t noti;
      
      noti.pValue = GATT_bm_alloc(connHandle, ATT_HANDLE_VALUE_NOTI,
                                  DL_IMG_BLK_NUM_SIZE, NULL);
      if ( noti.pValue != NULL )
      {
        noti.handle = pAttr->handle;
        noti.len = DL_IMG_BLK_NUM_SIZE;
        noti.pValue[0] = LO_UINT16(blkNum);
        noti.pValue[1] = HI_UINT16(blkNum);

        if ( GATT_Notification(connHandle, &noti, FALSE) != SUCCESS )
        {
          GATT_bm_free((gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI);
        }
      }
    }
  }
}

/*********************************************************************
 * @fn      dlImgIdentifyReq
 *
 * @brief   Process the Image Identify Request.
 *
 * @param   connHandle - connection message was received on
 * @param   pImgHdr - Pointer to the dl_hdr_t data to send.
 *
 * @return  None
 */
static void dlImgIdentifyReq(uint16 connHandle, dl_hdr_t *pImgHdr)
{
  uint16 value = GATTServApp_ReadCharCfg( connHandle, dlImgIdentifyConfig );

  // If notifications enabled
  if ( value & GATT_CLIENT_CFG_NOTIFY )
  {
    gattAttribute_t *pAttr = GATTServApp_FindAttr(dlAttrTbl, GATT_NUM_ATTRS(dlAttrTbl),
                                                  dlCharVals+DL_CHAR_IMG_IDENTIFY);
    if ( pAttr != NULL )
    {
      attHandleValueNoti_t noti;
      
      noti.pValue = GATT_bm_alloc(connHandle, ATT_HANDLE_VALUE_NOTI,
                                  DL_IMG_HDR_SIZE, NULL);
      if ( noti.pValue != NULL )
      {
        noti.handle = pAttr->handle;
        noti.len = DL_IMG_HDR_SIZE;
        noti.pValue[0] = LO_UINT16(pImgHdr->ver);
        noti.pValue[1] = HI_UINT16(pImgHdr->ver);

        noti.pValue[2] = LO_UINT16(pImgHdr->len);
        noti.pValue[3] = HI_UINT16(pImgHdr->len);

        (void)osal_memcpy(noti.pValue+4, pImgHdr->uid, sizeof(pImgHdr->uid));

        if ( GATT_Notification(connHandle, &noti, FALSE) != SUCCESS )
        {
          GATT_bm_free((gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI);
        }
      }
    }
  }
}

#if !defined FEATURE_DL_SECURE

#if 0
/**************************************************************************************************
 * @fn          crcCalcDL
 *
 * @brief       Run the CRC16 Polynomial calculation over the DL image.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      The CRC16 calculated.
 **************************************************************************************************
 */
static uint16 crcCalcDL(void)
{
  uint8 pageEnd = dlBlkTot / DL_BLOCKS_PER_PAGE;
  uint16 osetEnd = (dlBlkTot - (pageEnd * DL_BLOCKS_PER_PAGE)) * DL_BLOCK_SIZE;

#if defined HAL_IMAGE_B
  pageEnd += DL_IMG_D_PAGE + DL_IMG_B_AREA;
#else
  pageEnd += DL_IMG_D_PAGE;
#endif

  HalCRCInit(0x0000);  // Seed thd CRC calculation with zero.

  for (uint8 page = DL_IMG_D_PAGE; ; page++)
  {
#if defined HAL_IMAGE_B
    // Skip the Image-B area which lies between the lower & upper Image-A parts.
    if (page == DL_IMG_B_PAGE)
    {
      page += DL_IMG_B_AREA;
    }
#endif

    for (uint16 oset = 0; oset < HAL_FLASH_PAGE_SIZE; oset += HAL_FLASH_WORD_SIZE)
    {
      if ((page == DL_IMG_D_PAGE) && (oset == DL_IMG_CRC_OSET))
      {
        continue;  // Skip the CRC and shadow.
      }
      else if ((page == pageEnd) && (oset == osetEnd))
      {
        return HalCRCCalc();
      }
      else
      {
        uint8 buf[HAL_FLASH_WORD_SIZE];
        HalFlashRead(page, oset, buf, HAL_FLASH_WORD_SIZE);

        for (uint8 idx = 0; idx < HAL_FLASH_WORD_SIZE; idx++)
        {
          HalCRCExec(buf[idx]);
        }
      }
    }
  }
}
#endif

/**************************************************************************************************
 * @fn          crcCalcDLDMA
 *
 * @brief       Run the CRC16 Polynomial calculation over the DL image,
 *              using DMA to read the flash memory into the CRC register
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      The CRC16 calculated.
 **************************************************************************************************
 */
static uint16 crcCalcDLDMA(void)
{
  uint8 pageBeg = DL_IMG_D_PAGE;
  uint8 pageEnd = dlBlkTot / DL_BLOCKS_PER_PAGE;

#if defined HAL_IMAGE_B
  pageEnd += DL_IMG_D_PAGE + DL_IMG_B_AREA;
#else
  pageEnd += DL_IMG_D_PAGE;
#endif

  HalCRCInit(0x0000);  // Seed thd CRC calculation with zero.

  // Handle first page differently to skip CRC and CRC shadow when calculating
  DMAExecCrc(pageBeg, 4, HAL_FLASH_PAGE_SIZE-4);

  // Do remaining pages
  for (uint8 pg = pageBeg + 1; pg < pageEnd; pg++)
  {
#if defined HAL_IMAGE_B  // Means we are receiving ImgA, so skip ImgB pages
    if (pg == DL_IMG_B_PAGE)
    {
      pg += DL_IMG_B_AREA;
    }
#endif
    
    DMAExecCrc(pg, 0, HAL_FLASH_PAGE_SIZE);
  }
  
  return HalCRCCalc();
}

/**************************************************************************************************
 * @fn          DMAExecCrc
 *
 * @brief       This function assumes CRC has been initialized and sets up and
 *              starts a dma tranfer from a flash page to the CRC HW module.
 *
 * @note        This function uses NV DMA Ch; Ch0
 *
 * input parameters
 *
 * @param       page - A valid flash page number.
 * @param       offset - A valid offset into the page.
 * @param       len - A valid number of bytes to calculate crc of.
 *
 * @return      None.
 **************************************************************************************************
 */
void DMAExecCrc(uint8 page, uint16 offset, uint16 len) {

  uint8 memctr = MEMCTR;  // Save to restore.
  
  // Calculate the offset into the containing flash bank as it gets mapped into XDATA.
  uint16 address = (offset + HAL_FLASH_PAGE_MAP) +
                   ((page % HAL_FLASH_PAGE_PER_BANK) * HAL_FLASH_PAGE_SIZE);

  // Pointer to DMA config structure
  halDMADesc_t *dmaCh0_p = HAL_DMA_GET_DESC0();
  
#if !defined HAL_DL_BOOT_CODE
  halIntState_t is;
#endif

  page /= HAL_FLASH_PAGE_PER_BANK;  // Calculate the flash bank from the flash page.

#if !defined HAL_DL_BOOT_CODE
  HAL_ENTER_CRITICAL_SECTION(is);
#endif
  
  // Calculate and map the containing flash bank into XDATA.
  MEMCTR = (MEMCTR & 0xF8) | page;  // page is actually bank
  
  
  // Start address for CRC calculation in the XDATA mapped flash bank
  HAL_DMA_SET_SOURCE(dmaCh0_p, address);
  
  // Destination for data transfer, RNDH mapped to XDATA
  HAL_DMA_SET_DEST(dmaCh0_p, 0x70BD);
  
  // One whole page (or len) at a time
  HAL_DMA_SET_LEN(dmaCh0_p, len);
  
  // 8-bit, block, no trigger
  HAL_DMA_SET_WORD_SIZE(dmaCh0_p, HAL_DMA_WORDSIZE_BYTE);
  HAL_DMA_SET_TRIG_MODE(dmaCh0_p, HAL_DMA_TMODE_BLOCK);
  HAL_DMA_SET_TRIG_SRC(dmaCh0_p, HAL_DMA_TRIG_NONE);
  
  // SRC += 1, DST = constant, no IRQ, all 8 bits, high priority
  HAL_DMA_SET_SRC_INC(dmaCh0_p, HAL_DMA_SRCINC_1);
  HAL_DMA_SET_DST_INC(dmaCh0_p, HAL_DMA_DSTINC_0);
  HAL_DMA_SET_IRQ(dmaCh0_p, HAL_DMA_IRQMASK_DISABLE);
  HAL_DMA_SET_M8(dmaCh0_p, HAL_DMA_M8_USE_8_BITS);
  HAL_DMA_SET_PRIORITY(dmaCh0_p, HAL_DMA_PRI_HIGH);
  
  // Arm the DMA channel (0)
  HAL_DMA_ARM_CH(0);
  
  // 9 cycles wait
  asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
  asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop");
  
  // Start DMA tranfer
  HAL_DMA_MAN_TRIGGER(0);
  
  // Wait for dma to finish.
  while(DMAREQ & 0x1);
  
  // Restore bank mapping
  MEMCTR = memctr;

#if !defined HAL_DL_BOOT_CODE
  HAL_EXIT_CRITICAL_SECTION(is);
#endif
}

/**************************************************************************************************
 * @fn          checkDL
 *
 * @brief       Check validity of the downloaded image.
 *
 * input parameters
 *
 * None.
 *
 * output parameters
 *
 * None.
 *
 * @return      TRUE or FALSE for image valid.
 **************************************************************************************************
 */
static uint8 checkDL(void)
{
  uint16 crc[2];

  HalFlashRead(DL_IMG_D_PAGE, DL_IMG_CRC_OSET, (uint8 *)crc, sizeof(crc));

  if ((crc[0] == 0xFFFF) || (crc[0] == 0x0000))
  {
    return FALSE;
  }

  if (crc[1] == 0xFFFF)
  {
    //P0DIR |= 1;
    //P0_0 = 0;
    //P0_0 = 1;
    //P0_0 = 0;
    //P0_0 = 1;
    //P0_0 = 0;
    //P0_0 = 1;
    crc[1] = crcCalcDLDMA();
    //P0_0 = 0;

#if defined FEATURE_DL_BIM  // If download image is made to run in-place, enable it here.
    uint16 addr = DL_IMG_D_PAGE * DL_FLASH_PAGE_MULT + DL_IMG_CRC_OSET / HAL_FLASH_WORD_SIZE;
    crc[0] = 0xFFFF;
    HalFlashWrite(addr, (uint8 *)crc, 1);
    HalFlashRead(DL_IMG_D_PAGE, DL_IMG_CRC_OSET, (uint8 *)crc, sizeof(crc));
#endif
  }

  return (crc[0] == crc[1]);
}
#endif // !FEATURE_DL_SECURE


/*********************************************************************
*********************************************************************/

/*******************************************************************************
  Filename:       bim_main.c
  Revised:        $Date: 2017-04-29 13:13:13$
  Revision:       $Revision: 43595 $
  Description:
  This module contains the definitions for the main functionality of an Boot 
  Image Manager.
*******************************************************************************/

/* -----------------------------------------------------------------------------
 *                                 Includes
 * -----------------------------------------------------------------------------
 */
#include "hal_dma.h"
#include "hal_flash.h"
#include "hal_types.h"


/* -----------------------------------------------------------------------------
 *                                 Constants
 * -----------------------------------------------------------------------------
 */
#define BIM_IMG_A_PAGE        1
#define BIM_IMG_A_AREA        50

#define BIM_IMG_B_PAGE        6
#define BIM_IMG_B_AREA       (124 - BIM_IMG_A_AREA)

#define BIM_CRC_OSET          0x00
#define BIM_HDR_OSET          0x00

/* -----------------------------------------------------------------------------
 *                                  Typedefs
 * -----------------------------------------------------------------------------
 */
typedef struct {
  // Secure OAD uses the Signature for image validation instead of calculating a CRC, but the use
  // of CRC==CRC-Shadow for quick boot-up determination of a validated image is still used.
  uint16 crc0;       // CRC must not be 0x0000 or 0xFFFF.
  uint16 crc1;       // CRC-shadow must be 0xFFFF.
  // User-defined Image Version Number - default logic uses simple a '<' comparison to start an OAD.
  uint16 ver;
  uint16 len;        // Image length in 4-byte blocks (i.e. HAL_FLASH_WORD_SIZE blocks).
  uint8  uid[4];     // User-defined Image Identification bytes.
  uint8  res[4];     // Reserved space for future use.
} img_hdr_t;


/* -----------------------------------------------------------------------------
 *                               Global Variables
 * -----------------------------------------------------------------------------
 */
__no_init halDMADesc_t dmaCh0;  // Locally setup for use by HalFlashWrite().


/* -----------------------------------------------------------------------------
 *                                Local Variables
 * -----------------------------------------------------------------------------
 */
__no_init uint8 pgBuf[HAL_FLASH_PAGE_SIZE];

__no_init __data uint8 JumpToImageAorB @ 0x09;

#pragma location = "ALIGNED_CODE"
void halSleepExec(void);

void DMAExecCrc(uint8 page, uint16 offset, uint16 len);


/*******************************************************************************
 * @fn          halSleepExec
 * @brief       This function puts the CC254x to sleep by writing to the PCON 
 *              register. The instruction after writing to PCON must not be 
 *              4-byte aligned or excessive power consumption may result. Since 
 *              the write to PCON is 3 instructions, this function is forced to 
 *              be even-byte aligned. Thus, this function must not have any
 *              automatic variables and the write to PCON must be the first C 
 *              statement.
 *              See the linker file ".xcl" for actual placement of this function.
 * input parameters
 * @param       None.
 * output parameters
 * None.
 * @return      None.
 *******************************************************************************
 */
#pragma optimize=none
void halSleepExec(void)
{
  PCON = 0x01;
  ASM_NOP;
}


/*******************************************************************************
 * @fn          crcCalcDMA
 * @brief       Run the CRC16 Polynomial calculation over the image specified,
 *              using DMA to read the flash memory into the CRC register
 * input parameters
 * @param       page - Flash page on which to beging the CRC calculation.
 * output parameters
 * None.
 * @return      The CRC16 calculated.
 *******************************************************************************
 */
static uint16 crcCalcDMA(uint8 page)
{
  uint16 crc;
  uint8 pageBeg;
  uint8 pageEnd;
  const img_hdr_t *pImgHdr;
  
  // Read fist page in the pgBuf(2048)
  HalFlashRead(page, 0, pgBuf, HAL_FLASH_PAGE_SIZE);

  // Pointer to img_hdr in pgBuf
  pImgHdr = (const img_hdr_t *)(pgBuf + BIM_HDR_OSET);

  pageBeg = page;
  // end page = (len(indicated in imgHdr in blocks of 4 bytes)/2048)*4
  pageEnd = pImgHdr->len / (HAL_FLASH_PAGE_SIZE / HAL_FLASH_WORD_SIZE);

  // One page is used for BIM, so we move this image's last page forward
  pageEnd += pageBeg;
  
  // If image A, set last page to be ImgA size + ImgB size
  if (pageBeg == BIM_IMG_A_PAGE)
  {
    pageEnd += BIM_IMG_B_AREA;
  }

  ADCCON1 &= 0xF3;  // CRC configuration of LRSR.

  // CRC seed of 0x0000.
  RNDL = 0x00;
  RNDL = 0x00;

  // Handle first page differently to skip CRC and CRC shadow when calculating
  DMAExecCrc(pageBeg, 4, HAL_FLASH_PAGE_SIZE-4);
  
  // Do remaining pages
  for (uint8 pg = pageBeg + 1; pg < pageEnd; pg++)
  {
    if (pg == BIM_IMG_B_PAGE)
    {
      pg += BIM_IMG_B_AREA;
    }
     
    DMAExecCrc(pg, 0, HAL_FLASH_PAGE_SIZE);
  }
  
  crc = RNDH;
  crc = (crc << 8) | RNDL;

  return crc;
}

/*******************************************************************************
 * @fn          crcCheck
 * @brief       Calculate the image CRC and set it ready-to-run if it is good.
 * input parameters
 * @param       page - Flash page on which to beging the CRC calculation.
 * output parameters
 * None.
 * @return      None, but no return from this function if the CRC check is good.
 *******************************************************************************
 */
static void crcCheck(uint8 page, uint16 *crc)
{
  HAL_BOARD_INIT();

  if (crc[0] == crcCalcDMA(page))
  {
    //P0_0 = 0;
    uint16 addr = page * (HAL_FLASH_PAGE_SIZE / HAL_FLASH_WORD_SIZE) +
                                 BIM_CRC_OSET / HAL_FLASH_WORD_SIZE;
    crc[1] = crc[0];
    crc[0] = 0xFFFF;

    HAL_DMA_SET_ADDR_DESC0(&dmaCh0);
    HalFlashWrite(addr, (uint8 *)crc, 1);
    HAL_SYSTEM_RESET();
  }
}


/*******************************************************************************
 * @fn          main
 * @brief       C-code main function.
 * input parameters
 * None.
 * output parameters
 * None.
 * @return      None.
 *******************************************************************************
 */
void main(void)
{
  uint16 crc[2];

  // Prefer to run Image-B over Image-A so that Image-A does not have to invalidate itself.
  HalFlashRead(BIM_IMG_B_PAGE, BIM_CRC_OSET, (uint8 *)crc, 4);

  if ((crc[0] != 0xFFFF) && (crc[0] != 0x0000))
  {
    if (crc[0] == crc[1])
    {
      JumpToImageAorB = 1;
      // Simulate a reset for the Application code by an absolute jump to the expected INTVEC addr.
      asm("LJMP 0x3030");
      HAL_SYSTEM_RESET();  // Should not get here.
    }
    
    crcCheck(BIM_IMG_B_PAGE, crc);
  }

  HalFlashRead(BIM_IMG_A_PAGE, BIM_CRC_OSET, (uint8 *)crc, 4);

  if ((crc[0] != 0xFFFF) && (crc[0] != 0x0000))
  {
    if (crc[0] == crc[1])
    {
      JumpToImageAorB = 0;
      // Simulate a reset for the Application code by an absolute jump to the expected INTVEC addr.
      asm("LJMP 0x0830");
      HAL_SYSTEM_RESET();  // Should not get here.
    }
    else if (crc[1] == 0xFFFF)  // If first run of an image that was physically downloaded.
    {
      crcCheck(BIM_IMG_A_PAGE, crc);
    }
  }

  SLEEPCMD |= 0x03;  // PM3, All clock oscillators off, voltage regulator off.
  halSleepExec();
  HAL_SYSTEM_RESET();  // Should not get here.
}


/*******************************************************************************
 * @fn          DMAExecCrc
 *
 * @brief       This function assumes CRC has been initialized and sets up and
 *              starts a dma tranfer from a flash page to the CRC HW module.
 * @note        This function assumes DMA channel 0 is available for use.
 * input parameters
 * @param       page - A valid flash page number.
 * @param       offset - A valid offset into the page.
 * @param       len - A valid number of bytes to calculate crc of.
 * @return      None.
 *******************************************************************************
 */
void DMAExecCrc(uint8 page, uint16 offset, uint16 len) {

  uint8 memctr = MEMCTR;  // Save to restore.
  
  // Calculate the offset into the containing flash bank as it gets mapped into XDATA.
  uint16 address = (offset + HAL_FLASH_PAGE_MAP) +
                   ((page % HAL_FLASH_PAGE_PER_BANK) * HAL_FLASH_PAGE_SIZE);

  // Pointer to DMA config structure
  halDMADesc_t *dmaCh0_p = &dmaCh0;
  
#if !defined HAL_OAD_BOOT_CODE
  halIntState_t is;
#endif

  page /= HAL_FLASH_PAGE_PER_BANK;  // Calculate the flash bank from the flash page.

#if !defined HAL_OAD_BOOT_CODE
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
  
  // Tell DMA Controller where above configuration can be found
  HAL_DMA_SET_ADDR_DESC0(&dmaCh0);
  
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

#if !defined HAL_OAD_BOOT_CODE
  HAL_EXIT_CRITICAL_SECTION(is);
#endif
}


/*******************************************************************************
*/

/***************************************************************************
**+----------------------------------------------------------------------+**
**|                                ****                                  |**
**|                                ****                                  |**
**|                                ******o***                            |**
**|                          ********_///_****                           |**
**|                           ***** /_//_/ ****                          |**
**|                            ** ** (__/ ****                           |**
**|                                *********                             |**
**|                                 ****                                 |**
**|                                  ***                                 |**
**|                                                                      |**
**|     Copyright (c) 1998 - 2010 Texas Instruments Incorporated         |**
**|                        ALL RIGHTS RESERVED                           |**
**|                                                                      |**
**| Permission is hereby granted to licensees of Texas Instruments       |**
**| Incorporated (TI) products to use this computer program for the sole |**
**| purpose of implementing a licensee product based on TI products.     |**
**| No other rights to reproduce, use, or disseminate this computer      |**
**| program, whether in part or in whole, are granted.                   |**
**|                                                                      |**
**| TI makes no representation or warranties with respect to the         |**
**| performance of this computer program, and specifically disclaims     |**
**| any responsibility for any damages, special or consequential,        |**
**| connected with the use of this program.                              |**
**|                                                                      |**
**+----------------------------------------------------------------------+**
***************************************************************************/

/** \file   SdioDrv.c 
 *  \brief  The OMAP3530 Linux SDIO driver (platform and OS dependent) 
 * 
 * The lower SDIO driver (BSP) for OMAP3530 on Linux OS.
 * Provides all SDIO commands and read/write operation methods.
 *  
 *  \see    SdioDrv.h
 */

/*
 * modification history
 * --------------------
 * \version xxx,DDMMMYY,yyy details of modification
 */
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/delay.h>
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,22)
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,29)
#include <linux/i2c/twl.h>
#include <plat/board.h>
#include <plat/clock.h>
#include <plat/dma.h>

#else
#include <mach/io.h>
#include <mach/hardware.h>
#include <linux/i2c/twl4030.h>
#include <mach/board.h>
#include <mach/clock.h>
#include <mach/dma.h>
#endif
#else
#include <asm/arch/io.h>
#include <asm/arch/hardware.h>
#include <asm/arch/twl4030.h>
#include <asm/arch/board.h>
#include <asm/arch/clock.h>
#include <asm/arch/dma.h>
#endif
#include <linux/types.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <asm/io.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <asm/io.h>

#include "omap34xx_defs.h"
#include "omap3430_sdiodrv_debug.h"
#include "SdioDrv.h"

/************************************************************************
 * Defines
 ************************************************************************/

#define TIWLAN_MMC_CONTROLLER               2
#define TIWLAN_MMC_CONTROLLER_BASE_ADDR     OMAP_HSMMC2_BASE
#define TIWLAN_MMC_CONTROLLER_BASE_SIZE     512

#define OMAP_MMC_MASTER_CLOCK               96000000
/*
 *  HSMMC Host Controller Registers
 */
#define OMAP_HSMMC_SYSCONFIG           0x0010
#define OMAP_HSMMC_SYSSTATUS           0x0014
#define OMAP_HSMMC_CSRE                0x0024
#define OMAP_HSMMC_SYSTEST             0x0028
#define OMAP_HSMMC_CON                 0x002C
#define OMAP_HSMMC_BLK                 0x0104
#define OMAP_HSMMC_ARG                 0x0108
#define OMAP_HSMMC_CMD                 0x010C
#define OMAP_HSMMC_RSP10               0x0110
#define OMAP_HSMMC_RSP32               0x0114
#define OMAP_HSMMC_RSP54               0x0118
#define OMAP_HSMMC_RSP76               0x011C
#define OMAP_HSMMC_DATA                0x0120
#define OMAP_HSMMC_PSTATE              0x0124
#define OMAP_HSMMC_HCTL                0x0128
#define OMAP_HSMMC_SYSCTL              0x012C
#define OMAP_HSMMC_STAT                0x0130
#define OMAP_HSMMC_IE                  0x0134
#define OMAP_HSMMC_ISE                 0x0138
#define OMAP_HSMMC_AC12                0x013C
#define OMAP_HSMMC_CAPA                0x0140
#define OMAP_HSMMC_CUR_CAPA            0x0148
#define OMAP_HSMMC_REV                 0x01FC

#define VS18                           (1 << 26)
#define VS30                           (1 << 25)
#define SRA                            (1 << 24)
#define SDVS18                         (0x5 << 9)
#define SDVS30                         (0x6 << 9)
#define SDVSCLR                        0xFFFFF1FF
#define SDVSDET                        0x00000400
#define SIDLE_MODE                     (0x2 << 3)
#define AUTOIDLE                       0x1
#define SDBP                           (1 << 8)
#define DTO                            0xE
#define ICE                            0x1
#define ICS                            0x2
#define CEN                            (1 << 2)
#define CLKD_MASK                      0x0000FFC0
#define IE_EN_MASK                     0x317F0137
#define INIT_STREAM                    (1 << 1)
#define DP_SELECT                      (1 << 21)
#define DDIR                           (1 << 4)
#define DMA_EN                         0x1
#define MSBS                           (1 << 5)
#define BCE                            (1 << 1)
#define ONE_BIT                        (~(0x2))
#define EIGHT_BIT                      (~(0x20))
#define CC                             0x1
#define TC                             0x02
#define OD                             0x1
#define BRW                            0x400
#define BRR                            0x800
#define BRE                            (1 << 11)
#define BWE                            (1 << 10)
#define SBGR                           (1 << 16)
#define CT                             (1 << 17)
#define SDIO_READ                      (1 << 31)
#define SDIO_BLKMODE                   (1 << 27)
#define OMAP_HSMMC_ERR                 (1 << 15)/* Any error */
/* Com mand response time-out */
#define OMAP_HSMMC_CMD_TIMEOUT         (1 << 16)
#define OMAP_HSMMC_DATA_TIMEOUT        (1 << 20)/* Data response time-out */
#define OMAP_HSMMC_CMD_CRC             (1 << 17)/* Command CRC error */
#define OMAP_HSMMC_DATA_CRC            (1 << 21)/* Date CRC error */
#define OMAP_HSMMC_CARD_ERR            (1 << 28)/* Card ERR */
#define OMAP_HSMMC_STAT_CLEAR          0xFFFFFFFF
#define INIT_STREAM_CMD                0x00000000
#define INT_CLEAR                      0x00000000
#define BLK_CLEAR                      0x00000000

/* SCM CONTROL_DEVCONF1 MMC1 overwrite but */

#define MMC1_ACTIVE_OVERWRITE          (1 << 31)

#define sdio_blkmode_regaddr           0x2000
#define sdio_blkmode_mask              0xFF00

#define IO_RW_DIRECT_MASK              0xF000FF00
#define IO_RW_DIRECT_ARG_MASK          0x80001A00

#define RMASK                          (MMC_RSP_MASK | MMC_RSP_CRC)

#define MMC_TIMEOUT_MS                 100

#define MMCA_VSN_4                     4

#define VMMC1_DEV_GRP                  0x27
#define P1_DEV_GRP                     0x20
#define VMMC1_DEDICATED                0x2A
#define VSEL_3V                        0x02
#define VSEL_18V                       0x00
#define PBIAS_3V                       0x03
#define PBIAS_18V                      0x02
#define PBIAS_LITE                     0x04A0
#define PBIAS_CLR                      0x00

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
#define OMAP_MMC_REGS_BASE  OMAP2_L4_IO_ADDRESS(TIWLAN_MMC_CONTROLLER_BASE_ADDR)
#else
#define OMAP_MMC_REGS_BASE  IO_ADDRESS(TIWLAN_MMC_CONTROLLER_BASE_ADDR)
#endif

#define INT_MMC2_IRQ                   86
#define OMAP_MMC_IRQ                   INT_MMC2_IRQ
/* 
 * MMC Host controller read/write API's.
 */
#define OMAP_HSMMC_READ_OFFSET(offset) \
		(__raw_readl((OMAP_MMC_REGS_BASE) + (offset)))
#define OMAP_HSMMC_READ(reg) \
		(__raw_readl((OMAP_MMC_REGS_BASE) + OMAP_HSMMC_##reg))
#define OMAP_HSMMC_WRITE(reg, val) \
		(__raw_writel((val), (OMAP_MMC_REGS_BASE) + OMAP_HSMMC_##reg))

#define OMAP_HSMMC_SEND_COMMAND(cmd, arg) do \
{ \
	OMAP_HSMMC_WRITE(ARG, arg); \
	OMAP_HSMMC_WRITE(CMD, cmd); \
} while (0)

#define OMAP_HSMMC_CMD52_WRITE \
((SD_IO_RW_DIRECT    << 24) | (OMAP_HSMMC_CMD_SHORT_RESPONSE << 16))
#define OMAP_HSMMC_CMD52_READ \
(((SD_IO_RW_DIRECT   << 24) | (OMAP_HSMMC_CMD_SHORT_RESPONSE << 16)) | DDIR)
#define OMAP_HSMMC_CMD53_WRITE \
(((SD_IO_RW_EXTENDED << 24) | (OMAP_HSMMC_CMD_SHORT_RESPONSE << 16)) | \
							DP_SELECT)
#define OMAP_HSMMC_CMD53_READ \
(((SD_IO_RW_EXTENDED << 24) | (OMAP_HSMMC_CMD_SHORT_RESPONSE << 16)) | \
							DP_SELECT | DDIR)
#define OMAP_HSMMC_CMD53_READ_DMA  (OMAP_HSMMC_CMD53_READ  | DMA_EN)
#define OMAP_HSMMC_CMD53_WRITE_DMA (OMAP_HSMMC_CMD53_WRITE | DMA_EN)

/* Macros to build commands 52 and 53 in format according to SDIO spec */
#define SDIO_CMD52_READ(v1,v2,v3,v4) \
(SDIO_RWFLAG(v1)|SDIO_FUNCN(v2)|SDIO_RAWFLAG(v3)| SDIO_ADDRREG(v4))
#define SDIO_CMD52_WRITE(v1,v2,v3,v4,v5) \
(SDIO_RWFLAG(v1)|SDIO_FUNCN(v2)|SDIO_RAWFLAG(v3)| SDIO_ADDRREG(v4)|(v5))
#define SDIO_CMD53_READ(v1,v2,v3,v4,v5,v6) \
(SDIO_RWFLAG(v1)|SDIO_FUNCN(v2)|SDIO_BLKM(v3)| \
				SDIO_OPCODE(v4)|SDIO_ADDRREG(v5)|(v6&0x1ff))
#define SDIO_CMD53_WRITE(v1,v2,v3,v4,v5,v6) \
(SDIO_RWFLAG(v1)|SDIO_FUNCN(v2)|SDIO_BLKM(v3)| \
				SDIO_OPCODE(v4)|SDIO_ADDRREG(v5)|(v6&0x1ff))

#define SDIODRV_MAX_LOOPS   1000000

#define VMMC2_DEV_GRP		0x2B
#define VMMC2_DEDICATED		0x2E
#define VSEL_S2_18V		0x05
#define LDO_CLR			0x00
#define VSEL_S2_CLR		0x40
#define GPIO_0_BIT_POS		1 << 0
#define GPIO_1_BIT_POS		1 << 1
#define VSIM_DEV_GRP		0x37
#define VSIM_DEDICATED		0x3A
#define TWL4030_MODULE_PM_RECIEVER	0x13

/********************************************************************/
/*	SDIO driver parameters and structures			    */
/********************************************************************/

typedef struct OMAP3530_sdiodrv {
	struct clk *fclk, *iclk, *dbclk;
	int dma_tx_channel;
	int dma_rx_channel;
	int irq;
	void (*BusTxnCB) (void *BusTxnHandle, int status);
	void *BusTxnHandle;
	unsigned int uBlkSize;
	unsigned int uBlkSizeShift;
	int async_status;
	int (*wlanDrvIf_pm_resume) (void);
	int (*wlanDrvIf_pm_suspend) (void);
	struct device *dev;
	dma_addr_t dma_read_addr;
	size_t dma_read_size;
	dma_addr_t dma_write_addr;
	size_t dma_write_size;

} OMAP3530_sdiodrv_t;

module_param(g_sdio_debug_level, int, 0644);
MODULE_PARM_DESC(g_sdio_debug_level, "debug level");

int g_sdio_debug_level = SDIO_DEBUGLEVEL_INFO;
EXPORT_SYMBOL(g_sdio_debug_level);

OMAP3530_sdiodrv_t g_drv;
struct platform_device adhoc_mmc2;
static int sdiodrv_irq_requested = 0;
static int sdiodrv_mmc_power_ena = 0;
static int sdiodrv_dma_on = 0;
static int sdiodrv_iclk_enable = 0;
static int sdiodrv_fclk_enable = 0;
static int sdiodrv_suspend_enable = 0;
static int initCount = 0;

struct work_struct sdiodrv_work;
/* The sdio work queue */
static struct workqueue_struct *pWorkQueue;

#define SDIO_DRV_WK_NAME "ti_sdio_drv"
#define SDIO_DRIVER_NAME            "TIWLAN_SDIO"

/********************************************************************/
/*  SDIO driver routines                                            */
/********************************************************************/

static int enable_mmc_power(int slot)
{
	int ret = 0;

	PDEBUG("Enable_mmc_power() %d\n", slot);

	if (slot == 2) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
		ret = twl_i2c_write_u8(TWL4030_MODULE_PM_RECIEVER,
				       P1_DEV_GRP, VMMC2_DEV_GRP);
#else
		ret = twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECIEVER,
					   P1_DEV_GRP, VMMC2_DEV_GRP);
#endif
		if (ret != 0) {
			PERR("Configuring MMC2 device group  failed\n");
			return ret;
		}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
		ret = twl_i2c_write_u8(TWL4030_MODULE_PM_RECIEVER,
				       VSEL_S2_18V, VMMC2_DEDICATED);
#else
		ret = twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECIEVER,
					   VSEL_S2_18V, VMMC2_DEDICATED);
#endif

		if (ret != 0) {
			PERR("Configuring MMC2 dedicated  failed\n");
			return ret;
		}
	} else {
		PERR("Wrong mmc controller\n");
		return -1;
	}

	return 0;

} /* end of enable_mmc_power */

static int disable_mmc_power(int slot)
{
	int ret = 0;

	PDEBUG("Disable_mmc_power() %d\n", slot);

	if (slot == 2) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
		ret = twl_i2c_write_u8(TWL4030_MODULE_PM_RECIEVER,
				       LDO_CLR, VMMC2_DEV_GRP);
#else
		ret = twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECIEVER,
					   LDO_CLR, VMMC2_DEV_GRP);
#endif

		if (ret != 0) {
			PERR("Configuring MMC2 dev grp failed\n");
			return ret;
		}
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
		ret = twl_i2c_write_u8(TWL4030_MODULE_PM_RECIEVER,
				       VSEL_S2_CLR, VMMC2_DEDICATED);
#else
		ret = twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECIEVER,
					   VSEL_S2_CLR, VMMC2_DEDICATED);
#endif
		if (ret != 0) {
			PERR("Configuring MMC2 dedicated failed\n");
			return ret;
		}
	} else {
		PERR("Wrong mmc controller\n");
		return -1;
	}

	return 0;
}/* end of disable_mmc_power */

static void sdiodrv_task(struct work_struct *unused)
{

	if (g_drv.dma_read_addr != 0) {
		dma_unmap_single(g_drv.dev,
				 g_drv.dma_read_addr,
				 g_drv.dma_read_size, DMA_FROM_DEVICE);
		g_drv.dma_read_addr = 0;
		g_drv.dma_read_size = 0;
	}

	if (g_drv.dma_write_addr != 0) {
		dma_unmap_single(g_drv.dev,
				 g_drv.dma_write_addr,
				 g_drv.dma_write_size, DMA_TO_DEVICE);
		g_drv.dma_write_addr = 0;
		g_drv.dma_write_size = 0;
	}

	if (g_drv.BusTxnCB != NULL) {
		g_drv.BusTxnCB(g_drv.BusTxnHandle, g_drv.async_status);
	}

}/* end of sdiodrv_task() */

/********************************************************************/
/*	SDIO driver interrupt handling                              */
/********************************************************************/

static irqreturn_t sdiodrv_irq(int irq, void *drv)
{
	int status;

	PDEBUG("sdiodrv_irq()\n");

	status = OMAP_HSMMC_READ(STAT);
	OMAP_HSMMC_WRITE(ISE, 0);
	g_drv.async_status = status & (OMAP_HSMMC_ERR);

	if (g_drv.async_status) {
		PERR("sdiodrv_irq: ERROR in STAT = 0x%x\n", status);
	}

	status = queue_work(pWorkQueue, &sdiodrv_work);
	if (!status) {
		PERR("ERROR in sdiodrv_irq: queue_work fails "
		     "with status = %d \n", status);
	}

	/* Flush posted write */
	OMAP_HSMMC_READ(STAT);

	return IRQ_HANDLED;

}/*end of sdiodrv_irq() */

/********************************************************************/
/*	SDIO driver internal functions                                  */
/********************************************************************/

/*------------------------------------------------------------------*/
/*==================================== DMA stuff ===================*/
/*------------------------------------------------------------------*/

static void sdiodrv_dma_cb(int lch, u16 ch_status, void *data)
{
	int status;

	PDEBUG("sdiodrv_dma_cb() channel=%d status=0x%x\n",
	       lch, (int)ch_status);

	g_drv.async_status = ch_status & (1 << 7);

	status = queue_work(pWorkQueue, &sdiodrv_work);
	if (!status) {
		PERR("ERROR in sdiodrv_dma_cb: queue_work fails "
		     "with status = %d \n", status);
	}

}/* end of sdiodrv_dma_cb() */

/*------------------------------------------------------------------*/

static int sdiodrv_dma_init(void)
{
	int rc;

	rc = omap_request_dma(OMAP24XX_DMA_MMC2_TX,
			      "SDIO WRITE",
			      NULL, &g_drv, &g_drv.dma_tx_channel);
	if (rc != 0) {
		PERR("sdiodrv_dma_init() omap_request_dma"
		     "(OMAP24XX_DMA_MMC2_TX) FAILED\n");
		return rc;
	}

	rc = omap_request_dma(OMAP24XX_DMA_MMC2_RX,
			      "SDIO READ",
			      sdiodrv_dma_cb, &g_drv, &g_drv.dma_rx_channel);
	if (rc != 0) {
		PERR("sdiodrv_dma_init() omap_request_dma"
		     "(OMAP24XX_DMA_MMC2_RX) FAILED\n");
		omap_free_dma(g_drv.dma_tx_channel);
		return rc;
	}
	/* src_port is only for OMAP1 */
	omap_set_dma_src_params(g_drv.dma_rx_channel, 0,
				OMAP_DMA_AMODE_CONSTANT,
				(OMAP_HSMMC2_BASE) + OMAP_HSMMC_DATA, 0, 0);
	/* dest_port is only for OMAP1 */
	omap_set_dma_dest_params(g_drv.dma_tx_channel, 0,
				 OMAP_DMA_AMODE_CONSTANT,
				 (OMAP_HSMMC2_BASE) + OMAP_HSMMC_DATA, 0, 0);

	return rc;

}/* end of sdiodrv_dma_init() */

/*------------------------------------------------------------------*/

static void sdiodrv_dma_shutdown(void)
{
	omap_free_dma(g_drv.dma_tx_channel);
	omap_free_dma(g_drv.dma_rx_channel);

}/* end of sdiodrv_dma_shutdown() */

/*------------------------------------------------------------------*/
/*================================= End of DMA stuff ===============*/
/*------------------------------------------------------------------*/

static u32 sdiodrv_poll_status(u32 reg_offset, u32 stat, unsigned int msecs)
{
	u32 status = 0, loops = 0;

	do {
		status = OMAP_HSMMC_READ_OFFSET(reg_offset);
		if ((status & stat)) {
			break;
		}
	} while (loops++ < SDIODRV_MAX_LOOPS);

	return status;

}/* end of sdiodrv_poll_status */

/*------------------------------------------------------------------*/

//cmd flow p. 3609 obc
static int sdiodrv_send_command(u32 cmdreg, u32 cmdarg)
{
	OMAP_HSMMC_WRITE(STAT, OMAP_HSMMC_STAT_CLEAR);
	OMAP_HSMMC_SEND_COMMAND(cmdreg, cmdarg);

	return sdiodrv_poll_status(OMAP_HSMMC_STAT, CC, MMC_TIMEOUT_MS);

}/* end of sdiodrv_send_command() */

/*------------------------------------------------------------------*/
/*
 *  Disable clock to the card
 */
static void OMAP3530_mmc_stop_clock(void)
{
	OMAP_HSMMC_WRITE(SYSCTL, OMAP_HSMMC_READ(SYSCTL) & ~CEN);
	if ((OMAP_HSMMC_READ(SYSCTL) & CEN) != 0x0) {
		PERR("MMC clock not stoped, clock freq can not be altered\n");
	}
}/* end of OMAP3530_mmc_stop_clock */

/*------------------------------------------------------------------*/
/*
 *  Reset the SD system
 */

static int OMAP3530_mmc_reset(void)
{
	int status, loops = 0;

	/* p. 3598 - need to set SOFTRESET to 0x1 0bc */
	OMAP_HSMMC_WRITE(SYSCTL, OMAP_HSMMC_READ(SYSCTL) | SRA);
	while ((status = OMAP_HSMMC_READ(SYSCTL) & SRA) &&
	       loops++ < SDIODRV_MAX_LOOPS) ;

	if (status & SRA) {
		PERR("OMAP3530_mmc_reset() MMC reset FAILED!! "
		     "status=0x%x\n", status);
	}

	return status;

}/* end of OMAP3530_mmc_reset */

/*------------------------------------------------------------------*/
static void OMAP3530_mmc_set_clock(unsigned int clock,
				   OMAP3530_sdiodrv_t * host)
{
	u16 dsor = 0;
	unsigned long regVal;
	int status;

	PDEBUG("OMAP3530_mmc_set_clock(%d)\n", clock);

	if (clock) {
		/* Enable MMC_SD_CLK */
		dsor = OMAP_MMC_MASTER_CLOCK / clock;

		if (dsor < 1) {
			dsor = 1;
		}

		if (OMAP_MMC_MASTER_CLOCK / dsor > clock) {
			dsor++;
		}

		if (dsor > 250) {
			dsor = 250;
		}
	}

	OMAP3530_mmc_stop_clock();

	regVal = OMAP_HSMMC_READ(SYSCTL);
	regVal = regVal & ~(CLKD_MASK);
	regVal = regVal | (dsor << 6);
	regVal = regVal | (DTO << 16);
	OMAP_HSMMC_WRITE(SYSCTL, regVal);
	/* internal clock enable. obc not mentioned in the spec */
	OMAP_HSMMC_WRITE(SYSCTL, OMAP_HSMMC_READ(SYSCTL) | ICE);
	/* 
	 * wait till the the clock is satble (ICS) bit is set
	 */
	status = sdiodrv_poll_status(OMAP_HSMMC_SYSCTL, ICS, MMC_TIMEOUT_MS);
	if (!(status & ICS)) {
		PERR("OMAP3530_mmc_set_clock() clock not stable!! "
		     "status=0x%x\n", status);
	}

	/* 
	 * Enable clock to the card
	 */
	OMAP_HSMMC_WRITE(SYSCTL, OMAP_HSMMC_READ(SYSCTL) | CEN);

}/* end of OMAP3530_mmc_set_clock() */

static void sdiodrv_free_resources(void)
{
	/* Disable the interface clock and functional clock */
	if (sdiodrv_fclk_enable) {
		omap_writel(omap_readl(CM_FCLKEN1_CORE) & (0xFBFFFFFF),
			    CM_FCLKEN1_CORE);
		sdiodrv_fclk_enable = 0;
	}
	if (sdiodrv_iclk_enable) {
		omap_writel(omap_readl(CM_ICLKEN1_CORE) & (0xFBFFFFFF),
			    CM_ICLKEN1_CORE);
		sdiodrv_iclk_enable = 0;
	}

	if (sdiodrv_irq_requested) {
		free_irq(OMAP_MMC_IRQ, &g_drv);
		sdiodrv_irq_requested = 0;
	}

	if (sdiodrv_mmc_power_ena) {
		disable_mmc_power(TIWLAN_MMC_CONTROLLER);
		sdiodrv_mmc_power_ena = 0;
	}
}/* end of sdiodrv_free_resources() */

static int sdioDrv_InitHw(void)
{
	int rc;
	u32 status;

	struct platform_device *pdev;

	unsigned long clock_rate = 24000000;	/* SDIO-CLK = 24MHz */

	adhoc_mmc2.name = "adhoc_mmc2";
	adhoc_mmc2.id = 2;
	adhoc_mmc2.dev.bus = &platform_bus_type;
	adhoc_mmc2.num_resources = 0;
	adhoc_mmc2.resource = NULL;

	pdev = (struct platform_device *)(&adhoc_mmc2);

	/* remember device struct for future DMA operations */
	g_drv.dev = &pdev->dev;

	rc = enable_mmc_power(TIWLAN_MMC_CONTROLLER);
	if (rc != 0) {
		PERR("enable_mmc_power() returned %d !!!\n", rc);
		goto err;
	}
	sdiodrv_mmc_power_ena = 1;

	PDEBUG("Setting SDIO F&I clock Configuration\n");

	omap_writel(omap_readl(CM_ICLKEN1_CORE) | (1 << 25), CM_ICLKEN1_CORE);
	omap_writel(omap_readl(CM_FCLKEN1_CORE) | (1 << 25), CM_FCLKEN1_CORE);
	sdiodrv_iclk_enable = 1;
	sdiodrv_fclk_enable = 1;
	/* Wait */
	udelay(400);

	omap_writel(omap_readl(OMAP2_CONTROL_DEVCONF1) | (1 << 6),
		    OMAP2_CONTROL_DEVCONF1);

	PDEBUG("Done setting SDIO F&I clock Configuration\n");

	OMAP3530_mmc_reset();

	/* obc - init sequence p. 3600,3617 */
	/* 1.8V */
	OMAP_HSMMC_WRITE(CAPA, OMAP_HSMMC_READ(CAPA) | VS18);
	//SDVS fits p. 3650
	OMAP_HSMMC_WRITE(HCTL, OMAP_HSMMC_READ(HCTL) | SDVS18);

	/* clock gating */
	OMAP_HSMMC_WRITE(SYSCONFIG, OMAP_HSMMC_READ(SYSCONFIG) | AUTOIDLE);

	/* bus power */
	/* SDBP fits p. 3650 */
	OMAP_HSMMC_WRITE(HCTL, OMAP_HSMMC_READ(HCTL) | SDBP);

	/* interrupts */
	OMAP_HSMMC_WRITE(ISE, 0);
	OMAP_HSMMC_WRITE(IE, IE_EN_MASK);

	/* p. 3601 suggests moving to the end */
	OMAP3530_mmc_set_clock(clock_rate, &g_drv);
	PINFO("SDIO clock Configuration is now set to %dMhz\n",
	      (int)clock_rate / 1000000);

	/* Bus width */
	PDEBUG("%s() setting %d data lines\n", __FUNCTION__, 4);
	/* DTW 4 bits - p. 3650 */
	OMAP_HSMMC_WRITE(HCTL, OMAP_HSMMC_READ(HCTL) | (1 << 1));

	rc = request_irq(OMAP_MMC_IRQ,
			 sdiodrv_irq, 0, SDIO_DRIVER_NAME, &g_drv);
	if (rc != 0) {
		PERR("sdioDrv_InitHw() - request_irq FAILED!!\n");
		goto err;
	}
	sdiodrv_irq_requested = 1;

	if (sdiodrv_dma_on == 0) {
		rc = sdiodrv_dma_init();
		if (rc != 0) {
			PERR("sdiodrv_init() - sdiodrv_dma_init FAILED!!\n");
			goto err;
		}
		sdiodrv_dma_on = 1;
	}

	/* 
	 * send the init sequence. 80 clocks of synchronization
	 * in the SDIO 
	 */

	/* doesn't match p. 3601,3617 - obc */
	OMAP_HSMMC_WRITE(CON, OMAP_HSMMC_READ(CON) | INIT_STREAM);
	OMAP_HSMMC_SEND_COMMAND(0, 0);

	status = sdiodrv_poll_status(OMAP_HSMMC_STAT, CC, MMC_TIMEOUT_MS);
	if (!(status & CC)) {
		PERR("sdioDrv_InitHw() SDIO Command error status = 0x%x\n",
		     status);
		rc = -1;
		goto err;
	}

	OMAP_HSMMC_WRITE(CON, OMAP_HSMMC_READ(CON) & ~INIT_STREAM);

	return 0;

err:
	sdiodrv_free_resources();
	return rc;

}/* end of sdioDrv_InitHw */

static void sdiodrv_shutdown(void)
{
	PDEBUG("entering %s()\n", __FUNCTION__);

	sdiodrv_free_resources();
	if (sdiodrv_dma_on) {
		sdiodrv_dma_shutdown();
		sdiodrv_dma_on = 0;
	}

	PDEBUG("exiting %s\n", __FUNCTION__);
}/* end of sdiodrv_shutdown() */

/*------------------------------------------------------------------*/

static int sdiodrv_send_data_xfer_commad(u32 cmd,
					 u32 cmdarg,
					 int length,
					 u32 buffer_enable_status,
					 unsigned int bBlkMode)
{
	int status;

	PDEBUG("%s() writing CMD 0x%x ARG 0x%x\n", __FUNCTION__, cmd, cmdarg);

	/* block mode */
	if (bBlkMode) {
		/* 
		 * Bits 31:16 of BLK reg: 
		 * NBLK Blocks count for current transfer.
		 * in case of Block MOde the lenght is treated here 
		 * as number of blocks (and not as a length).
		 * 
		 * Bits 11:0 of BLK reg: 
		 * BLEN Transfer Block Size. in case of block mode
		 * set that field to block size. 
		 */
		OMAP_HSMMC_WRITE(BLK, (length << 16) | (g_drv.uBlkSize << 0));

		/*
		 * In CMD reg:
		 * BCE: Block Count Enable
		 * MSBS: Multi/Single block select
		 */
		cmd |= MSBS | BCE;
	} else {
		OMAP_HSMMC_WRITE(BLK, length);
	}

	status = sdiodrv_send_command(cmd, cmdarg);
	if (!(status & CC)) {
		PERR("sdiodrv_send_data_xfer_commad() "
		     "SDIO Command error! STAT = 0x%x\n", status);
		return 0;
	}

	PDEBUG("%s() length = %d(%dw) BLK = 0x%x\n",
	       __FUNCTION__, length, ((length + 3) >> 2), OMAP_HSMMC_READ(BLK));

	return sdiodrv_poll_status(OMAP_HSMMC_PSTATE,
				   buffer_enable_status, MMC_TIMEOUT_MS);

}/* end of sdiodrv_send_data_xfer_commad() */

/*------------------------------------------------------------------*/

static int sdiodrv_data_xfer_sync(u32 cmd,
				  u32 cmdarg,
				  void *data,
				  int length, u32 buffer_enable_status)
{
	u32 buf_start, buf_end, data32;
	int status;

	PDEBUG("%s. cmd=0x%08x, cmdarg=0x%08x, length=%d.\n",
	       __FUNCTION__, cmd, cmdarg, length);

	status = sdiodrv_send_data_xfer_commad(cmd,
					       cmdarg,
					       length, buffer_enable_status, 0);
	if (!(status & buffer_enable_status)) {
		PERR("sdiodrv_data_xfer_sync() buffer disabled! "
		     "length = %d BLK = 0x%x PSTATE = 0x%x\n",
		     length, OMAP_HSMMC_READ(BLK), status);
		return -1;
	}

	buf_end = (u32) data + (u32) length;

	/* obc need to check BRE/BWE every time, see p. 3605 */
	/*
	 * Read loop 
	 */
	if (buffer_enable_status == BRE) {
		if (((u32) data & 3) == 0) {	/* 4 bytes aligned */
			for (buf_start = (u32) data;
			     (u32) data < buf_end;
			     data += sizeof(unsigned long)) {
				*((unsigned long *)(data)) =
				    OMAP_HSMMC_READ(DATA);
			}
		} else {	/* 2 bytes aligned */

			for (buf_start = (u32) data;
			     (u32) data < buf_end;
			     data += sizeof(unsigned long)) {
				data32 = OMAP_HSMMC_READ(DATA);
				*((unsigned short *)data) =
				    (unsigned short)data32;
				*((unsigned short *)data + 1) =
				    (unsigned short)(data32 >> 16);
			}
		}
	}

	/*
	 * Write loop 
	 */
	else {
		if (((u32) data & 3) == 0) {	/* 4 bytes aligned */
			for (buf_start = (u32) data;
			     (u32) data < buf_end;
			     data += sizeof(unsigned long)) {
				OMAP_HSMMC_WRITE(DATA,
						 *((unsigned long *)(data)));
			}
		} else {	/* 2 bytes aligned */

			for (buf_start = (u32) data;
			     (u32) data < buf_end;
			     data += sizeof(unsigned long)) {
				OMAP_HSMMC_WRITE(DATA,
						 *((unsigned short *)data) |
						 *((unsigned short *)data +
						   1) << 16);
			}
		}
	}

	status = sdiodrv_poll_status(OMAP_HSMMC_STAT, TC, MMC_TIMEOUT_MS);
	if (!(status & TC)) {
		PERR("sdiodrv_data_xfer_sync() transfer error! "
		     "STAT = 0x%x\n", status);
		return -1;
	}

	return 0;

}				/* end of sdiodrv_data_xfer_sync() */

/*------------------------------------------------------------------*/

/********************************************************************/
/*	SDIO driver interface functions                             */
/********************************************************************/
/*------------------------------------------------------------------*/

int sdioDrv_ConnectBus(void *fCbFunc,
		       void *hCbArg,
		       unsigned int uBlkSizeShift,
		       unsigned int uSdioThreadPriority)
{
	g_drv.BusTxnCB = fCbFunc;
	g_drv.BusTxnHandle = hCbArg;
	g_drv.uBlkSizeShift = uBlkSizeShift;
	g_drv.uBlkSize = 1 << uBlkSizeShift;

	if (0 == initCount) {
		initCount = 1;
		pWorkQueue = create_singlethread_workqueue(SDIO_DRV_WK_NAME);
		INIT_WORK(&sdiodrv_work, sdiodrv_task);
	}

	return sdioDrv_InitHw();
}/* end of sdioDrv_ConnectBus() */

/*------------------------------------------------------------------*/

int sdioDrv_DisconnectBus(void)
{
	sdiodrv_free_resources();
	return 0;
}/* end of sdioDrv_DisconnectBus() */

/*------------------------------------------------------------------*/
int sdioDrv_ExecuteCmd(unsigned int uCmd,
		       unsigned int uArg,
		       unsigned int uRespType,
		       void *pResponse, unsigned int uLen)
{
	unsigned int uCmdReg = 0;
	unsigned int uStatus = 0;
	unsigned int uResponse = 0;

	PDEBUG("sdioDrv_ExecuteCmd() starting cmd %02x arg %08x\n",
	       (int)uCmd, (int)uArg);

	uCmdReg = (uCmd << 24) | (uRespType << 16);

	uStatus = sdiodrv_send_command(uCmdReg, uArg);
	if (!(uStatus & CC)) {
		PERR("sdioDrv_ExecuteCmd() SDIO Command error "
		     "status = 0x%x\n", uStatus);
		return -1;
	}

	/*obc what happends if uLen > 4 ? shouldn't read anything ? */
	if ((uLen > 0) && (uLen <= 4)) {
		uResponse = OMAP_HSMMC_READ(RSP10);
		memcpy(pResponse, (char *)&uResponse, uLen);
		PDEBUG("sdioDrv_ExecuteCmd() response = 0x%x\n", uResponse);
	}

	return 0;
}/* end of sdioDrv_ExecuteCmd() */

/*------------------------------------------------------------------*/

int sdioDrv_ReadSync(unsigned int uFunc,
		     unsigned int uHwAddr,
		     void *pData,
		     unsigned int uLen,
		     unsigned int bIncAddr, unsigned int bMore)
{
	unsigned int uCmdArg;
	int iStatus;

	uCmdArg = SDIO_CMD53_READ(0, uFunc, 0, bIncAddr, uHwAddr, uLen);

	iStatus = sdiodrv_data_xfer_sync(OMAP_HSMMC_CMD53_READ,
					 uCmdArg, pData, uLen, BRE);
	if (iStatus != 0) {
		PERR("sdioDrv_ReadSync() FAILED!!\n");
	}

	return iStatus;
}/* end of sdioDrv_ReadSync() */

int sdioDrv_ReadAsync(unsigned int uFunc,
		      unsigned int uHwAddr,
		      void *pData,
		      unsigned int uLen,
		      unsigned int bBlkMode,
		      unsigned int bIncAddr, unsigned int bMore)
{
	int iStatus;
	unsigned int uCmdArg;
	unsigned int uNumBlks;
	unsigned int uDmaBlockCount;
	unsigned int uNumOfElem;
	dma_addr_t dma_bus_address;

	if (bBlkMode) {
		/* 
		 * For block mode use number of blocks 
		 * instead of length in bytes
		 */
		uNumBlks = uLen >> g_drv.uBlkSizeShift;
		uDmaBlockCount = uNumBlks;
		/* 
		 * due to the DMA config to 32Bit per element 
		 *(OMAP_DMA_DATA_TYPE_S32) the division is by 4 
		 */
		uNumOfElem = g_drv.uBlkSize >> 2;
	} else {
		uNumBlks = uLen;
		uDmaBlockCount = 1;
		uNumOfElem = (uLen + 3) >> 2;
	}

	uCmdArg = SDIO_CMD53_READ(0,
				  uFunc, bBlkMode, bIncAddr, uHwAddr, uNumBlks);

	iStatus = sdiodrv_send_data_xfer_commad(OMAP_HSMMC_CMD53_READ_DMA,
						uCmdArg,
						uNumBlks, BRE, bBlkMode);

	if (!(iStatus & BRE)) {
		PERR("sdioDrv_ReadAsync() buffer disabled! "
		     "length = %d BLK = 0x%x PSTATE = 0x%x, BlkMode = %d\n",
		     uLen, OMAP_HSMMC_READ(BLK), iStatus, bBlkMode);
		goto err;
	}

	PDEBUG("sdiodrv_read_async() dma_ch=%d \n", g_drv.dma_rx_channel);

	dma_bus_address = dma_map_single(g_drv.dev,
					 pData, uLen, DMA_FROM_DEVICE);

	if (!dma_bus_address) {
		PERR("sdioDrv_ReadAsync: dma_map_single failed\n");
		goto err;
	}

	if (g_drv.dma_read_addr != 0) {
		PERR("sdioDrv_ReadAsync: previous DMA op is not finished!\n");
		BUG();
	}

	g_drv.dma_read_addr = dma_bus_address;
	g_drv.dma_read_size = uLen;
	/* dest_port is only for OMAP1 */
	omap_set_dma_dest_params(g_drv.dma_rx_channel, 0,
				 OMAP_DMA_AMODE_POST_INC,
				 dma_bus_address, 0, 0);

	omap_set_dma_transfer_params(g_drv.dma_rx_channel,
				     OMAP_DMA_DATA_TYPE_S32,
				     uNumOfElem,
				     uDmaBlockCount,
				     OMAP_DMA_SYNC_FRAME,
				     OMAP24XX_DMA_MMC2_RX, OMAP_DMA_SRC_SYNC);

	omap_start_dma(g_drv.dma_rx_channel);

	/* Continued at sdiodrv_irq() after DMA transfer is finished */
	return 0;

err:
	return -1;

}/* end of sdioDrv_ReadAsync() */

/*------------------------------------------------------------------*/

int sdioDrv_WriteSync(unsigned int uFunc,
		      unsigned int uHwAddr,
		      void *pData,
		      unsigned int uLen,
		      unsigned int bIncAddr, unsigned int bMore)
{
	unsigned int uCmdArg;
	int iStatus;

	PDEBUG("WriteSync: func=%d. blkMode=0. Opcode=%d."
	       " HwAddr=0x%x. len=%d", uFunc, bIncAddr, uHwAddr, uLen);

	uCmdArg = SDIO_CMD53_WRITE(1, uFunc, 0, bIncAddr, uHwAddr, uLen);
	iStatus = sdiodrv_data_xfer_sync(OMAP_HSMMC_CMD53_WRITE,
					 uCmdArg, pData, uLen, BWE);
	if (iStatus != 0) {
		PERR("sdioDrv_WriteSync() FAILED!!\n");
	}

	return iStatus;
}/* end of sdioDrv_WriteSync() */

int sdioDrv_WriteAsync(unsigned int uFunc,
		       unsigned int uHwAddr,
		       void *pData,
		       unsigned int uLen,
		       unsigned int bBlkMode,
		       unsigned int bIncAddr, unsigned int bMore)
{
	int iStatus;
	unsigned int uCmdArg;
	unsigned int uNumBlks;
	unsigned int uDmaBlockCount;
	unsigned int uNumOfElem;
	dma_addr_t dma_bus_address;

	if (bBlkMode) {
		/* 
		 * For block mode use number of blocks 
		 * instead of length in bytes
		 */
		uNumBlks = uLen >> g_drv.uBlkSizeShift;
		uDmaBlockCount = uNumBlks;
		/* 
		 * due to the DMA config to 32Bit per element 
		 *(OMAP_DMA_DATA_TYPE_S32) the division is by 4 
		 */
		uNumOfElem = g_drv.uBlkSize >> 2;
	} else {
		uNumBlks = uLen;
		uDmaBlockCount = 1;
		uNumOfElem = (uLen + 3) >> 2;
	}

	uCmdArg = SDIO_CMD53_WRITE(1,
				   uFunc,
				   bBlkMode, bIncAddr, uHwAddr, uNumBlks);

	iStatus =
	    sdiodrv_send_data_xfer_commad(OMAP_HSMMC_CMD53_WRITE_DMA,
					  uCmdArg, uNumBlks, BWE, bBlkMode);
	if (!(iStatus & BWE)) {
		PERR("sdioDrv_WriteAsync() buffer disabled! "
		     "length = %d, BLK = 0x%x, Status = 0x%x\n",
		     uLen, OMAP_HSMMC_READ(BLK), iStatus);
		goto err;
	}

	OMAP_HSMMC_WRITE(ISE, TC);
	dma_bus_address = dma_map_single(g_drv.dev, pData,
					uLen, DMA_TO_DEVICE);

	if (!dma_bus_address) {
		PERR("sdioDrv_WriteAsync: dma_map_single failed\n");
		goto err;
	}

	if (g_drv.dma_write_addr != 0) {
		PERR("sdioDrv_WriteAsync: previous DMA op is not finished!\n");
		BUG();
	}

	g_drv.dma_write_addr = dma_bus_address;
	g_drv.dma_write_size = uLen;

	omap_set_dma_src_params(g_drv.dma_tx_channel, 0,
				OMAP_DMA_AMODE_POST_INC,
				dma_bus_address, 0, 0);

	omap_set_dma_transfer_params(g_drv.dma_tx_channel,
				     OMAP_DMA_DATA_TYPE_S32,
				     uNumOfElem,
				     uDmaBlockCount,
				     OMAP_DMA_SYNC_FRAME,
				     OMAP24XX_DMA_MMC2_TX, OMAP_DMA_DST_SYNC);

	omap_start_dma(g_drv.dma_tx_channel);

	/* Continued at sdiodrv_irq() after DMA transfer is finished */
	return 0;

err:
	return -1;
}/* end of sdioDrv_WriteAsync() */

/*------------------------------------------------------------------*/

int sdioDrv_ReadSyncBytes(unsigned int uFunc,
			  unsigned int uHwAddr,
			  unsigned char *pData,
			  unsigned int uLen, unsigned int bMore)
{
	unsigned int uCmdArg;
	unsigned int i;
	int iStatus;

	for (i = 0; i < uLen; i++) {
		uCmdArg = SDIO_CMD52_READ(0, uFunc, 0, uHwAddr);

		iStatus = sdiodrv_send_command(OMAP_HSMMC_CMD52_READ, uCmdArg);

		if (!(iStatus & CC)) {
			PERR("sdioDrv_ReadSyncBytes() SDIO Command "
			     "error status = 0x%x\n", iStatus);
			return -1;
		} else {
			*pData = (unsigned char)(OMAP_HSMMC_READ(RSP10));
		}

		uHwAddr++;
		pData++;
	}

	return 0;
}/* end of sdioDrv_ReadSyncBytes() */

/*-------------------------------------------------------------------*/

int sdioDrv_WriteSyncBytes(unsigned int uFunc,
			   unsigned int uHwAddr,
			   unsigned char *pData,
			   unsigned int uLen, unsigned int bMore)
{
	unsigned int uCmdArg;
	unsigned int i;
	int iStatus;

	for (i = 0; i < uLen; i++) {
		uCmdArg = SDIO_CMD52_WRITE(1, uFunc, 0, uHwAddr, *pData);

		iStatus = sdiodrv_send_command(OMAP_HSMMC_CMD52_WRITE, 
								uCmdArg);

		if (!(iStatus & CC)) {
			PERR("sdioDrv_WriteSyncBytes() SDIO Command error"
			     " status = 0x%x\n", iStatus);
			return -1;
		}

		uHwAddr++;
		pData++;
	}

	return 0;
}/* end of sdioDrv_WriteSyncBytes() */

/*------------------------------------------------------------------*/
void sdioDrv_PrintRegisters(void)
{
	printk(" OMAP MMC controller registers dump:\n");
	printk(" OMAP MMC Register Dump:\n");
	printk("SCONF =  0x%08x \n", OMAP_HSMMC_READ(SYSCONFIG));
	printk("SSTAT =  0x%08x \n", OMAP_HSMMC_READ(SYSSTATUS));
	printk(" CSRE =  0x%08x \n", OMAP_HSMMC_READ(CSRE));
	printk("STEST =  0x%08x \n", OMAP_HSMMC_READ(SYSTEST));
	printk(" CON  =  0x%08x \n", OMAP_HSMMC_READ(CON));
	printk(" BLK  =  0x%08x \n", OMAP_HSMMC_READ(BLK));
	printk(" ARG  =  0x%08x \n", OMAP_HSMMC_READ(ARG));
	printk(" CMD  =  0x%08x \n", OMAP_HSMMC_READ(CMD));
	printk("RSP10 =  0x%08x \n", OMAP_HSMMC_READ(RSP10));
	printk("RSP32 =  0x%08x \n", OMAP_HSMMC_READ(RSP32));
	printk("RSP54 =  0x%08x \n", OMAP_HSMMC_READ(RSP54));
	printk("RSP76 =  0x%08x \n", OMAP_HSMMC_READ(RSP76));
	printk(" DATA =  0x%08x \n", OMAP_HSMMC_READ(DATA));
	printk("PSTATE=  0x%08x \n", OMAP_HSMMC_READ(PSTATE));
	printk(" HCTL =  0x%08x \n", OMAP_HSMMC_READ(HCTL));
	printk("SYSCTL=  0x%08x \n", OMAP_HSMMC_READ(SYSCTL));
	printk(" STAT =  0x%08x \n", OMAP_HSMMC_READ(STAT));
	printk(" IE   =  0x%08x \n", OMAP_HSMMC_READ(IE));
	printk(" ISE  =  0x%08x \n", OMAP_HSMMC_READ(ISE));
	printk(" AC12 =  0x%08x \n", OMAP_HSMMC_READ(AC12));
	printk(" CAPA =  0x%08x \n", OMAP_HSMMC_READ(CAPA));
	printk("CCAPA =  0x%08x \n", OMAP_HSMMC_READ(CUR_CAPA));
	printk(" REV  =  0x%08x \n", OMAP_HSMMC_READ(REV));

	printk("\n  SDIO control, muxing registers:");

	printk("\n  CONTROL_PADCONF_MMC2_CLK = ");
	printk("  0x%08x", CONTROL_PADCONF_MMC2_CLK);
	printk("\n  CONTROL_PADCONF_MMC2_CMD = ");
	printk("  0x%08x", CONTROL_PADCONF_MMC2_DAT0);
	printk("\n  CONTROL_PADCONF_MMC2_DAT2 = ");
	printk("  0x%08x", CONTROL_PADCONF_MMC2_DAT2);
	printk("\n  CONTROL_DEVCONF1 = ");
	printk("  0x%08x", OMAP2_CONTROL_DEVCONF1);

	printk("\n  GPIO1 bank registers:");
	printk("\n PRCM_GPIO1_SYSCONFIG  = 0x%08x", omap_readl(0x48310010));
	printk("\n GPIO1_IRQSTATUS1  = 0x%08x", omap_readl(0x48310018));
	printk("\n GPIO1_IRQSTATUS2  = 0x%08x", omap_readl(0x48310028));
	printk("\n GPIO1_IRQENABLE1  = 0x%08x", omap_readl(0x4831001c));
	printk("\n GPIO1_WAKEUPENABLE  = 0x%08x", omap_readl(0x48310020));
	printk("\n GPIO1_SETIRQENABLE1 = 0x%08x", omap_readl(0x48310064));
	printk("\n GPIO1_SETWKUENA = 0x%08x", omap_readl(0x48310084));
	printk("\n GPIO1_FALLINGDETECT = 0x%08x", omap_readl(0x4831004c));

	printk("\n");

}/* end of sdioDrv_PrintRegisters() */

/****************************************************************************/
/*           Start: Power Management Functions (by SKC)                     */
/****************************************************************************/
#ifdef CONFIG_PM
#define WLAN_DRIVER_NAME "tiwlan_pm_driver"
#define SET 1
int event_status;

int sdio_setget_param(int setget, int enable_disable)
{
	int returnStatus = 0;
	if (setget == SET)
		event_status = enable_disable;
	returnStatus = event_status;
	return returnStatus;
}

static int sdio_probe(struct platform_device *pdev)
{
	return 0;
}

static int sdio_remove(struct platform_device *pdev)
{
	sdiodrv_shutdown();
	destroy_workqueue(pWorkQueue);
	return 0;
}

static int sdio_suspend(struct platform_device *pdev, pm_message_t state)
{
	int rc = 0;
	printk("WLAN: Suspend call\n");

	if (sdiodrv_suspend_enable)
		return 0;
	/* Tell WLAN driver to suspend, 
	if a suspension function has been registered */
	if (g_drv.wlanDrvIf_pm_suspend) {
		printk("WLAN_firmware Suspend\n");
		return (g_drv.wlanDrvIf_pm_suspend());
	}
	/* sdio_setget_param(SET, 1); */
	if (sdiodrv_suspend_enable == 0) {
		OMAP_HSMMC_WRITE(ISE, 0);
		OMAP_HSMMC_WRITE(IE, 0);
		sdiodrv_free_resources();
		sdiodrv_suspend_enable = 1;
	}
	printk("Shutting Down I&F Clock Interface\n");
	return rc;
}

static int sdio_resume(struct platform_device *pdev)
{
	int rc;

	printk("WLAN: resume call\n");
	sdiodrv_suspend_enable = 0;
	/* sdio_setget_param(SET, 0); */
	rc = sdioDrv_InitHw();
	if (rc != 0) {
		printk(KERN_ERR "TISDIO: resume error\n");
		return rc;
	}

	if (g_drv.wlanDrvIf_pm_resume) {
		printk("WLAN: firmaware resume \n");
		return (g_drv.wlanDrvIf_pm_resume());
	}
	return 0;
}

void sdioDrv_register_pm(int (*wlanDrvIf_Start) (void),
			 int (*wlanDrvIf_Stop) (void))
{
	g_drv.wlanDrvIf_pm_resume = wlanDrvIf_Start;
	g_drv.wlanDrvIf_pm_suspend = wlanDrvIf_Stop;
}

static struct platform_driver sdiodrv_struct = {
	.probe = sdio_probe,
	.remove = sdio_remove,
	.suspend = sdio_suspend,
	.resume = sdio_resume,
	.driver = {
		   .name = WLAN_DRIVER_NAME,
		   },
};
#endif

/*****************************************************************************/
/*           End: Power Management Functions (by SKC)                        */
/*****************************************************************************/

/*---------------------------------------------------------------------------*/

static int __init sdioDrv_init(void)
{

	PDEBUG("entering %s()\n", __FUNCTION__);
	memset(&g_drv, 0, sizeof(g_drv));

#ifdef CONFIG_PM
	/* 
	 * Register the platform device 
	 */
	return platform_driver_register(&sdiodrv_struct);
#else
	return 0;
#endif
}/* end of sdioDrv_init() */

/*---------------------------------------------------------------------------*/

static void __exit sdioDrv_exit(void)
{
	sdiodrv_shutdown();
}/* end of sdioDrv_exit() */

/*---------------------------------------------------------------------------*/

module_init(sdioDrv_init);
module_exit(sdioDrv_exit);

#ifdef CONFIG_PM
EXPORT_SYMBOL(sdioDrv_register_pm);
#endif
EXPORT_SYMBOL(sdioDrv_ConnectBus);
EXPORT_SYMBOL(sdioDrv_DisconnectBus);
EXPORT_SYMBOL(sdioDrv_ExecuteCmd);
EXPORT_SYMBOL(sdioDrv_ReadSync);
EXPORT_SYMBOL(sdioDrv_WriteSync);
EXPORT_SYMBOL(sdioDrv_ReadAsync);
EXPORT_SYMBOL(sdioDrv_WriteAsync);
EXPORT_SYMBOL(sdioDrv_ReadSyncBytes);
EXPORT_SYMBOL(sdioDrv_WriteSyncBytes);
EXPORT_SYMBOL(sdioDrv_PrintRegisters);
MODULE_LICENSE("GPL");

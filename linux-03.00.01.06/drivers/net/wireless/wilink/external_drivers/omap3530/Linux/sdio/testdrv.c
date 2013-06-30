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
**|     Copyright (c) 1998 - 2008 Texas Instruments Incorporated         |**
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

/* includes */
#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/delay.h>
/* #include <linux/mmc/card.h> */
#include <asm/system.h>
#include <asm/io.h>
#include <asm/irq.h>
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,22)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
#include <plat/dma.h>
#include <plat/hardware.h>
#include <plat/gpio.h>
#include <plat/clock.h>
#include <plat/board.h>
#else
#include <mach/dma.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <mach/clock.h>
#include <mach/board.h>

#endif

#else
#include <asm/arch/dma.h>
#include <asm/arch/hardware.h>
#include <asm/arch/gpio.h>
#include <asm/arch/clock.h>
#include <asm/arch/board.h>
#endif

#include "omap34xx_defs.h"
#include "testdrv.h"
#include "TxnDefs.h"
#include "SdioDrv.h"
int sdioAdapt_ConnectBus(void *fCbFunc, void *hCbArg,
			 unsigned int uBlkSizeShift,
			 unsigned int uSdioThreadPriority,
			 unsigned char **pTxDmaSrcAddr, unsigned int *rxLen,
			 unsigned char **pRxDmaSrcAddr, unsigned int *txLen);

static unsigned int mem_partition_size = TESTDRV_MEM_DOWNLOAD_PART_SIZE;
static unsigned int reg_partition_size = TESTDRV_REG_DOWNLOAD_PART_SIZE;

typedef enum {
	SdioSync = 0,
	SdioAsync = 1,
	SdioSyncMax
} SdioSyncMode_e;

typedef enum {
	SDIO_BLOCK_NONE = 0,
	SDIO_BLOCK_MODE = 1
} ESdioBlockMode;

#define TESTDRV_READ_TEST    0x00000001
#define TESTDRV_WRITE_TEST   0x00000002
#define TESTDRV_COMPARE_TEST 0x00000004

#define DO_ON_ERROR(RETURN_VALUE, STRING) \
	printk(STRING); \
	if (RETURN_VALUE!=0) \
	{ \
		printk(" FAILED !!!\n"); \
		return ret; \
	} \
	else \
	{ \
		printk(" PASSED\n" ); \
	} \

struct _g_sdio {
	void *SDIO_handle;
	struct semaphore sem;
	int status;
} g_sdio;

#define SDIO_TEST_ALIGNMENT		4
#define SDIO_TEST_R_W_BUFS_NUM		10
#define SDIO_TEST_R_W_BUF_LEN		512
#define SDIO_TEST_MAX_R_W_BUF_ARRAY	\
	(SDIO_TEST_R_W_BUF_LEN*SDIO_TEST_R_W_BUFS_NUM)

unsigned long g_last_read = 0;
int g_Quiet = 1;
char *read_buf = NULL;
char *read_buf_array[SDIO_TEST_R_W_BUFS_NUM];
char *write_buf = NULL;
char *write_buf_array[SDIO_TEST_R_W_BUFS_NUM];
char complete_test_read_buf[TESTDRV_MAX_SDIO_BLOCK];
extern int g_sdio_debug_level;

struct parts {
	unsigned long mem_size;
	unsigned long mem_off;
	unsigned long reg_size;
	unsigned long reg_off;
} g_parts;

/*--------------------------------------------------------------------------*/
inline int set_partition(int clientID,
			 unsigned long mem_partition_size,
			 unsigned long mem_partition_offset,
			 unsigned long reg_partition_size,
			 unsigned long reg_partition_offset)
{
	int status;
	char byteData[4];
	g_parts.mem_size = mem_partition_size;
	g_parts.mem_off = mem_partition_offset;
	g_parts.reg_size = reg_partition_size;
	g_parts.reg_off = reg_partition_offset;
	/* printk("entering %s(%lx,%lx,%lx,%lx)\n",__FUNCTION__, 
		v1, v2, v3, v4); */
	/* set mem partition size */
	byteData[0] = g_parts.mem_size & 0xff;
	byteData[1] = (g_parts.mem_size >> 8) & 0xff;
	byteData[2] = (g_parts.mem_size >> 16) & 0xff;
	byteData[3] = (g_parts.mem_size >> 24) & 0xff;
	status =
	    sdioDrv_WriteSyncBytes(TXN_FUNC_ID_WLAN, TESTDRV_SDIO_FUNC1_OFFSET,
				   &byteData[0], 1, 0);
	if (status)
		return status;
	status =
	    sdioDrv_WriteSyncBytes(TXN_FUNC_ID_WLAN,
				   TESTDRV_SDIO_FUNC1_OFFSET + 1, &byteData[1],
				   1, 0);
	if (status)
		return status;
	status =
	    sdioDrv_WriteSyncBytes(TXN_FUNC_ID_WLAN,
				   TESTDRV_SDIO_FUNC1_OFFSET + 2, &byteData[2],
				   1, 0);
	if (status)
		return status;
	status =
	    sdioDrv_WriteSyncBytes(TXN_FUNC_ID_WLAN,
				   TESTDRV_SDIO_FUNC1_OFFSET + 3, &byteData[3],
				   1, 0);
	if (status)
		return status;

	/* set mem partition offset */
	byteData[0] = g_parts.mem_off & 0xff;
	byteData[1] = (g_parts.mem_off >> 8) & 0xff;
	byteData[2] = (g_parts.mem_off >> 16) & 0xff;
	byteData[3] = (g_parts.mem_off >> 24) & 0xff;
	status =
	    sdioDrv_WriteSyncBytes(TXN_FUNC_ID_WLAN,
				   TESTDRV_SDIO_FUNC1_OFFSET + 4, &byteData[0],
				   1, 0);
	if (status)
		return status;
	status =
	    sdioDrv_WriteSyncBytes(TXN_FUNC_ID_WLAN,
				   TESTDRV_SDIO_FUNC1_OFFSET + 5, &byteData[1],
				   1, 0);
	if (status)
		return status;
	status =
	    sdioDrv_WriteSyncBytes(TXN_FUNC_ID_WLAN,
				   TESTDRV_SDIO_FUNC1_OFFSET + 6, &byteData[2],
				   1, 0);
	if (status)
		return status;
	status =
	    sdioDrv_WriteSyncBytes(TXN_FUNC_ID_WLAN,
				   TESTDRV_SDIO_FUNC1_OFFSET + 7, &byteData[3],
				   1, 0);
	if (status)
		return status;

	/* set reg partition size */
	byteData[0] = g_parts.reg_size & 0xff;
	byteData[1] = (g_parts.reg_size >> 8) & 0xff;
	byteData[2] = (g_parts.reg_size >> 16) & 0xff;
	byteData[3] = (g_parts.reg_size >> 24) & 0xff;
	status =
	    sdioDrv_WriteSyncBytes(TXN_FUNC_ID_WLAN,
				   TESTDRV_SDIO_FUNC1_OFFSET + 8, &byteData[0],
				   1, 0);
	if (status)
		return status;
	status =
	    sdioDrv_WriteSyncBytes(TXN_FUNC_ID_WLAN,
				   TESTDRV_SDIO_FUNC1_OFFSET + 9, &byteData[1],
				   1, 0);
	if (status)
		return status;
	status =
	    sdioDrv_WriteSyncBytes(TXN_FUNC_ID_WLAN,
				   TESTDRV_SDIO_FUNC1_OFFSET + 10, &byteData[2],
				   1, 0);
	if (status)
		return status;
	status =
	    sdioDrv_WriteSyncBytes(TXN_FUNC_ID_WLAN,
				   TESTDRV_SDIO_FUNC1_OFFSET + 11, &byteData[3],
				   1, 0);
	if (status)
		return status;

	/* set reg partition offset */
	byteData[0] = g_parts.reg_off & 0xff;
	byteData[1] = (g_parts.reg_off >> 8) & 0xff;
	byteData[2] = (g_parts.reg_off >> 16) & 0xff;
	byteData[3] = (g_parts.reg_off >> 24) & 0xff;
	status =
	    sdioDrv_WriteSyncBytes(TXN_FUNC_ID_WLAN,
				   TESTDRV_SDIO_FUNC1_OFFSET + 12, &byteData[0],
				   1, 0);
	if (status)
		return status;
	status =
	    sdioDrv_WriteSyncBytes(TXN_FUNC_ID_WLAN,
				   TESTDRV_SDIO_FUNC1_OFFSET + 13, &byteData[1],
				   1, 0);
	if (status)
		return status;
	status =
	    sdioDrv_WriteSyncBytes(TXN_FUNC_ID_WLAN,
				   TESTDRV_SDIO_FUNC1_OFFSET + 14, &byteData[2],
				   1, 0);
	if (status)
		return status;
	status =
	    sdioDrv_WriteSyncBytes(TXN_FUNC_ID_WLAN,
				   TESTDRV_SDIO_FUNC1_OFFSET + 15, &byteData[3],
				   1, 0);
	if (status)
		return status;

	/* printk("exiting %s\n",__FUNCTION__); */
	return status;

}				/* set_partition */

static int OMAP3430_TNETW_Power(int power_on)
{
	printk("OMAP3430_TNETW_Power(-%s-)\n", power_on ? "ON" : "OFF");

	if (power_on) {
		gpio_direction_output(TESTDRV_GPIO_PM_3430, 1);
	} else {
		gpio_direction_output(TESTDRV_GPIO_PM_3430, 0);
	}

	return 0;

}				/* OMAP3430_TNETW_Power() */

/*----------------------------------------------------------------------------*/

static int OMAP3430_TNETW_HardReset(void)
{
	int err;

	/* Turn power OFF */
	if ((err = OMAP3430_TNETW_Power(0)) == 0) {
		mdelay(50);
		/* Turn power ON */
		err = OMAP3430_TNETW_Power(1);
		mdelay(50);
	}
	return err;

}				/* OMAP3430_TNETW_HardReset() */

/*----------------------------------------------------------------------------*/

static void pad_config(unsigned long pad_addr, u32 andmask, u32 ormask)
{
	int val;
	u32 *addr;

	addr = (u32 *) ioremap(pad_addr, 4);
	if (!addr) {
		printk(KERN_ERR
		       "OMAP3530_pad_config: ioremap failed with addr %lx\n",
		       pad_addr);
		return;
	}

	val = __raw_readl(addr);
	val &= andmask;
	val |= ormask;
	__raw_writel(val, addr);

	iounmap(addr);
}

static int OMAP3430_Wlan_Hardware_Init(void)
{
	printk("OMAP3430_Wlan_Hardware_Init() enter\n");

	pad_config(CONTROL_PADCONF_UART1_CTS, 0xFFFFFFE4, 0x4);
	pad_config(CONTROL_PADCONF_UART1_RTS, 0xFFFCFFFF, 0x11C0000);
	pad_config(CONTROL_PADCONF_MMC2_CLK, 0xFFF8FFFF, 0x00180000);
	pad_config(CONTROL_PADCONF_MMC2_DAT0, 0xFFF8FFF8, 0x00180018);
	pad_config(CONTROL_PADCONF_MMC2_DAT2, 0xFFF8FFF8, 0x01180118);

	printk("OMAP3430_Wlan_Hardware_Init() exit\n");

	return 0;
}				/* OMAP3430_Wlan_Hardware_Init */

/*---------------------------------------------------------------------------*/
/*===========================================================================*/
/*---------------------------------------------------------------------------*/
/* This function allocates memory for all write buffers, and fill 
	them with non-zero characters */
static int init_write_buf(void)
{
	int i, z;

	if ((write_buf =
	     kmalloc(SDIO_TEST_MAX_R_W_BUF_ARRAY, __GFP_DMA)) == NULL) {
		kfree(write_buf);
		return -ENOMEM;
	}

	for (i = 0; i < SDIO_TEST_R_W_BUFS_NUM; i++) {
		/* fill write buffers with non-sero content */
		write_buf_array[i] = write_buf + i * SDIO_TEST_R_W_BUF_LEN;
		for (z = 0; z < SDIO_TEST_R_W_BUF_LEN; z++)
			write_buf_array[i][z] = z % 250 + 1;
	}

	return 0;
}

/* This function allocates memory for all write buffers,
	and fill them with zeros */
static int init_read_buf(void)
{
	int i;

	if ((read_buf =
	     kmalloc(SDIO_TEST_MAX_R_W_BUF_ARRAY, __GFP_DMA)) == NULL) {
		kfree(read_buf);
		return -ENOMEM;
	}

	for (i = 0; i < SDIO_TEST_R_W_BUFS_NUM; i++) {
		/* zero read buffers */
		read_buf_array[i] = read_buf + i * SDIO_TEST_R_W_BUF_LEN;
		memset(read_buf_array[i], 0, SDIO_TEST_R_W_BUF_LEN);
	}

	return 0;
}

/* This function allocates memory for all write & read buffers, 
fills the read buffers with zeros and the write buffers with 
non-zeros characters */
static int init_buffers(void)
{
	if (init_write_buf() != 0) {
		printk("init_buffers: init_write_buf failed\n");
		return (-1);
	}

	if (init_read_buf() != 0) {
		printk("init_buffers: init_read_buf failed\n");
		return (-1);
	}

	return (0);

}/* init_buffers() */

/* This function zeros all read buffers */
static void zero_read_buf(void)
{
	int i;

	if (read_buf == NULL) {
		printk("zero_read_buf: read_buf not allocated\n");
		return;
	}
	for (i = 0; i < SDIO_TEST_R_W_BUFS_NUM; i++) {
		read_buf_array[i] = read_buf + i * SDIO_TEST_R_W_BUF_LEN;
		memset(read_buf_array[i], 0, SDIO_TEST_R_W_BUF_LEN);
	}

}/* zero_buffers() */

/* This function fills all write buffers with non-zero characters */
static void fill_write_buf(void)
{
	int i, z;

	if (write_buf == NULL) {
		printk("fill_write_buf: write_buf not allocated\n");
		return;
	}
	for (i = 0; i < SDIO_TEST_R_W_BUFS_NUM; i++) {
		write_buf_array[i] = write_buf + i * SDIO_TEST_R_W_BUF_LEN;
		memset(write_buf_array[i], 0, SDIO_TEST_R_W_BUF_LEN);
		for (z = 0; z < SDIO_TEST_R_W_BUF_LEN; z++)
			write_buf_array[i][z] = z % 250 + 1;
	}
}				/* zero_buffers() */

/*----------------------------------------------------------------------------*/

static int sync_write_test(int number_of_transactions, int block_len)
{
	int i;
	int ret = 0;
	for (i = 0; i < number_of_transactions; i++) {
		ret =
		    sdioDrv_WriteSync(TXN_FUNC_ID_WLAN,
				      SDIO_TEST_FIRST_VALID_DMA_ADDR,
				      write_buf_array[i %
						      SDIO_TEST_R_W_BUFS_NUM],
				      block_len, 1, 1);
		if (ret) {
			return -ENODEV;
		}
	}

	return ret;

}/* sync_write_test() */

/*----------------------------------------------------------------------------*/

static int sdio_test_write_async(unsigned long addr, char *buf, int length)
{
	int ret =
	    sdioDrv_WriteAsync(TXN_FUNC_ID_WLAN, addr, buf, length,
			       SDIO_BLOCK_NONE, 1, 1);

	if (down_interruptible(&g_sdio.sem)) {
		printk(KERN_ERR "sdio_test_write_async() - FAILED !\n");
		ret = -ENODEV;
	}
	if (ret == 0) {
		ret = g_sdio.status;
	}

	return ret;

}/* sdio_test_write_async() */

/*----------------------------------------------------------------------------*/

static int async_write_test(int number_of_transactions, int block_len)
{
	int i;
	int ret = 0;
	for (i = 0; i < number_of_transactions; i++) {
		ret = sdio_test_write_async(SDIO_TEST_FIRST_VALID_DMA_ADDR,
				write_buf_array[i % SDIO_TEST_R_W_BUFS_NUM],
				block_len);
		if (ret) {
			return -ENODEV;
		}
	}

	return ret;

}/* async_write_test() */

/*---------------------------------------------------------------------------*/

static int sync_read_test(int number_of_transactions, int block_len)
{
	int i;
	int ret = 0;
	for (i = 0; i < number_of_transactions; i++) {
		ret =
		    sdioDrv_ReadSync(TXN_FUNC_ID_WLAN,
				SDIO_TEST_FIRST_VALID_DMA_ADDR,
				read_buf_array[i % SDIO_TEST_R_W_BUFS_NUM],
				block_len, 1, 1);
		if (ret) {
			return -ENODEV;
		}
	}

	return ret;

}/* sync_read_test() */

/*----------------------------------------------------------------------------*/
/* This test reads aligned/non-aligned block in asynch way.
	originally the function read only aligned blocks in asynch way */
static int sdio_test_read_async(unsigned long addr, char *buf, int length)
{
	int ret =
	    sdioDrv_ReadAsync(TXN_FUNC_ID_WLAN, addr, buf, length,
			      SDIO_BLOCK_NONE, 1, 1);

	if (down_interruptible(&g_sdio.sem)) {
		printk(KERN_ERR "sdio_test_read_async() - FAILED !\n");
		ret = -ENODEV;
	}
	if (ret == 0) {
		ret = g_sdio.status;
	}

	return ret;

}/* sdio_test_read_async() */

/*----------------------------------------------------------------------------*/

static int async_read_test(int number_of_transaction, int block_len)
{
	int i;
	int ret = 0;
	for (i = 0; i < number_of_transaction; i++) {
		ret =
		    sdio_test_read_async(SDIO_TEST_FIRST_VALID_DMA_ADDR,
					 read_buf_array[i %
							SDIO_TEST_R_W_BUFS_NUM],
					 block_len);
		if (ret) {
			return -ENODEV;
		}
	}

	return ret;

}/* async_read_test() */

/*----------------------------------------------------------------------------*/

static int sync_compare_test(int number_of_transactions, int block_len)
{
	int i, j, ret = 0, *my_read_buf, *my_write_buf;

	for (i = 0; i < number_of_transactions; i++) {
		my_read_buf = (int *)read_buf_array[i % SDIO_TEST_R_W_BUFS_NUM];
		my_write_buf =
		    (int *)write_buf_array[i % SDIO_TEST_R_W_BUFS_NUM];
		ret =
		    sdioDrv_WriteSync(TXN_FUNC_ID_WLAN,
				      SDIO_TEST_FIRST_VALID_DMA_ADDR,
				      my_write_buf, block_len, 1, 1);
		if (ret) {
			return -ENODEV;
		}
		ret =
		    sdioDrv_ReadSync(TXN_FUNC_ID_WLAN,
				     SDIO_TEST_FIRST_VALID_DMA_ADDR,
				     my_read_buf, block_len, 1, 1);
		if (ret) {
			return -ENODEV;
		}
		if (memcmp(my_write_buf, my_read_buf, block_len)) {
			printk("\nERROR: Write(0x%x)/Read(0x%x) (%d)"
				" buffers are not identical!\n",
				(int)my_write_buf, (int)my_read_buf, i);
			for (j = 0; j < block_len / 4;
			     j++, my_read_buf++, my_write_buf++) {
				printk("  R[%d] = 0x%x, W[%d] = 0x%x\n", j,
				       *my_read_buf, j, *my_write_buf);
			}
			return -1;
		}
	}

	return ret;

}/* sync_compare_test() */

static int sync_gen_compare_test(int number_of_transactions,
				 unsigned int address, int block_len)
{
	int i, j;
	int ret_write = 0;
	int ret_read = 0;
	int *my_read_buf, *my_write_buf;

	for (i = 0; i < number_of_transactions; i++) {
		my_read_buf = (int *)read_buf_array[i % SDIO_TEST_R_W_BUFS_NUM];
		my_write_buf =
		    (int *)write_buf_array[i % SDIO_TEST_R_W_BUFS_NUM];

		ret_write =
		    sdioDrv_WriteSync(TXN_FUNC_ID_WLAN, address, my_write_buf,
				      block_len, 1, 1);
		if (ret_write) {
			printk("sync_gen_compare_test:"
				" sdioDrv_WriteSync failed\n");
			return -ENODEV;
		}
		ret_read =
		    sdioDrv_ReadSync(TXN_FUNC_ID_WLAN, address, my_read_buf,
				     block_len, 1, 1);
		if (ret_read) {
			printk("sync_gen_compare_test:"
				" sdioDrv_ReadSync failed\n");
			return -ENODEV;
		}

		/* If both read & write passed */
		if ((ret_write == 0) && (ret_read == 0)) {
			if (memcmp(my_write_buf, my_read_buf, block_len)) {
				printk("\nERROR: Write(0x%x)/Read(0x%x) (%d)"
					" buffers are not identical!\n",
					(int)my_write_buf, (int)my_read_buf, i);
				for (j = 0; j < block_len / 4;
				     j++, my_read_buf++, my_write_buf++) {
					printk("Addr = 0x%X,	"
						"R[%d] = 0x%x, W[%d] = 0x%x\n",
						address, j, *my_read_buf, j,
						*my_write_buf);
				}
				return -1;
			}
		}
	}

	return 0;

}/* sync_gen_compare_test() */

/*---------------------------------------------------------------------------*/

static int async_compare_test(int number_of_transactions, int block_len)
{
	int i, j, ret = 0, *my_read_buf, *my_write_buf;

	for (i = 0; i < number_of_transactions; i++) {
		my_read_buf = (int *)read_buf_array[i % SDIO_TEST_R_W_BUFS_NUM];
		my_write_buf =
		    (int *)write_buf_array[i % SDIO_TEST_R_W_BUFS_NUM];
		sdio_test_write_async(SDIO_TEST_FIRST_VALID_DMA_ADDR,
				      (char *)my_write_buf, block_len);
		if (ret) {
			return -ENODEV;
		}
		ret =
		    sdio_test_read_async(SDIO_TEST_FIRST_VALID_DMA_ADDR,
					 (char *)my_read_buf, block_len);
		if (ret) {
			return -ENODEV;
		}
		if (memcmp(my_write_buf, my_read_buf, block_len)) {
			printk("\nERROR: Write(0x%x)/Read(0x%x) (%d)"
				" buffers are not identical!\n",
				(int)my_write_buf, (int)my_read_buf, i);
			for (j = 0; j < block_len / 4;
			     j++, my_read_buf++, my_write_buf++) {
				printk("  R[%d] = 0x%x, W[%d] = 0x%x\n", j,
				       *my_read_buf, j, *my_write_buf);
			}
			return -1;
		}
	}

	return ret;
}/* async_compare_test() */

static int async_gen_compare_test(int number_of_transactions,
				  unsigned int address, int block_len)
{
	int i, j;
	int ret_write = 0;
	int ret_read = 0;
	int *my_read_buf, *my_write_buf;

	for (i = 0; i < number_of_transactions; i++) {
		my_read_buf = (int *)read_buf_array[i % SDIO_TEST_R_W_BUFS_NUM];
		my_write_buf =
		    (int *)write_buf_array[i % SDIO_TEST_R_W_BUFS_NUM];

		ret_write =
		    sdio_test_write_async(address, (char *)my_write_buf,
					  block_len);
		if (ret_write) {
			printk("sync_gen_compare_test:"
				" sdioDrv_WriteSync failed\n");
			return -ENODEV;
		}
		ret_read =
		    sdio_test_read_async(address, (char *)my_read_buf,
					 block_len);
		if (ret_read) {
			printk
			    ("sync_gen_compare_test:"
				" sdioDrv_ReadSync failed\n");
			return -ENODEV;
		}
		/* If both read & write passed */
		if ((ret_write == 0) && (ret_read == 0)) {
			if (memcmp(my_write_buf, my_read_buf, block_len)) {
				printk("\nasync_gen_compare_test, ERROR:"
					" Write(0x%x)/Read(0x%x) (%d) "
					"buffers are not identical!\n",
				     (int)write_buf, (int)read_buf, i);
				for (j = 0; j < block_len / 4;
				     j++, my_read_buf++, my_write_buf++) {
					printk("  R[%d] = 0x%x, W[%d] = 0x%x\n",
					       j, *my_read_buf, j,
					       *my_write_buf);
				}
				return -1;
			}
		}
	}

	return 0;

}/* async_gen_compare_test() */

/*---------------------------------------------------------------------------*/

static int perform_test(char *string, int (*test_func) (int, int),
			int number_of_transactions, int block_len)
{
	unsigned long exectime, basetime, endtime;
	int ret;

	printk("%s", string);
	basetime = jiffies;
	ret = test_func(number_of_transactions, block_len);
	endtime = jiffies;
	exectime =
	    endtime >=
	    basetime ? endtime - basetime : 0xffffffff - basetime + endtime;
	exectime = jiffies_to_msecs(exectime);
	printk(": %d*%d bytes took %lu [msecs] ", number_of_transactions,
	       block_len, exectime);
	if (exectime != 0)
		printk("=> %d [Mbps]\n",
		       (int)((number_of_transactions * 8 * block_len) /
			     (exectime * 1000)));
	else
		printk("\n");

	return ret;

}/* perform_test() */

/*----------------------------------------------------------------------------*/

static int do_test_sync(unsigned long tests)
{
	int ret = 0;

	if (tests & TESTDRV_READ_TEST) {
		ret =
		    perform_test("Starting sync read test", sync_read_test,
				 10000, TESTDRV_TESTING_DATA_LENGTH);
		DO_ON_ERROR(ret, "sync read test");
	}
	if (tests & TESTDRV_WRITE_TEST) {
		ret =
		    perform_test("Starting sync write test", sync_write_test,
				 10000, TESTDRV_TESTING_DATA_LENGTH);
		DO_ON_ERROR(ret, "sync write test");
	}
	if (tests & TESTDRV_COMPARE_TEST) {
		ret =
		    perform_test("Starting sync wr/rd compare test",
				 sync_compare_test, 10000,
				 TESTDRV_TESTING_DATA_LENGTH);
		DO_ON_ERROR(ret, "sync wr/rd compare test");
	}

	return ret;

}				/* do_test_sync() */

/*----------------------------------------------------------------------------*/

static int do_test_async(unsigned long tests)
{
	int ret = 0;

	if (tests & TESTDRV_READ_TEST) {
		ret =
		    perform_test("Starting A-sync read test", async_read_test,
				 10000, TESTDRV_TESTING_DATA_LENGTH);
		DO_ON_ERROR(ret, "A-sync read test");
	}
	if (tests & TESTDRV_WRITE_TEST) {
		ret =
		    perform_test("Starting A-sync write test", async_write_test,
				 10000, TESTDRV_TESTING_DATA_LENGTH);
		DO_ON_ERROR(ret, "A-sync write test");
	}
	if (tests & TESTDRV_COMPARE_TEST) {
		ret =
		    perform_test("Starting A-sync wr/rd compare test",
				 async_compare_test, 10000,
				 TESTDRV_TESTING_DATA_LENGTH);
		DO_ON_ERROR(ret, "A-sync wr/rd compare test");
	}

	return ret;

}				/* do_test_async() */

static int do_test_async_data_mem(unsigned long tests)
{
	int ret = 0;
	int status = 0;

	/* set Partition to Data Memory */
	status =
	    set_partition(0, TESTDRV_MEM_DOWNLOAD_PART_SIZE,
			  TESTDRV_DATA_RAM_PART_START_ADDR,
			  TESTDRV_REG_DOWNLOAD_PART_SIZE,
			  TESTDRV_REG_PART_START_ADDR);

	{
		ret =
		    perform_test("Starting A-sync Data Partition read test",
				 async_read_test, 10000,
				 TESTDRV_TESTING_DATA_LENGTH);
		DO_ON_ERROR(ret, "A-sync read test");
	}
	{
		ret =
		    perform_test("Starting A-sync Data Partition write test",
				 async_write_test, 10000,
				 TESTDRV_TESTING_DATA_LENGTH);
		DO_ON_ERROR(ret, "A-sync write test");
	}
	{
		ret =
		    perform_test
		    ("Starting A-sync Data Partition wr/rd compare test",
		     async_compare_test, 10000, TESTDRV_TESTING_DATA_LENGTH);
		DO_ON_ERROR(ret, "A-sync wr/rd compare test");
	}

	return ret;

}				/* do_test_async_data_mem() */

static int do_test_async_code_mem(unsigned long tests)
{
	int ret = 0;
	int status = 0;

	/* set Partition to Data Memory */
	status =
	    set_partition(0, TESTDRV_MEM_DOWNLOAD_PART_SIZE,
			  TESTDRV_CODE_RAM_PART_START_ADDR,
			  TESTDRV_REG_DOWNLOAD_PART_SIZE,
			  TESTDRV_REG_PART_START_ADDR);

	{
		ret =
		    perform_test("Starting A-sync Code Partition read test",
				 async_read_test, 10000,
				 TESTDRV_TESTING_DATA_LENGTH);
		DO_ON_ERROR(ret, "A-sync read test");
	}
	{
		ret =
		    perform_test("Starting A-sync Code Partition write test",
				 async_write_test, 10000,
				 TESTDRV_TESTING_DATA_LENGTH);
		DO_ON_ERROR(ret, "A-sync write test");
	}
	{
		ret =
		    perform_test
		    ("Starting A-sync Code Partition wr/rd compare test",
		     async_compare_test, 10000, TESTDRV_TESTING_DATA_LENGTH);
		DO_ON_ERROR(ret, "A-sync wr/rd compare test");
	}

	return ret;

}				/* do_test_async_code_mem() */

static int do_test_async_packet_mem(unsigned long tests)
{
	int ret = 0;
	int status = 0;

	/* set Partition to Data Memory */
	status =
	    set_partition(0, TESTDRV_MEM_WORKING_PART_SIZE,
			  TESTDRV_PACKET_RAM_PART_START_ADDR,
			  TESTDRV_REG_WORKING_PART_SIZE,
			  TESTDRV_REG_PART_START_ADDR);

	{
		ret =
		    perform_test("Starting A-sync Packet Partition read test",
				 async_read_test, 10000,
				 TESTDRV_TESTING_DATA_LENGTH);
		DO_ON_ERROR(ret, "A-sync read test");
	}
	{
		ret =
		    perform_test("Starting A-sync Packet Partition write test",
				 async_write_test, 10000,
				 TESTDRV_TESTING_DATA_LENGTH);
		DO_ON_ERROR(ret, "A-sync write test");
	}
	{
		ret =
		    perform_test
		    ("Starting A-sync Packet Partition wr/rd compare test",
		     async_compare_test, 10000, TESTDRV_TESTING_DATA_LENGTH);
		DO_ON_ERROR(ret, "A-sync wr/rd compare test");
	}

	return ret;

}				/* do_test_async_packet_mem() */

static void sdio_test_is_alive(void)
{
	int ChipID = 0;

	sdioDrv_ReadSync(TXN_FUNC_ID_WLAN, 0x16800 + 0x5674, &(ChipID), 4, 1,
			 1);
	printk("Read device ID via ReadSync: 0x%x\n", ChipID);

}				/* sdio_test_is_alive() */

/*----------------------------------------------------------------------------*/

static unsigned long sdio_test_get_base(char target)
{

	unsigned long base;

	switch (target) {
	case 'r':		/* WLAN Registers */
	case 'R':
		base = mem_partition_size;
		break;

	case 'm':		/* WLAN Memory */
	case 'M':
		base = 0;
		break;

	default:
		base = 0;
		break;
	}

	return base;

}/* sdio_test_get_base() */

/*---------------------------------------------------------------------------*/

static void sdio_test_read(const char *buffer, unsigned long base,
			   SdioSyncMode_e SyncMode, ESdioBlockMode eBlockMode)
{
	unsigned long addr, length, quiet, j;
	u8 *buf;

	if (base == mem_partition_size) {	/* register */
		sscanf(buffer, "%X", (unsigned int *)&addr);
		length = sizeof(unsigned long);
		quiet = g_Quiet;
	} else {
		sscanf(buffer, "%X %d", (unsigned int *)&addr,
		       (unsigned int *)&length);
		printk("Read %s from 0x%x length %d\n",
		       (SyncMode == SdioSync) ? "sync" : "A-sync",
		       (unsigned int)(addr), (unsigned int)length);
		quiet = g_Quiet;
	}

	addr = addr + base;

	if ((buf = kmalloc(length * 2, __GFP_DMA)) == NULL) {
		printk(" sdio_test_read() kmalloc(%d) FAILED !!!\n",
		       (unsigned int)length);
		return;
	}

	if (SyncMode == SdioSync) {
		quiet =
		    sdioDrv_ReadSync(TXN_FUNC_ID_WLAN, addr, buf, length, 1, 1);
	} else {
		quiet =
		    sdioDrv_ReadAsync(TXN_FUNC_ID_WLAN, addr, buf, length,
				      eBlockMode, 1, 1);
	}

	if (!quiet) {
		printk("The value at 0x%x is ", (unsigned int)(addr - base));
		for (j = 0; j < length / 4; j++) {
			printk("0x%08x ",
			       (unsigned int)*(unsigned int *)(buf +
							       j *
							       sizeof(unsigned
								      int)));
		}
		printk("\n");
	}
	g_last_read = (unsigned int)*(unsigned int *)(buf);
	kfree(buf);

}				/* sdio_test_read() */

static void sdio_test_read_byte(const char *buffer, unsigned long base)
{
	unsigned long addr, length, quiet, j;
	u8 *buf;

	if (base == mem_partition_size) {	/* register */
		sscanf(buffer, "%X", (unsigned int *)&addr);
		length = sizeof(unsigned char);
		quiet = g_Quiet;
	} else {
		sscanf(buffer, "%X %d", (unsigned int *)&addr,
		       (unsigned int *)&length);
		printk("Read from 0x%x length %d\n", (unsigned int)(addr),
		       (unsigned int)length);
		quiet = g_Quiet;
	}

	addr = addr + base;

	if ((buf = kmalloc(length * 2, __GFP_DMA)) == NULL) {
		printk(" sdio_test_read() kmalloc(%d) FAILED !!!\n",
		       (unsigned int)length);
		return;
	}

	for (j = 0; j < length; j++) {
		quiet =
		    sdioDrv_ReadSyncBytes(TXN_FUNC_ID_WLAN, addr + j, (buf + j),
					  1, 0);
	}

	if (!quiet) {
		printk("The value at 0x%x is ", (unsigned int)(addr - base));
		for (j = 0; j < length; j++) {
			printk("0x%02x ", (unsigned int)*(buf + j));
		}
		printk("\n");
	}
	g_last_read = (unsigned int)*(unsigned int *)(buf);
	kfree(buf);

}				/* sdio_test_read_byte() */

/*---------------------------------------------------------------------------*/
#define BUFLEN 100

static void sdio_test_write(const char *buffer, unsigned int base,
			    SdioSyncMode_e SyncMode, ESdioBlockMode eBlockMode)
{
	unsigned long addr, buflen = 4, j;
	u8 *value;
	unsigned long len;

	if ((value = kmalloc(BUFLEN * 2, __GFP_DMA)) == NULL) {
		printk("sdio_test_write:"
			" sdio_test_write() kmalloc(%d) FAILED !!!\n",
			(unsigned int)BUFLEN);
		return;
	}
	sscanf(buffer, "%X %X %d", (unsigned int *)&addr, (unsigned int *)value,
	       (unsigned int *)&len);

	if (!g_Quiet) {
		printk("sdio_test_write: Writing %X to %X  len %d\n",
		       (unsigned int)*(unsigned int *)value, (unsigned int)addr,
		       (unsigned int)len);
	}
	addr = addr + base;

	if (base != mem_partition_size) {	/* memory */
		buflen = len;

		for (j = 0; j < buflen * 2 - 1; j++) {
			value[j + 1] = value[0] + j + 1;
		}
	}
	if (SyncMode == SdioSync) {
		sdioDrv_WriteSync(TXN_FUNC_ID_WLAN, addr, (u8 *) value, buflen,
				  1, 1);
	} else {
		sdioDrv_WriteAsync(TXN_FUNC_ID_WLAN, addr, (u8 *) value, buflen,
				   eBlockMode, 1, 1);
	}
	kfree(value);

}/* sdio_test_write() */

/*---------------------------------------------------------------------------*/
static void sdio_test_write_byte(const char *buffer, unsigned int base)
{
	unsigned long addr, buflen = 1, j;
	u8 *value;
	unsigned long len;

	if ((value = kmalloc(BUFLEN * 2, __GFP_DMA)) == NULL) {
		printk("sdio_test_write_byte:"
			" sdio_test_write() kmalloc(%d) FAILED !!!\n",
			(unsigned int)BUFLEN);
		return;
	}

	sscanf(buffer, "%X %X %d", (unsigned int *)&addr, (unsigned int *)value,
	       (unsigned int *)&len);

	if (!g_Quiet) {
		printk("sdio_test_write_byte: Writing %X to %X  len %d\n",
		       (unsigned int)*(unsigned int *)value, (unsigned int)addr,
		       *(unsigned int *)len);
	}
	addr = addr + base;

	if (base != mem_partition_size) {	/* memory */
		buflen = len;

		for (j = 0; j < buflen - 1; j++) {
			value[j + 1] = value[0] + j + 1;
		}
	}

	for (j = 0; j < buflen; j++) {
		sdioDrv_WriteSyncBytes(TXN_FUNC_ID_WLAN, addr + j, value + j, 1,
				       0);
	}
	kfree(value);

}/* sdio_test_write_byte() */

/*---------------------------------------------------------------------------*/

static void sdio_test_reset(void)
{
	int err;

	disable_irq(TESTDRV_TIWLAN_IRQ);
	printk("OMAP3430_TNETW_HardReset start\n");
	err = OMAP3430_TNETW_HardReset();
	printk("OMAP3430_TNETW_HardReset - %s\n", err ? "FAIL" : "SUCCESS");
	enable_irq(TESTDRV_TIWLAN_IRQ);

	return;

}/* sdio_test_reset() */

/*----------------------------------------------------------------------------*/

void is_alive_test(void)
{
	unsigned long base;
	char *cmd = "rr 5674";

	base = sdio_test_get_base(cmd[1]);
	sdio_test_read(&cmd[2], base, SdioSync, SDIO_BLOCK_NONE);

}/* is_alive_test() */

/*----------------------------------------------------------------------------*/

void sdio_test_CB(void *Handle, int status)
{
	//    DEBUG(LEVEL1,"enter\n");
	g_sdio.status = status;
	up(&g_sdio.sem);

}/* sdio_test_CB() */

/*----------------------------------------------------------------------------*/

static void sdio_test_interrupt(void)
{
	unsigned long base;
	char *cmd = "wr 4f4 1";

/* printk("Current GPIO val : %d\n", gpio_get_value(TESTDRV_GPIO_IRQ_3430)); */

	base = sdio_test_get_base(cmd[1]);
	sdio_test_write(&cmd[2], base, SdioSync, SDIO_BLOCK_NONE);
/*
do{
schedule_timeout_uninterruptible(1000);
printk("Waiting\n");
} while(gpio_get_value(TESTDRV_GPIO_IRQ_3430)); 
*/
}/* sdio_test_interrupt() */

/*---------------------------------------------------------------------------*/

static void sdio_test_init(void)
{
	int ChipID = 0, status;
#ifndef TNETW1253
	unsigned long base;
	char *cmd;
#endif
	unsigned char *pTxDmaSrcAddr = NULL;

	unsigned char *pRxDmaSrcAddr = NULL;
	unsigned int rxLen, txLen;

	sdioAdapt_ConnectBus(sdio_test_CB, NULL, 9, 90, &pRxDmaSrcAddr, &rxLen,
			     &pTxDmaSrcAddr, &txLen);

	/* Set Registers Partition */
	printk("In sdio_test_init: going to perform set_partition."
		" mem size = 0x%X, mem addr =  0x%X, reg size = 0x%X,"
		" reg addr = 0x%X\n", mem_partition_size,
		TESTDRV_CODE_RAM_PART_START_ADDR, reg_partition_size,
		TESTDRV_REG_PART_START_ADDR);

	printk("Running set_partition status = ");
	status =
	    set_partition(0, mem_partition_size,
			  TESTDRV_CODE_RAM_PART_START_ADDR, reg_partition_size,
			  TESTDRV_REG_PART_START_ADDR);
	printk("%d %s\n", status, status ? "NOT OK" : "OK");

	sdioDrv_ReadSyncBytes(TXN_FUNC_ID_WLAN, 0x16800 + 0x5674,
			      (unsigned char *)&(ChipID), 1, 0);
	sdioDrv_ReadSyncBytes(TXN_FUNC_ID_WLAN, 0x16800 + 0x5675,
			      (unsigned char *)&(ChipID) + 1, 1, 0);
	sdioDrv_ReadSyncBytes(TXN_FUNC_ID_WLAN, 0x16800 + 0x5676,
			      (unsigned char *)&(ChipID) + 2, 1, 0);
	sdioDrv_ReadSyncBytes(TXN_FUNC_ID_WLAN, 0x16800 + 0x5677,
			      (unsigned char *)&(ChipID) + 3, 1, 0);

	printk("Read device ID via ReadByte: 0x%x\n", ChipID);

#ifndef TNETW1253
	/* for 1273 only init sequence in order to get access to the memory */
	{
		unsigned long base;
		char *cmd = "wr 6040 3";

		base = sdio_test_get_base(cmd[1]);
		sdio_test_write(&cmd[2], base, SdioSync, SDIO_BLOCK_NONE);
		cmd = "wr 6100 4";
		sdio_test_write(&cmd[2], base, SdioSync, SDIO_BLOCK_NONE);
	}

	/* HI_CFG - host interrupt active high */
	cmd = "wr 808 b8";
	base = sdio_test_get_base(cmd[1]);
	sdio_test_write(&cmd[2], base, SdioSync, SDIO_BLOCK_NONE);
	/* HINT_MASK - unmask all */
	cmd = "wr 4dc FFFFFFFE";
	sdio_test_write(&cmd[2], base, SdioSync, SDIO_BLOCK_NONE);
#endif
}				/* sdio_test_init() */

/*---------------------------------------------------------------------------*/

static void usage(void)
{
	printk("\nSDIO Test proc usage: \n");
	printk("----------------------- \n\n");
	printk("    rr\\a   addr              - read register \\async\n");
	printk("    rm\\a\\b addr length       - read memory \\async"
		" \\block mode,"" length (align to 4) in bytes, block"
		" mode just in async \n");
	printk("    wr\\a   addr value        - write register \\async\n");
	printk("    wm\\a\\b addr value length - write memory and increment"
		" value \\async \\block mode, length (align to 4) in bytes,"
		" block mode just in async\n\n");

	printk("    brr   addr              - read register byte (cmd 52)\n");
	printk("    brm   addr length       - read memory byte (cmd 52), "
		"length in bytes\n");
	printk("    bwr   addr value        - write register byte (cmd 52)\n");
	printk("    bwm   addr value length - write memory byte and increment"
		" value (cmd 52), length in bytes\n\n");

	printk("    sr | sw | sc            - sync  read/write/compare test\n");
	printk("    ar | aw | ac            - async read/write/compare"
		" test\n\n");

	printk("    cs\\i                    - sync SDIO complete test"
		" \\Infinite\n");
	printk("    ca\\i                    - async SDIO complete test"
		" \\Infinite\n");
	printk("    ct\\i                    - sync & async SDIO complete"
		" test \\Infinite  \n\n");

	printk("    er    addr              - error read memory,  running a "
		"second CMD53 immediately after the first one"
		"(for abort transaction test)\n");
	printk("    ew    addr value        - error write memory, running a "
		"second CMD53 immediately after the first one"
		"(for abort transaction test)\n\n");

	printk("    na\\i                    - async SDIO non-sligned complete"
		" test \\Infinite\n");
	printk("    ns\\i                    - sync SDIO non-sligned complete"
		" test \\Infinite \n");
	printk("    nt\\i                    - sync & async SDIO non-sligned "
		"complete test \\Infinite\n\n");

	printk("    h                       - help\n");
	printk("    t                       - TENETW reset\n");
	printk("    i                       - init\n");
	/*  printk("    z     divider           - SDIO 
		CLK=(Main clk / divider( 9bit)).  Main clk = 96M\n"); */
	printk ("    p     memOffset memSize regOffset regSize - set"
		" partitions\n\n");

	printk("    fr    function addr       - read register byte "
		"(cmd 52) from function 0\\1\\2\n");
	printk("    fw    function addr value - write register byte "
		"(cmd 52) to function 0\\1\\2\n\n");

	printk("    md                      - w|r|c  to Data Memory async\n");
	printk("    mc                      - w|r|c  to Code Memory async\n");
	printk("    mp                      - w|r|c  to Packet Memory async\n");

	printk("    l                       - keep alive test\n");

	printk("    q                       - 0: disable quite mode, "
		"1: enable quite mode\n");

}				/* usage */

/*----------------------------------------------------------------------------*/
/* The following fuctions are for SDIO Memory Complete Test                   */
/*----------------------------------------------------------------------------*/

/* This function calculates how many iterations have to be made in order to 
go over	all full blocks of size block_zise (not partiol blocks) in memory of 
size mem_zise */
static int sdio_test_calc_iterations_no(int mem_zise, int block_zise)
{
	int iteration_no = 1;

	if (mem_zise > block_zise) {
		if ((mem_zise % block_zise) == 0) {
			iteration_no = 0;
		}

		iteration_no += mem_zise / block_zise;
	}

	return (iteration_no);

}/* sdio_test_calc_iterations_no */

/* Perform Write - Read - Compare Test of i blocks */
static int sdio_test_compare_blocks(unsigned int mem_test_size,
				    unsigned int block_size,
				    SdioSyncMode_e SyncMode)
{
	int block_remain_size = 0;
	unsigned long addr = 0;
	int blocks_no = 0;
	int ret = 0;
	int j;

	/* check parameters */
	if ((block_size > TESTDRV_MAX_SDIO_BLOCK) || 
		(SyncMode >= SdioSyncMax)) {
		printk("ERROR:	sdio_test_compare_blocks: Invalid Parameter\n");
		return (-1);
	}

	blocks_no = sdio_test_calc_iterations_no(mem_test_size, block_size);
	if (blocks_no <= 0) {
		printk("ERROR:	sdio_test_compare_blocks: blocks_no < 1\n");
		return (-1);
	}

	for (j = 0; j < (blocks_no - 1); j++) {
		addr = j * block_size;

		if (SyncMode == SdioSync) {
			if (addr < SDIO_TEST_FIRST_VALID_DMA_ADDR) {
			if (block_size > SDIO_TEST_FIRST_VALID_DMA_ADDR) {
				ret =
				    sync_gen_compare_test
				    (SDIO_TEST_NO_OF_TRANSACTIONS,
				     SDIO_TEST_FIRST_VALID_DMA_ADDR,
				     block_size -
				     SDIO_TEST_FIRST_VALID_DMA_ADDR);
				}
			} else {
				ret =
				    sync_gen_compare_test
				    (SDIO_TEST_NO_OF_TRANSACTIONS, addr,
				     block_size);
			}
		} else {
			if (addr < SDIO_TEST_FIRST_VALID_DMA_ADDR) {
			if (block_size > SDIO_TEST_FIRST_VALID_DMA_ADDR) {
				ret =
				    async_gen_compare_test
				    (SDIO_TEST_NO_OF_TRANSACTIONS,
				     SDIO_TEST_FIRST_VALID_DMA_ADDR,
				     block_size -
				     SDIO_TEST_FIRST_VALID_DMA_ADDR);
				}
			} else {
				ret =
				    async_gen_compare_test
				    (SDIO_TEST_NO_OF_TRANSACTIONS, addr,
				     block_size);
			}
		}
	}

	/* Test Last Block - if Exists */
	//printk("In sdio_test_compare_blocks: test last block\n");

	block_remain_size = mem_test_size % block_size;
	if (block_remain_size > 0) {
		addr = (blocks_no - 1) * block_size;

		/* write Last block to Mem */
		if (SyncMode == SdioSync) {
			if (addr < SDIO_TEST_FIRST_VALID_DMA_ADDR) {
				if ((block_size -
				     SDIO_TEST_FIRST_VALID_DMA_ADDR) > 0) {
					ret =
					    sync_gen_compare_test
					    (SDIO_TEST_NO_OF_TRANSACTIONS,
					     SDIO_TEST_FIRST_VALID_DMA_ADDR,
					     block_remain_size -
					     SDIO_TEST_FIRST_VALID_DMA_ADDR);
				}
			} else {
				ret =
				    sync_gen_compare_test
				    (SDIO_TEST_NO_OF_TRANSACTIONS, addr,
				     block_remain_size);
			}
		} else {
			if (addr < SDIO_TEST_FIRST_VALID_DMA_ADDR) {
				if ((block_size -
				     SDIO_TEST_FIRST_VALID_DMA_ADDR) > 0) {
					ret =
					    async_gen_compare_test
					    (SDIO_TEST_NO_OF_TRANSACTIONS,
					     SDIO_TEST_FIRST_VALID_DMA_ADDR,
					     block_size -
					     SDIO_TEST_FIRST_VALID_DMA_ADDR);
				}
			} else {
				ret =
				    async_gen_compare_test
				    (SDIO_TEST_NO_OF_TRANSACTIONS, addr,
				     block_remain_size);
			}
		}
	}

	return (0);

}				/* sdio_test_compare_blocks */

/* Perform Read Test of i blocks */
static int sdio_test_read_blocks(unsigned int mem_test_size,
				 unsigned int block_size,
				 SdioSyncMode_e SyncMode)
{
	int block_remain_size = 0;
	int addr = 0;
	int j;
	int blocks_no = 0;
	int ret = 0;

	/* check parameters */
	if ((block_size > TESTDRV_MAX_SDIO_BLOCK) || 
		(SyncMode >= SdioSyncMax)) {
		printk("ERROR:	sdio_test_read_blocks: Invalid Parameter\n");
		return (-1);
	}

	blocks_no = sdio_test_calc_iterations_no(mem_test_size, block_size);
	if (blocks_no <= 0) {
		printk("ERROR:	sdio_test_read_blocks: blocks_no < 1\n");
		return (-1);
	}
	for (j = 0; j < (blocks_no - 1); j++) {
		addr = j * block_size;

		/* Read block from Mem */
		if (SyncMode == SdioSync) {
			ret =
			    sdioDrv_ReadSync(TXN_FUNC_ID_WLAN, addr,
					     (u8 *) complete_test_read_buf,
					     block_size, 1, 1);
		} else {
			ret = sdio_test_read_async(addr,
						 (char *)complete_test_read_buf,
						 block_size);
		}
		if (ret != 0) {
			/* Report Error, but continue the Test */
			printk("ERROR:	sdio_test_read_blocks:"
				" ERROR - Read buffer failed!\n");
		}

	}/* for ( j = 0 ; j < (blocks_no-1) ; j++ ) */

	/* Test Last Block - if Exists */
	block_remain_size = mem_test_size % block_size;
	if (block_remain_size > 0) {
		addr = (blocks_no - 1) * block_size;

		/* Read block from Mem */
		if (SyncMode == SdioSync) {
			ret = sdioDrv_ReadSync(TXN_FUNC_ID_WLAN, addr,
					     (u8 *) complete_test_read_buf,
					     block_remain_size, 1, 1);
		} else {
			ret = sdio_test_read_async(addr,
						 (char *)complete_test_read_buf,
						 block_size);
		}
		if (ret != 0) {
			/* Report Error, but continue the Test */
			printk("ERROR:	sdio_test_read_blocks:"
				" ERROR - ****Last**** Read buffer failed!\n");
		}
	}

	return (0);

}/* sdio_test_read_blocks */

/* Perform Write - Read - Compare Test */
static int sdio_test_complete_mem(unsigned int mem_test_start,
				  unsigned int mem_test_size,
				  unsigned int block_test_size,
				  SdioSyncMode_e SyncMode)
{
	int i, status;
	int mem_remain_size = 0;
	int iteration_no = 0;

	/* check parameters */
	if ((mem_test_size == 0) ||
	    (block_test_size > TESTDRV_MAX_SDIO_BLOCK) ||
	    (SyncMode >= SdioSyncMax)) {
		printk("ERROR:	sdio_test_complete_mem: invalid Parameter\n");
		return (-1);
	}

	/* Calculate Number of Iterations. Each Iteration 
		Is At Most the size of one memory Pratition */
	iteration_no = sdio_test_calc_iterations_no(mem_test_size, 
						mem_partition_size);
	if (iteration_no <= 0) {
		printk("ERROR:	sdio_test_complete_mem: iteration_no < 1\n");
		return (-1);
	}
	/* Go over all Ram Memory */
	for (i = 0; i < (iteration_no - 1); i++) {
		/* Set Memory Partition at the Start of each Iteration */
		status =
		    set_partition(0, mem_partition_size,
				  (mem_test_start + i * mem_partition_size),
				  reg_partition_size,
				  TESTDRV_REG_PART_START_ADDR);
		if (status) {
			printk("ERROR:	sdio_test_complete_mem:"
			" Set Partition to addr 0x%X on Iteration %d failed\n",
			mem_test_start + mem_test_size - mem_remain_size, i);
			return (-1);
		}

		sdio_test_compare_blocks(mem_partition_size, block_test_size,
					 SyncMode);
	}
	/* Go over Last Interation */
	mem_remain_size =
	    mem_test_size - mem_partition_size * (iteration_no - 1);
	if (mem_remain_size > 0) {
		/* Set Memory Partition at the Start of each Iteration */
		status =
		    set_partition(0, mem_remain_size,
				  (mem_test_start + mem_test_size -
				   mem_remain_size), reg_partition_size,
				  TESTDRV_REG_PART_START_ADDR);
		if (status) {
			printk("ERROR:	sdio_test_complete_mem:"
			" Set Partition to addr 0x%X on Iteration %d failed\n",
			     mem_test_start + mem_test_size - mem_remain_size,
			     i);
			return (-1);
		}
		return (sdio_test_compare_blocks
			(mem_remain_size, block_test_size, SyncMode));
	}

	return (0);

}				/* sdio_test_complete_mem */

/* Perform Read from Regs Test */
static int sdio_test_complete_reg(unsigned int mem_test_size,
				  unsigned int block_test_size,
				  SdioSyncMode_e SyncMode)
{
	int i;
	int mem_remain_size = 0;
	int iteration_no = 0;

	/* check parameters */
	if ((mem_test_size == 0) ||
	    (block_test_size > TESTDRV_MAX_SDIO_BLOCK) ||
	    (SyncMode >= SdioSyncMax)) {
		printk("ERROR:	sdio_test_complete_reg: invalid Parameter\n");
		return (-1);
	}

	iteration_no =
	    sdio_test_calc_iterations_no(mem_test_size, reg_partition_size);
	if (iteration_no <= 0) {
		printk("ERROR:	sdio_test_complete_reg: iteration_no < 1\n");
		return (-1);
	}
	/* Go over all Ram Memory */
	for (i = 0; i < (iteration_no - 1); i++) {
		sdio_test_read_blocks(mem_test_size, block_test_size, SyncMode);
	}
	/* Go over Last Interation */
	mem_remain_size = mem_test_size - mem_test_size * (iteration_no - 1);
	if (mem_remain_size > 0) {
		return (sdio_test_read_blocks
			(mem_remain_size, block_test_size, SyncMode));
	}

	return 0;		/* ??? */

}				/* sdio_test_complete_reg */

/*	This is a complete Test of SDIO. 
	The Test checks all the Memory Partition (Read/Write/Compare) 
	and all the Register Partition (Read) */
static void sdio_test_complete(unsigned int block_size, SdioSyncMode_e SyncMode)
{
	/* check parameters */
	if ((SdioAsync >= SdioSyncMax) || 
		(block_size > TESTDRV_MAX_SDIO_BLOCK)) {
		printk("sdio_test_complete: invalid Parameter\n");
		return;
	}

	mem_partition_size = TESTDRV_MEM_DOWNLOAD_PART_SIZE;
	reg_partition_size = TESTDRV_REG_DOWNLOAD_PART_SIZE;

	/*--------------------------*/
	/* Test Memory RAM                      */
	/*--------------------------*/
	/* Test Memory RAM: 
		Go over all Memory Partitions and for each addrs Perform 3 
		Times W & R with compare */

	/*--------------------------*/
	/* Test Code Ram Partition      */
	/*--------------------------*/
	if (sdio_test_complete_mem
	    (TESTDRV_CODE_RAM_PART_START_ADDR, TESTDRV_CODE_RAM_SIZE,
	     block_size, SyncMode) != 0) {
		printk("ERROR:	Test Code Ram Failed\n");
	} else {
		printk("Test Code Ram Succeed\n");
	}

	/*--------------------------*/
	/* Test Data Ram Partition      */
	/*--------------------------*/
	if (sdio_test_complete_mem
	    (TESTDRV_DATA_RAM_PART_START_ADDR, TESTDRV_DATA_RAM_SIZE,
	     block_size, SyncMode) != 0) {
		printk("ERROR:	Test Data Ram Failed\n");
	} else {
		printk("Test Data Ram Succeed\n");
	}

	/*--------------------------*/
	/* Test Packet Ram Partition */
	/*--------------------------*/
	mem_partition_size = TESTDRV_MEM_WORKING_PART_SIZE;
	reg_partition_size = TESTDRV_REG_WORKING_PART_SIZE;
	if (sdio_test_complete_mem(TESTDRV_PACKET_RAM_PART_START_ADDR, 
				TESTDRV_PACKET_RAM_SIZE, block_size, 
				SyncMode) != 0) {
		printk("ERROR:	Test Packet Ram Failed\n");
	} else {
		printk("Test Packet Ram Succeed\n");
	}

	/*--------------------------*/
	/* Test Registers Partition */
	/*--------------------------*/
	if (sdio_test_complete_reg(reg_partition_size, block_size, SyncMode) !=
	    0) {
		printk("ERROR:	Test Registers Ram Failed\n");
	} else {
		printk("Test Registers Ram Succeed\n");
	}

}				/* sdio_test_complete() */

/* This Test perform sdio_test_complete on differnt block
sizes - in async and once only */
static void sdio_test_once_async_complete(void)
{
	printk("Perform Async Complete SDIO\n");
	printk("************	SDIO Complete Async Test,"
		" 4 byte block	************\n");
	sdio_test_complete(4, SdioAsync);
	printk("************	SDIO Complete Async Test,"
		" 64 byte block	************\n");
	sdio_test_complete(64, SdioAsync);
	printk("************	SDIO Complete Async Test,"
		" 128 byte block	************\n");
	sdio_test_complete(128, SdioAsync);
	printk("************	SDIO Complete Async Test,"
		" 256 byte block	************\n");
	sdio_test_complete(256, SdioAsync);
	printk("************	SDIO Complete Async Test,"
		" 384 byte block	************\n");
	sdio_test_complete(384, SdioAsync);
	printk("************	SDIO Complete Async Test,"
		" 512 byte block	************\n");
	/* 0 equivalent to 512 bytes block */
	sdio_test_complete(TESTDRV_512_SDIO_BLOCK, SdioAsync);
}

/* This Test perform sdio_test_complete on differnt block
sizes - in async and for ever */
static void sdio_test_infinit_async_complete(void)
{
	/* Loop Forever */
	while (1) {
		sdio_test_once_async_complete();
	}
}

/* This Test perform sdio_test_complete on differnt block
sizes - in sync and once only */
static void sdio_test_once_sync_complete(void)
{
	printk("Perform Synch Complete SDIO\n");
	printk("************	SDIO Complete Test,"
		" 4 byte block	************\n");
	sdio_test_complete(4, SdioSync);
	printk("************	SDIO Complete Sync Test,"
		" 64 byte block	************\n");
	sdio_test_complete(64, SdioSync);
	printk("************	SDIO Complete Sync Test,"
		" 128 byte block	************\n");
	sdio_test_complete(128, SdioSync);
	printk("************	SDIO Complete Sync Test,"
		" 256 byte block	************\n");
	sdio_test_complete(256, SdioSync);
	printk("************	SDIO Complete Sync Test,"
		" 384 byte block	************\n");
	sdio_test_complete(384, SdioSync);
	printk("************	SDIO Complete Sync Test,"
		" 512 byte block	************\n");
	/* 0 equivalent to 512 bytes block */
	sdio_test_complete(TESTDRV_512_SDIO_BLOCK, SdioSync);
}

/* This Test perform sdio_test_complete on differnt block sizes
- in sync and for ever */
static void sdio_test_infinit_sync_complete(void)
{
	/* Loop Forever */
	while (1) {
		sdio_test_once_sync_complete();
	}
}

/* This Test perform sdio_test_complete on differnt block sizes
- in sync & async and once only */
static void sdio_test_once_complete(void)
{
	sdio_test_once_async_complete();

	sdio_test_once_sync_complete();
}

/* This Test perform sdio_test_complete on differnt block sizes
- in sync & async and forever */
static void sdio_test_infinit_complete(void)
{
	while (1) {
		sdio_test_once_complete();
	}
}

/*----------------------------------------------------------------------------*/
/* The following functions are for test which checks reading from not-aligned */
/* addresses                                                                  */
/*----------------------------------------------------------------------------*/

/* Perform aligned Write of i blocks */
static int sdio_test_write_non_aligned_blocks(unsigned int block_size,
					      SdioSyncMode_e SyncMode)
{
	unsigned long addr = 0;
	int blocks_no = 0;
	int ret = 0;
	int j, i;

	/* check parameters */
	if ((block_size > TESTDRV_MAX_SDIO_BLOCK) ||
	    (block_size % SDIO_TEST_ALIGNMENT != 0) ||
	    (SyncMode >= SdioSyncMax) ||
	    (SDIO_TEST_MAX_R_W_BUF_ARRAY + 2 * block_size > mem_partition_size))
	{
		printk("ERROR:	sdio_test_write_non_aligned_blocks:"
			" Invalid Parameter\n");
		return (-1);
	}
	if (SDIO_TEST_R_W_BUF_LEN % block_size != 0) {
		printk("ERROR:	sdio_test_write_non_aligned_blocks:"
			" block_size not aligned\n");
		return (-1);
	}

	fill_write_buf();
	blocks_no = SDIO_TEST_R_W_BUF_LEN / block_size;

	for (j = 0; j < SDIO_TEST_R_W_BUFS_NUM; j++) {
		for (i = 0; i < blocks_no; i++) {
			char *temp_write_buf =
			    write_buf_array[j] + i * block_size;
			/* address in memory: will not begine in 0x0 */
			addr = j * SDIO_TEST_R_W_BUF_LEN + (i + 1) * block_size;

			/* write Synch */
			if (SyncMode == SdioSync) {
				/* write non-sligned block */
				ret = sdioDrv_WriteSync(TXN_FUNC_ID_WLAN,
							addr,
							temp_write_buf,
							block_size, 1, 1);
				if (ret) {
					printk("sdio_test_write_non_aligned"
					"_blocks: sdioDrv_WriteSync failed\n");
					return -ENODEV;
				}
			}
			/* write A-Synch */
			else {
				ret = sdio_test_write_async(addr,
							    temp_write_buf,
							    block_size);
				if (ret) {
					printk("sdio_test_write_non_aligned"
					"_blocks: sdio_test_write_async"
					" failed\n");
					return -ENODEV;
				}
			}
		}
	}

	return (0);

}/* sdio_test_write_non_aligned_blocks */

/* Perform non-aligned Read (and Compere to Wrote values) of i blocks */
static int sdio_test_read_compare_non_aligned_blocks(unsigned int block_size,
						     SdioSyncMode_e SyncMode,
						     int alignDec)
{
	unsigned long addr = 0;
	int blocks_no = 0;
	int ret = 0;
	int j, i, k;

	/* check parameters */
	if ((block_size > TESTDRV_MAX_SDIO_BLOCK) ||
	    (block_size % SDIO_TEST_ALIGNMENT != 0) ||
	    (SyncMode >= SdioSyncMax) ||
	    (SDIO_TEST_MAX_R_W_BUF_ARRAY + 2 * block_size > mem_partition_size))
	{
		printk("ERROR:	sdio_test_read_compare_non_aligned_blocks:"
			" Invalid Parameter\n");
		return (-1);
	}
	if (SDIO_TEST_R_W_BUF_LEN % block_size != 0) {
		printk("ERROR:	sdio_test_read_compare_non_aligned_blocks:"
			" block_size not aligned\n");
		return (-1);
	}

	zero_read_buf();
	blocks_no = SDIO_TEST_R_W_BUF_LEN / block_size;

	for (j = 0; j < SDIO_TEST_R_W_BUFS_NUM; j++) {
		for (i = 0; i < blocks_no; i++) {
			char *temp_read_buf =
			    read_buf_array[j] + i * block_size;
			/* address in memory: will not begine in 0x0 and will be not-aligned */
			addr =
			    j * SDIO_TEST_R_W_BUF_LEN + (i + 1) * block_size +
			    alignDec;

			/* read Synch */
			if (SyncMode == SdioSync) {
				/* write non-sligned block */
				ret = sdioDrv_ReadSync(TXN_FUNC_ID_WLAN,
						       addr,
						       temp_read_buf,
						       block_size - alignDec, 1,
						       1);
				if (ret) {
					printk("sdio_test_read_non_aligned"
					"_blocks: sdioDrv_ReadSync failed\n");
					return -ENODEV;
				}
			}
			/* read A-Synch */
			else {
				ret = sdio_test_read_async(addr,
							   temp_read_buf,
							   block_size -
							   alignDec);
				if (ret) {
					printk("sdio_test_read_compare_non"
					"_aligned_blocks: "
					"sdio_test_read_async failed\n");
					return -ENODEV;
				}
			}
		}
	}

	/* check read buffers */
	for (j = 0; j < SDIO_TEST_R_W_BUFS_NUM; j++) {
		for (i = 0; i < blocks_no; i++) {
			char *temp_read_buf_ptr =
			    read_buf_array[j] + i * block_size;
			char *temp_write_buf_ptr =
			    write_buf_array[j] + i * block_size + alignDec;

			for (k = 0; k < (block_size - alignDec); k++) {
				if (temp_read_buf_ptr[k] !=
				    temp_write_buf_ptr[k]) {
					printk("sdio_test_read_compare_non_"
					"aligned_blocks: read & write mismatch!\n");
					printk("index = %d	;	read:"
					" 0x%X	;	write: 0x%X\n", k,
					temp_read_buf_ptr[k],
					temp_write_buf_ptr[k]);
					return (-ENODEV);
				}
			}
			for (k = block_size - alignDec; k < block_size; k++) {
				if (temp_read_buf_ptr[k] != 0) {
					printk("read buf supposed to be 0,"
						" but its not!\n");
					printk("read buf: 0x%X\n",
					       temp_read_buf_ptr[k]);
					return (-ENODEV);
				}
			}
		}
	}

	return (0);

}				/* sdio_test_read_compare_non_aligned_blocks */

/* This function performs test of reading from not-aligned addresses once 
	(sync/async mode is passed to the function) */
static int sdio_test_once_not_aligned(unsigned int block_size,
				      SdioSyncMode_e SyncMode, int alignDec)
{
	int ret = 0;

	ret = sdio_test_write_non_aligned_blocks(block_size, SyncMode);
	if (ret != 0) {
		printk("sdio_test_once_not_aligned:"
			" sdio_test_write_non_aligned_blocks failed\n");
		return (ret);
	}

	return (sdio_test_read_compare_non_aligned_blocks
		(block_size, SyncMode, alignDec));
}

/*	This is a complete Test of reading from Not-Aligned addresses. 
	This function performs test of reading from not-aligned addresses 
	- from all different Memory Sections
	(DATA/CODE/PACKET),(sync/async mode is passed to the function) */
static void sdio_test_non_aligned_complete(unsigned int block_size,
					   SdioSyncMode_e SyncMode,
					   int alignDec)
{
	int status = 0;

	/* check parameters */
	if ((SdioAsync >= SdioSyncMax) ||
	    (block_size > TESTDRV_MAX_SDIO_BLOCK) ||
	    (alignDec < 1) || (alignDec >= SDIO_TEST_ALIGNMENT)) {
		printk("sdio_test_non_aligned_complete: invalid Parameter\n");
		return;
	}

	mem_partition_size = TESTDRV_MEM_DOWNLOAD_PART_SIZE;
	reg_partition_size = TESTDRV_REG_DOWNLOAD_PART_SIZE;

	/*--------------------------*/
	/* Test Memory RAM                      */
	/*--------------------------*/
	/* Test Memory RAM: 
	   Go over all Memory Partitions and for each addrs Perform 3 Times W & R with compare */

	/*--------------------------*/
	/* Test Code Ram Partition      */
	/*--------------------------*/
	/* Set Memory Partition */
	status = set_partition(0,
			       mem_partition_size,
			       TESTDRV_CODE_RAM_PART_START_ADDR,
			       reg_partition_size, TESTDRV_REG_PART_START_ADDR);
	if (status) {
		printk("ERROR:	sdio_test_non_aligned_complete:"
			" set_partition failed\n");
		return;
	}
	if (sdio_test_once_not_aligned(block_size, SyncMode, alignDec) != 0) {
		printk("ERROR:	Test Code Ram not aligned Failed\n");
	} else {
		printk("Test Code Ram not aligned Succeed\n");
	}

	/*--------------------------*/
	/* Test Data Ram Partition      */
	/*--------------------------*/
	/* Set Memory Partition */
	status = set_partition(0,
			       mem_partition_size,
			       TESTDRV_DATA_RAM_PART_START_ADDR,
			       reg_partition_size, TESTDRV_REG_PART_START_ADDR);
	if (sdio_test_once_not_aligned(block_size, SyncMode, alignDec) != 0) {
		printk("ERROR:	Test Data Ram not aligned Failed\n");
	} else {
		printk("Test Data Ram not aligned Succeed\n");
	}

	/*--------------------------*/
	/* Test Packet Ram Partition */
	/*--------------------------*/
	mem_partition_size = TESTDRV_MEM_WORKING_PART_SIZE;
	reg_partition_size = TESTDRV_REG_WORKING_PART_SIZE;
	/* Set Memory Partition */
	status = set_partition(0,
			       mem_partition_size,
			       TESTDRV_PACKET_RAM_SIZE,
			       reg_partition_size, TESTDRV_REG_PART_START_ADDR);
	if (status) {
		printk("ERROR:	sdio_test_non_aligned_complete:"
			" set_partition failed\n");
		return;
	}
	if (sdio_test_once_not_aligned(block_size, SyncMode, alignDec) != 0) {
		printk("ERROR:	Test Packet Ram not aligned Failed\n");
	} else {
		printk("Test Packet Ram not aligned Succeed\n");
	}

}				/* sdio_test_non_aligned_complete() */

/* This function tests reading non-sligned blocks in sync way, only once */
static void sdio_test_once_not_aligned_sync_complete(void)
{
	printk("************	SDIO not_aligned Complete Sync Test,"
		" 16 byte block	************\n");
	sdio_test_non_aligned_complete(16, SdioSync, 2);
	printk("************	SDIO not_aligned Complete Sync Test,"
		" 64 byte block	************\n");
	sdio_test_non_aligned_complete(64, SdioSync, 2);
	printk("************	SDIO not_aligned Complete Sync Test,"
		" 128 byte block	************\n");
	sdio_test_non_aligned_complete(128, SdioSync, 2);
	printk("************	SDIO not_aligned Complete Sync Test,"
		" 256 byte block	************\n");
	sdio_test_non_aligned_complete(256, SdioSync, 2);
	printk("************	SDIO not_aligned Complete Sync Test,"
		" 512 byte block	************\n");
	/* 0 equivalent to 512 bytes block */
	sdio_test_non_aligned_complete(512, SdioSync, 2);
}

/* This function tests reading non-sligned blocks in async way, only once */
static void sdio_test_once_not_aligned_async_complete(void)
{
	printk("************	SDIO not_aligned Complete Async Test,"
		" 16 byte block	************\n");
	sdio_test_non_aligned_complete(16, SdioAsync, 2);
	printk("************	SDIO not_aligned Complete Async Test,"
		" 64 byte block	************\n");
	sdio_test_non_aligned_complete(64, SdioAsync, 2);
	printk("************	SDIO not_aligned Complete Async Test,"
		" 128 byte block	************\n");
	sdio_test_non_aligned_complete(128, SdioAsync, 2);
	printk("************	SDIO not_aligned Complete Async Test,"
		" 256 byte block	************\n");
	sdio_test_non_aligned_complete(256, SdioAsync, 2);
	printk("************	SDIO not_aligned Complete Async Test,"
		" 512 byte block	************\n");
	/* 0 equivalent to 512 bytes block */
	sdio_test_non_aligned_complete(512, SdioAsync, 2);
}

/* This function tests reading non-sligned blocks in sync way, for ever */
static void sdio_test_infinit_not_aligned_sync_complete(void)
{
	while (1) {
		sdio_test_once_not_aligned_sync_complete();
	}
}

/* This function tests reading non-sligned blocks in async way, for ever */
static void sdio_test_infinit_not_aligned_async_complete(void)
{
	while (1) {
		sdio_test_once_not_aligned_async_complete();
	}
}

/* This function tests reading non-sligned blocks in sync & azync way,
only once */
static void sdio_test_once_not_aligned_complete(void)
{
	sdio_test_once_not_aligned_sync_complete();
	sdio_test_once_not_aligned_async_complete();
}

/* This function tests reading non-sligned blocks in sync & azync way,
for ever */
static void sdio_test_infinit_not_aligned_complete(void)
{
	while (1) {
		sdio_test_once_not_aligned_sync_complete();
		sdio_test_once_not_aligned_async_complete();
	}
}

/*---------------------------------------------------------------------------*/

/*	This function receive and parse user requests */
static int sdio_test_write_proc(struct file *file,
				const char *buffer,
				unsigned long count, void *data)
{
	int flag = 0;
	unsigned long base;
	int j;
	SdioSyncMode_e SyncMode;
	ESdioBlockMode eBlockMode;

	if (buffer[2] == 'a' || buffer[2] == 'A') {
		SyncMode = SdioAsync;
		j = 3;
	} else {
		SyncMode = SdioSync;
		j = 2;
	}

	if (buffer[3] == 'b' || buffer[3] == 'B') {
		eBlockMode = SDIO_BLOCK_MODE;
		j = 4;
	} else {
		eBlockMode = SDIO_BLOCK_NONE;
	}

	switch (buffer[0]) {
		/*
		    case 'z':
		    case 'Z':
		        {
		            unsigned int divider;

		            sscanf((buffer + 1), "%d", (unsigned int *)&divider);
		*/
		/* 
		 * SDIO CLK = (Main clk / (divider( 9bit) << 6)) | 5 
		 * Main clk = 96M. 
		 * |5 in order to set Clock enable 
		 */
		/*            divider = (divider << 6) | 5; */

		/* write to 3430 SYSCTL register */
		/*OMAP_HSMMC_WRITE(SYSCTL, divider); 
		}
		break;
		*/
	case 'f':
	case 'F':
		{
			unsigned int Address, data, function;
			unsigned char value;

			if (buffer[1] == 'w' || buffer[1] == 'W') {
				sscanf((buffer + 2), "%d %X %X",
				       (unsigned int *)&function,
				       (unsigned int *)&Address,
				       (unsigned int *)&data);
				sdioDrv_WriteSyncBytes(function, Address,
						       (char *)&data, 1, 0);
			} else {
				sscanf((buffer + 2), "%d %X",
				       (unsigned int *)&function,
				       (unsigned int *)&Address);
				sdioDrv_ReadSyncBytes(function, Address,
						      (char *)&data, 1, 0);
				value = (unsigned char)data;
				printk("\nfunction %d address to write: %x." 
					"data to write %x. value %x\n",
					function, Address, data, value);
				sdioDrv_WriteSyncBytes(function, Address,
						       &value, 1, 0);
				printk("The value at 0x%x is 0x%02x\n", Address,
				       value);
			}

		}
		break;
	case 'p':
	case 'P':
		{
			unsigned int memOffset, memSize, regOffset, regSize;
			int status;

			sscanf((buffer + 1), "%X %X %X %X",
			       (unsigned int *)&memOffset,
			       (unsigned int *)&memSize,
			       (unsigned int *)&regOffset,
			       (unsigned int *)&regSize);
			/* Set Registers Partition */
			printk("perform set partitions. mem addr =  0x%X"
				", mem size = 0x%X, reg addr = 0x%X,"
				"  reg size = 0x%X,\n", memOffset, 
				memSize, regOffset, regSize);
			printk("Running load_vals status = ");
			status =
			    set_partition(0, memSize, memOffset, regSize,
					  regOffset);
			printk("%d %s\n", status, status ? "NOT OK" : "OK");
			mem_partition_size = memSize;
			reg_partition_size = regSize;
		}
		break;

	case 'b':
	case 'B':
		if (buffer[1] == 'r' || buffer[1] == 'R') {
			base = sdio_test_get_base(buffer[2]);
			sdio_test_read_byte(&buffer[3], base);
		}
		if (buffer[1] == 'w' || buffer[1] == 'W') {
			base = sdio_test_get_base(buffer[2]);
			sdio_test_write_byte(&buffer[3], base);
		}
		break;

	case 'm':
	case 'M':
		if (buffer[1] == 'd' || buffer[1] == 'D') {
			printk("In switch (buffer[0]):"
				" calling do_test_async_data_mem\n");
			do_test_async_data_mem(flag);
		}
		if (buffer[1] == 'c' || buffer[1] == 'C') {
			printk("In switch (buffer[0]):"
				" calling do_test_async_code_mem\n");
			do_test_async_code_mem(flag);
		}
		if (buffer[1] == 'p' || buffer[1] == 'P') {
			printk("In switch (buffer[0]):"
				" calling do_test_async_packet_mem\n");
			do_test_async_packet_mem(flag);
		}
		break;

	case 'r':
	case 'R':
		base = sdio_test_get_base(buffer[1]);
		sdio_test_read(&buffer[j], base, SyncMode, eBlockMode);
		break;

	case 'w':
	case 'W':
		base = sdio_test_get_base(buffer[1]);
		sdio_test_write(&buffer[j], base, SyncMode, eBlockMode);
		break;

	case 'q':
	case 'Q':
		if (g_Quiet)
			g_Quiet = 0;
		else
			g_Quiet = 1;
		printk("Changed Quiet mode to %d\n", g_Quiet);
		break;

	case 's':
	case 'S':
		switch (buffer[1]) {
		case 'r':
		case 'R':
			do_test_sync(TESTDRV_READ_TEST);
			break;

		case 'w':
		case 'W':
			do_test_sync(TESTDRV_WRITE_TEST);
			break;

		case 'c':
		case 'C':
			do_test_sync(TESTDRV_COMPARE_TEST);
			break;

		default:
			do_test_sync(TESTDRV_READ_TEST | TESTDRV_WRITE_TEST |
				     TESTDRV_COMPARE_TEST);
			break;
		}		/* switch (buffer[1]) */
		break;

	case 'a':
	case 'A':
		switch (buffer[1]) {
		case 'r':
		case 'R':
			do_test_async(TESTDRV_READ_TEST);
			break;

		case 'w':
		case 'W':
			do_test_async(TESTDRV_WRITE_TEST);
			break;

		case 'c':
		case 'C':
			do_test_async(TESTDRV_COMPARE_TEST);
			break;

		default:
			do_test_async(TESTDRV_READ_TEST | TESTDRV_WRITE_TEST |
				      TESTDRV_COMPARE_TEST);
			break;

		}		/* switch (buffer[1]) */
		break;

	case 'i':
	case 'I':
		sdio_test_init();
		break;

	case 'l':
	case 'L':
		sdio_test_is_alive();
		break;

	case 't':
	case 'T':
		sdio_test_reset();
		break;

	case 'u':
	case 'U':
		sdio_test_interrupt();
		break;

		/* Complete Memory SDIO Test */
	case 'c':
	case 'C':
		/* Total Complete memory Test */
		if (buffer[1] == 't' || buffer[1] == 'T') {
			/*check if Infinit test */
			if (buffer[2] == 'i' || buffer[2] == 'I') {
				/* Loop Forever */
				sdio_test_infinit_complete();
			} else {
				sdio_test_once_complete();
			}
		}
		/* A-Sync Complete memory Test */
		else if (buffer[1] == 'a' || buffer[1] == 'A') {
			/*check if Infinit test */
			if (buffer[2] == 'i' || buffer[2] == 'I') {
				sdio_test_infinit_async_complete();
			} else {	/* test is not infinit */

				sdio_test_once_async_complete();
			}
		}
		/* Sync Complete memory Test */
		else {
			/*check if Infinit test */
			if (buffer[2] == 'i' || buffer[2] == 'I') {
				sdio_test_infinit_sync_complete();
			} else {	/* test is not infinit */

				sdio_test_once_sync_complete();
			}
		}
		break;

		/* Not-aligned Complete Memory SDIO Test */
	case 'n':
	case 'N':

		/* Total Not-aligned Complete memory Test */
		if (buffer[1] == 't' || buffer[1] == 'T') {
			/*check if Infinit test */
			if (buffer[2] == 'i' || buffer[2] == 'I') {
				sdio_test_infinit_not_aligned_complete();
			} else {
				sdio_test_once_not_aligned_complete();
			}
		}
		/* A-Sync Not-aligned Complete memory Test */
		else if (buffer[1] == 'a' || buffer[1] == 'A') {
			/*check if Infinit test */
			if (buffer[2] == 'i' || buffer[2] == 'I') {
				sdio_test_infinit_not_aligned_async_complete();
			} else {
				sdio_test_once_not_aligned_async_complete();
			}
		}
		/* Sync Not-aligned Complete memory Test */
		else {
			/*check if Infinit test */
			if (buffer[2] == 'i' || buffer[2] == 'I') {
				sdio_test_infinit_not_aligned_sync_complete();
			} else {
				sdio_test_once_not_aligned_sync_complete();
			}
		}

		break;
	case 'd':
	case 'D':		// dump out the registers
		sdioDrv_PrintRegisters();

		break;

	default:
		usage();
		break;
	};

	return count;

}				/* sdio_test_write_proc() */

/*----------------------------------------------------------------------------*/

static int sdio_test_read_proc(char *page, char **start, off_t off, int count,
			       int *eof, void *data)
{
	int len = 0;

	len = sprintf(page, "%x\n", (unsigned int)g_last_read);
	*eof = 1;

	if (!g_Quiet) {
		printk("Read_proc %X\n", (unsigned int)g_last_read);
	}

	return len;

}				/* sdio_test_read_proc() */

/*----------------------------------------------------------------------------*/

static irqreturn_t tiwlan_interrupt(int irq, void *not_used)
{
	unsigned long base;
	char *cmd = "wr 4f0 1";

	base = sdio_test_get_base(cmd[1]);
	sdio_test_write(&cmd[2], base, SdioSync, SDIO_BLOCK_NONE);

	if (!g_Quiet) {
		printk("tiwlan_interrupt() handled\n");
	}

	return IRQ_HANDLED;

}				/* tiwlan_interrupt() */

/*----------------------------------------------------------------------------*/

static int __init sdio_test_module_init(void)
{
	struct proc_dir_entry *mmc_api_file;
	int err;

	if ((mmc_api_file =
	     create_proc_entry(TESTDRV_MODULE_NAME, 0666, NULL)) == NULL) {
		printk("Could not create /proc entry\n");
		return -ENOMEM;
	}

	mmc_api_file->data = NULL;
	mmc_api_file->read_proc = (read_proc_t *) sdio_test_read_proc;
	mmc_api_file->write_proc = (write_proc_t *) sdio_test_write_proc;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
	mmc_api_file->owner = THIS_MODULE;
#endif

#ifndef TNETW1253
	if (gpio_request(TESTDRV_GPIO_PM_3430, "PMENA_GPIO") != 0)
	/* if (omap_request_gpio(TESTDRV_GPIO_PM_3430) != 0)  */
	{
		printk(KERN_ERR
		       "%s:OMAP3430_TNETW_Power() omap_request_gpio FAILED\n",
		       __FILE__);
		return -EINVAL;
	}
#endif
/* omap_set_gpio_direction(TESTDRV_GPIO_PM_3430, TESTDRV_GPIO_OUTPUT); */
	gpio_direction_output(TESTDRV_GPIO_PM_3430, 0);
	printk("OMAP3430_Wlan_Hardware_Init start\n");
	err = OMAP3430_Wlan_Hardware_Init();
	printk("OMAP3430_Wlan_Hardware_Init - %s\n", err ? "FAIL" : "SUCCESS");

	sdio_test_reset();
	printk("sdio_test_module_init: SDIO Test Reset OK\n");

	memset(&g_sdio, 0, sizeof(g_sdio));
	sema_init(&g_sdio.sem, 0);
#ifndef TNETW1253
	/* if (omap_request_gpio(TESTDRV_GPIO_IRQ_3430) != 0)  */
	if (gpio_request(TESTDRV_GPIO_IRQ_3430, "IRQ_GPIO") != 0) {
		printk(KERN_ERR 
		"sdio_test_module_init() omap_request_gpio() FAILED !!\n");
		return -EINVAL;
	};

	/*omap_set_gpio_direction(TESTDRV_GPIO_IRQ_3430, GPIO_DIR_INPUT); */
	gpio_direction_input(TESTDRV_GPIO_IRQ_3430);
#endif
	/* set_irq_type (TESTDRV_TIWLAN_OMAP3430_IRQ, O
		MAP_GPIO_FALLING_EDGE);
	omap_set_gpio_edge_ctrl(TESTDRV_GPIO_IRQ_3430, 
		OMAP_GPIO_FALLING_EDGE);*/
	set_irq_type(TESTDRV_TIWLAN_IRQ, IRQ_TYPE_EDGE_FALLING);
	if (request_irq
	    (TESTDRV_TIWLAN_IRQ, tiwlan_interrupt, IRQF_SHARED,
	     TESTDRV_MODULE_NAME, &g_sdio)) {
		printk("sdio_test_module_init()"
			" Failed to register interrupt handler!!\n");
	}

	printk("sdio_test_module_init: OK\n");

	return init_buffers();

}				/*  sdio_test_module_init() */

/*----------------------------------------------------------------------------*/

static void __exit sdio_test_module_exit(void)
{
	sdioDrv_DisconnectBus();

	if (write_buf) {
		kfree(write_buf);
	}
	if (read_buf) {
		kfree(read_buf);
	}
	free_irq(TESTDRV_TIWLAN_IRQ, &g_sdio);
	/* omap_free_gpio(TESTDRV_GPIO_IRQ_3430); */
	gpio_free(TESTDRV_GPIO_IRQ_3430);
	/* omap_free_gpio(TESTDRV_GPIO_PM_3430); */
	gpio_free(TESTDRV_GPIO_PM_3430);
	remove_proc_entry("sdio_test", NULL);

}				/* sdio_test_exit() */

/*--------------------------------------------------------------------------------------*/

MODULE_AUTHOR("Texas Instruments");
MODULE_DESCRIPTION("OMAP3430 MMC Test");
MODULE_LICENSE("GPL");

module_init(sdio_test_module_init);
module_exit(sdio_test_module_exit);

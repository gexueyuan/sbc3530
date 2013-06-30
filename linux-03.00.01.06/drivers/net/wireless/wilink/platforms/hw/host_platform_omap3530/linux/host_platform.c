/*
 * host_platform.c
 *
 * Copyright(c) 1998 - 2010 Texas Instruments. All rights reserved.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name Texas Instruments nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "tidef.h"
#include <linux/version.h>
#include <linux/kernel.h>
#include <asm/io.h>

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,22)
#include <asm/arch-omap/tc.h>
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
#include <plat/io.h>
#else
#include <mach/tc.h>
#endif
#endif

#include <linux/delay.h>

#include "host_platform.h"
#include "ioctl_init.h"
#include "WlanDrvIf.h"
#include "Device1273.h"

#define OS_API_MEM_ADRR  	0x0000000
#define OS_API_REG_ADRR  	0x300000

#define SDIO_ATTEMPT_LONGER_DELAY_LINUX  200

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

/*----------------------------------------------------------------------------*/

static int OMAP3430_TNETW_Power(int power_on)
{
	if (power_on) {
		gpio_direction_output(PMENA_GPIO, 1);
	} else {
		gpio_direction_output(PMENA_GPIO, 0);
	}

	return 0;

}				/* OMAP3430_TNETW_Power() */

/*-----------------------------------------------------------------------------

Routine Name:

        hPlatform_hardResetTnetw

Routine Description:

        set the GPIO to low after awaking the TNET from ELP.

Arguments:

        OsContext - our adapter context.

Return Value:

        None

-----------------------------------------------------------------------------*/

int hPlatform_hardResetTnetw(void)
{
	int err;

	/* Turn power OFF */
	if ((err = OMAP3430_TNETW_Power(0)) == 0) {
		mdelay(500);
		/* Turn power ON */
		err = OMAP3430_TNETW_Power(1);
		mdelay(50);
	}

	return err;

}				/* hPlatform_hardResetTnetw() */

/*----------------------------------------------------------------------------*/

/* Turn device power off */
int hPlatform_DevicePowerOff(void)
{
	int err;
	err = OMAP3430_TNETW_Power(0);
	mdelay(100);

	return err;
}

/*----------------------------------------------------------------------------*/

/* Turn device power off according to a given delay */
int hPlatform_DevicePowerOffSetLongerDelay(void)
{
	int err;

	err = OMAP3430_TNETW_Power(0);
	mdelay(SDIO_ATTEMPT_LONGER_DELAY_LINUX);

	return err;
}

/*----------------------------------------------------------------------------*/

/* Turn device power on */

int hPlatform_DevicePowerOn(void)
{
	int err;

	err = OMAP3430_TNETW_Power(1);
	mdelay(200);

	return err;
}

/*----------------------------------------------------------------------------*/

void hPlatform_Wlan_Hardware_DeInit(void)
{
	gpio_free(PMENA_GPIO);
}

/*----------------------------------------------------------------------------*/

int hPlatform_Wlan_Hardware_Init(void)
{
	pad_config(CONTROL_PADCONF_UART1_CTS, 0xFFFFFFE4, 0x4);
	pad_config(CONTROL_PADCONF_UART1_RTS, 0xFFFCFFFF, 0x11C0000);
	pad_config(CONTROL_PADCONF_MMC2_CLK, 0xFFF8FFFF, 0x00180000);
	pad_config(CONTROL_PADCONF_MMC2_DAT0, 0xFFF8FFF8, 0x00180018);
	pad_config(CONTROL_PADCONF_MMC2_DAT2, 0xFFF8FFF8, 0x01180118);
#if defined(CONFIG_MACH_SBC3530)
	/*stalker config gpio 136 137*/
	pad_config(0x48002164, 0xF000F000, 0x011c011c);	
#endif

	gpio_request(PMENA_GPIO, "PWR ENBL for WL1271");

	return 0;

}				/* hPlatform_Wlan_Hardware_Init() */

/*-----------------------------------------------------------------------------

Routine Name:

        InitInterrupt

Routine Description:

        this function init the interrupt to the Wlan ISR routine.

Arguments:

        tnet_drv - Golbal Tnet driver pointer.

Return Value:

        status

-----------------------------------------------------------------------------*/

int hPlatform_initInterrupt(void *tnet_drv, void *handle_add)
{
	TWlanDrvIfObj *drv = tnet_drv;
	int rc;

	if (drv->irq == 0 || handle_add == NULL) {
		print_err("hPlatform_initInterrupt() bad param drv->irq=%d "
			  "handle_add=0x%x !!!\n", drv->irq, (int)handle_add);
		return -EINVAL;
	}

	if (gpio_request(IRQ_GPIO, "WLAN_IRQ") != 0) {
		print_err
		    ("hPlatform_initInterrupt() gpio_request() FAILED !!\n");
		return -EINVAL;
	}

	gpio_direction_input(IRQ_GPIO);

	if ((rc =
	     request_irq(drv->irq, handle_add, IRQF_TRIGGER_FALLING,
			 drv->netdev->name, drv))) {
		print_err("TIWLAN: Failed to register interrupt handler\n");
	}

	return rc;

}				/* hPlatform_initInterrupt() */

/*----------------------------------------------------------------------------*/

void hPlatform_freeInterrupt(void)
{
	gpio_free(IRQ_GPIO);
}

/*----------------------------------------------------------------------------*/

void *hPlatform_hwGetRegistersAddr(TI_HANDLE OsContext)
{
	return (void *)OS_API_REG_ADRR;
}

/*----------------------------------------------------------------------------*/

void *hPlatform_hwGetMemoryAddr(TI_HANDLE OsContext)
{
	return (void *)OS_API_MEM_ADRR;
}

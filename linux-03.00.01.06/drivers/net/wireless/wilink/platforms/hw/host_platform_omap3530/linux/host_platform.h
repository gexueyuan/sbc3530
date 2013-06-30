/*
 * host_platform.h
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

/*--------------------------------------------------------------------------
 Module:      host_platform_sdio.h

 Purpose:     This module defines unified interface to the host platform 
		specific sources and services.

--------------------------------------------------------------------------*/

#ifndef __HOST_PLATFORM_SDIO__H__
#define __HOST_PLATFORM_SDIO__H__

#include <mach/hardware.h>

#define OMAP_HSMMC2_BASE		0x480b4000
#define OMAP2_CONTROL_DEVCONF1          0x480022D8
/* mmc2_cmd */
#define CONTROL_PADCONF_MMC2_CLK	0x48002158
/* mmc2_dat0, mmc2_dat1 */
#define CONTROL_PADCONF_MMC2_DAT0	0x4800215C
/* mmc2_dat2, mmc2_dat3 */
#define CONTROL_PADCONF_MMC2_DAT2	0x48002160
/* WLAN_IRQ */
#define CONTROL_PADCONF_UART1_RTS       0x4800217C
/* WLAN_ENABLE */
#define CONTROL_PADCONF_UART1_CTS       0x48002180

#define CM_FCLKEN1_CORE                 0x48004A00
#define CM_ICLKEN1_CORE                 0x48004A10

#if defined(CONFIG_MACH_SBC3530)
#define PMENA_GPIO                      137	/* WLAN_ENABLE */
#define IRQ_GPIO                        136	/* WLAN_IRQ */
#else
#define PMENA_GPIO                      150	/* WLAN_ENABLE */
#define IRQ_GPIO                        149	/* WLAN_IRQ */
#endif

#define TNETW_IRQ                       (OMAP_GPIO_IRQ(IRQ_GPIO))
#define TIWLAN_IRQ_POLL_INTERVAL	HZ/100
#define HZ_IN_MSEC			HZ/1000
#define TIWLAN_IRQ_POLL_INTERVAL_MS	TIWLAN_IRQ_POLL_INTERVAL/HZ_IN_MSEC

int hPlatform_initInterrupt(void *tnet_drv, void *handle_add);
void *hPlatform_hwGetRegistersAddr(TI_HANDLE OsContext);
void *hPlatform_hwGetMemoryAddr(TI_HANDLE OsContext);
void hPlatform_freeInterrupt(void);
int hPlatform_hardResetTnetw(void);
int hPlatform_Wlan_Hardware_Init(void);
void hPlatform_Wlan_Hardware_DeInit(void);
int hPlatform_DevicePowerOff(void);
int hPlatform_DevicePowerOffSetLongerDelay(void);
int hPlatform_DevicePowerOn(void);
#endif /* __HOST_PLATFORM_SDIO__H__ */

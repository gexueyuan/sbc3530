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

/*--------------------------------------------------------------------------
 Module:      omap34xx_defs.h

 Purpose:     This header file defines omap34xx/35xx  specific addresses

--------------------------------------------------------------------------*/

#ifndef __OMAP34XX_DEFS__H__
#define __OMAP34XX_DEFS__H__

#include <mach/hardware.h>

#define OMAP_HSMMC2_BASE                0x480b4000
#define OMAP2_CONTROL_DEVCONF1          0x480022D8
/* mmc2_cmd */
#define CONTROL_PADCONF_MMC2_CLK        0x48002158
/* mmc2_dat0, mmc2_dat1 */
#define CONTROL_PADCONF_MMC2_DAT0       0x4800215C
/* mmc2_dat2, mmc2_dat3 */
#define CONTROL_PADCONF_MMC2_DAT2       0x48002160
/* WLAN_IRQ */
#define CONTROL_PADCONF_UART1_RTS       0x4800217C
/* WLAN_ENABLE */
#define CONTROL_PADCONF_UART1_CTS       0x48002180

#define CM_FCLKEN1_CORE                 0x48004A00
#define CM_ICLKEN1_CORE                 0x48004A10

#endif /* __OMAP34XX_DEFS__H__ */

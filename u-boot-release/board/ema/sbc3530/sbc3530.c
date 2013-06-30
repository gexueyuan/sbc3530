/*
 * (C) Copyright 2004-2010
 *
 * Author :
 *	Illidan <illidanfly@gmail.com>
 *
 * Derived from Beagle Board and 3430 SDP code by
 *	Richard Woodruff <r-woodruff2@ti.com>
 *	Syed Mohammed Khasim <khasim@ti.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#include <common.h>
#include <twl4030.h>
#include <asm/io.h>
#include <asm/arch/mem.h>
#include <asm/arch/mux.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/gpio.h>
#include <i2c.h>
#include <asm/mach-types.h>
#include "sbc3530.h"
#include "logo.h"

#if defined(CONFIG_CMD_NET)
#include <net.h>
#include <netdev.h>
#endif

/*
 * Routine: board_init
 * Description: Early hardware init.
 */
int board_init(void)
{
	DECLARE_GLOBAL_DATA_PTR;

	gpmc_init(); /* in SRAM or SDRAM, finish GPMC */
	/* board id for Linux */
	gd->bd->bi_arch_number = MACH_TYPE_SBC3530;
	/* boot param addr */
	gd->bd->bi_boot_params = (OMAP34XX_SDRC_CS0 + 0x100);

	return 0;
}

/*
 * Routine: dss_init_r
 * Description: Init DSS and display logo
 */
int dss_init_r(void)
{
	unsigned int i,j;
	unsigned short *px;
	
#ifdef DSS_LOGO_ENABLE
	unsigned short *py,*pz;
#endif
	
	/* Assuming a resolution of 1280x720-16 - clear dss buffer with background color */
	
	for ( i=0,px=(unsigned short *)(DSS_FRAMEBUFFER_ADDR);i<720;i++ ) 
	{
		for ( j=0;j<1280;j++,px++ )
		{
			(*px)=DSS_BACKGROUND_COLOR;
		}
	}
	
#ifdef DSS_LOGO_ENABLE
	/* Write logo data to dss buffer */
	pz = dss_logo_data;
	
	for ( i=0,px=(unsigned short *)(DSS_FRAMEBUFFER_ADDR);i<720;i++ ) 
	{
		for ( j=0;j<1280;j++,px++,pz++)
		{
			(*px)=(*pz);
		}
	}
#endif
	
	/* DSS configure */
	*((uint *) 0x48310034) = 0xfefffedf;
	*((uint *) 0x48310094) = 0x01000120;
	*((uint *) 0x48004D44) = 0x0001b00c;
	*((uint *) 0x48004E40) = 0x00001006;
	*((uint *) 0x48004D00) = 0x00370037;
	*((uint *) 0x48050C00) = 0x00000002;
	*((uint *) 0x48050C04) = 0x0000001B;
	*((uint *) 0x48050C08) = 0x00000040;
	*((uint *) 0x48050C0C) = 0x00000000;
	*((uint *) 0x48050C10) = 0x00000000;
	*((uint *) 0x48050C14) = 0x00008000;
	*((uint *) 0x48050C18) = 0x00000000;
	*((uint *) 0x48050C1C) = 0x00008359;
	*((uint *) 0x48050C20) = 0x0000020C;
	*((uint *) 0x48050C24) = 0x00000000;
	*((uint *) 0x48050C28) = 0x043F2631;
	*((uint *) 0x48050C2C) = 0x00000024;
	*((uint *) 0x48050C30) = 0x00000130;
	*((uint *) 0x48050C34) = 0x00000198;
	*((uint *) 0x48050C38) = 0x000001C0;
	*((uint *) 0x48050C3C) = 0x0000006A;
	*((uint *) 0x48050C40) = 0x0000005C;
	*((uint *) 0x48050C44) = 0x00000000;
	*((uint *) 0x48050C48) = 0x00000001;
	*((uint *) 0x48050C4C) = 0x0000003F;
	*((uint *) 0x48050C50) = 0x21F07C1F;
	*((uint *) 0x48050C54) = 0x00000000;
	*((uint *) 0x48050C58) = 0x00000015;
	*((uint *) 0x48050C5C) = 0x00001400;
	*((uint *) 0x48050C60) = 0x00000000;
	*((uint *) 0x48050C64) = 0x069300F4;
	*((uint *) 0x48050C68) = 0x0016020C;
	*((uint *) 0x48050C6C) = 0x00060107;
	*((uint *) 0x48050C70) = 0x008D034E;
	*((uint *) 0x48050C74) = 0x000F0359;
	*((uint *) 0x48050C78) = 0x01A00000;
	*((uint *) 0x48050C7C) = 0x020501A0;
	*((uint *) 0x48050C80) = 0x01AC0024;
	*((uint *) 0x48050C84) = 0x020D01AC;
	*((uint *) 0x48050C88) = 0x00000006;
	*((uint *) 0x48050C8C) = 0x00000000;
	*((uint *) 0x48050C90) = 0x03480079;
	*((uint *) 0x48050C94) = 0x02040024;
	*((uint *) 0x48050C98) = 0x00000000;
	*((uint *) 0x48050C9C) = 0x00000000;
	*((uint *) 0x48050CA0) = 0x0001008A;
	*((uint *) 0x48050CA4) = 0x01AC0106;
	*((uint *) 0x48050CA8) = 0x01060006;
	*((uint *) 0x48050CAC) = 0x00000000;
	*((uint *) 0x48050CB0) = 0x00140001;
	*((uint *) 0x48050CB4) = 0x00010001;
	*((uint *) 0x48050CB8) = 0x00FF0000;
	*((uint *) 0x48050CBC) = 0x00000000;
	*((uint *) 0x48050CC0) = 0x00000000;
	*((uint *) 0x48050CC4) = 0x0000000D;
	*((uint *) 0x48050CC8) = 0x00000000;
	*((uint *) 0x48050010) = 0x00000001;
	*((uint *) 0x48050040) = 0x00000078;
	*((uint *) 0x48050044) = 0x00000000;
	*((uint *) 0x48050048) = 0x00000000;
	*((uint *) 0x48050050) = 0x00000000;
	*((uint *) 0x48050058) = 0x00000000;
	*((uint *) 0x48050410) = 0x00002015;
	*((uint *) 0x48050414) = 0x00000001;
	*((uint *) 0x48050444) = 0x00000004;
	*((uint *) 0x4805044c) = 0xFFFFFFFF;
	*((uint *) 0x48050450) = 0x00000000;
	*((uint *) 0x48050454) = 0x00000000;
	*((uint *) 0x48050458) = 0x00000000;
	*((uint *) 0x48050464) = 0x0ff03f31;
	*((uint *) 0x48050468) = 0x01400504;
	*((uint *) 0x4805046c) = 0x00007028;
	*((uint *) 0x48050470) = 0x00010002;
	*((uint *) 0x48050478) = 0x00ef027f;
	*((uint *) 0x4805047c) = 0x02cf04ff;
	*((uint *) 0x48050480) = DSS_FRAMEBUFFER_ADDR;
	*((uint *) 0x48050484) = DSS_FRAMEBUFFER_ADDR;
	*((uint *) 0x48050488) = 0x00000000;
	*((uint *) 0x4805048c) = 0x02cf04ff;
	*((uint *) 0x480504a0) = 0x0000008d;
	*((uint *) 0x480504a4) = 0x03fc03bc;
	*((uint *) 0x480504a8) = 0x00000400;
	*((uint *) 0x480504ac) = 0x00000001;
	*((uint *) 0x480504b0) = 0x00000001;
	*((uint *) 0x480504b4) = 0x00000000;
	*((uint *) 0x480504b8) = 0x807ff000;
	udelay(1000);
	*((uint *) 0x48050440) = 0x0001836b;
	udelay(1000);
	*((uint *) 0x48050440) = 0x0001836b;
	udelay(1000);
	*((uint *) 0x48050440) = 0x0001836b;
	udelay(1000);

	return 0;
}

/*
 * Routine: misc_init_r
 * Description: Configure board specific parts
 */
int misc_init_r(void)
{	
#ifdef CONFIG_DRIVER_OMAP34XX_I2C
	i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);
#endif
	
	twl4030_power_init();
	twl4030_led_init(TWL4030_LED_LEDEN_LEDAON | TWL4030_LED_LEDEN_LEDBON);
	
#if defined(CONFIG_CMD_NET)
	setup_net_chip();
#endif
	
	dieid_num_r();
	
#ifdef DSS_ENABLE
	dss_init_r();
#endif
	
	return 0;
}

/*
 * Routine: set_muxconf_regs
 * Description: Setting up the configuration Mux registers specific to the
 *		hardware. Many pins need to be moved from protect to primary
 *		mode.
 */
void set_muxconf_regs(void)
{
#ifdef CONFIG_OMAP3_SBC3530_REV_A
	MUX_SBC3530_REV_A();
#endif
}

/*
 * Routine: setup_net_chip
 * Description: Setting up the configuration GPMC registers specific to the
 *		Ethernet hardware.
 */
static void setup_net_chip(void)
{
	struct ctrl_id *id_base = (struct ctrl_id *)OMAP34XX_ID_L4_IO_BASE;
	uchar enetaddr[6];
	u32 die_id_0;
	u32 die_id_1;	
	
#ifdef CONFIG_SMC911X
	struct ctrl *ctrl_base = (struct ctrl *)OMAP34XX_CTRL_BASE;
	
	/* Configure GPMC registers */
	writel(NET_GPMC_CONFIG1, &gpmc_cfg->cs[5].config1);
	writel(NET_GPMC_CONFIG2, &gpmc_cfg->cs[5].config2);
	writel(NET_GPMC_CONFIG3, &gpmc_cfg->cs[5].config3);
	writel(NET_GPMC_CONFIG4, &gpmc_cfg->cs[5].config4);
	writel(NET_GPMC_CONFIG5, &gpmc_cfg->cs[5].config5);
	writel(NET_GPMC_CONFIG6, &gpmc_cfg->cs[5].config6);
	writel(NET_GPMC_CONFIG7, &gpmc_cfg->cs[5].config7);
	
	/* Enable off mode for NWE in PADCONF_GPMC_NWE register */
	writew(readw(&ctrl_base ->gpmc_nwe) | 0x0E00, &ctrl_base->gpmc_nwe);
	/* Enable off mode for NOE in PADCONF_GPMC_NADV_ALE register */
	writew(readw(&ctrl_base->gpmc_noe) | 0x0E00, &ctrl_base->gpmc_noe);
	/* Enable off mode for ALE in PADCONF_GPMC_NADV_ALE register */
	writew(readw(&ctrl_base->gpmc_nadv_ale) | 0x0E00,
		&ctrl_base->gpmc_nadv_ale);
#endif
#ifdef CONFIG_DRIVER_DM9000	
	/* Configure GPMC registers for DM9000 */
	writel(NET_GPMC_CONFIG1, &gpmc_cfg->cs[6].config1);
	writel(NET_GPMC_CONFIG2, &gpmc_cfg->cs[6].config2);
	writel(NET_GPMC_CONFIG3, &gpmc_cfg->cs[6].config3);
	writel(NET_GPMC_CONFIG4, &gpmc_cfg->cs[6].config4);
	writel(NET_GPMC_CONFIG5, &gpmc_cfg->cs[6].config5);
	writel(NET_GPMC_CONFIG6, &gpmc_cfg->cs[6].config6);
	writel(NET_GPMC_CONFIG7, &gpmc_cfg->cs[6].config7);
#endif

	/* Use OMAP DIE_ID as MAC address */
	if (!eth_getenv_enetaddr("ethaddr", enetaddr))
	{
		die_id_0 = readl(&id_base->die_id_0);
		die_id_1 = readl(&id_base->die_id_1);
		
		enetaddr[0] = 0x02; /* locally administered */
		enetaddr[1] = (die_id_1) & 0xFF;
		enetaddr[2] = (die_id_0 >> 24) & 0xFF;
		enetaddr[3] = (die_id_0 >> 16) & 0xFF;
		enetaddr[4] = (die_id_0 >> 8) & 0xFF;
		enetaddr[5] = (die_id_0) & 0xFF;
		
		eth_setenv_enetaddr("ethaddr", enetaddr);
	}
	
	printf("Ethernet MAC address: %02x:%02x:%02x:%02x:%02x:%02x\n",enetaddr[0],enetaddr[1],enetaddr[2],enetaddr[3],enetaddr[4],enetaddr[5]);
}

/*
 * Routine: board_eth_init
 * Description: Setting up the Ethernet hardware.
 */
int board_eth_init(bd_t *bis)
{
	int rc = 0;
#ifdef CONFIG_SMC911X
	rc = smc911x_initialize(0, CONFIG_SMC911X_BASE);
#endif
#ifdef CONFIG_DRIVER_DM9000
	rc = dm9000_initialize(bis);
#endif
	return rc;
}

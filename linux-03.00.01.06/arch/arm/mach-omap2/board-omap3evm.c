/*
 * linux/arch/arm/mach-omap2/board-omap3evm.c
 *
 * Copyright (C) 2008 Texas Instruments
 *
 * Modified from mach-omap2/board-3430sdp.c
 *
 * Initial code: Syed Mohammed Khasim
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/input/matrix_keypad.h>
#include <linux/leds.h>
#include <linux/interrupt.h>
#include <linux/backlight.h>

#include <linux/spi/spi.h>
#include <linux/spi/ads7846.h>
#include <linux/i2c/twl.h>
#include <linux/usb/otg.h>
#include <linux/smsc911x.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>


#include <linux/regulator/machine.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/onenand.h>
#include <plat/gpmc.h>
#include <plat/nand.h>
#include <plat/usb.h>
#include <plat/common.h>
#include <plat/control.h>
#include <plat/mcspi.h>
#include <plat/clock.h>
#include <plat/omap-pm.h>
#include <plat/display.h>

#include "mux.h"
#include "sdram-micron-mt46h32m32lf-6.h"
#include "sdram-hynix-h8kds0un0mer-4em.h"
#include "mmc-twl4030.h"
#include "pm.h"
#include "prm-regbits-34xx.h"
#include "omap3-opp.h"
#include "board-omap3evm-camera.h"

#define GPMC_CS0_BASE  0x60
#define GPMC_CS_SIZE   0x30

#define NAND_BLOCK_SIZE        SZ_128K


#define OMAP3_EVM_TS_GPIO	175
#define OMAP3_EVM_EHCI_VBUS	22
#define OMAP3_EVM_EHCI_SELECT	61

#define OMAP3EVM_ETHR_START	0x2c000000
#define OMAP3EVM_ETHR_SIZE	1024
#define OMAP3EVM_ETHR_ID_REV	0x50
#define OMAP3EVM_ETHR_GPIO_IRQ	176
#define OMAP3EVM_SMSC911X_CS	5

extern struct regulator_consumer_supply twl4030_vmmc1_supply;
extern struct regulator_consumer_supply twl4030_vsim_supply;

extern struct regulator_init_data vmmc1_data;
extern struct regulator_init_data vsim_data;



static u8 omap3_evm_version;

u8 get_omap3_evm_rev(void)
{
	return omap3_evm_version;
}
EXPORT_SYMBOL(get_omap3_evm_rev);

static void __init omap3_evm_get_revision(void)
{
	void __iomem *ioaddr;
	unsigned int smsc_id;

	/* Ethernet PHY ID is stored at ID_REV register */
	ioaddr = ioremap_nocache(OMAP3EVM_ETHR_START, SZ_1K);
	if (!ioaddr)
		return;
	smsc_id = readl(ioaddr + OMAP3EVM_ETHR_ID_REV) & 0xFFFF0000;
	iounmap(ioaddr);

	switch (smsc_id) {
	/*SMSC9115 chipset*/
	case 0x01150000:
		omap3_evm_version = OMAP3EVM_BOARD_GEN_1;
		break;
	/*SMSC 9220 chipset*/
	case 0x92200000:
	default:
		omap3_evm_version = OMAP3EVM_BOARD_GEN_2;
	}
}

static struct mtd_partition omap3evm_onenand_partitions[] = {
	{
		.name           = "xloader-onenand",
		.offset         = 0,
		.size           = 4*(64*2048),
		.mask_flags     = MTD_WRITEABLE
	},
	{
		.name           = "uboot-onenand",
		.offset         = MTDPART_OFS_APPEND,
		.size		= 15*(64*2048),
		.mask_flags	= MTD_WRITEABLE
	},
	{
		.name           = "params-onenand",
		.offset         = MTDPART_OFS_APPEND,
		.size		= 1*(64*2048),
	},
	{
		.name           = "linux-onenand",
		.offset         = MTDPART_OFS_APPEND,
		.size		= 40*(64*2048),
	},
	{
		.name           = "jffs2-onenand",
		.offset         = MTDPART_OFS_APPEND,
		.size		= MTDPART_SIZ_FULL,
	},
};

static struct omap_onenand_platform_data omap3evm_onenand_data = {
	.parts = omap3evm_onenand_partitions,
	.nr_parts = ARRAY_SIZE(omap3evm_onenand_partitions),
	.dma_channel	= -1,	/* disable DMA in OMAP OneNAND driver */
};

static void __init omap3evm_onenand_init(void)
{
	gpmc_onenand_init(&omap3evm_onenand_data);
}

static struct mtd_partition omap3evm_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name           = "xloader-nand",
		.offset         = 0,
		.size           = 4*(SZ_128K),
		.mask_flags     = MTD_WRITEABLE
	},
	{
		.name           = "uboot-nand",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 14*(SZ_128K),
		.mask_flags     = MTD_WRITEABLE
	},
	{
		.name           = "params-nand",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 2*(SZ_128K)
	},
	{
		.name           = "linux-nand",
		.offset         = MTDPART_OFS_APPEND,
		.size           = 40*(SZ_128K)
	},
	{
		.name           = "jffs2-nand",
		.size           = MTDPART_SIZ_FULL,
		.offset         = MTDPART_OFS_APPEND,
	},
};

static struct omap_nand_platform_data omap3evm_nand_data = {
	.parts          = omap3evm_nand_partitions,
	.nr_parts       = ARRAY_SIZE(omap3evm_nand_partitions),
	.nand_setup     = NULL,
	.dma_channel    = -1,           /* disable DMA in OMAP NAND driver */
	.dev_ready      = NULL,
};

static struct resource omap3evm_nand_resource = {
	.flags          = IORESOURCE_MEM,
};

static struct platform_device omap3evm_nand_device = {
	.name           = "omap2-nand",
	.id             = 0,
	.dev            = {
		.platform_data  = &omap3evm_nand_data,
	},
	.num_resources  = 1,
	.resource       = &omap3evm_nand_resource,
};

#define ONENAND_MAP	0x20000000 /* OneNand flash */

void __init omap3evm_flash_init(void)
{
	u8 cs = 0;
	u8 nandcs = GPMC_CS_NUM + 1, onenandcs = GPMC_CS_NUM + 1;

	u32 gpmc_base_add = OMAP34XX_GPMC_VIRT;

	while (cs < GPMC_CS_NUM) {
		u32 ret = 0;
		ret = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG1);

		if ((ret & 0xC00) == 0x800) {

			/* Found it!! */
			if (nandcs > GPMC_CS_NUM)
				nandcs = cs;
		} else {
			ret = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG7);
			if ((ret & 0x3F) == (ONENAND_MAP >> 24))
				onenandcs = cs;
		}
		cs++;
	}

	if ((nandcs > GPMC_CS_NUM) && (onenandcs > GPMC_CS_NUM)) {
		printk(KERN_INFO "NAND/OneNAND: Unable to find configuration "
			" in GPMC\n ");
		return;
	}

	if (nandcs < GPMC_CS_NUM) {
		omap3evm_nand_data.cs   = nandcs;
		omap3evm_nand_data.gpmc_cs_baseaddr = (void *)(gpmc_base_add +
					GPMC_CS0_BASE + nandcs*GPMC_CS_SIZE);
		omap3evm_nand_data.gpmc_baseaddr   = (void *) (gpmc_base_add);

		if (platform_device_register(&omap3evm_nand_device) < 0) {
			printk(KERN_ERR "Unable to register NAND device\n");
		}
	}
	if (onenandcs < GPMC_CS_NUM)
		omap3evm_onenand_data.cs = onenandcs;
}


#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)
static struct resource omap3evm_smsc911x_resources[] = {
	[0] =	{
		.start	= OMAP3EVM_ETHR_START,
		.end	= (OMAP3EVM_ETHR_START + OMAP3EVM_ETHR_SIZE - 1),
		.flags	= IORESOURCE_MEM,
	},
	[1] =	{
		.start	= OMAP_GPIO_IRQ(OMAP3EVM_ETHR_GPIO_IRQ),
		.end	= OMAP_GPIO_IRQ(OMAP3EVM_ETHR_GPIO_IRQ),
		.flags	= (IORESOURCE_IRQ | IRQF_TRIGGER_LOW),
	},
};

static struct smsc911x_platform_config smsc911x_config = {
	.phy_interface  = PHY_INTERFACE_MODE_MII,
	.irq_polarity   = SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
	.irq_type       = SMSC911X_IRQ_TYPE_OPEN_DRAIN,
	.flags          = (SMSC911X_USE_32BIT | SMSC911X_SAVE_MAC_ADDRESS),
};

static struct platform_device omap3evm_smsc911x_device = {
	.name		= "smsc911x",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(omap3evm_smsc911x_resources),
	.resource	= &omap3evm_smsc911x_resources[0],
	.dev		= {
		.platform_data = &smsc911x_config,
	},
};

static inline void __init omap3evm_init_smsc911x(void)
{
	int eth_cs;
	struct clk *l3ck;
	unsigned int rate;

	eth_cs = OMAP3EVM_SMSC911X_CS;

	l3ck = clk_get(NULL, "l3_ck");
	if (IS_ERR(l3ck))
		rate = 100000000;
	else
		rate = clk_get_rate(l3ck);

	if (gpio_request(OMAP3EVM_ETHR_GPIO_IRQ, "SMSC911x irq") < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for smsc911x IRQ\n",
			OMAP3EVM_ETHR_GPIO_IRQ);
		return;
	}

	gpio_direction_input(OMAP3EVM_ETHR_GPIO_IRQ);
	platform_device_register(&omap3evm_smsc911x_device);
}

#else
static inline void __init omap3evm_init_smsc911x(void) { return; }
#endif

/*
 * OMAP3EVM LCD Panel control signals
 */
#define OMAP3EVM_LCD_PANEL_LR		2
#define OMAP3EVM_LCD_PANEL_UD		3
#define OMAP3EVM_LCD_PANEL_INI		152
#define OMAP3EVM_LCD_PANEL_ENVDD	153
#define OMAP3EVM_LCD_PANEL_QVGA		154
#define OMAP3EVM_LCD_PANEL_RESB		155
#define OMAP3EVM_LCD_PANEL_BKLIGHT_GPIO	210
#define OMAP3EVM_DVI_PANEL_EN_GPIO	199

static int lcd_enabled;
static int dvi_enabled;

static void __init omap3_evm_display_init(void)
{
	int r;

	r = gpio_request(OMAP3EVM_LCD_PANEL_RESB, "lcd_panel_resb");
	if (r) {
		printk(KERN_ERR "failed to get lcd_panel_resb\n");
		return;
	}
	gpio_direction_output(OMAP3EVM_LCD_PANEL_RESB, 1);

	r = gpio_request(OMAP3EVM_LCD_PANEL_INI, "lcd_panel_ini");
	if (r) {
		printk(KERN_ERR "failed to get lcd_panel_ini\n");
		goto err_1;
	}
	gpio_direction_output(OMAP3EVM_LCD_PANEL_INI, 1);

	r = gpio_request(OMAP3EVM_LCD_PANEL_QVGA, "lcd_panel_qvga");
	if (r) {
		printk(KERN_ERR "failed to get lcd_panel_qvga\n");
		goto err_2;
	}
	gpio_direction_output(OMAP3EVM_LCD_PANEL_QVGA, 0);

	r = gpio_request(OMAP3EVM_LCD_PANEL_LR, "lcd_panel_lr");
	if (r) {
		printk(KERN_ERR "failed to get lcd_panel_lr\n");
		goto err_3;
	}
	gpio_direction_output(OMAP3EVM_LCD_PANEL_LR, 1);

	r = gpio_request(OMAP3EVM_LCD_PANEL_UD, "lcd_panel_ud");
	if (r) {
		printk(KERN_ERR "failed to get lcd_panel_ud\n");
		goto err_4;
	}
	gpio_direction_output(OMAP3EVM_LCD_PANEL_UD, 1);

	r = gpio_request(OMAP3EVM_LCD_PANEL_ENVDD, "lcd_panel_envdd");
	if (r) {
		printk(KERN_ERR "failed to get lcd_panel_envdd\n");
		goto err_5;
	}
	gpio_direction_output(OMAP3EVM_LCD_PANEL_ENVDD, 0);

	return;

err_5:
	gpio_free(OMAP3EVM_LCD_PANEL_UD);
err_4:
	gpio_free(OMAP3EVM_LCD_PANEL_LR);
err_3:
	gpio_free(OMAP3EVM_LCD_PANEL_QVGA);
err_2:
	gpio_free(OMAP3EVM_LCD_PANEL_INI);
err_1:
	gpio_free(OMAP3EVM_LCD_PANEL_RESB);

}

static int omap3_evm_enable_lcd(struct omap_dss_device *dssdev)
{
	if (dvi_enabled) {
		printk(KERN_ERR "cannot enable LCD, DVI is enabled\n");
		return -EINVAL;
	}
	gpio_set_value(OMAP3EVM_LCD_PANEL_ENVDD, 0);

	/* AM/DM37x: To get DSS working with 75MHz, we must use sys_bootx
	 * pins for DSS, but since thes GPIO pins are reuired for LCD
	 * orientation we must change the mux configuration to GPIO[2-3] for
	 * SYS_BOOT[0-1]
	 */
	if (cpu_is_omap3630()) {
		omap_mux_set_gpio(OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT, 2);
		omap_mux_set_gpio(OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT, 3);

		gpio_direction_output(OMAP3EVM_LCD_PANEL_LR, 1);
		gpio_direction_output(OMAP3EVM_LCD_PANEL_UD, 1);
	}

	if (get_omap3_evm_rev() >= OMAP3EVM_BOARD_GEN_2)
		gpio_set_value(OMAP3EVM_LCD_PANEL_BKLIGHT_GPIO, 0);
	else
		gpio_set_value(OMAP3EVM_LCD_PANEL_BKLIGHT_GPIO, 1);

	lcd_enabled = 1;
	return 0;
}

static void omap3_evm_disable_lcd(struct omap_dss_device *dssdev)
{
	gpio_set_value(OMAP3EVM_LCD_PANEL_ENVDD, 1);

	if (get_omap3_evm_rev() >= OMAP3EVM_BOARD_GEN_2)
		gpio_set_value(OMAP3EVM_LCD_PANEL_BKLIGHT_GPIO, 1);
	else
		gpio_set_value(OMAP3EVM_LCD_PANEL_BKLIGHT_GPIO, 0);

	lcd_enabled = 0;
}

static struct omap_dss_device omap3_evm_lcd_device = {
	.name			= "lcd",
	.driver_name		= "sharp_ls_panel",
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines	= 18,
	.platform_enable	= omap3_evm_enable_lcd,
	.platform_disable	= omap3_evm_disable_lcd,
};

static int omap3_evm_enable_tv(struct omap_dss_device *dssdev)
{
	return 0;
}

static void omap3_evm_disable_tv(struct omap_dss_device *dssdev)
{
}

static struct omap_dss_device omap3_evm_tv_device = {
	.name			= "tv",
	.driver_name		= "venc",
	.type			= OMAP_DISPLAY_TYPE_VENC,
#if defined(CONFIG_OMAP2_VENC_OUT_TYPE_SVIDEO)
	.phy.venc.type		= OMAP_DSS_VENC_TYPE_SVIDEO,
#elif defined(CONFIG_OMAP2_VENC_OUT_TYPE_COMPOSITE)
	.phy.venc.type		= OMAP_DSS_VENC_TYPE_COMPOSITE,
#endif
	.platform_enable	= omap3_evm_enable_tv,
	.platform_disable	= omap3_evm_disable_tv,
};

static int omap3_evm_enable_dvi(struct omap_dss_device *dssdev)
{
	if (lcd_enabled) {
		printk(KERN_ERR "cannot enable DVI, LCD is enabled\n");
		return -EINVAL;
	}

	gpio_set_value(OMAP3EVM_DVI_PANEL_EN_GPIO, 1);

	/* AM/DM37x: To get DSS working with 75MHz, we must use sys_bootx
	 * pins for DSS, but since thes GPIO pins are reuired for LCD
	 * orientation we must change the mux configuration to GPIO[2-3] for
	 * SYS_BOOT[0-1]
	 */
	if (cpu_is_omap3630()) {
		omap_mux_set_gpio(OMAP_MUX_MODE3, 2);
		omap_mux_set_gpio(OMAP_MUX_MODE3, 3);
	}

	dvi_enabled = 1;
	return 0;
}

static void omap3_evm_disable_dvi(struct omap_dss_device *dssdev)
{
	gpio_set_value(OMAP3EVM_DVI_PANEL_EN_GPIO, 0);

	dvi_enabled = 0;
}

static struct omap_dss_device omap3_evm_dvi_device = {
	.name			= "dvi",
	.driver_name		= "generic_panel",
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines	= 24,
	.platform_enable	= omap3_evm_enable_dvi,
	.platform_disable	= omap3_evm_disable_dvi,
};

static struct omap_dss_device *omap3_evm_dss_devices[] = {
	&omap3_evm_lcd_device,
	&omap3_evm_tv_device,
	&omap3_evm_dvi_device,
};

static struct omap_dss_board_info omap3_evm_dss_data = {
	.num_devices	= ARRAY_SIZE(omap3_evm_dss_devices),
	.devices	= omap3_evm_dss_devices,
	.default_device	= &omap3_evm_lcd_device,
};

static struct platform_device omap3_evm_dss_device = {
	.name		= "omapdss",
	.id		= -1,
	.dev		= {
		.platform_data = &omap3_evm_dss_data,
	},
};

#ifdef CONFIG_WL1271_WLAN
static struct platform_device omap3_evm_wl1271_device = {
	.name = "tiwlan_pm_driver",
	.id   = -1,
	.dev = {
	.platform_data = NULL,
	},
};
#endif
/*
 * PWMA/B register offsets (TWL4030_MODULE_PWMA)
 */
#define TWL_LED_EN	0x0
#define TWL_LED_PWMON	0x0
#define TWL_LED_PWMOFF	0x1

static void omap3evm_set_bl_intensity(int intensity)
{
	unsigned char c;

	if (intensity > 100)
		return;
	/*
	 * Enable LEDA for backlight
	 */
	twl_i2c_write_u8(TWL4030_MODULE_LED, 0x11, TWL_LED_EN);

	if (get_omap3_evm_rev() >= OMAP3EVM_BOARD_GEN_2) {
		c = ((125 * (100 - intensity)) / 100) + 1;
		twl_i2c_write_u8(TWL4030_MODULE_PWMA, 0x7F,
				TWL_LED_PWMOFF);
		twl_i2c_write_u8(TWL4030_MODULE_PWMA, c,
				TWL_LED_PWMON);
	} else {
		c = ((125 * (100 - intensity)) / 100) + 2;
		twl_i2c_write_u8(TWL4030_MODULE_PWMA, 0x1,
				TWL_LED_PWMON);
		twl_i2c_write_u8(TWL4030_MODULE_PWMA, c,
				TWL_LED_PWMOFF);
	}
}

static struct generic_bl_info omap3evm_bl_platform_data = {
	.name			= "omap3evm-bklight",
	.max_intensity		= 100,
	.default_intensity	= 70,
	.limit_mask		= 0,
	.set_bl_intensity	= omap3evm_set_bl_intensity,
	.kick_battery		= NULL,
};

static struct platform_device omap3evm_bklight_device = {
	.name		= "generic-bl",
	.id		= -1,
	.dev		= {
		.parent		= &omap3_evm_dss_device.dev,
		.platform_data	= &omap3evm_bl_platform_data,
	},
};


static struct platform_device omap3evm_camkit_device = {
	.name		= "omap3evm_camkit",
	.id		= -1,
};

static struct regulator_consumer_supply omap3evm_vaux2_supplies = {
	.supply		= "hsusb1",
};

/* VAUX2 for USB */
static struct regulator_init_data omap3evm_vaux2 = {
	.constraints = {
		.min_uV		= 2800000,
		.max_uV		= 2800000,
		.apply_uV	= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies		= 1,
	.consumer_supplies		= &omap3evm_vaux2_supplies,
};

static struct twl4030_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.wires		= 4,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= 63,
	},
	{}	/* Terminator */
};

static struct gpio_led gpio_leds[] = {
	{
		.name			= "omap3evm::ledb",
		/* normally not visible (board underside) */
		.default_trigger	= "default-on",
		.gpio			= -EINVAL,	/* gets replaced */
		.active_low		= true,
	},
};

static struct gpio_led_platform_data gpio_led_info = {
	.leds		= gpio_leds,
	.num_leds	= ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_led_info,
	},
};

#ifdef CONFIG_PM
/*
 * Save the state of keypad
 *
 * TODO: This definition should ideally be in a header file, but
 *       matrix_keypad.h is not the right one. Also, plat/keypad.h
 *       is no longer used.
 */
struct omap_keypad_pm_state {
	void __iomem *wk_st;
	void __iomem *wk_en;
	u32 wk_mask;
	u32 padconf;
};

/*
 * Board specific hook for keypad suspend
 */
void omap3_evm_kp_suspend(void *ptr)
{
	struct omap_keypad_pm_state *pstate =
			(struct omap_keypad_pm_state *)ptr;

	if (pstate) {
		/*
		 * Set wake-enable bit
		 */
		if (pstate->wk_en && pstate->wk_mask) {
			u32 v = __raw_readl(pstate->wk_en);
			v |= pstate->wk_mask;
			__raw_writel(v, pstate->wk_en);
		}
		/*
		 * Set corresponding IOPAD wakeup-enable
		 */
		if (cpu_is_omap34xx() && pstate->padconf) {
			u16 v = omap_ctrl_readw(pstate->padconf);
			v |= OMAP3_PADCONF_WAKEUPENABLE0;
			omap_ctrl_writew(v, pstate->padconf);
		}
	}
}

/*
 * Board specific hook for keypad resume
 */
void omap3_evm_kp_resume(void *ptr)
{
	struct omap_keypad_pm_state *pstate =
			(struct omap_keypad_pm_state *)ptr;

	if (pstate) {
		/*
		 * Clear wake-enable bit
		 */
		if (pstate->wk_en && pstate->wk_mask) {
			u32 v = __raw_readl(pstate->wk_en);
			v &= ~pstate->wk_mask;
			__raw_writel(v, pstate->wk_en);
		}
		/*
		 * Clear corresponding IOPAD wakeup-enable
		 */
		if (cpu_is_omap34xx() && pstate->padconf) {
			u16 v = omap_ctrl_readw(pstate->padconf);
			v &= ~OMAP3_PADCONF_WAKEUPENABLE0;
			omap_ctrl_writew(v, pstate->padconf);
		}
	}
}

static struct omap_keypad_pm_state omap3evm_kp_pm_state = {
	.wk_st		= OMAP34XX_PRM_REGADDR(WKUP_MOD, PM_WKST1),
	.wk_en		= OMAP34XX_PRM_REGADDR(WKUP_MOD, PM_WKEN1),
	.wk_mask	= OMAP3430_EN_GPIO1,
	.padconf	= 0x1e0,
};

static struct omap_opp * _omap35x_mpu_rate_table	= omap35x_mpu_rate_table;
static struct omap_opp * _omap37x_mpu_rate_table	= omap37x_mpu_rate_table;
static struct omap_opp * _omap35x_dsp_rate_table	= omap35x_dsp_rate_table;
static struct omap_opp * _omap37x_dsp_rate_table	= omap37x_dsp_rate_table;
static struct omap_opp * _omap35x_l3_rate_table		= omap35x_l3_rate_table;
static struct omap_opp * _omap37x_l3_rate_table		= omap37x_l3_rate_table;
#else	/* CONFIG_PM */
static struct omap_opp * _omap35x_mpu_rate_table	= NULL;
static struct omap_opp * _omap37x_mpu_rate_table	= NULL;
static struct omap_opp * _omap35x_dsp_rate_table	= NULL;
static struct omap_opp * _omap37x_dsp_rate_table	= NULL;
static struct omap_opp * _omap35x_l3_rate_table		= NULL;
static struct omap_opp * _omap37x_l3_rate_table		= NULL;
#endif	/* CONFIG_PM */

static int omap3evm_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	/* gpio + 0 is "mmc0_cd" (input/IRQ) */
	omap_mux_init_gpio(63, OMAP_PIN_INPUT);
	mmc[0].gpio_cd = gpio + 0;
	twl4030_mmc_init(mmc);

	/* link regulators to MMC adapters */
	twl4030_vmmc1_supply.dev = mmc[0].dev;
	twl4030_vsim_supply.dev = mmc[0].dev;

	/*
	 * Most GPIOs are for USB OTG.  Some are mostly sent to
	 * the P2 connector; notably LEDA for the LCD backlight.
	 */

	/* TWL4030_GPIO_MAX + 0 == ledA, LCD Backlight control */
	gpio_request(gpio + TWL4030_GPIO_MAX, "EN_LCD_BKL");
	if (get_omap3_evm_rev() >= OMAP3EVM_BOARD_GEN_2)
		gpio_direction_output(gpio + TWL4030_GPIO_MAX, 1);
	else
		gpio_direction_output(gpio + TWL4030_GPIO_MAX, 0);

	/* gpio + 7 == DVI Enable */
	gpio_request(gpio + 7, "EN_DVI");
	gpio_direction_output(gpio + 7, 0);

	/* TWL4030_GPIO_MAX + 1 == ledB (out, active low LED) */
	gpio_leds[2].gpio = gpio + TWL4030_GPIO_MAX + 1;

	platform_device_register(&leds_gpio);

	return 0;
}

static struct twl4030_gpio_platform_data omap3evm_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.use_leds	= true,
	.setup		= omap3evm_twl_gpio_setup,
};

static struct twl4030_usb_data omap3evm_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};

static int board_keymap[] = {
	KEY(0, 0, KEY_LEFT),
	KEY(0, 1, KEY_DOWN),
	KEY(0, 2, KEY_ENTER),
	KEY(0, 3, KEY_M),

	KEY(1, 0, KEY_RIGHT),
	KEY(1, 1, KEY_UP),
	KEY(1, 2, KEY_I),
	KEY(1, 3, KEY_N),

	KEY(2, 0, KEY_A),
	KEY(2, 1, KEY_E),
	KEY(2, 2, KEY_J),
	KEY(2, 3, KEY_O),

	KEY(3, 0, KEY_B),
	KEY(3, 1, KEY_F),
	KEY(3, 2, KEY_K),
	KEY(3, 3, KEY_P)
};

static struct matrix_keymap_data board_map_data = {
	.keymap			= board_keymap,
	.keymap_size		= ARRAY_SIZE(board_keymap),
};

static struct twl4030_keypad_data omap3evm_kp_data = {
	.keymap_data	= &board_map_data,
	.rows		= 4,
	.cols		= 4,
	.rep		= 1,
#ifdef CONFIG_PM
	.pm_state	= (void *)&omap3evm_kp_pm_state,
	.on_suspend	= omap3_evm_kp_suspend,
	.on_resume	= omap3_evm_kp_resume,
#endif	/* CONFIG_PM */
};

static struct twl4030_madc_platform_data omap3evm_madc_data = {
	.irq_line	= 1,
};

static struct twl4030_codec_audio_data omap3evm_audio_data = {
	.audio_mclk = 26000000,
};

static struct twl4030_codec_data omap3evm_codec_data = {
	.audio_mclk = 26000000,
	.audio = &omap3evm_audio_data,
};

static struct regulator_consumer_supply omap3_evm_vdda_dac_supply = {
	.supply		= "vdda_dac",
	.dev		= &omap3_evm_dss_device.dev,
};

/* VDAC for DSS driving S-Video */
static struct regulator_init_data omap3_evm_vdac = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &omap3_evm_vdda_dac_supply,
};

/* VPLL2 for digital video outputs */
static struct regulator_consumer_supply omap3_evm_vpll2_supplies[] = {
	{
		.supply	= "vdvi",
		.dev	= &omap3_evm_lcd_device.dev,
	},
	{
		.supply	= "vdds_dsi",
		.dev	= &omap3_evm_dss_device.dev,
	},
};

static struct regulator_init_data omap3_evm_vpll2 = {
	.constraints = {
		.name			= "VDVI",
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= ARRAY_SIZE(omap3_evm_vpll2_supplies),
	.consumer_supplies	= omap3_evm_vpll2_supplies,
};

/* VUSB1V5 */
static struct regulator_consumer_supply omap3_evm_vusb1v5_supply = {
	.supply		= "hsusb1-aux",
};

static struct regulator_init_data omap3_evm_vusb1v5 = {
	.constraints = {
		.name			= "VUSB1V5",
		.min_uV			= 1500000,
		.max_uV			= 1500000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &omap3_evm_vusb1v5_supply,
};

/* VUSB1V8 */
static struct regulator_consumer_supply omap3_evm_vusb1v8_supply = {
	.supply		= "hsusb1",
};

static struct regulator_init_data omap3_evm_vusb1v8 = {
	.constraints = {
		.name			= "VUSB1V8",
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.apply_uV		= true,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &omap3_evm_vusb1v8_supply,
};

static struct twl4030_platform_data omap3evm_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.keypad		= &omap3evm_kp_data,
	.madc		= &omap3evm_madc_data,
	.usb		= &omap3evm_usb_data,
	.gpio		= &omap3evm_gpio_data,
	.codec		= &omap3evm_codec_data,
	.vdac		= &omap3_evm_vdac,
	.vpll2		= &omap3_evm_vpll2,
	.vaux2		= &omap3evm_vaux2,
};

static struct i2c_board_info __initdata omap3evm_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl4030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &omap3evm_twldata,
	},
};

static int __init omap3_evm_i2c_init(void)
{
	/*
	 * REVISIT: These entries can be set in omap3evm_twl_data
	 * after a merge with MFD tree
	 */
	omap3evm_twldata.vmmc1 = &vmmc1_data;
	omap3evm_twldata.vsim = &vsim_data;

	omap_register_i2c_bus(1, 2600, omap3evm_i2c_boardinfo,
			ARRAY_SIZE(omap3evm_i2c_boardinfo));
	omap_register_i2c_bus(2, 400, NULL, 0);
	omap_register_i2c_bus(3, 400, NULL, 0);
	return 0;
}

static void ads7846_dev_init(void)
{
	if (gpio_request(OMAP3_EVM_TS_GPIO, "ADS7846 pendown") < 0)
		printk(KERN_ERR "can't get ads7846 pen down GPIO\n");

	gpio_direction_input(OMAP3_EVM_TS_GPIO);

	omap_set_gpio_debounce(OMAP3_EVM_TS_GPIO, 1);
	omap_set_gpio_debounce_time(OMAP3_EVM_TS_GPIO, 0xa);
}

static int ads7846_get_pendown_state(void)
{
	return !gpio_get_value(OMAP3_EVM_TS_GPIO);
}

struct ads7846_platform_data ads7846_config = {
	.x_max			= 0x0fff,
	.y_max			= 0x0fff,
	.x_plate_ohms		= 180,
	.pressure_max		= 255,
	.debounce_max		= 10,
	.debounce_tol		= 3,
	.debounce_rep		= 1,
	.get_pendown_state	= ads7846_get_pendown_state,
	.keep_vref_on		= 1,
	.settle_delay_usecs	= 150,
	.wakeup				= true,
};

static struct omap2_mcspi_device_config ads7846_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel	= 1,	/* 0: slave, 1: master */
};

struct spi_board_info omap3evm_spi_board_info[] = {
	[0] = {
		.modalias		= "ads7846",
		.bus_num		= 1,
		.chip_select		= 0,
		.max_speed_hz		= 1500000,
		.controller_data	= &ads7846_mcspi_config,
		.irq			= OMAP_GPIO_IRQ(OMAP3_EVM_TS_GPIO),
		.platform_data		= &ads7846_config,
	},
};

static struct omap_board_config_kernel omap3_evm_config[] __initdata = {
};

static void __init omap3_evm_init_irq(void)
{
	omap_board_config = omap3_evm_config;
	omap_board_config_size = ARRAY_SIZE(omap3_evm_config);

	if (cpu_is_omap3630())
	{
		omap2_init_common_hw(h8kds0un0mer4em_sdrc_params,
					NULL,
					_omap37x_mpu_rate_table,
					_omap37x_dsp_rate_table,
					_omap37x_l3_rate_table);
	}
	else
	{
		omap2_init_common_hw(mt46h32m32lf6_sdrc_params,
					NULL,
					_omap35x_mpu_rate_table,
					_omap35x_dsp_rate_table,
					_omap35x_l3_rate_table);
	}
	omap_init_irq();
	omap_gpio_init();
}

static struct platform_device *omap3_evm_devices[] __initdata = {
	&omap3_evm_dss_device,
	&omap3evm_camkit_device,
	&omap3evm_bklight_device,
#ifdef CONFIG_WL1271_WLAN
	&omap3_evm_wl1271_device,
#endif
};

static struct ehci_hcd_omap_platform_data ehci_pdata __initconst = {

	.port_mode[0] = EHCI_HCD_OMAP_MODE_UNKNOWN,
	.port_mode[1] = EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[2] = EHCI_HCD_OMAP_MODE_UNKNOWN,

	.phy_reset  = true,
	/* PHY reset GPIO will be runtime programmed based on EVM version */
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL,

	.aux[0]	= 0,
	.aux[1]	= 0,
	.aux[2]	= 0,
};

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux omap35x_board_mux[] __initdata = {
#ifdef CONFIG_KEYBOARD_TWL4030
	OMAP3_MUX(SYS_NIRQ, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP |
				OMAP_PIN_OFF_INPUT_PULLUP | OMAP_PIN_OFF_OUTPUT_LOW |
				OMAP_PIN_OFF_WAKEUPENABLE),
#endif
#ifdef CONFIG_TOUCHSCREEN_ADS7846
	OMAP3_MUX(MCSPI1_CS1, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP |
				OMAP_PIN_OFF_INPUT_PULLUP | OMAP_PIN_OFF_OUTPUT_LOW |
				OMAP_PIN_OFF_WAKEUPENABLE),
#endif

	{ .reg_offset = OMAP_MUX_TERMINATOR },
};

static struct omap_board_mux omap36x_board_mux[] __initdata = {
#ifdef CONFIG_KEYBOARD_TWL4030
	OMAP3_MUX(SYS_NIRQ, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP |
				OMAP_PIN_OFF_INPUT_PULLUP | OMAP_PIN_OFF_OUTPUT_LOW |
				OMAP_PIN_OFF_WAKEUPENABLE),
#endif
#ifdef CONFIG_TOUCHSCREEN_ADS7846
	OMAP3_MUX(MCSPI1_CS1, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP |
				OMAP_PIN_OFF_INPUT_PULLUP | OMAP_PIN_OFF_OUTPUT_LOW |
				OMAP_PIN_OFF_WAKEUPENABLE),
#endif
	OMAP3_MUX(DSS_DATA18, OMAP_MUX_MODE3 | OMAP_PIN_OFF_NONE),
	OMAP3_MUX(DSS_DATA19, OMAP_MUX_MODE3 | OMAP_PIN_OFF_NONE),
	OMAP3_MUX(DSS_DATA22, OMAP_MUX_MODE3 | OMAP_PIN_OFF_NONE),
	OMAP3_MUX(DSS_DATA21, OMAP_MUX_MODE3 | OMAP_PIN_OFF_NONE),
	OMAP3_MUX(DSS_DATA22, OMAP_MUX_MODE3 | OMAP_PIN_OFF_NONE),
	OMAP3_MUX(DSS_DATA23, OMAP_MUX_MODE3 | OMAP_PIN_OFF_NONE),
	OMAP3_MUX(SYS_BOOT0, OMAP_MUX_MODE3 | OMAP_PIN_OFF_NONE),
	OMAP3_MUX(SYS_BOOT1, OMAP_MUX_MODE3 | OMAP_PIN_OFF_NONE),
	OMAP3_MUX(SYS_BOOT3, OMAP_MUX_MODE3 | OMAP_PIN_OFF_NONE),
	OMAP3_MUX(SYS_BOOT4, OMAP_MUX_MODE3 | OMAP_PIN_OFF_NONE),
	OMAP3_MUX(SYS_BOOT5, OMAP_MUX_MODE3 | OMAP_PIN_OFF_NONE),
	OMAP3_MUX(SYS_BOOT6, OMAP_MUX_MODE3 | OMAP_PIN_OFF_NONE),

	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define omap36x_board_mux	NULL
#define omap35x_board_mux	NULL
#endif

static void __init omap3_evm_init(void)
{
	omap3_evm_get_revision();

	if (get_omap3_evm_rev() >= OMAP3EVM_BOARD_GEN_2) {
		omap3evm_twldata.vaux2 = &omap3evm_vaux2;
	} else {
		omap3evm_twldata.vusb1v5 = &omap3_evm_vusb1v5;
		omap3evm_twldata.vusb1v8 = &omap3_evm_vusb1v8;
	}

	if (cpu_is_omap3630())
		omap3_mux_init(omap36x_board_mux, OMAP_PACKAGE_CBB);
	else
		omap3_mux_init(omap35x_board_mux, OMAP_PACKAGE_CBB);

	omap3_evm_i2c_init();

	platform_add_devices(omap3_evm_devices, ARRAY_SIZE(omap3_evm_devices));

	spi_register_board_info(omap3evm_spi_board_info,
				ARRAY_SIZE(omap3evm_spi_board_info));

	omap_serial_init();
#ifdef CONFIG_NOP_USB_XCEIV
	/* OMAP3EVM uses ISP1504 phy and so register nop transceiver */
	usb_nop_xceiv_register();
#endif
	if (get_omap3_evm_rev() >= OMAP3EVM_BOARD_GEN_2) {
		/* enable EHCI VBUS using GPIO22 */
		omap_mux_init_gpio(22, OMAP_PIN_INPUT_PULLUP);
		gpio_request(OMAP3_EVM_EHCI_VBUS, "enable EHCI VBUS");
		gpio_direction_output(OMAP3_EVM_EHCI_VBUS, 0);
		gpio_set_value(OMAP3_EVM_EHCI_VBUS, 1);

		/* Select EHCI port on main board */
		omap_mux_init_gpio(61, OMAP_PIN_INPUT_PULLUP);
		gpio_request(OMAP3_EVM_EHCI_SELECT, "select EHCI port");
		gpio_direction_output(OMAP3_EVM_EHCI_SELECT, 0);
		gpio_set_value(OMAP3_EVM_EHCI_SELECT, 0);

		/* setup EHCI phy reset config */
		omap_mux_init_gpio(21, OMAP_PIN_INPUT_PULLUP);
		ehci_pdata.reset_gpio_port[1] = 21;

	} else {
		/* setup EHCI phy reset on MDC */
		omap_mux_init_gpio(135, OMAP_PIN_OUTPUT);
		ehci_pdata.reset_gpio_port[1] = 135;

		/* MDC also need VUSB1V5 regulator */
		ehci_pdata.aux[1] = 1;
	}
	usb_musb_init();
	omap3evm_flash_init();
	omap3evm_onenand_init();
	usb_ehci_init(&ehci_pdata);
	ads7846_dev_init();
	omap3evm_init_smsc911x();

	omap3_evm_display_init();
}

static void __init omap3_evm_map_io(void)
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}

MACHINE_START(OMAP3EVM, "OMAP3 EVM")
	/* Maintainer: Syed Mohammed Khasim - Texas Instruments */
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xfa000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap3_evm_map_io,
	.init_irq	= omap3_evm_init_irq,
	.init_machine	= omap3_evm_init,
	.timer		= &omap_timer,
MACHINE_END

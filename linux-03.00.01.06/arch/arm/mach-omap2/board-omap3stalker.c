/*
 * linux/arch/arm/mach-omap2/board-omap3stalker.c
 *
 * Copyright (C) 2008 Guangzhou EMA-Tech
 *
 * Modified from mach-omap2/board-omap3evm.c
 *
 * Initial code: Jason Lam
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
#include <linux/ctype.h>

#include <linux/spi/spi.h>
#include <linux/spi/ads7846.h>
#include <linux/i2c/twl.h>
#include <linux/usb/otg.h>
#include <linux/smsc911x.h>
#include <linux/dm9000.h>
#include <linux/gpio_keys.h>
#include <linux/i2c/at24.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>

#include <linux/regulator/machine.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>

#include <plat/board.h>
#include <plat/gpmc.h>
#include <plat/nand.h>
#include <plat/usb.h>
#include <plat/common.h>
#include <plat/control.h>
#include <plat/mcspi.h>
#include <plat/clock.h>
#include <plat/omap-pm.h>
#include <plat/display.h>
#include <plat/timer-gp.h>

#include "mux.h"
#include "sdram-micron-mt46h32m32lf-6.h"
#include "mmc-twl4030.h"
#include "pm.h"
#include "prm-regbits-34xx.h"
#include "omap3-opp.h"
#include "board-omap3stalker-camera.c"

/*board model
**------------------------------------------------------------------------------
*/
#define MODEL_STRLENGTH 20
#define HAS_SMSC	0
#define HAS_DM9000	1
#define OTG_ONLY	0
#define HOST_OTG	1

enum stalker_type_index {
	DEV35X_A2,
	DEV35X_B1,
	SBC35X_A2,
	EVM35X_B3,
	SBC35X_B1,
};

struct stalker_hwinfo {
	char *name;
	int eth;
	int usb;
	bool analog_video_in;
	bool security;
	bool wifi;
	int fram;
};

static struct stalker_hwinfo omap3stalker_hwinfo[] = {
	{
		.name	= "DEV35X-A2",
		.eth	= HAS_DM9000,
		.usb	= OTG_ONLY,
		.analog_video_in = false,
		.security = false,
		.wifi	= false,
		.fram	= 0,
	 },
	{
		.name	= "DEV35X-B1",
		.eth	= HAS_SMSC,
		.usb	= OTG_ONLY,
		.analog_video_in = false,
		.security = false,
		.wifi	= false,
		.fram	= 0,
	 },
	{
		.name	= "SBC35X-A2",
		.eth	= HAS_SMSC,
		.usb	= HOST_OTG,
		.analog_video_in = true,
		.security = false,
		.wifi	= false,
		.fram	= 0,
	 },
	{
		.name	= "EVM35X-B3",
		.eth	= HAS_SMSC,
		.usb	= HOST_OTG,
		.analog_video_in = false,
		.security = false,
		.wifi	= false,
		.fram	= 0,
	 },
	{
		.name	= "SBC35X-B1",
		.eth	= HAS_SMSC,
		.usb	= HOST_OTG,
		.analog_video_in = true,
		.security = false,
		.wifi	= false,
		.fram	= 0,
	 },
};

static int board_model = SBC35X_A2;

static int __init omap3stalker_model_setup(char *str)
{
	char *tmp_str;
	char tmp_char;
	int index;
	bool match;
	if (MODEL_STRLENGTH == strlen(str)) {
		tmp_str = str;
		for (index = DEV35X_A2;
		     index < ARRAY_SIZE(omap3stalker_hwinfo); index++) {
			if (!strnicmp
			    (tmp_str, omap3stalker_hwinfo[index].name,
			     9)) {
				board_model = index;
				match = true;
				break;
			}
		}
		if (match) {	/*decode other part */
			tmp_char = toupper(tmp_str[17]);
			if (board_model >= SBC35X_A2) {
				switch (tmp_char) {
				case 'A':
					break;
				case 'N':
					omap3stalker_hwinfo[board_model].
					    analog_video_in = false;
					break;
				}
				tmp_char = tmp_str[19];
				if (tmp_char > '0' && tmp_char <= '9') {
					omap3stalker_hwinfo[board_model].
					    fram = (int) tmp_char - '0';
				}
			}
		}
	} else {
		printk
		    ("boardmodel length mismatch, use default type:%s \n",
		     omap3stalker_hwinfo[board_model].name);
	}
	return 1;
}

__setup("boardmodel=", omap3stalker_model_setup);

static void __init omap3stalker_show_hwinfo(void)
{
	printk("EMA OMAP3XX Stalker Platform:\nmodel:\t\t\t%s\n",
	       omap3stalker_hwinfo[board_model].name);
	printk("eth:\t\t\t%s\n",
	       (omap3stalker_hwinfo[board_model].eth ==
		HAS_SMSC) ? "SMSC922x" : "DM9000A");
	printk("usbhost:\t\t\t%c\n",
	       (omap3stalker_hwinfo[board_model].usb >
		OTG_ONLY) ? 'y' : 'n');
	printk("analog video in:\t\t%c\n",
	       (omap3stalker_hwinfo[board_model].
		analog_video_in) ? 'y' : 'n');
	printk("hardward security:\t%c\n",
	       (omap3stalker_hwinfo[board_model].security) ? 'y' : 'n');
	printk("on board WIFI:\t\t%c\n",
	       (omap3stalker_hwinfo[board_model].wifi) ? 'y' : 'n');
	printk("Fram/Eeprom:\t\t%d Kbit\n",
	       (omap3stalker_hwinfo[board_model].fram ==
		0) ? 0 : 2 << (omap3stalker_hwinfo[board_model].fram - 1));
}

#define GPMC_CS0_BASE	0x60
#define GPMC_CS_SIZE	0x30

extern struct regulator_consumer_supply twl4030_vmmc1_supply;
extern struct regulator_init_data vmmc1_data;

/*nand
**------------------------------------------------------------------------------
*/
static struct mtd_partition omap3stalker_nand_partitions[] = {
       /* All the partition sizes are listed in terms of NAND block size */
       {
               .name           = "X-Loader",
               .offset         = 0,
               .size           = 4*(SZ_128K),
               .mask_flags     = MTD_WRITEABLE
       },
       {
               .name           = "U-Boot",
               .offset         = MTDPART_OFS_APPEND,
               .size           = 15*(SZ_128K),
               .mask_flags     = MTD_WRITEABLE
       },
       {
               .name           = "U-Boot Env",
               .offset         = MTDPART_OFS_APPEND,
               .size           = 1*(SZ_128K)
       },
       {
               .name           = "Kernel",
               .offset         = MTDPART_OFS_APPEND,
               .size           = 32*(SZ_128K)
       },
       {
               .name           = "File System",
               .size           = MTDPART_SIZ_FULL,
               .offset         = MTDPART_OFS_APPEND,
       },
};

static struct omap_nand_platform_data omap3stalker_nand_data = {
       .parts          = omap3stalker_nand_partitions,
       .nr_parts       = ARRAY_SIZE(omap3stalker_nand_partitions),
       .nand_setup     = NULL,
       .dma_channel    = -1,           /* disable DMA in OMAP NAND driver */
       .dev_ready      = NULL,
};

static struct resource omap3stalker_nand_resource = {
       .flags          = IORESOURCE_MEM,
};

static struct platform_device omap3stalker_nand_device = {
       .name           = "omap2-nand",
       .id             = 0,
       .dev            = {
               .platform_data  = &omap3stalker_nand_data,
       },
       .num_resources  = 1,
       .resource       = &omap3stalker_nand_resource,
};


void __init omap3stalker_flash_init(void)
{
       u8 cs = 0;
       u8 nandcs = GPMC_CS_NUM + 1;
       u32 gpmc_base_add = OMAP34XX_GPMC_VIRT;

       while (cs < GPMC_CS_NUM) {
               u32 ret = 0;
               ret = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG1);

               if ((ret & 0xC00) == 0x800) {
                       /* Found it!! */
                       if (nandcs > GPMC_CS_NUM)
                               nandcs = cs;
               }
               cs++;
       }
       if (nandcs > GPMC_CS_NUM) {
               printk(KERN_INFO "NAND: Unable to find configuration "
                       " in GPMC\n ");
               return;
       }

       if (nandcs < GPMC_CS_NUM) {
               omap3stalker_nand_data.cs   = nandcs;
               omap3stalker_nand_data.gpmc_cs_baseaddr = (void *)(gpmc_base_add +
                                       GPMC_CS0_BASE + nandcs*GPMC_CS_SIZE);
               omap3stalker_nand_data.gpmc_baseaddr   = (void *) (gpmc_base_add);

               if (platform_device_register(&omap3stalker_nand_device) < 0) {
                       printk(KERN_ERR "Unable to register NAND device\n");
                       }
       }
}

 /*smsc
 **------------------------------------------------------------------------------
 */
#if defined(CONFIG_SMSC911X) || defined(CONFIG_SMSC911X_MODULE)\
	|| defined(CONFIG_DM9000) || defined(CONFIG_DM9000_MODULE)
#define OMAP3STALKER_ETHR_START	0x2c000000
#define OMAP3STALKER_ETHR_SIZE	1024
#define OMAP3STALKER_ETHR_GPIO_IRQ	19
#define OMAP3STALKER_SMC911X_CS	5
#define OMAP3STALKER_ETHR_START_DM	0x30000000
#define OMAP3STALKER_ETHR_GPIO_IRQ_DM	21
#define OMAP3STALKER_DM9K_CS	6

#ifdef CONFIG_OMAP_DIEID_MAC
static void inline __init omap3stalker_dieid2mac(char* mac)
{
	u32 die_id_1 = omap_readl(0x4830a220);
	u32 die_id_0 = omap_readl(0x4830a224);
	mac[0] = 0x02;
	mac[1] = (die_id_1) & 0xff;
	mac[2] = (die_id_0 >>24) & 0xff;
	mac[3] = (die_id_0 >>16) & 0xff;
	mac[4] = (die_id_0 >>8) & 0xff;
	mac[5] = (die_id_0) & 0xFF;
}
#else
static void inline __init omap3stalker_dieid2mac(char* mac)
{
	return;
}
#endif

static struct resource omap3stalker_smsc911x_resources[] = {
	[0] =	{
		.start	= OMAP3STALKER_ETHR_START,
		.end	= (OMAP3STALKER_ETHR_START + OMAP3STALKER_ETHR_SIZE - 1),
		.flags	= IORESOURCE_MEM,
	},
	[1] =	{
		.start	= OMAP_GPIO_IRQ(OMAP3STALKER_ETHR_GPIO_IRQ),
		.end	= OMAP_GPIO_IRQ(OMAP3STALKER_ETHR_GPIO_IRQ),
		.flags	= (IORESOURCE_IRQ | IRQF_TRIGGER_LOW),
	},
};

static struct smsc911x_platform_config smsc911x_config = {
	.phy_interface  = PHY_INTERFACE_MODE_MII,
	.irq_polarity   = SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
	.irq_type       = SMSC911X_IRQ_TYPE_OPEN_DRAIN,
	.flags          = (SMSC911X_USE_16BIT | SMSC911X_SAVE_MAC_ADDRESS),
};

static struct platform_device omap3stalker_smsc911x_device = {
	.name		= "smsc911x",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(omap3stalker_smsc911x_resources),
	.resource	= &omap3stalker_smsc911x_resources[0],
	.dev		= {
		.platform_data = &smsc911x_config,
	},
};

static inline void __init omap3stalker_init_smsc911x(void)
{
	int eth_cs;
	struct clk *l3ck;
	unsigned int rate;

	omap3stalker_dieid2mac(smsc911x_config.mac);
	eth_cs = OMAP3STALKER_SMC911X_CS;

	l3ck = clk_get(NULL, "l3_ck");
	if (IS_ERR(l3ck))
		rate = 100000000;
	else
		rate = clk_get_rate(l3ck);

	omap_mux_init_gpio(OMAP3STALKER_ETHR_GPIO_IRQ, OMAP_PIN_INPUT_PULLUP);
	if (gpio_request(OMAP3STALKER_ETHR_GPIO_IRQ, "SMC911x irq") < 0) {
		printk(KERN_ERR "Failed to request GPIO%d for smc911x IRQ\n",
			OMAP3STALKER_ETHR_GPIO_IRQ);
		return;
	}

	gpio_direction_input(OMAP3STALKER_ETHR_GPIO_IRQ);
	platform_device_register(&omap3stalker_smsc911x_device);
}

static struct resource omap3stalker_dm9k_resource[] = {
	[0] = {
		.start	= OMAP3STALKER_ETHR_START_DM,
		.end	= OMAP3STALKER_ETHR_START_DM + 3,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= OMAP3STALKER_ETHR_START_DM + (1 << 10),
		.end	= OMAP3STALKER_ETHR_START_DM + 0x4 + 0x7c,
		.flags	= IORESOURCE_MEM,
	},
	[2] = {
		.start	= OMAP_GPIO_IRQ(OMAP3STALKER_ETHR_GPIO_IRQ_DM),
		.end	= OMAP_GPIO_IRQ(OMAP3STALKER_ETHR_GPIO_IRQ_DM),
		.flags	= IORESOURCE_IRQ | IRQF_TRIGGER_RISING,
	},
};

static struct dm9000_plat_data omap3stalker_dm9k_platdata = {
	.flags	= DM9000_PLATF_16BITONLY,
};

static struct platform_device omap3stalker_device_dm9k = {
	.name		= "dm9000",
	.id		= -1,
	.num_resources  = ARRAY_SIZE(omap3stalker_dm9k_resource),
	.resource	= &omap3stalker_dm9k_resource[0],
	.dev		= {
		.platform_data = &omap3stalker_dm9k_platdata,
	}
};

static inline void __init omap3stalker_init_dm9k(void)
{
	int eth_cs;
	unsigned long cs_mem_base;
	unsigned int rate;
	struct clk *l3ck;
	int eth_gpio = 0;
	int ret=0;

	omap3stalker_dieid2mac(omap3stalker_dm9k_platdata.dev_addr);
	eth_cs = OMAP3STALKER_DM9K_CS;

	l3ck = clk_get(NULL, "l3_ck");
	if (IS_ERR(l3ck))
		rate = 100000000;
	else
		rate = clk_get_rate(l3ck);

	if (gpmc_cs_request(eth_cs, SZ_16M, &cs_mem_base) < 0) {
		printk(KERN_ERR "Failed to request GPMC mem for dm9000\n");
		return;
	}
	omap3stalker_dm9k_resource[0].start = cs_mem_base + 0x0;
	omap3stalker_dm9k_resource[0].end   = cs_mem_base + 0x3;
	omap3stalker_dm9k_resource[1].start = cs_mem_base + (1 << 10);
	omap3stalker_dm9k_resource[1].end   = cs_mem_base + (1 << 10) + 0x7c;

	eth_gpio = OMAP3STALKER_ETHR_GPIO_IRQ_DM;

	omap3stalker_dm9k_resource[2].start = OMAP_GPIO_IRQ(eth_gpio);

	omap_mux_init_gpio(eth_gpio, OMAP_PIN_INPUT_PULLUP);
	ret = gpio_request(eth_gpio, "dm9000 IRQ");
	if (ret < 0) {
		printk(KERN_ERR "Failed to request GPIO %d for dm9000 IRQ\n",
				eth_gpio);
		return;
	}

	gpio_direction_input(eth_gpio);
	platform_device_register(&omap3stalker_device_dm9k);
}

static inline void __init omap3stalker_init_eth(void)
{
	switch(omap3stalker_hwinfo[board_model].eth){
		case HAS_DM9000:
			omap3stalker_init_dm9k();
			break;
		default:
			omap3stalker_init_smsc911x();
	}
}
#else
static inline void __init omap3stalker_init_eth(void)
{
	return;
}
#endif

/*dss
**------------------------------------------------------------------------------
*/
/*
 * OMAP3 DSS control signals
 */

static int dss_enable_gpio;
static int lcd_panel_bklight_gpio;
static int lcd_enabled;
static int dvi_enabled;

static void __init omap3_stalker_display_init(void)
{
	return;
}

static int omap3_stalker_enable_lcd(struct omap_dss_device *dssdev)
{
	if (dvi_enabled) {
		printk(KERN_ERR "cannot enable LCD, DVI is enabled\n");
		return -EINVAL;
	}
	gpio_direction_output(dss_enable_gpio, 1);
	gpio_direction_output(lcd_panel_bklight_gpio, 1);
	lcd_enabled = 1;
	return 0;
}

static void omap3_stalker_disable_lcd(struct omap_dss_device *dssdev)
{
	gpio_direction_output(dss_enable_gpio, 0);
	gpio_direction_output(lcd_panel_bklight_gpio, 0);
	lcd_enabled = 0;
}

static struct omap_dss_device omap3_stalker_lcd070_device = {
	.name			= "lcd070",
	.driver_name		= "panel-at070tn13",
	.phy.dpi.data_lines	= 24,
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.platform_enable	= omap3_stalker_enable_lcd,
	.platform_disable	= omap3_stalker_disable_lcd,
};

static struct omap_dss_device omap3_stalker_lcd043_device = {
	.name			= "lcd043",
	.driver_name		= "panel-at043tn13",
	.phy.dpi.data_lines	= 24,
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.platform_enable	= omap3_stalker_enable_lcd,
	.platform_disable	= omap3_stalker_disable_lcd,
};

static int omap3_stalker_enable_tv(struct omap_dss_device *dssdev)
{
	return 0;
}

static void omap3_stalker_disable_tv(struct omap_dss_device *dssdev)
{
}

static struct omap_dss_device omap3_stalker_tv_device = {
	.name			= "tv",
	.driver_name		= "venc",
	.type			= OMAP_DISPLAY_TYPE_VENC,
#if defined(CONFIG_OMAP2_VENC_OUT_TYPE_SVIDEO)
	.phy.venc.type		= OMAP_DSS_VENC_TYPE_SVIDEO,
#elif defined(CONFIG_OMAP2_VENC_OUT_TYPE_COMPOSITE)
	.u.venc.type		= OMAP_DSS_VENC_TYPE_COMPOSITE,
#endif
	.platform_enable	= omap3_stalker_enable_tv,
	.platform_disable	= omap3_stalker_disable_tv,
};

static int omap3_stalker_enable_dvi(struct omap_dss_device *dssdev)
{
	if (lcd_enabled) {
		printk(KERN_ERR "cannot enable DVI, LCD is enabled\n");
		return -EINVAL;
	}

	gpio_direction_output(dss_enable_gpio, 1);
	dvi_enabled = 1;
	return 0;
}

static void omap3_stalker_disable_dvi(struct omap_dss_device *dssdev)
{
	gpio_direction_output(dss_enable_gpio, 0);
	dvi_enabled = 0;
}

static struct omap_dss_device omap3_stalker_dvi_device = {
	.name			= "dvi",
	.driver_name		= "generic_panel",
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines	= 24,
	.platform_enable	= omap3_stalker_enable_dvi,
	.platform_disable	= omap3_stalker_disable_dvi,
};

static struct omap_dss_device *omap3_stalker_dss_devices[] = {
	&omap3_stalker_lcd070_device,
	&omap3_stalker_lcd043_device,
	&omap3_stalker_tv_device,
	&omap3_stalker_dvi_device,
};

static struct omap_dss_board_info omap3_stalker_dss_data = {
	.num_devices	= ARRAY_SIZE(omap3_stalker_dss_devices),
	.devices	= omap3_stalker_dss_devices,
	.default_device	= &omap3_stalker_dvi_device,
};

static struct platform_device omap3_stalker_dss_device = {
	.name		= "omapdss",
	.id		= -1,
	.dev		= {
		.platform_data = &omap3_stalker_dss_data,
	},
};

/*gpio_key
**------------------------------------------------------------------------------
*/

static struct gpio_keys_button gpio_buttons[] = {
	{
		.code			= BTN_EXTRA,
		.gpio			= 18,
		.desc			= "user",
		.wakeup			= 1,
	},
};

static struct gpio_keys_platform_data gpio_key_info = {
	.buttons	= gpio_buttons,
	.nbuttons	= ARRAY_SIZE(gpio_buttons),
};

static struct platform_device keys_gpio = {
	.name	= "gpio-keys",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_key_info,
	},
};

static void __init omap3stalker_gpio_key(void)
{
	if(board_model >= DEV35X_B1){
		omap_mux_init_gpio(18, OMAP_PIN_INPUT_PULLUP);
		platform_device_register(&keys_gpio);
	}
}
/*twl4030
**------------------------------------------------------------------------------
*/

static struct twl4030_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.wires		= 4,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= 23,
	},
	{}	/* Terminator */
};

static struct gpio_led gpio_leds[] = {
 	{
		.name			= "stalker:D8:usr0",
 		.default_trigger	= "default-on",
		.gpio			= -EINVAL,
 		.active_low		= true,
	},
	{
		.name			= "stalker:D9:usr1",
		.default_trigger	= "default-on",
		.gpio			= -EINVAL,
 		.active_low		= true,
	},
	{
		.name			= "stalker:D3:mmc0",
		.gpio			= -EINVAL,	/* gets replaced */
		.active_low		= true,
		.default_trigger	= "mmc0",
	},
	{
		.name			= "stalker:D4:heartbeat",
 		.gpio			= -EINVAL,	/* gets replaced */
 		.active_low		= true,
		.default_trigger	= "heartbeat",
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

static void omap3stalker_gpio_led_init(unsigned gpio)
{
	switch(board_model){
		case SBC35X_B1 :
			gpio_leds[0].gpio = 126;
			gpio_leds[1].gpio = 127;
			/* GPIO + TWL4030_GPIO_MAX (out, heartbeat) */
			gpio_leds[3].gpio = gpio + TWL4030_GPIO_MAX;
			break;
		case SBC35X_A2 :
			gpio_leds[0].gpio = 126;
			gpio_leds[1].gpio = 127;
			/* GPIO + 13 == ledsync (out, heartbeat) */
			gpio_leds[3].gpio = gpio + 13;
			break;
		case EVM35X_B3 :
			/* GPIO + TWL4030_GPIO_MAX (out, heartbeat) */
			gpio_leds[3].gpio = gpio + TWL4030_GPIO_MAX;
			break;
		default :
			break;
	}
	/* TWL4030_GPIO_MAX + 1 == ledB (out, mmc0) */
	gpio_leds[2].gpio = gpio + TWL4030_GPIO_MAX + 1;
	platform_device_register(&leds_gpio);
}

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
void omap3_stalker_kp_suspend(void *ptr)
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
void omap3_stalker_kp_resume(void *ptr)
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

static struct omap_keypad_pm_state omap3stalker_kp_pm_state = {
	.wk_st		= OMAP34XX_PRM_REGADDR(WKUP_MOD, PM_WKEN1),
	.wk_en		= OMAP34XX_PRM_REGADDR(WKUP_MOD, PM_WKST1),
	.wk_mask	= OMAP3430_EN_GPIO1,
	.padconf	= 0x1e0,
};

static struct omap_opp * _omap35x_mpu_rate_table	= omap35x_mpu_rate_table;
static struct omap_opp * _omap35x_dsp_rate_table	= omap35x_dsp_rate_table;
static struct omap_opp * _omap35x_l3_rate_table		= omap35x_l3_rate_table;
#else	/* CONFIG_PM */
static struct omap_opp * _omap35x_mpu_rate_table	= NULL;
static struct omap_opp * _omap35x_dsp_rate_table	= NULL;
static struct omap_opp * _omap35x_l3_rate_table		= NULL;
#endif	/* CONFIG_PM */

static int omap3stalker_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	/* gpio + 0 is "mmc0_cd" (input/IRQ) */
	omap_mux_init_gpio(23, OMAP_PIN_INPUT);
	mmc[0].gpio_cd = gpio + 0;
	twl4030_mmc_init(mmc);

	/* link regulators to MMC adapters */
	twl4030_vmmc1_supply.dev = mmc[0].dev;

	/*
	 * Most GPIOs are for USB OTG.  Some are mostly sent to
	 * the P2 connector; notably LEDA for the LCD backlight.
	 */

	/* EVM35xB3 and SBC35xB1 use tps65930 gpio6 to control lcd backlight
	 * while other use TWL4030_GPIO_MAX + 0 == ledA
	 */
	switch(board_model){
		case EVM35X_B3:
		case SBC35X_B1:
			lcd_panel_bklight_gpio = gpio + 6;
			break;
		default:
			lcd_panel_bklight_gpio = gpio + TWL4030_GPIO_MAX;
	}
	/* DEV35xA2/B1 use omap35x gpio170 to enable display
	 * SBC35xA2/EVM35xB3/SBC35xB1 use tsp65930 gpio7 to do so.
	 */
	if(board_model < SBC35X_A2)
		dss_enable_gpio = 170;
	else
		dss_enable_gpio = gpio + 7;

	gpio_request(dss_enable_gpio, "EN_DVI");
	gpio_request(lcd_panel_bklight_gpio, "EN_LCD_BKL");

	gpio_direction_output(dss_enable_gpio, 0);
	gpio_direction_output(lcd_panel_bklight_gpio, 0);

	omap3stalker_gpio_led_init(gpio);
	return 0;
}

static struct twl4030_gpio_platform_data omap3stalker_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.use_leds	= true,
	.setup		= omap3stalker_twl_gpio_setup,
};

static struct twl4030_usb_data omap3stalker_usb_data = {
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
	KEY(2, 3, KEY_K),

	KEY(3, 0, KEY_B),
	KEY(3, 1, KEY_F),
	KEY(3, 2, KEY_O),
	KEY(3, 3, KEY_P)
};

static struct matrix_keymap_data board_map_data = {
	.keymap			= board_keymap,
	.keymap_size		= ARRAY_SIZE(board_keymap),
};

static struct twl4030_keypad_data omap3stalker_kp_data = {
	.keymap_data	= &board_map_data,
	.rows		= 4,
	.cols		= 4,
	.rep		= 1,
#ifdef CONFIG_PM
	.pm_state	= (void *)&omap3stalker_kp_pm_state,
	.on_suspend	= omap3_stalker_kp_suspend,
	.on_resume	= omap3_stalker_kp_resume,
#endif	/* CONFIG_PM */
};

static struct twl4030_madc_platform_data omap3stalker_madc_data = {
	.irq_line	= 1,
};

static struct twl4030_codec_audio_data omap3stalker_audio_data = {
	.audio_mclk = 26000000,
};

static struct twl4030_codec_data omap3stalker_codec_data = {
	.audio_mclk = 26000000,
	.audio = &omap3stalker_audio_data,
};

static struct regulator_consumer_supply omap3_stalker_vdda_dac_supply = {
	.supply		= "vdda_dac",
	.dev		= &omap3_stalker_dss_device.dev,
};

/* VDAC for DSS driving S-Video */
static struct regulator_init_data omap3_stalker_vdac = {
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
	.consumer_supplies	= &omap3_stalker_vdda_dac_supply,
};

/* VPLL1 for digital video outputs */
static struct regulator_consumer_supply omap3_stalker_vpll1_supply = {
		.supply	= "vdds_dsi",
		.dev	= &omap3_stalker_dss_device.dev,
};

static struct regulator_init_data omap3_stalker_vpll1 = {
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
	.consumer_supplies	= &omap3_stalker_vpll1_supply,
};

static struct twl4030_platform_data omap3stalker_twldata __initdata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.keypad		= &omap3stalker_kp_data,
	.madc		= &omap3stalker_madc_data,
	.usb		= &omap3stalker_usb_data,
	.gpio		= &omap3stalker_gpio_data,
	.codec		= &omap3stalker_codec_data,
	.vdac		= &omap3_stalker_vdac,
	.vpll1		= &omap3_stalker_vpll1,
	.vmmc1		= &vmmc1_data,
};
/*----------------------------------------------------------------------------*/

/*cam
**------------------------------------------------------------------------------
*/
static struct platform_device omap3stalker_camkit_device = {
	.name		= "omap3evm_camkit",
	.id		= -1,
};
/*----------------------------------------------------------------------------*/

/*ads7846
**------------------------------------------------------------------------------
*/
#define OMAP3_STALKER_TS_GPIO	20
static void ads7846_dev_init(void)
{
	if (gpio_request(OMAP3_STALKER_TS_GPIO, "ADS7846 pendown") < 0) {
		printk(KERN_ERR "can't get ads7846 pen down GPIO\n");
		return;
	}

	gpio_direction_input(OMAP3_STALKER_TS_GPIO);

	omap_set_gpio_debounce(OMAP3_STALKER_TS_GPIO, 1);
	omap_set_gpio_debounce_time(OMAP3_STALKER_TS_GPIO, 0xa);
}

static int ads7846_get_pendown_state(void)
{
	return !gpio_get_value(OMAP3_STALKER_TS_GPIO);
}

struct ads7846_platform_data ads7846_config = {
	.x_max			= 0x0fff,
	.y_max			= 0x0fff,
	.x_plate_ohms		= 180,
	.pressure_max		= 255,
	.debounce_max		= 10,
	.debounce_tol		= 10,
	.debounce_rep		= 1,
	.get_pendown_state	= ads7846_get_pendown_state,
	.keep_vref_on		= 1,
	.settle_delay_usecs	= 150,
	.wakeup			= true,
};

static struct omap2_mcspi_device_config ads7846_mcspi_config = {
	.turbo_mode	= 0,
	.single_channel	= 1,	/* 0: slave, 1: master */
};

/*fram
**------------------------------------------------------------------------------
*/
static struct at24_platform_data fram_info = {
	.byte_len	= (64*1024) / 8,
	.page_size	= 8192,
	.flags		= AT24_FLAG_ADDR16 | AT24_FLAG_IRUGO,
};

static struct i2c_board_info __initdata omap3stalker_i2c_boardinfo3[] = {
	{
		I2C_BOARD_INFO("24c64", 0x50),
		.flags = I2C_CLIENT_WAKE,
		.platform_data = &fram_info,
	},
};

static struct i2c_board_info __initdata omap3stalker_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("tps65930", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &omap3stalker_twldata,
	},
};

static int __init omap3_stalker_i2c_init(void)
{
	char name[I2C_NAME_SIZE];
	int size;

	omap_register_i2c_bus(1, 2600, omap3stalker_i2c_boardinfo,
			ARRAY_SIZE(omap3stalker_i2c_boardinfo));
	omap_register_i2c_bus(2, 400, NULL, 0);

	size = omap3stalker_hwinfo[board_model].fram;
	if(size != 0)
	{
		fram_info.byte_len = (2 << (size - 1)) * 1024 / 8;
		if(size < 4){
			sprintf(name, "24c0%d", (2 << (size - 1)));
		}
		else {
			sprintf(name, "24c%d", (2 << (size - 1)));
		}
		strcpy(omap3stalker_i2c_boardinfo3[0].type, name);
		omap_register_i2c_bus(3, 400, omap3stalker_i2c_boardinfo3,
			ARRAY_SIZE(omap3stalker_i2c_boardinfo3));
	}
	else{
		omap_register_i2c_bus(3, 400, NULL, 0);
	}
	return 0;
}

struct spi_board_info omap3stalker_spi_board_info[] = {
	[0] = {
		.modalias		= "ads7846",
		.bus_num		= 1,
		.chip_select		= 0,
		.max_speed_hz		= 1500000,
		.controller_data	= &ads7846_mcspi_config,
		.irq			= OMAP_GPIO_IRQ(OMAP3_STALKER_TS_GPIO),
		.platform_data		= &ads7846_config,
	},
};

static struct omap_board_config_kernel omap3_stalker_config[] __initdata = {
};

static void __init omap3_stalker_init_irq(void)
{
	omap_board_config = omap3_stalker_config;
	omap_board_config_size = ARRAY_SIZE(omap3_stalker_config);
	omap2_init_common_hw(mt46h32m32lf6_sdrc_params,
				NULL,
				_omap35x_mpu_rate_table,
				_omap35x_dsp_rate_table,
				_omap35x_l3_rate_table);
	omap_init_irq();
#ifdef CONFIG_OMAP_32K_TIMER
	omap2_gp_clockevent_set_gptimer(12);
#endif
	omap_gpio_init();
}

static struct platform_device omap3stalker_bl = {
        .name   = "twl4030-pwm0-bl",
        .id     = -1,
};

static struct platform_device *omap3_stalker_devices[] __initdata = {
        &omap3_stalker_dss_device,
        &omap3stalker_camkit_device,
};

static struct ehci_hcd_omap_platform_data ehci_pdata __initdata = {
	.port_mode[0] = EHCI_HCD_OMAP_MODE_UNKNOWN,
	.port_mode[1] = EHCI_HCD_OMAP_MODE_UNKNOWN,
	.port_mode[2] = EHCI_HCD_OMAP_MODE_UNKNOWN,

	.phy_reset  = true,
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
	OMAP3_MUX(ETK_D6, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLUP |
				OMAP_PIN_OFF_INPUT_PULLUP | OMAP_PIN_OFF_OUTPUT_LOW |
				OMAP_PIN_OFF_WAKEUPENABLE),
#endif

	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define omap35x_board_mux	NULL
#endif

static void __init omap3_stalker_usb_init(void)
{
	if(board_model >= SBC35X_A2){
		ehci_pdata.port_mode[1] = EHCI_HCD_OMAP_MODE_PHY;
		ehci_pdata.reset_gpio_port[1] = 21;
		omap_mux_init_gpio(21, OMAP_PIN_OUTPUT);
		usb_ehci_init(&ehci_pdata);
	}
	usb_musb_init();
}

static void __init omap3_stalker_backlight_init(void)
{
	if(board_model >= EVM35X_B3)
		platform_device_register(&omap3stalker_bl);
}

static void __init omap3stalker_dc_init(void)
{
	if(omap3stalker_hwinfo[board_model].analog_video_in)
		omap3stalker_add_dc();
}

static void __init omap3_stalker_init(void)
{
	omap3stalker_show_hwinfo();
	omap3_mux_init(omap35x_board_mux, OMAP_PACKAGE_CUS);

	omap3_stalker_i2c_init();

	regulator_has_full_constraints();

	platform_add_devices(omap3_stalker_devices, ARRAY_SIZE(omap3_stalker_devices));

	spi_register_board_info(omap3stalker_spi_board_info,
				ARRAY_SIZE(omap3stalker_spi_board_info));

	omap_serial_init();

	omap3_stalker_usb_init();
	omap3stalker_gpio_key();

	omap3stalker_flash_init();
	ads7846_dev_init();
 	omap3stalker_init_eth();
	omap3stalker_dc_init();
	omap3_stalker_display_init();
	omap3_stalker_backlight_init();
}

static void __init omap3_stalker_map_io(void)
{
	omap2_set_globals_343x();
	omap2_map_common_io();
}

MACHINE_START(SBC3530, "OMAP3 STALKER")
	/* Maintainer: Jason Lam -lzg@ema-tech.com */
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xfa000000) >> 18) & 0xfffc,
	.boot_params	= 0x80000100,
	.map_io		= omap3_stalker_map_io,
	.init_irq	= omap3_stalker_init_irq,
	.init_machine	= omap3_stalker_init,
	.timer		= &omap_timer,
MACHINE_END

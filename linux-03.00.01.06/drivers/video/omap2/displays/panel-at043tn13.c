/*
 * at043tn13 panel support
 *
 * Copyright (C) 200 EAM-Tech
 * Author: Jason Lam <lzg@ema-tech.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/delay.h>

#include <plat/display.h>
static struct omap_video_timings at043tn13_panel_timings = {
	.x_res		= 480,
	.y_res		= 272,
	.pixel_clock	= 15000,
	.hsw		= 39,		/* hsync_len (4) - 1 */
	.hfp		= 2,		/* right_margin (4) - 1 */
	.hbp		= 2,		/* left_margin (40) - 1 */
	.vsw		= 44,		/* vsync_len (2) - 1 */
	.vfp		= 2,		/* lower_margin */
	.vbp		= 2,		/* upper_margin (8) - 1 */
};

static int at043tn13_panel_probe(struct omap_dss_device *dssdev)
{
	dssdev->panel.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS |	OMAP_DSS_LCD_IHS;
	dssdev->panel.timings = at043tn13_panel_timings;
	dssdev->panel.recommended_bpp = 16;
	return 0;
}

static void at043tn13_panel_remove(struct omap_dss_device *dssdev)
{
}

static int at043tn13_panel_enable(struct omap_dss_device *dssdev)
{
	int r = 0;
	if (dssdev->platform_enable)
	{
		r = dssdev->platform_enable(dssdev);
	}
	return r;
}

static void at043tn13_panel_disable(struct omap_dss_device *dssdev)
{
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
}

static int at043tn13_panel_suspend(struct omap_dss_device *dssdev)
{
	at043tn13_panel_disable(dssdev);
	return 0;
}

static int at043tn13_panel_resume(struct omap_dss_device *dssdev)
{
	return at043tn13_panel_enable(dssdev);
}

static struct omap_dss_driver at043tn13_panel = {
	.probe		= at043tn13_panel_probe,
	.remove		= at043tn13_panel_remove,

	.enable		= at043tn13_panel_enable,
	.disable	= at043tn13_panel_disable,
	.suspend	= at043tn13_panel_suspend,
	.resume		= at043tn13_panel_resume,

	.driver         = {
		.name   = "panel-at043tn13",
		.owner  = THIS_MODULE,
	},
};


static int __init at043tn13_panel_drv_init(void)
{
	omap_dss_register_driver(&at043tn13_panel);
	return 0;
}

static void __exit at043tn13_panel_drv_exit(void)
{
	omap_dss_unregister_driver(&at043tn13_panel);
}

module_init(at043tn13_panel_drv_init);
module_exit(at043tn13_panel_drv_exit);
MODULE_LICENSE("GPL");


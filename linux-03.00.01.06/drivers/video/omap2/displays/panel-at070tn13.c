/*
 * at070tn13 panel support
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
static struct omap_video_timings at070tn13_panel_timings = {
	.x_res		= 800,
	.y_res		= 480,
	.pixel_clock	= 26520,
	.hsw            = 48,	/* hsync_len (4) - 1 */
	.hfp            = 1,      /* right_margin (4) - 1 */
	.hbp            = 1,      /* left_margin (40) - 1 */
	.vsw            = 3,       /* vsync_len (2) - 1 */
	.vfp            = 12,     /* lower_margin */
	.vbp            = 25,     /* upper_margin (8) - 1 */
};

static int at070tn13_panel_probe(struct omap_dss_device *dssdev)
{
	dssdev->panel.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS |	OMAP_DSS_LCD_IHS;
	dssdev->panel.timings = at070tn13_panel_timings;
	dssdev->panel.recommended_bpp = 16;
	return 0;
}

static void at070tn13_panel_remove(struct omap_dss_device *dssdev)
{
}

static int at070tn13_panel_enable(struct omap_dss_device *dssdev)
{
	int r = 0;
	if (dssdev->platform_enable)
	{
		r = dssdev->platform_enable(dssdev);
	}
	return r;
}

static void at070tn13_panel_disable(struct omap_dss_device *dssdev)
{
	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);
}

static int at070tn13_panel_suspend(struct omap_dss_device *dssdev)
{
	at070tn13_panel_disable(dssdev);
	return 0;
}

static int at070tn13_panel_resume(struct omap_dss_device *dssdev)
{
	return at070tn13_panel_enable(dssdev);
}

static struct omap_dss_driver at070tn13_panel = {
	.probe		= at070tn13_panel_probe,
	.remove		= at070tn13_panel_remove,

	.enable		= at070tn13_panel_enable,
	.disable	= at070tn13_panel_disable,
	.suspend	= at070tn13_panel_suspend,
	.resume		= at070tn13_panel_resume,

	.driver         = {
		.name   = "panel-at070tn13",
		.owner  = THIS_MODULE,
	},
};


static int __init at070tn13_panel_drv_init(void)
{
	omap_dss_register_driver(&at070tn13_panel);
	return 0;
}

static void __exit at070tn13_panel_drv_exit(void)
{
	omap_dss_unregister_driver(&at070tn13_panel);
}

module_init(at070tn13_panel_drv_init);
module_exit(at070tn13_panel_drv_exit);
MODULE_LICENSE("GPL");


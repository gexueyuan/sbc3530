/*
 * (C) Copyright 2004-2008
 *
 * Author :
 *	Illidan <illidanfly@gmail.com>
 *
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
#ifndef _STALKER_LOGO_H_
#define _STALKER_LOGO_H_

/* DSS framebuffer address set to (memory base offset 16M) */
#define DSS_FRAMEBUFFER_ADDR	(0x81000000)

/* DSS logo config */
#define DSS_BACKGROUND_COLOR	(0xFFFF)
#undef DSS_ENABLE
#undef DSS_LOGO_ENABLE

#ifdef DSS_LOGO_ENABLE
unsigned short dss_logo_data[] =
{
	0xFFFF,
};
#endif

#endif

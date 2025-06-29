/*
 * i8253_ref.h - I8253 PIT refresh counter platform device definitions.
 *
 * Copyright (c) 2017 Akinori Furuta <afuruta@m7.dion.ne.jp>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301 USA.
 */

#if (!defined(_LINUX_I8253_REF_H))
#define _LINUX_I8253_REF_H

#define	I8253_REF_DEVICE_NAME	"i8253_ref"

#define	I8253_REF_RATE_DEFAULT_KEEP		(~(uint32_t)0)
#define	I8253_REF_RESOURCE_REFRESH_COUNTER	"refresh_counter"
#define	I8253_REF_RESOURCE_CONTROL_WORD		"control_word"


/*
 * platform device parameter passed from i8253_ref_setup.c to
 * i8253_ref.c
 */
struct i8253_ref_platfrom_data {
	uint8_t		channel;	/*!< channel. */
	uint32_t	rate_default;	/*!< divider rate. */
};

#endif /* (!defined(_LINUX_I8253_REF_H)) */
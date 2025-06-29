/*
 * i8253_control.h - I8253 PIT control register bitmap definitions.
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

#if (!defined(_LINUX_I8253_CONTROL_H))
#define _LINUX_I8253_CONTROL_H

			/* M  K   */
#define	I8253_CLK_X3	  (3579545)
#define	I8253_CLK	  ((I8253_CLK_X3 * 10 + 15) / 30)

#define	I8253_CONTROL_WORD_SC1	(0x80)
#define	I8253_CONTROL_WORD_SC0	(0x40)
#define	I8253_CONTROL_WORD_RL1	(0x20)
#define	I8253_CONTROL_WORD_RL0	(0x10)
#define	I8253_CONTROL_WORD_M2	(0x08)
#define	I8253_CONTROL_WORD_M1	(0x04)
#define	I8253_CONTROL_WORD_M0	(0x02)
#define	I8253_CONTROL_WORD_BCD	(0x01)

#define	I8253_CONTROL_WORD_SCX_CH(channel)	(I8253_CONTROL_WORD_SC0 * (channel))

#define	I8253_CONTROL_WORD_RLX_LATCH	(0x00 * I8253_CONTROL_WORD_RL0)
#define	I8253_CONTROL_WORD_RLX_LSB	(0x01 * I8253_CONTROL_WORD_RL0)
#define	I8253_CONTROL_WORD_RLX_MSB	(0x02 * I8253_CONTROL_WORD_RL0)
#define	I8253_CONTROL_WORD_RLX_LSB_MSB	(0x03 * I8253_CONTROL_WORD_RL0)

#define	I8253_CONTROL_WORD_MX_INTERRUPT		(0x00 * I8253_CONTROL_WORD_M0)
#define	I8253_CONTROL_WORD_MX_ONE_SHOT		(0x01 * I8253_CONTROL_WORD_M0)
#define	I8253_CONTROL_WORD_MX_RATE_GENERATOR	(0x02 * I8253_CONTROL_WORD_M0)
#define	I8253_CONTROL_WORD_MX_SQUARE_WAVE	(0x03 * I8253_CONTROL_WORD_M0)
#define	I8253_CONTROL_WORD_MX_SOFTWARE_TRIGGER	(0x04 * I8253_CONTROL_WORD_M0)
#define	I8253_CONTROL_WORD_MX_HARDWARE_TRIGGER	(0x05 * I8253_CONTROL_WORD_M0)

#endif /* (!defined(I8253_CONTROL_H)) */
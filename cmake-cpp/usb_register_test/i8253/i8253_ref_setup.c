/*
 * i8253_ref_setup.c - Register I8253 PIT DRAM refresh timer platform device.
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
 
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <asm/hpet.h>
#include <linux/i8253.h>
#include <linux/i8253_ref.h>
 
#if (!defined(PIT_CH1))
#define PIT_CH1 (PIT_CH0 + 1)
#endif /* (!defined(PIT_CH1)) */
 
/* i8253 PIT Refresh counter and control port resource.
 * @note name(s) are compared with argument name of platform_get_resource_byname().
 */
static struct resource i8253_ref_ports[] = {
        {       .name =         I8253_REF_RESOURCE_REFRESH_COUNTER,
                .start =        PIT_CH1,
                .end =          PIT_CH1,
                .flags =        IORESOURCE_IO,
        },
        {       .name =         I8253_REF_RESOURCE_CONTROL_WORD,
                .start =        PIT_MODE,
                .end =          PIT_MODE,
                .flags =        IORESOURCE_IO,
        },
};
 
/*
 * Platform parameters will be passed to driver.
 */
static struct i8253_ref_platfrom_data i8253_ref_ch1 = {
        .channel =      1,      /*!< Refresh counter channel. */
        .rate_default = I8253_REF_RATE_DEFAULT_KEEP,    /*!< Initial rate value. */
};
 
/*
 * Define i8253 PIT channel 1, refresh counter as platform device.
 * 
 */
static struct platform_device i8253_ref_platform_device = {
        .name =          I8253_REF_DEVICE_NAME, /*!< device name match to driver name. */
        .id =            PLATFORM_DEVID_AUTO,   /*!< automatic id numbering. */
        .id_auto =       true,                  /*!< automatic id numbering. */
        .num_resources = ARRAY_SIZE(i8253_ref_ports),
        .resource =      i8253_ref_ports,
        .dev = {        /*!< struct device. */
                .platform_data = &i8253_ref_ch1, /*!< pass platform parameter to driver. */
        },
};
 
/*
 * Register i8253 PIT platform device.
 */
static int __init i8253_ref_device_initcall(void)
{       int             ret;
 
        /* Register i8253 as platform device. */
        ret = platform_device_register(&i8253_ref_platform_device);
        if (ret != 0) {
                pr_err("%s: Can not register platform %s device. "
                       "ret=%d\n",
                        __func__, I8253_REF_DEVICE_NAME, ret
                );
        }
        return ret;
}
 
/* Add i8253_ref_device_initcall() to initialize function table. */
arch_initcall(i8253_ref_device_initcall);
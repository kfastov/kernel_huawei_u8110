/* linux/drivers/video/msm/huawei_backlight.c
 *
 * Copyright (C) 2009-2010 HUAWEI Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <mach/gpio.h>
#include <linux/earlysuspend.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include "lcdc_huawei_config.h"

static atomic_t suspend_flag = ATOMIC_INIT(0); 
static int restore_level=1;

void pwm_set_backlight(int level)
{
	if(atomic_read(&suspend_flag)) 
    	{
    	    restore_level = level;
	    return;
    	}

	lcd_set_backlight_pwm(level);
	return;
}


#ifdef CONFIG_HAS_EARLYSUSPEND
static void pwm_backlight_suspend( struct early_suspend *h)
{
	atomic_set(&suspend_flag,1);
}

static void pwm_backlight_resume( struct early_suspend *h)
{
	lcd_set_backlight_pwm(restore_level);
	atomic_set(&suspend_flag,0);	
}

static struct early_suspend pwm_backlight_early_suspend = {
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
	.suspend = pwm_backlight_suspend,
	.resume = pwm_backlight_resume,
};
#endif


static int __init pwm_backlight_init(void)
{
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&pwm_backlight_early_suspend);
#endif

	return 0;
}

module_init(pwm_backlight_init);

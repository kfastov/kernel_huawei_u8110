/* linux/arch/arm/mach-msm/include/mach/huawei_battery.h
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

#ifndef HUAWEI_BATTERY_H
#define HUAWEI_BATTERY_H

typedef enum {   
   EVENT_LCD_BACKLIGHT = 0, /* LCD */
   EVENT_IN_CAMERA = 1, /* In camera */
   EVENT_OUT_CAMERA = 2, /* Out camera */
   EVENT_CAMERA_MODULE = 3, /* camera Include In and out camera*/  
   EVENT_WIFI_NOTIFY = 4, /* WIFI */
   EVENT_BT_NOTIFY = 5, /* Blue Tooth */
   EVENT_FM_NOTIFY = 6, /* FM */
   EVENT_CODEC_NOTIFY = 7, 
   EVENT_CAMERA_FLASH_NOTIFY = 8, 
   EVENT_INVALID
} device_current_consume_type;

#define DEVICE_POWER_STATE_OFF 0
#define DEVICE_POWER_STATE_ON 1

#ifdef CONFIG_HUAWEI_EVALUATE_POWER_CONSUMPTION
extern int huawei_rpc_current_consuem_notify(device_current_consume_type device_event, __u32 device_state );
#endif 


#endif //end of   HUAWEI_BATTERY_H







































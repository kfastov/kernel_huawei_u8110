/* linux/drivers/input/touchscreen/synaptics_i2c_rmi_tm1458.c
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

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/synaptics_i2c_rmi.h>

#include <mach/gpio.h>
#include <mach/vreg.h>

#include <linux/hardware_self_adapt.h>

/*
 * DEBUG SWITCH
 *
 */ 

//#define TS_DEBUG 
#undef TS_DEBUG 

#ifdef TS_DEBUG
#define SYNAPITICS_DEBUG(fmt, args...) printk(KERN_DEBUG fmt, ##args)
#else
#define SYNAPITICS_DEBUG(fmt, args...)
#endif
#define TS_X_OFFSET  3*(TS_X_MAX/LCD_X_MAX)
#define TS_Y_OFFSET  TS_X_OFFSET

#define TS_X_MAX 1514
#define TS_Y_MAX 2558
#define LCD_X_MAX 240

#ifdef CONFIG_HUAWEI_TOUCHSCREEN_EXTRA_KEY

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef min
#define min(a,b) ((a)>(b)?(b):(a))
#endif
#ifndef max
#define max(a,b) ((b)>(a)?(b):(a))
#endif
#ifndef abs
#define abs(a)  ((0 < (a)) ? (a) : -(a))
#endif

#define X_START    (0)
#define X_END      (TS_X_MAX) 
#define Y_START    (TS_Y_MAX-300)
#define Y_END      (TS_Y_MAX)
#define EXTRA_MAX_TOUCH_KEY    4
#define TS_KEY_DEBOUNCE_TIMER_MS 80


/* to define a region of touch panel */
typedef struct
{
    u16 touch_x_start;
    u16 touch_x_end;
    u16 touch_y_start;
    u16 touch_y_end;
} touch_region;

/* to define virt button of touch panel */
typedef struct 
{
    u16  center_x;
    u16  center_y;
    u16  x_width;
    u16  y_width;
    u32   touch_keycode;
} button_region;

/* to define extra touch region and virt key region */
typedef struct
{
    touch_region   extra_touch_region;
    button_region  extra_key[EXTRA_MAX_TOUCH_KEY];
} extra_key_region;

/* to init extra region and touch virt key region */
static extra_key_region   touch_extra_key_region =
{
    {X_START, X_END,Y_START,Y_END},								/* extra region */
    {
       {(TS_X_MAX*1/8),   (TS_Y_MAX-2), 160, 150, KEY_BACK},  /* back key */
       {(TS_X_MAX*3/8),   (TS_Y_MAX-2), 160, 150, KEY_MENU},  /* menu key */
       {(TS_X_MAX*5/8),   (TS_Y_MAX-2), 160, 150, KEY_HOME},  /* home key */
       {(TS_X_MAX*7/8),   (TS_Y_MAX-2), 160, 150, KEY_SEARCH},  /* Search key */
    },
};
#endif


static struct workqueue_struct *synaptics_wq;

struct synaptics_ts_data {
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct work_struct  work;
	int use_irq;
	struct hrtimer timer;	
	int (*power)(struct i2c_client* client, int on);
	struct early_suspend early_suspend;
#ifdef CONFIG_HUAWEI_TOUCHSCREEN_EXTRA_KEY
  struct input_dev *key_input;
  struct timer_list key_timer;
  u32 last_x;
  u32 last_y;
  bool is_first_point;
#endif
};

#ifdef CONFIG_HUAWEI_TOUCHSCREEN_EXTRA_KEY
/* to record keycode */
typedef struct {
	u32                 record_extra_key;             /*key value*/   
	bool                bRelease;                     /*be released?*/   
	bool                bSentPress;                  
	bool                touch_region_first;           /* to record first touch event*/
} RECORD_EXTRA_KEYCODE;

/* to record the key pressed */
static RECORD_EXTRA_KEYCODE  record_extra_keycode = {KEY_RESERVED, TRUE, TRUE, FALSE};
static void ts_update_pen_state(struct synaptics_ts_data *ts, int x, int y, int pressure);
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_ts_early_suspend(struct early_suspend *h);
static void synaptics_ts_late_resume(struct early_suspend *h);
#endif

static int synaptics_ts_power(struct i2c_client *client, int on);

#ifdef CONFIG_HUAWEI_TOUCHSCREEN_EXTRA_KEY
/*===========================================================================
FUNCTION      is_in_extra_region
DESCRIPTION
DEPENDENCIES
  None
RETURN VALUE
  true or false
SIDE EFFECTS
  None
===========================================================================*/
static bool is_in_extra_region(int pos_x, int pos_y)
{
    if (pos_x >= touch_extra_key_region.extra_touch_region.touch_x_start
        && pos_x <= touch_extra_key_region.extra_touch_region.touch_x_end
        && pos_y >= touch_extra_key_region.extra_touch_region.touch_y_start
        && pos_y <= touch_extra_key_region.extra_touch_region.touch_y_end)
    {
		SYNAPITICS_DEBUG("is_in_extra_region \n");
        return TRUE;
    }

    return FALSE;
}

/*===========================================================================
FUNCTION      touch_get_extra_keycode
DESCRIPTION
DEPENDENCIES
  None
RETURN VALUE
  KEY_VALUE
SIDE EFFECTS
  None
===========================================================================*/
static u32 touch_get_extra_keycode(int pos_x, int pos_y)
{
    int i = 0;
    u32  touch_keycode = KEY_RESERVED;
    for (i=0; i<EXTRA_MAX_TOUCH_KEY; i++)
    {
        if (abs(pos_x - touch_extra_key_region.extra_key[i].center_x) <= touch_extra_key_region.extra_key[i].x_width
         && abs(pos_y - touch_extra_key_region.extra_key[i].center_y) <= touch_extra_key_region.extra_key[i].y_width )
        {
	        touch_keycode = touch_extra_key_region.extra_key[i].touch_keycode;
	        break;
        }
    }
	
	SYNAPITICS_DEBUG("touch_keycode = %d \n",touch_keycode);
    return touch_keycode;
}

/*===========================================================================
FUNCTION      touch_pass_extra_keycode
DESCRIPTION:  
DEPENDENCIES
  None
RETURN VALUE
  None
SIDE EFFECTS
  None
===========================================================================*/
static void touch_pass_extra_keycode(struct synaptics_ts_data *ts)
{
    u32 key_code = record_extra_keycode.record_extra_key;

    if(KEY_RESERVED != key_code)
    {
        input_report_key(ts->key_input, key_code, !record_extra_keycode.bRelease);
        input_sync(ts->key_input);
		SYNAPITICS_DEBUG("input_report_key=%d, release=%d	\n", key_code, record_extra_keycode.bRelease);
    }

    return;
}

#if 0
/*===========================================================================
FUNCTION      touch_extra_key_proc
DESCRIPTION:  
DEPENDENCIES
  None
RETURN VALUE
  KEY_VALUE
SIDE EFFECTS
  None
===========================================================================*/
static void touch_extra_key_proc(struct synaptics_ts_data *ts)
{
    u32  key_tmp = KEY_RESERVED;

     
    SYNAPITICS_DEBUG("touch_extra_key_proc   \n");
    key_tmp = touch_get_extra_keycode(ts->last_x, ts->last_y) ;

    if (key_tmp == record_extra_keycode.record_extra_key 
        && key_tmp != KEY_RESERVED)    
    {
        record_extra_keycode.bRelease = FALSE; 
        touch_pass_extra_keycode(ts);
        record_extra_keycode.bSentPress= TRUE; 
    }
    else
    {
        record_extra_keycode.bRelease = TRUE;
        record_extra_keycode.record_extra_key = KEY_RESERVED;
        record_extra_keycode.bSentPress= FALSE;  
    }

    return;
}
#endif 

/*===========================================================================
FUNCTION      ts_key_timer
DESCRIPTION:  
DEPENDENCIES
  None
RETURN VALUE
  None
SIDE EFFECTS
  None
===========================================================================*/
static void ts_key_timer(unsigned long arg)
{
    u32  key_tmp = KEY_RESERVED;
    struct synaptics_ts_data *ts = (struct synaptics_ts_data *)arg;
	
	SYNAPITICS_DEBUG("ts_key_timer  \n");

    key_tmp = touch_get_extra_keycode(ts->last_x, ts->last_y) ;
    if (key_tmp == record_extra_keycode.record_extra_key 
        && key_tmp != KEY_RESERVED)    
    {
        record_extra_keycode.bRelease = FALSE; 
        touch_pass_extra_keycode(ts);
        record_extra_keycode.bSentPress= TRUE; 
    }
    else
    {
        record_extra_keycode.bRelease = TRUE;
        record_extra_keycode.record_extra_key = KEY_RESERVED;
        record_extra_keycode.bSentPress= FALSE;  
    }
}

/*===========================================================================
FUNCTION      update_pen_and_key_state
DESCRIPTION:  
DEPENDENCIES
  None
RETURN VALUE
  None
SIDE EFFECTS
  None
===========================================================================*/
static void update_pen_and_key_state(struct synaptics_ts_data *ts, int x, int y, int pressure)
{
    u32  key_tmp = KEY_RESERVED;
    
    if(pressure)  /*press*/
    {
		SYNAPITICS_DEBUG("update_pen_and_key_state x=%d, y=%d   pressure = %d  \n", x, y, pressure);
        if(is_in_extra_region(ts->last_x, ts->last_y))
        {
			if ((FALSE == record_extra_keycode.bRelease && KEY_RESERVED != record_extra_keycode.record_extra_key)
				 || true == record_extra_keycode.touch_region_first )
            {
				SYNAPITICS_DEBUG("update_pen_and_key_state return  \n");
                return;
            }

            key_tmp = touch_get_extra_keycode(x, y) ;

            if (KEY_RESERVED != key_tmp)
            {
                record_extra_keycode.record_extra_key = key_tmp;
                record_extra_keycode.bRelease = FALSE;
                record_extra_keycode.bSentPress = FALSE;
				SYNAPITICS_DEBUG("update_pen_and_key_state KEY_RESERVED != key_tmp  \n");

                /* start timer */
                mod_timer(&ts->key_timer,jiffies + msecs_to_jiffies(TS_KEY_DEBOUNCE_TIMER_MS));
            }
        }
        else
        {
			SYNAPITICS_DEBUG("update_pen_and_key_state pressure else \n");
			record_extra_keycode.touch_region_first = true;
			ts_update_pen_state(ts, x, y, pressure);
        }
    }
    else /*release*/
    {
		SYNAPITICS_DEBUG("update_pen_and_key_state  else x=%d, y=%d pressure = %d  \n", x, y, pressure);
        if(is_in_extra_region(ts->last_x, ts->last_y))
        {
			SYNAPITICS_DEBUG("update_pen_and_key_state	is_in_extra_region \n");
            del_timer(&ts->key_timer);
            if (KEY_RESERVED != record_extra_keycode.record_extra_key 
                && FALSE == record_extra_keycode.bRelease
                && TRUE == record_extra_keycode.bSentPress)
            {
                record_extra_keycode.bRelease = TRUE;
                touch_pass_extra_keycode(ts);
            }
            else
            {
				SYNAPITICS_DEBUG("update_pen_and_key_state	ts_update_pen_state \n");
                ts_update_pen_state(ts, x, y, pressure);
            }

            record_extra_keycode.bRelease = FALSE;
            record_extra_keycode.record_extra_key = KEY_RESERVED;
            record_extra_keycode.bSentPress= FALSE;
        }
        else
        {
			SYNAPITICS_DEBUG("update_pen_and_key_state	else else \n");
            ts_update_pen_state(ts, x, y, pressure);
        }

		record_extra_keycode.touch_region_first = false;
    }
}

static void ts_update_pen_state(struct synaptics_ts_data *ts, int x, int y, int pressure)
{
    SYNAPITICS_DEBUG("ts_update_pen_state x=%d, y=%d pressure = %3d  \n", x, y, pressure);
    if (pressure) {
        input_report_abs(ts->input_dev, ABS_X, x);
        input_report_abs(ts->input_dev, ABS_Y, y);
        input_report_abs(ts->input_dev, ABS_PRESSURE, pressure);
        input_report_key(ts->input_dev, BTN_TOUCH, !!pressure);
    } else {
        input_report_abs(ts->input_dev, ABS_PRESSURE, 0);
        input_report_key(ts->input_dev, BTN_TOUCH, 0);
    }

    input_sync(ts->input_dev);
}
#endif /*CONFIG_HUAWEI_TOUCHSCREEN_EXTRA_KEY*/

static void synaptics_ts_work_func(struct work_struct *work)
{
	int i;
	int ret;
	int bad_data = 0;
	struct i2c_msg msg[2];
	uint8_t start_reg;
	uint8_t buf[18];
	uint16_t x, y;
	uint8_t z,wx,wy;
	uint8_t finger;
	uint8_t gesture0;
	uint8_t gesture1;
  uint8_t device_st;
  uint8_t irq_st;
	static uint16_t last_x = 0; 
	static uint16_t last_y = 0;
	static bool is_first_point = true;
	struct synaptics_ts_data *ts = container_of(work, struct synaptics_ts_data, work);

	start_reg = 0x13;
	msg[0].addr = ts->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &start_reg;
	msg[1].addr = ts->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = sizeof(buf);
	msg[1].buf = buf;
	SYNAPITICS_DEBUG("synaptics_ts_work_func\n"); 

	for (i = 0; i < ((ts->use_irq && !bad_data) ? 1 : 3); i++) { 	
	ret = i2c_transfer(ts->client->adapter, msg, 2);
	if (ret < 0) {
			SYNAPITICS_DEBUG(KERN_ERR "synaptics_ts_work_func: i2c_transfer failed\n");
			bad_data = 1;
	} else {
				bad_data = 0;	
        device_st = buf[0];
        irq_st = buf[1];
				x = (buf[5] & 0x0f) | ((uint16_t)buf[3] << 4); /* x aixs */ 
				y= ((buf[5] & 0xf0) >> 4) | ((uint16_t)buf[4] << 4);  /* y aixs */ 
				z = buf[7];				    /* pressure */	
				wx = buf[6] & 0x0f;			    /* x width */ 	
				wy = (buf[6] & 0x1f) >> 4;			    /* y width */ 	
				finger = buf[2] & 0x0f;                        /* numbers of fingers */  
				gesture0 = buf[13];		            /* code of gesture */ 
				gesture1 = buf[14];                         /* enhanced data of gesture */  
	SYNAPITICS_DEBUG("device_st = 0x%x irq_st = 0x%x x = %d y = %d z = %d wx = %d wy = %d finger = %d gesture0 = 0x%x gesture1 = 0x%x\n",
					device_st, irq_st, x, y, z, wx, wy, finger,gesture0,gesture1);

        x = TS_X_MAX - x;
        y = TS_Y_MAX - y;
			if (z) 
			{
				/* 
				* always report the first point  whether slip	or click
				*/ 
				if (ts->is_first_point) {

					SYNAPITICS_DEBUG("is_first_point  \n");
					ts->last_x = x;
					ts->last_y = y;
					ts->is_first_point = false; 
					update_pen_and_key_state(ts, x, y, z);
				} else {
					SYNAPITICS_DEBUG("else is_first_point  \n");
					if(((max(ts->last_x, x)-min(ts->last_x, x)) > TS_X_OFFSET)
						|| ((max(ts->last_y, y)-min(ts->last_y, y)) > TS_Y_OFFSET))
					{
						ts->last_x = x;
						ts->last_y = y;
						
						SYNAPITICS_DEBUG("else update_pen_and_key_state  \n");
						update_pen_and_key_state(ts, x, y, z);
					}
				}
			} else {
				/* 
				* The next point must be first point whether slip or click after 
				* this up event
				*/ 		
				ts->is_first_point = true;
				update_pen_and_key_state(ts, x, y, z);
			}
		}

	}

	if (ts->use_irq) {
		enable_irq(ts->client->irq);
		SYNAPITICS_DEBUG("enable irq\n");
	}
}
static enum hrtimer_restart synaptics_ts_timer_func(struct hrtimer *timer)
{
	struct synaptics_ts_data *ts = container_of(timer, struct synaptics_ts_data, timer);
	SYNAPITICS_DEBUG("synaptics_ts_timer_func\n");
	queue_work(synaptics_wq, &ts->work);
	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static irqreturn_t synaptics_ts_irq_handler(int irq, void *dev_id)
{
	struct synaptics_ts_data *ts = dev_id;
	disable_irq(ts->client->irq);
	SYNAPITICS_DEBUG("synaptics_ts_irq_handler,disable irq\n");
	queue_work(synaptics_wq, &ts->work);
	return IRQ_HANDLED;
}

static int synaptics_ts_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	struct synaptics_ts_data *ts;
	struct vreg *v_gp5;
	int ret = 0;
	int gpio_config;
	int i;
	struct synaptics_i2c_rmi_platform_data *pdata;
	
	SYNAPITICS_DEBUG(" In synaptics_ts_probe: \n");
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		SYNAPITICS_DEBUG(KERN_ERR "synaptics_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}
	
	/* power on touchscreen */
    v_gp5 = vreg_get(NULL,"gp5");
    ret = IS_ERR(v_gp5);
    if(ret) 
        goto err_power_on_failed;
    ret = vreg_set_level(v_gp5,2800);
    if (ret)
        goto err_power_on_failed;
    ret = vreg_enable(v_gp5);
    if (ret)
        goto err_power_on_failed;   
    mdelay(250);

/* driver  detect its device  */  
	for(i = 0; i < 3; i++) {
		
		ret = i2c_smbus_read_byte_data(client, 0x6c);
		if (ret == 1){
			SYNAPITICS_DEBUG("synaptics_ts manufacturer id = %d\n", ret); 
			goto succeed_find_device;
		}
	}
	if ( i == 3) {
	
		SYNAPITICS_DEBUG("no synaptics_ts device\n ");	
		goto err_find_touchpanel_failed;
	}

succeed_find_device:
	synaptics_wq = create_singlethread_workqueue("synaptics_wq");
	if (!synaptics_wq) {
		SYNAPITICS_DEBUG("create synaptics_wq error\n");
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	ts->client = client;
	i2c_set_clientdata(client, ts);
	INIT_WORK(&ts->work, synaptics_ts_work_func);

	pdata = client->dev.platform_data;

	ts->power = synaptics_ts_power;
	if (ts->power) {
		ret = ts->power(ts->client, 1);
		if (ret < 0) {
			SYNAPITICS_DEBUG(KERN_ERR "synaptics_ts_probe reset failed\n");
			goto err_power_failed;
		}
		msleep(200);
	}
	
#ifdef CONFIG_HUAWEI_TOUCHSCREEN_EXTRA_KEY
	ts->key_input = input_allocate_device();
	if (!ts->key_input  || !ts) {
		ret = -ENOMEM;
		goto err_input_dev_alloc_failed;
	}
#endif
	
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		SYNAPITICS_DEBUG(KERN_ERR "synaptics_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = "synaptics-rmi-touchscreen";
    ts->is_first_point = true;
	
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(BTN_2, ts->input_dev->keybit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	input_set_abs_params(ts->input_dev, ABS_X, 0, TS_X_MAX, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0, (TS_Y_MAX-300), 0, 0);

	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_TOOL_WIDTH, 0, 15, 0, 0);
	ret = input_register_device(ts->input_dev);
	if (ret) {
		SYNAPITICS_DEBUG(KERN_ERR "synaptics_ts_probe: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}
	
#ifdef CONFIG_HUAWEI_TOUCHSCREEN_EXTRA_KEY
	{
		int i;
		
		set_bit(EV_KEY, ts->key_input->evbit);
		for (i = 0; i < EXTRA_MAX_TOUCH_KEY; i++)
		{
			set_bit(touch_extra_key_region.extra_key[i].touch_keycode & KEY_MAX, ts->key_input->keybit);
		}

		ret = input_register_device(ts->key_input);
		if (ret)
			goto err_input_register_device_failed;

		setup_timer(&ts->key_timer, ts_key_timer, (unsigned long)ts);
	}
#endif

	
	gpio_config = GPIO_CFG(29, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA);
	ret = gpio_tlmm_config(gpio_config, GPIO_ENABLE);
	SYNAPITICS_DEBUG(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n", __func__, 29, ret);
	if (ret) 
	{
		ret = -EIO;
		goto err_input_register_device_failed;
	}
	if (gpio_request(29, "synaptics_ts_int\n"))
		pr_err("failed to request gpio synaptics_ts_int\n");
	
	ret = gpio_configure(29, GPIOF_INPUT | IRQF_TRIGGER_LOW);/*gpio 29is interupt for touchscreen.*/
	if (ret) {
		SYNAPITICS_DEBUG(KERN_ERR "synaptics_ts_probe: gpio_configure 29 failed\n");
		goto err_input_register_device_failed;
	}


	if (client->irq) {
		ret = request_irq(client->irq, synaptics_ts_irq_handler, 0, client->name, ts);
		if (ret == 0) {
			ret = i2c_smbus_write_byte_data(ts->client, 0x26, 0x07); /* enable  int */
			if (ret) {
				free_irq(client->irq, ts);
				SYNAPITICS_DEBUG("synaptics_ts_probe: enable abs int failed");
			}			
			else
				SYNAPITICS_DEBUG("synaptics_ts_probe: enable abs int succeed!");
		}
		if (ret == 0)
			ts->use_irq = 1;
		else
			dev_err(&client->dev, "synaptics_ts_probe: request_irq failed\n");
	}
	if (!ts->use_irq) {
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = synaptics_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = synaptics_ts_early_suspend;
	ts->early_suspend.resume = synaptics_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	SYNAPITICS_DEBUG(KERN_INFO "synaptics_ts_probe: Start touchscreen %s in %s mode\n", ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");

	return 0;

err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
err_power_failed:
	kfree(ts);
err_alloc_data_failed:
	
err_find_touchpanel_failed:
err_power_on_failed:
    (void)vreg_disable(v_gp5);

err_check_functionality_failed:
	return ret;
}

static int synaptics_ts_power(struct i2c_client *client, int on)
{
	int ret;
	if (on) {		
		ret = i2c_smbus_write_byte_data(client, 0x25, 0x80);/*sensor on*/	
		if (ret < 0)
			SYNAPITICS_DEBUG(KERN_ERR "synaptics sensor can not wake up\n");
//        ret = i2c_smbus_write_byte_data(client, 0x61, 0x01);/*touchscreen reset*/
//		if (ret < 0)
//			SYNAPITICS_DEBUG(KERN_ERR "synaptics chip can not reset\n");	
	}

	else {
		ret = i2c_smbus_write_byte_data(client, 0x25, 0x81); /* set touchscreen to deep sleep mode*/
		if (ret < 0)
			SYNAPITICS_DEBUG(KERN_ERR "synaptics touch can not enter very-low power state\n");
	}
	return ret;	
}

static int synaptics_ts_remove(struct i2c_client *client)
{
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	unregister_early_suspend(&ts->early_suspend);
	if (ts->use_irq)
		free_irq(client->irq, ts);
	else
		hrtimer_cancel(&ts->timer);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

static int synaptics_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	SYNAPITICS_DEBUG("In synaptics_ts_suspend\n");
	if (ts->use_irq)
		disable_irq(client->irq);
	else
		hrtimer_cancel(&ts->timer);
	ret = cancel_work_sync(&ts->work);
	if (ret && ts->use_irq) /* if work was pending disable-count is now 2 */
		enable_irq(client->irq);
	ret = i2c_smbus_write_byte_data(ts->client, 0x26, 0); /* disable interrupt */
	if (ret < 0)
		SYNAPITICS_DEBUG(KERN_ERR "synaptics_ts_suspend disable interrupt failed\n");
	if (ts->power) {
		ret = ts->power(client,0);
		if (ret < 0)
			SYNAPITICS_DEBUG(KERN_ERR "synaptics_ts_resume power off failed\n");
	}
	return 0;
}

static int synaptics_ts_resume(struct i2c_client *client)
{
	int ret;
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);

	SYNAPITICS_DEBUG("In synaptics_ts_resume\n");
	if (ts->power) {
		ret = ts->power(client, 1);
		if (ret < 0)
			SYNAPITICS_DEBUG(KERN_ERR "synaptics_ts_resume power on failed\n");
//			return -1;
	}

	msleep(200);  /* wait for device reset; */
	
	if (ts->use_irq) {
		enable_irq(client->irq);
		ret =i2c_smbus_write_byte_data(ts->client, 0x26, 0x07); /* enable abs int */
		{
			if (ret < 0)
				SYNAPITICS_DEBUG("enable asb interrupt failed\n");		
				return -1;	
		}
	}

	else
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_ts_early_suspend(struct early_suspend *h)
{
	struct synaptics_ts_data *ts;
	ts = container_of(h, struct synaptics_ts_data, early_suspend);
	synaptics_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void synaptics_ts_late_resume(struct early_suspend *h)
{
	struct synaptics_ts_data *ts;
	ts = container_of(h, struct synaptics_ts_data, early_suspend);
	synaptics_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id synaptics_ts_id[] = {
	{ "synaptics-tm1458", 0 },
	{ }
};

static struct i2c_driver synaptics_ts_driver = {
	.probe		= synaptics_ts_probe,
	.remove		= synaptics_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= synaptics_ts_suspend,
	.resume		= synaptics_ts_resume,
#endif
	.id_table	= synaptics_ts_id,
	.driver = {
		.name	= "synaptics-tm1458",
	},
};

static int __devinit synaptics_ts_init(void)
{
	return i2c_add_driver(&synaptics_ts_driver);
}

static void __exit synaptics_ts_exit(void)
{
	i2c_del_driver(&synaptics_ts_driver);
	if (synaptics_wq)
		destroy_workqueue(synaptics_wq);
}

module_init(synaptics_ts_init);
module_exit(synaptics_ts_exit);

MODULE_DESCRIPTION("Synaptics Touchscreen Driver");
MODULE_LICENSE("GPL");

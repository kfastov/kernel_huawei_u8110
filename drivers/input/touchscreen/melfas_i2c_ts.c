/* linux/drivers/input/touchscreen/melfas_i2c_ts.c
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
#include <mach/mpp.h>
#include <mach/gpio.h>
#include <mach/vreg.h>

#include <linux/hardware_self_adapt.h>


//#define TS_DEBUG
#ifdef TS_DEBUG
#define MELFAS_DEBUG(fmt, args...) printk(KERN_ERR fmt, ##args)
#else
#define MELFAS_DEBUG(fmt, args...)
#endif

#define TS_X_OFFSET		1
#define TS_Y_OFFSET		TS_X_OFFSET

#define TS_INT_GPIO		29
#define TS_RESET_GPIO	96

#define MELFAS_I2C_NAME "melfas-ts"

enum
{
   INPUT_INFO_NONE     = 0,
   INPUT_INFO_TOUCH_UP = 1,
   INPUT_INFO_KEY_UP   = 2,
};

enum
{
   TOUCH_KEY_INDEX0     = 0,
   TOUCH_KEY_INDEX1     = 1,
   TOUCH_KEY_INDEX2     = 2,
   TOUCH_KEY_INDEX3     = 3,
   TOUCH_KEY_INDEX_NONE = 4
};

struct melfas_ts_data {
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
    struct input_dev *key_input;
    struct workqueue_struct *melfas_wq;
	struct work_struct  work;
	int use_irq;
	struct hrtimer timer;	
	int (*power)(struct i2c_client* client, int on);
	struct early_suspend early_suspend;
	
    bool is_first_point;
    bool use_touch_key;
    uint16_t last_x; 
	uint16_t last_y;
	uint8_t key_index_save;
	unsigned int x_max;
	unsigned int y_max;
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void melfas_ts_early_suspend(struct early_suspend *h);
static void melfas_ts_late_resume(struct early_suspend *h);
#endif

static int melfas_ts_power(struct i2c_client *client, int on);

static uint8_t touch_key_value[] = {KEY_BACK, KEY_MENU, KEY_HOME, KEY_SEARCH, KEY_RESERVED};

static void melfas_ts_work_func(struct work_struct *work)
{
	int i;
	int ret;
	int bad_data = 0;	
	struct i2c_msg msg[2];
	uint8_t start_reg;
	uint8_t input_info = 0;
	uint8_t buf[9];
	uint8_t key_info = 0;
	uint8_t key_index = 0;
	uint8_t key_pressed = 0;
	uint16_t position[2][2];
	uint8_t w = 0;

	struct melfas_ts_data *ts = container_of(work, struct melfas_ts_data, work);

	start_reg = 0x10;
	msg[0].addr = ts->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &start_reg;
	
	msg[1].addr = ts->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = sizeof(buf);
	msg[1].buf = buf;

	for (i = 0; i < ((ts->use_irq && !bad_data)? 1 : 5 ); i++)
	{
		ret = i2c_transfer(ts->client->adapter, msg, 2);
		if (ret < 0) 
		{
			MELFAS_DEBUG("%d times i2c_transfer failed\n",i);
			bad_data = 1;
			continue;
		}
		else
		{
		    bad_data = 0;
		}
    	if (i == 5) 
		{
			pr_err("five times i2c_transfer error\n");
			break;
		}

#ifdef TS_DEBUG
        /*printf debug info*/
        for(int k=0; k < sizeof(buf); k++)
        {
            MELFAS_DEBUG("%s:register[0x%x]= 0x%x \n",__FUNCTION__, 0x10+k, buf[k]);
        }
#endif

	    input_info = buf[0] & 0x07;
		position[0][0] = buf[2] | (uint16_t)(buf[1] & 0x03) << 8;
		position[0][1] = buf[4] | (uint16_t)(buf[3] & 0x03) << 8;
		key_info = buf[8] & 0x3;
		key_index = (buf[8] & 0x0C) >> 2;
		key_pressed = (buf[0] & 0xC0) >> 6;
		w = buf[6];
		
        MELFAS_DEBUG("input_type = %d touch_key_status = %d \n",input_info, key_info);
		
		if (input_info == 1) //single point touch
	    {     
	        MELFAS_DEBUG("Touch_Area: X = %d Y = %d \n",position[0][0],position[0][1]);
	        
			if (ts->is_first_point) 
			{
				input_report_abs(ts->input_dev, ABS_X, position[0][0]);
				input_report_abs(ts->input_dev, ABS_Y, position[0][1]);
				//input_report_abs(ts->input_dev, ABS_PRESSURE, z);
				input_report_abs(ts->input_dev, ABS_TOOL_WIDTH, w);
				input_report_key(ts->input_dev, BTN_TOUCH, 1);
				input_sync(ts->input_dev);
				ts->last_x = position[0][0];
				ts->last_y = position[0][1];
				ts->is_first_point = false;
			}
			else 
			{
				 if (((position[0][0]-ts->last_x) >= TS_X_OFFSET) 
					     || ((ts->last_x-position[0][0]) >= TS_X_OFFSET) 			
					     || ((position[0][1]-ts->last_y) >= TS_Y_OFFSET) 
					     || ((ts->last_y-position[0][1]) >= TS_Y_OFFSET)) 
				 {
					input_report_abs(ts->input_dev, ABS_X, position[0][0]);
					input_report_abs(ts->input_dev, ABS_Y, position[0][1]);	
					//input_report_abs(ts->input_dev, ABS_PRESSURE, z);
   				    input_report_abs(ts->input_dev, ABS_TOOL_WIDTH, w);
					input_report_key(ts->input_dev, BTN_TOUCH, 1);
					input_sync(ts->input_dev);
					ts->last_x = position[0][0];
					ts->last_y = position[0][1];
				}
            }
        }
        else if (input_info == 0)
        {
            if(!ts->use_touch_key)
            {
                MELFAS_DEBUG("Touch_Area: touch release!! \n");
                ts->is_first_point = true;
                //input_report_abs(ts->input_dev, ABS_PRESSURE, z);
        	    input_report_abs(ts->input_dev, ABS_TOOL_WIDTH, w);
        	    input_report_key(ts->input_dev, BTN_TOUCH, 0);	
        	    input_sync(ts->input_dev);
            }
            else
            {
                if(key_info != 0)
                {
                    if(ts->key_index_save != key_index)
                    {
                        ts->key_index_save = key_index;
                        MELFAS_DEBUG("Touch_key_Area: touch_key_value[%d]= %d , pressed = %d\n",key_index,touch_key_value[key_index],key_info);
                        input_report_key(ts->key_input, touch_key_value[key_index], key_info);	
                        input_sync(ts->key_input);
                    }
                }
                else
                {
                    if(key_pressed == INPUT_INFO_KEY_UP)
                    {
                        if(ts->key_index_save < TOUCH_KEY_INDEX_NONE)
                        {
                            MELFAS_DEBUG("Touch_key release, touch_key_value[%d]= %d, released = %d \n",ts->key_index_save,touch_key_value[ts->key_index_save], key_info);
                	        input_report_key(ts->key_input, touch_key_value[ts->key_index_save], key_info);	
                            input_sync(ts->key_input);
                        }
                    }
                    else
                    {
                        MELFAS_DEBUG("Touch_Area: touch release!! \n");
                        ts->is_first_point = true;
                        //input_report_abs(ts->input_dev, ABS_PRESSURE, z);
                	    input_report_abs(ts->input_dev, ABS_TOOL_WIDTH, w);
                	    input_report_key(ts->input_dev, BTN_TOUCH, 0);	
                	    input_sync(ts->input_dev);

                	    if(ts->key_index_save < TOUCH_KEY_INDEX_NONE)
                	    {
                	        MELFAS_DEBUG("Touch_key release, touch_key_value[%d]= %d, released = %d \n",ts->key_index_save,touch_key_value[ts->key_index_save], key_info);
                	        input_report_key(ts->key_input, touch_key_value[ts->key_index_save], key_info);	
                            input_sync(ts->key_input);
                	    }
                    }

                    ts->key_index_save = TOUCH_KEY_INDEX_NONE;
                }
            }
        }
	}

	if (ts->use_irq)
	{
	    enable_irq(ts->client->irq);
	    MELFAS_DEBUG("melfas_ts_work_func,enable_irq\n");
	}
}

static enum hrtimer_restart melfas_ts_timer_func(struct hrtimer *timer)
{
	struct melfas_ts_data *ts = container_of(timer, struct melfas_ts_data, timer);
	MELFAS_DEBUG("melfas_ts_timer_func\n");
	queue_work(ts->melfas_wq, &ts->work);
	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static irqreturn_t melfas_ts_irq_handler(int irq, void *dev_id)
{
	struct melfas_ts_data *ts = dev_id;
	disable_irq(ts->client->irq);
 	MELFAS_DEBUG("melfas_ts_irq_handler: disable irq\n");
	queue_work(ts->melfas_wq, &ts->work);
	return IRQ_HANDLED;
}

static int melfas_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct melfas_ts_data *ts;
	int ret = 0;
	int gpio_config;
	int i;
	struct vreg *v_gp5;
  
	MELFAS_DEBUG(" In melfas_ts_probe: \n");
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		pr_err(KERN_ERR "melfas_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}
	
	/* power on touchscreen */
    v_gp5 = vreg_get(NULL,"gp5");
    ret = IS_ERR(v_gp5);
    if(ret) 
        return ret;

    ret = vreg_set_level(v_gp5,2800);
    if (ret)
        return ret;
    ret = vreg_enable(v_gp5);
    if (ret)
    {
        goto err_power_on_failed; 
    }

	ret = gpio_tlmm_config(GPIO_CFG(TS_RESET_GPIO, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA), GPIO_ENABLE);

	ret = gpio_direction_output(TS_RESET_GPIO, 0);
	
	mdelay(50);
	
	ret = gpio_direction_output(TS_RESET_GPIO, 1);
	mdelay(300);
	
	/* driver  detect its device  */  
	for(i = 0; i < 3; i++) 
	{		
		ret = i2c_smbus_read_byte_data(client, 0x00);
		MELFAS_DEBUG("id:%d\n",ret);
		if (ret < 0)
			continue;
		else
			goto  succeed_find_device;
	}
	if (i == 3) 
	{	
		pr_err("%s:check %d times, but dont find melfas_ts device\n",__FUNCTION__, i);	
		goto err_find_touchpanel_failed;
	}

succeed_find_device:
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) 
	{
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	ts->client = client;
	i2c_set_clientdata(client, ts);

	ts->power = melfas_ts_power;
	if (ts->power) 
	{
		ret = ts->power(ts->client, 1);
		if (ret < 0) 
		{
			pr_err("melfas_ts_probe: reset failed\n");
			goto err_power_failed;
		}
	}

	ts->melfas_wq = create_singlethread_workqueue("melfas_wq");
	if (!ts->melfas_wq) 
	{
		pr_err("%s:create melfas_wq error\n",__FUNCTION__);
		ret = -ENOMEM;
		goto err_destroy_wq;
	}
	INIT_WORK(&ts->work, melfas_ts_work_func);

    ts->is_first_point = true;
    ts->use_touch_key = false;
    ts->key_index_save = TOUCH_KEY_INDEX_NONE;
    if(board_use_tssc_touch(&ts->use_touch_key))
    {
        printk(KERN_ERR "%s: Cannot support melfas touch_keypad!\n", __FUNCTION__);
        ret = -ENODEV;
        goto err_destroy_wq;
    }
	if(machine_is_msm7x25_c8600())
	{
		ts->x_max = 320;
		ts->y_max = 480;
	}
	else
	{
		ts->x_max = 240;
		ts->y_max = 320;
	}
    	
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		pr_err("melfas_ts_probe: Failed to allocate touch input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = "melfas-touchscreen";
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	input_set_abs_params(ts->input_dev, ABS_X, 0, ts->x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0, ts->y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_TOOL_WIDTH, 0, 15, 0, 0);

	ret = input_register_device(ts->input_dev);
	if (ret) 
	{
		pr_err("melfas_ts_probe: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}   

    if(ts->use_touch_key)
    {
    	ts->key_input = input_allocate_device();
    	if (!ts->key_input  || !ts) {
    		ret = -ENOMEM;
    		pr_err("melfas_ts_probe: Failed to allocate key input device\n");
    		goto err_key_input_dev_alloc_failed;
    	}

	    ts->key_input->name = "melfas-touch-keypad";
		set_bit(EV_KEY, ts->key_input->evbit);
		for (i = 0; i < sizeof(touch_key_value); i++)
		{
			set_bit(touch_key_value[i] & KEY_MAX, ts->key_input->keybit);
		}

		ret = input_register_device(ts->key_input);
		if (ret)
			goto err_key_input_register_device_failed;
	}
  
	gpio_config = GPIO_CFG(TS_INT_GPIO, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA);
	ret = gpio_tlmm_config(gpio_config, GPIO_ENABLE);
	if (ret < 0) 
	{
	    pr_err("%s: gpio_tlmm_config(%#x)=%d\n", __func__, TS_INT_GPIO, ret);
		ret = -EIO;
		goto err_key_input_register_device_failed; 
	}
	
	if (gpio_request(TS_INT_GPIO, "melfas_ts_int\n"))
		pr_err("failed to request gpio melfas_ts_int\n");
	
	ret = gpio_configure(TS_INT_GPIO, GPIOF_INPUT | IRQF_TRIGGER_FALLING);/*gpio 29 is interupt for touchscreen.*/
	if (ret) 
	{
		pr_err("melfas_ts_probe: gpio_configure %d irq failed\n", TS_INT_GPIO);
		goto err_key_input_register_device_failed;
	}

	if (client->irq) 
	{
		ret = request_irq(client->irq, melfas_ts_irq_handler, 0, client->name, ts);		
		if (ret == 0)
			ts->use_irq = 1;
		else
			dev_err(&client->dev, "melfas_ts_probe: request_irq failed\n");
	}
	if (!ts->use_irq) 
	{
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = melfas_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = melfas_ts_early_suspend;
	ts->early_suspend.resume = melfas_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	printk(KERN_INFO "melfas_ts_probe: Start touchscreen %s in %s mode\n", ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");

	return 0;

err_key_input_register_device_failed:
    if(ts->use_touch_key)
    {
	    input_free_device(ts->key_input);
	}
err_key_input_dev_alloc_failed:
err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
err_destroy_wq:
	destroy_workqueue(ts->melfas_wq);
	
err_power_failed:
	kfree(ts);
err_alloc_data_failed:
err_find_touchpanel_failed:
err_power_on_failed:
    (void)vreg_disable(v_gp5);
err_check_functionality_failed:
	return ret;
}

static int melfas_ts_power(struct i2c_client *client, int on)
{
    int ret = 0;
    MELFAS_DEBUG("melfas_ts_power on = %d\n", on);

    ret = gpio_tlmm_config(GPIO_CFG(TS_RESET_GPIO, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA), GPIO_ENABLE);
    if(ret < 0)
    {
        pr_err("%s: fail to config TS_RESET_GPIO(#%d)\n", __func__,TS_RESET_GPIO);
    }

    if (on) 
    {			  
    	ret = gpio_direction_output(TS_RESET_GPIO, 1);
    	if (ret) 
    	{
    		pr_err("%s: Failed to configure power on = (%d)\n",__func__, ret);
    	} 
    }
    else 
    {
    	ret = gpio_direction_output(TS_RESET_GPIO, 0);
    	if (ret) 
    	{
    		pr_err("%s: Failed to configure power off = (%d)\n",__func__, ret);
    	}       	
    }	

	return ret;	
}

static int melfas_ts_remove(struct i2c_client *client)
{
	struct melfas_ts_data *ts = i2c_get_clientdata(client);
	unregister_early_suspend(&ts->early_suspend);
	if (ts->use_irq)
		free_irq(client->irq, ts);
	else
		hrtimer_cancel(&ts->timer);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}

static int melfas_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct melfas_ts_data *ts = i2c_get_clientdata(client);
	MELFAS_DEBUG("In melfas_ts_suspend\n");
	
	if (ts->use_irq)
		disable_irq(client->irq);
	else
		hrtimer_cancel(&ts->timer);
	ret = cancel_work_sync(&ts->work);
	if (ret && ts->use_irq) /* if work was pending disable-count is now 2 */
	{
		enable_irq(client->irq);
	}
	ret = melfas_ts_power(client,0);	
	if (ret < 0) {
        pr_err("melfas_ts_suspend: power off failed\n");			
	}
	return 0;
}

static int melfas_ts_resume(struct i2c_client *client)
{
	int ret;
	struct melfas_ts_data *ts = i2c_get_clientdata(client);
	MELFAS_DEBUG("In melfas_ts_resume\n");
	
	ret = melfas_ts_power(client,1);	
	if (ret < 0) 
	{
	    pr_err("melfas_ts_resume: power on failed\n");			
	}

	msleep(200);  /* wait for device reset; */
	
	if (ts->use_irq) 
	{
		enable_irq(client->irq);
	}

	else
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void melfas_ts_early_suspend(struct early_suspend *h)
{
	struct melfas_ts_data *ts;

	MELFAS_DEBUG("melfas_ts_early_suspend\n");
	ts = container_of(h, struct melfas_ts_data, early_suspend);
	melfas_ts_suspend(ts->client, PMSG_SUSPEND);  
}

static void melfas_ts_late_resume(struct early_suspend *h)
{
	struct melfas_ts_data *ts;

	MELFAS_DEBUG("melfas_ts_late_resume\n");
	ts = container_of(h, struct melfas_ts_data, early_suspend);
	melfas_ts_resume(ts->client);	
}
#endif

static const struct i2c_device_id melfas_ts_id[] = {
	{ MELFAS_I2C_NAME, 0 },
	{ }
};

static struct i2c_driver melfas_ts_driver = {
	.probe		= melfas_ts_probe,
	.remove		= melfas_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= melfas_ts_suspend,
	.resume		= melfas_ts_resume,
#endif
	.id_table	= melfas_ts_id,
	.driver = {
		.name	= MELFAS_I2C_NAME,
	},
};

static int __devinit melfas_ts_init(void)
{
	MELFAS_DEBUG(KERN_ERR "melfas_ts_init\n ");
	return i2c_add_driver(&melfas_ts_driver);
}

static void __exit melfas_ts_exit(void)
{
	i2c_del_driver(&melfas_ts_driver);
}

module_init(melfas_ts_init);
module_exit(melfas_ts_exit);

MODULE_DESCRIPTION("Melfas Touchscreen Driver");
MODULE_LICENSE("GPL");


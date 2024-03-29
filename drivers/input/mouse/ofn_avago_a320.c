/* linux/drivers/input/mouse/ofn_avago_a320.c
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
#include <mach/gpio.h>
#include <mach/vreg.h>
#include "linux/hardware_self_adapt.h"

/*
 * DEBUG SWITCH
 *
 */ 

//#define TS_DEBUG 
//#undef TS_DEBUG 

#ifdef TS_DEBUG
#define AVAGO_DEBUG(fmt, args...) printk(KERN_ERR fmt, ##args)
#else
#define AVAGO_DEBUG(fmt, args...)
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define AVAGO_OFN_NAME "avago_OFN"

#define ADBM_A320_POWERUP_RETRIES  		 10

// Configuration Register Individual Bit Field Settings
#define ADBM_A320_MOTION_MOT       		 0x80
#define ADBM_A320_MOTION_PIXRDY    		 0x40
#define ADBM_A320_MOTION_PIXFIRST  		 0x20
#define ADBM_A320_MOTION_OVF       		 0x10
#define ADBM_A320_MOTION_GPIO      		 0x01

// Configuration Register Settings
#define ADBM_A320_SOFTRESET_INIT  		 0x5A
#define ADBM_A320_SELF_TEST     		 0x01
#define ADBM_A320_NO_REST              	 0x00
#define ADBM_A320_REST1              	 0x10
#define ADBM_A320_REST2              	 0x20
#define ADBM_A320_REST3              	 0x30
#define ADBM_A320_RES500CPI          	 0x00
#define ADBM_A320_RES1000CPI          	 0x80
#define ADBM_A320_LED0               	 0x01     
#define ADBM_A320_LED3             	 	 0x08     //0:13mA; 1:20mA
#define ADBM_A320_BURST               	 0x10
#define ADBM_A320_SPI               	 0x04
#define ADBM_A320_TWI              	 	 0x01
#define ADBM_A320_RUN_MODE          	 0x00
#define ADBM_A320_REST1_MODE          	 0x40
#define ADBM_A320_REST2_MODE          	 0x80
#define ADBM_A320_REST3_MODE          	 0xC0

// ADBM_A320 Register Addresses
#define ADBM_A320_PRODUCTID_ADDR         0x00     //0x83
#define ADBM_A320_REVISIONID_ADDR        0x01     //0x00
#define ADBM_A320_MOTION_ADDR            0x02
#define ADBM_A320_DELTAX_ADDR            0x03
#define ADBM_A320_DELTAY_ADDR            0x04
#define ADBM_A320_SQUAL_ADDR             0x05
#define ADBM_A320_SHUTTERUPPER_ADDR      0x06
#define ADBM_A320_SHUTTERLOWER_ADDR      0x07
#define ADBM_A320_MAXIMUMPIXEL_ADDR      0x08
#define ADBM_A320_PIXELSUM_ADDR          0x09
#define ADBM_A320_MINIMUMPIXEL_ADDR      0x0A
#define ADBM_A320_PIXELGRAB_ADDR         0x0B
#define ADBM_A320_CRC0_ADDR              0x0C
#define ADBM_A320_CRC1_ADDR              0x0D
#define ADBM_A320_CRC2_ADDR              0x0E
#define ADBM_A320_CRC3_ADDR              0x0F
#define ADBM_A320_SELFTEST_ADDR          0x10
#define ADBM_A320_CONFIGURATIONBITS_ADDR 0x11
#define ADBM_A320_LED_CONTROL_ADDR       0x1A
#define ADBM_A320_IO_MODE_ADDR           0x1C
#define ADBM_A320_OBSERVATION_ADDR       0x2E
#define ADBM_A320_SOFTRESET_ADDR      	 0x3A     //0x5A
#define ADBM_A320_SHUTTER_MAX_HI_ADDR    0x3B
#define ADBM_A320_SHUTTER_MAX_LO_ADDR    0x3C
#define ADBM_A320_INVERSEREVISIONID_ADDR 0x3E     //0xFF
#define ADBM_A320_INVERSEPRODUCTID_ADDR  0x3F     //0x7C

#define ADBM_A320_OFN_ENGINE_ADDR  				0x60
#define ADBM_A320_OFN_RESOLUTION_ADDR  			0x62
#define ADBM_A320_OFN_SPEED_CONTROL_ADDR  		0x63
#define ADBM_A320_OFN_SPEED_ST12_ADDR  			0x64
#define ADBM_A320_OFN_SPEED_ST21_ADDR  			0x65
#define ADBM_A320_OFN_SPEED_ST23_ADDR  			0x66
#define ADBM_A320_OFN_SPEED_ST32_ADDR  			0x67
#define ADBM_A320_OFN_SPEED_ST34_ADDR  			0x68
#define ADBM_A320_OFN_SPEED_ST43_ADDR  			0x69
#define ADBM_A320_OFN_SPEED_ST45_ADDR  			0x6A
#define ADBM_A320_OFN_SPEED_ST54_ADDR  			0x6B
#define ADBM_A320_OFN_AD_CTRL_ADDR  			0x6D
#define ADBM_A320_OFN_AD_ATH_HIGH_ADDR  		0x6E
#define ADBM_A320_OFN_AD_DTH_HIGH_ADDR  		0x6F
#define ADBM_A320_OFN_AD_ATH_LOW_ADDR  			0x70
#define ADBM_A320_OFN_AD_DTH_LOW_ADDR  			0x71
#define ADBM_A320_OFN_QUANTIZE_CTRL_ADDR  		0x73
#define ADBM_A320_OFN_XYQ_THRESH_ADDR  			0x74
#define ADBM_A320_OFN_FPD_CTRL_ADDR  			0x75
#define ADBM_A320_OFN_ORIENTATION_CTRL_ADDR  	0x77
#define ADBM_A320_OFN_ID                        0x83

#define OFN_GPIO_RESET                          31
#define OFN_GPIO_SHUT_DOWN                      36
#define OFN_GPIO_INT                            37
#define OPTNAV_STABILIZE_DELAY_MS               30
#define OFN_VREG_GP5_LEVEL                      2850
#define OFN_READ_DATA_INTERVAL_MS               30
#define OFN_DEVICE_DETECT_TIMES                  3

static struct workqueue_struct *avago_OFN_wq;

struct avago_OFN_data {
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct work_struct  work;
	int use_irq;
	struct hrtimer timer;	
	int (*power)(struct i2c_client* client, int on);
	struct early_suspend early_suspend;
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void avago_OFN_early_suspend(struct early_suspend *h);
static void avago_OFN_late_resume(struct early_suspend *h);
#endif

static int avago_OFN_power(struct i2c_client *client, int on);

static u8 avago_OFN_powerup(struct i2c_client *client )
{
	int ret;	

	ret = gpio_tlmm_config(GPIO_CFG(OFN_GPIO_SHUT_DOWN, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA), GPIO_ENABLE);
	ret = gpio_direction_output(OFN_GPIO_SHUT_DOWN, 0);

    mdelay(1);

	ret = gpio_tlmm_config(GPIO_CFG(OFN_GPIO_RESET, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA), GPIO_ENABLE);
	AVAGO_DEBUG("avago_OFN_powerup gpio_tlmm_config ret=%d \n",ret); 
    
	ret = gpio_direction_output(OFN_GPIO_RESET, 0);
	mdelay(1);
	ret = gpio_direction_output(OFN_GPIO_RESET, 1);
	mdelay(5);

	ret = i2c_smbus_read_byte_data(client, ADBM_A320_PRODUCTID_ADDR);
	AVAGO_DEBUG("avago_OFN_powerup id =0x%x \n",ret); 

	ret = i2c_smbus_write_byte_data(client, ADBM_A320_SOFTRESET_ADDR, 0x5A); /* enable	int */
	//ret = i2c_smbus_write_byte_data(client, ADBM_A320_OFN_ENGINE_ADDR, 0xE4);
	ret = i2c_smbus_write_byte_data(client, ADBM_A320_OFN_ENGINE_ADDR, 0xF4);  /*enable XY quantization*/
	ret = i2c_smbus_write_byte_data(client, ADBM_A320_OFN_RESOLUTION_ADDR, 0x12); 
	ret = i2c_smbus_write_byte_data(client, ADBM_A320_OFN_SPEED_CONTROL_ADDR, 0x0E); 
	ret = i2c_smbus_write_byte_data(client, ADBM_A320_OFN_SPEED_ST12_ADDR, 0x08); 
	ret = i2c_smbus_write_byte_data(client, ADBM_A320_OFN_SPEED_ST21_ADDR, 0x06); 
	ret = i2c_smbus_write_byte_data(client, ADBM_A320_OFN_SPEED_ST23_ADDR, 0x40); 
	ret = i2c_smbus_write_byte_data(client, ADBM_A320_OFN_SPEED_ST32_ADDR, 0x08); 
	ret = i2c_smbus_write_byte_data(client, ADBM_A320_OFN_SPEED_ST34_ADDR, 0x48); 
	ret = i2c_smbus_write_byte_data(client, ADBM_A320_OFN_SPEED_ST43_ADDR, 0x0A); 
	ret = i2c_smbus_write_byte_data(client, ADBM_A320_OFN_SPEED_ST45_ADDR, 0x50); 
	ret = i2c_smbus_write_byte_data(client, ADBM_A320_OFN_SPEED_ST54_ADDR, 0x48); 
	ret = i2c_smbus_write_byte_data(client, ADBM_A320_OFN_AD_ATH_HIGH_ADDR, 0x34); 
	ret = i2c_smbus_write_byte_data(client, ADBM_A320_OFN_AD_DTH_HIGH_ADDR, 0x3C); 
	ret = i2c_smbus_write_byte_data(client, ADBM_A320_OFN_AD_ATH_LOW_ADDR, 0x18); 
	ret = i2c_smbus_write_byte_data(client, ADBM_A320_OFN_AD_DTH_LOW_ADDR, 0x20);     
	ret = i2c_smbus_write_byte_data(client, ADBM_A320_OFN_FPD_CTRL_ADDR, 0x50); 
	ret = i2c_smbus_write_byte_data(client, ADBM_A320_OFN_QUANTIZE_CTRL_ADDR, 0x99); 
	//ret = i2c_smbus_write_byte_data(client, ADBM_A320_OFN_XYQ_THRESH_ADDR, 0x02); 
	ret = i2c_smbus_write_byte_data(client, ADBM_A320_OFN_XYQ_THRESH_ADDR, 0x07); 
	ret = i2c_smbus_write_byte_data(client, ADBM_A320_OFN_FPD_CTRL_ADDR, 0x50); 
	ret = i2c_smbus_read_byte_data(client, ADBM_A320_INVERSEPRODUCTID_ADDR);
	ret = i2c_smbus_read_byte_data(client, 0);
    
	return ret;  
}

static void avago_OFN_work_func(struct work_struct *work)
{
	int ret;
	s8 deltax = 0;
	s8 deltay = 0;
    s8 x_val = 0;
    s8 y_val = 0;
	int delay = 0;
    
    struct avago_OFN_data *ofn = container_of(work, struct avago_OFN_data, work);
	
    msleep(OPTNAV_STABILIZE_DELAY_MS);
    
	/* Poll the device. While we keep getting new data, poll with */
	/* no additional delay. When we receive no new data,          */
	/* gradually reduce the time between polls. When we have gone */
	/* a significant time with no new data, wait for another irq .  */
    do {
        if(delay)
            msleep(delay);

        /* The keys status reg must be read to unlatch the  */
        /* X, Y values, even though we don't need keys data */
        ret = i2c_smbus_read_byte_data(ofn->client, ADBM_A320_MOTION_ADDR);
        if (ret < 0)
        {
        	AVAGO_DEBUG("avago_OFN_work_func get motion addr failed \n");
            goto on_workf_exit;
        }

        if (!(ret&ADBM_A320_MOTION_MOT))
        {
        	AVAGO_DEBUG("avago_OFN_work_func Motion occurred, data not ready for reading in data registers \n");
            goto on_workf_exit;            
        }
        
        deltay = i2c_smbus_read_byte_data(ofn->client, ADBM_A320_DELTAX_ADDR);
        deltax = i2c_smbus_read_byte_data(ofn->client, ADBM_A320_DELTAY_ADDR);

        if(!deltay && !deltax)
        {
            delay = delay + OFN_READ_DATA_INTERVAL_MS;
        }
        else
        {
            delay = 0;
            x_val = deltay;
            y_val = -deltax;
        	input_report_rel(ofn->input_dev, REL_Y, y_val/4);
        	input_report_rel(ofn->input_dev, REL_X, x_val/4);
        	input_sync(ofn->input_dev);
            AVAGO_DEBUG("avago report rel:  x_val=%d y_val=%d \n", x_val, y_val);       
        	ret = i2c_smbus_write_byte_data(ofn->client, ADBM_A320_MOTION_ADDR, 0xFF); 
        }

		/* when delay reaches > 40ms we have had successive waits */
		/* of 10+20+30+40 = 100ms, as recommended in datasheet    */
    }while(delay <= OFN_READ_DATA_INTERVAL_MS * 4);
    
on_workf_exit:
	enable_irq(ofn->client->irq);
    AVAGO_DEBUG("avago_OFN_irq_handler,enable irq\n");
    return;
}
static enum hrtimer_restart avago_OFN_timer_func(struct hrtimer *timer)
{
	struct avago_OFN_data *ofn = container_of(timer, struct avago_OFN_data, timer);
	AVAGO_DEBUG("avago_OFN_timer_func\n");
	queue_work(avago_OFN_wq, &ofn->work);
	hrtimer_start(&ofn->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static irqreturn_t avago_OFN_irq_handler(int irq, void *dev_id)
{
	struct avago_OFN_data *ofn = dev_id;
	disable_irq(ofn->client->irq);
	AVAGO_DEBUG("avago_OFN_irq_handler,disable irq\n");
	queue_work(avago_OFN_wq, &ofn->work);
	return IRQ_HANDLED;
}

static int avago_OFN_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	struct avago_OFN_data *ofn;
	struct vreg *v_gp5;
	int ret = 0;
	int gpio_config, rc;
	int i;
    bool msm_ofn_support = false;

	AVAGO_DEBUG(KERN_ERR "In avago_OFN_probe: \n");
    board_support_ofn(&msm_ofn_support);
    if(false == msm_ofn_support)
    {
        /*this board don't support OFN, and don't need OFN key */
        return 0;
    }

	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		AVAGO_DEBUG(KERN_ERR "avago_OFN_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}
	
	/* power on ofn */
    v_gp5 = vreg_get(NULL,"gp5");
    ret = IS_ERR(v_gp5);
    if(ret) 
        return ret;

    ret = vreg_set_level(v_gp5, OFN_VREG_GP5_LEVEL);
    if (ret)
        return ret;
    ret = vreg_enable(v_gp5);
    if (ret)
        return ret;
        
    msleep(10);

	avago_OFN_powerup(client);

	/* driver  detect its device  */  
	for(i = 0; i < OFN_DEVICE_DETECT_TIMES; i++) {
		
		AVAGO_DEBUG("avago_OFN_powerup ok \n"); 
		ret = i2c_smbus_read_byte_data(client, 0x0);
		if (ret == ADBM_A320_OFN_ID){
			printk(KERN_ERR "avago_OFN_probe manufacturer success id=0x%x\n", ret); 
			goto succeed_find_device;
		}
	}
	if ( i == OFN_DEVICE_DETECT_TIMES) {

        ret = vreg_disable(v_gp5);
		printk(KERN_ERR "no avago_OFN_probe device,vreg_disable: gp5 = %d \n ", ret);	
		goto err_find_touchpanel_failed;
	}

succeed_find_device:
	avago_OFN_wq = create_singlethread_workqueue("avago_OFN_wq");
	if (!avago_OFN_wq) {
		AVAGO_DEBUG("create avago_OFN_wq error\n");
		return -ENOMEM;
	}
	ofn = kzalloc(sizeof(*ofn), GFP_KERNEL);
	if (ofn == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}
		AVAGO_DEBUG("no avago_OFN_probe   22 \n ");	

	ofn->client = client;
	i2c_set_clientdata(client, ofn);
	INIT_WORK(&ofn->work, avago_OFN_work_func);

	ofn->power = avago_OFN_power;
	if (ofn->power) {
		ret = ofn->power(ofn->client, 1);
		if (ret < 0) {
			AVAGO_DEBUG(KERN_ERR "avago_OFN_probe reset failed\n");
			goto err_power_failed;
		}
	}
    
	ofn->input_dev = input_allocate_device();
	if (ofn->input_dev == NULL) {
		ret = -ENOMEM;
		AVAGO_DEBUG(KERN_ERR "avago_OFN_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ofn->input_dev->name = "avago-OFN";
	
	input_set_capability(ofn->input_dev, EV_KEY, BTN_MOUSE);
    input_set_capability(ofn->input_dev, EV_REL, REL_X);
    input_set_capability(ofn->input_dev, EV_REL, REL_Y);
	
	ret = input_register_device(ofn->input_dev);
	if (ret) {
		AVAGO_DEBUG(KERN_ERR "avago_OFN_probe: Unable to register %s input device\n", ofn->input_dev->name);
		goto err_input_register_device_failed;
	}   
	gpio_config = GPIO_CFG(OFN_GPIO_INT, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_2MA);
	rc = gpio_tlmm_config(gpio_config, GPIO_ENABLE);
	AVAGO_DEBUG(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n", __func__, 37, rc);
	if (rc) 
		return -EIO;
	if (gpio_request(OFN_GPIO_INT, "avago_OFN_int\n"))
		pr_err("failed to request gpio avago_OFN_int\n");
	
	ret = gpio_configure(OFN_GPIO_INT, GPIOF_INPUT | IRQF_TRIGGER_LOW);/*gpio 20is interupt for touchscreen.*/
	if (ret) {
		AVAGO_DEBUG(KERN_ERR "avago_OFN_probe: gpio_configure 20 failed\n");
		goto err_input_register_device_failed;
	}

	if (client->irq) {
		
		AVAGO_DEBUG(KERN_ERR "client->irq   \n");
		ret = request_irq(client->irq, avago_OFN_irq_handler, 0, client->name, ofn);
		if (ret != 0) {
			free_irq(client->irq, ofn);
			AVAGO_DEBUG("avago_OFN_probe: enable abs int failed");
		}
		if (ret == 0)
			ofn->use_irq = 1;
		else
			dev_err(&client->dev, "avago_OFN_probe: request_irq failed\n");
	}
	if (!ofn->use_irq) {
		hrtimer_init(&ofn->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ofn->timer.function = avago_OFN_timer_func;
		hrtimer_start(&ofn->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	ofn->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 1;
	ofn->early_suspend.suspend = avago_OFN_early_suspend;
	ofn->early_suspend.resume = avago_OFN_late_resume;
	register_early_suspend(&ofn->early_suspend);
#endif

	AVAGO_DEBUG(KERN_INFO "avago_OFN_probe: Start touchscreen %s in %s mode\n", ofn->input_dev->name, ofn->use_irq ? "interrupt" : "polling");

	return 0;

err_input_register_device_failed:
	input_free_device(ofn->input_dev);

err_input_dev_alloc_failed:
err_power_failed:
	kfree(ofn);
err_alloc_data_failed:
err_find_touchpanel_failed:
err_check_functionality_failed:
	return ret;
}

static int avago_OFN_power(struct i2c_client *client, int on)
{
    int ret = 0;

    ret = gpio_tlmm_config(GPIO_CFG(OFN_GPIO_SHUT_DOWN, 0, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_2MA), GPIO_ENABLE);
    if(ret < 0)
    {
       ret = -EIO;
       pr_err("failed to config gpio OFN_GPIO_SHUT_DOWN\n");
       goto power_fail;
    }
    
    if(on)
    {
        ret = gpio_direction_output(OFN_GPIO_SHUT_DOWN, 0);
        AVAGO_DEBUG(KERN_ERR "avago_OFN_power disable shutdown\n");

        /*you must read I2c after 100ms */
    }
    else
    {
        ret = gpio_direction_output(OFN_GPIO_SHUT_DOWN, 1);
        AVAGO_DEBUG(KERN_ERR "avago_OFN_power enable shutdown\n");
    }

    return 0;
    
power_fail:
    return ret;
}

static int avago_OFN_remove(struct i2c_client *client)
{
	struct avago_OFN_data *ofn = i2c_get_clientdata(client);
	unregister_early_suspend(&ofn->early_suspend);
	if (ofn->use_irq)
		free_irq(client->irq, ofn);
	else
		hrtimer_cancel(&ofn->timer);
	input_unregister_device(ofn->input_dev);
	kfree(ofn);
	return 0;
}

static int avago_OFN_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret=1;
	struct avago_OFN_data *ofn = i2c_get_clientdata(client);
	AVAGO_DEBUG("In avago_OFN_suspend\n");
	if (ofn->use_irq)
		disable_irq(client->irq);
	else
		hrtimer_cancel(&ofn->timer);
	ret = cancel_work_sync(&ofn->work);
	if (ret && ofn->use_irq) /* if work was pending disable-count is now 2 */
		enable_irq(client->irq);
	
	if (ofn->power) {
		ret = ofn->power(client,0);
		if (ret < 0)
			AVAGO_DEBUG(KERN_ERR "avago_OFN_suspend power off failed\n");
	}
	return ret;
}

static int avago_OFN_resume(struct i2c_client *client)
{
	int ret=1;
	struct avago_OFN_data *ofn = i2c_get_clientdata(client);

	AVAGO_DEBUG("In avago_OFN_resume\n");
	if (ofn->power) {
		ret = ofn->power(client, 1);
		if (ret < 0)
			AVAGO_DEBUG(KERN_ERR "avago_OFN_resume power on failed\n");
	}	

	/*you must read I2c after 100ms */
	msleep(100);
	
	if (ofn->use_irq) {
		enable_irq(client->irq);
	}
	else
		hrtimer_start(&ofn->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void avago_OFN_early_suspend(struct early_suspend *h)
{
	struct avago_OFN_data *ts;
	ts = container_of(h, struct avago_OFN_data, early_suspend);
	avago_OFN_suspend(ts->client, PMSG_SUSPEND);
}

static void avago_OFN_late_resume(struct early_suspend *h)
{
	struct avago_OFN_data *ofn;
	ofn = container_of(h, struct avago_OFN_data, early_suspend);
	avago_OFN_resume(ofn->client);
}
#endif

static const struct i2c_device_id avago_OFN_id[] = {
	{ AVAGO_OFN_NAME, 0 },
	{ }
};

static struct i2c_driver avago_OFN_driver = {
	.probe		= avago_OFN_probe,
	.remove		= avago_OFN_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= avago_OFN_suspend,
	.resume		= avago_OFN_resume,
#endif
	.id_table	= avago_OFN_id,
	.driver = {
		.name	= AVAGO_OFN_NAME,
	},
};

static int __devinit avago_OFN_init(void)
{
	return i2c_add_driver(&avago_OFN_driver);
}

static void __exit avago_OFN_exit(void)
{
	i2c_del_driver(&avago_OFN_driver);
	if (avago_OFN_wq)
		destroy_workqueue(avago_OFN_wq);
}

module_init(avago_OFN_init);
module_exit(avago_OFN_exit);

MODULE_DESCRIPTION("Avago OFN Driver");
MODULE_LICENSE("GPL");

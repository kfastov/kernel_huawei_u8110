/* linux/drivers/video/msm/lcdc_ili9325c_qvga.c
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


#include <linux/delay.h>
#include <mach/gpio.h>
#include "msm_fb.h"
#include "lcdc_huawei_config.h"

struct lcd_ili9325c_state_type{
	boolean disp_initialized;
	boolean display_on;
	boolean disp_powered_up;
};

extern void pwm_set_backlight(int level);

static int lcd_reset_gpio;
static struct lcd_ili9325c_state_type lcd_ili9325c_state = { 0 };
static struct msm_panel_common_pdata *lcdc_ili9325c_pdata;
static lcd_panel_type lcd_panel_qvga = LCD_NONE;


static uint16 ili9325c_wintek_disp_off[][3] = 
{
        {0x0007, 0x0131, 10},
        {0x0007, 0x0130, 10},
        {0x0007, 0x0000, 0},
        /*power off sequence*/
        {0x0010, 0x0080, 0},
        {0x0011, 0x0000, 0},
        {0x0012, 0x0000, 0},
        {0x0013, 0x0000, 200},
        {0x0010, 0x0082, 0},
};

static uint16 ili9325c_wintek_disp_on[][3] = 
{
        /*power on sequence*/
        {0x0010, 0x0080, 0},
        {0x0011, 0x0000, 0},
        {0x0012, 0x0000, 0},
        {0x0013, 0x0000, 50},
        {0x0010, 0x1590, 0},
        {0x0011, 0x0225, 10},
        {0x0011, 0x0227, 25},
        {0x0012, 0x009A, 25},
        {0x0013, 0x1A00, 0},
        {0x0029, 0x002E, 25},
        {0x0007, 0x0133, 0},

};


#define NUM_ILI9325C_WINTEK_DISP_OFF          (sizeof(ili9325c_wintek_disp_off)/(sizeof(uint16) * 3))
#define NUM_ILI9325C_WINTEK_DISP_ON           (sizeof(ili9325c_wintek_disp_on)/(sizeof(uint16) * 3)) 

static void seriout(uint16 reg, uint16 data)
{
    uint8 start_byte_reg = 0x70;
    uint8 start_byte_data = 0x72;
    
    seriout_cmd(reg, start_byte_reg);
    seriout_data(data, start_byte_data);
}

static void lcd_ili9325c_disp_powerup(void)
{
	if (!lcd_ili9325c_state.disp_powered_up && !lcd_ili9325c_state.display_on) 
    {
		/* Reset the hardware first */
		/* Include DAC power up implementation here */
	    lcd_ili9325c_state.disp_powered_up = TRUE;
	}
}

static void lcd_ili9325c_disp_exit_sleep(void)
{
	unsigned char i = 0;

	if (lcd_ili9325c_state.disp_powered_up && !lcd_ili9325c_state.display_on)
    {
        switch (lcd_panel_qvga) 
        {
            case LCD_ILI9325C_WINTEK_QVGA:
                for(i = 0; i < NUM_ILI9325C_WINTEK_DISP_ON; i++)
                {
                    seriout(ili9325c_wintek_disp_on[i][0], ili9325c_wintek_disp_on[i][1]);
                    mdelay(ili9325c_wintek_disp_on[i][2]);
                }
                break;

            default :
                break;
              
        }
        printk(KERN_ERR "lcd_ili9325c_disp_exit_sleep:  LCD should be on, LCD_Panel = %d!\n", lcd_panel_qvga);

        lcd_ili9325c_state.display_on = TRUE;
    }   

}
static int lcdc_ili9325c_panel_on(struct platform_device *pdev)
{

	if (!lcd_ili9325c_state.disp_initialized) 
    {
		/* Configure reset GPIO that drives DAC */
		lcdc_ili9325c_pdata->panel_config_gpio(1);
		lcd_reset_gpio = *(lcdc_ili9325c_pdata->gpio_num + 4);
        
		lcd_spi_init(lcdc_ili9325c_pdata);	/* LCD needs SPI */
		lcd_ili9325c_disp_powerup();
		lcd_ili9325c_state.display_on = TRUE;
		lcd_ili9325c_state.disp_initialized = TRUE;

	}
    else if(!lcd_ili9325c_state.display_on)
    {
        lcd_ili9325c_disp_exit_sleep();
    }
	return 0;
}

static int lcdc_ili9325c_panel_off(struct platform_device *pdev)
{
	unsigned char i = 0;

	if (lcd_ili9325c_state.disp_powered_up && lcd_ili9325c_state.display_on)
    {
        switch (lcd_panel_qvga) 
        {
              
            case LCD_ILI9325C_WINTEK_QVGA:
                for(i = 0; i < NUM_ILI9325C_WINTEK_DISP_OFF; i++)
                {
                    seriout(ili9325c_wintek_disp_off[i][0], ili9325c_wintek_disp_off[i][1]);
                    mdelay(ili9325c_wintek_disp_off[i][2]);
                }
                break;

            default :
                break;
              
        }
		lcd_ili9325c_state.display_on = FALSE;
	}
	return 0;
}

static void lcdc_ili9325c_panel_set_backlight(struct msm_fb_data_type *mfd)
{
    int bl_level = mfd->bl_level;
       
   // lcd_set_backlight_pwm(bl_level);
    pwm_set_backlight(bl_level);
    
    return;
}

static void lcdc_ili9325c_panel_set_contrast(struct msm_fb_data_type *mfd, unsigned int contrast)
{

    return;
}

static int __init lcdc_ili9325c_probe(struct platform_device *pdev)
{
	if (pdev->id == 0) {
		lcdc_ili9325c_pdata = pdev->dev.platform_data;
		return 0;
	}
	msm_fb_add_device(pdev);
	return 0;
}

static struct platform_driver this_driver = 
{
	.probe  = lcdc_ili9325c_probe,
	.driver = {
		.name   = "lcdc_ili9325c_qvga",
	},
};

static struct msm_fb_panel_data ili9325c_panel_data =
{
	.on = lcdc_ili9325c_panel_on,
	.off = lcdc_ili9325c_panel_off,
	.set_backlight = lcdc_ili9325c_panel_set_backlight,
	.set_contrast = lcdc_ili9325c_panel_set_contrast,
};

static struct platform_device this_device = 
{
	.name   = "lcdc_ili9325c_qvga",
	.id	= 1,
	.dev	=
	{
		.platform_data = &ili9325c_panel_data,
	}
};

static int __init lcdc_ili9325c_panel_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;


    lcd_panel_qvga = lcd_panel_probe();
    if((LCD_ILI9325C_WINTEK_QVGA != lcd_panel_qvga) &&  \
       (msm_fb_detect_client("lcdc_ili9325c_qvga"))
      )
    {
        return 0;
    }

	ret = platform_driver_register(&this_driver);
	if (ret)
		return ret;

	pinfo = &ili9325c_panel_data.panel_info;
    pinfo->xres = 240;
	pinfo->yres = 320;
	pinfo->type = LCDC_PANEL;
	pinfo->pdest = DISPLAY_1;
	pinfo->wait_cycle = 0;
	pinfo->bpp = 18;
	pinfo->fb_num = 2;
    /*the pixel clk is different for different Resolution LCD*/
	//pinfo->clk_rate = 24500000; /*for VGA pixel clk*/
    pinfo->clk_rate = 6125000;  /*for QVGA pixel clk*/   
    pinfo->lcdc.h_back_porch = 3;
	pinfo->lcdc.h_front_porch = 5;
	pinfo->lcdc.h_pulse_width = 5;    
	pinfo->lcdc.v_back_porch = 3;
	pinfo->lcdc.v_front_porch = 3;
	pinfo->lcdc.v_pulse_width = 3;
	pinfo->lcdc.border_clr = 0;     /* blk */
	pinfo->lcdc.underflow_clr = 0xff;       /* blue */
	pinfo->lcdc.hsync_skew = 0;
    pinfo->bl_max = 255;

	ret = platform_device_register(&this_device);
	if (ret)
		platform_driver_unregister(&this_driver);

	return ret;
}


module_init(lcdc_ili9325c_panel_init);

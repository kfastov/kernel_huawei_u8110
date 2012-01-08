/* linux/drivers/video/msm/lcdc_hx8347d_qvga.c
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

#define TRACE_LCD_DEBUG 0
#if TRACE_LCD_DEBUG
#define LCD_DEBUG(x...) printk(KERN_ERR "[LCD_DEBUG] " x)
#else
#define LCD_DEBUG(x...) do {} while (0)
#endif

#define lCD_DRIVER_NAME "lcdc_hx8347d_qvga"

#define LCD_MIN_BACKLIGHT_LEVEL 0
#define LCD_MAX_BACKLIGHT_LEVEL	255

#define DEVICE_ID 				0x70 //BS0=1
#define WRITE_REGISTER 			0x00
#define WRITE_CONTENT 			0x02


struct hx8347d_state_type{
	boolean disp_initialized;
	boolean display_on;
	boolean disp_powered_up;
};

extern void pwm_set_backlight(int level);

static int lcd_reset_gpio;
static struct hx8347d_state_type hx8347d_state = { 0 };
static struct msm_panel_common_pdata *lcdc_hx8347d_pdata;
static lcd_panel_type lcd_panel_qvga = LCD_NONE;

struct command{
	uint32 reg;
	uint32 value;
	uint32 time;
};

static const struct command hx8347d_init_table[] =
{
    /* Select Command Page */
    {0xFF, 0x00, 0},
    //Driving ability Setting 
    {0xEA, 0x00, 0}, //PTBA[15:8]
    {0xEB, 0x20, 0}, //PTBA[7:0] 
    {0xEC, 0x0C, 0}, //STBA[15:8]
    {0xED, 0xC4, 0}, //STBA[7:0]
    {0xE8, 0x40, 0}, //OPON[7:0] 
    {0xE9, 0x38, 0}, //OPON1[7:0]
    {0xF1, 0x01, 0}, //OTPS1B
    {0xF2, 0x10, 0}, //GEN    
    {0x27, 0xA3, 0},
    /* Gamma control */
    {0x40, 0x02, 0},
    {0x41, 0x21, 0},
    {0x42, 0x1F, 0},
    {0x43, 0x21, 0},
    {0x44, 0x1F, 0},
    {0x45, 0x3D, 0},
    {0x46, 0x0F, 0},
    {0x47, 0x61, 0},
    {0x48, 0x06, 0},
    {0x49, 0x05, 0},
    {0x4A, 0x06, 0},
    {0x4B, 0x11, 0},
    {0x4C, 0x1E, 0},
    {0x50, 0x02, 0},
    {0x51, 0x20, 0},
    {0x52, 0x1E, 0},
    {0x53, 0x20, 0},
    {0x54, 0x1E, 0},
    {0x55, 0x3D, 0},
    {0x56, 0x1E, 0},
    {0x57, 0x70, 0},
    {0x58, 0x01, 0},
    {0x59, 0x0E, 0},
    {0x5A, 0x19, 0},
    {0x5B, 0x1A, 0},
    {0x5C, 0x19, 0},
    {0x5D, 0x00, 0},
    //Power Voltage Setting 
    {0x1B, 0x1B, 0}, //VRH=4.65V                     
    {0x1A, 0x01, 0}, //BT (VGH~15V,VGL~-10V,DDVDH~5V)
    {0x24, 0x2F, 0}, //VMH(VCOM High voltage ~3.2V)  
    {0x25, 0x57, 0}, //VML(VCOM Low voltage -1.2V)   
    //****VCOM offset**///
    {0x23, 0x86, 0}, //for Flicker adjust //can reload from OTP 
    //Power on Setting 
    {0x18, 0x36, 0}, //I/P_RADJ,N/P_RADJ, Normal mode 75Hz                   
    {0x19, 0x01, 10}, //OSC_EN='1', start Osc                                 
    {0x01, 0x00, 0}, //DP_STB='0', out deep sleep                            
    {0x1F, 0x88, 10}, // GAS=1, VOMG=00, PON=0, DK=1, XDK=0, DVDH_TRI=0, STB=0
    {0x1F, 0x80, 10}, // GAS=1, VOMG=00, PON=0, DK=0, XDK=0, DVDH_TRI=0, STB=0 
    {0x1F, 0x90, 10}, // GAS=1, VOMG=00, PON=1, DK=0, XDK=0, DVDH_TRI=0, STB=0 
    {0x1F, 0xD0, 10}, // GAS=1, VOMG=10, PON=1, DK=0, XDK=0, DVDH_TRI=0, STB=0 
    //262k/65k color selection 
//    {0x17, 0x05, 0}, //default 0x06 262k color // 0x05 65k color 
    {0x17, 0x60, 0}, //RGB interface 18bit
    {0x31, 0x02, 0},
    {0x32, 0x0E, 0},
    {0x33, 0x02, 0},
    {0x34, 0x02, 0},
    //SET PANEL 
    {0x36, 0x00, 0}, //SS_P, GS_P,REV_P,BGR_P 
    //Display ON Setting 
    {0x28, 0x38, 40}, //GON=1, DTE=1, D=1000 
    {0x28, 0x3C, 40},  //GON=1, DTE=1, D=1100
    //Set GRAM Area 
    {0x02, 0x00, 0},
    {0x03, 0x00, 0}, //Column Start 
    {0x04, 0x00, 0},
    {0x05, 0xEF, 0}, //Column End 
    {0x06, 0x00, 0},
    {0x07, 0x00, 0}, //Row Start
    {0x08, 0x01, 0},
    {0x09, 0x3F, 0}, //Row End
};

static const struct command hx8347d_standby_enter_table[] = 
{
    { 0x28, 0xB8, 40},	
    { 0x1F, 0x89, 40},
    { 0x28, 0x04, 40},	
    { 0x21, 0x00, 5},
};

static const struct command hx8347d_standby_exit_table[] = 
{
    { 0x18, 0x36, 0},
    { 0x19, 0x01, 0},
    { 0x1F, 0x88, 5},
    { 0x1F, 0x80, 5},
    { 0x1F, 0x90, 5},
    { 0x1F, 0xD0, 5},
    { 0x28, 0x38, 40},
    { 0x28, 0x3F, 0},
};

static const struct command hx8347d_display_area_table[] = 
{
	/* Select command page */
	{ 0xFF, 0x00, 0},
		
	/*Column address start register*/
	{ 0x02, 0x00, 0},
	{ 0x03, 0x00, 0},
	
	/*Column address end register*/
	{ 0x04, 0x01, 0},
	{ 0x05, 0xDF, 0},
	
	/*Row address start register*/
	{ 0x06, 0x00, 0},
	{ 0x07, 0x00, 0},
	
	/*Row address end register*/
	{ 0x08, 0x01, 0},
	{ 0x09, 0x3F, 0},
};

static void _serigo(uint8 reg, uint8 data)
{
    uint8 start_byte_reg = DEVICE_ID | WRITE_REGISTER;
    uint8 start_byte_data = DEVICE_ID | WRITE_CONTENT;
    
    seriout_transfer_byte(reg, start_byte_reg);
    seriout_transfer_byte(data, start_byte_data);
}

static void process_lcdc_table(struct command *table, size_t count)
{
    int i;
    uint32 reg = 0;
    uint32 value = 0;
    uint32 time = 0;

    for (i = 0; i < count; i++) {
        reg = table[i].reg;
        value = table[i].value;
        time = table[i].time;

        _serigo(reg, value);

        if (time != 0)
        	mdelay(time);
    }
}

static void hx8347d_disp_powerup(void)
{
    if (!hx8347d_state.disp_powered_up && !hx8347d_state.display_on) {
        /* Reset the hardware first */
        /* Include DAC power up implementation here */
        hx8347d_state.disp_powered_up = TRUE;
    }
}

static void hx8347d_disp_on(void)
{
    if (hx8347d_state.disp_powered_up && !hx8347d_state.display_on) 
    {
        LCD_DEBUG("%s: disp on lcd\n", __func__);
        /* Initialize LCD */
        //process_lcdc_table((struct command*)&hx8347d_init_table, ARRAY_SIZE(hx8347d_init_table));
        //seriout_transfer_byte(0x22, DEVICE_ID | WRITE_REGISTER);
        hx8347d_state.display_on = TRUE;
    }
}

static void hx8347d_reset(void)
{
    /* Reset LCD*/
    lcdc_hx8347d_pdata->panel_config_gpio(1);
    lcd_reset_gpio = *(lcdc_hx8347d_pdata->gpio_num + 4);
    
}

static int hx8347d_panel_on(struct platform_device *pdev)
{
    if (!hx8347d_state.disp_initialized) 
    {
        hx8347d_reset();
        lcd_spi_init(lcdc_hx8347d_pdata);	/* LCD needs SPI */
        hx8347d_disp_powerup();
        hx8347d_disp_on();
        hx8347d_state.disp_initialized = TRUE;
        LCD_DEBUG("%s: hx8347d lcd initialized\n", __func__);
    } 
    else if (!hx8347d_state.display_on) 
    {
        /* Exit Standby Mode */
        process_lcdc_table((struct command*)&hx8347d_standby_exit_table, ARRAY_SIZE(hx8347d_standby_exit_table));
        LCD_DEBUG("%s: Exit Standby Mode\n", __func__);
        hx8347d_state.display_on = TRUE;
    }
    
    return 0;
}

static int hx8347d_panel_off(struct platform_device *pdev)
{
    if (hx8347d_state.disp_powered_up && hx8347d_state.display_on) {
        /* Enter Standby Mode */
        process_lcdc_table((struct command*)&hx8347d_standby_enter_table, ARRAY_SIZE(hx8347d_standby_enter_table));
        hx8347d_state.display_on = FALSE;
        LCD_DEBUG("%s: Enter Standby Mode\n", __func__);
    }

    return 0;
}

static void hx8347d_set_backlight(struct msm_fb_data_type *mfd)
{
    int bl_level = mfd->bl_level;
       
   // lcd_set_backlight_pwm(bl_level);
    pwm_set_backlight(bl_level);
     return;
}

static void hx8347d_panel_set_contrast(struct msm_fb_data_type *mfd, unsigned int contrast)
{

    return;
}

static int __init hx8347d_probe(struct platform_device *pdev)
{
    if (pdev->id == 0) {
        lcdc_hx8347d_pdata = pdev->dev.platform_data;
        return 0;
    }
    msm_fb_add_device(pdev);
    return 0;
}

static struct platform_driver this_driver = {
    .probe  = hx8347d_probe,
    .driver = {
    	.name   = lCD_DRIVER_NAME,
    },
};

static struct msm_fb_panel_data hx8347d_panel_data = {
    .on = hx8347d_panel_on,
    .off = hx8347d_panel_off,
    .set_backlight = hx8347d_set_backlight,
    .set_contrast = hx8347d_panel_set_contrast,
};

static struct platform_device this_device = {
    .name   = lCD_DRIVER_NAME,
    .id	= 1,
    .dev	= {
    	.platform_data = &hx8347d_panel_data,
    }
};

static int __init hx8347d_panel_init(void)
{
    int ret;
    struct msm_panel_info *pinfo;

    lcd_panel_qvga = lcd_panel_probe();
    if((LCD_HX8347D_TRULY_QVGA != lcd_panel_qvga) && \
       (msm_fb_detect_client(lCD_DRIVER_NAME))
      )
    {
        return 0;
    }


    ret = platform_driver_register(&this_driver);
    if (ret)
        return ret;

    pinfo = &hx8347d_panel_data.panel_info;
    pinfo->xres = 240;
    pinfo->yres = 320;
    pinfo->type = LCDC_PANEL;
    pinfo->pdest = DISPLAY_1;
    pinfo->wait_cycle = 0;
    pinfo->bpp = 18;
    pinfo->fb_num = 2;
    pinfo->bl_max = LCD_MAX_BACKLIGHT_LEVEL;
    pinfo->bl_min = LCD_MIN_BACKLIGHT_LEVEL;

    pinfo->clk_rate = 6125000;  /*for QVGA pixel clk*/   
    pinfo->lcdc.h_back_porch = 2;
    pinfo->lcdc.h_front_porch = 2;
    pinfo->lcdc.h_pulse_width = 2;
    pinfo->lcdc.v_back_porch = 2;
    pinfo->lcdc.v_front_porch = 2;
    pinfo->lcdc.v_pulse_width = 2;

    pinfo->lcdc.border_clr = 0;     /* blk */
    pinfo->lcdc.underflow_clr = 0xff;       /* blue */
    pinfo->lcdc.hsync_skew = 0;

    ret = platform_device_register(&this_device);
    if (ret)
        platform_driver_unregister(&this_driver);

    return ret;
}

module_init(hx8347d_panel_init);

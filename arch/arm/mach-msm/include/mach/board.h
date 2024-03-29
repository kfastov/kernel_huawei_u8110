/* arch/arm/mach-msm/include/mach/board.h
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ASM_ARCH_MSM_BOARD_H
#define __ASM_ARCH_MSM_BOARD_H

#include <linux/types.h>
#include <linux/input.h>
#include <linux/usb.h>

#include <asm/setup.h>

/* platform device data structures */
struct msm_acpu_clock_platform_data {
	uint32_t acpu_switch_time_us;
	uint32_t max_speed_delta_khz;
	uint32_t vdd_switch_time_us;
	unsigned long power_collapse_khz;
	unsigned long wait_for_irq_khz;
	unsigned int max_axi_khz;
	unsigned int max_vdd;
	int (*acpu_set_vdd) (int mvolts);
};

struct msm_camera_io_ext {
	uint32_t mdcphy;
	uint32_t mdcsz;
	uint32_t appphy;
	uint32_t appsz;
};

struct msm_camera_device_platform_data {
	void (*camera_gpio_on) (void);
	void (*camera_gpio_off)(void);
	struct msm_camera_io_ext ioext;
#ifdef CONFIG_HUAWEI_CAMERA  
    bool (*get_board_support_flash) (void);    
#endif

};

#ifdef CONFIG_SENSORS_MT9T013
struct msm_camera_legacy_device_platform_data {
	int sensor_reset;
	int sensor_pwd;
	int vcm_pwd;
	void (*config_gpio_on) (void);
	void (*config_gpio_off)(void);
};
#endif

#define MSM_CAMERA_FLASH_NONE 0
#define MSM_CAMERA_FLASH_LED  1
#ifdef CONFIG_HUAWEI_CAMERA
struct msm_camera_sensor_vreg {
	const char *vreg_name;
	unsigned int mv;
    uint8_t always_on;
};
#endif //CONFIG_HUAWEI_CAMERA

struct msm_camera_sensor_info {
	const char *sensor_name;
	int sensor_reset;
	int sensor_pwd;
	int vcm_pwd;
	int mclk;
	int flash_type;
	struct msm_camera_device_platform_data *pdata;
	struct resource *resource;
	uint8_t num_resources;
#ifdef CONFIG_HUAWEI_CAMERA
    int sensor_module_id;
    int sensor_module_value;
    struct msm_camera_sensor_vreg *sensor_vreg;
    uint8_t vreg_num;
    int32_t (*vreg_enable_func) (struct msm_camera_sensor_vreg*,uint8_t);
	int32_t (*vreg_disable_func)(struct msm_camera_sensor_vreg*,uint8_t);
    uint8_t slave_sensor;
    int32_t (*slave_elude_func) (struct msm_camera_sensor_info*, struct msm_camera_sensor_info*);
    int32_t (*master_init_control_slave) (const struct msm_camera_sensor_info*);
#endif //CONFIG_HUAWEI_CAMERA
};

struct clk;

struct snd_endpoint {
	int id;
	const char *name;
};

struct msm_snd_endpoints {
	struct snd_endpoint *endpoints;
	unsigned num;
};

/* 7k target ADSP information */
/* Bit 23:0, for codec identification like mp3, wav etc *
 * Bit 27:24, for mode identification like tunnel, non tunnel*
 * bit 31:28, for operation support like DM, DMA */
enum msm_adspdec_concurrency {
	MSM_ADSP_CODEC_WAV = 0,
	MSM_ADSP_CODEC_ADPCM = 1,
	MSM_ADSP_CODEC_MP3 = 2,
	MSM_ADSP_CODEC_REALAUDIO = 3,
	MSM_ADSP_CODEC_WMA = 4,
	MSM_ADSP_CODEC_AAC = 5,
	MSM_ADSP_CODEC_RESERVED = 6,
	MSM_ADSP_CODEC_MIDI = 7,
	MSM_ADSP_CODEC_YADPCM = 8,
	MSM_ADSP_CODEC_QCELP = 9,
	MSM_ADSP_CODEC_AMRNB = 10,
	MSM_ADSP_CODEC_AMRWB = 11,
	MSM_ADSP_CODEC_EVRC = 12,
	MSM_ADSP_CODEC_WMAPRO = 13,
	MSM_ADSP_MODE_TUNNEL = 24,
	MSM_ADSP_MODE_NONTUNNEL = 25,
	MSM_ADSP_OP_DMA = 28,
	MSM_ADSP_OP_DM = 29,
};

struct msm_adspdec_info {
	const char *module_name;
	unsigned module_queueid;
	int module_decid; /* objid */
	unsigned nr_codec_support;
};

struct msm_adspdec_database {
	unsigned num_dec;
	unsigned num_concurrency_support;
	unsigned int *dec_concurrency_table; /* Bit masked entry to *
					      *	represents codec, mode etc */
	struct msm_adspdec_info  *dec_info_list;
};

struct msm_panel_common_pdata {
	int gpio;
	int (*backlight_level)(int level, int max, int min);
	int (*pmic_backlight)(int level);
	int (*panel_num)(void);
	void (*panel_config_gpio)(int);
	int *gpio_num;
};

struct lcdc_platform_data {
	int (*lcdc_gpio_config)(int on);
	void (*lcdc_power_save)(int);
};

struct tvenc_platform_data {
	int (*pm_vid_en)(int on);
};

struct mddi_platform_data {
	void (*mddi_power_save)(int on);
	int (*mddi_sel_clk)(u32 *clk_rate);
	int (*mddi_power_on)(int);
};

struct msm_fb_platform_data {
	int (*detect_client)(const char *name);
	int mddi_prescan;
};

struct msm_i2c_platform_data {
	int clk_freq;
	uint32_t *rmutex;
	int rsl_id;
	uint32_t pm_lat;
	int pri_clk;
	int pri_dat;
	int aux_clk;
	int aux_dat;
	void (*msm_i2c_config_gpio)(int iface, int config_type);
};

/* common init routines for use by arch/arm/mach-msm/board-*.c */

void __init msm_add_devices(void);
void __init msm_map_common_io(void);
void __init msm_map_qsd8x50_io(void);
void __init msm_map_msm7x30_io(void);
void __init msm_map_comet_io(void);
void __init msm_init_irq(void);
void __init msm_clock_init(struct clk *clock_tbl, unsigned num_clocks);
void __init msm_acpu_clock_init(struct msm_acpu_clock_platform_data *);

struct mmc_platform_data;

int __init msm_add_sdcc(unsigned int controller,
			struct mmc_platform_data *plat,
			unsigned int stat_irq,
			unsigned long stat_irq_flags);

struct msm_usb_host_platform_data;
int __init msm_add_host(unsigned int host,
		struct msm_usb_host_platform_data *plat);
#if defined(CONFIG_USB_FUNCTION_MSM_HSUSB) || defined(CONFIG_USB_MSM_72K)
void msm_hsusb_set_vbus_state(int online);
#else
static inline void msm_hsusb_set_vbus_state(int online) {}
#endif

extern int msm_shared_ram_phys; /* defined in arch/arm/mach-msm/io.c */

void __init msm_add_usb_devices(u32 latency);

int __init parse_tag_camera_id(const struct tag *tags);

int __init parse_tag_lcd_id(const struct tag *tags);

int __init parse_tag_ts_id(const struct tag *tags);
int __init parse_tag_sub_board_id(const struct tag *tags);
    
#endif

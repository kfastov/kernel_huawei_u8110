/* linux/drivers/video/msm/lcdc_huawei_config.h
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

#ifndef LCDC_HUAWEI_CONFIG_H
#include "msm_fb.h"
#include <linux/hardware_self_adapt.h>

#define LCDC_HUAWEI_CONFIG_H

#define GP_MD_REG_ADDR_OFFSET              0x0058
#define GP_NS_REG_ADDR_OFFSET              0x005C
#define MSM_GP_MD_REG_VIRT_ADD            (MSM_CLK_CTL_BASE + GP_MD_REG_ADDR_OFFSET)
#define MSM_GP_NS_REG_VIRT_ADD            (MSM_CLK_CTL_BASE + GP_NS_REG_ADDR_OFFSET)

#define PWM_LCD_NOT_N_M_VAL                0xFE4D
#define PWM_LCD_M_VAL                      0x0001

#define GP_ROOT_ENA                        (1 << 11)
#define GP_CLK_BRANCH_ENA                  (1 << 9)
#define GP_MNCNTR_EN                       (1 << 8)
#define GP_NS_REG_SRC_SEL                  (0 << 0)
#define GP_NS_REG_PRE_DIV_SEL              (0 << 3)
#define GP_NS_REG_MNCNTR_MODE              (3 << 5)
#define GP_NS_REG_GP_N_VAL                 (PWM_LCD_NOT_N_M_VAL << 16)
#define GP_MD_REG_M_VAL                    (PWM_LCD_M_VAL << 16)


void lcd_spi_init(struct msm_panel_common_pdata *lcdc_pnael_data);
void seriout_transfer_byte(uint8 reg, uint8 start_byte);
void seriout_cmd(uint16 reg, uint8 start_byte);
void seriout_data(uint16 data, uint8 start_byte);
void lcd_set_backlight_pwm(int level);

#endif

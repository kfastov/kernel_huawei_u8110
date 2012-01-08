/* linux/include/asm-arm/huawei/smem_vendor_huawei.h
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

#ifndef _SMEM_VENDOR_HUAWEI_H_
#define _SMEM_VENDOR_HUAWEI_H_

#define VENDOR_PROP_CMDLINE_ID  " androidboot.localproppath="

#define APP_USB_SERIAL_LEN   16

#define VENDOR_NAME_LEN      32

typedef struct _app_usb_para_smem
{
  /* Stores usb serial number for apps */
  unsigned char usb_serial[APP_USB_SERIAL_LEN];
  unsigned usb_pid_index;
} app_usb_para_smem;

typedef struct _app_verder_name
{
  unsigned char vender_name[VENDOR_NAME_LEN];
  unsigned char country_name[VENDOR_NAME_LEN];
}app_vender_name;

typedef struct
{
  app_usb_para_smem      usb_para;
  app_vender_name   vender_para;
} smem_huawei_vender;

#endif //_SMEM_VENDOR_HUAWEI_H_


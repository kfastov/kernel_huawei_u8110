/* linux/drivers/usb/function/usb_switch_huawei.h
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

#ifndef __USB_SWITCH_HUAWEI_H__
#define __USB_SWITCH_HUAWEI_H__

#include <asm-arm/huawei/smem_vendor_huawei.h>

/* enable the autorun feature
 * this feature is defined in hw_msm7x25_defconfig and can be set in Kconfig
 */
//#define CONFIG_USB_AUTO_INSTALL

/* enable printing information about USB switching */
#define USB_AUTO_DEBUG

/* CDROM iso file location */
#define CDROM_PATH_FILE                 "/data/cdrom/autorun.iso"

#define UDISK_PATH_FILE                 "/dev/block/vold/179:0"

/* less than 14 characters */
#define USB_SN_STRING                   "hw-smart-phone"

#define CHG_SUSP_WORK_DELAY	            msecs_to_jiffies(100)

/*****USB PID for TMO product*****/
/* TMO CDROM PID */
#define PID_ONLY_CDROM_TMO              0x1030
/* TMO authentication PID */
#define PID_AUTH_TMO                    0x1032
/* TMO multiport PID */
#define PID_NORMAL_TMO                  0x1034

/*****USB PID for COMMON products*****/
/* COMMON CDROM PID */
#define PID_ONLY_CDROM                  0x1031
/* COMMON authentication PID */
#define PID_AUTH                        0x1033
/* COMMON multiport PID */
#define PID_NORMAL                      0x1035

/* UDISK PID for TMO and COMMON products */
#define PID_UDISK                       0x1005


/* which value should be written to file:
       sys/devices/platform/msm_hsusb_periphera/fixusb
   when the user want to keep the USB composition stable.
   echo 21>fixusb       ->  normal usb switch
   echo 22>fixusb       ->  usb is kept in multiport and not switch
*/
#define ORI_INDEX                       0
#define CDROM_INDEX                     21
#define NORM_INDEX                      22
#define AUTH_INDEX                      99

/* READ TOC command */
#define SC_READ_TOC                     0x43

/* READ_TOC command structure */
typedef  struct _usbsdms_read_toc_cmd_type
{
   u8  op_code;  
   u8  msf;             /* bit1 is MSF, 0: address format is LBA form
                                        1: address format is MSF form */
   u8  format;          /* bit3~bit0,   MSF Field   Track/Session Number
                           0000b:       Valid       Valid as a Track Number
                           0001b:       Valid       Ignored by Drive
                           0010b:       Ignored     Valid as a Session Number
                           0011b~0101b: Ignored     Ignored by Drive
                           0110b~1111b: Reserved
                        */
   u8  reserved1;  
   u8  reserved2;  
   u8  reserved3;  
   u8  session_num;     /* a specific session or a track */
   u8  allocation_length_msb;
   u8  allocation_length_lsb;
   u8  control;
} usbsdms_read_toc_cmd_type;
/*
When the mobile received rewind��SCSI command, (it's bCBWCBLength is 0),
the usb composition should switch from ONLY_CDROM to multiport composition, 
and their PIDs should be different. 

format: 0x11 0x06  <PC_type> <Timeout> <PID>

0x11: main SCSI command
0x06: indicate this is an extended SCSI command, should not conflict with protocol 
<PC_type> OS type
     0x00�� Windows(default)
     0x01:  win98 
     0x02:  win2K 
     0x03:  xp 
     0x04:  vista32 
     0x05:  vista64
     0x10�� Mac OS X 
     0x20�� Linux
     0x30�� gateway
     
<Timeout> how long should be wait to execute the real usb switching. unit is second. 
    0:          execute switching immediately
    0x01-0xFF:  waiting time in second to execute switching
    
< PID > solve the kernel problem in some Linux OS, not used currently, 
    set 0 for this filed
*/
typedef struct _scsi_rewind_cmd_type
{
   u8 cmd;
   u8 ind;
   u8 os_type;
   u8 time_to_delay;
   u8 pidh;
   u8 pidl;
} scsi_rewind_cmd_type;

/* support switch udisk interface from pc */
#define MOUNT_AS_UDISK     "mount_as_udisk"
typedef struct _scsi_rewind_cmd_type_extend
{
   scsi_rewind_cmd_type cmd;
   u8 switch_udisk_maincmd;
   u8 switch_udisk_subcmd;
   u8 reserver[4];
} scsi_rewind_cmd_type_extend;

typedef enum _switch_udisk_maincmd_
{
  MAINCMD_INQUIRY,
  MAINCMD_RUN,
}switch_udisk_maincmd;

typedef enum _switch_udisk_subcmd_
{
  CMD_SWITCH_UDISK = 1,
  CMD_SWITCH_CDROM = 2,
}switch_udisk_subcmd;

typedef enum _switch_udisk_result_
{
  RET_SUCESS,
  RET_DO_NOTHING,
  RET_SWITCH_BUSY,
  RET_SWITCH_SD_ABSENCE,
  RET_SD_INBUSY,
  RET_UNKNOWN_CMD,
}switch_udisk_result;


#define SC_REWIND               0x01
#define SC_REWIND_11            0x11

#define OS_TYPE_MASK            0xf0
#define OS_TYPE_WINDOWS         0x00
#define OS_TYPE_WIN98           0x01
#define OS_TYPE_WIN2K           0x02
#define OS_TYPE_WINXP           0x03
#define OS_TYPE_VISTA32         0x04
#define OS_TYPE_VISTA64         0x05
#define OS_TYPE_MAC             0x10
#define OS_TYPE_LINUX           0x20
#define OS_TYPE_GATEWAY         0x30

#define USB_SERIAL_KEY_SIZE 16


typedef struct _app_usb_para
{
  unsigned usb_pid_index;
  unsigned usb_pid;
}app_usb_para;

typedef struct _usb_pid_stru
{
    u16     cdrom_pid;
    u16     norm_pid;
    u16     udisk_pid;
    u16     auth_pid;
}usb_pid_stru;

extern usb_pid_stru *curr_usb_pid_ptr;

typedef struct _usb_switch_stru
{
    u16     dest_pid;       /* destination usb pid */
    u16     inprogress;     /* 1: inprogress, 0: finished */
}usb_switch_stru;

typedef struct _adb_io_stru
{
    int     read_num;
    int     write_num;
    u8      active;     /* 1: active, 0: inactive*/
    u8      query_num;
}adb_io_stru;

extern app_usb_para usb_para_info;

#ifdef USB_AUTO_DEBUG
#define USB_PR(fmt, ...) \
        printk(KERN_ERR pr_fmt(fmt), ##__VA_ARGS__)
#else
#define USB_PR(fmt, ...) 
#endif


u16 pid_index_to_pid(u32 pid_index);

#endif  /* __USB_SWITCH_HUAWEI_H__ */


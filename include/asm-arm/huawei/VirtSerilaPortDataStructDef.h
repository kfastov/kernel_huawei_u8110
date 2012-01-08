/* linux/include/asm-arm/huawei/VirtSerilaPortDataStructDef.h
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

/******************************************************************************

  Copyright (C), 2001-2011, Huawei Tech. Co., Ltd.

 ******************************************************************************
  File Name     : VirtSerilaPortDataStructDef.h
  Version       : Initial Draft
  Author        : lihongxi
  Created       : 2009/8/10
  Last Modified :
  Description   : Virtual serial port date struct define
  Function List :
  History       :
  1.Date        : 2009/8/10
    Author      : lihongxi
    Modification: Created file

******************************************************************************/
#ifndef _VIRT_SERIAL_PORT_DATA_STRUCT_DEF_H_
#define _VIRT_SERIAL_PORT_DATA_STRUCT_DEF_H_

#ifdef T_ARM
#include "stdint.h"
#endif

/* virtual serial port message head define */
typedef struct _virt_port_package_head_
{
    uint32_t    unLen;      /* package lenth */
}VirtPortPackageHd;

#endif /* END _VIRT_SERIAL_PORT_DATA_STRUCT_DEF_H_ */


/*
 * Rockchip driver for CIF ISP 1.0
 * (Based on Intel driver for sofiaxxx)
 *
 * Copyright (C) 2015 Intel Mobile Communications GmbH
 * Copyright (C) 2017 Fuzhou Rockchip Electronics Co., Ltd.
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _RKISP1_PLTFRM_H
#define _RKISP1_PLTFRM_H

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/wait.h>
#include <linux/platform_data/rkisp1-platform.h>



extern spinlock_t iowrite32_verify_lock;

/* BUG: Register write seems to fail sometimes w/o read before write. */
#define cif_iowrite32_verify(d, a, mask) \
	{ \
		unsigned int i = 0; \
		unsigned long flags = 0; \
		spin_lock_irqsave(&iowrite32_verify_lock, flags); \
		do { \
			iowrite32(d, a); \
			udelay(1); \
			if (i++ == 50) { \
				pr_err("Error in writing %x@0x%p, read %x\n", \
					(d) & (mask), a, ioread32(a)); \
					WARN_ON(1); \
			} \
		} while ((ioread32(a) & mask) != ((d) & mask)); \
		spin_unlock_irqrestore(&iowrite32_verify_lock, flags);\
	}


#endif /* _RKISP1_PLTFRM_H */

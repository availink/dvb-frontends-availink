// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Platform interface definition for Availink demod/tuner drivers
 *
 * Copyright (C) 2020 Availink, Inc. (gpl@availink.com)
 *
 */

#ifndef _AVL_BSP_H_
#define _AVL_BSP_H_

/* Platform-specific includes */
#include <linux/i2c.h>
#include <linux/dvb/frontend.h>
#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#include <linux/types.h>
#endif
#include <stddef.h>

typedef struct mutex avl_sem_t;
/********************************/


struct i2c_adapter *avl_bsp_assoc_i2c_adapter(uint16_t slave_addr,
					      struct i2c_adapter *i2c_adpt);
int32_t avl_bsp_initialize(void);
int32_t avl_bsp_reset(void);
int32_t avl_bsp_delay(uint32_t delay_ms);
int32_t avl_bsp_i2c_read(uint16_t slave_addr, uint8_t *buf, uint16_t *size);
int32_t avl_bsp_i2c_write(uint16_t slave_addr, uint8_t *buf, uint16_t *size);
int32_t avl_bsp_dispose(void);
int32_t avl_bsp_init_semaphore(avl_sem_t *sem);
int32_t avl_bsp_release_semaphore(avl_sem_t *sem);
int32_t avl_bsp_wait_semaphore(avl_sem_t *sem);



#endif

// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Linux implementation of platform interface for Availink demod/tuner drivers
 *
 * Copyright (C) 2020 Availink, Inc. (gpl@availink.com)
 *
 */


#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/bitrev.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/mutex.h>

#include "avl_lib.h"

/* associations between demods and I2C masters */
struct i2c_adapter *i2c_adapter_list[AVL_MAX_NUM_DEMODS];

struct i2c_adapter *avl_bsp_assoc_i2c_adapter(uint16_t slave_addr,
					      struct i2c_adapter *i2c_adpt)
{
	//static struct i2c_adapter *my_i2c_adapter[AVL_MAX_NUM_DEMODS] = {NULL};

	/* use demod ID portion of slave address */
	uint8_t demod_id = (slave_addr >> 8) & AVL_DEMOD_ID_MASK;

	if(i2c_adpt != NULL)
	{
		printk(KBUILD_MODNAME ":%s(): assoc I2C for demod_id %d\n",
		       __FUNCTION__,
		       demod_id);
		i2c_adapter_list[demod_id] = i2c_adpt;
	}
	if(i2c_adapter_list[demod_id] == NULL)
	{
		printk(KBUILD_MODNAME ":%s(): NULL i2c_adapter for demod_id %d\n",
		       __FUNCTION__,
		       demod_id);
	}
	return i2c_adapter_list[demod_id];
}

int32_t avl_bsp_initialize(void)
{
	return (0);
}

int32_t avl_bsp_reset()
{
	return (0);
}

int32_t avl_bsp_delay(uint32_t delay_ms)
{
	//msleep(delay_ms);
	//https://www.kernel.org/doc/Documentation/timers/timers-howto.txt
	if(delay_ms > 20) {
		msleep(delay_ms);
	} else {
		usleep_range(delay_ms * 1000, delay_ms * 2000);
	}
	return (0);
}

int32_t avl_bsp_i2c_read(uint16_t slave_addr, uint8_t *buf, uint16_t *size)
{
	int ret;
	struct i2c_msg msg = {
	    .addr = (slave_addr & 0xff),
	    .flags = I2C_M_RD,
	    .len = *size,
	    .buf = buf};
	ret = i2c_transfer(avl_bsp_assoc_i2c_adapter(slave_addr, NULL),
			   &msg,
			   1);
	if (ret == 1)
	{
		ret = 0;
	}
	else
	{
		dev_warn(&(avl_bsp_assoc_i2c_adapter(slave_addr, NULL)->dev),
			 "%s: i2c rd failed=%d len=%d\n",
			 KBUILD_BASENAME, ret, *size);
		ret = -EREMOTEIO;
	}
	return ret;
}

int32_t avl_bsp_i2c_write(uint16_t slave_addr, uint8_t *buf, uint16_t *size)
{
	int ret;
	struct i2c_msg msg = {
	    .addr = (slave_addr & 0xff),
	    .flags = 0,
	    .buf = buf,
	    .len = *size};
	ret = i2c_transfer(avl_bsp_assoc_i2c_adapter(slave_addr, NULL),
			   &msg,
			   1);
	if (ret == 1)
	{
		ret = 0;
	}
	else
	{
		dev_warn(&(avl_bsp_assoc_i2c_adapter(slave_addr, NULL)->dev),
			 "%s: i2c wr failed=%d len=%d\n",
			 KBUILD_BASENAME, ret, *size);
		ret = -EREMOTEIO;
	}
	return ret;
}

int32_t avl_bsp_dispose(void)
{
	return (0);
}

int32_t avl_bsp_init_semaphore(avl_sem_t *sem)
{
	mutex_init(sem);
	return (0);
}

int32_t avl_bsp_release_semaphore(avl_sem_t *sem)
{
	mutex_unlock(sem);
	return (0);
}

int32_t avl_bsp_wait_semaphore(avl_sem_t *sem)
{
	mutex_lock(sem);
	return (0);
}


EXPORT_SYMBOL_GPL(avl_bsp_assoc_i2c_adapter);
EXPORT_SYMBOL_GPL(avl_bsp_initialize);
EXPORT_SYMBOL_GPL(avl_bsp_reset);
EXPORT_SYMBOL_GPL(avl_bsp_delay);
EXPORT_SYMBOL_GPL(avl_bsp_i2c_read);
EXPORT_SYMBOL_GPL(avl_bsp_i2c_write);
EXPORT_SYMBOL_GPL(avl_bsp_dispose);
EXPORT_SYMBOL_GPL(avl_bsp_init_semaphore);
EXPORT_SYMBOL_GPL(avl_bsp_release_semaphore);
EXPORT_SYMBOL_GPL(avl_bsp_wait_semaphore);
EXPORT_SYMBOL_GPL(avl_bms_sem);

EXPORT_SYMBOL_GPL(avl_bms_initialize);
EXPORT_SYMBOL_GPL(avl_bms_read);
EXPORT_SYMBOL_GPL(avl_bms_read8);
EXPORT_SYMBOL_GPL(avl_bms_read16);
EXPORT_SYMBOL_GPL(avl_bms_read32);
EXPORT_SYMBOL_GPL(avl_bms_read_direct);
EXPORT_SYMBOL_GPL(avl_bms_write);
EXPORT_SYMBOL_GPL(avl_bms_write8);
EXPORT_SYMBOL_GPL(avl_bms_write16);
EXPORT_SYMBOL_GPL(avl_bms_write32);
EXPORT_SYMBOL_GPL(avl_bms_write_direct);

EXPORT_SYMBOL_GPL(avl_add_32to64);
EXPORT_SYMBOL_GPL(avl_divide_64);
EXPORT_SYMBOL_GPL(avl_gte_64);
EXPORT_SYMBOL_GPL(avl_sub_64);
EXPORT_SYMBOL_GPL(avl_mult_32to64);
EXPORT_SYMBOL_GPL(avl_min_32);
EXPORT_SYMBOL_GPL(avl_max_32);

EXPORT_SYMBOL_GPL(avl_bytes_to_short);
EXPORT_SYMBOL_GPL(avl_bytes_to_int);
EXPORT_SYMBOL_GPL(avl_short_to_bytes);
EXPORT_SYMBOL_GPL(avl_int_to_bytes);
EXPORT_SYMBOL_GPL(avl_int_to_3bytes);

MODULE_DESCRIPTION("Linux implementation of platform interface for Availink demod/tuner drivers");
MODULE_AUTHOR("Availink, Inc. (opensource@availink.com)");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.0");

// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Library routines and defines for Availink demod and tuner drivers
 *
 * Copyright (C) 2020 Availink, Inc. (gpl@availink.com)
 *
 */

#include "avl_lib.h"

/* 
 * semaphore to enforce atomicity of bms transactions
 * to each demod
 */
avl_sem_t avl_bms_sem[AVL_MAX_NUM_DEMODS];

uint16_t avl_bms_initialize(uint16_t i2c_addr)
{
	uint16_t r = AVL_EC_OK;
	uint8_t demod_id = (i2c_addr >> AVL_DEMOD_ID_SHIFT) & AVL_DEMOD_ID_MASK;
	r = avl_bsp_init_semaphore(&(avl_bms_sem[demod_id]));
	avl_bsp_initialize();
	return r;
}

uint16_t avl_bms_read(uint16_t slave_addr,
		      uint32_t offset,
		      uint8_t *buf,
		      uint32_t size)
{
	uint16_t r = AVL_EC_OK;
	uint8_t t_buf[3] = {0};
	uint16_t us1 = 0;
	uint32_t ui2 = 0;
	uint16_t usSize = 0;
	uint8_t demod_id = (slave_addr >> AVL_DEMOD_ID_SHIFT) &
			   AVL_DEMOD_ID_MASK;

	r = avl_bsp_wait_semaphore(&(avl_bms_sem[demod_id]));
	if (AVL_EC_OK == r)
	{
		avl_int_to_3bytes(offset, t_buf);
		us1 = 3;
		r = avl_bsp_i2c_write(slave_addr, t_buf, &us1);
		if (AVL_EC_OK == r)
		{
			usSize = size;
			while (usSize > AVL_MAX_II2C_READ_SIZE)
			{
				us1 = AVL_MAX_II2C_READ_SIZE;
				r |= avl_bsp_i2c_read(slave_addr,
						      buf + ui2,
						      &us1);
				ui2 += AVL_MAX_II2C_READ_SIZE;
				usSize -= AVL_MAX_II2C_READ_SIZE;
			}

			if (0 != usSize)
			{
				r |= avl_bsp_i2c_read(slave_addr,
						      buf + ui2,
						      &usSize);
			}
		}
	}
	r |= avl_bsp_release_semaphore(&(avl_bms_sem[demod_id]));

	return (r);
}

uint16_t avl_bms_read8(uint16_t slave_addr, uint32_t addr, uint8_t *data)
{
	uint16_t r = AVL_EC_OK;
	uint8_t t_data = 0;

	r = avl_bms_read(slave_addr, addr, &t_data, 1);
	if (AVL_EC_OK == r)
	{
		*data = t_data;
	}

	return (r);
}

uint16_t avl_bms_read16(uint16_t slave_addr, uint32_t addr, uint16_t *data)
{
	uint16_t r = AVL_EC_OK;
	uint8_t buf[2] = {0};

	r = avl_bms_read(slave_addr, addr, buf, 2);
	if (AVL_EC_OK == r)
	{
		*data = avl_bytes_to_short(buf);
	}

	return (r);
}

uint16_t avl_bms_read32(uint16_t slave_addr, uint32_t addr, uint32_t *data)
{
	uint16_t r = AVL_EC_OK;
	uint8_t buf[4] = {0};

	r = avl_bms_read(slave_addr, addr, buf, 4);
	if (AVL_EC_OK == r)
	{
		*data = avl_bytes_to_int(buf);
	}

	return (r);
}

uint16_t avl_bms_read_direct(uint16_t slave_addr, uint8_t *buf, uint16_t size)
{
	uint16_t r = AVL_EC_OK;
	uint16_t ui1 = 0;
	uint32_t ui2 = 0;
	uint16_t t_size = 0;
	uint8_t demod_id = (slave_addr >> AVL_DEMOD_ID_SHIFT) &
			   AVL_DEMOD_ID_MASK;

	r = avl_bsp_wait_semaphore(&(avl_bms_sem[demod_id]));
	if (AVL_EC_OK == r)
	{
		t_size = size;
		while (t_size > AVL_MAX_II2C_READ_SIZE)
		{
			ui1 = AVL_MAX_II2C_READ_SIZE;
			r |= avl_bsp_i2c_read(slave_addr, buf + ui2, &ui1);
			ui2 += AVL_MAX_II2C_READ_SIZE;
			t_size -= AVL_MAX_II2C_READ_SIZE;
		}

		if (0 != t_size)
		{
			r |= avl_bsp_i2c_read(slave_addr, buf + ui2, &t_size);
		}
	}
	r |= avl_bsp_release_semaphore(&(avl_bms_sem[demod_id]));

	return (r);
}

uint16_t avl_bms_write_direct(uint16_t slave_addr, uint8_t *buf, uint16_t size)
{
	uint16_t r = AVL_EC_OK;
	uint16_t ui1 = 0;
	uint32_t ui2 = 0;
	uint32_t tmp = 0;
	uint32_t t_size = 0;
	uint8_t demod_id = (slave_addr >> AVL_DEMOD_ID_SHIFT) &
			   AVL_DEMOD_ID_MASK;

	r = avl_bsp_wait_semaphore(&(avl_bms_sem[demod_id]));
	if (AVL_EC_OK == r)
	{
		t_size = size;
		tmp = (AVL_MAX_II2C_WRITE_SIZE - 3) & 0xfffe;
		while (t_size > tmp)
		{
			ui1 = tmp;
			r |= avl_bsp_i2c_write(slave_addr, buf + ui2, &ui1);
			ui2 += tmp;
			t_size -= tmp;
		}
		ui1 = t_size;
		r |= avl_bsp_i2c_write(slave_addr, buf + ui2, &ui1);
		ui2 += t_size;
	}
	r |= avl_bsp_release_semaphore(&(avl_bms_sem[demod_id]));

	return (r);
}

uint16_t avl_bms_write(uint16_t slave_addr, uint8_t *buf, uint32_t size)
{
	uint16_t r = AVL_EC_OK;
	uint8_t t_buf[5] = {0};
	uint16_t ui1 = 0;
	uint32_t ui2 = 0;
	uint16_t tmp = 0;
	uint32_t t_size = 0;
	uint32_t t_addr = 0;
	uint8_t demod_id = (slave_addr >> AVL_DEMOD_ID_SHIFT) &
			   AVL_DEMOD_ID_MASK;

	if (size < 3)
	{
		r = AVL_EC_GENERAL_FAIL;
	}
	else
	{

		/* size includes 3 bytes for address */
		size -= 3;
		r = avl_bsp_wait_semaphore(&(avl_bms_sem[demod_id]));
		if (AVL_EC_OK == r)
		{
			/* address portion */
			t_addr = buf[0];
			t_addr = t_addr << 8;
			t_addr += buf[1];
			t_addr = t_addr << 8;
			t_addr += buf[2];

			t_size = size;

			/* how many bytes data we can transfer every time */
			tmp = (AVL_MAX_II2C_WRITE_SIZE - 3) & 0xfffe;

			ui2 = 0;
			while (t_size > tmp)
			{
				ui1 = tmp + 3;
				/* save data */
				t_buf[0] = buf[ui2];
				t_buf[1] = buf[ui2 + 1];
				t_buf[2] = buf[ui2 + 2];
				avl_int_to_3bytes(t_addr, buf + ui2);
				r |= avl_bsp_i2c_write(slave_addr, buf + ui2, &ui1);
				/* restore data */
				buf[ui2] = t_buf[0];
				buf[ui2 + 1] = t_buf[1];
				buf[ui2 + 2] = t_buf[2];
				t_addr += tmp;
				ui2 += tmp;
				t_size -= tmp;
			}
			ui1 = t_size + 3;

			t_buf[0] = buf[ui2];
			t_buf[1] = buf[ui2 + 1];
			t_buf[2] = buf[ui2 + 2];
			avl_int_to_3bytes(t_addr, buf + ui2);
			r |= avl_bsp_i2c_write(slave_addr, buf + ui2, &ui1);

			buf[ui2] = t_buf[0];
			buf[ui2 + 1] = t_buf[1];
			buf[ui2 + 2] = t_buf[2];
			t_addr += t_size;
			ui2 += t_size;
		}
		r |= avl_bsp_release_semaphore(&(avl_bms_sem[demod_id]));
	}
	return (r);
}

uint16_t avl_bms_write8(uint16_t slave_addr, uint32_t addr, uint8_t data)
{
	uint16_t r = AVL_EC_OK;
	uint8_t buf[4] = {0};

	avl_int_to_3bytes(addr, buf);
	buf[3] = data;
	r = avl_bms_write(slave_addr, buf, 4);

	return (r);
}

uint16_t avl_bms_write16(uint16_t slave_addr, uint32_t addr, uint16_t data)
{
	uint16_t r = AVL_EC_OK;
	uint8_t buf[5] = {0};

	avl_int_to_3bytes(addr, buf);
	avl_short_to_bytes(data, buf + 3);
	r = avl_bms_write(slave_addr, buf, 5);

	return (r);
}

uint16_t avl_bms_write32(uint16_t slave_addr, uint32_t addr, uint32_t data)
{
	uint16_t r = AVL_EC_OK;
	uint8_t buf[7] = {0};

	avl_int_to_3bytes(addr, buf);
	avl_int_to_bytes(data, buf + 3);
	r = avl_bms_write(slave_addr, buf, 7);

	return (r);
}

void avl_add_32to64(struct avl_uint64 *sum, uint32_t addend)
{
	uint32_t uiTemp = 0;

	uiTemp = sum->low_word;
	sum->low_word += addend;
	sum->low_word &= 0xFFFFFFFF;

	if (sum->low_word < uiTemp)
	{
		sum->high_word++;
	}
}

uint32_t avl_divide_64(struct avl_uint64 divisor, struct avl_uint64 dividend)
{
	uint32_t uFlag = 0x0;
	uint32_t uQuto = 0x0;
	uint32_t i = 0;
	uint32_t dividend_H = dividend.high_word;
	uint32_t dividend_L = dividend.low_word;
	uint32_t divisor_H = divisor.high_word;
	uint32_t divisor_L = divisor.low_word;

	if (((divisor_H == 0x0) && (divisor_L == 0x0)) || (dividend_H / divisor_L))
	{
		return 0;
	}
	else if ((divisor_H == 0x0) && (dividend_H == 0x0))
	{
		return (dividend_L / divisor_L);
	}
	else
	{
		if (divisor_H != 0)
		{
			while (divisor_H)
			{
				dividend_L /= 2;
				if (dividend_H % 2)
				{
					dividend_L += 0x80000000;
				}
				dividend_H /= 2;

				divisor_L /= 2;
				if (divisor_H % 2)
				{
					divisor_L += 0x80000000;
				}
				divisor_H /= 2;
			}
		}
		for (i = 0; i <= 31; i++)
		{
			uFlag = (int32_t)dividend_H >> 31;
			dividend_H = (dividend_H << 1) | (dividend_L >> 31);
			dividend_L <<= 1;
			uQuto <<= 1;

			if ((dividend_H | uFlag) >= divisor_L)
			{
				dividend_H -= divisor_L;
				uQuto++;
			}
		}
		return uQuto;
	}
}

uint32_t avl_gte_64(struct avl_uint64 a, struct avl_uint64 b)
{
	uint32_t result = 0;

	if ((a.high_word == b.high_word) && (a.low_word == b.low_word))
	{
		result = 1;
	}
	if (a.high_word > b.high_word)
	{
		result = 1;
	}
	else if (a.high_word == b.high_word)
	{
		if (a.low_word > b.low_word)
		{
			result = 1;
		}
	}

	return result;
}

void avl_sub_64(struct avl_uint64 *a, struct avl_uint64 b)
{
	struct avl_uint64 temp_1 = {0, 0};
	struct avl_uint64 temp_2 = {0, 0};

	temp_1.high_word = a->high_word;
	temp_1.low_word = a->low_word;

	temp_2.high_word = temp_1.high_word - b.high_word;
	if (temp_1.low_word >= b.low_word)
	{
		temp_2.low_word = temp_1.low_word - b.low_word;
	}
	else
	{
		temp_2.low_word = b.low_word - temp_1.low_word;
		temp_2.high_word >>= 1;
	}

	a->high_word = temp_2.high_word;
	a->low_word = temp_2.low_word;
}

void avl_mult_32to64(struct avl_uint64 *product, uint32_t m1, uint32_t m2)
{
	product->low_word = (m1 & 0xFFFF) * (m2 & 0xFFFF);
	product->high_word = 0;

	avl_add_scaled32to64(product, (m1 >> 16) * (m2 & 0xFFFF));
	avl_add_scaled32to64(product, (m2 >> 16) * (m1 & 0xFFFF));

	product->high_word += (m1 >> 16) * (m2 >> 16);
}

void avl_add_scaled32to64(struct avl_uint64 *sum, uint32_t addend)
{
	uint32_t saved = 0;

	saved = sum->low_word;
	sum->low_word += (addend << 16);

	sum->low_word &= 0xFFFFFFFF;
	sum->high_word += ((sum->low_word < saved) ? 1 : 0) + (addend >> 16);
}

uint32_t avl_min_32(uint32_t a, uint32_t b)
{
	if (a < b)
	{
		return (a);
	}
	else
	{
		return (b);
	}
}

uint32_t avl_max_32(uint32_t a, uint32_t b)
{
	if (a > b)
	{
		return (a);
	}
	else
	{
		return (b);
	}
}

/*
 * All of these conversion routines implement
 * big-endian style byte ordering.
 */
uint16_t avl_bytes_to_short(const uint8_t *buf)
{
	uint16_t data = 0;
	data = buf[0];
	data = (uint16_t)(data << 8) + buf[1];

	return data;
}

uint32_t avl_bytes_to_int(const uint8_t *buf)
{
	uint32_t data = 0;
	data = buf[0];
	data = (data << 8) + buf[1];
	data = (data << 8) + buf[2];
	data = (data << 8) + buf[3];

	return data;
}

void avl_short_to_bytes(uint16_t data, uint8_t *buf)
{
	buf[0] = (uint8_t)(data >> 8);
	buf[1] = (uint8_t)(data);

	return;
}

void avl_int_to_3bytes(uint32_t data, uint8_t *buf)
{
	buf[0] = (uint8_t)(data >> 16);
	buf[1] = (uint8_t)(data >> 8);
	buf[2] = (uint8_t)(data);

	return;
}

void avl_int_to_bytes(uint32_t data, uint8_t *buf)
{
	buf[0] = (uint8_t)(data >> 24);
	buf[1] = (uint8_t)(data >> 16);
	buf[2] = (uint8_t)(data >> 8);
	buf[3] = (uint8_t)(data);

	return;
}
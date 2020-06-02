// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Availink AVL62X1 DVB-S/S2/S2X demodulator driver
 * Supports AVL6221 and AVL6261. NOT AVL6211
 *
 * Copyright (C) 2020 Availink, Inc. (gpl@availink.com)
 *
 */

#ifndef _AVL62X1_H_
#define _AVL62X1_H_

#include <linux/firmware.h>
#include <linux/dvb/frontend.h>
#include <linux/dvb/version.h>
#include <media/dvb_frontend.h>

#include "avl62x1_lib.h"

#define str(a) #a
#define xstr(a) str(a)

#define DVB_VER_INT(maj,min) (((maj) << 16) + (min))
#define DVB_VER_ATLEAST(maj, min) \
 (DVB_VER_INT(DVB_API_VERSION,  DVB_API_VERSION_MINOR) >= DVB_VER_INT(maj, min))

#define AVL62X1_FIRMWARE	"availink/dvb-fe-avl62x1.fw"

#define AVL62X1_VERSION xstr(AVL62X1_VER_MAJOR) "." xstr(AVL62X1_VER_MINOR) "." xstr(AVL62X1_VER_BUILD)

#define AVL62X1_BS_CTRL_PROP			isdbt_sb_segment_idx
//isdbt_sb_segment_idx fields
#define AVL62X1_BS_CTRL_VALID_STREAM_MASK	(0x80000000)
#define AVL62X1_BS_CTRL_NEW_TUNE_MASK		(0x40000000)
#define AVL62X1_BS_CTRL_MORE_RESULTS_MASK	(0x20000000)

//stream_id fields
#define AVL62X1_BS_IS_T2MI_SHIFT	29
#define AVL62X1_BS_T2MI_PID_SHIFT	16
#define AVL62X1_BS_T2MI_PLP_ID_SHIFT	8

struct i2cctl_ioctl_lock_req {
	int demod;
	int tuner;
};

struct avl62x1_priv
{
	struct i2c_adapter *i2c;
	//struct avl62x1_config *config;
	struct dvb_frontend frontend;
	enum fe_delivery_system delivery_system;
	struct avl62x1_chip *chip;
	const struct firmware *fw;
};

struct avl62x1_config
{
	//int i2c_id;	    // i2c adapter (master) id
	//void *i2c_adapter;     // i2c adapter (master)

	//structure of user-configurable parameters
	struct avl62x1_chip_pub *chip_pub;

	//uint8_t demod_address; // demodulator i2c address


};

struct avl62x1_bs_state {
	uint8_t bs_mode;
	uint8_t num_carriers;
	int8_t cur_carrier;
	int8_t cur_stream;
	struct avl62x1_blind_scan_params params;
	struct avl62x1_blind_scan_info info;
	struct avl62x1_carrier_info *carriers;
	struct avl62x1_stream_info *streams;
};

extern struct dvb_frontend *avl62x1_attach(struct avl62x1_config *config,
					   struct i2c_adapter *i2c);

#endif /* _AVL62X1_H_ */

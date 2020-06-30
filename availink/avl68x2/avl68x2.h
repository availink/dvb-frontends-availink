// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Availink AVL68x2 DVB-S/S2/T/T2/C, ISDB-T, J83.B demodulator driver
 *
 * Copyright (C) 2020 Availink, Inc. (gpl@availink.com)
 *
 */

#ifndef _AVL68x2_H_
#define _AVL68x2_H_

#include <linux/i2c.h>
#include <linux/firmware.h>
#include <linux/dvb/frontend.h>
#include <linux/dvb/version.h>
#include <media/dvb_frontend.h>

#include "avl68x2_common.h"
#include "avl68x2_dvbsx.h"

#define str(a) #a
#define xstr(a) str(a)

#define DVB_VER_INT(maj, min) (((maj) << 16) + (min))
#define DVB_VER_ATLEAST(maj, min) \
	(DVB_VER_INT(DVB_API_VERSION, DVB_API_VERSION_MINOR) >= DVB_VER_INT(maj, min))

#define AVL68X2_DVBSX_FW	"availink/dvb-fe-avl68x2-dvbsx.fw"
#define AVL68X2_DVBC_FW		"availink/dvb-fe-avl68x2-dvbc.fw"
#define AVL68X2_DVBTX_FW	"availink/dvb-fe-avl68x2-dvbtx.fw"
#define AVL68X2_ISDBT_FW	"availink/dvb-fe-avl68x2-isdbt.fw"

#define AVL68X2_VERSION xstr(AVL68X2_VER_MAJOR) "." xstr(AVL68X2_VER_MINOR) "." xstr(AVL68X2_VER_BUILD)

#define AVL68X2_BS_CTRL_PROP			isdbt_sb_segment_idx
//isdbt_sb_segment_idx fields
#define AVL68X2_BS_CTRL_VALID_STREAM_MASK	(0x80000000)
#define AVL68X2_BS_CTRL_NEW_TUNE_MASK		(0x40000000)
#define AVL68X2_BS_CTRL_MORE_RESULTS_MASK	(0x20000000)

struct i2cctl_ioctl_lock_req {
	int demod;
	int tuner;
};

struct avl68x2_priv
{
	struct i2c_adapter *i2c;
	//struct avl68x2_config *config;
	struct dvb_frontend frontend;
	enum fe_delivery_system delivery_system;
	struct avl68x2_chip *chip;
	const struct firmware *fw;
};

struct avl68x2_config
{
	//structure of user-configurable parameters
	struct avl68x2_chip_pub *chip_pub;
};

extern const AVL_DVBTxConfig default_dvbtx_config;
extern const AVL_DVBSxConfig default_dvbsx_config;
extern const AVL_ISDBTConfig default_isdbt_config;
extern const AVL_DVBCConfig default_dvbc_config;

extern struct dvb_frontend *avl68x2_attach(struct avl68x2_config *config, struct i2c_adapter *i2c);

#endif /* AVL68x2_H */

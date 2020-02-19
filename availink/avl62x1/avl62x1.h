/*
 * Availink AVL62X1 DVB-S/S2/S2X demodulator driver
 * Supports AVL6221 and AVL6261. NOT AVL6211
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (C) 2020 Availink, Inc. (opensource@availink.com)
 *
 */

#ifndef _AVL62X1_H_
#define _AVL62X1_H_

#include <linux/firmware.h>
#include <linux/dvb/frontend.h>
#include <media/dvb_frontend.h>

#include "avl62x1_lib.h"

#define xstr(a) str(a)
#define str(a) #a

//MAJOR.minor.build
//MAJOR = public API rev
//minor = SDK API rev (a.k.a. SDK API MAJOR rev)
//build number = increment on every change to implementation
#define AVL62X1_VERSION "1." xstr(AVL62X1_API_VER_MAJOR) ".0"

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

extern struct dvb_frontend *avl62x1_attach(struct avl62x1_config *config,
					   struct i2c_adapter *i2c);

#endif /* _AVL62X1_H_ */

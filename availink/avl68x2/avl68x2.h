// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Availink AVL68x2 DVB-S/S2/T/T2/C, ISDB-T, J83.B demodulator driver
 *
 * Copyright (C) 2020 Availink, Inc. (gpl@availink.com)
 *
 */


#ifndef _AVL68x2_H_
#define _AVL68x2_H_

#include <linux/firmware.h>
#include <linux/dvb/frontend.h>
#include <linux/dvb/version.h>
#include <media/dvb_frontend.h>

#include "AVL_Demod.h"

#define str(a) #a
#define xstr(a) str(a)

#define DVB_VER_INT(maj,min) (((maj) << 16) + (min))
#define DVB_VER_ATLEAST(maj, min) \
 (DVB_VER_INT(DVB_API_VERSION,  DVB_API_VERSION_MINOR) >= DVB_VER_INT(maj, min))

#define AVL68X2_DVBSX_FW	"availink/dvb-fe-avl68x2-dvbsx.fw"
#define AVL68X2_DVBC_FW		"availink/dvb-fe-avl68x2-dvbc.fw"
#define AVL68X2_DVBTX_FW	"availink/dvb-fe-avl68x2-dvbtx.fw"
#define AVL68X2_ISDBT_FW	"availink/dvb-fe-avl68x2-isdbt.fw"

//MAJOR.minor.build
//MAJOR = public API rev
//minor = SDK API rev (a.k.a. SDK API MAJOR rev)
//build number = increment on every change to implementation
#define AVL68x2_VERSION       "1." xstr(AVL_API_VER_MAJOR) ".0"


struct avl68x2_priv
{
  struct i2c_adapter *i2c;
  struct avl68x2_config *config;
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

extern struct dvb_frontend *avl68x2_attach(struct avl68x2_config *config, struct i2c_adapter *i2c);

#endif /* AVL68x2_H */

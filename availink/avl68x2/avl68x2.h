/*
 * Availink AVL68x2 DVB-S/S2/T/T2/C, ISDB-T, J83.B demodulator driver
 *
 * Copyright (C) 2020 Availink, Inc. (opensource@availink.com)
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License along
 *    with this program; if not, write to the Free Software Foundation, Inc.,
 *    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef _AVL68x2_H_
#define _AVL68x2_H_

#include <linux/firmware.h>
#include <linux/dvb/frontend.h>
#include <media/dvb_frontend.h>

#include "AVL_Demod.h"

#define xstr(a) str(a)
#define str(a) #a

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
  struct AVL_ChipInternal *chip;  
};

struct avl68x2_config
{
  int i2c_id;        // i2c adapter (master) id
  void *i2c_adapter; // i2c adapter (master)
  uint8_t demod_address;  // demodulator i2c address
  uint8_t tuner_address;  // tuner i2c address
  unsigned char eDiseqcStatus;
};

extern struct dvb_frontend *avl68x2_attach(struct avl68x2_config *config, struct i2c_adapter *i2c);

#endif /* AVL68x2_H */

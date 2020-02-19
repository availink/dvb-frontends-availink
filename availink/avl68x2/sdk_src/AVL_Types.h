/*
 * Availink AVL6261 DVB-S/S2/S2X demodulator driver
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

#ifndef _AVL_TYPES_H_
#define _AVL_TYPES_H_

#define nullptr     NULL
#define AVL_FALSE   0
#define AVL_TRUE    1

#ifdef AVL_CPLUSPLUS
extern "C"
{
#endif

  typedef unsigned char avl_bool_t;

  typedef unsigned char avl_sem_t;   ///< the semaphore data type.

  typedef uint16_t avl_error_code_t; // Defines the error code

#ifdef AVL_CPLUSPLUS
}
#endif

#endif

// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Availink AVL6261 DVB-S/S2/S2X demodulator driver - tuner abstraction
 *
 * Copyright (C) 2020 Availink, Inc. (gpl@availink.com)
 *
 */

#ifndef _AVL_TUNER_H_
#define _AVL_TUNER_H_

#include "avl_lib.h"

#ifdef AVL_CPLUSPLUS
extern "C"
{
#endif

#define AVL_TUNER_EC_OK 0 // There is no error.

  struct avl_tuner
  {
    uint16_t i2c_addr;
    uint8_t is_locked; //1 Lock;   0 unlock

    uint32_t rf_freq_hz;
    uint32_t lpf_hz; //only valid for satellite tuner

    uint8_t blindscan_mode;

    void *more_params;

    uint32_t (*initialize)(struct avl_tuner *);
    uint32_t (*lock)(struct avl_tuner *);
    uint32_t (*get_lock_status)(struct avl_tuner *);
    uint32_t (*get_rf_strength)(struct avl_tuner *, int32_t *);

    //Maximum tuner low pass filter bandwidth in Hz
    uint32_t (*get_max_lpf)(struct avl_tuner *, uint32_t *);

    //Minimum tuner low pass filter bandwidth in Hz
    uint32_t (*get_min_lpf)(struct avl_tuner *, uint32_t *);

    //Low pass filter bandwidth step size in Hz
    uint32_t (*get_lpf_step_size)(struct avl_tuner *, uint32_t *);

    //Tuner AGC gain slope in dB per Volt (dB/V).
    //Tuners with non-inverted AGC sense have a positive slope.
    //Tuners with inverted AGC sense have a negative slope.
    //If the gain slope is not known, implement a function that
    //  returns 1 if the AGC sense is non-inverted,
    //  and returns -1 if the AGC sense is inverted.
    uint32_t (*get_agc_slope)(struct avl_tuner *, int32_t *);

    //Voltage at which gain reaches minimum value.  Voltage in millivolts.
    //For a tuner with non-inverted sense (positive slope), this will be a small value.
    //For a tuner with inverted sense (negative slope), this will be a large value.
    uint32_t (*get_min_gain_voltage)(struct avl_tuner *, uint32_t *);

    //Voltage at which gain reaches its maximum value. Voltage in millivolts.
    //For a tuner with non-inverted sense (positive slope), this will be a large value.
    //For a tuner with inverted sense (negative slope), this will be a small value.
    uint32_t (*get_max_gain_voltage)(struct avl_tuner *, uint32_t *);

    //RF Frequency step size in Hz
    uint32_t (*get_rf_freq_step_size)(struct avl_tuner *, uint32_t *);

  };

#ifdef AVL_CPLUSPLUS
}
#endif

#endif

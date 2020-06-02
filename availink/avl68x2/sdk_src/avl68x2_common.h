// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Availink AVL68x2 DVB-S/S2/T/T2/C, ISDB-T, J83.B demodulator driver
 *
 * Copyright (C) 2020 Availink, Inc. (gpl@availink.com)
 *
 */

#ifndef __avl68x2_common_h__
#define __avl68x2_common_h__

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

#include "avl_lib.h"
#include "avl68x2_internal.h"


typedef enum avl68x2_spectrum_polarity
{
    AVL_SPECTRUM_NORMAL     =   0,
    AVL_SPECTRUM_INVERTED   =   1,
    AVL_SPECTRUM_AUTO       =   2
} avl68x2_spectrum_polarity;


avl_error_code_t avl68x2_demod_initialize(AVL_DemodMode eStartupMode, avl68x2_chip *chip);
avl_error_code_t avl68x2_demod_get_chip_id(uint32_t * puiChipID,avl68x2_chip *chip);
avl_error_code_t avl68x2_demod_get_lock_status(uint8_t * pucDemodLocked, avl68x2_chip *chip);
avl_error_code_t avl68x2_demod_get_snr (uint32_t * puiSNRx100, avl68x2_chip *chip);
avl_error_code_t avl68x2_demod_get_sqi (uint16_t * pusSQI, avl68x2_chip *chip);
avl_error_code_t avl68x2_demod_get_ssi(uint16_t * pusSSI, avl68x2_chip *chip);
avl_error_code_t avl68x2_demod_get_per(uint32_t *puiPERxe9, avl68x2_chip *chip);
avl_error_code_t avl68x2_demod_set_mode(AVL_DemodMode eDemodMode,avl68x2_chip *chip);
avl_error_code_t avl68x2_demod_sleep(avl68x2_chip *chip);
avl_error_code_t avl68x2_demod_wakeup(avl68x2_chip *chip);
avl_error_code_t avl68x2_demod_i2c_passthru_on(avl68x2_chip *chip);
avl_error_code_t avl68x2_demod_i2c_passthru_off(avl68x2_chip *chip);
avl_error_code_t avl68x2_demod_get_version(AVL_DemodVersion *pstDemodVersion, avl68x2_chip *chip);
avl_error_code_t avl68x2_demod_set_gpio(AVL_GPIOPinNumber ePinNumber, AVL_GPIOPinValue ePinValue, avl68x2_chip *chip);
avl_error_code_t avl68x2_demod_get_gpio(AVL_GPIOPinNumber ePinNumber, AVL_GPIOPinValue *pePinValue, avl68x2_chip *chip);
avl_error_code_t avl68x2_demod_ts_on(avl68x2_chip *chip);
avl_error_code_t avl68x2_demod_ts_off(avl68x2_chip *chip);


#endif


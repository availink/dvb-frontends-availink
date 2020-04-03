// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Availink AVL68x2 DVB-S/S2/T/T2/C, ISDB-T, J83.B demodulator driver
 *
 * Copyright (C) 2020 Availink, Inc. (gpl@availink.com)
 *
 */

#ifndef AVL_Demod_H
#define AVL_Demod_H

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

#include "avl_lib.h"
#include "AVL_Demod_CommonInternal.h"



// Defines the AVL device spectrum inversion mode
typedef enum AVL_SpectrumInversion
{
    AVL_SPECTRUM_NORMAL     =   0,                      // Signal spectrum in normal.
    AVL_SPECTRUM_INVERTED   =   1,                      // Signal spectrum in inverted.
    AVL_SPECTRUM_AUTO       =   2                       // Signal spectrum isn't known.
}AVL_SpectrumInversion;


avl_error_code_t AVL_Demod_Initialize(AVL_DemodMode eStartupMode, avl68x2_chip *chip);
avl_error_code_t AVL_Demod_GetChipID(uint32_t * puiChipID,avl68x2_chip *chip);
avl_error_code_t AVL_Demod_GetLockStatus(uint8_t * pucDemodLocked, avl68x2_chip *chip);
avl_error_code_t AVL_Demod_GetSNR (uint32_t * puiSNRx100, avl68x2_chip *chip);
avl_error_code_t AVL_Demod_GetSQI (uint16_t * pusSQI, avl68x2_chip *chip);
avl_error_code_t AVL_Demod_GetSSI(uint16_t * pusSSI, avl68x2_chip *chip);
avl_error_code_t AVL_Demod_GetPER(uint32_t *puiPERxe9, avl68x2_chip *chip);
avl_error_code_t AVL_Demod_SetMode(AVL_DemodMode eDemodMode,avl68x2_chip *chip);
avl_error_code_t AVL_Demod_Sleep(avl68x2_chip *chip);
avl_error_code_t AVL_Demod_Wakeup(avl68x2_chip *chip);
avl_error_code_t AVL_Demod_I2CPassThruOn(avl68x2_chip *chip);
avl_error_code_t AVL_Demod_I2CPassThruOff(avl68x2_chip *chip);
avl_error_code_t AVL_Demod_GetVersion(AVL_DemodVersion *pstDemodVersion, avl68x2_chip *chip);
avl_error_code_t AVL_Demod_SetGPIO(AVL_GPIOPinNumber ePinNumber, AVL_GPIOPinValue ePinValue, avl68x2_chip *chip);
avl_error_code_t AVL_Demod_GetGPIOValue(AVL_GPIOPinNumber ePinNumber, AVL_GPIOPinValue *pePinValue, avl68x2_chip *chip);
avl_error_code_t AVL_Demod_TsOn(avl68x2_chip *chip);
avl_error_code_t AVL_Demod_TsOff(avl68x2_chip *chip);


#endif


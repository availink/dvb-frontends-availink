#ifndef AVL_Demod_H
#define AVL_Demod_H

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

#include "avl_lib.h"

#include "AVL_Demod_CommonInternal.h"

#if defined(_AVL68XX_)
#include "AVL_Demod_DVBSx.h"
#endif

#if defined(_AVL68XX_)
#include "AVL_Demod_DVBTx.h"
#endif

#if defined(_AVL68XX_)
#include "AVL_Demod_DVBC.h"
#endif

#if defined(_AVL68XX_)
#include "AVL_Demod_ISDBT.h"
#endif

#if defined(_AVL63XX_)
#include "AVL_Demod_DTMB.h"
#endif

#ifdef AVL_CPLUSPLUS
extern "C" {
#endif






// Defines the AVL device spectrum inversion mode
typedef enum AVL_SpectrumInversion
{
    AVL_SPECTRUM_NORMAL     =   0,                      // Signal spectrum in normal.
    AVL_SPECTRUM_INVERTED   =   1,                      // Signal spectrum in inverted.
    AVL_SPECTRUM_AUTO       =   2                       // Signal spectrum isn't known.
}AVL_SpectrumInversion;





avl_error_code_t AVL_Demod_Initialize(AVL_DemodMode eStartupMode, AVL_ChipInternal *chip, uint8_t bI2CChipSelect);
avl_error_code_t AVL_Demod_GetChipID(uint32_t * puiChipID,AVL_ChipInternal *chip);
avl_error_code_t AVL_Demod_GetLockStatus(uint8_t * pucDemodLocked, AVL_ChipInternal *chip);
avl_error_code_t AVL_Demod_GetSNR (uint32_t * puiSNRx100, AVL_ChipInternal *chip);
avl_error_code_t AVL_Demod_GetSQI (uint16_t * pusSQI, AVL_ChipInternal *chip);
avl_error_code_t AVL_Demod_GetSSI(uint16_t * pusSSI, AVL_ChipInternal *chip);
avl_error_code_t AVL_Demod_GetPER(uint32_t *puiPERxe9, AVL_ChipInternal *chip);
avl_error_code_t AVL_Demod_SetMode(AVL_DemodMode eDemodMode,AVL_ChipInternal *chip);
avl_error_code_t AVL_Demod_Sleep(AVL_ChipInternal *chip);
avl_error_code_t AVL_Demod_Wakeup(AVL_ChipInternal *chip);
avl_error_code_t AVL_Demod_I2CBypassOn(AVL_ChipInternal *chip);
avl_error_code_t AVL_Demod_I2CBypassOff(AVL_ChipInternal *chip);
avl_error_code_t AVL_Demod_GetVersion(AVL_DemodVersion *pstDemodVersion, AVL_ChipInternal *chip);
avl_error_code_t AVL_Demod_SetGPIO(AVL_GPIOPinNumber ePinNumber, AVL_GPIOPinValue ePinValue, AVL_ChipInternal *chip);
avl_error_code_t AVL_Demod_GetGPIOValue(AVL_GPIOPinNumber ePinNumber, AVL_GPIOPinValue *pePinValue, AVL_ChipInternal *chip);
avl_error_code_t AVL_Demod_TsOn(AVL_ChipInternal *chip);
avl_error_code_t AVL_Demod_TsOff(AVL_ChipInternal *chip);

#ifdef AVL_CPLUSPLUS
}
#endif

#endif


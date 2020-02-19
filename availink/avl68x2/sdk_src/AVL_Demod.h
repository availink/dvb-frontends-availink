#ifndef AVL_Demod_H
#define AVL_Demod_H

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

#include "AVL_Types.h"
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

//#define AVL_EC_OK                   0           // There is no error.
#define AVL_EC_WARNING              1           // There is warning.
//#define AVL_EC_GENERAL_FAIL         2           // A general failure has occurred.
#define AVL_EC_I2C_FAIL             4           // An I2C operation failed during communication with the AVLEM61 through the BSP.
#define AVL_EC_I2C_REPEATER_FAIL    8           // An error ocurred during communication between AVLEM61 I2C master and tuner. This error code is defined by enum AVLEM61_MessageType_II2C_Repeater_Status.                        	
//#define AVL_EC_RUNNING              16          // The AVLEM61 device is busy processing a previous receiver/i2c repeater command.
#define AVL_EC_TIMEOUT              32          // Operation failed in a given time period
#define AVL_EC_SLEEP                64          // Demod in sleep mode
#define AVL_EC_NOT_SUPPORTED        128         // Specified operation isn't support in current senario
#define AVL_EC_BSP_ERROR1           256         // BSP Error 1, if it's used, need to be customized
#define AVL_EC_BSP_ERROR2           512         // BSP Error 2, if it's used, need to be customized

#define AVL_CONSTANT_10_TO_THE_9TH      1000000000  //10e9



#define AVL_min(x,y) (((x) < (y)) ? (x) : (y))
#define AVL_max(x,y) (((x) < (y)) ? (y) : (x))
#define AVL_floor(a) (((a) == (int)(a))? ((int)(a)) : (((a) < 0)? ((int)((a)-1)) : ((int)(a))))
#define AVL_ceil(a)  (((a) == (int)(a))? ((int)(a)) : (((a) < 0)? ((int)(a)) : ((int)((a)+1))))
#define AVL_abs(a)  (((a)>0) ? (a) : (-(a)))

// Defines the AVL device spectrum inversion mode
typedef enum AVL_SpectrumInversion
{
    AVL_SPECTRUM_NORMAL     =   0,                      // Signal spectrum in normal.
    AVL_SPECTRUM_INVERTED   =   1,                      // Signal spectrum in inverted.
    AVL_SPECTRUM_AUTO       =   2                       // Signal spectrum isn't known.
}AVL_SpectrumInversion;

/// Chunk two bytes uidata in to pBuff.
/// 
/// @param uidata The input 2 bytes data.
/// @param pBuff The destination buffer, at least 2 bytes length.
/// 
/// @remarks This function is used to eliminates the big endian and little endian problem.
void Chunk16_Demod(uint16_t uidata, uint8_t * pBuff);

/// Composes a ::uint16_t from two bytes in a uint8_t array.
/// 
/// @param pBuff  The buffer has at least 2 bytes data.
/// 
/// @return uint16_t
/// @remarks This function is used to eliminates the big endian and little endian problem.
uint16_t DeChunk16_Demod(const uint8_t * pBuff);

/// Chunk four bytes \a uidata in to \a pBuff.
/// 
/// @param uidata The input 3 bytes data.
/// @param pBuff The destination buffer, at least 3 bytes length.
/// 
/// @remarks This function is used to eliminates the big endian and little endian problem.
void Chunk32_Demod(uint32_t uidata, uint8_t * pBuff);

/// Composes a ::uint16_t from four bytes in a uint8_t array.
/// 
/// @param pBuff  The buffer has at least 4 bytes data.
/// 
/// @return uint32_t
/// @remarks This function is used to eliminates the big endian and little endian problem.
uint32_t DeChunk32_Demod(const uint8_t * pBuff);

/// Chunk 3 byte of \a uiaddr into the \a pBuff
/// 
/// @param uiaddr The address. Only the three LSB bytes will be used.
/// @param pBuff The destination buffer, at lease three bytes length.
/// 
/// @remarks This function is used to eliminates the big endian and little endian problem.
void ChunkAddr_Demod(uint32_t uiaddr, uint8_t * pBuff);

/// Adds a 32-bit unsigned integer to a 64-bit unsigned integer.  Stores the result in a 64-bit unsigned integer.
///
/// @param pSum Contains the 64-bit addend.  Also carries back the resulting sum.
/// @param uiAddend Contains the 32-bit addend.
///
/// @remarks This function is an 'internal' function. Availink does not recommend that the user call it directly.
void avl_add_32to64(struct avl_uint64 *pSum, uint32_t uiAddend);

/// Divides two 64-bit unsigned integers.  Stores the result in a 64-bit unsigned integer.
///
/// @param y Contains the 64-bit divisor.  Also carries back the result.
/// @param x Contains the 64-bit dividend.
///
/// @remarks This function is an 'internal' function. Availink does not recommend that the user call it directly.
uint32_t Divide64_Demod(struct avl_uint64 y, struct avl_uint64 x);


/// Compares two 64-bit unsigned integers to determine whether the first integer is greater than or equal to the second integer.
///
/// @param a Number which is compared.
/// @param b Number against which the comparison takes place.
///
/// @return :: Returns 1 if a >= b, 0 otherwise.
///
/// @remarks This function is an 'internal' function. Availink does not recommend that the user call it directly.
uint32_t GreaterThanOrEqual64_Demod(struct avl_uint64 a, struct avl_uint64 b);

/// Subtracts a 64-bit unsigned integer (the subtrahend) from another 64-bit unsigned integer (the minuend).  Stores the result in a 64-bit unsigned integer.
///
/// @param pA Contains the 64-bit minuend.  This is the number from which the other input is subtracted.  The contents of pA must be larger than b.
/// @param b Contains the 64-bit subtrahend.  Also stores the result of the subtraction operation.  This is the number which is subtracted from the other input.  Must be smaller than the contents of pA.
///
/// @remarks This function is an 'internal' function. Availink does not recommend that the user call it directly.
void Subtract64_Demod(struct avl_uint64 *pA, struct avl_uint64 b);


/// Multiplies two 32-bit unsigned integers.  Stores the result in a 64-bit unsigned integer.
///
/// @param pDst Carries back the 64-bit product of the multiplication.
/// @param m1 Contains one of the 32-bit factors to be used in the multiplication.
/// @param m2 Contains the other 32-bit factor to be used in the multiplication.
///
/// @remarks This function is an 'internal' function. Availink does not recommend that the user call it directly.
void Multiply32_Demod(struct avl_uint64 *pDst, uint32_t m1, uint32_t m2);

/// Shifts a 32-bit unsigned integer left by 16 bits and then adds the result to a 64-bit unsigned integer.  Stores the sum in a 64-bit unsigned integer.
///
/// @param pDst Contains the 64-bit addend.  Also carries back the resulting sum.
/// @param a Contains the 32-bit input which is shifted and added to the other addend.
///
/// @remarks This function is an 'internal' function. Availink does not recommend that the user call it directly.
void AddScaled32To64_Demod(struct avl_uint64 *pDst, uint32_t a);

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


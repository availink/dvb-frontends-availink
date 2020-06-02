// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Availink AVL68x2 DVB-S/S2/T/T2/C, ISDB-T, J83.B demodulator driver
 *
 * Copyright (C) 2020 Availink, Inc. (gpl@availink.com)
 *
 */

#ifndef __avl68x2_dvbtx_h__
#define __avl68x2_dvbtx_h__

#include "avl68x2_common.h"


#define rc_DVBTx_fund_rate_Hz_iaddr_offset                       0x00000004
#define rc_DVBTx_sample_rate_Hz_iaddr_offset                     0x00000008
#define rc_DVBTx_rf_agc_pol_caddr_offset                         0x0000000f
#define rc_DVBTx_tuner_type_caddr_offset                         0x00000040
#define rc_DVBTx_input_format_caddr_offset                       0x00000041
#define rc_DVBTx_spectrum_invert_caddr_offset                    0x00000042
#define rc_DVBTx_input_select_caddr_offset                       0x00000043
#define rc_DVBTx_nom_carrier_freq_Hz_iaddr_offset                0x00000044
#define rc_DVBTx_l1_proc_only_caddr_offset                       0x00000054
#define rc_DVBTx_common_PLP_present_caddr_offset                 0x00000055
#define rc_DVBTx_common_PLP_ID_caddr_offset                      0x00000056
#define rc_DVBTx_data_PLP_ID_caddr_offset                        0x00000057
#define rc_DVBTx_dvbt_layer_select_caddr_offset                  0x0000006a
#define rc_DVBTx_acquire_mode_caddr_offset                       0x0000006b
#define rc_DVBTx_mpeg_clk_rate_Hz_iaddr_offset                   0x0000006c
#define rc_DVBTx_adc_sel_caddr_offset							 0x00000077
#define rc_DVBTx_adc_use_pll_clk_caddr_offset                    0x00000076


#define rs_DVBTx_rx_mode_caddr_offset                            0x000000d0
#define rs_DVBTx_fec_lock_caddr_offset                           0x000000d2
#define rs_DVBTx_snr_dB_x100_saddr_offset                        0x000000d6
#define rs_DVBTx_post_viterbi_BER_estimate_x10M_iaddr_offset     0x00000114
#define rs_DVBTx_post_LDPC_BER_estimate_x1B_iaddr_offset         0x00000118
#define rs_DVBTx_pre_LDPC_BER_estimate_x10M_iaddr_offset         0x0000011c
#define rs_DVBTx_plp_list_request_caddr_offset                   0x00000133

#define rs_DVBTx_data_PLP_ID_caddr_offset                   0x00000000
#define rs_DVBTx_data_PLP_TYPE_caddr_offset                 0x00000001
#define rs_DVBTx_data_PLP_COD_caddr_offset                  0x00000007
#define rs_DVBTx_data_PLP_MOD_caddr_offset                  0x00000008
#define rs_DVBTx_data_PLP_ROTATION_caddr_offset             0x00000009
#define rs_DVBTx_data_PLP_FEC_TYPE_caddr_offset             0x0000000b

#define rs_DVBTx_common_PLP_ID_caddr_offset                 0x00000000
#define rs_DVBTx_common_PLP_COD_caddr_offset                0x00000007
#define rs_DVBTx_common_PLP_MOD_caddr_offset                0x00000008
#define rs_DVBTx_common_PLP_ROTATION_caddr_offset           0x00000009
#define rs_DVBTx_common_PLP_FEC_TYPE_caddr_offset           0x0000000b

#define rs_DVBTx_P1_S2_field_2_caddr_offset                 0x00000003
#define rs_DVBTx_MISO_SISO_caddr_offset                     0x00000005
#define rs_DVBTx_T2_profile_caddr_offset                    0x00000006
#define rs_DVBTx_FFT_size_caddr_offset                      0x00000007

#define rs_DVBTx_NUM_PLP_caddr_offset                       0x00000002

#define rs_DVBTx_TPS_length_caddr_offset					0x00000000
#define rs_DVBTx_constellation_caddr_offset                 0x00000001
#define rs_DVBTx_hierarchy_caddr_offset                     0x00000002
#define rs_DVBTx_HP_code_rate_caddr_offset                  0x00000003
#define rs_DVBTx_LP_code_rate_caddr_offset                  0x00000004
#define rs_DVBTx_guard_interval_caddr_offset                0x00000005
#define rs_DVBTx_transmission_mode_caddr_offset             0x00000006
#define rs_DVBTx_TPS_cell_id_saddr_offset                   0x0000000a

#define rs_DVBTx_BWT_EXT_caddr_offset                0x00000001
#define rs_DVBTx_GUARD_INTERVAL_caddr_offset         0x00000005
#define rs_DVBTx_PAPR_caddr_offset                   0x00000006
#define rs_DVBTx_L1_MOD_caddr_offset                 0x00000007
#define rs_DVBTx_PILOT_PATTERN_caddr_offset          0x00000014
#define rs_DVBTx_CELL_ID_saddr_offset                0x00000016
#define rs_DVBTx_NETWORK_ID_saddr_offset             0x00000018
#define rs_DVBTx_T2_SYSTEM_ID_saddr_offset           0x0000001a
#define rs_DVBTx_NUM_T2_FRAMES_caddr_offset          0x0000001d
#define rs_DVBTx_NUM_DATA_SYMBOLS_saddr_offset       0x0000001e

#define rs_DVBTx_Signal_Presence_iaddr_offset        0x00000150


typedef enum AVL_DVBT2_PLP_Type
{
    AVL_DVBT2_SINGLE_PLP         =   0,
    AVL_DVBT2_MULTIPLE_PLP       =   1
}AVL_DVBT2_PLP_Type;

typedef enum AVL_DVBTxBandWidth
{
    AVL_DVBTx_BW_1M7 =   0,
    AVL_DVBTx_BW_5M  =   1,
    AVL_DVBTx_BW_6M  =   2,
    AVL_DVBTx_BW_7M  =   3,
    AVL_DVBTx_BW_8M  =   4
}AVL_DVBTxBandWidth;

typedef enum AVL_DVBTx_LockMode
{
    AVL_DVBTx_LockMode_T2BASE_T   =   0,
    AVL_DVBTx_LockMode_T2LITE_T   =   1,
    AVL_DVBTx_LockMode_T2BASE     =   2,
    AVL_DVBTx_LockMode_T2LITE     =   3,
    AVL_DVBTx_LockMode_T_ONLY     =   4,
	AVL_DVBTx_LockMode_ALL        =   5
}AVL_DVBTx_LockMode;

typedef enum AVL_DVBT_Layer
{
    AVL_DVBT_LAYER_LP    =   0,
    AVL_DVBT_LAYER_HP    =   1
}AVL_DVBT_Layer;

typedef enum AVL_DVBT_FFTSize
{
    AVL_DVBT_FFT_2K      =   0, 
    AVL_DVBT_FFT_8K      =   1,
    AVL_DVBT_FFT_UNKNOWN =   2
}AVL_DVBT_FFTSize;

typedef enum AVL_DVBT_GuardInterval
{
    AVL_DVBT_GUARD_1_32  =   0,
    AVL_DVBT_GUARD_1_16  =   1,
    AVL_DVBT_GUARD_1_8   =   2,
    AVL_DVBT_GUARD_1_4   =   3
}AVL_DVBT_GuardInterval;

typedef enum AVL_DVBT_ModulationMode
{
    AVL_DVBT_QPSK            =   0,
    AVL_DVBT_16QAM           =   1,
    AVL_DVBT_64QAM           =   2,
    AVL_DVBT_MOD_UNKNOWN     =   3
}AVL_DVBT_ModulationMode;

typedef enum AVL_DVBT_Hierarchy
{
    AVL_DVBT_HIER_NONE       =   0,
    AVL_DVBT_HIER_ALPHA_1    =   1,
    AVL_DVBT_HIER_ALPHA_2    =   2,
    AVL_DVBT_HIER_ALPHA_4    =   3
}AVL_DVBT_Hierarchy;

typedef enum AVL_DVBT_CodeRate
{
    AVL_DVBT_CR_1_2  =   0,
    AVL_DVBT_CR_2_3  =   1,
    AVL_DVBT_CR_3_4  =   2,
    AVL_DVBT_CR_5_6  =   3,
    AVL_DVBT_CR_7_8  =   4
}AVL_DVBT_CodeRate;

typedef enum AVL_DVBT2_FFTSize
{
    AVL_DVBT2_FFT_1K     =   0,
    AVL_DVBT2_FFT_2K     =   1,
    AVL_DVBT2_FFT_4K     =   2,
    AVL_DVBT2_FFT_8K     =   3,
    AVL_DVBT2_FFT_16K    =   4,
    AVL_DVBT2_FFT_32K    =   5
}AVL_DVBT2_FFTSize;

typedef enum AVL_DVBT2_MISO_SISO
{
    AVL_DVBT2_SISO   =   0,
    AVL_DVBT2_MISO   =   1
}AVL_DVBT2_MISO_SISO;


typedef enum AVL_DVBT2_PROFILE
{
    AVL_DVBT2_PROFILE_BASE    =  0,
    AVL_DVBT2_PROFILE_LITE    =  1,
    AVL_DVBT2_PROFILE_UNKNOWN =  2
}AVL_DVBT2_PROFILE;

typedef enum AVL_DVBT2_PILOT_PATTERN
{
    AVL_DVBT2_PP_PP1           = 0,
    AVL_DVBT2_PP_PP2           = 1,
    AVL_DVBT2_PP_PP3           = 2,
    AVL_DVBT2_PP_PP4           = 3,
    AVL_DVBT2_PP_PP5           = 4,
    AVL_DVBT2_PP_PP6           = 5,
    AVL_DVBT2_PP_PP7           = 6,
    AVL_DVBT2_PP_PP8           = 7,
    AVL_DVBT2_PP_DVBT          = 8,
    AVL_DVBT2_PP_DVBT_REVERSE  = 9,
    AVL_DVBT2_PP_UNKNOWN       = 10
}AVL_DVBT2_PILOT_PATTERN;

typedef enum AVL_DVBT2_DATA_PLP_TYPE
{
    AVL_DVBT2_DATA_PLP_TYPE1 =   1,
    AVL_DVBT2_DATA_PLP_TYPE2 =   2
}AVL_DVBT2_DATA_PLP_TYPE;

typedef enum AVL_DVBT2_CodeRate
{
    AVL_DVBT2_CR_1_2 = 0,
    AVL_DVBT2_CR_3_5 = 1,
    AVL_DVBT2_CR_2_3 = 2,
    AVL_DVBT2_CR_3_4 = 3,
    AVL_DVBT2_CR_4_5 = 4,
    AVL_DVBT2_CR_5_6 = 5,
    AVL_DVBT2_CR_1_3 = 6,
    AVL_DVBT2_CR_2_5 = 7
}AVL_DVBT2_CodeRate;

typedef enum AVL_DVBT2_PLP_ModulationMode
{
    AVL_DVBT2_QPSK   = 0,
    AVL_DVBT2_16QAM  = 1,
    AVL_DVBT2_64QAM  = 2, 
    AVL_DVBT2_256QAM = 3
}AVL_DVBT2_PLP_ModulationMode;

typedef enum AVL_DVBT2_L1_Modulation
{
    AVL_DVBT2_L1_BPSK = 0,
    AVL_DVBT2_L1_QPSK = 1,
	AVL_DVBT2_L1_16QAM = 2,
	AVL_DVBT2_L1_64QAM = 3
}AVL_DVBT2_L1_Modulation;

typedef enum AVL_DVBT2_PLP_Constellation_Rotation
{
    AVL_DVBT2_PLP_NOT_ROTATION   =   0,
    AVL_DVBT2_PLP_ROTATION       =   1
}AVL_DVBT2_PLP_Constellation_Rotation;

typedef enum AVL_DVBT2_PLP_FEC_Type
{
    AVL_DVBT2_FEC_LDPC16K    =   0,
    AVL_DVBT2_FEC_LDPC64K    =   1
}AVL_DVBT2_PLP_FEC_Type;

typedef enum AVL_DVBTx_Standard
{
    AVL_DVBTx_Standard_T     =   0,                  //the DVB-T standard
    AVL_DVBTx_Standard_T2    =   1                   //the DVB-T2 standard
}AVL_DVBTx_Standard;    

typedef enum AVL_DVBT2_PAPR
{
    AVL_DVBT2_PAPR_NONE       =     0,
    AVL_DVBT2_PAPR_ACE        =     1,
    AVL_DVBT2_PAPR_TR         =     2,
    AVL_DVBT2_PAPR_BOTH       =     3
}AVL_DVBT2_PAPR;

typedef enum AVL_DVBT2_GUARD_INTERVAL
{
    AVL_DVBT2_GI_1_32         =     0,
    AVL_DVBT2_GI_1_16         =     1,
    AVL_DVBT2_GI_1_8          =     2,
    AVL_DVBT2_GI_1_4          =     3,
    AVL_DVBT2_GI_1_128        =     4,
    AVL_DVBT2_GI_19_128       =     5,
    AVL_DVBT2_GI_19_256       =     6
}AVL_DVBT2_GUARD_INTERVAL;

typedef struct AVL_DVBT2_SignalID
{
    uint16_t usCellID;
    uint16_t usNetworkID;
    uint16_t usSystemID;
}AVL_DVBT2_SignalID;

typedef struct AVL_DVBT_SignalInfo
{
    AVL_DVBT_FFTSize        eDVBTFFTSize;
    AVL_DVBT_GuardInterval  eDVBTGuardInterval;
    AVL_DVBT_ModulationMode eDVBTModulationMode;
    AVL_DVBT_Hierarchy      eDVBTHierarchy;
    AVL_DVBT_CodeRate       eDVBTHPCodeRate;
    AVL_DVBT_CodeRate       eDVBTLPCodeRate;
}AVL_DVBT_SignalInfo;

typedef struct AVL_DVBT2_SignalInfo
{
    AVL_DVBT2_FFTSize                      eDVBT2FFTSize;
    AVL_DVBT2_MISO_SISO                    eDVBT2MISOorSISO;
    AVL_DVBT2_PROFILE                      eDVBT2Profile;
    AVL_DVBT2_PILOT_PATTERN                eDVBT2PilotPatten;
    uint8_t                              ucDVBT2DataPLPID;
    AVL_DVBT2_DATA_PLP_TYPE                eDVBT2DataPLPType;
    AVL_DVBT2_CodeRate                     eDVBT2DataPLPCodeRate;
    AVL_DVBT2_PLP_ModulationMode           eDVBT2DataPLPModulationMode;
    AVL_DVBT2_PLP_Constellation_Rotation   eDVBT2DataPLPRotation;
    AVL_DVBT2_PLP_FEC_Type                 eDVBT2DataPLPFecType;
    uint8_t                              ucDVBT2CommonPLPExist;
    uint8_t                              ucDVBT2CommonPLPID;
    AVL_DVBT2_CodeRate                     eDVBT2CommonPLPCodeRate;
    AVL_DVBT2_PLP_ModulationMode           eDVBT2CommonPLPModulationMode;
    AVL_DVBT2_PLP_Constellation_Rotation   eDVBT2CommonPLPRotation;
    AVL_DVBT2_PLP_FEC_Type                 eDVBT2CommonPLPFecType;
    uint8_t                              ucDVBT2FEFExist;
	AVL_DVBT2_SignalID                     eDVBT2SignalID;
	AVL_DVBT2_L1_Modulation                eDVBT2L1ModulationMode;
	AVL_DVBT2_PAPR                         eDVBT2PAPR;
	AVL_DVBT2_GUARD_INTERVAL               eDVBT2GuardInterval;
	uint8_t                              ucDVBT2BWExtended;
	uint8_t							   ucNumberDPLP;
	uint8_t							   ucNumberFrames;
	uint16_t							   usNumberDataSymbols;
}AVL_DVBT2_SignalInfo;


typedef struct AVL_DVBTxModulationInfo
{
    AVL_DVBTx_Standard      ucDVBxStandard;
    AVL_DVBT_SignalInfo     eDVBTSignalInfo;
    AVL_DVBT2_SignalInfo    eDVBT2SignalInfo;
}AVL_DVBTxModulationInfo;
    
 /// Holds the DVB-C channel information.The structure contains the parameters used for locking to a channel.
typedef struct AVL_DVBTx_Channel
{
    AVL_DVBTxBandWidth      eDVBTxBandWith;                       ///< The symbol rate in units of Hz. 
    avl68x2_spectrum_polarity   eSpectrumInversion;
    AVL_DVBTx_LockMode      eDVBTxLockMode;
    AVL_DVBT_Layer          eDVBTLayer;
    uint8_t               ucDVBT2DataPLPID;
    uint8_t               ucDVBT2CommonPLPID;
    uint8_t               ucDVBT2CommonPLPPresent;
}AVL_DVBTx_Channel;

typedef struct AVL_DVBTxScanInfo
{
    AVL_DVBTx_Standard eTxStandard;   //T OR T2
    uint8_t          ucTxInfo;     // for T2(base or lite); for T
    uint8_t          ucFEFInfo;   //only for T2
}AVL_DVBTxScanInfo;

typedef struct AVL_DVBT_TpsInfo
{
	uint8_t ucTpsLength;
    AVL_DVBT_ModulationMode eTpsConstellation;
    AVL_DVBT_Hierarchy		eTpsHierarchy;
    AVL_DVBT_CodeRate       eTpsHPCodeRate;
    AVL_DVBT_CodeRate       eTpsLPCodeRate;
	AVL_DVBT_GuardInterval  eTpsGuardInterval;
	AVL_DVBT_FFTSize		eTpsTransmissionMode;
	uint16_t usTpsCellID;

}AVL_DVBT_TpsInfo;

avl_error_code_t AVL_Demod_DVBTxChannelScan(AVL_DVBTxBandWidth eBandWidth, AVL_DVBTx_LockMode eLockMode, avl68x2_chip *chip);
avl_error_code_t AVL_Demod_DVBTxGetScanInfo(AVL_DVBTxScanInfo* stDVBTxScanInfo, avl68x2_chip *chip);
avl_error_code_t AVL_Demod_DVBT2AutoLock(AVL_DVBTxBandWidth eBandWidth, AVL_DVBT2_PROFILE eDVTB2Profile, uint8_t ucDVBT2PLPID, avl68x2_chip *chip);
avl_error_code_t AVL_Demod_DVBT2GetPLPList(uint8_t * pucPLPIndexArray, uint8_t * pucPLPNumber, avl68x2_chip *chip);
avl_error_code_t AVL_Demod_DVBTxGetNorDigSSI(uint8_t *pucSSI, int32_t iRFPowerdBm, avl68x2_chip *chip);
avl_error_code_t AVL_Demod_DVBTxSignalDetection(uint8_t *pucNoSig, avl68x2_chip *chip);
avl_error_code_t AVL_Demod_DVBTAutoLock(AVL_DVBTxBandWidth eBandWidth, uint8_t ucDVBTLayer, avl68x2_chip *chip);
avl_error_code_t AVL_Demod_DVBTxGetModulationInfo(AVL_DVBTxModulationInfo *pstModulationInfo, avl68x2_chip *chip);
avl_error_code_t DVBTx_Initialize_Demod(avl68x2_chip *chip);
avl_error_code_t DVBTx_GetLockStatus_Demod(uint8_t * pucLocked, avl68x2_chip *chip);
avl_error_code_t DVBTx_GetSNR_Demod(uint32_t * puiSNR_db, avl68x2_chip *chip);
avl_error_code_t DVBTx_GetSignalQuality_Demod(uint16_t * puiQuality , avl68x2_chip *chip);
avl_error_code_t IRx_GetSQI(uint32_t * puiSQI, avl68x2_chip *chip);
avl_error_code_t IRx_GetSSI(uint32_t * puiSSI, int32_t RF_Power, avl68x2_chip *chip);
avl_error_code_t IRx_GetSQI_DVBT(uint32_t * puiSQI, avl68x2_chip *chip);
avl_error_code_t IRx_GetSQI_DVBT2(uint32_t * puiSQI, avl68x2_chip *chip);
avl_error_code_t IRx_GetSSI_DVBT(uint32_t * puiSSI, int32_t RF_Power, avl68x2_chip *chip);
avl_error_code_t IRx_GetSSI_DVBT2(uint32_t * puiSSI, int32_t RF_Power, avl68x2_chip *chip);
avl_error_code_t DVBTx_SetIFInputPath_Demod(AVL_InputPath eInputPath, avl68x2_chip *chip);
avl_error_code_t DVBTx_SetIFFrequency_Demod(uint32_t uiIFFrequencyHz, avl68x2_chip *chip);
avl_error_code_t DVBTx_SetBandWidth_Demod(AVL_DVBTxBandWidth eBandWidth, avl68x2_chip *chip);
avl_error_code_t DVBTx_GetPrePostBER_Demod(uint32_t *puiBERxe9, AVL_BER_Type eBERType, avl68x2_chip *chip);
avl_error_code_t AVL_Demod_DVBT_GetTPSInfo(AVL_DVBT_TpsInfo *pstDVBTTpsInfo, avl68x2_chip *chip);
avl_error_code_t DVBTx_GetCellID_Demod(uint16_t *puiCellID,  avl68x2_chip *chip);



#endif



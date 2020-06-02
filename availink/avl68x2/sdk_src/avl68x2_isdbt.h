// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Availink AVL68x2 DVB-S/S2/T/T2/C, ISDB-T, J83.B demodulator driver
 *
 * Copyright (C) 2020 Availink, Inc. (gpl@availink.com)
 *
 */
#ifndef __avl68x2_isdbt_h__
#define __avl68x2_isdbt_h__

#include "avl68x2_common.h"


#define rc_ISDBT_fund_rate_Hz_iaddr_offset                                0x00000004
#define rc_ISDBT_rf_agc_pol_caddr_offset                                  0x0000000f
#define rc_ISDBT_sample_rate_Hz_iaddr_offset                              0x00000008
#define rc_ISDBT_tuner_type_caddr_offset                                  0x00000040
#define rc_ISDBT_input_format_caddr_offset                                0x00000041
#define rc_ISDBT_input_select_caddr_offset                                0x00000043
#define rc_ISDBT_nom_carrier_freq_Hz_iaddr_offset                         0x00000044
#define rc_ISDBT_TS_clk_rate_Hz_iaddr_offset                              0x0000006c
#define rc_ISDBT_adc_sel_caddr_offset                                     0x00000077
#define rc_ISDBT_adc_use_pll_clk_caddr_offset                             0x00000076
#define rc_ISDBT_layer_out_auto_filt_caddr_offset                         0x00000082
#define rc_ISDBT_mult_layer_op_mode_caddr_offset                          0x00000083

#define rs_ISDBT_mode_caddr_offset                                        0x00000001
#define rs_ISDBT_system_type_caddr_offset                                 0x00000002
#define rs_ISDBT_GI_saddr_offset                                          0x0000000c
#define rs_ISDBT_layA_constel_size_caddr_offset                           0x0000001c
#define rs_ISDBT_layA_fec_rate_caddr_offset                               0x0000001d
#define rs_ISDBT_layA_itlv_len_caddr_offset                               0x0000001e
#define rs_ISDBT_layA_seg_no_caddr_offset                                 0x0000001f
#define rs_ISDBT_partial_reception_caddr_offset                           0x00000023
#define rs_ISDBT_layB_constel_size_caddr_offset                           0x00000024
#define rs_ISDBT_layB_fec_rate_caddr_offset                               0x00000025
#define rs_ISDBT_layB_itlv_len_caddr_offset                               0x00000026
#define rs_ISDBT_layB_seg_no_caddr_offset                                 0x00000027
#define rs_ISDBT_layC_constel_size_caddr_offset                           0x00000028
#define rs_ISDBT_layC_fec_rate_caddr_offset                               0x00000029
#define rs_ISDBT_layC_itlv_len_caddr_offset                               0x0000002a
#define rs_ISDBT_layC_seg_no_caddr_offset                                 0x0000002b
#define rs_ISDBT_fec_lock_caddr_offset                                    0x000000ce
#define rs_ISDBT_snr_dB_x100_saddr_offset                                 0x000000d2
#define rs_ISDBT_ISDBT_sys_state_iaddr_offset                             0x000000e0
#define rs_ISDBT_Signal_Presence_iaddr_offset                             0x0000010c
#define rs_ISDBT_layA_tot_pkts_iaddr_offset                               0x00000110
#define rs_ISDBT_layA_err_pkts_iaddr_offset                               0x00000114
#define rs_ISDBT_layB_tot_pkts_iaddr_offset                               0x00000118
#define rs_ISDBT_layB_err_pkts_iaddr_offset                               0x0000011c
#define rs_ISDBT_layC_tot_pkts_iaddr_offset                               0x00000120
#define rs_ISDBT_layC_err_pkts_iaddr_offset                               0x00000124

#define rs_errstat_clear__offset                                          0x0000002c /* type = RW */


	typedef enum AVL_ISDBT_Standard
	{
		AVL_ISDBT_FULL_SEG   =  0,
		AVL_ISDBT_ONE_SEG    =  1,
		AVL_ISDBT_THREE_SEG  =  2
	}AVL_ISDBT_Standard;

	typedef enum AVL_ISDBT_Layer
	{
		AVL_ISDBT_LAYER_ALL = 0,
		AVL_ISDBT_LAYER_A = 1,
		AVL_ISDBT_LAYER_B = 2,
		AVL_ISDBT_LAYER_C = 3
	}AVL_ISDBT_Layer;

	typedef enum AVL_ISDBT_SystemType
	{
		AVL_ISDBT_Std = 0,
		AVL_ISDBTsb_1seg = 1,
		AVL_ISDBTsb_3seg = 2
	}AVL_ISDBT_SystemType;

	typedef enum AVL_ISDBT_Mode
	{
		AVL_ISDBT_Mode1 = 0,
		AVL_ISDBT_Mode2 = 1,
		AVL_ISDBT_Mode3 = 2
	}AVL_ISDBT_Mode;

	typedef enum AVL_ISDBT_GuardInterval
	{
		AVL_ISDBT_GUARD_1_32 =   0,
		AVL_ISDBT_GUARD_1_16 =   1,
		AVL_ISDBT_GUARD_1_8  =   2,
		AVL_ISDBT_GUARD_1_4  =   3
	}AVL_ISDBT_GuardInterval;

	typedef enum AVL_ISDBT_PartialReception
	{
		AVL_ISDBT_PartialReception_Off = 0,
		AVL_ISDBT_PartialReception_On = 1
	}AVL_ISDBT_PartialReception;

	typedef enum AVL_ISDBT_CodeRate
	{
		AVL_ISDBT_CR_1_2 = 0,
		AVL_ISDBT_CR_2_3 = 1,
		AVL_ISDBT_CR_3_4 = 2,
		AVL_ISDBT_CR_5_6 = 3,
		AVL_ISDBT_CR_7_8 = 4
	}AVL_ISDBT_CodeRate;

	typedef enum AVL_ISDBT_ModulationMode
	{
		AVL_ISDBT_DQPSK  = 0,
		AVL_ISDBT_QPSK   = 1,
		AVL_ISDBT_16QAM  = 2,
		AVL_ISDBT_64QAM  = 3
	}AVL_ISDBT_ModulationMode;

	typedef struct AVL_ISDBT_LayerSignalInfo
	{
		AVL_ISDBT_ModulationMode  eISDBTModulationMode;
		AVL_ISDBT_CodeRate        eISDBTCodeRate;
		uint8_t                 ucISDBTInterleaverLen;
		uint8_t                 ucISDBTSegmentNum;
	}AVL_ISDBT_LayerSignalInfo;

	typedef struct AVL_ISDBTModulationInfo
	{
		AVL_ISDBT_SystemType       eISDBTSystemType;
		AVL_ISDBT_Mode             eISDBTMode;
		AVL_ISDBT_GuardInterval    eISDBTGuardInterval;
		AVL_ISDBT_PartialReception eISDBTPartialReception;
		AVL_ISDBT_LayerSignalInfo  eISDBTLayerA;
		AVL_ISDBT_LayerSignalInfo  eISDBTLayerB;
		AVL_ISDBT_LayerSignalInfo  eISDBTLayerC;
	}AVL_ISDBTModulationInfo;

	typedef struct AVL_ISDBTLayerErrorStats
	{
		struct avl_uint64 stSwCntNumPkts;     // A software counter which stores the number of packets which have been received.
		struct avl_uint64 stSwCntPktErrors;   // A software counter which stores the number of packet errors which have been detected.
		struct avl_uint64 stNumPkts;          // The total number of packets which have been received.
		struct avl_uint64 stPktErrors;        // The total number of packet errors which have been detected.
		uint32_t uiPER;             // The packet error rate scaled by 1e9.
	}AVL_ISDBTLayerErrorStats;
    avl_error_code_t ISDBT_GetEWBSChangeFlag_Demod(uint8_t * pucEWBSChangeFlag,avl68x2_chip *chip);
    avl_error_code_t ISDBT_GetEWBS_Demod(uint8_t * pucEWBS,avl68x2_chip *chip);
    avl_error_code_t ISDBT_Reset_EWBSChangeFlag_Demod(avl68x2_chip *chip);

	avl_error_code_t AVL_Demod_ISDBTAutoLock(AVL_ISDBT_BandWidth bw, avl68x2_chip *chip);
	avl_error_code_t AVL_Demod_ISDBTGetModulationInfo(AVL_ISDBTModulationInfo *pstModulationInfo, avl68x2_chip *chip);

	avl_error_code_t ISDBT_Initialize_Demod(avl68x2_chip *chip);
	avl_error_code_t ISDBT_GetLockStatus_Demod( uint8_t * pucLocked, avl68x2_chip *chip );
	avl_error_code_t ISDBT_GetSignalQuality_Demod(uint16_t * puiQuality , avl68x2_chip *chip );
	avl_error_code_t ISDBT_GetSNR_Demod( uint32_t * puiSNR_db, avl68x2_chip *chip );
	avl_error_code_t ISDBT_GetSQI(uint32_t * puiSQI, avl68x2_chip *chip);
	avl_error_code_t ISDBT_SetIFInputPath_Demod(AVL_InputPath eInputPath, avl68x2_chip *chip);
	avl_error_code_t ISDBT_SetIFFrequency_Demod(uint32_t uiIFFrequencyHz, avl68x2_chip *chip);
	avl_error_code_t ISDBT_SetBandWidth_Demod(AVL_ISDBT_BandWidth eISDBTBandWidth, avl68x2_chip *chip);
	avl_error_code_t ISDBT_GetPrePostBER_Demod(uint32_t *puiBERxe9, AVL_BER_Type eBERType, avl68x2_chip *chip);
	avl_error_code_t ISDBT_GetSignalDetection(uint8_t *pucNoSig, avl68x2_chip *chip);
	avl_error_code_t ISDBT_ResetLayerPER_Demod(avl68x2_chip *chip);
	avl_error_code_t ISDBT_GetLayerPER_Demod(uint32_t *puiPERxe9, enum AVL_ISDBT_Layer eLayerNum, avl68x2_chip *chip);


#endif


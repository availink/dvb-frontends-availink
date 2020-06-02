// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Availink AVL68x2 DVB-S/S2/T/T2/C, ISDB-T, J83.B demodulator driver
 *
 * Copyright (C) 2020 Availink, Inc. (gpl@availink.com)
 *
 */
#include "avl68x2_common.h"
#include "avl68x2_isdbt.h"

struct AVL_ISDBTLayerErrorStats gstISDBTLayerA_ErrorStats, gstISDBTLayerB_ErrorStats, gstISDBTLayerC_ErrorStats;

static int BW_FFT_Table[2]=
  {
    8126984,    //bw=6.0MHz
    10835979    //bw=8.0MHz
  };

avl_error_code_t AVL_Demod_ISDBTAutoLock(
	AVL_ISDBT_BandWidth bw,
avl68x2_chip *chip)
{
	avl_error_code_t r = AVL_EC_OK;

	if (1 == chip->chip_priv->sleep_flag)
	{
		r = AVL_EC_SLEEP;
		return r;
	}
	r = IBase_SendRxOPWait_Demod(AVL_FW_CMD_HALT, chip);

	r |= ISDBT_SetBandWidth_Demod(bw,chip);
	r |= ISDBT_SetIFFrequency_Demod(
	    (uint32_t)chip->chip_pub->isdbt_config.uiISDBTIFFreqHz,
	    chip);

	// Enable CS_0 as GPIO for EWBS feature
	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     stBaseAddrSet.hw_emerald_io_base +
				 emerald_io_pad_CS_0_sel_offset,
			     0x1);

	// for ISDBT Layer independence lock
	r |= avl_bms_write8(chip->chip_pub->i2c_addr,
			    stBaseAddrSet.fw_ISDBT_config_reg_base +
				rc_ISDBT_mult_layer_op_mode_caddr_offset,
			    0x01);

	r |= IBase_SendRxOPWait_Demod(AVL_FW_CMD_ACQUIRE, chip);

	return (r);
}

avl_error_code_t AVL_Demod_ISDBTGetModulationInfo(AVL_ISDBTModulationInfo *pstModulationInfo, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint8_t ucTemp = 0;
    uint16_t usTemp = 0;
    static unsigned int fsba = 0;

    if(fsba == 0) 
    {
        avl_bms_read32(chip->chip_pub->i2c_addr, stBaseAddrSet.fw_ISDBT_status_reg_base + rs_ISDBT_ISDBT_sys_state_iaddr_offset, &fsba);
    }

    r = avl_bms_read8(chip->chip_pub->i2c_addr, fsba + rs_ISDBT_system_type_caddr_offset, &ucTemp);
    pstModulationInfo->eISDBTSystemType = (AVL_ISDBT_SystemType)ucTemp;

    r |= avl_bms_read8(chip->chip_pub->i2c_addr, fsba + rs_ISDBT_mode_caddr_offset, &ucTemp);
    pstModulationInfo->eISDBTMode = (AVL_ISDBT_Mode)ucTemp;

    r |= avl_bms_read16(chip->chip_pub->i2c_addr, fsba + rs_ISDBT_GI_saddr_offset, &usTemp);
    pstModulationInfo->eISDBTGuardInterval = (AVL_ISDBT_GuardInterval)usTemp;

    r |= avl_bms_read8(chip->chip_pub->i2c_addr, fsba + rs_ISDBT_partial_reception_caddr_offset, &ucTemp);
    pstModulationInfo->eISDBTPartialReception = (AVL_ISDBT_PartialReception)ucTemp;

    r |= avl_bms_read8(chip->chip_pub->i2c_addr, fsba + rs_ISDBT_layA_constel_size_caddr_offset, &ucTemp);
    pstModulationInfo->eISDBTLayerA.eISDBTModulationMode = (AVL_ISDBT_ModulationMode)ucTemp;
    r |= avl_bms_read8(chip->chip_pub->i2c_addr, fsba + rs_ISDBT_layA_fec_rate_caddr_offset, &ucTemp);
    pstModulationInfo->eISDBTLayerA.eISDBTCodeRate = (AVL_ISDBT_CodeRate)ucTemp;
    r |= avl_bms_read8(chip->chip_pub->i2c_addr, fsba + rs_ISDBT_layA_itlv_len_caddr_offset, &ucTemp);
    pstModulationInfo->eISDBTLayerA.ucISDBTInterleaverLen = ucTemp;
    r |= avl_bms_read8(chip->chip_pub->i2c_addr, fsba + rs_ISDBT_layA_seg_no_caddr_offset, &ucTemp);
    pstModulationInfo->eISDBTLayerA.ucISDBTSegmentNum = ucTemp;

    r |= avl_bms_read8(chip->chip_pub->i2c_addr, fsba + rs_ISDBT_layB_constel_size_caddr_offset, &ucTemp);
    pstModulationInfo->eISDBTLayerB.eISDBTModulationMode = (AVL_ISDBT_ModulationMode)ucTemp;
    r |= avl_bms_read8(chip->chip_pub->i2c_addr, fsba + rs_ISDBT_layB_fec_rate_caddr_offset, &ucTemp);
    pstModulationInfo->eISDBTLayerB.eISDBTCodeRate = (AVL_ISDBT_CodeRate)ucTemp;
    r |= avl_bms_read8(chip->chip_pub->i2c_addr, fsba + rs_ISDBT_layB_itlv_len_caddr_offset, &ucTemp);
    pstModulationInfo->eISDBTLayerB.ucISDBTInterleaverLen = ucTemp;
    r |= avl_bms_read8(chip->chip_pub->i2c_addr, fsba + rs_ISDBT_layB_seg_no_caddr_offset, &ucTemp);
    pstModulationInfo->eISDBTLayerB.ucISDBTSegmentNum = ucTemp;

    r |= avl_bms_read8(chip->chip_pub->i2c_addr, fsba + rs_ISDBT_layC_constel_size_caddr_offset, &ucTemp);
    pstModulationInfo->eISDBTLayerC.eISDBTModulationMode = (AVL_ISDBT_ModulationMode)ucTemp;
    r |= avl_bms_read8(chip->chip_pub->i2c_addr, fsba + rs_ISDBT_layC_fec_rate_caddr_offset, &ucTemp);
    pstModulationInfo->eISDBTLayerC.eISDBTCodeRate = (AVL_ISDBT_CodeRate)ucTemp;
    r |= avl_bms_read8(chip->chip_pub->i2c_addr, fsba + rs_ISDBT_layC_itlv_len_caddr_offset, &ucTemp);
    pstModulationInfo->eISDBTLayerC.ucISDBTInterleaverLen = ucTemp;
    r |= avl_bms_read8(chip->chip_pub->i2c_addr, fsba + rs_ISDBT_layC_seg_no_caddr_offset, &ucTemp);
    pstModulationInfo->eISDBTLayerC.ucISDBTSegmentNum = ucTemp;

    return (r);
}

avl_error_code_t ISDBT_Initialize_Demod(avl68x2_chip *chip)
{
	avl_error_code_t r = AVL_EC_OK;

	r = avl_bms_write32(chip->chip_pub->i2c_addr,
			    stBaseAddrSet.fw_ISDBT_config_reg_base +
				rc_ISDBT_sample_rate_Hz_iaddr_offset,
			    chip->uiADCFrequencyHz);
	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     stBaseAddrSet.fw_ISDBT_config_reg_base +
				 rc_ISDBT_TS_clk_rate_Hz_iaddr_offset,
			     chip->uiTSFrequencyHz);

	//DDC configuration
	r |= avl_bms_write8(chip->chip_pub->i2c_addr,
			    stBaseAddrSet.fw_ISDBT_config_reg_base +
				rc_ISDBT_input_format_caddr_offset,
			    AVL_OFFBIN); //ADC in
	r |= avl_bms_write8(chip->chip_pub->i2c_addr,
			    stBaseAddrSet.fw_ISDBT_config_reg_base +
				rc_ISDBT_input_select_caddr_offset,
			    AVL_ADC_IN); //RX_OFFBIN
	r |= avl_bms_write8(chip->chip_pub->i2c_addr,
			    stBaseAddrSet.fw_ISDBT_config_reg_base +
				rc_ISDBT_tuner_type_caddr_offset,
			    chip->chip_pub->tc_tuner_type);

	r |= avl_bms_write8(chip->chip_pub->i2c_addr,
			    stBaseAddrSet.fw_ISDBT_config_reg_base +
				rc_ISDBT_rf_agc_pol_caddr_offset,
			    chip->chip_pub->isdbt_config.eISDBTAGCPola);

	r |= ISDBT_SetIFFrequency_Demod(
	    chip->chip_pub->isdbt_config.uiISDBTIFFreqHz,
	    chip);
	r |= ISDBT_SetIFInputPath_Demod(
	    (AVL_InputPath)(chip->chip_pub->isdbt_config.eISDBTInputPath ^ 1),
	    chip);

	//ADC configuration
	switch (chip->chip_pub->xtal)
	{
	case Xtal_16M:
	case Xtal_24M:
	{
		r |= avl_bms_write8(chip->chip_pub->i2c_addr,
				    stBaseAddrSet.fw_ISDBT_config_reg_base +
					rc_ISDBT_adc_use_pll_clk_caddr_offset,
				    1);
	}
	break;

	case Xtal_30M:
	case Xtal_27M:
	{
		r |= avl_bms_write8(chip->chip_pub->i2c_addr,
				    stBaseAddrSet.fw_ISDBT_config_reg_base +
					rc_ISDBT_adc_use_pll_clk_caddr_offset,
				    0);
	}
	break;
	}

	r |= ConfigAGCOutput_Demod(chip);

	return (r);
}

avl_error_code_t ISDBT_GetLockStatus_Demod( uint8_t * pucLocked, avl68x2_chip *chip )
{
    avl_error_code_t r = AVL_EC_OK;

    r = avl_bms_read8(chip->chip_pub->i2c_addr, stBaseAddrSet.fw_ISDBT_status_reg_base + rs_ISDBT_fec_lock_caddr_offset, pucLocked);

    return (r);
}

avl_error_code_t ISDBT_GetSignalQuality_Demod(uint16_t * puiQuality , avl68x2_chip *chip )
{
    avl_error_code_t r = AVL_EC_OK;
    uint32_t uiTemp = 0;
    
    r = ISDBT_GetSQI(&uiTemp, chip);
    
    *puiQuality = (uint16_t)uiTemp;//uiTemp's range is 0 to 100. It's safe here.
    
    return (r);

}

avl_error_code_t ISDBT_GetEWBS_Demod(uint8_t * pucEWBS, avl68x2_chip *chip)
{
    avl_error_code_t  r = AVL_EC_OK;
    uint32_t   uiBaseAddr = 0; 


    r = avl_bms_read32(chip->chip_pub->i2c_addr, 0x8e0, &uiBaseAddr);
    r |= avl_bms_read8(chip->chip_pub->i2c_addr, uiBaseAddr + 0x36, pucEWBS); //1--Turn on Alert Broadcasting;  0-- Turn off Alert Broadcasting
    return r;
}

avl_error_code_t ISDBT_GetEWBSChangeFlag_Demod(uint8_t * pucEWBSChangeFlag, avl68x2_chip *chip)
{
    avl_error_code_t  r = AVL_EC_OK;
    uint32_t   uiBaseAddr = 0;

  
    r = avl_bms_read32(chip->chip_pub->i2c_addr, 0x8e0, &uiBaseAddr);
    r |=  avl_bms_read8(chip->chip_pub->i2c_addr, uiBaseAddr + 0x37, pucEWBSChangeFlag);//1--EWBS Flag changed; 0--EWBS Flag unchanged.
    return r;
}

avl_error_code_t ISDBT_Reset_EWBSChangeFlag_Demod(avl68x2_chip *chip)
{
    avl_error_code_t  r = AVL_EC_OK;
    uint32_t   uiBaseAddr = 0;

  
    r = avl_bms_read32(chip->chip_pub->i2c_addr, 0x8e0, &uiBaseAddr);
    r |=  avl_bms_write8(chip->chip_pub->i2c_addr, uiBaseAddr + 0x37, 0);
    return r;
}


avl_error_code_t ISDBT_GetSNR_Demod( uint32_t * puiSNR_db, avl68x2_chip *chip )
{
    avl_error_code_t r = AVL_EC_OK;
    uint16_t uiTemp = 0;

    r = avl_bms_read16(chip->chip_pub->i2c_addr, stBaseAddrSet.fw_ISDBT_status_reg_base + rs_ISDBT_snr_dB_x100_saddr_offset,&uiTemp);

    *puiSNR_db = (uint32_t)uiTemp;

    return (r);
}

typedef struct AVL_ISDBT_CN_Table_Element
{
    AVL_ISDBT_ModulationMode modulation;
    AVL_ISDBT_CodeRate code_rate;
    int32_t CN_NordigP1_x100_db;
}AVL_ISDBT_CN_Table_Element;

AVL_ISDBT_CN_Table_Element AVL_ISDBT_CN_Table[]=
{
    //Gaussian
    {AVL_ISDBT_DQPSK, AVL_ISDBT_CR_1_2, 510},//FIXME, CN needs to be corrected
    {AVL_ISDBT_DQPSK, AVL_ISDBT_CR_2_3, 690},//FIXME
    {AVL_ISDBT_DQPSK, AVL_ISDBT_CR_3_4, 790},//FIXME
    {AVL_ISDBT_DQPSK, AVL_ISDBT_CR_5_6, 890},//FIXME
    {AVL_ISDBT_DQPSK, AVL_ISDBT_CR_7_8, 970},//FIXME
    
    {AVL_ISDBT_QPSK, AVL_ISDBT_CR_1_2, 510},
    {AVL_ISDBT_QPSK, AVL_ISDBT_CR_2_3, 690},
    {AVL_ISDBT_QPSK, AVL_ISDBT_CR_3_4, 790},
    {AVL_ISDBT_QPSK, AVL_ISDBT_CR_5_6, 890},
    {AVL_ISDBT_QPSK, AVL_ISDBT_CR_7_8, 970},

    {AVL_ISDBT_16QAM, AVL_ISDBT_CR_1_2, 1080},
    {AVL_ISDBT_16QAM, AVL_ISDBT_CR_2_3, 1310},
    {AVL_ISDBT_16QAM, AVL_ISDBT_CR_3_4, 1460},
    {AVL_ISDBT_16QAM, AVL_ISDBT_CR_5_6, 1560},
    {AVL_ISDBT_16QAM, AVL_ISDBT_CR_7_8, 1600},

    {AVL_ISDBT_64QAM, AVL_ISDBT_CR_1_2, 1650},
    {AVL_ISDBT_64QAM, AVL_ISDBT_CR_2_3, 1870},
    {AVL_ISDBT_64QAM, AVL_ISDBT_CR_3_4, 2020},
    {AVL_ISDBT_64QAM, AVL_ISDBT_CR_5_6, 2160},
    {AVL_ISDBT_64QAM, AVL_ISDBT_CR_7_8, 2250}
};

typedef struct AVL_ISDBT_BERSQI_List
{
    uint32_t                 m_ber;
    uint32_t                 m_ber_sqi;
}AVL_ISDBT_BERSQI_List;

AVL_ISDBT_BERSQI_List ISDBT_BERSQI_Table[]=
{
    {1000      ,    60  },
    {1778      ,    65  },
    {3162      ,    70  },
    {5623      ,    75  },
    {10000     ,    80  },
    {17783     ,    85  },
    {31623     ,    90  },
    {56234     ,    95  },
    {100000    ,    100 },
    {177828    ,    105 },
    {316228    ,    110 },
    {562341    ,    115 },
    {1000000   ,    120 },
    {1778279   ,    125 },
    {3162278   ,    130 },
    {5623413   ,    135 },
    {10000000  ,    140 }
};

avl_error_code_t ISDBT_GetSQI(uint32_t * puiSQI, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint32_t post_viterbi_ber_x1e9 = 0;
    uint32_t ber_sqi = 0;
    int32_t CN_NordigP1_x100_db = 0;
    uint32_t CN_received_x100_db = 0;
    int32_t CN_relative_x100_db = 0;

    AVL_ISDBTModulationInfo ISDBTSignalInfo;

    AVL_ISDBT_ModulationMode modulation = AVL_ISDBT_DQPSK;
    AVL_ISDBT_CodeRate code_rate = AVL_ISDBT_CR_1_2;
    uint32_t uiTemp0 = 0;
    uint32_t uiTemp1 = 0;
    uint16_t index = 0;    

    //tweaked: This will cause the SQI to be a meaningful indicator at and around the "60s clean" C/N threshold.
    r = GetBER_Demod(&post_viterbi_ber_x1e9,AVL_POST_VITERBI_BER, chip);
    if (post_viterbi_ber_x1e9 <= 100)//RS BER <= 1e-7
    {
        ber_sqi = 100;
    }
    else if((post_viterbi_ber_x1e9 > 100)&&(post_viterbi_ber_x1e9 <= 1000000))//RS BER >1e-7, <=1e-3
    {
        //ber_sqi = (int32_t)(20.0*AVL_LOG10((int32_t)(1e9/post_viterbi_ber_x1e9))) - 40;
        uiTemp0 = (uint32_t)(1000000000/post_viterbi_ber_x1e9);
        uiTemp1 = sizeof(ISDBT_BERSQI_Table)/sizeof(AVL_ISDBT_BERSQI_List);
        for (index = 0; index < uiTemp1; index++)
        {
            if (uiTemp0 < ISDBT_BERSQI_Table[index].m_ber)
            {
                break;
            }
        }
        if (0 == index)
        {
            index = 1;
        }
        ber_sqi = ISDBT_BERSQI_Table[index - 1].m_ber_sqi - 40;
    }
    else //RS BER >1e-3
    {
        ber_sqi = 0;
    }

    //get signal info for mapping
    r |= AVL_Demod_ISDBTGetModulationInfo(&ISDBTSignalInfo, chip);

    modulation = ISDBTSignalInfo.eISDBTLayerA.eISDBTModulationMode;
    code_rate = ISDBTSignalInfo.eISDBTLayerA.eISDBTCodeRate;


    //Get Nordig C/N

    for(index=0;index<sizeof(AVL_ISDBT_CN_Table)/sizeof(AVL_ISDBT_CN_Table_Element);index++)
    {
        if((AVL_ISDBT_CN_Table[index].modulation == modulation)&&
            (AVL_ISDBT_CN_Table[index].code_rate == code_rate))
        {
            CN_NordigP1_x100_db = AVL_ISDBT_CN_Table[index].CN_NordigP1_x100_db;
            break;
        }
    }

    if(index == sizeof(AVL_ISDBT_CN_Table)/sizeof(AVL_ISDBT_CN_Table_Element))
    {
        return AVL_EC_NOT_SUPPORTED;
    }
    
    //Get received C/N
    r |= ISDBT_GetSNR_Demod(&CN_received_x100_db, chip);
    CN_received_x100_db += CN_received_x100_db/12;  //adjust Rx C/N estimate to Tx C/N estimate
    CN_received_x100_db += 100; //bias Rx C/N to provide more SQI resolution near threshold while keeping the result within NorDig acceptable range


    //Calculate relative C/N
    CN_relative_x100_db = (int32_t)CN_received_x100_db - CN_NordigP1_x100_db;

    if(CN_relative_x100_db < -700)  
    {
        *puiSQI = 0;
    }
    else if((CN_relative_x100_db >= -700) && (CN_relative_x100_db < 300))   
    {
        *puiSQI = (((CN_relative_x100_db - 300)/10) + 100)*ber_sqi/100;
    }
    else
    {
        *puiSQI = ber_sqi;
    }

    return r;
}


avl_error_code_t ISDBT_SetIFInputPath_Demod(AVL_InputPath eInputPath, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    r = avl_bms_write8(chip->chip_pub->i2c_addr,
        stBaseAddrSet.fw_ISDBT_config_reg_base + rc_ISDBT_adc_sel_caddr_offset, (uint8_t)eInputPath);

    return r;
}

avl_error_code_t ISDBT_GetSignalDetection(uint8_t *pucNoSig, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint32_t uiTemp = 0;
   
    r = avl_bms_read32(chip->chip_pub->i2c_addr,
                stBaseAddrSet.fw_ISDBT_status_reg_base + rs_ISDBT_Signal_Presence_iaddr_offset,&uiTemp);
    if(uiTemp == 1)// detected
    {
        *pucNoSig = 1;
    }
    else if(uiTemp == 0)//no signal (0)
    {
        *pucNoSig = 0;
    }
    else if(uiTemp == 2)//unknown (not send command acquire)
    {
        *pucNoSig = 1;
    }
    
    return (r);
}

avl_error_code_t ISDBT_SetIFFrequency_Demod(uint32_t uiIFFrequencyHz, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint32_t carrier_offset_hz = 0;

    if(uiIFFrequencyHz >= chip->uiADCFrequencyHz)
    {
       carrier_offset_hz = uiIFFrequencyHz - chip->uiADCFrequencyHz;
    }
    else
    {
      carrier_offset_hz = uiIFFrequencyHz;
    }

    r = avl_bms_write32(chip->chip_pub->i2c_addr,
        stBaseAddrSet.fw_ISDBT_config_reg_base + rc_ISDBT_nom_carrier_freq_Hz_iaddr_offset, carrier_offset_hz);

    return r;
}

avl_error_code_t ISDBT_SetBandWidth_Demod(AVL_ISDBT_BandWidth eISDBTBandWidth, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    
    r = avl_bms_write32(chip->chip_pub->i2c_addr,
        stBaseAddrSet.fw_ISDBT_config_reg_base + rc_ISDBT_fund_rate_Hz_iaddr_offset, BW_FFT_Table[eISDBTBandWidth]);

    return r;
}

avl_error_code_t ISDBT_GetPrePostBER_Demod(uint32_t *puiBERxe9, AVL_BER_Type eBERType, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    return r;
}

avl_error_code_t ISDBT_ResetLayerPER_Demod(avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint32_t uiTemp = 0;

    gstISDBTLayerA_ErrorStats.stNumPkts.high_word = 0;
    gstISDBTLayerA_ErrorStats.stNumPkts.low_word = 0;
    gstISDBTLayerA_ErrorStats.stPktErrors.high_word = 0;
    gstISDBTLayerA_ErrorStats.stPktErrors.low_word = 0;
    gstISDBTLayerA_ErrorStats.stSwCntNumPkts.high_word = 0;
    gstISDBTLayerA_ErrorStats.stSwCntNumPkts.low_word = 0;
    gstISDBTLayerA_ErrorStats.stSwCntPktErrors.high_word = 0;
    gstISDBTLayerA_ErrorStats.stSwCntPktErrors.low_word = 0;
    gstISDBTLayerA_ErrorStats.uiPER = 0;

    gstISDBTLayerB_ErrorStats.stNumPkts.high_word = 0;
    gstISDBTLayerB_ErrorStats.stNumPkts.low_word = 0;
    gstISDBTLayerB_ErrorStats.stPktErrors.high_word = 0;
    gstISDBTLayerB_ErrorStats.stPktErrors.low_word = 0;
    gstISDBTLayerB_ErrorStats.stSwCntNumPkts.high_word = 0;
    gstISDBTLayerB_ErrorStats.stSwCntNumPkts.low_word = 0;
    gstISDBTLayerB_ErrorStats.stSwCntPktErrors.high_word = 0;
    gstISDBTLayerB_ErrorStats.stSwCntPktErrors.low_word = 0;
    gstISDBTLayerB_ErrorStats.uiPER = 0;

    gstISDBTLayerC_ErrorStats.stNumPkts.high_word = 0;
    gstISDBTLayerC_ErrorStats.stNumPkts.low_word = 0;
    gstISDBTLayerC_ErrorStats.stPktErrors.high_word = 0;
    gstISDBTLayerC_ErrorStats.stPktErrors.low_word = 0;
    gstISDBTLayerC_ErrorStats.stSwCntNumPkts.high_word = 0;
    gstISDBTLayerC_ErrorStats.stSwCntNumPkts.low_word = 0;
    gstISDBTLayerC_ErrorStats.stSwCntPktErrors.high_word = 0;
    gstISDBTLayerC_ErrorStats.stSwCntPktErrors.low_word = 0;
    gstISDBTLayerC_ErrorStats.uiPER = 0;

    r |= avl_bms_read32(chip->chip_pub->i2c_addr,
        stBaseAddrSet.hw_dvb_gen1_fec__base + rs_errstat_clear__offset, &uiTemp);
    uiTemp |= 0x00000001;
    r |= avl_bms_write32(chip->chip_pub->i2c_addr,
        stBaseAddrSet.hw_dvb_gen1_fec__base + rs_errstat_clear__offset, uiTemp);
    uiTemp &= 0xFFFFFFFE;
    r |= avl_bms_write32(chip->chip_pub->i2c_addr,
        stBaseAddrSet.hw_dvb_gen1_fec__base + rs_errstat_clear__offset, uiTemp);

    r |= avl_bms_read32(chip->chip_pub->i2c_addr,
        stBaseAddrSet.hw_dvb_gen1_fec__base + rs_errstat_clear__offset, &uiTemp);
    uiTemp |= 0x00000004;
    r |= avl_bms_write32(chip->chip_pub->i2c_addr,
        stBaseAddrSet.hw_dvb_gen1_fec__base + rs_errstat_clear__offset, uiTemp);
    uiTemp &= 0xFFFFFFFB;
    r |= avl_bms_write32(chip->chip_pub->i2c_addr,
        stBaseAddrSet.hw_dvb_gen1_fec__base + rs_errstat_clear__offset, uiTemp);

    return r;
}

avl_error_code_t ISDBT_GetLayerPER_Demod(uint32_t *puiPERxe9, enum AVL_ISDBT_Layer eLayerNum, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint32_t uiHwCntPktErrors = 0;
    uint32_t uiHwCntNumPkts = 0;
    uint32_t uiTemp = 0;
    struct avl_uint64 uiTemp64 = {0,0};

    if (eLayerNum == AVL_ISDBT_LAYER_A)
    {
        r |= avl_bms_read32(chip->chip_pub->i2c_addr, 
            stBaseAddrSet.fw_ISDBT_status_reg_base + rs_ISDBT_layA_tot_pkts_iaddr_offset, &uiHwCntNumPkts);
        r |= avl_bms_read32(chip->chip_pub->i2c_addr, 
            stBaseAddrSet.fw_ISDBT_status_reg_base + rs_ISDBT_layA_err_pkts_iaddr_offset, &uiHwCntPktErrors);
    }
    else if (eLayerNum == AVL_ISDBT_LAYER_B)
    {
        r |= avl_bms_read32(chip->chip_pub->i2c_addr, 
            stBaseAddrSet.fw_ISDBT_status_reg_base + rs_ISDBT_layB_tot_pkts_iaddr_offset, &uiHwCntNumPkts);
        r |= avl_bms_read32(chip->chip_pub->i2c_addr, 
            stBaseAddrSet.fw_ISDBT_status_reg_base + rs_ISDBT_layB_err_pkts_iaddr_offset, &uiHwCntPktErrors);
    }
    else if (eLayerNum == AVL_ISDBT_LAYER_C)
    {
        r |= avl_bms_read32(chip->chip_pub->i2c_addr, 
            stBaseAddrSet.fw_ISDBT_status_reg_base + rs_ISDBT_layC_tot_pkts_iaddr_offset, &uiHwCntNumPkts);
        r |= avl_bms_read32(chip->chip_pub->i2c_addr, 
            stBaseAddrSet.fw_ISDBT_status_reg_base + rs_ISDBT_layC_err_pkts_iaddr_offset, &uiHwCntPktErrors);
    }
    else
    {
        return AVL_EC_GENERAL_FAIL;
    }

    if(uiHwCntNumPkts > (uint32_t)(1 << 31))
    {
        r |= avl_bms_read32(chip->chip_pub->i2c_addr,
            stBaseAddrSet.hw_dvb_gen1_fec__base + rs_errstat_clear__offset, &uiTemp);
        uiTemp |= 0x00000001;
        r |= avl_bms_write32(chip->chip_pub->i2c_addr,
            stBaseAddrSet.hw_dvb_gen1_fec__base + rs_errstat_clear__offset, uiTemp);
        uiTemp &= 0xFFFFFFFE;
        r |= avl_bms_write32(chip->chip_pub->i2c_addr,
            stBaseAddrSet.hw_dvb_gen1_fec__base + rs_errstat_clear__offset, uiTemp);

        r |= avl_bms_read32(chip->chip_pub->i2c_addr,
            stBaseAddrSet.hw_dvb_gen1_fec__base + rs_errstat_clear__offset, &uiTemp);
        uiTemp |= 0x00000004;
        r |= avl_bms_write32(chip->chip_pub->i2c_addr,
            stBaseAddrSet.hw_dvb_gen1_fec__base + rs_errstat_clear__offset, uiTemp);
        uiTemp &= 0xFFFFFFFB;
        r |= avl_bms_write32(chip->chip_pub->i2c_addr,
            stBaseAddrSet.hw_dvb_gen1_fec__base + rs_errstat_clear__offset, uiTemp);

        if (eLayerNum == AVL_ISDBT_LAYER_A)
        {
            avl_add_32to64(&gstISDBTLayerA_ErrorStats.stSwCntNumPkts, uiHwCntNumPkts);
            avl_add_32to64(&gstISDBTLayerA_ErrorStats.stSwCntPktErrors, uiHwCntPktErrors);
        }
        else if (eLayerNum == AVL_ISDBT_LAYER_B)
        {
            avl_add_32to64(&gstISDBTLayerB_ErrorStats.stSwCntNumPkts, uiHwCntNumPkts);
            avl_add_32to64(&gstISDBTLayerB_ErrorStats.stSwCntPktErrors, uiHwCntPktErrors);
        }
        else if (eLayerNum == AVL_ISDBT_LAYER_C)
        {
            avl_add_32to64(&gstISDBTLayerC_ErrorStats.stSwCntNumPkts, uiHwCntNumPkts);
            avl_add_32to64(&gstISDBTLayerC_ErrorStats.stSwCntPktErrors, uiHwCntPktErrors);
        }
        else
        {
            return AVL_EC_GENERAL_FAIL;
        }

        uiHwCntNumPkts = 0;
        uiHwCntPktErrors = 0;
    }

    if (eLayerNum == AVL_ISDBT_LAYER_A)
    {
        gstISDBTLayerA_ErrorStats.stNumPkts.high_word = gstISDBTLayerA_ErrorStats.stSwCntNumPkts.high_word;
        gstISDBTLayerA_ErrorStats.stNumPkts.low_word = gstISDBTLayerA_ErrorStats.stSwCntNumPkts.low_word;
        avl_add_32to64(&gstISDBTLayerA_ErrorStats.stNumPkts, uiHwCntNumPkts);

        gstISDBTLayerA_ErrorStats.stPktErrors.high_word = gstISDBTLayerA_ErrorStats.stSwCntPktErrors.high_word;
        gstISDBTLayerA_ErrorStats.stPktErrors.low_word = gstISDBTLayerA_ErrorStats.stSwCntPktErrors.low_word;
        avl_add_32to64(&gstISDBTLayerA_ErrorStats.stPktErrors, uiHwCntPktErrors);

        avl_mult_32to64(&uiTemp64, gstISDBTLayerA_ErrorStats.stPktErrors.low_word, AVL_CONSTANT_10_TO_THE_9TH);
        gstISDBTLayerA_ErrorStats.uiPER = avl_divide_64(gstISDBTLayerA_ErrorStats.stNumPkts, uiTemp64);

        *puiPERxe9 = gstISDBTLayerA_ErrorStats.uiPER;
    }
    else if (eLayerNum == AVL_ISDBT_LAYER_B)
    {
        gstISDBTLayerB_ErrorStats.stNumPkts.high_word = gstISDBTLayerB_ErrorStats.stSwCntNumPkts.high_word;
        gstISDBTLayerB_ErrorStats.stNumPkts.low_word = gstISDBTLayerB_ErrorStats.stSwCntNumPkts.low_word;
        avl_add_32to64(&gstISDBTLayerB_ErrorStats.stNumPkts, uiHwCntNumPkts);

        gstISDBTLayerB_ErrorStats.stPktErrors.high_word = gstISDBTLayerB_ErrorStats.stSwCntPktErrors.high_word;
        gstISDBTLayerB_ErrorStats.stPktErrors.low_word = gstISDBTLayerB_ErrorStats.stSwCntPktErrors.low_word;
        avl_add_32to64(&gstISDBTLayerB_ErrorStats.stPktErrors, uiHwCntPktErrors);

        avl_mult_32to64(&uiTemp64, gstISDBTLayerB_ErrorStats.stPktErrors.low_word, AVL_CONSTANT_10_TO_THE_9TH);
        gstISDBTLayerB_ErrorStats.uiPER = avl_divide_64(gstISDBTLayerB_ErrorStats.stNumPkts, uiTemp64);

        *puiPERxe9 = gstISDBTLayerB_ErrorStats.uiPER;
    }
    else if (eLayerNum == AVL_ISDBT_LAYER_C)
    {
        gstISDBTLayerC_ErrorStats.stNumPkts.high_word = gstISDBTLayerC_ErrorStats.stSwCntNumPkts.high_word;
        gstISDBTLayerC_ErrorStats.stNumPkts.low_word = gstISDBTLayerC_ErrorStats.stSwCntNumPkts.low_word;
        avl_add_32to64(&gstISDBTLayerC_ErrorStats.stNumPkts, uiHwCntNumPkts);

        gstISDBTLayerC_ErrorStats.stPktErrors.high_word = gstISDBTLayerC_ErrorStats.stSwCntPktErrors.high_word;
        gstISDBTLayerC_ErrorStats.stPktErrors.low_word = gstISDBTLayerC_ErrorStats.stSwCntPktErrors.low_word;
        avl_add_32to64(&gstISDBTLayerC_ErrorStats.stPktErrors, uiHwCntPktErrors);

        avl_mult_32to64(&uiTemp64, gstISDBTLayerC_ErrorStats.stPktErrors.low_word, AVL_CONSTANT_10_TO_THE_9TH);
        gstISDBTLayerC_ErrorStats.uiPER = avl_divide_64(gstISDBTLayerC_ErrorStats.stNumPkts, uiTemp64);

        *puiPERxe9 = gstISDBTLayerC_ErrorStats.uiPER;
    }
    else
    {
        return AVL_EC_GENERAL_FAIL;
    }

    return r;
}

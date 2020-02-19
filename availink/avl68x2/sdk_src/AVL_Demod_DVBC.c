
#include "AVL_Demod.h"
#include "AVL_Demod_DVBC.h"


extern AVL_ChipInternal gstChipInternalArray[2];

avl_error_code_t AVL_Demod_DVBCAutoLock(AVL_ChipInternal *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    if(1 == chip->ucSleepFlag)
    {
        r = AVL_EC_SLEEP;
        return r;
    }

    r = IBase_SendRxOPWait_Demod(AVL_FW_CMD_HALT, chip);

    r |= avl_bms_write32(chip->usI2CAddr, 
        stBaseAddrSet.fw_DVBC_config_reg_base + rc_DVBC_qam_mode_scan_control_iaddr_offset, 0x0101);  
    
    r |= IBase_SendRxOPWait_Demod(AVL_FW_CMD_ACQUIRE, chip);

    return r;
}

avl_error_code_t AVL_Demod_DVBCManualLock (uint32_t uiSymbolRateSps,AVL_DVBCQAMMode eQAMMode, AVL_ChipInternal *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint32_t uiTemp = 0;

    if(1 == chip->ucSleepFlag)
    {
        r = AVL_EC_SLEEP;
        return r;
    }

    r = IBase_SendRxOPWait_Demod(AVL_FW_CMD_HALT, chip);
    
    r |= avl_bms_write32(chip->usI2CAddr, 
        stBaseAddrSet.fw_DVBC_config_reg_base + rc_DVBC_symbol_rate_Hz_iaddr_offset, uiSymbolRateSps);
    r |= avl_bms_read32(chip->usI2CAddr, 
        stBaseAddrSet.fw_DVBC_config_reg_base + rc_DVBC_qam_mode_iaddr_offset, &uiTemp);                   
    uiTemp &= 0xFFFFFF00;
    
    uiTemp |= ((uint32_t)(eQAMMode))<<0;
    r |= avl_bms_write32(chip->usI2CAddr, 
        stBaseAddrSet.fw_DVBC_config_reg_base + rc_DVBC_qam_mode_iaddr_offset, uiTemp);   

    r |= avl_bms_write8(chip->usI2CAddr, 
        stBaseAddrSet.fw_DVBC_config_reg_base + rc_DVBC_lock_mode_caddr_offset, 0);

    r |= IBase_SendRxOPWait_Demod(AVL_FW_CMD_ACQUIRE, chip);

    return r ;
}

avl_error_code_t AVL_Demod_DVBCGetModulationInfo(AVL_DVBCModulationInfo *pstModulationInfo, AVL_ChipInternal *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint32_t uiTemp = 0;
    uint8_t ucTemp = 0;
    
    r = avl_bms_read32(chip->usI2CAddr, 
        stBaseAddrSet.fw_DVBC_config_reg_base + rc_DVBC_qam_mode_iaddr_offset, &uiTemp);
    pstModulationInfo->eQAMMode = (AVL_DVBCQAMMode)(uiTemp & 0x00000007);

    r |= avl_bms_read8(chip->usI2CAddr, 
        stBaseAddrSet.fw_DVBC_status_reg_base + rs_DVBC_j83b_il_mode_caddr_offset, &ucTemp);
        pstModulationInfo->eInterleaveMode = (AVL_DVBCInterleaveMode)ucTemp;

    return r;
}

avl_error_code_t DVBC_Initialize_Demod(AVL_ChipInternal *chip)
{
    avl_error_code_t r = AVL_EC_OK;  

    r = avl_bms_write8(chip->usI2CAddr, 
         stBaseAddrSet.fw_DVBC_config_reg_base + rc_DVBC_j83b_mode_caddr_offset, 
         chip->stDVBCPara.eDVBCStandard);

    r |= avl_bms_write32(chip->usI2CAddr, 
        stBaseAddrSet.fw_DVBC_config_reg_base + rc_DVBC_dmd_clk_Hz_iaddr_offset,
        chip->uiCoreFrequencyHz);
    r |= avl_bms_write32(chip->usI2CAddr, 
        stBaseAddrSet.fw_DVBC_config_reg_base + rc_DVBC_fec_clk_Hz_iaddr_offset,
        chip->uiFECFrequencyHz);

    r |= avl_bms_write8(chip->usI2CAddr,
        stBaseAddrSet.fw_DVBC_config_reg_base + rc_DVBC_rfagc_pol_caddr_offset,
        chip->stDVBCPara.eDVBCAGCPola);
    
   
    
    r |= DVBC_SetIFFrequency_Demod(chip->stDVBCPara.uiDVBCIFFreqHz,chip);
    r |= DVBC_SetIFInputPath_Demod((AVL_InputPath)(chip->stDVBCPara.eDVBCInputPath^1),chip);
    r |= DVBC_SetSymbolRate_Demod(chip->stDVBCPara.uiDVBCSymbolRateSps,chip);
    r |= DVBC_SetStandard_Demod(chip->stDVBCPara.eDVBCStandard,chip);

    //DDC configuration
    r |= avl_bms_write8(chip->usI2CAddr, 
        stBaseAddrSet.fw_DVBC_config_reg_base + rc_DVBC_input_format_caddr_offset, AVL_ADC_IN);//ADC in
    r |= avl_bms_write8(chip->usI2CAddr, 
        stBaseAddrSet.fw_DVBC_config_reg_base + rc_DVBC_input_select_caddr_offset, AVL_OFFBIN);//RX_OFFBIN
    r |= avl_bms_write8(chip->usI2CAddr, 
        stBaseAddrSet.fw_DVBC_config_reg_base + rc_DVBC_tuner_type_caddr_offset, AVL_DVBC_IF);//IF

    //ADC configuration 
    switch(chip->eDemodXtal)
    {    
     case Xtal_16M :
     case Xtal_24M :
        {
          r |= avl_bms_write8(chip->usI2CAddr, 
            stBaseAddrSet.fw_DVBC_config_reg_base + rc_DVBC_adc_use_pll_clk_caddr_offset, 1);
          break;
        }
            
      case Xtal_30M :
      case Xtal_27M :
        {
          r |= avl_bms_write8(chip->usI2CAddr, 
            stBaseAddrSet.fw_DVBC_config_reg_base + rc_DVBC_adc_use_pll_clk_caddr_offset, 0);
          break;
        }       
    }
    
    r |= avl_bms_write32(chip->usI2CAddr, 
        stBaseAddrSet.fw_DVBC_config_reg_base + rc_DVBC_sample_rate_Hz_iaddr_offset,
        chip->uiADCFrequencyHz);

    if(chip->ucDisableTCAGC == 0)
    {
        r |= EnableTCAGC_Demod(chip);
    }
    else
    {
        r |= DisableTCAGC_Demod(chip);
    }

    return (r);
}

avl_error_code_t DVBC_GetLockStatus_Demod( uint8_t * pucLocked, AVL_ChipInternal *chip )
{
    avl_error_code_t r = AVL_EC_OK;  
    uint32_t uiTemp = 0;


    r = avl_bms_read32(chip->usI2CAddr, 
        stBaseAddrSet.fw_DVBC_status_reg_base + rs_DVBC_mode_status_iaddr_offset, &uiTemp);

    if((r == AVL_EC_OK) && (0x15 == ((uiTemp)&0xff)))
    {
        *pucLocked = 1;
    }
    else
    {
        *pucLocked = 0;
    }

    return r;
}

avl_error_code_t DVBC_GetSNR_Demod(uint32_t * puiSNR_db, AVL_ChipInternal *chip)
{
    avl_error_code_t r = AVL_EC_OK;  
    uint16_t usSNR = 0;
    uint32_t uiTemp = 0;

    r = avl_bms_read32(chip->usI2CAddr, 
        stBaseAddrSet.fw_DVBC_config_reg_base + rc_DVBC_get_btr_crl_iaddr_offset, &uiTemp);
    if(0x00 == uiTemp)
    {
        r = avl_bms_read16(chip->usI2CAddr, 
            stBaseAddrSet.fw_DVBC_status_reg_base + rs_DVBC_snr_dB_x100_saddr_offset, &usSNR);
        *puiSNR_db = (uint32_t)usSNR;
        r |= avl_bms_write32(chip->usI2CAddr, 
            stBaseAddrSet.fw_DVBC_config_reg_base + rc_DVBC_get_btr_crl_iaddr_offset, 0x01);
    }
    else
    {
        r |= AVL_EC_RUNNING;
    }

    return r;
}

typedef struct AVL_DVBC_SQI_CN_Table_Element
{
    AVL_DVBC_Standard eStandard;
    AVL_DVBCQAMMode modulation;
    uint32_t CN_Test_Result_x100_db;
}AVL_DVBC_SQI_CN_Table_Element;

AVL_DVBC_SQI_CN_Table_Element AVL_DVBC_CN_Table[]=
{
    //profile 1, AWGN
    {AVL_DVBC_J83A, AVL_DVBC_16QAM  , 1700}, 
    {AVL_DVBC_J83A, AVL_DVBC_32QAM  , 1980},
    {AVL_DVBC_J83A, AVL_DVBC_64QAM  , 2300}, 
    {AVL_DVBC_J83A, AVL_DVBC_128QAM , 2600}, 
    {AVL_DVBC_J83A, AVL_DVBC_256QAM , 2920},
    
    {AVL_DVBC_J83B, AVL_DVBC_64QAM ,  2180},
    {AVL_DVBC_J83B, AVL_DVBC_256QAM , 2810}
};

avl_error_code_t DVBC_GetSignalQuality_Demod(uint16_t * puiQuality , AVL_ChipInternal *chip)
{
    avl_error_code_t r = AVL_EC_OK;  
    uint32_t uiEstimated_SNR = 0;
    uint32_t CN_Ref_x100_db = 0;
    int32_t  index = 0;
    int32_t  temp_range = 0;
    AVL_LockStatus lock_status;
    AVL_DVBCModulationInfo DVBCSignalInfo;
    AVL_DVBCQAMMode modulation;
    AVL_DVBC_Standard eStandard = AVL_DVBC_UNKNOWN;
    uint8_t ucTemp;

    r = DVBC_GetLockStatus_Demod( &ucTemp, chip );
    lock_status = (AVL_LockStatus)ucTemp;

    if ((lock_status == AVL_STATUS_LOCK)&&(r == AVL_EC_OK))
    {
        //get the signal modulation mode
        r |= AVL_Demod_DVBCGetModulationInfo(&DVBCSignalInfo, chip);
        modulation = DVBCSignalInfo.eQAMMode;

        avl_bms_read8(chip->usI2CAddr, 
        stBaseAddrSet.fw_DVBC_config_reg_base + rc_DVBC_j83b_mode_caddr_offset,
        (uint8_t *)&eStandard);
        
        //get the signal reference CN for CN table
        temp_range = sizeof(AVL_DVBC_CN_Table) / sizeof(AVL_DVBC_SQI_CN_Table_Element);
        for(index=0; index < temp_range; index++)
        {
            if(AVL_DVBC_CN_Table[index].modulation == modulation && AVL_DVBC_CN_Table[index].eStandard == eStandard)
            {
                CN_Ref_x100_db = AVL_DVBC_CN_Table[index].CN_Test_Result_x100_db;
                break;
            }
        }

        //get the signal estimate SNR
        r |= DVBC_GetSNR_Demod(&uiEstimated_SNR, chip);

        //SQI is 100 when the CN higher than threshold 2dB
        if (uiEstimated_SNR > CN_Ref_x100_db + 200)
        {
            *puiQuality = 100;
        }
        else if (uiEstimated_SNR < CN_Ref_x100_db -100)
        {
            *puiQuality = 0;
        }
        else
        {
            *puiQuality = (uiEstimated_SNR - CN_Ref_x100_db + 100) / 3;    
        }
        
        return (r);
    }
    else
    {
        *puiQuality = 0;
        return(r);
    }

    return r;
}

avl_error_code_t DVBC_SetIFInputPath_Demod(AVL_InputPath eInputPath, AVL_ChipInternal *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    r = avl_bms_write8(chip->usI2CAddr, 
        stBaseAddrSet.fw_DVBC_config_reg_base + rc_DVBC_adc_sel_caddr_offset,(uint8_t)eInputPath);

    return r;
}

avl_error_code_t DVBC_SetIFFrequency_Demod(uint32_t uiIFFrequencyHz, AVL_ChipInternal *chip)
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
 
    r = avl_bms_write32(chip->usI2CAddr, 
        stBaseAddrSet.fw_DVBC_config_reg_base + rc_DVBC_if_freq_Hz_iaddr_offset,carrier_offset_hz);

    return r;
}

avl_error_code_t DVBC_SetStandard_Demod(AVL_DVBC_Standard eDVBCStandard, AVL_ChipInternal *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    r = avl_bms_write8(chip->usI2CAddr, 
        stBaseAddrSet.fw_DVBC_config_reg_base + rc_DVBC_j83b_mode_caddr_offset, eDVBCStandard);

    return r;
}

avl_error_code_t DVBC_SetSymbolRate_Demod(uint32_t uiDVBCSymbolRateSps, AVL_ChipInternal *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    r = avl_bms_write32(chip->usI2CAddr, 
        stBaseAddrSet.fw_DVBC_config_reg_base + rc_DVBC_symbol_rate_Hz_iaddr_offset, uiDVBCSymbolRateSps);

    return r;
}

void DVBC_SetFwData_Demod(uint8_t * pInitialData, AVL_ChipInternal *chip)
{
    chip->fwData = pInitialData;
}

avl_error_code_t DVBC_GetPrePostBER_Demod(uint32_t *puiBERxe9, AVL_BER_Type eBERType, AVL_ChipInternal *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint32_t uiTemp = 0;
    
    switch(eBERType)
    {
        case AVL_PRE_VITERBI_BER:
            *puiBERxe9 = 0;
            break;
        case AVL_POST_VITERBI_BER:
             r = avl_bms_read32(chip->usI2CAddr,
                 stBaseAddrSet.fw_DVBC_status_reg_base + rs_DVBC_post_viterbi_BER_estimate_x10M_iaddr_offset,&uiTemp);
            *puiBERxe9 = uiTemp * 100;//match 1e9
            break;
        case AVL_PRE_LDPC_BER:
            *puiBERxe9 = 0;
            break;
        case AVL_POST_LDPC_BER:
            *puiBERxe9 = 0;
            break;
        default:
            break;
    }

    return r;
}


avl_error_code_t AVL_Demod_DVBCSignalDetection(uint8_t *pucNoSig, AVL_ChipInternal *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint32_t uiTemp = 1;
    
    r = avl_bms_read32(chip->usI2CAddr,
                stBaseAddrSet.fw_DVBC_status_reg_base + rc_DVBC_no_signal_input_iaddr_offset,&uiTemp);
    if(uiTemp == 1)// detected 1
    {
        *pucNoSig = 0;
    }
    else if(uiTemp == 0)//no signal (0)
    {
        *pucNoSig = 1;
    }
    else if(uiTemp == 2)//unknown (not send command acquire) 1
    {
        *pucNoSig = 1;
    }
    
    return (r);
}

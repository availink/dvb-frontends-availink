// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Availink AVL68x2 DVB-S/S2/T/T2/C, ISDB-T, J83.B demodulator driver
 *
 * Copyright (C) 2020 Availink, Inc. (gpl@availink.com)
 *
 */

#include "AVL_Demod.h"
#include "AVL_Demod_DVBTx.h"

extern avl68x2_chip gstChipInternalArray[2];

static int BW_FFT_Table[5]=
{
    1845070,    //bw=1.7MHz
    5714285,    //bw=5.0MHz
    6857143,    //bw=6.0MHz
    8000000,    //bw=7.0MHz
    9142857     //bw=8.0MHz
};

avl_error_code_t AVL_Demod_DVBTxChannelScan(AVL_DVBTxBandWidth eBandWidth, AVL_DVBTx_LockMode eLockMode, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    
    r = DVBTx_SetBandWidth_Demod(eBandWidth, chip);

    if(1 == chip->chip_priv->sleep_flag)
    {
        r = AVL_EC_SLEEP;
        return r;
    }

    r = IBase_SendRxOPWait_Demod(AVL_FW_CMD_HALT, chip);

    r |= avl_bms_write8(chip->chip_pub->i2c_addr,
        stBaseAddrSet.fw_DVBTx_config_reg_base + rc_DVBTx_acquire_mode_caddr_offset,
        eLockMode);

    r |= avl_bms_write8(chip->chip_pub->i2c_addr,
         stBaseAddrSet.fw_DVBTx_config_reg_base + rc_DVBTx_l1_proc_only_caddr_offset,
         1);

    r |= IBase_SendRxOPWait_Demod(AVL_FW_CMD_ACQUIRE, chip);

    return r;
}

avl_error_code_t AVL_Demod_DVBTxGetScanInfo(AVL_DVBTxScanInfo* pstDVBTxScanInfo, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint8_t ucTemp0 = 0;
    uint8_t ucTemp1 = 0;
    uint8_t ucDemodLockStatus = 0;
    uint8_t nWait = 20;
    while((nWait -- ) > 0)
    {
        ucDemodLockStatus = 0;
        r = AVL_Demod_GetLockStatus(&ucDemodLockStatus, chip);
        if(ucDemodLockStatus != 1 )
        {
            avl_bsp_delay(20);
        }
        else
        {
             break;
        }
    }   
    if(ucDemodLockStatus != 1)
    {
        return r;
    }

    r |= avl_bms_read8(chip->chip_pub->i2c_addr,
                         stBaseAddrSet.fw_DVBTx_status_reg_base + rs_DVBTx_rx_mode_caddr_offset,
                         &ucTemp0);

    pstDVBTxScanInfo->eTxStandard = (AVL_DVBTx_Standard)ucTemp0;

    if(AVL_DVBTx_Standard_T == pstDVBTxScanInfo->eTxStandard)
    {
        r |= avl_bms_read8(chip->chip_pub->i2c_addr,
                              stBaseAddrSet.fw_DVBT_TPS_reg_base + rs_DVBTx_hierarchy_caddr_offset,
                              &ucTemp0);
    }
    else if(AVL_DVBTx_Standard_T2 == pstDVBTxScanInfo->eTxStandard)
    {
        r |= avl_bms_read8(chip->chip_pub->i2c_addr,
                              stBaseAddrSet.fw_DVBT2_P1_reg_base + rs_DVBTx_P1_S2_field_2_caddr_offset,
                              &ucTemp1);
        r |= avl_bms_read8(chip->chip_pub->i2c_addr,
                              stBaseAddrSet.fw_DVBT2_P1_reg_base + rs_DVBTx_T2_profile_caddr_offset,
                              &ucTemp0);
    }

    pstDVBTxScanInfo->ucTxInfo = ucTemp0;
    pstDVBTxScanInfo->ucFEFInfo = ucTemp1;
    
    return r;
}

avl_error_code_t AVL_Demod_DVBT2AutoLock(AVL_DVBTxBandWidth eBandWidth, AVL_DVBT2_PROFILE eDVTB2Profile, uint8_t ucDVBT2PLPID, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    AVL_DVBTx_LockMode eDVBTxLockMode = AVL_DVBTx_LockMode_ALL;

    if(1 == chip->chip_priv->sleep_flag)
    {
        r = AVL_EC_SLEEP;
        return r;
    }

    r = IBase_SendRxOPWait_Demod(AVL_FW_CMD_HALT, chip);

    r = avl_bms_write8(chip->chip_pub->i2c_addr,
              stBaseAddrSet.fw_DVBTx_config_reg_base + rc_DVBTx_l1_proc_only_caddr_offset,
              0);

    if (eDVTB2Profile == AVL_DVBT2_PROFILE_BASE)
    {
        eDVBTxLockMode = AVL_DVBTx_LockMode_T2BASE;
    }
    else if (eDVTB2Profile == AVL_DVBT2_PROFILE_LITE)
    {
        eDVBTxLockMode = AVL_DVBTx_LockMode_T2LITE;
    }
    else
    {
        eDVBTxLockMode = AVL_DVBTx_LockMode_ALL;
    }
    
    r |= DVBTx_SetBandWidth_Demod(eBandWidth, chip);
    r |= avl_bms_write8(chip->chip_pub->i2c_addr,
        stBaseAddrSet.fw_DVBTx_config_reg_base + rc_DVBTx_acquire_mode_caddr_offset,
        (uint8_t)eDVBTxLockMode);
    r |= avl_bms_write8(chip->chip_pub->i2c_addr,
        stBaseAddrSet.fw_DVBTx_config_reg_base + rc_DVBTx_spectrum_invert_caddr_offset,
        AVL_SPECTRUM_AUTO);

    r |= avl_bms_write8(chip->chip_pub->i2c_addr,
        stBaseAddrSet.fw_DVBTx_config_reg_base + rc_DVBTx_data_PLP_ID_caddr_offset,ucDVBT2PLPID);
    r |= avl_bms_write8(chip->chip_pub->i2c_addr,
        stBaseAddrSet.fw_DVBTx_config_reg_base + rc_DVBTx_common_PLP_ID_caddr_offset,0);
    r |= avl_bms_write8(chip->chip_pub->i2c_addr,
        stBaseAddrSet.fw_DVBTx_config_reg_base + rc_DVBTx_common_PLP_present_caddr_offset,2);
    
    r |= IBase_SendRxOPWait_Demod(AVL_FW_CMD_ACQUIRE, chip);
    return (r);
}

avl_error_code_t AVL_Demod_DVBT2GetPLPList(uint8_t * pucPLPIndexArray, uint8_t * pucPLPNumber, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint8_t ucTemp = 0;
    uint32_t uiPLPBuffer = stBaseAddrSet.hw_plp_list_base;
    uint8_t ucPLPID = 0;
    uint8_t ucPLPType = 0; 
    uint8_t ucPLPGroupID = 0;
    uint32_t i = 0;
    uint8_t ucDataPLPNum = 0;
    uint32_t uiDelayMS = 20;
    uint32_t uiTimes = 10; //time-out window 10*20 = 200ms
    uint8_t ucDemodLockStatus = 0;
    uint8_t nWait = 20;

    while((nWait -- ) > 0)
    {
        ucDemodLockStatus = 0;
        r = AVL_Demod_GetLockStatus(&ucDemodLockStatus, chip);
        if(ucDemodLockStatus != 1)
        {
            avl_bsp_delay(20);
        }
        else
        {
            break;
        }
    }   
    if(ucDemodLockStatus != 1)
    {
        *pucPLPNumber = 0;
         return r;
    }

    for(i = 0; i < uiTimes; i++)
    {
        avl_bsp_delay(uiDelayMS);
        r |= avl_bms_read8(chip->chip_pub->i2c_addr,
            stBaseAddrSet.fw_DVBTx_status_reg_base + rs_DVBTx_plp_list_request_caddr_offset, &ucTemp);
        if(ucTemp == 0)
        {
            break;
        }
    }

    if(i == uiTimes)
    {
        r |= AVL_EC_GENERAL_FAIL;
        return (r);
    }

    r |= avl_bms_read8(chip->chip_pub->i2c_addr,
        stBaseAddrSet.fw_DVBT2_L1_post_config_reg_base + rs_DVBTx_NUM_PLP_caddr_offset, &ucTemp);

    for(i = 0; i<ucTemp; i++)
    {
        r |= avl_bms_read8(chip->chip_pub->i2c_addr,uiPLPBuffer++, &ucPLPID);
        r |= avl_bms_read8(chip->chip_pub->i2c_addr,uiPLPBuffer++, &ucPLPType);
        r |= avl_bms_read8(chip->chip_pub->i2c_addr,uiPLPBuffer++, &ucPLPGroupID);

        if(ucPLPType != 0)
        {
            *(pucPLPIndexArray + ucDataPLPNum) = ucPLPID;
            ucDataPLPNum++;
        }
    }

    *pucPLPNumber = ucDataPLPNum;

    return (r);

}

avl_error_code_t AVL_Demod_DVBTAutoLock(
    AVL_DVBTxBandWidth eBandWidth,
    uint8_t ucDVBTLayer,
    avl68x2_chip *chip)
{

	avl_error_code_t r = AVL_EC_OK;

	if (1 == chip->chip_priv->sleep_flag)
	{
		r = AVL_EC_SLEEP;
		return r;
	}
	r = IBase_SendRxOPWait_Demod(AVL_FW_CMD_HALT, chip);

	r = avl_bms_write8(chip->chip_pub->i2c_addr,
			   stBaseAddrSet.fw_DVBTx_config_reg_base +
			       rc_DVBTx_l1_proc_only_caddr_offset,
			   0);

	r = DVBTx_SetBandWidth_Demod(eBandWidth, chip);
	
	r |= avl_bms_write8(chip->chip_pub->i2c_addr,
			    stBaseAddrSet.fw_DVBTx_config_reg_base +
				rc_DVBTx_acquire_mode_caddr_offset,
			    (uint8_t)AVL_DVBTx_LockMode_T_ONLY);
	r |= avl_bms_write8(chip->chip_pub->i2c_addr,
			    stBaseAddrSet.fw_DVBTx_config_reg_base +
				rc_DVBTx_spectrum_invert_caddr_offset,
			    AVL_SPECTRUM_AUTO);

	r |= avl_bms_write8(chip->chip_pub->i2c_addr,
			    stBaseAddrSet.fw_DVBTx_config_reg_base +
				rc_DVBTx_dvbt_layer_select_caddr_offset,
			    ucDVBTLayer);

	r |= avl_bms_write8(chip->chip_pub->i2c_addr,
			    stBaseAddrSet.fw_DVBTx_config_reg_base +
				rc_DVBTx_data_PLP_ID_caddr_offset,
			    0);
	r |= avl_bms_write8(chip->chip_pub->i2c_addr,
			    stBaseAddrSet.fw_DVBTx_config_reg_base +
				rc_DVBTx_common_PLP_ID_caddr_offset,
			    0);
	r |= avl_bms_write8(chip->chip_pub->i2c_addr,
			    stBaseAddrSet.fw_DVBTx_config_reg_base +
				rc_DVBTx_common_PLP_present_caddr_offset,
			    0);

	r |= IBase_SendRxOPWait_Demod(AVL_FW_CMD_ACQUIRE, chip);

	return (r);
}

avl_error_code_t AVL_Demod_DVBTxGetNorDigSSI(uint8_t *pucSSI, int32_t iRFPowerdBm, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint32_t uiSSI = 0;

    r = IRx_GetSSI(&uiSSI,iRFPowerdBm, chip);

    *pucSSI = (uint8_t)uiSSI;

    return r;
}

avl_error_code_t AVL_Demod_DVBTxSignalDetection(uint8_t *pucNoSig, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint32_t uiTemp = 0;

    r = avl_bms_read32(chip->chip_pub->i2c_addr,
                stBaseAddrSet.fw_DVBTx_status_reg_base + rs_DVBTx_Signal_Presence_iaddr_offset,&uiTemp);
    if(uiTemp == 1)// detected 1
    {
        *pucNoSig = 1;
    }
    else if(uiTemp == 0)//no signal (0)
    {
        *pucNoSig = 0;
    }
    else if(uiTemp == 2)//unknown (not send command acquire) 1
    {
        *pucNoSig = 1;
    }
    
    return (r);
}

avl_error_code_t AVL_Demod_DVBTxGetModulationInfo(AVL_DVBTxModulationInfo *pstModulationInfo, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint8_t ucTemp = 0;
    uint16_t usTemp = 0;
    AVL_DVBTx_Standard current_standard = AVL_DVBTx_Standard_T;
    uint32_t uiPLPBuffer = stBaseAddrSet.hw_plp_list_base;
    uint8_t ucPLPType = 0; 
    uint32_t i = 0;
    uint8_t ucCommonPLPExist = 0;
    uint8_t ucNumberDPLP = 0;

    r = avl_bms_read8(chip->chip_pub->i2c_addr, 
        stBaseAddrSet.fw_DVBTx_status_reg_base + rs_DVBTx_rx_mode_caddr_offset,&ucTemp);

    current_standard = (AVL_DVBTx_Standard)ucTemp;

    if(AVL_DVBTx_Standard_T == current_standard)
    {
        pstModulationInfo->ucDVBxStandard = AVL_DVBTx_Standard_T;
        r |= avl_bms_read8(chip->chip_pub->i2c_addr, stBaseAddrSet.fw_DVBT_TPS_reg_base + rs_DVBTx_transmission_mode_caddr_offset,&ucTemp);
        pstModulationInfo->eDVBTSignalInfo.eDVBTFFTSize = (AVL_DVBT_FFTSize)ucTemp;
        
        r |= avl_bms_read8(chip->chip_pub->i2c_addr, stBaseAddrSet.fw_DVBT_TPS_reg_base + rs_DVBTx_guard_interval_caddr_offset,&ucTemp);
        pstModulationInfo->eDVBTSignalInfo.eDVBTGuardInterval = (AVL_DVBT_GuardInterval)ucTemp;
        
        r |= avl_bms_read8(chip->chip_pub->i2c_addr, stBaseAddrSet.fw_DVBT_TPS_reg_base + rs_DVBTx_constellation_caddr_offset,&ucTemp);
        pstModulationInfo->eDVBTSignalInfo.eDVBTModulationMode = (AVL_DVBT_ModulationMode)ucTemp;
        
        r |= avl_bms_read8(chip->chip_pub->i2c_addr, stBaseAddrSet.fw_DVBT_TPS_reg_base + rs_DVBTx_hierarchy_caddr_offset,&ucTemp);
        pstModulationInfo->eDVBTSignalInfo.eDVBTHierarchy = (AVL_DVBT_Hierarchy)ucTemp;

        r |= avl_bms_read8(chip->chip_pub->i2c_addr, stBaseAddrSet.fw_DVBT_TPS_reg_base + rs_DVBTx_HP_code_rate_caddr_offset,&ucTemp);
        pstModulationInfo->eDVBTSignalInfo.eDVBTHPCodeRate = (AVL_DVBT_CodeRate)ucTemp;

        if(pstModulationInfo->eDVBTSignalInfo.eDVBTHierarchy != AVL_DVBT_HIER_NONE)
        {
            r |= avl_bms_read8(chip->chip_pub->i2c_addr, stBaseAddrSet.fw_DVBT_TPS_reg_base + rs_DVBTx_LP_code_rate_caddr_offset,&ucTemp);
            pstModulationInfo->eDVBTSignalInfo.eDVBTLPCodeRate = (AVL_DVBT_CodeRate)ucTemp;
        }
    }
    else if(AVL_DVBTx_Standard_T2 == current_standard)
    {
        pstModulationInfo->ucDVBxStandard = AVL_DVBTx_Standard_T2;
        
        r |= avl_bms_read8(chip->chip_pub->i2c_addr, stBaseAddrSet.fw_DVBT2_P1_reg_base + rs_DVBTx_FFT_size_caddr_offset,&ucTemp);
        pstModulationInfo->eDVBT2SignalInfo.eDVBT2FFTSize = (AVL_DVBT2_FFTSize)ucTemp;
        
        r |= avl_bms_read8(chip->chip_pub->i2c_addr, stBaseAddrSet.fw_DVBT2_P1_reg_base + rs_DVBTx_MISO_SISO_caddr_offset,&ucTemp);
        pstModulationInfo->eDVBT2SignalInfo.eDVBT2MISOorSISO = (AVL_DVBT2_MISO_SISO)ucTemp;

        r |= avl_bms_read8(chip->chip_pub->i2c_addr, stBaseAddrSet.fw_DVBT2_P1_reg_base + rs_DVBTx_T2_profile_caddr_offset,&ucTemp);
        pstModulationInfo->eDVBT2SignalInfo.eDVBT2Profile = (AVL_DVBT2_PROFILE)ucTemp;

        r |= avl_bms_read8(chip->chip_pub->i2c_addr, stBaseAddrSet.fw_DVBT2_L1_pre_reg_base + rs_DVBTx_PILOT_PATTERN_caddr_offset,&ucTemp);
        pstModulationInfo->eDVBT2SignalInfo.eDVBT2PilotPatten = (AVL_DVBT2_PILOT_PATTERN)ucTemp;

        r |= avl_bms_read8(chip->chip_pub->i2c_addr, stBaseAddrSet.fw_DVBT2_data_PLP_config_reg_base + rs_DVBTx_data_PLP_TYPE_caddr_offset,&ucTemp);
        pstModulationInfo->eDVBT2SignalInfo.eDVBT2DataPLPType = (AVL_DVBT2_DATA_PLP_TYPE)ucTemp;
        
        r |= avl_bms_read8(chip->chip_pub->i2c_addr, stBaseAddrSet.fw_DVBT2_data_PLP_config_reg_base + rs_DVBTx_data_PLP_ID_caddr_offset,&ucTemp);
        pstModulationInfo->eDVBT2SignalInfo.ucDVBT2DataPLPID = ucTemp;

        r |= avl_bms_read8(chip->chip_pub->i2c_addr, stBaseAddrSet.fw_DVBT2_data_PLP_config_reg_base + rs_DVBTx_data_PLP_COD_caddr_offset,&ucTemp);
        pstModulationInfo->eDVBT2SignalInfo.eDVBT2DataPLPCodeRate = (AVL_DVBT2_CodeRate)ucTemp;

        r |= avl_bms_read8(chip->chip_pub->i2c_addr, stBaseAddrSet.fw_DVBT2_data_PLP_config_reg_base + rs_DVBTx_data_PLP_MOD_caddr_offset,&ucTemp);
        pstModulationInfo->eDVBT2SignalInfo.eDVBT2DataPLPModulationMode= (AVL_DVBT2_PLP_ModulationMode)ucTemp;

        r |= avl_bms_read8(chip->chip_pub->i2c_addr, stBaseAddrSet.fw_DVBT2_data_PLP_config_reg_base + rs_DVBTx_data_PLP_ROTATION_caddr_offset,&ucTemp);
        pstModulationInfo->eDVBT2SignalInfo.eDVBT2DataPLPRotation= (AVL_DVBT2_PLP_Constellation_Rotation)ucTemp;

        r |= avl_bms_read8(chip->chip_pub->i2c_addr, stBaseAddrSet.fw_DVBT2_data_PLP_config_reg_base + rs_DVBTx_data_PLP_FEC_TYPE_caddr_offset,&ucTemp);
        pstModulationInfo->eDVBT2SignalInfo.eDVBT2DataPLPFecType= (AVL_DVBT2_PLP_FEC_Type)ucTemp;

        r |= avl_bms_read8(chip->chip_pub->i2c_addr,
            stBaseAddrSet.fw_DVBT2_L1_post_config_reg_base + rs_DVBTx_NUM_PLP_caddr_offset, &ucTemp);
        
        for(i = 0; i<ucTemp; i++)
        {
            uiPLPBuffer++;
            r |= avl_bms_read8(chip->chip_pub->i2c_addr,uiPLPBuffer++, &ucPLPType);
            uiPLPBuffer++;

            if(ucPLPType == 0)
            {
                ucCommonPLPExist = 1;
            }
            else
            {
                ucNumberDPLP++;
            }
        }

        pstModulationInfo->eDVBT2SignalInfo.ucNumberDPLP = ucNumberDPLP;
        pstModulationInfo->eDVBT2SignalInfo.ucDVBT2CommonPLPExist = ucCommonPLPExist;
        
        if (ucCommonPLPExist == 1)
        {
            r |= avl_bms_read8(chip->chip_pub->i2c_addr, 
                stBaseAddrSet.fw_DVBT2_common_PLP_config_reg_base + rs_DVBTx_common_PLP_ID_caddr_offset,&ucTemp);
            pstModulationInfo->eDVBT2SignalInfo.ucDVBT2CommonPLPID = ucTemp;

            r |= avl_bms_read8(chip->chip_pub->i2c_addr, 
                stBaseAddrSet.fw_DVBT2_common_PLP_config_reg_base + rs_DVBTx_common_PLP_COD_caddr_offset,&ucTemp);
            pstModulationInfo->eDVBT2SignalInfo.eDVBT2CommonPLPCodeRate = (AVL_DVBT2_CodeRate)ucTemp;

            r |= avl_bms_read8(chip->chip_pub->i2c_addr, 
                stBaseAddrSet.fw_DVBT2_common_PLP_config_reg_base + rs_DVBTx_common_PLP_MOD_caddr_offset,&ucTemp);
            pstModulationInfo->eDVBT2SignalInfo.eDVBT2CommonPLPModulationMode = (AVL_DVBT2_PLP_ModulationMode)ucTemp;

            r |= avl_bms_read8(chip->chip_pub->i2c_addr, 
                stBaseAddrSet.fw_DVBT2_common_PLP_config_reg_base + rs_DVBTx_common_PLP_ROTATION_caddr_offset,&ucTemp);
            pstModulationInfo->eDVBT2SignalInfo.eDVBT2CommonPLPRotation = (AVL_DVBT2_PLP_Constellation_Rotation)ucTemp;

            r |= avl_bms_read8(chip->chip_pub->i2c_addr, 
                stBaseAddrSet.fw_DVBT2_common_PLP_config_reg_base + rs_DVBTx_common_PLP_FEC_TYPE_caddr_offset,&ucTemp);
            pstModulationInfo->eDVBT2SignalInfo.eDVBT2CommonPLPFecType = (AVL_DVBT2_PLP_FEC_Type)ucTemp;
        }

        r |= avl_bms_read8(chip->chip_pub->i2c_addr, 
            stBaseAddrSet.fw_DVBT2_P1_reg_base + rs_DVBTx_P1_S2_field_2_caddr_offset,&ucTemp);
        pstModulationInfo->eDVBT2SignalInfo.ucDVBT2FEFExist= ucTemp;

        r |= avl_bms_read16(chip->chip_pub->i2c_addr, 
            stBaseAddrSet.fw_DVBT2_L1_pre_reg_base + rs_DVBTx_CELL_ID_saddr_offset,&usTemp);
        pstModulationInfo->eDVBT2SignalInfo.eDVBT2SignalID.usCellID = usTemp;

        r |= avl_bms_read16(chip->chip_pub->i2c_addr, 
            stBaseAddrSet.fw_DVBT2_L1_pre_reg_base + rs_DVBTx_NETWORK_ID_saddr_offset,&usTemp);
        pstModulationInfo->eDVBT2SignalInfo.eDVBT2SignalID.usNetworkID = usTemp;

        r |= avl_bms_read16(chip->chip_pub->i2c_addr, 
            stBaseAddrSet.fw_DVBT2_L1_pre_reg_base + rs_DVBTx_T2_SYSTEM_ID_saddr_offset,&usTemp);
        pstModulationInfo->eDVBT2SignalInfo.eDVBT2SignalID.usSystemID = usTemp;

         r |= avl_bms_read8(chip->chip_pub->i2c_addr, 
             stBaseAddrSet.fw_DVBT2_L1_pre_reg_base + rs_DVBTx_GUARD_INTERVAL_caddr_offset,&ucTemp);
         pstModulationInfo->eDVBT2SignalInfo.eDVBT2GuardInterval = (AVL_DVBT2_GUARD_INTERVAL)ucTemp;

         r |= avl_bms_read8(chip->chip_pub->i2c_addr, 
             stBaseAddrSet.fw_DVBT2_L1_pre_reg_base + rs_DVBTx_PAPR_caddr_offset,&ucTemp);
         pstModulationInfo->eDVBT2SignalInfo.eDVBT2PAPR = (AVL_DVBT2_PAPR)ucTemp;
         
         r |= avl_bms_read8(chip->chip_pub->i2c_addr, 
             stBaseAddrSet.fw_DVBT2_L1_pre_reg_base + rs_DVBTx_L1_MOD_caddr_offset,&ucTemp);
         pstModulationInfo->eDVBT2SignalInfo.eDVBT2L1ModulationMode = (AVL_DVBT2_L1_Modulation)ucTemp;

        r |= avl_bms_read8(chip->chip_pub->i2c_addr, 
            stBaseAddrSet.fw_DVBT2_L1_pre_reg_base + rs_DVBTx_BWT_EXT_caddr_offset,&ucTemp);
        pstModulationInfo->eDVBT2SignalInfo.ucDVBT2BWExtended = ucTemp;
 
         r |= avl_bms_read8(chip->chip_pub->i2c_addr, 
             stBaseAddrSet.fw_DVBT2_L1_pre_reg_base + rs_DVBTx_NUM_T2_FRAMES_caddr_offset,&ucTemp);
         pstModulationInfo->eDVBT2SignalInfo.ucNumberFrames = ucTemp;

         r |= avl_bms_read16(chip->chip_pub->i2c_addr, 
             stBaseAddrSet.fw_DVBT2_L1_pre_reg_base + rs_DVBTx_NUM_DATA_SYMBOLS_saddr_offset,&usTemp);
         pstModulationInfo->eDVBT2SignalInfo.usNumberDataSymbols = usTemp;
    }
    
        
    return (r);
}

avl_error_code_t AVL_Demod_DVBT_GetTPSInfo(AVL_DVBT_TpsInfo *pstDVBTTpsInfo, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint8_t ucTemp = 0;
    uint16_t usTemp = 0;
    AVL_DVBTx_Standard current_standard = AVL_DVBTx_Standard_T;


    r = avl_bms_read8(chip->chip_pub->i2c_addr, 
        stBaseAddrSet.fw_DVBTx_status_reg_base + rs_DVBTx_rx_mode_caddr_offset,&ucTemp);

    current_standard = (AVL_DVBTx_Standard)ucTemp;

    if(AVL_DVBTx_Standard_T == current_standard)
    {
        r |= avl_bms_read8(chip->chip_pub->i2c_addr, stBaseAddrSet.fw_DVBT_TPS_reg_base + rs_DVBTx_TPS_length_caddr_offset,&ucTemp);
        pstDVBTTpsInfo->ucTpsLength = ucTemp;

        r |= avl_bms_read8(chip->chip_pub->i2c_addr, stBaseAddrSet.fw_DVBT_TPS_reg_base + rs_DVBTx_constellation_caddr_offset,&ucTemp);
        pstDVBTTpsInfo->eTpsConstellation = (AVL_DVBT_ModulationMode)ucTemp;

        r |= avl_bms_read8(chip->chip_pub->i2c_addr, stBaseAddrSet.fw_DVBT_TPS_reg_base + rs_DVBTx_hierarchy_caddr_offset,&ucTemp);
        pstDVBTTpsInfo->eTpsHierarchy = (AVL_DVBT_Hierarchy)ucTemp;

        r |= avl_bms_read8(chip->chip_pub->i2c_addr, stBaseAddrSet.fw_DVBT_TPS_reg_base + rs_DVBTx_HP_code_rate_caddr_offset,&ucTemp);
        pstDVBTTpsInfo->eTpsHPCodeRate = (AVL_DVBT_CodeRate)ucTemp;

		if(pstDVBTTpsInfo->eTpsHierarchy != AVL_DVBT_HIER_NONE)
		{
			r |= avl_bms_read8(chip->chip_pub->i2c_addr, stBaseAddrSet.fw_DVBT_TPS_reg_base + rs_DVBTx_LP_code_rate_caddr_offset,&ucTemp);
			pstDVBTTpsInfo->eTpsLPCodeRate = (AVL_DVBT_CodeRate)ucTemp;
		}

		r |= avl_bms_read8(chip->chip_pub->i2c_addr, stBaseAddrSet.fw_DVBT_TPS_reg_base + rs_DVBTx_guard_interval_caddr_offset,&ucTemp);
		pstDVBTTpsInfo->eTpsGuardInterval = (AVL_DVBT_GuardInterval)ucTemp;

		r |= avl_bms_read8(chip->chip_pub->i2c_addr,stBaseAddrSet.fw_DVBT_TPS_reg_base + rs_DVBTx_transmission_mode_caddr_offset,&ucTemp);
		pstDVBTTpsInfo->eTpsTransmissionMode = (AVL_DVBT_FFTSize)ucTemp;

		r |= avl_bms_read16(chip->chip_pub->i2c_addr, stBaseAddrSet.fw_DVBT_TPS_reg_base +  rs_DVBTx_TPS_cell_id_saddr_offset,&usTemp);
		pstDVBTTpsInfo->usTpsCellID = usTemp;

    }
        
    return (r);
}

avl_error_code_t DVBTx_Initialize_Demod(avl68x2_chip *chip)
{
	avl_error_code_t r = AVL_EC_OK;

	r = avl_bms_write32(chip->chip_pub->i2c_addr,
			    stBaseAddrSet.fw_DVBTx_config_reg_base +
				rc_DVBTx_sample_rate_Hz_iaddr_offset,
			    chip->uiADCFrequencyHz);
	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     stBaseAddrSet.fw_DVBTx_config_reg_base +
				 rc_DVBTx_mpeg_clk_rate_Hz_iaddr_offset,
			     chip->uiTSFrequencyHz);

	////DDC configuration
	r |= avl_bms_write8(chip->chip_pub->i2c_addr,
			    stBaseAddrSet.fw_DVBTx_config_reg_base +
				rc_DVBTx_input_format_caddr_offset,
			    AVL_OFFBIN);
	r |= avl_bms_write8(chip->chip_pub->i2c_addr,
			    stBaseAddrSet.fw_DVBTx_config_reg_base +
				rc_DVBTx_input_select_caddr_offset,
			    AVL_ADC_IN);
	r |= avl_bms_write8(chip->chip_pub->i2c_addr,
			    stBaseAddrSet.fw_DVBTx_config_reg_base +
				rc_DVBTx_tuner_type_caddr_offset,
			    chip->chip_pub->tc_tuner_type);

	r |= avl_bms_write8(chip->chip_pub->i2c_addr,
			    stBaseAddrSet.fw_DVBTx_config_reg_base +
				rc_DVBTx_rf_agc_pol_caddr_offset,
			    chip->chip_pub->dvbtx_config.eDVBTxAGCPola);
	r |= DVBTx_SetIFFrequency_Demod(
	    chip->chip_pub->dvbtx_config.uiDVBTxIFFreqHz,
	    chip);
	r |= DVBTx_SetIFInputPath_Demod(
	    (AVL_InputPath)(chip->chip_pub->dvbtx_config.eDVBTxInputPath ^ 1),
	    chip);

	//ADC configuration
	switch (chip->chip_pub->xtal)
	{
	case Xtal_16M:
	case Xtal_24M:
	{
		r |= avl_bms_write8(chip->chip_pub->i2c_addr,
				    stBaseAddrSet.fw_DVBTx_config_reg_base +
					rc_DVBTx_adc_use_pll_clk_caddr_offset,
				    1);
	}
	break;
	case Xtal_30M:
	case Xtal_27M:
	{
		r |= avl_bms_write8(chip->chip_pub->i2c_addr,
				    stBaseAddrSet.fw_DVBTx_config_reg_base +
					rc_DVBTx_adc_use_pll_clk_caddr_offset,
				    0);
	}
	break;
	}

	r |= ConfigAGCOutput_Demod(chip);

	return (r);
}

avl_error_code_t DVBTx_GetLockStatus_Demod( uint8_t * pucLocked, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    r = avl_bms_read8(chip->chip_pub->i2c_addr,
        stBaseAddrSet.fw_DVBTx_status_reg_base + rs_DVBTx_fec_lock_caddr_offset, pucLocked);

    return r;
}

avl_error_code_t DVBTx_GetSNR_Demod(uint32_t * puiSNR_db, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint16_t uiTemp = 0;

    r = avl_bms_read16(chip->chip_pub->i2c_addr, stBaseAddrSet.fw_DVBTx_status_reg_base + rs_DVBTx_snr_dB_x100_saddr_offset,&uiTemp);
    *puiSNR_db = (uint32_t)(uiTemp);
    return r;
}

avl_error_code_t DVBTx_GetSignalQuality_Demod(uint16_t * puiQuality , avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint32_t uiTemp = 0;
    
    r = IRx_GetSQI(&uiTemp, chip);
    
    *puiQuality = (uint16_t)uiTemp;//uiTemp's range is 0 to 100. It's safe here.
    

    return r;
}

avl_error_code_t IRx_GetSQI(uint32_t * puiSQI, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint8_t ucTemp = 0;
    uint32_t uiTemp = 0;
    AVL_DemodMode current_standard = AVL_DVBSX;
    AVL_DVBTx_Standard current_dvbtx_std = AVL_DVBTx_Standard_T;
    AVL_LockStatus lock_status = AVL_STATUS_UNLOCK;

    r = DVBTx_GetLockStatus_Demod( &ucTemp, chip );
    lock_status = (AVL_LockStatus)ucTemp;

    if(lock_status == AVL_STATUS_LOCK) 
    {
        r = avl_bms_read32(chip->chip_pub->i2c_addr,
            stBaseAddrSet.fw_config_reg_base + rs_current_active_mode_iaddr_offset,&uiTemp);
        current_standard = (AVL_DemodMode)uiTemp;

        switch(current_standard)
        {
        case AVL_DVBTX:
            r = avl_bms_read8(chip->chip_pub->i2c_addr,
                stBaseAddrSet.fw_DVBTx_status_reg_base + rs_DVBTx_rx_mode_caddr_offset,&ucTemp);

            current_dvbtx_std = (AVL_DVBTx_Standard)ucTemp;
            switch(current_dvbtx_std)
            {
            case AVL_DVBTx_Standard_T:
                r |= IRx_GetSQI_DVBT(puiSQI, chip);
                break;
            case AVL_DVBTx_Standard_T2:
                r |= IRx_GetSQI_DVBT2(puiSQI, chip);
                break;
            default:
                return AVL_EC_GENERAL_FAIL;
            }
            break;
        case AVL_DVBSX:
            return AVL_EC_NOT_SUPPORTED; //currently, not avaiable
        case AVL_DVBC:
            return AVL_EC_NOT_SUPPORTED; //currently, not avaiable
        case AVL_ISDBT:
            return AVL_EC_NOT_SUPPORTED; //currently, not avaiable
        case AVL_DTMB:
            return AVL_EC_NOT_SUPPORTED; //currently, not avaiable
        default:
            return AVL_EC_GENERAL_FAIL;
        }
    } 
    else 
    {
        *puiSQI = 0;
    }

    return (r);
}

avl_error_code_t IRx_GetSSI(uint32_t * puiSSI, int32_t RF_Power, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;    
    uint32_t uiTemp = 0;
    AVL_DemodMode current_standard = AVL_DVBTX;
    static AVL_DVBTx_Standard current_dvbtx_std = AVL_DVBTx_Standard_T;
    uint8_t ucTemp = 0;
    AVL_LockStatus lock_status = AVL_STATUS_UNLOCK;

    r = avl_bms_read32(chip->chip_pub->i2c_addr,
            stBaseAddrSet.fw_config_reg_base + rs_current_active_mode_iaddr_offset,&uiTemp);
   current_standard = (AVL_DemodMode)uiTemp;

    switch(current_standard)
    {
    case AVL_DVBTX:
        //only reading DVBTx mode when locked, otherwise use previous value

        r = DVBTx_GetLockStatus_Demod( &ucTemp, chip );
        lock_status = (AVL_LockStatus)ucTemp;

        if(lock_status == AVL_STATUS_LOCK) 
        {
            r |= avl_bms_read8(chip->chip_pub->i2c_addr, stBaseAddrSet.fw_DVBTx_status_reg_base + rs_DVBTx_rx_mode_caddr_offset,&ucTemp);
            current_dvbtx_std = (AVL_DVBTx_Standard)ucTemp;
        }

        switch(current_dvbtx_std)
        {
        case AVL_DVBTx_Standard_T:
            r |= IRx_GetSSI_DVBT(puiSSI, RF_Power, chip);
            break;
        case AVL_DVBTx_Standard_T2:
            r |= IRx_GetSSI_DVBT2(puiSSI, RF_Power, chip);
            break;
        default:
            return AVL_EC_GENERAL_FAIL;
        }
        break;
    case AVL_DVBSX:
        return AVL_EC_NOT_SUPPORTED;
        break;
    case AVL_DVBC:
        return AVL_EC_NOT_SUPPORTED;
        break;
    case AVL_ISDBT:
        return AVL_EC_NOT_SUPPORTED;
        break;
    default:
        return AVL_EC_GENERAL_FAIL;
    }

    return (r);
}

typedef struct AVL_DVBT_Non_Hierarchical_CN_Table_Element
{
    AVL_DVBT_ModulationMode modulation;
    AVL_DVBT_CodeRate hp_code_rate;
    int32_t CN_NordigP1_x100_db;
}AVL_DVBT_Non_Hierarchical_CN_Table_Element;

AVL_DVBT_Non_Hierarchical_CN_Table_Element AVL_DVBT_Non_Hierarchical_CN_Table[]=
{
    //profile 1, Gaussian
    {AVL_DVBT_QPSK, AVL_DVBT_CR_1_2, 510},
    {AVL_DVBT_QPSK, AVL_DVBT_CR_2_3, 690},
    {AVL_DVBT_QPSK, AVL_DVBT_CR_3_4, 790},
    {AVL_DVBT_QPSK, AVL_DVBT_CR_5_6, 890},
    {AVL_DVBT_QPSK, AVL_DVBT_CR_7_8, 970},

    {AVL_DVBT_16QAM, AVL_DVBT_CR_1_2, 1080},
    {AVL_DVBT_16QAM, AVL_DVBT_CR_2_3, 1310},
    {AVL_DVBT_16QAM, AVL_DVBT_CR_3_4, 1460},
    {AVL_DVBT_16QAM, AVL_DVBT_CR_5_6, 1560},
    {AVL_DVBT_16QAM, AVL_DVBT_CR_7_8, 1600},

    {AVL_DVBT_64QAM, AVL_DVBT_CR_1_2, 1650},
    {AVL_DVBT_64QAM, AVL_DVBT_CR_2_3, 1870},
    {AVL_DVBT_64QAM, AVL_DVBT_CR_3_4, 2020},
    {AVL_DVBT_64QAM, AVL_DVBT_CR_5_6, 2160},
    {AVL_DVBT_64QAM, AVL_DVBT_CR_7_8, 2250}
};

typedef struct AVL_DVBT_Hierarchical_CN_Table_Element
{
    AVL_DVBT_Layer selected_layer;
    AVL_DVBT_ModulationMode modulation;
    AVL_DVBT_CodeRate code_rate;
    AVL_DVBT_Hierarchy hierarchy;
    int32_t CN_NordigP1_x100_db;
}AVL_DVBT_Hierarchical_CN_Table_Element;

AVL_DVBT_Hierarchical_CN_Table_Element AVL_DVBT_Hierarchical_CN_Table[]=
{
    //profile 1, Gaussian
    //For HP, only QPSK is used

    //64QAM
    {AVL_DVBT_LAYER_HP, AVL_DVBT_64QAM, AVL_DVBT_CR_1_2,AVL_DVBT_HIER_ALPHA_1,1090},
    {AVL_DVBT_LAYER_LP, AVL_DVBT_64QAM, AVL_DVBT_CR_1_2,AVL_DVBT_HIER_ALPHA_1,1670},

    {AVL_DVBT_LAYER_HP, AVL_DVBT_64QAM, AVL_DVBT_CR_2_3,AVL_DVBT_HIER_ALPHA_1,1410},
    {AVL_DVBT_LAYER_LP, AVL_DVBT_64QAM, AVL_DVBT_CR_2_3,AVL_DVBT_HIER_ALPHA_1,1910},

    {AVL_DVBT_LAYER_HP, AVL_DVBT_64QAM, AVL_DVBT_CR_3_4,AVL_DVBT_HIER_ALPHA_1,1570},
    {AVL_DVBT_LAYER_LP, AVL_DVBT_64QAM, AVL_DVBT_CR_3_4,AVL_DVBT_HIER_ALPHA_1,2090},

	{AVL_DVBT_LAYER_HP, AVL_DVBT_64QAM, AVL_DVBT_CR_5_6,AVL_DVBT_HIER_ALPHA_1,1940},
	{AVL_DVBT_LAYER_LP, AVL_DVBT_64QAM, AVL_DVBT_CR_5_6,AVL_DVBT_HIER_ALPHA_1,2250},

	{AVL_DVBT_LAYER_HP, AVL_DVBT_64QAM, AVL_DVBT_CR_7_8,AVL_DVBT_HIER_ALPHA_1,2060},
	{AVL_DVBT_LAYER_LP, AVL_DVBT_64QAM, AVL_DVBT_CR_7_8,AVL_DVBT_HIER_ALPHA_1,2370},



    {AVL_DVBT_LAYER_HP, AVL_DVBT_64QAM, AVL_DVBT_CR_1_2,AVL_DVBT_HIER_ALPHA_2,850},
    {AVL_DVBT_LAYER_LP, AVL_DVBT_64QAM, AVL_DVBT_CR_1_2,AVL_DVBT_HIER_ALPHA_2,1850},

    {AVL_DVBT_LAYER_HP, AVL_DVBT_64QAM, AVL_DVBT_CR_2_3,AVL_DVBT_HIER_ALPHA_2,1100},
    {AVL_DVBT_LAYER_LP, AVL_DVBT_64QAM, AVL_DVBT_CR_2_3,AVL_DVBT_HIER_ALPHA_2,2120},

    {AVL_DVBT_LAYER_HP, AVL_DVBT_64QAM, AVL_DVBT_CR_3_4,AVL_DVBT_HIER_ALPHA_2,1280},
    {AVL_DVBT_LAYER_LP, AVL_DVBT_64QAM, AVL_DVBT_CR_3_4,AVL_DVBT_HIER_ALPHA_2,2360},

	{AVL_DVBT_LAYER_HP, AVL_DVBT_64QAM, AVL_DVBT_CR_5_6,AVL_DVBT_HIER_ALPHA_2,1500},
	{AVL_DVBT_LAYER_LP, AVL_DVBT_64QAM, AVL_DVBT_CR_5_6,AVL_DVBT_HIER_ALPHA_2,2470},

	{AVL_DVBT_LAYER_HP, AVL_DVBT_64QAM, AVL_DVBT_CR_7_8,AVL_DVBT_HIER_ALPHA_2,1600},
	{AVL_DVBT_LAYER_LP, AVL_DVBT_64QAM, AVL_DVBT_CR_7_8,AVL_DVBT_HIER_ALPHA_2,2590},



	{AVL_DVBT_LAYER_HP, AVL_DVBT_64QAM, AVL_DVBT_CR_1_2,AVL_DVBT_HIER_ALPHA_4,800},
    {AVL_DVBT_LAYER_LP, AVL_DVBT_64QAM, AVL_DVBT_CR_1_2,AVL_DVBT_HIER_ALPHA_4,2190},

    {AVL_DVBT_LAYER_HP, AVL_DVBT_64QAM, AVL_DVBT_CR_2_3,AVL_DVBT_HIER_ALPHA_4,930},
    {AVL_DVBT_LAYER_LP, AVL_DVBT_64QAM, AVL_DVBT_CR_2_3,AVL_DVBT_HIER_ALPHA_4,2420},

    {AVL_DVBT_LAYER_HP, AVL_DVBT_64QAM, AVL_DVBT_CR_3_4,AVL_DVBT_HIER_ALPHA_4,1160},
    {AVL_DVBT_LAYER_LP, AVL_DVBT_64QAM, AVL_DVBT_CR_3_4,AVL_DVBT_HIER_ALPHA_4,2560},

	{AVL_DVBT_LAYER_HP, AVL_DVBT_64QAM, AVL_DVBT_CR_5_6,AVL_DVBT_HIER_ALPHA_4,1300},
	{AVL_DVBT_LAYER_LP, AVL_DVBT_64QAM, AVL_DVBT_CR_5_6,AVL_DVBT_HIER_ALPHA_4,2690},

	{AVL_DVBT_LAYER_HP, AVL_DVBT_64QAM, AVL_DVBT_CR_7_8,AVL_DVBT_HIER_ALPHA_4,1290},
	{AVL_DVBT_LAYER_LP, AVL_DVBT_64QAM, AVL_DVBT_CR_7_8,AVL_DVBT_HIER_ALPHA_4,2780},

    //16QAM
    {AVL_DVBT_LAYER_HP, AVL_DVBT_16QAM, AVL_DVBT_CR_1_2,AVL_DVBT_HIER_ALPHA_1,910},
    {AVL_DVBT_LAYER_LP, AVL_DVBT_16QAM, AVL_DVBT_CR_1_2,AVL_DVBT_HIER_ALPHA_1,1250},

    {AVL_DVBT_LAYER_HP, AVL_DVBT_16QAM, AVL_DVBT_CR_2_3,AVL_DVBT_HIER_ALPHA_1,1200},
    {AVL_DVBT_LAYER_LP, AVL_DVBT_16QAM, AVL_DVBT_CR_2_3,AVL_DVBT_HIER_ALPHA_1,1430},

    {AVL_DVBT_LAYER_HP, AVL_DVBT_16QAM, AVL_DVBT_CR_3_4,AVL_DVBT_HIER_ALPHA_1,1360},
    {AVL_DVBT_LAYER_LP, AVL_DVBT_16QAM, AVL_DVBT_CR_3_4,AVL_DVBT_HIER_ALPHA_1,1530},

    {AVL_DVBT_LAYER_HP, AVL_DVBT_16QAM, AVL_DVBT_CR_5_6,AVL_DVBT_HIER_ALPHA_1,1500},
    {AVL_DVBT_LAYER_LP, AVL_DVBT_16QAM, AVL_DVBT_CR_5_6,AVL_DVBT_HIER_ALPHA_1,1630},

	{AVL_DVBT_LAYER_HP, AVL_DVBT_16QAM, AVL_DVBT_CR_7_8,AVL_DVBT_HIER_ALPHA_1,1660},
    {AVL_DVBT_LAYER_LP, AVL_DVBT_16QAM, AVL_DVBT_CR_7_8,AVL_DVBT_HIER_ALPHA_1,1690},


	
    {AVL_DVBT_LAYER_HP, AVL_DVBT_16QAM, AVL_DVBT_CR_1_2,AVL_DVBT_HIER_ALPHA_2,680},
    {AVL_DVBT_LAYER_LP, AVL_DVBT_16QAM, AVL_DVBT_CR_1_2,AVL_DVBT_HIER_ALPHA_2,1500},

    {AVL_DVBT_LAYER_HP, AVL_DVBT_16QAM, AVL_DVBT_CR_2_3,AVL_DVBT_HIER_ALPHA_2,910},
    {AVL_DVBT_LAYER_LP, AVL_DVBT_16QAM, AVL_DVBT_CR_2_3,AVL_DVBT_HIER_ALPHA_2,1720},

    {AVL_DVBT_LAYER_HP, AVL_DVBT_16QAM, AVL_DVBT_CR_3_4,AVL_DVBT_HIER_ALPHA_2,1040},
    {AVL_DVBT_LAYER_LP, AVL_DVBT_16QAM, AVL_DVBT_CR_3_4,AVL_DVBT_HIER_ALPHA_2,1840},

    {AVL_DVBT_LAYER_HP, AVL_DVBT_16QAM, AVL_DVBT_CR_5_6,AVL_DVBT_HIER_ALPHA_2,1190},
    {AVL_DVBT_LAYER_LP, AVL_DVBT_16QAM, AVL_DVBT_CR_5_6,AVL_DVBT_HIER_ALPHA_2,1910},

	{AVL_DVBT_LAYER_HP, AVL_DVBT_16QAM, AVL_DVBT_CR_7_8,AVL_DVBT_HIER_ALPHA_2,1270},
    {AVL_DVBT_LAYER_LP, AVL_DVBT_16QAM, AVL_DVBT_CR_7_8,AVL_DVBT_HIER_ALPHA_2,2010},



    {AVL_DVBT_LAYER_HP, AVL_DVBT_16QAM, AVL_DVBT_CR_1_2,AVL_DVBT_HIER_ALPHA_4,580},
    {AVL_DVBT_LAYER_LP, AVL_DVBT_16QAM, AVL_DVBT_CR_1_2,AVL_DVBT_HIER_ALPHA_4,1950},

    {AVL_DVBT_LAYER_HP, AVL_DVBT_16QAM, AVL_DVBT_CR_2_3,AVL_DVBT_HIER_ALPHA_4,790},
    {AVL_DVBT_LAYER_LP, AVL_DVBT_16QAM, AVL_DVBT_CR_2_3,AVL_DVBT_HIER_ALPHA_4,2140},

    {AVL_DVBT_LAYER_HP, AVL_DVBT_16QAM, AVL_DVBT_CR_3_4,AVL_DVBT_HIER_ALPHA_4,910},
    {AVL_DVBT_LAYER_LP, AVL_DVBT_16QAM, AVL_DVBT_CR_3_4,AVL_DVBT_HIER_ALPHA_4,2250},

	{AVL_DVBT_LAYER_HP, AVL_DVBT_16QAM, AVL_DVBT_CR_5_6,AVL_DVBT_HIER_ALPHA_4,1030},
    {AVL_DVBT_LAYER_LP, AVL_DVBT_16QAM, AVL_DVBT_CR_5_6,AVL_DVBT_HIER_ALPHA_4,2370},

	{AVL_DVBT_LAYER_HP, AVL_DVBT_16QAM, AVL_DVBT_CR_7_8,AVL_DVBT_HIER_ALPHA_4,1210},
    {AVL_DVBT_LAYER_LP, AVL_DVBT_16QAM, AVL_DVBT_CR_7_8,AVL_DVBT_HIER_ALPHA_4,2470}
};

typedef struct DVBT2_CN_Table_Element
{
    AVL_DVBT2_PLP_ModulationMode modulation;
    AVL_DVBT2_CodeRate code_rate;
    int32_t CN_NordigP1_x100_db;
}DVBT2_CN_Table_Element;

DVBT2_CN_Table_Element DVBT2_RAW_CN_Table[]=
{
    //profile 1, Gaussian
    {AVL_DVBT2_QPSK, AVL_DVBT2_CR_1_3, -120}, //from DVB-S2 std
    {AVL_DVBT2_QPSK, AVL_DVBT2_CR_2_5, -030}, //from DVB-S2 std
    {AVL_DVBT2_QPSK, AVL_DVBT2_CR_1_2, 100},
    {AVL_DVBT2_QPSK, AVL_DVBT2_CR_3_5, 220},
    {AVL_DVBT2_QPSK, AVL_DVBT2_CR_2_3, 310},
    {AVL_DVBT2_QPSK, AVL_DVBT2_CR_3_4, 410},
    {AVL_DVBT2_QPSK, AVL_DVBT2_CR_4_5, 470},
    {AVL_DVBT2_QPSK, AVL_DVBT2_CR_5_6, 520},

    {AVL_DVBT2_16QAM, AVL_DVBT2_CR_1_3, 370},
    {AVL_DVBT2_16QAM, AVL_DVBT2_CR_2_5, 490},
    {AVL_DVBT2_16QAM, AVL_DVBT2_CR_1_2, 620},
    {AVL_DVBT2_16QAM, AVL_DVBT2_CR_3_5, 760},
    {AVL_DVBT2_16QAM, AVL_DVBT2_CR_2_3, 890},
    {AVL_DVBT2_16QAM, AVL_DVBT2_CR_3_4, 1000},
    {AVL_DVBT2_16QAM, AVL_DVBT2_CR_4_5, 1080},
    {AVL_DVBT2_16QAM, AVL_DVBT2_CR_5_6, 1130},

    {AVL_DVBT2_64QAM, AVL_DVBT2_CR_1_3, 760},
    {AVL_DVBT2_64QAM, AVL_DVBT2_CR_2_5, 920},
    {AVL_DVBT2_64QAM, AVL_DVBT2_CR_1_2, 1050},
    {AVL_DVBT2_64QAM, AVL_DVBT2_CR_3_5, 1230},
    {AVL_DVBT2_64QAM, AVL_DVBT2_CR_2_3, 1360},
    {AVL_DVBT2_64QAM, AVL_DVBT2_CR_3_4, 1510},
    {AVL_DVBT2_64QAM, AVL_DVBT2_CR_4_5, 1610},
    {AVL_DVBT2_64QAM, AVL_DVBT2_CR_5_6, 1670},

    {AVL_DVBT2_256QAM, AVL_DVBT2_CR_1_3, 1110},
    {AVL_DVBT2_256QAM, AVL_DVBT2_CR_2_5, 1290},
    {AVL_DVBT2_256QAM, AVL_DVBT2_CR_1_2, 1440},
    {AVL_DVBT2_256QAM, AVL_DVBT2_CR_3_5, 1670},
    {AVL_DVBT2_256QAM, AVL_DVBT2_CR_2_3, 1810},
    {AVL_DVBT2_256QAM, AVL_DVBT2_CR_3_4, 2000},
    {AVL_DVBT2_256QAM, AVL_DVBT2_CR_4_5, 2130},
    {AVL_DVBT2_256QAM, AVL_DVBT2_CR_5_6, 2200}
};

// DVBT2 pilot boosting correct CN table
typedef struct DVBT2_PBC_CN_Table_Element
{
    AVL_DVBT2_FFTSize fft_size;
    AVL_DVBT2_PILOT_PATTERN pilot_pattern;
    int32_t PCB_CN;
}DVBT2_PBC_CN_Table_Element;

DVBT2_PBC_CN_Table_Element DVBT2_PCB_CN_Table[]=
{
    {AVL_DVBT2_FFT_1K, AVL_DVBT2_PP_PP1, 34},
    {AVL_DVBT2_FFT_1K, AVL_DVBT2_PP_PP2, 32},
    {AVL_DVBT2_FFT_1K, AVL_DVBT2_PP_PP3, 44},
    {AVL_DVBT2_FFT_1K, AVL_DVBT2_PP_PP4, 42},
    {AVL_DVBT2_FFT_1K, AVL_DVBT2_PP_PP5, 48},
    {AVL_DVBT2_FFT_1K, AVL_DVBT2_PP_PP6, 0},
    {AVL_DVBT2_FFT_1K, AVL_DVBT2_PP_PP7, 29},
    {AVL_DVBT2_FFT_1K, AVL_DVBT2_PP_PP8, 0},

    {AVL_DVBT2_FFT_2K, AVL_DVBT2_PP_PP1, 35},
    {AVL_DVBT2_FFT_2K, AVL_DVBT2_PP_PP2, 33},
    {AVL_DVBT2_FFT_2K, AVL_DVBT2_PP_PP3, 43},
    {AVL_DVBT2_FFT_2K, AVL_DVBT2_PP_PP4, 42},
    {AVL_DVBT2_FFT_2K, AVL_DVBT2_PP_PP5, 47},
    {AVL_DVBT2_FFT_2K, AVL_DVBT2_PP_PP6, 0},
    {AVL_DVBT2_FFT_2K, AVL_DVBT2_PP_PP7, 29},
    {AVL_DVBT2_FFT_2K, AVL_DVBT2_PP_PP8, 0},

    {AVL_DVBT2_FFT_4K, AVL_DVBT2_PP_PP1, 39},
    {AVL_DVBT2_FFT_4K, AVL_DVBT2_PP_PP2, 37},
    {AVL_DVBT2_FFT_4K, AVL_DVBT2_PP_PP3, 47},
    {AVL_DVBT2_FFT_4K, AVL_DVBT2_PP_PP4, 45},
    {AVL_DVBT2_FFT_4K, AVL_DVBT2_PP_PP5, 51},
    {AVL_DVBT2_FFT_4K, AVL_DVBT2_PP_PP6, 0},
    {AVL_DVBT2_FFT_4K, AVL_DVBT2_PP_PP7, 34},
    {AVL_DVBT2_FFT_4K, AVL_DVBT2_PP_PP8, 0},

    {AVL_DVBT2_FFT_8K, AVL_DVBT2_PP_PP1, 41},
    {AVL_DVBT2_FFT_8K, AVL_DVBT2_PP_PP2, 39},
    {AVL_DVBT2_FFT_8K, AVL_DVBT2_PP_PP3, 49},
    {AVL_DVBT2_FFT_8K, AVL_DVBT2_PP_PP4, 48},
    {AVL_DVBT2_FFT_8K, AVL_DVBT2_PP_PP5, 53},
    {AVL_DVBT2_FFT_8K, AVL_DVBT2_PP_PP6, 0},
    {AVL_DVBT2_FFT_8K, AVL_DVBT2_PP_PP7, 37},
    {AVL_DVBT2_FFT_8K, AVL_DVBT2_PP_PP8, 37},

    {AVL_DVBT2_FFT_16K, AVL_DVBT2_PP_PP1, 41},
    {AVL_DVBT2_FFT_16K, AVL_DVBT2_PP_PP2, 38},
    {AVL_DVBT2_FFT_16K, AVL_DVBT2_PP_PP3, 49},
    {AVL_DVBT2_FFT_16K, AVL_DVBT2_PP_PP4, 47},
    {AVL_DVBT2_FFT_16K, AVL_DVBT2_PP_PP5, 52},
    {AVL_DVBT2_FFT_16K, AVL_DVBT2_PP_PP6, 49},
    {AVL_DVBT2_FFT_16K, AVL_DVBT2_PP_PP7, 33},
    {AVL_DVBT2_FFT_16K, AVL_DVBT2_PP_PP8, 35},

    {AVL_DVBT2_FFT_32K, AVL_DVBT2_PP_PP1, 0},
    {AVL_DVBT2_FFT_32K, AVL_DVBT2_PP_PP2, 37},
    {AVL_DVBT2_FFT_32K, AVL_DVBT2_PP_PP3, 48},
    {AVL_DVBT2_FFT_32K, AVL_DVBT2_PP_PP4, 45},
    {AVL_DVBT2_FFT_32K, AVL_DVBT2_PP_PP5, 0},
    {AVL_DVBT2_FFT_32K, AVL_DVBT2_PP_PP6, 48},
    {AVL_DVBT2_FFT_32K, AVL_DVBT2_PP_PP7, 33},
    {AVL_DVBT2_FFT_32K, AVL_DVBT2_PP_PP8, 35},
};

typedef struct AVL_DVBT_BERSQI_List
{
    uint32_t                 m_ber;
    uint32_t                 m_ber_sqi;
}AVL_DVBT_BERSQI_List;

AVL_DVBT_BERSQI_List DVBT_BERSQI_Table[]=
{
    {100       ,    40  },
    {178       ,    45  },
    {316       ,    50  },
    {562       ,    55  },
    {1000      ,    60  },
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

avl_error_code_t IRx_GetSQI_DVBT(uint32_t * puiSQI, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint32_t post_viterbi_ber_x1e9 = 0;
    uint32_t ber_sqi = 0;
    int32_t CN_NordigP1_x100_db = 0;
    uint32_t CN_received_x100_db = 0;
    int32_t CN_relative_x100_db = 0;
    uint32_t temp = 0;
    uint32_t temp1 = 0;    
    uint32_t i_temp = 0;
    uint16_t index = 0;
    static uint16_t usTemp = 0;

    AVL_DVBTxModulationInfo DVBTSignalInfo ;
    AVL_DVBT_ModulationMode modulation = AVL_DVBT_MOD_UNKNOWN;
    AVL_DVBT_Hierarchy hierarchy = AVL_DVBT_HIER_NONE;
    AVL_DVBT_CodeRate hp_code_rate = AVL_DVBT_CR_1_2;
    AVL_DVBT_Layer selected_layer = AVL_DVBT_LAYER_HP;

    //tweaked: This will cause the SQI to be a meaningful indicator at and around the "60s clean" C/N threshold.
    r = GetBER_Demod(&post_viterbi_ber_x1e9,AVL_POST_VITERBI_BER, chip);
    if (post_viterbi_ber_x1e9 <= 100)//RS BER <= 1e-7
    {
        ber_sqi = 100;
    }
    else if((post_viterbi_ber_x1e9 > 100)&&(post_viterbi_ber_x1e9 <= 1000000))//RS BER >1e-7, <=1e-3
    {
        //ber_sqi = (int32_t)(20.0*AVL_LOG10((int32_t)(1e9/post_viterbi_ber_x1e9))) - 40;
        temp = (uint32_t)(1000000000/post_viterbi_ber_x1e9);
        temp1 = sizeof(DVBT_BERSQI_Table)/sizeof(AVL_DVBT_BERSQI_List);
        for (i_temp = 0; i_temp < temp1; i_temp++)
        {
            if (temp < DVBT_BERSQI_Table[i_temp].m_ber)
            {
                break;
            }
        }
        if (0 == i_temp)
        {
            i_temp = 1;
        }
        ber_sqi = DVBT_BERSQI_Table[i_temp - 1].m_ber_sqi - 40;
    }
    else //RS BER >1e-3
    {
        ber_sqi = 15;
    }

    //get signal info for mapping
    r |= AVL_Demod_DVBTxGetModulationInfo(&DVBTSignalInfo, chip);

    modulation = DVBTSignalInfo.eDVBTSignalInfo.eDVBTModulationMode;
    hp_code_rate = DVBTSignalInfo.eDVBTSignalInfo.eDVBTHPCodeRate;
    hierarchy = DVBTSignalInfo.eDVBTSignalInfo.eDVBTHierarchy;
    
    //Get Nordig C/N
    if(AVL_DVBT_HIER_NONE == hierarchy)
    {
        for(index=0;index<sizeof(AVL_DVBT_Non_Hierarchical_CN_Table)/sizeof(AVL_DVBT_Non_Hierarchical_CN_Table_Element);index++)
        {
            if((AVL_DVBT_Non_Hierarchical_CN_Table[index].modulation == modulation)&&
                (AVL_DVBT_Non_Hierarchical_CN_Table[index].hp_code_rate == hp_code_rate))
            {
                CN_NordigP1_x100_db = AVL_DVBT_Non_Hierarchical_CN_Table[index].CN_NordigP1_x100_db;
                break;
            }
        }

        if(index == sizeof(AVL_DVBT_Non_Hierarchical_CN_Table)/sizeof(AVL_DVBT_Non_Hierarchical_CN_Table_Element))
        {
            return AVL_EC_NOT_SUPPORTED;
        }
    }
    else
    {
        //get selected layer
        r |= avl_bms_read8(chip->chip_pub->i2c_addr,stBaseAddrSet.fw_DVBTx_config_reg_base + rc_DVBTx_dvbt_layer_select_caddr_offset,(uint8_t *)&selected_layer);

        for(index=0;index<sizeof(AVL_DVBT_Hierarchical_CN_Table)/sizeof(AVL_DVBT_Hierarchical_CN_Table_Element);index++)
        {
            if((AVL_DVBT_Hierarchical_CN_Table[index].modulation == modulation)&&
                (AVL_DVBT_Hierarchical_CN_Table[index].code_rate == hp_code_rate)&&
                (AVL_DVBT_Hierarchical_CN_Table[index].selected_layer == selected_layer)&&
                (AVL_DVBT_Hierarchical_CN_Table[index].hierarchy == hierarchy))
            {
                CN_NordigP1_x100_db = AVL_DVBT_Hierarchical_CN_Table[index].CN_NordigP1_x100_db;
                break;
            }
        }

        if(index == sizeof(AVL_DVBT_Hierarchical_CN_Table)/sizeof(AVL_DVBT_Hierarchical_CN_Table_Element))
        {
            return AVL_EC_NOT_SUPPORTED;
        }
    }

    //Get received C/N
    r |= DVBTx_GetSNR_Demod(&CN_received_x100_db, chip);
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

    if ((CN_relative_x100_db > -700) && (1000000000 == post_viterbi_ber_x1e9))
    {
        *puiSQI = usTemp ;
        usTemp += 10;
        if(usTemp > 100)
        { 
            usTemp = 0; 
        }
    }

    return (r);
}

avl_error_code_t IRx_GetSQI_DVBT2(uint32_t * puiSQI, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint32_t post_ldpc_ber_x1e9 = 0;
    int32_t CN_NordigP1_x100_db = 0;
    uint32_t CN_received_x100_db = 0;
    int32_t CN_relative_x100_db = 0;

    AVL_DVBTxModulationInfo DVBTSignalInfo;
    AVL_DVBT2_PLP_ModulationMode modulation = AVL_DVBT2_QPSK;
    AVL_DVBT2_CodeRate code_rate = AVL_DVBT2_CR_1_2;
    uint16_t index = 0;
    uint16_t PCB_CN_index = 0;

    r = GetBER_Demod(&post_ldpc_ber_x1e9, AVL_POST_LDPC_BER, chip);

    //get signal info for mapping
    r |= AVL_Demod_DVBTxGetModulationInfo(&DVBTSignalInfo, chip);

    modulation = DVBTSignalInfo.eDVBT2SignalInfo.eDVBT2DataPLPModulationMode;
    code_rate = DVBTSignalInfo.eDVBT2SignalInfo.eDVBT2DataPLPCodeRate;

    //Get Nordig C/N
    for(index=0;index<sizeof(DVBT2_RAW_CN_Table)/sizeof(DVBT2_CN_Table_Element);index++)
    {
        if((DVBT2_RAW_CN_Table[index].modulation == modulation)&&
            (DVBT2_RAW_CN_Table[index].code_rate == code_rate))
        {
            CN_NordigP1_x100_db = DVBT2_RAW_CN_Table[index].CN_NordigP1_x100_db; // C/N in (C/N + A + B + C + D)

            CN_NordigP1_x100_db += 10; //actual 0.1  A in (C/N + A + B + C + D)

            PCB_CN_index = DVBTSignalInfo.eDVBT2SignalInfo.eDVBT2FFTSize*8 + DVBTSignalInfo.eDVBT2SignalInfo.eDVBT2PilotPatten;
            CN_NordigP1_x100_db += DVBT2_PCB_CN_Table[PCB_CN_index].PCB_CN;     //B (C/N + A + B + C + D)

            //Ignore different PPx, add 2db to compensate all of PPx
            CN_NordigP1_x100_db += 200; //actual 2.0  C in (C/N + A + B + C + D)
            
            //D in  (C/N + A + B + C + D)
            if(DVBTSignalInfo.eDVBT2SignalInfo.eDVBT2DataPLPModulationMode == AVL_DVBT2_QPSK ||
               DVBTSignalInfo.eDVBT2SignalInfo.eDVBT2DataPLPModulationMode == AVL_DVBT2_16QAM)
            {
                CN_NordigP1_x100_db += 5; //actual 0.05
            }
            else if(DVBTSignalInfo.eDVBT2SignalInfo.eDVBT2DataPLPModulationMode == AVL_DVBT2_64QAM)
            {
                if(DVBTSignalInfo.eDVBT2SignalInfo.eDVBT2DataPLPCodeRate == AVL_DVBT2_CR_1_2 ||
                DVBTSignalInfo.eDVBT2SignalInfo.eDVBT2DataPLPCodeRate == AVL_DVBT2_CR_3_5)
                {
                    CN_NordigP1_x100_db += 5; //actual 0.05
                }                
                else if(DVBTSignalInfo.eDVBT2SignalInfo.eDVBT2DataPLPCodeRate == AVL_DVBT2_CR_5_6)
                {
                    CN_NordigP1_x100_db += 25; //actual 0.17
                }
                else  // (AVL_DVBT2_CR_2_3) (AVL_DVBT2_CR_3_4)( AVL_DVBT2_CR_4_5 )
                {
                    CN_NordigP1_x100_db += 15; //actual 0.06                    
                }
            }
            else
            {
                if(DVBTSignalInfo.eDVBT2SignalInfo.eDVBT2DataPLPCodeRate == AVL_DVBT2_CR_1_2)
                {
                    CN_NordigP1_x100_db += 15; //actual 0.15
                } 
                else if(DVBTSignalInfo.eDVBT2SignalInfo.eDVBT2DataPLPCodeRate == AVL_DVBT2_CR_3_4)
                {
                    CN_NordigP1_x100_db += 45; //actual 0.45
                } 
                else if(DVBTSignalInfo.eDVBT2SignalInfo.eDVBT2DataPLPCodeRate == AVL_DVBT2_CR_4_5)
                {
                    CN_NordigP1_x100_db += 55; //actual 0.55
                }                
                else if(DVBTSignalInfo.eDVBT2SignalInfo.eDVBT2DataPLPCodeRate == AVL_DVBT2_CR_5_6)
                {
                    CN_NordigP1_x100_db += 65; //actual 0.65
                }
                else  // (AVL_DVBT2_CR_2_3)  (AVL_DVBT2_CR_3_5)
                {
                    CN_NordigP1_x100_db += 25; //actual 0.25        
                }
            }
            break;
        }
    }

    if(index == sizeof(DVBT2_RAW_CN_Table)/sizeof(DVBT2_CN_Table_Element))
    {
        return AVL_EC_NOT_SUPPORTED;
    }

    //Get received C/N
    r |= DVBTx_GetSNR_Demod(&CN_received_x100_db, chip);
    CN_received_x100_db += CN_received_x100_db/12; //adjust Rx C/N estimate to Tx C/N estimate

    //Calculate relative C/N
    CN_relative_x100_db = (int32_t)CN_received_x100_db - CN_NordigP1_x100_db;

    if(CN_relative_x100_db < -300)  
    {
        *puiSQI = 0;
    }
    else if((CN_relative_x100_db >= -300) && (CN_relative_x100_db <= 300))   
    {
        if((post_ldpc_ber_x1e9 <= 100))//BER < 1e-7
        {
            *puiSQI = (CN_relative_x100_db + 300)/6;
        }
        else if((post_ldpc_ber_x1e9 >= 100)&&(post_ldpc_ber_x1e9 <= 100000))//1e-7<= BER < 1e-4
        {
            *puiSQI = (CN_relative_x100_db + 300)/15;
        }
        else //BER >= 1e-4
        {
            *puiSQI = 0;
        }
    }
    else
    {
        *puiSQI = 100;
    }

    return (r);
}

typedef struct AVL_DVBT_RF_Table_Element
{
    AVL_DVBT_ModulationMode modulation;
    AVL_DVBT_CodeRate code_rate;
    int32_t Nordig_RF_Ref_dbm;
}AVL_DVBT_RF_Table_Element;

AVL_DVBT_RF_Table_Element AVL_DVBT_RF_TABLE[]=
{
    {AVL_DVBT_QPSK,AVL_DVBT_CR_1_2,-93},
    {AVL_DVBT_QPSK,AVL_DVBT_CR_2_3,-91},
    {AVL_DVBT_QPSK,AVL_DVBT_CR_3_4,-90},
    {AVL_DVBT_QPSK,AVL_DVBT_CR_5_6,-89},
    {AVL_DVBT_QPSK,AVL_DVBT_CR_7_8,-88},

    {AVL_DVBT_16QAM,AVL_DVBT_CR_1_2,-87},
    {AVL_DVBT_16QAM,AVL_DVBT_CR_2_3,-85},
    {AVL_DVBT_16QAM,AVL_DVBT_CR_3_4,-84},
    {AVL_DVBT_16QAM,AVL_DVBT_CR_5_6,-83},
    {AVL_DVBT_16QAM,AVL_DVBT_CR_7_8,-82},

    {AVL_DVBT_64QAM,AVL_DVBT_CR_1_2,-82},
    {AVL_DVBT_64QAM,AVL_DVBT_CR_2_3,-80},
    {AVL_DVBT_64QAM,AVL_DVBT_CR_3_4,-78},
    {AVL_DVBT_64QAM,AVL_DVBT_CR_5_6,-77},
    {AVL_DVBT_64QAM,AVL_DVBT_CR_7_8,-76}
};

typedef struct AVL_DVBT2_RF_Table_Element
{
    AVL_DVBT2_PLP_ModulationMode modulation;
    AVL_DVBT2_CodeRate code_rate;
    int32_t Nordig_RF_Ref_dbm;
}AVL_DVBT2_RF_Table_Element;

AVL_DVBT2_RF_Table_Element AVL_DVBT2_RF_TABLE[]=
{
    {AVL_DVBT2_QPSK,AVL_DVBT2_CR_1_3,-101},
    {AVL_DVBT2_QPSK,AVL_DVBT2_CR_2_5,-100},
    {AVL_DVBT2_QPSK,AVL_DVBT2_CR_1_2,-96},
    {AVL_DVBT2_QPSK,AVL_DVBT2_CR_3_5,-95},
    {AVL_DVBT2_QPSK,AVL_DVBT2_CR_2_3,-94},
    {AVL_DVBT2_QPSK,AVL_DVBT2_CR_3_4,-93},
    {AVL_DVBT2_QPSK,AVL_DVBT2_CR_4_5,-92},
    {AVL_DVBT2_QPSK,AVL_DVBT2_CR_5_6,-92},

    {AVL_DVBT2_16QAM,AVL_DVBT2_CR_1_3,-96},
    {AVL_DVBT2_16QAM,AVL_DVBT2_CR_2_5,-95},
    {AVL_DVBT2_16QAM,AVL_DVBT2_CR_1_2,-91},
    {AVL_DVBT2_16QAM,AVL_DVBT2_CR_3_5,-89},
    {AVL_DVBT2_16QAM,AVL_DVBT2_CR_2_3,-88},
    {AVL_DVBT2_16QAM,AVL_DVBT2_CR_3_4,-87},
    {AVL_DVBT2_16QAM,AVL_DVBT2_CR_4_5,-86},
    {AVL_DVBT2_16QAM,AVL_DVBT2_CR_5_6,-86},

    {AVL_DVBT2_64QAM,AVL_DVBT2_CR_1_3,-93},
    {AVL_DVBT2_64QAM,AVL_DVBT2_CR_2_5,-92},
    {AVL_DVBT2_64QAM,AVL_DVBT2_CR_1_2,-86},
    {AVL_DVBT2_64QAM,AVL_DVBT2_CR_3_5,-85},
    {AVL_DVBT2_64QAM,AVL_DVBT2_CR_2_3,-83},
    {AVL_DVBT2_64QAM,AVL_DVBT2_CR_3_4,-82},
    {AVL_DVBT2_64QAM,AVL_DVBT2_CR_4_5,-81},
    {AVL_DVBT2_64QAM,AVL_DVBT2_CR_5_6,-80},

    {AVL_DVBT2_256QAM,AVL_DVBT2_CR_1_3,-89},
    {AVL_DVBT2_256QAM,AVL_DVBT2_CR_2_5,-88},
    {AVL_DVBT2_256QAM,AVL_DVBT2_CR_1_2,-82},
    {AVL_DVBT2_256QAM,AVL_DVBT2_CR_3_5,-80},
    {AVL_DVBT2_256QAM,AVL_DVBT2_CR_2_3,-78},
    {AVL_DVBT2_256QAM,AVL_DVBT2_CR_3_4,-76},
    {AVL_DVBT2_256QAM,AVL_DVBT2_CR_4_5,-75},
    {AVL_DVBT2_256QAM,AVL_DVBT2_CR_5_6,-74}
};


avl_error_code_t IRx_GetSSI_DVBT(uint32_t * puiSSI, int32_t RF_Power, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    AVL_DVBTxModulationInfo DVBTSignalInfo;
    static AVL_DVBT_ModulationMode modulation = AVL_DVBT_QPSK;
    static AVL_DVBT_CodeRate hp_code_rate = AVL_DVBT_CR_1_2;
    uint16_t index = 0;
    int32_t RF_Power_Ref = 0;
    int32_t RF_Power_Rel = 0;
    AVL_LockStatus lock_status = AVL_STATUS_UNLOCK;
    uint8_t ucTemp = 0;

    r = DVBTx_GetLockStatus_Demod( &ucTemp, chip );
    lock_status = (AVL_LockStatus)ucTemp;

    if(lock_status == AVL_STATUS_LOCK) 
    {
        r |= AVL_Demod_DVBTxGetModulationInfo(&DVBTSignalInfo, chip);
        modulation = DVBTSignalInfo.eDVBTSignalInfo.eDVBTModulationMode;
        hp_code_rate = DVBTSignalInfo.eDVBTSignalInfo.eDVBTHPCodeRate;
    }

    for(index=0;index<sizeof(AVL_DVBT_RF_TABLE)/sizeof(AVL_DVBT_RF_Table_Element);index++)
    {
        if((AVL_DVBT_RF_TABLE[index].modulation == modulation)&&(AVL_DVBT_RF_TABLE[index].code_rate == hp_code_rate))
        {
            RF_Power_Ref = AVL_DVBT_RF_TABLE[index].Nordig_RF_Ref_dbm;
            break;
        }
    }
    if(index == sizeof(AVL_DVBT_RF_TABLE)/sizeof(AVL_DVBT_RF_Table_Element))
    {
        return AVL_EC_NOT_SUPPORTED;
    }

    RF_Power_Rel = RF_Power - RF_Power_Ref;

    if(RF_Power_Rel < -15)
    {
        *puiSSI = 0;
    }
    else if((RF_Power_Rel >= -15) && (RF_Power_Rel < 0))
    {
        *puiSSI = (RF_Power_Rel + 15)*2/3;
    }
    else if((RF_Power_Rel >= 0) && (RF_Power_Rel < 20))
    {
        *puiSSI = RF_Power_Rel*4 + 10;
    }
    else if((RF_Power_Rel >= 20) && (RF_Power_Rel < 35))
    {
        *puiSSI = (RF_Power_Rel - 20)*2/3 + 90;
    }
    else
    {
        *puiSSI = 100;
    }
    return (r);
}

avl_error_code_t IRx_GetSSI_DVBT2(uint32_t * puiSSI, int32_t RF_Power, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    AVL_DVBTxModulationInfo DVBTSignalInfo;
    static AVL_DVBT2_PLP_ModulationMode modulation = AVL_DVBT2_QPSK;
    static AVL_DVBT2_CodeRate code_rate = AVL_DVBT2_CR_1_2;
    uint16_t index = 0;
    int32_t RF_Power_Ref = 0;
    int32_t RF_Power_Rel = 0;
    AVL_LockStatus lock_status = AVL_STATUS_UNLOCK;
    uint8_t ucTemp = 0;

    r = DVBTx_GetLockStatus_Demod( &ucTemp, chip );
    lock_status = (AVL_LockStatus)ucTemp;

    if(lock_status == AVL_STATUS_LOCK) 
    {
        r |= AVL_Demod_DVBTxGetModulationInfo(&DVBTSignalInfo, chip);
        modulation = DVBTSignalInfo.eDVBT2SignalInfo.eDVBT2DataPLPModulationMode;
        code_rate = DVBTSignalInfo.eDVBT2SignalInfo.eDVBT2DataPLPCodeRate;
    }

    for(index=0; index < sizeof(AVL_DVBT2_RF_TABLE)/sizeof(AVL_DVBT2_RF_Table_Element);index++)
    {
        if((AVL_DVBT2_RF_TABLE[index].modulation == modulation)&&(AVL_DVBT2_RF_TABLE[index].code_rate == code_rate))
        {
            RF_Power_Ref = AVL_DVBT2_RF_TABLE[index].Nordig_RF_Ref_dbm;
            break;
        }
    }
    if(index == sizeof(AVL_DVBT2_RF_TABLE)/sizeof(AVL_DVBT2_RF_Table_Element))
    {
        return AVL_EC_NOT_SUPPORTED;
    }

    RF_Power_Rel = RF_Power - RF_Power_Ref;

    if(RF_Power_Rel < -15)
    {
        *puiSSI = 0;
    }
    else if((RF_Power_Rel >= -15) && (RF_Power_Rel < 0))
    {
        *puiSSI = (RF_Power_Rel + 15)*2/3;
    }
    else if((RF_Power_Rel >= 0) && (RF_Power_Rel < 20))
    {
        *puiSSI = RF_Power_Rel*4 + 10;
    }
    else if((RF_Power_Rel >= 20) && (RF_Power_Rel < 35))
    {
        *puiSSI = (RF_Power_Rel - 20)*2/3 + 90;
    }
    else
    {
        *puiSSI = 100;
    }

    return (r);
}

avl_error_code_t DVBTx_SetIFInputPath_Demod(AVL_InputPath eInputPath, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    r = avl_bms_write8(chip->chip_pub->i2c_addr,
        stBaseAddrSet.fw_DVBTx_config_reg_base + rc_DVBTx_adc_sel_caddr_offset, (uint8_t)eInputPath);

    return r;
}

avl_error_code_t DVBTx_SetIFFrequency_Demod(uint32_t uiIFFrequencyHz, avl68x2_chip *chip)
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
        stBaseAddrSet.fw_DVBTx_config_reg_base + rc_DVBTx_nom_carrier_freq_Hz_iaddr_offset, carrier_offset_hz);

    return r;
}

avl_error_code_t DVBTx_SetBandWidth_Demod(AVL_DVBTxBandWidth eBandWidth, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    r = avl_bms_write32(chip->chip_pub->i2c_addr,
        stBaseAddrSet.fw_DVBTx_config_reg_base + rc_DVBTx_fund_rate_Hz_iaddr_offset,BW_FFT_Table[eBandWidth]);

    return r;
}


avl_error_code_t DVBTx_GetPrePostBER_Demod(uint32_t *puiBERxe9, AVL_BER_Type eBERType, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint32_t uiTemp = 0;
    
    switch(eBERType)
    {
        case AVL_PRE_VITERBI_BER:
            *puiBERxe9 = 0;
            break;
        case AVL_POST_VITERBI_BER:
             r = avl_bms_read32(chip->chip_pub->i2c_addr,
                 stBaseAddrSet.fw_DVBTx_status_reg_base + rs_DVBTx_post_viterbi_BER_estimate_x10M_iaddr_offset,&uiTemp);
            *puiBERxe9 = uiTemp * 100;//match 1e9
            break;
        case AVL_PRE_LDPC_BER:
             r = avl_bms_read32(chip->chip_pub->i2c_addr,
                 stBaseAddrSet.fw_DVBTx_status_reg_base + rs_DVBTx_pre_LDPC_BER_estimate_x10M_iaddr_offset,&uiTemp);
            *puiBERxe9 = uiTemp * 100;//match 1e9
            break;
        case AVL_POST_LDPC_BER:
            r = avl_bms_read32(chip->chip_pub->i2c_addr,
                stBaseAddrSet.fw_DVBTx_status_reg_base + rs_DVBTx_post_LDPC_BER_estimate_x1B_iaddr_offset,&uiTemp);
            *puiBERxe9 = uiTemp;
            break;
        default:
            break;
    }

    return r;
}

avl_error_code_t DVBTx_GetCellID_Demod(uint16_t *puiCellID,  avl68x2_chip *chip)
{
	avl_error_code_t r = AVL_EC_OK;
    uint8_t ucTemp = 0;
	uint16_t usTemp = 0;
    AVL_DVBTx_Standard current_standard = AVL_DVBTx_Standard_T;
	
	r = avl_bms_read8(chip->chip_pub->i2c_addr, 
        stBaseAddrSet.fw_DVBTx_status_reg_base + rs_DVBTx_rx_mode_caddr_offset,&ucTemp);

    current_standard = (AVL_DVBTx_Standard)ucTemp;

    if(AVL_DVBTx_Standard_T2 == current_standard)
	{
	    r |= avl_bms_read16(chip->chip_pub->i2c_addr, 
			stBaseAddrSet.fw_DVBT2_L1_pre_reg_base + rs_DVBTx_CELL_ID_saddr_offset,&usTemp);
	    *puiCellID =usTemp ;
	}
    else if(AVL_DVBTx_Standard_T == current_standard)
	{
		r |= avl_bms_read16(chip->chip_pub->i2c_addr, 
			stBaseAddrSet.fw_DVBT_TPS_reg_base +  rs_DVBTx_TPS_cell_id_saddr_offset,&usTemp);
	    *puiCellID =usTemp ;

	}
	return r;
}
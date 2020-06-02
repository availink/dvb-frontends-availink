// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Availink AVL68x2 DVB-S/S2/T/T2/C, ISDB-T, J83.B demodulator driver
 *
 * Copyright (C) 2020 Availink, Inc. (gpl@availink.com)
 *
 */
#include "avl68x2_common.h"
#include "avl68x2_dvbsx.h"

#define Diseqc_delay 20

avl_error_code_t AVL_Demod_DVBSxAutoLock(
	uint32_t uiSymbolRateSps,
	avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    AVL_FunctionalMode enumFunctionalMode = AVL_FuncMode_BlindScan;

    if(1 == chip->chip_priv->sleep_flag)
    {
        r = AVL_EC_SLEEP;
        return r;
    }

    r = AVL_Demod_DVBSx_GetFunctionalMode(&enumFunctionalMode, chip);

    r |= IBase_SendRxOPWait_Demod(AVL_FW_CMD_HALT, chip);

    if(enumFunctionalMode == AVL_FuncMode_Demod)
    {
    
        r |= avl_bms_write16(chip->chip_pub->i2c_addr,
                                stBaseAddrSet.fw_DVBSx_status_reg_base +
				rs_DVBSx_fec_lock_saddr_offset, 0);

        r |= avl_bms_write16(chip->chip_pub->i2c_addr, 
                                stBaseAddrSet.fw_DVBSx_config_reg_base +
				rc_DVBSx_decode_mode_saddr_offset, 0x14);

        r |= avl_bms_write16(chip->chip_pub->i2c_addr, 
                                stBaseAddrSet.fw_DVBSx_config_reg_base +
				rc_DVBSx_fec_bypass_coderate_saddr_offset, 
                                0);//DVBS auto lock

        r |= avl_bms_write16(chip->chip_pub->i2c_addr, 
                                stBaseAddrSet.fw_DVBSx_config_reg_base +
				rc_DVBSx_iq_mode_saddr_offset, 
                                1);//enable spectrum auto detection
        
        r |= avl_bms_write16(chip->chip_pub->i2c_addr, 
                                stBaseAddrSet.fw_DVBSx_config_reg_base +
				rc_DVBSx_fec_bypass_coderate_saddr_offset, 0);

	r |= avl_bms_write32(chip->chip_pub->i2c_addr, 
                                stBaseAddrSet.fw_DVBSx_config_reg_base +
				rc_DVBSx_int_sym_rate_MHz_iaddr_offset, 
                                uiSymbolRateSps);
      

        r |= IBase_SendRxOPWait_Demod(AVL_FW_CMD_ACQUIRE, chip);
    }
    else if(enumFunctionalMode == AVL_FuncMode_BlindScan)
    {
      return AVL_EC_NOT_SUPPORTED;
    }

    return (r);
}

avl_error_code_t AVL_Demod_DVBSxGetModulationInfo(AVL_DVBSxModulationInfo *pstModulationInfo, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint32_t uiTemp = 0;
    uint8_t  temp_uchar = 0;
    
    r = avl_bms_read32(chip->chip_pub->i2c_addr, stBaseAddrSet.fw_DVBSx_status_reg_base + rs_DVBSx_pilot_iaddr_offset, &uiTemp);
    pstModulationInfo->eDVBSxPilot = (AVL_DVBSx_Pilot)(uiTemp);

    r |= avl_bms_read32(chip->chip_pub->i2c_addr, stBaseAddrSet.fw_DVBSx_config_reg_base + rc_DVBSx_internal_decode_mode_iaddr_offset,&uiTemp);
    pstModulationInfo->eDVBSxStandard = (AVL_DVBSx_Standard)uiTemp;

    if(AVL_DVBS == (AVL_DVBSx_Standard)uiTemp)
    {
        r |= avl_bms_read32(chip->chip_pub->i2c_addr, stBaseAddrSet.fw_DVBSx_config_reg_base + rc_DVBSx_dvbs_fec_coderate_iaddr_offset,&uiTemp);
        pstModulationInfo->eDVBSCodeRate = (AVL_DVBS_CodeRate)(uiTemp);
    }
    else
    {
        r |= avl_bms_read8(chip->chip_pub->i2c_addr, stBaseAddrSet.fw_DVBSx_config_reg_base + rc_DVBSx_dvbs2_fec_coderate_caddr_offset,&temp_uchar);
        pstModulationInfo->eDVBS2CodeRate = (AVL_DVBS2_CodeRate)(temp_uchar);
    }

    r |= avl_bms_read32(chip->chip_pub->i2c_addr, stBaseAddrSet.fw_DVBSx_status_reg_base + rs_DVBSx_modulation_iaddr_offset, &uiTemp);
    pstModulationInfo->eDVBSxModulationMode = (AVL_DVBSx_ModulationMode)(uiTemp);
    
    r |= avl_bms_read32(chip->chip_pub->i2c_addr, stBaseAddrSet.fw_DVBSx_status_reg_base + rs_DVBSx_detected_alpha_iaddr_offset, &uiTemp);
    pstModulationInfo->eDVBSxRollOff = (AVL_DVBSx_RollOff)(uiTemp);

    return (r);

}

avl_error_code_t AVL_Demod_DVBSx_BlindScan_Start(AVL_BlindScanPara *pBSPara, avl68x2_chip *chip)
{
	avl_error_code_t r = AVL_EC_OK;
	uint16_t uiMinSymRate = 0;

	r |= avl_bms_write16(chip->chip_pub->i2c_addr,
			     stBaseAddrSet.fw_DVBSx_config_reg_base +
			     rc_DVBSx_tuner_LPF_100kHz_saddr_offset,
			     pBSPara->tuner_lpf_100khz);
	r |= avl_bms_write16(chip->chip_pub->i2c_addr,
			     stBaseAddrSet.fw_DVBSx_config_reg_base +
			     rc_DVBSx_blind_scan_tuner_spectrum_inversion_saddr_offset,
			     chip->chip_pub->tuner_pol);

	uiMinSymRate = pBSPara->min_symrate_khz - 200; // give some tolerance

	if (uiMinSymRate < 800) //Blind scan doesn't support symbol rate less then 1M, give 200K margin
	{
		uiMinSymRate = 800;
	}

	if(r)
		return r;

	r |= avl_bms_write16(chip->chip_pub->i2c_addr,
			     stBaseAddrSet.fw_DVBSx_config_reg_base +
				 rc_DVBSx_tuner_frequency_100kHz_saddr_offset,
			     pBSPara->tuner_center_freq_100khz);
	r |= avl_bms_write16(chip->chip_pub->i2c_addr,
			     stBaseAddrSet.fw_DVBSx_config_reg_base +
				 rc_DVBSx_blind_scan_min_sym_rate_kHz_saddr_offset,
			     uiMinSymRate);
	r |= avl_bms_write16(chip->chip_pub->i2c_addr,
			     stBaseAddrSet.fw_DVBSx_config_reg_base +
				 rc_DVBSx_blind_scan_max_sym_rate_kHz_saddr_offset,
			     (pBSPara->max_symrate_khz) + 200);
	r |= avl_bms_write16(chip->chip_pub->i2c_addr,
			     stBaseAddrSet.fw_DVBSx_config_reg_base +
				 rc_DVBSx_blind_scan_start_freq_100kHz_saddr_offset,
			     (pBSPara->tuner_center_freq_100khz -
			      pBSPara->tuner_lpf_100khz));
	
	r |= avl_bms_write16(chip->chip_pub->i2c_addr,
			     stBaseAddrSet.fw_DVBSx_status_reg_base +
				 rc_DVBSx_blind_scan_freq_step_100kHz_saddr_offset,
			     80); //always step by at least 8MHz

	r |= avl_bms_write16(chip->chip_pub->i2c_addr,
			     stBaseAddrSet.fw_DVBSx_status_reg_base +
				 rs_DVBSx_blind_scan_progress_saddr_offset,
			     0);

	if(r)
		return r;
	
	r |= IBase_SendRxOPWait_Demod(AVL_FW_CMD_HALT, chip);
	r |= IBase_SendRxOPWait_Demod(AVL_FW_CMD_BLIND_SCAN, chip);
	
	return (r);
}

avl_error_code_t AVL_Demod_DVBSx_BlindScan_GetStatus(
	AVL_BlindScanPara *pBSPara,
	AVL_BSInfo *pBSInfo,
	avl68x2_chip *chip)
{
	avl_error_code_t r = AVL_EC_OK;
	uint16_t next_start_freq_100khz;

	r = avl_bms_read16(chip->chip_pub->i2c_addr,
			   stBaseAddrSet.fw_DVBSx_status_reg_base +
			       rs_DVBSx_blind_scan_progress_saddr_offset,
			   &(pBSInfo->progress));
	r |= avl_bms_read16(chip->chip_pub->i2c_addr,
			    stBaseAddrSet.fw_DVBSx_status_reg_base +
				rs_DVBSx_blind_scan_channel_count_saddr_offset,
			    &(pBSInfo->num_carriers));
	r |= avl_bms_read16(chip->chip_pub->i2c_addr,
			    stBaseAddrSet.fw_DVBSx_config_reg_base +
				rc_DVBSx_blind_scan_start_freq_100kHz_saddr_offset,
			    &next_start_freq_100khz);
	r |= avl_bms_read16(chip->chip_pub->i2c_addr,
			    stBaseAddrSet.fw_DVBSx_status_reg_base +
				rs_DVBSx_blind_scan_error_code_saddr_offset,
			    &(pBSInfo->result_code));
	if (pBSInfo->progress > 100)
	{
		pBSInfo->progress = 100;
	}

	pBSInfo->next_tuner_center_freq_100khz = next_start_freq_100khz +
		pBSPara->tuner_lpf_100khz;

	return (r);
}

avl_error_code_t AVL_Demod_DVBSx_BlindScan_Cancel(avl68x2_chip *chip)
{
    avl_error_code_t r;
    enum AVL_FunctionalMode enumFunctionalMode = AVL_FuncMode_Demod;

    r = AVL_Demod_DVBSx_GetFunctionalMode(&enumFunctionalMode, chip);

    if(enumFunctionalMode == AVL_FuncMode_BlindScan)
    {
        r |= IBase_SendRxOPWait_Demod(AVL_FW_CMD_HALT, chip);
    }
    else
    {
        r = AVL_EC_GENERAL_FAIL;
    }

    return(r);
}

avl_error_code_t AVL_Demod_DVBSx_BlindScan_ReadChannelInfo(uint16_t max_chans, AVL_ChannelInfo *pChannel, avl68x2_chip *chip)
{
	avl_error_code_t r = 0;
	uint32_t channel_addr = 0;
	uint16_t i1 = 0;
	uint16_t i2 = 0;
	uint32_t uiMinFreq = 0;
	uint16_t iMinIdx = 0;
	uint16_t n_chans;
	AVL_ChannelInfo sTempChannel;

	r = avl_bms_read16(chip->chip_pub->i2c_addr,
			   stBaseAddrSet.fw_DVBSx_status_reg_base +
			       rs_DVBSx_blind_scan_channel_count_saddr_offset,
			   &i1);
	n_chans = max_chans;
	if (n_chans > i1)
	{
		n_chans = i1;
	}

	r |= avl_bms_read16(chip->chip_pub->i2c_addr,
			    stBaseAddrSet.fw_DVBSx_config_reg_base +
				rc_DVBSx_blind_scan_channel_info_offset_saddr_offset,
			    &i1);

	channel_addr = stBaseAddrSet.hw_blind_scan_info_base;

	for (i1 = 0; i1 < n_chans; i1++)
	{
		r |= avl_bms_read32(chip->chip_pub->i2c_addr,
				    channel_addr,
				    &(pChannel[i1].rf_freq_khz));
		channel_addr += 4;
		r |= avl_bms_read32(chip->chip_pub->i2c_addr,
				    channel_addr,
				    &(pChannel[i1].symbol_rate_hz));
		channel_addr += 4;
		r |= avl_bms_read32(chip->chip_pub->i2c_addr,
				    channel_addr,
				    &(pChannel[i1].flags));
		channel_addr += 4;
	}

	// Sort the results
	for (i1 = 0; i1 < n_chans; i1++)
	{
		iMinIdx = i1;
		uiMinFreq = pChannel[i1].rf_freq_khz;
		for (i2 = (i1 + 1); i2 < n_chans; i2++)
		{
			if (pChannel[i2].rf_freq_khz < uiMinFreq)
			{
				uiMinFreq = pChannel[i2].rf_freq_khz;
				iMinIdx = i2;
			}
		}
		sTempChannel = pChannel[iMinIdx];
		pChannel[iMinIdx] = pChannel[i1];
		pChannel[i1] = sTempChannel;
	}

	return (r);
}

avl_error_code_t AVL_Demod_DVBSx_BlindScan_Reset(avl68x2_chip *chip)
{
    avl_error_code_t r = 0;
    
    r = avl_bms_write16(chip->chip_pub->i2c_addr,
        stBaseAddrSet.fw_DVBSx_config_reg_base + rc_DVBSx_blind_scan_reset_saddr_offset, 1);

    return(r);
}

avl_error_code_t AVL_Demod_DVBSx_SetFunctionalMode(AVL_FunctionalMode enumFunctionalMode,avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    r = avl_bms_write16(chip->chip_pub->i2c_addr,
        stBaseAddrSet.fw_DVBSx_config_reg_base + rc_DVBSx_functional_mode_saddr_offset, (uint16_t)enumFunctionalMode);
    r |= avl_bms_write16(chip->chip_pub->i2c_addr,
        stBaseAddrSet.fw_DVBSx_config_reg_base + rc_DVBSx_iq_mode_saddr_offset,0);

    return(r);
}

avl_error_code_t AVL_Demod_DVBSx_GetFunctionalMode(AVL_FunctionalMode * pFunctionalMode, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;    
    uint16_t uiTemp = 0;
    
    r = avl_bms_read16(chip->chip_pub->i2c_addr,
        stBaseAddrSet.fw_DVBSx_config_reg_base + rc_DVBSx_functional_mode_saddr_offset, &uiTemp);
    *pFunctionalMode = (AVL_FunctionalMode)(uiTemp & 0x0001);   

    return(r);
}

avl_error_code_t AVL_Demod_DVBSx_SetDishPointingMode( AVL_Switch enumOn_Off, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    AVL_FunctionalMode enumFunctionalMode = AVL_FuncMode_BlindScan;


    r |= AVL_Demod_DVBSx_GetFunctionalMode(&enumFunctionalMode, chip);
    if(enumFunctionalMode == AVL_FuncMode_Demod)
    {
        if(enumOn_Off == AVL_ON)
        {
            r |= avl_bms_write16(chip->chip_pub->i2c_addr,
                stBaseAddrSet.fw_DVBSx_config_reg_base + rc_DVBSx_aagc_acq_gain_saddr_offset, 12);
            r |= avl_bms_write16(chip->chip_pub->i2c_addr,
                stBaseAddrSet.fw_DVBSx_config_reg_base + rc_DVBSx_dishpoint_mode_saddr_offset, 1);
        }
        else
        {
            r |= avl_bms_write16(chip->chip_pub->i2c_addr,
                stBaseAddrSet.fw_DVBSx_config_reg_base + rc_DVBSx_aagc_acq_gain_saddr_offset, 10);
            r |= avl_bms_write16(chip->chip_pub->i2c_addr,
                stBaseAddrSet.fw_DVBSx_config_reg_base + rc_DVBSx_dishpoint_mode_saddr_offset, 0);
        }
    }
    else
    {
        r = AVL_EC_GENERAL_FAIL;
    }

    return(r);
}

#define Diseqc_delay 20

avl_error_code_t AVL_Demod_DVBSx_Diseqc_IsSafeToSwitchMode(avl68x2_chip *chip)
{
	avl_error_code_t r = AVL_EC_OK;
	uint32_t i1 = 0;

	switch (chip->chip_pub->eDiseqcStatus)
	{
	case AVL_DOS_InModulation:
	case AVL_DOS_InTone:
		r = avl_bms_read32(chip->chip_pub->i2c_addr,
				   stBaseAddrSet.hw_diseqc_base + hw_diseqc_tx_st_offset, &i1);
		if (1 != ((i1 & 0x00000040) >> 6)) //check if the last transmit is done
		{
			r |= AVL_EC_RUNNING;
		}
		break;
	case AVL_DOS_InContinuous:
	case AVL_DOS_Initialized:
		break;
	default:
		r = AVL_EC_GENERAL_FAIL;
		break;
	}

	return (r);
}

avl_error_code_t AVL_Demod_DVBSx_Diseqc_ReadModulationData( uint8_t * pucBuff, uint8_t * pucSize, avl68x2_chip *chip)
{
    avl_error_code_t r = 0;
    uint32_t i1 = 0;
    uint32_t i2 = 0;
    uint8_t pucBuffTemp[4] = {0};
    
    r = avl_bsp_wait_semaphore(&(chip->diseqc_sem));
    r |= avl_bms_read32(chip->chip_pub->i2c_addr,
        stBaseAddrSet.hw_diseqc_base + hw_diseqc_rx_st_offset, &i1);
    r |= avl_bms_read32(chip->chip_pub->i2c_addr,
        stBaseAddrSet.hw_diseqc_base + hw_diseqc_tx_cntrl_offset, &i2);
    if((i2>>8) & 0x01)
    {
        chip->chip_pub->eDiseqcStatus = AVL_DOS_InModulation; 
    }
    
    if( AVL_DOS_InModulation == chip->chip_pub->eDiseqcStatus )
    {
        // In modulation mode
        if( (!((i2>>8) & 0x01 ) && (0x00000004 == (i1 & 0x00000004))) || (((i2>>8) & 0x01 ) &&(0x00000004 != (i1 & 0x00000004))))
        {
            *pucSize = (uint8_t)((i1 & 0x00000078)>>3);
            //Receive data
            for( i1=0; i1<*pucSize; i1++ )
            {
                r |= avl_bms_read(chip->chip_pub->i2c_addr,
                    stBaseAddrSet.hw_diseqc_base + hw_rx_fifo_map_offset, pucBuffTemp, 4);
                    pucBuff[i1] = pucBuffTemp[3];
            }
        }
        else
        {
            r = AVL_EC_GENERAL_FAIL;
        }
    }
    else
    {
        r = AVL_EC_GENERAL_FAIL;
    }

    r |= avl_bsp_release_semaphore(&(chip->diseqc_sem));
    
    return (r);
}
//#include "stdio.h"
avl_error_code_t AVL_Demod_DVBSx_Diseqc_SendModulationData( uint8_t * pucBuff, uint8_t ucSize, avl68x2_chip *chip)
{
    avl_error_code_t r = 0;
    uint32_t i1 = 0;
    uint32_t i2 = 0;
    uint8_t pucBuffTemp[8] = {0};
    uint8_t Continuousflag = 0;
    uint16_t uiTempOutTh = 0;

    if( ucSize>8 )
    {
        r = AVL_EC_WARNING;
    }
    else
    {
        r = avl_bsp_wait_semaphore(&(chip->diseqc_sem));   
        r |= AVL_Demod_DVBSx_Diseqc_IsSafeToSwitchMode(chip);
        if( AVL_EC_OK ==  r)
        {
            if (chip->chip_pub->eDiseqcStatus == AVL_DOS_InContinuous)
            {
                r |= avl_bms_read32(chip->chip_pub->i2c_addr,
                    stBaseAddrSet.hw_diseqc_base + hw_diseqc_tx_cntrl_offset, &i1);
                if ((i1>>10) & 0x01)
                {
                    Continuousflag = 1;
                    i1 &= 0xfffff3ff;
                    r |= avl_bms_write32(chip->chip_pub->i2c_addr,
                        stBaseAddrSet.hw_diseqc_base + hw_diseqc_tx_cntrl_offset, i1);
                    r |= avl_bsp_delay(Diseqc_delay);      //delay 20ms
                }
            }
            //reset rx_fifo
            r |= avl_bms_read32(chip->chip_pub->i2c_addr,
            stBaseAddrSet.hw_diseqc_base + hw_diseqc_rx_cntrl_offset, &i2);
            r |= avl_bms_write32(chip->chip_pub->i2c_addr,
                stBaseAddrSet.hw_diseqc_base + hw_diseqc_rx_cntrl_offset, (i2|0x01));
            r |= avl_bms_write32(chip->chip_pub->i2c_addr,
                stBaseAddrSet.hw_diseqc_base + hw_diseqc_rx_cntrl_offset, (i2&0xfffffffe));

            r |= avl_bms_read32(chip->chip_pub->i2c_addr,
                stBaseAddrSet.hw_diseqc_base + hw_diseqc_tx_cntrl_offset, &i1);
            i1 &= 0xfffffff8;   //set to modulation mode and put it to FIFO load mode
            r |= avl_bms_write32(chip->chip_pub->i2c_addr,
                stBaseAddrSet.hw_diseqc_base + hw_diseqc_tx_cntrl_offset, i1);

            avl_int_to_3bytes(stBaseAddrSet.hw_diseqc_base + hw_tx_fifo_map_offset, pucBuffTemp);
            pucBuffTemp[3] = 0;
            pucBuffTemp[4] = 0;
            pucBuffTemp[5] = 0;
            for( i2=0; i2<ucSize; i2++ )
            {
                pucBuffTemp[6] = pucBuff[i2];

                r |= avl_bms_write(chip->chip_pub->i2c_addr, pucBuffTemp, 7);
            }                           
            i1 |= (1<<2);  //start fifo transmit.
            r |= avl_bms_write32(chip->chip_pub->i2c_addr,
                stBaseAddrSet.hw_diseqc_base + hw_diseqc_tx_cntrl_offset, i1);

            if( AVL_EC_OK == r )
            {
                chip->chip_pub->eDiseqcStatus = AVL_DOS_InModulation;
            }
            do 
            {
                r |= avl_bsp_delay(3);
                if (++uiTempOutTh > 500)
                {
                    r |= AVL_EC_TIMEOUT;
                    r |= avl_bsp_release_semaphore(&(chip->diseqc_sem));
                    return(r);
                }
                r = avl_bms_read32(chip->chip_pub->i2c_addr,
                    stBaseAddrSet.hw_diseqc_base + hw_diseqc_tx_st_offset, &i1);
            } while ( 1 != ((i1 & 0x00000040) >> 6) );
          
            r = avl_bsp_delay(Diseqc_delay);       //delay 20ms
            if (Continuousflag == 1)            //resume to send out wave
            {
                //No data in FIFO
                r |= avl_bms_read32(chip->chip_pub->i2c_addr,
                stBaseAddrSet.hw_diseqc_base + hw_diseqc_tx_cntrl_offset, &i1);
                i1 &= 0xfffffff8; 
                i1 |= 0x03;     //switch to continuous mode
                r |= avl_bms_write32(chip->chip_pub->i2c_addr,
                    stBaseAddrSet.hw_diseqc_base + hw_diseqc_tx_cntrl_offset, i1);

                //start to send out wave
                i1 |= (1<<10);  
                r |= avl_bms_write32(chip->chip_pub->i2c_addr,
                    stBaseAddrSet.hw_diseqc_base + hw_diseqc_tx_cntrl_offset, i1);
                if( AVL_EC_OK == r )
                {
                    chip->chip_pub->eDiseqcStatus = AVL_DOS_InContinuous;
                }
            }
        }
        r |= avl_bsp_release_semaphore(&(chip->diseqc_sem));
    }

    return (r);
}

avl_error_code_t AVL_Demod_DVBSx_Diseqc_GetTxStatus( AVL_Diseqc_TxStatus * pTxStatus, avl68x2_chip *chip)
{
    avl_error_code_t r = 0;
    uint32_t i1 = 0;

    r = avl_bsp_wait_semaphore(&(chip->diseqc_sem));

    r |= avl_bms_read32(chip->chip_pub->i2c_addr,
            stBaseAddrSet.hw_diseqc_base + hw_diseqc_tx_st_offset, &i1);
    pTxStatus->m_TxDone = (uint8_t)((i1 & 0x00000040)>>6);
    pTxStatus->m_TxFifoCount = (uint8_t)((i1 & 0x0000003c)>>2);

    r |= avl_bsp_release_semaphore(&(chip->diseqc_sem));
    
    return (r);
}

avl_error_code_t AVL_Demod_DVBSx_Diseqc_GetRxStatus( AVL_Diseqc_RxStatus * pRxStatus, avl68x2_chip *chip)
{
    avl_error_code_t r = 0;
    uint32_t i1 = 0;
    
    r = avl_bsp_wait_semaphore(&(chip->diseqc_sem));
    if( AVL_DOS_InModulation == chip->chip_pub->eDiseqcStatus )
    {
        r |= avl_bms_read32(chip->chip_pub->i2c_addr,
            stBaseAddrSet.hw_diseqc_base + hw_diseqc_rx_st_offset, &i1);
        pRxStatus->m_RxDone = (uint8_t)((i1 & 0x00000004)>>2);
        pRxStatus->m_RxFifoCount = (uint8_t)((i1 & 0x000000078)>>3);
        r |= avl_bms_read32(chip->chip_pub->i2c_addr,
            stBaseAddrSet.hw_diseqc_base + hw_diseqc_rx_parity_offset, &i1);
        pRxStatus->m_RxFifoParChk = (uint8_t)(i1 & 0x000000ff);
    }
    else
    {
        r |= AVL_EC_GENERAL_FAIL;
    }
    r |= avl_bsp_release_semaphore(&(chip->diseqc_sem));
    
    return (r);
}

avl_error_code_t AVL_Demod_DVBSx_Diseqc_SendTone( uint8_t ucTone, uint8_t ucCount, avl68x2_chip *chip)
{
    avl_error_code_t r = 0;
    uint32_t i1 = 0;
    uint32_t i2 = 0;
    uint8_t pucBuffTemp[8] = {0};
    uint8_t Continuousflag = 0;
    uint16_t uiTempOutTh = 0;

    if( ucCount>8 )
    {
        r = AVL_EC_WARNING;
    }
    else
    {
        r = avl_bsp_wait_semaphore(&(chip->diseqc_sem));
        r |= AVL_Demod_DVBSx_Diseqc_IsSafeToSwitchMode(chip);

        if( AVL_EC_OK == r )
        {
            if (chip->chip_pub->eDiseqcStatus == AVL_DOS_InContinuous)
            {
                r |= avl_bms_read32(chip->chip_pub->i2c_addr,
                    stBaseAddrSet.hw_diseqc_base + hw_diseqc_tx_cntrl_offset, &i1);
                if ((i1>>10) & 0x01)
                {
                    Continuousflag = 1;
                    i1 &= 0xfffff3ff;
                    r |= avl_bms_write32(chip->chip_pub->i2c_addr,
                        stBaseAddrSet.hw_diseqc_base + hw_diseqc_tx_cntrl_offset, i1);
                    r |= avl_bsp_delay(Diseqc_delay);      //delay 20ms
                }
            }
            //No data in the FIFO.
            r |= avl_bms_read32(chip->chip_pub->i2c_addr,
            stBaseAddrSet.hw_diseqc_base + hw_diseqc_tx_cntrl_offset, &i1);
            i1 &= 0xfffffff8;  //put it into the FIFO load mode.
            if( 0 == ucTone )
            {
                i1 |= 0x01;
            }
            else
            {
                i1 |= 0x02;
            }
            r |= avl_bms_write32(chip->chip_pub->i2c_addr,
                stBaseAddrSet.hw_diseqc_base + hw_diseqc_tx_cntrl_offset, i1);

            avl_int_to_3bytes(stBaseAddrSet.hw_diseqc_base + hw_tx_fifo_map_offset, pucBuffTemp);
            pucBuffTemp[3] = 0;
            pucBuffTemp[4] = 0;
            pucBuffTemp[5] = 0;
            pucBuffTemp[6] = 1;

            for( i2=0; i2<ucCount; i2++ )
            {
                r |= avl_bms_write(chip->chip_pub->i2c_addr, pucBuffTemp, 7);
            }

            i1 |= (1<<2);  //start fifo transmit.
            r |= avl_bms_write32(chip->chip_pub->i2c_addr,
                stBaseAddrSet.hw_diseqc_base + hw_diseqc_tx_cntrl_offset, i1);
            if( AVL_EC_OK == r )
            {
                chip->chip_pub->eDiseqcStatus = AVL_DOS_InTone;
            }
            do 
            {
                r |= avl_bsp_delay(1);
                if (++uiTempOutTh > 500)
                {
                    r |= AVL_EC_TIMEOUT;
                    r |= avl_bsp_release_semaphore(&(chip->diseqc_sem));
                    return(r);
                }
                r = avl_bms_read32(chip->chip_pub->i2c_addr,
                    stBaseAddrSet.hw_diseqc_base + hw_diseqc_tx_st_offset, &i1);
            } while ( 1 != ((i1 & 0x00000040) >> 6) );

            r = avl_bsp_delay(Diseqc_delay);       //delay 20ms
            if (Continuousflag == 1)            //resume to send out wave
            {
                //No data in FIFO
                r |= avl_bms_read32(chip->chip_pub->i2c_addr,
                stBaseAddrSet.hw_diseqc_base + hw_diseqc_tx_cntrl_offset, &i1);
                i1 &= 0xfffffff8; 
                i1 |= 0x03;     //switch to continuous mode
                r |= avl_bms_write32(chip->chip_pub->i2c_addr,
                    stBaseAddrSet.hw_diseqc_base + hw_diseqc_tx_cntrl_offset, i1);

                //start to send out wave
                i1 |= (1<<10);  
                r |= avl_bms_write32(chip->chip_pub->i2c_addr,
                    stBaseAddrSet.hw_diseqc_base + hw_diseqc_tx_cntrl_offset, i1);
                if( AVL_EC_OK == r )
                {
                    chip->chip_pub->eDiseqcStatus = AVL_DOS_InContinuous;
                }
            }
        }
        r |= avl_bsp_release_semaphore(&(chip->diseqc_sem));
    }
    
    return (r);
}

avl_error_code_t AVL_Demod_DVBSx_Diseqc_StartContinuous(avl68x2_chip *chip)
{
	avl_error_code_t r = 0;
	uint32_t i1 = 0;

	r = avl_bsp_wait_semaphore(&(chip->diseqc_sem));
	r |= AVL_Demod_DVBSx_Diseqc_IsSafeToSwitchMode(chip);

	if (AVL_EC_OK == r)
	{
		//No data in FIFO
		r |= avl_bms_read32(chip->chip_pub->i2c_addr,
				    stBaseAddrSet.hw_diseqc_base +
					hw_diseqc_tx_cntrl_offset,
				    &i1);
		i1 &= 0xfffffff8;
		i1 |= 0x3; //switch to continuous mode
		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     stBaseAddrSet.hw_diseqc_base +
					 hw_diseqc_tx_cntrl_offset,
				     i1);

		i1 |= (1 << 10); //start to send out wave
		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     stBaseAddrSet.hw_diseqc_base +
					 hw_diseqc_tx_cntrl_offset,
				     i1);
		if (AVL_EC_OK == r)
		{
			chip->chip_pub->eDiseqcStatus = AVL_DOS_InContinuous;
		}
	}
	r |= avl_bsp_release_semaphore(&(chip->diseqc_sem));

	return (r);
}

avl_error_code_t AVL_Demod_DVBSx_Diseqc_StopContinuous(avl68x2_chip *chip)
{
	avl_error_code_t r = 0;
	uint32_t i1 = 0;

	r = avl_bsp_wait_semaphore(&(chip->diseqc_sem));
	if (AVL_DOS_InContinuous == chip->chip_pub->eDiseqcStatus)
	{
		r |= avl_bms_read32(chip->chip_pub->i2c_addr,
				    stBaseAddrSet.hw_diseqc_base +
					hw_diseqc_tx_cntrl_offset,
				    &i1);
		i1 &= 0xfffff3ff;
		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     stBaseAddrSet.hw_diseqc_base +
					 hw_diseqc_tx_cntrl_offset,
				     i1);

		if (r == AVL_EC_OK)
		{
			chip->chip_pub->eDiseqcStatus = AVL_DOS_Initialized;
		}
	}

	r |= avl_bsp_release_semaphore(&(chip->diseqc_sem));
	return (r);
}

avl_error_code_t DVBSx_Initialize_Demod(avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    AVL_Diseqc_Para stDiseqcConfig;

    r = avl_bms_write16(chip->chip_pub->i2c_addr,
        stBaseAddrSet.fw_DVBSx_config_reg_base + rc_DVBSx_int_mpeg_clk_MHz_saddr_offset, 
        chip->uiTSFrequencyHz/10000);
    r |= avl_bms_write16(chip->chip_pub->i2c_addr,
        stBaseAddrSet.fw_DVBSx_config_reg_base + rc_DVBSx_int_fec_clk_MHz_saddr_offset,
        chip->uiCoreFrequencyHz/10000);

    r |= avl_bms_write16(chip->chip_pub->i2c_addr, 
        stBaseAddrSet.fw_DVBSx_config_reg_base + rc_DVBSx_int_adc_clk_MHz_saddr_offset,
        chip->uiADCFrequencyHz/10000);
    r |= avl_bms_write16(chip->chip_pub->i2c_addr,
        stBaseAddrSet.fw_DVBSx_config_reg_base + rc_DVBSx_int_dmd_clk_MHz_saddr_offset,
        chip->uiDDCFrequencyHz/10000);

    r |= DVBSx_SetAGCPola(chip->chip_pub->dvbsx_config.eDVBSxAGCPola, chip);

    r |= avl_bms_write32(chip->chip_pub->i2c_addr, 
        stBaseAddrSet.fw_DVBSx_config_reg_base + rc_DVBSx_format_iaddr_offset, 
        AVL_OFFBIN);//Offbin
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, 
        stBaseAddrSet.fw_DVBSx_config_reg_base + rc_DVBSx_input_iaddr_offset, 
        AVL_ADC_IN);//ADC in

    r |= avl_bms_write16(chip->chip_pub->i2c_addr,
        stBaseAddrSet.fw_DVBSx_config_reg_base + rc_DVBSx_IF_Offset_10kHz_saddr_offset,
        0);

    r |= avl_bms_write16(chip->chip_pub->i2c_addr, 
            stBaseAddrSet.fw_DVBSx_config_reg_base + rc_DVBSx_show_detail_saddr_offset, 
            0);// 1: print more std_out information

    r |= ConfigAGCOutput_Demod(chip);


   /* r |= avl_bms_write16(chip->chip_pub->i2c_addr,
      stBaseAddrSet.fw_DVBSx_config_reg_base + rc_DVBSx_auto_pnd_collect_frames_saddr_offset, 3);*/


    stDiseqcConfig.eRxTimeout = AVL_DRT_150ms;
    stDiseqcConfig.eRxWaveForm = chip->chip_pub->dvbsx_config.e22KWaveForm;
    stDiseqcConfig.uiToneFrequencyKHz = 22;
    stDiseqcConfig.eTXGap = AVL_DTXG_15ms;
    stDiseqcConfig.eTxWaveForm = AVL_DWM_Normal;
           
    r |= DVBSx_Diseqc_Initialize_Demod(&stDiseqcConfig,chip);

    return (r);
}

avl_error_code_t DVBSx_Diseqc_Initialize_Demod(AVL_Diseqc_Para *pDiseqcPara, avl68x2_chip *chip)
{
	avl_error_code_t r = AVL_EC_OK;
	uint32_t i1 = 0;

	r = avl_bsp_wait_semaphore(&(chip->diseqc_sem));
	if (AVL_EC_OK == r)
	{
		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     stBaseAddrSet.hw_diseqc_base +
					 hw_diseqc_srst_offset,
				     1);

		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     stBaseAddrSet.hw_diseqc_base +
					 hw_diseqc_samp_frac_n_offset,
				     2000000); //2M=200*10kHz
		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     stBaseAddrSet.hw_diseqc_base +
					 hw_diseqc_samp_frac_d_offset,
				     chip->uiDDCFrequencyHz);

		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     stBaseAddrSet.hw_diseqc_base +
					 hw_diseqc_tone_frac_n_offset,
				     (pDiseqcPara->uiToneFrequencyKHz * 2));
		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     stBaseAddrSet.hw_diseqc_base +
					 hw_diseqc_tone_frac_d_offset,
				     (chip->uiDDCFrequencyHz / 1000));

		// Initialize the tx_control
		r |= avl_bms_read32(chip->chip_pub->i2c_addr,
				    stBaseAddrSet.hw_diseqc_base +
					hw_diseqc_tx_cntrl_offset,
				    &i1);
		i1 &= 0x00000300;
		i1 |= 0x20; //reset tx_fifo
		i1 |= ((uint32_t)(pDiseqcPara->eTXGap) << 6);
		i1 |= ((uint32_t)(pDiseqcPara->eTxWaveForm) << 4);
		i1 |= (1 << 3); //enable tx gap.
		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     stBaseAddrSet.hw_diseqc_base +
					 hw_diseqc_tx_cntrl_offset,
				     i1);
		i1 &= ~(0x20); //release tx_fifo reset
		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     stBaseAddrSet.hw_diseqc_base +
					 hw_diseqc_tx_cntrl_offset,
				     i1);

		// Initialize the rx_control
		i1 = ((uint32_t)(pDiseqcPara->eRxWaveForm) << 2);
		i1 |= (1 << 1); //active the receiver
		i1 |= (1 << 3); //envelop high when tone present
		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     stBaseAddrSet.hw_diseqc_base +
					 hw_diseqc_rx_cntrl_offset,
				     i1);

		i1 = (uint32_t)(pDiseqcPara->eRxTimeout);
		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     stBaseAddrSet.hw_diseqc_base +
					 hw_diseqc_rx_msg_tim_offset,
				     i1);

		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     stBaseAddrSet.hw_diseqc_base +
					 hw_diseqc_srst_offset,
				     0);

		if (AVL_EC_OK == r)
		{
			chip->chip_pub->eDiseqcStatus = AVL_DOS_Initialized;
		}

		//turn off LNB voltage
		r = avl68x2_demod_set_gpio(AVL_Pin37, AVL_LOGIC_0, chip);
		r |= avl68x2_demod_set_gpio(AVL_Pin38, AVL_LOGIC_0, chip);
	}
	r |= avl_bsp_release_semaphore(&(chip->diseqc_sem));

	return (r);
}

avl_error_code_t DVBSx_GetLockStatus_Demod( uint8_t * pucLocked, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint16_t usTemp = 0;

    r = avl_bms_read16(chip->chip_pub->i2c_addr,
        stBaseAddrSet.fw_DVBSx_status_reg_base + rs_DVBSx_fec_lock_saddr_offset, &usTemp);

    if(0x1 == usTemp)
    {
        *pucLocked = 1;
    }
    else
    {
        *pucLocked = 0;
    }
    
    return r;
}

avl_error_code_t DVBSx_GetSNR_Demod(uint32_t * puiSNR_db, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    r = avl_bms_read32(chip->chip_pub->i2c_addr,
        stBaseAddrSet.fw_DVBSx_status_reg_base + rs_DVBSx_int_SNR_dB_iaddr_offset,
        puiSNR_db);
    if( (*puiSNR_db) > 10000 )
    {
        // Not get stable SNR value yet.
        *puiSNR_db = 0;
    }

    return r;
}

avl_error_code_t DVBSx_GetSignalQuality_Demod(uint16_t * puiQuality , avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint32_t uiTemp = 0;

    r = DVBSx_GetSNR_Demod(&uiTemp, chip);

    if(uiTemp > 2500)
    {
        *puiQuality = 100;
    }
    else
    {
        *puiQuality = uiTemp*100/2500;
    }

    return r;
}


avl_error_code_t DVBSx_SetAGCPola(AVL_AGCPola enumAGC_Pola, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    r = avl_bms_write32(chip->chip_pub->i2c_addr, 
        stBaseAddrSet.fw_DVBSx_config_reg_base + rc_DVBSx_rfagc_pol_iaddr_offset, (uint32_t)enumAGC_Pola);

    return r;
}

avl_error_code_t DVBSx_GetPrePostBER_Demod(uint32_t *puiBERxe9, AVL_BER_Type eBERType, avl68x2_chip *chip)
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
                 stBaseAddrSet.fw_DVBSx_status_reg_base + rs_DVBSx_post_viterbi_BER_estimate_x10M_iaddr_offset,&uiTemp);
            *puiBERxe9 = uiTemp * 100;//match 1e9
            break;
        case AVL_PRE_LDPC_BER:
            r = avl_bms_read32(chip->chip_pub->i2c_addr,
                stBaseAddrSet.fw_DVBSx_status_reg_base + rs_DVBSx_pre_LDPC_BER_estimate_x10M_iaddr_offset,&uiTemp);
            *puiBERxe9 = uiTemp * 100;//match 1e9
            break;
        case AVL_POST_LDPC_BER:
            r = avl_bms_read32(chip->chip_pub->i2c_addr,
                stBaseAddrSet.fw_DVBSx_status_reg_base + rs_DVBSx_post_LDPC_BER_estimate_x10M_iaddr_offset,&uiTemp);
            *puiBERxe9 = uiTemp * 100;//match 1e9
            break;
        default:
            break;
    }

    return r;
}
avl_error_code_t AVL_Demod_DVBSx_GetFreqOffset( int32_t * piFreqOffsetHz, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    int16_t FreqOffset_100KHz;
    
    r = avl_bms_read16(chip->chip_pub->i2c_addr,
        stBaseAddrSet.fw_DVBSx_status_reg_base + rs_DVBSx_int_carrier_freq_100kHz_saddr_offset, (uint16_t *)&FreqOffset_100KHz);
    *piFreqOffsetHz = (int32_t)(FreqOffset_100KHz*100000);

    return r;
}


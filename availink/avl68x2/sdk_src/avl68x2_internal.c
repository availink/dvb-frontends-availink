// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Availink AVL68x2 DVB-S/S2/T/T2/C, ISDB-T, J83.B demodulator driver
 *
 * Copyright (C) 2020 Availink, Inc. (gpl@availink.com)
 *
 */

#include "avl68x2_common.h"
#include "avl68x2_dvbc.h"
#include "avl68x2_dvbsx.h"
#include "avl68x2_dvbtx.h"
#include "avl68x2_isdbt.h"

const AVL_BaseAddressSet stBaseAddrSet = {
  0x110840, //hw_mcu_reset_base           
  0x110010, //hw_mcu_system_reset_base    
  0x132050, //hw_esm_base                 
  0x118000, //hw_tuner_i2c_base           
  0x124000, //hw_gpio_control_base        
  0x120000, //hw_gpio_debug_base          
  0x124010, //hw_gpio_modu_002_base       
  0x10c000, //hw_emerald_io_base          
  0x130420, //hw_TS_tri_state_cntrl_base  
  0x120000, //hw_AGC_tri_state_cntrl_base 
  0x16c000, //hw_diseqc_base              
  0x2912b4, //hw_plp_list_base            
  0x200C00, //hw_blind_scan_info_base     
  0x108000, //hw_member_ID_base           
  0x110048, //hw_dma_sys_status_base      
  0x110050, //hw_dma_sys_cmd_base         
  0x140000, //hw_dvb_gen1_fec__base       
  0x200,    //fw_config_reg_base          
  0x0a4,    //fw_status_reg_base          
  0xa00,    //fw_DVBTx_config_reg_base    
  0x800,    //fw_DVBTx_status_reg_base    
  0x868,    //fw_DVBT2_data_PLP_config_reg_base     
  0x884,    //fw_DVBT2_common_PLP_config_reg_base   
  0x800,    //fw_DVBT2_P1_reg_base                  
  0x808,    //fw_DVBT2_L1_pre_reg_base              
  0x830,    //fw_DVBT2_L1_post_config_reg_base      
  0x8f0,    //fw_DVBT_TPS_reg_base                  
  0xe00,    //fw_DVBSx_config_reg_base    
  0xc00,    //fw_DVBSx_status_reg_base    
  0xa00,    //fw_ISDBT_config_reg_base    
  0x800,    //fw_ISDBT_status_reg_base    
  0x600,    //fw_DVBC_config_reg_base     
  0x400    //fw_DVBC_status_reg_base         
};

avl_error_code_t avl68x2_init_chip_object(avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    if(chip->rx_sem_initialized == 0) {
      r |= avl_bsp_init_semaphore(&(chip->rx_sem));
      printk("INITED RX SEM\n");
      chip->rx_sem_initialized = 1;
    }
    if(chip->diseqc_sem_initialized == 0) {
      r |= avl_bsp_init_semaphore(&(chip->diseqc_sem));
      printk("INITED DISEQC SEM\n");
      chip->diseqc_sem_initialized = 1;
    }    

    r |= avl_bms_initialize(chip->chip_pub->i2c_addr);

    chip->chip_priv->agc_driven = 0;

    chip->chip_pub->eDiseqcStatus = AVL_DOS_Uninitialized;

    return (r);
}

avl_error_code_t IBase_Initialize_Demod(avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint8_t dl_patch_parse_format = 0;
    uint32_t patch_idx = 0;
    uint32_t patch_script_version = 0;

    if ((chip->chip_priv->patch_data[0] & 0x0f0) == 0x10)
    {
        dl_patch_parse_format = 1;
    }
    else
    {
        return AVL_EC_GENERAL_FAIL; //Neither format enumeration was found. Firmware File is Corrupt
    }

    r |= avl_bms_write32(chip->chip_pub->i2c_addr,
                         stBaseAddrSet.hw_mcu_system_reset_base, 1);

    // Configure the PLL
    r |= SetPLL_Demod(chip);
    if (r)
        return r;

    if (dl_patch_parse_format)
    {
        r |= avl_bms_write32(chip->chip_pub->i2c_addr,
                             stBaseAddrSet.fw_status_reg_base + rs_core_ready_word_iaddr_offset,
                             0x00000000);

        r |= avl_bms_write32(chip->chip_pub->i2c_addr,
                             stBaseAddrSet.hw_mcu_system_reset_base, 0);

        patch_script_version = AVL_patch_read32(
            chip->chip_priv->patch_data,
            &patch_idx, 1);

        if ((patch_script_version & 0x00ff) == 0x1)
        {
            //read patch script version
            r |= AVL_ParseFwPatch_v0(chip, 0);
        }
        else
        {
            r |= AVL_EC_GENERAL_FAIL;
        }
    }
    return r;
}

avl_error_code_t TunerI2C_Initialize_Demod(avl68x2_chip *chip)
{
	avl_error_code_t r = AVL_EC_OK;
	uint32_t bit_rpt_divider = 0;
	uint32_t uiTemp = 0;
	r = avl_bms_write32(chip->chip_pub->i2c_addr,
			    stBaseAddrSet.hw_tuner_i2c_base +
				tuner_i2c_srst_offset,
			    1);
	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     stBaseAddrSet.hw_tuner_i2c_base +
				 tuner_i2c_bit_rpt_cntrl_offset,
			     0x6);
	r |= avl_bms_read32(chip->chip_pub->i2c_addr,
			    stBaseAddrSet.hw_tuner_i2c_base +
				tuner_i2c_cntrl_offset,
			    &uiTemp);
	uiTemp = (uiTemp & 0xFFFFFFFE);
	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     stBaseAddrSet.hw_tuner_i2c_base +
				 tuner_i2c_cntrl_offset,
			     uiTemp);

	bit_rpt_divider = (0x2A) *
			  (chip->uiCoreFrequencyHz / 1000) / (240 * 1000);

	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     stBaseAddrSet.hw_tuner_i2c_base +
				 tuner_i2c_bit_rpt_clk_div_offset,
			     bit_rpt_divider);
	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     stBaseAddrSet.hw_tuner_i2c_base +
				 tuner_i2c_srst_offset,
			     0);

	return r;
}

avl_error_code_t EnableTSOutput_Demod(avl68x2_chip *chip)
{

    avl_error_code_t r = AVL_EC_OK;

    r = avl_bms_write32(chip->chip_pub->i2c_addr, 
       stBaseAddrSet.hw_TS_tri_state_cntrl_base, 0);

    return r;
}

avl_error_code_t DisableTSOutput_Demod(avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    r = avl_bms_write32(chip->chip_pub->i2c_addr, 
        stBaseAddrSet.hw_TS_tri_state_cntrl_base, 0xfff);

    return r;
}

avl_error_code_t InitErrorStat_Demod(avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    AVL_ErrorStatConfig stErrorStatConfig;

    stErrorStatConfig.eErrorStatMode = AVL_ERROR_STAT_AUTO;
    stErrorStatConfig.eAutoErrorStatType = AVL_ERROR_STAT_TIME;
    stErrorStatConfig.uiTimeThresholdMs = 3000;
    stErrorStatConfig.uiNumberThresholdByte = 0;


    r = ErrorStatMode_Demod(stErrorStatConfig, chip);
    r |= ResetPER_Demod(chip);

    return (r);
}

avl_error_code_t ErrorStatMode_Demod(
    AVL_ErrorStatConfig stErrorStatConfig,
    avl68x2_chip *chip)
{
	avl_error_code_t r = AVL_EC_OK;
	struct avl_uint64 time_tick_num = {0, 0};

	r = avl_bms_write32(chip->chip_pub->i2c_addr,
			    stBaseAddrSet.hw_esm_base + esm_mode_offset,
			    (uint32_t)stErrorStatConfig.eErrorStatMode);
	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     stBaseAddrSet.hw_esm_base + tick_type_offset,
			     (uint32_t)stErrorStatConfig.eAutoErrorStatType);

	avl_mult_32to64(&time_tick_num, chip->uiTSFrequencyHz / 1000,
			stErrorStatConfig.uiTimeThresholdMs);
	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     stBaseAddrSet.hw_esm_base + time_tick_low_offset,
			     time_tick_num.low_word);
	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     stBaseAddrSet.hw_esm_base + time_tick_high_offset,
			     time_tick_num.high_word);

	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     stBaseAddrSet.hw_esm_base + byte_tick_low_offset,
			     stErrorStatConfig.uiTimeThresholdMs);
	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     stBaseAddrSet.hw_esm_base + byte_tick_high_offset,
			     0); //high 32-bit is not used

	if (stErrorStatConfig.eErrorStatMode == AVL_ERROR_STAT_AUTO) //auto mode
	{
		//reset auto error stat
		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     stBaseAddrSet.hw_esm_base + tick_clear_offset, 0);
		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     stBaseAddrSet.hw_esm_base + tick_clear_offset, 1);
		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     stBaseAddrSet.hw_esm_base + tick_clear_offset, 0);
	}

	return (r);
}

avl_error_code_t ResetErrorStat_Demod(avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    r = ResetPER_Demod(chip);

    return r;
}

avl_error_code_t ResetPER_Demod(avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint32_t uiTemp = 0;

    chip->stAVLErrorStat.usLostLock = 0;
    chip->stAVLErrorStat.stSwCntPktErrors.high_word = 0;
    chip->stAVLErrorStat.stSwCntPktErrors.low_word = 0;
    chip->stAVLErrorStat.stSwCntNumPkts.high_word = 0;
    chip->stAVLErrorStat.stSwCntNumPkts.low_word = 0;
    chip->stAVLErrorStat.stPktErrors.high_word = 0;
    chip->stAVLErrorStat.stPktErrors.low_word = 0;
    chip->stAVLErrorStat.stNumPkts.high_word = 0;
    chip->stAVLErrorStat.stNumPkts.low_word = 0;

    r |= avl_bms_read32(chip->chip_pub->i2c_addr, 
        stBaseAddrSet.hw_esm_base + esm_cntrl_offset, &uiTemp);
    uiTemp |= 0x00000001;
    r |= avl_bms_write32(chip->chip_pub->i2c_addr,
        stBaseAddrSet.hw_esm_base + esm_cntrl_offset, uiTemp);

    r |= avl_bms_read32(chip->chip_pub->i2c_addr, 
        stBaseAddrSet.hw_esm_base + esm_cntrl_offset, &uiTemp);
    uiTemp |= 0x00000008;
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, 
        stBaseAddrSet.hw_esm_base + esm_cntrl_offset, uiTemp);

    uiTemp |= 0x00000001;
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, 
        stBaseAddrSet.hw_esm_base + esm_cntrl_offset, uiTemp);
    uiTemp &= 0xFFFFFFFE;
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, 
        stBaseAddrSet.hw_esm_base + esm_cntrl_offset, uiTemp);

    return r;
}

avl_error_code_t ResetBER_Demod(AVL_BERConfig *pstErrorStatConfig, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint32_t uiLFSRSynced = 0;
    uint32_t uiTemp = 0;
    uint32_t uiCnt = 0;
    uint32_t uiByteCnt = 0;
    uint32_t uiBER_FailCnt = 0;
    uint32_t uiBitErrors = 0;


    chip->stAVLErrorStat.usLostLock = 0;
    chip->stAVLErrorStat.usLFSRSynced = 0;
    chip->stAVLErrorStat.stSwCntBitErrors.high_word = 0;
    chip->stAVLErrorStat.stSwCntBitErrors.low_word = 0;
    chip->stAVLErrorStat.stSwCntNumBits.high_word = 0;
    chip->stAVLErrorStat.stSwCntNumBits.low_word = 0;
    chip->stAVLErrorStat.stBitErrors.high_word = 0;
    chip->stAVLErrorStat.stBitErrors.low_word = 0;
    chip->stAVLErrorStat.stNumBits.high_word = 0;
    chip->stAVLErrorStat.stNumBits.low_word = 0;

    //ber software reset
    r |= avl_bms_read32(chip->chip_pub->i2c_addr, 
        stBaseAddrSet.hw_esm_base + esm_cntrl_offset, &uiTemp);
    uiTemp |= 0x00000002;
    r |= avl_bms_write32(chip->chip_pub->i2c_addr,
        stBaseAddrSet.hw_esm_base + esm_cntrl_offset, uiTemp);

    //alway inverted
    pstErrorStatConfig->eBERFBInversion = AVL_LFSR_FB_INVERTED;

    //set Test Pattern and Inversion
    r |= avl_bms_read32(chip->chip_pub->i2c_addr,
        stBaseAddrSet.hw_esm_base + esm_cntrl_offset, &uiTemp);
    uiTemp &= 0xFFFFFFCF;
    uiTemp |= ((((uint32_t)pstErrorStatConfig->eBERTestPattern) << 5) | (((uint32_t)pstErrorStatConfig->eBERFBInversion) << 4));
    r |= avl_bms_write32(chip->chip_pub->i2c_addr,
        stBaseAddrSet.hw_esm_base + esm_cntrl_offset, uiTemp);
    uiTemp &= 0xFFFFFE3F;
    uiTemp |= (pstErrorStatConfig->uiLFSRStartPos<<6);
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, 
        stBaseAddrSet.hw_esm_base + esm_cntrl_offset, uiTemp);

    while(!uiLFSRSynced)
    {
        uiTemp |= 0x00000006;
        r |= avl_bms_write32(chip->chip_pub->i2c_addr, 
            stBaseAddrSet.hw_esm_base + esm_cntrl_offset, uiTemp);
        uiTemp &= 0xFFFFFFFD;
        r |= avl_bms_write32(chip->chip_pub->i2c_addr, 
            stBaseAddrSet.hw_esm_base + esm_cntrl_offset, uiTemp);

        uiCnt = 0;
        uiByteCnt = 0;
        while((uiByteCnt < 1000) && (uiCnt < 200))
        {
            r |= avl_bms_read32(chip->chip_pub->i2c_addr, 
                stBaseAddrSet.hw_esm_base + byte_num_offset, &uiByteCnt);
            uiCnt++;
        }

        uiTemp |= 0x00000006;
        r |= avl_bms_write32(chip->chip_pub->i2c_addr, 
            stBaseAddrSet.hw_esm_base + esm_cntrl_offset, uiTemp);
        uiTemp &= 0xFFFFFFF9;
        r |= avl_bms_write32(chip->chip_pub->i2c_addr, 
            stBaseAddrSet.hw_esm_base + esm_cntrl_offset, uiTemp);

        uiCnt = 0;
        uiByteCnt = 0;
        while((uiByteCnt < 10000) && (uiCnt < 200))
        {
            uiCnt++;
            r |= avl_bms_read32(chip->chip_pub->i2c_addr, 
                stBaseAddrSet.hw_esm_base + byte_num_offset, &uiByteCnt);
        }

        uiTemp &= 0xFFFFFFF9;
        uiTemp |= 0x00000002;
        r |= avl_bms_write32(chip->chip_pub->i2c_addr, 
            stBaseAddrSet.hw_esm_base + esm_cntrl_offset, uiTemp);

        r |= avl_bms_read32(chip->chip_pub->i2c_addr, 
            stBaseAddrSet.hw_esm_base + byte_num_offset, &uiByteCnt);
        r |= avl_bms_read32(chip->chip_pub->i2c_addr, 
            stBaseAddrSet.hw_esm_base + bit_error_offset, &uiBitErrors);
        if(uiCnt == 200)
        {
            break; 
        }
        else if((uiByteCnt << 3) < (10 * uiBitErrors))
        {
            uiBER_FailCnt++;
            if(uiBER_FailCnt > 10)
            {
                break;
            }
        }
        else
        {
            uiLFSRSynced = 1;
        }
    }

    if(uiLFSRSynced == 1)
    {
        uiTemp &= 0xFFFFFFF9;
        r |= avl_bms_write32(chip->chip_pub->i2c_addr,
            stBaseAddrSet.hw_esm_base + esm_cntrl_offset, uiTemp);
    }

    pstErrorStatConfig->uiLFSRSynced = uiLFSRSynced;

    return(r);
}

avl_error_code_t SetPLL_Demod(avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    switch(chip->family_id)
    {
    case AVL68XX:
        SetPLL0_Demod(chip);
        break;
    default:
        break;
    }

    return(r);
}


AVL_PLL_Conf0 gstPLLConfigArray0[] = 
{
    // Note: All the unit of clock frequency in the following is Hz.
    //----------------------------------------------------------------------------------------------------------------------------------------------
    //| REF_Hz    | C_R | C_F | C_Q | M_R | M_F | M_Q | A_R | A_F | A_Q | CORE_Hz | MPEG_Hz |  ADC_Hz | D_S | D_Q | S_S | S_Q |  DDC_Hz | SDRAM_Hz |
    //----------------------------------------------------------------------------------------------------------------------------------------------
    {30000000,  3,  100,    8,   1,   36,   8,    3,  100,    8, 250000000, 270000000, 250000000, 2,   12,   2,   14, 166666667, 142857143},
  //{30000000,  3,  100,    8,   1,   36,   12,   3,  100,    8, 250000000, 180000000, 250000000, 2,   12,   2,   14, 166666667, 142857143},//please change "#if 1" to "#if 0" located at function "SetTSMode_Demod" if want to confige serial MPEG clk to 90MHz for DVBTx mode.
  //{30000000,  3,  100,    8,   1,   36,   10,   3,  100,    8, 250000000, 216000000, 250000000, 2,   12,   2,   14, 166666667, 142857143},
    {16000000,  1,   55,    7,   1,   68,   8,    1,   60,   32, 251428571, 272000000, 60000000,  2,   12,   2,   14, 160000000, 137142857},
    {24000000,  2,   84,    8,   2,   90,   8,    2,   80,   32, 252000000, 270000000, 60000000,  2,   12,   2,   14, 160000000, 137142857},
  //{24000000,  2,   84,    8,   2,   90,   12,    2,   80,   32, 252000000, 180000000, 60000000,  2,   12,   2,   14, 160000000, 137142857},//please change "#if 1" to "#if 0" located at function "SetTSMode_Demod" if want to confige serial MPEG clk to 90MHz for DVBTx mode.
  //{24000000,  2,   84,    8,   2,   90,   10,    2,   80,   32, 252000000, 216000000, 60000000,  2,   12,   2,   14, 160000000, 137142857},
    {27000000,  2,   65,    7,   2,   80,   8,    2,   74,    8, 250714286, 270000000, 249750000, 2,   12,   2,   14, 165000000, 142714286},
    {30000000,  3,  100,    8,   1,   36,   8,    3,  100,    8, 250000000, 270000000, 250000000, 2,   12,   2,   14, 166666667, 142857143}, //DVBS 
    {16000000,  1,   55,    7,   1,   68,   8,    1,   62,    8, 251428571, 272000000, 248000000, 2,   12,   2,   14, 165333333, 141714286}, 
    {24000000,  2,   84,    8,   2,   90,   8,    2,   83,    8, 252000000, 270000000, 249000000, 2,   12,   2,   14, 166000000, 142857143},
    {27000000,  2,   65,    7,   2,   80,   8,    2,   74,    8, 250714286, 270000000, 249750000, 2,   12,   2,   14, 166500000, 142714286}

};


avl_error_code_t SetPLL0_Demod(avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint32_t DivRefHz = 0;
    uint32_t PLLRange = 1;

    AVL_PLL_Conf0 *pPLL_Conf = &gstPLLConfigArray0[chip->chip_pub->xtal];

    if(chip->chip_pub->cur_demod_mode == AVL_DVBSX )
    {
        pPLL_Conf = &gstPLLConfigArray0[chip->chip_pub->xtal+4];
    }
    //sys_pll
    DivRefHz = pPLL_Conf->m_RefFrequency_Hz / (uint32_t)pPLL_Conf->m_PLL_CoreClock_DivR;

    if(DivRefHz < 16000000)
        PLLRange = 1;
    else if(DivRefHz < 25000000)
        PLLRange = 2;
    else
        PLLRange = 3;
    r = avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_sys_pll_divr, pPLL_Conf->m_PLL_CoreClock_DivR-1);//DIVR
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_sys_pll_divf, pPLL_Conf->m_PLL_CoreClock_DivF-1);//DIVF
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_sys_pll_divq, pPLL_Conf->m_PLL_CoreClock_DivQ-1);//DIVQ1
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_sys_pll_range, PLLRange);//range
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_sys_pll_divq2, pPLL_Conf->m_PLL_DDCClock_DivQ-1);//DIVQ2
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_sys_pll_divq3, pPLL_Conf->m_PLL_SDRAMClock_DivQ-1);//DIVQ3
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_sys_pll_enable2, (pPLL_Conf->m_PLL_DDCClock_sel == hw_E2_PLL_SEL_CORE));
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_sys_pll_enable3, (pPLL_Conf->m_PLL_SDRAMClock_sel == hw_E2_PLL_SEL_CORE));

    //mpeg_pll
    DivRefHz = pPLL_Conf->m_RefFrequency_Hz / (uint32_t)pPLL_Conf->m_PLL_MPEGClock_DivR;
    if(DivRefHz < 16000000)
        PLLRange = 1;
    else if(DivRefHz < 25000000)
        PLLRange = 2;
    else
        PLLRange = 3;
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_mpeg_pll_divr, pPLL_Conf->m_PLL_MPEGClock_DivR-1);
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_mpeg_pll_divf, pPLL_Conf->m_PLL_MPEGClock_DivF-1);
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_mpeg_pll_divq, pPLL_Conf->m_PLL_MPEGClock_DivQ-1);
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_mpeg_pll_range, PLLRange);
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_mpeg_pll_divq2, pPLL_Conf->m_PLL_DDCClock_DivQ-1);
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_mpeg_pll_divq3, pPLL_Conf->m_PLL_SDRAMClock_DivQ-1);
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_mpeg_pll_enable2, (pPLL_Conf->m_PLL_DDCClock_sel == hw_E2_PLL_SEL_MPEG));
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_mpeg_pll_enable3, (pPLL_Conf->m_PLL_SDRAMClock_sel == hw_E2_PLL_SEL_MPEG));

    //adc_pll
    DivRefHz = pPLL_Conf->m_RefFrequency_Hz / (uint32_t)pPLL_Conf->m_PLL_ADCClock_DivR;
    if(DivRefHz < 16000000)
        PLLRange = 1;
    else if(DivRefHz < 25000000)
        PLLRange = 2;
    else
        PLLRange = 3;
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_adc_pll_divr, pPLL_Conf->m_PLL_ADCClock_DivR-1);
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_adc_pll_divf, pPLL_Conf->m_PLL_ADCClock_DivF-1);
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_adc_pll_divq, pPLL_Conf->m_PLL_ADCClock_DivQ-1);
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_adc_pll_range, PLLRange);
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_adc_pll_divq2, pPLL_Conf->m_PLL_DDCClock_DivQ-1);
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_adc_pll_divq3, pPLL_Conf->m_PLL_SDRAMClock_DivQ-1);
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_adc_pll_enable2, (pPLL_Conf->m_PLL_DDCClock_sel == hw_E2_PLL_SEL_ADC));
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_adc_pll_enable3, (pPLL_Conf->m_PLL_SDRAMClock_sel == hw_E2_PLL_SEL_ADC));

    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_reset_register, 0);
    avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_reset_register, 1);//no I2C ACK
    avl_bsp_delay(20);



    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_dll_out_phase, 96);
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_dll_rd_phase, 0);
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_deglitch_mode, 1);
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_dll_init, 1);
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_dll_init, 0);



    chip->uiCoreFrequencyHz = pPLL_Conf->m_CoreFrequency_Hz;
    chip->uiFECFrequencyHz = chip->uiCoreFrequencyHz;
    chip->uiTSFrequencyHz = pPLL_Conf->m_MPEGFrequency_Hz;
    //chip->uiADCFrequencyHz = pPLL_Conf->m_ADCFrequency_Hz;


    if(chip->chip_pub->cur_demod_mode == AVL_DVBSX )
    {
        chip->uiADCFrequencyHz = (pPLL_Conf->m_ADCFrequency_Hz)/2;
    }
    else
    {
        switch(chip->chip_pub->xtal)
        {     
        case Xtal_16M :
        case Xtal_24M :
            {
                chip->uiADCFrequencyHz = (pPLL_Conf->m_ADCFrequency_Hz)/2;
                break;
            }

        case Xtal_30M :
        case Xtal_27M :
            {
                chip->uiADCFrequencyHz = pPLL_Conf->m_RefFrequency_Hz;
                break;
            }       
        }
    }

    chip->uiRefFrequencyHz = pPLL_Conf->m_RefFrequency_Hz;
    chip->uiDDCFrequencyHz = pPLL_Conf->m_DDCFrequency_Hz;
    chip->uiSDRAMFrequencyHz = pPLL_Conf->m_SDRAMFrequency_Hz;

    return r;
}

avl_error_code_t IBase_CheckChipReady_Demod(avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint32_t uiCoreReadyWord = 0;
    uint32_t uiCoreRunning = 0;

    r = avl_bms_read32(chip->chip_pub->i2c_addr, 
        stBaseAddrSet.hw_mcu_reset_base, &uiCoreRunning);

    r |= avl_bms_read32(chip->chip_pub->i2c_addr,rs_core_ready_word_iaddr_offset, &uiCoreReadyWord);
    if( (AVL_EC_OK == r) )
    {
        if((1 == uiCoreRunning) || (uiCoreReadyWord != 0x5aa57ff7))
        {
            r = AVL_EC_GENERAL_FAIL;
        }
    }

    return(r);
}

avl_error_code_t IRx_Initialize_Demod(avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    // Load the default configuration
    r = IBase_SendRxOPWait_Demod(AVL_FW_CMD_HALT, chip);

    r = IBase_SendRxOPWait_Demod(AVL_FW_CMD_LD_DEFAULT, chip);

    r |= IBase_SendRxOPWait_Demod(AVL_FW_CMD_INIT_SDRAM, chip);

    r |= IBase_SendRxOPWait_Demod(AVL_FW_CMD_INIT_ADC, chip);

    r |= chip->stStdSpecFunc->fpRxInitializeFunc(chip);

    return (r);
}

avl_error_code_t IBase_SendRxOPWait_Demod(uint8_t ucOpCmd, avl68x2_chip *chip )
{
    avl_error_code_t r = AVL_EC_OK;
    uint8_t pucBuff[4] = {0};
    uint16_t uiTemp = 0;
    const uint16_t uiTimeDelay = 10;
    const uint16_t uiMaxRetries = 50;//the time out window is 10*50 = 500ms
    uint32_t i = 0;

    r = avl_bsp_wait_semaphore(&(chip->rx_sem));

    while (AVL_EC_OK != IBase_GetRxOPStatus_Demod(chip))
    {
        if (uiMaxRetries < i++)
        {
            r |= AVL_EC_RUNNING;
            break;
        }
        avl_bsp_delay(uiTimeDelay);
    }

    if( AVL_EC_OK == r )
    {
        pucBuff[0] = 0;
        pucBuff[1] = ucOpCmd;
        uiTemp = avl_bytes_to_short(pucBuff);
        r |= avl_bms_write16(chip->chip_pub->i2c_addr,  
            stBaseAddrSet.fw_config_reg_base + rc_fw_command_saddr_offset, uiTemp);
    }

    i = 0;

    do{
        avl_bsp_delay(uiTimeDelay);
        {
            if (uiMaxRetries < i++)
            {
                r |= AVL_EC_RUNNING;
                break;
            }
        }
    }while(AVL_EC_OK != IBase_GetRxOPStatus_Demod(chip));


    r |= avl_bsp_release_semaphore(&(chip->rx_sem));

    return(r);
}



avl_error_code_t IBase_GetRxOPStatus_Demod(avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint16_t uiCmd = 0;

    r = avl_bms_read16(chip->chip_pub->i2c_addr,
        stBaseAddrSet.fw_config_reg_base + rc_fw_command_saddr_offset, &uiCmd);

    //System::Console::WriteLine("status: {0}",uiCmd);
    if( AVL_EC_OK == r )
    {
        if( 0 != uiCmd )
        {
            r = AVL_EC_RUNNING;
        }
    }

    return(r);
}

avl_error_code_t SetTSMode_Demod(avl68x2_chip *chip)
{
	avl_error_code_t r = AVL_EC_OK;
	r |= SetTSSerialPin_Demod(chip->chip_pub->ts_config.eSerialPin, chip);
	r |= SetTSSerialSyncPulse_Demod(chip->chip_pub->ts_config.eSerialSyncPulse, chip);
	r |= SetTSErrorBit_Demod(chip->chip_pub->ts_config.eErrorBit, chip);
	r |= SetTSErrorPola_Demod(chip->chip_pub->ts_config.eErrorPolarity, chip);
	r |= SetTSValidPola_Demod(chip->chip_pub->ts_config.eValidPolarity, chip);
	r |= SetTSParallelPhase_Demod(chip->chip_pub->ts_config.eParallelPhase, chip);

	r |= avl_bms_write8(chip->chip_pub->i2c_addr,
			    stBaseAddrSet.fw_config_reg_base +
				rc_ts_serial_caddr_offset,
			    chip->chip_pub->ts_config.eMode);
	r |= avl_bms_write8(chip->chip_pub->i2c_addr,
			    stBaseAddrSet.fw_config_reg_base +
				rc_ts_clock_edge_caddr_offset,
			    chip->chip_pub->ts_config.eClockEdge);

	r |= avl_bms_write8(chip->chip_pub->i2c_addr,
			    stBaseAddrSet.fw_config_reg_base +
				rc_ts_ts0_tsp1_caddr_offset,
			    0);

	if (chip->chip_pub->ts_config.eMode == AVL_TS_SERIAL)
	{
		r |= SetTSSerialOrder_Demod(chip->chip_pub->ts_config.eSerialOrder, chip);
	}
	else
	{
		r |= SetTSParallelOrder_Demod(chip->chip_pub->ts_config.eParallelOrder, chip);
	}

	r |= avl_bms_write8(chip->chip_pub->i2c_addr,
			    stBaseAddrSet.fw_config_reg_base +
				rc_enable_ts_continuous_caddr_offset,
			    chip->chip_pub->ts_config.eClockMode ==
				AVL_TS_CONTINUOUS_ENABLE);
	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     stBaseAddrSet.fw_config_reg_base +
				 rc_ts_cntns_clk_frac_d_iaddr_offset,
			     chip->uiTSFrequencyHz);

	if (chip->chip_pub->ts_config.eMode == AVL_TS_SERIAL)
	{
		if (chip->chip_pub->cur_demod_mode == AVL_DVBSX)
		{
			r |= avl_bms_write32(chip->chip_pub->i2c_addr,
						stBaseAddrSet.fw_config_reg_base +
							rc_ts_cntns_clk_frac_n_iaddr_offset,
						chip->uiTSFrequencyHz);
		}
		else
		{
			r |= avl_bms_write32(chip->chip_pub->i2c_addr,
						stBaseAddrSet.fw_config_reg_base +
							rc_ts_cntns_clk_frac_n_iaddr_offset,
						chip->uiTSFrequencyHz / 2);
		}
	}
	else
	{
		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
					stBaseAddrSet.fw_config_reg_base +
						rc_ts_cntns_clk_frac_n_iaddr_offset,
					chip->uiTSFrequencyHz / 8);
	}

	return r;
}

avl_error_code_t SetInternalFunc_Demod(AVL_DemodMode eDemodMode, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    switch (eDemodMode)
    {
    case AVL_DVBC:
        chip->stStdSpecFunc->fpRxInitializeFunc = DVBC_Initialize_Demod;
        chip->stStdSpecFunc->fpGetLockStatus = DVBC_GetLockStatus_Demod;
        chip->stStdSpecFunc->fpGetSNR = DVBC_GetSNR_Demod;
        chip->stStdSpecFunc->fpGetSQI = DVBC_GetSignalQuality_Demod;
        chip->stStdSpecFunc->fpGetPrePostBER = DVBC_GetPrePostBER_Demod;
        break;

    case AVL_DVBSX:
        chip->stStdSpecFunc->fpRxInitializeFunc = DVBSx_Initialize_Demod;
        chip->stStdSpecFunc->fpGetLockStatus = DVBSx_GetLockStatus_Demod;
        chip->stStdSpecFunc->fpGetSNR = DVBSx_GetSNR_Demod;
        chip->stStdSpecFunc->fpGetSQI = DVBSx_GetSignalQuality_Demod;
        chip->stStdSpecFunc->fpGetPrePostBER = DVBSx_GetPrePostBER_Demod;
        break;

    case AVL_DVBTX:
        chip->stStdSpecFunc->fpRxInitializeFunc = DVBTx_Initialize_Demod;
        chip->stStdSpecFunc->fpGetLockStatus = DVBTx_GetLockStatus_Demod;
        chip->stStdSpecFunc->fpGetSNR = DVBTx_GetSNR_Demod;
        chip->stStdSpecFunc->fpGetSQI = DVBTx_GetSignalQuality_Demod;
        chip->stStdSpecFunc->fpGetPrePostBER = DVBTx_GetPrePostBER_Demod;
        break;

    case AVL_ISDBT:
        chip->stStdSpecFunc->fpRxInitializeFunc = ISDBT_Initialize_Demod;
        chip->stStdSpecFunc->fpGetLockStatus = ISDBT_GetLockStatus_Demod;
        chip->stStdSpecFunc->fpGetSNR = ISDBT_GetSNR_Demod;
        chip->stStdSpecFunc->fpGetSQI = ISDBT_GetSignalQuality_Demod;
        chip->stStdSpecFunc->fpGetPrePostBER = ISDBT_GetPrePostBER_Demod;
        break;

    default:
        break;
    }

    return r;
}

avl_error_code_t EnableTCAGC_Demod(avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    r = avl_bms_write32(chip->chip_pub->i2c_addr,
        stBaseAddrSet.hw_gpio_debug_base + agc1_sel_offset, AVL_AGC_ON_VAL);

    return r;
}

avl_error_code_t DisableTCAGC_Demod(avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    r = avl_bms_write32(chip->chip_pub->i2c_addr,
        stBaseAddrSet.hw_gpio_debug_base + agc1_sel_offset, AVL_AGC_OFF_VAL);

    return r;
}

avl_error_code_t EnableSAGC_Demod(avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    r = avl_bms_write32(chip->chip_pub->i2c_addr,
        stBaseAddrSet.hw_gpio_debug_base + agc2_sel_offset, AVL_AGC_ON_VAL);

    return r;
}

avl_error_code_t DisableSAGC_Demod(avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    r = avl_bms_write32(chip->chip_pub->i2c_addr,
        stBaseAddrSet.hw_gpio_debug_base + agc2_sel_offset, AVL_AGC_OFF_VAL);

    return r;
}

avl_error_code_t ConfigAGCOutput_Demod(avl68x2_chip *chip)
{
	avl_error_code_t r = AVL_EC_OK;
	AVL_AGC_Selection agc_sel = AVL_BOTH_AGC;

	switch (chip->chip_pub->cur_demod_mode)
	{
	case AVL_DVBC:
	case AVL_DVBTX:
	case AVL_ISDBT:
		agc_sel = chip->chip_pub->tc_agc_selection;
		break;
	case AVL_DVBSX:
	default:
		agc_sel = chip->chip_pub->s_agc_selection;
		break;
	}
	printk("AGC SEL %d\n",(int)agc_sel);
	if ((agc_sel == AVL_S_AGC_ONLY) || (agc_sel == AVL_BOTH_AGC))
	{
		r |= EnableSAGC_Demod(chip);
	}
	else
	{
		r |= DisableSAGC_Demod(chip);
	}
	if ((agc_sel == AVL_TC_AGC_ONLY) || (agc_sel == AVL_BOTH_AGC))
	{
		r |= EnableTCAGC_Demod(chip);
	}
	else
	{
		r |= DisableTCAGC_Demod(chip);
	}
	return r;
}



avl_error_code_t SetTSSerialPin_Demod(AVL_TSSerialPin eTSSerialPin, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    chip->chip_pub->ts_config.eSerialPin = eTSSerialPin;
    printk("TS serial pin %d\n",eTSSerialPin);

    r = avl_bms_write8(chip->chip_pub->i2c_addr,
                       stBaseAddrSet.fw_config_reg_base + rc_ts_serial_outpin_caddr_offset,
                       (uint8_t)eTSSerialPin);

    return r;
}

avl_error_code_t SetTSSerialOrder_Demod(AVL_TSSerialOrder eTSSerialOrder, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    chip->chip_pub->ts_config.eSerialOrder = eTSSerialOrder;
    printk("TS serial order %d\n",eTSSerialOrder);

    r = avl_bms_write8(chip->chip_pub->i2c_addr,
                       stBaseAddrSet.fw_config_reg_base + rc_ts_serial_msb_caddr_offset,
                       (uint8_t)eTSSerialOrder);

    return r;
}

avl_error_code_t SetTSSerialSyncPulse_Demod(AVL_TSSerialSyncPulse eTSSerialSyncPulse, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    chip->chip_pub->ts_config.eSerialSyncPulse = eTSSerialSyncPulse;
    printk("TS serial sync pulse %d\n",eTSSerialSyncPulse);

    r = avl_bms_write8(chip->chip_pub->i2c_addr,
                       stBaseAddrSet.fw_config_reg_base + rc_ts_sync_pulse_caddr_offset,
                       (uint8_t)eTSSerialSyncPulse);

    return r;
}

avl_error_code_t SetTSErrorBit_Demod(AVL_TSErrorBit eTSErrorBit, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    chip->chip_pub->ts_config.eErrorBit = eTSErrorBit;
    printk("TS set err bit %d\n",eTSErrorBit);

    r = avl_bms_write8(chip->chip_pub->i2c_addr,
		       stBaseAddrSet.fw_config_reg_base + rc_ts_error_bit_en_caddr_offset,
		       chip->chip_pub->ts_config.eErrorBit);

    return r;
}

avl_error_code_t SetTSErrorPola_Demod(AVL_TSErrorPolarity eTSErrorPola, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    chip->chip_pub->ts_config.eErrorPolarity = eTSErrorPola;
    printk("TS err polarity %d\n",eTSErrorPola);

    r = avl_bms_write8(chip->chip_pub->i2c_addr,
        stBaseAddrSet.fw_config_reg_base + rc_ts_error_polarity_caddr_offset,
        (uint8_t)eTSErrorPola);

    return r;
}

avl_error_code_t SetTSValidPola_Demod(AVL_TSValidPolarity eTSValidPola, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    chip->chip_pub->ts_config.eValidPolarity = eTSValidPola;
    printk("TS valid polarity %d\n",eTSValidPola);

    r = avl_bms_write8(chip->chip_pub->i2c_addr,
        stBaseAddrSet.fw_config_reg_base + rc_ts_valid_polarity_caddr_offset,
        (uint8_t)eTSValidPola);

    return r;
}

avl_error_code_t SetTSParallelOrder_Demod(AVL_TSParallelOrder eTSParallelOrder, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    chip->chip_pub->ts_config.eParallelOrder = eTSParallelOrder;
    printk("TS parallel order %d\n",eTSParallelOrder);

    r = avl_bms_write8(chip->chip_pub->i2c_addr,
        stBaseAddrSet.fw_config_reg_base + rc_ts_packet_order_caddr_offset,
        (uint8_t)eTSParallelOrder);

    return r;
}

avl_error_code_t SetTSParallelPhase_Demod(AVL_TSParallelPhase eTSParallelPhase, avl68x2_chip *chip)
{
	avl_error_code_t r = AVL_EC_OK;

	chip->chip_pub->ts_config.eParallelPhase = eTSParallelPhase;
	printk("TS clk phase %d\n", eTSParallelPhase);

	r = avl_bms_write8(chip->chip_pub->i2c_addr,
			   stBaseAddrSet.fw_config_reg_base + ts_clock_phase_caddr_offset,
			   (uint8_t)eTSParallelPhase);

	return r;
}

avl_error_code_t IBase_SetSleepClock_Demod(avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint32_t uiTemp = 0;
    uint32_t uiMaxRetries = 10;
    uint32_t delay_unit_ms = 20;//the time out window is 10*20=200ms

    r = avl_bms_write32(chip->chip_pub->i2c_addr, 
        stBaseAddrSet.hw_mcu_reset_base, 1);
    r = avl_bms_write32(chip->chip_pub->i2c_addr, 
        stBaseAddrSet.hw_mcu_system_reset_base, 1);

    switch(chip->family_id)
    {
    case AVL68XX:
        SetSleepPLL0_Demod(chip);
        break;
    default:
        break;
    }

    r |= avl_bms_write32(chip->chip_pub->i2c_addr, 
        stBaseAddrSet.fw_status_reg_base + rs_core_ready_word_iaddr_offset, 0x00000000);

    r = avl_bms_write32(chip->chip_pub->i2c_addr, 
        stBaseAddrSet.hw_mcu_system_reset_base, 0);
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, 
        stBaseAddrSet.hw_mcu_reset_base, 0);

    while (AVL_EC_OK != IBase_CheckChipReady_Demod(chip))
    {
        if (uiMaxRetries <= uiTemp++)
        {
            r |= AVL_EC_GENERAL_FAIL;
            break;
        }
        avl_bsp_delay(delay_unit_ms);
    }

    return r;

}

AVL_PLL_Conf0 gstSleepPLLConfigArray0[] = 
{
    // Note: All the unit of clock frequency in the following is Hz.
    //----------------------------------------------------------------------------------------------------------------------------------------------
    //| REF_Hz    | C_R | C_F | C_Q | M_R | M_F | M_Q | A_R | A_F | A_Q | CORE_Hz | MPEG_Hz |  ADC_Hz | D_S | D_Q | S_S | S_Q |  DDC_Hz | SDRAM_Hz |
    //----------------------------------------------------------------------------------------------------------------------------------------------
    {30000000,  1,   20,   20,   1,   20,   20,  1,   20,   30, 60000000, 60000000, 40000000,  2,   20,   2,   20,  60000000, 60000000} 
};

avl_error_code_t SetSleepPLL0_Demod(avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    AVL_PLL_Conf0 *pPLL_Conf = &gstPLLConfigArray0[chip->chip_pub->xtal];

    //sys_pll
    uint32_t DivRefHz = pPLL_Conf->m_RefFrequency_Hz / (uint32_t)pPLL_Conf->m_PLL_CoreClock_DivR;
    uint32_t PLLRange = 1;
    if(DivRefHz < 16000000)
        PLLRange = 1;
    else if(DivRefHz < 25000000)
        PLLRange = 2;
    else
        PLLRange = 3;
    r = avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_sys_pll_divr, pPLL_Conf->m_PLL_CoreClock_DivR-1);//DIVR
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_sys_pll_divf, pPLL_Conf->m_PLL_CoreClock_DivF-1);//DIVF
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_sys_pll_divq, pPLL_Conf->m_PLL_CoreClock_DivQ-1);//DIVQ1
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_sys_pll_range, PLLRange);//range
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_sys_pll_divq2, pPLL_Conf->m_PLL_DDCClock_DivQ-1);//DIVQ2
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_sys_pll_divq3, pPLL_Conf->m_PLL_SDRAMClock_DivQ-1);//DIVQ3
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_sys_pll_enable2, (pPLL_Conf->m_PLL_DDCClock_sel == hw_E2_PLL_SEL_CORE));
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_sys_pll_enable3, (pPLL_Conf->m_PLL_SDRAMClock_sel == hw_E2_PLL_SEL_CORE));

    //mpeg_pll
    DivRefHz = pPLL_Conf->m_RefFrequency_Hz / (uint32_t)pPLL_Conf->m_PLL_MPEGClock_DivR;
    if(DivRefHz < 16000000)
        PLLRange = 1;
    else if(DivRefHz < 25000000)
        PLLRange = 2;
    else
        PLLRange = 3;
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_mpeg_pll_divr, pPLL_Conf->m_PLL_MPEGClock_DivR-1);
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_mpeg_pll_divf, pPLL_Conf->m_PLL_MPEGClock_DivF-1);
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_mpeg_pll_divq, pPLL_Conf->m_PLL_MPEGClock_DivQ-1);
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_mpeg_pll_range, PLLRange);
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_mpeg_pll_divq2, pPLL_Conf->m_PLL_DDCClock_DivQ-1);
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_mpeg_pll_divq3, pPLL_Conf->m_PLL_SDRAMClock_DivQ-1);
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_mpeg_pll_enable2, (pPLL_Conf->m_PLL_DDCClock_sel == hw_E2_PLL_SEL_MPEG));
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_mpeg_pll_enable3, (pPLL_Conf->m_PLL_SDRAMClock_sel == hw_E2_PLL_SEL_MPEG));

    //adc_pll
    DivRefHz = pPLL_Conf->m_RefFrequency_Hz / (uint32_t)pPLL_Conf->m_PLL_ADCClock_DivR;
    if(DivRefHz < 16000000)
        PLLRange = 1;
    else if(DivRefHz < 25000000)
        PLLRange = 2;
    else
        PLLRange = 3;
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_adc_pll_divr, pPLL_Conf->m_PLL_ADCClock_DivR-1);
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_adc_pll_divf, pPLL_Conf->m_PLL_ADCClock_DivF-1);
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_adc_pll_divq, pPLL_Conf->m_PLL_ADCClock_DivQ-1);
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_adc_pll_range, PLLRange);
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_adc_pll_divq2, pPLL_Conf->m_PLL_DDCClock_DivQ-1);
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_adc_pll_divq3, pPLL_Conf->m_PLL_SDRAMClock_DivQ-1);
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_adc_pll_enable2, (pPLL_Conf->m_PLL_DDCClock_sel == hw_E2_PLL_SEL_ADC));
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_E2_AVLEM61_adc_pll_enable3, (pPLL_Conf->m_PLL_SDRAMClock_sel == hw_E2_PLL_SEL_ADC));

    r |= avl_bms_write32(chip->chip_pub->i2c_addr, 0x100000, 0);
    avl_bms_write32(chip->chip_pub->i2c_addr, 0x100000, 1);//no I2C ACK
    avl_bsp_delay(1);

    chip->uiCoreFrequencyHz = pPLL_Conf->m_CoreFrequency_Hz;
    chip->uiFECFrequencyHz = chip->uiCoreFrequencyHz;
    chip->uiTSFrequencyHz = pPLL_Conf->m_MPEGFrequency_Hz;
    chip->uiADCFrequencyHz = pPLL_Conf->m_ADCFrequency_Hz;
    chip->uiRefFrequencyHz = pPLL_Conf->m_RefFrequency_Hz;
    chip->uiDDCFrequencyHz = pPLL_Conf->m_DDCFrequency_Hz;
    chip->uiSDRAMFrequencyHz = pPLL_Conf->m_SDRAMFrequency_Hz;

    return(r);
}

avl_error_code_t GetMode_Demod(AVL_DemodMode* peCurrentMode, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint32_t uiTemp = 0;

    r = avl_bms_read32(chip->chip_pub->i2c_addr, 
        stBaseAddrSet.fw_config_reg_base + rs_current_active_mode_iaddr_offset, &uiTemp);
    *peCurrentMode = (AVL_DemodMode)(uiTemp);

    return r;
}

avl_error_code_t GetBER_Demod(uint32_t *puiBERxe9, AVL_BER_Type eBERType, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint32_t uiHwCntBitErrors = 0;
    uint32_t uiHwCntNumBits = 0;
    uint32_t uiTemp = 0;
    struct avl_uint64 uiTemp64;
    uint8_t ucLocked;

    r = avl68x2_demod_get_lock_status(&ucLocked,chip);

    //record the lock status before return the BER
    if(1 == ucLocked)
    {
        chip->stAVLErrorStat.usLostLock = 0;
    }
    else
    {
        chip->stAVLErrorStat.usLostLock = 1;
        return *puiBERxe9 = AVL_CONSTANT_10_TO_THE_9TH;
    }

    switch (eBERType)
    {
    case AVL_PRE_VITERBI_BER:
    case AVL_POST_VITERBI_BER:
    case AVL_PRE_LDPC_BER:
    case AVL_POST_LDPC_BER:
        {
            r = chip->stStdSpecFunc->fpGetPrePostBER(puiBERxe9,eBERType, chip); 
            return (r);
        }
    case AVL_FINAL_BER:
        {
            //just break for following statistics
            break;
        }
    default:
        {
            // the inputted type is not supported, final ber will be presented
            r = AVL_EC_WARNING;
            break;
        }
    }


    r = avl_bms_read32(chip->chip_pub->i2c_addr,
        stBaseAddrSet.hw_esm_base + bit_error_offset, &uiHwCntBitErrors);
    r |= avl_bms_read32(chip->chip_pub->i2c_addr,
        stBaseAddrSet.hw_esm_base + byte_num_offset, &uiHwCntNumBits);
    uiHwCntNumBits <<= 3;

    if(uiHwCntNumBits > (uint32_t)(1 << 31))
    {
        r |= avl_bms_read32(chip->chip_pub->i2c_addr,
            stBaseAddrSet.hw_esm_base + esm_cntrl_offset, &uiTemp);
        uiTemp |= 0x00000002;
        r |= avl_bms_write32(chip->chip_pub->i2c_addr,
            stBaseAddrSet.hw_esm_base + esm_cntrl_offset, uiTemp);
        r |= avl_bms_read32(chip->chip_pub->i2c_addr,
            stBaseAddrSet.hw_esm_base + bit_error_offset, &uiHwCntBitErrors);
        r |= avl_bms_read32(chip->chip_pub->i2c_addr,
            stBaseAddrSet.hw_esm_base + byte_num_offset, &uiHwCntNumBits);
        uiTemp &= 0xFFFFFFFD;
        r |= avl_bms_write32(chip->chip_pub->i2c_addr,
            stBaseAddrSet.hw_esm_base + esm_cntrl_offset, uiTemp);
        uiHwCntNumBits <<= 3;
        avl_add_32to64(&chip->stAVLErrorStat.stSwCntNumBits, uiHwCntNumBits);
        avl_add_32to64(&chip->stAVLErrorStat.stSwCntBitErrors, uiHwCntBitErrors);
        uiHwCntNumBits = 0;
        uiHwCntBitErrors = 0;
    }

    chip->stAVLErrorStat.stNumBits.high_word= chip->stAVLErrorStat.stSwCntNumBits.high_word;
    chip->stAVLErrorStat.stNumBits.low_word = chip->stAVLErrorStat.stSwCntNumBits.low_word;
    avl_add_32to64(&chip->stAVLErrorStat.stNumBits, uiHwCntNumBits);
    chip->stAVLErrorStat.stBitErrors.high_word = chip->stAVLErrorStat.stSwCntBitErrors.high_word;
    chip->stAVLErrorStat.stBitErrors.low_word = chip->stAVLErrorStat.stSwCntBitErrors.low_word;
    avl_add_32to64(&chip->stAVLErrorStat.stBitErrors, uiHwCntBitErrors);

    avl_mult_32to64(&uiTemp64, chip->stAVLErrorStat.stBitErrors.low_word, AVL_CONSTANT_10_TO_THE_9TH);
    chip->stAVLErrorStat.uiBER = avl_divide_64(chip->stAVLErrorStat.stNumBits, uiTemp64);

    //keep the BER user wanted
    *puiBERxe9 = chip->stAVLErrorStat.uiBER;

    return r;
}







avl_error_code_t TestSDRAM_Demod(uint32_t * puiTestResult, uint32_t * puiTestPattern, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    r = IBase_SendRxOPWait_Demod(AVL_FW_CMD_HALT, chip);

    r = IBase_SendRxOPWait_Demod(AVL_FW_CMD_SDRAM_TEST, chip);
    if(AVL_EC_OK == r )
    {

        r |= avl_bms_read32(chip->chip_pub->i2c_addr, 
            stBaseAddrSet.fw_status_reg_base + rc_sdram_test_return_iaddr_offset, puiTestPattern);
        r |= avl_bms_read32(chip->chip_pub->i2c_addr,
            stBaseAddrSet.fw_status_reg_base + rc_sdram_test_result_iaddr_offset, puiTestResult);
    }

    return r;
}

avl_error_code_t GetValidModeList_Demod(uint8_t *pucValidModeList, avl68x2_chip *chip)
{
	avl_error_code_t r = AVL_EC_OK;
	uint32_t uiMemberID = 0x0;

	r = GetFamilyID_Demod(&(chip->family_id), chip);

	r |= avl_bms_read32(chip->chip_pub->i2c_addr,
			    stBaseAddrSet.hw_member_ID_base,
			    &uiMemberID);

	switch (chip->family_id)
	{
	case AVL68XX:
		GetValidModeList0_Demod(pucValidModeList, uiMemberID);
		break;
	default:
		r = AVL_EC_GENERAL_FAIL;
		break;
	}

	return r;
}

void GetValidModeList0_Demod(uint8_t * pucValidModeList, uint32_t uiMemberID)
{
    if(uiMemberID == 0xB)
    {
        *pucValidModeList++ = 1;//DVBC valid
        *(pucValidModeList++) = 1;//DVBSx valid
        *(pucValidModeList++) = 1;//DVBTx valid
        *(pucValidModeList++) = 1;//ISDBT valid
    }
    else if(uiMemberID == 0xF)
    {
        *pucValidModeList++ = 1;//DVBC valid
        *(pucValidModeList++) = 1;//DVBSx valid
        *(pucValidModeList++) = 1;//DVBTx valid
        *(pucValidModeList++) = 0;//ISDBT invalid
    }
    else if(uiMemberID == 0xE)
    {
        *pucValidModeList++ = 1;//DVBC valid
        *(pucValidModeList++) = 0;//DVBSx invalid
        *(pucValidModeList++) = 1;//DVBTx valid
        *(pucValidModeList++) = 0;//ISDBT invalid
    }
    else if(uiMemberID == 0xD)
    {
        *pucValidModeList++ = 0;//DVBC invalid
        *(pucValidModeList++) = 1;//DVBSx valid
        *(pucValidModeList++) = 0;//DVBTx invalid
        *(pucValidModeList++) = 1;//ISDBT valid
    }
}

avl_error_code_t GetFamilyID_Demod(uint32_t *puiFamilyID, avl68x2_chip *chip)
{
	avl_error_code_t r = AVL_EC_OK;
	const uint32_t ChipIDRegAddr = 0x40000;

	r = avl_bms_read32(chip->chip_pub->i2c_addr,
			   ChipIDRegAddr,
			   puiFamilyID);

	return r;
}

avl_error_code_t SetGPIOStatus_Demod(avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    if(chip->ucPin37Voltage == 0)
    {
        r = avl_bms_write32(chip->chip_pub->i2c_addr,
            stBaseAddrSet.hw_gpio_debug_base + lnb_cntrl_0_sel_offset, AVL_LOGIC_0);
    }
    if(chip->ucPin37Voltage == 1)
    {
        r = avl_bms_write32(chip->chip_pub->i2c_addr,
            stBaseAddrSet.hw_gpio_debug_base + lnb_cntrl_0_sel_offset, AVL_LOGIC_1);
    }
    if(chip->ucPin37Voltage == 2)
    {
        r = avl_bms_write32(chip->chip_pub->i2c_addr,
            stBaseAddrSet.hw_gpio_debug_base + lnb_cntrl_0_sel_offset, AVL_HIGH_Z);
    }

    if(chip->ucPin38Voltage == 0)
    {
        r = avl_bms_write32(chip->chip_pub->i2c_addr,
            stBaseAddrSet.hw_gpio_debug_base + lnb_cntrl_1_sel_offset, AVL_LOGIC_0);
    }
    if(chip->ucPin38Voltage == 1)
    {
        r = avl_bms_write32(chip->chip_pub->i2c_addr,
            stBaseAddrSet.hw_gpio_debug_base + lnb_cntrl_1_sel_offset, AVL_LOGIC_1);
    }
    if(chip->ucPin38Voltage == 2)
    {
        r = avl_bms_write32(chip->chip_pub->i2c_addr,
            stBaseAddrSet.hw_gpio_debug_base + lnb_cntrl_1_sel_offset, AVL_HIGH_Z);
    }
    return r;
}

avl_error_code_t Initilize_GPIOStatus_Demod(avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    r = avl_bms_write32(chip->chip_pub->i2c_addr,
        stBaseAddrSet.hw_gpio_debug_base + lnb_cntrl_0_sel_offset, AVL_LOGIC_0);
    chip->ucPin37Voltage = 0;


    r = avl_bms_write32(chip->chip_pub->i2c_addr,
        stBaseAddrSet.hw_gpio_debug_base + lnb_cntrl_1_sel_offset, AVL_LOGIC_0);
    chip->ucPin38Voltage = 0;

    // set CS_0 to AVL_LOGIC_0
    r = avl_bms_write32(chip->chip_pub->i2c_addr,
        stBaseAddrSet.hw_gpio_modu_002_base + gpio_module_002_gpio_config_offset, 0x2);

    return r;
}

uint8_t AVL_patch_read8(uint8_t * pPatchBuf, uint32_t *idx, uint8_t update_idx) 
{
    uint8_t tmp = 0;
    tmp = pPatchBuf[*idx];
    if(update_idx)
        *idx += 1;
    return tmp;
}
uint16_t AVL_patch_read16(uint8_t * pPatchBuf, uint32_t *idx, uint8_t update_idx) 
{
    uint16_t tmp = 0;
    tmp = (pPatchBuf[*idx+0]<<8) | (pPatchBuf[*idx+1]);
    if(update_idx)
        *idx += 2;
    return tmp;
}
uint32_t AVL_patch_read32(uint8_t * pPatchBuf, uint32_t *idx, uint8_t update_idx) 
{
    uint32_t tmp = 0;
#define SWIZZLE
#ifdef SWIZZLE
    tmp = (pPatchBuf[*idx+0]<<24) | (pPatchBuf[*idx+1]<<16) | (pPatchBuf[*idx+2]<<8) | pPatchBuf[*idx+3];
#else
    tmp = (pPatchBuf[*idx+3]<<24) | (pPatchBuf[*idx+2]<<16) | (pPatchBuf[*idx+1]<<8) | pPatchBuf[*idx+0];
#endif
    if(update_idx)
        *idx += 4;
    return tmp;
}



avl_error_code_t AVL_ParseFwPatch_v0(avl68x2_chip *chip, uint8_t download_only) {

    avl_error_code_t r = AVL_EC_OK;
    uint8_t * patch_data = 0;
    uint32_t patch_idx = 0; 
    uint32_t args_addr = 0;
    uint32_t data_section_offset = 0;
    uint32_t reserved_len = 0;
    uint32_t script_len = 0;
    uint8_t unary_op = 0;
    uint8_t binary_op = 0;
    uint8_t addr_mode_op = 0;
    uint32_t script_start_idx = 0;
    uint32_t num_cmd_words = 0;
    uint32_t next_cmd_idx = 0;
    uint32_t num_cond_words = 0;
    uint32_t condition = 0;
    uint32_t operation =0;
    uint32_t value = 0;
    uint32_t tmp_top_valid = 0;
    uint32_t core_rdy_word = 0;
    uint32_t cmd = 0;
    uint32_t num_rvs = 0;
    uint32_t rv0_idx = 0;
    uint32_t exp_crc_val = 0;
    uint32_t start_addr = 0;
    uint32_t crc_result = 0;
    uint32_t length = 0; 
    uint32_t dest_addr = 0;
    uint32_t src_data_offset = 0;
    uint32_t data = 0;
    uint16_t data1 = 0;
    uint8_t data2 = 0;
    uint32_t src_addr = 0;
    uint32_t descr_addr = 0;
    uint32_t num_descr = 0;
    uint32_t type = 0;
    uint32_t ref_addr = 0;
    uint32_t ref_size = 0;
    uint32_t ready = 0;
    uint32_t dma_max_tries = 0;
    uint32_t dma_tries = 0;
    uint32_t rv = 0;
    int8_t temp[3];
    int8_t tem[3];
    uint8_t * pPatchDatatemp = 0 ;
    uint8_t * pPatchDatatemp1 = 0;
    uint8_t * pPatchDatatem = 0;
    uint8_t * pPatchDatatem1 = 0;
    uint32_t cond=0;
    uint32_t d=0;
    uint32_t e = 0; 


    patch_data = chip->chip_priv->patch_data;
    patch_idx = 12;
    args_addr = AVL_patch_read32(patch_data, &patch_idx,1);
    data_section_offset = AVL_patch_read32(patch_data, &patch_idx,1);
    reserved_len = AVL_patch_read32(patch_data, &patch_idx,1);
    patch_idx += 4*reserved_len; //skip over reserved area for now
    script_len = AVL_patch_read32(patch_data, &patch_idx,1);


    if((patch_idx/4 + script_len) != data_section_offset) 
    {
        // while(1);
        r = AVL_EC_GENERAL_FAIL;
        return r;
    }

    script_start_idx = patch_idx/4;

    while(patch_idx/4 <(script_start_idx+script_len)) 
    {
        num_cmd_words = AVL_patch_read32(patch_data, &patch_idx,1);
        next_cmd_idx = patch_idx + (num_cmd_words-1)*4; //BYTE OFFSET
        num_cond_words = AVL_patch_read32(patch_data, &patch_idx,1);

        if(num_cond_words == 0) 
        {
            condition = 1;
        }
        else
        {
            for( cond=0; cond<num_cond_words; cond++) 
            {   
                operation = AVL_patch_read32(patch_data, &patch_idx,1);
                value = AVL_patch_read32(patch_data, &patch_idx,1);
                unary_op = (operation>>8) & 0xFF;
                binary_op = operation & 0xFF;
                addr_mode_op = ((operation>>16)&0x3);

                if( (addr_mode_op == PATCH_OP_ADDR_MODE_VAR_IDX) && (binary_op != PATCH_OP_BINARY_STORE)) 
                { 

                    value = chip->chip_priv->variable_array[value]; //grab variable value

                }

                switch(unary_op) 
                {
                case PATCH_OP_UNARY_LOGICAL_NEGATE:
                    value = !value;
                    break;
                case PATCH_OP_UNARY_BITWISE_NEGATE:
                    value = ~value;
                    break;
                case PATCH_OP_UNARY_BITWISE_AND:
                    //value = why can't C be more like Verilog?! FIXME
                    break;
                case PATCH_OP_UNARY_BITWISE_OR:
                    //value = FIXME
                    break;
                }
                switch(binary_op) 
                {
                case PATCH_OP_BINARY_LOAD:
                    condition = value;
                    break;
                case PATCH_OP_BINARY_STORE:
                    chip->chip_priv->variable_array[value] = condition;
                    break;
                case PATCH_OP_BINARY_AND:
                    condition = condition && value;
                    break;

                case PATCH_OP_BINARY_OR:
                    condition = condition || value;
                    break;
                case PATCH_OP_BINARY_BITWISE_AND:
                    condition = condition & value;
                    break;
                case PATCH_OP_BINARY_BITWISE_OR:
                    condition = condition | value;
                    break;
                case PATCH_OP_BINARY_EQUALS:
                    condition = condition == value;
                    break;
                case PATCH_OP_BINARY_NOT_EQUALS:
                    condition = condition != value;
                    break;
                default:
                    //TODO error!
                    ;;
                } 
            } //for conditions
        }

        avl_bms_read32((uint16_t)(chip->chip_pub->i2c_addr),(uint32_t)0x29A648,&tmp_top_valid);
        avl_bms_read32((uint16_t)(chip->chip_pub->i2c_addr),(uint32_t)0x0A0,&core_rdy_word);

        if(condition) 
        {
            cmd = AVL_patch_read32(patch_data, &patch_idx,1);
            switch(cmd) 
            {
            case PATCH_CMD_PING: //1
                {
                    r = IBase_SendRxOPWait_Demod(AVL_FW_CMD_PING, chip);

                    num_rvs = AVL_patch_read32(patch_data, &patch_idx,1);
                    rv0_idx = AVL_patch_read32(patch_data, &patch_idx,1);
                    chip->chip_priv->variable_array[rv0_idx] = (r == AVL_EC_OK);
                    patch_idx += 4*(num_rvs - 1); //skip over any extraneous RV's
                    break;
                }
            case PATCH_CMD_VALIDATE_CRC://0
                {
                    exp_crc_val = AVL_patch_read32(patch_data, &patch_idx,1);
                    start_addr = AVL_patch_read32(patch_data, &patch_idx,1);
                    length = AVL_patch_read32(patch_data, &patch_idx,1);

                    avl_bms_write32((uint16_t)(chip->chip_pub->i2c_addr),
                        stBaseAddrSet.fw_config_reg_base + rc_fw_command_args_addr_iaddr_offset, 
                        args_addr);
                    avl_bms_write32((uint16_t)(chip->chip_pub->i2c_addr),
                        args_addr+0, start_addr);
                    avl_bms_write32((uint16_t)(chip->chip_pub->i2c_addr),
                        args_addr+4, length);
                    r = IBase_SendRxOPWait_Demod(AVL_FW_CMD_CALC_CRC, chip);

                    avl_bms_read32((uint16_t)(chip->chip_pub->i2c_addr),
                        args_addr+8,&crc_result);

                    num_rvs = AVL_patch_read32(patch_data, &patch_idx,1);
                    rv0_idx = AVL_patch_read32(patch_data, &patch_idx,1);
                    chip->chip_priv->variable_array[rv0_idx] = (crc_result == exp_crc_val);
                    patch_idx += 4*(num_rvs - 1); //skip over any extraneous RV's

                    break;
                }
            case PATCH_CMD_LD_TO_DEVICE://2
                {
                    length = AVL_patch_read32(patch_data, &patch_idx,1); //in words  41
                    dest_addr = AVL_patch_read32(patch_data, &patch_idx,1);
                    src_data_offset = AVL_patch_read32(patch_data, &patch_idx,1);
                    src_data_offset += data_section_offset; //add in base offset
                    src_data_offset *= 4; //convert to byte offset

                    length *= 4; //Convert to byte length

                    pPatchDatatemp = patch_data + src_data_offset;
                    pPatchDatatemp1 = pPatchDatatemp - 3;
                    temp[0] = *(pPatchDatatemp -1);
                    temp[1] = *(pPatchDatatemp -2);
                    temp[2] = *(pPatchDatatemp -3);
                    avl_int_to_3bytes(dest_addr, pPatchDatatemp1);

                    r |= avl_bms_write((uint16_t)(chip->chip_pub->i2c_addr), pPatchDatatemp1, (uint32_t)(length+3));

                    * pPatchDatatemp1 = temp[2];
                    *(pPatchDatatemp1+1) = temp[1];
                    *(pPatchDatatemp1+2) = temp[0];

                    num_rvs = AVL_patch_read32(patch_data, &patch_idx,1);
                    patch_idx += 4*(num_rvs); //no RV's defined yet

                    break;
                }

            case PATCH_CMD_LD_TO_DEVICE_IMM://7
                {
                    length = AVL_patch_read32(patch_data, &patch_idx,1); //in bytes
                    dest_addr = AVL_patch_read32(patch_data, &patch_idx,1);
                    data = AVL_patch_read32(patch_data, &patch_idx,1);

                    if(length == 4) 
                    {
                        r = avl_bms_write32((uint16_t)(chip->chip_pub->i2c_addr),
                            dest_addr,data);                
                    } 
                    else if(length == 2) 
                    {
                        r = avl_bms_write16((uint16_t)(chip->chip_pub->i2c_addr),
                            dest_addr,data);                
                    } 
                    else if(length == 1) 
                    {
                        r = avl_bms_write8((uint16_t)(chip->chip_pub->i2c_addr),
                            dest_addr,data);                    
                    }
                    num_rvs = AVL_patch_read32(patch_data, &patch_idx,1);
                    patch_idx += 4*(num_rvs); //no RV's defined yet
                    break;
                }
            case PATCH_CMD_RD_FROM_DEVICE://8 8
                {
                    length = AVL_patch_read32(patch_data, &patch_idx,1); //in bytes
                    src_addr = AVL_patch_read32(patch_data, &patch_idx,1);
                    num_rvs = AVL_patch_read32(patch_data, &patch_idx,1);
                    rv0_idx = AVL_patch_read32(patch_data, &patch_idx,1);

                    if(length == 4)
                    {
                        r = avl_bms_read32((uint16_t)(chip->chip_pub->i2c_addr),
                            src_addr,&data);
                        chip->chip_priv->variable_array[rv0_idx] = data;                      
                    } 
                    else if(length == 2) 
                    {         

                        r = avl_bms_read16((uint16_t)(chip->chip_pub->i2c_addr),
                            src_addr,&data1);
                        chip->chip_priv->variable_array[rv0_idx] = data1;
                    } 
                    else if(length == 1) 
                    {

                        r = avl_bms_read8((uint16_t)(chip->chip_pub->i2c_addr),
                            src_addr,&data2);
                        chip->chip_priv->variable_array[rv0_idx] = data2;
                    }
                    patch_idx += 4*(num_rvs - 1); //skip over any extraneous RV's
                    break;
                }
            case PATCH_CMD_DMA://3
                {
                    descr_addr = AVL_patch_read32(patch_data, &patch_idx,1);
                    num_descr = AVL_patch_read32(patch_data, &patch_idx,1);

                    pPatchDatatem = patch_data + patch_idx;
                    pPatchDatatem1 = pPatchDatatem - 3;
                    tem[0] = *(pPatchDatatem -1);
                    tem[1] = *(pPatchDatatem -2);
                    tem[2] = *(pPatchDatatem -3);
                    avl_int_to_3bytes(descr_addr, pPatchDatatem1);

                    for(d=0; d<num_descr; d++)
                    {
                        for(e=0; e<3; e++) 
                        { //LENGTH, SRC_ADDR, DEST_ADDR

                            e = 3;
                            d = num_descr;                                
                        }
                    }
                    r |= avl_bms_write((uint16_t)(chip->chip_pub->i2c_addr), pPatchDatatem1, (uint32_t)(num_descr*3*4));
                    * pPatchDatatem1 = tem[2];
                    *(pPatchDatatem1+1) = tem[1];
                    *(pPatchDatatem1+2) = tem[0];
                    patch_idx += 12*num_descr;

                    avl_bms_write32((uint16_t)(chip->chip_pub->i2c_addr),
                        stBaseAddrSet.fw_config_reg_base + rc_fw_command_args_addr_iaddr_offset, 
                        descr_addr);
                    r = IBase_SendRxOPWait_Demod(AVL_FW_CMD_DMA, chip);

                    num_rvs = AVL_patch_read32(patch_data, &patch_idx,1);
                    patch_idx += 4*(num_rvs); //no RV's defined yet
                    break;

                }
            case PATCH_CMD_DECOMPRESS://4
                {
                    type = AVL_patch_read32(patch_data, &patch_idx,1);
                    src_addr = AVL_patch_read32(patch_data, &patch_idx,1);
                    dest_addr = AVL_patch_read32(patch_data, &patch_idx,1);

                    if(type == PATCH_CMP_TYPE_ZLIB) 
                    {
                        ref_addr = AVL_patch_read32(patch_data, &patch_idx,1);
                        ref_size = AVL_patch_read32(patch_data, &patch_idx,1);
                    }

                    avl_bms_write32((uint16_t)(chip->chip_pub->i2c_addr),
                        stBaseAddrSet.fw_config_reg_base + rc_fw_command_args_addr_iaddr_offset, 
                        args_addr);
                    avl_bms_write32((uint16_t)(chip->chip_pub->i2c_addr),
                        args_addr+0, type);
                    avl_bms_write32((uint16_t)(chip->chip_pub->i2c_addr),
                        args_addr+4, src_addr);
                    avl_bms_write32((uint16_t)(chip->chip_pub->i2c_addr),
                        args_addr+8, dest_addr);
                    if(type == PATCH_CMP_TYPE_ZLIB) {
                        avl_bms_write32((uint16_t)(chip->chip_pub->i2c_addr),
                            args_addr+12, ref_addr);
                        avl_bms_write32((uint16_t)(chip->chip_pub->i2c_addr),
                            args_addr+16, ref_size);
                    }

                    r = IBase_SendRxOPWait_Demod(AVL_FW_CMD_DECOMPRESS, chip);

                    num_rvs = AVL_patch_read32(patch_data, &patch_idx,1);
                    patch_idx += 4*(num_rvs); //no RV's defined yet
                    break;
                }
            case PATCH_CMD_ASSERT_CPU_RESET://5
                {
                    r |= avl_bms_write32(chip->chip_pub->i2c_addr, 
                        stBaseAddrSet.hw_mcu_reset_base, 1);

                    num_rvs = AVL_patch_read32(patch_data, &patch_idx,1);
                    patch_idx += 4*(num_rvs); //no RV's defined yet
                    break;
                }
            case PATCH_CMD_RELEASE_CPU_RESET://6
                {
                    //FIXME: are both of these resets necessary? Does one reset the sys DMA??
                    avl_bms_write32((uint16_t)(chip->chip_pub->i2c_addr), stBaseAddrSet.hw_mcu_reset_base, 0);

                    num_rvs = AVL_patch_read32(patch_data, &patch_idx,1);
                    patch_idx += 4*(num_rvs); //no RV's defined yet
                    break;
                }
            case PATCH_CMD_DMA_HW://9
                {

                    //hw_AVL_dma_sys_status          hw_AVL_dma_sys_cmd
                    descr_addr = AVL_patch_read32(patch_data, &patch_idx,1);
                    num_descr = AVL_patch_read32(patch_data, &patch_idx,1); //10

                    temp[0] = *(patch_data + patch_idx -1);
                    temp[1] = *(patch_data + patch_idx -2);
                    temp[2] = *(patch_data + patch_idx -3);
                    avl_int_to_3bytes(descr_addr, patch_data + patch_idx -3);

                    if(num_descr >0)
                    {
                        r |= avl_bms_write((uint16_t)(chip->chip_pub->i2c_addr), patch_data + patch_idx -3, (uint16_t)(num_descr*12+3));//not putting in >2^16 size yet, come on, thats alot of dma's...
                    }

                    *(patch_data + patch_idx -1) = temp[0];
                    *(patch_data + patch_idx -2) = temp[1];
                    *(patch_data + patch_idx -3) = temp[2];

                    patch_idx += num_descr * 3 * 4;                
                    dma_max_tries = 20;                 
                    do{
                        if(dma_tries > dma_max_tries)
                        {
                            return AVL_EC_GENERAL_FAIL; //FIXME return a value to check instead, and load the bootstrap
                            break;
                        }
                        r |= avl_bms_read32((uint16_t)(chip->chip_pub->i2c_addr), 
                            stBaseAddrSet.hw_dma_sys_status_base, 
                            &ready);
                        dma_tries++;
                    } while(!(0x01 & ready));

                    if(ready)
                    {
                        r |= avl_bms_write32((uint16_t)(chip->chip_pub->i2c_addr),
                            stBaseAddrSet.hw_dma_sys_cmd_base,
                            descr_addr); //Trigger DMA
                    }
                    //Add return value later
                    num_rvs = AVL_patch_read32(patch_data, &patch_idx,1);
                    patch_idx += 4*(num_rvs); //no RV's defined yet
                    break;
                }

            case PATCH_CMD_SET_COND_IMM://10
                {
                    rv = AVL_patch_read32(patch_data, &patch_idx,1);
                    num_rvs = AVL_patch_read32(patch_data, &patch_idx,1);
                    rv0_idx = AVL_patch_read32(patch_data, &patch_idx,1);
                    chip->chip_priv->variable_array[rv0_idx] = rv;
                    patch_idx += 4*(num_rvs - 1); //skip over any extraneous RV's
                    break;
                }
            }
        } 
        else {
            patch_idx = next_cmd_idx; //jump to next command
            continue;
        }
    }

    return r;
}

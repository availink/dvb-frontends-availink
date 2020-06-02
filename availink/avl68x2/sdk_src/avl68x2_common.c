// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Availink AVL68x2 DVB-S/S2/T/T2/C, ISDB-T, J83.B demodulator driver
 *
 * Copyright (C) 2020 Availink, Inc. (gpl@availink.com)
 *
 */

#include "avl68x2_common.h"

avl_error_code_t avl68x2_demod_initialize(
    AVL_DemodMode eStartupMode,
    avl68x2_chip *chip)
{
	avl_error_code_t r = AVL_EC_OK;
	uint32_t uiMaxRetries = 100;
	uint32_t delay_unit_ms = 20; //the time out window is 10*20=200ms
	uint32_t i = 0;
	uint32_t j = 0;

	chip->chip_pub->cur_demod_mode = eStartupMode;

	for (i = 0; i < PATCH_VAR_ARRAY_SIZE; i++)
	{
		chip->chip_priv->variable_array[i] = 0;
	}
	chip->chip_priv->sleep_flag = 0;

	//r = avl68x2_init_chip_object(chip);

	r |= GetFamilyID_Demod(&(chip->family_id), chip);

	r |= IBase_Initialize_Demod(chip); //config PLL, boot FW

	while (AVL_EC_OK != IBase_CheckChipReady_Demod(chip))
	{
		if (uiMaxRetries <= j++)
		{
			r |= AVL_EC_GENERAL_FAIL;
			break;
		}
		avl_bsp_delay(delay_unit_ms);
	}

	r |= SetInternalFunc_Demod(chip->chip_pub->cur_demod_mode, chip);

	r |= IRx_Initialize_Demod(chip); //load defaults

	r |= SetTSMode_Demod(chip);

	r |= EnableTSOutput_Demod(chip);

	r |= TunerI2C_Initialize_Demod(chip);
	r |= InitErrorStat_Demod(chip);

	r |= Initilize_GPIOStatus_Demod(chip);

	return r;
}

avl_error_code_t avl68x2_demod_get_chip_id(uint32_t *puiChipID, avl68x2_chip *chip)
{
	avl_error_code_t r = AVL_EC_OK;

	r = GetFamilyID_Demod(&(chip->family_id), chip);

	r |= avl_bms_read32(chip->chip_pub->i2c_addr,
			    stBaseAddrSet.hw_member_ID_base,
			    puiChipID);

	return r;
}

avl_error_code_t avl68x2_demod_get_lock_status(uint8_t * pucDemodLocked, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    *pucDemodLocked = 0;

    if(chip->stStdSpecFunc->fpGetLockStatus)
    {
        r = chip->stStdSpecFunc->fpGetLockStatus(pucDemodLocked, chip);
    }

    return r;
}

avl_error_code_t avl68x2_demod_get_snr (uint32_t * puiSNRx100, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    *puiSNRx100 = 0;

    if(chip->stStdSpecFunc->fpGetSNR)
    {
        r = chip->stStdSpecFunc->fpGetSNR(puiSNRx100, chip);
    }

    return (r);
}

avl_error_code_t avl68x2_demod_get_sqi (uint16_t * pusSQI, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    *pusSQI = 0;
    if(chip->stStdSpecFunc->fpGetSQI)
    {
        r = chip->stStdSpecFunc->fpGetSQI(pusSQI, chip);
    }

    return (r);
}

avl_error_code_t avl68x2_demod_get_ssi(uint16_t * pusSSI, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    *pusSSI = 0;
    r = avl_bms_read16(chip->chip_pub->i2c_addr, 
        stBaseAddrSet.fw_status_reg_base + rs_rf_agc_saddr_offset, pusSSI);

    return (r);
}

avl_error_code_t avl68x2_demod_get_per(uint32_t *puiPERxe9, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint32_t uiHwCntPktErrors = 0;
    uint32_t uiHwCntNumPkts = 0;
    uint32_t uiTemp = 0;
    struct avl_uint64 uiTemp64 = {0,0};
    uint8_t uclock_status = 0;

    r = avl68x2_demod_get_lock_status(&uclock_status, chip);

    //record the lock status before return the PER
    if(1 == uclock_status)
    {
        chip->stAVLErrorStat.usLostLock = 0;
    }
    else
    {
        chip->stAVLErrorStat.usLostLock = 1;
        *puiPERxe9 = AVL_CONSTANT_10_TO_THE_9TH;
        return r;
    }

    r |= avl_bms_read32(chip->chip_pub->i2c_addr, 
                           stBaseAddrSet.hw_esm_base + packet_error_offset, &uiHwCntPktErrors);
    r |= avl_bms_read32(chip->chip_pub->i2c_addr, 
                           stBaseAddrSet.hw_esm_base + packet_num_offset, &uiHwCntNumPkts);

    if(uiHwCntNumPkts > (uint32_t)(1 << 31))
    {
        //write 1
        r |= avl_bms_read32(chip->chip_pub->i2c_addr, stBaseAddrSet.hw_esm_base, &uiTemp);
        uiTemp |= 0x00000001;
        r |= avl_bms_write32(chip->chip_pub->i2c_addr, 
            stBaseAddrSet.hw_esm_base, uiTemp);

        r |= avl_bms_read32(chip->chip_pub->i2c_addr, 
            stBaseAddrSet.hw_esm_base + packet_error_offset, &uiHwCntPktErrors);
        r |= avl_bms_read32(chip->chip_pub->i2c_addr, 
            stBaseAddrSet.hw_esm_base + packet_num_offset, &uiHwCntNumPkts);

        //write 0
        uiTemp &= 0xFFFFFFFE;
        r |= avl_bms_write32(chip->chip_pub->i2c_addr, 
            stBaseAddrSet.hw_esm_base, uiTemp);

        avl_add_32to64(&chip->stAVLErrorStat.stSwCntNumPkts, uiHwCntNumPkts);
        avl_add_32to64(&chip->stAVLErrorStat.stSwCntPktErrors, uiHwCntPktErrors);

        uiHwCntNumPkts = 0;
        uiHwCntPktErrors = 0;
    }

    chip->stAVLErrorStat.stNumPkts.high_word = chip->stAVLErrorStat.stSwCntNumPkts.high_word;
    chip->stAVLErrorStat.stNumPkts.low_word = chip->stAVLErrorStat.stSwCntNumPkts.low_word;
    avl_add_32to64(&chip->stAVLErrorStat.stNumPkts, uiHwCntNumPkts);

    chip->stAVLErrorStat.stPktErrors.high_word = chip->stAVLErrorStat.stSwCntPktErrors.high_word;
    chip->stAVLErrorStat.stPktErrors.low_word = chip->stAVLErrorStat.stSwCntPktErrors.low_word;
    avl_add_32to64(&chip->stAVLErrorStat.stPktErrors, uiHwCntPktErrors);

    avl_mult_32to64(&uiTemp64, chip->stAVLErrorStat.stPktErrors.low_word, AVL_CONSTANT_10_TO_THE_9TH);
    chip->stAVLErrorStat.uiPER = avl_divide_64(chip->stAVLErrorStat.stNumPkts, uiTemp64);

    *puiPERxe9 = chip->stAVLErrorStat.uiPER;

    return r;

}

avl_error_code_t avl68x2_demod_set_mode(AVL_DemodMode eDemodMode, avl68x2_chip *chip)
{
	avl_error_code_t r = AVL_EC_OK;
	uint32_t uiTimeDelay = 10;
	uint32_t uiMaxRetries = 80;
	uint8_t *pInitialData = 0;
	uint32_t i = 0;

	r = GetMode_Demod(&chip->chip_pub->cur_demod_mode, chip);
	if (r != AVL_EC_OK)
	{
		return r;
	}

	r |= IBase_SendRxOPWait_Demod(AVL_FW_CMD_HALT, chip);

	if (chip->chip_pub->cur_demod_mode != eDemodMode)
	{
		//switch modes
		pInitialData = chip->chip_priv->patch_data;

		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
					stBaseAddrSet.hw_mcu_system_reset_base, 1);

		// Configure the PLL
		if ((chip->chip_pub->cur_demod_mode != AVL_DVBSX && eDemodMode == AVL_DVBSX) ||
			(chip->chip_pub->cur_demod_mode == AVL_DVBSX && eDemodMode != AVL_DVBSX))
		{
			chip->chip_pub->cur_demod_mode = eDemodMode;
			r |= SetPLL_Demod(chip);
		}
		else
		{
			chip->chip_pub->cur_demod_mode = eDemodMode;
		}

		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
					stBaseAddrSet.fw_status_reg_base +
						rs_core_ready_word_iaddr_offset,
					0x00000000);

		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
					stBaseAddrSet.hw_mcu_system_reset_base, 0);

		
		r |= AVL_ParseFwPatch_v0(chip, 0);

		while (AVL_EC_OK != IBase_CheckChipReady_Demod(chip))
		{
			if (uiMaxRetries <= i++)
			{
				r |= AVL_EC_GENERAL_FAIL;
				break;
			}
			avl_bsp_delay(uiTimeDelay);
		}
		avl_bms_read32(chip->chip_pub->i2c_addr,rs_core_ready_word_iaddr_offset, &i);
		
		chip->chip_pub->cur_demod_mode = eDemodMode;

		r |= SetInternalFunc_Demod(chip->chip_pub->cur_demod_mode, chip);

		r |= IRx_Initialize_Demod(chip);
	}

	

	printk("%s:%d r=%d\n", __FUNCTION__, __LINE__, r);
	r |= SetTSMode_Demod(chip);
	r |= EnableTSOutput_Demod(chip);
	printk("%s:%d r=%d\n", __FUNCTION__, __LINE__, r);

	//r |= SetGPIOStatus_Demod(chip);

	r |= TunerI2C_Initialize_Demod(chip);

	r |= InitErrorStat_Demod(chip);

	return r;
}

avl_error_code_t avl68x2_demod_sleep(avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    r |= IBase_SendRxOPWait_Demod(AVL_FW_CMD_HALT, chip);

    r |= IBase_SetSleepClock_Demod(chip);

    r |= IBase_SendRxOPWait_Demod(AVL_FW_CMD_SLEEP, chip);

    if(AVL_EC_OK == r)
    {
        chip->chip_priv->sleep_flag = 1;
    }

    r |= TunerI2C_Initialize_Demod(chip);

    r |= SetGPIOStatus_Demod(chip);


    return r;

}

avl_error_code_t avl68x2_demod_wakeup(avl68x2_chip *chip)
{

    avl_error_code_t r = AVL_EC_OK;
    uint32_t uiTimeDelay = 5;
    uint32_t uiMaxRetries = 10;
    uint32_t uiIndex=0;

    r |= IBase_SendRxOPWait_Demod(AVL_FW_CMD_HALT, chip );

    r = IBase_SendRxOPWait_Demod(AVL_FW_CMD_WAKE, chip); 

    if(r != AVL_EC_OK)
    {
        return r;
    }

    r |= avl_bms_write32(chip->chip_pub->i2c_addr, 
        stBaseAddrSet.hw_mcu_reset_base, 1);
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, 
        stBaseAddrSet.hw_mcu_system_reset_base, 1);

    r |= SetPLL_Demod(chip);

    r |= avl_bms_write32(chip->chip_pub->i2c_addr,
        stBaseAddrSet.fw_status_reg_base + rs_core_ready_word_iaddr_offset, 0x00000000);

    r |= avl_bms_write32(chip->chip_pub->i2c_addr, 
        stBaseAddrSet.hw_mcu_system_reset_base, 0);
    r |= avl_bms_write32(chip->chip_pub->i2c_addr, 
        stBaseAddrSet.hw_mcu_reset_base, 0);

    while (AVL_EC_OK != IBase_CheckChipReady_Demod(chip))
    {
        if (uiMaxRetries < uiIndex++)
        {
            return AVL_EC_GENERAL_FAIL;
        }
        avl_bsp_delay(uiTimeDelay);
    }

    r |= IRx_Initialize_Demod(chip);

    r |= SetTSMode_Demod(chip);
    r |= SetTSSerialPin_Demod(chip->chip_pub->ts_config.eSerialPin, chip);
    r |= SetTSSerialOrder_Demod(chip->chip_pub->ts_config.eSerialOrder, chip);
    r |= SetTSSerialSyncPulse_Demod(chip->chip_pub->ts_config.eSerialSyncPulse, chip);
    r |= SetTSErrorBit_Demod(chip->chip_pub->ts_config.eErrorBit, chip);
    r |= SetTSErrorPola_Demod(chip->chip_pub->ts_config.eErrorPolarity, chip);
    r |= SetTSValidPola_Demod(chip->chip_pub->ts_config.eValidPolarity, chip);
    r |= SetTSParallelOrder_Demod(chip->chip_pub->ts_config.eParallelOrder, chip);
    r |= SetTSParallelPhase_Demod(chip->chip_pub->ts_config.eParallelPhase, chip);



    r |= EnableTSOutput_Demod(chip);

    r |= TunerI2C_Initialize_Demod(chip);

    r |= InitErrorStat_Demod(chip);

    r |= SetGPIOStatus_Demod(chip);


    r |= GetMode_Demod(&chip->chip_pub->cur_demod_mode, chip);
    if(chip->chip_pub->cur_demod_mode == AVL_ISDBT)
    {
        r |= avl_bms_write32(chip->chip_pub->i2c_addr,
            stBaseAddrSet.hw_emerald_io_base + emerald_io_pad_CS_0_sel_offset , 0x1);
    }


    if(AVL_EC_OK == r)
    {
        chip->chip_priv->sleep_flag = 0;
    }

    return r;

}

avl_error_code_t avl68x2_demod_i2c_passthru_on(avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint8_t   ucNum = 0;

    for(ucNum = 0; ucNum < 2; ucNum++)
    {
        r = avl_bms_write32(chip->chip_pub->i2c_addr,
            stBaseAddrSet.hw_tuner_i2c_base + tuner_i2c_bit_rpt_cntrl_offset, 0x07);
    }
    return (r);
}

avl_error_code_t avl68x2_demod_i2c_passthru_off(avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint8_t   ucNum = 0;

    for(ucNum = 0; ucNum < 1; ucNum++)
    {
        r = avl_bms_write32(chip->chip_pub->i2c_addr,
            stBaseAddrSet.hw_tuner_i2c_base + tuner_i2c_bit_rpt_cntrl_offset, 0x06);
    }
    return (r);
}

avl_error_code_t avl68x2_demod_ts_on(avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    r = avl_bms_write32(chip->chip_pub->i2c_addr, 
        stBaseAddrSet.hw_TS_tri_state_cntrl_base, 0);

    return r;
}

avl_error_code_t avl68x2_demod_ts_off(avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    r = avl_bms_write32(chip->chip_pub->i2c_addr, 
        stBaseAddrSet.hw_TS_tri_state_cntrl_base, 0xfff);

    return r;
}

avl_error_code_t avl68x2_demod_get_version(
    AVL_DemodVersion *pstDemodVersion,
    avl68x2_chip *chip)
{
	avl_error_code_t r = AVL_EC_OK;
	uint32_t uiTemp = 0;
	uint8_t ucBuff[4] = {0};

	r = avl_bms_read32(chip->chip_pub->i2c_addr, 0x40000, &uiTemp);
	if (AVL_EC_OK == r)
	{
		pstDemodVersion->hardware = uiTemp;
	}

	r |= avl_bms_read32(chip->chip_pub->i2c_addr,
			    stBaseAddrSet.fw_status_reg_base +
				rs_patch_ver_iaddr_offset,
			    &uiTemp);
	if (AVL_EC_OK == r)
	{
		avl_int_to_bytes(uiTemp, ucBuff);
		pstDemodVersion->firmware.major = ucBuff[0];
		pstDemodVersion->firmware.minor = ucBuff[1];
		pstDemodVersion->firmware.build = ucBuff[2];
		pstDemodVersion->firmware.build =
		    ((uint16_t)((pstDemodVersion->firmware.build) << 8)) + ucBuff[3];

		pstDemodVersion->driver.major = AVL68X2_VER_MAJOR;
		pstDemodVersion->driver.minor = AVL68X2_VER_MINOR;
		pstDemodVersion->driver.build = AVL68X2_VER_BUILD;
	}

	return r;
}

avl_error_code_t avl68x2_demod_set_gpio(AVL_GPIOPinNumber ePinNumber, AVL_GPIOPinValue ePinValue, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    switch(ePinNumber)
    {
    case AVL_Pin15:
        switch(ePinValue)
        {
        case AVL_LOGIC_0:
            r = avl_bms_write32(chip->chip_pub->i2c_addr,
                stBaseAddrSet.hw_gpio_modu_002_base + gpio_module_002_gpio_config_offset, 0x2);
        break;
        case AVL_LOGIC_1:
            r = avl_bms_write32(chip->chip_pub->i2c_addr,
                stBaseAddrSet.hw_gpio_modu_002_base + gpio_module_002_gpio_config_offset, 0x3);  

            break;
        default:
            break;
        }
        break;

    case AVL_Pin37:

        switch(ePinValue)
        {
        case AVL_LOGIC_0:
            r = avl_bms_write32(chip->chip_pub->i2c_addr,
                stBaseAddrSet.hw_gpio_debug_base + lnb_cntrl_0_sel_offset, ePinValue);
            chip->ucPin37Voltage = 0;
            break;
        case AVL_LOGIC_1:
            r = avl_bms_write32(chip->chip_pub->i2c_addr,
                stBaseAddrSet.hw_gpio_debug_base + lnb_cntrl_0_sel_offset, ePinValue);
            chip->ucPin37Voltage = 1;
            break;
        case AVL_HIGH_Z:
            r = avl_bms_write32(chip->chip_pub->i2c_addr,
                stBaseAddrSet.hw_gpio_debug_base + lnb_cntrl_0_sel_offset, ePinValue);
            chip->ucPin37Voltage = 2;
            break;
        default:
            break;
        }
        break;
    case AVL_Pin38:

        switch(ePinValue)
        {
        case AVL_LOGIC_0:
            r = avl_bms_write32(chip->chip_pub->i2c_addr,
                stBaseAddrSet.hw_gpio_debug_base + lnb_cntrl_1_sel_offset, ePinValue);
            chip->ucPin38Voltage = 0;
            break;
        case AVL_LOGIC_1:
            r = avl_bms_write32(chip->chip_pub->i2c_addr,
                stBaseAddrSet.hw_gpio_debug_base + lnb_cntrl_1_sel_offset, ePinValue);
            chip->ucPin38Voltage = 1;
            break;
        case AVL_HIGH_Z:
            r = avl_bms_write32(chip->chip_pub->i2c_addr,
                stBaseAddrSet.hw_gpio_debug_base + lnb_cntrl_1_sel_offset, ePinValue);
            chip->ucPin38Voltage = 2;
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }

    return r;
}

avl_error_code_t avl68x2_demod_get_gpio(AVL_GPIOPinNumber ePinNumber, AVL_GPIOPinValue *pePinValue, avl68x2_chip *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint32_t uiTemp = 0;

    switch(ePinNumber)
    {
    case AVL_Pin15:
        r = avl_bms_read32(chip->chip_pub->i2c_addr,
            stBaseAddrSet.hw_gpio_modu_002_base + gpio_module_002_gpio_config_offset, &uiTemp);
        if(uiTemp == 0x07)
        {
            *pePinValue = (AVL_GPIOPinValue)1;
        }
        else if(uiTemp == 0x02)
        {
            *pePinValue = (AVL_GPIOPinValue)0;
        }
        break;

    case AVL_Pin37:
        r = avl_bms_write32(chip->chip_pub->i2c_addr,
            stBaseAddrSet.hw_gpio_debug_base + lnb_cntrl_0_sel_offset, AVL_HIGH_Z);

        r = avl_bms_read32(chip->chip_pub->i2c_addr,
            stBaseAddrSet.hw_gpio_debug_base + lnb_cntrl_0_i_offset, (uint32_t *)pePinValue);
        break;

    case AVL_Pin38:
        r = avl_bms_write32(chip->chip_pub->i2c_addr,
            stBaseAddrSet.hw_gpio_debug_base + lnb_cntrl_1_sel_offset, AVL_HIGH_Z);

        r = avl_bms_read32(chip->chip_pub->i2c_addr,
            stBaseAddrSet.hw_gpio_debug_base + lnb_cntrl_1_i_offset, (uint32_t *)pePinValue);
        break;
    default:
        break;
    }

    return r;
}


#include "AVL_Demod.h"


extern AVL_PLL_Conf0 gstPLLConfigArray0[8];
extern AVL_CommonConfig AVL_CommonConfigChip;
extern AVL_DVBTxConfig AVL_DVBTxConfigChip;
extern AVL_DVBSxConfig AVL_DVBSxConfigChip;
extern AVL_ISDBTConfig AVL_ISDBTConfigChip;
extern AVL_DVBCConfig AVL_DVBCConfigChip;

avl_error_code_t AVL_Demod_Initialize(AVL_DemodMode eStartupMode, AVL_ChipInternal *chip, uint8_t bI2CChipSelect)
{
    avl_error_code_t r = AVL_EC_OK;
    uint32_t uiMaxRetries = 100;
    uint32_t delay_unit_ms = 20;//the time out window is 10*20=200ms
    uint32_t i = 0;
    uint32_t j = 0;

    chip->usI2CAddr = AVL_CommonConfigChip.usI2CAddr + (uint16_t)bI2CChipSelect;
    chip->eDemodXtal = AVL_CommonConfigChip.eDemodXtal;
    chip->eCurrentDemodMode = eStartupMode;
    chip->stTSConfig.eMode = AVL_CommonConfigChip.eTSMode;
    chip->stTSConfig.eClockEdge = AVL_CommonConfigChip.eClockEdge;
    chip->stTSConfig.eClockMode = AVL_CommonConfigChip.eClockMode;
    
    chip->stDVBTxPara.eDVBTxInputPath = AVL_DVBTxConfigChip.eDVBTxInputPath;
    chip->stDVBTxPara.uiDVBTxIFFreqHz = AVL_DVBTxConfigChip.uiDVBTxIFFreqHz;
    chip->stDVBTxPara.eDVBTxAGCPola = AVL_DVBTxConfigChip.eDVBTxAGCPola;
    
    chip->stDVBSxPara.eDVBSxAGCPola = AVL_DVBSxConfigChip.eDVBSxAGCPola;
    chip->stDVBSxPara.e22KWaveForm = AVL_DVBSxConfigChip.e22KWaveForm;
    
    chip->stISDBTPara.eISDBTInputPath = AVL_ISDBTConfigChip.eISDBTInputPath;
    chip->stISDBTPara.eISDBTBandwidth = AVL_ISDBTConfigChip.eISDBTBandwidth;
    chip->stISDBTPara.uiISDBTIFFreqHz = AVL_ISDBTConfigChip.uiISDBTIFFreqHz;
    chip->stISDBTPara.eISDBTAGCPola = AVL_ISDBTConfigChip.eISDBTAGCPola;
    
    
    chip->stDVBCPara.eDVBCInputPath = AVL_DVBCConfigChip.eDVBCInputPath;
    chip->stDVBCPara.uiDVBCIFFreqHz = AVL_DVBCConfigChip.uiDVBCIFFreqHz;
    chip->stDVBCPara.uiDVBCSymbolRateSps = AVL_DVBCConfigChip.uiDVBCSymbolRateSps;
    chip->stDVBCPara.eDVBCAGCPola = AVL_DVBCConfigChip.eDVBCAGCPola;
    chip->stDVBCPara.eDVBCStandard = AVL_DVBCConfigChip.eDVBCStandard;
    
    for(i=0; i<PATCH_VAR_ARRAY_SIZE; i++) 
      {
        chip->variable_array[i] = 0;
      }
    chip->ucSleepFlag = 0;

    r = InitSemaphore_Demod(chip);
    
    r |= GetFamilyID_Demod(&(chip->uiFamilyID), chip);

    r |= IBase_Initialize_Demod(chip);

    while (AVL_EC_OK != IBase_CheckChipReady_Demod(chip))
    {
        if (uiMaxRetries <= j++)
        {
            r |= AVL_EC_GENERAL_FAIL;
            break;
        }
        avl_bsp_delay(delay_unit_ms);
    }

    r |= SetInternalFunc_Demod(chip->eCurrentDemodMode, chip);

    chip->ucDisableTCAGC = 0;
    chip->ucDisableSAGC = 0;

    r |= IRx_Initialize_Demod(chip);

    r |= SetTSMode_Demod(chip);

    r |= SetTSSerialPin_Demod(AVL_MPSP_DATA0, chip);
    r |= SetTSSerialOrder_Demod(AVL_MPBO_MSB, chip);
    r |= SetTSSerialSyncPulse_Demod(AVL_TS_SERIAL_SYNC_1_PULSE, chip);
    r |= SetTSErrorBit_Demod(AVL_TS_ERROR_BIT_DISABLE, chip);
    r |= SetTSErrorPola_Demod(AVL_MPEP_Normal, chip);
    r |= SetTSValidPola_Demod(AVL_MPVP_Normal, chip);
    r |= SetTSPacketLen_Demod(AVL_TS_188, chip);
    r |= SetTSParallelOrder_Demod(AVL_TS_PARALLEL_ORDER_NORMAL, chip);
    r |= SetTSParallelPhase_Demod(AVL_TS_PARALLEL_PHASE_0, chip);  //it's available for parallel and serial mode.

    r |= EnableTSOutput_Demod(chip);

    r |= TunerI2C_Initialize_Demod(chip);
    r |= InitErrorStat_Demod(chip);

    // Enable CS_0 as GPIO for EWBS feature
    if(eStartupMode == AVL_ISDBT )
    {
        r |= avl_bms_write32(chip->usI2CAddr,
            stBaseAddrSet.hw_emerald_io_base + emerald_io_pad_CS_0_sel_offset , 0x1);
    }
   
    r |= avl_bms_write8(chip->usI2CAddr, 0xA83, 0x01);  // for ISDBT Layer independence lock

    r |= Initilize_GPIOStatus_Demod(chip);
  
    return r;    
}

avl_error_code_t AVL_Demod_GetChipID(uint32_t * puiChipID,AVL_ChipInternal *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint32_t uiMemberIDRegAddr = 0x0;
    uint16_t usAddrSize = 3;
    uint16_t usDataSize = 4;
    uint8_t pucBuffAddr[3] = {0};
    uint8_t pucBuffData[4]= {0};

    r = GetFamilyID_Demod(&(chip->uiFamilyID), chip);

    uiMemberIDRegAddr = stBaseAddrSet.hw_member_ID_base;

    ChunkAddr_Demod(uiMemberIDRegAddr, pucBuffAddr);

    r = avl_bsp_wait_semaphore(&(chip->semI2C));
    r |= avl_bsp_i2c_write(chip->usI2CAddr, pucBuffAddr, &usAddrSize);
    r |= avl_bsp_i2c_read(chip->usI2CAddr, pucBuffData, &usDataSize);
    r |= avl_bsp_release_semaphore(&(chip->semI2C));

    *puiChipID = DeChunk32_Demod(pucBuffData);

    return r;
}

avl_error_code_t AVL_Demod_GetLockStatus(uint8_t * pucDemodLocked, AVL_ChipInternal *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    *pucDemodLocked = 0;

    if(chip->stStdSpecFunc->fpGetLockStatus)
    {
        r = chip->stStdSpecFunc->fpGetLockStatus(pucDemodLocked, chip);
    }

    return r;
}

avl_error_code_t AVL_Demod_GetSNR (uint32_t * puiSNRx100, AVL_ChipInternal *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    *puiSNRx100 = 0;

    if(chip->stStdSpecFunc->fpGetSNR)
    {
        r = chip->stStdSpecFunc->fpGetSNR(puiSNRx100, chip);
    }

    return (r);
}

avl_error_code_t AVL_Demod_GetSQI (uint16_t * pusSQI, AVL_ChipInternal *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    *pusSQI = 0;
    if(chip->stStdSpecFunc->fpGetSQI)
    {
        r = chip->stStdSpecFunc->fpGetSQI(pusSQI, chip);
    }

    return (r);
}

avl_error_code_t AVL_Demod_GetSSI(uint16_t * pusSSI, AVL_ChipInternal *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    *pusSSI = 0;
    r = avl_bms_read16(chip->usI2CAddr, 
        stBaseAddrSet.fw_status_reg_base + rs_rf_agc_saddr_offset, pusSSI);

    return (r);
}

avl_error_code_t AVL_Demod_GetPER(uint32_t *puiPERxe9, AVL_ChipInternal *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint32_t uiHwCntPktErrors = 0;
    uint32_t uiHwCntNumPkts = 0;
    uint32_t uiTemp = 0;
    struct avl_uint64 uiTemp64 = {0,0};
    uint8_t uclock_status = 0;

    r = AVL_Demod_GetLockStatus(&uclock_status, chip);

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

    r |= avl_bms_read32(chip->usI2CAddr, 
                           stBaseAddrSet.hw_esm_base + packet_error_offset, &uiHwCntPktErrors);
    r |= avl_bms_read32(chip->usI2CAddr, 
                           stBaseAddrSet.hw_esm_base + packet_num_offset, &uiHwCntNumPkts);

    if(uiHwCntNumPkts > (uint32_t)(1 << 31))
    {
        //write 1
        r |= avl_bms_read32(chip->usI2CAddr, stBaseAddrSet.hw_esm_base, &uiTemp);
        uiTemp |= 0x00000001;
        r |= avl_bms_write32(chip->usI2CAddr, 
            stBaseAddrSet.hw_esm_base, uiTemp);

        r |= avl_bms_read32(chip->usI2CAddr, 
            stBaseAddrSet.hw_esm_base + packet_error_offset, &uiHwCntPktErrors);
        r |= avl_bms_read32(chip->usI2CAddr, 
            stBaseAddrSet.hw_esm_base + packet_num_offset, &uiHwCntNumPkts);

        //write 0
        uiTemp &= 0xFFFFFFFE;
        r |= avl_bms_write32(chip->usI2CAddr, 
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

    Multiply32_Demod(&uiTemp64, chip->stAVLErrorStat.stPktErrors.low_word, AVL_CONSTANT_10_TO_THE_9TH);
    chip->stAVLErrorStat.uiPER = Divide64_Demod(chip->stAVLErrorStat.stNumPkts, uiTemp64);

    *puiPERxe9 = chip->stAVLErrorStat.uiPER;

    return r;

}

avl_error_code_t AVL_Demod_SetMode(AVL_DemodMode eDemodMode,AVL_ChipInternal *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint32_t uiTimeDelay = 5;
    uint32_t uiMaxRetries = 40;//time out window 5*40 = 200ms
    uint8_t * pInitialData = 0;
    uint32_t i = 0;
    uint8_t uc_dl_patch_parse_format = 0;
    uint32_t ui_patch_idx = 0;
    uint32_t ui_patch_script_version = 0;

    r = GetMode_Demod(&chip->eCurrentDemodMode, chip);
    if(r != AVL_EC_OK)
    {
        return r;
    }


    if(chip->eCurrentDemodMode == eDemodMode)
    {
        return AVL_EC_OK;
    }

    r |= IBase_SendRxOPWait_Demod(AVL_FW_CMD_HALT, chip);

    pInitialData = chip->fwData;

    if( AVL_EC_OK == r )
    {

        if((pInitialData[0] & 0x0f0) == 0x10)
        {
            uc_dl_patch_parse_format = 1;
        } 
        else 
        {
            return AVL_EC_GENERAL_FAIL;//Neither format enumeration was found. Firmware File is Corrupt
        }
    }

    if( AVL_EC_OK == r )
    {

        r |= avl_bms_write32(chip->usI2CAddr, 
            stBaseAddrSet.hw_mcu_system_reset_base, 1);


        // Configure the PLL
        if( (chip->eCurrentDemodMode != AVL_DVBSX && eDemodMode == AVL_DVBSX) ||
            (chip->eCurrentDemodMode == AVL_DVBSX && eDemodMode != AVL_DVBSX) )
        {
            chip->eCurrentDemodMode = eDemodMode;
            r |= SetPLL_Demod(chip);
        }
        else
        {
            chip->eCurrentDemodMode = eDemodMode;
        }

        if (AVL_EC_OK == r)
        {
            if(uc_dl_patch_parse_format)
            {         
                r |= avl_bms_write32(chip->usI2CAddr, 
                    stBaseAddrSet.fw_status_reg_base + rs_core_ready_word_iaddr_offset, 0x00000000);

                r |= avl_bms_write32(chip->usI2CAddr, 
                    stBaseAddrSet.hw_mcu_system_reset_base, 0);

                ui_patch_script_version = AVL_patch_read32(pInitialData, &ui_patch_idx,1);

                if((ui_patch_script_version & 0x00ff) == 0x1) 
                { //read patch script version
                    r |= AVL_ParseFwPatch_v0(chip, 0);
                } 
                else 
                {
                    r |= AVL_EC_GENERAL_FAIL;
                    return r;
                }     

            } 
        }
    }
    while (AVL_EC_OK != IBase_CheckChipReady_Demod(chip))
    {
        if (uiMaxRetries <= i++)
        {
            r |= AVL_EC_GENERAL_FAIL;
            break;
        }
        avl_bsp_delay(uiTimeDelay);
    }

    chip->eCurrentDemodMode = eDemodMode;

    r |= SetInternalFunc_Demod(chip->eCurrentDemodMode,chip);

    r |= IRx_Initialize_Demod(chip);

    r |= SetTSMode_Demod(chip);
    r |= SetTSSerialPin_Demod(chip->eTSSerialPin, chip);
    r |= SetTSSerialOrder_Demod(chip->eTSSerialOrder, chip);

    r |= SetTSSerialSyncPulse_Demod(chip->eTSSerialSyncPulse, chip);
    r |= SetTSErrorBit_Demod(chip->eTSErrorBit, chip);
    r |= SetTSErrorPola_Demod(chip->eTSErrorPola, chip);
    r |= SetTSValidPola_Demod(chip->eTSValidPola, chip);
    r |= SetTSPacketLen_Demod(chip->eTSPacketLen, chip);
    r |= SetTSParallelOrder_Demod(chip->eTSParallelOrder, chip);
    r |= SetTSParallelPhase_Demod(chip->eTSParallelPhase, chip);
    r |= SetGPIOStatus_Demod(chip);

    if(chip->ucDisableTSOutput == 0)
    {
        r |= EnableTSOutput_Demod(chip);
    }

    r |= TunerI2C_Initialize_Demod(chip);

    r |= InitErrorStat_Demod(chip);

    return r;
}

avl_error_code_t AVL_Demod_Sleep(AVL_ChipInternal *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    r |= IBase_SendRxOPWait_Demod(AVL_FW_CMD_HALT, chip);

    r |= IBase_SetSleepClock_Demod(chip);

    r |= IBase_SendRxOPWait_Demod(AVL_FW_CMD_SLEEP, chip);

    if(AVL_EC_OK == r)
    {
        chip->ucSleepFlag = 1;
    }

    r |= TunerI2C_Initialize_Demod(chip);

    r |= SetGPIOStatus_Demod(chip);


    return r;

}

avl_error_code_t AVL_Demod_Wakeup(AVL_ChipInternal *chip)
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

    r |= avl_bms_write32(chip->usI2CAddr, 
        stBaseAddrSet.hw_mcu_reset_base, 1);
    r |= avl_bms_write32(chip->usI2CAddr, 
        stBaseAddrSet.hw_mcu_system_reset_base, 1);

    r |= SetPLL_Demod(chip);

    r |= avl_bms_write32(chip->usI2CAddr,
        stBaseAddrSet.fw_status_reg_base + rs_core_ready_word_iaddr_offset, 0x00000000);

    r |= avl_bms_write32(chip->usI2CAddr, 
        stBaseAddrSet.hw_mcu_system_reset_base, 0);
    r |= avl_bms_write32(chip->usI2CAddr, 
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
    r |= SetTSSerialPin_Demod(chip->eTSSerialPin, chip);
    r |= SetTSSerialOrder_Demod(chip->eTSSerialOrder, chip);
    r |= SetTSSerialSyncPulse_Demod(chip->eTSSerialSyncPulse, chip);
    r |= SetTSErrorBit_Demod(chip->eTSErrorBit, chip);
    r |= SetTSErrorPola_Demod(chip->eTSErrorPola, chip);
    r |= SetTSValidPola_Demod(chip->eTSValidPola, chip);
    r |= SetTSPacketLen_Demod(chip->eTSPacketLen, chip);
    r |= SetTSParallelOrder_Demod(chip->eTSParallelOrder, chip);
    r |= SetTSParallelPhase_Demod(chip->eTSParallelPhase, chip);

    if(chip->ucDisableTSOutput == 0)
    {
        r |= EnableTSOutput_Demod(chip);
    }

    r |= TunerI2C_Initialize_Demod(chip);

    r |= InitErrorStat_Demod(chip);

    r |= SetGPIOStatus_Demod(chip);


    r |= GetMode_Demod(&chip->eCurrentDemodMode, chip);
    if(chip->eCurrentDemodMode == AVL_ISDBT)
    {
        r |= avl_bms_write32(chip->usI2CAddr,
            stBaseAddrSet.hw_emerald_io_base + emerald_io_pad_CS_0_sel_offset , 0x1);
    }


    if(AVL_EC_OK == r)
    {
        chip->ucSleepFlag = 0;
    }

    return r;

}

avl_error_code_t AVL_Demod_I2CBypassOn(AVL_ChipInternal *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint8_t   ucNum = 0;

    for(ucNum = 0; ucNum < 3; ucNum++)
    {
        r = avl_bms_write32(chip->usI2CAddr,
            stBaseAddrSet.hw_tuner_i2c_base + tuner_i2c_bit_rpt_cntrl_offset, 0x07);
    }
    return (r);
}

avl_error_code_t AVL_Demod_I2CBypassOff(AVL_ChipInternal *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint8_t   ucNum = 0;

    for(ucNum = 0; ucNum < 3; ucNum++)
    {
        r = avl_bms_write32(chip->usI2CAddr,
            stBaseAddrSet.hw_tuner_i2c_base + tuner_i2c_bit_rpt_cntrl_offset, 0x06);
    }
    return (r);
}

avl_error_code_t AVL_Demod_TsOn(AVL_ChipInternal *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    chip->ucDisableTSOutput = 0;

    r = avl_bms_write32(chip->usI2CAddr, 
        stBaseAddrSet.hw_TS_tri_state_cntrl_base, 0);

    return r;
}

avl_error_code_t AVL_Demod_TsOff(AVL_ChipInternal *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    chip->ucDisableTSOutput = 1;

    r = avl_bms_write32(chip->usI2CAddr, 
        stBaseAddrSet.hw_TS_tri_state_cntrl_base, 0xfff);

    return r;
}

avl_error_code_t AVL_Demod_GetVersion(AVL_DemodVersion *pstDemodVersion, AVL_ChipInternal *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint32_t uiTemp = 0;
    uint8_t ucBuff[4] = {0};

    r =  avl_bms_read32(chip->usI2CAddr, 0x40000, &uiTemp);
    if( AVL_EC_OK == r )
    {
        pstDemodVersion->uiChip = uiTemp;
    }

    r |=  avl_bms_read32(chip->usI2CAddr, 
        stBaseAddrSet.fw_status_reg_base + rs_patch_ver_iaddr_offset, &uiTemp);
    if( AVL_EC_OK == r )
    {
        Chunk32_Demod(uiTemp, ucBuff);
        pstDemodVersion->stPatch.ucMajor = ucBuff[0];
        pstDemodVersion->stPatch.ucMinor = ucBuff[1];
        pstDemodVersion->stPatch.usBuild = ucBuff[2];
        pstDemodVersion->stPatch.usBuild = ((uint16_t)((pstDemodVersion->stPatch.usBuild)<<8)) + ucBuff[3];

        pstDemodVersion->stAPI.ucMajor = AVL_API_VER_MAJOR;
        pstDemodVersion->stAPI.ucMinor = AVL_API_VER_MINOR;
        pstDemodVersion->stAPI.usBuild = AVL_API_VER_BUILD;
    }

    return r;
}

avl_error_code_t AVL_Demod_SetGPIO(AVL_GPIOPinNumber ePinNumber, AVL_GPIOPinValue ePinValue, AVL_ChipInternal *chip)
{
    avl_error_code_t r = AVL_EC_OK;

    switch(ePinNumber)
    {
    case AVL_Pin15:
        switch(ePinValue)
        {
        case AVL_LOGIC_0:
            r = avl_bms_write32(chip->usI2CAddr,
                stBaseAddrSet.hw_gpio_modu_002_base + gpio_module_002_gpio_config_offset, 0x2);
        break;
        case AVL_LOGIC_1:
            r = avl_bms_write32(chip->usI2CAddr,
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
            r = avl_bms_write32(chip->usI2CAddr,
                stBaseAddrSet.hw_gpio_debug_base + lnb_cntrl_0_sel_offset, ePinValue);
            chip->ucPin37Voltage = 0;
            break;
        case AVL_LOGIC_1:
            r = avl_bms_write32(chip->usI2CAddr,
                stBaseAddrSet.hw_gpio_debug_base + lnb_cntrl_0_sel_offset, ePinValue);
            chip->ucPin37Voltage = 1;
            break;
        case AVL_HIGH_Z:
            r = avl_bms_write32(chip->usI2CAddr,
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
            r = avl_bms_write32(chip->usI2CAddr,
                stBaseAddrSet.hw_gpio_debug_base + lnb_cntrl_1_sel_offset, ePinValue);
            chip->ucPin38Voltage = 0;
            break;
        case AVL_LOGIC_1:
            r = avl_bms_write32(chip->usI2CAddr,
                stBaseAddrSet.hw_gpio_debug_base + lnb_cntrl_1_sel_offset, ePinValue);
            chip->ucPin38Voltage = 1;
            break;
        case AVL_HIGH_Z:
            r = avl_bms_write32(chip->usI2CAddr,
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

avl_error_code_t AVL_Demod_GetGPIOValue(AVL_GPIOPinNumber ePinNumber, AVL_GPIOPinValue *pePinValue, AVL_ChipInternal *chip)
{
    avl_error_code_t r = AVL_EC_OK;
    uint32_t uiTemp = 0;

    switch(ePinNumber)
    {
    case AVL_Pin15:
        r = avl_bms_read32(chip->usI2CAddr,
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
        r = avl_bms_write32(chip->usI2CAddr,
            stBaseAddrSet.hw_gpio_debug_base + lnb_cntrl_0_sel_offset, AVL_HIGH_Z);

        r = avl_bms_read32(chip->usI2CAddr,
            stBaseAddrSet.hw_gpio_debug_base + lnb_cntrl_0_i_offset, (uint32_t *)pePinValue);
        break;

    case AVL_Pin38:
        r = avl_bms_write32(chip->usI2CAddr,
            stBaseAddrSet.hw_gpio_debug_base + lnb_cntrl_1_sel_offset, AVL_HIGH_Z);

        r = avl_bms_read32(chip->usI2CAddr,
            stBaseAddrSet.hw_gpio_debug_base + lnb_cntrl_1_i_offset, (uint32_t *)pePinValue);
        break;
    default:
        break;
    }

    return r;
}


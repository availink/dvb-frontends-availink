// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Availink AVL68x2 DVB-S/S2/T/T2/C, ISDB-T, J83.B demodulator driver
 *
 * Copyright (C) 2020 Availink, Inc. (gpl@availink.com)
 *
 */

#include <linux/slab.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/bitrev.h>
#include <linux/amlogic/aml_gpio_consumer.h>

#include <media/dvb_frontend.h>

#include "avl68x2.h"
#include "avl_tuner.h"

#include "AVL_Demod.h"
#include "avl_tuner.h"
#include "AVL_Demod_DVBSx.h"
#include "AVL_Demod_DVBTx.h"
#include "AVL_Demod_DVBC.h"
#include "AVL_Demod_ISDBT.h"

#define dbg_avl(fmt, args...)                                           \
	do                                                              \
	{                                                               \
		if (debug)                                              \
			printk("AVL: %s: " fmt "\n", __func__, ##args); \
	} while (0);

static int debug = 0;

module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "\n\t\t Enable debug information");


const AVL_DVBTxConfig default_dvbtx_config =
{
    .eDVBTxInputPath = AVL_IF_I,
    .uiDVBTxIFFreqHz = 5*1000*1000,
    .eDVBTxAGCPola = AVL_AGC_NORMAL//AVL_AGC_INVERTED
};

const AVL_DVBSxConfig default_dvbsx_config =
{
    .eDVBSxAGCPola = AVL_AGC_INVERTED,
    .e22KWaveForm = AVL_DWM_Normal
};

const AVL_ISDBTConfig default_isdbt_config =
{
    .eISDBTInputPath = AVL_IF_I,
    .eISDBTBandwidth = AVL_ISDBT_BW_6M,
    .uiISDBTIFFreqHz = 5*1000*1000,
    .eISDBTAGCPola = AVL_AGC_NORMAL
};

const AVL_DVBCConfig default_dvbc_config =
{
    .eDVBCInputPath = AVL_IF_I,
    .uiDVBCIFFreqHz = 5*1000*1000,
    .uiDVBCSymbolRateSps = 6875*1000,
    .eDVBCAGCPola = AVL_AGC_NORMAL,
    .eDVBCStandard = AVL_DVBC_J83A
};

struct avl_tuner default_avl_tuner = {
    .blindscan_mode = 0,
    .more_params = NULL,
    .initialize = NULL,
    .lock = NULL,
    .get_lock_status = NULL,
    .get_rf_strength = NULL,
    .get_max_lpf = NULL,
    .get_min_lpf = NULL,
    .get_lpf_step_size = NULL,
    .get_agc_slope = NULL,
    .get_min_gain_voltage = NULL,
    .get_max_gain_voltage = NULL,
    .get_rf_freq_step_size = NULL};

static int diseqc_set_voltage(
    struct dvb_frontend *fe,
    enum fe_sec_voltage voltage)
{
	struct avl68x2_priv *priv = fe->demodulator_priv;
	AVL_GPIOPinValue pwr, vol;
	avl_error_code_t ret;

	dbg_avl("volt: %d", voltage);

	switch (voltage)
	{
	case SEC_VOLTAGE_OFF:
		pwr = AVL_LOGIC_0;
		vol = AVL_LOGIC_0;
		break;
	case SEC_VOLTAGE_13:
		//power on
		pwr = AVL_LOGIC_1;
		vol = AVL_LOGIC_0;
		break;
	case SEC_VOLTAGE_18:
		//power on
		pwr = AVL_LOGIC_1;
		vol = AVL_HIGH_Z;
		break;
	default:
		return -EINVAL;
	}
	ret = AVL_Demod_SetGPIO(AVL_Pin37, pwr, priv->chip);
	ret |= AVL_Demod_SetGPIO(AVL_Pin38, vol, priv->chip);
	return ret;
}

static int avl68x2_init_diseqc(struct dvb_frontend *fe)
{
  struct avl68x2_priv *priv = fe->demodulator_priv;
  struct AVL_Diseqc_Para Diseqc_para;
  avl_error_code_t r = AVL_EC_OK;

  Diseqc_para.uiToneFrequencyKHz = 22;
  Diseqc_para.eTXGap = AVL_DTXG_15ms;
  Diseqc_para.eTxWaveForm = AVL_DWM_Normal;
  Diseqc_para.eRxTimeout = AVL_DRT_150ms;
  Diseqc_para.eRxWaveForm = AVL_DWM_Normal;

  r |= DVBSx_Diseqc_Initialize_Demod(&Diseqc_para, priv->chip);
  if (AVL_EC_OK != r)
  {
    dbg_avl("Diseqc Init failed !\n");
  }
  else
  {
    priv->chip->chip_pub->dvbsx_para.eDiseqcStatus = AVL_DOS_Initialized;
  }

  diseqc_set_voltage(fe, SEC_VOLTAGE_OFF);

  return r;
}

static int avl68x2_i2c_gate_ctrl(struct dvb_frontend *fe, int enable)
{
  struct avl68x2_priv *priv = fe->demodulator_priv;
  avl_error_code_t ret = AVL_EC_OK;

  dbg_avl("%s: %d\n", __func__, enable);

  if (enable)
  {
    ret =  AVL_Demod_I2CBypassOn(priv->chip);
  }
  else
  {
    ret = AVL_Demod_I2CBypassOff(priv->chip);
  }
  return ret;
}

static int avl68x2_acquire_dvbsx(struct dvb_frontend *fe)
{
  struct avl68x2_priv *priv = fe->demodulator_priv;
  struct dtv_frontend_properties *c = &fe->dtv_property_cache;
  avl_error_code_t r = AVL_EC_OK;
  dbg_avl("ACQUIRE S/S2");
  dbg_avl("Freq:%d khz,sym:%d hz", c->frequency, c->symbol_rate);

  r =  AVL_Demod_DVBSxAutoLock(c->symbol_rate, priv->chip);

  return r;
}

static int avl68x2_acquire_dvbtx(struct dvb_frontend *fe)
{
  struct avl68x2_priv *priv = fe->demodulator_priv;
  struct dtv_frontend_properties *c = &fe->dtv_property_cache;
  avl_error_code_t r = AVL_EC_OK;
  AVL_DVBTxBandWidth bw;

  dbg_avl("ACQUIRE T/T2");

  if (c->bandwidth_hz <= 1700000)
  {
    bw = AVL_DVBTx_BW_1M7;
  }
  else if (c->bandwidth_hz <= 5000000)
  {
    bw = AVL_DVBTx_BW_5M;
  }
  else if (c->bandwidth_hz <= 6000000)
  {
    bw = AVL_DVBTx_BW_6M;
  }
  else if (c->bandwidth_hz <= 7000000)
  {
    bw = AVL_DVBTx_BW_7M;
  }
  else
  {
    bw = AVL_DVBTx_BW_8M;
  }

  r = AVL_Demod_DVBT2AutoLock(bw,
                              AVL_DVBT2_PROFILE_UNKNOWN,
                              c->stream_id,
                              priv->chip);

  return r;
}

static int avl68x2_acquire_dvbc(struct dvb_frontend *fe)
{
  avl_error_code_t r = AVL_EC_OK;
  struct avl68x2_priv *priv = fe->demodulator_priv;
  r = AVL_Demod_DVBCAutoLock(priv->chip);
  return r;
}

static int avl68x2_acquire_dvbc_b(struct dvb_frontend *fe)
{
  avl_error_code_t r = AVL_EC_OK;
  r = avl68x2_acquire_dvbc(fe);
  return r;
}

static int avl68x2_acquire_isdbt(struct dvb_frontend *fe)
{
  avl_error_code_t r = AVL_EC_OK;
  struct avl68x2_priv *priv = fe->demodulator_priv;
  r = AVL_Demod_ISDBTAutoLock(priv->chip);
  return r;
}

static int avl68x2_set_standard(struct dvb_frontend *fe)
{
  struct avl68x2_priv *priv = fe->demodulator_priv;
  int32_t r = AVL_EC_OK;
  struct AVL_DemodVersion ver_info;
  int fw_status;
  unsigned int fw_maj, fw_min, fw_build;
  char fw_path[256];
  AVL_DemodMode dmd_mode = AVL_DVBSX;

  r = GetMode_Demod(&dmd_mode, priv->chip);
  //r = 100; //HACK FIXME

  //check for (FW) equivalent modes
  switch (priv->delivery_system)
  {
  case SYS_DVBS:
  case SYS_DVBS2:
    if (dmd_mode == AVL_DVBSX)
      return 0;
    strncpy(fw_path,AVL68X2_DVBSX_FW,255);
    dmd_mode = AVL_DVBSX;
    break;
  case SYS_ISDBT:
    if(dmd_mode == AVL_ISDBT)
      return 0;
    strncpy(fw_path,AVL68X2_ISDBT_FW,255);
    dmd_mode = AVL_ISDBT;
    break;
  case SYS_DVBC_ANNEX_A: //"DVB-C"
  case SYS_DVBC_ANNEX_B: //J.83-B
    if(dmd_mode == AVL_DVBC) //J.83-B
      return 0;
    strncpy(fw_path,AVL68X2_DVBC_FW,255);
    dmd_mode = AVL_DVBC;
    break;
  case SYS_DVBT:
  case SYS_DVBT2:
  default:
    if(dmd_mode == AVL_DVBTX)
      return 0;
    strncpy(fw_path,AVL68X2_DVBTX_FW,255);
    dmd_mode = AVL_DVBTX;
    break;
    break;
  }

  dbg_avl("initing demod for delsys=%d", priv->delivery_system);

  fw_status = request_firmware(&priv->fw, fw_path, priv->i2c->dev.parent);
  if (fw_status != 0)
  {
    dev_err(&priv->i2c->dev,
            KBUILD_MODNAME ": firmware file not found");
    return fw_status;
  }
  else
  {
    priv->chip->chip_priv->patch_data = (unsigned char *)(priv->fw->data);
    fw_maj = priv->chip->chip_priv->patch_data[24]; //major rev
    fw_min = priv->chip->chip_priv->patch_data[25]; //SDK-FW API rev
    fw_build = (priv->chip->chip_priv->patch_data[26] << 8) |
               priv->chip->chip_priv->patch_data[27]; //internal rev
    if (fw_min != AVL68X2_SDK_VER_MINOR)
    {
      //SDK-FW API rev must match
      dev_err(&priv->i2c->dev,
              KBUILD_MODNAME ": Firmware version %d.%d.%d incompatible with this driver version",
              fw_maj, fw_min, fw_build);
      dev_err(&priv->i2c->dev,
              KBUILD_MODNAME ": Firmware minor version must be %d",
              AVL68X2_SDK_VER_MINOR);
      r = 1;
      goto err;
    }
    else
    {
      dev_info(&priv->i2c->dev,
               KBUILD_MODNAME ": Firmware version %d.%d.%d found",
               fw_maj, fw_min, fw_build);
    }
  }

  //Reset Demod
  r = avl_bsp_reset();
  if (AVL_EC_OK != r)
  {
    dbg_avl("Failed to Resed demod via BSP!\n");
    goto err;
  }

  // boot the firmware here
  r |= AVL_Demod_Initialize(dmd_mode, priv->chip);
  if (AVL_EC_OK != r)
  {
    dbg_avl("AVL_Demod_Initialize failed !\n");
    goto err;
  }

  r |= AVL_Demod_GetVersion(&ver_info, priv->chip);
  if (AVL_EC_OK != r)
  {
    dbg_avl("AVL_Demod_GetVersion failed\n");
    goto err;
  }
  dbg_avl("FW version %d.%d.%d\n", ver_info.firmware.major, ver_info.firmware.minor, ver_info.firmware.build);
  dbg_avl("API version %d.%d.%d\n", ver_info.sdk.major, ver_info.sdk.minor, ver_info.sdk.build);

  switch (priv->delivery_system)
  {
  case SYS_DVBS:
  case SYS_DVBS2:
  default:
    r |= avl68x2_init_diseqc(fe);
    break;
  }

  r |= InitErrorStat_Demod(priv->chip);

  if (r)
  {
    dev_err(&priv->i2c->dev, "%s: demod init failed",
            KBUILD_MODNAME);
  }

err:
  release_firmware(priv->fw);

  return r;
}

avl_error_code_t AVL_SX_DiseqcSendCmd(
	struct avl68x2_priv *priv,
	uint8_t * pCmd,
	uint8_t CmdSize)
{
  avl_error_code_t r = AVL_EC_OK;
  struct AVL_Diseqc_TxStatus TxStatus;
  dbg_avl(" %*ph", CmdSize, pCmd);

  r = AVL_Demod_DVBSx_Diseqc_SendModulationData(pCmd, CmdSize, priv->chip);
  if (r != AVL_EC_OK)
  {
    printk("AVL_SX_DiseqcSendCmd failed !\n");
  }
  else
  {
    do
    {
      msleep(5);
      r |= AVL_Demod_DVBSx_Diseqc_GetTxStatus(&TxStatus, priv->chip);
    } while (TxStatus.m_TxDone != 1);
    if (r == AVL_EC_OK)
    {
    }
    else
    {
      printk("AVL_SX_DiseqcSendCmd Err. !\n");
    }
  }
  return (int)(r);
}

static int avl68x2_diseqc_cmd(struct dvb_frontend *fe,
                          struct dvb_diseqc_master_cmd *cmd)
{
  struct avl68x2_priv *priv = fe->demodulator_priv;

  return AVL_SX_DiseqcSendCmd(priv, cmd->msg, cmd->msg_len);
}

static int avl68x2_burst(struct dvb_frontend *fe, enum fe_sec_mini_cmd burst)
{
	struct avl68x2_priv *priv = fe->demodulator_priv;

	return AVL_Demod_DVBSx_Diseqc_SendTone(
	    burst == SEC_MINI_A ? 1 : 0,
	    1,
	    priv->chip);
}

static int diseqc_set_tone(struct dvb_frontend *fe, enum fe_sec_tone_mode tone)
{
	struct avl68x2_priv *priv = fe->demodulator_priv;
	struct avl68x2_chip_pub *chip_pub = priv->chip->chip_pub;
	avl_error_code_t r = AVL_EC_OK;

	dbg_avl("tone: %d", tone);
	switch (tone)
	{
	case SEC_TONE_ON:
		if (chip_pub->dvbsx_para.eDiseqcStatus !=
		AVL_DOS_InContinuous)
		{
			r = AVL_Demod_DVBSx_Diseqc_StartContinuous(priv->chip);
		}
		break;
	case SEC_TONE_OFF:
		if (chip_pub->dvbsx_para.eDiseqcStatus ==
		AVL_DOS_InContinuous)
		{
			r = AVL_Demod_DVBSx_Diseqc_StopContinuous(priv->chip);
		}
		break;
	default:
		return -EINVAL;
	}
	return r;
}

static int update_fe_props_sx(struct dvb_frontend *fe,
			   struct dtv_frontend_properties *props)
{
	struct avl68x2_priv *priv = fe->demodulator_priv;
	int ret = 0;
	int32_t cfo = 0;
	struct AVL_DVBSxModulationInfo modinfo;

	ret |= AVL_Demod_DVBSx_GetFreqOffset(&cfo, priv->chip);
	props->frequency += cfo;

	ret |= AVL_Demod_DVBSxGetModulationInfo(&modinfo, priv->chip);
	switch (modinfo.eDVBSxModulationMode)
	{
	case AVL_DVBSx_QPSK:
		props->modulation = QPSK;
		break;
	case AVL_DVBSx_8PSK:
		props->modulation = PSK_8;
		break;
	case AVL_DVBSx_16APSK:
		props->modulation = APSK_16;
		break;
	default:
		props->modulation = APSK_32;
		break;
	}
	
	props->inversion = INVERSION_AUTO; //FIXME

	if (modinfo.eDVBSxPilot == AVL_DVBSx_Pilot_ON)
	{
		props->pilot = PILOT_ON;
	}
	else
	{
		props->pilot = PILOT_OFF;
	}

	switch (modinfo.eDVBSxRollOff)
	{
	case AVL_DVBSx_RollOff_35:
		props->rolloff = ROLLOFF_35;
		break;
	case AVL_DVBSx_RollOff_25:
		props->rolloff = ROLLOFF_25;
		break;
	case AVL_DVBSx_RollOff_20:
		props->rolloff = ROLLOFF_20;
		break;
	default:
		props->rolloff = ROLLOFF_20;
	}

	if (modinfo.eDVBSxStandard == AVL_DVBS2)
	{
		props->delivery_system = SYS_DVBS2;
		switch (modinfo.eDVBS2CodeRate)
		{
		case AVL_DVBS2_CR_2_5:
			props->fec_inner = FEC_2_5;
			break;
		case AVL_DVBS2_CR_1_2:
			props->fec_inner = FEC_1_2;
			break;
		case AVL_DVBS2_CR_3_5:
			props->fec_inner = FEC_3_5;
			break;
		case AVL_DVBS2_CR_2_3:
			props->fec_inner = FEC_2_3;
			break;
		case AVL_DVBS2_CR_3_4:
			props->fec_inner = FEC_3_4;
			break;
		case AVL_DVBS2_CR_4_5:
			props->fec_inner = FEC_4_5;
			break;
		case AVL_DVBS2_CR_5_6:
			props->fec_inner = FEC_5_6;
			break;
		case AVL_DVBS2_CR_8_9:
			props->fec_inner = FEC_8_9;
			break;
		case AVL_DVBS2_CR_9_10:
			props->fec_inner = FEC_9_10;
			break;
		default: //DVBv5 missing many rates
			props->fec_inner = FEC_AUTO;
		}
	}
	else
	{
		props->delivery_system = SYS_DVBS;
		switch (modinfo.eDVBSCodeRate)
		{
		case AVL_DVBS_CR_1_2:
			props->fec_inner = FEC_1_2;
			break;
		case AVL_DVBS_CR_2_3:
			props->fec_inner = FEC_2_3;
			break;
		case AVL_DVBS_CR_3_4:
			props->fec_inner = FEC_3_4;
			break;
		case AVL_DVBS_CR_5_6:
			props->fec_inner = FEC_5_6;
			break;
		case AVL_DVBS_CR_6_7:
			props->fec_inner = FEC_6_7;
			break;
		default:
			props->fec_inner = FEC_7_8;
		}
	}
	
	return ret;
}

static int update_fe_props_tx(struct dvb_frontend *fe,
			   struct dtv_frontend_properties *props)
{
	struct avl68x2_priv *priv = fe->demodulator_priv;
	int ret = 0;
	struct AVL_DVBTxModulationInfo modinfo;

	ret |= AVL_Demod_DVBTxGetModulationInfo(&modinfo, priv->chip);

	props->inversion = INVERSION_AUTO; //FIXME

	props->delivery_system =
	    modinfo.ucDVBxStandard == AVL_DVBTx_Standard_T2 ? SYS_DVBT2 : SYS_DVBT;

	if(modinfo.ucDVBxStandard == AVL_DVBTx_Standard_T2)
	{
		props->stream_id = modinfo.eDVBT2SignalInfo.ucDVBT2DataPLPID;

		switch(modinfo.eDVBT2SignalInfo.eDVBT2FFTSize)
		{
			case AVL_DVBT2_FFT_1K:
			props->transmission_mode = TRANSMISSION_MODE_1K;
			break;
			case AVL_DVBT2_FFT_2K:
			props->transmission_mode = TRANSMISSION_MODE_2K;
			break;
			case AVL_DVBT2_FFT_4K:
			props->transmission_mode = TRANSMISSION_MODE_4K;
			break;
			case AVL_DVBT2_FFT_8K:
			props->transmission_mode = TRANSMISSION_MODE_8K;
			break;
			case AVL_DVBT2_FFT_16K:
			props->transmission_mode = TRANSMISSION_MODE_16K;
			break;
			default:
			props->transmission_mode = TRANSMISSION_MODE_32K;
			break;
		}

		switch (modinfo.eDVBT2SignalInfo.eDVBT2DataPLPCodeRate)
		{
		case AVL_DVBT2_CR_2_5:
			props->fec_inner = FEC_2_5;
			break;
		case AVL_DVBT2_CR_1_2:
			props->fec_inner = FEC_1_2;
			break;
		case AVL_DVBT2_CR_3_5:
			props->fec_inner = FEC_3_5;
			break;
		case AVL_DVBT2_CR_2_3:
			props->fec_inner = FEC_2_3;
			break;
		case AVL_DVBT2_CR_3_4:
			props->fec_inner = FEC_3_4;
			break;
		case AVL_DVBT2_CR_4_5:
			props->fec_inner = FEC_4_5;
			break;
		default:
			props->fec_inner = FEC_5_6;
			break;
		}

		switch (modinfo.eDVBT2SignalInfo.eDVBT2DataPLPModulationMode)
		{
		case AVL_DVBT2_QPSK:
			props->modulation = QPSK;
			break;
		case AVL_DVBT2_16QAM:
			props->modulation = QAM_16;
			break;
		case AVL_DVBT2_64QAM:
			props->modulation = QAM_64;
			break;
		default:
			props->modulation = QAM_256;
			break;
		}

		switch (modinfo.eDVBT2SignalInfo.eDVBT2GuardInterval)
		{
		case AVL_DVBT2_GI_1_32:
			props->guard_interval = GUARD_INTERVAL_1_32;
			break;
		case AVL_DVBT2_GI_1_16:
			props->guard_interval = GUARD_INTERVAL_1_16;
			break;
		case AVL_DVBT2_GI_1_8:
			props->guard_interval = GUARD_INTERVAL_1_8;
			break;
		case AVL_DVBT2_GI_1_4:
			props->guard_interval = GUARD_INTERVAL_1_4;
			break;
		case AVL_DVBT2_GI_1_128:
			props->guard_interval = GUARD_INTERVAL_1_128;
			break;
		case AVL_DVBT2_GI_19_128:
			props->guard_interval = GUARD_INTERVAL_19_128;
			break;
		default:
			props->guard_interval = GUARD_INTERVAL_19_256;
			break;
		}
	}
	else 
	{

	}

	return ret;
}

static int update_fe_props_isdbt(struct dvb_frontend *fe,
				 struct dtv_frontend_properties *props)
{
	struct avl68x2_priv *priv = fe->demodulator_priv;
	int ret = 0;
	struct AVL_ISDBTModulationInfo modinfo;
	struct AVL_ISDBT_LayerSignalInfo *layer_info;
	uint8_t l;

	ret |= AVL_Demod_ISDBTGetModulationInfo(&modinfo, priv->chip);

	props->inversion = INVERSION_AUTO; //FIXME

	//FIXME: this is all a bit...wonky
	switch (modinfo.eISDBTSystemType)
	{
	case AVL_ISDBT_Std:
		props->isdbt_sb_mode = 0;
		break;
	case AVL_ISDBTsb_1seg:
		props->isdbt_sb_mode = 1;
		break;
	case AVL_ISDBTsb_3seg:
		props->isdbt_sb_mode = 1;
		break;
	}

	switch (modinfo.eISDBTMode)
	{
	case AVL_ISDBT_Mode1:
		props->transmission_mode = TRANSMISSION_MODE_2K;
		break;
	case AVL_ISDBT_Mode2:
		props->transmission_mode = TRANSMISSION_MODE_4K;
		break;
	case AVL_ISDBT_Mode3:
		props->transmission_mode = TRANSMISSION_MODE_8K;
		break;
	}

	switch (modinfo.eISDBTGuardInterval)
	{
	case AVL_ISDBT_GUARD_1_32:
		props->guard_interval = GUARD_INTERVAL_1_32;
		break;
	case AVL_ISDBT_GUARD_1_16:
		props->guard_interval = GUARD_INTERVAL_1_16;
		break;
	case AVL_ISDBT_GUARD_1_8:
		props->guard_interval = GUARD_INTERVAL_1_8;
		break;
	case AVL_ISDBT_GUARD_1_4:
		break;
	}

	props->isdbt_partial_reception = modinfo.eISDBTPartialReception;

	for (l = 0; l < 3; l++)
	{
		switch (l)
		{
		case 0:
			layer_info = &modinfo.eISDBTLayerA;
			break;
		case 1:
			layer_info = &modinfo.eISDBTLayerB;
			break;
		case 2:
			layer_info = &modinfo.eISDBTLayerC;
			break;
		}

		props->layer[l].segment_count = layer_info->ucISDBTSegmentNum;
		switch (layer_info->eISDBTCodeRate)
		{
		case AVL_ISDBT_CR_1_2:
			props->layer[l].fec = FEC_1_2;
			break;
		case AVL_ISDBT_CR_2_3:
			props->layer[l].fec = FEC_2_3;
			break;
		case AVL_ISDBT_CR_3_4:
			props->layer[l].fec = FEC_3_4;
			break;
		case AVL_ISDBT_CR_5_6:
			props->layer[l].fec = FEC_5_6;
			break;
		case AVL_ISDBT_CR_7_8:
			props->layer[l].fec = FEC_7_8;
			break;
		}
		switch (layer_info->eISDBTModulationMode)
		{
		case AVL_ISDBT_DQPSK:
			props->layer[l].modulation = DQPSK;
			break;
		case AVL_ISDBT_QPSK:
			props->layer[l].modulation = QPSK;
			break;
		case AVL_ISDBT_16QAM:
			props->layer[l].modulation = QAM_16;
			break;
		case AVL_ISDBT_64QAM:
			props->layer[l].modulation = QAM_64;
			break;
		}
		props->layer[l].interleaving = layer_info->ucISDBTInterleaverLen;
	}

	return ret;
}

static int update_fe_props_c(struct dvb_frontend *fe,
			   struct dtv_frontend_properties *props)
{
	struct avl68x2_priv *priv = fe->demodulator_priv;
	int ret = 0;
	struct AVL_DVBCModulationInfo modinfo;

	ret |= AVL_Demod_DVBCGetModulationInfo(&modinfo, priv->chip);

	switch(modinfo.eQAMMode)
	{
		case AVL_DVBC_16QAM:
		props->modulation = QAM_16;
		break;
		case AVL_DVBC_32QAM:
		props->modulation = QAM_32;
		break;
		case AVL_DVBC_64QAM:
		props->modulation = QAM_64;
		break;
		case AVL_DVBC_128QAM:
		props->modulation = QAM_128;
		break;
		default:
		props->modulation = QAM_256;
		break;
	}

	return ret;
}

static int get_frontend(struct dvb_frontend *fe,
			struct dtv_frontend_properties *props)
{
	struct avl68x2_priv *priv = fe->demodulator_priv;
	int ret = 0;
	int32_t SNR_x100db = 0;
	uint8_t lock = 0;
	uint16_t ssi = 0;


	props->cnr.len = 1;
	props->cnr.stat[0].scale = FE_SCALE_NOT_AVAILABLE;

	props->strength.len = 1;
	props->strength.stat[1].scale = FE_SCALE_NOT_AVAILABLE;

	props->block_error.len = 1;
	props->block_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;

	props->block_count.len = 1;
	props->block_count.stat[0].scale = FE_SCALE_NOT_AVAILABLE;

	ret = AVL_Demod_GetLockStatus(&lock, priv->chip);

	if(lock)
	{
		ret |= AVL_Demod_GetSSI(&ssi, priv->chip);

		props->strength.len = 2;

		props->strength.stat[1].scale = FE_SCALE_RELATIVE;
		props->strength.stat[1].uvalue = (ssi * 65535) / 100;

		props->strength.stat[0].scale = FE_SCALE_DECIBEL;
		props->strength.stat[0].svalue = -80 + ssi / 2;

		ret |= AVL_Demod_GetSNR (&SNR_x100db, priv->chip);
		props->cnr.len = 2;
		props->cnr.stat[0].scale = FE_SCALE_DECIBEL; //0.001dB
		props->cnr.stat[0].svalue = SNR_x100db * 10;
		props->cnr.stat[1].scale = FE_SCALE_RELATIVE;
		props->cnr.stat[1].uvalue = ((SNR_x100db + 300) / 10) * 250;
		if (props->cnr.stat[1].uvalue > 0xffff)
			props->cnr.stat[1].uvalue = 0xffff;
		

		//DVB-S pre/post viterbi
		props->pre_bit_error.len = 0;
		props->pre_bit_count.len = 0;
		props->post_bit_error.len = 0;
		props->post_bit_count.len = 0;
		
		//TODO: post outer FEC block errors
		props->block_error.len = 1;
		props->block_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
		props->block_error.stat[0].uvalue = 0;

		props->block_count.len = 1;
		props->block_count.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
		props->block_count.stat[0].uvalue = 0;

		switch (props->delivery_system)
		{
		case SYS_DVBS:
		case SYS_DVBS2:
			ret |= update_fe_props_sx(fe, props);
			break;
		case SYS_DVBT:
		case SYS_DVBT2:
			ret |= update_fe_props_tx(fe, props);
			break;
		case SYS_DVBC_ANNEX_A:
		case SYS_DVBC_ANNEX_B:
			ret |= update_fe_props_c(fe, props);
			break;
		default:
			ret |= update_fe_props_isdbt(fe, props);
		}
	}
	return ret;
}

static int avl68x2_read_status(struct dvb_frontend *fe, enum fe_status *status)
{
	struct avl68x2_priv *priv = fe->demodulator_priv;
	int ret = 0;
	uint8_t lock = 0;
	int32_t SNR_x100db = 0;
	int32_t ber = 0;
	int lock_led = priv->chip->chip_pub->gpio_lock_led;

	ret = AVL_Demod_GetLockStatus(&lock, priv->chip);
	if (!ret && lock == AVL_STATUS_LOCK)
	{
		*status = FE_HAS_SIGNAL | FE_HAS_CARRIER |
			  FE_HAS_VITERBI | FE_HAS_SYNC | FE_HAS_LOCK;
		ret |= get_frontend(fe, &fe->dtv_property_cache);
		if (lock_led)
		{
			gpio_request(lock_led, KBUILD_MODNAME);
			gpio_direction_output(lock_led, 1);
		}
	}
	else
	{
		*status = FE_HAS_SIGNAL;
		if (lock_led)
		{
			gpio_request(lock_led, KBUILD_MODNAME);
			gpio_direction_output(lock_led, 0);
		}
	}
	
	if(debug > 1) {
	  ret |= AVL_Demod_GetSNR (&SNR_x100db, priv->chip);
	  ret = (int)AVL_Demod_GetPER(&ber, priv->chip);
	  printk("AVL: %s: read status %d, snr = %d, ber = %d\n",__func__,*status, SNR_x100db, ber);
	}
	return ret;
}

static int avl68x2_read_signal_strength(
	struct dvb_frontend *fe,
	uint16_t *strength)
{
  struct dtv_frontend_properties *c = &fe->dtv_property_cache;
  int i;

  *strength = 0;
  for (i = 0; i < c->strength.len; i++)
    if (c->strength.stat[i].scale == FE_SCALE_RELATIVE)
      *strength = (uint16_t)c->strength.stat[i].uvalue;

  return 0;
}

static int avl68x2_read_snr(struct dvb_frontend *fe, uint16_t *snr)
{
  struct dtv_frontend_properties *c = &fe->dtv_property_cache;
  int i;

  *snr = 0;
  for (i = 0; i < c->cnr.len; i++)
    if (c->cnr.stat[i].scale == FE_SCALE_RELATIVE)
      *snr = (uint16_t)c->cnr.stat[i].uvalue;

  return 0;
}

static int avl68x2_read_ber(struct dvb_frontend *fe, uint32_t *ber)
{
  struct avl68x2_priv *priv = fe->demodulator_priv;
  int ret;

  *ber = 10e7;
  ret = (int)AVL_Demod_GetPER(ber, priv->chip);
  if (!ret)
    *ber /= 100;

  return ret;
}

static int avl68x2fe_algo(struct dvb_frontend *fe)
{
  return DVBFE_ALGO_HW;
}

//static  struct dtv_frontend_properties _last_dtv;

static int set_frontend(struct dvb_frontend *fe)
{
  int ret;
  struct dtv_frontend_properties *c = &fe->dtv_property_cache;
  struct avl68x2_priv *priv = fe->demodulator_priv;
  int lock_led = priv->chip->chip_pub->gpio_lock_led;

  if (lock_led)
  {
	  gpio_request(lock_led, KBUILD_MODNAME);
	  gpio_direction_output(lock_led, 0);
  }

  ret = avl68x2_set_standard(fe);
  if(ret)
  {
    dev_err(&priv->i2c->dev, "%s: avl68x2_set_standard() failed!!!",
			KBUILD_MODNAME);
    return ret;
  }

  /* setup tuner */
  if (fe->ops.tuner_ops.set_params)
  {
    if (fe->ops.i2c_gate_ctrl)
      fe->ops.i2c_gate_ctrl(fe, 1);
    ret = fe->ops.tuner_ops.set_params(fe);
    if (fe->ops.i2c_gate_ctrl)
      fe->ops.i2c_gate_ctrl(fe, 0);
    if (ret)
      return ret;
  }

  if (c->delivery_system == SYS_DVBC_ANNEX_A && c->symbol_rate < 6000000)
  {
    c->delivery_system = SYS_DVBC_ANNEX_B;
    c->bandwidth_hz = 6000000;
  }

  switch (c->delivery_system)
  {
  case SYS_DVBT:
  case SYS_DVBT2:
    ret = avl68x2_acquire_dvbtx(fe);
    break;
  case SYS_DVBC_ANNEX_A:
    ret = avl68x2_acquire_dvbc(fe);
    break;
  case SYS_DVBC_ANNEX_B:
    ret = avl68x2_acquire_dvbc_b(fe);
    break;
  case SYS_DVBS:
  case SYS_DVBS2:
    ret = avl68x2_acquire_dvbsx(fe);
    break;
  case SYS_ISDBT:
    ret = avl68x2_acquire_isdbt(fe);
    break;
  default:
    ret = -EINVAL;
    break;
  }

  return ret;
}

static int avl68x2_tune(struct dvb_frontend *fe,
			bool re_tune,
			unsigned int mode_flags,
			unsigned int *delay,
			enum fe_status *status)
{
	*delay = HZ / 5;
	if (re_tune)
	{
		int ret = set_frontend(fe);
		if (ret)
			return ret;
	}
	return avl68x2_read_status(fe, status);
}

static int avl68x2_init(struct dvb_frontend *fe)
{
  return 0;
}

static int avl68x2_sleep(struct dvb_frontend *fe)
{
  return 0;
}

static void avl68x2_release(struct dvb_frontend *fe)
{
	struct avl68x2_priv *priv = fe->demodulator_priv;
	kfree(priv->chip->chip_pub);
	kfree(priv->chip->chip_priv);
	kfree(priv->chip->stStdSpecFunc);
	kfree(priv->chip);
	kfree(priv);
}

static struct dvb_frontend_ops avl68x2_ops = {
    .delsys = {SYS_DVBS, SYS_DVBS2,
	       SYS_DVBT, SYS_DVBT2,
	       SYS_DVBC_ANNEX_A, SYS_DVBC_ANNEX_B,
	       SYS_ISDBT},
    .info = {
	.name = "Availink avl68x2",
	.frequency_min_hz = 175 * MHz,
	.frequency_max_hz = 2150 * MHz,
	.frequency_stepsize_hz = 0,
	.frequency_tolerance_hz = 0,
	.symbol_rate_min = 1 * MHz,
	.symbol_rate_max = 55 * MHz,
	.caps =
	    FE_CAN_FEC_1_2 |
	    FE_CAN_FEC_2_3 |
	    FE_CAN_FEC_3_4 |
	    FE_CAN_FEC_4_5 |
	    FE_CAN_FEC_5_6 |
	    FE_CAN_FEC_6_7 |
	    FE_CAN_FEC_7_8 |
	    FE_CAN_FEC_AUTO |
	    FE_CAN_QPSK |
	    FE_CAN_QAM_16 |
	    FE_CAN_QAM_32 |
	    FE_CAN_QAM_64 |
	    FE_CAN_QAM_AUTO |
	    FE_CAN_TRANSMISSION_MODE_AUTO |
	    FE_CAN_MUTE_TS |
	    FE_CAN_2G_MODULATION |
	    FE_CAN_MULTISTREAM |
	    FE_CAN_INVERSION_AUTO |
	    FE_CAN_GUARD_INTERVAL_AUTO |
	    FE_CAN_HIERARCHY_AUTO |
	    FE_CAN_BANDWIDTH_AUTO |
	    FE_CAN_RECOVER},

    .release = avl68x2_release,
    .init = avl68x2_init,

    .sleep = avl68x2_sleep,
    .i2c_gate_ctrl = avl68x2_i2c_gate_ctrl,

    .read_status = avl68x2_read_status,
    .read_signal_strength = avl68x2_read_signal_strength,
    .read_snr = avl68x2_read_snr,
    .read_ber = avl68x2_read_ber,
    .set_tone = diseqc_set_tone,
    .set_voltage = diseqc_set_voltage,
    .diseqc_send_master_cmd = avl68x2_diseqc_cmd,
    .diseqc_send_burst = avl68x2_burst,
    .get_frontend_algo = avl68x2fe_algo,
    .tune = avl68x2_tune,

    .set_frontend = set_frontend,
    .get_frontend = get_frontend,
};

struct dvb_frontend *avl68x2_attach(struct avl68x2_config *config,
				    struct i2c_adapter *i2c)
{
	struct avl68x2_priv *priv;
	avl_error_code_t ret;
	uint32_t id;

	dbg_avl("start demod attach");

	priv = kzalloc(sizeof(struct avl68x2_priv), GFP_KERNEL);
	if (priv == NULL)
		goto err;

	dbg_avl("priv alloc'ed = %llx", (unsigned long long int)priv);

	memcpy(&priv->frontend.ops, &avl68x2_ops,
	       sizeof(struct dvb_frontend_ops));

	priv->frontend.demodulator_priv = priv;
	priv->i2c = i2c;
	priv->delivery_system = SYS_DVBT;

	priv->chip = kzalloc(sizeof(struct avl68x2_chip), GFP_KERNEL);
	if (priv->chip == NULL)
		goto err1;

	priv->chip->stStdSpecFunc = kzalloc(sizeof(struct AVL_StandardSpecificFunctions), GFP_KERNEL);
	if (priv->chip->stStdSpecFunc == NULL)
		goto err2;

	priv->chip->chip_priv = kzalloc(sizeof(struct avl68x2_chip_priv), GFP_KERNEL);
	if (priv->chip->chip_priv == NULL)
		goto err3;

	priv->chip->chip_pub = kzalloc(sizeof(struct avl68x2_chip_pub), GFP_KERNEL);
	if (priv->chip->chip_pub == NULL)
		goto err4;

	/* copy (ephemeral?) public part of chip config into alloc'd area */
	memcpy(priv->chip->chip_pub,
	       config->chip_pub,
	       sizeof(struct avl68x2_chip_pub));

	priv->chip->chip_pub->tuner = &default_avl_tuner;

	dbg_avl("Demod ID %d, I2C addr 0x%x",
		(priv->chip->chip_pub->i2c_addr >> AVL_DEMOD_ID_SHIFT) & AVL_DEMOD_ID_MASK,
		priv->chip->chip_pub->i2c_addr & 0xFF);

	// associate demod ID with i2c_adapter
	avl_bsp_assoc_i2c_adapter(priv->chip->chip_pub->i2c_addr, i2c);

	ret = avl68x2_init_chip_object(priv->chip);

	/* get chip id */
	ret |= GetFamilyID_Demod(&id, priv->chip);
	if (ret)
	{
		dev_err(&priv->i2c->dev, "%s: attach failed reading id",
			KBUILD_MODNAME);
		goto err5;
	}

	dbg_avl("chip_id= 0x%x\n", id);

	if (id != AVL68XX)
	{
		dev_err(&priv->i2c->dev, "%s: attach failed, id mismatch",
			KBUILD_MODNAME);
		goto err5;
	}

	dev_info(&priv->i2c->dev, "%s: found AVL68x2 id=0x%x",
		 KBUILD_MODNAME, id);

	if (!avl68x2_set_standard(&priv->frontend))
	{
		dev_info(&priv->i2c->dev,
			 KBUILD_MODNAME ": Firmware booted");
		return &priv->frontend;
	}

err5:
	kfree(priv->chip->chip_pub);
err4:
	kfree(priv->chip->chip_priv);
err3:
	kfree(priv->chip->stStdSpecFunc);
err2:
	kfree(priv->chip);
err1:
	kfree(priv);
err:
	return NULL;
}
EXPORT_SYMBOL_GPL(avl68x2_attach);
EXPORT_SYMBOL_GPL(default_dvbtx_config);
EXPORT_SYMBOL_GPL(default_dvbsx_config);
EXPORT_SYMBOL_GPL(default_isdbt_config);
EXPORT_SYMBOL_GPL(default_dvbc_config);

MODULE_DESCRIPTION("Availink AVL68x2 DVB-S/S2/T/T2/C, ISDB-T, J.83B demodulator driver");
MODULE_AUTHOR("Availink, Inc. (gpl@availink.com)");
MODULE_LICENSE("GPL");
MODULE_VERSION(AVL68X2_VERSION);

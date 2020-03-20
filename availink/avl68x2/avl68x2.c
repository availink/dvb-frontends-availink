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

#include <media/dvb_frontend.h>

#include "avl68x2.h"
#include "avl_tuner.h"

#include "AVL_Demod.h"
//#include "avl_tuner.h"
#include "AVL_Demod_DVBSx.h"

#define dbg_avl(fmt, args...)                                           \
	do                                                              \
	{                                                               \
		if (debug)                                              \
			printk("AVL: %s: " fmt "\n", __func__, ##args); \
	} while (0);

MODULE_PARM_DESC(debug, "\n\t\t Enable AVL demodulator debug information");
static int debug;
module_param(debug, int, 0644);

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



static int init_dvbs_s2(struct dvb_frontend *fe)
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

static int avl68x2_set_dvbs(struct dvb_frontend *fe)
{
  struct avl68x2_priv *priv = fe->demodulator_priv;
  struct dtv_frontend_properties *c = &fe->dtv_property_cache;
  avl_error_code_t r = AVL_EC_OK;
  dbg_avl("Freq:%d khz,sym:%d hz", c->frequency, c->symbol_rate);

  r =  AVL_Demod_DVBSxAutoLock(c->symbol_rate, priv->chip);

  return r;
}

static int set_dvb_mode(struct dvb_frontend *fe,
                               enum fe_delivery_system delsys)
{
  struct avl68x2_priv *priv = fe->demodulator_priv;
  avl_error_code_t r = AVL_EC_OK;
  struct AVL_DemodVersion ver_info;
  int fw_status;
  unsigned int fw_maj, fw_min, fw_build;
  char fw_path[256];
  AVL_DemodMode dmd_mode;

  /* already in desired mode */
  if (priv->delivery_system == delsys)
    return 0;

  dbg_avl("initing demod for delsys=%d", delsys);

  //check for (FW) equivalent modes
  switch (priv->delivery_system)
  {
  case SYS_DVBS:
    if (delsys == SYS_DVBS2)
      return 0;
    strncpy(fw_path,AVL68X2_DVBSX_FW,255);
    dmd_mode = AVL_DVBSX;
    break;
  case SYS_DVBS2:
    if (delsys == SYS_DVBS)
      return 0;
    strncpy(fw_path,AVL68X2_DVBSX_FW,255);
    dmd_mode = AVL_DVBSX;
    break;
  case SYS_DVBT:
    if(delsys == SYS_DVBT2)
      return 0;
    strncpy(fw_path,AVL68X2_DVBTX_FW,255);
    dmd_mode = AVL_DVBTX;
    break;
  case SYS_DVBT2:
    if(delsys == SYS_DVBT)
      return 0;
    strncpy(fw_path,AVL68X2_DVBTX_FW,255);
    dmd_mode = AVL_DVBTX;
    break;
  case SYS_ISDBT:
    strncpy(fw_path,AVL68X2_ISDBT_FW,255);
    dmd_mode = AVL_ISDBT;
    break;
  case SYS_DVBC_ANNEX_A: //"DVB-C"
    if(delsys == SYS_DVBC_ANNEX_B) //J.83-B
      return 0;
    strncpy(fw_path,AVL68X2_DVBC_FW,255);
    dmd_mode = AVL_DVBC;
    break;
  case SYS_DVBC_ANNEX_B:
    if(delsys == SYS_DVBC_ANNEX_A)
      return 0;
    strncpy(fw_path,AVL68X2_DVBC_FW,255);
    dmd_mode = AVL_DVBC;
    break;
  default:
    break;
  }
  priv->delivery_system = delsys;

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
    if (fw_min != AVL62X1_SDK_VER_MINOR)
    {
      //SDK-FW API rev must match
      dev_err(&priv->i2c->dev,
              KBUILD_MODNAME ": Firmware version %d.%d.%d incompatible with this driver version",
              fw_maj, fw_min, fw_build);
      dev_err(&priv->i2c->dev,
              KBUILD_MODNAME ": Firmware minor version must be %d",
              AVL62X1_SDK_VER_MINOR);
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
  r = AVL_IBSP_Reset();
  if (AVL_EC_OK != r)
  {
    dbg_avl("Failed to Resed demod via BSP!\n");
    goto err;
  }

  // boot the firmware here
  r |= AVL_Demod_Initialize(dmd_mode, priv->chip, 0);
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
    r |= init_dvbs_s2(fe);
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

avl_error_code_t AVL_SX_DiseqcSendCmd(struct avl68x2_priv *priv, uint8_t * pCmd, u8 CmdSize)
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

static int avl68x2_diseqc(struct dvb_frontend *fe,
                          struct dvb_diseqc_master_cmd *cmd)
{
  struct avl68x2_priv *priv = fe->demodulator_priv;

  return AVL_SX_DiseqcSendCmd(priv, cmd->msg, cmd->msg_len);
}

static int avl68x2_burst(struct dvb_frontend *fe, enum fe_sec_mini_cmd burst)
{
  struct avl68x2_priv *priv = fe->demodulator_priv;
  int ret;

  ret = (int)AVL_Demod_DVBSx_Diseqc_SendTone(burst == SEC_MINI_A ? 1 : 0, 1, priv->chip);

  return ret;
}

static int diseqc_set_tone(struct dvb_frontend *fe, enum fe_sec_tone_mode tone)
{
  struct avl68x2_priv *priv = fe->demodulator_priv;
  int r = AVL_EC_OK;

  dbg_avl("tone: %d", tone);
  switch (tone)
  {
  case SEC_TONE_ON:
    if (priv->chip->chip_pub->dvbsx_para.eDiseqcStatus != AVL_DOS_InContinuous)
    {
      r = (int)AVL_Demod_DVBSx_Diseqc_StartContinuous(priv->chip);
    }
    break;
  case SEC_TONE_OFF:
    if (priv->chip->chip_pub->dvbsx_para.eDiseqcStatus == AVL_DOS_InContinuous)
    {
      r = (int)AVL_Demod_DVBSx_Diseqc_StopContinuous(priv->chip);
    }
    break;
  default:
    return -EINVAL;
  }
  return r;
}

static int diseqc_set_voltage(struct dvb_frontend *fe, enum fe_sec_voltage voltage)
{
  struct avl68x2_priv *priv = fe->demodulator_priv;
  AVL_GPIOPinValue pwr, vol;
  int ret;

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
  ret = (int)AVL_Demod_SetGPIO(AVL_Pin37, pwr, priv->chip);
  ret |= (int)AVL_Demod_SetGPIO(AVL_Pin38, vol, priv->chip);
  return ret;
}

#define Level_High_Stage 36
#define Level_Low_Stage 76

#define Percent_Space_High 10
#define Percent_Space_Mid 30
#define Percent_Space_Low 60

static int avl68x2_read_status(struct dvb_frontend *fe, enum fe_status *status)
{
  struct avl68x2_priv *priv = fe->demodulator_priv;
  struct dtv_frontend_properties *c = &fe->dtv_property_cache;
  int ret = 0;
  uint8_t lock = 0;
  int32_t SNR_x100db = 0;
  int16_t SignalStrength = 0;

  switch (priv->delivery_system)
  {
  case SYS_DVBS:
  case SYS_DVBS2:
    ret = AVL_Demod_GetLockStatus(&lock, priv->chip);
    //dbg_avl("lock: %d", lock);
    if (lock == AVL_STATUS_LOCK)
    {
      ret |= AVL_Demod_GetSNR(&SNR_x100db, priv->chip);
      dbg_avl("SNR_x100db: %d", SNR_x100db);
      if (ret || SNR_x100db > 10000)
        SNR_x100db = 0;
    }
    else
    {
      *status = 0;
      return ret;
    }
    break;
  default:
    *status = 0;
    return 1;
  }

  if (ret)
  {
    *status = 0;
    return ret;
  }
  *status = FE_HAS_SIGNAL;

  //dbg_avl("%s", read_stdout(priv->chip));

  //ret = AVL_Demod_GetSignalStrength(&SignalStrength, priv->chip);

  c->strength.len = 2;

  c->strength.stat[1].scale = FE_SCALE_RELATIVE;
  c->strength.stat[1].uvalue = (SignalStrength * 65535) / 100;

  c->strength.stat[0].scale = FE_SCALE_DECIBEL;
  c->strength.stat[0].svalue = -80 + SignalStrength / 2;

  if (lock == (uint8_t)AVL_STATUS_LOCK)
  {
    *status |= FE_HAS_CARRIER | FE_HAS_VITERBI | FE_HAS_SYNC | FE_HAS_LOCK;
    c->cnr.len = 2;
    c->cnr.stat[0].scale = FE_SCALE_DECIBEL;
    c->cnr.stat[0].svalue = SNR_x100db * 10;
    c->cnr.stat[1].scale = FE_SCALE_RELATIVE;
    c->cnr.stat[1].uvalue = ((SNR_x100db + 300) / 10) * 250;
    if (c->cnr.stat[1].uvalue > 0xffff)
      c->cnr.stat[1].uvalue = 0xffff;
  }
  else
  {
    c->cnr.len = 1;
    c->cnr.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
  }
  return ret;
}

static int avl68x2_read_signal_strength(struct dvb_frontend *fe, u16 *strength)
{
  struct dtv_frontend_properties *c = &fe->dtv_property_cache;
  int i;

  *strength = 0;
  for (i = 0; i < c->strength.len; i++)
    if (c->strength.stat[i].scale == FE_SCALE_RELATIVE)
      *strength = (u16)c->strength.stat[i].uvalue;

  return 0;
}

static int avl68x2_read_snr(struct dvb_frontend *fe, u16 *snr)
{
  struct dtv_frontend_properties *c = &fe->dtv_property_cache;
  int i;

  *snr = 0;
  for (i = 0; i < c->cnr.len; i++)
    if (c->cnr.stat[i].scale == FE_SCALE_RELATIVE)
      *snr = (u16)c->cnr.stat[i].uvalue;

  return 0;
}

static int avl68x2_read_ber(struct dvb_frontend *fe, u32 *ber)
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

static int avl68x2_set_frontend(struct dvb_frontend *fe)
{
  int ret;
  //struct avl68x2_priv *priv = fe->demodulator_priv;

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

  dbg_avl("ACQUIRE");
  ret = avl68x2_set_dvbs(fe);

  return ret;
}

static int avl68x2_tune(struct dvb_frontend *fe, bool re_tune,
                        unsigned int mode_flags, unsigned int *delay, enum fe_status *status)
{
  *delay = HZ / 5;
  if (re_tune)
  {
    int ret = avl68x2_set_frontend(fe);
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
  kfree(priv->chip);
  kfree(priv);
}

static struct dvb_frontend_ops avl68x2_ops = {
    .delsys = {SYS_DVBS, SYS_DVBS2},
    .info = {
        .name = "Availink avl68x2",
        .frequency_min_hz = 950000000,
        .frequency_max_hz = 2150000000,
        .frequency_stepsize_hz = 0,
        .frequency_tolerance_hz = 0,
        .symbol_rate_min = 1000000,
        .symbol_rate_max = 55000000,
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
    .diseqc_send_master_cmd = avl68x2_diseqc,
    .diseqc_send_burst = avl68x2_burst,
    .get_frontend_algo = avl68x2fe_algo,
    .tune = avl68x2_tune,

    .set_frontend = avl68x2_set_frontend,
};

struct dvb_frontend *avl68x2_attach(struct avl68x2_config *config,
                                    struct i2c_adapter *i2c)
{
  struct avl68x2_priv *priv;
  avl_error_code_t ret;
  u32 id;

  dbg_avl("start demod attach");

  priv = kzalloc(sizeof(struct avl68x2_priv), GFP_KERNEL);
  if (priv == NULL)
    goto err;

  dbg_avl("priv alloc'ed = %llx", (unsigned long long int)priv);

  memcpy(&priv->frontend.ops, &avl68x2_ops,
         sizeof(struct dvb_frontend_ops));

  priv->frontend.demodulator_priv = priv;
  priv->i2c = i2c;
  priv->delivery_system = SYS_UNDEFINED;

  priv->chip = kzalloc(sizeof(struct avl68x2_chip), GFP_KERNEL);
  if (priv->chip == NULL)
    goto err1;
  
  priv->chip->chip_priv = kzalloc(sizeof(struct avl68x2_chip_priv), GFP_KERNEL);
  if(priv->chip->chip_priv == NULL)
    goto err2;
  
  priv->chip->chip_pub = kzalloc(sizeof(struct avl68x2_chip_pub), GFP_KERNEL);
  if(priv->chip->chip_pub == NULL)
    goto err3;

  /* copy (ephemeral?) public part of chip config into alloc'd area */
	memcpy(priv->chip->chip_pub,
	       config->chip_pub,
	       sizeof(struct avl62x1_chip_pub));

	priv->chip->chip_pub->tuner = &default_avl_tuner;

	dbg_avl("Demod ID %d, I2C addr 0x%x",
		(priv->chip->chip_pub->i2c_addr >> AVL_DEMOD_ID_SHIFT) & AVL_DEMOD_ID_MASK,
		priv->chip->chip_pub->i2c_addr & 0xFF);

	// associate demod ID with i2c_adapter
	avl_bsp_assoc_i2c_adapter(priv->chip->chip_pub->i2c_addr, i2c);
  

  /* get chip id */
  ret = AVL_Demod_GetChipID(&id, priv->chip);
  if (ret)
  {
    dev_err(&priv->i2c->dev, "%s: attach failed reading id",
            KBUILD_MODNAME);
    goto err4;
  }

  dbg_avl("chip_id= %d\n", id);

  if (id != AVL68XX)
  {
    dev_err(&priv->i2c->dev, "%s: attach failed, id mismatch",
            KBUILD_MODNAME);
    goto err4;
  }

  dev_info(&priv->i2c->dev, "%s: found AVL68x2 id=0x%x",
           KBUILD_MODNAME, id);


  if (!set_dvb_mode(&priv->frontend, SYS_DVBS))
  {
    dev_info(&priv->i2c->dev,
             KBUILD_MODNAME ": Firmware booted");
    return &priv->frontend;
  }

err4:
  kfree(priv->chip->chip_pub);
err3:
  kfree(priv->chip->chip_priv);
err2:
  kfree(priv->chip);
err1:
  kfree(priv);
err:
  return NULL;
}
EXPORT_SYMBOL_GPL(avl68x2_attach);

MODULE_DESCRIPTION("Availink AVL68x2 DVB-S/S2/T/T2/C, ISDB-T, J.83B demodulator driver");
MODULE_AUTHOR("Availink, Inc. (gpl@availink.com)");
MODULE_LICENSE("GPL");
MODULE_VERSION(AVL68x2_VERSION);

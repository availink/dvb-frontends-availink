/*
 * Availink AVL62X1 DVB-S/S2/S2X demodulator driver
 * Supports AVL6221 and AVL6261. NOT AVL6211
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright (C) 2020 Availink, Inc. (opensource@availink.com)
 *
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/bitrev.h>
#include <linux/types.h>

#include "avl62x1.h"
#include "avl62x1_api.h"
#include "avl_tuner.h"

#define dbg_avl(fmt, args...)                                           \
	do                                                              \
	{                                                               \
		if (debug)                                              \
			printk("%s(): " fmt "\n", __func__, ##args); \
	} while (0);

static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "\n\t\t Enable debug");

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

int init_error_stat(struct avl62x1_priv *priv)
{
	uint16_t r = AVL_EC_OK;
	struct avl62x1_error_stats_config err_stats_config;
	struct avl62x1_ber_config ber_config;

	err_stats_config.error_stats_mode = avl62x1_error_stats_auto;
	err_stats_config.auto_error_stats_type = avl62x1_error_stats_time;
	err_stats_config.time_threshold_ms = 3000;
	err_stats_config.bytes_threshold = 0;

	r = avl62x1_config_error_stats(&err_stats_config, priv->chip);

	ber_config.test_pattern = avl62x1_test_lfsr_23;
	ber_config.fb_inversion = avl62x1_lfsr_fb_inverted;
	ber_config.lfsr_sync = 0;
	ber_config.lfsr_start_pos = 4;
	r |= avl62x1_reset_ber(&ber_config, priv->chip);

	return r;
}

static int init_dvbs_s2(struct dvb_frontend *fe)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	struct avl62x1_diseqc_params params;
	uint16_t r = AVL_EC_OK;

	params.tone_freq_khz = 22;
	params.tx_gap = avl62x1_dtxg_15ms;
	params.tx_waveform = avl62x1_dwm_normal;
	params.rx_timeout = avl62x1_drt_150ms;
	params.rx_waveform = avl62x1_dwm_normal;

	r |= avl62x1_init_diseqc(&params, priv->chip);
	if (AVL_EC_OK != r)
	{
		dbg_avl("Diseqc Init failed !\n");
	}

	r |= (int)avl62x1_set_gpio_dir(avl62x1_gpio_pin_lnb_pwr_en,
				       avl62x1_gpio_dir_output,
				       priv->chip);
	r |= (int)avl62x1_set_gpio_dir(avl62x1_gpio_pin_lnb_pwr_sel,
				       avl62x1_gpio_dir_output,
				       priv->chip);

	return r;
}

static int i2c_gate_ctrl(struct dvb_frontend *fe, int enable)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	uint16_t ret = AVL_EC_OK;

	dbg_avl("%d\n", enable);

	if (priv->chip == NULL)
	{
		dev_err(&priv->i2c->dev,
			KBUILD_MODNAME ": NULL fe->demodulator_priv->chip");
	}

	if (enable)
	{
		ret = avl62x1_enable_tuner_i2c(priv->chip);
	}
	else
	{
		ret = avl62x1_disable_tuner_i2c(priv->chip);
	}
	return ret;
}

static int acquire_dvbs_s2(struct dvb_frontend *fe)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	uint16_t r = AVL_EC_OK;
	struct avl62x1_carrier_info carrier_info;
	struct avl62x1_stream_info stream_info;
	dbg_avl("Freq:%d khz,sym:%d hz", c->frequency, c->symbol_rate);

	carrier_info.rf_freq_khz = c->frequency;
	carrier_info.carrier_freq_offset_hz = 0;
	carrier_info.symbol_rate_hz = c->symbol_rate;
	carrier_info.pl_scrambling = AVL62X1_PL_SCRAM_AUTO;
	stream_info.stream_type = avl62x1_transport;
	stream_info.isi = c->stream_id;

	r = avl62x1_lock_tp(&carrier_info,
			    &stream_info,
			    AVL_FALSE, /* don't do blind symbol rate */
			    priv->chip);

	return r;
}

static int set_dvb_mode(struct dvb_frontend *fe,
			enum fe_delivery_system delsys)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	uint16_t ret = AVL_EC_OK;
	uint16_t r = AVL_EC_OK;
	struct avl62x1_ver_info ver_info;

	/* already in desired mode */
	if (priv->delivery_system == delsys)
		return 0;

	dbg_avl("initing demod for delsys=%d", delsys);

	switch (priv->delivery_system)
	{
	case SYS_DVBS:
		if (delsys == SYS_DVBS2)
			return 0;
		break;
	case SYS_DVBS2:
		if (delsys == SYS_DVBS)
			return 0;
		break;
	default:
		break;
	}
	priv->delivery_system = delsys;

	//Reset Demod
	r = avl_bsp_reset();
	if (AVL_EC_OK != r)
	{
		dbg_avl("Failed to Resed demod via BSP!\n");
		return 0;
	}

	// boot the firmware here
	r |= avl62x1_initialize(priv->chip);
	if (AVL_EC_OK != r)
	{
		dbg_avl("AVL_AVL62X1_Initialize failed !\n");
		return (r);
	}

	r |= avl62x1_get_version(&ver_info, priv->chip);
	if (AVL_EC_OK != r)
	{
		dbg_avl("avl62x1_get_version failed\n");
		return (r);
	}
	dbg_avl("FW version %d.%d.%d\n",
		ver_info.firmware.major,
		ver_info.firmware.minor,
		ver_info.firmware.build);
	dbg_avl("SDK version %d.%d.%d\n",
		ver_info.sdk.major,
		ver_info.sdk.minor,
		ver_info.sdk.build);

	switch (priv->delivery_system)
	{
	case SYS_DVBS:
	case SYS_DVBS2:
	default:
		ret |= init_dvbs_s2(fe);
		break;
	}

	ret |= init_error_stat(priv);

	if (ret)
	{
		dev_err(&priv->i2c->dev, KBUILD_MODNAME ": demod init failed");
	}

	return ret;
}

uint16_t diseqc_send_cmd(struct avl62x1_priv *priv,
			 uint8_t *cmd,
			 uint8_t cmdsize)
{
	uint16_t r = AVL_EC_OK;
	struct avl62x1_diseqc_tx_status tx_status;
	dbg_avl(" %*ph", cmdsize, cmd);

	r = avl62x1_send_diseqc_data(cmd, cmdsize, priv->chip);
	if (r != AVL_EC_OK)
	{
		printk("diseqc_send_cmd failed !\n");
	}
	else
	{
		do
		{
			msleep(5);
			r |= avl62x1_get_diseqc_tx_status(&tx_status,
							  priv->chip);
		} while (tx_status.tx_complete != 1);
		if (r == AVL_EC_OK)
		{
		}
		else
		{
			printk("diseqc_send_cmd Err. !\n");
		}
	}
	return (int)(r);
}

static int diseqc_send_master_cmd(struct dvb_frontend *fe,
				  struct dvb_diseqc_master_cmd *cmd)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;

	return diseqc_send_cmd(priv, cmd->msg, cmd->msg_len);
}

static int diseqc_send_burst(struct dvb_frontend *fe,
			     enum fe_sec_mini_cmd burst)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	int ret;
	uint8_t tone = burst == SEC_MINI_A ? 1 : 0;
	uint8_t count = 1;
	ret = (int)avl62x1_send_diseqc_tone(tone, count, priv->chip);

	return ret;
}

static int diseqc_set_tone(struct dvb_frontend *fe, enum fe_sec_tone_mode tone)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	int r = AVL_EC_OK;

	dbg_avl("tone: %d", tone);
	switch (tone)
	{
	case SEC_TONE_ON:
		if (priv->chip->chip_priv->diseqc_op_status !=
		    avl62x1_dos_continuous)
		{
			r = (int)avl62x1_diseqc_tone_off(priv->chip);
		}
		break;
	case SEC_TONE_OFF:
		if (priv->chip->chip_priv->diseqc_op_status ==
		    avl62x1_dos_continuous)
		{
			r = (int)avl62x1_diseqc_tone_on(priv->chip);
		}
		break;
	default:
		return -EINVAL;
	}
	return r;
}

static int diseqc_set_voltage(struct dvb_frontend *fe,
			      enum fe_sec_voltage voltage)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	avl62x1_gpio_pin_value pwr, vol;
	int ret;

	dbg_avl("volt: %d", voltage);

	switch (voltage)
	{
	case SEC_VOLTAGE_OFF:
		pwr = avl62x1_gpio_value_logic_0;
		vol = avl62x1_gpio_value_logic_0;
		break;
	case SEC_VOLTAGE_13:
		//power on
		pwr = avl62x1_gpio_value_logic_1;
		vol = avl62x1_gpio_value_logic_0;
		break;
	case SEC_VOLTAGE_18:
		//power on
		pwr = avl62x1_gpio_value_logic_1;
		vol = avl62x1_gpio_value_high_z;
		break;
	default:
		return -EINVAL;
	}
	ret = (int)avl62x1_set_gpio_value(avl62x1_gpio_pin_lnb_pwr_en,
					  pwr,
					  priv->chip);
	ret |= (int)avl62x1_set_gpio_value(avl62x1_gpio_pin_lnb_pwr_sel,
					   vol,
					   priv->chip);
	return ret;
}

static int get_frontend(struct dvb_frontend *fe,
			struct dtv_frontend_properties *props)
{
	int ret = 0;
	struct avl62x1_priv *priv = fe->demodulator_priv;
	avl62x1_lock_status lock;
	struct avl62x1_carrier_info carrier_info;
	struct avl62x1_stream_info stream_info;
	int16_t snr_db_x100;
	uint16_t sig_strength;

	//dbg_avl("ENTER");

	ret = avl62x1_get_lock_status(&lock,
				      priv->chip);

	if ((lock == avl62x1_status_locked) &&
	    (ret == AVL_EC_OK))
	{

		ret |= avl62x1_get_signal_info(&carrier_info,
					priv->chip);
		
		
		ret |= avl62x1_get_stream_info(&stream_info,
					priv->chip);

		props->frequency = carrier_info.rf_freq_khz * 1000 +
				carrier_info.carrier_freq_offset_hz;
		
		switch(carrier_info.modulation) {
		case avl62x1_qpsk:
			props->modulation = QPSK;
			break;
		case avl62x1_8psk:
			props->modulation = PSK_8;
			break;
		case avl62x1_16apsk:
			props->modulation = APSK_16;
			break;
		default: //DVBv5 has no >=APSK_64
			props->modulation = APSK_32;
			break;
		}

		props->inversion =
		(carrier_info.spectrum_invert == avl62x1_specpol_inverted)
			? INVERSION_ON
			: INVERSION_OFF;

		props->symbol_rate = carrier_info.symbol_rate_hz;

		if(carrier_info.pilot == avl62x1_pilot_on) {
			props->pilot = PILOT_ON;
		} else {
			props->pilot = PILOT_OFF;
		}

		switch(carrier_info.roll_off) {
		case avl62x1_rolloff_35:
			props->rolloff = ROLLOFF_35;
			break;
		case avl62x1_rolloff_25:
			props->rolloff = ROLLOFF_25;
			break;
		case avl62x1_rolloff_20:
			props->rolloff = ROLLOFF_20;
			break;
		default:
			props->rolloff = ROLLOFF_20;
		}

		if(carrier_info.signal_type == avl62x1_dvbs2) {
			props->delivery_system = SYS_DVBS2;
			switch(carrier_info.code_rate.dvbs2_code_rate) {
			case avl62x1_dvbs2_cr_2_5:
				props->fec_inner = FEC_2_5;
				break;
			case avl62x1_dvbs2_cr_1_2:
				props->fec_inner = FEC_1_2;
				break;
			case avl62x1_dvbs2_cr_3_5:
				props->fec_inner = FEC_3_5;
				break;
			case avl62x1_dvbs2_cr_2_3:
				props->fec_inner = FEC_2_3;
				break;
			case avl62x1_dvbs2_cr_3_4:
				props->fec_inner = FEC_3_4;
				break;
			case avl62x1_dvbs2_cr_4_5:
				props->fec_inner = FEC_4_5;
				break;
			case avl62x1_dvbs2_cr_5_6:
				props->fec_inner = FEC_5_6;
				break;
			case avl62x1_dvbs2_cr_8_9:
				props->fec_inner = FEC_8_9;
				break;
			case avl62x1_dvbs2_cr_9_10:
				props->fec_inner = FEC_9_10;
				break;
			default: //DVBv5 missing many rates (e.g. all S2/X)
				props->fec_inner = FEC_AUTO;
			}
		} else {
			props->delivery_system = SYS_DVBS;
			switch(carrier_info.code_rate.dvbs_code_rate) {
			case avl62x1_dvbs_cr_1_2:
				props->fec_inner = FEC_1_2;
				break;
			case avl62x1_dvbs_cr_2_3:
				props->fec_inner = FEC_2_3;
				break;
			case avl62x1_dvbs_cr_3_4:
				props->fec_inner = FEC_3_4;
				break;
			case avl62x1_dvbs_cr_5_6:
				props->fec_inner = FEC_5_6;
				break;
			default:
				props->fec_inner = FEC_7_8;
			}
		}
		
		props->stream_id = stream_info.isi;

		/*  STATS  */
		//SNR
		ret |= avl62x1_get_snr(&snr_db_x100,
				       priv->chip);
		props->cnr.len = 2;
		props->cnr.stat[0].scale = FE_SCALE_DECIBEL; //0.001dB
		props->cnr.stat[0].svalue = snr_db_x100 * 10;
		props->cnr.stat[1].scale = FE_SCALE_RELATIVE;
		//props->cnr.stat[1].uvalue = ((snr_db_x100 + 300) / 10) * 250;
		//max SNR is about 28dB, min is about -3
		props->cnr.stat[1].uvalue = ((snr_db_x100+300) * 0xffff) / (31*100);
		if (props->cnr.stat[1].uvalue > 0xffff) {
			props->cnr.stat[1].uvalue = 0xffff;
		}

		//RF strength
		ret |= avl62x1_get_signal_strength(&sig_strength,
						   priv->chip);

		props->strength.len = 2;
		props->strength.stat[0].scale = FE_SCALE_DECIBEL;
		props->strength.stat[0].svalue = -80 + sig_strength / 2;
		props->strength.stat[1].scale = FE_SCALE_RELATIVE;
		props->strength.stat[1].uvalue = (sig_strength * 0xffff) / 100;
		if(props->strength.stat[1].uvalue > 0xffff) {
			props->strength.stat[1].uvalue = 0xffff;
		}

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
	} else {
		//not locked
		props->cnr.len = 1;
		props->cnr.stat[0].scale = FE_SCALE_NOT_AVAILABLE;

		props->strength.len = 1;
		props->strength.stat[1].scale = FE_SCALE_NOT_AVAILABLE;

		props->block_error.len = 1;
		props->block_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;

		props->block_count.len = 1;
		props->block_count.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
	}
	return ret;
}

static int read_status(struct dvb_frontend *fe, enum fe_status *status)
{
	int r;
	struct avl62x1_priv *priv = fe->demodulator_priv;
	avl62x1_lock_status lock;

	//dbg_avl("ENTER");

	r = avl62x1_get_lock_status(&lock,
				    priv->chip);
	if(r == AVL_EC_OK) {
		if(lock == avl62x1_status_locked) {
			*status = FE_HAS_LOCK;
			r = get_frontend(fe,
					 &fe->dtv_property_cache);
		} else {
			*status = FE_NONE;
		}
	} else {
		r = -EIO;
	}
	return r;
}

static int read_signal_strength(struct dvb_frontend *fe, u16 *strength)
{
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int i;

	//dbg_avl("ENTER");

	*strength = 0;
	for (i = 0; i < c->strength.len; i++)
		if (c->strength.stat[i].scale == FE_SCALE_RELATIVE)
			*strength = (u16)c->strength.stat[i].uvalue;

	return 0;
}

static int read_snr(struct dvb_frontend *fe, u16 *snr)
{
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int i;

	//dbg_avl("ENTER");

	*snr = 0;
	for (i = 0; i < c->cnr.len; i++)
		if (c->cnr.stat[i].scale == FE_SCALE_RELATIVE)
			*snr = (u16)c->cnr.stat[i].uvalue;

	return 0;
}
static int read_ber(struct dvb_frontend *fe, u32 *ber)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	int ret;

	//dbg_avl("ENTER");

	//FIXME
	*ber = 10e7;
	ret = (int)avl62x1_get_per(ber, priv->chip);
	if (!ret)
		*ber /= 100;

	return ret;
}

static int get_frontend_algo(struct dvb_frontend *fe)
{
	return DVBFE_ALGO_HW;
}

static int set_frontend(struct dvb_frontend *fe)
{
	int ret;
	dbg_avl("ENTER");

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
	ret = acquire_dvbs_s2(fe);

	return ret;
}

static int tune(struct dvb_frontend *fe,
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
	return read_status(fe, status);
}

static int init_fe(struct dvb_frontend *fe)
{
	return 0;
}

static int sleep_fe(struct dvb_frontend *fe)
{
	return 0;
}

static void release_fe(struct dvb_frontend *fe)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	kfree(priv->chip->chip_pub);
	kfree(priv->chip->chip_priv);
	kfree(priv->chip);
	kfree(priv);
}

static struct dvb_frontend_ops avl62x1_ops = {
    .delsys = {SYS_DVBS, SYS_DVBS2},
    .info = {
	.name = "Availink avl62x1",
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

    .release = release_fe,
    .init = init_fe,

    .sleep = sleep_fe,
    .i2c_gate_ctrl = i2c_gate_ctrl,

    .read_status = read_status,
    .read_ber = read_ber, //V3
    .read_snr = read_snr, //V3
    .read_signal_strength = read_signal_strength, //V3
    .set_tone = diseqc_set_tone,
    .set_voltage = diseqc_set_voltage,
    .diseqc_send_master_cmd = diseqc_send_master_cmd,
    .diseqc_send_burst = diseqc_send_burst,
    .get_frontend_algo = get_frontend_algo,
    .tune = tune,
    .set_frontend = set_frontend,
    .get_frontend = get_frontend,
};

struct dvb_frontend *avl62x1_attach(struct avl62x1_config *config,
				    struct i2c_adapter *i2c)
{
	struct avl62x1_priv *priv;
	uint16_t ret;
	u32 id;
	int fw_status;
	unsigned int fw_maj, fw_min, fw_build;

	dbg_avl("enter %s()", __FUNCTION__);

	priv = kzalloc(sizeof(struct avl62x1_priv), GFP_KERNEL);
	if (priv == NULL)
		goto err;

	priv->chip = kzalloc(sizeof(struct avl62x1_chip), GFP_KERNEL);
	if (priv->chip == NULL)
		goto err1;

	priv->chip->chip_priv = kzalloc(sizeof(struct avl62x1_chip_priv),
					GFP_KERNEL);
	if (priv->chip->chip_priv == NULL)
		goto err2;

	priv->chip->chip_pub = kzalloc(sizeof(struct avl62x1_chip_pub),
				       GFP_KERNEL);
	if (priv->chip->chip_pub == NULL)
		goto err3;

	memcpy(&priv->frontend.ops, &avl62x1_ops,
	       sizeof(struct dvb_frontend_ops));

	priv->frontend.demodulator_priv = priv;
	priv->i2c = i2c;
	priv->delivery_system = -1;

	/* copy (ephemeral?) public part of chip config into alloc'd area */
	memcpy(priv->chip->chip_pub,
	       config->chip_pub,
	       sizeof(struct avl62x1_chip_pub));

	priv->chip->chip_pub->tuner = &default_avl_tuner;

	dbg_avl("Demod %d, I2C addr 0x%x",
		(priv->chip->chip_pub->i2c_addr >> 8) & 0x7,
		priv->chip->chip_pub->i2c_addr & 0xFF);

	// associate demod ID with i2c_adapter
	avl_bsp_assoc_i2c_adapter(priv->chip->chip_pub->i2c_addr, i2c);

	//set up semaphores
	ret = avl62x1_init_chip_object(priv->chip);
	if (ret)
	{
		dev_err(&priv->i2c->dev,
			KBUILD_MODNAME ": chip object init failed");
		goto err4;
	}

	/* get chip id */
	ret = avl62x1_get_chip_id(priv->chip->chip_pub->i2c_addr, &id);
	if (ret)
	{
		dev_err(&priv->i2c->dev,
			KBUILD_MODNAME ": attach failed reading id");
		goto err4;
	}

	dbg_avl("chip_id 0x%x\n", id);

	if (id != AVL62X1_CHIP_ID)
	{
		dev_err(&priv->i2c->dev,
			KBUILD_MODNAME ": attach failed, id mismatch");
		goto err4;
	}

	dev_info(&priv->i2c->dev, KBUILD_MODNAME ": found AVL62x1 id=0x%x", id);

	fw_status = request_firmware(&priv->fw,
				     "availink/avl62x1.patch",
				     i2c->dev.parent);
	if (fw_status != 0)
	{
		dev_err(&priv->i2c->dev,
			KBUILD_MODNAME ": firmware file not found");
		goto err4;
	}
	else
	{
		priv->chip->chip_priv->patch_data = (unsigned char *)(priv->fw->data);
		fw_maj = priv->chip->chip_priv->patch_data[24]; //major rev
		fw_min = priv->chip->chip_priv->patch_data[25]; //SDK-FW API rev
		fw_build = (priv->chip->chip_priv->patch_data[26] << 8) |
			   priv->chip->chip_priv->patch_data[27]; //internal rev
		if(fw_min != AVL62X1_SDK_VER_MINOR)
		{
			//SDK-FW API rev must match
			dev_err(&priv->i2c->dev,
				KBUILD_MODNAME ": Firmware version %d.%d.%d incompatible with this driver version",
				fw_maj, fw_min, fw_build);
			dev_err(&priv->i2c->dev,
				KBUILD_MODNAME ": Firmware minor version must be %d",
				AVL62X1_SDK_VER_MINOR);
			goto err5;
		}
		else
		{
			dev_info(&priv->i2c->dev,
				 KBUILD_MODNAME ": Firmware version %d.%d.%d found",
				 fw_maj, fw_min, fw_build);
		}
	}

	if (!set_dvb_mode(&priv->frontend, SYS_DVBS2))
	{
		dev_info(&priv->i2c->dev,
			 KBUILD_MODNAME ": Firmware booted");
		release_firmware(priv->fw);
		return &priv->frontend;
	}

err5:
	release_firmware(priv->fw);
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
} /* end avl62x1_attach() */

EXPORT_SYMBOL_GPL(avl62x1_attach);

MODULE_DESCRIPTION("Availink AVL62X1 DVB-S/S2/S2X demodulator driver");
MODULE_AUTHOR("Availink, Inc. (opensource@availink.com)");
MODULE_LICENSE("GPL");
MODULE_VERSION(AVL62X1_VERSION);

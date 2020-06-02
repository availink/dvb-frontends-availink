// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Availink AVL6261 DVB-S/S2/S2X demodulator public APIs
 * 
 * Copyright (C) 2020 Availink, Inc. (gpl@availink.com)
 *
 */

#include "avl62x1_api.h"

uint16_t avl62x1_init_chip_object(struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;

	chip->chip_priv->diseqc_op_status = avl62x1_dos_uninit;
	r = avl_bsp_init_semaphore(&(chip->chip_priv->rx_cmd_sem));
	r |= avl_bsp_init_semaphore(&(chip->chip_priv->diseqc_sem));

	r |= avl_bms_initialize(chip->chip_pub->i2c_addr);

	chip->chip_priv->agc_driven = 0;

	return (r);
}

uint16_t avl62x1_get_version(struct avl62x1_ver_info *version,
			     struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	uint32_t tmp = 0;
	uint8_t buf[4] = {0};

	r = avl_bms_read32(chip->chip_pub->i2c_addr,
			   0x40000,
			   &tmp);
	version->hw_version = tmp;

	r |= avl_bms_read32(chip->chip_pub->i2c_addr,
			    s_AVL62X1_DMD_patch_ver_iaddr,
			    &tmp);
	avl_int_to_bytes(tmp, buf);
	version->firmware.major = buf[0];
	version->firmware.minor = buf[1];
	version->firmware.build = buf[2];
	version->firmware.build =
	    (uint16_t)((version->firmware.build) << 8) + buf[3];

	version->driver.major = AVL62X1_VER_MAJOR;
	version->driver.minor = AVL62X1_VER_MINOR;
	version->driver.build = AVL62X1_VER_BUILD;

	return (r);
}

uint16_t avl62x1_get_chip_id(uint16_t slave_addr, uint32_t *chip_id)
{
	uint16_t r = AVL_EC_OK;

	r = avl_bms_read32(slave_addr, 0x40000, chip_id);
	return (r);
}

uint16_t avl62x1_initialize(struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	uint16_t i = 0;
	uint16_t max_retries = 100;
	uint16_t delay_unit_ms = 50;

	//download, boot, load defaults
	r |= __avl62x1_initialize(chip);
	while (AVL_EC_OK != __avl62x1_check_chip_ready(chip))
	{
		if (i++ >= max_retries)
		{
			r |= AVL_EC_GENERAL_FAIL;
			return (r);
		}
		avl_bsp_delay(delay_unit_ms);
	}

	r |= __avl62x1_init_tuner_i2c(chip); //config i2c repeater
	r |= __avl62x1_init_adc(chip);
	r |= __avl62x1_set_tuner_polarity(chip->chip_pub->tuner_pol, chip);
	r |= __avl62x1_drive_agc(avl62x1_on, chip);
	r |= __avl62x1_set_mpeg_mode(chip);
	r |= __avl62x1_drive_mpeg_output(avl62x1_on, chip);

	return (r);
}

uint16_t avl62x1_config_error_stats(
    struct avl62x1_error_stats_config *config,
    struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	struct avl_uint64 time_tick_num = {0, 0};

	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     ber_esm__auto1_manual0_mode__offset,
			     (uint32_t)config->error_stats_mode);

	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     ber_esm__timetick1_bytetick0__offset,
			     (uint32_t)config->auto_error_stats_type);

	avl_mult_32to64(
	    &time_tick_num,
	    chip->chip_priv->mpeg_clk_freq_hz / 1000,
	    config->time_threshold_ms);

	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     ber_esm__time_tick_low__offset,
			     time_tick_num.low_word);

	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     ber_esm__time_tick_high__offset,
			     time_tick_num.high_word);

	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     ber_esm__byte_tick_low__offset,
			     (uint32_t)config->bytes_threshold);

	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     ber_esm__byte_tick_high__offset,
			     0); //high 32-bit is not used

	if (config->error_stats_mode == avl62x1_error_stats_auto)
	{
		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     ber_esm__tick_clear_req__offset,
				     0);

		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     ber_esm__tick_clear_req__offset,
				     1);

		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     ber_esm__tick_clear_req__offset,
				     0);
	}

	return (r);
}

uint16_t avl62x1_reset_ber(struct avl62x1_ber_config *config,
			   struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	uint32_t tmp32 = 0;
	uint16_t lfsr_sync = 0;
	uint32_t cnt = 0;
	uint32_t byte_cnt = 0;
	uint32_t ber_fail_cnt = 0;
	uint32_t bit_errs = 0;

	chip->chip_priv->error_stats.lfsr_sync = 0;
	chip->chip_priv->error_stats.lost_lock = 0;
	chip->chip_priv->error_stats.sw_cnt_num_bits.high_word = 0;
	chip->chip_priv->error_stats.sw_cnt_num_bits.low_word = 0;
	chip->chip_priv->error_stats.sw_cnt_bit_errs.high_word = 0;
	chip->chip_priv->error_stats.sw_cnt_bit_errs.low_word = 0;
	chip->chip_priv->error_stats.num_bits.high_word = 0;
	chip->chip_priv->error_stats.num_bits.low_word = 0;
	chip->chip_priv->error_stats.bit_errs.high_word = 0;
	chip->chip_priv->error_stats.bit_errs.low_word = 0;
	chip->chip_priv->error_stats.ber_x1e9 = 0;

	//ber software reset
	r = avl_bms_read32(chip->chip_pub->i2c_addr,
			   ber_esm__esm_cntrl,
			   &tmp32);
	tmp32 |= 0x00000002;

	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     ber_esm__esm_cntrl,
			     tmp32);

	//alway inverted
	config->fb_inversion = avl62x1_lfsr_fb_inverted;

	//set Test Pattern and Inversion
	r |= avl_bms_read32(chip->chip_pub->i2c_addr,
			    ber_esm__esm_cntrl,
			    &tmp32);
	tmp32 &= 0xFFFFFFCF;

	//BER_Test_Pattern:bit 5 --- 0:LFSR_15; 1:LFSR_23
	//BER_FB_Inversion:bit 4 --- 0:NOT_INVERTED; 1:INVERTED
	tmp32 |= ((uint32_t)(config->test_pattern) << 5) |
		 ((uint32_t)(config->fb_inversion) << 4);

	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     ber_esm__esm_cntrl,
			     tmp32);

	/*For SFU or other standard, the start position of LFSR is 1,
	just following the 0x47 sync byte.*/
	tmp32 &= 0xFFFFFE3F;
	tmp32 |= ((config->lfsr_start_pos) << 6);

	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     ber_esm__esm_cntrl,
			     tmp32);

	while (!lfsr_sync)
	{
		tmp32 |= 0x00000006;
		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     ber_esm__esm_cntrl,
				     tmp32);

		tmp32 &= 0xFFFFFFFD;
		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     ber_esm__esm_cntrl,
				     tmp32);

		cnt = 0;
		byte_cnt = 0;
		while ((byte_cnt < 1000) && (cnt < 200))
		{
			r |= avl_bms_read32(chip->chip_pub->i2c_addr,
					    ber_esm__byte_num,
					    &byte_cnt);
			cnt++;
		}

		tmp32 |= 0x00000006;
		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     ber_esm__esm_cntrl,
				     tmp32);

		tmp32 &= 0xFFFFFFF9;
		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     ber_esm__esm_cntrl,
				     tmp32);

		cnt = 0;
		byte_cnt = 0;
		while ((byte_cnt < 10000) && (cnt < 200))
		{
			cnt++;
			r |= avl_bms_read32(chip->chip_pub->i2c_addr,
					    ber_esm__byte_num,
					    &byte_cnt);
		}

		tmp32 &= 0xFFFFFFF9;
		tmp32 |= 0x00000002;
		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     ber_esm__esm_cntrl,
				     tmp32);

		r |= avl_bms_read32(chip->chip_pub->i2c_addr,
				    ber_esm__byte_num,
				    &byte_cnt);
		r |= avl_bms_read32(chip->chip_pub->i2c_addr,
				    ber_esm__ber_err_cnt,
				    &bit_errs);
		if (cnt == 200)
		{
			break;
		}
		else if ((byte_cnt << 3) < (10 * bit_errs))
		{
			ber_fail_cnt++;
			if (ber_fail_cnt > 10)
			{
				break;
			}
		}
		else
		{
			lfsr_sync = 1;
		}
	}

	if (lfsr_sync == 1)
	{
		tmp32 &= 0xFFFFFFF9;
		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     ber_esm__esm_cntrl,
				     tmp32);
	}

	config->lfsr_sync = lfsr_sync;
	chip->chip_priv->error_stats.lfsr_sync = lfsr_sync;

	return (r);
}

uint16_t avl62x1_get_ber(uint32_t *ber_x1e9, struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	uint32_t hw_cnt_bit_errs = 0;
	uint32_t hw_cnt_num_bits = 0;
	uint32_t tmp32 = 0;
	struct avl_uint64 tmp64;

	r = avl_bms_read32(chip->chip_pub->i2c_addr,
			   ber_esm__ber_err_cnt,
			   &hw_cnt_bit_errs);

	r |= avl_bms_read32(chip->chip_pub->i2c_addr,
			    ber_esm__byte_num,
			    &hw_cnt_num_bits);
	hw_cnt_num_bits <<= 3;

	//Keep the hw counts into sw struct to avoid hw registers overflow
	if (hw_cnt_num_bits > (uint32_t)(1 << 31))
	{
		r |= avl_bms_read32(chip->chip_pub->i2c_addr,
				    ber_esm__esm_cntrl,
				    &tmp32);
		tmp32 |= 0x00000002;

		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     ber_esm__esm_cntrl,
				     tmp32);

		r |= avl_bms_read32(chip->chip_pub->i2c_addr,
				    ber_esm__ber_err_cnt,
				    &hw_cnt_bit_errs);

		r |= avl_bms_read32(chip->chip_pub->i2c_addr,
				    ber_esm__byte_num,
				    &hw_cnt_num_bits);
		tmp32 &= 0xFFFFFFFD;

		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     ber_esm__esm_cntrl,
				     tmp32);
		hw_cnt_num_bits <<= 3;

		avl_add_32to64(
		    &chip->chip_priv->error_stats.sw_cnt_num_bits,
		    hw_cnt_num_bits);
		avl_add_32to64(
		    &chip->chip_priv->error_stats.sw_cnt_bit_errs,
		    hw_cnt_bit_errs);
		hw_cnt_num_bits = 0;
		hw_cnt_bit_errs = 0;
	}

	chip->chip_priv->error_stats.num_bits.high_word =
	    chip->chip_priv->error_stats.sw_cnt_num_bits.high_word;

	chip->chip_priv->error_stats.num_bits.low_word =
	    chip->chip_priv->error_stats.sw_cnt_num_bits.low_word;

	avl_add_32to64(
	    &chip->chip_priv->error_stats.num_bits,
	    hw_cnt_num_bits);

	chip->chip_priv->error_stats.bit_errs.high_word =
	    chip->chip_priv->error_stats.sw_cnt_bit_errs.high_word;

	chip->chip_priv->error_stats.bit_errs.low_word =
	    chip->chip_priv->error_stats.sw_cnt_bit_errs.low_word;

	avl_add_32to64(
	    &chip->chip_priv->error_stats.bit_errs,
	    hw_cnt_bit_errs);

	avl_mult_32to64(
	    &tmp64,
	    chip->chip_priv->error_stats.bit_errs.low_word,
	    AVL_CONSTANT_10_TO_THE_9TH);

	chip->chip_priv->error_stats.ber_x1e9 =
	    avl_divide_64(
		chip->chip_priv->error_stats.num_bits,
		tmp64);

	//keep the BER user wanted
	*ber_x1e9 = chip->chip_priv->error_stats.ber_x1e9;

	return (r);
}

uint16_t avl62x1_init_diseqc(
    struct avl62x1_diseqc_params *params,
    struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	uint32_t i1 = 0;

	r = avl_bsp_wait_semaphore(&(chip->chip_priv->diseqc_sem));
	if (AVL_EC_OK == r)
	{
		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     diseqc__diseqc_srst,
				     1);

		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     diseqc__diseqc_samp_frac_n,
				     2000000);

		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     diseqc__diseqc_samp_frac_d,
				     chip->chip_priv->core_clk_freq_hz);

		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     diseqc__diseqc_tone_frac_n,
				     (params->tone_freq_khz) << 1);

		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     diseqc__diseqc_tone_frac_d,
				     (chip->chip_priv->core_clk_freq_hz / 1000));

		// Initialize the tx_control
		r |= avl_bms_read32(chip->chip_pub->i2c_addr,
				    diseqc__diseqc_tx_cntrl,
				    &i1);

		i1 &= 0x00000300;
		i1 |= 0x20; //reset tx_fifo
		i1 |= ((uint32_t)(params->tx_gap) << 6);
		i1 |= ((uint32_t)(params->tx_waveform) << 4);
		i1 |= (1 << 3); //enable tx gap.

		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     diseqc__diseqc_tx_cntrl,
				     i1);

		i1 &= ~(0x20); //release tx_fifo reset

		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     diseqc__diseqc_tx_cntrl,
				     i1);

		// Initialize the rx_control
		i1 = ((uint32_t)(params->rx_waveform) << 2);
		i1 |= (1 << 1); //active the receiver
		i1 |= (1 << 3); //envelop high when tone present

		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     diseqc__diseqc_rx_cntrl,
				     i1);

		i1 = (uint32_t)(params->rx_timeout);

		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     diseqc__rx_msg_tim,
				     i1);

		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     diseqc__diseqc_srst,
				     0);

		if (AVL_EC_OK == r)
		{
			chip->chip_priv->diseqc_op_status = avl62x1_dos_init;
		}
	}
	r |= avl_bsp_release_semaphore(&(chip->chip_priv->diseqc_sem));

	return (r);
}

uint16_t avl62x1_lock_tp(struct avl62x1_carrier_info *carrier_info,
			 struct avl62x1_stream_info *stream_info,
			 avl_bool_t blind_sym_rate,
			 struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;

	r |= __avl62x1_send_cmd(CMD_HALT, chip);

	if (chip->chip_priv->agc_driven == 0)
	{
		r |= __avl62x1_drive_agc(avl62x1_on, chip);
	}

	if (blind_sym_rate)
	{
		r |= avl_bms_write8(chip->chip_pub->i2c_addr,
				    c_AVL62X1_S2X_blind_sym_rate_enable_caddr, 1);
	}
	else
	{
		r |= avl_bms_write8(chip->chip_pub->i2c_addr,
				    c_AVL62X1_S2X_blind_sym_rate_enable_caddr, 0);
	}

	r |= avl_bms_write8(chip->chip_pub->i2c_addr,
			    c_AVL62X1_S2X_blind_cfo_enable_caddr, 1);

	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     c_AVL62X1_S2X_nom_symbol_rate_Hz_iaddr,
			     carrier_info->symbol_rate_hz);

	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     c_AVL62X1_S2X_nom_carrier_freq_Hz_iaddr,
			     carrier_info->carrier_freq_offset_hz);

	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     c_AVL62X1_S2X_ConfiguredPLScramKey_iaddr,
			     carrier_info->pl_scrambling);

	if (stream_info != nullptr)
	{
		r |= avl_bms_write8(chip->chip_pub->i2c_addr,
				    c_AVL62X1_SP_S2X_ConfiguredStreamType_caddr,
				    stream_info->stream_type);
		r |= avl_bms_write8(chip->chip_pub->i2c_addr,
				    c_AVL62X1_SP_S2X_sp_wanted_stream_id_caddr,
				    stream_info->isi);
		r |= avl_bms_write8(chip->chip_pub->i2c_addr,
				    c_AVL62X1_SP_S2X_PLP_ID_caddr,
				    stream_info->t2mi.plp_id);
		if(stream_info->t2mi.pid != 0) {
			r |= avl_bms_write16(chip->chip_pub->i2c_addr,
				     c_AVL62X1_SP_S2X_sp_t2mi_ts_pid_cur_saddr,
				     stream_info->t2mi.pid);
		} else {
			r |= avl_bms_write16(chip->chip_pub->i2c_addr,
				     c_AVL62X1_SP_S2X_sp_t2mi_ts_pid_cur_saddr,
				     0x1000);
		}
		r |= avl_bms_write8(chip->chip_pub->i2c_addr,
				    c_AVL62X1_SP_S2X_sp_t2mi_ts_pid_detect_auto_en_caddr,
				    stream_info->t2mi.pid_autodiscover);
		r |= avl_bms_write8(chip->chip_pub->i2c_addr,
				    c_AVL62X1_SP_S2X_sp_raw_t2mi_mode_caddr,
				    stream_info->t2mi.raw_mode);
	}
	else
	{
		r |= avl_bms_write8(chip->chip_pub->i2c_addr,
				    c_AVL62X1_SP_S2X_ConfiguredStreamType_caddr,
				    avl62x1_undetermined);
		r |= avl_bms_write8(chip->chip_pub->i2c_addr,
				    c_AVL62X1_SP_S2X_sp_wanted_stream_id_caddr, 0);
		r |= avl_bms_write8(chip->chip_pub->i2c_addr,
				    c_AVL62X1_SP_S2X_sp_raw_t2mi_mode_caddr, 0);
	}

	r |= __avl62x1_send_cmd(CMD_ACQUIRE, chip);

	return (r);
}

uint16_t avl62x1_get_lock_status(enum avl62x1_lock_status *lock_status,
				 struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	uint8_t status = 0;

	r |= avl_bms_read8(chip->chip_pub->i2c_addr,
			   s_AVL62X1_S2X_sp_lock_caddr,
			   &status);
	if (status != 0)
	{
		*lock_status = avl62x1_status_locked;
	}
	else
	{
		*lock_status = avl62x1_status_unlocked;
	}

	return (r);
}

uint16_t avl62x1_get_lost_lock_status(
    enum avl62x1_lost_lock_status *lost_lock_status,
    struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	uint8_t lost_lock = 0;

	r |= avl_bms_read8(chip->chip_pub->i2c_addr,
			   s_AVL62X1_S2X_lost_lock_caddr,
			   &lost_lock);
	if (lost_lock != 0)
	{
		*lost_lock_status = avl62x1_lost_lock_yes;
		r |= avl_bms_write8(chip->chip_pub->i2c_addr,
				    s_AVL62X1_S2X_lost_lock_caddr,
				    0);
	}
	else
	{
		*lost_lock_status = avl62x1_lost_lock_no;
	}

	return (r);
}

uint16_t avl62x1_discover_streams(
    struct avl62x1_carrier_info *carrier_info,
    avl_bool_t blind_sym_rate,
    struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	r |= avl62x1_lock_tp(carrier_info, nullptr, blind_sym_rate, chip);
	return (r);
}

uint16_t avl62x1_get_discovery_status(
    enum avl62x1_discovery_status *status,
    struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	uint8_t ds = 0;

	r |= avl_bms_read8(chip->chip_pub->i2c_addr,
			   s_AVL62X1_SP_S2X_sp_stream_discover_done_caddr,
			   &ds);
	if (ds != 0)
	{
		*status = avl62x1_discovery_finished;
	}
	else
	{
		*status = avl62x1_discovery_running;
	}
	return (r);
}

uint16_t avl62x1_get_num_streams(uint8_t *num_streams,
				 struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;

	r |= avl_bms_read8(chip->chip_pub->i2c_addr,
			   s_AVL62X1_SP_S2X_sp_NumStreams_cur_TP_caddr,
			   num_streams);

	return (r);
}

uint16_t avl62x1_get_stream_list(
    struct avl62x1_stream_info *streams,
    const uint8_t max_streams,
    struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	uint32_t stream_list_ptr = 0;
	uint32_t stream_list_ptr_2 = 0;
	uint8_t num_streams = 0;
	uint8_t stream = 0;
	uint8_t tmp8 = 0;
	uint16_t tmp16 = 0;

	r |= avl_bms_read32(chip->chip_pub->i2c_addr,
			    s_AVL62X1_SP_S2X_sp_DVB_STREAM_addr_iaddr,
			    &stream_list_ptr);
	
	//this list is just an array of structs which currently
	//only contains a uint16_t T2MI_PID
	r |= avl_bms_read32(chip->chip_pub->i2c_addr,
			    s_AVL62X1_SP_S2X_sp_DVB_STREAM2_addr_iaddr,
			    &stream_list_ptr_2);

	r |= avl62x1_get_num_streams(&num_streams, chip);
	if (num_streams > max_streams)
	{
		num_streams = max_streams;
	}
	for (stream = 0; stream < num_streams; stream++)
	{
		//TODO: this could be optimized to a single I2C burst read
		r |= avl_bms_read8(chip->chip_pub->i2c_addr,
				   stream_list_ptr +
				       stream * AVL62X1_DVB_STREAM_struct_size +
				       AVL62X1_DVB_STREAM_CarrierIndex_caddr,
				   &tmp8);
		streams[stream].carrier_idx = tmp8;

		r |= avl_bms_read8(chip->chip_pub->i2c_addr,
				   stream_list_ptr +
				       stream * AVL62X1_DVB_STREAM_struct_size +
				       AVL62X1_DVB_STREAM_StreamType_caddr,
				   &tmp8);
		streams[stream].stream_type = (avl62x1_dvb_stream_type)tmp8;

		r |= avl_bms_read8(chip->chip_pub->i2c_addr,
				   stream_list_ptr +
				       stream * AVL62X1_DVB_STREAM_struct_size +
				       AVL62X1_DVB_STREAM_ISI_caddr,
				   &tmp8);
		streams[stream].isi = tmp8;

		if (streams[stream].stream_type == avl62x1_t2mi)
		{
			r |= avl_bms_read16(chip->chip_pub->i2c_addr,
					    stream_list_ptr_2 + stream * 4,
					    &tmp16);
			streams[stream].t2mi.pid = tmp16;

			r |= avl62x1_switch_stream(&streams[stream],chip);
			avl_bsp_delay(500);
			r |= avl62x1_get_t2mi_plp_list(
			    &streams[stream].t2mi.plp_list,
			    chip);
		}

	}

	return (r);
}

uint16_t avl62x1_switch_stream(struct avl62x1_stream_info *stream_info,
			       struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	uint8_t lock_status = 0;

	r |= avl_bms_read8(chip->chip_pub->i2c_addr,
			   s_AVL62X1_S2X_fec_lock_caddr,
			   &lock_status);
	if (lock_status != 0)
	{
		r |= __avl62x1_send_sp_cmd(SP_CMD_HALT, chip);
		r |= avl_bms_write8(chip->chip_pub->i2c_addr,
				    c_AVL62X1_SP_S2X_ConfiguredStreamType_caddr,
				    stream_info->stream_type);
		r |= avl_bms_write8(chip->chip_pub->i2c_addr,
				    c_AVL62X1_SP_S2X_sp_wanted_stream_id_caddr,
				    stream_info->isi);
		r |= avl_bms_write8(chip->chip_pub->i2c_addr,
				    c_AVL62X1_SP_S2X_PLP_ID_caddr,
				    stream_info->t2mi.plp_id);
		if(stream_info->t2mi.pid != 0) {
			r |= avl_bms_write16(chip->chip_pub->i2c_addr,
				     c_AVL62X1_SP_S2X_sp_t2mi_ts_pid_cur_saddr,
				     stream_info->t2mi.pid);
		} else {
			r |= avl_bms_write16(chip->chip_pub->i2c_addr,
				     c_AVL62X1_SP_S2X_sp_t2mi_ts_pid_cur_saddr,
				     0x1000);
		}
		r |= avl_bms_write8(chip->chip_pub->i2c_addr,
				    c_AVL62X1_SP_S2X_sp_t2mi_ts_pid_detect_auto_en_caddr,
				    stream_info->t2mi.pid_autodiscover);
		r |= avl_bms_write8(chip->chip_pub->i2c_addr,
				    c_AVL62X1_SP_S2X_sp_raw_t2mi_mode_caddr,
				    stream_info->t2mi.raw_mode);

		r |= avl_bms_write8(chip->chip_pub->i2c_addr,
				    s_AVL62X1_S2X_sp_lock_caddr, 0);
		r |= __avl62x1_send_sp_cmd(SP_CMD_ACQUIRE, chip);
	}
	else
	{
		r |= AVL_EC_GENERAL_FAIL; // demod isn't locked.
	}

	return (r);
}

uint16_t avl62x1_get_snr(int16_t *snr_x100db, struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;

	r |= avl_bms_read16(chip->chip_pub->i2c_addr,
			    s_AVL62X1_S2X_snr_dB_x100_saddr,
			    (uint16_t *)snr_x100db);

	return (r);
}

uint16_t avl62x1_get_signal_info(struct avl62x1_carrier_info *carrier_info,
				 struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	uint8_t tmp8 = 0;
	uint32_t tmp32 = 0;

	/* If blind carrier freq search was performed,
	 * this is the carrier freq as determined by the
	 * blind search algorithm. Otherwise, it is just
	 * the nominal carrier freq from the config.
	 */
	r |= avl_bms_read32(chip->chip_pub->i2c_addr,
			    s_AVL62X1_S2X_carrier_freq_Hz_iaddr,
			    &tmp32);
	carrier_info->carrier_freq_offset_hz = (int32_t)tmp32;

	/* Difference, in Hertz, between nominal carrier
	 * freq and current freq as indicated by the
	 * frequency loop.
	 */
	r |= avl_bms_read32(chip->chip_pub->i2c_addr,
			    s_AVL62X1_S2X_carrier_freq_err_Hz_iaddr,
			    &tmp32);
	carrier_info->carrier_freq_offset_hz += (int32_t)tmp32;
	carrier_info->carrier_freq_offset_hz -=
	    chip->chip_priv->carrier_freq_offset_hz;

	r |= avl_bms_read32(chip->chip_pub->i2c_addr,
			    s_AVL62X1_S2X_symbol_rate_Hz_iaddr,
			    &tmp32);
	carrier_info->symbol_rate_hz = tmp32;

	r |= avl_bms_read8(chip->chip_pub->i2c_addr,
			   s_AVL62X1_S2X_signal_type_caddr,
			   &tmp8);
	carrier_info->signal_type = (avl62x1_standard)(tmp8);

	r |= avl_bms_read8(chip->chip_pub->i2c_addr,
			   s_AVL62X1_S2X_carrier_spectrum_invert_status_caddr,
			   &tmp8);
	carrier_info->spectrum_invert = (tmp8 == 1)
					    ? avl62x1_specpol_inverted
					    : avl62x1_specpol_normal;

	if (carrier_info->signal_type == avl62x1_dvbs)
	{
		carrier_info->modulation = avl62x1_qpsk;

		r |= avl_bms_read8(chip->chip_pub->i2c_addr,
				   s_AVL62X1_S2X_dvbs_code_rate_caddr,
				   &tmp8);
		carrier_info->code_rate.dvbs_code_rate =
		    (avl62x1_dvbs_code_rate)(tmp8);

		carrier_info->roll_off = avl62x1_rolloff_35;
	}
	else
	{
		r |= avl_bms_read8(chip->chip_pub->i2c_addr,
				   s_AVL62X1_S2X_s2_pilot_on_caddr,
				   &tmp8);
		carrier_info->pilot = (avl62x1_pilot)(tmp8);

		r |= avl_bms_read8(chip->chip_pub->i2c_addr,
				   s_AVL62X1_S2X_s2_fec_len_caddr,
				   &tmp8);
		carrier_info->fec_length = (avl62x1_fec_length)(tmp8);

		r |= avl_bms_read8(chip->chip_pub->i2c_addr,
				   s_AVL62X1_S2X_s2_modulation_caddr,
				   &tmp8);
		carrier_info->modulation = (avl62x1_modulation_mode)(tmp8);

		r |= avl_bms_read8(chip->chip_pub->i2c_addr,
				   s_AVL62X1_S2X_s2_code_rate_caddr,
				   &tmp8);
		carrier_info->code_rate.dvbs2_code_rate =
		    (avl62x1_dvbs2_code_rate)(tmp8);

		r |= avl_bms_read8(chip->chip_pub->i2c_addr,
				   s_AVL62X1_S2X_ccm1_acm0_caddr,
				   &tmp8);
		if (tmp8 == 0)
		{
			carrier_info->dvbs2_ccm_acm = avl62x1_dvbs2_acm;
			carrier_info->pls_acm = 0;
		}
		else
		{
			carrier_info->dvbs2_ccm_acm = avl62x1_dvbs2_ccm;
			r |= avl_bms_read8(chip->chip_pub->i2c_addr,
					   s_AVL62X1_S2X_ccm_pls_mode_caddr,
					   &tmp8);
			carrier_info->pls_acm = tmp8;
		}

		r |= avl_bms_read8(chip->chip_pub->i2c_addr,
				   s_AVL62X1_S2X_alpha_caddr,
				   &tmp8);
		carrier_info->roll_off = (avl62x1_rolloff)(tmp8);

		//r |= AVL_AVL62X1_GetStreamNumber(&carrier_info->num_streams, chip);
		r |= avl_bms_read8(chip->chip_pub->i2c_addr,
				   s_AVL62X1_SP_S2X_sp_SIS_MIS_caddr,
				   &tmp8);
		carrier_info->sis_mis = (avl62x1_sis_mis)(tmp8);
	}
	return (r);
}

uint16_t avl62x1_get_signal_strength(uint16_t *signal_strength,
				     struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	uint16_t tmp16 = 0;

	r = avl_bms_read16(chip->chip_pub->i2c_addr,
			   s_AVL62X1_DMD_rfagc_gain_saddr,
			   &tmp16);
	if (tmp16 <= 25000)
	{
		*signal_strength = 100;
	}
	else if (tmp16 >= 55000)
	{
		*signal_strength = 10;
	}
	else
	{
		*signal_strength = (55000 - tmp16) * 90 / 30000 + 10;
	}

	return (r);
}

uint16_t avl62x1_get_signal_quality(uint16_t *signal_quality,
				    struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	int16_t tmp = 0;

	r = avl62x1_get_snr(&tmp, chip);
	if (tmp >= 2500)
	{
		*signal_quality = 100;
	}
	else if (tmp <= 0)
	{
		*signal_quality = 10;
	}
	else
	{
		*signal_quality = tmp * 90 / 2500 + 10;
	}

	return (r);
}

uint16_t avl62x1_reset_per(struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	uint32_t tmp = 0;

	chip->chip_priv->error_stats.sw_cnt_num_pkts.high_word = 0;
	chip->chip_priv->error_stats.sw_cnt_num_pkts.low_word = 0;
	chip->chip_priv->error_stats.sw_cnt_pkt_errs.high_word = 0;
	chip->chip_priv->error_stats.sw_cnt_pkt_errs.low_word = 0;
	chip->chip_priv->error_stats.num_pkts.high_word = 0;
	chip->chip_priv->error_stats.num_pkts.low_word = 0;
	chip->chip_priv->error_stats.pkt_errs.high_word = 0;
	chip->chip_priv->error_stats.pkt_errs.low_word = 0;
	chip->chip_priv->error_stats.per_x1e9 = 0;

	r |= avl_bms_read32(chip->chip_pub->i2c_addr, ber_esm__esm_cntrl, &tmp);
	tmp |= 0x00000001;
	r |= avl_bms_write32(chip->chip_pub->i2c_addr, ber_esm__esm_cntrl, tmp);

	r |= avl_bms_read32(chip->chip_pub->i2c_addr, ber_esm__esm_cntrl, &tmp);
	tmp |= 0x00000008;
	r |= avl_bms_write32(chip->chip_pub->i2c_addr, ber_esm__esm_cntrl, tmp);
	tmp |= 0x00000001;
	r |= avl_bms_write32(chip->chip_pub->i2c_addr, ber_esm__esm_cntrl, tmp);
	tmp &= 0xFFFFFFFE;
	r |= avl_bms_write32(chip->chip_pub->i2c_addr, ber_esm__esm_cntrl, tmp);

	return (r);
}

uint16_t avl62x1_get_per(uint32_t *per_x1e9, struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	enum avl62x1_lock_status lock_status = avl62x1_status_unlocked;
	uint32_t hw_cnt_pkt_errs = 0;
	uint32_t hw_cnt_num_pkts = 0;
	uint32_t tmp32 = 0;
	struct avl_uint64 tmp64 = {0, 0};

	r |= avl62x1_get_lock_status(&lock_status, chip);

	//record the lock status before return the PER
	if (avl62x1_status_locked == lock_status)
	{
		chip->chip_priv->error_stats.lost_lock = 0;
	}
	else
	{
		chip->chip_priv->error_stats.lost_lock = 1;
		return *per_x1e9 = AVL_CONSTANT_10_TO_THE_9TH;
	}

	r = avl_bms_read32(chip->chip_pub->i2c_addr,
			   ber_esm__packet_err_cnt,
			   &hw_cnt_pkt_errs);
	r |= avl_bms_read32(chip->chip_pub->i2c_addr,
			    ber_esm__packet_num,
			    &hw_cnt_num_pkts);

	if (hw_cnt_num_pkts > (1 << 30))
	{
		r |= avl_bms_read32(chip->chip_pub->i2c_addr,
				    ber_esm__esm_cntrl,
				    &tmp32);
		tmp32 |= 0x00000001;
		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     ber_esm__esm_cntrl,
				     tmp32);
		r |= avl_bms_read32(chip->chip_pub->i2c_addr,
				    ber_esm__packet_err_cnt,
				    &hw_cnt_pkt_errs);
		r |= avl_bms_read32(chip->chip_pub->i2c_addr,
				    ber_esm__packet_num,
				    &hw_cnt_num_pkts);
		tmp32 &= 0xFFFFFFFE;
		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     ber_esm__esm_cntrl,
				     tmp32);
		avl_add_32to64(&chip->chip_priv->error_stats.sw_cnt_num_pkts,
			       hw_cnt_num_pkts);
		avl_add_32to64(&chip->chip_priv->error_stats.sw_cnt_pkt_errs,
			       hw_cnt_pkt_errs);
		hw_cnt_num_pkts = 0;
		hw_cnt_pkt_errs = 0;
	}

	chip->chip_priv->error_stats.num_pkts.high_word =
	    chip->chip_priv->error_stats.sw_cnt_num_pkts.high_word;
	chip->chip_priv->error_stats.num_pkts.low_word =
	    chip->chip_priv->error_stats.sw_cnt_num_pkts.low_word;
	avl_add_32to64(&chip->chip_priv->error_stats.num_pkts,
		       hw_cnt_num_pkts);
	chip->chip_priv->error_stats.pkt_errs.high_word =
	    chip->chip_priv->error_stats.sw_cnt_pkt_errs.high_word;
	chip->chip_priv->error_stats.pkt_errs.low_word =
	    chip->chip_priv->error_stats.sw_cnt_pkt_errs.low_word;
	avl_add_32to64(&chip->chip_priv->error_stats.pkt_errs,
		       hw_cnt_pkt_errs);

	// Compute the PER
	avl_mult_32to64(&tmp64,
			chip->chip_priv->error_stats.pkt_errs.low_word,
			AVL_CONSTANT_10_TO_THE_9TH);
	chip->chip_priv->error_stats.per_x1e9 =
	    avl_divide_64(chip->chip_priv->error_stats.num_pkts,
			  tmp64);
	//keep the PER user wanted
	*per_x1e9 = chip->chip_priv->error_stats.per_x1e9;

	return (r);
}

/************************************************************************/
/* Diseqc                                                               */
/************************************************************************/
uint16_t avl62x1_receive_diseqc_data(uint8_t *buf,
				     uint8_t *size,
				     struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	uint32_t i1 = 0;
	uint32_t i2 = 0;
	uint8_t buf_tmp[4] = {0};

	r = avl_bsp_wait_semaphore(&(chip->chip_priv->diseqc_sem));
	r |= avl_bms_read32(chip->chip_pub->i2c_addr,
			    diseqc__diseqc_rx_st,
			    &i1);
	r |= avl_bms_read32(chip->chip_pub->i2c_addr,
			    diseqc__diseqc_tx_cntrl,
			    &i2);
	if ((i2 >> 8) & 0x01)
	{
		chip->chip_priv->diseqc_op_status = avl62x1_dos_modulation;
	}
	if (avl62x1_dos_modulation == chip->chip_priv->diseqc_op_status)
	{
		// In modulation mode
		if ((!((i2 >> 8) & 0x01) && (0x00000004 == (i1 & 0x00000004))) ||
		    (((i2 >> 8) & 0x01) && (0x00000004 != (i1 & 0x00000004))))
		{
			*size = (uint8_t)((i1 & 0x00000078) >> 3);
			//Receive data
			for (i1 = 0; i1 < *size; i1++)
			{
				r |= avl_bms_read(chip->chip_pub->i2c_addr,
						  diseqc__rx_fifo,
						  buf_tmp, 4);
				buf[i1] = buf_tmp[3];
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

	r |= avl_bsp_release_semaphore(&(chip->chip_priv->diseqc_sem));

	return (r);
}

uint16_t avl62x1_send_diseqc_data(const uint8_t *buf,
				  uint8_t size,
				  struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	uint32_t i1 = 0;
	uint32_t i2 = 0;
	uint8_t buf_tmp[8] = {0};
	uint8_t cont_flag = 0;
	uint16_t cnt = 0;

	if (size > 8)
	{
		return AVL_EC_MemoryRunout;
	}

	r = avl_bsp_wait_semaphore(&(chip->chip_priv->diseqc_sem));
	r |= __avl62x1_diseqc_mode_switch_check(chip);
	if (AVL_EC_OK == r)
	{
		if (chip->chip_priv->diseqc_op_status == avl62x1_dos_continuous)
		{
			r |= avl_bms_read32(chip->chip_pub->i2c_addr,
					    diseqc__diseqc_tx_cntrl,
					    &i1);
			if ((i1 >> 10) & 0x01)
			{
				cont_flag = 1;
				i1 &= 0xfffff3ff;
				r |= avl_bms_write32(chip->chip_pub->i2c_addr,
						     diseqc__diseqc_tx_cntrl,
						     i1);
				r |= avl_bsp_delay(AVL62X1_DISEQC_DELAY);
			}
		}
		//reset rx_fifo
		r |= avl_bms_read32(chip->chip_pub->i2c_addr,
				    diseqc__diseqc_rx_cntrl,
				    &i2);
		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     diseqc__diseqc_rx_cntrl,
				     (i2 | 0x01));
		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     diseqc__diseqc_rx_cntrl,
				     (i2 & 0xfffffffe));

		r |= avl_bms_read32(chip->chip_pub->i2c_addr,
				    diseqc__diseqc_tx_cntrl,
				    &i1);

		//set to modulation mode and put it to FIFO load mode
		i1 &= 0xfffffff8;
		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     diseqc__diseqc_tx_cntrl,
				     i1);

		avl_int_to_3bytes(diseqc__tx_fifo, buf_tmp);
		buf_tmp[3] = 0;
		buf_tmp[4] = 0;
		buf_tmp[5] = 0;
		for (i2 = 0; i2 < size; i2++)
		{
			buf_tmp[6] = buf[i2];

			r |= avl_bms_write(chip->chip_pub->i2c_addr,
					   buf_tmp, 7);
		}
		i1 |= (1 << 2); //start fifo transmit.
		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     diseqc__diseqc_tx_cntrl,
				     i1);

		if (AVL_EC_OK == r)
		{
			chip->chip_priv->diseqc_op_status = avl62x1_dos_modulation;
		}
		do
		{
			r |= avl_bsp_delay(1);
			if (++cnt > 500)
			{
				r |= AVL_EC_TimeOut;
				r |= avl_bsp_release_semaphore(&(chip->chip_priv->diseqc_sem));
				return (r);
			}
			r = avl_bms_read32(chip->chip_pub->i2c_addr,
					   diseqc__diseqc_tx_st,
					   &i1);
		} while (1 != ((i1 & 0x00000040) >> 6));

		r = avl_bsp_delay(AVL62X1_DISEQC_DELAY);
		if (cont_flag == 1) //resume to send out wave
		{
			//No data in FIFO
			r |= avl_bms_read32(chip->chip_pub->i2c_addr,
					    diseqc__diseqc_tx_cntrl,
					    &i1);
			i1 &= 0xfffffff8;
			i1 |= 0x03; //switch to continuous mode
			r |= avl_bms_write32(chip->chip_pub->i2c_addr,
					     diseqc__diseqc_tx_cntrl,
					     i1);

			//start to send out wave
			i1 |= (1 << 10);
			r |= avl_bms_write32(chip->chip_pub->i2c_addr,
					     diseqc__diseqc_tx_cntrl,
					     i1);
			if (AVL_EC_OK == r)
			{
				chip->chip_priv->diseqc_op_status =
				    avl62x1_dos_continuous;
			}
		}
	}
	r |= avl_bsp_release_semaphore(&(chip->chip_priv->diseqc_sem));

	return (r);
}

uint16_t avl62x1_get_diseqc_tx_status(struct avl62x1_diseqc_tx_status *status,
				      struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	uint32_t i1 = 0;

	r = avl_bsp_wait_semaphore(&(chip->chip_priv->diseqc_sem));
	if ((avl62x1_dos_modulation == chip->chip_priv->diseqc_op_status) ||
	    (avl62x1_dos_tone == chip->chip_priv->diseqc_op_status))
	{
		r |= avl_bms_read32(chip->chip_pub->i2c_addr,
				    diseqc__diseqc_tx_st,
				    &i1);
		status->tx_complete = (uint8_t)((i1 & 0x00000040) >> 6);
		status->tx_fifo_count = (uint8_t)((i1 & 0x0000003c) >> 2);
	}
	else
	{
		r |= AVL_EC_GENERAL_FAIL;
	}
	r |= avl_bsp_release_semaphore(&(chip->chip_priv->diseqc_sem));

	return (r);
}

uint16_t avl62x1_get_diseqc_rx_status(struct avl62x1_diseqc_rx_status *status,
				      struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	uint32_t i1 = 0;

	r = avl_bsp_wait_semaphore(&(chip->chip_priv->diseqc_sem));
	if (avl62x1_dos_modulation == chip->chip_priv->diseqc_op_status)
	{
		r |= avl_bms_read32(chip->chip_pub->i2c_addr,
				    diseqc__diseqc_rx_st,
				    &i1);
		status->rx_complete = (uint8_t)((i1 & 0x00000004) >> 2);
		status->rx_fifo_count = (uint8_t)((i1 & 0x000000078) >> 3);
	}
	else
	{
		r |= AVL_EC_GENERAL_FAIL;
	}
	r |= avl_bsp_release_semaphore(&(chip->chip_priv->diseqc_sem));

	return (r);
}

uint16_t avl62x1_send_diseqc_tone(uint8_t tone,
				  uint8_t count,
				  struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	uint32_t i1 = 0;
	uint32_t i2 = 0;
	uint8_t buf_tmp[8];
	uint8_t cont_flag = 0;
	uint16_t cnt = 0;

	if (count > 8)
	{
		return AVL_EC_MemoryRunout;
	}

	r = avl_bsp_wait_semaphore(&(chip->chip_priv->diseqc_sem));
	r |= __avl62x1_diseqc_mode_switch_check(chip);

	if (AVL_EC_OK == r)
	{
		if (chip->chip_priv->diseqc_op_status == avl62x1_dos_continuous)
		{
			r |= avl_bms_read32(chip->chip_pub->i2c_addr,
					    diseqc__diseqc_tx_cntrl,
					    &i1);
			if ((i1 >> 10) & 0x01)
			{
				cont_flag = 1;
				i1 &= 0xfffff3ff;
				r |= avl_bms_write32(chip->chip_pub->i2c_addr,
						     diseqc__diseqc_tx_cntrl,
						     i1);
				r |= avl_bsp_delay(AVL62X1_DISEQC_DELAY); //delay 20ms
			}
		}
		//No data in the FIFO.
		r |= avl_bms_read32(chip->chip_pub->i2c_addr,
				    diseqc__diseqc_tx_cntrl,
				    &i1);

		//put it into the FIFO load mode.
		i1 &= 0xfffffff8;
		if (0 == tone)
		{
			i1 |= 0x01;
		}
		else
		{
			i1 |= 0x02;
		}
		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     diseqc__diseqc_tx_cntrl,
				     i1);

		avl_int_to_3bytes(diseqc__tx_fifo, buf_tmp);
		buf_tmp[3] = 0;
		buf_tmp[4] = 0;
		buf_tmp[5] = 0;
		buf_tmp[6] = 1;

		for (i2 = 0; i2 < count; i2++)
		{
			r |= avl_bms_write(chip->chip_pub->i2c_addr,
					   buf_tmp, 7);
		}

		i1 |= (1 << 2); //start fifo transmit.
		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     diseqc__diseqc_tx_cntrl,
				     i1);
		if (AVL_EC_OK == r)
		{
			chip->chip_priv->diseqc_op_status = avl62x1_dos_tone;
		}
		do
		{
			r |= avl_bsp_delay(1);
			if (++cnt > 500)
			{
				r |= AVL_EC_TimeOut;
				r |= avl_bsp_release_semaphore(
				    &(chip->chip_priv->diseqc_sem));
				return (r);
			}
			r = avl_bms_read32(chip->chip_pub->i2c_addr,
					   diseqc__diseqc_tx_st,
					   &i1);
		} while (1 != ((i1 & 0x00000040) >> 6));

		r = avl_bsp_delay(AVL62X1_DISEQC_DELAY);
		if (cont_flag == 1) //resume to send out wave
		{
			//No data in FIFO
			r |= avl_bms_read32(chip->chip_pub->i2c_addr,
					    diseqc__diseqc_tx_cntrl,
					    &i1);
			i1 &= 0xfffffff8;
			i1 |= 0x03; //switch to continuous mode
			r |= avl_bms_write32(chip->chip_pub->i2c_addr,
					     diseqc__diseqc_tx_cntrl,
					     i1);

			//start to send out wave
			i1 |= (1 << 10);
			r |= avl_bms_write32(chip->chip_pub->i2c_addr,
					     diseqc__diseqc_tx_cntrl,
					     i1);
			if (AVL_EC_OK == r)
			{
				chip->chip_priv->diseqc_op_status =
				    avl62x1_dos_continuous;
			}
		}
	}
	r |= avl_bsp_release_semaphore(&(chip->chip_priv->diseqc_sem));

	return (r);
}

uint16_t avl62x1_diseqc_tone_on(struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	uint32_t i1 = 0;

	r = avl_bsp_wait_semaphore(&(chip->chip_priv->diseqc_sem));
	r |= __avl62x1_diseqc_mode_switch_check(chip);

	if (AVL_EC_OK == r)
	{
		//No data in FIFO
		r |= avl_bms_read32(chip->chip_pub->i2c_addr,
				    diseqc__diseqc_tx_cntrl,
				    &i1);
		i1 &= 0xfffffff8;
		i1 |= 0x03; //switch to continuous mode
		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     diseqc__diseqc_tx_cntrl,
				     i1);

		//start to send out wave
		i1 |= (1 << 10);
		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     diseqc__diseqc_tx_cntrl,
				     i1);
		if (AVL_EC_OK == r)
		{
			chip->chip_priv->diseqc_op_status =
			    avl62x1_dos_continuous;
		}
	}
	r |= avl_bsp_release_semaphore(&(chip->chip_priv->diseqc_sem));

	return (r);
}

uint16_t avl62x1_diseqc_tone_off(struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	uint32_t i1 = 0;

	r = avl_bsp_wait_semaphore(&(chip->chip_priv->diseqc_sem));
	if (avl62x1_dos_continuous == chip->chip_priv->diseqc_op_status)
	{
		r |= avl_bms_read32(chip->chip_pub->i2c_addr,
				    diseqc__diseqc_tx_cntrl,
				    &i1);
		i1 &= 0xfffff3ff;
		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     diseqc__diseqc_tx_cntrl,
				     i1);
		
		if(r == AVL_EC_OK)
		{
			chip->chip_priv->diseqc_op_status = avl62x1_dos_init;
		}
	}

	r |= avl_bsp_release_semaphore(&(chip->chip_priv->diseqc_sem));

	return (r);
}

/************************************************************************/
/* Tuner I2C                                                            */
/************************************************************************/
uint16_t avl62x1_enable_tuner_i2c(struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;

	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     psc_tuner_i2c__tuner_hw_i2c_bit_rpt_cntrl,
			     0x07);
	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     psc_tuner_i2c__tuner_hw_i2c_bit_rpt_cntrl,
			     0x07);
	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     psc_tuner_i2c__tuner_hw_i2c_bit_rpt_cntrl,
			     0x07);

	return (r);
}

uint16_t avl62x1_disable_tuner_i2c(struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;

	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     psc_tuner_i2c__tuner_hw_i2c_bit_rpt_cntrl,
			     0x06);
	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     psc_tuner_i2c__tuner_hw_i2c_bit_rpt_cntrl,
			     0x06);
	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     psc_tuner_i2c__tuner_hw_i2c_bit_rpt_cntrl,
			     0x06);

	return (r);
}

/************************************************************************/
/* GPIO                                                                 */
/************************************************************************/
uint16_t avl62x1_set_gpio_dir(enum avl62x1_gpio_pin pin,
			      enum avl62x1_gpio_pin_dir dir,
			      struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	uint32_t addr = gpio_debug__i2c_data2_sel;
	uint32_t data = avl62x1_gpio_value_logic_0;

	if (dir == avl62x1_gpio_dir_output)
	{
		data = avl62x1_gpio_value_logic_0;
	}
	else if (dir == avl62x1_gpio_dir_input)
	{
		data = avl62x1_gpio_value_high_z;
	}
	else
	{
		return AVL_EC_GENERAL_FAIL;
	}

	switch (pin)
	{
	case avl62x1_gpio_pin_tuner_sda:
		addr = gpio_debug__i2c_data2_sel;
		break;
	case avl62x1_gpio_pin_tuner_scl:
		addr = gpio_debug__i2c_clk2_sel;
		break;
	case avl62x1_gpio_pin_s_agc2:
		addr = gpio_debug__agc2_sel;
		break;
	case avl62x1_gpio_pin_lnb_pwr_en:
		addr = gpio_debug__lnb_cntrl_0_sel;
		break;
	case avl62x1_gpio_pin_lnb_pwr_sel:
		addr = gpio_debug__lnb_cntrl_1_sel;
		break;
	default:
		return AVL_EC_GENERAL_FAIL;
	}

	r = avl_bms_write32(chip->chip_pub->i2c_addr,
			    addr,
			    data);

	return (r);
}

uint16_t avl62x1_set_gpio_value(enum avl62x1_gpio_pin pin,
				enum avl62x1_gpio_pin_value val,
				struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	uint32_t addr = gpio_debug__i2c_data2_sel;

	if ((val != avl62x1_gpio_value_logic_0) &&
	    (val != avl62x1_gpio_value_logic_1) &&
	    (val != avl62x1_gpio_value_high_z))
	{
		return AVL_EC_GENERAL_FAIL;
	}

	switch (pin)
	{
	case avl62x1_gpio_pin_tuner_sda:
		addr = gpio_debug__i2c_data2_sel;
		break;
	case avl62x1_gpio_pin_tuner_scl:
		addr = gpio_debug__i2c_clk2_sel;
		break;
	case avl62x1_gpio_pin_s_agc2:
		addr = gpio_debug__agc2_sel;
		break;
	case avl62x1_gpio_pin_lnb_pwr_en:
		addr = gpio_debug__lnb_cntrl_0_sel;
		break;
	case avl62x1_gpio_pin_lnb_pwr_sel:
		addr = gpio_debug__lnb_cntrl_1_sel;
		break;
	default:
		return AVL_EC_GENERAL_FAIL;
	}
	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     addr,
			     (uint32_t)(val));
	return (r);
}

uint16_t avl62x1_get_gpio_value(enum avl62x1_gpio_pin pin,
				enum avl62x1_gpio_pin_value *val,
				struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	uint32_t tmp = 0;
	uint32_t addr = gpio_debug__i2c_data2_i;

	switch (pin)
	{
	case avl62x1_gpio_pin_tuner_sda:
		addr = gpio_debug__i2c_data2_i;
		break;
	case avl62x1_gpio_pin_tuner_scl:
		addr = gpio_debug__i2c_clk2_i;
		break;
	case avl62x1_gpio_pin_s_agc2:
		addr = gpio_debug__agc2_i;
		break;
	case avl62x1_gpio_pin_lnb_pwr_en:
		addr = gpio_debug__lnb_cntrl_0_i;
		break;
	case avl62x1_gpio_pin_lnb_pwr_sel:
		addr = gpio_debug__lnb_cntrl_1_i;
		break;
	default:
		return AVL_EC_GENERAL_FAIL;
	}

	r |= avl_bms_read32(chip->chip_pub->i2c_addr,
			    addr,
			    &tmp);
	*val = (avl62x1_gpio_pin_value)tmp;
	return (r);
}

/************************************************************************/
/* BlindScan                                                            */
/************************************************************************/
//Start the blind scan operation on a single tuner step
uint16_t avl62x1_blindscan_start(
    struct avl62x1_blind_scan_params *params,
    struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	uint32_t samp_rate_Hz = 0;
	uint16_t samp_rate_ratio = 0;

	if (chip->chip_priv->agc_driven == 0)
	{
		r |= __avl62x1_drive_agc(avl62x1_on, chip);
	}

	r |= avl_bms_read32(chip->chip_pub->i2c_addr,
			    c_AVL62X1_S2X_sample_rate_Hz_iaddr,
			    &samp_rate_Hz);

	if (samp_rate_Hz == 0)
	{
		samp_rate_ratio = (1 << 15);
	}
	else
	{
		//TunerLPF is single-sided, ratio based on double-sided
		samp_rate_ratio = (uint16_t)(
		    ((uint32_t)(2 * params->tuner_lpf_100khz) * (1 << 15)) /
		    (samp_rate_Hz / 100000));
	}

	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     c_AVL62X1_S2X_nom_carrier_freq_Hz_iaddr,
			     0);

	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     c_AVL62X1_S2X_ConfiguredPLScramKey_iaddr,
			     AVL62X1_PL_SCRAM_AUTO);

	r |= avl_bms_write16(chip->chip_pub->i2c_addr,
			     c_AVL62X1_S2X_BWfilt_to_Rsamp_ratio_saddr,
			     samp_rate_ratio);

	r |= avl_bms_write8(chip->chip_pub->i2c_addr,
			    c_AVL62X1_S2X_search_range_percent_caddr,
			    90);

	r |= avl_bms_write16(chip->chip_pub->i2c_addr,
			     c_AVL62X1_S2X_blind_scan_min_sym_rate_kHz_saddr,
			     params->min_symrate_khz);

	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     c_AVL62X1_S2X_bs_cent_freq_tuner_Hz_iaddr,
			     params->tuner_center_freq_100khz * 100 * 1000);

	r |= __avl62x1_send_cmd(CMD_BLIND_SCAN, chip);

	return (r);
}

//Cancel blind scan process
uint16_t avl62x1_blindscan_cancel(struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;

	r |= __avl62x1_send_cmd(CMD_HALT, chip);

	return (r);
}

//Get the status of a currently-running blind scan operation (single tuner step)
uint16_t avl62x1_blindscan_get_status(
    struct avl62x1_blind_scan_info *info,
    struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	uint8_t tmp8 = 0;
	uint8_t tmp8_1 = 0;
	uint32_t tmp32 = 0;
	enum avl62x1_functional_mode func_mode;

	r |= avl_bms_read8(chip->chip_pub->i2c_addr,
			   s_AVL62X1_S2X_current_bs_pair_index_caddr,
			   &tmp8);

	r |= avl_bms_read8(chip->chip_pub->i2c_addr,
			   s_AVL62X1_S2X_bs_num_carrier_candidates_caddr,
			   &tmp8_1);

	if (tmp8_1 == 0)
	{
		info->progress = 0;
	}
	else
	{
		info->progress = (uint8_t)((100 * (uint32_t)tmp8) /
					   (uint32_t)tmp8_1);
	}

	r |= avl_bms_read8(chip->chip_pub->i2c_addr,
			   s_AVL62X1_S2X_bs_num_confirmed_carriers_caddr,
			   &tmp8);

	if (tmp8 == 255)
	{
		info->num_carriers = 0;
	}
	else
	{
		info->num_carriers = tmp8;
	}

	r |= __avl62x1_get_func_mode(&func_mode, chip);
	//idle and num confirmed carriers not init value
	info->finished = (func_mode == avl62x1_funcmode_idle) &&
			 (tmp8 != 255);

	//info->num_streams = 0; //DEPRECATED

	r |= avl_bms_read32(chip->chip_pub->i2c_addr,
			    s_AVL62X1_S2X_bs_next_start_freq_Hz_iaddr,
			    &tmp32);

	info->next_freq_step_hz = tmp32;

	return (r);
}

uint16_t avl62x1_blindscan_get_carrier_list(
    const struct avl62x1_blind_scan_params *params,
    struct avl62x1_blind_scan_info *info,
    struct avl62x1_carrier_info *carriers,
    struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	uint8_t carrier = 0;
	uint8_t tmp8 = 0;
	uint16_t tmp16 = 0;
	uint32_t carrier_list_ptr = 0;
	uint32_t tmp32 = 0;

	avl62x1_blindscan_get_status(info, chip);
	if (info->finished == 0)
	{
		r = AVL_EC_RUNNING;
		return (r);
	}

	r |= avl_bms_read32(chip->chip_pub->i2c_addr,
			    s_AVL62X1_S2X_bs_carrier_list_address_iaddr,
			    &carrier_list_ptr);

	for (carrier = 0; carrier < info->num_carriers; carrier++)
	{
		//TODO: this could be optimized to a single I2C burst read
		carriers[carrier].pl_scrambling = AVL62X1_PL_SCRAM_AUTO;
		carriers[carrier].carrier_idx = carrier;
		carriers[carrier].roll_off = avl62x1_rolloff_unknown;

		r |= avl_bms_read32(chip->chip_pub->i2c_addr,
				    carrier_list_ptr +
					carrier * AVL62X1_SAT_CARRIER_struct_size +
					AVL62X1_SAT_CARRIER_CarrierFreqHz_iaddr,
				    &tmp32);

		carriers[carrier].carrier_freq_offset_hz = 0;
		carriers[carrier].rf_freq_khz =
		    ((int32_t)params->tuner_center_freq_100khz * 100) +
		    (((int32_t)tmp32 + 500) / 1000);

		r |= avl_bms_read32(chip->chip_pub->i2c_addr,
				    carrier_list_ptr +
					carrier * AVL62X1_SAT_CARRIER_struct_size +
					AVL62X1_SAT_CARRIER_SymbolRateHz_iaddr,
				    &tmp32);
		carriers[carrier].symbol_rate_hz = tmp32;

		r |= avl_bms_read8(chip->chip_pub->i2c_addr,
				   carrier_list_ptr +
				       carrier * AVL62X1_SAT_CARRIER_struct_size +
				       AVL62X1_SAT_CARRIER_SignalType_caddr,
				   &tmp8);
		carriers[carrier].signal_type = (avl62x1_standard)tmp8;

		carriers[carrier].num_streams = 0;
		carriers[carrier].sis_mis = avl62x1_sis_mis_unknown;

		r |= avl_bms_read16(chip->chip_pub->i2c_addr,
				    carrier_list_ptr +
					carrier * AVL62X1_SAT_CARRIER_struct_size +
					AVL62X1_SAT_CARRIER_SNR_dB_x100_saddr,
				    &tmp16);
		carriers[carrier].snr_db_x100 = tmp16;

		if (carriers[carrier].signal_type == avl62x1_dvbs2)
		{
			r |= avl_bms_read8(chip->chip_pub->i2c_addr,
					   carrier_list_ptr +
					       carrier * AVL62X1_SAT_CARRIER_struct_size +
					       AVL62X1_SAT_CARRIER_PLS_ACM_caddr,
					   &tmp8);
			carriers[carrier].pls_acm = tmp8;
			carriers[carrier].dvbs2_ccm_acm =
			    (avl62x1_dvbs2_ccm_acm)(carriers[carrier].pls_acm != 0);

			r |= avl_bms_read8(chip->chip_pub->i2c_addr,
					   carrier_list_ptr +
					       carrier * AVL62X1_SAT_CARRIER_struct_size +
					       AVL62X1_SAT_CARRIER_Mod_caddr,
					   &tmp8);
			carriers[carrier].modulation = (avl62x1_modulation_mode)tmp8;

			r |= avl_bms_read8(chip->chip_pub->i2c_addr,
					   carrier_list_ptr +
					       carrier * AVL62X1_SAT_CARRIER_struct_size +
					       AVL62X1_SAT_CARRIER_Pilot_caddr,
					   &tmp8);
			carriers[carrier].pilot = (avl62x1_pilot)tmp8;

			r |= avl_bms_read8(chip->chip_pub->i2c_addr,
					   carrier_list_ptr +
					       carrier * AVL62X1_SAT_CARRIER_struct_size +
					       AVL62X1_SAT_CARRIER_FECLen_caddr,
					   &tmp8);
			carriers[carrier].fec_length = (avl62x1_fec_length)tmp8;

			r |= avl_bms_read8(chip->chip_pub->i2c_addr,
					   carrier_list_ptr +
					       carrier * AVL62X1_SAT_CARRIER_struct_size +
					       AVL62X1_SAT_CARRIER_CodeRate_caddr,
					   &tmp8);
			carriers[carrier].code_rate.dvbs2_code_rate =
			    (avl62x1_dvbs2_code_rate)tmp8;
		}
		else if (carriers[carrier].signal_type == avl62x1_dvbs)
		{
			carriers[carrier].pls_acm = 16; //QPSK 1/2
			carriers[carrier].dvbs2_ccm_acm = avl62x1_dvbs2_ccm;
			carriers[carrier].modulation = avl62x1_qpsk;
			carriers[carrier].pilot = avl62x1_pilot_off;

			r |= avl_bms_read8(chip->chip_pub->i2c_addr,
					   carrier_list_ptr +
					       carrier * AVL62X1_SAT_CARRIER_struct_size +
					       AVL62X1_SAT_CARRIER_CodeRate_caddr,
					   &tmp8);
			carriers[carrier].code_rate.dvbs_code_rate =
			    (avl62x1_dvbs_code_rate)tmp8;
		}
	} //for carriers

	return (r);
}

uint16_t avl62x1_blindscan_get_stream_list(
    struct avl62x1_carrier_info *carrier,
    struct avl62x1_stream_info *streams,
    const uint8_t max_streams,
    struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	uint8_t tmp8;

	r |= avl62x1_get_num_streams(&tmp8, chip);
	if (carrier != nullptr)
	{
		carrier->num_streams = tmp8;
	}

	r |= avl62x1_get_stream_list(streams, max_streams, chip);

	if (carrier != nullptr)
	{
		r |= avl_bms_read8(chip->chip_pub->i2c_addr,
				   s_AVL62X1_SP_S2X_sp_SIS_MIS_caddr,
				   &tmp8);
		carrier->sis_mis = (avl62x1_sis_mis)(tmp8);
	}

	return (r);
}

uint16_t avl62x1_blindscan_confirm_carrier(
    const struct avl62x1_blind_scan_params *params,
    struct avl62x1_carrier_info *carrier,
    struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	int32_t cfo_Hz = 0;

	r |= avl_bms_write8(chip->chip_pub->i2c_addr,
			    c_AVL62X1_S2X_blind_sym_rate_enable_caddr,
			    0);

	r |= avl_bms_write8(chip->chip_pub->i2c_addr,
			    c_AVL62X1_S2X_blind_cfo_enable_caddr,
			    0);

	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     c_AVL62X1_S2X_nom_symbol_rate_Hz_iaddr,
			     carrier->symbol_rate_hz);

	//back-calculate CFO from BS RF freq result
	cfo_Hz = ((int32_t)(carrier->rf_freq_khz) -
		  (int32_t)(params->tuner_center_freq_100khz) * 100) *
		 1000;

	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     c_AVL62X1_S2X_nom_carrier_freq_Hz_iaddr,
			     (uint32_t)(cfo_Hz));

	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     c_AVL62X1_S2X_ConfiguredPLScramKey_iaddr,
			     carrier->pl_scrambling);

	r |= avl_bms_write8(chip->chip_pub->i2c_addr,
			    c_AVL62X1_SP_S2X_ConfiguredStreamType_caddr,
			    avl62x1_undetermined);

	r |= avl_bms_write8(chip->chip_pub->i2c_addr,
			    c_AVL62X1_SP_S2X_sp_wanted_stream_id_caddr,
			    0);

	r |= avl_bms_write8(chip->chip_pub->i2c_addr,
			    c_AVL62X1_SP_S2X_sp_t2mi_ts_pid_detect_auto_en_caddr,
			    1);

	r |= avl_bms_write8(chip->chip_pub->i2c_addr,
			    c_AVL62X1_SP_S2X_sp_raw_t2mi_mode_caddr,
			    0);

	r |= __avl62x1_send_cmd(CMD_ACQUIRE, chip);
	return (r);
}

uint16_t avl62x1_optimize_carrier(struct avl_tuner *tuner, struct avl62x1_carrier_info *carrier, struct avl62x1_chip *chip)
{
	/* When calling this function in blindscan mode,
 * either set tuner->blindscan_mode = 1, carrier = nullptr,
 * or carrier->symbol_rate_hz = 0xFFFFFFFF
 */
	uint16_t r = AVL_EC_OK;
	const uint32_t sym_rate_error_hz = 5 * 1000 * 1000;
	uint32_t carrier_bw_hz = 0;
	uint32_t max_lpf_hz = 0;
	uint32_t min_lpf_hz = 0;
	int32_t if_hz = 0;
	uint32_t tuner_step_size_hz = 0;
	uint32_t lpf_step_size_hz = 0;

	if (tuner->get_rf_freq_step_size == nullptr)
	{
		tuner_step_size_hz = 250000;
	}
	else
	{
		tuner->get_rf_freq_step_size(tuner, &tuner_step_size_hz);
	}
	if (tuner->get_max_lpf == nullptr)
	{
		max_lpf_hz = 34000000;
	}
	else
	{
		tuner->get_max_lpf(tuner, &max_lpf_hz);
	}
	if (tuner->get_min_lpf == nullptr)
	{
		min_lpf_hz = 10000000;
	}
	else
	{
		tuner->get_min_lpf(tuner, &min_lpf_hz);
	}
	if (tuner->get_lpf_step_size == nullptr)
	{
		lpf_step_size_hz = 1000000;
	}
	else
	{
		tuner->get_lpf_step_size(tuner, &lpf_step_size_hz);
	}

	if (carrier == nullptr)
	{
		tuner->blindscan_mode = 1;
	}
	else if (carrier->symbol_rate_hz == 0xFFFFFFFF)
	{
		tuner->blindscan_mode = 1;
	}
	if (tuner->blindscan_mode == 1)
	{
		//Set tuner LPF wide open
		tuner->lpf_hz = max_lpf_hz;
	}
	else
	{
		//double-sided carrier BW
		carrier_bw_hz = (carrier->symbol_rate_hz / 100) * 135; //35%
		carrier_bw_hz += sym_rate_error_hz + tuner_step_size_hz / 2;

		if (carrier->symbol_rate_hz < AVL62X1_IF_SHIFT_MAX_SR_HZ)
		{
			//remove apriori CFO
			carrier->rf_freq_khz +=
			    carrier->carrier_freq_offset_hz / 1000;
			//adjust by IF
			if_hz = (carrier_bw_hz / 2);
			carrier->rf_freq_khz -= if_hz / 1000;
			carrier->carrier_freq_offset_hz = if_hz;
			tuner->rf_freq_hz = carrier->rf_freq_khz * 1000;
			carrier_bw_hz *= 2;
		}

		chip->chip_priv->carrier_freq_offset_hz =
		    carrier->carrier_freq_offset_hz;
		//Set tuner LPF so that carrier edge is an
		//  octave from the LPF 3dB point
		tuner->lpf_hz = avl_min_32(carrier_bw_hz + lpf_step_size_hz / 2,
					   max_lpf_hz);
		tuner->lpf_hz = avl_max_32(tuner->lpf_hz, min_lpf_hz);
	}

	return (r);
}

/* avl62x1_lock_tp() or avl62x1_switch_stream() must be called after calling either of these functions
* in order for them to take effect.,if the T2MI TS PID is not 0x1000, please set it .
*/
uint16_t avl62x1_manual_set_t2mi_pid(
	uint16_t pid,
	struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;

	r |= avl_bms_write16(chip->chip_pub->i2c_addr,
	c_AVL62X1_SP_S2X_sp_t2mi_ts_pid_saddr,
	pid); //default value 0x1000 //Set T2MI ID

	return (r);
}

uint16_t avl62x1_manual_set_t2mi_pid_1(
	uint16_t pid,
	struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;

	r |= avl_bms_write16(chip->chip_pub->i2c_addr,
			     c_AVL62X1_SP_S2X_sp_t2mi_ts_pid_1_saddr,
			     pid); //default value 0x40 //Set T2MI ID

	return (r);
}

// Get T2MI PID value ,and save it after lock TP without isi id ,plp id stream type parameter with avl62x1_lock_tp(&CarrierInfo, nullptr, AVL_FALSE, chip)
uint16_t avl62x1_get_current_stream_t2mi_pid(
	uint16_t *pid,
	struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;

	r |= avl_bms_read16(chip->chip_pub->i2c_addr,
			    c_AVL62X1_SP_S2X_sp_t2mi_ts_pid_cur_saddr,
			    pid); //Get current T2MI stream T2MI PID

	return (r);
}

uint16_t avl62x1_set_current_stream_t2mi_pid(
	uint16_t pid,
	struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;

	r |= avl_bms_write16(chip->chip_pub->i2c_addr,
			     c_AVL62X1_SP_S2X_sp_t2mi_ts_pid_cur_saddr,
			     pid); //Set current T2MI stream T2MI PID

	return (r);
}

//set collect frame num to chek MPLP ID ,it is related to time for get PLP ID
uint16_t avl62x1_set_t2mi_plp_id_scan_frames(
	uint16_t frame_num,
	struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;

	r |= avl_bms_write16(chip->chip_pub->i2c_addr,
	c_AVL62X1_SP_S2X_sp_t2mi_mplp_id_scan_time_caddr,
	frame_num); //default value 10

	return (r);
}

uint16_t avl62x1_get_t2mi_plp_list(
	struct avl62x1_t2mi_plp_list *list,
	struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	uint8_t num_plps = 0;
	uint8_t i = 0;
	const uint16_t time_delay = 20;
	const uint16_t max_retries = 100;
	enum avl62x1_lock_status lock_status = avl62x1_status_unlocked;
	enum avl62x1_dvb_stream_type StreamType = avl62x1_transport;

	r |= avl62x1_get_lock_status(&lock_status, chip);
	r |= avl62x1_get_stream_type(&StreamType, chip);

	if ((lock_status == avl62x1_status_locked) &&
	    (avl62x1_t2mi == StreamType))
	{
		do
		{
			r |= avl_bms_read8(chip->chip_pub->i2c_addr,
					   s_AVL62X1_SP_S2X_sp_t2mi_mplp_id_num_caddr,
					   &num_plps);
			if (max_retries < i++)
			{
				break;
			}
			avl_bsp_delay(time_delay);
		} while (0xff == num_plps);

		if (num_plps != 0xff)
		{
			//done building PLP list - fetch it
			list->list_size = num_plps;
			if (num_plps > AVL62X1_PLP_LIST_SIZE)
			{
				list->list_size = AVL62X1_PLP_LIST_SIZE;
			}
			for (i = 0; i < list->list_size; i++)
			{
				r |= avl_bms_read8(chip->chip_pub->i2c_addr,
						   s_AVL62X1_SP_S2X_sp_mplp_id_list_iaddr + i,
						   &(list->list[i]));
			}
		}
		else
		{
			list->list_size = 0;
		}
	}
	else
	{
		list->list_size = 0;
	}
	return (r);
}

uint16_t avl62x1_get_stream_type(enum avl62x1_dvb_stream_type *stream_type,
				 struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	uint8_t tmp8 = 0;
	r |= avl_bms_read8(chip->chip_pub->i2c_addr,
			   s_AVL62X1_SP_S2X_DetectedStreamType_caddr,
			   &tmp8);
	*stream_type = (avl62x1_dvb_stream_type)tmp8;

	return (r);
}

uint16_t avl62x1_get_stream_info(
    struct avl62x1_stream_info *stream_info,
    struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	r |= avl62x1_get_stream_type(&stream_info->stream_type,chip);

	r |= avl_bms_read8(chip->chip_pub->i2c_addr,
			   s_AVL62X1_SP_S2X_SelectedStreamID_caddr,
			   &stream_info->isi);
	
	//TODO: add T2MI info when appropriate
	return r;
}

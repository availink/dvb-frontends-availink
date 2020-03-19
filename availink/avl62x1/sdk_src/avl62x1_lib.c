// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Availink AVL6261 DVB-S/S2/S2X demodulator library routines
 * 
 * Copyright (C) 2020 Availink, Inc. (gpl@availink.com)
 *
 */

#include "avl62x1_lib.h"

uint16_t __avl62x1_check_chip_ready(struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	uint32_t ready = 0;
	uint32_t srst = 0;

	r = avl_bms_read32((uint16_t)(chip->chip_pub->i2c_addr),
			   hw_AVL62X1_cpucore_top_srst,
			   &srst);
	r |= avl_bms_read32((uint16_t)(chip->chip_pub->i2c_addr),
			    rs_AVL62X1_core_ready_word,
			    &ready);
	if ((AVL_EC_OK == r))
	{
		if ((1 == srst) || (ready != 0x5AA57FF7))
		{
			r = AVL_EC_GENERAL_FAIL;
		}
	}

	return (r);
}

uint16_t __avl62x1_initialize(struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;

	r |= __avl62x1_exec_patchscript(chip);

	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     c_AVL62X1_S2X_mpeg_ref_clk_Hz_iaddr,
			     chip->chip_pub->req_mpeg_clk_freq_hz);
	r |= avl_bms_write8(chip->chip_pub->i2c_addr,
			    c_AVL62X1_DMD_xtal_frequency_caddr,
			    (uint8_t)(chip->chip_pub->ref_clk));

	/*load defaults command will load S2X defaults and
	 * program PLL based on XTAL and MPEG ref clk configurations
	 */
	r |= __avl62x1_send_cmd(CMD_LD_DEFAULT, chip);

	r |= avl_bms_read32(chip->chip_pub->i2c_addr,
			    c_AVL62X1_S2X_fec_clk_Hz_iaddr,
			    &(chip->chip_priv->fec_clk_freq_hz));
	r |= avl_bms_read32(chip->chip_pub->i2c_addr,
			    c_AVL62X1_S2X_sys_clk_Hz_iaddr,
			    &(chip->chip_priv->core_clk_freq_hz));
	r |= avl_bms_read32(chip->chip_pub->i2c_addr,
			    c_AVL62X1_S2X_mpeg_ref_clk_Hz_iaddr,
			    &(chip->chip_priv->mpeg_clk_freq_hz));

	return (r);
}

uint16_t __avl62x1_get_func_mode(
    enum avl62x1_functional_mode *mode,
    struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	uint8_t tmp8 = 0;

	r = avl_bms_read8(chip->chip_pub->i2c_addr,
			  s_AVL62X1_S2X_active_demod_mode_caddr,
			  &tmp8);
	*mode = (avl62x1_functional_mode)tmp8;

	return (r);
}

uint16_t __avl62x1_get_cmd_status(struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	uint16_t cmd = 0;

	r = avl_bms_read16(chip->chip_pub->i2c_addr,
			   c_AVL62X1_DMD_command_saddr,
			   &cmd);
	if (AVL_EC_OK == r)
	{
		if (CMD_IDLE != cmd)
		{
			r = AVL_EC_RUNNING;
		}
		else if (CMD_FAILED == cmd)
		{
			r = AVL_EC_COMMAND_FAILED;
		}
	}

	return (r);
}

uint16_t __avl62x1_send_cmd(uint8_t cmd, struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	uint8_t buf[4] = {0};
	uint16_t tmp16 = 0;
	const uint16_t dly = 10; //ms
	const uint16_t max_retries = 50;
	uint32_t i = 0;

	r = avl_bsp_wait_semaphore(&(chip->chip_priv->rx_cmd_sem));

	while (AVL_EC_OK != __avl62x1_get_cmd_status(chip))
	{
		if (max_retries < i++)
		{
			r |= AVL_EC_RUNNING;
			break;
		}
		avl_bsp_delay(dly);
	}
	if (AVL_EC_OK == r)
	{
		buf[0] = 0;
		buf[1] = cmd;
		tmp16 = avl_bytes_to_short(buf);
		r |= avl_bms_write16((uint16_t)(chip->chip_pub->i2c_addr),
				     c_AVL62X1_DMD_command_saddr,
				     tmp16);
	}

	i = 0;
	while (AVL_EC_OK != __avl62x1_get_cmd_status(chip))
	{
		if (max_retries < i++)
		{
			r |= AVL_EC_RUNNING;
			break;
		}
		avl_bsp_delay(dly);
	}

	r |= avl_bsp_release_semaphore(&(chip->chip_priv->rx_cmd_sem));

	return (r);
}

uint16_t __avl62x1_get_sp_cmd_status(struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	uint16_t cmd = 0;

	r = avl_bms_read16(chip->chip_pub->i2c_addr,
			   c_AVL62X1_SP_sp_command_saddr,
			   &cmd);
	if (AVL_EC_OK == r)
	{
		if (CMD_IDLE != cmd)
		{
			r = AVL_EC_RUNNING;
		}
		else if (CMD_FAILED == cmd)
		{
			r = AVL_EC_COMMAND_FAILED;
		}
	}

	return (r);
}

uint16_t __avl62x1_send_sp_cmd(uint8_t cmd, struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	uint8_t buf[4] = {0};
	uint16_t tmp16 = 0;
	const uint16_t dly = 10;
	const uint16_t max_retries = 50;
	uint32_t i = 0;

	r = avl_bsp_wait_semaphore(&(chip->chip_priv->rx_cmd_sem));

	while (AVL_EC_OK != __avl62x1_get_sp_cmd_status(chip))
	{
		if (max_retries < i++)
		{
			r |= AVL_EC_RUNNING;
			break;
		}
		avl_bsp_delay(dly);
	}
	if (AVL_EC_OK == r)
	{
		buf[0] = 0;
		buf[1] = cmd;
		tmp16 = avl_bytes_to_short(buf);
		r |= avl_bms_write16((uint16_t)(chip->chip_pub->i2c_addr),
				     c_AVL62X1_SP_sp_command_saddr,
				     tmp16);
	}

	i = 0;
	while (AVL_EC_OK != __avl62x1_get_sp_cmd_status(chip))
	{
		if (max_retries < i++)
		{
			r |= AVL_EC_RUNNING;
			break;
		}
		avl_bsp_delay(dly);
	}

	r |= avl_bsp_release_semaphore(&(chip->chip_priv->rx_cmd_sem));

	return (r);
}

uint16_t __avl62x1_halt(struct avl62x1_chip *chip)
{
  uint16_t r = AVL_EC_OK;

  r |= __avl62x1_send_cmd(CMD_HALT, chip);

  return (r);
}

uint16_t __avl62x1_sleep(struct avl62x1_chip *chip)
{
  uint16_t r = AVL_EC_OK;

  r |= __avl62x1_send_cmd(CMD_SLEEP, chip);

  return (r);
}

uint16_t __avl62x1_wakeup(struct avl62x1_chip *chip)
{
  uint16_t r = AVL_EC_OK;

  r |= __avl62x1_send_cmd(CMD_WAKE, chip);

  return (r);
}

uint16_t __avl62x1_init_tuner_i2c(struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	uint32_t tmp32 = 0;
	uint32_t bit_rpt_divider = 0;

	//reset tuner i2c block
	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     psc_tuner_i2c__tuner_i2c_srst,
			     1);
	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     psc_tuner_i2c__tuner_i2c_srst,
			     0);

	//tuner_i2c_cntrl: {rpt_addr[23:16],...,i2c_mode[8],...,src_sel[0]}
	r |= avl_bms_read32(chip->chip_pub->i2c_addr,
			    psc_tuner_i2c__tuner_i2c_cntrl,
			    &tmp32);
	tmp32 = (tmp32 & 0xFFFFFFFE);
	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     psc_tuner_i2c__tuner_i2c_cntrl,
			     tmp32);

	//hw_i2c_bit_rpt_cntrl: {doubleFFen, stop_check, rpt_sel, rpt_enable}
	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     psc_tuner_i2c__tuner_hw_i2c_bit_rpt_cntrl,
			     0x6);

	bit_rpt_divider = (0x2A) *
			  (chip->chip_priv->core_clk_freq_hz / 1000) /
			  (240 * 1000);
	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     psc_tuner_i2c__tuner_hw_i2c_bit_rpt_clk_div,
			     bit_rpt_divider);

	//configure GPIO
	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     gpio_debug__i2c_clk2_sel,
			     7); //M3_SCL
	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     gpio_debug__i2c_data2_sel,
			     8); //M3_SDA
	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     gpio_debug__m3_scl_sel,
			     6); //I2C_CLK2
	r |= avl_bms_write32(chip->chip_pub->i2c_addr,
			     gpio_debug__m3_sda_sel,
			     5); //I2C_DATA2

	return (r);
}

uint16_t __avl62x1_init_adc(struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;

	r |= avl_bms_write8(chip->chip_pub->i2c_addr,
			    c_AVL62X1_S2X_input_format_caddr,
			    0x0); // 0: 2's complement, 1: offset binary
	r |= avl_bms_write8(chip->chip_pub->i2c_addr,
			    c_AVL62X1_S2X_input_select_caddr,
			    0x1); //0: Digital, 1: ADC in

	return (r);
}

uint16_t __avl62x1_get_tuner_polarity(
    enum avl62x1_spectrum_polarity *polarity,
    struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	uint8_t tmp8 = 0;

	r |= avl_bms_read8(chip->chip_pub->i2c_addr,
			   s_AVL62X1_S2X_carrier_spectrum_invert_status_caddr,
			   &tmp8);
	*polarity = (enum avl62x1_spectrum_polarity)tmp8;

	return (r);
}

uint16_t __avl62x1_set_tuner_polarity(
    enum avl62x1_spectrum_polarity polarity,
    struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;

	r |= avl_bms_write8(chip->chip_pub->i2c_addr,
			    c_AVL62X1_S2X_tuner_spectrum_invert_caddr,
			    (uint8_t)polarity);

	return (r);
}

uint16_t __avl62x1_drive_agc(
    enum avl62x1_switch state,
    struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	uint32_t tmp32 = 0;
	int32_t rfagc_slope = 0;
	uint32_t min_gain = 0;
	uint32_t max_gain = 0;
	uint32_t min_gain_mv = 0;
	uint32_t max_gain_mv = 0;

	if (avl62x1_on == state)
	{
		//set RF AGC polarity according to AGC slope sign
		if (chip->chip_pub->tuner->get_agc_slope == nullptr)
		{
			rfagc_slope = -1;
		}
		else
		{
			chip->chip_pub->tuner->get_agc_slope(
			    chip->chip_pub->tuner,
			    &rfagc_slope);
		}
		r |= avl_bms_write8(chip->chip_pub->i2c_addr,
				    c_AVL62X1_S2X_rf_agc_pol_caddr,
				    (uint8_t)(rfagc_slope < 0));

		//set min and max gain values according to AGC saturation points
		if (chip->chip_pub->tuner->get_min_gain_voltage == nullptr ||
		    chip->chip_pub->tuner->get_max_gain_voltage == nullptr)
		{
			//set some reasonable defaults
			if (rfagc_slope > 0)
			{
				min_gain_mv = 100;
				max_gain_mv = 3200;
			}
			else
			{
				min_gain_mv = 3200;
				max_gain_mv = 100;
			}
		}
		else
		{
			chip->chip_pub->tuner->get_min_gain_voltage(
			    chip->chip_pub->tuner,
			    &min_gain_mv);
			chip->chip_pub->tuner->get_max_gain_voltage(
			    chip->chip_pub->tuner,
			    &max_gain_mv);
			min_gain_mv = AVL_MIN(min_gain_mv, 3300);
			max_gain_mv = AVL_MIN(max_gain_mv, 3300);
		}

		if (rfagc_slope > 0)
		{
			min_gain = (min_gain_mv * (1 << 16)) / 3300;
			max_gain = (max_gain_mv * (1 << 16)) / 3300;
		}
		else
		{
			min_gain = ((3300 - min_gain_mv) * (1 << 16)) / 3300;
			max_gain = ((3300 - max_gain_mv) * (1 << 16)) / 3300;
		}

		min_gain = AVL_MIN(min_gain, 0xFFFF);
		max_gain = AVL_MIN(max_gain, 0xFFFF);

		r |= avl_bms_write16((chip->chip_pub->i2c_addr),
				     c_AVL62X1_S2X_rf_agc_min_gain_saddr,
				     min_gain);
		r |= avl_bms_write16((chip->chip_pub->i2c_addr),
				     c_AVL62X1_S2X_rf_agc_max_gain_saddr,
				     max_gain);

		//enable sigma delta output
		r |= avl_bms_read32((chip->chip_pub->i2c_addr),
				    aagc__analog_agc_sd_control_reg,
				    &tmp32);
		tmp32 |= (0x1 << 1); //agc_sd_on bit
		r |= avl_bms_write32((chip->chip_pub->i2c_addr),
				     aagc__analog_agc_sd_control_reg,
				     tmp32);

		//configure GPIO
		r |= avl_bms_write32((chip->chip_pub->i2c_addr),
				     gpio_debug__agc1_sel,
				     6);
		r |= avl_bms_write32((chip->chip_pub->i2c_addr),
				     gpio_debug__agc2_sel,
				     6);
		
		chip->chip_priv->agc_driven = 1;
	}
	else if (avl62x1_off == state)
	{
		r |= avl_bms_read32((chip->chip_pub->i2c_addr),
				    aagc__analog_agc_sd_control_reg,
				    &tmp32);
		tmp32 &= ~(0x1 << 1); //agc_sd_on bit
		r |= avl_bms_write32((chip->chip_pub->i2c_addr),
				     aagc__analog_agc_sd_control_reg,
				     tmp32);

		//configure GPIO
		r |= avl_bms_write32((chip->chip_pub->i2c_addr),
				     gpio_debug__agc1_sel,
				     2); //high-Z
		r |= avl_bms_write32((chip->chip_pub->i2c_addr),
				     gpio_debug__agc2_sel,
				     2); //high-Z
		
		chip->chip_priv->agc_driven = 0;
	}
	else
	{
		chip->chip_priv->agc_driven = 0;
		r |= AVL_EC_GENERAL_FAIL;
	}
	return (r);
}

uint16_t __avl62x1_get_cfo(int32_t *cfo, struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	uint32_t tmp32 = 0;

	r |= avl_bms_read32(chip->chip_pub->i2c_addr,
			    s_AVL62X1_S2X_carrier_freq_Hz_iaddr,
			    &tmp32);
	*cfo = (int32_t)tmp32;

	r |= avl_bms_read32(chip->chip_pub->i2c_addr,
			    s_AVL62X1_S2X_carrier_freq_err_Hz_iaddr,
			    &tmp32);
	*cfo += (int32_t)tmp32;
	*cfo -= chip->chip_priv->carrier_freq_offset_hz;

	return (r);
}

uint16_t __avl62x1_get_sro(int32_t *sro_ppm, struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	int32_t sr_error = 0;
	uint32_t sr = 0;

	r |= avl_bms_read32(chip->chip_pub->i2c_addr,
			    s_AVL62X1_S2X_symbol_rate_Hz_iaddr,
			    &sr);
	r |= avl_bms_read32(chip->chip_pub->i2c_addr,
			    s_AVL62X1_S2X_symbol_rate_error_Hz_iaddr,
			    (uint32_t *)&sr_error);

	*sro_ppm = (int32_t)(
	    ((long long int)sr_error * ((long long int)1000000)) /
	    (long long int)sr);
	return (r);
}

uint16_t __avl62x1_get_acq_retries(uint8_t *retries, struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	r |= avl_bms_read8(chip->chip_pub->i2c_addr,
			   s_AVL62X1_S2X_acq_retry_count_caddr,
			   retries);
	return (r);
}

uint16_t __avl62x1_set_mpeg_mode(struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;

	r |= avl_bms_write8(chip->chip_pub->i2c_addr,
			    c_AVL62X1_SP_sp_ts_serial_caddr,
			    (uint8_t)(chip->chip_pub->mpeg_mode));
	r |= avl_bms_write8(chip->chip_pub->i2c_addr,
			    c_AVL62X1_SP_sp_ts0_tsp1_caddr,
			    (uint8_t)(chip->chip_pub->mpeg_format));
	r |= avl_bms_write8(chip->chip_pub->i2c_addr,
			    c_AVL62X1_SP_sp_ts_clock_edge_caddr,
			    (uint8_t)(chip->chip_pub->mpeg_clk_pol));
	r |= avl_bms_write8(chip->chip_pub->i2c_addr,
			    c_AVL62X1_SP_sp_ts_clock_phase_caddr,
			    (uint8_t)(chip->chip_pub->mpeg_clk_phase));
	r |= avl_bms_write8(chip->chip_pub->i2c_addr,
			    c_AVL62X1_SP_sp_ts_adapt_clk_caddr,
			    (uint8_t)(chip->chip_pub->mpeg_clk_adapt ==
				      avl62x1_mpca_adaptive));
	r |= avl_bms_write8(chip->chip_pub->i2c_addr,
			    (c_AVL62X1_SP_sp_mpeg_bus_misc_2_iaddr + 2),
			    0x3); //enhance the MPEG driver ability.
	r |= avl_bms_write8(chip->chip_pub->i2c_addr,
			    c_AVL62X1_SP_sp_ts_error_polarity_caddr,
			    (uint8_t)(chip->chip_pub->mpeg_err_pol));
	r |= avl_bms_write8(chip->chip_pub->i2c_addr,
			    c_AVL62X1_SP_sp_ts_valid_polarity_caddr,
			    (uint8_t)(chip->chip_pub->mpeg_valid_pol));

	if (chip->chip_pub->mpeg_mode == avl62x1_mpm_serial)
	{
		r |= avl_bms_write8(chip->chip_pub->i2c_addr,
				    c_AVL62X1_SP_sp_ts_serial_outpin_caddr,
				    (uint8_t)(chip->chip_pub->mpeg_serial_pin));
		r |= avl_bms_write8(chip->chip_pub->i2c_addr,
				    c_AVL62X1_SP_sp_ts_serial_msb_caddr,
				    (uint8_t)(chip->chip_pub->mpeg_bit_order));
		r |= avl_bms_write8((uint16_t)(chip->chip_pub->i2c_addr),
				    c_AVL62X1_SP_sp_enable_ts_continuous_caddr,
				    1);
	}

	if (chip->chip_pub->mpeg_mode == avl62x1_mpm_parallel)
	{
		r |= avl_bms_write8(chip->chip_pub->i2c_addr,
				    c_AVL62X1_SP_sp_ts_adapt_clk_caddr,
				    avl62x1_mpca_adaptive);
		r |= avl_bms_write8((uint16_t)(chip->chip_pub->i2c_addr),
				    c_AVL62X1_SP_sp_enable_ts_continuous_caddr,
				    0);
	}

	return (r);
}

uint16_t __avl62x1_drive_mpeg_output(
    enum avl62x1_switch state,
    struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;

	if (avl62x1_on == state)
	{
		r |= avl_bms_write8(chip->chip_pub->i2c_addr,
				    ts_output_intf__mpeg_bus_off,
				    0x00);
		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     ts_output_intf__mpeg_bus_e_b,
				     0x00);
	}
	else if (avl62x1_off == state)
	{
		r |= avl_bms_write8(chip->chip_pub->i2c_addr,
				    ts_output_intf__mpeg_bus_off,
				    0xFF);
		r |= avl_bms_write32(chip->chip_pub->i2c_addr,
				     ts_output_intf__mpeg_bus_e_b,
				     0xFFF);
	}
	else
	{
		r |= AVL_EC_GENERAL_FAIL;
	}
	return (r);
}

uint16_t __avl62x1_diseqc_mode_switch_check(struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	uint32_t i1 = 0;

	switch (chip->chip_priv->diseqc_op_status)
	{
	case avl62x1_dos_modulation:
	case avl62x1_dos_tone:
		r |= avl_bms_read32(chip->chip_pub->i2c_addr,
				    diseqc__diseqc_tx_st,
				    &i1);
		//check if the last transmit is done
		if (1 != ((i1 & 0x00000040) >> 6))
		{
			r |= AVL_EC_RUNNING;
		}
		break;
	case avl62x1_dos_continuous:
	case avl62x1_dos_init:
		break;
	default:
		r |= AVL_EC_GENERAL_FAIL;
		break;
	}
	return (r);
}

/* TODO: refactor this function to be less monolithic and reduce stack */
uint16_t __avl62x1_exec_patchscript(struct avl62x1_chip *chip)
{
	uint16_t r = AVL_EC_OK;
	uint8_t *patch_data = 0;
	uint32_t patch_idx = 0;
	uint32_t total_patch_len = 0;
	uint32_t standard = 0;
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
	uint32_t operation = 0;
	uint32_t value = 0;
	uint32_t cmd = 0;
	uint32_t num_rvs = 0;
	uint32_t num_rvs2 = 0;
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
	uint32_t ready = 0;
	uint32_t dma_max_tries = 0;
	uint32_t dma_tries = 0;
	uint32_t rv = 0;
	int8_t buf_tmp_3[3];
	uint8_t *patch_data_tmp = 0;
	uint8_t *patch_data_tmp_1 = 0;
	uint32_t cond = 0;
	uint8_t got_cmd_exit = 0;
	uint16_t num_records = 0;
	uint16_t record_length = 0;
	uint16_t addr_offset = 0;
	uint16_t record_cnt = 0;
	uint32_t match_value = 0;
	uint32_t max_polls = 0;
	uint32_t polls = 0;
	uint32_t variable_array[PATCH_VAR_ARRAY_SIZE];

	patch_data = chip->chip_priv->patch_data;

	patch_idx = 4; //INDEX IS A BYTE OFFSET
	total_patch_len = __avl62x1_patch_read32(patch_data, &patch_idx);
	standard = __avl62x1_patch_read32(patch_data, &patch_idx);
	args_addr = __avl62x1_patch_read32(patch_data, &patch_idx);
	data_section_offset = __avl62x1_patch_read32(patch_data, &patch_idx);
	reserved_len = __avl62x1_patch_read32(patch_data, &patch_idx);
	patch_idx += 4 * reserved_len; //skip over reserved area for now
	script_len = __avl62x1_patch_read32(patch_data, &patch_idx);

	if ((patch_idx / 4 + script_len) != data_section_offset)
	{
		r = AVL_EC_GENERAL_FAIL;
		return (r);
	}

	script_start_idx = patch_idx / 4;

	while (((patch_idx / 4) < (script_start_idx + script_len)) && !got_cmd_exit)
	{
		num_cmd_words = __avl62x1_patch_read32(patch_data, &patch_idx);
		next_cmd_idx = patch_idx + (num_cmd_words - 1) * 4; //BYTE OFFSET
		num_cond_words = __avl62x1_patch_read32(patch_data, &patch_idx);

		if (num_cond_words == 0)
		{
			condition = 1;
		}
		else
		{
			for (cond = 0; cond < num_cond_words; cond++)
			{
				operation = __avl62x1_patch_read32(patch_data, &patch_idx);
				value = __avl62x1_patch_read32(patch_data, &patch_idx);
				unary_op = (operation >> 8) & 0xFF;
				binary_op = operation & 0xFF;
				addr_mode_op = ((operation >> 16) & 0x3);

				if ((addr_mode_op == PATCH_OP_ADDR_MODE_VAR_IDX) && (binary_op != PATCH_OP_BINARY_STORE))
				{
					value = variable_array[value]; //grab variable value
				}

				switch (unary_op)
				{
				case PATCH_OP_UNARY_LOGICAL_NEGATE:
					value = !value;
					break;
				case PATCH_OP_UNARY_BITWISE_NEGATE:
					value = ~value;
					break;
				case PATCH_OP_UNARY_BITWISE_AND:
					//value = FIXME
					break;
				case PATCH_OP_UNARY_BITWISE_OR:
					//value = FIXME
					break;
				}
				switch (binary_op)
				{
				case PATCH_OP_BINARY_LOAD:
					condition = value;
					break;
				case PATCH_OP_BINARY_STORE:
					variable_array[value] = condition;
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
					r = AVL_EC_GENERAL_FAIL;
					return (r);
				}
			} //for conditions
		}

		if (condition)
		{
			cmd = __avl62x1_patch_read32(patch_data, &patch_idx);

			switch (cmd)
			{
			case PATCH_CMD_PING: //1
			{
				r |= __avl62x1_send_cmd(CMD_PING, chip);

				num_rvs = __avl62x1_patch_read32(patch_data, &patch_idx);
				rv0_idx = __avl62x1_patch_read32(patch_data, &patch_idx);
				variable_array[rv0_idx] = (r == AVL_EC_OK);
				patch_idx += 4 * (num_rvs - 1); //skip over any extraneous RV's
				break;
			}
			case PATCH_CMD_VALIDATE_CRC: //0
			{
				exp_crc_val = __avl62x1_patch_read32(patch_data, &patch_idx);
				start_addr = __avl62x1_patch_read32(patch_data, &patch_idx);
				length = __avl62x1_patch_read32(patch_data, &patch_idx);

				r |= avl_bms_write32(chip->chip_pub->i2c_addr, c_AVL62X1_DMD_fw_command_args_addr_iaddr, args_addr);
				r |= avl_bms_write32(chip->chip_pub->i2c_addr, args_addr + 0, start_addr);
				r |= avl_bms_write32(chip->chip_pub->i2c_addr, args_addr + 4, length);
				r |= __avl62x1_send_cmd(CMD_CALC_CRC, chip);

				r |= avl_bms_read32(chip->chip_pub->i2c_addr, args_addr + 8, &crc_result);

				num_rvs = __avl62x1_patch_read32(patch_data, &patch_idx);
				rv0_idx = __avl62x1_patch_read32(patch_data, &patch_idx);
				variable_array[rv0_idx] = (crc_result == exp_crc_val);

				patch_idx += 4 * (num_rvs - 1); //skip over any extraneous RV's

				break;
			}
			case PATCH_CMD_LD_TO_DEVICE: //2
			{
				length = __avl62x1_patch_read32(patch_data, &patch_idx); //in words
				dest_addr = __avl62x1_patch_read32(patch_data, &patch_idx);
				src_data_offset = __avl62x1_patch_read32(patch_data, &patch_idx);
				src_data_offset += data_section_offset; //add in base offset
				src_data_offset *= 4;			//convert to byte offset
#define BURST
#ifdef BURST
				length *= 4; //Convert to byte length

				patch_data_tmp = patch_data + src_data_offset;
				patch_data_tmp_1 = patch_data_tmp - 3;
				buf_tmp_3[0] = *(patch_data_tmp - 1);
				buf_tmp_3[1] = *(patch_data_tmp - 2);
				buf_tmp_3[2] = *(patch_data_tmp - 3);
				avl_int_to_3bytes(dest_addr, patch_data_tmp_1);

				r |= avl_bms_write(chip->chip_pub->i2c_addr, patch_data_tmp_1, (uint32_t)(length + 3));

				*patch_data_tmp_1 = buf_tmp_3[2];
				*(patch_data_tmp_1 + 1) = buf_tmp_3[1];
				*(patch_data_tmp_1 + 2) = buf_tmp_3[0];

#else
				for (uint32_t i = 0; i < length; i++)
				{
					//FIXME: make this a burst write
					uint32_t tdata = __avl62x1_patch_read32(patch_data, &src_data_offset);
					r |= avl_bms_write32(chip->chip_pub->i2c_addr, dest_addr + i * 4, tdata);
				}
#endif

				num_rvs = __avl62x1_patch_read32(patch_data, &patch_idx);
				patch_idx += 4 * (num_rvs); //no RV's defined yet

				break;
			}

			case PATCH_CMD_LD_TO_DEVICE_IMM: //7
			{
				length = __avl62x1_patch_read32(patch_data, &patch_idx); //in bytes
				dest_addr = __avl62x1_patch_read32(patch_data, &patch_idx);
				data = __avl62x1_patch_read32(patch_data, &patch_idx);

				if (length == 4)
				{
					r |= avl_bms_write32(chip->chip_pub->i2c_addr, dest_addr, data);
				}
				else if (length == 2)
				{
					r |= avl_bms_write16(chip->chip_pub->i2c_addr, dest_addr, data);
				}
				else if (length == 1)
				{
					r |= avl_bms_write8(chip->chip_pub->i2c_addr, dest_addr, data);
				}

				num_rvs = __avl62x1_patch_read32(patch_data, &patch_idx);
				patch_idx += 4 * (num_rvs); //no RV's defined yet
				break;
			}
			case PATCH_CMD_LD_TO_DEVICE_PACKED:
			{
				length = __avl62x1_patch_read32(patch_data, &patch_idx); //in words
				src_data_offset = __avl62x1_patch_read32(patch_data, &patch_idx);

				src_data_offset += data_section_offset; //add in base offset to make it absolute
				src_data_offset *= 4;			//convert to byte offset
				length *= 4;				//Convert to byte length

				src_data_offset += 2;								    //skip over address offset length. assume it's 2 for now!
				num_records = (patch_data[src_data_offset] << 8) + patch_data[src_data_offset + 1]; //number of records B.E.
				src_data_offset += 2;
				dest_addr = (patch_data[src_data_offset] << 24) + (patch_data[src_data_offset + 1] << 16) +
					    (patch_data[src_data_offset + 2] << 8) + (patch_data[src_data_offset + 3] << 0); //destination address B.E.
				src_data_offset += 4;

				//AVL_puchar pRecordData = new uint8_t[(1<<16)+3];
				for (record_cnt = 0; record_cnt < num_records; record_cnt++)
				{
					addr_offset = (patch_data[src_data_offset] << 8) + patch_data[src_data_offset + 1]; //address offset B.E.
					src_data_offset += 2;
					record_length = (patch_data[src_data_offset] << 8) + patch_data[src_data_offset + 1]; //data len B.E.
					src_data_offset += 2;

					//temporarily save patch data that will be overwritten
					buf_tmp_3[0] = patch_data[src_data_offset - 3];
					buf_tmp_3[1] = patch_data[src_data_offset - 2];
					buf_tmp_3[2] = patch_data[src_data_offset - 1];

					//break address into 3 bytes and put in patch array right in front of data
					avl_int_to_3bytes(dest_addr + addr_offset, &(patch_data[src_data_offset - 3]));
					r |= avl_bms_write(chip->chip_pub->i2c_addr, &(patch_data[src_data_offset - 3]), record_length + 3);

					//restore patch data
					patch_data[src_data_offset - 3] = buf_tmp_3[0];
					patch_data[src_data_offset - 2] = buf_tmp_3[1];
					patch_data[src_data_offset - 1] = buf_tmp_3[2];

					src_data_offset += record_length;
				}
				num_rvs2 = __avl62x1_patch_read32(patch_data, &patch_idx);
				patch_idx += 4 * (num_rvs2); //no RV's defined yet
				break;
			}
			case PATCH_CMD_RD_FROM_DEVICE: //8 8
			{
				length = __avl62x1_patch_read32(patch_data, &patch_idx); //in bytes
				src_addr = __avl62x1_patch_read32(patch_data, &patch_idx);
				num_rvs = __avl62x1_patch_read32(patch_data, &patch_idx);
				rv0_idx = __avl62x1_patch_read32(patch_data, &patch_idx);

				if (length == 4)
				{
					r |= avl_bms_read32(chip->chip_pub->i2c_addr, src_addr, &data);
					variable_array[rv0_idx] = data;
				}
				else if (length == 2)
				{
					r |= avl_bms_read16(chip->chip_pub->i2c_addr, src_addr, &data1);
					variable_array[rv0_idx] = data1;
				}
				else if (length == 1)
				{
					r |= avl_bms_read8(chip->chip_pub->i2c_addr, src_addr, &data2);
					variable_array[rv0_idx] = data2;
				}
				patch_idx += 4 * (num_rvs - 1); //skip over any extraneous RV's
				break;
			}
			case PATCH_CMD_DMA: //3
			{
				descr_addr = __avl62x1_patch_read32(patch_data, &patch_idx);
				num_descr = __avl62x1_patch_read32(patch_data, &patch_idx);

				patch_data_tmp = patch_data + patch_idx;
				patch_data_tmp_1 = patch_data_tmp - 3;
				buf_tmp_3[0] = *(patch_data_tmp - 1);
				buf_tmp_3[1] = *(patch_data_tmp - 2);
				buf_tmp_3[2] = *(patch_data_tmp - 3);
				avl_int_to_3bytes(descr_addr, patch_data_tmp_1);

				r |= avl_bms_write(chip->chip_pub->i2c_addr, patch_data_tmp_1, (uint32_t)(num_descr * 3 * 4));
				*patch_data_tmp_1 = buf_tmp_3[2];
				*(patch_data_tmp_1 + 1) = buf_tmp_3[1];
				*(patch_data_tmp_1 + 2) = buf_tmp_3[0];
				patch_idx += 12 * num_descr;

				r |= avl_bms_write32(chip->chip_pub->i2c_addr, c_AVL62X1_DMD_fw_command_args_addr_iaddr, descr_addr);
				r |= __avl62x1_send_cmd(CMD_DMA, chip);

				num_rvs = __avl62x1_patch_read32(patch_data, &patch_idx);
				patch_idx += 4 * (num_rvs); //no RV's defined yet
				break;
			}
			case PATCH_CMD_EXTRACT: //4
			{
				type = __avl62x1_patch_read32(patch_data, &patch_idx);
				src_addr = __avl62x1_patch_read32(patch_data, &patch_idx);
				dest_addr = __avl62x1_patch_read32(patch_data, &patch_idx);

				r |= avl_bms_write32(chip->chip_pub->i2c_addr, c_AVL62X1_DMD_fw_command_args_addr_iaddr, args_addr);
				r |= avl_bms_write32(chip->chip_pub->i2c_addr, args_addr + 0, type);
				r |= avl_bms_write32(chip->chip_pub->i2c_addr, args_addr + 4, src_addr);
				r |= avl_bms_write32(chip->chip_pub->i2c_addr, args_addr + 8, dest_addr);

				r |= __avl62x1_send_cmd(CMD_DECOMPRESS, chip);

				num_rvs = __avl62x1_patch_read32(patch_data, &patch_idx);
				patch_idx += 4 * (num_rvs); //no RV's defined yet
				break;
			}
			case PATCH_CMD_ASSERT_CPU_RESET: //5
			{
				r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_AVL62X1_cpucore_top_srst, 1);

				num_rvs = __avl62x1_patch_read32(patch_data, &patch_idx);
				patch_idx += 4 * (num_rvs); //no RV's defined yet
				break;
			}
			case PATCH_CMD_RELEASE_CPU_RESET: //6
			{
				r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_AVL62X1_cpucore_top_srst, 0);

				num_rvs = __avl62x1_patch_read32(patch_data, &patch_idx);
				patch_idx += 4 * (num_rvs); //no RV's defined yet
				break;
			}
			case PATCH_CMD_DMA_HW: //9
			{
				descr_addr = __avl62x1_patch_read32(patch_data, &patch_idx);
				num_descr = __avl62x1_patch_read32(patch_data, &patch_idx);

				buf_tmp_3[0] = *(patch_data + patch_idx - 1);
				buf_tmp_3[1] = *(patch_data + patch_idx - 2);
				buf_tmp_3[2] = *(patch_data + patch_idx - 3);
				avl_int_to_3bytes(descr_addr, patch_data + patch_idx - 3);

				if (num_descr > 0)
				{
					r |= avl_bms_write(chip->chip_pub->i2c_addr, patch_data + patch_idx - 3, num_descr * 12 + 3);
				}

				*(patch_data + patch_idx - 1) = buf_tmp_3[0];
				*(patch_data + patch_idx - 2) = buf_tmp_3[1];
				*(patch_data + patch_idx - 3) = buf_tmp_3[2];

				patch_idx += num_descr * 3 * 4;
				dma_tries = 0;
				dma_max_tries = 20;
				do
				{
					if (dma_tries > dma_max_tries)
					{
						return AVL_EC_GENERAL_FAIL; //FIXME return a value to check instead, and load the bootstrap
					}
					avl_bsp_delay(10); //ms
					r |= avl_bms_read32(chip->chip_pub->i2c_addr, hw_AVL62X1_dma_sys_status, &ready);
					//System::Console::WriteLine("num_dma_tries pre: {0}",dma_tries);
					dma_tries++;
				} while (!(0x01 & ready));

				if (0x01 & ready)
				{
					r |= avl_bms_write32(chip->chip_pub->i2c_addr, hw_AVL62X1_dma_sys_cmd, descr_addr); //Trigger DMA
				}

				dma_tries = 0;
				dma_max_tries = 20;
				do
				{
					if (dma_tries > dma_max_tries)
					{
						return AVL_EC_GENERAL_FAIL; //FIXME return a value to check instead, and load the bootstrap
					}
					avl_bsp_delay(10); //ms
					r |= avl_bms_read32(chip->chip_pub->i2c_addr, hw_AVL62X1_dma_sys_status, &ready);
					//System::Console::WriteLine("num_dma_tries pre: {0}",dma_tries);
					dma_tries++;
				} while (0x100 & ready);

				//Add return value later
				num_rvs = __avl62x1_patch_read32(patch_data, &patch_idx);
				patch_idx += 4 * (num_rvs); //no RV's defined yet
				break;
			}

			case PATCH_CMD_SET_COND_IMM: //10
			{
				rv = __avl62x1_patch_read32(patch_data, &patch_idx);
				num_rvs = __avl62x1_patch_read32(patch_data, &patch_idx);
				rv0_idx = __avl62x1_patch_read32(patch_data, &patch_idx);
				variable_array[rv0_idx] = rv;
				patch_idx += 4 * (num_rvs - 1); //skip over any extraneous RV's
				break;
			}
			case PATCH_CMD_EXIT:
			{
				got_cmd_exit = 1;
				num_rvs = __avl62x1_patch_read32(patch_data, &patch_idx);
				patch_idx = 4 * (script_start_idx + script_len); //skip over any extraneous RV's
				break;
			}
			case PATCH_CMD_POLL_WAIT:
			{
				length = __avl62x1_patch_read32(patch_data, &patch_idx); //in bytes
				src_addr = __avl62x1_patch_read32(patch_data, &patch_idx);
				match_value = __avl62x1_patch_read32(patch_data, &patch_idx);
				max_polls = __avl62x1_patch_read32(patch_data, &patch_idx);
				polls = 0;
				do
				{
					if (length == 4)
					{
						r |= avl_bms_read32(chip->chip_pub->i2c_addr, src_addr, &data);
						if (data == match_value)
							break;
					}
					else if (length == 2)
					{
						r = avl_bms_read16(chip->chip_pub->i2c_addr, src_addr, &data1);
						if (data1 == match_value)
							break;
					}
					else if (length == 1)
					{
						r = avl_bms_read8(chip->chip_pub->i2c_addr, src_addr, &data2);
						if (data2 == match_value)
							break;
					}
					avl_bsp_delay(10); //ms
					polls += 1;
				} while (polls < max_polls);
				num_rvs = __avl62x1_patch_read32(patch_data, &patch_idx);
				patch_idx += 4 * (num_rvs); //no RV's defined yet
				break;
			}
			} //switch cmd
		}
		else
		{
			patch_idx = next_cmd_idx; //jump to next command
			continue;
		}
	}

	return (r);
}

uint8_t __avl62x1_patch_read8(uint8_t *buf, uint32_t *idx)
{
	uint8_t tmp = 0;
	tmp = buf[*idx];
	*idx += 1;
	return tmp;
}
uint16_t __avl62x1_patch_read16(uint8_t *buf, uint32_t *idx)
{
	uint16_t tmp = 0;
	tmp = (buf[*idx + 0] << 8) |
	      (buf[*idx + 1]);
	*idx += 2;
	return tmp;
}
uint32_t __avl62x1_patch_read32(uint8_t *buf, uint32_t *idx)
{
	uint32_t tmp = 0;
	tmp = (buf[*idx + 0] << 24) |
	      (buf[*idx + 1] << 16) |
	      (buf[*idx + 2] << 8) |
	      buf[*idx + 3];
	*idx += 4;
	return tmp;
}

uint16_t __avl62x1_conv_xlfsr_state_to_n(uint32_t state, uint32_t *n)
{
	uint32_t search_state = 0x0001;
	state &= ~(1 << 18); //clear x(18) just to be safe

	for (*n = 0; *n <= 262141; *n += 1)
	{
		if (search_state == state)
		{
			return AVL_EC_OK;
		}
		search_state |= ((search_state & 0x1) ^
				 ((search_state >> 7) & 0x1)) << 18;
		search_state >>= 1;
	}

	return AVL_EC_GENERAL_FAIL;
}

uint16_t __avl62x1_conv_n_to_xlfsr_state(uint32_t n, uint32_t *state)
{
	uint32_t t_state = 0x1;
	while (n-- > 0)
	{
		t_state |= ((t_state & 0x1) ^ ((t_state >> 7) & 0x1)) << 18;
		t_state >>= 1;
	}
	*state = t_state;

	return AVL_EC_OK;
}

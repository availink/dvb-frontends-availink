// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Availink AVL6261 DVB-S/S2/S2X demodulator driver
 * 
 * Copyright (C) 2020 Availink, Inc. (gpl@availink.com)
 *
 */

#ifndef _AVL62X1_REG_H_
#define _AVL62X1_REG_H_

#define hw_AVL62X1_cpucore_top_srst                  (0x00110840)
#define hw_AVL62X1_dma_sys_status                    (0x00110048)
#define hw_AVL62X1_dma_sys_cmd                       (0x00110050)
#define rs_AVL62X1_core_ready_word                   (0x000000a0)


#ifndef AVL62X1_DEMOD_config_regs_base
#define AVL62X1_DEMOD_config_regs_base (0x00000200)
#endif

#define c_AVL62X1_DMD_fw_command_iaddr                    (AVL62X1_DEMOD_config_regs_base + 0x00000000)
#define c_AVL62X1_DMD_command_saddr                       (AVL62X1_DEMOD_config_regs_base + 0x00000000)
#define c_AVL62X1_DMD_last_command_saddr                  (AVL62X1_DEMOD_config_regs_base + 0x00000002)
#define c_AVL62X1_DMD_demod_config_0_iaddr                (AVL62X1_DEMOD_config_regs_base + 0x00000004)
#define c_AVL62X1_DMD_clk_set_select_caddr                (AVL62X1_DEMOD_config_regs_base + 0x00000004)
#define c_AVL62X1_DMD_xtal_frequency_caddr                (AVL62X1_DEMOD_config_regs_base + 0x00000005)
#define c_AVL62X1_DMD_bootstrap_verbosity_caddr           (AVL62X1_DEMOD_config_regs_base + 0x00000006)
#define c_AVL62X1_DMD_capture_config_iaddr                (AVL62X1_DEMOD_config_regs_base + 0x00000018)
#define c_AVL62X1_DMD_capture_length_saddr                (AVL62X1_DEMOD_config_regs_base + 0x00000018)
#define c_AVL62X1_DMD_settling_time_10Ksamples_caddr      (AVL62X1_DEMOD_config_regs_base + 0x0000001a)
#define c_AVL62X1_DMD_signal_select_caddr                 (AVL62X1_DEMOD_config_regs_base + 0x0000001b)
#define c_AVL62X1_DMD_current_active_mode_iaddr           (AVL62X1_DEMOD_config_regs_base + 0x00000024)
#define c_AVL62X1_DMD_fw_command_args_addr_iaddr          (AVL62X1_DEMOD_config_regs_base + 0x00000058)
#define c_AVL62X1_DMD_watchdog_config_iaddr               (AVL62X1_DEMOD_config_regs_base + 0x00000068)
#define c_AVL62X1_DMD_DVBSx_watchdog_timeout_caddr        (AVL62X1_DEMOD_config_regs_base + 0x00000068)
#define c_AVL62X1_DMD_DVBTx_sw_watchdog_timeout_caddr     (AVL62X1_DEMOD_config_regs_base + 0x00000069)
#define c_AVL62X1_DMD_DVBC_watchdog_timeout_caddr         (AVL62X1_DEMOD_config_regs_base + 0x0000006a)
#define c_AVL62X1_DMD_DVBTx_watchdog_timeout_caddr        (AVL62X1_DEMOD_config_regs_base + 0x0000006b)


#ifndef AVL62X1_DVBS2X_config_regs_base
#define AVL62X1_DVBS2X_config_regs_base (0x00000800)
#endif

#define c_AVL62X1_S2X_temp0_iaddr                         (AVL62X1_DVBS2X_config_regs_base + 0x00000000)
#define c_AVL62X1_S2X_nom_symbol_rate_Hz_iaddr            (AVL62X1_DVBS2X_config_regs_base + 0x00000004)
#define c_AVL62X1_S2X_sample_rate_Hz_iaddr                (AVL62X1_DVBS2X_config_regs_base + 0x00000008)
#define c_AVL62X1_S2X_rf_agc_control_iaddr                (AVL62X1_DVBS2X_config_regs_base + 0x0000000c)
#define c_AVL62X1_S2X_rf_agc_dcm_caddr                    (AVL62X1_DVBS2X_config_regs_base + 0x0000000c)
#define c_AVL62X1_S2X_rf_agc_trk_gain_caddr               (AVL62X1_DVBS2X_config_regs_base + 0x0000000d)
#define c_AVL62X1_S2X_rf_agc_acq_gain_caddr               (AVL62X1_DVBS2X_config_regs_base + 0x0000000e)
#define c_AVL62X1_S2X_rf_agc_pol_caddr                    (AVL62X1_DVBS2X_config_regs_base + 0x0000000f)
#define c_AVL62X1_S2X_rf_agc_ref_iaddr                    (AVL62X1_DVBS2X_config_regs_base + 0x00000010)
#define c_AVL62X1_S2X_dig_agc_control_iaddr               (AVL62X1_DVBS2X_config_regs_base + 0x00000014)
#define c_AVL62X1_S2X_dagc_ref_caddr                      (AVL62X1_DVBS2X_config_regs_base + 0x00000015)
#define c_AVL62X1_S2X_dagc_trk_gain_caddr                 (AVL62X1_DVBS2X_config_regs_base + 0x00000016)
#define c_AVL62X1_S2X_dagc_acq_gain_caddr                 (AVL62X1_DVBS2X_config_regs_base + 0x00000017)
#define c_AVL62X1_S2X_dc_comp_control_iaddr               (AVL62X1_DVBS2X_config_regs_base + 0x00000018)
#define c_AVL62X1_S2X_dc_comp_enable_caddr                (AVL62X1_DVBS2X_config_regs_base + 0x00000019)
#define c_AVL62X1_S2X_dc_comp_trk_gain_caddr              (AVL62X1_DVBS2X_config_regs_base + 0x0000001a)
#define c_AVL62X1_S2X_dc_comp_acq_gain_caddr              (AVL62X1_DVBS2X_config_regs_base + 0x0000001b)
#define c_AVL62X1_S2X_dc_comp_init_iaddr                  (AVL62X1_DVBS2X_config_regs_base + 0x0000001c)
#define c_AVL62X1_S2X_dc_comp_init_q_caddr                (AVL62X1_DVBS2X_config_regs_base + 0x0000001d)
#define c_AVL62X1_S2X_dc_comp_init_i_caddr                (AVL62X1_DVBS2X_config_regs_base + 0x0000001f)
#define c_AVL62X1_S2X_iq_comp_control_iaddr               (AVL62X1_DVBS2X_config_regs_base + 0x00000020)
#define c_AVL62X1_S2X_iq_comp_enable_caddr                (AVL62X1_DVBS2X_config_regs_base + 0x00000021)
#define c_AVL62X1_S2X_iq_comp_trk_gain_caddr              (AVL62X1_DVBS2X_config_regs_base + 0x00000022)
#define c_AVL62X1_S2X_iq_comp_acq_gain_caddr              (AVL62X1_DVBS2X_config_regs_base + 0x00000023)
#define c_AVL62X1_S2X_iq_comp_init_iaddr                  (AVL62X1_DVBS2X_config_regs_base + 0x00000024)
#define c_AVL62X1_S2X_iq_comp_init_phs_saddr              (AVL62X1_DVBS2X_config_regs_base + 0x00000024)
#define c_AVL62X1_S2X_iq_comp_init_amp_saddr              (AVL62X1_DVBS2X_config_regs_base + 0x00000026)
#define c_AVL62X1_S2X_btr_bandwidth1_iaddr                (AVL62X1_DVBS2X_config_regs_base + 0x00000028)
#define c_AVL62X1_S2X_btr_trk_lpbw_ppm_saddr              (AVL62X1_DVBS2X_config_regs_base + 0x00000028)
#define c_AVL62X1_S2X_btr_acq_lpbw_ppm_saddr              (AVL62X1_DVBS2X_config_regs_base + 0x0000002a)
#define c_AVL62X1_S2X_ddc_input_select_iaddr              (AVL62X1_DVBS2X_config_regs_base + 0x0000002c)
#define c_AVL62X1_S2X_tuner_type_caddr                    (AVL62X1_DVBS2X_config_regs_base + 0x0000002c)
#define c_AVL62X1_S2X_input_format_caddr                  (AVL62X1_DVBS2X_config_regs_base + 0x0000002d)
#define c_AVL62X1_S2X_tuner_spectrum_invert_caddr         (AVL62X1_DVBS2X_config_regs_base + 0x0000002e)
#define c_AVL62X1_S2X_input_select_caddr                  (AVL62X1_DVBS2X_config_regs_base + 0x0000002f)
#define c_AVL62X1_S2X_nom_carrier_freq_Hz_iaddr           (AVL62X1_DVBS2X_config_regs_base + 0x00000030)
#define c_AVL62X1_S2X_dig_agc2_control_iaddr              (AVL62X1_DVBS2X_config_regs_base + 0x00000034)
#define c_AVL62X1_S2X_dagc2_ref_caddr                     (AVL62X1_DVBS2X_config_regs_base + 0x00000035)
#define c_AVL62X1_S2X_dagc2_trk_gain_caddr                (AVL62X1_DVBS2X_config_regs_base + 0x00000036)
#define c_AVL62X1_S2X_dagc2_acq_gain_caddr                (AVL62X1_DVBS2X_config_regs_base + 0x00000037)
#define c_AVL62X1_S2X_dfe_eq_control_iaddr                (AVL62X1_DVBS2X_config_regs_base + 0x00000038)
#define c_AVL62X1_S2X_dfe_agc_on_caddr                    (AVL62X1_DVBS2X_config_regs_base + 0x00000039)
#define c_AVL62X1_S2X_dfe_fbf_eq_on_caddr                 (AVL62X1_DVBS2X_config_regs_base + 0x0000003a)
#define c_AVL62X1_S2X_dfe_fff_eq_on_caddr                 (AVL62X1_DVBS2X_config_regs_base + 0x0000003b)
#define c_AVL62X1_S2X_sys_clk_Hz_iaddr                    (AVL62X1_DVBS2X_config_regs_base + 0x0000003c)
#define c_AVL62X1_S2X_fec_clk_Hz_iaddr                    (AVL62X1_DVBS2X_config_regs_base + 0x00000040)
#define c_AVL62X1_S2X_cfo_control_iaddr                   (AVL62X1_DVBS2X_config_regs_base + 0x00000044)
#define c_AVL62X1_S2X_dvbs_cfo_num_caddr                  (AVL62X1_DVBS2X_config_regs_base + 0x00000044)
#define c_AVL62X1_S2X_init_cfo_stop_frame_caddr           (AVL62X1_DVBS2X_config_regs_base + 0x00000045)
#define c_AVL62X1_S2X_cfo_bw_step_rate_caddr              (AVL62X1_DVBS2X_config_regs_base + 0x00000046)
#define c_AVL62X1_S2X_cfo_init_step_frame_caddr           (AVL62X1_DVBS2X_config_regs_base + 0x00000047)
#define c_AVL62X1_S2X_mpeg_ref_clk_Hz_iaddr               (AVL62X1_DVBS2X_config_regs_base + 0x00000048)
#define c_AVL62X1_S2X_blind_scan_control_iaddr            (AVL62X1_DVBS2X_config_regs_base + 0x0000004c)
#define c_AVL62X1_S2X_blind_scan_carrier_db_saddr         (AVL62X1_DVBS2X_config_regs_base + 0x0000004c)
#define c_AVL62X1_S2X_freq_scan_minpk_saddr               (AVL62X1_DVBS2X_config_regs_base + 0x0000004e)
#define c_AVL62X1_S2X_blind_scan_control2_iaddr           (AVL62X1_DVBS2X_config_regs_base + 0x00000050)
#define c_AVL62X1_S2X_blind_sym_rate_enable_caddr         (AVL62X1_DVBS2X_config_regs_base + 0x00000051)
#define c_AVL62X1_S2X_blind_cfo_enable_caddr              (AVL62X1_DVBS2X_config_regs_base + 0x00000052)
#define c_AVL62X1_S2X_blind_scan_capture_psd_caddr        (AVL62X1_DVBS2X_config_regs_base + 0x00000053)
#define c_AVL62X1_S2X_blind_scan_control3_iaddr           (AVL62X1_DVBS2X_config_regs_base + 0x00000054)
#define c_AVL62X1_S2X_search_range_percent_caddr          (AVL62X1_DVBS2X_config_regs_base + 0x00000055)
#define c_AVL62X1_S2X_BWfilt_to_Rsamp_ratio_saddr         (AVL62X1_DVBS2X_config_regs_base + 0x00000056)
#define c_AVL62X1_S2X_blind_scan_control4_iaddr           (AVL62X1_DVBS2X_config_regs_base + 0x00000058)
#define c_AVL62X1_S2X_blind_scan_ACM_filter_strength_caddr (AVL62X1_DVBS2X_config_regs_base + 0x00000059)
#define c_AVL62X1_S2X_blind_scan_min_sym_rate_kHz_saddr   (AVL62X1_DVBS2X_config_regs_base + 0x0000005a)
#define c_AVL62X1_S2X_ConfiguredPLScramKey_reg_iaddr      (AVL62X1_DVBS2X_config_regs_base + 0x0000005c)
#define c_AVL62X1_S2X_ConfiguredPLScramKey_iaddr          (AVL62X1_DVBS2X_config_regs_base + 0x0000005c)
#define c_AVL62X1_S2X_bs_cent_freq_tuner_Hz_iaddr         (AVL62X1_DVBS2X_config_regs_base + 0x00000060)
#define c_AVL62X1_S2X_rf_agc_max_gain_saddr               (AVL62X1_DVBS2X_config_regs_base + 0x00000064)
#define c_AVL62X1_S2X_rf_agc_min_gain_saddr               (AVL62X1_DVBS2X_config_regs_base + 0x00000066)


#ifndef AVL62X1_SP_config_regs_base
#define AVL62X1_SP_config_regs_base (0x00a00200)
#endif

#define c_AVL62X1_SP_sp_fw_command_iaddr                  (AVL62X1_SP_config_regs_base + 0x00000000)
#define c_AVL62X1_SP_sp_command_saddr                     (AVL62X1_SP_config_regs_base + 0x00000000)
#define c_AVL62X1_SP_sp_last_command_saddr                (AVL62X1_SP_config_regs_base + 0x00000002)
#define c_AVL62X1_SP_sp_mpeg_ref_clk_config_iaddr         (AVL62X1_SP_config_regs_base + 0x00000004)
#define c_AVL62X1_SP_sp_mpeg_ref_clk_Hz_iaddr             (AVL62X1_SP_config_regs_base + 0x00000004)
#define c_AVL62X1_SP_sp_mpeg_bus_format1_iaddr            (AVL62X1_SP_config_regs_base + 0x00000008)
#define c_AVL62X1_SP_sp_set_tei_bit_caddr                 (AVL62X1_SP_config_regs_base + 0x00000008)
#define c_AVL62X1_SP_sp_ts0_tsp1_caddr                    (AVL62X1_SP_config_regs_base + 0x00000009)
#define c_AVL62X1_SP_sp_enable_ts_continuous_caddr        (AVL62X1_SP_config_regs_base + 0x0000000a)
#define c_AVL62X1_SP_sp_ts_clock_edge_caddr               (AVL62X1_SP_config_regs_base + 0x0000000b)
#define c_AVL62X1_SP_sp_mpeg_seri_format_iaddr            (AVL62X1_SP_config_regs_base + 0x0000000c)
#define c_AVL62X1_SP_sp_ts_serial_caddr                   (AVL62X1_SP_config_regs_base + 0x0000000c)
#define c_AVL62X1_SP_sp_seri_sync_1_pulse_caddr           (AVL62X1_SP_config_regs_base + 0x0000000d)
#define c_AVL62X1_SP_sp_ts_serial_msb_caddr               (AVL62X1_SP_config_regs_base + 0x0000000e)
#define c_AVL62X1_SP_sp_ts_serial_outpin_caddr            (AVL62X1_SP_config_regs_base + 0x0000000f)
#define c_AVL62X1_SP_sp_mpeg_bus_misc_iaddr               (AVL62X1_SP_config_regs_base + 0x00000010)
#define c_AVL62X1_SP_sp_ts_valid_polarity_caddr           (AVL62X1_SP_config_regs_base + 0x00000010)
#define c_AVL62X1_SP_sp_ts_error_polarity_caddr           (AVL62X1_SP_config_regs_base + 0x00000011)
#define c_AVL62X1_SP_sp_tsp_valid_high_caddr              (AVL62X1_SP_config_regs_base + 0x00000012)
#define c_AVL62X1_SP_sp_mod_sync_word_caddr               (AVL62X1_SP_config_regs_base + 0x00000013)
#define c_AVL62X1_SP_sp_watchdog_config_iaddr             (AVL62X1_SP_config_regs_base + 0x00000014)
#define c_AVL62X1_SP_sp_DVBC_watchdog_timeout_caddr       (AVL62X1_SP_config_regs_base + 0x00000016)
#define c_AVL62X1_SP_sp_DVBTx_watchdog_timeout_caddr      (AVL62X1_SP_config_regs_base + 0x00000017)
#define c_AVL62X1_SP_sp_intnl_lfsr_iaddr                  (AVL62X1_SP_config_regs_base + 0x00000018)
#define c_AVL62X1_SP_sp_reverse_multi_bit_serial_caddr    (AVL62X1_SP_config_regs_base + 0x00000018)
#define c_AVL62X1_SP_sp_idle_mpeg_clk_en_caddr            (AVL62X1_SP_config_regs_base + 0x00000019)
#define c_AVL62X1_SP_sp_lfsr_out_mode_caddr               (AVL62X1_SP_config_regs_base + 0x0000001a)
#define c_AVL62X1_SP_sp_intnl_lfsr_en_caddr               (AVL62X1_SP_config_regs_base + 0x0000001b)
#define c_AVL62X1_SP_sp_idle_clk_frac_n_iaddr             (AVL62X1_SP_config_regs_base + 0x0000001c)
#define c_AVL62X1_SP_sp_idle_clk_frac_d_iaddr             (AVL62X1_SP_config_regs_base + 0x00000020)
#define c_AVL62X1_SP_sp_ts_cntns_clk_frac_n_iaddr         (AVL62X1_SP_config_regs_base + 0x00000024)
#define c_AVL62X1_SP_sp_ts_cntns_clk_frac_d_iaddr         (AVL62X1_SP_config_regs_base + 0x00000028)
#define c_AVL62X1_SP_sp_misc_config_iaddr                 (AVL62X1_SP_config_regs_base + 0x0000002c)
#define c_AVL62X1_SP_sp_reset_ts_at_acq_caddr             (AVL62X1_SP_config_regs_base + 0x0000002d)
#define c_AVL62X1_SP_sp_esm_interval_second_caddr         (AVL62X1_SP_config_regs_base + 0x0000002e)
#define c_AVL62X1_SP_sp_fw_esm_on_caddr                   (AVL62X1_SP_config_regs_base + 0x0000002f)
#define c_AVL62X1_SP_sp_fw_lfsr_config_iaddr              (AVL62X1_SP_config_regs_base + 0x00000030)
#define c_AVL62X1_SP_sp_fw_lfsr_type_caddr                (AVL62X1_SP_config_regs_base + 0x00000030)
#define c_AVL62X1_SP_sp_fw_lfsr_start_pos_caddr           (AVL62X1_SP_config_regs_base + 0x00000031)
#define c_AVL62X1_SP_sp_fw_lfsr_ber_on_caddr              (AVL62X1_SP_config_regs_base + 0x00000032)
#define c_AVL62X1_SP_sp_mpeg_bus_misc_2_iaddr             (AVL62X1_SP_config_regs_base + 0x00000034)
#define c_AVL62X1_SP_sp_ts_adapt_clk_caddr                (AVL62X1_SP_config_regs_base + 0x00000035)
#define c_AVL62X1_SP_sp_serial_ts_ds_caddr                (AVL62X1_SP_config_regs_base + 0x00000036)
#define c_AVL62X1_SP_sp_ts_clock_phase_caddr              (AVL62X1_SP_config_regs_base + 0x00000037)
#define c_AVL62X1_SP_sp_current_active_mode_iaddr         (AVL62X1_SP_config_regs_base + 0x00000038)
#define c_AVL62X1_SP_sp_ts_rate_bps_iaddr                 (AVL62X1_SP_config_regs_base + 0x0000003c)
#define c_AVL62X1_SP_sp_symbol_rate_Hz_iaddr              (AVL62X1_SP_config_regs_base + 0x00000040)


#ifndef AVL62X1_SP_S2X_config_regs_base
#define AVL62X1_SP_S2X_config_regs_base (0x00a00600)
#endif

#define c_AVL62X1_SP_S2X_sp_s2x_config_iaddr              (AVL62X1_SP_S2X_config_regs_base + 0x00000000)
#define c_AVL62X1_SP_S2X_sp_alpha_detection_cnt_max_caddr (AVL62X1_SP_S2X_config_regs_base + 0x00000002)
#define c_AVL62X1_SP_S2X_sp_standard_caddr                (AVL62X1_SP_S2X_config_regs_base + 0x00000003)
#define c_AVL62X1_SP_S2X_sp_s2x_mode_control_iaddr        (AVL62X1_SP_S2X_config_regs_base + 0x00000004)
#define c_AVL62X1_SP_S2X_sp_raw_t2mi_mode_caddr           (AVL62X1_SP_S2X_config_regs_base + 0x00000004)
#define c_AVL62X1_SP_S2X_ConfiguredStreamType_caddr       (AVL62X1_SP_S2X_config_regs_base + 0x00000005)
#define c_AVL62X1_SP_S2X_PLP_ID_caddr                     (AVL62X1_SP_S2X_config_regs_base + 0x00000006)
#define c_AVL62X1_SP_S2X_sp_output_mode_caddr             (AVL62X1_SP_S2X_config_regs_base + 0x00000007)
#define c_AVL62X1_SP_S2X_sp_stream_filter_config_iaddr    (AVL62X1_SP_S2X_config_regs_base + 0x00000008)
#define c_AVL62X1_SP_S2X_sp_stream_id_match_count_caddr   (AVL62X1_SP_S2X_config_regs_base + 0x00000009)
#define c_AVL62X1_SP_S2X_sp_wanted_stream_id_caddr        (AVL62X1_SP_S2X_config_regs_base + 0x0000000a)
#define c_AVL62X1_SP_S2X_sp_en_stream_id_filter_caddr     (AVL62X1_SP_S2X_config_regs_base + 0x0000000b)
#define c_AVL62X1_SP_S2X_sp_rate_configs_iaddr            (AVL62X1_SP_S2X_config_regs_base + 0x0000000c)
#define c_AVL62X1_SP_S2X_sp_force_rate_control_off_caddr  (AVL62X1_SP_S2X_config_regs_base + 0x0000000d)
#define c_AVL62X1_SP_S2X_sp_code_rate_caddr               (AVL62X1_SP_S2X_config_regs_base + 0x0000000e)
#define c_AVL62X1_SP_S2X_sp_s2_modulation_caddr           (AVL62X1_SP_S2X_config_regs_base + 0x0000000f)
#define c_AVL62X1_SP_S2X_sp_blind_scan_config_iaddr       (AVL62X1_SP_S2X_config_regs_base + 0x00000010)
#define c_AVL62X1_SP_S2X_sp_blind_scan_en_caddr           (AVL62X1_SP_S2X_config_regs_base + 0x00000010)
#define c_AVL62X1_SP_S2X_sp_blind_scan_wait_frames_caddr  (AVL62X1_SP_S2X_config_regs_base + 0x00000011)
#define c_AVL62X1_SP_S2X_sp_blind_scan_CarrierIndex_caddr (AVL62X1_SP_S2X_config_regs_base + 0x00000012)
#define c_AVL62X1_SP_S2X_sp_t2mi_config_iaddr             (AVL62X1_SP_S2X_config_regs_base + 0x00000014)
#define c_AVL62X1_SP_S2X_sp_t2mi_ts_pid_detect_auto_en_caddr    (AVL62X1_SP_S2X_config_regs_base + 0x00000014)
#define c_AVL62X1_SP_S2X_sp_t2mi_mplp_id_scan_time_caddr  (AVL62X1_SP_S2X_config_regs_base + 0x00000015)
#define c_AVL62X1_SP_S2X_sp_t2mi_ts_pid_saddr             (AVL62X1_SP_S2X_config_regs_base + 0x00000016)
#define c_AVL62X1_SP_S2X_sp_t2mi_ts_pid_1_saddr           (AVL62X1_SP_S2X_config_regs_base + 0x00000018)
#define c_AVL62X1_SP_S2X_sp_t2mi_ts_pid_cur_saddr         (AVL62X1_SP_S2X_config_regs_base + 0x0000001a) 

#ifndef AVL62X1_DEMOD_status_regs_base
#define AVL62X1_DEMOD_status_regs_base (0x000000a4)
#endif

#define s_AVL62X1_DMD_debug_reg_1_iaddr                   (AVL62X1_DEMOD_status_regs_base + 0x00000000)
#define s_AVL62X1_DMD_patch_ver_iaddr                     (AVL62X1_DEMOD_status_regs_base + 0x00000004)
#define s_AVL62X1_DMD_watchdog_status_iaddr               (AVL62X1_DEMOD_status_regs_base + 0x00000028)
#define s_AVL62X1_DMD_misc_status_iaddr                   (AVL62X1_DEMOD_status_regs_base + 0x00000038)
#define s_AVL62X1_DMD_sleep_mode_caddr                    (AVL62X1_DEMOD_status_regs_base + 0x00000038)
#define s_AVL62X1_DMD_adc_status_reg_iaddr                (AVL62X1_DEMOD_status_regs_base + 0x00000044)
#define s_AVL62X1_DMD_rfagc_gain_saddr                    (AVL62X1_DEMOD_status_regs_base + 0x00000044)
#define s_AVL62X1_DMD_dagc_gain_saddr                     (AVL62X1_DEMOD_status_regs_base + 0x00000046)
#define s_AVL62X1_DMD_sw_time_cnt_iaddr                   (AVL62X1_DEMOD_status_regs_base + 0x0000004c)
#define s_AVL62X1_DMD_last_timer_val_iaddr                (AVL62X1_DEMOD_status_regs_base + 0x00000050)
#define s_AVL62X1_DMD_breakpoint_info_ptr_iaddr           (AVL62X1_DEMOD_status_regs_base + 0x00000054)



#ifndef AVL62X1_DVBS2X_status_regs_base
#define AVL62X1_DVBS2X_status_regs_base (0x00000400)
#endif

#define s_AVL62X1_S2X_circ_buf_slack_iaddr                (AVL62X1_DVBS2X_status_regs_base + 0x00000000)
#define s_AVL62X1_S2X_circ_buf_read_index_iaddr           (AVL62X1_DVBS2X_status_regs_base + 0x00000004)
#define s_AVL62X1_S2X_lock_status_iaddr                   (AVL62X1_DVBS2X_status_regs_base + 0x00000008)
#define s_AVL62X1_S2X_sp_lock_caddr                       (AVL62X1_DVBS2X_status_regs_base + 0x00000008)
#define s_AVL62X1_S2X_lost_lock_caddr                     (AVL62X1_DVBS2X_status_regs_base + 0x00000009)
#define s_AVL62X1_S2X_fec_lock_caddr                      (AVL62X1_DVBS2X_status_regs_base + 0x0000000a)
#define s_AVL62X1_S2X_frame_lock_caddr                    (AVL62X1_DVBS2X_status_regs_base + 0x0000000b)
#define s_AVL62X1_S2X_signal_status_iaddr                 (AVL62X1_DVBS2X_status_regs_base + 0x0000000c)
#define s_AVL62X1_S2X_dvbs_code_rate_caddr                (AVL62X1_DVBS2X_status_regs_base + 0x0000000c)
#define s_AVL62X1_S2X_ccm_pls_mode_caddr                  (AVL62X1_DVBS2X_status_regs_base + 0x0000000d)
#define s_AVL62X1_S2X_ccm1_acm0_caddr                     (AVL62X1_DVBS2X_status_regs_base + 0x0000000e)
#define s_AVL62X1_S2X_signal_type_caddr                   (AVL62X1_DVBS2X_status_regs_base + 0x0000000f)
#define s_AVL62X1_S2X_channel_status_iaddr                (AVL62X1_DVBS2X_status_regs_base + 0x00000010)
#define s_AVL62X1_S2X_phase_noise_bw_ppm_saddr            (AVL62X1_DVBS2X_status_regs_base + 0x00000010)
#define s_AVL62X1_S2X_snr_dB_x100_saddr                   (AVL62X1_DVBS2X_status_regs_base + 0x00000012)
#define s_AVL62X1_S2X_decim_samp_rate_Hz_iaddr            (AVL62X1_DVBS2X_status_regs_base + 0x00000014)
#define s_AVL62X1_S2X_main_state_reg_iaddr                (AVL62X1_DVBS2X_status_regs_base + 0x00000018)
#define s_AVL62X1_S2X_last_main_state_saddr               (AVL62X1_DVBS2X_status_regs_base + 0x00000018)
#define s_AVL62X1_S2X_main_state_saddr                    (AVL62X1_DVBS2X_status_regs_base + 0x0000001a)
#define s_AVL62X1_S2X_symbol_rate_Hz_iaddr                (AVL62X1_DVBS2X_status_regs_base + 0x0000001c)
#define s_AVL62X1_S2X_fec_block_cnt_iaddr                 (AVL62X1_DVBS2X_status_regs_base + 0x00000020)
#define s_AVL62X1_S2X_fec_block_err_cnt_iaddr             (AVL62X1_DVBS2X_status_regs_base + 0x00000024)
#define s_AVL62X1_S2X_demod_status_0_iaddr                (AVL62X1_DVBS2X_status_regs_base + 0x00000028)
#define s_AVL62X1_S2X_carrier_spectrum_invert_status_caddr (AVL62X1_DVBS2X_status_regs_base + 0x00000028)
#define s_AVL62X1_S2X_CurrentFrameIndex_caddr             (AVL62X1_DVBS2X_status_regs_base + 0x00000029)
#define s_AVL62X1_S2X_iq_swap_active_caddr                (AVL62X1_DVBS2X_status_regs_base + 0x0000002a)
#define s_AVL62X1_S2X_acq_retry_count_caddr               (AVL62X1_DVBS2X_status_regs_base + 0x0000002b)
#define s_AVL62X1_S2X_post_viterbi_BER_estimate_x10M_iaddr (AVL62X1_DVBS2X_status_regs_base + 0x0000002c)
#define s_AVL62X1_S2X_pre_LDPC_BER_estimate_x100K_iaddr   (AVL62X1_DVBS2X_status_regs_base + 0x00000034)
#define s_AVL62X1_S2X_btr_status_iaddr                    (AVL62X1_DVBS2X_status_regs_base + 0x00000038)
#define s_AVL62X1_S2X_alpha_caddr                         (AVL62X1_DVBS2X_status_regs_base + 0x00000038)
#define s_AVL62X1_S2X_btr_lock_caddr                      (AVL62X1_DVBS2X_status_regs_base + 0x00000039)
#define s_AVL62X1_S2X_frame_count_iaddr                   (AVL62X1_DVBS2X_status_regs_base + 0x0000003c)
#define s_AVL62X1_S2X_clks_per_sym_reg_iaddr              (AVL62X1_DVBS2X_status_regs_base + 0x00000040)
#define s_AVL62X1_S2X_fec_clks_per_sym_x10_saddr          (AVL62X1_DVBS2X_status_regs_base + 0x00000040)
#define s_AVL62X1_S2X_sys_clks_per_sym_x10_saddr          (AVL62X1_DVBS2X_status_regs_base + 0x00000042)
#define s_AVL62X1_S2X_signal_status_1_iaddr               (AVL62X1_DVBS2X_status_regs_base + 0x00000044)
#define s_AVL62X1_S2X_N_BCH_saddr                         (AVL62X1_DVBS2X_status_regs_base + 0x00000046)
#define s_AVL62X1_S2X_s2_signal_status_0_iaddr            (AVL62X1_DVBS2X_status_regs_base + 0x00000048)
#define s_AVL62X1_S2X_s2_pilot_on_caddr                   (AVL62X1_DVBS2X_status_regs_base + 0x00000048)
#define s_AVL62X1_S2X_s2_fec_len_caddr                    (AVL62X1_DVBS2X_status_regs_base + 0x00000049)
#define s_AVL62X1_S2X_s2_modulation_caddr                 (AVL62X1_DVBS2X_status_regs_base + 0x0000004a)
#define s_AVL62X1_S2X_s2_code_rate_caddr                  (AVL62X1_DVBS2X_status_regs_base + 0x0000004b)
#define s_AVL62X1_S2X_blind_scan_status_iaddr             (AVL62X1_DVBS2X_status_regs_base + 0x0000004c)
#define s_AVL62X1_S2X_bs_num_confirmed_carriers_caddr     (AVL62X1_DVBS2X_status_regs_base + 0x0000004d)
#define s_AVL62X1_S2X_bs_num_raw_qam_carriers_caddr       (AVL62X1_DVBS2X_status_regs_base + 0x0000004e)
#define s_AVL62X1_S2X_bs_num_carrier_candidates_caddr     (AVL62X1_DVBS2X_status_regs_base + 0x0000004f)
#define s_AVL62X1_S2X_demod_mode_status_iaddr             (AVL62X1_DVBS2X_status_regs_base + 0x00000050)
#define s_AVL62X1_S2X_bs_psd_ready_caddr                  (AVL62X1_DVBS2X_status_regs_base + 0x00000050)
#define s_AVL62X1_S2X_current_bs_pair_index_caddr         (AVL62X1_DVBS2X_status_regs_base + 0x00000051)
#define s_AVL62X1_S2X_active_demod_mode_caddr             (AVL62X1_DVBS2X_status_regs_base + 0x00000052)
#define s_AVL62X1_S2X_pre_viterbi_BER_estimate_x10M_iaddr (AVL62X1_DVBS2X_status_regs_base + 0x00000054)
#define s_AVL62X1_S2X_symbol_rate_error_Hz_iaddr          (AVL62X1_DVBS2X_status_regs_base + 0x00000058)
#define s_AVL62X1_S2X_carrier_freq_Hz_iaddr               (AVL62X1_DVBS2X_status_regs_base + 0x0000005c)
#define s_AVL62X1_S2X_carrier_freq_err_Hz_iaddr           (AVL62X1_DVBS2X_status_regs_base + 0x00000060)
#define s_AVL62X1_S2X_bs_next_start_freq_Hz_iaddr         (AVL62X1_DVBS2X_status_regs_base + 0x00000064)
#define s_AVL62X1_S2X_bs_carrier_list_address_iaddr       (AVL62X1_DVBS2X_status_regs_base + 0x00000068)

#ifndef AVL62X1_SP_status_regs_base
#define AVL62X1_SP_status_regs_base (0x00a000a4)
#endif

#define s_AVL62X1_SP_sp_debug_reg_1_iaddr                 (AVL62X1_SP_status_regs_base + 0x00000000)
#define s_AVL62X1_SP_sp_patch_ver_iaddr                   (AVL62X1_SP_status_regs_base + 0x00000004)
#define s_AVL62X1_SP_sp_djb_curr_depth_iaddr              (AVL62X1_SP_status_regs_base + 0x00000008)
#define s_AVL62X1_SP_sp_residue_iaddr                     (AVL62X1_SP_status_regs_base + 0x0000000c)
#define s_AVL62X1_SP_sp_old_frame_cnt_iaddr               (AVL62X1_SP_status_regs_base + 0x00000010)
#define s_AVL62X1_SP_sp_rate_error_iaddr                  (AVL62X1_SP_status_regs_base + 0x00000014)
#define s_AVL62X1_SP_sp_djb_lwm_iaddr                     (AVL62X1_SP_status_regs_base + 0x00000018)
#define s_AVL62X1_SP_sp_djb_hwm_iaddr                     (AVL62X1_SP_status_regs_base + 0x0000001c)
#define s_AVL62X1_SP_sp_ts_rate_est_state_iaddr           (AVL62X1_SP_status_regs_base + 0x00000020)
#define s_AVL62X1_SP_sp_ts_cur_frm_cnt_iaddr              (AVL62X1_SP_status_regs_base + 0x00000024)
#define s_AVL62X1_SP_sp_watchdog_status_iaddr             (AVL62X1_SP_status_regs_base + 0x00000028)
#define s_AVL62X1_SP_sp_frac_d_set_val_iaddr              (AVL62X1_SP_status_regs_base + 0x0000002c)
#define s_AVL62X1_SP_sp_frac_d_norm_val_iaddr             (AVL62X1_SP_status_regs_base + 0x00000030)
#define s_AVL62X1_SP_sp_frac_n_norm_val_iaddr             (AVL62X1_SP_status_regs_base + 0x00000034)
#define s_AVL62X1_SP_sp_misc_status_iaddr                 (AVL62X1_SP_status_regs_base + 0x00000038)
#define s_AVL62X1_SP_sp_sleep_mode_caddr                  (AVL62X1_SP_status_regs_base + 0x00000038)
#define s_AVL62X1_SP_sp_frac_n_set_val_iaddr              (AVL62X1_SP_status_regs_base + 0x00000048)
#define s_AVL62X1_SP_sp_sw_time_cnt_iaddr                 (AVL62X1_SP_status_regs_base + 0x0000004c)
#define s_AVL62X1_SP_sp_last_timer_val_iaddr              (AVL62X1_SP_status_regs_base + 0x00000050)
#define s_AVL62X1_SP_sp_breakpoint_info_ptr_iaddr         (AVL62X1_SP_status_regs_base + 0x00000054)
#define s_AVL62X1_SP_sp_SIS_MIS_caddr                     (AVL62X1_SP_status_regs_base + 0x00000058)
#define s_AVL62X1_SP_sp_raw_t2mi_mode_active_caddr        (AVL62X1_SP_status_regs_base + 0x00000059)


#ifndef AVL62X1_SP_S2X_status_regs_base
#define AVL62X1_SP_S2X_status_regs_base (0x00a00400)
#endif

#define s_AVL62X1_SP_S2X_sp_bb_header_iaddr               (AVL62X1_SP_S2X_status_regs_base + 0x00000000)
#define s_AVL62X1_SP_S2X_sp_main_state_iaddr              (AVL62X1_SP_S2X_status_regs_base + 0x00000010)
#define s_AVL62X1_SP_S2X_sp_last_main_state_iaddr         (AVL62X1_SP_S2X_status_regs_base + 0x00000014)
#define s_AVL62X1_SP_S2X_sp_bb_local_iaddr                (AVL62X1_SP_S2X_status_regs_base + 0x00000018)
#define s_AVL62X1_SP_S2X_sp_local_syncd_saddr             (AVL62X1_SP_S2X_status_regs_base + 0x00000018)
#define s_AVL62X1_SP_S2X_sp_local_dfl_saddr               (AVL62X1_SP_S2X_status_regs_base + 0x0000001a)
#define s_AVL62X1_SP_S2X_sp_bb_frame_acq_status_iaddr     (AVL62X1_SP_S2X_status_regs_base + 0x0000001c)
#define s_AVL62X1_SP_S2X_sp_bb_acquired_caddr             (AVL62X1_SP_S2X_status_regs_base + 0x0000001c)
#define s_AVL62X1_SP_S2X_sp_alpha_detected_caddr          (AVL62X1_SP_S2X_status_regs_base + 0x0000001d)
#define s_AVL62X1_SP_S2X_sp_bb_crc_err_caddr              (AVL62X1_SP_S2X_status_regs_base + 0x0000001e)
#define s_AVL62X1_SP_S2X_sp_bch_err_caddr                 (AVL62X1_SP_S2X_status_regs_base + 0x0000001f)
#define s_AVL62X1_SP_S2X_sp_bb_frame_info_iaddr           (AVL62X1_SP_S2X_status_regs_base + 0x00000020)
#define s_AVL62X1_SP_S2X_sp_CurrentFrameIndex_caddr       (AVL62X1_SP_S2X_status_regs_base + 0x00000020)
#define s_AVL62X1_SP_S2X_sp_lock_caddr                    (AVL62X1_SP_S2X_status_regs_base + 0x00000021)
#define s_AVL62X1_SP_S2X_sp_syncd_matched_caddr           (AVL62X1_SP_S2X_status_regs_base + 0x00000022)
#define s_AVL62X1_SP_S2X_stream_lock_caddr                (AVL62X1_SP_S2X_status_regs_base + 0x00000023)
#define s_AVL62X1_SP_S2X_sp_stream_id_0_iaddr             (AVL62X1_SP_S2X_status_regs_base + 0x00000024)
#define s_AVL62X1_SP_S2X_sp_stream_id_1_iaddr             (AVL62X1_SP_S2X_status_regs_base + 0x00000028)
#define s_AVL62X1_SP_S2X_sp_stream_id_2_iaddr             (AVL62X1_SP_S2X_status_regs_base + 0x0000002c)
#define s_AVL62X1_SP_S2X_sp_stream_id_3_iaddr             (AVL62X1_SP_S2X_status_regs_base + 0x00000030)
#define s_AVL62X1_SP_S2X_sp_stream_id_4_iaddr             (AVL62X1_SP_S2X_status_regs_base + 0x00000034)
#define s_AVL62X1_SP_S2X_sp_stream_id_5_iaddr             (AVL62X1_SP_S2X_status_regs_base + 0x00000038)
#define s_AVL62X1_SP_S2X_sp_stream_id_6_iaddr             (AVL62X1_SP_S2X_status_regs_base + 0x0000003c)
#define s_AVL62X1_SP_S2X_sp_stream_id_7_iaddr             (AVL62X1_SP_S2X_status_regs_base + 0x00000040)
#define s_AVL62X1_SP_S2X_djb_base_addr_iaddr              (AVL62X1_SP_S2X_status_regs_base + 0x00000044)
#define s_AVL62X1_SP_S2X_djb_pkt_status_iaddr             (AVL62X1_SP_S2X_status_regs_base + 0x00000048)
#define s_AVL62X1_SP_S2X_djb_input_pkt_index_caddr        (AVL62X1_SP_S2X_status_regs_base + 0x00000049)
#define s_AVL62X1_SP_S2X_djb_output_pkt_index_caddr       (AVL62X1_SP_S2X_status_regs_base + 0x0000004a)
#define s_AVL62X1_SP_S2X_djb_pkt_size_caddr               (AVL62X1_SP_S2X_status_regs_base + 0x0000004b)
#define s_AVL62X1_SP_S2X_selected_stream_status_iaddr     (AVL62X1_SP_S2X_status_regs_base + 0x0000004c)
#define s_AVL62X1_SP_S2X_StreamParseErrorCount_caddr      (AVL62X1_SP_S2X_status_regs_base + 0x0000004c)
#define s_AVL62X1_SP_S2X_DetectedStreamType_caddr         (AVL62X1_SP_S2X_status_regs_base + 0x0000004d)
#define s_AVL62X1_SP_S2X_SelectedStreamID_caddr           (AVL62X1_SP_S2X_status_regs_base + 0x0000004e)
#define s_AVL62X1_SP_S2X_GenericStreamDialect_caddr       (AVL62X1_SP_S2X_status_regs_base + 0x0000004f)
#define s_AVL62X1_SP_S2X_sp_bb_frame_info2_iaddr          (AVL62X1_SP_S2X_status_regs_base + 0x00000050)
#define s_AVL62X1_SP_S2X_sp_LastFrameIndex_caddr          (AVL62X1_SP_S2X_status_regs_base + 0x00000050)
#define s_AVL62X1_SP_S2X_sp_NumStreams_total_caddr        (AVL62X1_SP_S2X_status_regs_base + 0x00000051)
#define s_AVL62X1_SP_S2X_sp_NumStreams_cur_TP_caddr       (AVL62X1_SP_S2X_status_regs_base + 0x00000052)
#define s_AVL62X1_SP_S2X_sp_stream_discover_done_caddr    (AVL62X1_SP_S2X_status_regs_base + 0x00000053)
#define s_AVL62X1_SP_S2X_sp_DVB_STREAM_addr_iaddr         (AVL62X1_SP_S2X_status_regs_base + 0x00000054)
#define s_AVL62X1_SP_S2X_sp_bb_frame_info3_iaddr          (AVL62X1_SP_S2X_status_regs_base + 0x00000058)
#define s_AVL62X1_SP_S2X_sp_SIS_MIS_caddr                 (AVL62X1_SP_S2X_status_regs_base + 0x00000058)
#define s_AVL62X1_SP_S2X_sp_t2mi_mplp_id_num_caddr        (AVL62X1_SP_S2X_status_regs_base + 0x0000005a)
#define s_AVL62X1_SP_S2X_sp_DVB_STREAM2_addr_iaddr        (AVL62X1_SP_S2X_status_regs_base + 0x0000005c)
#define s_AVL62X1_SP_S2X_sp_mplp_id_list_iaddr            (AVL62X1_SP_S2X_status_regs_base + 0x00000060)  //max buffer 16byte
//SAT_CARRIER FW struct addresses
#define AVL62X1_SAT_CARRIER_CarrierFreqHz_iaddr                              (0x00000000)
#define AVL62X1_SAT_CARRIER_SymbolRateHz_iaddr                               (0x00000004)
#define AVL62X1_SAT_CARRIER_SignalType_caddr                                 (0x00000008)
#define AVL62X1_SAT_CARRIER_PLScramKeyMSBs_caddr                             (0x00000009)
#define AVL62X1_SAT_CARRIER_PLScramKeyLSBs_saddr                             (0x0000000a)
#define AVL62X1_SAT_CARRIER_PLS_ACM_caddr                                    (0x0000000c)
#define AVL62X1_SAT_CARRIER_NumStreams_caddr                                 (0x0000000d)
#define AVL62X1_SAT_CARRIER_SNR_dB_x100_saddr                                (0x0000000e)
#define AVL62X1_SAT_CARRIER_Mod_caddr                                        (0x00000010)
#define AVL62X1_SAT_CARRIER_Pilot_caddr                                      (0x00000011)
#define AVL62X1_SAT_CARRIER_FECLen_caddr                                     (0x00000012)
#define AVL62X1_SAT_CARRIER_CodeRate_caddr                                   (0x00000013)
#define AVL62X1_SAT_CARRIER_struct_size                                      (AVL62X1_SAT_CARRIER_CodeRate_caddr + 1)

//DVB_STREAM FW struct addresses
#define AVL62X1_DVB_STREAM_CarrierIndex_caddr                               (0x00000000)
#define AVL62X1_DVB_STREAM_StreamType_caddr                                 (0x00000001)
#define AVL62X1_DVB_STREAM_PLP_ID_caddr                                     (0x00000002)
#define AVL62X1_DVB_STREAM_ISI_caddr                                        (0x00000003)
#define AVL62X1_DVB_STREAM_struct_size                                      (AVL62X1_DVB_STREAM_ISI_caddr + 1)

#define AVL62X1_DVB_STREAM_T2MI_PID_caddr                                   (900)  // address offset with AVL62X1_DVB_STREAM_CarrierIndex_caddr for T2MI PID

#ifndef AVL62X1_reset_manager__base
#define AVL62X1_reset_manager__base                                                                  0x00100000
#endif
#define AVL62X1_reset_manager__reset_register__offset                                                0x00000000 /* type = RW */
#define AVL62X1_reset_manager__system_srst__offset                                                   0x00000004 /* type = RW */
#define AVL62X1_reset_manager__system_srst_ack__offset                                               0x00000008 /* type = RD */
#define AVL62X1_reset_manager__reset_delay__offset                                                   0x0000000c /* type = RW, reset_val = 5000 */
#define AVL62X1_reset_manager__xtl_disable_reg__offset                                               0x00000010 /* type = RW, reset_val = 0 */
#define AVL62X1_reset_manager__spi_enable__offset                                                    0x00000014 /* type = RW, reset_val = 0x5A5AFFFF */
#define AVL62X1_reset_manager__fec_clk_enable__offset                                                0x00000018 /* type = RW, reset_val = 1 */
#define AVL62X1_reset_manager__mpeg_ref_clk_enable__offset                                           0x0000001c /* type = RW, reset_val = 1 */
#define AVL62X1_reset_manager__adc_clk_src_sel_reg__offset                                           0x00000020 /* type = RW, reset_val = 0 */
#define AVL62X1_reset_manager__test_reg__offset                                                      0x00000024 /* type = RW, reset_val = 0x20160208 */
#define AVL62X1_reset_manager__reg_pll_bypass__offset                                                0x00000040 /* type = RW, reset_val = 0 */
#define AVL62X1_reset_manager__reg_pll_enable1__offset                                               0x00000044 /* type = RW, reset_val = 1 */
#define AVL62X1_reset_manager__reg_pll_divr__offset                                                  0x00000048 /* type = RW, reset_val = 1 */
#define AVL62X1_reset_manager__reg_pll_divf__offset                                                  0x0000004c /* type = RW, reset_val = 77 */
#define AVL62X1_reset_manager__reg_pll_divq1__offset                                                 0x00000050 /* type = RW, reset_val = 17 */
#define AVL62X1_reset_manager__reg_pll_divq2__offset                                                 0x00000054 /* type = RW, reset_val = 15 */
#define AVL62X1_reset_manager__reg_pll_divq3__offset                                                 0x00000058 /* type = RW, reset_val = 23 */
#define AVL62X1_reset_manager__reg_pll_range__offset                                                 0x0000005c /* type = RW, reset_val = 1 */
#define AVL62X1_reset_manager__reg_pll_lock__offset                                                  0x00000060 /* type = RD */
#define AVL62X1_reset_manager__reg_pll_enable2__offset                                               0x00000064 /* type = RW, reset_val = 1 */
#define AVL62X1_reset_manager__reg_pll_enable3__offset                                               0x00000068 /* type = RW, reset_val = 1 */
#define AVL62X1_reset_manager__reg_pll_reset__offset                                                 0x0000006c /* type = RW, reset_val = 0 */
#define AVL62X1_reset_manager__reg_pll_div2qen1__offset                                              0x00000070 /* type = RW, reset_val = 0 */
#define AVL62X1_reset_manager__reg_pll_div2qen2__offset                                              0x00000074 /* type = RW, reset_val = 0 */
#define AVL62X1_reset_manager__reg_pll_div2qen3__offset                                              0x00000078 /* type = RW, reset_val = 0 */
#define AVL62X1_reset_manager__reg_pll_load__offset                                                  0x0000007c /* type = RW, reset_val = 0 */
#define AVL62X1_reset_manager__sys_clk_sel_XI__offset                                                0x00000080 /* type = RW, reset_val = 0 */
#define AVL62X1_reset_manager__chip_id__offset                                                       0x00000084 /* type = RD */

#define AVL62X1_reset_manager__reset_register               (AVL62X1_reset_manager__base + AVL62X1_reset_manager__reset_register__offset)
#define AVL62X1_reset_manager__system_srst                  (AVL62X1_reset_manager__base + AVL62X1_reset_manager__system_srst__offset)
#define AVL62X1_reset_manager__system_srst_ack              (AVL62X1_reset_manager__base + AVL62X1_reset_manager__system_srst_ack__offset)
#define AVL62X1_reset_manager__reset_delay                  (AVL62X1_reset_manager__base + AVL62X1_reset_manager__reset_delay__offset)
#define AVL62X1_reset_manager__xtl_disable_reg              (AVL62X1_reset_manager__base + AVL62X1_reset_manager__xtl_disable_reg__offset)
#define AVL62X1_reset_manager__spi_enable                   (AVL62X1_reset_manager__base + AVL62X1_reset_manager__spi_enable__offset)
#define AVL62X1_reset_manager__fec_clk_enable               (AVL62X1_reset_manager__base + AVL62X1_reset_manager__fec_clk_enable__offset)
#define AVL62X1_reset_manager__mpeg_ref_clk_enable          (AVL62X1_reset_manager__base + AVL62X1_reset_manager__mpeg_ref_clk_enable__offset)
#define AVL62X1_reset_manager__adc_clk_src_sel_reg          (AVL62X1_reset_manager__base + AVL62X1_reset_manager__adc_clk_src_sel_reg__offset)
#define AVL62X1_reset_manager__test_reg                     (AVL62X1_reset_manager__base + AVL62X1_reset_manager__test_reg__offset)
#define AVL62X1_reset_manager__reg_pll_bypass               (AVL62X1_reset_manager__base + AVL62X1_reset_manager__reg_pll_bypass__offset)
#define AVL62X1_reset_manager__reg_pll_enable1              (AVL62X1_reset_manager__base + AVL62X1_reset_manager__reg_pll_enable1__offset)
#define AVL62X1_reset_manager__reg_pll_divr                 (AVL62X1_reset_manager__base + AVL62X1_reset_manager__reg_pll_divr__offset)
#define AVL62X1_reset_manager__reg_pll_divf                 (AVL62X1_reset_manager__base + AVL62X1_reset_manager__reg_pll_divf__offset)
#define AVL62X1_reset_manager__reg_pll_divq1                (AVL62X1_reset_manager__base + AVL62X1_reset_manager__reg_pll_divq1__offset)
#define AVL62X1_reset_manager__reg_pll_divq2                (AVL62X1_reset_manager__base + AVL62X1_reset_manager__reg_pll_divq2__offset)
#define AVL62X1_reset_manager__reg_pll_divq3                (AVL62X1_reset_manager__base + AVL62X1_reset_manager__reg_pll_divq3__offset)
#define AVL62X1_reset_manager__reg_pll_range                (AVL62X1_reset_manager__base + AVL62X1_reset_manager__reg_pll_range__offset)
#define AVL62X1_reset_manager__reg_pll_lock                 (AVL62X1_reset_manager__base + AVL62X1_reset_manager__reg_pll_lock__offset)
#define AVL62X1_reset_manager__reg_pll_enable2              (AVL62X1_reset_manager__base + AVL62X1_reset_manager__reg_pll_enable2__offset)
#define AVL62X1_reset_manager__reg_pll_enable3              (AVL62X1_reset_manager__base + AVL62X1_reset_manager__reg_pll_enable3__offset)
#define AVL62X1_reset_manager__reg_pll_reset                (AVL62X1_reset_manager__base + AVL62X1_reset_manager__reg_pll_reset__offset)
#define AVL62X1_reset_manager__reg_pll_div2qen1             (AVL62X1_reset_manager__base + AVL62X1_reset_manager__reg_pll_div2qen1__offset)
#define AVL62X1_reset_manager__reg_pll_div2qen2             (AVL62X1_reset_manager__base + AVL62X1_reset_manager__reg_pll_div2qen2__offset)
#define AVL62X1_reset_manager__reg_pll_div2qen3             (AVL62X1_reset_manager__base + AVL62X1_reset_manager__reg_pll_div2qen3__offset)
#define AVL62X1_reset_manager__reg_pll_load                 (AVL62X1_reset_manager__base + AVL62X1_reset_manager__reg_pll_load__offset)
#define AVL62X1_reset_manager__sys_clk_sel_XI               (AVL62X1_reset_manager__base + AVL62X1_reset_manager__sys_clk_sel_XI__offset)
#define AVL62X1_reset_manager__chip_id                      (AVL62X1_reset_manager__base + AVL62X1_reset_manager__chip_id__offset)


#ifndef gpio_debug__base
#define gpio_debug__base                                                                           0x00120000
#endif
#define gpio_debug__agc1_sel__offset                                                               0x00000000 /* type = RW, reset_val = 2 */
#define gpio_debug__gpio1_sel__offset                                                              0x00000004 /* type = RW, reset_val = 2 */
#define gpio_debug__lnb_cntrl_1_sel__offset                                                        0x00000008 /* type = RW, reset_val = 2 */
#define gpio_debug__lnb_cntrl_0_sel__offset                                                        0x0000000c /* type = RW, reset_val = 2 */
#define gpio_debug__agc2_sel__offset                                                               0x00000010 /* type = RW, reset_val = 2 */
#define gpio_debug__i2c_data2_sel__offset                                                          0x00000014 /* type = RW, reset_val = 8 */
#define gpio_debug__i2c_clk2_sel__offset                                                           0x00000018 /* type = RW, reset_val = 7 */
#define gpio_debug__agc1_i__offset                                                                 0x00000040 /* type = RD */
#define gpio_debug__gpio1_i__offset                                                                0x00000044 /* type = RD */
#define gpio_debug__lnb_cntrl_1_i__offset                                                          0x00000048 /* type = RD */
#define gpio_debug__lnb_cntrl_0_i__offset                                                          0x0000004c /* type = RD */
#define gpio_debug__agc2_i__offset                                                                 0x00000050 /* type = RD */
#define gpio_debug__i2c_data2_i__offset                                                            0x00000054 /* type = RD */
#define gpio_debug__i2c_clk2_i__offset                                                             0x00000058 /* type = RD */
#define gpio_debug__m3_scl_sel__offset                                                             0x0000009c /* type = RW, reset_val = 6 */
#define gpio_debug__m3_sda_sel__offset                                                             0x000000a0 /* type = RW, reset_val = 5 */

#define gpio_debug__agc1_sel                                                               (gpio_debug__base+0x00000000) /* type = RW, reset_val = 2 */
#define gpio_debug__gpio1_sel                                                              (gpio_debug__base+0x00000004) /* type = RW, reset_val = 2 */
#define gpio_debug__lnb_cntrl_1_sel                                                        (gpio_debug__base+0x00000008) /* type = RW, reset_val = 2 */
#define gpio_debug__lnb_cntrl_0_sel                                                        (gpio_debug__base+0x0000000c) /* type = RW, reset_val = 2 */
#define gpio_debug__agc2_sel                                                               (gpio_debug__base+0x00000010) /* type = RW, reset_val = 2 */
#define gpio_debug__i2c_data2_sel                                                          (gpio_debug__base+0x00000014) /* type = RW, reset_val = 8 */
#define gpio_debug__i2c_clk2_sel                                                           (gpio_debug__base+0x00000018) /* type = RW, reset_val = 7 */
#define gpio_debug__agc1_i                                                                 (gpio_debug__base+0x00000040) /* type = RD */
#define gpio_debug__gpio1_i                                                                (gpio_debug__base+0x00000044) /* type = RD */
#define gpio_debug__lnb_cntrl_1_i                                                          (gpio_debug__base+0x00000048) /* type = RD */
#define gpio_debug__lnb_cntrl_0_i                                                          (gpio_debug__base+0x0000004c) /* type = RD */
#define gpio_debug__agc2_i                                                                 (gpio_debug__base+0x00000050) /* type = RD */
#define gpio_debug__i2c_data2_i                                                            (gpio_debug__base+0x00000054) /* type = RD */
#define gpio_debug__i2c_clk2_i                                                             (gpio_debug__base+0x00000058) /* type = RD */
#define gpio_debug__m3_scl_sel                                                             (gpio_debug__base+0x0000009c) /* type = RW, reset_val = 6 */
#define gpio_debug__m3_sda_sel                                                             (gpio_debug__base+0x000000a0) /* type = RW, reset_val = 5 */


#ifndef psc_tuner_i2c__base
#define psc_tuner_i2c__base                                                                        0x00118000
#endif
#define psc_tuner_i2c__tuner_i2c_srst__offset                                                      0x00000000 /* type = RW */
#define psc_tuner_i2c__tuner_i2c_cntrl__offset                                                     0x00000004 /* type = RW */
#define psc_tuner_i2c__tuner_i2c_clk_div__offset                                                   0x00000008 /* type = RW */
#define psc_tuner_i2c__tuner_i2c_intr_flag__offset                                                 0x0000000c /* type = RW */
#define psc_tuner_i2c__tuner_i2c_read_cnt__offset                                                  0x00000010 /* type = RW */
#define psc_tuner_i2c__tuner_i2c_error_flag__offset                                                0x00000014 /* type = RD */
#define psc_tuner_i2c__tuner_hw_i2c_bit_rpt_clk_div__offset                                        0x00000018 /* type = RW */
#define psc_tuner_i2c__tuner_hw_i2c_bit_rpt_cntrl__offset                                          0x0000001c /* type = RW */
#define psc_tuner_i2c__tuner_i2cm__offset                                                          0x00000400 /* type = MAP */

#define psc_tuner_i2c__tuner_i2c_srst               (psc_tuner_i2c__base + psc_tuner_i2c__tuner_i2c_srst__offset)
#define psc_tuner_i2c__tuner_i2c_cntrl              (psc_tuner_i2c__base + psc_tuner_i2c__tuner_i2c_cntrl__offset)
#define psc_tuner_i2c__tuner_i2c_clk_div            (psc_tuner_i2c__base + psc_tuner_i2c__tuner_i2c_clk_div__offset)
#define psc_tuner_i2c__tuner_i2c_intr_flag          (psc_tuner_i2c__base + psc_tuner_i2c__tuner_i2c_intr_flag__offset)
#define psc_tuner_i2c__tuner_i2c_read_cnt           (psc_tuner_i2c__base + psc_tuner_i2c__tuner_i2c_read_cnt__offset)
#define psc_tuner_i2c__tuner_i2c_error_flag         (psc_tuner_i2c__base + psc_tuner_i2c__tuner_i2c_error_flag__offset)
#define psc_tuner_i2c__tuner_hw_i2c_bit_rpt_clk_div (psc_tuner_i2c__base + psc_tuner_i2c__tuner_hw_i2c_bit_rpt_clk_div__offset)
#define psc_tuner_i2c__tuner_hw_i2c_bit_rpt_cntrl   (psc_tuner_i2c__base + psc_tuner_i2c__tuner_hw_i2c_bit_rpt_cntrl__offset)
#define psc_tuner_i2c__tuner_i2cm                   (psc_tuner_i2c__base + psc_tuner_i2c__tuner_i2cm__offset)



#ifndef aagc__base
#define aagc__base                                                                                 0x00160900
#endif
#define aagc__analog_agc_sd_control_reg__offset                                                    0x00000000 /* type = RW */
#define aagc__analog_agc_init_reg__offset                                                          0x00000004 /* type = RW */
#define aagc__analog_agc_loop_gain_reg__offset                                                     0x00000008 /* type = RW */
#define aagc__analog_agc_ref_reg__offset                                                           0x0000000c /* type = RW */
#define aagc__analog_agc_min_gain_reg__offset                                                      0x00000010 /* type = RW */
#define aagc__analog_agc_max_gain_reg__offset                                                      0x00000014 /* type = RW */
#define aagc__analog_agc_control_reg__offset                                                       0x00000018 /* type = RW */
#define aagc__analog_agc_gain_reg__offset                                                          0x0000001c /* type = RD */

#define aagc__analog_agc_sd_control_reg                (aagc__base + aagc__analog_agc_sd_control_reg__offset)
#define aagc__analog_agc_init_reg                      (aagc__base + aagc__analog_agc_init_reg__offset)
#define aagc__analog_agc_loop_gain_reg                 (aagc__base + aagc__analog_agc_loop_gain_reg__offset)
#define aagc__analog_agc_ref_reg                       (aagc__base + aagc__analog_agc_ref_reg__offset)
#define aagc__analog_agc_min_gain_reg                  (aagc__base + aagc__analog_agc_min_gain_reg__offset)
#define aagc__analog_agc_max_gain_reg                  (aagc__base + aagc__analog_agc_max_gain_reg__offset)
#define aagc__analog_agc_control_reg                   (aagc__base + aagc__analog_agc_control_reg__offset)
#define aagc__analog_agc_gain_reg                      (aagc__base + aagc__analog_agc_gain_reg__offset)



#ifndef diseqc__base
#define diseqc__base                                                                               0x0016c000
#endif
#define diseqc__diseqc_tx_cntrl__offset                                                            0x00000000 /* type = RW */
#define diseqc__diseqc_tone_frac_n__offset                                                         0x00000004 /* type = RW, reset_val = 11 */
#define diseqc__diseqc_tone_frac_d__offset                                                         0x00000008 /* type = RW, reset_val = 35000 */
#define diseqc__diseqc_tx_st__offset                                                               0x0000000c /* type = RD */
#define diseqc__rx_msg_tim__offset                                                                 0x00000014 /* type = RW, reset_val = 0 */
#define diseqc__diseqc_rx_st__offset                                                               0x00000018 /* type = RD */
#define diseqc__diseqc_rx_cntrl__offset                                                            0x0000001c /* type = RW */
#define diseqc__diseqc_srst__offset                                                                0x00000020 /* type = RW, reset_val = 1 */
#define diseqc__bit_time__offset                                                                   0x00000024 /* type = RW, reset_val = 26 */
#define diseqc__diseqc_samp_frac_n__offset                                                         0x00000028 /* type = RW, reset_val = 220 */
#define diseqc__diseqc_samp_frac_d__offset                                                         0x0000002c /* type = RW, reset_val = 35000 */
#define diseqc__bit_decode_range__offset                                                           0x00000030 /* type = RW */
#define diseqc__tx_fifo_curr_depth__offset                                                         0x00000034 /* type = RD */
#define diseqc__rx_fifo_curr_depth__offset                                                         0x00000038 /* type = RD */
#define diseqc__rx_fifo__offset                                                                    0x00000040 /* type = MAP */
#define diseqc__tx_fifo__offset                                                                    0x00000080 /* type = MAP */

#define diseqc__diseqc_tx_cntrl                   (diseqc__base +  diseqc__diseqc_tx_cntrl__offset)
#define diseqc__diseqc_tone_frac_n                (diseqc__base +  diseqc__diseqc_tone_frac_n__offset)
#define diseqc__diseqc_tone_frac_d                (diseqc__base +  diseqc__diseqc_tone_frac_d__offset)
#define diseqc__diseqc_tx_st                      (diseqc__base +  diseqc__diseqc_tx_st__offset)
#define diseqc__rx_msg_tim                        (diseqc__base +  diseqc__rx_msg_tim__offset)
#define diseqc__diseqc_rx_st                      (diseqc__base +  diseqc__diseqc_rx_st__offset)
#define diseqc__diseqc_rx_cntrl                   (diseqc__base +  diseqc__diseqc_rx_cntrl__offset)
#define diseqc__diseqc_srst                       (diseqc__base +  diseqc__diseqc_srst__offset)
#define diseqc__bit_time                          (diseqc__base +  diseqc__bit_time__offset)
#define diseqc__diseqc_samp_frac_n                (diseqc__base +  diseqc__diseqc_samp_frac_n__offset)
#define diseqc__diseqc_samp_frac_d                (diseqc__base +  diseqc__diseqc_samp_frac_d__offset)
#define diseqc__bit_decode_range                  (diseqc__base +  diseqc__bit_decode_range__offset)
#define diseqc__tx_fifo_curr_depth                (diseqc__base +  diseqc__tx_fifo_curr_depth__offset)
#define diseqc__rx_fifo_curr_depth                (diseqc__base +  diseqc__rx_fifo_curr_depth__offset)
#define diseqc__rx_fifo                           (diseqc__base +  diseqc__rx_fifo__offset)
#define diseqc__tx_fifo                           (diseqc__base +  diseqc__tx_fifo__offset)



#ifndef ber_esm__base
#define ber_esm__base                                                                              0x00174000
#endif
#define ber_esm__srst__offset                                                                      0x00000000 /* type = RW, reset_val = 0 */
#define ber_esm__esm_async_srst__offset                                                            0x00000004 /* type = RW, reset_val = 0 */
#define ber_esm__afifo_srst__offset                                                                0x00000008 /* type = RW, reset_val = 0 */
#define ber_esm__esm_cntrl__offset                                                                 0x00000054 /* type = RW */
#define ber_esm__ber_err_cnt__offset                                                               0x00000058 /* type = RD */
#define ber_esm__byte_num__offset                                                                  0x0000005c /* type = RD */
#define ber_esm__packet_err_cnt__offset                                                            0x00000060 /* type = RD */
#define ber_esm__packet_num__offset                                                                0x00000064 /* type = RD */
#define ber_esm__ber_err_rollover_cnt__offset                                                      0x000000c8 /* type = RD */
#define ber_esm__packet_err_rollover_cnt__offset                                                   0x000000cc /* type = RD */
#define ber_esm__byte_rollover_num__offset                                                         0x000000d0 /* type = RD */
#define ber_esm__packet_rollover_num__offset                                                       0x000000d4 /* type = RD */
#define ber_esm__tick_clear_req__offset                                                            0x000000d8 /* type = RW, reset_val = 0 */
#define ber_esm__timetick1_bytetick0__offset                                                       0x000000dc /* type = RW, reset_val = 1 */
#define ber_esm__time_tick_low__offset                                                             0x000000e0 /* type = RW, reset_val = 0xffffffff */
#define ber_esm__time_tick_high__offset                                                            0x000000e4 /* type = RW, reset_val = 0xfff */
#define ber_esm__byte_tick_low__offset                                                             0x000000e8 /* type = RW, reset_val = 0xffffffff */
#define ber_esm__byte_tick_high__offset                                                            0x000000ec /* type = RW, reset_val = 0xfff */
#define ber_esm__ber_err_num_low__offset                                                           0x000000f0 /* type = RD */
#define ber_esm__packet_err_num_low__offset                                                        0x000000f4 /* type = RD */
#define ber_esm__byte_num_low__offset                                                              0x000000f8 /* type = RD */
#define ber_esm__packet_num_low__offset                                                            0x000000fc /* type = RD */
#define ber_esm__ber_err_num_high__offset                                                          0x00000100 /* type = RD */
#define ber_esm__packet_err_num_high__offset                                                       0x00000104 /* type = RD */
#define ber_esm__byte_num_high__offset                                                             0x00000108 /* type = RD */
#define ber_esm__packet_num_high__offset                                                           0x0000010c /* type = RD */
#define ber_esm__auto1_manual0_mode__offset                                                        0x00000110 /* type = RW, reset_val = 0 */
#define ber_esm__total_null_pkt_cnt__offset                                                        0x00000114 /* type = RD, reset_val = 0 */
#define ber_esm__esm_cntrl_1__offset                                                               0x00000118 /* type = RW */
#define ber_esm__pid_mask__offset                                                                  0x0000011c /* type = RW */
#define ber_esm__continuity_cnt_err_clr__offset                                                    0x00000120 /* type = WT, reset_val = 0 */
#define ber_esm__continuity_cnt_err__offset                                                        0x00000124 /* type = RD */
#define ber_esm__continuity_cnt_debug__offset                                                      0x00000128 /* type = RD */
#define ber_esm__continuity_cnt_pid__offset                                                        0x0000012c /* type = RW, reset_val = 0x100 */
#define ber_esm__hw_version__offset                                                                0x00000130 /* type = RD */

#define ber_esm__srst                              (ber_esm__base + ber_esm__srst__offset)
#define ber_esm__esm_async_srst                    (ber_esm__base + ber_esm__esm_async_srst__offset)
#define ber_esm__afifo_srst                        (ber_esm__base + ber_esm__afifo_srst__offset)
#define ber_esm__esm_cntrl                         (ber_esm__base + ber_esm__esm_cntrl__offset)
#define ber_esm__ber_err_cnt                       (ber_esm__base + ber_esm__ber_err_cnt__offset)
#define ber_esm__byte_num                          (ber_esm__base + ber_esm__byte_num__offset)
#define ber_esm__packet_err_cnt                    (ber_esm__base + ber_esm__packet_err_cnt__offset)
#define ber_esm__packet_num                        (ber_esm__base + ber_esm__packet_num__offset)
#define ber_esm__ber_err_rollover_cnt              (ber_esm__base + ber_esm__ber_err_rollover_cnt__offset)
#define ber_esm__packet_err_rollover_cnt           (ber_esm__base + ber_esm__packet_err_rollover_cnt__offset)
#define ber_esm__byte_rollover_num                 (ber_esm__base + ber_esm__byte_rollover_num__offset)
#define ber_esm__packet_rollover_num               (ber_esm__base + ber_esm__packet_rollover_num__offset)
#define ber_esm__tick_clear_req                    (ber_esm__base + ber_esm__tick_clear_req__offset)
#define ber_esm__timetick1_bytetick0               (ber_esm__base + ber_esm__timetick1_bytetick0__offset)
#define ber_esm__time_tick_low                     (ber_esm__base + ber_esm__time_tick_low__offset)
#define ber_esm__time_tick_high                    (ber_esm__base + ber_esm__time_tick_high__offset)
#define ber_esm__byte_tick_low                     (ber_esm__base + ber_esm__byte_tick_low__offset)
#define ber_esm__byte_tick_high                    (ber_esm__base + ber_esm__byte_tick_high__offset)
#define ber_esm__ber_err_num_low                   (ber_esm__base + ber_esm__ber_err_num_low__offset)
#define ber_esm__packet_err_num_low                (ber_esm__base + ber_esm__packet_err_num_low__offset)
#define ber_esm__byte_num_low                      (ber_esm__base + ber_esm__byte_num_low__offset)
#define ber_esm__packet_num_low                    (ber_esm__base + ber_esm__packet_num_low__offset)
#define ber_esm__ber_err_num_high                  (ber_esm__base + ber_esm__ber_err_num_high__offset)
#define ber_esm__packet_err_num_high               (ber_esm__base + ber_esm__packet_err_num_high__offset)
#define ber_esm__byte_num_high                     (ber_esm__base + ber_esm__byte_num_high__offset)
#define ber_esm__packet_num_high                   (ber_esm__base + ber_esm__packet_num_high__offset)
#define ber_esm__auto1_manual0_mode                (ber_esm__base + ber_esm__auto1_manual0_mode__offset)
#define ber_esm__total_null_pkt_cnt                (ber_esm__base + ber_esm__total_null_pkt_cnt__offset)
#define ber_esm__esm_cntrl_1                       (ber_esm__base + ber_esm__esm_cntrl_1__offset)
#define ber_esm__pid_mask                          (ber_esm__base + ber_esm__pid_mask__offset)
#define ber_esm__continuity_cnt_err_clr            (ber_esm__base + ber_esm__continuity_cnt_err_clr__offset)
#define ber_esm__continuity_cnt_err                (ber_esm__base + ber_esm__continuity_cnt_err__offset)
#define ber_esm__continuity_cnt_debug              (ber_esm__base + ber_esm__continuity_cnt_debug__offset)
#define ber_esm__continuity_cnt_pid                (ber_esm__base + ber_esm__continuity_cnt_pid__offset)
#define ber_esm__hw_version                        (ber_esm__base + ber_esm__hw_version__offset)


#ifndef ts_output_intf__base
#define ts_output_intf__base                                                                       0x00b08400
#endif
#define ts_output_intf__idle_frac_n__offset                                                        0x00000000 /* type = RW, reset_val = 1 */
#define ts_output_intf__idle_frac_d__offset                                                        0x00000004 /* type = RW, reset_val = 4 */
#define ts_output_intf__seri_format__offset                                                        0x00000008 /* type = RW */
#define ts_output_intf__mpeg_bus_format__offset                                                    0x0000000c /* type = RW */
#define ts_output_intf__mpeg_bus_off__offset                                                       0x00000010 /* type = RW */
#define ts_output_intf__mpeg_bus_off_val__offset                                                   0x00000014 /* type = RW, reset_val = 0 */
#define ts_output_intf__mpeg_bus_misc__offset                                                      0x00000018 /* type = RW */
#define ts_output_intf__sync_word__offset                                                          0x0000001c /* type = RW, reset_val = 0x47 */
#define ts_output_intf__mpeg_bus_e_b__offset                                                       0x00000020 /* type = RW, reset_val = 0xfff */
#define ts_output_intf__idle_clk_en__offset                                                        0x00000024 /* type = RW, reset_val = 0x0 */
#define ts_output_intf__idle_clk_srst__offset                                                      0x00000028 /* type = RW, reset_val = 0x0 */
#define ts_output_intf__intnl_lfsr__offset                                                         0x0000002c /* type = RW */
#define ts_output_intf__use_idle_clk__offset                                                       0x00000030 /* type = RW, reset_val = 0x0 */
#define ts_output_intf__inst_switch__offset                                                        0x00000034 /* type = RW, reset_val = 0x0 */
#define ts_output_intf__mpeg_clk_phase__offset                                                     0x00000038 /* type = RW, reset_val = 0 */
#define ts_output_intf__cntns_div__offset                                                          0x0000003c /* type = RW, reset_val = 1 */
#define ts_output_intf__input_fifo_target_depth__offset                                            0x00000040 /* type = RW, reset_val = input_fifo_depth */
#define ts_output_intf__start_early_seri_clk_en__offset                                            0x00000044 /* type = RW, reset_val = 1 */
#define ts_output_intf__disable_mpeg_clk_last_16_bytes__offset                                     0x00000048 /* type = RW, reset_val = 0 */
#define ts_output_intf__multi_bit_serial_mode__offset                                              0x0000004c /* type = RW, reset_val = 0 */
#define ts_output_intf__reverse_multi_bit_serial__offset                                           0x00000050 /* type = RW, reset_val = 0 */

#define ts_output_intf__idle_frac_n                                                        (ts_output_intf__base+0x00000000) /* type = RW, reset_val = 1 */
#define ts_output_intf__idle_frac_d                                                        (ts_output_intf__base+0x00000004) /* type = RW, reset_val = 4 */
#define ts_output_intf__seri_format                                                        (ts_output_intf__base+0x00000008) /* type = RW */
#define ts_output_intf__mpeg_bus_format                                                    (ts_output_intf__base+0x0000000c) /* type = RW */
#define ts_output_intf__mpeg_bus_off                                                       (ts_output_intf__base+0x00000010) /* type = RW */
#define ts_output_intf__mpeg_bus_off_val                                                   (ts_output_intf__base+0x00000014) /* type = RW, reset_val = 0 */
#define ts_output_intf__mpeg_bus_misc                                                      (ts_output_intf__base+0x00000018) /* type = RW */
#define ts_output_intf__sync_word                                                          (ts_output_intf__base+0x0000001c) /* type = RW, reset_val = (ts_output_intf__base+0x47 */
#define ts_output_intf__mpeg_bus_e_b                                                       (ts_output_intf__base+0x00000020) /* type = RW, reset_val = (ts_output_intf__base+0xfff */
#define ts_output_intf__idle_clk_en                                                        (ts_output_intf__base+0x00000024) /* type = RW, reset_val = (ts_output_intf__base+0x0 */
#define ts_output_intf__idle_clk_srst                                                      (ts_output_intf__base+0x00000028) /* type = RW, reset_val = (ts_output_intf__base+0x0 */
#define ts_output_intf__intnl_lfsr                                                         (ts_output_intf__base+0x0000002c) /* type = RW */
#define ts_output_intf__use_idle_clk                                                       (ts_output_intf__base+0x00000030) /* type = RW, reset_val = (ts_output_intf__base+0x0 */
#define ts_output_intf__inst_switch                                                        (ts_output_intf__base+0x00000034) /* type = RW, reset_val = (ts_output_intf__base+0x0 */
#define ts_output_intf__mpeg_clk_phase                                                     (ts_output_intf__base+0x00000038) /* type = RW, reset_val = 0 */
#define ts_output_intf__cntns_div                                                          (ts_output_intf__base+0x0000003c) /* type = RW, reset_val = 1 */
#define ts_output_intf__input_fifo_target_depth                                            (ts_output_intf__base+0x00000040) /* type = RW, reset_val = input_fifo_depth */
#define ts_output_intf__start_early_seri_clk_en                                            (ts_output_intf__base+0x00000044) /* type = RW, reset_val = 1 */
#define ts_output_intf__disable_mpeg_clk_last_16_bytes                                     (ts_output_intf__base+0x00000048) /* type = RW, reset_val = 0 */
#define ts_output_intf__multi_bit_serial_mode                                              (ts_output_intf__base+0x0000004c) /* type = RW, reset_val = 0 */
#define ts_output_intf__reverse_multi_bit_serial                                           (ts_output_intf__base+0x00000050) /* type = RW, reset_val = 0 */


#ifndef cpu_system__base
#define cpu_system__base                                                                           0x00110000
#endif

/* register address offset : ${BLOCKNAME}__${REGISTERNAME}__offset */
#define cpu_system__stdout_fifo_cdepth__offset                                                     0x00000004 /* type = RD, reset_val = 0 */
#define cpu_system__cpu_pc_cnt__offset                                                             0x00000008 /* type = RD */
#define cpu_system__cpu_error_codes__offset                                                        0x0000000c /* type = RD */
#define cpu_system__cpu_srst__offset                                                               0x00000010 /* type = RW */
#define cpu_system__intr_vec_reg__offset                                                           0x00000014 /* type = RW, reset_val = 0x20 */
#define cpu_system__intr_flags__offset                                                             0x00000018 /* type = RD */
#define cpu_system__intr_clr_reg__offset                                                           0x0000001c /* type = RW */
#define cpu_system__intr_set_reg__offset                                                           0x00000020 /* type = RW */
#define cpu_system__dma_status__offset                                                             0x00000024 /* type = RD */
#define cpu_system__debug_0__offset                                                                0x00000028 /* type = RW, reset_val = 0 */
#define cpu_system__debug_1__offset                                                                0x0000002c /* type = RW, reset_val = 0 */
#define cpu_system__dma_cmd__offset                                                                0x00000040 /* type = MAP, reset_val = 0 */
#define cpu_system__dma_sys_status__offset                                                         0x00000048 /* type = RD */
#define cpu_system__dma_sys_cmd__offset                                                            0x00000050 /* type = MAP, reset_val = 0 */
#define cpu_system__cpucore_top_map__offset                                                        0x00000800 /* type = MAP, reset_val = 0 */
#define cpu_system__stdout_fifo__offset                                                            0x00004000 /* type = MAP */

#define cpu_system__stdout_fifo_cdepth                   (cpu_system__base + cpu_system__stdout_fifo_cdepth__offset)
#define cpu_system__cpu_pc_cnt                           (cpu_system__base + cpu_system__cpu_pc_cnt__offset)
#define cpu_system__cpu_error_codes                      (cpu_system__base + cpu_system__cpu_error_codes__offset)
#define cpu_system__cpu_srst                             (cpu_system__base + cpu_system__cpu_srst__offset)
#define cpu_system__intr_vec_reg                         (cpu_system__base + cpu_system__intr_vec_reg__offset)
#define cpu_system__intr_flags                           (cpu_system__base + cpu_system__intr_flags__offset)
#define cpu_system__intr_clr_reg                         (cpu_system__base + cpu_system__intr_clr_reg__offset)
#define cpu_system__intr_set_reg                         (cpu_system__base + cpu_system__intr_set_reg__offset)
#define cpu_system__dma_status                           (cpu_system__base + cpu_system__dma_status__offset)
#define cpu_system__debug_0                              (cpu_system__base + cpu_system__debug_0__offset)
#define cpu_system__debug_1                              (cpu_system__base + cpu_system__debug_1__offset)
#define cpu_system__dma_cmd                              (cpu_system__base + cpu_system__dma_cmd__offset)
#define cpu_system__dma_sys_status                       (cpu_system__base + cpu_system__dma_sys_status__offset)
#define cpu_system__dma_sys_cmd                          (cpu_system__base + cpu_system__dma_sys_cmd__offset)
#define cpu_system__cpucore_top_map                      (cpu_system__base + cpu_system__cpucore_top_map__offset)
#define cpu_system__stdout_fifo                          (cpu_system__base + cpu_system__stdout_fifo__offset)


#define dig_core__ram_sharing_control                                                      0x0017b01c /* type = RW */


#ifndef stream_proc__base
#define stream_proc__base                                                                          0x00b00000
#endif

/* register address offset : ${BLOCKNAME}__${REGISTERNAME}__offset */
#define stream_proc__stdout_fifo_cdepth__offset                                                    0x00000004 /* type = RD, reset_val = 0 */
#define stream_proc__stdout_fifo__offset                                                           0x00004000 /* type = MAP */

#define stream_proc__stdout_fifo_cdepth  (stream_proc__base + stream_proc__stdout_fifo_cdepth__offset)
#define stream_proc__stdout_fifo (stream_proc__stdout_fifo__offset + stream_proc__base)


#endif

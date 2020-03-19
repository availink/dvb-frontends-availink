// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Availink AVL6261 DVB-S/S2/S2X demodulator public APIs
 * 
 * Copyright (C) 2020 Availink, Inc. (gpl@availink.com)
 *
 */

#ifndef _AVL62X1_API_H_
#define _AVL62X1_API_H_

#include "avl62x1_lib.h"


uint16_t avl62x1_init_chip_object(struct avl62x1_chip *chip);
uint16_t avl62x1_get_chip_id(uint16_t slave_addr, uint32_t *chip_id);
uint16_t avl62x1_get_version(struct avl62x1_ver_info *version,
			     struct avl62x1_chip *chip);
uint16_t avl62x1_initialize(struct avl62x1_chip *chip);
uint16_t avl62x1_lock_tp(struct avl62x1_carrier_info *carrier_info,
			 struct avl62x1_stream_info *stream_info,
			 avl_bool_t blind_sym_rate,
			 struct avl62x1_chip *chip);
uint16_t avl62x1_get_lock_status(enum avl62x1_lock_status *lock_status,
				 struct avl62x1_chip *chip);
uint16_t avl62x1_get_lost_lock_status(
    enum avl62x1_lost_lock_status *lost_lock_status,
    struct avl62x1_chip *chip);
uint16_t avl62x1_get_snr(int16_t *snr_x100db,
			 struct avl62x1_chip *chip);
uint16_t avl62x1_get_signal_info(struct avl62x1_carrier_info *carrier_info,
				 struct avl62x1_chip *chip);
uint16_t avl62x1_get_signal_strength(uint16_t *signal_strength,
				     struct avl62x1_chip *chip);
uint16_t avl62x1_get_signal_quality(uint16_t *signal_quality,
				    struct avl62x1_chip *chip);
uint16_t avl62x1_config_error_stats(struct avl62x1_error_stats_config *config,
				    struct avl62x1_chip *chip);
uint16_t avl62x1_reset_ber(struct avl62x1_ber_config *config,
			   struct avl62x1_chip *chip);
uint16_t avl62x1_get_ber(uint32_t *ber_x1e9,
			 struct avl62x1_chip *chip);
uint16_t avl62x1_reset_per(struct avl62x1_chip *chip);
uint16_t avl62x1_get_per(uint32_t *per_x1e9,
			 struct avl62x1_chip *chip);
uint16_t avl62x1_discover_streams(struct avl62x1_carrier_info *carrier_info,
				  avl_bool_t blind_sym_rate,
				  struct avl62x1_chip *chip);
uint16_t avl62x1_get_discovery_status(
    enum avl62x1_discovery_status *status,
    struct avl62x1_chip *chip);
uint16_t avl62x1_get_num_streams(uint8_t *num_streams,
				 struct avl62x1_chip *chip);
uint16_t avl62x1_get_stream_list(struct avl62x1_stream_info *streams,
				 const uint8_t max_num_streams,
				 struct avl62x1_chip *chip);
uint16_t avl62x1_switch_stream(struct avl62x1_stream_info *stream_info,
			       struct avl62x1_chip *chip);
uint16_t avl62x1_get_stream_info(struct avl62x1_stream_info *stream_info,
				 struct avl62x1_chip *chip);

uint16_t avl62x1_init_diseqc(struct avl62x1_diseqc_params *params,
			     struct avl62x1_chip *chip);
uint16_t avl62x1_receive_diseqc_data(uint8_t *buf,
				     uint8_t *size,
				     struct avl62x1_chip *chip);
uint16_t avl62x1_send_diseqc_data(const uint8_t *buf,
				  uint8_t size,
				  struct avl62x1_chip *chip);
uint16_t avl62x1_get_diseqc_tx_status(struct avl62x1_diseqc_tx_status *status,
				      struct avl62x1_chip *chip);
uint16_t avl62x1_get_diseqc_rx_status(struct avl62x1_diseqc_rx_status *status,
				      struct avl62x1_chip *chip);
uint16_t avl62x1_send_diseqc_tone(uint8_t tone,
				  uint8_t count,
				  struct avl62x1_chip *chip);
uint16_t avl62x1_diseqc_tone_on(struct avl62x1_chip *chip);
uint16_t avl62x1_diseqc_tone_off(struct avl62x1_chip *chip);

uint16_t avl62x1_enable_tuner_i2c(struct avl62x1_chip *chip);
uint16_t avl62x1_disable_tuner_i2c(struct avl62x1_chip *chip);

uint16_t avl62x1_set_gpio_dir(enum avl62x1_gpio_pin pin,
			      enum avl62x1_gpio_pin_dir dir,
			      struct avl62x1_chip *chip);
uint16_t avl62x1_set_gpio_value(enum avl62x1_gpio_pin pin,
				enum avl62x1_gpio_pin_value val,
				struct avl62x1_chip *chip);
uint16_t avl62x1_get_gpio_value(enum avl62x1_gpio_pin pin,
				enum avl62x1_gpio_pin_value *val,
				struct avl62x1_chip *chip);

uint16_t avl62x1_blindscan_start(struct avl62x1_blind_scan_params *params,
				 struct avl62x1_chip *chip);
uint16_t avl62x1_blindscan_cancel(struct avl62x1_chip *chip);
uint16_t avl62x1_blindscan_get_status(struct avl62x1_blind_scan_info *info,
				      struct avl62x1_chip *chip);
uint16_t avl62x1_blindscan_get_carrier_list(
    const struct avl62x1_blind_scan_params *params,
    struct avl62x1_blind_scan_info *info,
    struct avl62x1_carrier_info *carriers,
    struct avl62x1_chip *chip);
uint16_t avl62x1_blindscan_confirm_carrier(
    const struct avl62x1_blind_scan_params *params,
    struct avl62x1_carrier_info *carrier,
    struct avl62x1_chip *chip);
uint16_t avl62x1_blindscan_get_stream_list(
    struct avl62x1_carrier_info *carrier,
    struct avl62x1_stream_info *streams,
    const uint8_t max_streams,
    struct avl62x1_chip *chip);

uint16_t avl62x1_get_stream_type(enum avl62x1_dvb_stream_type *stream_type,
				 struct avl62x1_chip *chip);

/*
 * TODO: integrate tuner adjustement part into tuner driver,
 * and leave only demod adjustment part here
 */
uint16_t avl62x1_optimize_carrier(struct avl_tuner *tuner,
				  struct avl62x1_carrier_info *carrier,
				  struct avl62x1_chip *chip);

/*
 * TODO: refactor T2MI interface -  controlling T2MI PID
 * autodiscover/semi-autodiscover and getting PLP ID list
 */

uint16_t avl62x1_manual_set_t2mi_pid(uint16_t pid,
				     struct avl62x1_chip *chip);
uint16_t avl62x1_manual_set_t2mi_pid_1(uint16_t pid,
				       struct avl62x1_chip *chip);
uint16_t avl62x1_get_current_stream_t2mi_pid(uint16_t *pid,
					     struct avl62x1_chip *chip);
uint16_t avl62x1_set_current_stream_t2mi_pid(uint16_t pid,
					     struct avl62x1_chip *chip);
uint16_t avl62x1_set_t2mi_plp_id_scan_frames(uint16_t frame_num,
					     struct avl62x1_chip *chip);
uint16_t avl62x1_get_t2mi_plp_list(struct avl62x1_t2mi_plp_list *list,
				   struct avl62x1_chip *chip);

#endif

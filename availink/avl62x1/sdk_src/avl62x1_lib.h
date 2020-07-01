// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Availink AVL6261 DVB-S/S2/S2X demodulator library routines
 * 
 * Copyright (C) 2020 Availink, Inc. (gpl@availink.com)
 *
 */

#ifndef _AVL62X1_LIB_H_
#define _AVL62X1_LIB_H_

#include "avl62x1_reg.h"
#include "avl_tuner.h"
#include "avl_lib.h"

//MAJOR.minor.build
//MAJOR = public API rev
//minor = FW-driver interface rev
//build number = increment on every change to implementation
#define AVL62X1_VER_MAJOR	3
#define AVL62X1_VER_MINOR	8
#define AVL62X1_VER_BUILD	5

#define AVL62X1_CHIP_ID			0x62615ca8

#define AVL62X1_PL_SCRAM_AUTO		(0x800000)
#define AVL62X1_PL_SCRAM_XSTATE		(0x040000)
#define AVL62X1_PL_SCRAM_XSTATE_MASK	(0x03FFFF)

#define AVL62X1_IF_SHIFT_MAX_SR_HZ	(15 * 1000 * 1000)

#define AVL62X1_PLP_LIST_SIZE		16

#define AVL62X1_DISEQC_DELAY		20

/* command enumeration definitions */
#define CMD_IDLE 0
#define CMD_LD_DEFAULT 1
#define CMD_ACQUIRE 2
#define CMD_HALT 3
#define CMD_DEBUG 4
#define CMD_SLEEP 7
#define CMD_WAKE 8
#define CMD_BLIND_SCAN 9
#define CMD_ROM_CRC 10
#define CMD_DEBUG_FW 15
#define CMD_ADC_TEST 20
#define CMD_DMA 21
#define CMD_CALC_CRC 22
#define CMD_PING 23
#define CMD_DECOMPRESS 24
#define CMD_CAPTURE_IQ 25
#define CMD_FAILED 255

#define SP_CMD_IDLE 0
#define SP_CMD_LD_DEFAULT 1
#define SP_CMD_ACQUIRE 2
#define SP_CMD_HALT 3
#define SP_CMD_BLIND_SCAN_CLR 4

/*
* Patch file stuff
*/
#define PATCH_CMD_VALIDATE_CRC 0
#define PATCH_CMD_PING 1
#define PATCH_CMD_LD_TO_DEVICE 2
#define PATCH_CMD_DMA 3
#define PATCH_CMD_EXTRACT 4
#define PATCH_CMD_ASSERT_CPU_RESET 5
#define PATCH_CMD_RELEASE_CPU_RESET 6
#define PATCH_CMD_LD_TO_DEVICE_IMM 7
#define PATCH_CMD_RD_FROM_DEVICE 8
#define PATCH_CMD_DMA_HW 9
#define PATCH_CMD_SET_COND_IMM 10
#define PATCH_CMD_EXIT 11
#define PATCH_CMD_POLL_WAIT 12
#define PATCH_CMD_LD_TO_DEVICE_PACKED 13

#define PATCH_CMP_TYPE_ZLIB 0
#define PATCH_CMP_TYPE_ZLIB_NULL 1
#define PATCH_CMP_TYPE_GLIB 2
#define PATCH_CMP_TYPE_NONE 3

#define PATCH_VAR_ARRAY_SIZE 32

//Addr modes 2 bits
#define PATCH_OP_ADDR_MODE_VAR_IDX 0
#define PATCH_OP_ADDR_MODE_IMMIDIATE 1

//Unary operators 6 bits
#define PATCH_OP_UNARY_NOP 0
#define PATCH_OP_UNARY_LOGICAL_NEGATE 1
#define PATCH_OP_UNARY_BITWISE_NEGATE 2
#define PATCH_OP_UNARY_BITWISE_AND 3
#define PATCH_OP_UNARY_BITWISE_OR 4

//Binary operators 1 Byte
#define PATCH_OP_BINARY_LOAD 0
#define PATCH_OP_BINARY_AND 1
#define PATCH_OP_BINARY_OR 2
#define PATCH_OP_BINARY_BITWISE_AND 3
#define PATCH_OP_BINARY_BITWISE_OR 4
#define PATCH_OP_BINARY_EQUALS 5
#define PATCH_OP_BINARY_STORE 6
#define PATCH_OP_BINARY_NOT_EQUALS 7

#define PATCH_COND_EXIT_AFTER_LD 8

#define PATCH_STD_DVBC 0
#define PATCH_STD_DVBSx 1
#define PATCH_STD_DVBTx 2
#define PATCH_STD_ISDBT 3

typedef enum avl62x1_slave_addr
{
	avl62x1_sa_0 = 0x14,
	avl62x1_sa_1 = 0x15
} avl62x1_slave_addr;

typedef enum avl62x1_xtal
{
	avl62x1_refclk_16mhz,
	avl62x1_refclk_27mhz,
	avl62x1_refclk_30mhz,
	avl62x1_refclk_25mhz
} avl62x1_xtal;

// Defines the device functional mode.
typedef enum avl62x1_functional_mode
{
	avl62x1_funcmode_idle = 0,
	avl62x1_funcmode_demod = 1,
	avl62x1_funcmode_blindscan = 2
} avl62x1_functional_mode;

typedef enum avl62x1_spectrum_polarity
{
	avl62x1_specpol_normal = 0,
	avl62x1_specpol_inverted = 1
} avl62x1_spectrum_polarity;

typedef enum avl62x1_lock_status
{
	avl62x1_status_unlocked = 0,
	avl62x1_status_locked = 1
} avl62x1_lock_status;

typedef enum avl62x1_lost_lock_status
{
	avl62x1_lost_lock_no = 0, //demod has not lost lock since last check
	avl62x1_lost_lock_yes = 1 //demod has lost lock since last check
} avl62x1_lost_lock_status;

// Defines stream discovery status
typedef enum avl62x1_discovery_status
{
	avl62x1_discovery_running = 0,
	avl62x1_discovery_finished = 1
} avl62x1_discovery_status;

// Defines the ON/OFF options
typedef enum avl62x1_switch
{
	avl62x1_on = 0,
	avl62x1_off = 1
} avl62x1_switch;

typedef enum avl62x1_standard
{
	avl62x1_dvbs = 0,
	avl62x1_dvbs2 = 1,
	avl62x1_qam_carrier = 2,
	avl62x1_edge_pair = 3
} avl62x1_standard;

typedef enum avl62x1_modulation_mode
{
	avl62x1_bpsk = 1,
	avl62x1_qpsk = 2,
	avl62x1_8psk = 3,
	avl62x1_16apsk = 4,
	avl62x1_32apsk = 5,
	avl62x1_64apsk = 6,
	avl62x1_128apsk = 7,
	avl62x1_256apsk = 8
} avl62x1_modulation_mode;

typedef enum avl62x1_dvbs_code_rate
{
	avl62x1_dvbs_cr_1_2 = 0,
	avl62x1_dvbs_cr_2_3 = 1,
	avl62x1_dvbs_cr_3_4 = 2,
	avl62x1_dvbs_cr_5_6 = 3,
	avl62x1_dvbs_cr_7_8 = 4
} avl62x1_dvbs_code_rate;

typedef enum avl62x1_dvbs2_code_rate
{
	avl62x1_dvbs2_cr_1_4 = 0,
	avl62x1_dvbs2_cr_1_3 = 1,
	avl62x1_dvbs2_cr_2_5 = 2,
	avl62x1_dvbs2_cr_1_2 = 3,
	avl62x1_dvbs2_cr_3_5 = 4,
	avl62x1_dvbs2_cr_2_3 = 5,
	avl62x1_dvbs2_cr_3_4 = 6,
	avl62x1_dvbs2_cr_4_5 = 7,
	avl62x1_dvbs2_cr_5_6 = 8,
	avl62x1_dvbs2_cr_8_9 = 9,
	avl62x1_dvbs2_cr_9_10 = 10,
	avl62x1_dvbs2_cr_2_9 = 11,
	avl62x1_dvbs2_cr_13_45 = 12,
	avl62x1_dvbs2_cr_9_20 = 13,
	avl62x1_dvbs2_cr_90_180 = 14,
	avl62x1_dvbs2_cr_96_180 = 15,
	avl62x1_dvbs2_cr_11_20 = 16,
	avl62x1_dvbs2_cr_100_180 = 17,
	avl62x1_dvbs2_cr_104_180 = 18,
	avl62x1_dvbs2_cr_26_45 = 19,
	avl62x1_dvbs2_cr_18_30 = 20,
	avl62x1_dvbs2_cr_28_45 = 21,
	avl62x1_dvbs2_cr_23_36 = 22,
	avl62x1_dvbs2_cr_116_180 = 23,
	avl62x1_dvbs2_cr_20_30 = 24,
	avl62x1_dvbs2_cr_124_180 = 25,
	avl62x1_dvbs2_cr_25_36 = 26,
	avl62x1_dvbs2_cr_128_180 = 27,
	avl62x1_dvbs2_cr_13_18 = 28,
	avl62x1_dvbs2_cr_132_180 = 29,
	avl62x1_dvbs2_cr_22_30 = 30,
	avl62x1_dvbs2_cr_135_180 = 31,
	avl62x1_dvbs2_cr_140_180 = 32,
	avl62x1_dvbs2_cr_7_9 = 33,
	avl62x1_dvbs2_cr_154_180 = 34,
	avl62x1_dvbs2_cr_11_45 = 35,
	avl62x1_dvbs2_cr_4_15 = 36,
	avl62x1_dvbs2_cr_14_45 = 37,
	avl62x1_dvbs2_cr_7_15 = 38,
	avl62x1_dvbs2_cr_8_15 = 39,
	avl62x1_dvbs2_cr_32_45 = 40
} avl62x1_dvbs2_code_rate;

typedef enum avl62x1_pilot
{
	avl62x1_pilot_off = 0,
	avl62x1_pilot_on = 1
} avl62x1_pilot;

typedef enum avl62x1_rolloff
{
	avl62x1_rolloff_35 = 0,
	avl62x1_rolloff_25 = 1,
	avl62x1_rolloff_20 = 2,
	avl62x1_rolloff_15 = 3,
	avl62x1_rolloff_10 = 4,
	avl62x1_rolloff_05 = 5,
	avl62x1_rolloff_unknown = 6
} avl62x1_rolloff;

typedef enum avl62x1_fec_length
{
	avl62x1_dvbs2_fec_short = 0,
	avl62x1_dvbs_fec_medium = 2,
	avl62x1_dvbs2_fec_long = 1
} avl62x1_fec_length;

typedef enum avl62x1_dvbs2_ccm_acm
{
	avl62x1_dvbs2_acm = 0,
	avl62x1_dvbs2_ccm = 1
} avl62x1_dvbs2_ccm_acm;

typedef enum avl62x1_dvb_stream_type
{
	avl62x1_generic_packetized = 0,
	avl62x1_generic_continuous = 1,
	avl62x1_gse_hem = 2,
	avl62x1_transport = 3,
	//stream type unknown/don't care. will output BB frames directly over TS
	avl62x1_unknown = 4,
	avl62x1_gse_lite = 5,
	avl62x1_gse_hem_lite = 6,
	avl62x1_t2mi = 7,
	//don't know stream type. demod will scan for streams then output first one found
	avl62x1_undetermined = 255
} avl62x1_dvb_stream_type;

//MPEG output format
typedef enum avl62x1_mpeg_format
{
	avl62x1_mpf_ts = 0, //Transport stream format.
	avl62x1_mpf_tsp = 1 //Transport stream plus parity format.
} avl62x1_mpeg_format;

//MPEG output mode
typedef enum avl62x1_mpeg_mode
{
	avl62x1_mpm_parallel = 0,   //parallel mode
	avl62x1_mpm_serial = 1,     //serial mode
	avl62x1_mpm_2bit_serial = 2 //2bit serial mode
} avl62x1_mpeg_mode;

//MPEG output clock polarity
typedef enum avl62x1_mpeg_clock_polarity
{
	avl62x1_mpcp_falling = 0, //data valid on falling edge of clk
	avl62x1_mpcp_rising = 1   //...on rising edge
} avl62x1_mpeg_clock_polarity;

/*
	* The phase of the MPEG clock edge relative to the data transition.
	* Applies to parallel mode only. 0,1,2,3 will delay the MPEG clock
	* edge by 0,1,2, or 3 mpeg_clk_freq_hz clock periods
	*/
typedef enum avl62x1_mpeg_clock_phase
{
	avl62x1_mpcp_phase_0 = 0, //no clock edge delay
	avl62x1_mpcp_phase_1 = 1, //delay clock edge by 1
	avl62x1_mpcp_phase_2 = 2, //delay clock edge by 2
	avl62x1_mpcp_phase_3 = 3  //delay clock edge by 3
} avl62x1_mpeg_clock_phase;

typedef enum avl62x1_mpeg_clock_adaptation
{
	avl62x1_mpca_fixed = 0,   //no adaptation. fixed frequency (mpeg_clk_freq_hz)
	avl62x1_mpca_adaptive = 1 //adapt clock frequency to data rate
} avl62x1_mpeg_clock_adaptation;

//MPEG bit order for serial mode
typedef enum avl62x1_mpeg_bit_order
{
	avl62x1_mpbo_lsb = 0, //least significant bit first
	avl62x1_mpbo_msb = 1  //most significant bit first
} avl62x1_mpeg_bit_order;

//Serial mode data pin
typedef enum avl62x1_mpeg_serial_pin
{
	avl62x1_mpsp_data_0 = 0, //pin MPEG_DATA_0
	avl62x1_mpsp_data_7 = 1  //pin MPEG_DATA_7
} avl62x1_mpeg_serial_pin;

/*
	* MPEG error signal polarity.  Error signal is asserted during the
	* payload of a packet which contains uncorrected errors
	*/
typedef enum avl62x1_mpeg_err_polarity
{
	avl62x1_mpep_normal = 0, //asserted high
	avl62x1_mpep_invert = 1  //asserted low
} avl62x1_mpeg_err_polarity;

/*
	* Defines whether the feeback bit of the LFSR used to generate
	* the BER/PER test pattern is inverted.
	*/
typedef enum avl62x1_lfsr_fb_bit
{
	avl62x1_lfsr_fb_noninverted = 0,
	avl62x1_lfsr_fb_inverted = 1
} avl62x1_lfsr_fb_bit;

//Defines the test pattern being used for BER/PER measurements.
typedef enum avl62x1_test_pattern
{
	avl62x1_test_lfsr_15 = 0, // BER test pattern is LFSR15
	avl62x1_test_lfsr_23 = 1  // BER test pattern is LFSR23
} avl62x1_test_pattern;

/*
	* Auto error stats mode. Error stats are automatically reset either
	* according to a time threshold or received bytes threshold
	*/
typedef enum avl62x1_auto_error_stats_type
{
	avl62x1_error_stats_bytes = 0,
	avl62x1_error_stats_time = 1
} avl62x1_auto_error_stats_type;

//Error statistics mode
typedef enum avl62x1_error_stats_mode
{
	avl62x1_error_stats_manual = 0,
	avl62x1_error_stats_auto = 1
} avl62x1_error_stats_mode;

//DiSEqC status
typedef enum avl62x1_diseqc_status
{
	avl62x1_dos_uninit = 0,     //not initialized yet
	avl62x1_dos_init = 1,       //initialized
	avl62x1_dos_continuous = 2, //in continuous mode
	avl62x1_dos_tone = 3,       //in tone burst mode
	avl62x1_dos_modulation = 4  //in modulation mode
} avl62x1_diseqc_status;

typedef enum avl62x1_diseqc_tx_gap
{
	avl62x1_dtxg_15ms = 0, // The gap is 15 ms.
	avl62x1_dtxg_20ms = 1, // The gap is 20 ms.
	avl62x1_dtxg_25ms = 2, // The gap is 25 ms.
	avl62x1_dtxg_30ms = 3  // The gap is 30 ms.
} avl62x1_diseqc_tx_gap;

typedef enum avl62x1_diseqc_tx_mode
{
	avl62x1_dtm_modulation = 0, // Use modulation mode.
	avl62x1_dtm_tone_0 = 1,     // Send out tone 0.
	avl62x1_dtm_tone_1 = 2,     // Send out tone 1.
	avl62x1_dtm_continuous = 3  // Continuously send out pulses.
} avl62x1_diseqc_tx_mode;

/*
	* RX Time. Receive data for X ms.
	*/
typedef enum avl62x1_diseqc_rx_time
{
	avl62x1_drt_150ms = 0,
	avl62x1_drt_170ms = 1,
	avl62x1_drt_190ms = 2,
	avl62x1_drt_210ms = 3
} avl62x1_diseqc_rx_time;

typedef enum avl62x1_diseqc_waveform_mode
{
	avl62x1_dwm_normal = 0,  // Normal waveform mode
	avl62x1_dwm_envelope = 1 // Envelope waveform mode
} avl62x1_diseqc_waveform_mode;

//GPIO pins by number and name
typedef enum avl62x1_gpio_pin
{
	avl62x1_gpio_pin_10 = 0,
	avl62x1_gpio_pin_tuner_sda = 0,
	avl62x1_gpio_pin_11 = 1,
	avl62x1_gpio_pin_tuner_scl = 1,
	avl62x1_gpio_pin_12 = 2,
	avl62x1_gpio_pin_s_agc2 = 2,
	avl62x1_gpio_pin_37 = 3,
	avl62x1_gpio_pin_lnb_pwr_en = 3,
	avl62x1_gpio_pin_38 = 4,
	avl62x1_gpio_pin_lnb_pwr_sel = 4
} avl62x1_gpio_pin;

typedef enum avl62x1_gpio_pin_dir
{
	avl62x1_gpio_dir_output = 0,
	avl62x1_gpio_dir_input = 1
} avl62x1_gpio_pin_dir;

typedef enum avl62x1_gpio_pin_value
{
	avl62x1_gpio_value_logic_0 = 0,
	avl62x1_gpio_value_logic_1 = 1,
	avl62x1_gpio_value_high_z = 2
} avl62x1_gpio_pin_value;

typedef enum avl62x1_sis_mis
{
	avl62x1_mis = 0,
	avl62x1_sis = 1,
	avl62x1_sis_mis_unknown = 2
} avl62x1_sis_mis;

/*
	* Carrier definition.  Output by blindscan; used by Lock()
	*/
struct avl62x1_carrier_info
{
	uint8_t carrier_idx;		//index of this carrier
	uint32_t rf_freq_khz;		//RF frequency of the carrier in kHz
	int32_t carrier_freq_offset_hz; //carrier frequency offset (from RF freq) in Hz

	/*
	* When locking with blind_sym_rate=false, this is the
	* nominal symbol rate. When locking with blind_sym_rate=true,
	* this is the max symbol rate to consider.
	*/
	uint32_t symbol_rate_hz;

	enum avl62x1_rolloff roll_off;
	enum avl62x1_standard signal_type;
	enum avl62x1_spectrum_polarity spectrum_invert;

	/*
	* Physical layer scrambling.
	* When used as an input, set to AVL62X1_PL_SCRAM_AUTO to enable
	* automatic scrambling detection, or set to
	* (AVL62X1_PL_SCRAM_XSTATE | (lfsr_state_xi & AVL62X1_PL_SCRAM_XSTATE_MASK))
	* where lfsr_state_xi is the initial state of the x(i) LFSR,
	* or to (lfsr_n_xi) where lfsr_n_xi is the sequence shift of
	* the x(i) sequence in the Gold code, defined as the
	* "code number n" in the DVB-S2 standard.
	* When used as an output, contains the initial state of the
	* x(i) LFSR.
	*/
	uint32_t pl_scrambling;

	uint8_t pls_acm; //PLS if CCM, 0 if ACM
	enum avl62x1_sis_mis sis_mis;
	uint8_t num_streams; //number of supported streams
	int16_t snr_db_x100;
	enum avl62x1_modulation_mode modulation;
	enum avl62x1_pilot pilot;
	enum avl62x1_fec_length fec_length;
	union {
		enum avl62x1_dvbs_code_rate dvbs_code_rate;
		enum avl62x1_dvbs2_code_rate dvbs2_code_rate;
	} code_rate;
	enum avl62x1_dvbs2_ccm_acm dvbs2_ccm_acm;
};

//list of PLP ID's
struct avl62x1_t2mi_plp_list
{
	uint8_t list_size;
	uint8_t list[AVL62X1_PLP_LIST_SIZE];
};

/*
* N.B. this structure and the API calls that use it
* are extremely likely to change in the future
*/
struct avl62x1_t2mi_info
{
	//config: ID of PLP to output
	uint8_t plp_id;

	//uint16_t plp_id_scan_frames;

	//config: enable T2MI raw mode (outputs T2MI packets directly)
	uint8_t raw_mode;

	//config: enable automatic detection of PID carrying T2MI
	uint8_t pid_autodiscover;

	//config: when not autodiscovering, this is the first
	//  T2MI PID candidate.  Defaults to 0x1000
	//status: when autodiscovering, this is the detected T2MI PID
	uint16_t pid;

	//config: when not autodiscovering, this is the second
	//  T2MI PID candidate
	uint16_t pid_1;

	//status: list of PLP's found
	struct avl62x1_t2mi_plp_list plp_list;
};

struct avl62x1_stream_info
{
	/*
	* //index of carrier (avl62x1_carrier_info.carrier_idx)
	* that this stream is in
	*/
	uint8_t carrier_idx;

	enum avl62x1_dvb_stream_type stream_type;
	uint8_t isi;
	struct avl62x1_t2mi_info t2mi;
};

struct avl62x1_error_stats
{
	/*
	* Indicates whether the receiver is synchronized
	* with the transmitter generating the BER test pattern.
	*/
	uint16_t lfsr_sync;

	/*
	* Indicates whether the receiver has lost lock since the
	* BER/PER measurement was started.
	*/
	uint16_t lost_lock;

	/*
	* Internal software counters which store the number
	* of bits/bit errors which have been received.
	*/
	struct avl_uint64 sw_cnt_num_bits;
	struct avl_uint64 sw_cnt_bit_errs;

	struct avl_uint64 num_bits; //total bits received
	struct avl_uint64 bit_errs; //total detected bit errors

	/*
	* Internal software counters which store the number
	* of pkts/pkt errors which have been received.
	*/
	struct avl_uint64 sw_cnt_num_pkts;
	struct avl_uint64 sw_cnt_pkt_errs;

	struct avl_uint64 num_pkts; //total received packets
	struct avl_uint64 pkt_errs; //total detected packet errors
	uint32_t ber_x1e9;	  //bit error rate scaled by 1e9
	uint32_t per_x1e9;	  //packet error rate scaled by 1e9
};

//Error statistics configuration
struct avl62x1_error_stats_config
{
	enum avl62x1_error_stats_mode error_stats_mode;
	enum avl62x1_auto_error_stats_type auto_error_stats_type;
	uint32_t time_threshold_ms;
	uint32_t bytes_threshold;
};

//BER measurement configuration
struct avl62x1_ber_config
{
	enum avl62x1_test_pattern test_pattern; // indicates the pattern of LFSR.
	enum avl62x1_lfsr_fb_bit fb_inversion;  // indicates LFSR feedback bit inversion.
	uint32_t lfsr_sync;			// indicates the LFSR synchronization status.
	uint32_t lfsr_start_pos;		//set LFSR start byte positon
};

//DiSEqC operation parameters
struct avl62x1_diseqc_params
{
	uint16_t tone_freq_khz; //normally 22kHz
	enum avl62x1_diseqc_tx_gap tx_gap;
	enum avl62x1_diseqc_waveform_mode tx_waveform;
	enum avl62x1_diseqc_rx_time rx_timeout;
	enum avl62x1_diseqc_waveform_mode rx_waveform;
};

//DiSEqC transmit status
struct avl62x1_diseqc_tx_status
{
	uint8_t tx_complete;
	uint8_t tx_fifo_count; //number of bytes in the transmit FIFO
};

//DiSEqC receive status
struct avl62x1_diseqc_rx_status
{
	uint8_t rx_fifo_count; //number of bytes in the receive FIFO
	uint8_t rx_complete;
};

struct avl_ver_info
{
	uint8_t major;  // The major version number.
	uint8_t minor;  // The minor version number.
	uint16_t build; // The build version number.
};

struct avl62x1_ver_info
{
	uint32_t hw_version; //silicon version
	struct avl_ver_info firmware;
	struct avl_ver_info driver;
};

struct avl62x1_chip_priv
{
	uint8_t *patch_data;
	avl_sem_t rx_cmd_sem; //protects the receiver command channel
	avl_sem_t diseqc_sem; //protects DiSEqC operation
	enum avl62x1_diseqc_status diseqc_op_status;

	uint32_t mpeg_clk_freq_hz; //actual freq after init
	uint32_t core_clk_freq_hz;
	uint32_t fec_clk_freq_hz;

	struct avl62x1_error_stats error_stats;

	int32_t carrier_freq_offset_hz;
	uint8_t agc_driven;
};

struct avl62x1_chip_pub
{
	/*
	* demod ID and I2C slave address
	* ((ID & AVL_DEMOD_ID_MASK)<<8) | (slv_addr & 0xFF)
	*/
	uint16_t i2c_addr;

	enum avl62x1_xtal ref_clk; //Crystal reference clock

	/*provides demod access to tuner characteristics*/
	struct avl_tuner *tuner;

	/*Tuner spectrum polarity (e.g. I/Q input swapped)*/
	enum avl62x1_spectrum_polarity tuner_pol;

	enum avl62x1_mpeg_mode			mpeg_mode;
	enum avl62x1_mpeg_clock_polarity	mpeg_clk_pol;
	enum avl62x1_mpeg_err_polarity		mpeg_err_pol;
	enum avl62x1_mpeg_err_polarity		mpeg_valid_pol;
	enum avl62x1_mpeg_clock_phase		mpeg_clk_phase;
	enum avl62x1_mpeg_clock_adaptation	mpeg_clk_adapt;
	enum avl62x1_mpeg_format		mpeg_format;
	enum avl62x1_mpeg_serial_pin		mpeg_serial_pin;
	enum avl62x1_mpeg_bit_order		mpeg_bit_order;

	/*
	* Requested MPEG clk freq for non-adaptive mode.
	* After initialization, avl62x1_chip_priv.mpeg_clk_freq_hz
	* contains actual operational frequency.
	*/
	uint32_t req_mpeg_clk_freq_hz;
};

struct avl62x1_chip
{
	struct avl62x1_chip_priv *chip_priv;
	struct avl62x1_chip_pub *chip_pub;
};

//blind scan (single tuner step) configuration
struct avl62x1_blind_scan_params
{
	uint16_t tuner_center_freq_100khz;
	uint16_t tuner_lpf_100khz;
	uint16_t min_symrate_khz;
};

//blind scan (single tuner step) info/status
struct avl62x1_blind_scan_info
{
	uint8_t progress;     //approx progress in percent
	uint8_t finished;     //nonzero when BS completed
	uint8_t num_carriers; //number of confirmed S or S2/X carriers

	/* 
		* DEPRECATED: number of confirmed DVB output streams
		*/
	//uint8_t num_streams;

	/*
		* Amount to move tuner (relative to its current position)
		* for next BS step.
		*/
	uint32_t next_freq_step_hz;
};

uint16_t __avl62x1_check_chip_ready(struct avl62x1_chip *chip);

/*
 * download and boot firmware, load default configuration,
 * read back clock frequencies
 */
uint16_t __avl62x1_initialize(struct avl62x1_chip *chip);

uint16_t __avl62x1_get_func_mode(
    enum avl62x1_functional_mode *mode,
    struct avl62x1_chip *chip);

uint16_t __avl62x1_get_cmd_status(struct avl62x1_chip *chip);

uint16_t __avl62x1_send_cmd(uint8_t cmd, struct avl62x1_chip *chip);

uint16_t __avl62x1_get_sp_cmd_status(struct avl62x1_chip *chip);

uint16_t __avl62x1_send_sp_cmd(uint8_t cmd, struct avl62x1_chip *chip);

uint16_t __avl62x1_halt(struct avl62x1_chip *chip);
uint16_t __avl62x1_sleep(struct avl62x1_chip *chip);
uint16_t __avl62x1_wakeup(struct avl62x1_chip *chip);
uint16_t __avl62x1_init_tuner_i2c(struct avl62x1_chip *chip);

uint16_t __avl62x1_init_adc(struct avl62x1_chip *chip);
uint16_t __avl62x1_get_tuner_polarity(
    enum avl62x1_spectrum_polarity *polarity,
    struct avl62x1_chip *chip);
uint16_t __avl62x1_set_tuner_polarity(
    enum avl62x1_spectrum_polarity polarity,
    struct avl62x1_chip *chip);
uint16_t __avl62x1_drive_agc(
    enum avl62x1_switch state,
    struct avl62x1_chip *chip);
uint16_t __avl62x1_get_cfo(int32_t *cfo, struct avl62x1_chip *chip);
uint16_t __avl62x1_get_sro(int32_t *sro_ppm, struct avl62x1_chip *chip);
uint16_t __avl62x1_get_acq_retries(uint8_t *retries, struct avl62x1_chip *chip);

uint16_t __avl62x1_set_mpeg_mode(struct avl62x1_chip *chip);

uint16_t __avl62x1_drive_mpeg_output(
    enum avl62x1_switch state,
    struct avl62x1_chip *chip);

uint16_t __avl62x1_diseqc_mode_switch_check(struct avl62x1_chip *chip);

uint16_t __avl62x1_exec_patchscript(struct avl62x1_chip *chip);
uint8_t __avl62x1_patch_read8(uint8_t *buf, uint32_t *idx);
uint16_t __avl62x1_patch_read16(uint8_t *buf, uint32_t *idx);
uint32_t __avl62x1_patch_read32(uint8_t *buf, uint32_t *idx);

uint16_t __avl62x1_conv_xlfsr_state_to_n(uint32_t state, uint32_t *n);
uint16_t __avl62x1_conv_n_to_xlfsr_state(uint32_t n, uint32_t *state);

#endif

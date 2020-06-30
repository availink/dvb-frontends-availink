// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Availink AVL68x2 DVB-S/S2/T/T2/C, ISDB-T, J83.B demodulator driver
 *
 * Copyright (C) 2020 Availink, Inc. (gpl@availink.com)
 *
 */

#ifndef __avl68x2_internal_h__
#define __avl68x2_internal_h__

#include "avl_lib.h"

//MAJOR.minor.build
//MAJOR = public API rev
//minor = FW-driver interface rev
//build number = increment on every change to implementation
#define AVL68X2_VER_MAJOR	2
#define AVL68X2_VER_MINOR	19
#define AVL68X2_VER_BUILD	4

#define AVL68XX			0x68624955

#define MAX_II2C_READ_SIZE	64
#define MAX_II2C_WRITE_SIZE	64

#define AVL_FW_CMD_IDLE                          0
#define AVL_FW_CMD_LD_DEFAULT                    1
#define AVL_FW_CMD_ACQUIRE                       2
#define AVL_FW_CMD_HALT                          3 
#define AVL_FW_CMD_DEBUG                         4
#define AVL_FW_CMD_SLEEP                         7
#define AVL_FW_CMD_WAKE                          8
#define AVL_FW_CMD_BLIND_SCAN                    9
#define AVL_FW_CMD_SDRAM_TEST                   16
#define AVL_FW_CMD_INIT_SDRAM                   17
#define AVL_FW_CMD_INIT_ADC                     18
#define AVL_FW_CMD_CHANGE_MODE                  19

#define AVL_FW_CMD_DMA                          21
#define AVL_FW_CMD_CALC_CRC                     22
#define AVL_FW_CMD_PING                         23
#define AVL_FW_CMD_DECOMPRESS                   24

    
/* SP FW command */
#define AVL_SP_CMD_IDLE                          0
#define AVL_SP_CMD_LD_DEFAULT                    1
#define AVL_SP_CMD_ACQUIRE                       2
#define AVL_SP_CMD_HALT                         3 
	
/*
 * Patch file stuff
 */
#define PATCH_VAR_ARRAY_SIZE                32

#define PATCH_CMD_VALIDATE_CRC              0
#define PATCH_CMD_PING                      1 
#define PATCH_CMD_LD_TO_DEVICE              2 
#define PATCH_CMD_DMA                       3 
#define PATCH_CMD_DECOMPRESS                4
#define PATCH_CMD_ASSERT_CPU_RESET          5 
#define PATCH_CMD_RELEASE_CPU_RESET         6
#define PATCH_CMD_LD_TO_DEVICE_IMM          7
#define PATCH_CMD_RD_FROM_DEVICE            8
#define PATCH_CMD_DMA_HW                    9
#define PATCH_CMD_SET_COND_IMM              10
#define PATCH_CMD_EXIT                      11


#define PATCH_CMP_TYPE_ZLIB                 0
#define PATCH_CMP_TYPE_ZLIB_NULL            1
#define PATCH_CMP_TYPE_GLIB                 2
#define PATCH_CMP_TYPE_NONE                 3

//Addr modes 2 bits
#define PATCH_OP_ADDR_MODE_VAR_IDX          0
#define PATCH_OP_ADDR_MODE_IMMIDIATE        1

//Unary operators 6 bits
#define PATCH_OP_UNARY_NOP                  0
#define PATCH_OP_UNARY_LOGICAL_NEGATE       1
#define PATCH_OP_UNARY_BITWISE_NEGATE       2
#define PATCH_OP_UNARY_BITWISE_AND          3
#define PATCH_OP_UNARY_BITWISE_OR           4

//Binary operators 1 Byte
#define PATCH_OP_BINARY_LOAD                0
#define PATCH_OP_BINARY_AND                 1
#define PATCH_OP_BINARY_OR                  2
#define PATCH_OP_BINARY_BITWISE_AND         3
#define PATCH_OP_BINARY_BITWISE_OR          4
#define PATCH_OP_BINARY_EQUALS              5
#define PATCH_OP_BINARY_STORE               6
#define PATCH_OP_BINARY_NOT_EQUALS          7

#define PATCH_COND_EXIT_AFTER_LD            8

#define PATCH_STD_DVBC                      0 
#define PATCH_STD_DVBSx                     1
#define PATCH_STD_DVBTx                     2
#define PATCH_STD_ISDBT                     3

#define tuner_i2c_srst_offset                0x0
#define tuner_i2c_cntrl_offset               0x4
#define tuner_i2c_bit_rpt_clk_div_offset     0x18
#define tuner_i2c_bit_rpt_cntrl_offset       0x1C

#define esm_cntrl_offset                    0x4
#define bit_error_offset                    0x8
#define byte_num_offset                     0xC
#define packet_error_offset                 0x10
#define packet_num_offset                   0x14
#define tick_clear_offset                   0x88
#define tick_type_offset                    0x8C
#define time_tick_low_offset                0x90
#define time_tick_high_offset               0x94
#define byte_tick_low_offset                0x98
#define byte_tick_high_offset               0x9C
#define esm_mode_offset                     0xC0

#define rs_current_active_mode_iaddr_offset 0x24
#define rc_fw_command_saddr_offset          0x00
#define rc_fw_last_command_saddr_offset     0x02
#define rs_core_ready_word_iaddr_offset     0xa0
#define rc_sdram_test_return_iaddr_offset   0x3C
#define rc_sdram_test_result_iaddr_offset   0x40
#define rs_rf_agc_saddr_offset              0x44

#define rc_fw_command_args_addr_iaddr_offset 0x58

#define rc_ts_cntns_clk_frac_d_iaddr_offset                 0x0000007c
#define rc_ts_cntns_clk_frac_n_iaddr_offset                 0x00000078
#define rc_enable_ts_continuous_caddr_offset                0x0000003a
#define rc_ts_clock_edge_caddr_offset                       0x0000003b
#define rc_ts_serial_caddr_offset                           0x0000003c
#define rc_ts_serial_outpin_caddr_offset                    0x0000003f
#define rc_ts_serial_msb_caddr_offset                       0x0000003e
#define rc_ts_ts0_tsp1_caddr_offset                         0x00000039
#define rc_ts_packet_order_caddr_offset                     rc_ts_serial_msb_caddr_offset
#define rc_ts_error_bit_en_caddr_offset                     0x00000038
#define rc_ts_error_polarity_caddr_offset                   0x00000041
#define rc_ts_valid_polarity_caddr_offset                   0x00000040
#define rc_ts_sync_pulse_caddr_offset                       0x00000097
#define ts_clock_phase_caddr_offset                         0x00000096

#define rs_patch_ver_iaddr_offset           0x00000004

//GPIO control
#define agc1_sel_offset                          0x00
#define agc2_sel_offset                          0x10
#define lnb_cntrl_1_sel_offset                   0x08 
#define lnb_cntrl_0_sel_offset                   0x0c 
#define lnb_cntrl_1_i_offset                     0x48
#define lnb_cntrl_0_i_offset                     0x4c

#define gpio_module_002_gpio_config_offset       0x00 
#define emerald_io_pad_CS_0_sel_offset           0x0c

#define hw_AVL_rx_rf_aagc_gain              0x160888



    //Define ADC channel selection
    typedef enum AVL_ADC_Channel
    {
        AVL_ADC_CHAN2   =   0,
        AVL_ADC_CHAN1   =   1,
        AVL_ADC_OFF     =   2
    }AVL_ADC_Channel;

    typedef enum AVL_ADC_Output_format
    {
        AVL_2COMP    =   0,
        AVL_OFFBIN   =   1
    }AVL_ADC_Output_format;

    //Input_select enumeration definitions
    typedef enum AVL_DDC_Input
    {
        AVL_DIG_IN       =   0,
        AVL_ADC_IN       =   1,
        AVL_VEC_IN       =   2,
        AVL_VEC1x_IN     =   3,
        AVL_DIG1x_IN     =   4
    }AVL_DDC_Input;

    // Defines BER type
    typedef enum AVL_BER_Type
    {
        AVL_PRE_VITERBI_BER     =   0,                      // previous viterbi BER will be acquired.
        AVL_POST_VITERBI_BER    =   1,                      // post viterbi BER will be acquired.
        AVL_PRE_LDPC_BER        =   2,                      // previous LDPC BER will be acquired.
        AVL_POST_LDPC_BER       =   3,                      // post LDPC BER will be acquired.
        AVL_FINAL_BER           =   4                       // final BER will be acquired.
    }AVL_BER_Type;

    // Defines different standards supported by the demod.
    typedef enum AVL_DemodMode
    {
        AVL_DVBC = 0,
        AVL_DVBSX = 1,
        AVL_DVBTX = 2,
        AVL_ISDBT = 3,
        AVL_DTMB = 4,
        AVL_ISDBS = 5,
        AVL_ABSS = 6,
        AVL_ATSC = 7,
        AVL_DVBC2 = 8
    } AVL_DemodMode;

    // Defines the channel lock mode.
    typedef enum AVL_LockMode
    {
        AVL_LOCK_MODE_AUTO      =   0,                      // lock channel automatically.
        AVL_LOCK_MODE_MANUAL    =   1                       // lock channel manually.
    }AVL_LockMode;

    // Defines channel lock status
    typedef enum AVL_LockStatus
    {
        AVL_STATUS_UNLOCK   =   0,                          // channel isn't locked
        AVL_STATUS_LOCK     =   1                           // channel is in locked state.
    }AVL_LockStatus;

    typedef enum AVL_TSMode
    {
        AVL_TS_PARALLEL = 0, 
        AVL_TS_SERIAL =   1
    }AVL_TSMode; 

    typedef enum AVL_TSClockEdge
    {
        AVL_MPCM_FALLING   =   0, 
        AVL_MPCM_RISING    =   1  
    } AVL_TSClockEdge; 

    typedef enum AVL_TSClockMode
    {
        AVL_TS_CONTINUOUS_ENABLE = 0,   
        AVL_TS_CONTINUOUS_DISABLE = 1      
    } AVL_TSClockMode; 

    typedef enum AVL_TSSerialPin
    {
        AVL_MPSP_DATA0  =   0, 
        AVL_MPSP_DATA7  =   1  
    } AVL_TSSerialPin; 

    typedef enum AVL_TSSerialOrder
    {
        AVL_MPBO_LSB = 0, 
        AVL_MPBO_MSB = 1 
    } AVL_TSSerialOrder; 

    typedef enum AVL_TSSerialSyncPulse
    {
        AVL_TS_SERIAL_SYNC_8_PULSE    =   0,         
        AVL_TS_SERIAL_SYNC_1_PULSE      =   1        
    } AVL_TSSerialSyncPulse; 

    typedef enum AVL_TSErrorBit
    {
        AVL_TS_ERROR_BIT_DISABLE  =   0,  
        AVL_TS_ERROR_BIT_ENABLE   =   1  
    } AVL_TSErrorBit; 

    typedef enum AVL_TSErrorPolarity
    {
        AVL_MPEP_Normal = 0,  
        AVL_MPEP_Invert = 1  
    } AVL_TSErrorPolarity; 

    typedef enum AVL_TSValidPolarity
    {
        AVL_MPVP_Normal     =   0, 
        AVL_MPVP_Invert     =   1   
    } AVL_TSValidPolarity; 

    typedef enum AVL_AGCPola
    {
        AVL_AGC_NORMAL  =   0,        //  normal AGC polarity. Used for a tuner whose gain increases with increased AGC voltage.
        AVL_AGC_INVERTED=   1         //  inverted AGC polarity. Used for tuner whose gain decreases with increased AGC voltage.
    }AVL_AGCPola;

    typedef enum AVL_TSParallelOrder
    {
        AVL_TS_PARALLEL_ORDER_INVERT =   0,
        AVL_TS_PARALLEL_ORDER_NORMAL =   1
    } AVL_TSParallelOrder; 

    typedef enum AVL_TSParallelPhase
    {
        AVL_TS_PARALLEL_PHASE_0 = 0,
        AVL_TS_PARALLEL_PHASE_1 = 1,
        AVL_TSG_PARALLEL_PHASE_2 = 2,
        AVL_TS_PARALLEL_PHASE_3 = 3
    }AVL_TSParallelPhase;

    // Defines whether the feeback bit of the LFSR used to generate the BER/PER test pattern is inverted.
    typedef enum AVL_LFSR_FbBit
    {
        AVL_LFSR_FB_NOT_INVERTED    =   0,          // LFSR feedback bit isn't inverted
        AVL_LFSR_FB_INVERTED        =   1           // LFSR feedback bit is inverted
    }AVL_LFSR_FbBit;

    // Defines the test pattern being used for BER/PER measurements.
    typedef enum AVL_TestPattern
    {
        AVL_TEST_LFSR_15    =   0,                  // BER test pattern is LFSR15
        AVL_TEST_LFSR_23    =   1                   // BER test pattern is LFSR23        
    }AVL_TestPattern;

    // Defines the type of auto error statistics 
    typedef enum AVL_AutoErrorStat_Type
    {
        AVL_ERROR_STAT_BYTE     =   0,                      // error statistics will be reset according to the number of received bytes.
        AVL_ERROR_STAT_TIME     =   1                       // error statistics will be reset according to time interval.
    }AVL_AutoErrorStat_Type;

    // Defines Error statistics mode
    typedef enum AVL_ErrorStat_Mode
    {
        AVL_ERROR_STAT_MANUAL   =   0,
        AVL_ERROR_STAT_AUTO     =   1
    }AVL_ErrorStat_Mode;

    //Emerald2  PLL
#define hw_E2_AVLEM61_reset_register                       0x00100000
#define hw_E2_AVLEM61_dll_init                             0x00100008
#define hw_E2_AVLEM61_deglitch_mode                        0x00100010
#define hw_E2_AVLEM61_sys_pll_bypass                       0x00100040
#define hw_E2_AVLEM61_sys_pll_enable                       0x00100044
#define hw_E2_AVLEM61_sys_pll_divr                         0x00100048
#define hw_E2_AVLEM61_sys_pll_divf                         0x0010004c
#define hw_E2_AVLEM61_sys_pll_divq                         0x00100050
#define hw_E2_AVLEM61_sys_pll_range                        0x00100054
#define hw_E2_AVLEM61_sys_pll_lock                         0x00100058
#define hw_E2_AVLEM61_mpeg_pll_bypass                      0x0010005c
#define hw_E2_AVLEM61_mpeg_pll_enable                      0x00100060
#define hw_E2_AVLEM61_mpeg_pll_divr                        0x00100064
#define hw_E2_AVLEM61_mpeg_pll_divf                        0x00100068
#define hw_E2_AVLEM61_mpeg_pll_divq                        0x0010006c
#define hw_E2_AVLEM61_mpeg_pll_range                       0x00100070
#define hw_E2_AVLEM61_mpeg_pll_lock                        0x00100074
#define hw_E2_AVLEM61_adc_pll_bypass                       0x00100078 
#define hw_E2_AVLEM61_adc_pll_enable                       0x0010007c
#define hw_E2_AVLEM61_adc_pll_divr                         0x00100080
#define hw_E2_AVLEM61_adc_pll_divf                         0x00100084
#define hw_E2_AVLEM61_adc_pll_divq                         0x00100088
#define hw_E2_AVLEM61_adc_pll_range                        0x0010008c
#define hw_E2_AVLEM61_adc_pll_lock                         0x00100090
#define hw_E2_AVLEM61_mpeg_pll_reset                       0x00100094
#define hw_E2_AVLEM61_adc_pll_reset                        0x00100098
#define hw_E2_AVLEM61_sys_pll_reset                        0x0010009c
#define hw_E2_AVLEM61_sys_pll_enable2                      0x001000b4
#define hw_E2_AVLEM61_sys_pll_enable3                      0x001000b8
#define hw_E2_AVLEM61_sys_pll_divq2                        0x001000bc
#define hw_E2_AVLEM61_sys_pll_divq3                        0x001000c0
#define hw_E2_AVLEM61_mpeg_pll_enable2                     0x001000c4
#define hw_E2_AVLEM61_mpeg_pll_enable3                     0x001000c8
#define hw_E2_AVLEM61_mpeg_pll_divq2                       0x001000cc
#define hw_E2_AVLEM61_mpeg_pll_divq3                       0x001000d0
#define hw_E2_AVLEM61_adc_pll_enable2                      0x001000d4
#define hw_E2_AVLEM61_adc_pll_enable3                      0x001000d8
#define hw_E2_AVLEM61_adc_pll_divq2                        0x001000dc
#define hw_E2_AVLEM61_adc_pll_divq3                        0x001000e0
#define hw_E2_AVLEM61_ddc_clk_sel                          0x001000e4
#define hw_E2_AVLEM61_sdram_clk_sel                        0x001000e8
#define hw_E2_AVLEM61_dll_out_phase                        0x00100100
#define hw_E2_AVLEM61_dll_rd_phase                         0x00100104

    typedef struct AVL_PLL_Conf0
    {
        uint32_t m_RefFrequency_Hz;                   // The reference clock frequency in units of Hz.
        uint8_t m_PLL_CoreClock_DivR;                 // PLL reference clock divider value
        uint8_t m_PLL_CoreClock_DivF;                 // PLL feedback clock divider value
        uint8_t m_PLL_CoreClock_DivQ;                 // PLL feedback clock divider value
        uint8_t m_PLL_MPEGClock_DivR;                 // PLL reference clock divider value
        uint8_t m_PLL_MPEGClock_DivF;                 // PLL feedback clock divider value
        uint8_t m_PLL_MPEGClock_DivQ;                 // PLL feedback clock divider value
        uint8_t m_PLL_ADCClock_DivR;                  // PLL reference clock divider value
        uint8_t m_PLL_ADCClock_DivF;                  // PLL feedback clock divider value
        uint8_t m_PLL_ADCClock_DivQ;                  // PLL feedback clock divider value
        uint32_t m_CoreFrequency_Hz;                  // The internal core clock frequency in units of Hz.
        uint32_t m_MPEGFrequency_Hz;                  // The MPEG clock frequency in units of Hz.
        uint32_t m_ADCFrequency_Hz;                   // The ADC clock frequency in units of Hz.
        uint8_t m_PLL_DDCClock_sel;                   // ddc_clk source select (Emerald2 only)
        uint8_t m_PLL_DDCClock_DivQ;                  // PLL feedback clock divider value(Emerald2 only)
        uint8_t m_PLL_SDRAMClock_sel;                 // sdram_clk source select (Emerald2 only)
        uint8_t m_PLL_SDRAMClock_DivQ;                // PLL feedback clock divider value(Emerald2 only)
        uint32_t m_DDCFrequency_Hz;                   // The DDC clock frequency in units of Hz (Emerald2 only).
        uint32_t m_SDRAMFrequency_Hz;                 // The SDRAM clock frequency in units of Hz (Emerald2 only).
    }AVL_PLL_Conf0;


#define hw_E2_PLL_SEL_CORE 0
#define hw_E2_PLL_SEL_MPEG 1
#define hw_E2_PLL_SEL_ADC  2


    // Used to configure the AVL PART device SDRAM controller. 
    typedef struct AVL_SDRAM_Conf0
    {
        uint16_t m_SDRAM_TRC;             // SDRAM active to active command period in ns. 
        uint16_t m_SDRAM_TXSR;            // SDRAM exit self-refresh to active command period in ns.
        uint16_t m_SDRAM_TRCAR;           // SDRAM auto refresh period in ns.  Minimum time between two auto refresh commands.
        uint16_t m_SDRAM_TWR;             // SDRAM write recovery time in SDRAM clock cycles->  The delay from the last data in to the next precharge command.  Valid range is 1 to 4 clock cycles->
        uint16_t m_SDRAM_TRP;             // SDRAM precharge period in ns.
        uint16_t m_SDRAM_TRCD;            // SDRAM minimum delay between active and read/write commands in ns.
        uint16_t m_SDRAM_TRAS;            // SDRAM minimum delay between active and precharge commands in ns.
        uint16_t m_SDRAM_CAS_Latency;     // SDRAM delay between read command and availability of first data in SDRAM clock cycles->  Valid range is 1 to 4 clock cycles->
    }AVL_SDRAM_Conf0;

    // Defines the DiSEqC status
    typedef enum AVL_DiseqcStatus
    {
        AVL_DOS_Uninitialized = 0,                  // DiSEqC has not been initialized yet.
        AVL_DOS_Initialized   = 1,                  // DiSEqC has been initialized.
        AVL_DOS_InContinuous  = 2,                  // DiSEqC is in continuous mode.
        AVL_DOS_InTone        = 3,                  // DiSEqC is in tone burst mode.
        AVL_DOS_InModulation  = 4                   // DiSEqC is in modulation mode.
    }AVL_DiseqcStatus;

    // Contains variables for storing error statistics used in the BER and PER calculations.
    typedef struct AVL_ErrorStats
    {
        uint16_t usLFSRSynced;    // Indicates whether the receiver is synchronized with the transmitter generating the BER test pattern.
        uint16_t usLostLock;      // Indicates whether the receiver has lost lock since the BER/PER measurement was started.
        struct avl_uint64 stSwCntNumBits;     // A software counter which stores the number of bits which have been received.
        struct avl_uint64 stSwCntBitErrors;   // A software counter which stores the number of bit errors which have been detected.
        struct avl_uint64 stNumBits;          // The total number of bits which have been received.
        struct avl_uint64 stBitErrors;        // The total number of bit errors which have been detected.
        struct avl_uint64 stSwCntNumPkts;     // A software counter which stores the number of packets which have been received.
        struct avl_uint64 stSwCntPktErrors;   // A software counter which stores the number of packet errors which have been detected.
        struct avl_uint64 stNumPkts;          // The total number of packets which have been received.
        struct avl_uint64 stPktErrors;        // The total number of packet errors which have been detected.
        uint32_t uiBER;             // The bit error rate scaled by 1e9.
        uint32_t uiPER;             // The packet error rate scaled by 1e9.
    }AVL_ErrorStats;

    typedef enum AVL_Demod_Xtal
    {
        Xtal_30M = 0,
        Xtal_16M,
        Xtal_24M,
        Xtal_27M
    } AVL_Demod_Xtal;

    typedef enum AVL_InputPath
    {
        AVL_IF_I,
        AVL_IF_Q
    } AVL_InputPath;

    // Contains variables for storing error statistics used in the BER and PER calculations.
    typedef struct AVL_ErrorStatConfig
    {
        AVL_ErrorStat_Mode      eErrorStatMode;           // indicates the error statistics mode. 
        AVL_AutoErrorStat_Type  eAutoErrorStatType;       // indicates the MPEG data sampling clock mode.
        uint32_t              uiTimeThresholdMs;        // used to set time interval for auto error statistics.
        uint32_t              uiNumberThresholdByte;    // used to set the received byte number threshold for auto error statistics.
    }AVL_ErrorStatConfig;

    // Contains variables for storing error statistics used in the BER and PER calculations.
    typedef struct AVL_BERConfig
    {
        AVL_TestPattern         eBERTestPattern;         // indicates the pattern of LFSR.
        AVL_LFSR_FbBit          eBERFBInversion;         // indicates LFSR feedback bit inversion.
        uint32_t              uiLFSRSynced;                // indicates the LFSR synchronization status.
        uint32_t              uiLFSRStartPos;         //set LFSR start byte positon
    }AVL_BERConfig;

  typedef enum AVL_GPIOPinNumber
    {
      AVL_Pin37 = 0,
      AVL_Pin38 = 1,
      AVL_Pin15 = 2  //CS_0 PIN
      
    }AVL_GPIOPinNumber;


  typedef enum AVL_GPIOPinValue
    {
      AVL_LOGIC_0 = 0,
      AVL_LOGIC_1 = 1,
      AVL_HIGH_Z = 2
    }AVL_GPIOPinValue;

  
  
  typedef enum AVL_Diseqc_WaveFormMode
    {
      AVL_DWM_Normal = 0,                         // Normal waveform mode
      AVL_DWM_Envelope = 1                        // Envelope waveform mode
    }AVL_Diseqc_WaveFormMode;
  
  typedef enum AVL_ISDBT_BandWidth
    {
      AVL_ISDBT_BW_6M  =   0,
      AVL_ISDBT_BW_8M  =   1,
    }AVL_ISDBT_BandWidth;
  
  typedef enum AVL_DVBC_Standard
    {
      AVL_DVBC_J83A    =   0,           //the J83A standard
      AVL_DVBC_J83B    =   1,           //the J83B standard
      AVL_DVBC_UNKNOWN =   2
    }AVL_DVBC_Standard;

  typedef enum AVL_AGC_Selection
  {
    AVL_NO_AGC = 0,
    AVL_TC_AGC_ONLY = 1,
    AVL_S_AGC_ONLY = 2,
    AVL_BOTH_AGC = 3
  } AVL_AGC_Selection;

  #define AVL_AGC_ON_VAL  6
  #define AVL_AGC_OFF_VAL 2
  
  // Defines the device spectrum polarity setting. 
  typedef enum AVL_SpectrumPolarity
  {
      AVL_Spectrum_Normal = 0,
      AVL_Spectrum_Invert = 1
  }AVL_SpectrumPolarity;
  /**************************************************/
  
  
  typedef struct AVL_TSConfig
  {
    AVL_TSMode eMode;
    AVL_TSClockEdge eClockEdge;
    AVL_TSClockMode eClockMode;
    AVL_TSSerialPin eSerialPin;
    AVL_TSSerialOrder eSerialOrder;
    AVL_TSSerialSyncPulse eSerialSyncPulse;
    AVL_TSErrorBit eErrorBit;
    AVL_TSErrorPolarity eErrorPolarity;
    AVL_TSValidPolarity eValidPolarity;
    AVL_TSParallelPhase eParallelPhase;
    AVL_TSParallelOrder eParallelOrder;
    uint32_t guiDVBTxSerialTSContinuousHz;
    uint32_t guiDVBSxSerialTSContinuousHz;
    uint32_t guiISDBTSerialTSContinuousHz;
    uint32_t guiDVBCSerialTSContinuousHz;
  }AVL_TSConfig;

  typedef struct AVL_CommonConfig
  {
    AVL_Demod_Xtal  xtal;
    AVL_TSConfig ts_config;
  }AVL_CommonConfig;
  
  typedef struct AVL_DVBTxConfig
  {
    AVL_InputPath eDVBTxInputPath;
    uint32_t  uiDVBTxIFFreqHz;
    AVL_AGCPola eDVBTxAGCPola;
  } AVL_DVBTxConfig;
  
  typedef struct AVL_DVBCConfig
  { 
    AVL_InputPath eDVBCInputPath;
    uint32_t uiDVBCIFFreqHz;
    AVL_AGCPola eDVBCAGCPola;
  } AVL_DVBCConfig;
  
  typedef struct AVL_DVBSxConfig
  {
    AVL_AGCPola eDVBSxAGCPola;
    AVL_Diseqc_WaveFormMode e22KWaveForm;
  } AVL_DVBSxConfig;
  
  
  typedef struct AVL_ISDBTConfig
  { 
    AVL_InputPath eISDBTInputPath;
    uint32_t uiISDBTIFFreqHz;
    AVL_AGCPola eISDBTAGCPola;
  } AVL_ISDBTConfig;

  typedef enum AVL_TunerType
	{
		AVL_REAL_IF            =   0,
		AVL_COMPLEX_BASEBAND   =   1,
		AVL_REAL_IF_FROM_Q     =   2
	} AVL_TunerType;

  typedef struct AVL_BaseAddressSet
  {
    uint32_t hw_mcu_reset_base;
    uint32_t hw_mcu_system_reset_base;
    uint32_t hw_esm_base;
    uint32_t hw_tuner_i2c_base;
    uint32_t hw_gpio_control_base;
    uint32_t hw_gpio_debug_base;
    uint32_t hw_gpio_modu_002_base;
    uint32_t hw_emerald_io_base ;
    uint32_t hw_TS_tri_state_cntrl_base;
    uint32_t hw_AGC_tri_state_cntrl_base;
    uint32_t hw_diseqc_base;
    uint32_t hw_plp_list_base;
    uint32_t hw_blind_scan_info_base;
    uint32_t hw_member_ID_base;
    uint32_t hw_dma_sys_status_base;
    uint32_t hw_dma_sys_cmd_base;
    uint32_t hw_dvb_gen1_fec__base;
    uint32_t fw_config_reg_base;
    uint32_t fw_status_reg_base;
    uint32_t fw_DVBTx_config_reg_base;
    uint32_t fw_DVBTx_status_reg_base;
    uint32_t fw_DVBT2_data_PLP_config_reg_base;
    uint32_t fw_DVBT2_common_PLP_config_reg_base;
    uint32_t fw_DVBT2_P1_reg_base;
    uint32_t fw_DVBT2_L1_pre_reg_base;
    uint32_t fw_DVBT2_L1_post_config_reg_base;
    uint32_t fw_DVBT_TPS_reg_base;
    uint32_t fw_DVBSx_config_reg_base;
    uint32_t fw_DVBSx_status_reg_base;
    uint32_t fw_ISDBT_config_reg_base;
    uint32_t fw_ISDBT_status_reg_base;
    uint32_t fw_DVBC_config_reg_base;
    uint32_t fw_DVBC_status_reg_base;
  }AVL_BaseAddressSet;

  struct AVL_StandardSpecificFunctions;

typedef struct avl68x2_chip_priv
{
  uint8_t * patch_data;
  uint32_t variable_array[PATCH_VAR_ARRAY_SIZE];

  uint8_t sleep_flag;  //0 - Wakeup, 1 - Sleep 
  uint8_t agc_driven;

} avl68x2_chip_priv;

typedef struct avl68x2_chip_pub
{
  /*
	* demod ID and I2C slave address
	* ((ID & AVL_DEMOD_ID_MASK)<<8) | (slv_addr & 0xFF)
	*/
	uint16_t i2c_addr;

  AVL_Demod_Xtal xtal;

  AVL_TSConfig ts_config;

  AVL_DemodMode cur_demod_mode;
  AVL_DVBTxConfig dvbtx_config;
  AVL_DVBSxConfig dvbsx_config;
  AVL_ISDBTConfig isdbt_config;
  AVL_DVBCConfig dvbc_config;

  int32_t gpio_lock_led;
  int32_t gpio_fec_reset;

  AVL_DiseqcStatus eDiseqcStatus;

  AVL_AGC_Selection tc_agc_selection;
  AVL_AGC_Selection s_agc_selection;
  AVL_TunerType tc_tuner_type;
  AVL_SpectrumPolarity tuner_pol; //S tuner spectrum polarity
  struct avl_tuner *tuner;
  int32_t tuner_freq_hz;
} avl68x2_chip_pub;

  typedef struct avl68x2_chip
  {
    struct avl68x2_chip_priv *chip_priv;
    struct avl68x2_chip_pub *chip_pub;

    uint8_t rx_sem_initialized;
    uint8_t diseqc_sem_initialized;
    uint32_t family_id;
    
    
    avl_sem_t rx_sem;
    avl_sem_t diseqc_sem;
    uint32_t uiCoreFrequencyHz;
    uint32_t uiFECFrequencyHz;
    uint32_t uiTSFrequencyHz;
    uint32_t uiADCFrequencyHz;
    uint32_t uiRefFrequencyHz;
    uint32_t uiDDCFrequencyHz;
    uint32_t uiSDRAMFrequencyHz;
    
    AVL_ErrorStatConfig stErrorStatConfig;
    
    struct AVL_StandardSpecificFunctions  *stStdSpecFunc;
    AVL_ErrorStats stAVLErrorStat;

    
    
    //uint8_t ucPin37Status; // 0 - InPut; 1- OutPut
    //uint8_t ucPin38Status;
    uint8_t ucPin37Voltage; // 0 - Low; 1- High; 2 - High_Z
    uint8_t ucPin38Voltage;   
    uint8_t ucPin15Voltage;
    
    
        
  } avl68x2_chip;


  
  // The Availink version structure.
  typedef struct AVL_Version
  {
    uint8_t   major;
    uint8_t   minor;
    uint16_t  build;
  }AVL_Version;
  
  // Stores AVLEM61 device version information.
  typedef struct AVL_DemodVersion
  {
    uint32_t  hardware;
    AVL_Version firmware;
    AVL_Version driver;
  } AVL_DemodVersion;
  
  
  extern const AVL_BaseAddressSet stBaseAddrSet;
  extern AVL_PLL_Conf0 gstPLLConfigArray0[];
  avl_error_code_t avl68x2_init_chip_object(avl68x2_chip *chip);
  avl_error_code_t IBase_Initialize_Demod(avl68x2_chip *chip);
  avl_error_code_t TunerI2C_Initialize_Demod(avl68x2_chip *chip);
  avl_error_code_t EnableTSOutput_Demod(avl68x2_chip *chip);
  avl_error_code_t DisableTSOutput_Demod(avl68x2_chip *chip);
  avl_error_code_t InitErrorStat_Demod(avl68x2_chip *chip);
  avl_error_code_t ErrorStatMode_Demod(AVL_ErrorStatConfig stErrorStatConfig,avl68x2_chip *chip);
  avl_error_code_t ResetErrorStat_Demod(avl68x2_chip *chip);
  avl_error_code_t ResetPER_Demod(avl68x2_chip *chip);
  avl_error_code_t ResetBER_Demod(AVL_BERConfig *pstErrorStatConfig, avl68x2_chip *chip);
  avl_error_code_t SetPLL_Demod(   avl68x2_chip *chip);
  avl_error_code_t IBase_CheckChipReady_Demod(avl68x2_chip *chip);
  avl_error_code_t IRx_Initialize_Demod(avl68x2_chip *chip);
  avl_error_code_t IBase_SendRxOPWait_Demod(uint8_t ucOpCmd, avl68x2_chip *chip);
  avl_error_code_t IBase_GetRxOPStatus_Demod(avl68x2_chip *chip);
  avl_error_code_t SetTSMode_Demod(avl68x2_chip *chip);
  avl_error_code_t SetInternalFunc_Demod(AVL_DemodMode eDemodMode, avl68x2_chip *chip);

  avl_error_code_t EnableTCAGC_Demod(avl68x2_chip *chip);
  avl_error_code_t DisableTCAGC_Demod(avl68x2_chip *chip);
  avl_error_code_t EnableSAGC_Demod(avl68x2_chip *chip);
  avl_error_code_t DisableSAGC_Demod(avl68x2_chip *chip);
  avl_error_code_t ConfigAGCOutput_Demod(avl68x2_chip *chip);

  avl_error_code_t SetTSSerialPin_Demod(AVL_TSSerialPin TSSerialPin, avl68x2_chip *chip);
  avl_error_code_t SetTSSerialOrder_Demod(AVL_TSSerialOrder TSSerialOrder, avl68x2_chip *chip);
  avl_error_code_t SetTSSerialSyncPulse_Demod(AVL_TSSerialSyncPulse TSSerialSyncPulse, avl68x2_chip *chip);
  avl_error_code_t SetTSErrorBit_Demod(AVL_TSErrorBit TSErrorBit, avl68x2_chip *chip);
  avl_error_code_t SetTSErrorPola_Demod(AVL_TSErrorPolarity TSErrorPola, avl68x2_chip *chip);
  avl_error_code_t SetTSValidPola_Demod(AVL_TSValidPolarity TSValidPola, avl68x2_chip *chip);
  avl_error_code_t SetTSParallelOrder_Demod(AVL_TSParallelOrder TSParallelOrder, avl68x2_chip *chip);
  avl_error_code_t SetTSParallelPhase_Demod(AVL_TSParallelPhase eParallelPhase, avl68x2_chip *chip);
  avl_error_code_t IBase_SetSleepClock_Demod(avl68x2_chip *chip);
  avl_error_code_t GetMode_Demod(AVL_DemodMode* peCurrentMode, avl68x2_chip *chip);
  avl_error_code_t GetBER_Demod(uint32_t *puiBERxe9, AVL_BER_Type  enumBERType, avl68x2_chip *chip);
  


  avl_error_code_t SetPLL0_Demod(avl68x2_chip *chip);
  avl_error_code_t SetSleepPLL0_Demod(avl68x2_chip *chip);
  avl_error_code_t TestSDRAM_Demod(uint32_t * puiTestResult, uint32_t * puiTestPattern, avl68x2_chip *chip);
  avl_error_code_t GetValidModeList_Demod(uint8_t * pucValidModeList, avl68x2_chip *chip);
  void GetValidModeList0_Demod(uint8_t * pucValidModeList, uint32_t uiMemberID);
  avl_error_code_t GetFamilyID_Demod(uint32_t * puiFamilyID,avl68x2_chip *chip);
  avl_error_code_t Initilize_GPIOStatus_Demod(avl68x2_chip *chip);
  avl_error_code_t SetGPIOStatus_Demod(avl68x2_chip *chip);
  avl_error_code_t AVL_ParseFwPatch_v0(avl68x2_chip *chip, uint8_t download_only );
  uint8_t AVL_patch_read8(uint8_t * pPatchBuf, uint32_t *idx, uint8_t update_idx );
  uint16_t AVL_patch_read16(uint8_t * pPatchBuf, uint32_t *idx, uint8_t update_idx );
  uint32_t AVL_patch_read32(uint8_t * pPatchBuf, uint32_t *idx, uint8_t update_idx );
  
  
  typedef avl_error_code_t (* AVL_Func_Initialize)(avl68x2_chip *chip);
  typedef avl_error_code_t (* AVL_Func_GetLockStatus)(uint8_t * pucLocked, avl68x2_chip *chip );
  typedef avl_error_code_t (* AVL_Func_GetSSI)(uint16_t * puiStrength , avl68x2_chip *chip);
  typedef avl_error_code_t (* AVL_Func_GetSQI)(uint16_t * puiQuality , avl68x2_chip *chip);
  typedef avl_error_code_t (* AVL_Func_GetSNR)(uint32_t * puiSNR_db, avl68x2_chip *chip); 
  typedef avl_error_code_t (* AVL_Func_GetPrePostBER)(uint32_t *puiBERxe9, AVL_BER_Type eBERType, avl68x2_chip *chip);
  
  typedef struct AVL_StandardSpecificFunctions
  {
    AVL_Func_Initialize fpRxInitializeFunc;
    AVL_Func_GetLockStatus fpGetLockStatus;
    AVL_Func_GetSNR fpGetSNR;
    AVL_Func_GetSQI fpGetSQI;
    AVL_Func_GetPrePostBER fpGetPrePostBER;
  }AVL_StandardSpecificFunctions;
 


#endif

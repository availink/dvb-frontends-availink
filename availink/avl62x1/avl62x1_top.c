// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Availink AVL62X1 DVB-S/S2/S2X demodulator driver
 * Supports AVL6221 and AVL6261. NOT AVL6211
 *
 * Copyright (C) 2020 Availink, Inc. (gpl@availink.com)
 *
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/string.h>
#include <linux/bitrev.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/ioctl.h>

#include "avl62x1.h"
#include "avl62x1_api.h"
#include "avl_tuner.h"

#define INCLUDE_STDOUT	0

#define p_debug(fmt, args...)					\
	do							\
	{							\
		if (debug)					\
			printk("avl62x1:%s DEBUG: " fmt "\n",	\
				__func__, ##args);		\
	} while (0);

#define p_debug_lvl(lvl,fmt, args...)				\
	do							\
	{							\
		if (debug >= lvl)				\
			printk("avl62x1:%s DEBUG: " fmt "\n",	\
				__func__, ##args);		\
	} while (0);

#define p_error(fmt, args...)					\
	do							\
	{							\
		printk("avl62x1:%s ERROR: " fmt "\n",		\
			__func__, ##args);			\
	} while (0);

#define p_info(fmt, args...)					\
	do							\
	{							\
		printk("avl62x1:%s INFO: " fmt "\n",		\
			__func__, ##args);			\
	} while (0);

#define p_warn(fmt, args...)					\
	do							\
	{							\
		printk("avl62x1:%s WARN: " fmt "\n",		\
			__func__, ##args);			\
	} while (0);

#define safe_mutex_unlock(m)					\
	if(mutex_is_locked(m)) {				\
		mutex_unlock(m);				\
	} else {						\
		p_warn("Tried to unlock mutex that wasn't locked"); \
	}

static struct avl62x1_bs_state bs_states[AVL_MAX_NUM_DEMODS] = {0};
static struct avl62x1_priv *global_priv = NULL;
static char sel_fw[256] = {0};

//--- module params ---
static int debug = 0;
static unsigned short bs_mode = 0;
static int bs_min_sr = 1000000;
char fw_path[256] = {0};
//-------------------

//---- character device stuff ----
#define I2CCTL_DEVICE_NAME "avl62x1_i2c" //device at /dev/DEVICE_NAME
#define I2CCTL_CLASS_NAME  "avl62x1_i2c"     //device class
#define I2CCTL_IOC_MAGIC 'k'
#define I2CCTL_IOCLOCK _IO(I2CCTL_IOC_MAGIC, 0)
#define I2CCTL_IOCUNLOCK _IO(I2CCTL_IOC_MAGIC, 1)
#define I2CCTL_IOC_MAXNR 1

static DEFINE_MUTEX(i2cctl_dev_mutex); //mutex for char device
//-----
//N.B. these mutexes are monolithic - there's only one for all demod instances
//this may be improved in future versions of this driver
static DEFINE_MUTEX(i2cctl_fe_mutex); //mutex for whole FE controlled thru ioctl
static DEFINE_MUTEX(i2cctl_tuneri2c_mutex); //mutex for tuneri2c access
//-----

static int    i2cctl_maj_num;                  ///< Stores the device number -- determined automatically
static struct class*  i2cctl_class  = NULL; ///< The device-driver class struct pointer
static struct device* i2cctl_device = NULL; ///< The device-driver device struct pointer
 
// The prototype functions for the character driver -- must come before the struct definition
static int     i2cctl_dev_open(struct inode *, struct file *);
static int     i2cctl_dev_release(struct inode *, struct file *);
static ssize_t i2cctl_dev_read(struct file *, char *, size_t, loff_t *);
static ssize_t i2cctl_dev_write(struct file *, const char *, size_t, loff_t *);
static long    i2cctl_dev_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

static struct file_operations i2cctl_fops =
    {
	.owner = THIS_MODULE,
	.open = i2cctl_dev_open,
	.read = i2cctl_dev_read,
	.write = i2cctl_dev_write,
	.release = i2cctl_dev_release,
	.unlocked_ioctl = i2cctl_dev_ioctl};

int init_avl62x1_i2cctl_device(struct avl62x1_priv *priv)
{
	i2cctl_maj_num = register_chrdev(0, I2CCTL_DEVICE_NAME, &i2cctl_fops);
	if (i2cctl_maj_num < 0)
	{
		p_error("failed to register a major number");
		return i2cctl_maj_num;
	}

	// Register the device class
	i2cctl_class = class_create(THIS_MODULE, I2CCTL_CLASS_NAME);
	if (IS_ERR(i2cctl_class))
	{ // Check for error and clean up if there is
		unregister_chrdev(i2cctl_maj_num, I2CCTL_DEVICE_NAME);
		p_error("Failed to register device class");
		return PTR_ERR(i2cctl_class); // Correct way to return an error on a pointer
	}

	// Register the device driver
	i2cctl_device = device_create(i2cctl_class, NULL, MKDEV(i2cctl_maj_num, 0), NULL, I2CCTL_DEVICE_NAME);
	if (IS_ERR(i2cctl_device))
	{				     // Clean up if there is an error
		class_destroy(i2cctl_class); // Repeated code but the alternative is goto statements
		unregister_chrdev(i2cctl_maj_num, I2CCTL_DEVICE_NAME);
		p_error("Failed to create the device");
		return PTR_ERR(i2cctl_device);
	}

	mutex_init(&i2cctl_dev_mutex);
	mutex_init(&i2cctl_fe_mutex);
	mutex_init(&i2cctl_tuneri2c_mutex);
	return 0;
}

void deinit_avl62x1_i2cctl_device(struct avl62x1_priv *priv)
{
	device_destroy(i2cctl_class, MKDEV(i2cctl_maj_num, 0)); // remove the device
	class_unregister(i2cctl_class);			     // unregister the device class
	class_destroy(i2cctl_class);			     // remove the device class
	unregister_chrdev(i2cctl_maj_num, I2CCTL_DEVICE_NAME);	     // unregister the major number

	mutex_unlock(&i2cctl_fe_mutex);
	mutex_unlock(&i2cctl_tuneri2c_mutex);

	mutex_destroy(&i2cctl_dev_mutex);
}

static long i2cctl_dev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct i2cctl_ioctl_lock_req *req;

	/*
	* extract the type and number bitfields, and don't decode
	* wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
	*/
	if (_IOC_TYPE(cmd) != I2CCTL_IOC_MAGIC)
		return -ENOTTY;
	if (_IOC_NR(cmd) > I2CCTL_IOC_MAXNR)
		return -ENOTTY;

	req = (struct i2cctl_ioctl_lock_req *)arg;

	switch (cmd)
	{
	case I2CCTL_IOCLOCK:
		if(req->demod) {
			mutex_lock(&i2cctl_fe_mutex);
			p_debug_lvl(4,"locked demod");
		}
		if(req->tuner) {
			mutex_lock(&i2cctl_tuneri2c_mutex);
			p_debug_lvl(4,"locked tuner");
		}
		break;
	case I2CCTL_IOCUNLOCK:
		if(req->demod) {
			if(mutex_is_locked(&i2cctl_fe_mutex)) {
				mutex_unlock(&i2cctl_fe_mutex);
				p_debug_lvl(4,"unlocked demod");
			}
		}
		if(req->tuner) {
			if(mutex_is_locked(&i2cctl_tuneri2c_mutex)) {
				mutex_unlock(&i2cctl_tuneri2c_mutex);
				p_debug_lvl(4,"unlocked tuner");
			}
		}
		break;
	default: /* redundant, as cmd was checked against MAXNR */
		p_error("Unknown IOCTL %d",cmd);
		return -ENOTTY;
	}

	return 0;
}

static int i2cctl_dev_open(struct inode *inodep, struct file *filep)
{
	p_debug_lvl(4,"");
	if (!mutex_trylock(&i2cctl_dev_mutex))
	{	/// Try to acquire the mutex (i.e., put the lock on/down)
		/// returns 1 if successful and 0 if there is contention
		p_error("Device in use by another process");
		return -EBUSY;
	}
	return 0;
}

//read from device from userspace
static ssize_t i2cctl_dev_read(
    struct file *filep,
    char *buffer,   /* The buffer to fill with data */
    size_t len,	    /* The length of the buffer     */
    loff_t *offset) /* Our offset in the file       */
{
	char msg[128];

	snprintf(msg, 127, "i2c_device=%d,i2c_addr=0x%.2X",
		i2c_adapter_id(global_priv->i2c),
		global_priv->chip->chip_pub->i2c_addr & 0xFF);
	p_debug_lvl(4,"%s",msg);

	return simple_read_from_buffer(buffer, len, offset, msg, strlen(msg));
}

//write to device from userspace
static ssize_t i2cctl_dev_write(struct file *filep, const char *buffer, size_t len, loff_t *offset)
{
	p_debug_lvl(4,"%lu",len);
	return len;
}

static int i2cctl_dev_release(struct inode *inodep, struct file *filep)
{
	p_debug_lvl(4,"");
	mutex_unlock(&i2cctl_dev_mutex);
	return 0;
}
//--------------------------------------------------------------------------

//-------------- stdout character device -----------------------------------
#if INCLUDE_STDOUT
#include "read_stdout_62x1.c"
#define STDOUT_DEVICE_NAME "avl62x1_stdout"
#define STDOUT_CLASS_NAME  "avl62x1_stdout"
static DEFINE_MUTEX(stdout_dev_mutex);
static int stdout_maj_num;
static struct class*  stdout_class  = NULL;
static struct device* stdout_device = NULL;
static int     stdout_dev_open(struct inode *, struct file *);
static int     stdout_dev_release(struct inode *, struct file *);
static ssize_t stdout_dev_read(struct file *, char *, size_t, loff_t *);
static struct file_operations stdout_fops =
    {
	.owner = THIS_MODULE,
	.open = stdout_dev_open,
	.read = stdout_dev_read,
	.release = stdout_dev_release};
int init_avl62x1_stdout_device(struct avl62x1_priv *priv)
{
	stdout_maj_num = register_chrdev(0, STDOUT_DEVICE_NAME, &stdout_fops);
	if (stdout_maj_num < 0)
	{
		p_error("failed to register a major number");
		return stdout_maj_num;
	}

	// Register the device class
	stdout_class = class_create(THIS_MODULE, STDOUT_CLASS_NAME);
	if (IS_ERR(stdout_class))
	{ // Check for error and clean up if there is
		unregister_chrdev(stdout_maj_num, STDOUT_DEVICE_NAME);
		p_error("Failed to register device class");
		return PTR_ERR(stdout_class); // Correct way to return an error on a pointer
	}

	// Register the device driver
	stdout_device = device_create(stdout_class, NULL, MKDEV(stdout_maj_num, 0), NULL, STDOUT_DEVICE_NAME);
	if (IS_ERR(stdout_device))
	{				     // Clean up if there is an error
		class_destroy(stdout_class); // Repeated code but the alternative is goto statements
		unregister_chrdev(stdout_maj_num, STDOUT_DEVICE_NAME);
		p_error("Failed to create the device");
		return PTR_ERR(stdout_device);
	}

	mutex_init(&stdout_dev_mutex);
	return 0;
}

void deinit_avl62x1_stdout_device(struct avl62x1_priv *priv)
{
	device_destroy(stdout_class, MKDEV(stdout_maj_num, 0)); // remove the device
	class_unregister(stdout_class);			     // unregister the device class
	class_destroy(stdout_class);			     // remove the device class
	unregister_chrdev(stdout_maj_num, STDOUT_DEVICE_NAME);	     // unregister the major number

	mutex_destroy(&stdout_dev_mutex);
}

static int stdout_dev_open(struct inode *inodep, struct file *filep)
{
	p_debug_lvl(4,"");
	if (!mutex_trylock(&stdout_dev_mutex))
	{	/// Try to acquire the mutex (i.e., put the lock on/down)
		/// returns 1 if successful and 0 if there is contention
		p_error("Device in use by another process");
		return -EBUSY;
	}
	return 0;
}

static ssize_t stdout_dev_read(
    struct file *filep,
    char *buffer,   /* The buffer to fill with data */
    size_t len,	    /* The length of the buffer     */
    loff_t *offset) /* Our offset in the file       */
{

	char *stdout_buf;
	char *sp_stdout_buf;
	int ttl;
	stdout_buf = read_stdout(&global_priv->frontend);
	sp_stdout_buf = read_sp_stdout(&global_priv->frontend);
	ttl = simple_read_from_buffer(buffer,len,offset,stdout_buf,strlen(stdout_buf));
	ttl += simple_read_from_buffer(&buffer[strlen(stdout_buf)],len,offset,sp_stdout_buf,strlen(sp_stdout_buf));
	return ttl;
}

static int stdout_dev_release(struct inode *inodep, struct file *filep)
{
	p_debug_lvl(4,"");
	safe_mutex_unlock(&stdout_dev_mutex);
	return 0;
}
#endif
//--------------------------------------------------------------------------

static int diseqc_set_voltage(struct dvb_frontend *fe,
			      enum fe_sec_voltage voltage);

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

static int avl68x2_init_diseqc(struct dvb_frontend *fe)
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
		p_debug("Diseqc Init failed !\n");
	}

	diseqc_set_voltage(fe, SEC_VOLTAGE_OFF);

	return r;
}

//called from dvb_frontend::dvb_frontend_init
// and from ::set_frontend
static int i2c_gate_ctrl(struct dvb_frontend *fe, int enable)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	uint16_t ret = AVL_EC_OK;

	p_debug("%d", enable);

	mutex_lock(&i2cctl_fe_mutex);


	if (enable)
	{
		mutex_lock(&i2cctl_tuneri2c_mutex);
		ret = avl62x1_enable_tuner_i2c(priv->chip);
	}
	else
	{
		ret = avl62x1_disable_tuner_i2c(priv->chip);
		safe_mutex_unlock(&i2cctl_tuneri2c_mutex);
	}

	safe_mutex_unlock(&i2cctl_fe_mutex);
	
	return ret;
}

static int acquire_dvbs_s2(struct dvb_frontend *fe)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	uint16_t r = AVL_EC_OK;
	struct avl62x1_carrier_info carrier_info;
	struct avl62x1_stream_info stream_info;
	
	p_debug("Freq:%d khz,sym:%d hz", c->frequency, c->symbol_rate);

	carrier_info.rf_freq_khz = c->frequency;
	carrier_info.carrier_freq_offset_hz = 0;
	carrier_info.symbol_rate_hz = c->symbol_rate;
	//"-1" means ISI not specified and is very unlikely to be a valid
	//  decoding of the other t2mi fields
	if ((c->stream_id != -1) &&
	    ((c->stream_id >> AVL62X1_BS_IS_T2MI_SHIFT) & 0x1))
	{
		stream_info.stream_type = avl62x1_t2mi;
		stream_info.t2mi.pid =
		    (c->stream_id >> AVL62X1_BS_T2MI_PID_SHIFT) & 0x1FFF;
		stream_info.t2mi.plp_id =
		    (c->stream_id >> AVL62X1_BS_T2MI_PLP_ID_SHIFT) & 0xFF;
		stream_info.t2mi.raw_mode = 0;
		stream_info.isi = c->stream_id & 0xFF;

		carrier_info.pl_scrambling = AVL62X1_PL_SCRAM_AUTO;
		printk("Acquire T2MI\n");
	}
	else
	{
		stream_info.stream_type = avl62x1_transport;
		stream_info.isi = (c->stream_id == -1) ? 0 : (c->stream_id & 0xFF);
#if DVB_VER_ATLEAST(5, 11)
		//use scrambling_sequence_index if it's not the default n=0 val
		if (c->scrambling_sequence_index)
		{
			carrier_info.pl_scrambling =
			    c->scrambling_sequence_index;
		}
		else
		{
			carrier_info.pl_scrambling = AVL62X1_PL_SCRAM_AUTO;
		}

#else
		carrier_info.pl_scrambling = AVL62X1_PL_SCRAM_AUTO;
#endif
		printk("Acquire TS\n");
	}

	r = avl62x1_lock_tp(&carrier_info,
			    &stream_info,
			    AVL_FALSE, /* don't do blind symbol rate */
			    priv->chip);

	return r;
}

static int set_dvb_mode(struct dvb_frontend *fe,
			enum fe_delivery_system delsys,
			bool force_reload)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	uint16_t ret = AVL_EC_OK;
	uint16_t r = AVL_EC_OK;
	struct avl62x1_ver_info ver_info;

	/* already in desired mode */
	if (!force_reload && (priv->delivery_system == delsys))
		return 0;

	p_debug("initing demod for delsys=%d", delsys);

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
		p_debug("Failed to Resed demod via BSP!\n");
		return r;
	}

	// boot the firmware here
	r |= avl62x1_initialize(priv->chip);
	if (AVL_EC_OK != r)
	{
		p_debug("AVL_AVL62X1_Initialize failed !\n");
		return (r);
	}

	r |= avl62x1_get_version(&ver_info, priv->chip);
	if (AVL_EC_OK != r)
	{
		p_debug("avl62x1_get_version failed\n");
		return (r);
	}
	p_debug("FW version %d.%d.%d\n",
		ver_info.firmware.major,
		ver_info.firmware.minor,
		ver_info.firmware.build);
	p_debug("Driver version %d.%d.%d\n",
		ver_info.driver.major,
		ver_info.driver.minor,
		ver_info.driver.build);

	switch (priv->delivery_system)
	{
	case SYS_DVBS:
	case SYS_DVBS2:
	default:
		ret |= avl68x2_init_diseqc(fe);
		break;
	}

	ret |= init_error_stat(priv);

	if (ret)
	{
		p_error("demod init failed");
	}

	return ret;
}

uint16_t diseqc_send_cmd(struct avl62x1_priv *priv,
			 uint8_t *cmd,
			 uint8_t cmdsize)
{
	uint16_t r = AVL_EC_OK;
	struct avl62x1_diseqc_tx_status tx_status;

	p_debug(" %*ph", cmdsize, cmd);

	mutex_lock(&i2cctl_fe_mutex);
	
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

	safe_mutex_unlock(&i2cctl_fe_mutex);

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

	p_debug("");

	mutex_lock(&i2cctl_fe_mutex);

	ret = (int)avl62x1_send_diseqc_tone(tone, count, priv->chip);

	safe_mutex_unlock(&i2cctl_fe_mutex);

	return ret;
}

static int diseqc_set_tone(struct dvb_frontend *fe, enum fe_sec_tone_mode tone)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	int r = AVL_EC_OK;

	p_debug("tone: %s", tone==SEC_TONE_ON ? "ON" : "OFF");

	mutex_trylock(&i2cctl_fe_mutex);

	switch (tone)
	{
	case SEC_TONE_ON:
		if (priv->chip->chip_priv->diseqc_op_status !=
		    avl62x1_dos_continuous)
		{
			p_debug("call avl62x1_diseqc_tone_on()");
			r = (int)avl62x1_diseqc_tone_on(priv->chip);
		}
		break;
	case SEC_TONE_OFF:
		if (priv->chip->chip_priv->diseqc_op_status ==
		    avl62x1_dos_continuous)
		{
			p_debug("call avl62x1_diseqc_tone_off()");
			r = (int)avl62x1_diseqc_tone_off(priv->chip);
		}
		break;
	default:
		r = -EINVAL;
	}

	if(r != AVL_EC_OK) {
		p_debug("diseqc_set_tone() FAILURE!");
	}

	safe_mutex_unlock(&i2cctl_fe_mutex);

	return r;
}

static int diseqc_set_voltage(struct dvb_frontend *fe,
			      enum fe_sec_voltage voltage)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	avl62x1_gpio_pin_value enable, sel;
	int ret;

	p_debug("volt: %d", voltage);

	mutex_lock(&i2cctl_fe_mutex);

	switch (voltage)
	{
	case SEC_VOLTAGE_OFF:
		enable = avl62x1_gpio_value_logic_0;
		sel = avl62x1_gpio_value_logic_0;
		break;
	case SEC_VOLTAGE_13:
		//power on
		enable = avl62x1_gpio_value_logic_1;
		sel = avl62x1_gpio_value_logic_0;
		break;
	case SEC_VOLTAGE_18:
		//power on
		enable = avl62x1_gpio_value_logic_1;
		sel = avl62x1_gpio_value_high_z;
		break;
	default:
		safe_mutex_unlock(&i2cctl_fe_mutex);
		return -EINVAL;
	}

	p_debug("lnb_pwr_en %d, lnb_pwr_sel %d",enable,sel);

	ret = (int)avl62x1_set_gpio_value(avl62x1_gpio_pin_lnb_pwr_en,
					  enable,
					  priv->chip);
	ret |= (int)avl62x1_set_gpio_value(avl62x1_gpio_pin_lnb_pwr_sel,
					   sel,
					   priv->chip);

	if(ret != AVL_EC_OK) {
		p_debug("diseqc_set_voltage() FAILURE!");
	}

	safe_mutex_unlock(&i2cctl_fe_mutex);

	return ret;
}

static int update_fe_props(
    struct dtv_frontend_properties *props,
    struct avl62x1_carrier_info *carrier_info,
    struct avl62x1_stream_info *stream_info)
{
	uint16_t r = AVL_EC_OK;
	props->frequency = carrier_info->rf_freq_khz +
			   carrier_info->carrier_freq_offset_hz/1000;

	switch (carrier_info->modulation)
	{
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
	    (carrier_info->spectrum_invert == avl62x1_specpol_inverted)
		? INVERSION_ON
		: INVERSION_OFF;

	props->symbol_rate = carrier_info->symbol_rate_hz;

	if (carrier_info->pilot == avl62x1_pilot_on)
	{
		props->pilot = PILOT_ON;
	}
	else
	{
		props->pilot = PILOT_OFF;
	}

	switch (carrier_info->roll_off)
	{
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

	if (carrier_info->signal_type == avl62x1_dvbs2)
	{
		props->delivery_system = SYS_DVBS2;
		switch (carrier_info->code_rate.dvbs2_code_rate)
		{
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
	}
	else
	{
		props->delivery_system = SYS_DVBS;
		switch (carrier_info->code_rate.dvbs_code_rate)
		{
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

	props->stream_id = stream_info->isi;

	return r;
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
	int8_t demod_id =
	    (priv->chip->chip_pub->i2c_addr >> AVL_DEMOD_ID_SHIFT) &
	    AVL_DEMOD_ID_MASK;

	p_debug_lvl(10,"");

	if(bs_states[demod_id].bs_mode) {
		return ret;
	}

	mutex_lock(&i2cctl_fe_mutex);

	ret = avl62x1_get_lock_status(&lock,
				      priv->chip);

	if ((lock == avl62x1_status_locked) &&
	    (ret == AVL_EC_OK))
	{

		ret |= avl62x1_get_signal_info(&carrier_info,
					priv->chip);
		
		
		ret |= avl62x1_get_stream_info(&stream_info,
					priv->chip);

		ret |= update_fe_props(props, &carrier_info, &stream_info);

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

	safe_mutex_unlock(&i2cctl_fe_mutex);

	return ret;
}

static int read_status(struct dvb_frontend *fe, enum fe_status *status)
{
	int r = AVL_EC_OK;
	struct avl62x1_priv *priv = fe->demodulator_priv;
	avl62x1_lock_status lock;
	int8_t demod_id =
	    (priv->chip->chip_pub->i2c_addr >> AVL_DEMOD_ID_SHIFT) &
	    AVL_DEMOD_ID_MASK;

	p_debug_lvl(10, "");

	if (bs_states[demod_id].bs_mode)
	{
		if (bs_states[demod_id].info.finished)
			*status = FE_HAS_LOCK;
		else
			*status = FE_NONE;
		return AVL_EC_OK;
	}

	mutex_lock(&i2cctl_fe_mutex);

	r = avl62x1_get_lock_status(&lock, priv->chip);

	safe_mutex_unlock(&i2cctl_fe_mutex);

	if (!r && lock == avl62x1_status_locked)
	{
		*status = FE_HAS_SIGNAL | FE_HAS_CARRIER |
			  FE_HAS_VITERBI | FE_HAS_SYNC |
			  FE_HAS_LOCK;
		r = get_frontend(fe,
				 &fe->dtv_property_cache);
	}
	else
	{
		*status = FE_HAS_SIGNAL;
	}

	return r;
}

static int read_signal_strength(struct dvb_frontend *fe, uint16_t *strength)
{
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int i;

	//p_debug("ENTER");

	*strength = 0;
	for (i = 0; i < c->strength.len; i++)
		if (c->strength.stat[i].scale == FE_SCALE_RELATIVE)
			*strength = (uint16_t)c->strength.stat[i].uvalue;

	return 0;
}

static int read_snr(struct dvb_frontend *fe, uint16_t *snr)
{
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int i;

	//p_debug("ENTER");

	*snr = 0;
	for (i = 0; i < c->cnr.len; i++)
		if (c->cnr.stat[i].scale == FE_SCALE_RELATIVE)
			*snr = (uint16_t)c->cnr.stat[i].uvalue;

	return 0;
}
static int read_ber(struct dvb_frontend *fe, uint32_t *ber)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	int ret;

	p_debug_lvl(10,"");

	mutex_lock(&i2cctl_fe_mutex);
	
	//FIXME
	*ber = 10e7;
	ret = (int)avl62x1_get_per(ber, priv->chip);
	if (!ret)
		*ber /= 100;

	safe_mutex_unlock(&i2cctl_fe_mutex);

	return ret;
}

static enum dvbfe_algo get_frontend_algo(struct dvb_frontend *fe)
{
	return DVBFE_ALGO_HW;
}

static int blindscan_get_stream_list(struct dvb_frontend *fe)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	uint16_t r = AVL_EC_OK;
	uint8_t demod_id =
	    (priv->chip->chip_pub->i2c_addr >> AVL_DEMOD_ID_SHIFT) &
	    AVL_DEMOD_ID_MASK;
	struct avl62x1_bs_state *state = &(bs_states[demod_id]);
	uint8_t num_streams;
	uint8_t t2mi_add_str;
	uint8_t s,p,s1;
	struct avl62x1_stream_info *tmp_strs;
	

	p_debug("ENTER");

	r = avl62x1_get_num_streams(&num_streams, priv->chip);
	
	if(state->streams != NULL)
		kfree(state->streams);
	state->streams = kzalloc(
		num_streams * sizeof(struct avl62x1_stream_info),
		GFP_KERNEL);
	if(!state->streams)
	{
		p_error("alloc failed");
		return AVL_EC_MemoryRunout;
	}

	state->carriers[state->cur_carrier].num_streams = num_streams;

	r |= avl62x1_get_stream_list(state->streams,num_streams,priv->chip);

	//we're going to expand any non-unary PLP lists into
	//  additional streams with unary PLP lists.
	//so first count how many additional streams we need to add
	t2mi_add_str = 0;
	for (s = 0; s < num_streams; s++)
	{
		printk("ISI %d, Stream type %d\n",
		       state->streams[s].isi,
		       state->streams[s].stream_type);
		if (state->streams[s].stream_type == avl62x1_t2mi)
		{
			printk("T2MI PID 0x%x\n",
			       state->streams[s].t2mi.pid);
			for (p = 0; p < state->streams[s].t2mi.plp_list.list_size; p++)
			{
				printk("PLP %d  ID %d\n",
				       p,
				       state->streams[s].t2mi.plp_list.list[p]);
			}
			if (state->streams[s].t2mi.plp_list.list_size > 0)
			{
				t2mi_add_str +=
				    (state->streams[s].t2mi.plp_list.list_size - 1);
			}
		}
	}
	p_debug("Adding %d streams for T2MI PLP's",t2mi_add_str);

	//realloc with new number of streams
	tmp_strs = kzalloc(
	    (num_streams + t2mi_add_str) * sizeof(struct avl62x1_stream_info),
	    GFP_KERNEL);
	if(!tmp_strs)
	{
		return AVL_EC_MemoryRunout;
	}

	//copy original stream set to new buffer
	memcpy(tmp_strs,
	       state->streams,
	       num_streams * sizeof(struct avl62x1_stream_info));

	//free original
	kfree(state->streams);

	state->streams = tmp_strs;
	
	//now go back thru streams and expand non-unary PLP lists
	s1 = num_streams; //first new stream entry
	for (s = 0; s < num_streams; s++)
	{
		if ((state->streams[s].stream_type == avl62x1_t2mi) &&
		    (state->streams[s].t2mi.plp_list.list_size > 1))
		{
			for (p = 1;
			     p < state->streams[s].t2mi.plp_list.list_size;
			     p++, s1++)
			{
				memcpy(&state->streams[s1],
				       &state->streams[s],
				       sizeof(struct avl62x1_stream_info));

				state->streams[s1].t2mi.plp_list.list_size = 1;
				state->streams[s1].t2mi.plp_list.list[0] =
				    state->streams[s].t2mi.plp_list.list[p];
				p_debug("expanded PLP ID %d",
					state->streams[s1].t2mi.plp_list.list[0]);
			}
		}
	}

	//update number of streams
	num_streams += t2mi_add_str;
	state->carriers[state->cur_carrier].num_streams = num_streams;

	p_debug("got %d streams",num_streams);
	p_debug("EXIT");

	return r;
}

static int blindscan_confirm_carrier(struct dvb_frontend *fe)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	uint16_t r = AVL_EC_OK;
	uint8_t demod_id =
	    (priv->chip->chip_pub->i2c_addr >> AVL_DEMOD_ID_SHIFT) &
	    AVL_DEMOD_ID_MASK;
	struct avl62x1_bs_state *state = &(bs_states[demod_id]);
	uint16_t cntr;
	enum avl62x1_discovery_status status;
	const uint16_t timeout = 20;
	const uint32_t delay = 100;

	p_debug("ENTER");
	p_debug("confirming carrier @ %d kHz...",
		state->carriers[state->cur_carrier].rf_freq_khz);
	
	r = avl62x1_blindscan_confirm_carrier(
	    &state->params,
	    &state->carriers[state->cur_carrier],
	    priv->chip);

	status = avl62x1_discovery_running;
	cntr = 0;
	do
	{
		p_debug("CC %dms",cntr*delay);
		r |= avl62x1_get_discovery_status(
		    &status,
		    priv->chip);
		avl_bsp_delay(delay);
		cntr++;

	} while ((status == avl62x1_discovery_running) &&
		 (cntr < timeout) && (r == AVL_EC_OK));

	if ((cntr >= timeout) || (r != AVL_EC_OK))
	{
		p_debug("confirm carrier timed out");
		p_debug("EXIT");
		return AVL_EC_TimeOut;
	}
	else
	{
		p_debug("carrier confirmed");
		p_debug("EXIT");
		return r;
	}
}

static int blindscan_get_next_stream(struct dvb_frontend *fe)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	uint16_t r = AVL_EC_OK;
	uint8_t demod_id =
	    (priv->chip->chip_pub->i2c_addr >> AVL_DEMOD_ID_SHIFT) &
	    AVL_DEMOD_ID_MASK;
	struct avl62x1_bs_state *state = &(bs_states[demod_id]);
	struct avl62x1_carrier_info *carrier;
	struct avl62x1_stream_info *stream;

	p_debug("ENTER");

	carrier = &state->carriers[state->cur_carrier];
	stream = NULL;

	//mark stream as invalid in case none of the carriers
	//  can be confirmed
	c->AVL62X1_BS_CTRL_PROP = 0;

	//get next stream
	//if at end of current stream list, go get another
	//  from the next carrier, if there is one.
	while (state->cur_carrier < state->info.num_carriers)
	{
		p_debug("cur_carrier %d",state->cur_carrier);
		carrier = &state->carriers[state->cur_carrier];
		stream = NULL;

		if (state->cur_stream == 0)
		{
			//new carrier, so confirm it first
			if (blindscan_confirm_carrier(fe) ==
			    AVL_EC_OK)
			{
				//carrier confirmed
				//get stream list
				r |= blindscan_get_stream_list(fe);
			}
			else
			{
				//not confirmed
				//go to next carrier
				state->cur_stream = 0;
				carrier->num_streams = 0;
				p_debug("carrier not confirmed");
				state->cur_carrier++;
				p_debug("next carrier %d", state->cur_carrier);
				continue;
			}
		}

		p_debug("carrier->num_streams %d",carrier->num_streams);

		while ((state->cur_stream < carrier->num_streams) &&
		       (stream == NULL))
		{
			p_debug("cur_stream %d",state->cur_stream);
			stream = &state->streams[state->cur_stream];

			//put current stream info into DVB props
			r |= update_fe_props(c, carrier, stream);

			//set over-loaded stream_id
			if (stream->stream_type == avl62x1_t2mi)
			{
				c->stream_id |= (1 << AVL62X1_BS_IS_T2MI_SHIFT);
				c->stream_id |=
				    (stream->t2mi.pid & 0x1FFF) <<
					AVL62X1_BS_T2MI_PID_SHIFT;
				c->stream_id |=
				    (stream->t2mi.plp_list.list[0] & 0xFF) <<
					AVL62X1_BS_T2MI_PLP_ID_SHIFT;
			}

			//mark stream as valid
			c->AVL62X1_BS_CTRL_PROP |=
			    AVL62X1_BS_CTRL_VALID_STREAM_MASK;

			state->cur_stream++;
			p_debug("next stream %d", state->cur_stream);

		}

		if(stream != NULL)
			break;

		state->cur_carrier++;
		p_debug("next carrier %d", state->cur_carrier);

	}

	if ((state->cur_stream >= carrier->num_streams) &&
	    (state->cur_carrier >= (state->info.num_carriers-1)))
	{
		//no more streams. signal back the tuner step
		c->AVL62X1_BS_CTRL_PROP |= state->info.next_freq_step_hz/1000;
		p_debug("no more streams. step tuner by %d",
			c->AVL62X1_BS_CTRL_PROP);
	}
	else
	{
		//there are more streams
		c->AVL62X1_BS_CTRL_PROP |= AVL62X1_BS_CTRL_MORE_RESULTS_MASK;
	}

	p_debug("EXIT");

	return r;
}

static int blindscan_step(struct dvb_frontend *fe)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	uint16_t r = AVL_EC_OK;
	uint8_t demod_id =
	    (priv->chip->chip_pub->i2c_addr >> AVL_DEMOD_ID_SHIFT) &
	    AVL_DEMOD_ID_MASK;
	struct avl62x1_bs_state *state = &(bs_states[demod_id]);
	uint16_t cntr;
	const uint16_t timeout = 50;
	const uint32_t delay = 100;

	p_debug("ENTER");

	p_debug("BS CTRL %d",c->AVL62X1_BS_CTRL_PROP);
	
	if(c->AVL62X1_BS_CTRL_PROP & AVL62X1_BS_CTRL_NEW_TUNE_MASK) {
		//allow tuner time to settle
		avl_bsp_delay(250);

		//new frequency was tuned, so run a new carrier search
		state->params.tuner_center_freq_100khz = c->frequency / 100;
		//state->params.tuner_lpf_100khz = bs_tuner_bw / 100000;
		state->params.tuner_lpf_100khz = ((c->symbol_rate * 135) / 100) / 100000;
		state->params.min_symrate_khz = bs_min_sr / 1000;

		p_debug("NEW TUNE: start carrier search @%d kHz, LPF %d MHz",
			c->frequency,
			state->params.tuner_lpf_100khz/10);

		state->num_carriers = 0;

		r = avl62x1_blindscan_start(&state->params, priv->chip);
		
		state->info.finished = 0;
		cntr = 0;
		do
		{
			avl_bsp_delay(delay);
			r = avl62x1_blindscan_get_status(&state->info,
							 priv->chip);
			p_debug("CS %dms",cntr*delay);
			cntr++;

		} while (!state->info.finished &&
			 (cntr < timeout) && (r == AVL_EC_OK));

		if ((cntr >= timeout) || (r != AVL_EC_OK))
		{
			p_debug("carrier search timeout");
		}

		p_debug("carrier search found %d carriers",
			state->info.num_carriers);

		if(state->info.num_carriers > 0) {
			//at least one carrier detected, get carrier list
			if(state->carriers != NULL)
				kfree(state->carriers);
			
			state->carriers = kzalloc(
			    state->info.num_carriers *
				sizeof(struct avl62x1_carrier_info),
			    GFP_KERNEL);

			r = avl62x1_blindscan_get_carrier_list(
			    &state->params,
			    &state->info,
			    state->carriers,
			    priv->chip);
			
			state->cur_carrier = 0;
			state->cur_stream = 0;
			r = blindscan_get_next_stream(fe);
		} else {
			//no carriers detected
			//signal back the tuner step
			c->AVL62X1_BS_CTRL_PROP = state->info.next_freq_step_hz;
		}
	} else {
		p_debug("OLD TUNE");
		r = blindscan_get_next_stream(fe);
	}

	p_debug("EXIT %d",r);

	return r;
}

static int set_frontend(struct dvb_frontend *fe)
{
	int ret;
	struct avl62x1_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	uint8_t demod_id =
	    (priv->chip->chip_pub->i2c_addr >> AVL_DEMOD_ID_SHIFT) &
	    AVL_DEMOD_ID_MASK;

	p_debug("");

	/* tune tuner if necessary*/
	if (fe->ops.tuner_ops.set_params &&
	    ((bs_states[demod_id].bs_mode &&
	      (c->AVL62X1_BS_CTRL_PROP & AVL62X1_BS_CTRL_NEW_TUNE_MASK)) ||
	     !bs_states[demod_id].bs_mode))
	{
		
		ret = fe->ops.i2c_gate_ctrl(fe, 1);
		if (ret)
		{
			p_debug("I2C gate control FAILED\n");
			return ret;
		}

		if(fe->ops.tuner_ops.set_params)
		{
			p_debug("calling tuner_ops.set_params()\n");
			mutex_lock(&i2cctl_fe_mutex);
			ret = fe->ops.tuner_ops.set_params(fe);
			safe_mutex_unlock(&i2cctl_fe_mutex);
			if (ret)
			{
				p_debug("Tuning FAILED\n");
				return ret;
			}
			else
			{
				p_debug("Tuned to %d kHz", c->frequency);
			}
		}

		ret = fe->ops.i2c_gate_ctrl(fe, 0);
		if (ret)
		{
			p_debug("I2C gate control FAILED\n");
			return ret;
		}
		
	}

	mutex_lock(&i2cctl_fe_mutex);
	
	if(bs_states[demod_id].bs_mode) {
		p_debug("BS STEP");
		ret = blindscan_step(fe);
	} else {
		p_debug("ACQUIRE");
		ret = acquire_dvbs_s2(fe);
	}

	safe_mutex_unlock(&i2cctl_fe_mutex);

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
	p_debug("");

	//TODO: call tuner resume, then init
	return 0;
}

static int sleep_fe(struct dvb_frontend *fe)
{
	p_debug("");

	//TODO: call tuner suspend, then sleep

	return 0;
}

static void release_fe(struct dvb_frontend *fe)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;

	p_debug("");
#if INCLUDE_STDOUT
	deinit_avl62x1_stdout_device(priv);
#endif
	deinit_avl62x1_i2cctl_device(priv);
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
    .tune = tune, //overrides the default swzigzag
    .set_frontend = set_frontend,
    .get_frontend = get_frontend,
};

static int avl62x1_get_firmware(struct dvb_frontend *fe) {
	unsigned int fw_maj, fw_min, fw_build;
	int fw_status;
	struct avl62x1_priv *priv = fe->demodulator_priv;

	fw_status = request_firmware(&priv->fw,
				     sel_fw,
				     priv->i2c->dev.parent);
	if (fw_status != 0)
	{
		p_error("firmware file %s not found",sel_fw);
		return fw_status;
	}

	priv->chip->chip_priv->patch_data = (unsigned char *)(priv->fw->data);
	fw_maj = priv->chip->chip_priv->patch_data[24]; //major rev
	fw_min = priv->chip->chip_priv->patch_data[25]; //SDK-FW API rev
	fw_build = (priv->chip->chip_priv->patch_data[26] << 8) |
			priv->chip->chip_priv->patch_data[27]; //internal rev
	if(fw_min != AVL62X1_VER_MINOR)
	{
		//SDK-FW API rev must match
		p_error("Firmware version %d.%d.%d incompatible with this driver version",
			fw_maj, fw_min, fw_build);
		p_error("Firmware minor version must be %d",
			AVL62X1_VER_MINOR);
		release_firmware(priv->fw);
		return 1;
	}
	else
	{
		p_info("Firmware version %d.%d.%d found",
				fw_maj, fw_min, fw_build);
	}

	return 0;
}

struct dvb_frontend *avl62x1_attach(struct avl62x1_config *config,
				    struct i2c_adapter *i2c)
{
	struct avl62x1_priv *priv;
	uint16_t ret;
	uint32_t id;

	p_info("driver version %s\n", AVL62X1_VERSION);

	p_debug("enter %s()", __FUNCTION__);

	priv = kzalloc(sizeof(struct avl62x1_priv), GFP_KERNEL);
	if (priv == NULL)
		goto err;
	global_priv = priv;

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
	priv->delivery_system = SYS_UNDEFINED;

	/* copy (ephemeral?) public part of chip config into alloc'd area */
	memcpy(priv->chip->chip_pub,
	       config->chip_pub,
	       sizeof(struct avl62x1_chip_pub));

	priv->chip->chip_pub->tuner = &default_avl_tuner;

	p_debug("Demod ID %d, I2C addr 0x%x",
		(priv->chip->chip_pub->i2c_addr >> AVL_DEMOD_ID_SHIFT) &
		    AVL_DEMOD_ID_MASK,
		priv->chip->chip_pub->i2c_addr & 0xFF);

	// associate demod ID with i2c_adapter
	avl_bsp_assoc_i2c_adapter(priv->chip->chip_pub->i2c_addr, i2c);

	//set up semaphores
	ret = avl62x1_init_chip_object(priv->chip);
	if (ret)
	{
		p_error("chip object init failed");
		goto err4;
	}

	/* get chip id */
	ret = avl62x1_get_chip_id(priv->chip->chip_pub->i2c_addr, &id);
	if (ret)
	{
		p_error("attach failed reading id");
		goto err4;
	}

	p_debug("chip_id 0x%x\n", id);

	if (id != AVL62X1_CHIP_ID)
	{
		p_error("attach failed, id mismatch");
		goto err4;
	}

	p_info("found AVL62x1 id=0x%x", id);

	if(!avl62x1_get_firmware(&priv->frontend)) {
		if(!set_dvb_mode(&priv->frontend, SYS_DVBS2, false))
		{
			p_info("Firmware booted");
			release_firmware(priv->fw);
			init_avl62x1_i2cctl_device(priv);
#if INCLUDE_STDOUT
			init_avl62x1_stdout_device(priv);
#endif
			
			return &priv->frontend;
		}
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
} /* end avl62x1_attach() */
EXPORT_SYMBOL_GPL(avl62x1_attach);

static int __init mod_init(void) {
	uint8_t i;

	p_debug("");

	global_priv = NULL;

	for(i=0; i<AVL_MAX_NUM_DEMODS; i++) {
		bs_states[i].bs_mode = (bs_mode>>i) & 1;
		bs_states[i].num_carriers = 0;
		bs_states[i].cur_carrier = 0;
		bs_states[i].cur_stream = 0;
		bs_states[i].carriers = NULL;
		bs_states[i].streams = NULL;
	}

	if(strlen(sel_fw) == 0) {
		strlcpy(sel_fw, AVL62X1_FIRMWARE, sizeof(sel_fw));
	}

	return 0;
}
module_init(mod_init);

static void __exit mod_exit(void) {
	uint8_t i;
	for(i=0; i<AVL_MAX_NUM_DEMODS; i++) {
		if(bs_states[i].carriers != NULL)
			kfree(bs_states[i].carriers);
		
		if(bs_states[i].streams != NULL)
			kfree(bs_states[i].streams);

	}
}
module_exit(mod_exit);

module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, " 1: enable debug messages");


static int bs_mode_set(const char *val, const struct kernel_param *kp)
{
	int n = 0, ret, i;
	ret = kstrtoint(val, 0, &n);
	for(i=0; i<AVL_MAX_NUM_DEMODS; i++) {
		bs_states[i].bs_mode = (n>>i) & 1;
	}
	printk("bs_mode = 0x%x\n",n);
	return param_set_int(val, kp);
}
static int bs_mode_get(char *buffer, const struct kernel_param *kp)
{
	sprintf(buffer,"0x%.4x",bs_mode);
	return strlen(buffer);
}
static const struct kernel_param_ops bs_mode_ops = {
	.set	= bs_mode_set,
	.get	= bs_mode_get
};
module_param_cb(bs_mode, &bs_mode_ops, &bs_mode, 0644);
MODULE_PARM_DESC(bs_mode, " 16 bit encoding [15:0], one per demod. 1: operate in blindscan mode, 0: normal DVB acquisition mode");



static int bs_min_sr_set(const char *val, const struct kernel_param *kp)
{
	int n = 0, ret;
	ret = kstrtoint(val, 10, &n);
	if (ret != 0 || n < 1000000 || n > 55000000)
		return -EINVAL;
	return param_set_int(val, kp);
}
static const struct kernel_param_ops bs_min_sr_ops = {
	.set	= bs_min_sr_set,
	.get	= param_get_int
};
module_param_cb(bs_min_sr, &bs_min_sr_ops, &bs_min_sr, 0644);
MODULE_PARM_DESC(bs_min_sr, " minimum symbol rate (Hz) for blindscan mode [1000000:55000000]");

static int fw_path_set(const char *val, const struct kernel_param *kp)
{
	char *clean_val = strim((char *)val);
	strlcpy(sel_fw, clean_val, sizeof(sel_fw));
	strlcpy(fw_path, clean_val, sizeof(fw_path));

	if (global_priv != NULL)
	{
		if (!avl62x1_get_firmware(&global_priv->frontend))
		{
			if (!set_dvb_mode(&global_priv->frontend, SYS_DVBS2, true))
			{
				p_info("New firmware %s booted",sel_fw);
				release_firmware(global_priv->fw);
			}
		}
	}

	return 0;
}
static int fw_path_get(char *buffer, const struct kernel_param *kp)
{
	sprintf(buffer, "%s",sel_fw);
	return strlen(buffer);
}
static const struct kernel_param_ops fw_path_ops = {
	.set	= fw_path_set,
	.get	= fw_path_get
};
module_param_cb(fw_path, &fw_path_ops, fw_path, 0644);
MODULE_PARM_DESC(fw_path, "<path>");


MODULE_DESCRIPTION("Availink AVL62X1 DVB-S/S2/S2X demodulator driver");
MODULE_AUTHOR("Availink, Inc. (gpl@availink.com)");
MODULE_LICENSE("GPL");
MODULE_VERSION(AVL62X1_VERSION);
MODULE_FIRMWARE(AVL62X1_FIRMWARE);

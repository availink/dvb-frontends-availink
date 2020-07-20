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
#include <linux/gpio.h>
//#include <linux/amlogic/aml_gpio_consumer.h>

#include <media/dvb_frontend.h>

#include "avl68x2.h"
#include "avl_tuner.h"

#include "avl68x2_common.h"
#include "avl68x2_dvbsx.h"
#include "avl68x2_dvbtx.h"
#include "avl68x2_dvbc.h"
#include "avl68x2_isdbt.h"

#define INCLUDE_STDOUT	0


#define p_debug(fmt, args...)					\
	do							\
	{							\
		if (debug)					\
			printk("avl68x2:%s DEBUG: " fmt "\n",	\
				__func__, ##args);		\
	} while (0);

#define p_debug_lvl(lvl,fmt, args...)				\
	do							\
	{							\
		if (debug >= lvl)				\
			printk("avl68x2:%s DEBUG: " fmt "\n",	\
				__func__, ##args);		\
	} while (0);

#define p_error(fmt, args...)					\
	do							\
	{							\
		printk("avl68x2:%s ERROR: " fmt "\n",		\
			__func__, ##args);			\
	} while (0);

#define p_info(fmt, args...)					\
	do							\
	{							\
		printk("avl68x2:%s INFO: " fmt "\n",		\
			__func__, ##args);			\
	} while (0);

#define p_warn(fmt, args...)					\
	do							\
	{							\
		printk("avl68x2:%s WARN: " fmt "\n",		\
			__func__, ##args);			\
	} while (0);

#define safe_mutex_unlock(m)					\
	if(mutex_is_locked(m)) {				\
		mutex_unlock(m);				\
	} else {						\
		p_warn("Tried to unlock mutex that wasn't locked"); \
	}

static struct avl68x2_bs_state bs_states[AVL_MAX_NUM_DEMODS] = {0};
static struct avl68x2_priv *global_priv = NULL; //just for debug
static struct dvb_frontend_ops avl68x2_ops;
static char sel_dvbsx_fw[256] = {0};
static char sel_dvbtx_fw[256] = {0};
static char sel_dvbc_fw[256] = {0};
static char sel_isdbt_fw[256] = {0};

//------ module parameters --------
int debug = 0;
int cable_auto_symrate = 1;
int cable_auto_cfo = 1;
unsigned short bs_mode = 0;
int bs_min_sr = 1000000;
int diseqc_tone = 0;
int diseqc_voltage = 0;
char fw_paths[1024] = {0};
//---------------------------------

//--------- i2c control device ---------------------------------------------
//---- character device stuff ----
#define I2CCTL_DEVICE_NAME "avl68x2_i2c" //device at /dev/DEVICE_NAME
#define I2CCTL_CLASS_NAME  "avl68x2_i2c"     //device class
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

int init_avl68x2_i2cctl_device(struct avl68x2_priv *priv)
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

void deinit_avl68x2_i2cctl_device(struct avl68x2_priv *priv)
{
	device_destroy(i2cctl_class, MKDEV(i2cctl_maj_num, 0)); // remove the device
	class_unregister(i2cctl_class);			     // unregister the device class
	class_destroy(i2cctl_class);			     // remove the device class
	unregister_chrdev(i2cctl_maj_num, I2CCTL_DEVICE_NAME);	     // unregister the major number

	safe_mutex_unlock(&i2cctl_fe_mutex);
	safe_mutex_unlock(&i2cctl_tuneri2c_mutex);

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
		if (req->demod)
		{
			mutex_lock(&i2cctl_fe_mutex);
			p_debug_lvl(4, "locked demod");
		}
		if (req->tuner)
		{
			mutex_lock(&i2cctl_tuneri2c_mutex);
			p_debug_lvl(4, "locked tuner");
		}
		break;
	case I2CCTL_IOCUNLOCK:
		if (req->demod)
		{
			if (mutex_is_locked(&i2cctl_fe_mutex))
			{
				mutex_unlock(&i2cctl_fe_mutex);
				p_debug_lvl(4, "unlocked demod");
			}
		}
		if (req->tuner)
		{
			if (mutex_is_locked(&i2cctl_tuneri2c_mutex))
			{
				mutex_unlock(&i2cctl_tuneri2c_mutex);
				p_debug_lvl(4, "unlocked tuner");
			}
		}
		break;
	default: /* redundant, as cmd was checked against MAXNR */
		p_error("Unknown IOCTL %d", cmd);
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
	safe_mutex_unlock(&i2cctl_fe_mutex);
	safe_mutex_unlock(&i2cctl_tuneri2c_mutex);
	safe_mutex_unlock(&i2cctl_dev_mutex);
	return 0;
}
//--------------------------------------------------------------------------

//-------------- stdout character device -----------------------------------
#if INCLUDE_STDOUT
#include "read_stdout_68x2.c"
#define STDOUT_DEVICE_NAME "avl68x2_stdout"
#define STDOUT_CLASS_NAME "avl68x2_stdout"
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
int init_avl68x2_stdout_device(struct avl68x2_priv *priv)
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

void deinit_avl68x2_stdout_device(struct avl68x2_priv *priv)
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
	stdout_buf = read_stdout(&global_priv->frontend);
	return simple_read_from_buffer(buffer, len, offset, stdout_buf, strlen(stdout_buf));
}

static int stdout_dev_release(struct inode *inodep, struct file *filep)
{
	p_debug_lvl(4,"");
	safe_mutex_unlock(&stdout_dev_mutex);
	return 0;
}
#endif
//--------------------------------------------------------------------------

const AVL_DVBTxConfig default_dvbtx_config =
    {
	.eDVBTxInputPath = AVL_IF_I,
	.uiDVBTxIFFreqHz = 5 * 1000 * 1000,
	.eDVBTxAGCPola = AVL_AGC_NORMAL //AVL_AGC_INVERTED
};

const AVL_DVBSxConfig default_dvbsx_config =
    {
	.eDVBSxAGCPola = AVL_AGC_INVERTED,
	.e22KWaveForm = AVL_DWM_Normal};

const AVL_ISDBTConfig default_isdbt_config =
    {
	.eISDBTInputPath = AVL_IF_I,
	.uiISDBTIFFreqHz = 5 * 1000 * 1000,
	.eISDBTAGCPola = AVL_AGC_NORMAL};

const AVL_DVBCConfig default_dvbc_config =
    {
	.eDVBCInputPath = AVL_IF_I,
	.uiDVBCIFFreqHz = 5 * 1000 * 1000,
	.eDVBCAGCPola = AVL_AGC_NORMAL};

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

void avl68x2_reset(int gpio, int i)
{
	if(gpio < 0)
		return;
	if(!gpio_is_valid(gpio))
		return;
	
	gpio_direction_output(gpio, i);
	msleep(600);
	gpio_direction_output(gpio, 1 - i);
	msleep(200);
}

void avl68x2_set_lock_led(struct dvb_frontend *fe, int val)
{
	struct avl68x2_priv *priv = fe->demodulator_priv;
	int lock_led = priv->chip->chip_pub->gpio_lock_led;
	if(lock_led < 0)
		return;
	if(gpio_is_valid(lock_led))
	{
		gpio_direction_output(lock_led, val);
	}
	else
	{
		p_debug_lvl(3,"invalid lock LED GPIO %d", lock_led);
	}
}

static int diseqc_set_voltage(
    struct dvb_frontend *fe,
    enum fe_sec_voltage voltage)
{
	struct avl68x2_priv *priv = fe->demodulator_priv;
	AVL_GPIOPinValue pwr, vol;
	avl_error_code_t ret;

	p_debug("volt: %d", voltage);

	mutex_lock(&i2cctl_fe_mutex);

	switch (voltage)
	{
	case SEC_VOLTAGE_OFF:
		pwr = AVL_LOGIC_0;
		vol = AVL_LOGIC_0;
		p_debug("voltage OFF");
		break;
	case SEC_VOLTAGE_13:
		//power on
		pwr = AVL_LOGIC_1;
		vol = AVL_LOGIC_0;
		p_debug("voltage 13");
		break;
	case SEC_VOLTAGE_18:
		//power on
		pwr = AVL_LOGIC_1;
		vol = AVL_HIGH_Z;
		p_debug("voltage 18");
		break;
	default:
		safe_mutex_unlock(&i2cctl_fe_mutex);
		return -EINVAL;
	}
	ret = avl68x2_demod_set_gpio(AVL_Pin37, pwr, priv->chip);
	ret |= avl68x2_demod_set_gpio(AVL_Pin38, vol, priv->chip);
	safe_mutex_unlock(&i2cctl_fe_mutex);
	return ret;
}

static int avl68x2_i2c_gate_ctrl(struct dvb_frontend *fe, int enable)
{
	struct avl68x2_priv *priv = fe->demodulator_priv;
	avl_error_code_t ret = AVL_EC_OK;

	p_debug("%d\n", enable);

	mutex_lock(&i2cctl_fe_mutex);

	if (enable)
	{
		mutex_lock(&i2cctl_tuneri2c_mutex);
		ret = avl68x2_demod_i2c_passthru_on(priv->chip);
	}
	else
	{
		ret = avl68x2_demod_i2c_passthru_off(priv->chip);
		safe_mutex_unlock(&i2cctl_tuneri2c_mutex);
	}
	safe_mutex_unlock(&i2cctl_fe_mutex);
	return ret;
}

static int avl68x2_acquire_dvbsx(struct dvb_frontend *fe)
{
  struct avl68x2_priv *priv = fe->demodulator_priv;
  struct dtv_frontend_properties *c = &fe->dtv_property_cache;
  avl_error_code_t r = AVL_EC_OK;
  p_debug("ACQUIRE S/S2");
  p_debug("Freq:%d khz,sym:%d hz", c->frequency, c->symbol_rate);

  r = AVL_Demod_DVBSxAutoLock(
      c->symbol_rate,
      priv->chip);

  return r;
}

static int avl68x2_acquire_dvbtx(struct dvb_frontend *fe)
{
	struct avl68x2_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	avl_error_code_t r = AVL_EC_OK;
	AVL_DVBTxBandWidth bw;

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

	if (c->delivery_system == SYS_DVBT)
	{
		p_debug("ACQUIRE T ONLY");
		r = AVL_Demod_DVBTAutoLock(bw, 0, priv->chip);
	}
	else
	{
		p_debug("ACQUIRE T/T2");
		r = AVL_Demod_DVBT2AutoLock(bw,
					    AVL_DVBT2_PROFILE_UNKNOWN,
					    c->stream_id,
					    priv->chip);
	}

	return r;
}

static int avl68x2_acquire_dvbc(struct dvb_frontend *fe)
{
  avl_error_code_t r = AVL_EC_OK;
  struct avl68x2_priv *priv = fe->demodulator_priv;
  struct dtv_frontend_properties *c = &fe->dtv_property_cache;
  p_debug("ACQUIRE DVB-C");
  r |= AVL_Demod_DVBCAutoLock(AVL_DVBC_J83A, c->symbol_rate, priv->chip);
  return r;
}

static int avl68x2_acquire_dvbc_b(struct dvb_frontend *fe)
{
  avl_error_code_t r = AVL_EC_OK;
  struct avl68x2_priv *priv = fe->demodulator_priv;
  struct dtv_frontend_properties *c = &fe->dtv_property_cache;
  p_debug("ACQUIRE J.83B");
  r |= AVL_Demod_DVBCAutoLock(AVL_DVBC_J83B, c->symbol_rate, priv->chip);
  return r;
}

static int avl68x2_acquire_isdbt(struct dvb_frontend *fe)
{
	avl_error_code_t r = AVL_EC_OK;
	struct avl68x2_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	AVL_ISDBT_BandWidth bw;
	p_debug("ACQUIRE ISDB-T");

	if (c->bandwidth_hz <= 6000000)
	{
		bw = AVL_ISDBT_BW_6M;
	}
	else
	{
		bw = AVL_ISDBT_BW_8M;
	}
	r = AVL_Demod_ISDBTAutoLock(bw, priv->chip);
	return r;
}

static int avl68x2_get_firmware(struct dvb_frontend *fe, int force_fw)
{
	struct avl68x2_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int32_t r = AVL_EC_OK;
	int fw_status;
	unsigned int fw_maj, fw_min, fw_build;
	char fw_path[256];
	int sys;

	if(force_fw > 0)
	{
		sys = force_fw;
	}
	else
	{
		sys = c->delivery_system;
	}
	
	
	switch (sys)
	{
	case SYS_DVBS:
	case SYS_DVBS2:
		strlcpy(fw_path, sel_dvbsx_fw, sizeof(fw_path));
		break;
	case SYS_ISDBT:
		strlcpy(fw_path, sel_isdbt_fw, sizeof(fw_path));
		break;
	case SYS_DVBC_ANNEX_A:		  //"DVB-C"
	case SYS_DVBC_ANNEX_B:		  //J.83-B
		strlcpy(fw_path, sel_dvbc_fw, sizeof(fw_path));
		break;
	case SYS_DVBT:
	case SYS_DVBT2:
	default:
		strlcpy(fw_path, sel_dvbtx_fw, sizeof(fw_path));
	}

	fw_status = request_firmware(&priv->fw, fw_path, priv->i2c->dev.parent);
	if (fw_status != 0)
	{
		p_error("firmware %s not found",fw_path);
		return fw_status;
	}
	else
	{
		priv->chip->chip_priv->patch_data = (unsigned char *)(priv->fw->data);
		fw_maj = priv->chip->chip_priv->patch_data[24]; //major rev
		fw_min = priv->chip->chip_priv->patch_data[25]; //FW-driver API rev
		fw_build = (priv->chip->chip_priv->patch_data[26] << 8) |
			   priv->chip->chip_priv->patch_data[27]; //internal rev
		if (fw_min != AVL68X2_VER_MINOR)
		{
			//SDK-FW API rev must match
			p_error("Firmware version %d.%d.%d incompatible with this driver version",
				fw_maj, fw_min, fw_build);
			p_error("Firmware minor version must be %d",
				AVL68X2_VER_MINOR);
			r = 1;
			release_firmware(priv->fw);
		}
		else
		{
			p_info("loaded firmware %s, version %d.%d.%d",
				 fw_path, fw_maj, fw_min, fw_build);
		}
	}

	return r;
}

static int avl68x2_set_standard(
    struct dvb_frontend *fe,
    enum fe_delivery_system force_delsys)
{
	struct avl68x2_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int32_t r = AVL_EC_OK;
	struct AVL_DemodVersion ver_info;
	AVL_DemodMode dmd_mode = AVL_DVBSX;

	r |= GetMode_Demod(&dmd_mode, priv->chip);
	p_debug("in mode %d", (unsigned int)dmd_mode);

	if(force_delsys != SYS_UNDEFINED)
		c->delivery_system = force_delsys;

	//check for (FW) equivalent modes
	switch (c->delivery_system)
	{
	case SYS_DVBS:
	case SYS_DVBS2:
		if(dmd_mode == AVL_DVBSX)
		{
			p_debug("already in requested mode");
			return 0;
		}
		dmd_mode = AVL_DVBSX;
		break;
	case SYS_ISDBT:
		if(dmd_mode == AVL_ISDBT)
		{
			p_debug("already in requested mode");
			return 0;
		}
		dmd_mode = AVL_ISDBT;
		break;
	case SYS_DVBC_ANNEX_A: //"DVB-C"
	case SYS_DVBC_ANNEX_B: //J.83-B
		if(dmd_mode == AVL_DVBC)
		{
			p_debug("already in requested mode");
			return 0;
		}
		dmd_mode = AVL_DVBC;
		break;
	case SYS_DVBT:
	case SYS_DVBT2:
	default:
		if(dmd_mode == AVL_DVBTX)
		{
			p_debug("already in requested mode");
			return 0;
		}
		dmd_mode = AVL_DVBTX;
	}

	p_debug("setting to mode  %d", (unsigned int)dmd_mode);

	r = avl68x2_get_firmware(fe,SYS_UNDEFINED);
	if (r != 0)
	{
		p_error("get firmware failed");
		return r;
	}

	//Reset Demod
	//   r = avl_bsp_reset();
	//   if (AVL_EC_OK != r)
	//   {
	//     p_debug("Failed to Resed demod via BSP!\n");
	//     goto err;
	//   }

	// boot the firmware here
	r |= avl68x2_demod_set_mode(dmd_mode, priv->chip);
	if (AVL_EC_OK != r)
	{
		p_debug("avl68x2_demod_set_mode failed !\n");
		return r;
	}

	r |= avl68x2_demod_get_version(&ver_info, priv->chip);
	if (AVL_EC_OK != r)
	{
		p_debug("avl68x2_demod_get_version failed\n");
		return r;
	}
	p_debug("FW version %d.%d.%d\n", ver_info.firmware.major, ver_info.firmware.minor, ver_info.firmware.build);
	p_debug("Driver version %d.%d.%d\n", ver_info.driver.major, ver_info.driver.minor, ver_info.driver.build);


	if (r)
	{
		p_error("demod init failed");
	}

	release_firmware(priv->fw);

	return r;
}

avl_error_code_t avl68x2_diseqc_cmd_guts(
    struct avl68x2_priv *priv,
    uint8_t *pCmd,
    uint8_t CmdSize)
{
	avl_error_code_t r = AVL_EC_OK;
	struct AVL_Diseqc_TxStatus TxStatus;
	p_debug(" %*ph", CmdSize, pCmd);

	mutex_lock(&i2cctl_fe_mutex);

	r = AVL_Demod_DVBSx_Diseqc_SendModulationData(pCmd, CmdSize, priv->chip);
	if (r != AVL_EC_OK)
	{
		p_error("AVL_Demod_DVBSx_Diseqc_SendModulationData failed");
	}
	else
	{
		do
		{
			msleep(5);
			r |= AVL_Demod_DVBSx_Diseqc_GetTxStatus(&TxStatus,
								priv->chip);
		} while (TxStatus.m_TxDone != 1);
		if (r != AVL_EC_OK)
		{
			p_error("AVL_Demod_DVBSx_Diseqc_GetTxStatus failed");
		}
	}

	safe_mutex_unlock(&i2cctl_fe_mutex);

	return (int)(r);
}

static int avl68x2_diseqc_cmd(struct dvb_frontend *fe,
                          struct dvb_diseqc_master_cmd *cmd)
{
  struct avl68x2_priv *priv = fe->demodulator_priv;

  return avl68x2_diseqc_cmd_guts(priv, cmd->msg, cmd->msg_len);
}

static int avl68x2_burst(struct dvb_frontend *fe, enum fe_sec_mini_cmd burst)
{
	struct avl68x2_priv *priv = fe->demodulator_priv;
	int ret;

	mutex_lock(&i2cctl_fe_mutex);

	ret = AVL_Demod_DVBSx_Diseqc_SendTone(
	    burst == SEC_MINI_A ? 1 : 0,
	    1,
	    priv->chip);

	safe_mutex_unlock(&i2cctl_fe_mutex);

	return ret;
}

static int diseqc_set_tone(struct dvb_frontend *fe, enum fe_sec_tone_mode tone)
{
	struct avl68x2_priv *priv = fe->demodulator_priv;
	struct avl68x2_chip_pub *chip_pub = priv->chip->chip_pub;
	avl_error_code_t r = AVL_EC_OK;

	p_debug("tone: %s", tone==SEC_TONE_ON ? "ON" : "OFF");

	mutex_lock(&i2cctl_fe_mutex);

	r |= avl68x2_set_standard(fe, SYS_DVBS2);
	if(r)
	{
		p_error("couldn't set DVBSx mode");
	}

	switch (tone)
	{
	case SEC_TONE_ON:
		if (chip_pub->eDiseqcStatus != AVL_DOS_InContinuous)
		{
			p_debug("start tone");
			r = AVL_Demod_DVBSx_Diseqc_StartContinuous(priv->chip);
		}
		break;
	case SEC_TONE_OFF:
		if (chip_pub->eDiseqcStatus == AVL_DOS_InContinuous)
		{
			p_debug("stop tone");
			r = AVL_Demod_DVBSx_Diseqc_StopContinuous(priv->chip);
		}
		break;
	default:
		r = -EINVAL;
	}

	safe_mutex_unlock(&i2cctl_fe_mutex);
	return r;
}

static int update_fe_props_sx(struct dvb_frontend *fe,
			   struct dtv_frontend_properties *props)
{
	struct avl68x2_priv *priv = fe->demodulator_priv;
	int ret = 0;
	int32_t cfo = 0;
	struct AVL_DVBSxModulationInfo modinfo;

	props->stream_id = 0;

	ret |= AVL_Demod_DVBSx_GetFreqOffset(&cfo, priv->chip);
	//p_debug("tuner_freq: %d", priv->chip->chip_pub->tuner_freq_hz);
	//p_debug("cfo: %d", cfo);
	props->frequency = (priv->chip->chip_pub->tuner_freq_hz + cfo)/1000;
  
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
	int8_t demod_id =
	    (priv->chip->chip_pub->i2c_addr >> AVL_DEMOD_ID_SHIFT) &
	    AVL_DEMOD_ID_MASK;

	p_debug_lvl(10,"");

	if(bs_states[demod_id].bs_mode) {
		return ret;
	}

	mutex_lock(&i2cctl_fe_mutex);

	props->cnr.len = 1;
	props->cnr.stat[0].scale = FE_SCALE_NOT_AVAILABLE;

	props->strength.len = 1;
	props->strength.stat[1].scale = FE_SCALE_NOT_AVAILABLE;

	props->block_error.len = 1;
	props->block_error.stat[0].scale = FE_SCALE_NOT_AVAILABLE;

	props->block_count.len = 1;
	props->block_count.stat[0].scale = FE_SCALE_NOT_AVAILABLE;

	ret = avl68x2_demod_get_lock_status(&lock, priv->chip);

	if(lock)
	{
		ret |= avl68x2_demod_get_ssi(&ssi, priv->chip);

		props->strength.len = 2;

		props->strength.stat[1].scale = FE_SCALE_RELATIVE;
		props->strength.stat[1].uvalue = (ssi * 65535) / 100;

		props->strength.stat[0].scale = FE_SCALE_DECIBEL;
		props->strength.stat[0].svalue = -80 + ssi / 2;

		ret |= avl68x2_demod_get_snr (&SNR_x100db, priv->chip);
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

	safe_mutex_unlock(&i2cctl_fe_mutex);
	return ret;
}

static int avl68x2_read_status(struct dvb_frontend *fe, enum fe_status *status)
{
	struct avl68x2_priv *priv = fe->demodulator_priv;
	int ret = 0;
	uint8_t lock = 0;
#if 0
	uint8_t tempc = 0;
	uint32_t tempi = 0;
#endif
	int32_t SNR_x100db = 0;
	int32_t ber = 0;
	int8_t demod_id =
	    (priv->chip->chip_pub->i2c_addr >> AVL_DEMOD_ID_SHIFT) &
	    AVL_DEMOD_ID_MASK;

	if (bs_states[demod_id].bs_mode)
	{
		if (bs_states[demod_id].info.progress == 100)
			*status = FE_HAS_LOCK;
		else
			*status = FE_NONE;
		return AVL_EC_OK;
	}

	mutex_lock(&i2cctl_fe_mutex);

	ret = avl68x2_demod_get_lock_status(&lock, priv->chip);

	if (debug > 1)
	{
		ret |= avl68x2_demod_get_snr(&SNR_x100db, priv->chip);
		ret = (int)avl68x2_demod_get_per(&ber, priv->chip);
		printk("read status %d, snr = %d, per = %d\n",
		       *status, SNR_x100db, ber);
	}

	safe_mutex_unlock(&i2cctl_fe_mutex);

	if (!ret && lock == AVL_STATUS_LOCK)
	{
		*status = FE_HAS_SIGNAL | FE_HAS_CARRIER |
			  FE_HAS_VITERBI | FE_HAS_SYNC | FE_HAS_LOCK;
		ret |= get_frontend(fe, &fe->dtv_property_cache);
		avl68x2_set_lock_led(fe, 1);
	}
	else
	{
		*status = FE_HAS_SIGNAL;
		avl68x2_set_lock_led(fe, 0);
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

	p_debug_lvl(10, "");

	mutex_lock(&i2cctl_fe_mutex);

	*ber = 10e7;
	ret = (int)avl68x2_demod_get_per(ber, priv->chip);
	if (!ret)
		*ber /= 100;

	safe_mutex_unlock(&i2cctl_fe_mutex);

	return ret;
}

static enum dvbfe_algo avl68x2fe_algo(struct dvb_frontend *fe)
{
  return DVBFE_ALGO_HW;
}

static int blindscan_confirm_carrier(struct dvb_frontend *fe)
{
	struct avl68x2_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	uint16_t r = AVL_EC_OK;
	uint8_t demod_id =
	    (priv->chip->chip_pub->i2c_addr >> AVL_DEMOD_ID_SHIFT) &
	    AVL_DEMOD_ID_MASK;
	struct avl68x2_bs_state *state = &(bs_states[demod_id]);
	uint16_t cntr;
	const uint16_t timeout = 20;
	const uint32_t delay = 100;
	uint8_t status = 0;

	p_debug("ENTER");

	p_debug("confirming carrier @ RF %d kHz...",
		state->carriers[state->cur_carrier].rf_freq_khz);

	c->frequency = state->carriers[state->cur_carrier].rf_freq_khz;
	c->symbol_rate = state->carriers[state->cur_carrier].symbol_rate_hz;

	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 1);
	r = fe->ops.tuner_ops.set_params(fe);
	priv->chip->chip_pub->tuner_freq_hz = c->frequency * 1000;
	p_debug("tuner_freq: %d", priv->chip->chip_pub->tuner_freq_hz);
	  
	if (fe->ops.i2c_gate_ctrl)
		fe->ops.i2c_gate_ctrl(fe, 0);
	if (r) {
		p_debug("Tuning FAILED\n");
		return r;
	} else {
		p_debug("Tuned to %d kHz",c->frequency);
	}

	r |= AVL_Demod_DVBSxAutoLock(
	    state->carriers[state->cur_carrier].symbol_rate_hz,
	    priv->chip);

	status = 0;
	cntr = 0;
	do
	{
		p_debug("CC %dms",cntr*delay);
		r |= DVBSx_GetLockStatus_Demod(&status, priv->chip);
		avl_bsp_delay(delay);
		cntr++;

	} while ((status == 0) &&
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
	struct avl68x2_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	uint16_t r = AVL_EC_OK;
	uint8_t demod_id =
	    (priv->chip->chip_pub->i2c_addr >> AVL_DEMOD_ID_SHIFT) &
	    AVL_DEMOD_ID_MASK;
	struct avl68x2_bs_state *state = &(bs_states[demod_id]);
	struct AVL_ChannelInfo *carrier;
	uint32_t tuner_step_khz;

	p_debug("ENTER");

	carrier = &state->carriers[state->cur_carrier];

	//mark stream as invalid in case none of the carriers
	//  can be confirmed
	c->AVL68X2_BS_CTRL_PROP = 0;

	//get next stream
	//if at end of current stream list, go get another
	//  from the next carrier, if there is one.
	while (state->cur_carrier < state->info.num_carriers)
	{
		p_debug("cur_carrier %d",state->cur_carrier);
		carrier = &state->carriers[state->cur_carrier];

		if (blindscan_confirm_carrier(fe) !=
			AVL_EC_OK)
		{
			//not confirmed
			//go to next carrier
			p_debug("carrier not confirmed");
			state->cur_carrier++;
			p_debug("next carrier %d", state->cur_carrier);
			continue;
		}

		//carrier confirmed
		//get signal info & put into DVB props
		update_fe_props_sx(fe,c);

		//mark stream as valid
		c->AVL68X2_BS_CTRL_PROP |=
			AVL68X2_BS_CTRL_VALID_STREAM_MASK;



		state->cur_carrier++;
		p_debug("next carrier %d", state->cur_carrier);

	}

	if (state->cur_carrier >= (state->info.num_carriers-1))
	{
		//no more streams. signal back the tuner step
		tuner_step_khz =
			    (uint32_t)state->info.next_tuner_center_freq_100khz -
			    (uint32_t)state->params.tuner_center_freq_100khz;
			tuner_step_khz *= 100;
		c->AVL68X2_BS_CTRL_PROP |= tuner_step_khz;
		p_debug("no more carriers. move tuner to %d kHz (step by %d kHz)",
			(uint32_t)state->info.next_tuner_center_freq_100khz * 100,
			tuner_step_khz);
	}
	else
	{
		//there are more streams
		c->AVL68X2_BS_CTRL_PROP |= AVL68X2_BS_CTRL_MORE_RESULTS_MASK;
	}

	p_debug("EXIT");

	return r;
}

static int blindscan_step(struct dvb_frontend *fe)
{
	struct avl68x2_priv *priv = fe->demodulator_priv;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	uint16_t r = AVL_EC_OK;
	uint8_t demod_id =
	    (priv->chip->chip_pub->i2c_addr >> AVL_DEMOD_ID_SHIFT) &
	    AVL_DEMOD_ID_MASK;
	struct avl68x2_bs_state *state = &(bs_states[demod_id]);
	uint16_t cntr;
	const uint16_t timeout = 50;
	const uint32_t delay = 100;
	uint32_t tuner_step_khz;

	p_debug("ENTER");

	p_debug("BS CTRL %d",c->AVL68X2_BS_CTRL_PROP);
	
	if(c->AVL68X2_BS_CTRL_PROP & AVL68X2_BS_CTRL_NEW_TUNE_MASK) {
		//allow tuner time to settle
		avl_bsp_delay(250);

		//new frequency was tuned, so run a new carrier search
		state->params.tuner_center_freq_100khz = c->frequency / 100;
		//state->params.tuner_lpf_100khz = bs_tuner_bw / 100000;
		state->params.tuner_lpf_100khz = ((c->symbol_rate * 135) / 100) / 100000;
		state->params.min_symrate_khz = bs_min_sr / 1000;
		state->params.max_symrate_khz =
		    avl68x2_ops.info.symbol_rate_max / 1000;

		p_debug("NEW TUNE: start carrier search @%d kHz, LPF %d MHz",
			c->frequency,
			state->params.tuner_lpf_100khz/10);

		state->num_carriers = 0;

		r |= IBase_SendRxOPWait_Demod(AVL_FW_CMD_HALT, priv->chip);
		r |= AVL_Demod_DVBSx_SetFunctionalMode(AVL_FuncMode_BlindScan, priv->chip);
		r |= AVL_Demod_DVBSx_BlindScan_Reset(priv->chip);
		r |= AVL_Demod_DVBSx_BlindScan_Start(&state->params, priv->chip);
		
		state->info.progress = 0;
		cntr = 0;
		do
		{
			avl_bsp_delay(delay);
			r = AVL_Demod_DVBSx_BlindScan_GetStatus(&state->params,
								&state->info,
								priv->chip);
			p_debug("CS %dms",cntr*delay);
			cntr++;

		} while ((state->info.progress != 100) &&
			 (cntr < timeout) && (r == AVL_EC_OK));

		r |= AVL_Demod_DVBSx_SetFunctionalMode(AVL_FuncMode_Demod, priv->chip);

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
				sizeof(AVL_ChannelInfo),
			    GFP_KERNEL);


			r = AVL_Demod_DVBSx_BlindScan_ReadChannelInfo(
				state->info.num_carriers,
				state->carriers,
				priv->chip);
			
			state->cur_carrier = 0;
			r = blindscan_get_next_stream(fe);
		} else {
			//no carriers detected
			//signal back the tuner step
			tuner_step_khz =
			    (uint32_t)state->info.next_tuner_center_freq_100khz -
			    (uint32_t)state->params.tuner_center_freq_100khz;
			tuner_step_khz *= 100;
			c->AVL68X2_BS_CTRL_PROP = tuner_step_khz;
			p_debug("no carriers. move tuner to %d kHz (step by %d kHz)",
				(uint32_t)state->info.next_tuner_center_freq_100khz * 100,
				tuner_step_khz);
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
	int annex_b = 0;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	struct avl68x2_priv *priv = fe->demodulator_priv;
	uint8_t demod_id =
	    (priv->chip->chip_pub->i2c_addr >> AVL_DEMOD_ID_SHIFT) &
	    AVL_DEMOD_ID_MASK;

	avl68x2_set_lock_led(fe,0);

	/* tune tuner if necessary*/
	if (fe->ops.tuner_ops.set_params &&
	    ((bs_states[demod_id].bs_mode &&
	      (c->AVL68X2_BS_CTRL_PROP & AVL68X2_BS_CTRL_NEW_TUNE_MASK)) ||
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
			priv->chip->chip_pub->tuner_freq_hz = c->frequency * 1000;
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

	ret = avl68x2_set_standard(fe, SYS_UNDEFINED);
	if (ret)
	{
		p_error("failed!!!");
		safe_mutex_unlock(&i2cctl_fe_mutex);
		return ret;
	}

	if ((c->delivery_system == SYS_DVBC_ANNEX_A) &&
	    ((c->symbol_rate == 5056941) || (c->symbol_rate == 5360537)))
	{
	  c->bandwidth_hz = 6000000;
	  annex_b = 1;
	}

	switch (c->delivery_system)
	{
	case SYS_DVBT:
	case SYS_DVBT2:
		ret = avl68x2_acquire_dvbtx(fe);
		break;
	case SYS_DVBC_ANNEX_A:
	        if(annex_b) {
		        ret = avl68x2_acquire_dvbc_b(fe);
	        } else { 
	                ret = avl68x2_acquire_dvbc(fe);
	        }
	        break;
	case SYS_DVBC_ANNEX_B:
		ret = avl68x2_acquire_dvbc_b(fe);
		break;
	case SYS_DVBS:
	case SYS_DVBS2:
		if(bs_states[demod_id].bs_mode)
		{
			p_debug("BS STEP");
			ret = blindscan_step(fe);
		}
		else
		{
			ret = avl68x2_acquire_dvbsx(fe);
		}
		break;
	case SYS_ISDBT:
		ret = avl68x2_acquire_isdbt(fe);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	safe_mutex_unlock(&i2cctl_fe_mutex);

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
	int lock_led = priv->chip->chip_pub->gpio_lock_led;
	p_debug("release");
	deinit_avl68x2_i2cctl_device(priv);
#if INCLUDE_STDOUT
	deinit_avl68x2_stdout_device(priv);
#endif
	if(gpio_is_valid(lock_led))
	{
		gpio_free(lock_led);
	}
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
	       SYS_ISDBT, SYS_UNDEFINED},
    .info = {
	.name = "Availink avl68x2",
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
	    FE_CAN_QAM_128 |
	    FE_CAN_QAM_256 |
	    FE_CAN_QAM_AUTO |
	    FE_CAN_TRANSMISSION_MODE_AUTO |
	    FE_CAN_MUTE_TS |
	    FE_CAN_2G_MODULATION |
	    FE_CAN_MULTISTREAM |
	    FE_CAN_INVERSION_AUTO |
	    FE_CAN_GUARD_INTERVAL_AUTO |
	    FE_CAN_HIERARCHY_AUTO |
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
	uint32_t id, chip_id, part_num;
	uint8_t delsys_b[8] = {
		    SYS_DVBC_ANNEX_A,
		    SYS_DVBC_ANNEX_B,
		    SYS_ISDBT,
		    SYS_DVBS,
		    SYS_DVBS2,
		    SYS_DVBT,
		    SYS_DVBT2,
		    SYS_UNDEFINED};
	uint8_t delsys_d[6] = {
		    SYS_DVBC_ANNEX_A,
		    SYS_DVBC_ANNEX_B,
		    SYS_ISDBT,
		    SYS_DVBS,
		    SYS_DVBS2,
		    SYS_UNDEFINED};
	uint8_t delsys_e[5] = {
		    SYS_DVBC_ANNEX_A,
		    SYS_DVBC_ANNEX_B,
		    SYS_DVBT,
		    SYS_DVBT2,
		    SYS_UNDEFINED};
	uint8_t delsys_f[7] = {
		    SYS_DVBC_ANNEX_A,
		    SYS_DVBC_ANNEX_B,
		    SYS_DVBS,
		    SYS_DVBS2,
		    SYS_DVBT,
		    SYS_DVBT2,
		    SYS_UNDEFINED};
	char feat_str[256];
	

	p_debug("start demod attach");

	priv = kzalloc(sizeof(struct avl68x2_priv), GFP_KERNEL);
	if (priv == NULL)
		goto err;
	global_priv = priv;

	p_debug("priv alloc'ed = %llx", (unsigned long long int)priv);

	priv->frontend.demodulator_priv = priv;
	priv->i2c = i2c;
	priv->delivery_system = -1;

	priv->chip = kzalloc(sizeof(struct avl68x2_chip), GFP_KERNEL);
	if (priv->chip == NULL)
		goto err1;

	priv->chip->stStdSpecFunc = kzalloc(
	    sizeof(struct AVL_StandardSpecificFunctions),
	    GFP_KERNEL);
	if (priv->chip->stStdSpecFunc == NULL)
		goto err2;

	priv->chip->chip_priv = kzalloc(sizeof(struct avl68x2_chip_priv),
					GFP_KERNEL);
	if (priv->chip->chip_priv == NULL)
		goto err3;

	priv->chip->chip_pub = kzalloc(sizeof(struct avl68x2_chip_pub),
				       GFP_KERNEL);
	if (priv->chip->chip_pub == NULL)
		goto err4;

	/* copy (ephemeral?) public part of chip config into alloc'd area */
	memcpy(priv->chip->chip_pub,
	       config->chip_pub,
	       sizeof(struct avl68x2_chip_pub));

	priv->chip->chip_pub->tuner = &default_avl_tuner;

	p_debug("Demod ID %d, I2C addr 0x%x",
		(priv->chip->chip_pub->i2c_addr >> AVL_DEMOD_ID_SHIFT) &
		    AVL_DEMOD_ID_MASK,
		priv->chip->chip_pub->i2c_addr & 0xFF);

	// associate demod ID with i2c_adapter
	avl_bsp_assoc_i2c_adapter(priv->chip->chip_pub->i2c_addr, i2c);

	ret = avl68x2_init_chip_object(priv->chip);

	avl68x2_reset(priv->chip->chip_pub->gpio_fec_reset, 0);

	/* get chip id */
	ret |= GetFamilyID_Demod(&id, priv->chip);
	if (ret)
	{
		p_error("attach failed reading id");
		goto err5;
	}
	p_debug("family_id= 0x%x\n", id);
	if (id != AVL68XX)
	{
		p_error("attach failed, id mismatch");
		goto err5;
	}

	ret |= avl68x2_demod_get_chip_id(&chip_id, priv->chip);
	p_debug("chip_id= 0x%x\n",chip_id);

	switch (chip_id)
	{
	case 0xb:
		//C, I, S, T
		memcpy(&avl68x2_ops.delsys,
		       delsys_b, sizeof(delsys_b));
		strlcpy(feat_str,"DVB-C/Sx/Tx, J.83B, and ISDB-T",sizeof(feat_str));
		part_num = 6882;
		break;
	case 0xd:
		//C, I, S
		memcpy(&avl68x2_ops.delsys,
		       delsys_d, sizeof(delsys_d));
		strlcpy(feat_str,"DVB-C/Sx, J.83B, and ISDB-T",sizeof(feat_str));
		part_num = 6812;
		break;
	case 0xe:
		//C, T
		memcpy(&avl68x2_ops.delsys,
		       delsys_e, sizeof(delsys_e));
		strlcpy(feat_str,"DVB-C/Tx and J.83B",sizeof(feat_str));
		part_num = 6762;
		break;
	case 0xf:
		//C, S, T
		memcpy(&avl68x2_ops.delsys,
		       delsys_f, sizeof(delsys_f));
		strlcpy(feat_str,"DVB-C/Sx/Tx and J.83B",sizeof(feat_str));
		part_num = 6862;
		break;
	default:
		part_num = 0;
	}
	memcpy(&priv->frontend.ops, &avl68x2_ops,
	       sizeof(avl68x2_ops));

	p_info("found AVL%d supporting %s", part_num, feat_str);

	if(avl68x2_get_firmware(&priv->frontend,SYS_DVBC_ANNEX_A))
	{
		goto err5;
	}

	if (!avl68x2_demod_initialize(AVL_DVBC,priv->chip))
	{
		p_info("Firmware booted");

		release_firmware(priv->fw);

		init_avl68x2_i2cctl_device(priv);
#if INCLUDE_STDOUT
		init_avl68x2_stdout_device(priv);
#endif

		if(gpio_is_valid(priv->chip->chip_pub->gpio_lock_led))
		{
			gpio_request(priv->chip->chip_pub->gpio_lock_led,
				KBUILD_MODNAME);
		}
		avl68x2_set_lock_led(&priv->frontend, 0);

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

static int __init mod_init(void) {
	uint8_t i;
	//this is called after the params 'set' callbacks

	p_debug("");

	global_priv = NULL;

	for(i=0; i<AVL_MAX_NUM_DEMODS; i++) {
		bs_states[i].bs_mode = (bs_mode>>i) & 1;
		bs_states[i].num_carriers = 0;
		bs_states[i].cur_carrier = 0;
		bs_states[i].carriers = NULL;
	}

	if(strlen(sel_dvbsx_fw) == 0) {
		strlcpy(sel_dvbsx_fw, AVL68X2_DVBSX_FW, sizeof(sel_dvbsx_fw));
	}
	if(strlen(sel_dvbtx_fw) == 0) {
		strlcpy(sel_dvbtx_fw, AVL68X2_DVBTX_FW, sizeof(sel_dvbtx_fw));
	}
	if(strlen(sel_dvbc_fw) == 0) {
		strlcpy(sel_dvbc_fw, AVL68X2_DVBC_FW, sizeof(sel_dvbc_fw));
	}
	if(strlen(sel_isdbt_fw) == 0) {
		strlcpy(sel_isdbt_fw, AVL68X2_ISDBT_FW, sizeof(sel_isdbt_fw));
	}

	return 0;
}
module_init(mod_init);

static void __exit mod_exit(void) {
	uint8_t i;
	for(i=0; i<AVL_MAX_NUM_DEMODS; i++) {
		if(bs_states[i].carriers != NULL)
			kfree(bs_states[i].carriers);

	}
	
}
module_exit(mod_exit);




module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "\n\t\tEnable debug information");

module_param(cable_auto_symrate, int, 0644);
MODULE_PARM_DESC(cable_auto_symrate, "\n\t\tEnable automatic symbol rate detection for cable standards (def. on)");

module_param(cable_auto_cfo, int, 0644);
MODULE_PARM_DESC(cable_auto_cfo, "\n\t\tEnable automatic carrier frequency offset detection for cable standards (def. on)");

static int bs_mode_set(const char *val, const struct kernel_param *kp)
{
	int n = 0, ret, i;
	ret = kstrtoint(val, 0, &n);
	for(i=0; i<AVL_MAX_NUM_DEMODS; i++) {
		bs_states[i].bs_mode = (n>>i) & 1;
	}
	p_info("bs_mode = 0x%x\n",n);
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


/////---------------------------------------------------------------------------
static int diseqc_tone_param_set(const char *val, const struct kernel_param *kp)
{
	int n = 0, ret;
	ret = kstrtoint(val, 10, &n);
	if (ret != 0)
		return -EINVAL;
	if(n > 0)
	{
		diseqc_set_tone(&global_priv->frontend, SEC_TONE_ON);
	}
	else
	{
		diseqc_set_tone(&global_priv->frontend, SEC_TONE_OFF);
	}
	
	return param_set_int(val, kp);
}
static const struct kernel_param_ops diseqc_tone_ops = {
	.set	= diseqc_tone_param_set,
	.get	= param_get_int
};
module_param_cb(diseqc_tone, &diseqc_tone_ops, &diseqc_tone, 0644);
MODULE_PARM_DESC(diseqc_tone, " tone control");


static int diseqc_voltage_param_set(const char *val, const struct kernel_param *kp)
{
	int n = 0, ret;
	ret = kstrtoint(val, 10, &n);
	if (ret != 0)
		return -EINVAL;
	if(n == 0)
	{
		diseqc_set_voltage(&global_priv->frontend, SEC_VOLTAGE_OFF);
	}
	else if(n == 1)
	{
		diseqc_set_voltage(&global_priv->frontend, SEC_VOLTAGE_13);
	}
	else
	{
		diseqc_set_voltage(&global_priv->frontend, SEC_VOLTAGE_18);
	}
	
	
	return param_set_int(val, kp);
}
static const struct kernel_param_ops diseqc_voltage_ops = {
	.set	= diseqc_voltage_param_set,
	.get	= param_get_int
};
module_param_cb(diseqc_voltage, &diseqc_voltage_ops, &diseqc_voltage, 0644);
MODULE_PARM_DESC(diseqc_voltage, " voltage control");
/////---------------------------------------------------------------------------

size_t get_fw_path(const char *val, const char *prefix, char **begin) {
	char *b = NULL;
	size_t len = 0;
	b = strstr(strim((char *)val),prefix);
	if(b != NULL) {
		len = strcspn(b,";");
		p_debug("len %d",(int)len);
		*begin = b;
	} else {
		*begin = NULL;
	}
	
	return len;
}

static int fw_paths_set(const char *val, const struct kernel_param *kp)
{
	char *b = NULL;
	size_t len = 0;
	enum AVL_DemodMode mode = 255;
	enum fe_delivery_system delsys = SYS_UNDEFINED;
	p_info("%s",val);
	len = get_fw_path(val,"C=",&b);
	if(len != 0) {
		memset(sel_dvbc_fw,0,sizeof(sel_dvbc_fw));
		//len - 1 to account for null terminator
		strlcpy(sel_dvbc_fw,
			&b[2],
			min(len - 1, sizeof(sel_dvbc_fw)));
		p_info("set C=%s",sel_dvbc_fw);
		mode = AVL_DVBC;
		delsys = SYS_DVBC_ANNEX_A;
	}

	len = get_fw_path(val,"I=",&b);
	if(len != 0) {
		memset(sel_isdbt_fw,0,sizeof(sel_isdbt_fw));
		strlcpy(sel_isdbt_fw,
			&b[2],
			min(len - 1, sizeof(sel_isdbt_fw)));
		p_info("set I=%s",sel_isdbt_fw);
		mode = AVL_ISDBT;
		delsys = SYS_ISDBT;
	}

	len = get_fw_path(val,"S=",&b);
	if(len != 0) {
		memset(sel_dvbsx_fw,0,sizeof(sel_dvbsx_fw));
		strlcpy(sel_dvbsx_fw,
			&b[2],
			min(len - 1, sizeof(sel_dvbsx_fw)));
		p_info("set S=%s",sel_dvbsx_fw);
		mode = AVL_DVBSX;
		delsys = SYS_DVBS2;
	}

	len = get_fw_path(val,"T=",&b);
	if(len != 0) {
		memset(sel_dvbtx_fw,0,sizeof(sel_dvbtx_fw));
		strlcpy(sel_dvbtx_fw,
			&b[2],
			min(len - 1, sizeof(sel_dvbtx_fw)));
		p_info("set T=%s",sel_dvbtx_fw);
		mode = AVL_DVBTX;
		delsys = SYS_DVBT2;
	}
	strlcpy(fw_paths,val,sizeof(fw_paths));

	if(global_priv != NULL) {
		if(avl68x2_get_firmware(&global_priv->frontend,delsys))
		{
			;;
		} else if (!avl68x2_demod_initialize(mode,global_priv->chip))
		{
			p_info("New firmware booted");
			release_firmware(global_priv->fw);
		}
	}

	return 0;
}
static int fw_paths_get(char *buffer, const struct kernel_param *kp)
{
	sprintf(buffer, "C=%s;I=%s;S=%s;T=%s;",
		sel_dvbc_fw,
		sel_isdbt_fw,
		sel_dvbsx_fw,
		sel_dvbtx_fw);
	return strlen(buffer);
}
static const struct kernel_param_ops fw_paths_ops = {
	.set	= fw_paths_set,
	.get	= fw_paths_get
};
module_param_cb(fw_paths, &fw_paths_ops, fw_paths, 0644);
MODULE_PARM_DESC(fw_paths, "C=<path>;I=<path>;S=<path>;T=<path>;");



EXPORT_SYMBOL_GPL(avl68x2_attach);
EXPORT_SYMBOL_GPL(default_dvbtx_config);
EXPORT_SYMBOL_GPL(default_dvbsx_config);
EXPORT_SYMBOL_GPL(default_isdbt_config);
EXPORT_SYMBOL_GPL(default_dvbc_config);

MODULE_DESCRIPTION("Availink AVL68x2 DVB-S/S2/T/T2/C, ISDB-T, J.83B demodulator driver");
MODULE_AUTHOR("Availink, Inc. (gpl@availink.com)");
MODULE_LICENSE("GPL");
MODULE_VERSION(AVL68X2_VERSION);
MODULE_FIRMWARE(AVL68X2_DVBSX_FW);
MODULE_FIRMWARE(AVL68X2_DVBC_FW);
MODULE_FIRMWARE(AVL68X2_DVBTX_FW);
MODULE_FIRMWARE(AVL68X2_ISDBT_FW);

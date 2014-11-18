/*
* Copyright (C) 2012 Invensense, Inc.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*/

/**
 *  @addtogroup  DRIVERS
 *  @brief       Hardware drivers.
 *
 *  @{
 *      @file    inv_ami306_core.c
 *      @brief   Invensense implementation for AMI306
 *      @details This driver currently works for the AMI306
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/jiffies.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/poll.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>

#include "inv_ami306_iio.h"
#include "sysfs.h"
#include "inv_test/inv_counters.h"

//ASUS_BSP +++ Jiunhau_Wang "[A86][Sensor][NA][Spec] Porting 9-axis sensor"
#include <linux/platform_device.h>
#include <linux/of_gpio.h>

static struct mpu_platform_data ami306_pdata = {
    .orientation = { 0, 0, 1, 0, -1, 0, 1, 0, 0},
};

//ASUS_BSP --- Jiunhau_Wang "[A86][Sensor][NA][Spec] Porting 9-axis sensor"

//ASUS_BSP +++ Jiunhau_Wang "[A86][Sensor][NA][ATD] support e-compass"
static int ecompass_status = 0;
//ASUS_BSP --- Jiunhau_Wang "[A86][Sensor][NA][ATD] support e-compass"

static unsigned char late_initialize = true;

s32 i2c_write(const struct i2c_client *client,
		u8 command, u8 length, const u8 *values)
{
	INV_I2C_INC_COMPASSWRITE(3);
	return i2c_smbus_write_i2c_block_data(client, command, length, values);
}

s32 i2c_read(const struct i2c_client *client,
		u8 command, u8 length, u8 *values)
{
	INV_I2C_INC_COMPASSWRITE(3);
	INV_I2C_INC_COMPASSREAD(length);
	return i2c_smbus_read_i2c_block_data(client, command, length, values);
}

//ASUS_BSP +++ Jiunhau_Wang "[A86][Sensor][NA][Other] support I2C stress test"
#ifdef CONFIG_I2C_STRESS_TEST
#include <linux/i2c_testcase.h>

#define I2C_TEST_FAIL_SENSOR (-1)

static int Test_AMI306_SensorI2C(struct i2c_client *apClient)
{
	int lnResult = I2C_TEST_PASS;
	int result = 0;
	char data;
	
	i2c_log_in_test_case("Test_AMI306_SensorI2C++\n");
	
	result = i2c_read(apClient, REG_AMI_WIA, 1, &data);
	if (result < 0)
	{
		printk(KERN_INFO "[AMI306][I2C] i2c read fail \n");
		return I2C_TEST_FAIL_SENSOR;
	}
	if (data != DATA_WIA)
	{
		printk(KERN_INFO "[AMI306][I2C] (%d) != (%d) \n", data, DATA_WIA);
		return I2C_TEST_FAIL_SENSOR;
	}
	printk(KERN_INFO "[AMI306][I2C] ID = (%d) \n", data);
	i2c_log_in_test_case("Test_AMI306_SensorI2C--\n");
	
	return lnResult;
}

static struct i2c_test_case_info gAMI306_TestCaseInfo[] =
{
	__I2C_STRESS_TEST_CASE_ATTR(Test_AMI306_SensorI2C),
};
#endif // CONFIG_I2C_STRESS_TEST
//ASUS_BSP --- Jiunhau_Wang "[A86][Sensor][NA][Other] support I2C stress test"

static int ami306_read_param(struct inv_ami306_state_s *st)
{
	int result = 0;
	unsigned char regs[AMI_PARAM_LEN];
	struct ami_sensor_parametor *param = &st->param;

	result = i2c_read(st->i2c, REG_AMI_SENX,
			AMI_PARAM_LEN, regs);
	if (result < 0)
		return result;

	/* Little endian 16 bit registers */
	param->m_gain.x = le16_to_cpup((__le16 *)(&regs[0]));
	param->m_gain.y = le16_to_cpup((__le16 *)(&regs[2]));
	param->m_gain.z = le16_to_cpup((__le16 *)(&regs[4]));

	param->m_interference.xy = regs[7];
	param->m_interference.xz = regs[6];
	param->m_interference.yx = regs[9];
	param->m_interference.yz = regs[8];
	param->m_interference.zx = regs[11];
	param->m_interference.zy = regs[10];

	param->m_offset.x = AMI_STANDARD_OFFSET;
	param->m_offset.y = AMI_STANDARD_OFFSET;
	param->m_offset.z = AMI_STANDARD_OFFSET;

	param->m_gain_cor.x = AMI_GAIN_COR_DEFAULT;
	param->m_gain_cor.y = AMI_GAIN_COR_DEFAULT;
	param->m_gain_cor.z = AMI_GAIN_COR_DEFAULT;

	return 0;
}

static int ami306_write_offset(const struct i2c_client *client,
				unsigned char *fine)
{
	int result = 0;
	unsigned char dat[3];
	dat[0] = (0x7f & fine[0]);
	dat[1] = 0;
	result = i2c_write(client, REG_AMI_OFFX, 2, dat);
	dat[0] = (0x7f & fine[1]);
	dat[1] = 0;
	result = i2c_write(client, REG_AMI_OFFY, 2, dat);
	dat[0] = (0x7f & fine[2]);
	dat[1] = 0;
	result = i2c_write(client, REG_AMI_OFFZ, 2, dat);

	return result;
}

static int ami306_wait_data_ready(struct inv_ami306_state_s *st,
				unsigned long usecs, unsigned long times)
{
	int result = 0;
	unsigned char buf;

	for (; 0 < times; --times) {
		udelay(usecs);
		result = i2c_read(st->i2c, REG_AMI_STA1, 1, &buf);
		if (result < 0)
			return INV_ERROR_COMPASS_DATA_NOT_READY;
		if (buf & AMI_STA1_DRDY_BIT)
			return 0;
		else if (buf & AMI_STA1_DOR_BIT)
			return INV_ERROR_COMPASS_DATA_OVERFLOW;
	}

	return INV_ERROR_COMPASS_DATA_NOT_READY;
}
int ami306_read_raw_data(struct inv_ami306_state_s *st,
			short dat[3])
{
	int result;
	unsigned char buf[6];
	result = i2c_read(st->i2c, REG_AMI_DATAX, sizeof(buf), buf);
	if (result < 0)
		return result;
	dat[0] = le16_to_cpup((__le16 *)(&buf[0]));
	dat[1] = le16_to_cpup((__le16 *)(&buf[2]));
	dat[2] = le16_to_cpup((__le16 *)(&buf[4]));

	return 0;
}

#define AMI_WAIT_DATAREADY_RETRY                3       /* retry times */
#define AMI_DRDYWAIT                            800     /* u(micro) sec */
static int ami306_force_measurement(struct inv_ami306_state_s *st,
					short ver[3])
{
	int result;
	int status;
	char buf;
	buf = AMI_CTRL3_FORCE_BIT;
	result = i2c_write(st->i2c, REG_AMI_CTRL3, 1, &buf);
	if (result < 0)
		return result;

	result = ami306_wait_data_ready(st,
			AMI_DRDYWAIT, AMI_WAIT_DATAREADY_RETRY);
	if (result && result != INV_ERROR_COMPASS_DATA_OVERFLOW)
		return result;
	/*  READ DATA X,Y,Z */
	status = ami306_read_raw_data(st, ver);
	if (status)
		return status;

	return result;
}

static int ami306_initial_b0_adjust(struct inv_ami306_state_s *st)
{
	int result;
	unsigned char fine[3] = { 0 };
	short data[3];
	int diff[3] = { 0x7fff, 0x7fff, 0x7fff };
	int fn = 0;
	int ax = 0;
	unsigned char buf[3];

	buf[0] = AMI_CTRL2_DREN;
	result = i2c_write(st->i2c, REG_AMI_CTRL2, 1, buf);
	if (result)
		return result;

	buf[0] = AMI_CTRL4_HS & 0xFF;
	buf[1] = (AMI_CTRL4_HS >> 8) & 0xFF;
	result = i2c_write(st->i2c, REG_AMI_CTRL4, 2, buf);
	if (result < 0)
		return result;

	for (fn = 0; fn < AMI_FINE_MAX; ++fn) { /* fine 0 -> 95 */
		fine[0] = fine[1] = fine[2] = fn;
		result = ami306_write_offset(st->i2c, fine);
		if (result)
			return result;

		result = ami306_force_measurement(st, data);
		if (result)
			return result;

		for (ax = 0; ax < 3; ax++) {
			/* search point most close to zero. */
			if (diff[ax] > abs(data[ax])) {
				st->fine[ax] = fn;
				diff[ax] = abs(data[ax]);
			}
		}
	}
	result = ami306_write_offset(st->i2c, st->fine);
	if (result)
		return result;

	/* Software Reset */
	buf[0] = AMI_CTRL3_SRST_BIT;
	result = i2c_write(st->i2c, REG_AMI_CTRL3, 1, buf);
	if (result < 0)
		return result;
	else
		return 0;
}

static int ami306_start_sensor(struct inv_ami306_state_s *st)
{
	int result = 0;
	unsigned char buf[2];

	/* Step 1 */
	buf[0] = (AMI_CTRL1_PC1 | AMI_CTRL1_FS1_FORCE);
	result = i2c_write(st->i2c, REG_AMI_CTRL1, 1, buf);
	if (result < 0)
		return result;
	/* Step 2 */
	buf[0] = AMI_CTRL2_DREN;
	result = i2c_write(st->i2c, REG_AMI_CTRL2, 1, buf);
	if (result < 0)
		return result;
	/* Step 3 */
	buf[0] = (AMI_CTRL4_HS & 0xFF);
	buf[1] = (AMI_CTRL4_HS >> 8) & 0xFF;

	result = i2c_write(st->i2c, REG_AMI_CTRL4, 2, buf);
	if (result < 0)
		return result;

	/* Step 4 */
	result = ami306_write_offset(st->i2c, st->fine);

	return result;
}

int set_ami306_enable(struct iio_dev *indio_dev, int state)
{
	struct inv_ami306_state_s *st = iio_priv(indio_dev);
	int result;
	char buf;

	buf = (AMI_CTRL1_PC1 | AMI_CTRL1_FS1_FORCE);
	result = i2c_write(st->i2c, REG_AMI_CTRL1, 1, &buf);
	if (result < 0)
		return result;

	result =  ami306_read_param(st);
	if (result)
		return result;
	if (late_initialize) {
		result = ami306_initial_b0_adjust(st);
		if (result)
			return result;
		late_initialize = false;
	}
	result = ami306_start_sensor(st);
	if (result)
		return result;
	buf = AMI_CTRL3_FORCE_BIT;
	st->timestamp = iio_get_time_ns();
	result = i2c_write(st->i2c, REG_AMI_CTRL3, 1, &buf);
	if (result)
		return result;

	return 0;
}

/**
 *  ami306_read_raw() - read raw method.
 */
static int ami306_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int *val,
			      int *val2,
			      long mask) {
	struct inv_ami306_state_s  *st = iio_priv(indio_dev);

	switch (mask) {
	case 0:
		if (!(iio_buffer_enabled(indio_dev)))
			return -EINVAL;
		if (chan->type == IIO_MAGN) {
			*val = st->compass_data[chan->channel2 - IIO_MOD_X];
			return IIO_VAL_INT;
		}

		return -EINVAL;
	case IIO_CHAN_INFO_SCALE:
		if (chan->type == IIO_MAGN) {
			*val = AMI_SCALE;
			return IIO_VAL_INT;
		}
		return -EINVAL;
	default:
		return -EINVAL;
	}
}

/**
 * inv_compass_matrix_show() - show orientation matrix
 */
static ssize_t inv_compass_matrix_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	signed char *m;
	struct inv_ami306_state_s *st = iio_priv(indio_dev);
	m = st->plat_data.orientation;
	return sprintf(buf,
	"%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
		m[0],  m[1],  m[2],  m[3], m[4], m[5], m[6], m[7], m[8]);
}

static ssize_t ami306_rate_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_ami306_state_s *st = iio_priv(indio_dev);

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	if (0 == data)
		return -EINVAL;
	/* transform rate to delay in ms */
	data = 1000 / data;
	if (data > AMI_MAX_DELAY)
		data = AMI_MAX_DELAY;
	if (data < AMI_MIN_DELAY)
		data = AMI_MIN_DELAY;
	st->delay = data;
	return count;
}

static ssize_t ami306_rate_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_ami306_state_s *st = iio_priv(indio_dev);
	/* transform delay in ms to rate */
	return sprintf(buf, "%d\n", 1000 / st->delay);
}


static void ami306_work_func(struct work_struct *work)
{
	struct inv_ami306_state_s *st =
		container_of((struct delayed_work *)work,
			struct inv_ami306_state_s, work);
	struct iio_dev *indio_dev = iio_priv_to_dev(st);
	unsigned long delay = msecs_to_jiffies(st->delay);

	mutex_lock(&indio_dev->mlock);
	if (!(iio_buffer_enabled(indio_dev)))
		goto error_ret;

	st->timestamp = iio_get_time_ns();
	schedule_delayed_work(&st->work, delay);
	inv_read_ami306_fifo(indio_dev);
	INV_I2C_INC_COMPASSIRQ();

error_ret:
	mutex_unlock(&indio_dev->mlock);
}

static const struct iio_chan_spec compass_channels[] = {
	{
		.type = IIO_MAGN,
		.modified = 1,
		.channel2 = IIO_MOD_X,
		.info_mask = IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.scan_index = INV_AMI306_SCAN_MAGN_X,
		.scan_type = IIO_ST('s', 16, 16, 0)
	}, {
		.type = IIO_MAGN,
		.modified = 1,
		.channel2 = IIO_MOD_Y,
		.info_mask = IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.scan_index = INV_AMI306_SCAN_MAGN_Y,
		.scan_type = IIO_ST('s', 16, 16, 0)
	}, {
		.type = IIO_MAGN,
		.modified = 1,
		.channel2 = IIO_MOD_Z,
		.info_mask = IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.scan_index = INV_AMI306_SCAN_MAGN_Z,
		.scan_type = IIO_ST('s', 16, 16, 0)
	},
	IIO_CHAN_SOFT_TIMESTAMP(INV_AMI306_SCAN_TIMESTAMP)
};

static DEVICE_ATTR(compass_matrix, S_IRUGO, inv_compass_matrix_show, NULL);
static DEVICE_ATTR(sampling_frequency, S_IRUGO | S_IWUSR, ami306_rate_show,
		ami306_rate_store);

//ASUS_BSP +++ Jiunhau_Wang "[A86][Sensor][NA][Spec] Porting 9-axis sensor"
static ssize_t inv_compass_id_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	unsigned char data[8];
	int result = 0;
	struct inv_ami306_state_s *st = iio_priv(dev_get_drvdata(dev));

	result = i2c_read(st->i2c, 0xF, sizeof(data), data);
	
	return sprintf(buf, "0x%x\n", data[0]);
}

static DEVICE_ATTR(compass_6500_id, S_IRUGO, inv_compass_id_show, NULL);
//ASUS_BSP --- Jiunhau_Wang "[A86][Sensor][NA][Spec] Porting 9-axis sensor"

//ASUS_BSP +++ Jiunhau_Wang "[A86][Sensor][NA][ATD] support e-compass"
static ssize_t read_compass_status(struct device *dev, struct device_attribute *devattr, char *buf)
{	
	return sprintf(buf, "%d\n", ecompass_status);
}

static DEVICE_ATTR(compass_6500_status, S_IRUGO, read_compass_status, NULL);

static ssize_t read_compass_raw(struct device *dev, struct device_attribute *devattr, char *buf)
{
	int result;
	short dat[3];
	unsigned char data[6];
	struct inv_ami306_state_s *st = iio_priv(dev_get_drvdata(dev));

	result = set_ami306_enable(dev_get_drvdata(dev), 1);
	if (result < 0)
		return sprintf(buf, "0 0 0\n"); 
	
	result = i2c_read(st->i2c, REG_AMI_DATAX, sizeof(data), data);
	if (result < 0)
		return sprintf(buf, "0 0 0\n"); 
	dat[0] = le16_to_cpup((__le16 *)(&data[0]));
	dat[1] = le16_to_cpup((__le16 *)(&data[2]));
	dat[2] = le16_to_cpup((__le16 *)(&data[4]));

	return sprintf(buf, "%d %d %d\n",dat[0],dat[1],dat[2]); 
}

static DEVICE_ATTR(compass_6500_raw, S_IRUGO, read_compass_raw, NULL);

//ASUS_BSP --- Jiunhau_Wang "[A86][Sensor][NA][ATD] support e-compass"

//ASUS_BSP +++ Jiunhau_Wang "[A86][Sensor][NA][ATD] support e-compass i2c status"
static ssize_t check_compass_i2c(struct device *dev, struct device_attribute *devattr, char *buf)
{
	unsigned char data[8];
	int result = 0, ret = 0;
	
	struct inv_ami306_state_s *st = iio_priv(dev_get_drvdata(dev));

	result = i2c_read(st->i2c, 0xF, sizeof(data), data);
	printk("[AMI306] chip ID = (%d) \n", result);
	
	ret = (data[0] == 0x46) ? (1) : (0);
	
	return sprintf(buf, "%d\n", ret);
}
DEVICE_ATTR(compass_6500_i2c, S_IRUGO, check_compass_i2c, NULL);


//ASUS_BSP --- Jiunhau_Wang "[A86][Sensor][NA][ATD] support e-compass i2c status"

//ASUS_BSP +++ Jiunhau_Wang "[A86][Sensor][NA][ATD] support info of e-compass mode"
static ssize_t check_compass_run_mode(struct device *dev, struct device_attribute *devattr, char *buf)
{
	unsigned char data[8];
	int result = 0;
	
	struct inv_ami306_state_s *st = iio_priv(dev_get_drvdata(dev));
	result = i2c_read(st->i2c, REG_AMI_CTRL1, sizeof(data), data);
	
	return sprintf(buf, "%d\n", (((data[0] & AMI_CTRL1_PC1) >> 7) & 0x1));   
}

DEVICE_ATTR(compass_6500_run_mode, S_IRUGO, check_compass_run_mode, NULL);
//ASUS_BSP --- Jiunhau_Wang "[A86][Sensor][NA][ATD] support info of e-compass mode"

static struct attribute *inv_ami306_attributes[] = {
	&dev_attr_compass_matrix.attr,
	&dev_attr_sampling_frequency.attr,
//ASUS_BSP +++ Jiunhau_Wang "[A86][Sensor][NA][Spec] Porting 9-axis sensor"
	&dev_attr_compass_6500_id.attr,
//ASUS_BSP --- Jiunhau_Wang "[A86][Sensor][NA][Spec] Porting 9-axis sensor"
//ASUS_BSP +++ Jiunhau_Wang "[A86][Sensor][NA][ATD] support e-compass"
	&dev_attr_compass_6500_status.attr,
	&dev_attr_compass_6500_raw.attr,
//ASUS_BSP --- Jiunhau_Wang "[A86][Sensor][NA][ATD] support e-compass"
//ASUS_BSP +++ Jiunhau_Wang "[A86][Sensor][NA][ATD] support e-compass i2c status"
	&dev_attr_compass_6500_i2c.attr,
//ASUS_BSP --- Jiunhau_Wang "[A86][Sensor][NA][ATD] support e-compass i2c status"
//ASUS_BSP +++ Jiunhau_Wang "[A86][Sensor][NA][ATD] support info of e-compass mode"
	&dev_attr_compass_6500_run_mode.attr,
//ASUS_BSP --- Jiunhau_Wang "[A86][Sensor][NA][ATD] support info of e-compass mode"
	NULL,
};
static const struct attribute_group inv_attribute_group = {
	.name = "ami306",
	.attrs = inv_ami306_attributes
};

static const struct iio_info ami306_info = {
	.driver_module = THIS_MODULE,
	.read_raw = &ami306_read_raw,
	.attrs = &inv_attribute_group,
};

/*constant IIO attribute */
/**
 *  inv_ami306_probe() - probe function.
 */
static int inv_ami306_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct inv_ami306_state_s *st;
	struct iio_dev *indio_dev;
	int result;
	char data;
	
	if (g_ASUS_hwID != A90_EVB0) {
		printk(KERN_INFO "[AMI306] only support A90-FF\n");
		return 0;
	}
	printk(KERN_INFO"[AMI306] %s +++\n",__FUNCTION__);
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		result = -ENODEV;
		printk(KERN_INFO "[AMI306][probe] device not found \n");
		goto out_no_free;
	}
	indio_dev = iio_allocate_device(sizeof(*st));
	if (indio_dev == NULL) {
		result =  -ENOMEM;
		printk(KERN_INFO "[AMI306][probe] no memory \n");
		goto out_no_free;
	}
	st = iio_priv(indio_dev);
	st->i2c = client;
	st->plat_data = ami306_pdata;
	
	st->delay = 10;

	/* Make state variables available to all _show and _store functions. */
	i2c_set_clientdata(client, indio_dev);

	result = i2c_read(st->i2c, REG_AMI_WIA, 1, &data);
	if (result < 0)
	{
		printk(KERN_INFO "[AMI306][probe] i2c read fail \n");
		goto out_free;
	}
	if (data != DATA_WIA)
	{
		printk(KERN_INFO "[AMI306][probe] (%d) != (%d) \n", data, DATA_WIA);
		goto out_free;
	}
	indio_dev->dev.parent = &client->dev;
	indio_dev->name = id->name;
	indio_dev->channels = compass_channels;
	indio_dev->num_channels = ARRAY_SIZE(compass_channels);
	indio_dev->info = &ami306_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->currentmode = INDIO_DIRECT_MODE;

	result = inv_ami306_configure_ring(indio_dev);
	if (result)
	{
		printk(KERN_INFO "[AMI306][probe] inv_ami306_configure_ring fail\n");
		goto out_free;
	}
	result = iio_buffer_register(indio_dev, indio_dev->channels,
					indio_dev->num_channels);
	if (result)
	{
		printk(KERN_INFO "[AMI306][probe] iio_buffer_register fail\n");
		goto out_unreg_ring;
	}
	result = inv_ami306_probe_trigger(indio_dev);
	if (result)
	{
		printk(KERN_INFO "[AMI306][probe] inv_ami306_probe_trigger fail\n");
		goto out_remove_ring;
	}
	result = iio_device_register(indio_dev);
	if (result)
	{
		printk(KERN_INFO "[AMI306][probe] iio_device_register fail\n");
		goto out_remove_trigger;
	}
	INIT_DELAYED_WORK(&st->work, ami306_work_func);
	printk(KERN_INFO "[AMI306][probe] %s is ready to go!\n",
					indio_dev->name);
	printk(KERN_INFO "[AMI306] %s ---\n",__FUNCTION__);

//ASUS_BSP +++ Jiunhau_Wang "[A86][Sensor][NA][ATD] support e-compass" //compass_6050_status
	ecompass_status = 1;
//ASUS_BSP --- Jiunhau_Wang "[A86][Sensor][NA][ATD] support e-compass

//ASUS_BSP +++ Jiunhau_Wang "[A86][Sensor][NA][Other] support I2C stress test"
#ifdef CONFIG_I2C_STRESS_TEST
       i2c_add_test_case(st->i2c, "Sensor_AMI306",ARRAY_AND_SIZE(gAMI306_TestCaseInfo));
#endif //CONFIG_I2C_STRESS_TEST
//ASUS_BSP --- Jiunhau_Wang "[A86][Sensor][NA][Other] support I2C stress test"

	return 0;
out_remove_trigger:
	if (indio_dev->modes & INDIO_BUFFER_TRIGGERED)
		inv_ami306_remove_trigger(indio_dev);
out_remove_ring:
	iio_buffer_unregister(indio_dev);
out_unreg_ring:
	inv_ami306_unconfigure_ring(indio_dev);
out_free:
	iio_free_device(indio_dev);
out_no_free:
	dev_err(&client->adapter->dev, "%s failed %d\n", __func__, result);
	return -EIO;
}

/**
 *  inv_ami306_remove() - remove function.
 */
static int inv_ami306_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct inv_ami306_state_s *st = iio_priv(indio_dev);
	
	printk(KERN_INFO "[AMI306] %s +++\n",__FUNCTION__);
	cancel_delayed_work_sync(&st->work);
	iio_device_unregister(indio_dev);
	inv_ami306_remove_trigger(indio_dev);
	iio_buffer_unregister(indio_dev);
	inv_ami306_unconfigure_ring(indio_dev);
	iio_free_device(indio_dev);

	dev_info(&client->adapter->dev, "inv-ami306-iio module removed.\n");
	printk(KERN_INFO "[AMI306] %s ---\n",__FUNCTION__);
	
	return 0;
}
static const unsigned short normal_i2c[] = { I2C_CLIENT_END };
/* device id table is used to identify what device can be
 * supported by this driver
 */
static const struct i2c_device_id inv_ami306_id[] = {
	{"ami306", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, inv_ami306_id);

//ASUS_BSP +++ Jiunhau_Wang "[A86][Sensor][NA][Spec] Porting 9-axis sensor"
#ifdef CONFIG_OF
static struct of_device_id ami306_match_table[] = {
	{ .compatible = "aichi,ami306",},
	{},
};
#else
#define ami306_match_table NULL
#endif
//ASUS_BSP --- Jiunhau_Wang "[A86][Sensor][NA][Spec] Porting 9-axis sensor"

static struct i2c_driver inv_ami306_driver = {
	.class = I2C_CLASS_HWMON,
	.probe		=	inv_ami306_probe,
	.remove		=	inv_ami306_remove,
	.id_table	=	inv_ami306_id,
	.driver = {
		.owner	=	THIS_MODULE,
		.name	=	"inv-ami306-iio",
//ASUS_BSP +++ Jiunhau_Wang "[A86][Sensor][NA][Spec] Porting 9-axis sensor"
		.of_match_table = ami306_match_table,
//ASUS_BSP --- Jiunhau_Wang "[A86][Sensor][NA][Spec] Porting 9-axis sensor"
	},
	.address_list = normal_i2c,
};

module_i2c_driver(inv_ami306_driver);

#if 0
static int __init inv_ami306_init(void)
{
	int result = i2c_add_driver(&inv_ami306_driver);

	if (result) {
		pr_err("%s failed\n", __func__);
		return result;
	}
	return 0;
}

static void __exit inv_ami306_exit(void)
{
	i2c_del_driver(&inv_ami306_driver);
}


module_init(inv_ami306_init);
module_exit(inv_ami306_exit);
#endif

MODULE_AUTHOR("Invensense Corporation");
MODULE_DESCRIPTION("Invensense device driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("inv-ami306-iio");
/**
 *  @}
 */


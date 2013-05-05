/* Himax Android Driver Sample Code Ver 2.4
 *
 * Copyright (C) 2012 Himax Corporation.
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

//=============================================================================================================
//
//	Segment : Include Header file 
//
//=============================================================================================================
#include <linux/module.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>
#include <linux/debugfs.h>
#include <linux/irq.h>
#include <linux/syscalls.h>
#include <linux/time.h>

// for linux 2.6.36.3
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <asm/ioctl.h>
#include <linux/switch.h>
#include <linux/proc_fs.h>
#include <linux/wakelock.h>
#include <linux/regulator/consumer.h>

#include <linux/fs.h>
#include <asm/segment.h>
#include <asm/uaccess.h>
#include <linux/buffer_head.h>
#include <linux/kthread.h>


#include <linux/notifier.h>
#include <linux/fb.h>

//Add for dts
#include <linux/of_gpio.h>

#if defined(CONFIG_EEPROM_NUVOTON)
#include <linux/microp_notify.h>
#include <linux/microp_notifier_controller.h>
#endif

#include "hx8528.h"

//=============================================================================================================
//
//	Segment : Himax Variable/Pre Declation Function
//
//=============================================================================================================
static struct himax_ts_data *private_ts = NULL;		// himax_ts_data variable
static uint8_t IC_STATUS_CHECK = 0xAA;	// for Hand shaking to check IC status
static int tpd_keys_local[HX_KEY_MAX_COUNT] = HX_KEY_ARRAY;	// for Virtual key array

static unsigned char IC_CHECKSUM = 0;
static unsigned char IC_TYPE = 0;

static int HX_TOUCH_INFO_POINT_CNT = 0;

static int HX_RX_NUM = 0;
static int HX_TX_NUM = 0;
static int HX_BT_NUM = 3;
static int HX_X_RES = 0;
static int HX_Y_RES = 0;
static int HX_MAX_PT = 0;
static bool HX_INT_IS_EDGE = false;
static unsigned int FW_VER_MAJ_FLASH_ADDR;
static unsigned int FW_VER_MAJ_FLASH_LENG;
static unsigned int FW_VER_MIN_FLASH_ADDR;
static unsigned int FW_VER_MIN_FLASH_LENG;
static unsigned int CFG_VER_MAJ_FLASH_ADDR;
static unsigned int CFG_VER_MAJ_FLASH_LENG;
static unsigned int CFG_VER_MIN_FLASH_ADDR;
static unsigned int CFG_VER_MIN_FLASH_LENG;

static u16 FW_VER_MAJ_buff[1];		// for Firmware Version
static u16 FW_VER_MIN_buff[1];
static u16 CFG_VER_MAJ_buff[12];
static u16 CFG_VER_MIN_buff[12];

static bool ts_suspend = false;
static bool ts_inpad = false;

/* +++jacob add for resume touch when dds finish after detach pad+++ */
volatile bool ts_resume_trigger = false;
static int dds_touch_resume_mode = 0;
static bool dds_notify_enable = false;
/* ---jacob add for resume touch when dds finish after detach pad--- */

static int hx_point_num = 0;					// for himax_ts_work_func use
static int p_point_num = 0xFFFF;
static int tpd_key = 0;
static int tpd_key_old = 0xFF;

struct kobject *android_touch_kobj;		// Sys kobject variable

static int finger_count[10][2];
static bool point_printed = 0;

struct switch_dev hx_sdev;
static int update_progress = 0;
static int TOTAL_COUNT = 100;

#ifdef ASUS_FACTORY_BUILD
static bool common_fw = false;
#endif

static struct delayed_work hx_resume_w;
#ifdef CONFIG_TOUCHSCREEN_DOUBLETAP2WAKE
extern int dt2w_switch;
#endif

static struct workqueue_struct *hx_attach_detach_wq;

#if defined(CONFIG_EEPROM_NUVOTON)
static struct work_struct mp_attach_w;
static struct delayed_work mp_detach_w;
#endif

//----[HX_TP_SYS_SELF_TEST]-----------------------------------------------------------------------------start
#ifdef HX_TP_SYS_SELF_TEST 
static ssize_t himax_chip_self_test_function(struct device *dev, struct device_attribute *attr, char *buf);
static int himax_chip_self_test(void);
static uint8_t rFE96_setting[8] = { 0x02, 0x58, 0x14, 0x14, 0x37, 0x08, 0x37, 0x08};
static int self_test_delay_time = 2;
static uint8_t logbuf[16];
#endif 
//----[HX_TP_SYS_SELF_TEST]-------------------------------------------------------------------------------end

//----[HX_TP_SYS_DEBUG_LEVEL]---------------------------------------------------------------------------start
#ifdef HX_TP_SYS_DEBUG_LEVEL
static uint8_t debug_log_level = 0;
static bool fw_update_complete = false;
static int handshaking_result = 0;
static unsigned char debug_level_cmd = 0;
static unsigned char upgrade_fw[32 * 1024];
static uint8_t getDebugLevel(void);
#endif
//----[HX_TP_SYS_DEBUG_LEVEL]-----------------------------------------------------------------------------end

//----[HX_TP_SYS_REGISTER]------------------------------------------------------------------------------start
#ifdef HX_TP_SYS_REGISTER
static uint8_t register_command = 0;
static uint8_t multi_register_command = 0;
static uint8_t multi_register[8] = { 0x00 };
static uint8_t multi_cfg_bank[8] = { 0x00 };
static uint8_t multi_value[1024] = { 0x00 };
static bool config_bank_reg = false;
#endif
//----[HX_TP_SYS_REGISTER]--------------------------------------------------------------------------------end

//----[HX_TP_SYS_DIAG]----------------------------------------------------------------------------------start
#ifdef HX_TP_SYS_DIAG
static uint8_t x_channel = 0;
static uint8_t y_channel = 0;
static uint8_t *diag_mutual = NULL;
static uint8_t diag_command = 0;
static uint8_t diag_coor[128]; // = {0xFF};

static uint8_t diag_self[100] = { 0 };

static uint8_t *getMutualBuffer(void);
static uint8_t *getSelfBuffer(void);
static uint8_t getDiagCommand(void);
static uint8_t getXChannel(void);
static uint8_t getYChannel(void);

static void setMutualBuffer(void);
static void setXChannel(uint8_t x);
static void setYChannel(uint8_t y);

static uint8_t coordinate_dump_enable = 0;
static struct file *coordinate_fn;
#endif
//----[HX_TP_SYS_DIAG]------------------------------------------------------------------------------------end

//----[HX_TP_SYS_FLASH_DUMP]----------------------------------------------------------------------------start	
#ifdef HX_TP_SYS_FLASH_DUMP
static uint8_t *flash_buffer = NULL;
static uint8_t flash_command = 0;
static uint8_t flash_read_step = 0;
static uint8_t flash_progress = 0;
static uint8_t flash_dump_complete = 0;
static uint8_t flash_dump_fail = 0;
static uint8_t sys_operation = 0;
static uint8_t flash_dump_sector = 0;
static uint8_t flash_dump_page = 0;
static bool flash_dump_going = false;

static uint8_t getFlashCommand(void);
static uint8_t getFlashDumpComplete(void);
static uint8_t getFlashDumpFail(void);
static uint8_t getFlashDumpProgress(void);
static uint8_t getFlashReadStep(void);
static uint8_t getSysOperation(void);
static uint8_t getFlashDumpSector(void);
static uint8_t getFlashDumpPage(void);
static bool getFlashDumpGoing(void);

static void setFlashBuffer(void);
static void setFlashCommand(uint8_t command);
static void setFlashReadStep(uint8_t step);
static void setFlashDumpComplete(uint8_t complete);
static void setFlashDumpFail(uint8_t fail);
static void setFlashDumpProgress(uint8_t progress);
static void setSysOperation(uint8_t operation);
static void setFlashDumpSector(uint8_t sector);
static void setFlashDumpPage(uint8_t page);
static void setFlashDumpGoing(bool going);
static int himax_dds_touch_resume(const char *val, struct kernel_param *kp);
#endif
//----[HX_TP_SYS_FLASH_DUMP]------------------------------------------------------------------------------end

int touch_debug_mask = DEF_POINT_INFO;
module_param_named(debug_mask, touch_debug_mask,
		   int, S_IRUGO | S_IWUSR | S_IWGRP);

/* +++jacob add for resume touch when dds finish after detach pad+++ */
module_param_call(dds_touch_resume_mode, himax_dds_touch_resume, param_get_int,
			&dds_touch_resume_mode, 0644);
/* ---jacob add for resume touch when dds finish after detach pad--- */


//=============================================================================================================
//
//	Segment : Himax Normal Function
//
//=============================================================================================================
//----[ normal function]--------------------------------------------------------------------------------start
static void calculate_point_number(void)
{
	HX_TOUCH_INFO_POINT_CNT = HX_MAX_PT * 4;

	if ((HX_MAX_PT % 4) == 0)
	{
		HX_TOUCH_INFO_POINT_CNT += (HX_MAX_PT / 4) * 4;
	}
	else
	{
		HX_TOUCH_INFO_POINT_CNT += ((HX_MAX_PT / 4) + 1) * 4;
	}
}

static int himax_ic_package_check(struct himax_ts_data *ts_modify)
{
	uint8_t cmd[3];
	uint8_t data[3];

	if (i2c_himax_read(ts_modify->client, 0xD1, cmd, 3, DEFAULT_RETRY_CNT) < 0)
	{
		return -1;
	}

	if (i2c_himax_read(ts_modify->client, 0x31, data, 3, DEFAULT_RETRY_CNT) < 0)
	{
		return -1;
	}

	if ((data[0] == 0x85 && data[1] == 0x28)
		|| (cmd[0] == 0x04 && cmd[1] == 0x85
				&& (cmd[2] == 0x26 || cmd[2] == 0x27 || cmd[2] == 0x28)))
	{
		IC_TYPE = HX_85XX_D_SERIES_PWON;
		IC_CHECKSUM = HX_TP_BIN_CHECKSUM_CRC;
		//Himax: Set FW and CFG Flash Address
		FW_VER_MAJ_FLASH_ADDR = 133;	//0x0085
		FW_VER_MAJ_FLASH_LENG = 1;

		FW_VER_MIN_FLASH_ADDR = 134;  //0x0086
		FW_VER_MIN_FLASH_LENG = 1;
		CFG_VER_MAJ_FLASH_ADDR = 160;	//0x00A0
		CFG_VER_MAJ_FLASH_LENG = 12;
		CFG_VER_MIN_FLASH_ADDR = 172;	//0x00AC
		CFG_VER_MIN_FLASH_LENG = 12;

	/*	printk("[Touch_H]Himax IC package 8528 D\n");*/

		return 0;
	}
	else
	{
		printk("[Touch_H][TOUCH_ERR]Himax IC package incorrect!!\n");

		return -1;
	}
}
static int himax_ts_poweron(struct himax_ts_data *ts_modify);
static int himax_resume(void);
int himax_ts_suspend(void)
{
	struct himax_ts_data *ts_modify;
	uint8_t buf[2] = { 0 };

	ts_modify = private_ts;

	if(ts_suspend)
	{
		if(touch_debug_mask & SUS_INFO)
			printk("[Touch_H] %s reject the suspend action\n",__func__);
		return 0;
	}

#ifdef HX_TP_SYS_FLASH_DUMP
	if(getFlashDumpGoing())
	{
		printk("[Touch_H] %s: Flash dump is going, reject suspend\n",__func__);
		return 0;
	}
#endif

	printk("[Touch_H] %s: TS suspend\n", __func__);

	//Wakelock Protect Start
	wake_lock(&ts_modify->wake_lock);
	//Wakelock Protect End

	//Mutexlock Protect Start
	mutex_lock(&ts_modify->mutex_lock);
	//Mutexlock Protect End

	input_report_key(private_ts->input_dev, BTN_TOUCH, 0);
	input_mt_sync(private_ts->input_dev);
	input_sync(private_ts->input_dev);


	buf[0] = HX_CMD_TSSOFF;
	if(i2c_himax_master_write(ts_modify->client, buf, 1, DEFAULT_RETRY_CNT) < 0)
	{
		printk("[Touch_H][TOUCH_ERR] %s: I2C access failed addr = 0x%x\n", 
			__func__, ts_modify->client->addr);
	}
	msleep(120);

	buf[0] = HX_CMD_TSSLPIN;
	if(i2c_himax_master_write(ts_modify->client, buf, 1, DEFAULT_RETRY_CNT) < 0)
	{
		printk("[Touch_H][TOUCH_ERR] %s: I2C access failed addr = 0x%x\n", 
			__func__, ts_modify->client->addr);
	}
	msleep(120);

	buf[0] = HX_CMD_SETDEEPSTB;
	buf[1] = 0x01;
	if(i2c_himax_master_write(ts_modify->client, buf, 2, DEFAULT_RETRY_CNT) < 0)
	{
		printk("[Touch_H][TOUCH_ERR] %s: I2C access failed addr = 0x%x\n", 
			__func__, ts_modify->client->addr);
	}
	msleep(120);

	//Mutexlock Protect Start
	mutex_unlock(&ts_modify->mutex_lock);
	//Mutexlock Protect End

	//Wakelock Protect Start
	wake_unlock(&ts_modify->wake_lock);
	//Wakelock Protect End

// 	disable_irq(client->irq);
	hx_irq_disable(private_ts);

	if(cancel_work_sync(&ts_modify->work))
	{
// 		enable_irq(client->irq);
		hx_irq_enable(private_ts);
	}

	ts_suspend = 1;
	
#ifdef CONFIG_TOUCHSCREEN_DOUBLETAP2WAKE
	if (dt2w_switch > 0) {
		wake_lock(&ts_modify->wake_lock);
		printk("[Touch_H] %s dt2w enabled. Re-enabe touch! \n",__func__);
		ts_suspend = true;
		buf[0] = HX_CMD_SETDEEPSTB;
		buf[1] = 0x00;
		if(i2c_himax_master_write(ts_modify->client, buf, 2, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s: I2C access failed addr = 0x%x\n", 
				__func__, ts_modify->client->addr);
		}
		udelay(100);
		wake_unlock(&ts_modify->wake_lock);
		himax_ts_poweron(ts_modify);
		hx_irq_enable(private_ts);

		enable_irq_wake(ts_modify->client->irq);
	}
#endif

	return 0;
}
EXPORT_SYMBOL(himax_ts_suspend);

int himax_ts_resume(void)
{
#ifdef CONFIG_TOUCHSCREEN_DOUBLETAP2WAKE
	if (dt2w_switch > 0) {
		printk("[Touch_H] %s Prevent suspend while dt2w enabled \n",__func__);
		ts_suspend = false;
		disable_irq_wake(private_ts->client->irq);
		return 0;
	} else
#endif
	{
	if(delayed_work_pending(&hx_resume_w)) {
		cancel_delayed_work_sync(&hx_resume_w);
		printk("[Touch_H] cancel last resume_work\n");
	}
	queue_delayed_work(hx_attach_detach_wq, &hx_resume_w, msecs_to_jiffies(2));
	}
	return 0;
}
EXPORT_SYMBOL(himax_ts_resume);

static int himax_resume(void)
{
	struct himax_ts_data *ts_modify;
	uint8_t buf[2] = { 0 };

	ts_modify = private_ts;
#ifdef CONFIG_TOUCHSCREEN_DOUBLETAP2WAKE
	if (dt2w_switch > 0) {
		printk("[Touch_H] %s dt2w enabled, skip resume \n",__func__);
		ts_suspend = false;
		return 0;
	}
#endif


	if(!ts_suspend)
	{
		if(touch_debug_mask & SUS_INFO)
			printk("[Touch_H] %s TP never enter suspend , reject the resume action\n",__func__);
		return 0;
	}

	if(ts_inpad)
	{
		if(touch_debug_mask & SUS_INFO)
			printk("[Touch_H] %s in pad now , reject the resume action\n",__func__);
		return 0;
	}

	printk("[Touch_H] %s: TS resume\n", __func__);

	//Wakelock Protect Start
	wake_lock(&ts_modify->wake_lock);
	//Wakelock Protect End

	buf[0] = HX_CMD_SETDEEPSTB;
	buf[1] = 0x00;
	if(i2c_himax_master_write(ts_modify->client, buf, 2, DEFAULT_RETRY_CNT) < 0)
	{
		printk("[Touch_H][TOUCH_ERR] %s: I2C access failed addr = 0x%x\n", 
			__func__, ts_modify->client->addr);
	}
	udelay(100);

	//Wakelock Protect Start
	wake_unlock(&ts_modify->wake_lock);
	//Wakelock Protect End

	himax_ts_poweron(ts_modify);
// 	enable_irq(client->irq);
	hx_irq_enable(private_ts);

	ts_suspend = 0;

	return 0;
}
//----[ normal function]----------------------------------------------------------------------------------end

//----[ i2c read/write function]------------------------------------------------------------------------start
static int i2c_himax_read(struct i2c_client *client, uint8_t command, uint8_t *data, uint8_t length, uint8_t toRetry)
{
	int retry;
	
	struct i2c_msg msg[] = 
	{
		{	.addr = client->addr,
			.flags = 0, .len = 1, 
			.buf = &command, 
		},
		{	.addr = client->addr, 
			.flags = I2C_M_RD, 
			.len = length, .buf = data, 
		} 
	};
	
	for (retry = 0; retry < toRetry; retry++)
	{
		if (i2c_transfer(client->adapter, msg, 2) == 2)
		{
			break;
		}
		msleep(10);
	}
	if (retry == toRetry)
	{
		printk("[Touch_H][TOUCH_ERR] %s: i2c_read_block retry over %d\n", __func__, toRetry);
		return -EIO;
	}
	return 0;
}

static int i2c_himax_write(struct i2c_client *client, uint8_t command, uint8_t *data, uint8_t length, uint8_t toRetry)
{
	int retry, loop_i;
	uint8_t *buf = kzalloc(sizeof(uint8_t) * (length + 1), GFP_KERNEL);
	
	struct i2c_msg msg[] =
	{
		{ 	.addr = client->addr, 
			.flags = 0, .len = length + 1, 
			.buf = buf, 
		} 
	};
	
	buf[0] = command;
	for (loop_i = 0; loop_i < length; loop_i++)
	{
		buf[loop_i + 1] = data[loop_i];
	}
	for (retry = 0; retry < toRetry; retry++)
	{
		if (i2c_transfer(client->adapter, msg, 1) == 1)
		{
			break;
		}
		msleep(10);
	}
	
	if (retry == toRetry)
	{
		printk("[Touch_H][TOUCH_ERR] %s: i2c_write_block retry over %d\n", __func__, toRetry);
		kfree(buf);
		return -EIO;
	}
	kfree(buf);
	return 0;
}

static int i2c_himax_write_command(struct i2c_client *client, uint8_t command, uint8_t toRetry)
{
	return i2c_himax_write(client, command, NULL, 0, toRetry);
}

int i2c_himax_master_write(struct i2c_client *client, uint8_t *data, uint8_t length, uint8_t toRetry)
{
	int retry, loop_i;
	uint8_t *buf = kzalloc(sizeof(uint8_t) * length, GFP_KERNEL);
	
	struct i2c_msg msg[] =
	{
		{	.addr = client->addr, 
			.flags = 0, .len = length, 
			.buf = buf, 
		} 
	};
	
	for (loop_i = 0; loop_i < length; loop_i++)
	{
		buf[loop_i] = data[loop_i];
	}
	for (retry = 0; retry < toRetry; retry++)
	{
		if (i2c_transfer(client->adapter, msg, 1) == 1)
		{
			break;
		}
		msleep(10);
	}
	
	if (retry == toRetry)
	{
		printk("[Touch_H][TOUCH_ERR] %s: i2c_write_block retry over %d\n", __func__, toRetry);
		kfree(buf);
		return -EIO;
	}
	kfree(buf);
	return 0;
}
//----[ i2c read/write function]--------------------------------------------------------------------------end


void hx_nosync_irq_disable(struct himax_ts_data *ts)
{
    unsigned long irqflags;

    spin_lock_irqsave(&ts->irq_lock, irqflags);
    if (!ts->irq_is_disable)
    {
        ts->irq_is_disable = 1; 
        disable_irq_nosync(ts->client->irq);
    }
    spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

void hx_irq_disable(struct himax_ts_data *ts)
{
    unsigned long irqflags;

    spin_lock_irqsave(&ts->irq_lock, irqflags);
    if (!ts->irq_is_disable)
    {
        ts->irq_is_disable = 1; 
        disable_irq(ts->client->irq);
    }
    spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}


void hx_irq_enable(struct himax_ts_data *ts)
{
    unsigned long irqflags = 0;

#ifdef ASUS_FACTORY_BUILD
    if(common_fw)
	return;
#endif

    spin_lock_irqsave(&ts->irq_lock, irqflags);
    if (ts->irq_is_disable) 
    {
        enable_irq(ts->client->irq);
        ts->irq_is_disable = 0; 
    }
    spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}




//----[ register flow function]-------------------------------------------------------------------------start
static int himax_ts_poweron(struct himax_ts_data *ts_modify)
{
	uint8_t buf0[11];

	//Wakelock Protect Start
	wake_lock(&ts_modify->wake_lock);
	//Wakelock Protect End

	//Mutexlock Protect Start
	mutex_lock(&ts_modify->mutex_lock);
	//Mutexlock Protect End

	if (IC_TYPE == HX_85XX_D_SERIES_PWON)
	{
		buf0[0] = HX_CMD_MANUALMODE;	//0x42
		buf0[1] = 0x02;
		//Reload Disable
		if(i2c_himax_master_write(ts_modify->client, buf0, 2, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H]i2c_master_send failed addr = 0x%x\n",
				ts_modify->client->addr);
			goto send_i2c_msg_fail;
		}
		udelay(100);

		buf0[0] = HX_CMD_SETROMRDY;	//0x36
		buf0[1] = 0x0F;
		buf0[2] = 0x53;
		//enable flash
		if(i2c_himax_master_write(ts_modify->client, buf0, 3, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H]i2c_master_send failed addr = 0x%x\n",
				ts_modify->client->addr);
			goto send_i2c_msg_fail;
		}
		udelay(100);

		buf0[0] = HX_CMD_SET_CACHE_FUN;	//0xDD
		buf0[1] = 0x04;
		buf0[2] = 0x03;
		if(i2c_himax_master_write(ts_modify->client, buf0, 3, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H]i2c_master_send failed addr = 0x%x\n",
				ts_modify->client->addr);
			goto send_i2c_msg_fail;
		}
		udelay(100);

		buf0[0] = HX_CMD_B9;	//setCVDD
		buf0[1] = 0x01;
		buf0[2] = 0x36;
		if(i2c_himax_master_write(ts_modify->client, buf0, 3, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H]i2c_master_send failed addr = 0x%x\n",
				ts_modify->client->addr);
			goto send_i2c_msg_fail;
		}
		udelay(100);

		buf0[0] = HX_CMD_CB;
		buf0[1] = 0x01;
		buf0[2] = 0xF5;
		if(i2c_himax_master_write(ts_modify->client, buf0, 3, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H]i2c_master_send failed addr = 0x%x\n",
				ts_modify->client->addr);
			goto send_i2c_msg_fail;
		}
		udelay(100);


		buf0[0] = HX_CMD_TSSON;
		//sense on
		if(i2c_himax_master_write(ts_modify->client, buf0, 1, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H]i2c_master_send failed addr = 0x%x\n",
				ts_modify->client->addr);
			goto send_i2c_msg_fail;
		}
		msleep(120); //120ms

		buf0[0] = HX_CMD_TSSLPOUT;	//0x81
		//sense on
		if(i2c_himax_master_write(ts_modify->client, buf0, 1, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H]i2c_master_send failed addr = 0x%x\n",
				ts_modify->client->addr);
			goto send_i2c_msg_fail;
		}
		msleep(120); //120ms
	}

	//Mutexlock Protect Start
	mutex_unlock(&ts_modify->mutex_lock);
	//Mutexlock Protect End

	//Wakelock Protect Start
	wake_unlock(&ts_modify->wake_lock);
	//Wakelock Protect End

	return 0;

send_i2c_msg_fail:

	printk("[Touch_H][TOUCH_ERR ]send_i2c_msg_failline: %d \n", __LINE__);

	//Mutexlock Protect Start
	mutex_unlock(&ts_modify->mutex_lock);
	//Mutexlock Protect End

	//Wakelock Protect Start
	wake_unlock(&ts_modify->wake_lock);
	//Wakelock Protect End

//----[ENABLE_CHIP_RESET_MACHINE]-----------------------------------------------------------------start
#ifdef ENABLE_CHIP_RESET_MACHINE
	if (private_ts->init_success)
	{
		queue_delayed_work(private_ts->himax_wq, &private_ts->himax_chip_reset_work, 0);
	}
#endif
//----[ENABLE_CHIP_RESET_MACHINE]-------------------------------------------------------------------end
	return -1;
}

static int himax_ManualMode(int enter)
{
	uint8_t cmd[2];
	cmd[0] = enter;
	if (i2c_himax_write(private_ts->client, 0x42, &cmd[0], 1, DEFAULT_RETRY_CNT) < 0)
	{
		printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
		return 0;
	}
	return 0;
}

static int himax_FlashMode(int enter)
{
	uint8_t cmd[2];
	cmd[0] = enter;
	if (i2c_himax_write(private_ts->client, 0x43, &cmd[0], 1, DEFAULT_RETRY_CNT) < 0)
	{
		printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
		return 0;
	}
	return 0;
}

static int himax_lock_flash(void)
{
	uint8_t cmd[5];
	
	/* lock sequence start */
	cmd[0] = 0x01;
	cmd[1] = 0x00;
	cmd[2] = 0x06;
	if (i2c_himax_write(private_ts->client, 0x43, &cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
	{
		printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
		return 0;
	}
	
	cmd[0] = 0x03;
	cmd[1] = 0x00;
	cmd[2] = 0x00;
	if (i2c_himax_write(private_ts->client, 0x44, &cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
	{
		printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
		return 0;
	}
	
	cmd[0] = 0x00;
	cmd[1] = 0x00;
	cmd[2] = 0x7D;
	cmd[3] = 0x03;
	if (i2c_himax_write(private_ts->client, 0x45, &cmd[0], 4, DEFAULT_RETRY_CNT) < 0)
	{
		printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
		return 0;
	}
	
	if (i2c_himax_write_command(private_ts->client, 0x4A, DEFAULT_RETRY_CNT) < 0)
	{
		printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
		return 0;
	}
	mdelay(50);
	return 0;
	/* lock sequence stop */
}
	
static int himax_unlock_flash(void)
{
	uint8_t cmd[5];
	
	/* unlock sequence start */
	cmd[0] = 0x01;
	cmd[1] = 0x00;
	cmd[2] = 0x06;
	if (i2c_himax_write(private_ts->client, 0x43, &cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
	{
		printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
		return 0;
	}
	
	cmd[0] = 0x03;
	cmd[1] = 0x00;
	cmd[2] = 0x00;
	if (i2c_himax_write(private_ts->client, 0x44, &cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
	{
		printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
		return 0;
	}
	
	cmd[0] = 0x00;
	cmd[1] = 0x00;
	cmd[2] = 0x3D;
	cmd[3] = 0x03;
	if (i2c_himax_write(private_ts->client, 0x45, &cmd[0], 4, DEFAULT_RETRY_CNT) < 0)
	{
		printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
		return 0;
	}
	
	if (i2c_himax_write_command(private_ts->client, 0x4A, DEFAULT_RETRY_CNT) < 0)
	{
		printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
		return 0;
	}
	mdelay(50);
	
	return 0;
	/* unlock sequence stop */
}

static uint8_t himax_calculateChecksum(char *ImageBuffer, int fullLength) //, int address, int RST)
{
//----[ HX_TP_BIN_CHECKSUM_SW]----------------------------------------------------------------------start
	if (IC_CHECKSUM == HX_TP_BIN_CHECKSUM_SW)
	{
		u16 checksum = 0;
		uint8_t cmd[5], last_byte;
		int FileLength, i, readLen, k, lastLength;
	
		FileLength = fullLength - 2;
		memset(cmd, 0x00, sizeof(cmd));
	
		himax_FlashMode(1);
	
		FileLength = (FileLength + 3) / 4;
		for (i = 0; i < FileLength; i++)
		{
			last_byte = 0;
			readLen = 0;
	
			cmd[0] = i & 0x1F;
			if (cmd[0] == 0x1F || i == FileLength - 1)
				last_byte = 1;
			cmd[1] = (i >> 5) & 0x1F;
			cmd[2] = (i >> 10) & 0x1F;
			if (i2c_himax_write(private_ts->client, 0x44, &cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
			{
				printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
				return 0;
			}
	
			if (i2c_himax_write_command(private_ts->client, 0x46, DEFAULT_RETRY_CNT) < 0)
			{
				printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
				return 0;
			}
	
			if (i2c_himax_read(private_ts->client, 0x59, cmd, 4, DEFAULT_RETRY_CNT) < 0)
			{
				printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
				return -1;
			}
	
			if (i < (FileLength - 1))
			{
				checksum += cmd[0] + cmd[1] + cmd[2] + cmd[3];
				if (i == 0)
				{
					printk(	"[Touch_H] %s: himax_marked cmd 0 to 3 (first 4 bytes): %d, %d, %d, %d\n", __func__, cmd[0], cmd[1], cmd[2], cmd[3]);
				}
			}
			else
			{
				printk("[Touch_H] %s: himax_marked cmd 0 to 3 (last 4 bytes): %d, %d, %d, %d\n",
					__func__, cmd[0], cmd[1], cmd[2], cmd[3]);
				printk("[Touch_H] %s: himax_marked, checksum (not last): %d\n",
					__func__, checksum);
				lastLength = (((fullLength - 2) % 4) > 0) ? ((fullLength - 2) % 4) : 4;
	
				for (k = 0; k < lastLength; k++)
				{
					checksum += cmd[k];
				}
				printk("[Touch_H] %s: himax_marked, checksum (final): %d\n", __func__, checksum);
	
				//Check Success
				if (ImageBuffer[fullLength - 1] == (u8)(0xFF & (checksum >> 8))
						&& ImageBuffer[fullLength - 2] == (u8)(0xFF & checksum))
				{
					himax_FlashMode(0);
					return 1;
				}
				else //Check Fail
				{
					himax_FlashMode(0);
					return 0;
				}
			}
		}
	}
	else if (IC_CHECKSUM == HX_TP_BIN_CHECKSUM_HW)
	{
		u32 sw_checksum = 0;
		u32 hw_checksum = 0;
		uint8_t cmd[5], last_byte;
		int FileLength, i, readLen, k, lastLength;
	
		FileLength = fullLength;
		memset(cmd, 0x00, sizeof(cmd));
	
		himax_FlashMode(1);
	
		FileLength = (FileLength + 3) / 4;
		for (i = 0; i < FileLength; i++)
		{
			last_byte = 0;
			readLen = 0;
	
			cmd[0] = i & 0x1F;
			if (cmd[0] == 0x1F || i == FileLength - 1)
			{
				last_byte = 1;
			}
			cmd[1] = (i >> 5) & 0x1F;
			cmd[2] = (i >> 10) & 0x1F;
			if (i2c_himax_write(private_ts->client, 0x44, &cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
			{
				printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
				return 0;
			}
	
			if (i2c_himax_write_command(private_ts->client, 0x46, DEFAULT_RETRY_CNT) < 0)
			{
				printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
				return 0;
			}
	
			if (i2c_himax_read(private_ts->client, 0x59, cmd, 4, DEFAULT_RETRY_CNT) < 0)
			{
				printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
				return -1;
			}
	
			if (i < (FileLength - 1))
			{
				sw_checksum += cmd[0] + cmd[1] + cmd[2] + cmd[3];
				if (i == 0)
				{
					printk("[Touch_H] %s: himax_marked cmd 0 to 3 (first 4 bytes): %d, %d, %d, %d\n", __func__, cmd[0], cmd[1], cmd[2], cmd[3]);
				}
			}
			else
			{
				printk("[Touch_H] %s: himax_marked cmd 0 to 3 (last 4 bytes): %d, %d, %d, %d\n",
						__func__, cmd[0], cmd[1], cmd[2], cmd[3]);
				printk("[Touch_H] %s: himax_marked, sw_checksum (not last): %d\n",
						__func__, sw_checksum);
	
				lastLength = ((fullLength % 4) > 0) ? (fullLength % 4) : 4;
	
				for (k = 0; k < lastLength; k++)
				{
					sw_checksum += cmd[k];
				}
				printk("[Touch_H] %s: himax_marked, sw_checksum (final): %d\n",
						__func__, sw_checksum);
	
				//Enable HW Checksum function.
				cmd[0] = 0x01;
				if (i2c_himax_write(private_ts->client, 0xE5, &cmd[0], 1, DEFAULT_RETRY_CNT) < 0)
				{
					printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
					return 0;
				}
	
				//Must sleep 5 ms.
				msleep(30);
	
				//Get HW Checksum.
				if (i2c_himax_read(private_ts->client, 0xAD, cmd, 4, DEFAULT_RETRY_CNT) < 0)
				{
					printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
					return -1;
				}
	
				hw_checksum = cmd[0] + cmd[1] * 0x100 + cmd[2] * 0x10000 + cmd[3] * 1000000;
				printk("[Touch_H] %s: himax_marked, sw_checksum (final): %d\n",
						__func__, sw_checksum);
				printk("[Touch_H] %s: himax_marked, hw_checkusm (final): %d\n",
						__func__, hw_checksum);
	
				//Compare the checksum.
				if (hw_checksum == sw_checksum)
				{
					himax_FlashMode(0);
					return 1;
				}
				else
				{
					himax_FlashMode(0);
					return 0;
				}
			}
		}
	}
	else if (IC_CHECKSUM == HX_TP_BIN_CHECKSUM_CRC)
	{
		uint8_t cmd[5];
	
		//Set Flash Clock Rate
		if (i2c_himax_read(private_ts->client, 0x7F, cmd, 5, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
			return -1;
		}
		cmd[3] = 0x02;
	
		if (i2c_himax_write(private_ts->client, 0x7F, &cmd[0], 5, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
			return 0;
		}
	
		//Enable Flash
		himax_FlashMode(1);
	
		//Select CRC Mode
		cmd[0] = 0x05;
		cmd[1] = 0x00;
		cmd[2] = 0x00;
		if (i2c_himax_write(private_ts->client, 0xD2, &cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
			return 0;
		}
	
		//Enable CRC Function
		cmd[0] = 0x01;
		if (i2c_himax_write(private_ts->client, 0xE5, &cmd[0], 1, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
			return 0;
		}
	
		//Must delay 30 ms
		msleep(30);
	
		//Read HW CRC
		if (i2c_himax_read(private_ts->client, 0xAD, cmd, 4, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
			return -1;
		}
	
		if (cmd[0] == 0 && cmd[1] == 0 && cmd[2] == 0 && cmd[3] == 0)
		{
			himax_FlashMode(0);
			return 1;
		}
		else
		{
			himax_FlashMode(0);
			return 0;
		}
	}
	return 0;
}

//----[HX_RST_PIN_FUNC]-------------------------------------------------------------------------------start
#ifdef HX_RST_PIN_FUNC
static void himax_HW_reset(void)
{
	gpio_set_value(private_ts->rst_gpio, 0);
	msleep(100);
	gpio_set_value(private_ts->rst_gpio, 1);
	msleep(100);
}
#endif
//----[HX_RST_PIN_FUNC]---------------------------------------------------------------------------------end
//----[ register flow function]---------------------------------------------------------------------------end

int himax_hang_shaking(void)    //0:Running, 1:Stop, 2:I2C Fail
{
	int result;
	uint8_t hw_reset_check[1];
	uint8_t hw_reset_check_2[1];
	uint8_t buf0[2];
	
	//Mutexlock Protect Start
	mutex_lock(&private_ts->mutex_lock);
	//Mutexlock Protect End
	
	//Write 0x92
	buf0[0] = 0x92;
	if (IC_STATUS_CHECK == 0xAA)
	{
		buf0[1] = 0xAA;
		IC_STATUS_CHECK = 0x55;
	}
	else
	{
		buf0[1] = 0x55;
		IC_STATUS_CHECK = 0xAA;
	}
	
	if(i2c_himax_master_write(private_ts->client, buf0, 2, DEFAULT_RETRY_CNT) < 0)
	{
		printk("[Touch_H]:write 0x92 failed line: %d \n", __LINE__);
		goto hang_shaking_i2c_msg_fail;
	}
	msleep(50); /* Must more than 1 frame */
	
	buf0[0] = 0x92;
	buf0[1] = 0x00;
	if(i2c_himax_master_write(private_ts->client, buf0, 2, DEFAULT_RETRY_CNT) < 0)
	{
		printk("[Touch_H]:write 0x92 failed line: %d \n", __LINE__);
		goto hang_shaking_i2c_msg_fail;
	}
	msleep(2);
	
	if(i2c_himax_read(private_ts->client, 0xDA, hw_reset_check, 1, DEFAULT_RETRY_CNT) < 0)
	{
		printk("[Touch_H]:i2c_himax_read 0xDA failed line: %d \n", __LINE__);
		goto hang_shaking_i2c_msg_fail;
	}
	//printk("[Touch_H]: ESD 0xDA - 0x%x.\n", hw_reset_check[0]);
	
	if ((IC_STATUS_CHECK != hw_reset_check[0]))
	{
		msleep(2);
		if(i2c_himax_read(private_ts->client, 0xDA, hw_reset_check_2, 1, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H]:i2c_himax_read 0xDA failed line: %d \n", __LINE__);
			goto hang_shaking_i2c_msg_fail;
		}
		//printk("[Touch_H]: ESD check 2 0xDA - 0x%x.\n", hw_reset_check_2[0]);
	
		if (hw_reset_check[0] == hw_reset_check_2[0])
		{
			result = 1; //MCU Stop
		}
		else
		{
			result = 0; //MCU Running
		}
	}
	else
	{
		result = 0; //MCU Running
	}
	
	//Mutexlock Protect Start
	mutex_unlock(&private_ts->mutex_lock);
	//Mutexlock Protect End
	return result;
	
hang_shaking_i2c_msg_fail:
	//Mutexlock Protect Start
	mutex_unlock(&private_ts->mutex_lock);
	//Mutexlock Protect End
	return 2;
}

 //----[ENABLE_CHIP_RESET_MACHINE]---------------------------------------------------------------------start
#ifdef ENABLE_CHIP_RESET_MACHINE
static void himax_chip_reset_function(struct work_struct *dat)
{
	printk("[Touch_H]:himax_chip_reset_function ++ \n"); //debug msg
	
	//Wakelock Protect start
	wake_lock(&private_ts->wake_lock);
	//Wakelock Protect end

	//Mutexlock Protect Start
	mutex_lock(&private_ts->mutex_lock);
	//Mutexlock Protect End

	gpio_set_value(private_ts->rst_gpio, 0);
	msleep(30);
	gpio_set_value(private_ts->rst_gpio, 1);
	msleep(30);

	//Mutexlock Protect Start
	mutex_unlock(&private_ts->mutex_lock);
	//Mutexlock Protect End

	himax_ts_poweron(private_ts);

	//Wakelock Protect start
	wake_unlock(&private_ts->wake_lock);
	//Wakelock Protect end

	if (gpio_get_value(private_ts->intr_gpio) == 0)
	{
		printk("[Touch_H] %s: Enable IRQ\n", __func__);
// 			enable_irq(private_ts->client->irq);
			hx_irq_enable(private_ts);
	}

	printk("[Touch_H] %s --\n",__func__); //debug msg
}
#endif
 //----[ENABLE_CHIP_RESET_MACHINE]-----------------------------------------------------------------------end
/*
static int himax_read_flash(unsigned char *buf, unsigned int addr_start, unsigned int length) //OK
{
	u16 i = 0;
	u16 j = 0;
	u16 k = 0;
	uint8_t cmd[4];
	u16 local_start_addr = addr_start / 4;
	u16 local_length = length;
	u16 local_end_addr = (addr_start + length) / 4 + 1;
	u16 local_addr = addr_start % 4;
	printk(	"[Touch_H]Himax %s addr_start = %d , local_start_addr = %d , local_length = %d , local_end_addr = %d , local_addr = %d \n", __func__, addr_start, local_start_addr, local_length, local_end_addr, local_addr);

	if (i2c_himax_write_command(private_ts->client, 0x81, DEFAULT_RETRY_CNT) < 0)
	{
		printk("[Touch_H][TOUCH_ERR] %s i2c write 81 fail.\n", __func__);
		return 0;
	}
	msleep(120);

	if (i2c_himax_write_command(private_ts->client, 0x82, DEFAULT_RETRY_CNT) < 0)
	{
		printk("[Touch_H][TOUCH_ERR] %s i2c write 82 fail.\n", __func__);
		return 0;
	}
	msleep(100);

	cmd[0] = 0x01;
	if (i2c_himax_write(private_ts->client, 0x43, &cmd[0], 1, DEFAULT_RETRY_CNT) < 0)
	{
		printk("[Touch_H][TOUCH_ERR] %s i2c write 43 fail.\n", __func__);
		return 0;
	}
	msleep(100);
	i = local_start_addr;
	do
	{
		cmd[0] = i & 0x1F;
		cmd[1] = (i >> 5) & 0x1F;
		cmd[2] = (i >> 10) & 0x1F;
		if (i2c_himax_write(private_ts->client, 0x44, &cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
			return 0;
		}
		if (i2c_himax_write(private_ts->client, 0x46, &cmd[0], 0, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
			return 0;
		}
		if (i2c_himax_read(private_ts->client, 0x59, cmd, 4, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
			return 0;
		}
		printk("[Touch_H]Himax cmd[0]=%d,cmd[1]=%d,cmd[2]=%d,cmd[3]=%d\n", cmd[0], cmd[1], cmd[2], cmd[3]);
		if (i == local_start_addr) //first page
		{
			j = 0;
			for (k = local_addr; k < 4 && j < local_length; k++)
			{
				buf[j++] = cmd[k];
			}
		}
		else //other page
		{
			for (k = 0; k < 4 && j < local_length; k++)
			{
				buf[j++] = cmd[k];
			}
		}
		i++;
	} while (i < local_end_addr);

	cmd[0] = 0;

	if (i2c_himax_write(private_ts->client, 0x43, &cmd[0], 1, DEFAULT_RETRY_CNT) < 0)
	{
		return 0;
	}

	return 1;
}
*/
 //----[firmware version read]---------------------------------------------------------------------------start
static u8 himax_read_FW_ver(void)
{
	u16 fw_ver_maj_start_addr;
	u16 fw_ver_maj_end_addr;
	u16 fw_ver_maj_addr;
	u16 fw_ver_maj_length;
	
	u16 fw_ver_min_start_addr;
	u16 fw_ver_min_end_addr;
	u16 fw_ver_min_addr;
	u16 fw_ver_min_length;
	
	u16 cfg_ver_maj_start_addr;
	u16 cfg_ver_maj_end_addr;
	u16 cfg_ver_maj_addr;
	u16 cfg_ver_maj_length;
	
	u16 cfg_ver_min_start_addr;
	u16 cfg_ver_min_end_addr;
	u16 cfg_ver_min_addr;
	u16 cfg_ver_min_length;
	
	uint8_t cmd[3];
	u16 i = 0;
	u16 j = 0;
	u16 k = 0;
	
	fw_ver_maj_start_addr = FW_VER_MAJ_FLASH_ADDR / 4; 
	// start addr = 133 / 4 = 33
	fw_ver_maj_length = FW_VER_MAJ_FLASH_LENG;	
	// length = 1
	fw_ver_maj_end_addr = (FW_VER_MAJ_FLASH_ADDR + fw_ver_maj_length) / 4 + 1;
	// end addr = 134 / 4 = 33
	fw_ver_maj_addr = FW_VER_MAJ_FLASH_ADDR % 4;	
	// 133 mod 4 = 1
	
	fw_ver_min_start_addr = FW_VER_MIN_FLASH_ADDR / 4;	
	// start addr = 134 / 4 = 33
	fw_ver_min_length = FW_VER_MIN_FLASH_LENG;	
	// length = 1
	fw_ver_min_end_addr = (FW_VER_MIN_FLASH_ADDR + fw_ver_min_length) / 4 + 1;
	// end addr = 135 / 4 = 33
	fw_ver_min_addr = FW_VER_MIN_FLASH_ADDR % 4;	
	// 134 mod 4 = 2
	
	cfg_ver_maj_start_addr = CFG_VER_MAJ_FLASH_ADDR / 4;
	// start addr = 160 / 4 = 40
	cfg_ver_maj_length = CFG_VER_MAJ_FLASH_LENG;	
	// length = 12
	cfg_ver_maj_end_addr = (CFG_VER_MAJ_FLASH_ADDR + cfg_ver_maj_length) / 4 + 1;
	// end addr = (160 + 12) / 4 = 43
	cfg_ver_maj_addr = CFG_VER_MAJ_FLASH_ADDR % 4;					
	// 160 mod 4 = 0
	
	cfg_ver_min_start_addr = CFG_VER_MIN_FLASH_ADDR / 4;
	// start addr = 172 / 4 = 43
	cfg_ver_min_length = CFG_VER_MIN_FLASH_LENG;					
	// length = 12
	cfg_ver_min_end_addr = (CFG_VER_MIN_FLASH_ADDR + cfg_ver_min_length) / 4 + 1;
	// end addr = (172 + 12) / 4 = 46
	cfg_ver_min_addr = CFG_VER_MIN_FLASH_ADDR % 4;					
	// 172 mod 4 = 0

// 	disable_irq(private_ts->client->irq);
	hx_irq_disable(private_ts);
#ifdef HX_RST_PIN_FUNC
	himax_HW_reset();
#endif

	mutex_lock(&private_ts->mutex_lock);

	//Sleep out
	if (i2c_himax_write(private_ts->client, 0x81, &cmd[0], 0, DEFAULT_RETRY_CNT) < 0)
	{
		printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
		return 0;
	}
	mdelay(120);
	
	//Enter flash mode
	himax_FlashMode(1);
	
	//Read Flash Start
	//FW Version MAJ
	i = fw_ver_maj_start_addr;
	do
	{
		cmd[0] = i & 0x1F;		//column = 33 mod 32 = 1
		cmd[1] = (i >> 5) & 0x1F;	//page = 33 / 32 = 1
		cmd[2] = (i >> 10) & 0x1F;	//sector = 33 / 1024 = 0
	
		if (i2c_himax_write(private_ts->client, 0x44, &cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
			return 0;
		}
		if (i2c_himax_write(private_ts->client, 0x46, &cmd[0], 0, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
			return 0;
		}
		if (i2c_himax_read(private_ts->client, 0x59, cmd, 4, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
			return 0;
		}
	
		if (i == fw_ver_maj_start_addr) //first page
		{
			j = 0;
			for (k = fw_ver_maj_addr; k < 4 && j < fw_ver_maj_length; k++)
			{
				FW_VER_MAJ_buff[j++] = cmd[k];
			}
		}
		else //other page
		{
			for (k = 0; k < 4 && j < fw_ver_maj_length; k++)
			{
				FW_VER_MAJ_buff[j++] = cmd[k];
			}
		}
		i++;
	} while (i < fw_ver_maj_end_addr);
	
	//FW Version MIN
	i = fw_ver_min_start_addr;
	do
	{
		cmd[0] = i & 0x1F;		//column = 33 mod 32 = 1
		cmd[1] = (i >> 5) & 0x1F;	//page	= 33 / 32 = 1
		cmd[2] = (i >> 10) & 0x1F;	//sector = 33 / 1024 = 0
	
		if (i2c_himax_write(private_ts->client, 0x44, &cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
			return 0;
		}
		if (i2c_himax_write(private_ts->client, 0x46, &cmd[0], 0, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
			return 0;
		}
		if (i2c_himax_read(private_ts->client, 0x59, cmd, 4, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
			return 0;
		}
	
		if (i == fw_ver_min_start_addr) //first page
		{
			j = 0;
			for (k = fw_ver_min_addr; k < 4 && j < fw_ver_min_length; k++)
			{
				FW_VER_MIN_buff[j++] = cmd[k];
			}
		}
		else //other page
		{
			for (k = 0; k < 4 && j < fw_ver_min_length; k++)
			{
				FW_VER_MIN_buff[j++] = cmd[k];
			}
		}
		i++;
	} while (i < fw_ver_min_end_addr);
	
	//CFG Version MAJ
	i = cfg_ver_maj_start_addr;
	do
	{
		cmd[0] = i & 0x1F;		//column = 40 mod 32 = 8
		cmd[1] = (i >> 5) & 0x1F;	//page = 40 / 32 = 1
		cmd[2] = (i >> 10) & 0x1F;	//sector = 40 / 1024 = 0
	
		if (i2c_himax_write(private_ts->client, 0x44, &cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
			return 0;
		}
		if (i2c_himax_write(private_ts->client, 0x46, &cmd[0], 0, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
			return 0;
		}
		if (i2c_himax_read(private_ts->client, 0x59, cmd, 4, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
			return 0;
		}
	
		if (i == cfg_ver_maj_start_addr) //first page
		{
			j = 0;
			for (k = cfg_ver_maj_addr; k < 4 && j < cfg_ver_maj_length; k++)
			{
				CFG_VER_MAJ_buff[j++] = cmd[k];
			}
		}
		else //other page
		{
			for (k = 0; k < 4 && j < cfg_ver_maj_length; k++)
			{
				CFG_VER_MAJ_buff[j++] = cmd[k];
			}
		}
		i++;
	} while (i < cfg_ver_maj_end_addr);
	
	//CFG Version MIN
	i = cfg_ver_min_start_addr;
	do
	{
		cmd[0] = i & 0x1F;		//column = 43 mod 32 = 11
		cmd[1] = (i >> 5) & 0x1F;	//page = 43 / 32 = 1
		cmd[2] = (i >> 10) & 0x1F;	//sector = 43 / 1024 = 0
	
		if (i2c_himax_write(private_ts->client, 0x44, &cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
			return 0;
		}
		if (i2c_himax_write(private_ts->client, 0x46, &cmd[0], 0, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
			return 0;
		}
		if (i2c_himax_read(private_ts->client, 0x59, cmd, 4, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
			return 0;
		}
	
		if (i == cfg_ver_min_start_addr) //first page
		{
			j = 0;
			for (k = cfg_ver_min_addr; k < 4 && j < cfg_ver_min_length; k++)
			{
				CFG_VER_MIN_buff[j++] = cmd[k];
			}
		}
		else //other page
		{
			for (k = 0; k < 4 && j < cfg_ver_min_length; k++)
			{
				CFG_VER_MIN_buff[j++] = cmd[k];
			}
		}
		i++;
	} while (i < cfg_ver_min_end_addr);
	
	//Exit flash mode
	himax_FlashMode(0);
	
	/***********************************
	Check FW Version , TBD
	FW Major version 	: FW_VER_MAJ_buff
	FW Minor version 	: FW_VER_MIN_buff
	CFG Major version 	: CFG_VER_MAJ_buff
	CFG Minor version 	: CFG_VER_MIN_buff
	
	return 0 :
	return 1 :
	return 2 :
	
	***********************************/
	
	printk("[Touch_H]FW_VER_MAJ_buff : %d \n", FW_VER_MAJ_buff[0]);
	printk("[Touch_H]FW_VER_MIN_buff : %d \n", FW_VER_MIN_buff[0]);
	
	printk("[Touch_H]CFG_VER_MAJ_buff : %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
		CFG_VER_MAJ_buff[0], CFG_VER_MAJ_buff[1], CFG_VER_MAJ_buff[2], CFG_VER_MAJ_buff[3],
		CFG_VER_MAJ_buff[4], CFG_VER_MAJ_buff[5], CFG_VER_MAJ_buff[6], CFG_VER_MAJ_buff[7],
		CFG_VER_MAJ_buff[8], CFG_VER_MAJ_buff[9], CFG_VER_MAJ_buff[10], CFG_VER_MAJ_buff[11]
		);
/*
	for (i = 0; i < 12; i++)
	{
		printk(" %d", CFG_VER_MAJ_buff[i]);
		if(i == 11)
			printk("\n");
		else
			printk(",");
	}
*/
	printk("[Touch_H]CFG_VER_MIN_buff : %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
		CFG_VER_MIN_buff[0], CFG_VER_MIN_buff[1], CFG_VER_MIN_buff[2], CFG_VER_MIN_buff[3],
		CFG_VER_MIN_buff[4], CFG_VER_MIN_buff[5], CFG_VER_MIN_buff[6], CFG_VER_MIN_buff[7],
		CFG_VER_MIN_buff[8], CFG_VER_MIN_buff[9], CFG_VER_MIN_buff[10], CFG_VER_MIN_buff[11]
		);
/*
	for (i = 0; i < 12; i++)
	{
		printk(" %d", CFG_VER_MIN_buff[i]);

		if(i == 11)
			printk("\n");
		else
			printk(",");
	}
*/

	mutex_unlock(&private_ts->mutex_lock);

#ifdef ENABLE_CHIP_RESET_MACHINE
	if (private_ts->init_success)
	{
		queue_delayed_work(private_ts->himax_wq, &private_ts->himax_chip_reset_work, 0);
	}
#endif
// 	enable_irq(private_ts->client->irq);
	hx_irq_enable(private_ts);
	return 0;
}
 //----[firmware version read]-----------------------------------------------------------------------------end

static void himax_touch_information(void)
{
//	static unsigned char temp_buffer[6];
	if (IC_TYPE == HX_85XX_D_SERIES_PWON)
	{
/*
		himax_read_flash(temp_buffer, 0x26E, 3);
		HX_RX_NUM = temp_buffer[0];
		HX_TX_NUM = temp_buffer[1];
		HX_MAX_PT = (temp_buffer[2] & 0xF0) >> 4;

// #ifdef HX_EN_MUT_BUTTON
// 		himax_read_flash( temp_buffer, 0x262, 1);
// 		HX_BT_NUM = (temp_buffer[0] & 0x07);
// #endif

		himax_read_flash(temp_buffer, 0x272, 6);
// 		HX_X_RES = temp_buffer[2] * 256 + temp_buffer[3];
// 		HX_Y_RES = temp_buffer[4] * 256 + temp_buffer[5];
*/

                HX_RX_NUM = 13;
                HX_TX_NUM = 24;
                HX_MAX_PT = 10;

		HX_X_RES = 1080;
		HX_Y_RES = 1920;

		HX_INT_IS_EDGE = false;
/*
		himax_read_flash(temp_buffer, 0x200, 6);
		if ((temp_buffer[1] & 0x01) == 1)
		{
			HX_INT_IS_EDGE = true;
		}
		else
		{
			HX_INT_IS_EDGE = false;
		}
*/
	}
	else
	{
		HX_RX_NUM = 0;
		HX_TX_NUM = 0;
		HX_BT_NUM = 0;
		HX_X_RES = 0;
		HX_Y_RES = 0;
		HX_MAX_PT = 0;
		HX_INT_IS_EDGE = false;
	}
}

//=============================================================================================================
//
//	Segment : Himax SYS Debug Function
//
//=============================================================================================================

//----[HX_TP_SYS_REGISTER]------------------------------------------------------------------------------start
#ifdef HX_TP_SYS_REGISTER
static ssize_t himax_register_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	int base = 0;
	uint16_t loop_i, loop_j;
	uint8_t data[128];
	uint8_t outData[5];
	
	memset(outData, 0x00, sizeof(outData));
	memset(data, 0x00, sizeof(data));
	
	printk("[Touch_H]Himax multi_register_command = %d \n", multi_register_command);
	
	if (multi_register_command == 1)
	{
		base = 0;
	
		for (loop_i = 0; loop_i < 6; loop_i++)
		{
			if (multi_register[loop_i] != 0x00)
			{
				if (multi_cfg_bank[loop_i] == 1) //config bank register
				{
					outData[0] = 0x15;
					i2c_himax_write(private_ts->client, 0xE1, &outData[0], 1, DEFAULT_RETRY_CNT);
					msleep(10);
	
					outData[0] = 0x00;
					outData[1] = multi_register[loop_i];
					i2c_himax_write(private_ts->client, 0xD8, &outData[0], 2, DEFAULT_RETRY_CNT);
					msleep(10);
	
					i2c_himax_read(private_ts->client, 0x5A, data, 128,
							DEFAULT_RETRY_CNT);
	
					outData[0] = 0x00;
					i2c_himax_write(private_ts->client, 0xE1, &outData[0], 1, DEFAULT_RETRY_CNT);
	
					for (loop_j = 0; loop_j < 128; loop_j++)
					{
						multi_value[base++] = data[loop_j];
					}
				}
				else //normal register
				{
					i2c_himax_read(private_ts->client, multi_register[loop_i], data, 128, DEFAULT_RETRY_CNT);
	
					for (loop_j = 0; loop_j < 128; loop_j++)
					{
						multi_value[base++] = data[loop_j];
					}
				}
			}
		}
	
		base = 0;
		for (loop_i = 0; loop_i < 6; loop_i++)
		{
			if (multi_register[loop_i] != 0x00)
			{
				if (multi_cfg_bank[loop_i] == 1)
				{
					ret += sprintf(buf + ret, "Register: FE(%x)\n",	multi_register[loop_i]);
				}
				else
				{
					ret += sprintf(buf + ret, "Register: %x\n",
							multi_register[loop_i]);
				}
	
				for (loop_j = 0; loop_j < 128; loop_j++)
				{
					ret += sprintf(buf + ret, "0x%2.2X ", multi_value[base++]);
					if ((loop_j % 16) == 15)
					{
						ret += sprintf(buf + ret, "\n");
					}
				}
			}
		}
		return ret;
	}
	
	if (config_bank_reg)
	{
		printk("[Touch_H] %s: register_command = FE(%x)\n", __func__, register_command);
	
		//Config bank register read flow.
		outData[0] = 0x15;
		i2c_himax_write(private_ts->client, 0xE1, &outData[0], 1, DEFAULT_RETRY_CNT);
	
		msleep(10);
	
		outData[0] = 0x00;
		outData[1] = register_command;
		i2c_himax_write(private_ts->client, 0xD8, &outData[0], 2, DEFAULT_RETRY_CNT);
	
		msleep(10);
	
		i2c_himax_read(private_ts->client, 0x5A, data, 128, DEFAULT_RETRY_CNT);
	
		msleep(10);
	
		outData[0] = 0x00;
		i2c_himax_write(private_ts->client, 0xE1, &outData[0], 1, DEFAULT_RETRY_CNT);
	}
	else
	{
		if (i2c_himax_read(private_ts->client, register_command, data, 128, DEFAULT_RETRY_CNT) < 0)
		{
			return ret;
		}
	}
	
	if (config_bank_reg)
	{
		ret += sprintf(buf, "command: FE(%x)\n", register_command);
	}
	else
	{
		ret += sprintf(buf, "command: %x\n", register_command);
	}
	
	for (loop_i = 0; loop_i < 128; loop_i++)
	{
		ret += sprintf(buf + ret, "0x%2.2X ", data[loop_i]);
		if ((loop_i % 16) == 15)
		{
			ret += sprintf(buf + ret, "\n");
		}
	}
	ret += sprintf(buf + ret, "\n");
	return ret;
}

static ssize_t himax_register_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	char buf_tmp[6], length = 0;
	unsigned long result = 0;
	uint8_t loop_i = 0;
	uint16_t base = 5;
	uint8_t write_da[128];
	uint8_t outData[5];
	
	memset(buf_tmp, 0x0, sizeof(buf_tmp));
	memset(write_da, 0x0, sizeof(write_da));
	memset(outData, 0x0, sizeof(outData));
	
	printk("[Touch_H]himax %s \n", buf);
	
	if (buf[0] == 'm' && buf[1] == 'r' && buf[2] == ':')
	{
		memset(multi_register, 0x00, sizeof(multi_register));
		memset(multi_cfg_bank, 0x00, sizeof(multi_cfg_bank));
		memset(multi_value, 0x00, sizeof(multi_value));
	
		printk("[Touch_H]himax multi register enter\n");
	
		multi_register_command = 1;
	
		base = 2;
		loop_i = 0;
	
		while (true)
		{
			if (buf[base] == '\n')
			{
				break;
			}
	
			if (loop_i >= 6)
			{
				break;
			}
	
			if (buf[base] == ':' && buf[base + 1] == 'x' && buf[base + 2] == 'F'
					&& buf[base + 3] == 'E' && buf[base + 4] != ':')
			{
				memcpy(buf_tmp, buf + base + 4, 2);
				if (!strict_strtoul(buf_tmp, 16, &result))
				{
					multi_register[loop_i] = result;
					multi_cfg_bank[loop_i++] = 1;
				}
				base += 6;
			}
			else
			{
				memcpy(buf_tmp, buf + base + 2, 2);
				if (!strict_strtoul(buf_tmp, 16, &result))
				{
					multi_register[loop_i] = result;
					multi_cfg_bank[loop_i++] = 0;
				}
				base += 4;
			}
		}
	
		printk("[Touch_H]========================== \n");
		for (loop_i = 0; loop_i < 6; loop_i++)
		{
			printk("[Touch_H]%d,%d:", multi_register[loop_i], multi_cfg_bank[loop_i]);
		}
		printk("[Touch_H]\n");
	}
	else if ((buf[0] == 'r' || buf[0] == 'w') && buf[1] == ':')
	{
		multi_register_command = 0;
	
		if (buf[2] == 'x')
		{
			if (buf[3] == 'F' && buf[4] == 'E') //Config bank register
			{
				config_bank_reg = true;
	
				memcpy(buf_tmp, buf + 5, 2);
				if (!strict_strtoul(buf_tmp, 16, &result))
				{
					register_command = result;
				}
				base = 7;
	
				printk("[Touch_H]CMD: FE(%x)\n", register_command);
			}
			else
			{
				config_bank_reg = false;
	
				memcpy(buf_tmp, buf + 3, 2);
				if (!strict_strtoul(buf_tmp, 16, &result))
				{
					register_command = result;
				}
				base = 5;
				printk("[Touch_H]CMD: %x\n", register_command);
			}
	
			for (loop_i = 0; loop_i < 128; loop_i++)
			{
				if (buf[base] == '\n')
				{
					if (buf[0] == 'w')
					{
						if (config_bank_reg)
						{
							outData[0] = 0x15;
							i2c_himax_write(private_ts->client, 0xE1, &outData[0],
									1, DEFAULT_RETRY_CNT);
	
							msleep(10);
	
							outData[0] = 0x00;
							outData[1] = register_command;
							i2c_himax_write(private_ts->client, 0xD8, &outData[0],
									2, DEFAULT_RETRY_CNT);
	
							msleep(10);
							i2c_himax_write(private_ts->client, 0x40, &write_da[0],
									length, DEFAULT_RETRY_CNT);
	
							msleep(10);
	
							outData[0] = 0x00;
							i2c_himax_write(private_ts->client, 0xE1, &outData[0],
									1, DEFAULT_RETRY_CNT);
	
							printk("[Touch_H]CMD: FE(%x), %x, %d\n", register_command, write_da[0], length);
						}
						else
						{
							printk(
									"[Touch_H] Deeo test private_ts->client->addr 0x%x",
									private_ts->client->addr);
							i2c_himax_write(private_ts->client, register_command,
									&write_da[0], length, DEFAULT_RETRY_CNT);
							printk("[Touch_H]CMD: %x, %x, %d\n", register_command,
									write_da[0], length);
						}
					}
	
					printk("[Touch_H]\n");
					return count;
				}
				if (buf[base + 1] == 'x')
				{
					buf_tmp[4] = '\n';
					buf_tmp[5] = '\0';
					memcpy(buf_tmp, buf + base + 2, 2);
					if (!strict_strtoul(buf_tmp, 16, &result))
					{
						write_da[loop_i] = result;
					}
					length++;
				}
				base += 4;
			}
		}
	}
	return count;
}
static DEVICE_ATTR(register, HX_RW_ATTR, himax_register_show, himax_register_store);
#endif
 //----[HX_TP_SYS_REGISTER]--------------------------------------------------------------------------------end

 //----[HX_TP_SYS_DEBUG_LEVEL]---------------------------------------------------------------------------start
#ifdef HX_TP_SYS_DEBUG_LEVEL
static uint8_t getDebugLevel(void)
{
	return debug_log_level;
}

static int fts_ctpm_fw_upgrade_with_sys_fs(unsigned char *fw, int len)
{
	unsigned char* ImageBuffer = fw; //CTPM_FW;
	int fullFileLength = len; //sizeof(CTPM_FW); //Paul Check
	int i, j;
	uint8_t cmd[5], last_byte, prePage;
	int FileLength;
	uint8_t checksumResult = 0;
	
	//Try 3 Times
	for (j = 0; j < 3; j++)
	{
		if (IC_CHECKSUM == HX_TP_BIN_CHECKSUM_CRC)
		{
			FileLength = fullFileLength;
		}
		else
		{
			FileLength = fullFileLength - 2;
		}
	
#ifdef HX_RST_PIN_FUNC
		himax_HW_reset();
#endif
	
		if (i2c_himax_write(private_ts->client, 0x81, &cmd[0], 0, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
			return 0;
		}
	
		mdelay(120);
	
		himax_unlock_flash();  //ok
	
		cmd[0] = 0x05;
		cmd[1] = 0x00;
		cmd[2] = 0x02;
		if (i2c_himax_write(private_ts->client, 0x43, &cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
			return 0;
		}
	
		if (i2c_himax_write(private_ts->client, 0x4F, &cmd[0], 0, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
			return 0;
		}
		mdelay(50);
	
		himax_ManualMode(1);
		himax_FlashMode(1);
	
		FileLength = (FileLength + 3) / 4;
		TOTAL_COUNT = FileLength;
		for (i = 0, prePage = 0; i < FileLength; i++)
		{
// 			printk("%d / %d \n",update_progress, TOTAL_COUNT);
			update_progress = i;
			last_byte = 0;
			cmd[0] = i & 0x1F;
			if (cmd[0] == 0x1F || i == FileLength - 1)
			{
				last_byte = 1;
			}
			cmd[1] = (i >> 5) & 0x1F;
			cmd[2] = (i >> 10) & 0x1F;
			if (i2c_himax_write(private_ts->client, 0x44, &cmd[0], 3, DEFAULT_RETRY_CNT) < 0)
			{
				printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
				return 0;
			}
	
			if (prePage != cmd[1] || i == 0)
			{
				prePage = cmd[1];
				cmd[0] = 0x01;
				cmd[1] = 0x09;  //cmd[2] = 0x02;
				if (i2c_himax_write(private_ts->client, 0x43, &cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
				{
					printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
					return 0;
				}
	
				cmd[0] = 0x01;
				cmd[1] = 0x0D;  //cmd[2] = 0x02;
				if (i2c_himax_write(private_ts->client, 0x43, &cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
				{
					printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
					return 0;
				}
	
				cmd[0] = 0x01;
				cmd[1] = 0x09;  //cmd[2] = 0x02;
				if (i2c_himax_write(private_ts->client, 0x43, &cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
				{
					printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
					return 0;
				}
			}
	
			memcpy(&cmd[0], &ImageBuffer[4 * i], 4);  //Paul
			if (i2c_himax_write(private_ts->client, 0x45, &cmd[0], 4, DEFAULT_RETRY_CNT) < 0)
			{
				printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
				return 0;
			}
	
			cmd[0] = 0x01;
			cmd[1] = 0x0D;  //cmd[2] = 0x02;
			if (i2c_himax_write(private_ts->client, 0x43, &cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
			{
				printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
				return 0;
			}
	
			cmd[0] = 0x01;
			cmd[1] = 0x09;  //cmd[2] = 0x02;
			if (i2c_himax_write(private_ts->client, 0x43, &cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
			{
				printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
				return 0;
			}
			
			if (last_byte == 1)
			{
				cmd[0] = 0x01;
				cmd[1] = 0x01;  //cmd[2] = 0x02;
				if (i2c_himax_write(private_ts->client, 0x43, &cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
				{
					printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
					return 0;
				}
	
				cmd[0] = 0x01;
				cmd[1] = 0x05;  //cmd[2] = 0x02;
				if (i2c_himax_write(private_ts->client, 0x43, &cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
				{
					printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
					return 0;
				}
	
				cmd[0] = 0x01;
				cmd[1] = 0x01;  //cmd[2] = 0x02;
				if (i2c_himax_write(private_ts->client, 0x43, &cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
				{
					printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
					return 0;
				}
	
				cmd[0] = 0x01;
				cmd[1] = 0x00;  //cmd[2] = 0x02;
				if (i2c_himax_write(private_ts->client, 0x43, &cmd[0], 2, DEFAULT_RETRY_CNT) < 0)
				{
					printk("[Touch_H][TOUCH_ERR] %s: i2c access fail!\n", __func__);
					return 0;
				}
	
				mdelay(10);
				if (i == (FileLength - 1))
				{
					himax_FlashMode(0);
					himax_ManualMode(0);
					checksumResult = himax_calculateChecksum(ImageBuffer, fullFileLength);
					//himax_ManualMode(0);
					himax_lock_flash();
	
					if (checksumResult) //Success
					{
						return 1;
					}
					else //Fail
					{
						return 0;
					}
				}
			}
		}
	}
	return 0;
}

static ssize_t himax_debug_level_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	size_t count = 0;
// 	int i = 0;

	if (debug_level_cmd == 't')
	{
		if (fw_update_complete)
		{
			printk("[Touch_H] FW Update Complete \n");
		}
		else
		{
			printk("[Touch_H] FW Update Fail \n");
		}
		count += sprintf(buf, "%d\n", fw_update_complete);
	}
	else if (debug_level_cmd == 'i')
	{
		if(!private_ts->irq_is_disable)
		{
			count += sprintf(buf, "IRQ is enable\n");
		}
		else
		{
			count += sprintf(buf, "IRQ is disable\n");
		}
	}
	else if (debug_level_cmd == 'h')
	{
		if (handshaking_result == 0)
		{
			count += sprintf(buf, "Handshaking Result = %d (MCU Running)\n",
					handshaking_result);
		}
		else if (handshaking_result == 1)
		{
			count += sprintf(buf, "Handshaking Result = %d (MCU Stop)\n",
					handshaking_result);
		}
		else if (handshaking_result == 2)
		{
			count += sprintf(buf, "Handshaking Result = %d (I2C Error)\n",
					handshaking_result);
		}
		else
		{
			count += sprintf(buf, "Handshaking Result = error \n");
		}
	}
	else if (debug_level_cmd == 'v')
	{
		count += sprintf(buf + count, "FW_VER_MAJ_buff = ");
		count += sprintf(buf + count, "0x%2.2X \n", FW_VER_MAJ_buff[0]);
	
		count += sprintf(buf + count, "FW_VER_MIN_buff = ");
		count += sprintf(buf + count, "0x%2.2X \n", FW_VER_MIN_buff[0]);
	
		count += sprintf(buf + count, "CFG_VER_MAJ_buff = ");
		count += sprintf(buf + count, "0x%2.2X \n", CFG_VER_MAJ_buff[11]);
// 		for (i = 0; i < 12; i++)
// 		{
// 			count += sprintf(buf + count, "0x%2.2X ", CFG_VER_MAJ_buff[i]);
// 		}
// 		count += sprintf(buf + count, "\n");
	
		count += sprintf(buf + count, "CFG_VER_MIN_buff = ");
		count += sprintf(buf + count, "0x%2.2X \n", CFG_VER_MIN_buff[11]);
// 		for (i = 0; i < 12; i++)
// 		{
// 			count += sprintf(buf + count, "0x%2.2X ", CFG_VER_MIN_buff[i]);
// 		}
// 		count += sprintf(buf + count, "\n");
	}
	else if (debug_level_cmd == 'd')
	{
		count += sprintf(buf + count, "Himax Touch IC Information :\n");
		if (IC_TYPE == HX_85XX_D_SERIES_PWON)
		{
			count += sprintf(buf + count, "IC Type : D\n");
		}
		else
		{
			count += sprintf(buf + count, "IC Type error.\n");
		}
	
		if (IC_CHECKSUM == HX_TP_BIN_CHECKSUM_SW)
		{
			count += sprintf(buf + count, "IC Checksum : SW\n");
		}
		else if (IC_CHECKSUM == HX_TP_BIN_CHECKSUM_HW)
		{
			count += sprintf(buf + count, "IC Checksum : HW\n");
		}
		else if (IC_CHECKSUM == HX_TP_BIN_CHECKSUM_CRC)
		{
			count += sprintf(buf + count, "IC Checksum : CRC\n");
		}
		else
		{
			count += sprintf(buf + count, "IC Checksum error.\n");
		}
	
		if (HX_INT_IS_EDGE)
		{
			count += sprintf(buf + count, "Interrupt : EDGE TIRGGER\n");
		}
		else
		{
			count += sprintf(buf + count, "Interrupt : LEVEL TRIGGER\n");
		}
	
		count += sprintf(buf + count, "RX Num : %d\n", HX_RX_NUM);
		count += sprintf(buf + count, "TX Num : %d\n", HX_TX_NUM);
		count += sprintf(buf + count, "BT Num : %d\n", HX_BT_NUM);
		count += sprintf(buf + count, "X Resolution : %d\n", HX_X_RES);
		count += sprintf(buf + count, "Y Resolution : %d\n", HX_Y_RES);
		count += sprintf(buf + count, "Max Point : %d\n", HX_MAX_PT);
	}
	else
	{
		count += sprintf(buf, "%d\n", debug_log_level);
	}
	return count;
}

static ssize_t himax_debug_level_dump(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct file* filp = NULL;
	mm_segment_t oldfs;
	int result = 0;
	char fileName[128];
	
	if (buf[0] >= '0' && buf[0] <= '9' && buf[1] == '\n')
	{
		debug_log_level = buf[0] - '0';
		return count;
	}
	
	if (buf[0] == 'i') //irq
	{
		debug_level_cmd = buf[0];
	
		if (buf[2] == '1') //enable irq
		{
			enable_irq(private_ts->client->irq);
// 			hx_irq_enable(private_ts);
			private_ts->irq_is_disable = false;
		}
		else if (buf[2] == '0') //disable irq
		{
			disable_irq(private_ts->client->irq);
// 			hx_irq_disable(private_ts);
			private_ts->irq_is_disable = true;
		}
		else
		{
			printk("[Touch_H] %s: debug_level command = 'i' , parameter error.\n", __func__);
		}
		return count;
	}
	
	if (buf[0] == 'h') //handshaking
	{
		debug_level_cmd = buf[0];

// 		disable_irq(private_ts->client->irq);
		hx_irq_disable(private_ts);

		handshaking_result = himax_hang_shaking(); //0:Running, 1:Stop, 2:I2C Fail

// 		enable_irq(private_ts->client->irq);
		hx_irq_enable(private_ts);

		return count;
	}
	
	if (buf[0] == 'v') //firmware version
	{
		debug_level_cmd = buf[0];
		himax_read_FW_ver();
		return count;
	}
	
	if (buf[0] == 'd') //test
	{
		debug_level_cmd = buf[0];
		return count;
	}

	if (buf[0] == 's') //suspend
	{
		himax_ts_suspend();
		return count;
	}

	if (buf[0] == 'r') //resume
	{
		himax_ts_resume();
		return count;
	}

	if (buf[0] == 'f') {
		HX_RX_NUM = (buf[2] - '0')*10 + buf[3] - '0';
		HX_TX_NUM = (buf[5] - '0')*10 + buf[6] - '0';
		setXChannel(HX_RX_NUM);
		setYChannel(HX_TX_NUM);
		return count;
	}
	
	//Wakelock Protect start
	wake_lock(&private_ts->wake_lock);
	//Wakelock Protect end
	
	//Mutexlock Protect Start
	mutex_lock(&private_ts->mutex_lock);
	//Mutexlock Protect End
	
	if (buf[0] == 't')
	{
		debug_level_cmd = buf[0];
		fw_update_complete = false;
	
		memset(fileName, 0, 128);
		// parse the file name
		snprintf(fileName, count - 2, "%s", &buf[2]);
		printk("[Touch_H] %s: upgrade from file(%s) start!\n", __func__, fileName);
		// open file
		filp = filp_open(fileName, O_RDONLY, 0);
		if (IS_ERR(filp))
		{
			printk("[Touch_H] %s: open firmware file failed\n", __func__);
			goto firmware_upgrade_done;
			//return count;
		}
		oldfs = get_fs();
		set_fs(get_ds());
	
		// read the latest firmware binary file
		result = filp->f_op->read(filp, upgrade_fw, sizeof(upgrade_fw), &filp->f_pos);
		if (result < 0)
		{
			printk("[Touch_H] %s: read firmware file failed\n", __func__);
			goto firmware_upgrade_done;
			//return count;
		}
	
		set_fs(oldfs);
		filp_close(filp, NULL );
	
		printk("[Touch_H] %s: upgrade start,len %d: %02X, %02X, %02X, %02X\n",
				__func__, result, upgrade_fw[0], upgrade_fw[1], upgrade_fw[2], upgrade_fw[3]);
	
		if (result > 0)
		{
			// start to upgrade
// 			disable_irq(private_ts->client->irq);
			hx_irq_disable(private_ts);
			if (fts_ctpm_fw_upgrade_with_sys_fs(upgrade_fw, result) == 0)
			{
				printk("[Touch_H] %s: TP upgrade error, line: %d\n", __func__,
						__LINE__);
				fw_update_complete = false;
			}
			else
			{
				printk("[Touch_H] %s: TP upgrade OK, line: %d\n", __func__,
						__LINE__);
				fw_update_complete = true;
#ifdef ASUS_FACTORY_BUILD
				common_fw = false;
#endif
			}
// 			enable_irq(private_ts->client->irq);
			hx_irq_enable(private_ts);
			goto firmware_upgrade_done;
			//return count;
		}
	}
	
	
firmware_upgrade_done:
	
	//Mutexlock Protect Start
	mutex_unlock(&private_ts->mutex_lock);
	//Mutexlock Protect End

	himax_read_FW_ver();
	
//----[ENABLE_CHIP_RESET_MACHINE]------------------------------------------------------------------start
#ifdef ENABLE_CHIP_RESET_MACHINE
// 	queue_delayed_work(private_ts->himax_wq, &private_ts->himax_chip_reset_work, 0);
#endif
//----[ENABLE_CHIP_RESET_MACHINE]--------------------------------------------------------------------end
	
	//todo himax_chip->tp_firmware_upgrade_proceed = 0;
	//todo himax_chip->suspend_state = 0;
	//todo enable_irq(himax_chip->irq);
	
	//Wakelock Protect start
	wake_unlock(&private_ts->wake_lock);
	//Wakelock Protect end
	
	
	//Power on touch IC after updating Firmware
	//himax_ts_poweron(private_ts);
	//printk("[Touch_H] Enabke IRQ after Finish Update Firmware. %d\n",private_ts->client->irq);
	//enable_irq(private_ts->client->irq);
	return count;
}
static DEVICE_ATTR(debug_level, HX_RW_ATTR, himax_debug_level_show, himax_debug_level_dump);
#endif
//----[HX_TP_SYS_DEBUG_LEVEL]-----------------------------------------------------------------------------end

//----[HX_TP_SYS_DIAG]----------------------------------------------------------------------------------start
#ifdef HX_TP_SYS_DIAG
static uint8_t *getMutualBuffer(void)
{
	return diag_mutual;
}

static uint8_t *getSelfBuffer(void)
{
	return &diag_self[0];
}

static uint8_t getXChannel(void)
{
	return x_channel;
}

static uint8_t getYChannel(void)
{
	return y_channel;
}

static uint8_t getDiagCommand(void)
{
	return diag_command;
}

static void setXChannel(uint8_t x)
{
	x_channel = x;
}

static void setYChannel(uint8_t y)
{
	y_channel = y;
}

static void setMutualBuffer(void)
{
	diag_mutual = kzalloc(x_channel * y_channel * sizeof(uint8_t), GFP_KERNEL);
}

static ssize_t himax_diag_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	size_t count = 0;
	uint32_t loop_i;
	uint16_t mutual_num, self_num, width;
	
	mutual_num = x_channel * y_channel;
	self_num = x_channel + y_channel; //don't add KEY_COUNT
	
	width = x_channel;
	count += sprintf(buf + count, "ChannelStart: %4d, %4d\n\n", x_channel, y_channel);
	
	// start to show out the raw data in adb shell
	if (diag_command >= 1 && diag_command <= 6)
	{
		if (diag_command <= 3)
		{
			for (loop_i = 0; loop_i < mutual_num; loop_i++)
			{
				count += sprintf(buf + count, "%4d", diag_mutual[loop_i]);
				if ((loop_i % width) == (width - 1))
				{
					count += sprintf(buf + count, " %3d\n", diag_self[width + loop_i / width]);
				}
			}
			count += sprintf(buf + count, "\n");
			for (loop_i = 0; loop_i < width; loop_i++)
			{
				count += sprintf(buf + count, "%4d", diag_self[loop_i]);
				if (((loop_i) % width) == (width - 1))
				{
					count += sprintf(buf + count, "\n");
				}
			}
		}
		else if (diag_command > 4)
		{
			for (loop_i = 0; loop_i < self_num; loop_i++)
			{
				count += sprintf(buf + count, "%4d", diag_self[loop_i]);
				if (((loop_i - mutual_num) % width) == (width - 1))
				{
					count += sprintf(buf + count, "\n");
				}
			}
		}
		else
		{
			for (loop_i = 0; loop_i < mutual_num; loop_i++)
			{
				count += sprintf(buf + count, "%4d", diag_mutual[loop_i]);
				if ((loop_i % width) == (width - 1))
				{
					count += sprintf(buf + count, "\n");
				}
			}
		}
		count += sprintf(buf + count, "ChannelEnd");
		count += sprintf(buf + count, "\n");
	}
	else if (diag_command == 7)
	{
		for (loop_i = 0; loop_i < 128; loop_i++)
		{
			if ((loop_i % 16) == 0)
			{
				count += sprintf(buf + count, "LineStart:");
			}
	
			count += sprintf(buf + count, "%4d", diag_coor[loop_i]);
			if ((loop_i % 16) == 15)
			{
				count += sprintf(buf + count, "\n");
			}
		}
	}
	return count;
}

static ssize_t himax_diag_dump(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	const uint8_t command_ec_128_raw_flag = 0x01;
	const uint8_t command_ec_24_normal_flag = 0x00;
	
	uint8_t command_ec_128_raw_baseline_flag = 0x02;
	uint8_t command_ec_128_raw_bank_flag = 0x03;
	
	uint8_t command_91h[2] = { 0x91, 0x00 };
	uint8_t command_82h[1] = { 0x82 };
	uint8_t command_F3h[2] = { 0xF3, 0x00 };
	uint8_t command_83h[1] = { 0x83 };
	uint8_t receive[1];
	
	if (IC_TYPE != HX_85XX_D_SERIES_PWON)
	{
		command_ec_128_raw_baseline_flag = 0x02 | command_ec_128_raw_flag;
	}
	else
	{
		command_ec_128_raw_baseline_flag = 0x02;
		command_ec_128_raw_bank_flag = 0x03;
	}
	
	if (buf[0] == '1')	//IIR
	{
		command_91h[1] = command_ec_128_raw_baseline_flag; //A:0x03 , D:0x02
		i2c_himax_write(private_ts->client, command_91h[0], &command_91h[1], 1,
				DEFAULT_RETRY_CNT);
		diag_command = buf[0] - '0';
		printk("[Touch_H]diag_command=0x%x\n", diag_command);
	}
	else if (buf[0] == '2')	//DC
	{
		command_91h[1] = command_ec_128_raw_flag;	//0x01
		i2c_himax_write(private_ts->client, command_91h[0], &command_91h[1], 1,
				DEFAULT_RETRY_CNT);
	
		i2c_himax_read(private_ts->client, command_F3h[0], receive, 1,
				DEFAULT_RETRY_CNT);
		command_F3h[1] = (receive[0] & 0x7F);
		i2c_himax_write(private_ts->client, command_F3h[0], &command_F3h[1], 1,
				DEFAULT_RETRY_CNT);
	
		diag_command = buf[0] - '0';
		printk(KERN_ERR "[Touch_H]diag_command=0x%x\n",diag_command);
	}
	else if (buf[0] == '3')	//BANK
	{
		if (IC_TYPE != HX_85XX_D_SERIES_PWON)
		{
			command_91h[1] = command_ec_128_raw_flag;	//0x01
			i2c_himax_write(private_ts->client, command_91h[0], &command_91h[1], 1, DEFAULT_RETRY_CNT);
		
			i2c_himax_read(private_ts->client, command_F3h[0], receive, 1, DEFAULT_RETRY_CNT);
			command_F3h[1] = (receive[0] | 0x80);
			i2c_himax_write(private_ts->client, command_F3h[0], &command_F3h[1], 1, DEFAULT_RETRY_CNT);
		
			msleep(50);
		}
		else
		{
			command_91h[1] = command_ec_128_raw_bank_flag;	//0x03
			i2c_himax_write(private_ts->client, command_91h[0], &command_91h[1], 1, DEFAULT_RETRY_CNT);
		}
		diag_command = buf[0] - '0';
		printk(KERN_ERR "[Touch_H]diag_command=0x%x\n",diag_command);
	}
	else if (buf[0] == '7')
	{
		diag_command = buf[0] - '0';
	}
	//coordinate dump start
	else if (buf[0] == '8')
	{
		diag_command = buf[0] - '0';
		
		coordinate_fn = filp_open(DIAG_COORDINATE_FILE, O_CREAT | O_WRONLY | O_APPEND | O_TRUNC, 0666);
		if (IS_ERR(coordinate_fn))
		{
			printk("[Touch_H][TOUCH_ERR]%s: coordinate_dump_file_create error\n", __func__);
			coordinate_dump_enable = 0;
			filp_close(coordinate_fn, NULL);
		}
		coordinate_dump_enable = 1;
	}
	else if (buf[0] == '9')
	{
		coordinate_dump_enable = 0;
		diag_command = buf[0] - '0';
		
		if (!IS_ERR(coordinate_fn))
		{
			filp_close(coordinate_fn, NULL);
		}
	}
		//coordinate dump end
	else
	{
		if (IC_TYPE != HX_85XX_D_SERIES_PWON)
		{
			i2c_himax_write(private_ts->client, command_82h[0], &command_82h[0], 0, DEFAULT_RETRY_CNT);
			msleep(50);
			command_91h[1] = command_ec_24_normal_flag;
			i2c_himax_write(private_ts->client, command_91h[0], &command_91h[1], 1, DEFAULT_RETRY_CNT);
			i2c_himax_read(private_ts->client, command_F3h[0], receive, 1, DEFAULT_RETRY_CNT);
			command_F3h[1] = (receive[0] & 0x7F);
			i2c_himax_write(private_ts->client, command_F3h[0], &command_F3h[1], 1, DEFAULT_RETRY_CNT);
			i2c_himax_write(private_ts->client, command_83h[0], &command_83h[0], 0, DEFAULT_RETRY_CNT);
		}
		else
		{
			command_91h[1] = command_ec_24_normal_flag;
			i2c_himax_write(private_ts->client, command_91h[0], &command_91h[1], 1, DEFAULT_RETRY_CNT);
		}
		diag_command = 0;
		printk("[Touch_H]diag_command=0x%x\n", diag_command);
	}
	return count;
}
static DEVICE_ATTR(diag, HX_RW_ATTR, himax_diag_show, himax_diag_dump);

static ssize_t himax_chip_raw_data_store(struct device *dev, struct device_attribute *attr, char *buf)
{
	size_t count = 0;
	uint32_t loop_i;
	uint16_t mutual_num, self_num, width;
	struct file* filp = NULL;
	mm_segment_t oldfs = { 0 };  //Deeo

	mutual_num = x_channel * y_channel;
	self_num = x_channel + y_channel;
	width = x_channel;

	if (diag_command == 1)
	{
		filp = filp_open("/sdcard/Touch_rawdata.txt", O_RDWR | O_CREAT, S_IRUSR);
		if (IS_ERR(filp))
		{
			printk("[Touch_H][TOUCH_ERR] %s: open /sdcard/Touch_rawdata.txt failed\n", __func__);
			return 0;
		}
		oldfs = get_fs();
		set_fs(get_ds());
	}
	else if (diag_command == 2)
	{
		filp = filp_open("/sdcard/Touch_diff.txt", O_RDWR | O_CREAT, S_IRUSR);
		if (IS_ERR(filp))
		{
			printk("[Touch_H][TOUCH_ERR] %s: open /sdcard/Touch_diff.txt failed\n", __func__);
			return 0;
		}
		oldfs = get_fs();
		set_fs(get_ds());
	}
	else if (diag_command == 3)
	{
		filp = filp_open("/sdcard/Touch_bank.txt", O_RDWR | O_CREAT, S_IRUSR);
		if (IS_ERR(filp))
		{
			printk("[Touch_H][TOUCH_ERR] %s: open /sdcard/Touch_bank.txt failed\n", __func__);
			return 0;
		}
		oldfs = get_fs();
		set_fs(get_ds());
	}

	count += sprintf(buf + count, "Channel: %4d, %4d\n\n", x_channel, y_channel);
	if (diag_command >= 1 && diag_command <= 6)
	{
		if (diag_command < 4)
		{
			for (loop_i = 0; loop_i < mutual_num; loop_i++)
			{
				count += sprintf(buf + count, "%4d", diag_mutual[loop_i]);
				if ((loop_i % width) == (width - 1))
				{
					count += sprintf(buf + count, " %3d\n", diag_self[width + loop_i / width]);
				}
			}
			count += sprintf(buf + count, "\n");
			for (loop_i = 0; loop_i < width; loop_i++)
			{
				count += sprintf(buf + count, "%4d", diag_self[loop_i]);
				if (((loop_i) % width) == (width - 1))
				{
					count += sprintf(buf + count, "\n");
				}
			}
		}
		else if (diag_command > 4)
		{
			for (loop_i = 0; loop_i < self_num; loop_i++)
			{
				count += sprintf(buf + count, "%4d", diag_self[loop_i]);
				if (((loop_i - mutual_num) % width) == (width - 1))
				{
					count += sprintf(buf + count, "\n");
				}
			}
		}
		else
		{
			for (loop_i = 0; loop_i < mutual_num; loop_i++)
			{
				count += sprintf(buf + count, "%4d", diag_mutual[loop_i]);
				if ((loop_i % width) == (width - 1))
				{
					count += sprintf(buf + count, "\n");
				}
			}
		}
	}
	if (diag_command >= 1 && diag_command <= 3)
	{
		filp->f_op->write(filp, buf, count, &filp->f_pos);
		set_fs(oldfs);
		filp_close(filp, NULL );
	}

	return count;
}
static DEVICE_ATTR(tp_output_raw_data, HX_RW_ATTR, himax_chip_raw_data_store, himax_diag_dump);
#endif
 //----[HX_TP_SYS_DIAG]------------------------------------------------------------------------------------end

 //----[HX_TP_SYS_FLASH_DUMP]----------------------------------------------------------------------------start
#ifdef HX_TP_SYS_FLASH_DUMP
static uint8_t getFlashCommand(void)
{
	return flash_command;
}

static uint8_t getFlashDumpProgress(void)
{
	return flash_progress;
}

static uint8_t getFlashDumpComplete(void)
{
	return flash_dump_complete;
}

static uint8_t getFlashDumpFail(void)
{
	return flash_dump_fail;
}

static uint8_t getSysOperation(void)
{
	return sys_operation;
}

static uint8_t getFlashReadStep(void)
{
	return flash_read_step;
}

static uint8_t getFlashDumpSector(void)
{
	return flash_dump_sector;
}

static uint8_t getFlashDumpPage(void)
{
	return flash_dump_page;
}

static bool getFlashDumpGoing(void)
{
	return flash_dump_going;
}

static void setFlashBuffer(void)
{
	int i = 0;
	flash_buffer = kzalloc(32768 * sizeof(uint8_t), GFP_KERNEL);
	for (i = 0; i < 32768; i++)
	{
		flash_buffer[i] = 0x00;
	}
}

static void setSysOperation(uint8_t operation)
{
	sys_operation = operation;
}

static void setFlashDumpProgress(uint8_t progress)
{
	flash_progress = progress;
	printk( "[Touch_H]TP setFlashDumpProgress : progress = %d ,flash_progress = %d \n",progress, flash_progress);
}

static void setFlashDumpComplete(uint8_t status)
{
	flash_dump_complete = status;
}

static void setFlashDumpFail(uint8_t fail)
{
	flash_dump_fail = fail;
}

static void setFlashCommand(uint8_t command)
{
	flash_command = command;
}

static void setFlashReadStep(uint8_t step)
{
	flash_read_step = step;
}

static void setFlashDumpSector(uint8_t sector)
{
	flash_dump_sector = sector;
}

static void setFlashDumpPage(uint8_t page)
{
	flash_dump_page = page;
}

static void setFlashDumpGoing(bool going)
{
	flash_dump_going = going;
}

static ssize_t himax_flash_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	int loop_i;
	uint8_t local_flash_read_step = 0;
	uint8_t local_flash_complete = 0;
	uint8_t local_flash_progress = 0;
	uint8_t local_flash_command = 0;
	uint8_t local_flash_fail = 0;

	local_flash_complete = getFlashDumpComplete();
	local_flash_progress = getFlashDumpProgress();
	local_flash_command = getFlashCommand();
	local_flash_fail = getFlashDumpFail();

	printk("[Touch_H]TP flash_progress = %d \n", local_flash_progress);

	if (local_flash_fail)
	{
		ret += sprintf(buf + ret, "FlashStart:Fail \n");
		ret += sprintf(buf + ret, "FlashEnd");
		ret += sprintf(buf + ret, "\n");
		return ret;
	}

	if (!local_flash_complete)
	{
		ret += sprintf(buf + ret, "FlashStart:Ongoing:0x%2.2x \n", flash_progress);
		ret += sprintf(buf + ret, "FlashEnd");
		ret += sprintf(buf + ret, "\n");
		return ret;
	}

	if (local_flash_command == 1 && local_flash_complete)
	{
		ret += sprintf(buf + ret, "FlashStart:Complete \n");
		ret += sprintf(buf + ret, "FlashEnd");
		ret += sprintf(buf + ret, "\n");
		return ret;
	}

	if (local_flash_command == 3 && local_flash_complete)
	{
		ret += sprintf(buf + ret, "FlashStart: \n");
		for (loop_i = 0; loop_i < 128; loop_i++)
		{
			ret += sprintf(buf + ret, "x%2.2x", flash_buffer[loop_i]);
			if ((loop_i % 16) == 15)
			{
				ret += sprintf(buf + ret, "\n");
			}
		}
		ret += sprintf(buf + ret, "FlashEnd");
		ret += sprintf(buf + ret, "\n");
		return ret;
	}

	//flash command == 0 , report the data
	local_flash_read_step = getFlashReadStep();

	ret += sprintf(buf + ret, "FlashStart:%2.2x \n", local_flash_read_step);

	for (loop_i = 0; loop_i < 1024; loop_i++)
	{
		ret += sprintf(buf + ret, "x%2.2X",
		flash_buffer[local_flash_read_step * 1024 + loop_i]);

		if ((loop_i % 16) == 15)
		{
			ret += sprintf(buf + ret, "\n");
		}
	}

	ret += sprintf(buf + ret, "Flash End");
	ret += sprintf(buf + ret, "\n");
	return ret;
}

 //-----------------------------------------------------------------------------------
 //himax_flash_store
 //
 //command 0 : Read the page by step number
 //command 1 : driver start to dump flash data, save it to mem
 //command 2 : driver start to dump flash data, save it to sdcard/Flash_Dump.bin
 //
 //-----------------------------------------------------------------------------------
static ssize_t himax_flash_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	char buf_tmp[6];
	unsigned long result = 0;
	uint8_t loop_i = 0;
	int base = 0;

	memset(buf_tmp, 0x0, sizeof(buf_tmp));

	printk("[Touch_H] %s: buf[0] = %s\n", __func__, buf);

	if (getSysOperation() == 1)
	{
		printk("[Touch_H] %s: SYS is busy , return!\n", __func__);
		return count;
	}

	if (buf[0] == '0')
	{
		setFlashCommand(0);
		if (buf[1] == ':' && buf[2] == 'x')
		{
			memcpy(buf_tmp, buf + 3, 2);
			printk("[Touch_H] %s: read_Step = %s\n", __func__, buf_tmp);
			if (!strict_strtoul(buf_tmp, 16, &result))
			{
				printk("[Touch_H] %s: read_Step = %lu \n", __func__, result);
				setFlashReadStep(result);
			}
		}
	}
	else if (buf[0] == '1')
	{
		setSysOperation(1);
		setFlashCommand(1);
		setFlashDumpProgress(0);
		setFlashDumpComplete(0);
		setFlashDumpFail(0);
		queue_work(private_ts->flash_wq, &private_ts->flash_work);
	}
	else if (buf[0] == '2')
	{
		setSysOperation(1);
		setFlashCommand(2);
		setFlashDumpProgress(0);
		setFlashDumpComplete(0);
		setFlashDumpFail(0);

		queue_work(private_ts->flash_wq, &private_ts->flash_work);
	}
	else if (buf[0] == '3')
	{
		setSysOperation(1);
		setFlashCommand(3);
		setFlashDumpProgress(0);
		setFlashDumpComplete(0);
		setFlashDumpFail(0);

		memcpy(buf_tmp, buf + 3, 2);
		if (!strict_strtoul(buf_tmp, 16, &result))
		{
			setFlashDumpSector(result);
		}

		memcpy(buf_tmp, buf + 7, 2);
		if (!strict_strtoul(buf_tmp, 16, &result))
		{
			setFlashDumpPage(result);
		}

		queue_work(private_ts->flash_wq, &private_ts->flash_work);
	}
	else if (buf[0] == '4')
	{
		printk("[Touch_H] %s: command 4 enter.\n", __func__);
		setSysOperation(1);
		setFlashCommand(4);
		setFlashDumpProgress(0);
		setFlashDumpComplete(0);
		setFlashDumpFail(0);

		memcpy(buf_tmp, buf + 3, 2);
		if (!strict_strtoul(buf_tmp, 16, &result))
		{
			setFlashDumpSector(result);
		}
		else
		{
			printk("[Touch_H] %s: command 4 , sector error.\n", __func__);
			return count;
		}

		memcpy(buf_tmp, buf + 7, 2);
		if (!strict_strtoul(buf_tmp, 16, &result))
		{
			setFlashDumpPage(result);
		}
		else
		{
			printk("[Touch_H] %s: command 4 , page error.\n", __func__);
			return count;
		}

		base = 11;

		printk("[Touch_H]=========Himax flash page buffer start=========\n");
		for (loop_i = 0; loop_i < 128; loop_i++)
		{
			memcpy(buf_tmp, buf + base, 2);
			if (!strict_strtoul(buf_tmp, 16, &result))
			{
				flash_buffer[loop_i] = result;
				printk("[Touch_H] %d ", flash_buffer[loop_i]);
				if (loop_i % 16 == 15)
				{
					printk("[Touch_H]\n");
				}
			}
			base += 3;
		}
		printk("[Touch_H]=========Himax flash page buffer end=========\n");

		queue_work(private_ts->flash_wq, &private_ts->flash_work);
	}
	return count;
}
static DEVICE_ATTR(flash_dump, HX_RW_ATTR, himax_flash_show, himax_flash_store);

static void himax_ts_flash_work_func(struct work_struct *work)
{
	struct himax_ts_data *ts = container_of(work, struct himax_ts_data, flash_work);

	uint8_t page_tmp[128];
	uint8_t x59_tmp[4] = { 0, 0, 0, 0 };
	int i = 0, j = 0, k = 0, l = 0,/* j_limit = 0,*/buffer_ptr = 0,
	flash_end_count = 0;
	uint8_t local_flash_command = 0;
	uint8_t sector = 0;
	uint8_t page = 0;

	uint8_t x81_command[2] = { 0x81, 0x00 };
	uint8_t x82_command[2] = { 0x82, 0x00 };
	uint8_t x42_command[2] = { 0x42, 0x00 };
	uint8_t x43_command[4] = { 0x43, 0x00, 0x00, 0x00 };
	uint8_t x44_command[4] = { 0x44, 0x00, 0x00, 0x00 };
	uint8_t x45_command[5] = { 0x45, 0x00, 0x00, 0x00, 0x00 };
	uint8_t x46_command[2] = { 0x46, 0x00 };
	uint8_t x4A_command[2] = { 0x4A, 0x00 };
	uint8_t x4D_command[2] = { 0x4D, 0x00 };
/*uint8_t x59_command[2] = {0x59,0x00};*/

// 	disable_irq(ts->client->irq);
	hx_irq_disable(ts);
	setFlashDumpGoing(true);

#ifdef HX_RST_PIN_FUNC
	himax_HW_reset();
#endif

	sector = getFlashDumpSector();
	page = getFlashDumpPage();

	local_flash_command = getFlashCommand();

	if (i2c_himax_master_write(ts->client, x81_command, 1, 3) < 0)		//sleep out
	{
		printk("[Touch_H][TOUCH_ERR] %s i2c write 81 fail.\n", __func__);
		goto Flash_Dump_i2c_transfer_error;
	}
	msleep(120);

	if (i2c_himax_master_write(ts->client, x82_command, 1, 3) < 0)
	{
		printk("[Touch_H][TOUCH_ERR] %s i2c write 82 fail.\n", __func__);
		goto Flash_Dump_i2c_transfer_error;
	}
	msleep(100);

	printk("[Touch_H] %s: local_flash_command = %d enter.\n", __func__, local_flash_command);
	printk("[Touch_H] %s: flash buffer start.\n", __func__);
	for (i = 0; i < 128; i++)
	{
		printk("[Touch_H] %2.2x ", flash_buffer[i]);
		if ((i % 16) == 15)
		{
			printk("[Touch_H]\n");
		}
	}
		printk("[Touch_H] %s: flash buffer end.\n", __func__);

	if (local_flash_command == 1 || local_flash_command == 2)
	{
		x43_command[1] = 0x01;
		if (i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 1, DEFAULT_RETRY_CNT) < 0)
		{
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(100);

		for (i = 0; i < 8; i++)
		{
			for (j = 0; j < 32; j++)
			{
				printk("[Touch_H]TP Step 2 i=%d , j=%d %s\n", i, j, __func__);
				//read page start
				for (k = 0; k < 128; k++)
				{
					page_tmp[k] = 0x00;
				}
				for (k = 0; k < 32; k++)
				{
					x44_command[1] = k;
					x44_command[2] = j;
					x44_command[3] = i;
					if (i2c_himax_write(ts->client, x44_command[0], &x44_command[1], 3,
						DEFAULT_RETRY_CNT) < 0)
					{
						printk("[Touch_H][TOUCH_ERR] %s i2c write 44 fail.\n", __func__);
						goto Flash_Dump_i2c_transfer_error;
					}

					if (i2c_himax_write_command(ts->client, x46_command[0],
						DEFAULT_RETRY_CNT) < 0)
					{
						printk("[Touch_H][TOUCH_ERR] %s i2c write 46 fail.\n", __func__);
						goto Flash_Dump_i2c_transfer_error;
					}	
					//msleep(2);
					if (i2c_himax_read(ts->client, 0x59, x59_tmp, 4, DEFAULT_RETRY_CNT) < 0)
					{
						printk("[Touch_H][TOUCH_ERR] %s i2c write 59 fail.\n", __func__);
						goto Flash_Dump_i2c_transfer_error;
					}
					//msleep(2);
					for (l = 0; l < 4; l++)
					{
						page_tmp[k * 4 + l] = x59_tmp[l];
					}
					//msleep(10);
				}
				//read page end

				for (k = 0; k < 128; k++)
				{
					flash_buffer[buffer_ptr++] = page_tmp[k];

					if (page_tmp[k] == 0xFF)
					{
						flash_end_count++;
						if (flash_end_count == 32)
						{
							flash_end_count = 0;
							buffer_ptr = buffer_ptr - 32;
							goto FLASH_END;
						}
					}
					else
					{
						flash_end_count = 0;
					}
				}
				setFlashDumpProgress(i * 32 + j);
			}
		}
	}
	else if (local_flash_command == 3)
	{
		x43_command[1] = 0x01;
		if (i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 1, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s i2c write 43 fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(100);

		for (i = 0; i < 128; i++)
		{
			page_tmp[i] = 0x00;
		}

		for (i = 0; i < 32; i++)
		{
			x44_command[1] = i;
			x44_command[2] = page;
			x44_command[3] = sector;

			if (i2c_himax_write(ts->client, x44_command[0], &x44_command[1], 3, DEFAULT_RETRY_CNT) < 0)
			{
				printk("[Touch_H][TOUCH_ERR] %s i2c write 44 fail.\n", __func__);
				goto Flash_Dump_i2c_transfer_error;
			}

			if (i2c_himax_write_command(ts->client, x46_command[0], DEFAULT_RETRY_CNT) < 0)
			{
				printk("[Touch_H][TOUCH_ERR] %s i2c write 46 fail.\n", __func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			//msleep(2);
			if (i2c_himax_read(ts->client, 0x59, x59_tmp, 4, DEFAULT_RETRY_CNT) < 0)
			{
				printk("[Touch_H][TOUCH_ERR] %s i2c write 59 fail.\n", __func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			//msleep(2);
			for (j = 0; j < 4; j++)
			{
				page_tmp[i * 4 + j] = x59_tmp[j];
			}
			//msleep(10);
		}
		//read page end
		for (i = 0; i < 128; i++)
		{
			flash_buffer[buffer_ptr++] = page_tmp[i];
		}
	}
	else if (local_flash_command == 4)
	{
		//page write flow.
		printk("[Touch_H] %s: local_flash_command = 4, enter.\n", __func__);

//-----------------------------------------------------------------------------------------------
// unlock flash
//-----------------------------------------------------------------------------------------------
		x43_command[1] = 0x01;
		x43_command[2] = 0x00;
		x43_command[3] = 0x06;
		if (i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 3, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s i2c write 43 fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		x44_command[1] = 0x03;
		x44_command[2] = 0x00;
		x44_command[3] = 0x00;
		if (i2c_himax_write(ts->client, x44_command[0], &x44_command[1], 3, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s i2c write 44 fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		x45_command[1] = 0x00;
		x45_command[2] = 0x00;
		x45_command[3] = 0x3D;
		x45_command[4] = 0x03;
		if (i2c_himax_write(ts->client, x45_command[0], &x45_command[1], 4, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s i2c write 45 fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		if (i2c_himax_write_command(ts->client, x4A_command[0], DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s i2c write 4A fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(50);

//-----------------------------------------------------------------------------------------------
// page erase
//-----------------------------------------------------------------------------------------------
		x43_command[1] = 0x01;
		x43_command[2] = 0x00;
		x43_command[3] = 0x02;
		if (i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 3, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s i2c write 43 fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		x44_command[1] = 0x00;
		x44_command[2] = page;
		x44_command[3] = sector;
		if (i2c_himax_write(ts->client, x44_command[0], &x44_command[1], 3, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s i2c write 44 fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		if (i2c_himax_write_command(ts->client, x4D_command[0], DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s i2c write 4D fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(100);

//-----------------------------------------------------------------------------------------------
// enter manual mode
//-----------------------------------------------------------------------------------------------
		x42_command[1] = 0x01;
		if (i2c_himax_write(ts->client, x42_command[0], &x42_command[1], 1, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s i2c write 42 fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(100);

//-----------------------------------------------------------------------------------------------
// flash enable
//-----------------------------------------------------------------------------------------------
		x43_command[1] = 0x01;
		x43_command[2] = 0x00;
		if (i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 2, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s i2c write 43 fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

//-----------------------------------------------------------------------------------------------
// set flash address
//-----------------------------------------------------------------------------------------------
		x44_command[1] = 0x00;
		x44_command[2] = page;
		x44_command[3] = sector;
		if (i2c_himax_write(ts->client, x44_command[0], &x44_command[1], 3, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s i2c write 44 fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

//-----------------------------------------------------------------------------------------------
// manual mode command : 47 to latch the flash address when page address change.
//-----------------------------------------------------------------------------------------------
		x43_command[1] = 0x01;
		x43_command[2] = 0x09;
		if (i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 2, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s i2c write 43 fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		x43_command[1] = 0x01;
		x43_command[2] = 0x0D;
		if (i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 2, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s i2c write 43 fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		x43_command[1] = 0x01;
		x43_command[2] = 0x09;
		if (i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 2, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s i2c write 43 fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		for (i = 0; i < 32; i++)
		{
			printk("[Touch_H]himax :i=%d \n", i);
			x44_command[1] = i;
			x44_command[2] = page;
			x44_command[3] = sector;
			if (i2c_himax_write(ts->client, x44_command[0], &x44_command[1], 3, DEFAULT_RETRY_CNT) < 0)
			{
				printk("[Touch_H][TOUCH_ERR] %s i2c write 44 fail.\n", __func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			msleep(10);

			x45_command[1] = flash_buffer[i * 4 + 0];
			x45_command[2] = flash_buffer[i * 4 + 1];
			x45_command[3] = flash_buffer[i * 4 + 2];
			x45_command[4] = flash_buffer[i * 4 + 3];
			if (i2c_himax_write(ts->client, x45_command[0], &x45_command[1], 4, DEFAULT_RETRY_CNT) < 0)
			{
				printk("[Touch_H][TOUCH_ERR] %s i2c write 45 fail.\n", __func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			msleep(10);

//-----------------------------------------------------------------------------------------------
// manual mode command : 48 ,data will be written into flash buffer
//-----------------------------------------------------------------------------------------------
			x43_command[1] = 0x01;
			x43_command[2] = 0x0D;
			if (i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 2, DEFAULT_RETRY_CNT) < 0)
			{
				printk("[Touch_H][TOUCH_ERR] %s i2c write 43 fail.\n", __func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			msleep(10);

			x43_command[1] = 0x01;
			x43_command[2] = 0x09;
			if (i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 2, DEFAULT_RETRY_CNT) < 0)
			{
				printk("[Touch_H][TOUCH_ERR] %s i2c write 43 fail.\n", __func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			msleep(10);
		}

//-----------------------------------------------------------------------------------------------
// manual mode command : 49 ,program data from flash buffer to this page
//-----------------------------------------------------------------------------------------------
		x43_command[1] = 0x01;
		x43_command[2] = 0x01;
		if (i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 2, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s i2c write 43 fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		x43_command[1] = 0x01;
		x43_command[2] = 0x05;
		if (i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 2, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s i2c write 43 fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		x43_command[1] = 0x01;
		x43_command[2] = 0x01;
		if (i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 2, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s i2c write 43 fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		x43_command[1] = 0x01;
		x43_command[2] = 0x00;
		if (i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 2, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s i2c write 43 fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

//-----------------------------------------------------------------------------------------------
// flash disable
//-----------------------------------------------------------------------------------------------
		x43_command[1] = 0x00;
		if (i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 1, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s i2c write 43 fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

//-----------------------------------------------------------------------------------------------
// leave manual mode
//-----------------------------------------------------------------------------------------------
		x42_command[1] = 0x00;
		if (i2c_himax_write(ts->client, x42_command[0], &x42_command[1], 1, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s i2c write 43 fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

//-----------------------------------------------------------------------------------------------
// lock flash
//-----------------------------------------------------------------------------------------------
		x43_command[1] = 0x01;
		x43_command[2] = 0x00;
		x43_command[3] = 0x06;
		if (i2c_himax_write(ts->client, x43_command[0], &x43_command[1], 3, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s i2c write 43 fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		x44_command[1] = 0x03;
		x44_command[2] = 0x00;
		x44_command[3] = 0x00;
		if (i2c_himax_write(ts->client, x44_command[0], &x44_command[1], 3, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s i2c write 44 fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		x45_command[1] = 0x00;
		x45_command[2] = 0x00;
		x45_command[3] = 0x7D;
		x45_command[4] = 0x03;
		if (i2c_himax_write(ts->client, x45_command[0], &x45_command[1], 4, DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s i2c write 45 fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}
		msleep(10);

		if (i2c_himax_write_command(ts->client, x4A_command[0], DEFAULT_RETRY_CNT) < 0)
		{
			printk("[Touch_H][TOUCH_ERR] %s i2c write 4D fail.\n", __func__);
			goto Flash_Dump_i2c_transfer_error;
		}

		msleep(50);

		buffer_ptr = 128;
		printk("[Touch_H]Himax: Flash page write Complete~~~~~~~~~~~~~~~~~~~~~~~\n");
	}

FLASH_END:

	printk("[Touch_H]Complete~~~~~~~~~~~~~~~~~~~~~~~\n");

	printk("[Touch_H] buffer_ptr = %d \n", buffer_ptr);

	for (i = 0; i < buffer_ptr; i++)
	{
		printk("[Touch_H]%2.2X ", flash_buffer[i]);

		if ((i % 16) == 15)
		{
			printk("[Touch_H]\n");
		}
	}

	printk("[Touch_H]End~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");

	i2c_himax_master_write(ts->client, x43_command, 1, 3);
	msleep(50);

	if (local_flash_command == 2)
	{
		struct file *fn;

		fn = filp_open(FLASH_DUMP_FILE, O_CREAT | O_WRONLY, 0);
		if (!IS_ERR(fn))
		{
			fn->f_op->write(fn, flash_buffer, buffer_ptr * sizeof(uint8_t), &fn->f_pos);
			filp_close(fn, NULL );
		}
	}

#ifdef ENABLE_CHIP_RESET_MACHINE
	if (private_ts->init_success)
	{
		queue_delayed_work(private_ts->himax_wq, &private_ts->himax_chip_reset_work, 0);
	}
#endif

// 	enable_irq(ts->client->irq);
	hx_irq_enable(ts);
	setFlashDumpGoing(false);

	setFlashDumpComplete(1);
	setSysOperation(0);
	return;

Flash_Dump_i2c_transfer_error:

#ifdef ENABLE_CHIP_RESET_MACHINE
	if (private_ts->init_success)
	{
		queue_delayed_work(private_ts->himax_wq, &private_ts->himax_chip_reset_work, 0);
	}
#endif

// 	enable_irq(ts->client->irq);
	hx_irq_enable(ts);
	setFlashDumpGoing(false);
	setFlashDumpComplete(0);
	setFlashDumpFail(1);
	setSysOperation(0);
	return;
}
#endif
//----[HX_TP_SYS_FLASH_DUMP]------------------------------------------------------------------------------end

//----[HX_TP_SYS_SELF_TEST]-----------------------------------------------------------------------------start
#ifdef HX_TP_SYS_SELF_TEST
static ssize_t himax_self_test_setting(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	char buf_tmp[6], length = 0;
	uint8_t veriLen = 0;
	uint8_t write_da[100];
	unsigned long result = 0;
	static uint8_t himax_command = 0;

	memset(buf_tmp, 0x0, sizeof(buf_tmp));
	memset(write_da, 0x0, sizeof(write_da));

	if (buf[0] == 't' && buf[1] == ':' && buf[2] == 'x'){
		if(buf[3] > 47 && buf[3] < 58 && buf[4] > 47 && buf[4] < 58){
			self_test_delay_time = ( buf[3] - 48 ) * 10 + buf[4] - 48;
			printk("self_test_delay_time: %d", self_test_delay_time);
		}
		return count;
	}

	if (buf[0] == 'w' && buf[1] == ':' && buf[2] == 'x') {
		uint8_t loop_i;
		uint16_t base = 5;
		memcpy(buf_tmp, buf + 3, 2);

		if (!strict_strtoul(buf_tmp, 16, &result))
			himax_command = result;
		for (loop_i = 0; loop_i < 100; loop_i++) {
			if (buf[base] == '\n') {
				if (buf[0] == 'w')
					printk("CMD: %x, %x, %d\n",himax_command,write_da[0], length);

				for (veriLen = 0; veriLen < length; veriLen++){
					printk("%x ", *((&write_da[0])+veriLen));
					rFE96_setting[veriLen] = *((&write_da[0])+veriLen);
					printk("rFE96_setting[%d] : %x \n",veriLen ,rFE96_setting[veriLen]);
				}
				printk("\n");
				return count;
			}
			if (buf[base + 1] == 'x') {
				buf_tmp[4] = '\n';
				buf_tmp[5] = '\0';
				memcpy(buf_tmp, buf + base + 2, 2);

				if (!strict_strtoul(buf_tmp, 16, &result))
					write_da[loop_i] = result;

				length++;
			}
			base += 4;
		}
	}
	return count;
}

static ssize_t himax_chip_self_test_function(struct device *dev, struct device_attribute *attr, char *buf)
{
	int val = 0x00;
	int ret = 0;
	int i = 0;

	val = himax_chip_self_test();

	if (val == 0x00)
	{
		return sprintf(buf, "Self_Test Pass\n");
	}
	else
	{
		ret += sprintf(buf+ret, "Self_Test Fail\n");
		for (i = 1; i < 16; i++)
		{
			ret += sprintf(buf+ret, "0xFE96 buff[%d] = 0x%x\n",i,logbuf[i]);
		}
		return ret;
	}
}

static int himax_chip_self_test(void)
{
	uint8_t cmdbuf[11];
// 	int ret = 0;
	uint8_t valuebuf[16];
	int i = 0, pf_value = 0x00;

//----[HX_RST_PIN_FUNC]-----------------------------------------------------------------------------start
#ifdef HX_RST_PIN_FUNC
	himax_HW_reset();
#endif
//----[HX_RST_PIN_FUNC]-------------------------------------------------------------------------------end

	himax_ts_poweron(private_ts);

	if (IC_TYPE == HX_85XX_D_SERIES_PWON)
	{
		//Step 0 : sensor off
		i2c_himax_write(private_ts->client, 0x82, &cmdbuf[0], 0, DEFAULT_RETRY_CNT);
		msleep(120);
	
		//Step 1 : Close Re-Calibration FE02
		//-->Read 0xFE02
		cmdbuf[0] = 0x15;
		i2c_himax_write(private_ts->client, 0xE1, &cmdbuf[0], 1, DEFAULT_RETRY_CNT);
		msleep(10);
	
		cmdbuf[0] = 0x00;
		cmdbuf[1] = 0x02; //FE02
		i2c_himax_write(private_ts->client, 0xD8, &cmdbuf[0], 2, DEFAULT_RETRY_CNT);
		msleep(10);
	
		i2c_himax_read(private_ts->client, 0x5A, valuebuf, 2, DEFAULT_RETRY_CNT);
		msleep(10);
	
		cmdbuf[0] = 0x00;
		i2c_himax_write(private_ts->client, 0xE1, &cmdbuf[0], 1, DEFAULT_RETRY_CNT);
	
		msleep(30);
	
		printk("[Touch_H]:0xFE02_0 = 0x%x\n", valuebuf[0]);
		printk("[Touch_H]:0xFE02_1 = 0x%x\n", valuebuf[1]);
	
		// close re-calibration  , shift first byte of config bank register read issue.
		valuebuf[0] = valuebuf[1] & 0xFD; 
	
		printk("[Touch_H]:0xFE02_valuebuf = 0x%x\n", valuebuf[0]);
	
		//-->Write 0xFE02
		cmdbuf[0] = 0x15;
		i2c_himax_write(private_ts->client, 0xE1, &cmdbuf[0], 1, DEFAULT_RETRY_CNT);
		msleep(10);
	
		cmdbuf[0] = 0x00;
		cmdbuf[1] = 0x02; //FE02
		i2c_himax_write(private_ts->client, 0xD8, &cmdbuf[0], 2, DEFAULT_RETRY_CNT);
		msleep(10);
	
		cmdbuf[0] = valuebuf[0];
		i2c_himax_write(private_ts->client, 0x40, &cmdbuf[0], 1, DEFAULT_RETRY_CNT);
		msleep(10);
	
		cmdbuf[0] = 0x00;
		i2c_himax_write(private_ts->client, 0xE1, &cmdbuf[0], 1, DEFAULT_RETRY_CNT);
	
		msleep(30);
		//0xFE02 Read Back
	
		//-->Read 0xFE02
		cmdbuf[0] = 0x15;
		i2c_himax_write(private_ts->client, 0xE1, &cmdbuf[0], 1, DEFAULT_RETRY_CNT);
		msleep(10);
	
		cmdbuf[0] = 0x00;
		cmdbuf[1] = 0x02; //FE02
		i2c_himax_write(private_ts->client, 0xD8, &cmdbuf[0], 2, DEFAULT_RETRY_CNT);
		msleep(10);
	
		i2c_himax_read(private_ts->client, 0x5A, valuebuf, 2, DEFAULT_RETRY_CNT);
		msleep(10);
	
		cmdbuf[0] = 0x00;
		i2c_himax_write(private_ts->client, 0xE1, &cmdbuf[0], 1, DEFAULT_RETRY_CNT);
		msleep(30);
	
		printk("[Touch_H]:0xFE02_0_back = 0x%x\n", valuebuf[0]);
		printk("[Touch_H]:0xFE02_1_back = 0x%x\n", valuebuf[1]);
	
		//Step 2 : Close Flash-Reload
		cmdbuf[0] = 0x00;
		i2c_himax_write(private_ts->client, 0xE3, &cmdbuf[0], 1, DEFAULT_RETRY_CNT);
	
		msleep(30);
	
		i2c_himax_read(private_ts->client, 0xE3, valuebuf, 1, DEFAULT_RETRY_CNT);
	
		printk("[Touch_H]:0xE3_back = 0x%x\n", valuebuf[0]);
	
		//Step 4 : Write self_test parameter to FE96~FE9D
		//-->Write FE96~FE9D
		cmdbuf[0] = 0x15;
		i2c_himax_write(private_ts->client, 0xE1, &cmdbuf[0], 1, DEFAULT_RETRY_CNT);
		msleep(10);
	
		cmdbuf[0] = 0x00;
		cmdbuf[1] = 0x96; //FE96
		i2c_himax_write(private_ts->client, 0xD8, &cmdbuf[0], 2, DEFAULT_RETRY_CNT);
		msleep(10);
	
		//-->Modify the initial value of self_test.
		cmdbuf[0] = rFE96_setting[0];
		cmdbuf[1] = rFE96_setting[1];
		cmdbuf[2] = rFE96_setting[2];
		cmdbuf[3] = rFE96_setting[3];
		cmdbuf[4] = rFE96_setting[4];
		cmdbuf[5] = rFE96_setting[5];
		cmdbuf[6] = rFE96_setting[6];
		cmdbuf[7] = rFE96_setting[7];
		i2c_himax_write(private_ts->client, 0x40, &cmdbuf[0], 8, DEFAULT_RETRY_CNT);
		msleep(10);
	
		cmdbuf[0] = 0x00;
		i2c_himax_write(private_ts->client, 0xE1, &cmdbuf[0], 1, DEFAULT_RETRY_CNT);
	
		msleep(30);
	
		//Read back
		cmdbuf[0] = 0x15;
		i2c_himax_write(private_ts->client, 0xE1, &cmdbuf[0], 1, DEFAULT_RETRY_CNT);
		msleep(10);
	
		cmdbuf[0] = 0x00;
		cmdbuf[1] = 0x96; //FE96
		i2c_himax_write(private_ts->client, 0xD8, &cmdbuf[0], 2, DEFAULT_RETRY_CNT);
		msleep(10);
	
		i2c_himax_read(private_ts->client, 0x5A, valuebuf, 16, DEFAULT_RETRY_CNT);
		msleep(10);
	
		cmdbuf[0] = 0x00;
		i2c_himax_write(private_ts->client, 0xE1, &cmdbuf[0], 1, DEFAULT_RETRY_CNT);
	
		for (i = 1; i < 16; i++)
		{
			printk("[Touch_H]:0xFE96 buff_back[%d] = 0x%x\n", i, valuebuf[i]);
		}
	
		msleep(30);
	
		//Step 5 : Enter self_test mode
		cmdbuf[0] = 0x16;
		i2c_himax_write(private_ts->client, 0x91, &cmdbuf[0], 1, DEFAULT_RETRY_CNT);
	
		i2c_himax_read(private_ts->client, 0x91, valuebuf, 1, DEFAULT_RETRY_CNT);
	
		printk("[Touch_H]:0x91_back = 0x%x\n", valuebuf[0]);
		msleep(10);
	
		//Step 6 : Sensor On
		i2c_himax_write(private_ts->client, 0x83, &cmdbuf[0], 0, DEFAULT_RETRY_CNT);
	
		mdelay(self_test_delay_time * 1000);
	
		//Step 7 : Sensor Off
		i2c_himax_write(private_ts->client, 0x82, &cmdbuf[0], 0, DEFAULT_RETRY_CNT);
	
		msleep(30);
	
		//Step 8 : Get self_test result
		cmdbuf[0] = 0x15;
		i2c_himax_write(private_ts->client, 0xE1, &cmdbuf[0], 1, DEFAULT_RETRY_CNT);
		msleep(10);
	
		cmdbuf[0] = 0x00;
		cmdbuf[1] = 0x96; //FE96
		i2c_himax_write(private_ts->client, 0xD8, &cmdbuf[0], 2, DEFAULT_RETRY_CNT);
		msleep(10);
	
		i2c_himax_read(private_ts->client, 0x5A, valuebuf, 16, DEFAULT_RETRY_CNT);
		msleep(10);
	
		cmdbuf[0] = 0x00;
		i2c_himax_write(private_ts->client, 0xE1, &cmdbuf[0], 1, DEFAULT_RETRY_CNT);
	
		//Final : Leave self_test mode
		cmdbuf[0] = 0x00;
		i2c_himax_write(private_ts->client, 0x91, &cmdbuf[0], 1, DEFAULT_RETRY_CNT);

		memset(logbuf, 0x00, 16);
	
		//get the self_test result,shift first byte for config bank read issue.
		if (valuebuf[1] == 0xAA)
		{
			printk("[Touch_H]: self-test pass\n");
			pf_value = 0x0;
		}
		else
		{
			printk("[Touch_H]: self-test fail\n");
			pf_value = 0x1;
			logbuf[0] = 0x1;

			for (i = 1; i < 16; i++)
			{
				printk("[Touch_H]:0xFE96 buff[%d] = 0x%x\n", i, valuebuf[i]);
				logbuf[i] = valuebuf[i];
			}
		}
	
	//HW reset and power on again.
//----[HX_RST_PIN_FUNC]-----------------------------------------------------------------------------start
	#ifdef HX_RST_PIN_FUNC
		himax_HW_reset();
	#endif
//----[HX_RST_PIN_FUNC]-------------------------------------------------------------------------------end
	
		himax_ts_poweron(private_ts);

	}

	return pf_value;
}
static DEVICE_ATTR(tp_self_test, HX_RW_ATTR, himax_chip_self_test_function, himax_self_test_setting);
#endif
//----[HX_TP_SYS_SELF_TEST]-------------------------------------------------------------------------------end

//----[HX_TP_SYS_RESET]---------------------------------------------------------------------------------start
#ifdef HX_TP_SYS_RESET
static ssize_t himax_reset_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
//----[ENABLE_CHIP_RESET_MACHINE]-------------------------------------------------------------------start
#ifdef ENABLE_CHIP_RESET_MACHINE
	if (private_ts->init_success)
	{
		queue_delayed_work(private_ts->himax_wq, &private_ts->himax_chip_reset_work, 0);
	}
#endif
//----[ENABLE_CHIP_RESET_MACHINE]--------------------------------------------------------------------end
	return count;
}
static DEVICE_ATTR(reset, HX_WO_ATTR,NULL, himax_reset_set);
#endif
//----[HX_TP_SYS_RESET]----------------------------------------------------------------------------------end

static ssize_t himax_init_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	if(private_ts == NULL)
		return sprintf(buf, "0\n");

	return sprintf(buf, "%d\n", private_ts->init_success);
}
static DEVICE_ATTR(touch_status, HX_RO_ATTR, himax_init_show, NULL);


static ssize_t fw_update_progress(struct device *dev, struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%d\n", update_progress*100/TOTAL_COUNT);
}
static DEVICE_ATTR(update_progress, HX_RO_ATTR, fw_update_progress, NULL);

#if defined(CONFIG_EEPROM_NUVOTON)
#if defined(CONFIG_ASUS_HDMI)
extern bool asus_padstation_exist_realtime(void);
#endif
static ssize_t get_pad(struct device *dev, struct device_attribute *devattr,char *buf)
{
	#if defined(CONFIG_ASUS_HDMI)
	return sprintf(buf, "%d\n", asus_padstation_exist_realtime());
	#else
	return sprintf(buf, "%d\n", 0);
	#endif
}
static DEVICE_ATTR(PAD, HX_RO_ATTR, get_pad, NULL);
#endif

static ssize_t himax_tp_id(struct device *dev, struct device_attribute *attr,char *buf)
{
	int pcbid0 , pcbid1;
	pcbid0 = gpio_get_value(private_ts->pcb_id0);
	pcbid1 = gpio_get_value(private_ts->pcb_id1);

	printk("[Touch_H] PCB_ID1: %d, PCB_ID0: %d\n", pcbid1, pcbid0);

	return sprintf(buf, "%d%d\n", pcbid1, pcbid0);
}
static DEVICE_ATTR(TPID, HX_RO_ATTR, himax_tp_id, NULL);


static ssize_t hx_fw_ver(struct device *dev, struct device_attribute *attr,char *buf)
{
	himax_read_FW_ver();
	return sprintf(buf, "%d-%d-%d-%d\n",
		 FW_VER_MAJ_buff[0], FW_VER_MIN_buff[0], CFG_VER_MAJ_buff[11], CFG_VER_MIN_buff[11]);
}
static DEVICE_ATTR(FW_VER, HX_RO_ATTR, hx_fw_ver, NULL);

static ssize_t hx_conf_ver(struct switch_dev *sdev, char *buf)
{
// 	himax_read_FW_ver();
	return sprintf(buf, "%d-%d\n", CFG_VER_MAJ_buff[11], CFG_VER_MIN_buff[11]);
}


static ssize_t himax_mcu_state(struct device *dev, struct device_attribute *attr,char *buf)
{
	hx_irq_disable(private_ts);

	handshaking_result = himax_hang_shaking(); //0:Running, 1:Stop, 2:I2C Fail

	hx_irq_enable(private_ts);

	return sprintf(buf, "%d\n", handshaking_result);
}
static DEVICE_ATTR(MCU, HX_RO_ATTR, himax_mcu_state, NULL);

static int himax_touch_sysfs_init(void)
{
	android_touch_kobj = kobject_create_and_add("android_touch", NULL );
	if (android_touch_kobj == NULL )
	{
		printk("[Touch_H]TOUCH_ERR: subsystem_register failed\n");
		return -1;
	}

#ifdef HX_TP_SYS_DEBUG_LEVEL
	if(sysfs_create_file(android_touch_kobj, &dev_attr_debug_level.attr))
	{
		printk("[Touch_H]TOUCH_ERR: create_file debug_level failed\n");
		return -1;
	}
#endif

#ifdef HX_TP_SYS_REGISTER
	register_command = 0;
	if(sysfs_create_file(android_touch_kobj, &dev_attr_register.attr))
	{
		printk("[Touch_H]TOUCH_ERR: create_file register failed\n");
		return -1;
	}
#endif

#ifdef HX_TP_SYS_DIAG
	if(sysfs_create_file(android_touch_kobj, &dev_attr_diag.attr))
	{
		printk("[Touch_H]TOUCH_ERR: sysfs_create_file failed\n");
		return -1;
	}
#endif

#ifdef HX_TP_SYS_SELF_TEST
	if(sysfs_create_file(android_touch_kobj, &dev_attr_tp_self_test.attr))
	{
		printk("[Touch_H]TOUCH_ERR: sysfs_create_file dev_attr_tp_self_test failed\n");
		return -1;
	}
#endif

#ifdef HX_TP_SYS_DIAG
	if(sysfs_create_file(android_touch_kobj, &dev_attr_tp_output_raw_data.attr))
	{
		printk("[Touch_H]TOUCH_ERR: sysfs_create_file dev_attr_tp_output_raw_data failed\n");
		return -1;
	}
#endif

#ifdef HX_TP_SYS_FLASH_DUMP
	if(sysfs_create_file(android_touch_kobj, &dev_attr_flash_dump.attr))
	{
		printk("[Touch_H]TOUCH_ERR: sysfs_create_file failed\n");
		return -1;
	}
#endif

#ifdef HX_TP_SYS_RESET
	if(sysfs_create_file(android_touch_kobj, &dev_attr_reset.attr))
	{
		printk("[Touch_H][TOUCH_ERR] sysfs_create_file failed\n");
		return -1;
	}
#endif

 	if(sysfs_create_file(android_touch_kobj, &dev_attr_touch_status.attr))
 	{
 		printk("[Touch_H][TOUCH_ERR] sysfs_create_file failed\n");
 		return -1;
 	}

	if(sysfs_create_file(android_touch_kobj, &dev_attr_update_progress.attr))
	{
		printk("[Touch_H][TOUCH_ERR] sysfs_create_file failed\n");
		return -1;
	}

#if defined(CONFIG_EEPROM_NUVOTON)
	if(sysfs_create_file(android_touch_kobj, &dev_attr_PAD.attr))
	{
		printk("[Touch_H][TOUCH_ERR] sysfs_create_file failed\n");
		return -1;
	}
#endif

	if(sysfs_create_file(android_touch_kobj, &dev_attr_TPID.attr))
	{
		printk("[Touch_H][TOUCH_ERR] sysfs_create_file failed\n");
		return -1;
	}

	if(sysfs_create_file(android_touch_kobj, &dev_attr_FW_VER.attr))
	{
		printk("[Touch_H][TOUCH_ERR] sysfs_create_file failed\n");
		return -1;
	}

	if(sysfs_create_file(android_touch_kobj, &dev_attr_MCU.attr))
	{
		printk("[Touch_H][TOUCH_ERR] sysfs_create_file failed\n");
		return -1;
	}

	return 0;
}

static void himax_touch_sysfs_deinit(void)
{
#ifdef HX_TP_SYS_DIAG
	sysfs_remove_file(android_touch_kobj, &dev_attr_diag.attr);
#endif

#ifdef HX_TP_SYS_DEBUG_LEVEL
	sysfs_remove_file(android_touch_kobj, &dev_attr_debug_level.attr);
#endif

#ifdef HX_TP_SYS_REGISTER
	sysfs_remove_file(android_touch_kobj, &dev_attr_register.attr);
#endif

#ifdef HX_TP_SYS_SELF_TEST
	sysfs_remove_file(android_touch_kobj, &dev_attr_tp_self_test.attr);
#endif

#ifdef HX_TP_SYS_DIAG
	sysfs_remove_file(android_touch_kobj, &dev_attr_tp_output_raw_data.attr);
#endif

#ifdef HX_TP_SYS_RESET
	sysfs_remove_file(android_touch_kobj, &dev_attr_reset.attr);
#endif

#ifdef HX_TP_SYS_FLASH_DUMP
	sysfs_remove_file(android_touch_kobj, &dev_attr_flash_dump.attr);
#endif

	sysfs_remove_file(android_touch_kobj, &dev_attr_update_progress.attr);

#if defined(CONFIG_EEPROM_NUVOTON)
	sysfs_remove_file(android_touch_kobj, &dev_attr_PAD.attr);
#endif

	sysfs_remove_file(android_touch_kobj, &dev_attr_TPID.attr);

	sysfs_remove_file(android_touch_kobj, &dev_attr_FW_VER.attr);

	sysfs_remove_file(android_touch_kobj, &dev_attr_MCU.attr);

 //kobject_del(android_touch_kobj);
}


static struct work_struct checkFWdis_w;
static void checkFWdisable(struct work_struct *work)
{
	himax_read_FW_ver();
#ifdef ASUS_FACTORY_BUILD
	if(g_ASUS_hwID >= A91_SR1){
		if((FW_VER_MAJ_buff[0] == 3) && (FW_VER_MIN_buff[0] < 10)){
		printk("[Touch_H] It's common Firmware, need update.\n");
		common_fw = true;
		private_ts->irq_is_disable = 1;
		disable_irq(private_ts->client->irq);
		}
	}
#endif
}

//=============================================================================================================
//
//	Segment : Himax Touch Work Function
//
//=============================================================================================================

static void himax_ts_work_func(struct work_struct *work)
{
	int i, temp1, temp2, c = 0;
	unsigned int x = 0, y = 0, area = 0, press = 0;
	const unsigned int x_res = HX_X_RES;
	const unsigned int y_res = HX_Y_RES;
	struct himax_ts_data *ts = container_of(work, struct himax_ts_data, work);
	unsigned char check_sum_cal = 0;
	struct i2c_msg msg[2];
	uint8_t start_reg;
	uint8_t buf[128] = { 0 };
	int RawDataLen = 0;

//----[HX_TP_SYS_DIAG]--------------------------------------------------------------------------------start
#ifdef HX_TP_SYS_DIAG
	uint8_t *mutual_data;
	uint8_t *self_data;
	uint8_t diag_cmd;
	int mul_num;
	int self_num;
	int index = 0;

 //coordinate dump start
	char coordinate_char[15 + (HX_MAX_PT + 5) * 2 * 5 + 2];
	struct timeval t;
	struct tm broken;
 //coordinate dump end
#endif
//----[HX_TP_SYS_DIAG]----------------------------------------------------------------------------------end

//Calculate the raw data length
//Bizzy added for common RawData
	int raw_cnt_max = HX_MAX_PT / 4;
	int raw_cnt_rmd = HX_MAX_PT % 4;
	int hx_touch_info_size;

	if (raw_cnt_rmd != 0x00) //more than 4 fingers
	{
		RawDataLen = 128 - ((HX_MAX_PT + raw_cnt_max + 3) * 4) - 1;

		hx_touch_info_size = (HX_MAX_PT + raw_cnt_max + 2) * 4;
	}
	else //less than 4 fingers
	{
		RawDataLen = 128 - ((HX_MAX_PT + raw_cnt_max + 2) * 4) - 1;

		hx_touch_info_size = (HX_MAX_PT + raw_cnt_max + 1) * 4;
	}


	start_reg = HX_CMD_RAE;
	msg[0].addr = ts->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &start_reg;

	msg[1].addr = ts->client->addr;
	msg[1].flags = I2C_M_RD;

#ifdef HX_TP_SYS_DIAG
	if (diag_command) //count the i2c read length
	{
		msg[1].len = 128; //hx_touch_info_size + RawDataLen + 4;	//4: RawData Header
	}
	else	
#endif
		msg[1].len = hx_touch_info_size;


	msg[1].buf = buf;

	//Mutexlock Protect Start
	mutex_lock(&ts->mutex_lock);
	//Mutexlock Protect End

	//read 0x86 all event
	if(i2c_transfer(ts->client->adapter, msg, 2) < 0)
	{
		printk("[Touch_H][TOUCH_ERR] %s:i2c_transfer fail.\n", __func__);
		memset(buf, 0xff, 128);

//----[ENABLE_CHIP_RESET_MACHINE]-------------------------------------------------------------------start
#ifdef ENABLE_CHIP_RESET_MACHINE
		//Mutexlock Protect Start
		mutex_unlock(&ts->mutex_lock);
 		//Mutexlock Protect End

// 		enable_irq(ts->client->irq);
		hx_irq_enable(ts);
		goto work_func_send_i2c_msg_fail;
#endif
//----[ENABLE_CHIP_RESET_MACHINE]---------------------------------------------------------------------end
	}


	//calculate the checksum
	for (i = 0; i < hx_touch_info_size; i++)
	{
		check_sum_cal += buf[i];
	}

	//check sum fail
	if ((check_sum_cal != 0x00) || (buf[HX_TOUCH_INFO_POINT_CNT] & 0xF0) != 0xF0)
	{
		printk("[Touch_H][HIMAX TP MSG] checksum fail : check_sum_cal: 0x%02X\n",
			check_sum_cal);
#ifdef CONFIG_TOUCHSCREEN_DOUBLETAP2WAKE
		if (dt2w_switch > 0) {
			printk("[Touch_H][HIMAX TP MSG] Ignore checksum in dt2w\n");
			goto bypass_checksum_failed_packet;
		}
#endif
		//Mutexlock Protect Start
		mutex_unlock(&ts->mutex_lock);
		//Mutexlock Protect End

// 		enable_irq(ts->client->irq);
		hx_irq_enable(ts);
		return;
	}

  //debug , printk the i2c packet data
//----[HX_TP_SYS_DEBUG_LEVEL]-------------------------------------------------------------------------start
#ifdef HX_TP_SYS_DEBUG_LEVEL
	if (getDebugLevel() & 0x1)
	{
		printk("[Touch_H][HIMAX TP MSG]%s: raw data:\n", __func__);
		for (i = 0; i < 128; i = i + 8)
		{
			printk(
			"[Touch_H]%d: 0x%2.2X, 0x%2.2X, 0x%2.2X, 0x%2.2X, 0x%2.2X, 0x%2.2X, 0x%2.2X, 0x%2.2X \n",
			i, buf[i], buf[i + 1], buf[i + 2], buf[i + 3], buf[i + 4], buf[i + 5],
			buf[i + 6], buf[i + 7]);
		}
	}
#endif
//----[HX_TP_SYS_DEBUG_LEVEL]---------------------------------------------------------------------------end

//touch monitor raw data fetch
//----[HX_TP_SYS_DIAG]--------------------------------------------------------------------------------start
#ifdef HX_TP_SYS_DIAG
	diag_cmd = getDiagCommand();
	if (diag_cmd >= 1 && diag_cmd <= 6)
	{


		//Check 128th byte CRC
		for (i = hx_touch_info_size, check_sum_cal = 0; i < 128; i++)
		{
			check_sum_cal += buf[i];
		}

		if (check_sum_cal % 0x100 != 0)
		{
			goto bypass_checksum_failed_packet;
		}
		

		mutual_data = getMutualBuffer();
		self_data = getSelfBuffer();

		// initiallize the block number of mutual and self
		mul_num = getXChannel() * getYChannel();

		self_num = getXChannel() + getYChannel();

		//Himax: Check Raw-Data Header
		if (buf[hx_touch_info_size] == buf[hx_touch_info_size + 1]
			&& buf[hx_touch_info_size + 1] == buf[hx_touch_info_size + 2]
			&& buf[hx_touch_info_size + 2] == buf[hx_touch_info_size + 3]
			&& buf[hx_touch_info_size] > 0)
		{
			index = (buf[hx_touch_info_size] - 1) * RawDataLen;
			//printk("[Touch_H]Header[%d]: %x, %x, %x, %x, mutual: %d, self: %d\n", index, buf[56], buf[57], buf[58], buf[59], mul_num, self_num);
			for (i = 0; i < RawDataLen; i++)
			{

				temp1 = index + i;

				if (temp1 < mul_num)
				{//mutual
					mutual_data[index + i] = buf[i + hx_touch_info_size + 4]; //4: RawData Header
				}
				else
				{//self

					temp1 = i + index;
					temp2 = self_num + mul_num;
					

					if (temp1 >= temp2)
					{
						break;
					}

					self_data[i + index - mul_num] = buf[i + hx_touch_info_size + 4];
					//4: RawData Header
				}
			}
		}
		else
		{
			printk("[Touch_H][HIMAX TP MSG]%s: header format is wrong!\n", __func__);
		}
	}
	else if (diag_cmd == 7)
	{
		memcpy(&(diag_coor[0]), &buf[0], 128);
	}
	//coordinate dump start
	if (coordinate_dump_enable == 1)
	{
		for (i = 0; i < (15 + (HX_MAX_PT + 5) * 2 * 5); i++)
		{
			coordinate_char[i] = 0x20;
		}
		coordinate_char[15 + (HX_MAX_PT + 5) * 2 * 5] = 0xD;
		coordinate_char[15 + (HX_MAX_PT + 5) * 2 * 5 + 1] = 0xA;
	}
	//coordinate dump end
#endif
//----[HX_TP_SYS_DIAG]----------------------------------------------------------------------------------end

bypass_checksum_failed_packet:

#if defined(HX_EN_SEL_BUTTON) || defined(HX_EN_MUT_BUTTON) 
	tpd_key = (buf[HX_TOUCH_INFO_POINT_CNT+2]>>4);
	if(tpd_key == 0x0F)
	{
		tpd_key = 0xFF;
	}	 
	//printk("TPD BT:  %x\r\n", tpd_key);
#else
	tpd_key = 0xFF;
#endif 

	p_point_num = hx_point_num;

	if (buf[HX_TOUCH_INFO_POINT_CNT] == 0xff)
	{
		hx_point_num = 0;
	}
	else
	{
		hx_point_num = buf[HX_TOUCH_INFO_POINT_CNT] & 0x0f;
	}

	// Touch Point information
	if (hx_point_num != 0 && tpd_key == 0xFF)
	{
	// parse the point information
		for (c = 0; c < HX_MAX_PT; c++)
		{
			if (buf[4 * c] != 0xFF)
			{
				// x and y axis
				x = buf[4 * c + 1] | (buf[4 * c] << 8);
				y = buf[4 * c + 3] | (buf[4 * c + 2] << 8);

				if ((x <= x_res) && (y <= y_res))
				{
					// caculate the pressure and area
					press = buf[4 * HX_MAX_PT + c];
					area = press;
					if (area > 31)
					{
						area = (area >> 3);
					}

		// kernel call for report point area, pressure and x-y axis
					input_report_key(ts->input_dev, BTN_TOUCH, 1);             // touch down
					input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, c); //ID of touched point
					input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, area); //Finger Size
					input_report_abs(ts->input_dev, ABS_MT_PRESSURE, press);   // Pressure
					input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);     // X axis
					input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);     // Y axis

					input_mt_sync(ts->input_dev);


					if(touch_debug_mask & DEF_POINT_INFO){
						if(!point_printed){
							//printk("[Touch_H] Touch Down !\n");
							point_printed = 1;
						}
					}
					else if(touch_debug_mask & ALL_POINT_INFO){
						if (finger_count[c][0] == 0)
							//printk("[Touch_H][%d][X,Y,W,P][%d,%d,%d,%d]\n"
							//	, c, x, y, area, press);
					
						finger_count[c][0]++;
						finger_count[c][1] = true;
					
					}
					else if(touch_debug_mask & DET_POINT_INFO){
						if ((finger_count[c][0] % 100) == 0)
							//printk("[Touch_H][%d][X,Y,W,P][%d,%d,%d,%d]-[%d]\n"
							//	, c, x, y, area, press, finger_count[c][0]);
					
						finger_count[c][0]++;
						finger_count[c][1] = true;
					}


//----[HX_TP_SYS_DIAG]--------------------------------------------------------------------------------start
#ifdef HX_TP_SYS_DIAG
					//coordinate dump start
					if (coordinate_dump_enable == 1)
					{
						do_gettimeofday(&t);
						time_to_tm(t.tv_sec, 0, &broken);

						sprintf(&coordinate_char[0], "%2d:%2d:%2d:%3li,", broken.tm_hour,
							broken.tm_min, broken.tm_sec, t.tv_usec / 1000);

						sprintf(&coordinate_char[15 + (i * 2) * 5], "%4d,", x);
						sprintf(&coordinate_char[15 + (i * 2) * 5 + 5], "%4d,", y);

						coordinate_fn->f_op->write(coordinate_fn, &coordinate_char[0],
						15 + (HX_MAX_PT + 5) * 2 * sizeof(char) * 5 + 2,
						&coordinate_fn->f_pos);
					}
					//coordinate dump end
#endif
//----[HX_TP_SYS_DIAG]----------------------------------------------------------------------------------end
				}
			}
			else
			{

				input_mt_sync(ts->input_dev);

				if(touch_debug_mask & (ALL_POINT_INFO|DET_POINT_INFO)){
					if (finger_count[c][1] == true)
					{
						//printk("[Touch_H][%d]-[%d] Touch Up !!\n", c, finger_count[c][0]);
						
						finger_count[c][0] = 0;
						finger_count[c][1] = false;
					}
				}

			}
		}
		input_sync(ts->input_dev);


	}
	else if (hx_point_num == 0 && tpd_key != 0xFF)
	{
		input_report_key(ts->input_dev, tpd_keys_local[tpd_key - 1], 1);
		input_mt_sync(ts->input_dev);
		input_sync(ts->input_dev);
	
		//printk("[Touch_H] Press Button %d \n",tpd_keys_local[tpd_key - 1]);

	}
	else if (hx_point_num == 0 && tpd_key == 0xFF)
	{
		if (tpd_key_old != 0xFF)
		{
			input_report_key(ts->input_dev, tpd_keys_local[tpd_key_old - 1], 0);
			input_mt_sync(ts->input_dev);
			input_sync(ts->input_dev);
			
			//printk("[Touch_H] Release Button %d \n",tpd_keys_local[tpd_key_old - 1]);
			
		}
		else
		{
			// leave event
			input_report_key(ts->input_dev, BTN_TOUCH, 0);  // touch up
			input_mt_sync(ts->input_dev);
			input_sync(ts->input_dev);

			if(touch_debug_mask & DEF_POINT_INFO){
				if(point_printed){
					//printk("[Touch_H] Touch Up!!\n");
#ifdef CONFIG_TOUCHSCREEN_DOUBLETAP2WAKE
							if (dt2w_switch > 0 && ts_suspend == true) {
								input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, -1); //DoubleTap
								input_mt_sync(ts->input_dev);
							}
#endif
					point_printed = 0;
				}
			}
			else if(touch_debug_mask & (ALL_POINT_INFO|DET_POINT_INFO)){
				for(c=0 ; c<10 ; c++)
				{
					if( finger_count[c][1] == true )
					{
						//printk("[Touch_H][%d]-[%d] Touch Up!!\n", c, finger_count[c][0]);
						finger_count[c][0] = 0;
						finger_count[c][1] = false;
						break;
					}
				}
			}

//----[HX_TP_SYS_DIAG]--------------------------------------------------------------------------------start
#ifdef HX_TP_SYS_DIAG
		//coordinate dump start
			if (coordinate_dump_enable == 1)
			{
				do_gettimeofday(&t);
				time_to_tm(t.tv_sec, 0, &broken);

				sprintf(&coordinate_char[0], "%2d:%2d:%2d:%lu,", broken.tm_hour,
					broken.tm_min, broken.tm_sec, t.tv_usec / 1000);
					sprintf(&coordinate_char[15], "Touch up!");
				coordinate_fn->f_op->write(coordinate_fn, &coordinate_char[0],
						15 + (HX_MAX_PT + 5) * 2 * sizeof(char) * 5 + 2,
						&coordinate_fn->f_pos);
			}
		//coordinate dump end
#endif
//----[HX_TP_SYS_DIAG]----------------------------------------------------------------------------------end
		}
	}

	tpd_key_old = tpd_key;

	//Mutexlock Protect Start
	mutex_unlock(&ts->mutex_lock);
	//Mutexlock Protect End

// 	enable_irq(ts->client->irq);
	hx_irq_enable(ts);

	return;

work_func_send_i2c_msg_fail:

	printk("[Touch_H][TOUCH_ERR] work_func_send_i2c_msg_fail: %d \n", __LINE__);

//----[ENABLE_CHIP_RESET_MACHINE]-------------------------------------------------------------------start
#ifdef ENABLE_CHIP_RESET_MACHINE
	if (private_ts->init_success)
	{
		queue_delayed_work(ts->himax_wq, &ts->himax_chip_reset_work, 0);
	}
#endif
//----[ENABLE_CHIP_RESET_MACHINE]---------------------------------------------------------------------end

	return;
}

/* +++jacob add for resume touch when dds finish after detach pad+++ */
static int himax_dds_touch_resume(const char *val, struct kernel_param *kp) {

	int ret;
	if (ts_resume_trigger == true){
		if(touch_debug_mask & ALL_POINT_INFO)
			printk("[Touch_H] %s resume had been trigger , reject the resume action\n",__func__);
		return 0;
	}
	ts_resume_trigger = true;
	ret = param_set_int(val, kp);
	if(touch_debug_mask & ALL_POINT_INFO)
	printk("[Touch_H] %s dds_touch_resume_mode == %d !! \n", __func__, dds_touch_resume_mode);
	if (ret) {
		printk("[Touch_H] %s param_set_int error !! \n", __func__);
		}
	if (dds_touch_resume_mode == 1 ) {

		if (dds_notify_enable == true) {
			printk("[Touch_H] (%s) resume notyfy by dds++\n", __func__);
#if defined(CONFIG_EEPROM_NUVOTON)
			cancel_delayed_work_sync(&mp_detach_w);
#endif
			ts_inpad = false;
			if(touch_debug_mask & ALL_POINT_INFO)
				printk("[Touch_H] Pad detach resume!!!\n");
			himax_ts_resume();
			// 	himax_usb_detection(0);
			dds_notify_enable = false;
			printk("[Touch_H] (%s) resume notyfy by dds--\n", __func__);
			}
	}
	ts_resume_trigger = false;
	return 0;
}
/* ---jacob add for resume touch when dds finish after detach pad--- */

static void ts_resume_work(struct work_struct *work)
{
	himax_resume();
}

//=============================================================================================================
//
//	Segment : Himax Linux Driver Probe Function
//
//=============================================================================================================

//----[ interrupt ]---------------------------------------------------------------------------------------start
static irqreturn_t himax_ts_irq_handler(int irq, void *dev_id)
{
	struct himax_ts_data *ts = dev_id;
	struct i2c_client *client = ts->client;

	dev_dbg(&client->dev, "[HIMAX TP MSG] %s\n", __func__);
// 	disable_irq_nosync(ts->client->irq);
	hx_nosync_irq_disable(ts);
	queue_work(ts->himax_wq, &ts->work);

	return IRQ_HANDLED;
}

static int himax_ts_register_interrupt(struct i2c_client *client)
{
	struct himax_ts_data *ts = i2c_get_clientdata(client);
	int err = 0;

	if (HX_INT_IS_EDGE)	//edge trigger
	{
		err = request_irq(client->irq, himax_ts_irq_handler, IRQF_TRIGGER_FALLING,
				client->name, ts);
	}
	else	//low level trigger
	{
		err = request_irq(client->irq, himax_ts_irq_handler, IRQF_TRIGGER_LOW,
				client->name, ts);
	}

	if (err)
	{

		printk("[Touch_H][TOUCH_ERR] %s: request_irq %d failed\n", __func__, client->irq);
	}
/*	else
	{
		printk("[Touch_H]%s request_irq ok \n", __func__);
	}
*/
	return err;
}
//----[ interrupt ]-----------------------------------------------------------------------------------------end


//----[ i2c ]---------------------------------------------------------------------------------------------start
static int himax_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct himax_i2c_platform_data *pdata;
	struct himax_ts_data *ts;
#if defined(HX_EN_SEL_BUTTON) || defined(HX_EN_MUT_BUTTON)
	int i = 0;
#endif
	struct regulator *hx_vcca;

	printk("[Touch_H] himax_ts_probe +++\n");

	himax_touch_sysfs_init();

	hx_vcca = regulator_get(&client->dev, "8941_l23");
	if (IS_ERR(hx_vcca)) {
		printk("[Touch_H][TOUCH_ERR] unable to get pm8941_l23\n");
	}

	if (regulator_set_voltage(hx_vcca, 3000000,3000000)) {
		printk( "[Touch_H][TOUCH_ERR] unable to set voltage level for pm8941_l23\n");
	}
 
	if (regulator_enable(hx_vcca)) {
		printk( "[Touch_H][TOUCH_ERR] unable to enable the pm8941_l23\n");
	}

//*********************************************************************************************************
// Allocate the himax_ts_data
//*********************************************************************************************************
	ts = kzalloc(sizeof(struct himax_ts_data), GFP_KERNEL);
	if (ts == NULL)
	{
		printk("[Touch_H][HIMAX TP ERROR] %s: allocate himax_ts_data failed\n", __func__);
		goto err_alloc_data_failed;
	}

	ts->client = client;
	ts->init_success = 0;
	i2c_set_clientdata(client, ts);

	private_ts = ts;

	if (client->dev.of_node)
	{

		pdata = devm_kzalloc(&client->dev, sizeof(struct himax_i2c_platform_data), GFP_KERNEL);
		if (!pdata)
		{
			printk("[Touch_H][TOUCH_ERR] Failed to allocate memory\n");
			goto err_alloc_data_failed;
		}

		pdata->rst_gpio = of_get_named_gpio_flags(client->dev.of_node, 
					"himax,reset-gpio", 0, &pdata->reset_gpio_flags);
		pdata->intr_gpio = of_get_named_gpio_flags(client->dev.of_node, 
					"himax,irq-gpio", 0, &pdata->irq_gpio_flags);

		pdata->pcb_id0 = of_get_named_gpio_flags(client->dev.of_node, 
					"himax,pcb-id0", 0, &pdata->pcb_id0_flags);
		pdata->pcb_id1 = of_get_named_gpio_flags(client->dev.of_node, 
					"himax,pcb-id1", 0, &pdata->pcb_id1_flags);

	}
	else
		pdata = client->dev.platform_data;

//*********************************************************************************************************
// TODO Interrupt Pin
//*********************************************************************************************************
	if(gpio_request(pdata->intr_gpio, "himax-irq"))
	{
		printk("[Touch_H][HIMAX PORTING ERROR] interrupt gpio %d request fail.\n", pdata->intr_gpio);
		goto err_request_irq_failed;
	}
	gpio_direction_input(pdata->intr_gpio);

	ts->intr_gpio = pdata->intr_gpio;
	ts->client->irq = gpio_to_irq(ts->intr_gpio);

//*********************************************************************************************************
// TODO Reset Pin
//*********************************************************************************************************
	if(gpio_request(pdata->rst_gpio, "himax-reset"))
	{
		printk("[Touch_H][HIMAX PORTING ERROR] reset gpio %d request fail.\n", pdata->rst_gpio);
		goto err_request_reset_failed;
	}
	gpio_direction_output(pdata->rst_gpio, 1);	//reset set high
	msleep(100);

#ifdef HX_RST_PIN_FUNC
	ts->rst_gpio = pdata->rst_gpio;
#endif

	//HanS: support TP ID info +++
	if(gpio_request( pdata->pcb_id0, "PCB_ID0"))
	{
		printk("[Touch_H] PCB_ID0 gpio %d request fail.\n", pdata->pcb_id0);
		goto err_request_pcb0_failed;
	}

	if(gpio_request( pdata->pcb_id1, "PCB_ID1"))
	{
		printk("[Touch_H] PCB_ID1 gpio %d request fail.\n", pdata->pcb_id1);
		goto err_request_pcb1_failed;
	}

	ts->pcb_id0 = pdata->pcb_id0;
	ts->pcb_id1 = pdata->pcb_id1;
	//HanS: support TP ID info ---

//*********************************************************************************************************
// Check i2c functionality
//*********************************************************************************************************
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		printk("[Touch_H][HIMAX TP ERROR] %s: i2c check functionality error\n", __func__);
		goto err_check_functionality_failed;
	}

//*********************************************************************************************************
// Create Work Queue
//*********************************************************************************************************

//----[ HX_TP_SYS_FLASH_DUMP ]------------------------------------------------------------------------start
#ifdef  HX_TP_SYS_FLASH_DUMP
	ts->flash_wq = create_singlethread_workqueue("himax_flash_wq");
	if (!ts->flash_wq)
	{
		printk("[Touch_H][HIMAX TP ERROR] %s: create flash workqueue failed\n", __func__);
		goto err_create_wq_failed;
	}
#endif
//----[ HX_TP_SYS_FLASH_DUMP ]--------------------------------------------------------------------------end

	ts->himax_wq = create_singlethread_workqueue("himax_wq");
	if (!ts->himax_wq)
	{
		printk("[Touch_H][HIMAX TP ERROR] %s: create workqueue failed\n", __func__);
		goto err_create_wq_failed;
	}

	ts->usb_wq = create_singlethread_workqueue("himax_usb_wq");
	if (!ts->usb_wq)
	{
		printk("[Touch_H][HIMAX TP ERROR] %s: create usb workqueue failed\n", __func__);
		goto err_create_wq_failed;
	}
	INIT_WORK(&ts->usb_detect_work, himax_cable_status);

//----[ENABLE_CHIP_RESET_MACHINE]---------------------------------------------------------------------start
#ifdef ENABLE_CHIP_RESET_MACHINE
	INIT_DELAYED_WORK(&ts->himax_chip_reset_work, himax_chip_reset_function);
#endif
//----[ENABLE_CHIP_RESET_MACHINE]-----------------------------------------------------------------------end


//----[HX_TP_SYS_FLASH_DUMP]--------------------------------------------------------------------------start
#ifdef HX_TP_SYS_FLASH_DUMP
	INIT_WORK(&ts->flash_work, himax_ts_flash_work_func);
#endif
 //----[HX_TP_SYS_FLASH_DUMP]----------------------------------------------------------------------------end

	INIT_WORK(&ts->work, himax_ts_work_func);

 //*********************************************************************************************************
 // Himax Information / Initial, I2C must be ready
 //*********************************************************************************************************

	if(himax_ic_package_check(ts))
	{
		goto err_ic_check_fail;
	}

	himax_touch_information();
	calculate_point_number();
/*
	ts->fb_notif.notifier_call = fb_notifier_callback; 

	if(fb_register_client(&ts->fb_notif))
		dev_err(&client->dev, "[Touch_H][TOUCH_ERR] Unable to register fb_notifier\n");
*/
//----[HX_TP_SYS_FLASH_DUMP]--------------------------------------------------------------------------start
#ifdef HX_TP_SYS_FLASH_DUMP
	setSysOperation(0);
	setFlashBuffer();
#endif
//----[HX_TP_SYS_FLASH_DUMP]----------------------------------------------------------------------------end

//----[HX_TP_SYS_DIAG]--------------------------------------------------------------------------------start
#ifdef HX_TP_SYS_DIAG
	setXChannel(HX_RX_NUM); // X channel
	setYChannel(HX_TX_NUM); // Y channel

	setMutualBuffer();
	if (getMutualBuffer() == NULL )
	{
		printk("[Touch_H][TOUCH_ERR] %s: mutual buffer allocate fail failed\n", __func__);
		goto err_mutual_buffer_fail;
	}
#endif
 //----[HX_TP_SYS_DIAG]----------------------------------------------------------------------------------end


	spin_lock_init(&ts->irq_lock);

 	//Mutexlock Protect Start
	mutex_init(&ts->mutex_lock);
 	//Mutexlock Protect End

	//Wakelock Protect Start
	wake_lock_init(&ts->wake_lock, WAKE_LOCK_SUSPEND, "himax_touch_wake_lock");
	//Wakelock Protect End

//*********************************************************************************************************
// Allocate Input Device
//*********************************************************************************************************
	ts->input_dev = input_allocate_device();

	if (ts->input_dev == NULL )
	{
		dev_err(&client->dev, "[TOUCH_ERR] Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}

	ts->input_dev->name = INPUT_DEV_NAME;
/*	printk("[Touch_H] Max X=%d, Max Y=%d\n", HX_X_RES, HX_Y_RES);*/

//----[HX_EN_XXX_BUTTON]----------------------------------------------------------------------------------start	
#if defined(HX_EN_SEL_BUTTON) || defined(HX_EN_MUT_BUTTON)    
	for(i = 0; i < HX_BT_NUM; i++)
	{
		set_bit(tpd_keys_local[i], ts->input_dev->keybit);
	}
#endif    
//----[HX_EN_XXX_BUTTON]------------------------------------------------------------------------------------end	

	__set_bit(EV_KEY, ts->input_dev->evbit);
	__set_bit(EV_ABS, ts->input_dev->evbit);
	__set_bit(BTN_TOUCH, ts->input_dev->keybit);

	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, HX_MAX_PT, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, HX_X_RES, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, HX_Y_RES, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 31, 0, 0); //Finger Size
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 31, 0, 0); //Touch Size
	input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, 0xFF, 0, 0);

	if(input_register_device(ts->input_dev))
	{
		dev_err(&client->dev, "[TOUCH_ERR] %s: unable to register %s input device\n", __func__,
			ts->input_dev->name);
		goto err_input_register_device_failed;
	}

 //*********************************************************************************************************
 // Himax Touch IC Power on
 //*********************************************************************************************************

 //HW Reset

 //----[HX_RST_PIN_FUNC]-------------------------------------------------------------------------------start
#ifdef HX_RST_PIN_FUNC
	if(gpio_direction_output(ts->rst_gpio, 1))
	{
		printk("[Touch_H][TOUCH_ERR] Failed to set reset direction\n");
		goto err_gpio_reset_direction_fail;
 		//gpio_free(himax_chip->rst_gpio);
	}

	gpio_set_value(ts->rst_gpio, 0);
	msleep(30);
	gpio_set_value(ts->rst_gpio, 1);
	msleep(30);
#endif
//----[HX_RST_PIN_FUNC]---------------------------------------------------------------------------------end

//PowerOn
	if(himax_ts_poweron(ts))
	{
		goto err_poweron_fail;
	}

 //TODO check the interrupt's API function by differrnt platform
	if(himax_ts_register_interrupt(ts->client))
	{
		goto err_register_interrupt;
	}

// 	if(gpio_get_value(ts->intr_gpio) == 0)
// 	{
// 		printk("[Touch_H] %s: handle missed interrupt\n", __func__);
// 		himax_ts_irq_handler(client->irq, ts);
// 	}

//*********************************************************************************************************
// Remain work of Probe
//*********************************************************************************************************


	hx_sdev.name = "touch";
	hx_sdev.print_name = hx_conf_ver;

	if(switch_dev_register(&hx_sdev) < 0)
	{
		goto err_sdev_register_fail;
	}

#if defined(CONFIG_EEPROM_NUVOTON)
	INIT_WORK(&mp_attach_w, attach_padstation_work);
	INIT_DELAYED_WORK(&mp_detach_w, detach_padstation_work);
#endif
	INIT_DELAYED_WORK(&hx_resume_w, ts_resume_work);

	hx_attach_detach_wq = create_singlethread_workqueue("hx_attach_detach_wq");
	if (!hx_attach_detach_wq) {
		printk("[Touch_H][TOUCH_ERR] %s: create workqueue failed: hx_attach_detach_wq\n", __func__);
	}


#if defined(CONFIG_EEPROM_NUVOTON)
	register_microp_notifier(&touch_mp_notifier);
	notify_register_microp_notifier(&touch_mp_notifier, "hx8528_d48");
#endif

	ts->init_success = 1;

#ifdef ASUS_FACTORY_BUILD
	hx_irq_disable(ts);
#endif

	INIT_WORK(&checkFWdis_w, checkFWdisable);
	queue_work(ts->himax_wq, &checkFWdis_w);

	printk("[Touch_H] himax_ts_probe ---\n");

	return 0;

err_sdev_register_fail:
err_register_interrupt:
err_poweron_fail:
err_gpio_reset_direction_fail:
err_input_register_device_failed:
err_input_dev_alloc_failed:
	if (ts->input_dev)
	{
		input_free_device(ts->input_dev);
	}

	//Mutexlock Protect Start
	mutex_destroy(&ts->mutex_lock);
	//Mutexlock Protect End

	//Wakelock Protect Start
	wake_lock_destroy(&ts->wake_lock);
 	//Wakelock Protect End

err_mutual_buffer_fail:
/*
	fb_unregister_client(&ts->fb_notif);
*/
err_ic_check_fail:

//----[ENABLE_CHIP_RESET_MACHINE]---------------------------------------------------------------------start
#ifdef ENABLE_CHIP_RESET_MACHINE
	cancel_delayed_work(&ts->himax_chip_reset_work);
#endif
//----[ENABLE_CHIP_RESET_MACHINE]-----------------------------------------------------------------------end

err_create_wq_failed:

	if(ts->usb_wq)
	{
		destroy_workqueue(ts->usb_wq);
	}

	if(ts->himax_wq)
	{
		destroy_workqueue(ts->himax_wq);
	}

//----[ HX_TP_SYS_FLASH_DUMP ]------------------------------------------------------------------------start
#ifdef  HX_TP_SYS_FLASH_DUMP
	if(ts->flash_wq)
	{
		destroy_workqueue(ts->flash_wq);
	}
#endif
//----[ HX_TP_SYS_FLASH_DUMP ]--------------------------------------------------------------------------end

err_check_functionality_failed:
err_request_pcb1_failed:
	gpio_free(pdata->pcb_id1);

err_request_pcb0_failed:
	gpio_free(pdata->pcb_id0);

err_request_reset_failed:
	disable_irq(ts->client->irq);
	gpio_free(pdata->rst_gpio);

err_request_irq_failed:
	gpio_free(pdata->intr_gpio);

err_alloc_data_failed:
	kfree(ts);
	himax_touch_sysfs_deinit();
	printk("[Touch_H][TOUCH_ERR] himax_ts_probe fail!!!\n");
	return -1;
}

static int himax_ts_remove(struct i2c_client *client)
{
	struct himax_ts_data *ts = i2c_get_clientdata(client);

	himax_touch_sysfs_deinit();
/*
	if (fb_unregister_client(&ts->fb_notif))
	dev_err(&client->dev, "[Touch_H] Error occurred while unregistering fb_notifier.\n");
*/
	free_irq(client->irq, ts);

	 switch_dev_unregister(&hx_sdev);

	//Mutexlock Protect Start
	mutex_destroy(&ts->mutex_lock);
	//Mutexlock Protect End

	if (ts->himax_wq)
	{
		destroy_workqueue(ts->himax_wq);
	}
	input_unregister_device(ts->input_dev);

	//Wakelock Protect Start
	wake_lock_destroy(&ts->wake_lock);
	//Wakelock Protect End

	kfree(ts);

	return 0;
}

static const struct i2c_device_id himax_ts_id[] =
{
	{ HIMAX_TS_NAME, 0 },
	{ }
};

static struct of_device_id himax_match_table[] =
{
	{ .compatible = "himax,himax-ts", },
	{ }, 
};

static struct i2c_driver himax_ts_driver =
{ 
	.probe = himax_ts_probe, 
	.remove = himax_ts_remove,
	.id_table = himax_ts_id, 
	.driver = { 
		.name = HIMAX_TS_NAME, 
		.owner = THIS_MODULE, 
		.of_match_table = himax_match_table,
	}, 
};

static int __devinit himax_ts_init(void)
{
	printk("[Touch_H] %s\n", __func__);
	return i2c_add_driver(&himax_ts_driver);
 //return himax_add_i2c_device(&himax_ts_info,&himax_ts_driver);
}

static void __exit himax_ts_exit(void)
{
	i2c_del_driver(&himax_ts_driver);
	return;
}

module_init( himax_ts_init);
module_exit( himax_ts_exit);

MODULE_DESCRIPTION("Himax Touchscreen Driver");
MODULE_LICENSE("GPL");
//----[ i2c ]-----------------------------------------------------------------------------------------------end

//=============================================================================================================
//
//  Segment : Other Function
//
//=============================================================================================================
void himax_usb_detection(bool plugin)
{
	if (private_ts == NULL)
	{
		printk("[Touch_H] himax_ts_data is null, skip %s \n",__func__);
		return;
	}

	if(ts_inpad){
		if(touch_debug_mask & SUS_INFO)
			printk("[Touch_H] reject %s \n",__func__);
		return;
	}

	if(private_ts->init_success == 1)
	{
		if (plugin)
			private_ts->usb_status = 1; //usb plug in
		else
			private_ts->usb_status = 0; //no usb

		queue_work(private_ts->usb_wq, &private_ts->usb_detect_work);
	}
}
	
static void himax_cable_status(struct work_struct *work)
{
	uint8_t buf0[2] = {0};
	int status = private_ts->usb_status;

	if(ts_inpad){
		if(touch_debug_mask & SUS_INFO)
			printk("[Touch_H] reject %s \n",__func__);
		return;
	}

	printk("[Touch_H] cable_status=%d, init_success=%d.\n", status, private_ts->init_success);

	if (private_ts->init_success == 1 && !ts_inpad)
	{
		if (status == 0) //non usb
		{
			buf0[0] = 0x00;
			i2c_himax_write(private_ts->client, 0x90, &buf0[0], 1, DEFAULT_RETRY_CNT);
		}
		else if (status == 1) //usb plug in
		{
			buf0[0] = 0x01;
			i2c_himax_write(private_ts->client, 0x90, &buf0[0], 1, DEFAULT_RETRY_CNT);
		}
	}
}

/*
static int fb_notifier_callback(struct notifier_block *self,unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct himax_ts_data *himax_dev_data = container_of(self, struct himax_ts_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK && himax_dev_data && himax_dev_data->client)
	{
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
		{
			//himax_ts_resume(himax_dev_data->client);
//			printk("should resume %s\n",__func__);
			if(delayed_work_pending(&hx_resume_w)) {
				cancel_delayed_work_sync(&hx_resume_w);
				printk("[Touch_H] cancel last resume_work\n");
			}
			queue_delayed_work(hx_attach_detach_wq, &hx_resume_w, msecs_to_jiffies(2));

		}
		else if (*blank == FB_BLANK_POWERDOWN)
		{
			if(!ts_inpad)
			{
				himax_ts_suspend(himax_dev_data->client, PMSG_SUSPEND);
			}
		}
	}

	return 0;
}
*/

#if defined(CONFIG_EEPROM_NUVOTON)
static void attach_padstation_work(struct work_struct *work)
{
	printk("[Touch_H] attach_padstation_work()++\n");

	if(touch_debug_mask & SUS_INFO)
		printk("[Touch_H] Pad attach suspend!!!\n");

	himax_ts_suspend();

	printk("[Touch_H] attach_padstation_work()--\n");
}

static void detach_padstation_work(struct work_struct *work)
{
	/* +++jacob add for resume touch when dds finish after detach pad+++ */
	if (ts_resume_trigger == true){
		if(touch_debug_mask & SUS_INFO)
			printk("[Touch_H] %s resume had been trigger , reject the resume action\n",__func__);
		return;
	}
	ts_resume_trigger = true;
	/* ---jacob add for resume touch when dds finish after detach pad--- */
	printk("[Touch_H] detach_padstation_work()++\n");
	ts_inpad = false;

	if(touch_debug_mask & SUS_INFO)
		printk("[Touch_H] Pad detach resume!!!\n");

	himax_ts_resume();
	dds_notify_enable = false;	/* jacob add for resume touch when dds finish after detach pad */
// 	himax_usb_detection(0);
	ts_resume_trigger = false;	/* jacob add for resume touch when dds finish after detach pad */
	printk("[Touch_H] detach_padstation_work()--\n");
}

static int touch_mp_event(struct notifier_block *this, unsigned long event, void *ptr)
{
	if(touch_debug_mask & SUS_INFO)
		printk("[Touch_H] Mircop event %d\n",(unsigned int)event);

	switch (event) {

		case P01_ADD:{
			hx_irq_disable(private_ts);
			ts_inpad = true;
			dds_notify_enable = false;	/* jacob add for resume touch when dds finish after detach pad */
			if(delayed_work_pending(&mp_detach_w)) {
				cancel_delayed_work_sync(&mp_detach_w);
				printk("[Touch_H] cancel last detach_work\n");
			}
			queue_work(hx_attach_detach_wq, &mp_attach_w);
			return NOTIFY_DONE;
		}
		case P01_REMOVE:{
	/* +++jacob add for resume touch when dds finish after detach pad+++ */
			printk("[Touch_H] dds_notify_enable\n");
			dds_notify_enable = true;
	/* ---jacob add for resume touch when dds finish after detach pad--- */
			if(delayed_work_pending(&mp_detach_w)) {
				cancel_delayed_work_sync(&mp_detach_w);
				printk("[Touch_H] cancel last detach_work\n");
			}
			queue_delayed_work(hx_attach_detach_wq, &mp_detach_w, msecs_to_jiffies(2000));
			return NOTIFY_DONE;
		}

	default:
		return NOTIFY_DONE;
    }
}

static struct notifier_block touch_mp_notifier = {
        .notifier_call = touch_mp_event,
        .priority = TOUCH_MP_NOTIFY,
};
#endif


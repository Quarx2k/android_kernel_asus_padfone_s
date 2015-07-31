 /* drivers/input/touchscreen/nt11003_touch.c
 *
 * Copyright (C) 2010 - 2011 Novatek, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/of_gpio.h> //Add for dts
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/input/mt.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/wakelock.h>

#if defined(CONFIG_EEPROM_NUVOTON)
#include <linux/microp_notify.h>
#include <linux/microp_notifier_controller.h>	//ASUS_BSP Lenter+
#endif

#include <linux/proc_fs.h>
#include <linux/nt11003.h>
#include <linux/switch.h>
#include "nt11306_firmware.h"

//#define HOME_ROW_KEYLIGHT 1

//ASUS_BSP HANS: add for led icon +++
#include <linux/workqueue.h>
#ifdef HOME_ROW_KEYLIGHT
#include <linux/leds.h>
#include <linux/pwm.h>
#endif
//ASUS_BSP HANS: add for led icon ---

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#ifdef ASUS_FACTORY_BUILD
#define Novatek_RC_FLAG  "/data/asusdata/A80_TP_RC"
#endif

#define MAX(a,b) ((a) < (b) ? (b) : (a))

#define DEVICE_NAME		"NVTflash"

#ifdef ASUS_FACTORY_BUILD
#define NV_RW_ATTR (S_IRUGO|S_IWUGO)
#define NV_WO_ATTR (S_IWUGO)
#define NV_RO_ATTR (S_IRUGO)
#else
#define NV_RW_ATTR (S_IRUGO|S_IWUSR|S_IWGRP)
#define NV_WO_ATTR (S_IWUSR|S_IWGRP)
#define NV_RO_ATTR (S_IRUGO)
#endif

//ASUS_BSP HANS: add for led icon +++
#ifdef HOME_ROW_KEYLIGHT
#define LED_MAX_LEVEL 100
#define LEVEL_TO_TABLE(x) (2*((x)/5))
#endif
//ASUS_BSP HANS: add for led icon ---

#define TOTAL_COUNT 4080

static uint8_t BUFFER_CHECK[30720] = {0};

#define FW_POINTER_RETURN 1

static bool irq_balance = 1;

static int update_result = -1;
static int update_progress = 0;

static bool rst_request = 0;	//avoid double request
static bool ts_suspend = 0;
static bool ts_inpad = 0;
static bool ts_update = 0;
static bool pre_suspend = 0;

static int rc_cal_fail = 1;		//ASUS_BSP HANS: some fw update need RC cal default set 1 ( fail ) 
static int firmware_version = 0xFF;
static int tpid = 0xFF;
static char last_info[8];

static int usb_state = 0; //ASUS_BSP Deeo: Get USB state

//ASUS_BSP : Add report location +++
static unsigned char touch_cunt_old;
static int REPORT_COUNT = 100;
static int finger_count[MAX_FINGER_NUM];
//ASUS_BSP : Add report location ---

//HanS support BMMI test++
static bool touch_init_status = 0;
static struct kobject *android_touch_kobj = NULL;
//HanS support BMMI test--

struct switch_dev nv_sdev;

//ASUS_BSP HANS: add for led icon +++
#ifdef HOME_ROW_KEYLIGHT
struct leddev {
	struct pwm_device *pwmb;
	int period;
	int duty;
	int value;
	int duration;
	int mode;
	int unlock;
	bool state;
};

static struct leddev home_led;

static int duty_table[] = {0,50,1,50,2,50,12,50,15,50,17,50,
			19,50,22,50,25,50,27,50,30,50,
			32,50,35,50,37,50,40,50,42,50,
			45,50,47,50,50,50,100,100,1000000,1000000};

struct workqueue_struct *wq_led_icon;
struct delayed_work icon_led_off_w, icon_led_on_w;
#endif
//ASUS_BSP HANS: add for led icon ---

static struct work_struct nv_mode_w;

static struct delayed_work g_resume_work_nv;
//ASUS_BSP Deeo : add for On/Off touch in P05 +++
static struct workqueue_struct *g_nv_wq_attach_detach;

#if defined(CONFIG_EEPROM_NUVOTON)
static struct work_struct g_mp_attach_work_nv;
static struct delayed_work g_mp_detach_work_nv;
#endif

//ASUS_BSP Deeo : add for On/Off touch in P05 ---
static struct wake_lock touch_wake_lock; //ASUS_BSP : Add for wake_lock when update firmware ++

static struct proc_dir_entry *NVT_proc_entry;

struct nvt_flash_data {
	rwlock_t lock;
	unsigned char bufferIndex;
	unsigned int length;
	struct i2c_client *client;
};

struct nvt_flash_data *flash_priv;


static int CTP_I2C_READ(struct i2c_client *client, uint8_t address, uint8_t *buf, uint8_t len)
{
	struct i2c_msg msgs[2];
	int ret = -1;
	int retries = 0;

	msgs[0].flags = !I2C_M_RD;
	msgs[0].addr  = address;
	msgs[0].len   = 1;
	msgs[0].buf   = &buf[0];

	msgs[1].flags = I2C_M_RD;
	msgs[1].addr  = address;
	msgs[1].len   = len-1;
	msgs[1].buf   = &buf[1];

	while(retries < 5)
	{
		ret = i2c_transfer(client->adapter, msgs, 2);
		if(ret == 2)	break;
		retries++;
	}
	return ret;
}

static int CTP_I2C_WRITE (struct i2c_client *client, uint8_t address, uint8_t *data, uint8_t len)
{
	struct i2c_msg msg;
	int ret = -1;
	int retries = 0;

	msg.flags = !I2C_M_RD;
	msg.addr  = address;
	msg.len   = len;
	msg.buf   = data;

	while(retries < 5)
	{
		ret = i2c_transfer(client->adapter, &msg, 1);
		if(ret == 1)	break;
		retries++;
	}
	return ret;
}


int readFile(struct file *fp,uint8_t *buf,int readlen)
{
	if (fp->f_op && fp->f_op->read)
		return fp->f_op->read(fp,buf,readlen, &fp->f_pos);
	else
		return -1;
}


//ASUS_BSP Deeo : Add for HWrst +++
static void nova_hwrst(void)
{
	int ret=1;
	int rec=1;

	if(!rst_request)
	{
		ret= gpio_request(nvts->rst_gpio, "Novatek_HWrst");
		if (ret) {
			pr_err("%s: Failed to get reset gpio %d. Code: %d.\n",
				__func__, nvts->rst_gpio, ret);
		}
		rec = gpio_direction_output(nvts->rst_gpio, 1);
		if (rec) {
			pr_err("%s: Failed to setup reset gpio %d. Code: %d.\n",
				__func__, nvts->rst_gpio, rec);
			gpio_free(nvts->rst_gpio);
		}

		if( ret || rec){
			printk("[Touch_N] Send hardware reset fail!!\n");
			return;
		}
		rst_request = true;
	}

	gpio_set_value( nvts->rst_gpio , 0 );
	msleep(3);
	gpio_set_value( nvts->rst_gpio , 1 );
	printk("[Touch_N] Send hardware reset succes!!\n");
}
//ASUS_BSP Deeo : Add for HWrst ---

//ASUS_BSP Deeo : Add for write RC flag to  asusdata +++
#ifdef ASUS_FACTORY_BUILD
static void nv_write_rc_calibration_work(void)
{
	struct file *fp = NULL; 
	//struct write_rcvalue *this = NULL;
	loff_t pos_lsts = 0;
	char writestr[16];
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	fp = filp_open( Novatek_RC_FLAG, O_RDWR|O_CREAT|O_TRUNC, 0666 );
	if(IS_ERR_OR_NULL(fp)) {
		printk("[Touch_N] nv_write_rc_calibration_work open (%s) fail\n", Novatek_RC_FLAG);
		return;
	}

	sprintf(writestr, "V%d %d", firmware_version , rc_cal_fail);

	printk("[Touch_N] Ver %d RC flag = %d[%s(%d)]\n", 
		firmware_version ,rc_cal_fail, writestr, strlen(writestr));

	if(fp->f_op != NULL && fp->f_op->write != NULL){
		pos_lsts = 0;

		fp->f_op->write(fp, writestr, strlen(writestr), &pos_lsts);
	}
	else
		printk("[Touch_N] nv_write_rc_calibration_work fail\n");

	set_fs(old_fs);
	filp_close(fp, NULL);

	return;
}
#endif
//ASUS_BSP Deeo : Add for write RC flag to asusdata ---

//ASUS_BSP HANS: temporary way to notify usb state +++
void nv_touch_mode(int state)
{
	if(nvts == NULL || nvts->bad_data != 0)
		return;

	if(usb_state == 95 && state == 0){
		return;
	}

	usb_state = state;

	queue_work(nt11003_wq, &nv_mode_w);
}
EXPORT_SYMBOL(nv_touch_mode);
//ASUS_BSP HANS: temporary way to notify usb state ---

int nvt_flash_write(struct file *file, const char __user *buff, size_t count, loff_t *offp)
{
 	char *str;
 	int ret=-1;
 	file->private_data = (uint8_t *)kmalloc(64, GFP_KERNEL);
 	str = file->private_data;
 	ret=copy_from_user(str, buff, count);

	ret = CTP_I2C_WRITE(flash_priv->client, str[0], str+2, str[1]);

 	return ret;
}

int nvt_flash_read(struct file *file, char __user *buff, size_t count, loff_t *offp)
{
 	char *str;
 	int ret = -1;
 	file->private_data = (uint8_t *)kmalloc(64, GFP_KERNEL);
 	str = file->private_data;
 	
	if(copy_from_user(str, buff, count))
		return -EFAULT;

 	CTP_I2C_READ(flash_priv->client,str[0],str+2,str[1]);

	ret=copy_to_user(buff, str, count);

	return ret;
}

int nvt_flash_open(struct inode *inode, struct file *file)
{
	struct nvt_flash_data *dev;

	dev = kmalloc(sizeof(struct nvt_flash_data), GFP_KERNEL);
	if (dev == NULL) {
		return -ENOMEM;
	}

	rwlock_init(&dev->lock);
	file->private_data = dev;

	return 0;
}

int nvt_flash_close(struct inode *inode, struct file *file)
{
	struct nvt_flash_data *dev = file->private_data;

	if (dev) {
		kfree(dev);
	}
	return 0;
}

struct file_operations nvt_flash_fops = {
	.owner = THIS_MODULE,
	.open = nvt_flash_open,
	.release = nvt_flash_close,
	.write = nvt_flash_write,
	.read = nvt_flash_read,
};

static int nvt_flash_init(struct nt11003_ts_data *ts)
{
	int ret=0;
	NVT_proc_entry = proc_create(DEVICE_NAME, 0666, NULL, &nvt_flash_fops);
	if(NVT_proc_entry == NULL)
	{
		printk("[Touch_N] Couldn't create proc entry!\n");
		ret = -ENOMEM;
		return ret ;
	}
	else
	{
		printk("[Touch_N] Create proc entry success!\n");
	}
	flash_priv=kzalloc(sizeof(*flash_priv),GFP_KERNEL);
	if (ts == NULL) {
                ret = -ENOMEM;
                goto error;
	}
	flash_priv->client = ts->client;
	printk("============================================================\n");
	printk("[Touch_N] NVT_flash driver loaded\n");
	printk("============================================================\n");
	return 0;
error:
	if(ret != 0)
	{
	printk("[Touch_N] flash_priv error!\n");
	}
	return -1;
}


//ASUS_BSP HANS: add for led icon +++
#ifdef HOME_ROW_KEYLIGHT
void led_init(struct leddev *leds)
{
	leds->pwmb = pwm_request(0,"keypad-touched");
	pwm_enable(leds->pwmb);

	leds->period = 50; //can't under 70000 in code base 2135
	leds->duty = 47;
	leds->value = 10;
	leds->duration = 2000; //in microseconds
	leds->mode = 0;
	leds->unlock = 0;

	leds->state = false;
}

void led_icon_config(struct leddev *leds, int value)
{
	leds->period = duty_table[LEVEL_TO_TABLE(value) + 1];
	leds->duty = duty_table[LEVEL_TO_TABLE(value)];
}

void led_icon_trigger(struct leddev *leds, int value)
{
	int dutytime = leds->duty;

	if(leds->mode == 3 || value == 0 || ts_suspend)
 		dutytime = 0;
	else
		dutytime = leds->duty;

	pwm_config(leds->pwmb, dutytime, leds->period);
 	pwm_enable(leds->pwmb);

	if(dutytime)
		leds->state = true;
 	else
		leds->state = false;
}

void led_icon_turnoff(struct work_struct *work)
{
	if(home_led.mode == 2)
		return;

	led_icon_trigger(&home_led, 0);

 	home_led.state = false;
}

void led_icon_turnon(struct work_struct *work)
{
	led_icon_trigger(&home_led, home_led.value);
}

static ssize_t led_icon_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "led_value=%d\n", home_led.value);
}

static ssize_t led_icon_store(struct device *dev, struct device_attribute *attr, 
				const char *buf, size_t count)
{
	if(sscanf(buf, "%d", &home_led.value) != 1)
		return -EINVAL;

	if(home_led.value > LED_MAX_LEVEL)
		home_led.value = LED_MAX_LEVEL;

	led_icon_config(&home_led, home_led.value);

	if(home_led.state)
		led_icon_trigger(&home_led, home_led.value);

	return count;
}

static ssize_t led_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
 	return snprintf(buf, PAGE_SIZE, "user_mode=%d\n", home_led.mode);
}

static ssize_t led_mode_store(struct device *dev, struct device_attribute *attr, 
			const char *buf, size_t count)
{
	if(sscanf(buf, "%d", &home_led.mode) != 1)
		return -EINVAL;

	switch(home_led.mode){
	case 0:{
		home_led.duration = 2000;
		break;
	}
	case 1:{
		home_led.duration = 10000;
		break;
	}
	case 2:
	case 3:{
		return count;
	}
	}

	if(delayed_work_pending(&icon_led_off_w)) {
		cancel_delayed_work_sync(&icon_led_off_w);
	}
	queue_delayed_work(wq_led_icon, &icon_led_off_w, msecs_to_jiffies(home_led.duration));

	return count;
}

static ssize_t screen_unlocked_store(struct device *dev, struct device_attribute *attr, 
				const char *buf, size_t count)
{
	if(sscanf(buf, "%d", &home_led.unlock) != 1)
		return -EINVAL;

	if((home_led.unlock == 1) && delayed_work_pending(&icon_led_off_w)) {
		cancel_delayed_work_sync(&icon_led_off_w);
		queue_delayed_work(wq_led_icon, &icon_led_off_w, msecs_to_jiffies(10000));
		home_led.unlock = 0;
	}

	return count;
}

static ssize_t period_duty_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf((char *)buf, PAGE_SIZE, "period= %d, DUTYTIME= %d\n", home_led.period, home_led.duty);
}

static ssize_t period_duty_store(struct device *dev, struct device_attribute *attr, 
			const char *buf,size_t count)
{
	if(sscanf(buf, "%d %d", &home_led.period, &home_led.duty) != 2)
	return -EINVAL;

	if(home_led.duty < 0)
 		home_led.period = 50;
	else if(home_led.duty > home_led.period)
		home_led.duty = home_led.period;

	led_icon_trigger(&home_led, home_led.duty);

	if(delayed_work_pending(&icon_led_off_w)) {
		cancel_delayed_work_sync(&icon_led_off_w);
	}
	queue_delayed_work(wq_led_icon, &icon_led_off_w, msecs_to_jiffies(home_led.duration));
	return count;
}
#endif
//ASUS_BSP HANS: add for led icon ---


static int nt11003_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct nt11003_ts_data *ts = i2c_get_clientdata(client);
	unsigned char i2c_control_buf[3] = {0xFF, 0x6F, 0xFF};	//suspend cmd, (I2C buffer = 255, data = 0)
	unsigned char i2c_control_buf2[2] = {0x00, 0xAF};	//suspend cmd, (I2C buffer = 255, data = 0)
#if FW_POINTER_RETURN
	uint8_t I2C_Buf[16] = {0};
#endif
 	if(ts_suspend)
 	{
 		printk("[Touch_N] Skip %s \n", __func__);
 		return 0;
 	}

	if (!ts_update)
	{
		printk("[Touch_N] nt11003_ts_suspend \n");

		CTP_I2C_WRITE(ts->client, ts->client->addr, i2c_control_buf, 3);
		CTP_I2C_WRITE(ts->client, ts->client->addr, i2c_control_buf2, 2);
		pre_suspend = 0;
 		ts_suspend = 1;
		if(usb_state == 95)
			usb_state = 0;

#if FW_POINTER_RETURN
		I2C_Buf[0] = 0xFF;
		I2C_Buf[1] = 0x6B;
		I2C_Buf[2] = 0x00;
		CTP_I2C_WRITE(ts->client, 0x01, I2C_Buf, 3);
#endif
	}
	else{
		pre_suspend = 1;
		printk("[Touch_N] Updating FW, Pause early_suspend!!\n");
	}

	return 0;
}

static int nt11003_ts_resume(struct i2c_client *client)
{
	unsigned char i2c_swrst_buf[2] = {0x00, 0x5A};	//suspend cmd, (I2C buffer = 255, data = 0)

 	if(ts_inpad || !ts_suspend )
 	{
 		printk("[Touch_N] Skip %s \n", __func__);
 		return 0;
 	}

	CTP_I2C_WRITE(client, 0x7F, i2c_swrst_buf, 2);
 	ts_suspend = false;


	printk("[Touch_N] nt11003_ts_resume \n");

	msleep(150);//ASUS_BSP Deeo: wait soft calibration finish

	if(!irq_balance){
		enable_irq(client->irq);
		irq_balance = 1;
		printk("[Touch_N] Enable IRQ : %d \n",irq_balance);
	}

	nv_touch_mode(usb_state);// ASUS_BSP Deeo: Add for resume to get USB state

	return 0;
}

static void ts_mode_work(struct work_struct *work)
{	
	uint8_t I2C_Buf[16] = {0};

	struct nt11003_ts_data *ts = nvts;

	if(ts == NULL)
		return;

	if(ts_update){
		printk("[Touch_N] Updating TP FW, Block USB notify!!\n");
		return;
	}

	I2C_Buf[0] = 0xFF;
	I2C_Buf[1] = 0x6F;
	I2C_Buf[2] = 0xFF;
	CTP_I2C_WRITE(ts->client, 0x01, I2C_Buf, 3);

	I2C_Buf[0] = 0x00;
    
	if(usb_state == 0){
	        printk("[Touch_N] Usb Power OFF.\n");
		I2C_Buf[1] = 0xF0;
	}
	else if(usb_state == 2){
	        printk("[Touch_N] Usb Power ON DC.\n");
	        I2C_Buf[1] = 0xF1;
	}
	else if(usb_state == 4){
	        printk("[Touch_N] Usb Power ON AC.\n");
	        I2C_Buf[1] = 0xF2;
	}
	else if(usb_state == 95){
	        printk("[Touch_N] Draw out from PAD.\n");
	        I2C_Buf[1] = 0xF3;
	}
	else{
		printk("[Touch_N] Use USB default value %d\n",usb_state);
		usb_state = 0;
		I2C_Buf[1] = 0xF0;
	}
	CTP_I2C_WRITE(ts->client, 0x01, I2C_Buf, 2);

	I2C_Buf[0] = 0xFF;
	I2C_Buf[1] = 0x6B;
	I2C_Buf[2] = 0x00;
	CTP_I2C_WRITE(ts->client, 0x01, I2C_Buf, 3);
}


static void ts_resume_work(struct work_struct *work)
{
	nt11003_ts_resume(nvts->client);
}

//ASUS_BSP Deeo : add for replace early suspend +++
#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct nt11003_ts_data *ts_dev_data = container_of(self, struct nt11003_ts_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK && ts_dev_data && ts_dev_data->client) 
	{
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK){
// 			nt11003_ts_resume(ts_dev_data->client);

			if(delayed_work_pending(&g_resume_work_nv)) {
				cancel_delayed_work_sync(&g_resume_work_nv);
				printk("[Touch_N] cancel last resume_work\n");
			}
			queue_delayed_work(g_nv_wq_attach_detach, &g_resume_work_nv, msecs_to_jiffies(2));

			//ASUS_BSP HANS: add for led icon +++
#ifdef HOME_ROW_KEYLIGHT
			if(delayed_work_pending(&icon_led_on_w)){
				cancel_delayed_work_sync(&icon_led_on_w);
			}
			queue_delayed_work(wq_led_icon, &icon_led_on_w, msecs_to_jiffies(900));

			if(delayed_work_pending(&icon_led_off_w)){
				cancel_delayed_work_sync(&icon_led_off_w);
			}

			queue_delayed_work(wq_led_icon, &icon_led_off_w, msecs_to_jiffies(home_led.duration));
#endif
			//ASUS_BSP HANS: add for led icon ---

		}
		else if (*blank == FB_BLANK_POWERDOWN){
			//ASUS_BSP HANS: add for led icon +++
#ifdef HOME_ROW_KEYLIGHT
			if(delayed_work_pending(&icon_led_off_w)){
				cancel_delayed_work_sync(&icon_led_off_w);
			}

			led_icon_trigger(&home_led , 0);
#endif
			//ASUS_BSP HANS: add for led icon ---
						
 			nt11003_ts_suspend(ts_dev_data->client,PMSG_SUSPEND);
		}
	}

	return 0;
}
#endif
//ASUS_BSP Deeo : add for replace early suspend ---

//ASUS_BSP Deeo : Add touch deriver attribute +++
static void get_fw_ver(void)
{
	uint8_t I2C_Buf[16] = {0};

	I2C_Buf[0] = 0xFF;
	I2C_Buf[1] = 0x6B;
	I2C_Buf[2] = 0x00;
	CTP_I2C_WRITE(nvts->client, 0x01, I2C_Buf, 3);

	I2C_Buf[0] = 0x78;
	CTP_I2C_READ(nvts->client, 0x01, I2C_Buf, 3);

	if ((I2C_Buf[1] & I2C_Buf[2]) == 0){
		printk("[Touch_N] Driver FW_VERSION: %d\n",I2C_Buf[1]);
		firmware_version = I2C_Buf[1];
	}
	else{
		printk("[Touch_N] I2C_Buf[1]: %d, I2C_Buf[2]: %d, ERROR!!!\n",I2C_Buf[1] , I2C_Buf[2]);
		firmware_version = 0xFF;
	} 

#if FW_POINTER_RETURN
	I2C_Buf[0] = 0xFF;
	I2C_Buf[1] = 0x6B;
	I2C_Buf[2] = 0x00;
	CTP_I2C_WRITE(nvts->client, 0x01, I2C_Buf, 3);
#endif
}

static ssize_t fw_ver(struct device *dev, struct device_attribute *devattr,char *buf)
{
	get_fw_ver();

	return snprintf(buf, PAGE_SIZE,"%d\n", firmware_version);
}

static ssize_t package_ver(struct device *dev, struct device_attribute *devattr,char *buf)
{
	return snprintf(buf, PAGE_SIZE,"%d\n", BUFFER_DATA[26368]);
}

static void get_tpid(void)
{	
	int cmd_ack=0xFF;
	int check=0xFF;
	uint8_t I2C_Buf[16] = {0};

	I2C_Buf[0] = 0xE8;
	I2C_Buf[1] = 0xC4;
	I2C_Buf[2] = 0x00;
	CTP_I2C_WRITE(nvts->client, 0x01, I2C_Buf, 3);

	msleep(2);

	I2C_Buf[0] = 0xE9;
	CTP_I2C_READ(nvts->client, 0x01, I2C_Buf, 4);

	cmd_ack = I2C_Buf[1];
	tpid = I2C_Buf[2];
	check = tpid + cmd_ack;

	if(I2C_Buf[1] == 0)
		tpid = 0xFF;
	
	printk("[Touch_N]status:0x%x Data:0x%x Result:0x%x Check:0x%x\n",I2C_Buf[1],I2C_Buf[2],I2C_Buf[3],check);

#if FW_POINTER_RETURN
	I2C_Buf[0] = 0xFF;
	I2C_Buf[1] = 0x6B;
	I2C_Buf[2] = 0x00;
	CTP_I2C_WRITE(nvts->client, 0x01, I2C_Buf, 3);
#endif
}

static ssize_t TPID(struct device *dev, struct device_attribute *devattr,char *buf)
{
	get_tpid();
	return snprintf(buf, PAGE_SIZE,"0x%x\n",tpid >>= 4);
}


static ssize_t about_phone(struct device *dev, struct device_attribute *devattr,char *buf)
{
	get_tpid();
	get_fw_ver();
	return snprintf(buf, PAGE_SIZE,"T%x -V%d\n",tpid >>= 4, firmware_version);
}

//ASUS_BSP HANS: some fw update need RC cal +++
static ssize_t rc_calibration(struct device *dev, struct device_attribute *devattr,
				const char *buf, size_t count)
{
	uint8_t I2C_Buf[16] = {0};

	struct i2c_client *client = to_i2c_client(dev);
	struct nt11003_ts_data *ts = i2c_get_clientdata(client);

	int cfg = 0;

	sscanf(buf, "%d\n",&cfg);
	printk("[Touch_N] cfg=%d\n",cfg);

	if ( cfg!=1 )
		return count;

	wake_lock(&touch_wake_lock);
	rc_cal_fail = 2;
	ts_update = 1;	// Avoid suspend

	I2C_Buf[0] = 0x00;
	I2C_Buf[1] = 0x5A;
	CTP_I2C_WRITE(ts->client, 0x7F, I2C_Buf, 2);

	msleep(500);

	I2C_Buf[0] = 0xFF;
	I2C_Buf[1] = 0x6C;
	I2C_Buf[2] = 0x00;
	CTP_I2C_WRITE(ts->client, 0x01, I2C_Buf, 3);

	I2C_Buf[0] = 0x00;
	I2C_Buf[1] = 0x06;
	I2C_Buf[2] = 0x09;
	I2C_Buf[3] = 0x0A;
	I2C_Buf[4] = 0x05;
	I2C_Buf[5] = 0xE6;
	CTP_I2C_WRITE(ts->client, 0x01, I2C_Buf, 6);

	msleep(10);

	I2C_Buf[0] = 0xFF;
	I2C_Buf[1] = 0x6F;
	I2C_Buf[2] = 0xFF;
	CTP_I2C_WRITE(ts->client, 0x01, I2C_Buf, 3);

	I2C_Buf[0] = 0x00;
	I2C_Buf[1] = 0xBE;
	CTP_I2C_WRITE(ts->client, 0x01, I2C_Buf, 2);


       msleep(2000);

	I2C_Buf[0] = 0xFF;
	I2C_Buf[1] = 0x6F;
	I2C_Buf[2] = 0xFE;
	CTP_I2C_WRITE(ts->client, 0x01, I2C_Buf, 3);

	I2C_Buf[0] = 0x00;
	I2C_Buf[1] = 0x00;
	CTP_I2C_READ(ts->client, 0x01, I2C_Buf, 2);

        if (I2C_Buf[1] == 0xAA){
		dev_info(&client->dev, "[Touch_N] Program : rc_cal get status(0x%2X) success\n", I2C_Buf[1]);

		I2C_Buf[0] = 0x00;
		I2C_Buf[1] = 0x5A;
		CTP_I2C_WRITE(ts->client, 0x7F, I2C_Buf, 2);

            rc_cal_fail = 0;
#ifdef ASUS_FACTORY_BUILD
		nv_write_rc_calibration_work();
#endif

		ts_update = false;

		msleep(200); 			//ASUS_BSP Deeo : wait soft calibration finish
		nv_touch_mode(usb_state);	//ASUS_BSP Deeo: Add for Get Usb state after rc_calibartion to avoid usb plugin while update FW

		wake_unlock(&touch_wake_lock);
		
		if(pre_suspend)
			nt11003_ts_suspend(ts->client, PMSG_SUSPEND);
		
		return count;
        }
        dev_info(&client->dev, "[Touch_N] Program : rc_cal get status(0x%2X), fail\n", I2C_Buf[1]);


	//ASUS_BSP Deeo : do soft_reset either RC K fail
	I2C_Buf[0] = 0x00;
	I2C_Buf[1] = 0x5A;
	CTP_I2C_WRITE(ts->client, 0x7F, I2C_Buf, 2);

	rc_cal_fail = 1;

#ifdef ASUS_FACTORY_BUILD
	nv_write_rc_calibration_work();
#endif

	printk("[Touch_N] RC Calibration Fail!!!\n");

	ts_update = 0;
	wake_unlock(&touch_wake_lock);

#if FW_POINTER_RETURN
	I2C_Buf[0] = 0xFF;
	I2C_Buf[1] = 0x6B;
	I2C_Buf[2] = 0x00;
	CTP_I2C_WRITE(ts->client, 0x01, I2C_Buf, 3);
#endif

	if(pre_suspend)
		nt11003_ts_suspend(ts->client, PMSG_SUSPEND);

	return count;
}

static ssize_t rc_calibration_result(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", rc_cal_fail);
}
//ASUS_BSP HANS: some fw update need RC cal ---

//ASUS_BSP Deeo : combine attribute +++
static ssize_t debug_level(struct device *dev, struct device_attribute *devattr,const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct nt11003_ts_data *ts = i2c_get_clientdata(client);
	uint8_t I2C_Buf[16] = {0};

	int cfg = 0;

	printk("[Touch_N] cmd:%c \n",buf[0]);

	if( buf[0] == 's' )
	{
		I2C_Buf[0] = 0x00;
		I2C_Buf[1] = 0x5A;
		CTP_I2C_WRITE(ts->client, 0x7F, I2C_Buf, 2);

		printk("[Touch_N] NT11003 soft-reset!!!\n");
	}
	else if(buf[0] == 'h')
	{
		nova_hwrst();
	}
	else if(buf[0] == 'r')
	{
		sscanf(buf+1, "%d\n", &cfg);
		REPORT_COUNT = cfg;
		printk("[Touch_N] REPORT_COUNT : %d\n",REPORT_COUNT);
	}
	else if(buf[0] == 't')
	{
		sscanf(buf+2, "%d\n", &cfg);

		if(cfg == 0)
		{
			nt11003_ts_suspend(nvts->client,PMSG_SUSPEND);
			printk("[Touch_N] Disable touch\n");
		}
		else
		{
			nt11003_ts_resume(nvts->client);
			printk("[Touch_N] Enable touch\n");
    		}
	}
	else
		printk("[Touch_N] cmd error, no use!!\n");

#if FW_POINTER_RETURN
	I2C_Buf[0] = 0xFF;
	I2C_Buf[1] = 0x6B;
	I2C_Buf[2] = 0x00;
	CTP_I2C_WRITE(ts->client, 0x01, I2C_Buf, 3);
#endif
	return count;
}
//ASUS_BSP Deeo : combine attribute ---


static ssize_t Check_CheckSum(struct device *dev)
{
#if FW_POINTER_RETURN
	uint8_t I2C_Buf[16] = {0};
#endif
	uint8_t buf1[64];
	uint8_t buf2[64];
	int i, j, k, Retry_Counter=0;
	int addr;
	uint8_t addrH, addrL;
	unsigned short RD_Filechksum = 0, WR_Filechksum = 0;
	unsigned char tmp=0;

	struct i2c_client *client = to_i2c_client(dev);
	struct nt11003_ts_data *ts = i2c_get_clientdata(client);


	buf1[0]=0x00;
	buf1[1]=0x5A;
	CTP_I2C_WRITE(ts->client, 0x7F, buf1, 2);
	msleep(1000);
	
	buf1[0]=0xFF;
	buf1[1]=0x6F;
	buf1[2]=0xFF;
	CTP_I2C_WRITE(ts->client, 0x01, buf1, 3);

	buf1[0]=0x00;
	buf1[1]=0xE1;
	CTP_I2C_WRITE(ts->client, 0x01, buf1, 2);

	for(i=0;i<(sizeof(BUFFER_CHECK))/128;i++)
	{
		addr = i * 128;
		for(j=0;j<16;j++)
		{
			tmp = 0;
			addrH = addr>>8;
			addrL = addr&0xFF;
			for(k=0;k<8;k++)
			{
				tmp+=BUFFER_CHECK[i*128+j*8+k];
			}
			tmp = tmp+addrH+addrL+8;
			tmp = ~tmp+1;
			WR_Filechksum += tmp;
			addr += 8;
		}
	}
	msleep(800);

	do
	{
		msleep(10);
		buf1[0]=0xFF;
		buf1[1]=0x6E;
		buf1[2]=0x0D;
		CTP_I2C_WRITE(ts->client, 0x01, buf1, 3);

		buf2[0]=0x00;
		buf2[1]=0x00;
		buf2[2]=0x00;
		buf2[3]=0x00;
		CTP_I2C_READ(ts->client, 0x01, buf2, 4);
		Retry_Counter++;
		msleep(10);
	}while((Retry_Counter<50) && (buf2[1]!=0xAA));

	//---------------------------------------------------------------------------------------

	if(buf2[1]==0xAA)
	{
		RD_Filechksum=(buf2[2]<<8)+buf2[3];

		printk("RD_Filechksum: %d,  WR_Filechksum: %d \n",RD_Filechksum, WR_Filechksum);

		if(RD_Filechksum==WR_Filechksum)
		{
			return 0;
		}
		else
		{
			return 1;
		}
	}
	else
	{
		return -1;
	}

#if FW_POINTER_RETURN
	I2C_Buf[0] = 0xFF;
	I2C_Buf[1] = 0x6B;
	I2C_Buf[2] = 0x00;
	CTP_I2C_WRITE(ts->client, 0x01, I2C_Buf, 3);
#endif

}


//ASUS_BSP Deeo : load bin file to updae FW from user-space +++
static ssize_t load_fw(struct device *dev, struct device_attribute *devattr,const char *buf, size_t count)
{
	struct file *fp = NULL;
	uint8_t I2C_Buf[16] = {0};
	uint8_t i = 0;
	uint8_t j = 0;
	unsigned int Flash_Address = 0;
	unsigned int Row_Address = 0;
	uint8_t CheckSum[16]= {0};	// 128/8 = 16 times ;
	struct i2c_client *client = to_i2c_client(dev);
	struct nt11003_ts_data *ts = i2c_get_clientdata(client);
	uint8_t buffer[8];
	int force = 0;
	char cfg[80];

	mm_segment_t old_fs = get_fs();

	update_progress = 0;
	update_result = 1;

	sscanf(buf, "%s", cfg);

	printk("[Touch_N] cfg (%s) \n",cfg);

	if( buf[0] == 'f'){
		force = 1;
	}
	else{
		set_fs(KERNEL_DS);

		fp = filp_open( cfg , O_RDONLY, 0 );
		if(IS_ERR_OR_NULL(fp)) {
			printk("[Touch_N] load_fw open (%s) fail\n",cfg);
			return count;
		}
	}

	printk("[Touch_N] Start Update Firmware!!!\n");
	printk("[Touch_N] Init touchFW_wake_lock\n");
	ts_update = 1;
	wake_lock(&touch_wake_lock);

	gpio_set_value( nvts->rst_gpio , 0);
	msleep(6);

	//-------------------------------
	// Step1 --> initial BootLoader
	// Note. 0x7F -> 0x00 -> 0x00 ;
	// 須配合 Reset Pin
	//-------------------------------
	I2C_Buf[0] = 0x00;
	I2C_Buf[1] = 0xA5;
	CTP_I2C_WRITE(ts->client, 0x7F, I2C_Buf, 2);	// Write a “A5H” to NT1100x

	msleep(6);

	gpio_set_value( nvts->rst_gpio , 1);

	printk("[Touch_N] inital Bootloader!!\n");

	msleep(20);

	//Step 1 : Initiate Flash Block
	I2C_Buf[0] = 0x00;
	I2C_Buf[1] = 0x00;
	CTP_I2C_WRITE(ts->client, 0x7F, I2C_Buf, 2);	// Write a 0x00 to NT1100x

	msleep(20);	// Delay

	printk("[Touch_N] inital Flash Block!!\n");

	// Read NT1100x status
	I2C_Buf[0] = 0x00;
	CTP_I2C_READ(ts->client, 0x7F, I2C_Buf, 2);
	// if return “AAH” then going next step
	if (I2C_Buf[1] != 0xAA)
	{
		dev_info(&client->dev, "[Touch_N] Program : init get status(0x%2X) error\n", I2C_Buf[1]);
		goto error;
	}
	dev_info(&client->dev, "[Touch_N] Program : init get status(0x%2X) success\n", I2C_Buf[1]);

	//---------------------------------------------------------
	// Step 2 : Erase 26K bytes via Row Erase Command ( 30H )
	//---------------------------------------------------------
	for (i = 0 ; i < 240 ; i++)	// 26K equals 208 Rows
	{
		Row_Address = i * 128;

		I2C_Buf [0] = 0x00;
		I2C_Buf [1] = 0x30;	// Row Erase command : 30H
		I2C_Buf [2] = (uint8_t)((Row_Address & 0xFF00) >> 8 );	// Address High Byte
		I2C_Buf [3] = (uint8_t)(Row_Address & 0x00FF);	// Address Low Byte

		CTP_I2C_WRITE(ts->client, 0x7F, I2C_Buf, 4);	// Write 30H, Addr_H & Addr_L to NT11003

		msleep(10);	// Delay 15 ms

	// Read NT11003 status
		CTP_I2C_READ(ts->client, 0x7F, I2C_Buf, 2);

	// if NT1003 return AAH then going next step
		if (I2C_Buf[1] != 0xAA)
		{
			dev_info(&client->dev, "[Touch_N] Program %d: erase(0x%2X) error\n",i,I2C_Buf[1]);
			goto error;
		}
		update_progress++;
	}
	dev_info(&client->dev, "[Touch_N] Program %d: erase(0x%2X) success\n",i, I2C_Buf[1]);

	Flash_Address = 0;

	////////////////////////////////////////////////////////////////////////////////////
	//----------------------------------------
	// Step3. Host write 128 bytes to NT11003
	// Step4. Host read checksum to verify
	//----------------------------------------
	dev_info(&client->dev, "[Touch_N] Program : write begin, please wait ...\n");
	for (j = 0 ; j < 240; j++)	// Write/ Read 208 times
	{
		Flash_Address = j * 128 ;

		for (i = 0 ; i < 16 ; i++)	// 128/8 = 16 times for One Row program
		{
			if( force ){
				// Step 3 : write binary data to NT11003
				I2C_Buf[0] = 0x00;
				I2C_Buf[1] = 0x55;	//Flash write command
				I2C_Buf[2] = (uint8_t)(Flash_Address  >> 8);	//Flash address [15:8]
				I2C_Buf[3] = (uint8_t)(Flash_Address & 0xFF);	//Flash address [7:0]
				I2C_Buf[4] = 0x08;	//How many prepare to write to NT1100
				I2C_Buf[6] = BUFFER_DATA[Flash_Address + 0];	//Binary data 1
				I2C_Buf[7] = BUFFER_DATA[Flash_Address + 1];	//Binary data 2
				I2C_Buf[8] = BUFFER_DATA[Flash_Address + 2];	//Binary data 3
				I2C_Buf[9] = BUFFER_DATA[Flash_Address + 3];	//Binary data 4
				I2C_Buf[10] = BUFFER_DATA[Flash_Address + 4];   //Binary data 5
				I2C_Buf[11] = BUFFER_DATA[Flash_Address + 5];	//Binary data 6
				I2C_Buf[12] = BUFFER_DATA[Flash_Address + 6];	//Binary data 7
				I2C_Buf[13] = BUFFER_DATA[Flash_Address + 7];	//Binary data 8	
			}
			else{
				memset(buffer,0,8);
				readFile(fp,buffer,8);

				// Step 3 : write binary data to NT11003
				I2C_Buf[0] = 0x00;
				I2C_Buf[1] = 0x55;	//Flash write command
				I2C_Buf[2] = (uint8_t)(Flash_Address  >> 8);	//Flash address [15:8]
				I2C_Buf[3] = (uint8_t)(Flash_Address & 0xFF);	//Flash address [7:0]
				I2C_Buf[4] = 0x08;	//How many prepare to write to NT1100
				I2C_Buf[6] = buffer[0];		//Binary data 1
				I2C_Buf[7] = buffer[1];		//Binary data 2
				I2C_Buf[8] = buffer[2];		//Binary data 3
				I2C_Buf[9] = buffer[3];		//Binary data 4
				I2C_Buf[10] = buffer[4];	//Binary data 5
				I2C_Buf[11] = buffer[5];	//Binary data 6
				I2C_Buf[12] = buffer[6];	//Binary data 7
				I2C_Buf[13] = buffer[7];	//Binary data 8

				BUFFER_CHECK[Flash_Address + 0] = buffer[0];		//Binary data 1
				BUFFER_CHECK[Flash_Address + 1] = buffer[1];		//Binary data 2
				BUFFER_CHECK[Flash_Address + 2] = buffer[2];		//Binary data 3
				BUFFER_CHECK[Flash_Address + 3] = buffer[3];		//Binary data 4
				BUFFER_CHECK[Flash_Address + 4] = buffer[4];	//Binary data 5
				BUFFER_CHECK[Flash_Address + 5] = buffer[5];	//Binary data 6
				BUFFER_CHECK[Flash_Address + 6] = buffer[6];	//Binary data 7
				BUFFER_CHECK[Flash_Address + 7] = buffer[7];	//Binary data 8

			}

			CheckSum[i] = ~(I2C_Buf[2] + I2C_Buf[3] + I2C_Buf[4] + I2C_Buf[6] + I2C_Buf[7] +
		          I2C_Buf[8] + I2C_Buf[9] + I2C_Buf[10] + I2C_Buf[11] + I2C_Buf[12] +
			      I2C_Buf[13]) + 1;

			I2C_Buf[5] = CheckSum[i];	// Load check sum to I2C Buffer
			CTP_I2C_WRITE(ts->client, 0x7F, I2C_Buf, 14);	//Host write I2C_Buf[0…12] to NT1100x.

			if(i == 15)
				msleep(7);

			// Read NT1100x status
			I2C_Buf[0] = 0x00;
			CTP_I2C_READ(ts->client, 0x7F, I2C_Buf, 2);

			// if return “AAH” then going next step
			if (I2C_Buf[1] != 0xAA)
			{
				dev_info(&client->dev, "[Touch_N] Program : write(j=%d, i=%d, 0x%2X) error\n", j, i, I2C_Buf[1]);
				goto error;
			}
			Flash_Address += 8 ;	// Increase Flash Address. 8 bytes for 1 time
			
			update_progress++;
		}

		msleep(10);	// Each Row program --> Need 15ms delay time
	}
	dev_info(&client->dev, "[Touch_N] Program : write finish ~~\n");
	/////////////////////////////////////////////////////////////////////

	update_result = Check_CheckSum(dev);
	
	if(update_result == 0)
	{
		dev_info(&client->dev, "[Touch_N] Program : Verify Pass\n");
	}
	else if(update_result == 1)
	{
		dev_info(&client->dev, "[Touch_N] Program : Verify NG\n");
	}
	else if(update_result == -1)
	{
		dev_info(&client->dev, "[Touch_N] Program : Can't Get CheckSum\n");
	}
	
	I2C_Buf[0] = 0x00;
	I2C_Buf[1] = 0x5A;
	CTP_I2C_WRITE(ts->client, 0x7F, I2C_Buf, 2);

	printk("[Touch_N] Soft reset!!! \n");

	printk("[Touch_N] Finish Update Firmware!!!\n");
	
	ts_update = 0;
	wake_unlock(&touch_wake_lock);

	if(pre_suspend)
		nt11003_ts_suspend(ts->client, PMSG_SUSPEND);

	if(!force)
	{
		set_fs(old_fs);
		filp_close(fp, NULL);
	}

	return count;

error:
	printk("[Touch_N] Update touch FW fail!!!\n");

	I2C_Buf[0] = 0x00;
	I2C_Buf[1] = 0x5A;
	CTP_I2C_WRITE(ts->client, 0x7F, I2C_Buf, 2);
	
	ts_update = 0;
	wake_unlock(&touch_wake_lock);


	if(pre_suspend)
		nt11003_ts_suspend(ts->client, PMSG_SUSPEND);

	if(!force)
	{
		set_fs(old_fs);
		filp_close(fp, NULL);
	}

	return count;
}
//ASUS_BSP Deeo : load bin file to updae FW from user-space ---

static ssize_t fw_update_progress(struct device *dev, struct device_attribute *devattr,char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", update_progress*100/TOTAL_COUNT);
}

static ssize_t fw_update_result(struct device *dev, struct device_attribute *devattr,char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", update_result);
}

#if defined(CONFIG_EEPROM_NUVOTON)
#if defined(CONFIG_ASUS_HDMI)
extern bool asus_padstation_exist_realtime(void);
#endif
static ssize_t get_pad(struct device *dev, struct device_attribute *devattr,char *buf)
{
	#if defined(CONFIG_ASUS_HDMI)
	return snprintf(buf, PAGE_SIZE, "%d\n", asus_padstation_exist_realtime());
	#else
	return snprintf(buf, PAGE_SIZE, "%d\n", 0);
	#endif
}
#endif

static ssize_t nv_fw_ver(struct switch_dev *sdev, char *buf)
{
	
	if(ts_inpad)
		return sizeof(buf);

 	get_tpid();
 	get_fw_ver();

	if(firmware_version == 0xFF || tpid == 0xFF){
		return snprintf(buf, PAGE_SIZE, last_info);
	}
	else{	
		snprintf(last_info,sizeof(last_info), "T%x -V%d",tpid >>= 4, firmware_version);
	}

	return snprintf(buf, PAGE_SIZE,"T%x -V%d",tpid >>= 4, firmware_version);	
}

#if defined(CONFIG_EEPROM_NUVOTON)
DEVICE_ATTR(PAD, NV_RO_ATTR,get_pad, NULL);
#endif
DEVICE_ATTR(package_ver, NV_RO_ATTR, package_ver, NULL);
DEVICE_ATTR(driver_ver, NV_RO_ATTR, fw_ver, NULL);
DEVICE_ATTR(load_fw, NV_RW_ATTR, NULL, load_fw);
DEVICE_ATTR(function, NV_RW_ATTR, NULL, debug_level);
DEVICE_ATTR(TPID, NV_RO_ATTR,TPID, NULL);
DEVICE_ATTR(about_phone, NV_RO_ATTR,about_phone, NULL);
DEVICE_ATTR(rc_calibration, NV_RW_ATTR, rc_calibration_result, rc_calibration); //ASUS_BSP HANS: fw v23 need RC cal++
//ASUS_BSP HANS: add for led +++
#ifdef HOME_ROW_KEYLIGHT
DEVICE_ATTR(key_led, NV_RW_ATTR, led_icon_show, led_icon_store);
DEVICE_ATTR(user_mode, NV_RW_ATTR, led_mode_show, led_mode_store);
DEVICE_ATTR(screen_unlocked, NV_WO_ATTR, NULL, screen_unlocked_store);
DEVICE_ATTR(period_duty, NV_RW_ATTR, period_duty_show, period_duty_store);
#endif
//ASUS_BSP HANS: add for led ---
DEVICE_ATTR(update_progress, NV_RO_ATTR, fw_update_progress, NULL);
DEVICE_ATTR(update_result, NV_RO_ATTR, fw_update_result, NULL);


static struct attribute *nt_attrs[] = {
#if defined(CONFIG_EEPROM_NUVOTON)
	&dev_attr_PAD.attr,
#endif
	&dev_attr_package_ver.attr,
	&dev_attr_driver_ver.attr,
	&dev_attr_load_fw.attr,
	&dev_attr_function.attr,
	&dev_attr_TPID.attr,
	&dev_attr_about_phone.attr,
	&dev_attr_rc_calibration.attr, //ASUS_BSP HANS: fw v23 need RC cal ++
//ASUS_BSP HANS: add for led +++
#ifdef HOME_ROW_KEYLIGHT
	&dev_attr_key_led.attr,
	&dev_attr_user_mode.attr,
	&dev_attr_screen_unlocked.attr,
	&dev_attr_period_duty.attr,
#endif
//ASUS_BSP HANS: add for led ---
	&dev_attr_update_progress.attr,
	&dev_attr_update_result.attr,
	NULL
};

static struct attribute_group nt11003_attr_group = {
        .attrs = nt_attrs,
};
//ASUS_BSP Deeo : Add touch deriver attribute ---

static void nt11003_ts_work_func(struct work_struct *work)
{
	uint8_t  point_data[1+MAX_FINGER_NUM*6]={0};
	unsigned int position = 0;
	uint8_t track_id[MAX_FINGER_NUM] = {0};
	unsigned int input_x = 0;
	unsigned int input_y = 0;
	unsigned int input_w = 0;
	unsigned char index = 0;
	unsigned char run_count = 0, lift_count = 0;
	unsigned char touch_cunt_now = 0;
	unsigned char Up_Buf[MAX_FINGER_NUM] = {1,1,1,1,1,1,1,1,1,1};
	unsigned char Up_Index = 0;
	unsigned char tpid=99;

	struct nt11003_ts_data *ts = container_of(work, struct nt11003_ts_data, work);

	if(ts_inpad)
		return;

	CTP_I2C_READ(ts->client, ts->client->addr, point_data,  sizeof(point_data)/sizeof(point_data[0]));

	for (index = 0; index < MAX_FINGER_NUM; index++) //0~9 (10 points)
	{
		position = 1 + 6*index;

		if ( ( ( (point_data[position]>>3)&0x1F) > 0) && ( ((point_data[position]>>3)&0x1F)  <= MAX_FINGER_NUM) )
		{
			track_id[index] = ( (point_data[1+index*6]>>3)&0x0F )-1;
			touch_cunt_now++;
		}
	}

	run_count = MAX(touch_cunt_old, touch_cunt_now);

	for (index = 0; index < run_count; index++) //0~9 (10 points)
	{
		position = 1 + 6*index;

		if ( (point_data[position]&0x03) == 0x03 )     // Touch UP    Up or No event
		{
			if( (Up_Buf[Up_Index]==1) && (Up_Index != tpid) )  //up for initial state
			{
				input_mt_slot(ts->input_dev,Up_Index);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
				lift_count++;
			}
			Up_Index ++;
			if ( Up_Buf[Up_Index] == 0 )
			{
				Up_Index ++;
			}
			if(Up_Buf[track_id[index]] == 1)
			{
			    input_mt_slot(ts->input_dev, track_id[index]);
				input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, false);
			}

			if(finger_count[index] != 0)
				printk("[Touch_N][%d]-[%d]Touch up!! \n",index,finger_count[index]);

			finger_count[index] = 0;

			//ASUS_BSP HANS: add for led icon +++
#ifdef HOME_ROW_KEYLIGHT
			if(delayed_work_pending(&icon_led_off_w)) {
				cancel_delayed_work_sync(&icon_led_off_w);
			}

			if(home_led.unlock == 1){
				queue_delayed_work(wq_led_icon, &icon_led_off_w, msecs_to_jiffies(10000));
				home_led.unlock = 0;
			}
			else
				queue_delayed_work(wq_led_icon, &icon_led_off_w, msecs_to_jiffies(home_led.duration));
#endif
			//ASUS_BSP HANS: add for led icon ---

		}
		else
		{
			input_x = (unsigned int)(point_data[position+1]<<4) + (unsigned int) (point_data[position+3]>>4);
			input_y = (unsigned int)(point_data[position+2]<<4) + (unsigned int) (point_data[position+3]&0x0f);
			input_w = (unsigned int) (point_data[position+4])+10;

			if (input_x < 0) continue;//	input_x = 0;
			if (input_y < 0) continue;//	input_y = 0;

			// ASUS_BSP : Add report location +++
			if (finger_count[index]%REPORT_COUNT == 0)
				printk("[Touch_N][%d]-[%d][x,y,w][%d,%d,%d]\n",index,finger_count[index],input_x,input_y,input_w);

			finger_count[index]++;
			// ASUS_BSP : Add report location ---

			//ASUS_BSP HANS: add for led icon +++
#ifdef HOME_ROW_KEYLIGHT
			if(delayed_work_pending(&icon_led_off_w)){
				cancel_delayed_work_sync(&icon_led_off_w);
			}

			led_icon_trigger(&home_led, home_led.value);
#endif
			//ASUS_BSP HANS: add for led icon ---

			if((input_x > 1090 )||(input_y > 1970))	continue;	//ASUS_BSP Deeo : Fix conditition +++
			
			input_mt_slot(ts->input_dev, track_id[index]);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, true);

			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, input_x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, input_y);
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, input_w);

			Up_Buf[track_id[index]] = 0;
			tpid = index;
		}
	}//end for loop

	input_sync(ts->input_dev);

	if ( (run_count - lift_count) == 0)
		touch_cunt_old = touch_cunt_now;
	else
		touch_cunt_old = run_count;

	goto END_WORK_FUNC;


END_WORK_FUNC:
	if(!irq_balance){
 		enable_irq(ts->client->irq);
		irq_balance = 1;
	}
}

static irqreturn_t nt11003_ts_irq_handler(int irq, void *dev_id)
{
	struct nt11003_ts_data *ts = dev_id;

	if(irq_balance){
		irq_balance = 0;
 		disable_irq_nosync(ts->client->irq);
	}

	queue_work(nt11003_wq, &ts->work);

	return IRQ_HANDLED;
}

#if defined(CONFIG_EEPROM_NUVOTON)
//ASUS_BSP Deeo : add for On/Off touch in P05 +++
static void attach_nv_padstation_work(struct work_struct *work)
{
	printk("[Touch_N] attach_padstation_work()++\n");

	if(ts_update)
		printk("[Touch_N] Skip suspend!!! Updating FW %d \n",ts_update);
	else{
		 if (irq_balance){
			irq_balance = 0;
 			disable_irq(nvts->client->irq);
			printk("[Touch_N] Disable IRQ : %d \n",irq_balance);
		}

		msleep(100);

		printk("[Touch_N] Pad attach suspend!!!\n");
		nt11003_ts_suspend(nvts->client, PMSG_SUSPEND);
	}

//ASUS_BSP HANS: add for led icon +++
#ifdef HOME_ROW_KEYLIGHT
	if(delayed_work_pending(&icon_led_off_w)){
		cancel_delayed_work_sync(&icon_led_off_w);
	}

	led_icon_trigger(&home_led, 0);
#endif
//ASUS_BSP HANS: add for led icon ---

	printk("[Touch_N] attach_padstation_work()--\n");
}

static void detach_nv_padstation_work(struct work_struct *work)
{
	printk("[Touch_N] detach_padstation_work()++\n");

	ts_inpad = false;
	usb_state = 95;

	printk("[Touch_N] Pad detach resume!!!\n");
	nt11003_ts_resume(nvts->client);

	printk("[Touch_N] detach_padstation_work()--\n");
}

static int touch_mp_nv_event(struct notifier_block *this, unsigned long event, void *ptr)
{
	printk("[Touch_N] Mircop event %d\n",(unsigned int)event);
	switch (event) {

		case P01_ADD:{
			ts_inpad = true;
			if(delayed_work_pending(&g_mp_detach_work_nv)) {
				cancel_delayed_work_sync(&g_mp_detach_work_nv);
				printk("[Touch_N] cancel last detach_work\n");
			}
			queue_work(g_nv_wq_attach_detach, &g_mp_attach_work_nv);
			return NOTIFY_DONE;
		}
		case P01_REMOVE:{
			if(delayed_work_pending(&g_mp_detach_work_nv)) {
				cancel_delayed_work_sync(&g_mp_detach_work_nv);
				printk("[Touch_N] cancel last detach_work\n");
			}
			queue_delayed_work(g_nv_wq_attach_detach, &g_mp_detach_work_nv, msecs_to_jiffies(2000));
			return NOTIFY_DONE;
		}

	default:
		return NOTIFY_DONE;
    }
}

static struct notifier_block touch_mp_notifier_nv = {
        .notifier_call = touch_mp_nv_event,
        .priority = TOUCH_MP_NOTIFY,
};
//ASUS_BSP Deeo : add for On/Off touch in P05 ---
#endif

//ASUS_BSP Deeo : Parse device tress info +++
static int novatek_get_dt_coords(struct device *dev, char *name,struct nt11003_i2c_platform_data *pdata)
{
	u32 coords[NOVATEK_COORDS_ARR_SIZE];
	struct property *prop;
	struct device_node *np = dev->of_node;
	int coords_size, rc;

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	coords_size = prop->length / sizeof(u32);
	if (coords_size != NOVATEK_COORDS_ARR_SIZE) {
		dev_err(dev, "invalid %s\n", name);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(np, name, coords, coords_size);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read %s\n", name);
		return rc;
	}

	if (strncmp(name, "novatek,panel-coords",
			sizeof("novatek,panel-coords")) == 0) {
		pdata->abs_x_min = coords[0];
		pdata->abs_y_min = coords[1];
		pdata->abs_x_max = coords[2];
		pdata->abs_y_max = coords[3];
	} 
	else {
		dev_err(dev, "unsupported property %s\n", name);
		return -EINVAL;
	}

	return 0;
}

static int novatek_parse_dt(struct device *dev, struct nt11003_i2c_platform_data *pdata)
{
	int rc;
	struct device_node *np = dev->of_node;

	rc = novatek_get_dt_coords(dev, "novatek,panel-coords", pdata);
	if (rc)
		return rc;

	/* reset, irq gpio info */
	pdata->rst_gpio = of_get_named_gpio_flags(np, "novatek,reset-gpio",0, &pdata->reset_gpio_flags);
	pdata->intr_gpio = of_get_named_gpio_flags(np, "novatek,irq-gpio",0, &pdata->irq_gpio_flags);

	return 0;
}
//ASUS_BSP Deeo : Parse device tress info ---

//HanS: support BMMI test++
static ssize_t nova_init_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", touch_init_status);
}
static DEVICE_ATTR(touch_status, NV_RO_ATTR, nova_init_show, NULL);

static int nova_touch_sysfs_init(void)
{
	android_touch_kobj = kobject_create_and_add("android_touch", NULL );
	if (android_touch_kobj == NULL )
	{
		printk("[Touch_N] subsystem_register failed\n");
		return -1;
	}

 	if(sysfs_create_file(android_touch_kobj, &dev_attr_touch_status.attr))
 	{
 		printk("[Touch_N] sysfs_create_file failed\n");
 		return -1;
 	}

	return 0;
}
//HanS support BMMI test--

static int nt11003_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	int retry=0;
	int error=0;
	struct nt11003_ts_data *ts=NULL;
	//char *version_info = NULL;
	uint8_t test_data[7] = {0};

	const char irq_table[4] = {IRQ_TYPE_EDGE_RISING,
				   IRQ_TYPE_EDGE_FALLING,
				   IRQ_TYPE_LEVEL_LOW,
				   IRQ_TYPE_LEVEL_HIGH};

	//unsigned char i2c_swrst_buf[2] = {0x00, 0x5A};//suspend cmd, (I2C buffer = 255, data = 0)

	struct nt11003_i2c_platform_data *pdata;

	//dev_info(&client->dev, "[Touch_N] nt11003_ts_probe +++\n");
	printk("[Touch_N] nt11306_ts_probe +++\n");

	//HanS: support BMMI test++
	nova_touch_sysfs_init();
	touch_init_status = 0;
	//HanS support BMMI test--

	//ASUS_BSP Deeo : Parse device tress info +++
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct nt11003_i2c_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "[Touch_N] Failed to allocate memory\n");
			return -ENOMEM;
		}

		error = novatek_parse_dt(&client->dev, pdata);
		if (error){
			dev_err(&client->dev, "[Touch_N] Failed to parse dts\n");
			error = -ENOMEM;
			return error;
		}
	} else
		pdata = client->dev.platform_data;

	printk("[Touch_FH] Parse platform data\n");
	//pdata = client->dev.platform_data;

	//ASUS_BSP Deeo : Parse device tress info ---

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		dev_info(&client->dev,  "[Touch_N] Must have I2C_FUNC_I2C.\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	INIT_WORK(&ts->work, nt11003_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);

	nvts = ts;

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		dev_info(&client->dev, "[Touch_N] Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}

	ts->abs_x_max = pdata->abs_x_max;
	ts->abs_y_max = pdata->abs_y_max;
	ts->max_touch_num = MAX_FINGER_NUM;
	printk("[Touch_N] Max X=%d, Max Y=%d \n", ts->abs_x_max, ts->abs_y_max);


	ts->input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) ;
	ts->input_dev->absbit[0] = BIT(ABS_X) | BIT(ABS_Y) | BIT(ABS_PRESSURE); 						// absolute coor (x,y)
	input_mt_init_slots(ts->input_dev, MAX_FINGER_NUM, 0); //TIODO Quarx check

	input_set_abs_params(ts->input_dev, ABS_X, 0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_Y, 0, ts->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);

#ifdef NT11003_MULTI_TOUCH
	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, ts->abs_x_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, ts->abs_y_max, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, ts->max_touch_num, 0, 0);
#endif

	sprintf(ts->phys, "input/ts");
	ts->input_dev->name = nt11003_ts_name;
	ts->input_dev->phys = ts->phys;
	ts->input_dev->id.bustype = BUS_I2C;
	ts->input_dev->id.vendor = 0x0306;
	ts->input_dev->id.product = 0xFF2F;

	//ASUS_BSP : Add touch deriver attribute +++
	ret = input_register_device(ts->input_dev);
	if (ret) {
		dev_info(&client->dev, "[Touch_N] Probe: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}
	//ASUS_BSP : Add touch deriver attribute ---

	//ASUS_BSP HANS: add for led icon +++
#ifdef HOME_ROW_KEYLIGHT
	led_init(&home_led);

	wq_led_icon = create_singlethread_workqueue("ICON_LED_WQ");
	INIT_DELAYED_WORK(&icon_led_off_w,led_icon_turnoff);
	INIT_DELAYED_WORK(&icon_led_on_w,led_icon_turnon);
#endif
	//ASUS_BSP HANS: add for led icon ---


	if (likely(pdata != NULL)) 
	{
		ts->intr_gpio = pdata->intr_gpio;
		ts->rst_gpio = pdata->rst_gpio;
	}
	printk("[Touch_N] setting rst %d & irq gpio %d \n",ts->rst_gpio,ts->intr_gpio);

	ts->int_trigger_type = INT_TRIGGER; //ASUS_BPS Deeo : set edge fulling (1)
	client->irq = gpio_to_irq(ts->intr_gpio);

	if (client->irq)
	{
		if (ret < 0)
		{
			dev_info(&client->dev, "[Touch_N] Failed to request GPIO:%d, ERRNO:%d\n",ts->intr_gpio,ret);
			goto err_gpio_request_failed;
		}
		dev_info(&client->dev, "[Touch_N] ts->int_trigger_type=%d\n",ts->int_trigger_type);
		ret  = request_irq(client->irq, nt11003_ts_irq_handler ,  irq_table[ts->int_trigger_type],
			client->name, ts);
		if (ret != 0) {
			dev_info(&client->dev, "[Touch_N] Cannot allocate ts INT!ERRNO:%d\n", ret);
			gpio_direction_input(ts->intr_gpio);
			gpio_free(ts->intr_gpio);
			goto err_gpio_request_failed;
		}
		else
		{
			disable_irq(client->irq);
			ts->use_irq = 1;
			dev_info(&client->dev, "[Touch_N]Reques EIRQ %d succesd on GPIO:%d\n",client->irq,ts->intr_gpio);
		}

		nova_hwrst(); //add hw reset
	}    //End of "if (client->irq)"


	i2c_connect_client_nt11003 = client;
	for(retry=0;retry < 30; retry++)
	{
		disable_irq(client->irq);
		msleep(5);
		enable_irq(client->irq);
		ret = CTP_I2C_READ(client, client->addr, test_data, 5);
		dev_info(&client->dev, "[Touch_N] test_data[1]=%d,test_data[2]=%d,test_data[3]=%d,test_data[4]=%d,test_data[5]=%d\n",test_data[1],test_data[2],test_data[3],test_data[4],test_data[5]);
		if (ret > 0)
			break;
		dev_info(&client->dev, "[Touch_N] nt11003 i2c test failed!\n");
	}
	if(ret <= 0)
	{
		dev_info(&client->dev,  "[Touch_N] I2C communication ERROR!nt11003 touchscreen driver become invalid\n");
		goto err_i2c_failed;
	}

	ts->bad_data = 0;

	if(ts->use_irq)
		enable_irq(client->irq);

	// ASUS_BSP Deeo : Add FB notify for early suspend +++
	#if defined(CONFIG_FB)
	ts->fb_notif.notifier_call = fb_notifier_callback;

	ret = fb_register_client(&ts->fb_notif);

	if (ret)
		dev_err(&client->dev, "[Touch_N] Unable to register fb_notifier: %d\n",ret);
	#endif
	// ASUS_BSP Deeo : Add FB notify for early suspend ---

 	nvt_flash_init(ts);


	nv_sdev.name = "touch";
	nv_sdev.print_name = nv_fw_ver;
	switch_dev_register(&nv_sdev);

	//ASUS_BSP : Add attribute +++
	error = sysfs_create_group(&client->dev.kobj, &nt11003_attr_group);
	if(error)
		printk("[Touch_N] Creat Touch IC Attribute Fail!!!\n");
	//ASUS_BSP : Add attribute ---

	dev_info(&client->dev, "[Touch_N] Start %s in %s mode\n",
	ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");


	//ASUS_BSP Deeo : add for On/Off touch in P05 +++
	g_nv_wq_attach_detach = create_singlethread_workqueue("g_nv_wq_attach_detach");
	if (!g_nv_wq_attach_detach) {
		printk("[Touch_N] %s: create workqueue failed: g_nv_wq_attach_detach\n", __func__);
	}
#if defined(CONFIG_EEPROM_NUVOTON)
	INIT_WORK(&g_mp_attach_work_nv, attach_nv_padstation_work);
	INIT_DELAYED_WORK(&g_mp_detach_work_nv, detach_nv_padstation_work);
#endif
	INIT_DELAYED_WORK(&g_resume_work_nv, ts_resume_work);

	INIT_WORK(&nv_mode_w, ts_mode_work);

	//ASUS_BSP Deeo: add wake lock
	wake_lock_init(&touch_wake_lock, WAKE_LOCK_SUSPEND, "touch_wake_lock");
#if defined(CONFIG_EEPROM_NUVOTON)
	register_microp_notifier(&touch_mp_notifier_nv);
	notify_register_microp_notifier(&touch_mp_notifier_nv, "nt11003_10p"); //ASUS_BSP Lenter+
	//ASUS_BSP Deeo : add for On/Off touch in P05 ---
#endif
	//HanS support BMMI test++
	touch_init_status = 1;
	//HanS support BMMI test--
	printk("[Touch_N] nt11306_ts_probe ---\n");

	return 0;


err_i2c_failed:
err_gpio_request_failed:
err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
	i2c_set_clientdata(client, NULL);

	kfree(ts);
err_alloc_data_failed:
err_check_functionality_failed:

	printk("[Touch_N] nt11306_ts_probe fail!!!\n");
	return ret;
}


static int nt11003_ts_remove(struct i2c_client *client)
{
	struct nt11003_ts_data *ts = i2c_get_clientdata(client);

// ASUS_BSP Deeo : Add FB notify for early suspend +++
#if defined(CONFIG_FB)
	if (fb_unregister_client(&ts->fb_notif))
		dev_err(&client->dev, "[Touch_FH] Error occurred while unregistering fb_notifier.\n");
#endif
// ASUS_BSP Deeo : Add FB notify for early suspend ---

	if (ts && ts->use_irq)
	{
		gpio_direction_input(ts->intr_gpio);
		gpio_free(ts->intr_gpio);
		free_irq(client->irq, ts);
	}

	switch_dev_unregister(&nv_sdev);

	dev_notice(&client->dev,"[Touch_N] The driver is removing...\n");
	i2c_set_clientdata(client, NULL);
	input_unregister_device(ts->input_dev);
	kfree(ts);
	return 0;
}


static const struct i2c_device_id nt11003_ts_id[] = {
	{ NT11003_I2C_NAME, 0 },
	{ }
};

static struct of_device_id nt_match_table[] = {
	{ .compatible = "novatek,nt1103-ts",},
	{ },
};

static struct i2c_driver nt11003_ts_driver = {
	.probe		= nt11003_ts_probe,
	.remove		= nt11003_ts_remove,
	.id_table	= nt11003_ts_id,
	.driver = {
		.name	= NT11003_I2C_NAME,
		.owner = THIS_MODULE,
		.of_match_table = nt_match_table,
	},
};


static int nt11003_ts_init(void)
{
	int ret = 0;

	if(g_ASUS_hwID != A90_EVB0){
		printk("[Touch_N] HW:%d not support %s \n",g_ASUS_hwID,__func__);
		return ret;
	}

	nt11003_wq = create_workqueue("nt11003_wq");		//create a work queue and worker thread
	if (!nt11003_wq) {
		printk(KERN_ALERT "[Touch_N] creat workqueue faiked\n");
		return -ENOMEM;
	}
	ret=i2c_add_driver(&nt11003_ts_driver);
	return ret;
}


static void __exit nt11003_ts_exit(void)
{
	printk(KERN_ALERT "[Touch_N] Touchscreen driver of guitar exited.\n");

	destroy_workqueue(g_nv_wq_attach_detach);

#if defined(CONFIG_EEPROM_NUVOTON)
	unregister_microp_notifier(&touch_mp_notifier_nv);
	notify_unregister_microp_notifier(&touch_mp_notifier_nv, "nt11003_10p"); //ASUS_BSP Lenter+
#endif

	i2c_del_driver(&nt11003_ts_driver);
	if (nt11003_wq)
		destroy_workqueue(nt11003_wq);		//release our work queue
}

late_initcall(nt11003_ts_init);
module_exit(nt11003_ts_exit);

MODULE_DESCRIPTION("Novatek Touchscreen Driver");
MODULE_LICENSE("GPL");

#include <linux/atomic.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/microp_notify.h>
#include <linux/microp_notifier_controller.h>
#include <linux/miscdevice.h>
#include <linux/microp.h>
#include <linux/microp_api.h>
#include <linux/microp_pin_def.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/time.h>
#include "linux/input/proximity_class.h"
//#include <linux/microp_api.h>
#include <linux/microp_pin_def.h>
#include <linux/ProximityBasic.h>
#include <linux/wakelock.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h> 
#endif

#define DRIVER_VERSION		"1.0"

/*-------------------------------------------------
    Debug Utility
    -----------------------------------------------*/
#define PROX_SENSOR_DEBUG 0
#define PROX_SENSOR_VERBOSE_DEBUG 1
#define inited_state 0
#define running_state 1
#define suspending_state 2
#define pending_state 3
#define body_detect 1
#define body_undetect 0

void Cap_timer_expired(unsigned long data);
DEFINE_TIMER(Cap_timer, Cap_timer_expired, 0, 0);

#if PROX_SENSOR_DEBUG
#define PROX_DEBUG(format, arg...)	\
	printk(KERN_INFO "CAP1106: [%s] " format , __FUNCTION__ ,  ## arg)
#else
#define PROX_DEBUG(format, arg...)
#endif

#define PROX_INFO(format, arg...)       \
    printk(KERN_INFO "CAP1106: [%s] " format , __FUNCTION__ , ## arg)
#define PROX_ERROR(format, arg...)      \
    printk(KERN_ERR "CAP1106: [%s] " format , __FUNCTION__ , ## arg)

#define CAP1106_DRV_NAME	"cap1106_proximity_sensor"

/*-------------------------------------------------
	Global Variable
    -----------------------------------------------*/

struct cap1106_data {
	struct i2c_client *client;
	struct mutex lock;
	struct input_dev   *input_dev;
	struct delayed_work cap1106_work;
	struct delayed_work cap1106_checking_work;
	int enable;
	int obj_detect;
	int overflow_status;
};

static DEFINE_MUTEX(prox_mtx);
static DEFINE_MUTEX(state_mtx);
static long checking_work_period = 100; //default (ms)
struct cap1106_data *g_cap1106_data_as;
static struct workqueue_struct *cap1106_workqueue = NULL;
static struct delayed_work cap_init_delay_work;
static struct wake_lock alarm_cap_wake_lock;
static int is_wood_sensitivity = 0;
static int no_suspend = 0;
static int c3_acc_cnt = 0;
static int c5_acc_cnt = 0;
static int acc_limit = 5;
static int force_enable = 1;
static int HWID = 0;
static int PadModel = 0;
static int cap_status = 0;
static int bodysar_flag = 0;
static int bodysar_disable = 0;
static int bodysar_counter = 0;
static int pack_mode = 0;
ProximityEventHandler *handlerP;

/*----------------------------------------------------------------------------
 ** FUNCTION DECLARATION
 **----------------------------------------------------------------------------*/

static int __devinit cap1106_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int __devexit cap1106_remove(struct i2c_client *client);
static int cap1106_suspend(struct i2c_client *client, pm_message_t mesg);
static int cap1106_resume(struct i2c_client *client);

static s32 cap1106_read_reg(struct i2c_client *client, u8 command);
static s32 cap1106_write_reg(struct i2c_client *client, u8 command, u8 value);
static void cap1106_enable_sensor(struct i2c_client *client, int enable);
static void cap1106_work_function(struct work_struct *work);
static void cap1106_checking_work_function(struct work_struct *work);
static void cap1106_init_sensor(struct work_struct *work);
static int init_sensor(struct i2c_client *client);
static void dump_registers(struct i2c_client *client);	//for dump
static void enter_state(int);
static int check_power_domain(void);
//void getCapHandler(const char *name, ProximityEventHandler *handler);
void Check_BalanceMode(int IsBalanceMode);

void Cap_onEvent(int val);

static struct switch_dev ls_switch_dev ={ 
        .name = CAP1106_DRV_NAME,
        .index = 0,
};

void Check_BalanceMode(int IsBalanceMode){
	long reg_value;
	static int flag = 0;
	reg_value = cap1106_read_reg(g_cap1106_data_as->client, 0x00);

	pack_mode = IsBalanceMode;
	if(2 == IsBalanceMode){
		cancel_delayed_work_sync(&g_cap1106_data_as->cap1106_checking_work);
		//flush_delayed_work_sync(&g_cap1106_data_as->cap1106_checking_work);
		cap1106_write_reg(g_cap1106_data_as->client, 0x00, (reg_value & 0xEF) | (1 << 4));
		enter_state(inited_state);
		flag = 1;
		printk("[Cap1106] Now in balance mode 2, disable Cap\n");
	}else{
		if(1 == flag){
			enter_state(running_state);
			queue_delayed_work(cap1106_workqueue, &cap_init_delay_work, 0);
			flag = 0;
			printk("[Cap1106] Now leave balance mode 2, enable Cap\n");
		}
	}
}

void Cap_onEvent(int val){
	ProximityEventHandler *ProxEventHandler = handlerP;
	ProxEventHandler->onEvent(handlerP, "cap_sensor", val);
}

void setCap1106Handler(const char * name, ProximityEventHandler * handler){
	handlerP = handler;
}

static int cap1106_sensor_dsleep(struct i2c_client *client)
{
	u8 bIdx;
	int rc = 0;
    
	const u8 InitTable[] = {
		0x00, 0xD0,
	};

	for (bIdx = 0; bIdx < sizeof(InitTable) / sizeof(InitTable[0]); bIdx += 2) {
		if ((rc = i2c_smbus_write_byte_data(client, InitTable[bIdx], InitTable[bIdx + 1]))) {
			printk("=== Write Error, rc=0x%X\n", rc);
			break;
		}
		else{
			printk("[Cap1106] set 0x%x, data = %x\n", InitTable[bIdx], InitTable[bIdx + 1]);
		}
	}
	return rc;
}

static int cap1106_sensor_active(struct i2c_client *client)
{
	u8 bIdx;
	int rc = 0;
    
	const u8 InitTable[] = {
		0x00, 0xC0,
	};

	for (bIdx = 0; bIdx < sizeof(InitTable) / sizeof(InitTable[0]); bIdx += 2) {
		if ((rc = i2c_smbus_write_byte_data(client, InitTable[bIdx], InitTable[bIdx + 1]))) {
			printk("=== Write Error, rc=0x%X\n", rc);
			break;
		}
		else{
			printk("[Cap1106] set 0x%x, data = %x\n", InitTable[bIdx], InitTable[bIdx + 1]);
		}
	}
	return rc;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Device Attributes Sysfs Show/Store
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static ssize_t show_obj_detect(struct device *dev, struct device_attribute *devattr, char *buf)
{
    int ret = 0;

    PROX_DEBUG("\n");

    mutex_lock(&prox_mtx);

    if (g_cap1106_data_as->enable) {
        ret = sprintf(buf, "%d\n", g_cap1106_data_as->obj_detect);
    } else {
        ret = sprintf(buf, "-1\n");
    }
    mutex_unlock(&prox_mtx);

    return ret;
}

static ssize_t show_sensitivity(struct device *dev, struct device_attribute *devattr, char *buf)
{
    int value;
    int ret = 0;

    PROX_DEBUG("\n");

    mutex_lock(&prox_mtx);

    if (g_cap1106_data_as->enable) {
        value = cap1106_read_reg(g_cap1106_data_as->client, 0x1F);
        ret = sprintf(buf, "%02X\n", value);
    } else {
        ret = sprintf(buf, "-1\n");
    }
    mutex_unlock(&prox_mtx);

    return ret;
}

static ssize_t store_sensitivity(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    long value;

    PROX_DEBUG("\n");

    if (strict_strtol(buf, 16, &value))
        return -EINVAL;

    mutex_lock(&prox_mtx);
    if (g_cap1106_data_as->enable) {
        cap1106_write_reg(g_cap1106_data_as->client, 0x1F, value & 0x7F);
    }
    mutex_unlock(&prox_mtx);

    return strnlen(buf, count);
}

static ssize_t show_sensor_gain(struct device *dev, struct device_attribute *devattr, char *buf)
{
    int value;
    int ret = 0;

    PROX_DEBUG("\n");

    mutex_lock(&prox_mtx);
    if (g_cap1106_data_as->enable) {
        value = cap1106_read_reg(g_cap1106_data_as->client, 0x00);
        ret = sprintf(buf, "%02X\n", (value & 0xC0) >> 6);
    } else {
        ret = sprintf(buf, "-1\n");
    }
    mutex_unlock(&prox_mtx);

    return ret;
}

static ssize_t store_sensor_gain(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    long gain_value;
    long reg_value;

    PROX_DEBUG("\n");

    if (strict_strtol(buf, 16, &gain_value))
        return -EINVAL;

    mutex_lock(&prox_mtx);
    if (g_cap1106_data_as->enable) {
        reg_value = cap1106_read_reg(g_cap1106_data_as->client, 0x00);
        cap1106_write_reg(g_cap1106_data_as->client, 0x00, (reg_value & 0x3F) | ((gain_value & 0x3) << 6));
    }
    mutex_unlock(&prox_mtx);

    return strnlen(buf, count);
}

static ssize_t show_sensor_input_3_delta_count(struct device *dev, struct device_attribute *devattr, char *buf)
{
    int value;
    int ret = 0;

    PROX_DEBUG("\n");

    mutex_lock(&prox_mtx);
    if (g_cap1106_data_as->enable) {
        value = cap1106_read_reg(g_cap1106_data_as->client, 0x12);
        ret = sprintf(buf, "%02X\n", value);
    } else {
        ret = sprintf(buf, "-1\n");
    }
    mutex_unlock(&prox_mtx);
    
    return ret;
}

static ssize_t show_sensor_input_3_th(struct device *dev, struct device_attribute *devattr, char *buf)
{
    int value;
    int ret = 0;

    PROX_DEBUG("\n");

    mutex_lock(&prox_mtx);
    if (g_cap1106_data_as->enable) {
        value = cap1106_read_reg(g_cap1106_data_as->client, 0x32);
        ret = sprintf(buf, "%02X\n", value);
    } else {
        ret = sprintf(buf, "-1\n");
    }
    mutex_unlock(&prox_mtx);

    return ret;
}

static ssize_t store_sensor_input_3_th(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    long value;

    PROX_DEBUG("\n");

    if (strict_strtol(buf, 16, &value))
        return -EINVAL;

    mutex_lock(&prox_mtx);
    if (g_cap1106_data_as->enable) {
        cap1106_write_reg(g_cap1106_data_as->client, 0x32, value & 0x7F);
    }
    mutex_unlock(&prox_mtx);

    return strnlen(buf, count);
}

static ssize_t show_sensor_input_5_delta_count(struct device *dev, struct device_attribute *devattr, char *buf)
{
    int value;
    int ret = 0;

    PROX_DEBUG("\n");

    mutex_lock(&prox_mtx);
    if (g_cap1106_data_as->enable) {
        value = cap1106_read_reg(g_cap1106_data_as->client, 0x14);
        ret = sprintf(buf, "%02X\n", value);
    } else {
        ret = sprintf(buf, "-1\n");
    }
    mutex_unlock(&prox_mtx);

    return ret;
}

static ssize_t show_sensor_input_5_th(struct device *dev, struct device_attribute *devattr, char *buf)
{
    int value;
    int ret = 0;

    PROX_DEBUG("\n");

    mutex_lock(&prox_mtx);
    if (g_cap1106_data_as->enable) {
        value = cap1106_read_reg(g_cap1106_data_as->client, 0x34);
        ret = sprintf(buf, "%02X\n", value);
    } else {
        ret = sprintf(buf, "-1\n");
    }
    mutex_unlock(&prox_mtx);

    return ret;
}

static ssize_t store_sensor_input_5_th(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    long value;

    PROX_DEBUG("\n");

    if (strict_strtol(buf, 16, &value))
        return -EINVAL;

    mutex_lock(&prox_mtx);
    if (g_cap1106_data_as->enable) {
        cap1106_write_reg(g_cap1106_data_as->client, 0x34, value & 0x7F);
    }
    mutex_unlock(&prox_mtx);

    return strnlen(buf, count);
}

static ssize_t show_sensor_input_noise_th(struct device *dev, struct device_attribute *devattr, char *buf)
{
    int value;
    int ret = 0;

    PROX_DEBUG("\n");

    mutex_lock(&prox_mtx);
    if (g_cap1106_data_as->enable) {
        value = cap1106_read_reg(g_cap1106_data_as->client, 0x38);
        ret = sprintf(buf, "%02X\n", value & 0x3);
    } else {
        ret = sprintf(buf, "-1\n");
    }
    mutex_unlock(&prox_mtx);

    return ret;
}

static ssize_t store_sensor_input_noise_th(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    long value;

    PROX_DEBUG("\n");

    if (strict_strtol(buf, 16, &value))
        return -EINVAL;

    mutex_lock(&prox_mtx);
    if (g_cap1106_data_as->enable) {
        cap1106_write_reg(g_cap1106_data_as->client, 0x38, value & 0x3);
    }
    mutex_unlock(&prox_mtx);

    return strnlen(buf, count);
}

static ssize_t show_sensor_input_status(struct device *dev, struct device_attribute *devattr, char *buf)
{
    int value;
    int ret = 0;

    PROX_DEBUG("\n");

    mutex_lock(&prox_mtx);

    if (g_cap1106_data_as->enable) {
        value = cap1106_read_reg(g_cap1106_data_as->client, 0x03);
        ret = sprintf(buf, "%02X\n", value);
    } else {
        ret = sprintf(buf, "-1\n");
    }
    mutex_unlock(&prox_mtx);

    return ret;
}

static ssize_t show_sensing_cycle(struct device *dev, struct device_attribute *devattr, char *buf)
{
    int value;
    int ret = 0;

    PROX_DEBUG("\n");

    mutex_lock(&prox_mtx);
    if (g_cap1106_data_as->enable) {
        value = cap1106_read_reg(g_cap1106_data_as->client, 0x24);
        ret = sprintf(buf, "%02X\n", value);
    } else {
        ret = sprintf(buf, "-1\n");
    }
    mutex_unlock(&prox_mtx);

    return ret;
}

static ssize_t store_sensing_cycle(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    long value;

    PROX_DEBUG("\n");

    if (strict_strtol(buf, 16, &value))
        return -EINVAL;

    mutex_lock(&prox_mtx);
    if (g_cap1106_data_as->enable) {
        cap1106_write_reg(g_cap1106_data_as->client, 0x24, value & 0x7F);
    }
    mutex_unlock(&prox_mtx);

    return strnlen(buf, count);
}

static ssize_t show_sensor_onoff(struct device *dev, struct device_attribute *devattr, char *buf)
{
    int ret = 0;

    PROX_DEBUG("\n");

    mutex_lock(&prox_mtx);
    ret = sprintf(buf, "%d\n", g_cap1106_data_as->enable);
    mutex_unlock(&prox_mtx);
    return ret;
}

static ssize_t store_sensor_onoff(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    long enable;

    PROX_DEBUG("\n");

    if (strict_strtol(buf, 10, &enable))
        return -EINVAL;

    if ((enable != 1) && (enable != 0))
        return -EINVAL;

    mutex_lock(&prox_mtx);
    force_enable = enable;
    cap1106_enable_sensor(g_cap1106_data_as->client, enable);
    mutex_unlock(&prox_mtx);

    return strnlen(buf, count);
}

static ssize_t show_sensor_recal(struct device *dev, struct device_attribute *devattr, char *buf)
{
    int value;
    int ret = 0;

    PROX_DEBUG("\n");

    mutex_lock(&prox_mtx);
    if (g_cap1106_data_as->enable) {
        value = cap1106_read_reg(g_cap1106_data_as->client, 0x26);
        ret = sprintf(buf, value == 0x0 ? "OK\n" : "FAIL\n");
    } else {
        ret = sprintf(buf, "FAIL\n");
    }
    mutex_unlock(&prox_mtx);

    return ret;
}

static ssize_t store_sensor_recal(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{

    PROX_DEBUG("\n");

    mutex_lock(&prox_mtx);
    if (g_cap1106_data_as->enable) {
        cap1106_write_reg(g_cap1106_data_as->client, 0x26, 0x14);
    }
    mutex_unlock(&prox_mtx);

    return strnlen(buf, count);
}

static int cap1106_init(struct device *dev, struct device_attribute *attr, char *buf){
	int lux = 0;

	printk("CAP1106 +++ init \n");
	lux = init_sensor(g_cap1106_data_as->client);
	printk("CAP1106 --- init \n");

	return 1;
}

static int cap1106_active(struct device *dev, struct device_attribute *attr, char *buf){
	int lux = 0;

	printk("CAP1106 +++ sensor active");
	lux = cap1106_sensor_active(g_cap1106_data_as->client);
	printk("CAP1106 --- sensor active");

	return 1;
}

static int cap1106_dsleep(struct device *dev, struct device_attribute *attr, char *buf){
	int lux = 0;

	printk("CAP1106 +++ sensor dsleep");
	lux = cap1106_sensor_dsleep(g_cap1106_data_as->client);
	printk("CAP1106 --- sensor dsleep");

	return 1;
}

static ssize_t cap1106_show_result(struct device *dev, struct device_attribute *devattr, char *buf){
	int state;
	int ret = 0;
	
	state = cap1106_read_reg(g_cap1106_data_as->client, 0x03);
	if(state == 0x04)
		ret = sprintf(buf, "1,0\n");
	else if(state == 0x10)
		ret = sprintf(buf, "0,1\n");
	else if(state == 0x14)
		ret = sprintf(buf, "1,1\n");
	else
		ret = sprintf(buf, "0,0\n");
	return ret;
}

static ssize_t cap1106_dump_register(struct device *dev, struct device_attribute *devattr, char *buf){

	dump_registers(g_cap1106_data_as->client);
	return 0;
}

static ssize_t cap1106_bodysar_read(struct device *dev, struct device_attribute *devattr, char *buf){
	int ret = 0;
	ret = sprintf(buf, "%d\n", bodysar_disable);
	return ret;
}

static ssize_t cap1106_bodysar_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count){
	long value;
	int lux;

	if (strict_strtol(buf, 10, &value))
		return -EINVAL;
	bodysar_disable = (int)value;

	if(bodysar_disable){
		lux = cap1106_sensor_dsleep(g_cap1106_data_as->client);
		Cap_onEvent(body_undetect);
	}else{
		lux = cap1106_sensor_active(g_cap1106_data_as->client);
	}

	return strnlen(buf, count);
}

static ssize_t bodysar_counter_read(struct device *dev, struct device_attribute *devattr, char *buf){
	int ret;
	ret = sprintf(buf, "%d\n", bodysar_counter);
	return ret;
}

static ssize_t bodysar_counter_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count){
	long value;
	if (strict_strtol(buf, 10, &value))
		return -EINVAL;
	bodysar_counter = (int)value;

	return strnlen(buf, count);
}

DEVICE_ATTR(obj_detect, 0644, show_obj_detect, NULL);
DEVICE_ATTR(sensitivity, 0644, show_sensitivity, store_sensitivity);
DEVICE_ATTR(sensor_gain, 0644, show_sensor_gain, store_sensor_gain);
DEVICE_ATTR(sensor_input_3_delta_count, 0644, show_sensor_input_3_delta_count, NULL);
DEVICE_ATTR(sensor_input_3_th, 0644, show_sensor_input_3_th, store_sensor_input_3_th);
DEVICE_ATTR(sensor_input_5_delta_count, 0644, show_sensor_input_5_delta_count, NULL);
DEVICE_ATTR(sensor_input_5_th, 0644, show_sensor_input_5_th, store_sensor_input_5_th);
DEVICE_ATTR(sensor_input_noise_th, 0644, show_sensor_input_noise_th, store_sensor_input_noise_th);
DEVICE_ATTR(sensor_input_status, 0644, show_sensor_input_status, NULL);
DEVICE_ATTR(sensing_cycle, 0644, show_sensing_cycle, store_sensing_cycle);
DEVICE_ATTR(sensor_onoff, 0644, show_sensor_onoff, store_sensor_onoff);
DEVICE_ATTR(sensor_recal, 0644, show_sensor_recal, store_sensor_recal);
DEVICE_ATTR(init, S_IRWXU | S_IRWXG | S_IROTH, cap1106_init, NULL);
DEVICE_ATTR(active, S_IRWXU | S_IRWXG | S_IROTH, cap1106_active, NULL);
DEVICE_ATTR(dsleep, S_IRWXU | S_IRWXG | S_IROTH, cap1106_dsleep, NULL);
DEVICE_ATTR(result, S_IRWXU | S_IRWXG | S_IROTH, cap1106_show_result, NULL);
DEVICE_ATTR(dump_reg, S_IRWXU | S_IRWXG | S_IROTH, cap1106_dump_register, NULL);
DEVICE_ATTR(bodysar_disable, 0644, cap1106_bodysar_read, cap1106_bodysar_write);
DEVICE_ATTR(bodysar_counter_check, 0644, bodysar_counter_read, bodysar_counter_write);

static struct attribute *cap1106_attr[] = {
    &dev_attr_obj_detect.attr,
    &dev_attr_sensitivity.attr,
    &dev_attr_sensor_gain.attr,
    &dev_attr_sensor_input_3_delta_count.attr,
    &dev_attr_sensor_input_3_th.attr,
    &dev_attr_sensor_input_5_delta_count.attr,
    &dev_attr_sensor_input_5_th.attr,
    &dev_attr_sensor_input_noise_th.attr,
    &dev_attr_sensor_input_status.attr,
    &dev_attr_sensing_cycle.attr,
    &dev_attr_sensor_onoff.attr,
    &dev_attr_sensor_recal.attr,
    &dev_attr_init.attr,
    &dev_attr_active.attr,
    &dev_attr_dsleep.attr,
    &dev_attr_result.attr,
    &dev_attr_dump_reg.attr,
    &dev_attr_bodysar_disable.attr,
    &dev_attr_bodysar_counter_check.attr,
    NULL
};

#if PROX_SENSOR_VERBOSE_DEBUG
static void dump_registers(struct i2c_client *client)
{
    int value;
    value = cap1106_read_reg(client, 0x00);
    PROX_ERROR("=== Main Control(0x00) is %x\n", value);
    value = cap1106_read_reg(client, 0x02);
    PROX_ERROR("=== Genaral Status(0x02) is %x\n", value);
    value = cap1106_read_reg(client, 0x03);
    PROX_ERROR("=== Sensor Input Status(0x03) is %x\n", value);
    value = cap1106_read_reg(client, 0x0A);
    PROX_ERROR("=== Noise Flag Status(0x0A) is %x\n", value);
    value = cap1106_read_reg(client, 0x21);
    PROX_ERROR("=== Sensor Input Enable Register(0x21) is %x\n", value);
    value = cap1106_read_reg(client, 0x27);
    PROX_ERROR("=== Sensor Input Enable Register(0x27) is %x\n", value);
    value = cap1106_read_reg(client, 0x44);
    PROX_ERROR("=== configuration 2(0x44) is %x\n", value);
    value = cap1106_read_reg(client, 0xFD);
    PROX_ERROR("=== Product ID(0xFD) is %x\n", value);
    value = cap1106_read_reg(client, 0xFE);
    PROX_ERROR("=== Manufacturer ID(0xFE) is %x\n", value);
    value = cap1106_read_reg(client, 0xFF);
    PROX_ERROR("=== Revision (0xFF) is %x\n", value);
}
#endif

static void cap1106_enable_sensor(struct i2c_client *client, int enable)
{
    long reg_value;

    if (g_cap1106_data_as->enable != enable) {
        reg_value = cap1106_read_reg(client, 0x00);
        if (enable) {
//            cap1106_write_reg(client, 0x00, (reg_value & 0xEF) | (!enable << 4));
            // Time to first conversion is 200ms (Max)
            queue_delayed_work(cap1106_workqueue, &g_cap1106_data_as->cap1106_work, msecs_to_jiffies(200));
            queue_delayed_work(cap1106_workqueue, &g_cap1106_data_as->cap1106_checking_work, checking_work_period);
        } else {
//   		cancel_delayed_work_sync(&g_cap1106_data_as->cap1106_work);
		cancel_delayed_work_sync(&g_cap1106_data_as->cap1106_checking_work);

//            switch_set_state(&ls_switch_dev, 0);
//            cap1106_write_reg(client, 0x00, (reg_value & 0xEF) | (!enable << 4));
        }
        g_cap1106_data_as->enable = enable;
    }
}

static s32 cap1106_read_reg(struct i2c_client *client, u8 command)
{
    return i2c_smbus_read_byte_data(client, command);
}

static s32 cap1106_write_reg(struct i2c_client *client, u8 command, u8 value)
{
	static int ret = 20;
	ret = i2c_smbus_write_byte_data(client, command, value);
	return ret;
}

void Cap_timer_expired(unsigned long data){
	if(AX_MicroP_IsP01Connected()){
		queue_delayed_work(cap1106_workqueue, &g_cap1106_data_as->cap1106_work, 0 * HZ);
	}
}
/*
static void enter_state(int  state){
	if(state == running_state){
		if(check_power_domain()){
			cap_status = running_state;
		}
		else{
			cap_status = pending_state;
		}
	}
	else if(state == suspending_state){
		cap_status = suspending_state;
	}
	else if(state == inited_state){
		cap_status = inited_state;
	}
}
*/

static void enter_state(int  state){
	mutex_lock(&state_mtx);
	switch(cap_status){
		case inited_state:
			if(check_power_domain()){
				if(state == running_state){
					cap_status = running_state;
				}else if(state == suspending_state){
					cap_status = suspending_state;
				}else{
					printk("[CapSensor] %s error state : inited to inited\n", __FUNCTION__);
				}
			}else{
				cap_status = pending_state;
			}
			break;

		case running_state:
			if(check_power_domain()){
				if(state == inited_state){
					cap_status = inited_state;
				}else if(state == suspending_state){
					cap_status = suspending_state;
				}else{
					printk("[CapSensor] %s error state : running to running\n", __FUNCTION__);
				}
			}else{
				cap_status = suspending_state;
			}
			break;

		case suspending_state:
			if(check_power_domain()){
				if(state == inited_state){
					cap_status = inited_state;
				}else if(state == running_state){
					cap_status = running_state;
				}else{
					printk("[CapSensor] %s error state : suspending to suspending\n", __FUNCTION__);
				}
			}else{
				if(state == inited_state){
					cap_status = inited_state;
				}else if(state == running_state){
					cap_status = pending_state;
				}else{
					printk("[CapSensor] %s error state : suspending to suspending\n", __FUNCTION__);
				}
			}
			break;

		case pending_state:
			if(state == inited_state){
				cap_status = inited_state;
			}else if(state == running_state){
				cap_status = running_state;
			}else if(state == suspending_state){
				cap_status = suspending_state;
			}else{
				printk("[CapSensor] %s error state : pending to pending\n", __FUNCTION__);
			}
			break;
	}
	mutex_unlock(&state_mtx);
	printk("[CapSensor] %s cap_status = %d\n", __FUNCTION__, cap_status);
}

static int check_power_domain(void){
	if(0 == HWID){
		return AX_MicroP_Is_3V3_ON();
	}
	else{
		return 1;
	}
}

static void cap1106_work_function(struct work_struct *work)
{
    int value;
    int status;
    int value_delta_3,value_delta_5;
    int bc3, bc5;
    struct cap1106_data *data = container_of(work, struct cap1106_data, cap1106_work.work);
	
    if(cap_status != pending_state){
        if(!check_power_domain() || inited_state == cap_status){	//for 3v3 off disable work function
            return;
        }
        value = cap1106_read_reg(g_cap1106_data_as->client, 0x00);
        if(!(value & 0x01)){
            return;
        }
    }
    else{
        if(check_power_domain()){
            enter_state(running_state);
        }
        else{
            queue_delayed_work(cap1106_workqueue, &g_cap1106_data_as->cap1106_work, 5 * HZ);
            return;
        }
    }

    if(bodysar_disable || 2 == pack_mode){
        return;
    }

    mutex_lock(&prox_mtx);

    cap1106_write_reg(g_cap1106_data_as->client, 0x00, 0x80); // Clear INT and Set Gain to MAX
   
    status = cap1106_read_reg(data->client, 0x03);
    value_delta_3 = cap1106_read_reg(g_cap1106_data_as->client, 0x12);
    value_delta_5 = cap1106_read_reg(g_cap1106_data_as->client, 0x14);
    bc3 = cap1106_read_reg(g_cap1106_data_as->client, 0x52);
    bc5 = cap1106_read_reg(g_cap1106_data_as->client, 0x54);
    PROX_DEBUG("Status: 0x%02X, BC3=0x%02X, D3=0x%02X, BC5=0x%02X, D5=0x%02X\n", status, bc3, value_delta_3, bc5, value_delta_5);
if(1){    
    if (0 == is_wood_sensitivity) {
        data->obj_detect = ((0x4 == status) || (0x10 == status) || (0x14 == status));
        if ((0x4 == status && 0x3e <= value_delta_3)
            || (0x10 == status && 0x32 <= value_delta_5)
            || (0x14 == status && (0x3e <= value_delta_3 || 0x32 <= value_delta_5))) {
            PROX_DEBUG("set to wood sensitivity------>\n");

            //set sensitivity and threshold for wood touch
            cap1106_write_reg(g_cap1106_data_as->client, 0x1f, 0x3f);
            cap1106_write_reg(g_cap1106_data_as->client, 0x32, 0x02);
            cap1106_write_reg(g_cap1106_data_as->client, 0x34, 0x02);

            is_wood_sensitivity = 1;
            data->overflow_status = status;
            c3_acc_cnt = 0;
            c5_acc_cnt = 0;
        }
    }  
}
    mod_timer(&Cap_timer, jiffies + msecs_to_jiffies(1000));
    mutex_unlock(&prox_mtx);    
}

static int init_sensor(struct i2c_client *client){
    u8 bIdx, InitTableSize;
    int rc = 0;
    
    const u8 InitTable[] = {
        0x1f, 0x1f, // Data sensitivity (need to be fine tune for real system).
        0x20, 0x20, // MAX duration disable
        0x21, 0x14, // Enable CS3+CS5.
        0x22, 0xff, // MAX duration time to max , repeat period time to max
        0x24, 0x38, // digital count update time to 140*64ms
        0x27, 0x14, // Enable INT. for CS3+CS5.
        0x28, 0x00, // disable repeat irq
        0x2a, 0x00, // all channel run in the same time
        0x32, 0x3e, // Threshold of CS 3 (need to be fine tune for real system).
        0x34, 0x32, // Threshold of CS 5 (need to be fine tune for real system).
        0x26, 0x14, // force re-cal
        0x2f, 0x82,        
        0x00, 0x80, // Reset INT. bit.
    };

    InitTableSize= sizeof(InitTable) / sizeof(InitTable[0]);
    for (bIdx = 0; bIdx < InitTableSize; bIdx += 2) {
        if ((rc = i2c_smbus_write_byte_data(g_cap1106_data_as->client, InitTable[bIdx], InitTable[bIdx + 1]))) {
            printk("=== Write Error, rc=0x%X\n", rc);
            break;
        }
    }
    return rc;
}

static void cap1106_init_sensor(struct work_struct *work)
{
    u8 bIdx, InitTableSize;
    int rc = 0;
    
    const u8 InitTable[] = {
        0x1f, 0x1f, // Data sensitivity (need to be fine tune for real system).
        0x20, 0x20, // MAX duration disable
        0x21, 0x14, // Enable CS3+CS5.
        0x22, 0xff, // MAX duration time to max , repeat period time to max
        0x24, 0x38, // digital count update time to 140*64ms
        0x27, 0x14, // Enable INT. for CS3+CS5.
        0x28, 0x00, // disable repeat irq
        0x2a, 0x00, // all channel run in the same time
        0x32, 0x3e, // Threshold of CS 3 (need to be fine tune for real system).
        0x34, 0x32, // Threshold of CS 5 (need to be fine tune for real system).
        0x26, 0x14, // force re-cal
        0x2f, 0x82,
        0x00, 0x80, // Reset INT. bit.
    };
printk("!!! SHAWN TEST !!!\n");
    AX_MicroP_setGPIOOutputPin(OUT_uP_CAP_PWR_EN,1);

//    enter_state(running_state);
    if(cap_status == inited_state){
        return;
    }
    if(cap_status == pending_state){
        queue_delayed_work(cap1106_workqueue, &cap_init_delay_work, 2 * HZ);
        return;
    }
    
    InitTableSize= sizeof(InitTable) / sizeof(InitTable[0]);
    for (bIdx = 0; bIdx < InitTableSize; bIdx += 2) {
        if ((rc = i2c_smbus_write_byte_data(g_cap1106_data_as->client, InitTable[bIdx], InitTable[bIdx + 1]))) {
            printk("=== Write Error, rc=0x%X\n", rc);
            break;
        }
    }

    if(bIdx == InitTableSize){
        g_cap1106_data_as->enable = 1;
        PROX_DEBUG("+++++++++++++CAP1106 delay work!!\n");
        queue_delayed_work(cap1106_workqueue, &g_cap1106_data_as->cap1106_work, msecs_to_jiffies(200));
        queue_delayed_work(cap1106_workqueue, &g_cap1106_data_as->cap1106_checking_work, checking_work_period);
        PROX_DEBUG("---------------CAP1106 init sucess!!\n");
    }
    else{
        queue_delayed_work(cap1106_workqueue, &cap_init_delay_work, 2 * HZ);
    }

    return;
}

static void cap1106_checking_work_function(struct work_struct *work) {
    int status;
    int value_delta_3;
    int value_delta_5;
    int bc3, bc5;

    //for P92L SR1 +++
    if(!check_power_domain() || cap_status == inited_state){	//for 3v3 off disable work function
        is_wood_sensitivity = 0;
        return;
    }
    //for P92L SR1 ---

//printk("%s !!! is_wood_sensitivity = %d, bodysar_flag = %d !!!\n", __FUNCTION__, is_wood_sensitivity, bodysar_flag);

    mutex_lock(&prox_mtx);	
    if (1 == is_wood_sensitivity && 2 != pack_mode){
        if (g_cap1106_data_as->enable) {
            status = cap1106_read_reg(g_cap1106_data_as->client, 0x03);
            value_delta_3 = cap1106_read_reg(g_cap1106_data_as->client, 0x12);
            value_delta_5 = cap1106_read_reg(g_cap1106_data_as->client, 0x14);
            bc3 = cap1106_read_reg(g_cap1106_data_as->client, 0x52);
            bc5 = cap1106_read_reg(g_cap1106_data_as->client, 0x54);
            PROX_DEBUG("Status: 0x%02X, BC3=0x%02X, D3=0x%02X, BC5=0x%02X, D5=0x%02X\n", status, bc3, value_delta_3, bc5, value_delta_5);
            if ((0x00 == value_delta_3 && 0x00 == value_delta_5)
                || (0xFF == value_delta_3 && 0xFF == value_delta_5)
                || (0x00 == value_delta_3 && 0xFF == value_delta_5)
                || (0xFF == value_delta_3 && 0x00 == value_delta_5)
		   ) {
                PROX_DEBUG("unset is_wood_sensitivity to 0\n");
                //set sensitivity and threshold for 2cm body distance
                cap1106_write_reg(g_cap1106_data_as->client, 0x1f, 0x1f);
                cap1106_write_reg(g_cap1106_data_as->client, 0x32, 0x3e);
                cap1106_write_reg(g_cap1106_data_as->client, 0x34, 0x32);
                if(1 == bodysar_flag){
                    Cap_onEvent(body_undetect);
                    wake_lock_timeout(&alarm_cap_wake_lock, 2 * HZ);
                    bodysar_flag = 0;
                    c3_acc_cnt = 0;
                    c5_acc_cnt = 0;
                }
                is_wood_sensitivity = 0;
                no_suspend = 0;
                queue_delayed_work(cap1106_workqueue, &g_cap1106_data_as->cap1106_work, 0);
            }else{
                if(0 == bodysar_flag){
                    Cap_onEvent(body_detect);
                    wake_lock_timeout(&alarm_cap_wake_lock, 2 * HZ);
                    bodysar_flag = 1;
                    bodysar_counter++;
                    if(bodysar_counter > 20000){
                        bodysar_counter = 0;
                    }
                }
                no_suspend = 0;
            }
        } else {
            PROX_DEBUG("delta 2 = -1\n");
        }
    }
    if ((0x03 <= value_delta_3 && 0x10 >= value_delta_3) || (0x20 <= value_delta_3 && 0x2e >= value_delta_3)){
        wake_lock_timeout(&alarm_cap_wake_lock, 6 * HZ);
        c3_acc_cnt++;
        no_suspend = 1;
    }
    if ((0x03 <= value_delta_5 && 0x10 >= value_delta_5)){
        wake_lock_timeout(&alarm_cap_wake_lock, 6 * HZ);
        c5_acc_cnt++;
        no_suspend = 1;
    }
    if(0x02 <= value_delta_3 && 0x02 <= value_delta_5){
        no_suspend = 0;
    }
	
    printk(DBGMSK_PRX_G6"!!! c3_acc_cnt=%d, c5_acc_cnt=%d, no_suspend = %d\n", c3_acc_cnt, c5_acc_cnt, no_suspend);
    if (c3_acc_cnt >= acc_limit || c5_acc_cnt >= acc_limit) {
        PROX_DEBUG("+++ FORCE RECALIBRATION +++\n");
        cap1106_write_reg(g_cap1106_data_as->client, 0x26, 0x14);
        c3_acc_cnt = 0;
        c5_acc_cnt = 0;
        no_suspend = 0;
    }
    mutex_unlock(&prox_mtx);
    queue_delayed_work(cap1106_workqueue, &g_cap1106_data_as->cap1106_checking_work, 100);
//printk("%s !!! is_wood_sensitivity = %d, bodysar_flag = %d !!!\n", __FUNCTION__, is_wood_sensitivity, bodysar_flag);
}

static const struct attribute_group cap1106_attr_group = {
    .name = "cap1106",
    .attrs = cap1106_attr,
};


static int cap1106_get_value(struct i2c_client *client)
{
	int err1, err2;
	int result = 0;
	printk(DBGMSK_PRX_G6"++cap1106_get_value \n");

	mutex_lock(&g_cap1106_data_as->lock);

	err1 = i2c_smbus_read_byte_data(client, 0x00);
	printk(DBGMSK_PRX_G6"cap1106_get_value: reg (0x00) = 0x%x\n", err1);
	err2 = i2c_smbus_write_byte_data(client, 0x1f, 0x1f);
	printk(DBGMSK_PRX_G6"cap1106_get_value: reg (0x1f) = 0x%x\n", err2);

	if(err1 < 0){
		result = err1;
	}else if(err2 < 0){
		result = err2;
	}else{
		result = 1;
	}
	
	mutex_unlock(&g_cap1106_data_as->lock);

	return result;
}

#ifdef CONFIG_I2C_STRESS_TEST

#include <linux/i2c_testcase.h>

#define I2C_TEST_FAIL_SENSOR (-1)

static int Test_Cap1106_SensorI2C(struct i2c_client *apClient)
{
	int err = 0;
	int lnResult = I2C_TEST_PASS;

	i2c_log_in_test_case("Test_Cap1106_SensorI2C++\n");

	//schedule_timeout(msecs_to_jiffies(100));
	if(cap_status == running_state){
		err = cap1106_get_value(apClient);
		if ( err <= 0 ){
			i2c_log_in_test_case("Fail to read/write cap1106 sensor\n");
			lnResult = I2C_TEST_FAIL_SENSOR;
		}
	}
	else{
		i2c_log_in_test_case("Fail to connect cap1106 sensor\n");
		lnResult = I2C_TEST_FAIL_SENSOR;
	}
	i2c_log_in_test_case("Test_Cap1106_SensorI2C--\n");

	return lnResult;
};

static struct i2c_test_case_info CapSensorTestInfo[] =
{
	__I2C_STRESS_TEST_CASE_ATTR(Test_Cap1106_SensorI2C),
};
#endif


static int capsensor_pad_mp_event(struct notifier_block *this, unsigned long event, void *ptr)
{
	switch (event) {
		case P01_ADD:
			printk("[CAP_P01][MicroP] CAP_P01_ADD \r\n");
			HWID = AX_MicroP_getHWID();
			PadModel = AX_MicroP_getPadModel();
			printk("[Cap1106] PadModel = %d  \r\n", PadModel);
			if(4 == PadModel){
				enter_state(running_state);
				queue_delayed_work(cap1106_workqueue, &cap_init_delay_work, 2 * HZ);
			}else{
				printk("[Cap1106] Not P92L!!Cap disable. \r\n");
			}
			return NOTIFY_DONE;

		case P01_REMOVE:
			printk("[CAP_P01][MicroP] CAP_P01_REMOVE \r\n");
			enter_state(inited_state);
			return NOTIFY_DONE;

		case PAD_CAP_SENSOR_STATUS_CHANGE:
			printk("[CAP_P01][MicroP] PAD_CAP_SENSOR_STATUS_CHANGE \r\n");
			if(AX_MicroP_IsP01Connected())	{
				queue_delayed_work(cap1106_workqueue, &g_cap1106_data_as->cap1106_work, 0 * HZ);
			}
			else{
				printk(DBGMSK_PRX_G2"[CAP_P01] Inerrupt error \r\n");
			}
			return NOTIFY_DONE;

		default:
			return NOTIFY_DONE;
		}
}

static struct notifier_block capsensor_pad_mp_notifier = {
       .notifier_call = capsensor_pad_mp_event,
       .priority = CAP_SENSOR_MP_NOTIFY,
};

static int __devinit cap1106_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	int err = 0;

	/*......................................init port......................................*/
	PROX_DEBUG("[cap_P92] cap1106_init++\n");

	cap1106_workqueue= create_singlethread_workqueue("cap1106_wq");
	if(!cap1106_workqueue) {
		PROX_ERROR("create_singlethread_workqueue failed!\n");
		err = -ENOMEM;
		goto exit_kfree;
    }
	/*......................................Driver prob port......................................*/
	
	g_cap1106_data_as = kzalloc(sizeof(struct cap1106_data), GFP_KERNEL);
	if (!g_cap1106_data_as)	{
		return -ENOMEM;
	}

   	// Check adapter supports everything 
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))	{
		goto exit_kfree;
	}

	//store i2c client structure
	g_cap1106_data_as->client = client;

	INIT_DELAYED_WORK(&g_cap1106_data_as->cap1106_work, cap1106_work_function);
	INIT_DELAYED_WORK(&g_cap1106_data_as->cap1106_checking_work, cap1106_checking_work_function);
	INIT_DELAYED_WORK(&cap_init_delay_work, cap1106_init_sensor);

	//Disable P01 attached temporarily for 1st ICS check-in
	register_microp_notifier(&capsensor_pad_mp_notifier);
	notify_register_microp_notifier(&capsensor_pad_mp_notifier, "cap1106");
	
	i2c_set_clientdata(client, g_cap1106_data_as);
	g_cap1106_data_as->client->flags = 0;
	strlcpy(g_cap1106_data_as->client->name, "cap1106", I2C_NAME_SIZE);
	g_cap1106_data_as->enable = 0;

//	handler = getProximityEventHandler("cap1106");
	
	mutex_init(&g_cap1106_data_as->lock);
	wake_lock_init(&alarm_cap_wake_lock, WAKE_LOCK_SUSPEND, "alarm_cap");

	PROX_DEBUG("[als_P01]++cap1106_probe: create_group\n");	

	/* register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &cap1106_attr_group);
	if (err){
		PROX_ERROR("Create the sysfs group failed!\n");
		goto err_register_switch_class_failed;
	}

	// registered as switch device
	err = switch_dev_register(&ls_switch_dev);
	if (err < 0){
		PROX_ERROR("Switch device registration failed!\n");
		goto err_register_switch_class_failed;
	}
	
	err = dev_info(&client->dev, "driver version %s enabled\n", DRIVER_VERSION);

	if (err)
		PROX_DEBUG("[cap_P92] ambientdl create sysfile fail.\n");

	PROX_DEBUG("[cap_P92]--cap1106_probe\n");

	PROX_DEBUG("+++ cap1106 g_cap1106_data_as->enable = %d\n", g_cap1106_data_as->enable);

#ifdef CONFIG_I2C_STRESS_TEST
       i2c_add_test_case(client, "Sensor_Cap1106",ARRAY_AND_SIZE(CapSensorTestInfo));
#endif

	g_cap1106_data_as->overflow_status = 0x0;
	return 0;

exit_kfree:
	kfree(g_cap1106_data_as);
	printk("[cap_P01]--cap1106_probe fail : %d\n", err);
	return err;
	
err_register_switch_class_failed:
	sysfs_remove_group(&g_cap1106_data_as->client->dev.kobj, &cap1106_attr_group);
	return err;
}

static int __devexit cap1106_remove(struct i2c_client *client)
{
    PROX_DEBUG("\n");
    destroy_workqueue(cap1106_workqueue);
    wake_lock_destroy(&alarm_cap_wake_lock);
    switch_dev_unregister(&ls_switch_dev);
    sysfs_remove_group(&client->dev.kobj, &cap1106_attr_group);
    kfree(g_cap1106_data_as);
	
    return 0;
}

static int cap1106_suspend(struct i2c_client *client, pm_message_t mesg)
{
	if(cap_status != inited_state){
		printk(DBGMSK_PRX_G2"!!! %s HWID = %d !!!\n", __FUNCTION__, HWID);
//		enter_state(suspending_state);
//		cap_status = 2;
		if(0 == HWID){
			return 0;
		}

	if(no_suspend){
		return EAGAIN;
	}
	cap1106_enable_sensor(client, 0);

	}
	return 0;
}

static int cap1106_resume(struct i2c_client *client)
{
	if(cap_status != inited_state){
		printk(DBGMSK_PRX_G2"!!! %s HWID = %d !!!\n", __FUNCTION__, HWID);
//		enter_state(running_state);
//		cap_status = 3;
		if(0 == HWID){
			printk(DBGMSK_PRX_G2"%s cap_status = %d\n", __FUNCTION__, cap_status);
			queue_delayed_work(cap1106_workqueue, &g_cap1106_data_as->cap1106_work, 0);
			queue_delayed_work(cap1106_workqueue, &g_cap1106_data_as->cap1106_checking_work, 0);
			return 0;
		}
		PROX_DEBUG("+\n");
		if (force_enable)
			cap1106_enable_sensor(client, 1);
		PROX_DEBUG("-\n");
	}
	return 0;
}

static const struct i2c_device_id cap1106_id[] = {
	{ "cap1106", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, cap1106_id);
static struct of_device_id cap1106_match_table[] = {
	{ .compatible = "cap,cap1106",},
	{},
};

static struct i2c_driver cap1106_driver = {
	.driver = {
		.name	= CAP1106_DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = cap1106_match_table,
	},
	.suspend = cap1106_suspend,
	.resume = cap1106_resume,
	.probe	= cap1106_probe,
	.remove = cap1106_remove,
	.id_table = cap1106_id,
};

MODULE_AUTHOR("ASUS");
MODULE_DESCRIPTION("version v1.1");

module_i2c_driver(cap1106_driver);

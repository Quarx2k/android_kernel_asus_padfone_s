#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/spmi.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/i2c.h>
#include <linux/semaphore.h>
#include <linux/device.h>
#include <linux/syscalls.h>
#include <asm/uaccess.h>
#include <linux/of_device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/sched.h>
#include <linux/spinlock_types.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <../../../drivers/staging/android/timed_output.h>
#include <linux/hrtimer.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/clk.h>
#include <linux/wakelock.h>

//ASUS_BSP +++ freddy "[A91][vib][NA][Spec] added to vibrator after phone inserted into pad" 
#include <linux/microp_notify.h>
#include <linux/microp_notifier_controller.h>	
#include <linux/microp_api.h>
#include <linux/microp_pin_def.h>
//ASUS_BSP --- freddy "[A91][vib][NA][Spec] added to vibrator after phone inserted into pad"


//Define register addresses of LC898301AXA 
#define LC898301_waddr 0x92
#define LC898301_raddr 0x93
#define DRIVING_VOLTAGE_01 0x01
#define RESOFRQ_02 0x02
#define START_UP_03 0x03
#define BRAKE_04 0x04
#define DRIVE_TIMES_05 0x05
#define DRIVING_VOLTAGE2_06 0x06
#define BRAKING_VOLTAGE_07 0x07
#define DRIVER_OUTPUT_08 0x08
#define SELFTEST_09 0x09

#define ON_SEMI_VIB_DEFAULT_TIMEOUT 15000

bool g_LC898301_probe_ok = 0;

static int LC898301_init_register(void);
struct LC898301_vib {
    struct wake_lock wklock;
    struct pwm_device *pwm_dev;
    struct hrtimer vib_timer;
    struct timer_list timer;
    struct timed_output_dev timed_dev;
    struct work_struct work;	
    struct mutex lock;
    int state;	
    int timeout;
};

/* platform data for the on_semi vibrator drive ic */
struct LC898301_platform_data
{			
	int gpio_rst;	
	u32 gpio_rst_flags;
};

struct LC898301_data {
	struct LC898301_platform_data *PlatData;
	struct i2c_client *client;
	int	device_id;
	struct mutex i2c_lock;
};

struct LC898301_data *pLC898301_data = NULL;
static struct LC898301_vib pvib_dev;
struct i2c_client *gLC898301_client;

static int vib_mode=0;
static int g_vib_stopval = 0;
unsigned int debounce_rst_pin = 10000;
unsigned int deb_maxtime_rst_pin =7200000 ;
bool g_ResetPinHigh = false;
/*
LC898301_write_reg():	write 8 bits reg function
slave_addr:	SMBus address (7 bits)
cmd_reg   :	cmd register for programming 
write_val  :	the value will be written 
*/
int LC898301_write_reg(uint8_t slave_addr, uint8_t cmd_reg, uint8_t write_val)
{
	int ret = 0;

	gLC898301_client->addr = (slave_addr >> 1); //real SMBus address (8 bits)
	ret = i2c_smbus_write_byte_data(gLC898301_client, cmd_reg, write_val);
	if (ret < 0) {
		printk("%s: failed to write i2c addr=%x\n",
			__func__, slave_addr);			
	}	
	return ret;
}


/*
LC898301_read_reg():	read 8 bits reg function
slave_addr:	SMBus address (7 bits)
cmd_reg   :	cmd register for programming 
store_read_val  :	value be read will store here
*/
int LC898301_read_reg(uint8_t slave_addr, uint8_t cmd_reg, uint8_t *store_read_val)
{
	int ret = 0;
       

	gLC898301_client->addr = (slave_addr >> 1);
	ret = i2c_smbus_read_byte_data(gLC898301_client, cmd_reg);
	if (ret < 0) {
		printk("%s: failed to read i2c addr=%x\n",	__func__, slave_addr);		
	}
	
	*store_read_val = (uint8_t) ret;

	return 0;
}

static void vibrator_rst_low_timer(unsigned long _data)
{
	gpio_direction_output(pLC898301_data->PlatData->gpio_rst, 0);
	g_ResetPinHigh = false;
	printk("[On_semi_vibrator] gpio_rst_pin:Low\n");
}

static int LC898301_vib_set(struct LC898301_vib *vib, int on)
{
	int rc = 0;

	if (on)
	{
		rc = LC898301_write_reg(LC898301_waddr, SELFTEST_09, 0x02);//vibrator on

		if (rc < 0)
			return rc;
			
		printk("[vibrator] Turn on vibrator, timer= %d ms\n",g_vib_stopval);
		hrtimer_start(&vib->vib_timer,
			      ktime_set(g_vib_stopval / 1000, (g_vib_stopval % 1000) * 1000000),
			      HRTIMER_MODE_REL);		
	
	}
	else
	{

		rc = LC898301_write_reg(LC898301_waddr, SELFTEST_09, 0x00);//vibrator off
		printk("[vibrator] Turn off vibrator !\n");
		if (rc < 0)
			return rc;	
		mod_timer(&pvib_dev.timer,jiffies + msecs_to_jiffies(debounce_rst_pin));	
	}

	return rc;
}


static void LC898301_vib_update(struct work_struct *work)
{
	struct LC898301_vib *vib = container_of(work, struct LC898301_vib, work);
	LC898301_vib_set(vib, vib->state);
}

static void LC898301_vib_enable(struct timed_output_dev *dev, int value)
{
	int ret = 0;
	struct LC898301_vib *vib = container_of(dev, struct LC898301_vib, timed_dev);
	if( !g_ResetPinHigh )
	{
		gpio_direction_output(pLC898301_data->PlatData->gpio_rst, 1);
		g_ResetPinHigh = true;
		msleep(1);
		ret = LC898301_init_register();
		if(ret)
		{
			printk("[On_semi_vibrator] %s:LC898301_init_register_fail\n",__func__);
		}
	}
	mod_timer(&pvib_dev.timer,jiffies + msecs_to_jiffies(deb_maxtime_rst_pin));	
	mutex_lock(&vib->lock);
	hrtimer_cancel(&vib->vib_timer);
	vib->timeout = ON_SEMI_VIB_DEFAULT_TIMEOUT; // timeout of vibrator, in ms. Default 15000 ms
	if (value == 0)
		vib->state = 0;
	else {
		value = (value > vib->timeout ?
				 vib->timeout : value);
		vib->state = 1;

		g_vib_stopval = value;
	}
	mutex_unlock(&vib->lock);
	schedule_work(&vib->work);	
}

static int LC898301_vib_get_time(struct timed_output_dev *dev)
{
	struct LC898301_vib *vib = container_of(dev, struct LC898301_vib, timed_dev);

	if (hrtimer_active(&vib->vib_timer)) {
		ktime_t r = hrtimer_get_remaining(&vib->vib_timer);
		return (int)ktime_to_us(r);
	} else
		return 0;
}

static enum hrtimer_restart LC898301_vib_timer_func(struct hrtimer *timer)
{
	struct LC898301_vib *vib = container_of(timer, struct LC898301_vib, vib_timer);

	vib->state = 0;
	schedule_work(&vib->work);
	printk("[vibrator] turn_on_time_up\n");

	return HRTIMER_NORESTART;
}


static int LC898301_dump_all_register(void)
{
	unsigned char  my_read_value=0;

	LC898301_read_reg(LC898301_raddr, DRIVING_VOLTAGE_01, &my_read_value);
	printk("[On_semi_vibrator] DRIVING_VOLTAGE_01 register, reg-01h= 0x%02xh\n", my_read_value);

	LC898301_read_reg(LC898301_raddr, RESOFRQ_02, &my_read_value);
	printk("[On_semi_vibrator] RESOFRQ_02 register, reg-02h= 0x%02xh\n", my_read_value);

	LC898301_read_reg(LC898301_raddr, START_UP_03, &my_read_value);
	printk("[On_semi_vibrator] START_UP_03 register, reg-03h= 0x%02xh\n", my_read_value);

	LC898301_read_reg(LC898301_raddr, BRAKE_04, &my_read_value);
	printk("[On_semi_vibrator] BRAKE_04 register, reg-04h= 0x%02xh\n", my_read_value);

	LC898301_read_reg(LC898301_raddr, DRIVE_TIMES_05, &my_read_value);
	printk("[On_semi_vibrator] DRIVE_TIMES_05 register, reg-05h= 0x%02xh\n", my_read_value);

	LC898301_read_reg(LC898301_raddr, DRIVING_VOLTAGE2_06, &my_read_value);
	printk("[On_semi_vibrator] DRIVING_VOLTAGE2_06 register, reg-06h= 0x%02xh\n", my_read_value);	

	LC898301_read_reg(LC898301_raddr, BRAKING_VOLTAGE_07, &my_read_value);
	printk("[On_semi_vibrator] BRAKING_VOLTAGE_07 register, reg-07h= 0x%02xh\n", my_read_value);	

	LC898301_read_reg(LC898301_raddr, DRIVER_OUTPUT_08, &my_read_value);
	printk("[On_semi_vibrator] DRIVER_OUTPUT_08 register, reg-08h= 0x%02xh\n", my_read_value);

	LC898301_read_reg(LC898301_raddr, SELFTEST_09, &my_read_value);
	printk("[On_semi_vibrator] SELFTEST_09 register, reg-09h= 0x%02xh\n", my_read_value);	

	return 0;
}



//ASUS_BSP +++ freddy "[A91][vib][NA][Spec] added to vibrator after phone inserted into pad"
static int mp_event_report(struct notifier_block *this, unsigned long event, void *ptr)
{
	
        switch (event)
	{
		case P01_ADD:
		{
			printk("[%s] PAD ADD vibrator enable!!\n",__func__);
			LC898301_vib_enable(&pvib_dev.timed_dev,500);
			
			return NOTIFY_DONE;
		}
		case P01_REMOVE:
		{
			printk("[%s] PAD REMOVE vibrator !!\n",__func__);
			LC898301_vib_enable(&pvib_dev.timed_dev,0);
			
		}
		default:
			
			return NOTIFY_DONE;
        }
}
/*
static struct notifier_block mp_notifier = {
        .notifier_call = mp_event_report,
        .priority = VIBRATOR_MP_NOTIFY,
};
*/
//ASUS_BSP --- freddy "[A91][vib][NA][Spec] added to vibrator after phone inserted into pad"
void MyDP_notify_vibrator_padInsert(void){
	printk("%s++++++++++++++++++\n",__func__);
	if (g_LC898301_probe_ok) 
		mp_event_report(NULL,P01_ADD,NULL);
}
static int LC898301_init_register(void)
{
	int ret = 0;
//	printk("[On_semi_vibrator] init: 100 degree, 1.363 Vrms, from 225Hz \n");
//driving voltage 15/15
	ret = LC898301_write_reg(LC898301_waddr, DRIVING_VOLTAGE_01, 0x0F);
	if (ret < 0)
	{
		printk("[On_semi_vibrator]write DRIVING_VOLTAGE_01 fail \n");
		return ret;
	}
//Resonance freq. = 225Hz
	ret = LC898301_write_reg(LC898301_waddr, RESOFRQ_02, 0x0F);
	if (ret < 0)
	{
		printk("[On_semi_vibrator]write RESOFRQ_02 fail \n");
		return ret;
	}
//100 degree and 3v
	ret = LC898301_write_reg(LC898301_waddr, DRIVER_OUTPUT_08, 0x20);
	if (ret < 0)
	{
		printk("[On_semi_vibrator]write DRIVER_OUTPUT_08 fail \n");
		return ret;
	}
//	printk("[On_semi_vibrator] init: succeed\n");
	return 0;

}


static int on_semi_vibrator_suspend(struct device *dev)
{

	if(del_timer(&pvib_dev.timer))
	{
		gpio_direction_output(pLC898301_data->PlatData->gpio_rst, 0);
		g_ResetPinHigh = false;
		printk("[On_semi_vibrator] suspend_gpio_rst_pin:Low\n");
	}

	return 0;
}

static SIMPLE_DEV_PM_OPS(on_semi_vibrator_pm_ops, on_semi_vibrator_suspend, NULL);

static int vib_test_function(const char *val, struct kernel_param *kp)
{
	int ret=0;
	int old_val = vib_mode;
//	unsigned char  my_read_value=0;

// ASUS BSP freddy+++ [A91][vib][spec][Others] "Change new vibrator driving IC after A91_SR5"
	if (g_ASUS_hwID < A91_SR5) 
	{
		printk("[On_semi_vibrator] g_ASUS_hwID=%d hwID_dont_support\n",g_ASUS_hwID);
		return 0;
	}
// ASUS BSP freddy+++ [A91][vib][spec][Others] "Change new vibrator driving IC after A91_SR5"	

	if (ret)
		return ret;

	if (vib_mode > 0xf) 
	{
		vib_mode = old_val;
		return -EINVAL;
	}

	ret = param_set_int(val, kp);

	if( !g_ResetPinHigh )
	{
		gpio_direction_output(pLC898301_data->PlatData->gpio_rst, 0);
		g_ResetPinHigh = false;
		msleep(1);
		gpio_direction_output(pLC898301_data->PlatData->gpio_rst, 1);
		g_ResetPinHigh = true;
		msleep(1);
	}

	if (vib_mode == 0)
	{
	
		printk("[On_semi_vibrator] set 100 degree & 1.363 Vrms\n");
	
		//driving voltage 15/15
		ret = LC898301_write_reg(LC898301_waddr, DRIVING_VOLTAGE_01, 0x0F);
		if (ret < 0)
			printk("[On_semi_vibrator] write DRIVING_VOLTAGE_01 fail \n");

		//Resonance freq. = 225Hz
		ret = LC898301_write_reg(LC898301_waddr, RESOFRQ_02, 0x0F);
		if (ret < 0)
			printk("[On_semi_vibrator] write RESOFRQ_02 fail \n");

		//100 degree and 3v
		ret = LC898301_write_reg(LC898301_waddr, DRIVER_OUTPUT_08, 0x20);
		if (ret < 0)
			printk("[On_semi_vibrator] write DRIVER_OUTPUT_08 fail \n");
				
	}
	else if (vib_mode == 1)
	{
		printk("[On_semi_vibrator] set 170 degree & 1.305 Vrms\n");
		
		//driving voltage 8/15
		ret = LC898301_write_reg(LC898301_waddr, DRIVING_VOLTAGE_01, 0x08);
		if (ret < 0)
			printk("[On_semi_vibrator] write DRIVING_VOLTAGE_01 fail \n");

		//Resonance freq. = 225Hz
		ret = LC898301_write_reg(LC898301_waddr, RESOFRQ_02, 0x0F);
		if (ret < 0)
			printk("[On_semi_vibrator] write RESOFRQ_02 fail \n");

		//170 degree and 3v
		ret = LC898301_write_reg(LC898301_waddr, DRIVER_OUTPUT_08, 0x24);
		if (ret < 0)
			printk("[On_semi_vibrator] write DRIVER_OUTPUT_08 fail \n");
	
	}
	else if (vib_mode == 2)
	{
		printk("[On_semi_vibrator] set 170 degree & 1.479 Vrms\n");
	
		//driving voltage 9/15
		ret = LC898301_write_reg(LC898301_waddr, DRIVING_VOLTAGE_01, 0x09);
		if (ret < 0)
			printk("[On_semi_vibrator] write DRIVING_VOLTAGE_01 fail \n");

		//Resonance freq. = 225Hz
		ret = LC898301_write_reg(LC898301_waddr, RESOFRQ_02, 0x0F);
		if (ret < 0)
			printk("[On_semi_vibrator] write RESOFRQ_02 fail \n");

		//170 degree and 3v
		ret = LC898301_write_reg(LC898301_waddr, DRIVER_OUTPUT_08, 0x24);
		if (ret < 0)
			printk("[On_semi_vibrator] write DRIVER_OUTPUT_08 fail \n");
	
	}
	else if (vib_mode == 3)
	{
		printk("[On_semi_vibrator] set 170 degree & 1.653 Vrms\n");
	
		//driving voltage 10/15
		ret = LC898301_write_reg(LC898301_waddr, DRIVING_VOLTAGE_01, 0x0A);
		if (ret < 0)
			printk("[On_semi_vibrator] write DRIVING_VOLTAGE_01 fail \n");

		//Resonance freq. = 225Hz
		ret = LC898301_write_reg(LC898301_waddr, RESOFRQ_02, 0x0F);
		if (ret < 0)
			printk("[On_semi_vibrator]write RESOFRQ_02 fail \n");

		//170 degree and 3v
		ret = LC898301_write_reg(LC898301_waddr, DRIVER_OUTPUT_08, 0x24);
		if (ret < 0)
			printk("[On_semi_vibrator] write DRIVER_OUTPUT_08 fail \n");
	}
	else if (vib_mode == 4)
	{
		printk("[On_semi_vibrator] set 170 degree & 1.827 Vrms\n");
	
		//driving voltage 11/15
		ret = LC898301_write_reg(LC898301_waddr, DRIVING_VOLTAGE_01, 0x0B);
		if (ret < 0)
			printk("[On_semi_vibrator] write DRIVING_VOLTAGE_01 fail \n");

		//Resonance freq. = 225Hz
		ret = LC898301_write_reg(LC898301_waddr, RESOFRQ_02, 0x0F);
		if (ret < 0)
			printk("[On_semi_vibrator] write RESOFRQ_02 fail \n");

		//170 degree and 3v
		ret = LC898301_write_reg(LC898301_waddr, DRIVER_OUTPUT_08, 0x24);
		if (ret < 0)
			printk("[On_semi_vibrator] write DRIVER_OUTPUT_08 fail \n");		
	}
	else if (vib_mode == 5)
	{
		printk("[On_semi_vibrator] set 170 degree & 2.001 Vrms\n");	
		//driving voltage 12/15
		ret = LC898301_write_reg(LC898301_waddr, DRIVING_VOLTAGE_01, 0x0C);
		if (ret < 0)
			printk("[On_semi_vibrator] write DRIVING_VOLTAGE_01 fail \n");

		//Resonance freq. = 225Hz
		ret = LC898301_write_reg(LC898301_waddr, RESOFRQ_02, 0x0F);
		if (ret < 0)
			printk("[On_semi_vibrator] write RESOFRQ_02 fail \n");

		//170 degree and 3v
		ret = LC898301_write_reg(LC898301_waddr, DRIVER_OUTPUT_08, 0x24);
		if (ret < 0)
			printk("[On_semi_vibrator] write DRIVER_OUTPUT_08 fail \n");	
	}
	else if (vib_mode == 11)
	{
		printk("[On_semi_vibrator] vibrator on \n");
		ret = LC898301_write_reg(LC898301_waddr, SELFTEST_09, 0x02);
		if (ret < 0)
			printk("[On_semi_vibrator]write DRIVER_OUTPUT_08 fail \n");	
	}
	else if (vib_mode == 10)
	{
		printk("[On_semi_vibrator] vibrator off \n");
		ret = LC898301_write_reg(LC898301_waddr, SELFTEST_09, 0x00);
		if (ret < 0)
			printk("[On_semi_vibrator] write SELFTEST_09 fail \n");	
	}
	
	printk("[On_semi_vibrator] LC898301_dump_all_register\n");
	LC898301_dump_all_register();
	return 0;
}


module_param_call(vib_mode, vib_test_function, param_get_int, &vib_mode, 0644);


/*
On_semi_vibrator works on 
A91_SR5,
after A91_ER(second source)
*/
static int LC898301_probe(struct i2c_client *client, const struct i2c_device_id *id)
{		
	
	int ret = 0, ini_ret = 0;
	
// ASUS BSP freddy+++ [A91][vib][spec][Others] "Change new vibrator driving IC after A91_SR5"
	if (g_ASUS_hwID < A91_SR5) 
	{
		printk("[On_semi_vibrator] g_ASUS_hwID=%d hwID_dont_support\n",g_ASUS_hwID);
	//ASUS_BSP +++ freddy"[A91][vib][NA][Spec]  for vibrator second source after A91_ER"	
		g_LC898301_probe_ok = 0; //match qpnp-vibrator.c 
		printk("[On_semi_vibrator] g_LC898301_probe_ok = 0\n");
	//ASUS_BSP --- freddy"[A91][vib][NA][Spec]  for vibrator second source after A91_ER"	
		return 0;
	}
// ASUS BSP freddy+++ [A91][vib][spec][Others] "Change new vibrator driving IC after A91_SR5"	

	printk("[On_semi_vibrator] -----%s:start----- \n",__func__);

	//i2c_check_functionality: Return 1 if adapter supports everything we need, 0 if not.
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk("[On_semi_vibrator]%s: i2c bus does not support the LC898301\n", __func__);
		ret = -ENODEV;
		goto exit;
	}
	
	pLC898301_data = kzalloc(sizeof(struct LC898301_data), GFP_KERNEL);
	if (!pLC898301_data) {
		printk("[On_semi_vibrator]%s: failed to allocate driver data\n", __func__);
		ret = -ENOMEM;
		goto exit;
	}
		
	pLC898301_data->PlatData = kzalloc(sizeof(struct LC898301_platform_data), GFP_KERNEL);
	
	if(!pLC898301_data->PlatData){
		printk("[On_semi_vibrator]%s: failed to allocate platform data \n", __func__);
		ret = -EINVAL;
		goto out_failed;
	}

	
	//to get gpio information +++
	if(client->dev.of_node)
	{
		pLC898301_data->PlatData->gpio_rst = of_get_named_gpio_flags(client->dev.of_node, 
				"on_semi,gpio_rst",0, &pLC898301_data->PlatData->gpio_rst_flags);
		
		printk("[On_semi_vibrator]: get GPIO information done \n");
	}
	
	printk("[On_semi_vibrator]gpio_rst[%d], Slave address[0x%02xh]\n",
			pLC898301_data->PlatData->gpio_rst,						
			client->addr);		


	pLC898301_data->client = client;
	gLC898301_client = client;
	//to get gpio information ---


	mutex_init(&pLC898301_data->i2c_lock);
	
	
	//int RST gpio +++	
	ret = gpio_request(pLC898301_data->PlatData->gpio_rst, "on_semi_pLC898301_gpio_rst");
	
	if (ret) 
	{
		printk("%s : failed to request gpio %d\n", __func__,
				pLC898301_data->PlatData->gpio_rst);
		goto err1;
	}
	
	gpio_direction_output(pLC898301_data->PlatData->gpio_rst, 0);
	gpio_direction_output(pLC898301_data->PlatData->gpio_rst, 1);
	g_ResetPinHigh = true;
	msleep(1);
	//int RST gpio  ---
	
	i2c_set_clientdata(client, pLC898301_data);

//init register setting +++
	ini_ret = LC898301_init_register();
	if(ini_ret)
	{
		printk("[On_semi_vibrator] %s:LC898301_init_register_fail\n",__func__);
		goto out_failed;
	}
//init register setting ---
/*
	printk("[On_semi_vibrator] %s: LC898301_dump_all_register+++\n",__func__);
	LC898301_dump_all_register();
	printk("[On_semi_vibrator] %s: LC898301_dump_all_register---\n",__func__);
*/
	setup_timer(&pvib_dev.timer, vibrator_rst_low_timer, (unsigned long)(&pvib_dev));

	gpio_direction_output(pLC898301_data->PlatData->gpio_rst, 0);//reset pin when vibrator off
	g_ResetPinHigh = false;

	pvib_dev.timed_dev.name = "vibrator";
	pvib_dev.timed_dev.get_time = LC898301_vib_get_time;
	pvib_dev.timed_dev.enable = LC898301_vib_enable;

      if (timed_output_dev_register(&(pvib_dev.timed_dev)) < 0)
      {
      		printk("[On_semi_vibrator]: fail to create timed output dev\n");
      		goto out_failed;
      }

	hrtimer_init(&pvib_dev.vib_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
      pvib_dev.vib_timer.function = LC898301_vib_timer_func;
	INIT_WORK(&pvib_dev.work, LC898301_vib_update);
      wake_lock_init(&pvib_dev.wklock, WAKE_LOCK_SUSPEND, "vibrator");
      mutex_init(&pvib_dev.lock);

//ASUS_BSP +++ freddy "[A91][vib][NA][Spec] added to vibrator after phone inserted into pad"
//	register_microp_notifier(&mp_notifier);  
//	notify_register_microp_notifier(&mp_notifier, "qpnp_vibrator"); 
//ASUS_BSP --- freddy "[A91][vib][NA][Spec] added to vibrator after phone inserted into pad"


	
	goto exit;


err1:
	gpio_free( pLC898301_data->PlatData->gpio_rst);

exit:	
//ASUS_BSP +++ freddy"[A91][vib][NA][Spec]  for vibrator second source after A91_ER"	
	g_LC898301_probe_ok = 1;
	printk("[On_semi_vibrator] g_LC898301_probe_ok = 1\n");
//ASUS_BSP --- freddy"[A91][vib][NA][Spec]  for vibrator second source after A91_ER"
	printk("[On_semi_vibrator] -------exit LC898301_probe -------\n");
	printk("\n");

	return ret;

	
out_failed:
//ASUS_BSP +++ freddy"[A91][vib][NA][Spec]  for vibrator second source after A91_ER"	
	printk("[On_semi_vibrator] goto out_failed\n");
	g_LC898301_probe_ok = 0;
	printk("[On_semi_vibrator] g_LC898301_probe_ok = 0\n");
//ASUS_BSP --- freddy"[A91][vib][NA][Spec]  for vibrator second source after A91_ER"
	kfree(pLC898301_data);
	return ret;	
}



static struct of_device_id LC898301_match_table[] = {
	{ .compatible = "ON_Semi,LC898301"},
	{ },
};


static const struct i2c_device_id LC898301_idtable[] = {
	{ "ON_Semi,LC898301", 0 },
	{ }
};

static struct i2c_driver LC898301_driver = {
	.driver = {
		.name	= "ON_Semi,LC898301",
		.owner	= THIS_MODULE,
		.of_match_table = LC898301_match_table,
		.pm	= &on_semi_vibrator_pm_ops,
	},
	.id_table	= LC898301_idtable,
	.probe		= LC898301_probe	
};

module_i2c_driver(LC898301_driver);
MODULE_DESCRIPTION("on_semi I2C Linear Vibrator Driver IC");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Freddy <Freddy_Ke@asus.com>");

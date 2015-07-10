/*
 * Driver for keys on GPIO lines capable of generating interrupts.
 *
 * Copyright 2005 Phil Blundell
 * Copyright 2010, 2011 David Jander <david@protonic.nl>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sysctl.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/spinlock.h>
//ASUS BSP freddy++ register wakelock for press key-code
#include <linux/kernel.h>
#include <linux/wakelock.h>
//ASUS BSP freddy-- register wakelock for press key-code
//ASUS BSP freddy++ for pad Key porting
#include <linux/microp_notify.h>
#include <linux/microp_notifier_controller.h>
#include <linux/microp_api.h>
#include <linux/microp_pin_def.h>
//ASUS BSP freddy-- for pad Key porting
#include <asm/uaccess.h>
#include <linux/of_irq.h>
static struct input_dev *g_input_dev; //ASUS BSP freddy+ get input_dev info for PAD-report keyevent and PAD-P01_debug
//ASUS BSP freddy++ define keyevent keycode
#define POWER_KEY_TEST 30 // for P01_debug
#define PAD_KEY_VOLUP   417
#define PAD_KEY_VOLDOWN 416
#define PAD_KEY_POWER   418
//ASUS BSP freddy-- define keyevent keycode

//ASUS_BSP + [ASDF]long press power key 6sec,reset device.. ++
#include <linux/reboot.h>
#include <asm/cacheflush.h>

extern void resetdevice(void);
//ASUS_BSP + [ASDF]long press power key 6sec,reset device.. --
//freddy +++ for key porting
static struct wake_lock pwr_key_wake_lock;
static bool g_bResume=1,g_bpwr_key_lock_sts=0;
static bool pad_volUp_still_press = 0 ,pad_volDown_still_press = 0;
static int g_keycheck_abort = 0, g_keystate = 0; //set tag for abort method
//freddy --- for key porting
static int vol_up_gpio;
static int vol_down_gpio;
static int pwr_gpio;
//ASUS BSP freddy++ for pad key porting
struct pad_buttons_code {
    int vol_up;
    int vol_down;
    int power_key;
};
//ASUS BSP freddy-- for pad key porting
struct gpio_button_data {
	struct gpio_keys_button *button; //ASUS BSP freddy+ remove const
	struct input_dev *input;
	struct timer_list timer;
	struct work_struct work;
	unsigned int timer_debounce;	/* in msecs */
	unsigned int irq;
	spinlock_t lock;
	bool disabled;
	bool key_pressed;
};

struct gpio_keys_drvdata {
	struct input_dev *input;
	struct mutex disable_lock;
	unsigned int n_buttons;
	int (*enable)(struct device *dev);
	void (*disable)(struct device *dev);
	struct pad_buttons_code pad_button_code;  //ASUS BSP freddy+ for pad key porting
	struct gpio_button_data data[3];  //ASUS BSP freddy+ change data[0]-> data[3]
};
//ASUS_BSP + [ASDF]long press power key 6sec,reset device.. ++
#include <asm/uaccess.h>
#include <linux/fs.h>
#define DEV_VIBRATOR "/sys/class/timed_output/vibrator/enable"
 void set_vib_enable(int value)
{
	char timeout_ms[5];
	static mm_segment_t oldfs;
	struct file *fp = NULL;
	loff_t pos_lsts = 0;

	sprintf(timeout_ms, "%d", value);
	oldfs = get_fs();
	set_fs(KERNEL_DS);
	fp = filp_open( DEV_VIBRATOR, O_RDWR|O_CREAT|O_TRUNC, 0664 );
	if(IS_ERR_OR_NULL(fp)) {
		printk("ASDF: fail to open vibrator.");
		return;
	}
	if(fp->f_op != NULL && fp->f_op->write != NULL){
		pos_lsts = 0;
		fp->f_op->write(fp, timeout_ms, strlen(timeout_ms), &pos_lsts);
	} else {
		printk("ASDF: fail to write value.\n");
	}
	filp_close(fp, NULL);
	set_fs(oldfs);
	printk("ASDF: set vibrator enable. (%s ms)\n", timeout_ms);
}
//check power 6sec enter sw reset
#define TIMEOUT_COUNT 55
static  struct work_struct __wait_for_power_key_6s_work;
static unsigned long press_time;
static int slow_ok;
void wait_for_power_key_6s_work(struct work_struct *work)
{
	static int power_key_6s_running = 0;
	int i, power_key, duration;
	unsigned long timeout, startime;
	power_key = pwr_gpio;
    if(!power_key_6s_running)
	{
		if (gpio_get_value_cansleep(power_key) != 0)
		{
			return;
		}
		power_key_6s_running = 1;
		startime = press_time;
		timeout = startime + HZ*TIMEOUT_COUNT/10;
		for(i = 0, slow_ok = 0; i < TIMEOUT_COUNT && slow_ok == 0 && time_before(jiffies, timeout) ; i++)
		{
			if (gpio_get_value_cansleep(power_key) == 0)
			{
				msleep(100);
			}
			else
			{
				break;
			}
		}
		if(((i == TIMEOUT_COUNT) || (slow_ok == 1) || time_after_eq(jiffies, timeout)) && (gpio_get_value_cansleep(power_key) == 0) && (i > 0))
		{
			duration = (jiffies - startime)*10/HZ;
			set_vib_enable(200);
			msleep(200);

			flush_cache_all();
			printk("force reset device!!\n");
			kernel_restart("asdf");
		}

		power_key_6s_running = 0;
	}
}
//
//ASUS_BSP + [ASDF]long press power key 6sec,reset device.. --

#define TIMEOUT_SLOW 30
static  struct work_struct __wait_for_slowlog_work;
void wait_for_slowlog_work(struct work_struct *work)
{
    static int one_slowlog_instance_running = 0;
    int i, volume_up_key, power_key, duration;
    unsigned long timeout, startime;

    volume_up_key = vol_up_gpio;
    power_key = pwr_gpio;
    if(!one_slowlog_instance_running)
    {
        if(gpio_get_value_cansleep(power_key) != 0)
        {
            return;
        }
        one_slowlog_instance_running = 1;
	startime = press_time;
	timeout = startime + HZ*TIMEOUT_SLOW/10;
	slow_ok=0;
			for(i = 0; i < TIMEOUT_SLOW && time_before(jiffies, timeout); i++)
			{
				if( gpio_get_value_cansleep(power_key) == 0)
				{
					msleep(100);
				}
				else
					break;
			}
			if(((i == TIMEOUT_SLOW) || time_after_eq(jiffies, timeout)) && (gpio_get_value_cansleep(power_key) == 0) && (i > 0))
			{
				printk("start to gi chk\n");
				duration = (jiffies - startime)*10/HZ;
				printk("start to gi chk after power press %d.%d sec (%d)\n", duration/10, duration%10, i);

				msleep(1 * 1000);

				duration = (jiffies - startime)*10/HZ;
				printk("start to gi delta after power press %d.%d sec (%d)\n", duration/10, duration%10, i);
				//Dump_wcd9310_reg();     //Bruno++
				//printk_lcd("slow log captured\n");
				slow_ok = 1;
			}
		one_slowlog_instance_running = 0;
	}

}

/*
 * SYSFS interface for enabling/disabling keys and switches:
 *
 * There are 4 attributes under /sys/devices/platform/gpio-keys/
 *	keys [ro]              - bitmap of keys (EV_KEY) which can be
 *	                         disabled
 *	switches [ro]          - bitmap of switches (EV_SW) which can be
 *	                         disabled
 *	disabled_keys [rw]     - bitmap of keys currently disabled
 *	disabled_switches [rw] - bitmap of switches currently disabled
 *
 * Userland can change these values and hence disable event generation
 * for each key (or switch). Disabling a key means its interrupt line
 * is disabled.
 *
 * For example, if we have following switches set up as gpio-keys:
 *	SW_DOCK = 5
 *	SW_CAMERA_LENS_COVER = 9
 *	SW_KEYPAD_SLIDE = 10
 *	SW_FRONT_PROXIMITY = 11
 * This is read from switches:
 *	11-9,5
 * Next we want to disable proximity (11) and dock (5), we write:
 *	11,5
 * to file disabled_switches. Now proximity and dock IRQs are disabled.
 * This can be verified by reading the file disabled_switches:
 *	11,5
 * If we now want to enable proximity (11) switch we write:
 *	5
 * to disabled_switches.
 *
 * We can disable only those keys which don't allow sharing the irq.
 */

/**
 * get_n_events_by_type() - returns maximum number of events per @type
 * @type: type of button (%EV_KEY, %EV_SW)
 *
 * Return value of this function can be used to allocate bitmap
 * large enough to hold all bits for given type.
 */
static inline int get_n_events_by_type(int type)
{
	BUG_ON(type != EV_SW && type != EV_KEY);

	return (type == EV_KEY) ? KEY_CNT : SW_CNT;
}

/**
 * gpio_keys_disable_button() - disables given GPIO button
 * @bdata: button data for button to be disabled
 *
 * Disables button pointed by @bdata. This is done by masking
 * IRQ line. After this function is called, button won't generate
 * input events anymore. Note that one can only disable buttons
 * that don't share IRQs.
 *
 * Make sure that @bdata->disable_lock is locked when entering
 * this function to avoid races when concurrent threads are
 * disabling buttons at the same time.
 */
static void gpio_keys_disable_button(struct gpio_button_data *bdata)
{
	if (!bdata->disabled) {
		/*
		 * Disable IRQ and possible debouncing timer.
		 */
		disable_irq(bdata->irq);
		if (bdata->timer_debounce)
			del_timer_sync(&bdata->timer);

		bdata->disabled = true;
	}
}

/**
 * gpio_keys_enable_button() - enables given GPIO button
 * @bdata: button data for button to be disabled
 *
 * Enables given button pointed by @bdata.
 *
 * Make sure that @bdata->disable_lock is locked when entering
 * this function to avoid races with concurrent threads trying
 * to enable the same button at the same time.
 */
static void gpio_keys_enable_button(struct gpio_button_data *bdata)
{
	if (bdata->disabled) {
		enable_irq(bdata->irq);
		bdata->disabled = false;
	}
}

/**
 * gpio_keys_attr_show_helper() - fill in stringified bitmap of buttons
 * @ddata: pointer to drvdata
 * @buf: buffer where stringified bitmap is written
 * @type: button type (%EV_KEY, %EV_SW)
 * @only_disabled: does caller want only those buttons that are
 *                 currently disabled or all buttons that can be
 *                 disabled
 *
 * This function writes buttons that can be disabled to @buf. If
 * @only_disabled is true, then @buf contains only those buttons
 * that are currently disabled. Returns 0 on success or negative
 * errno on failure.
 */
static ssize_t gpio_keys_attr_show_helper(struct gpio_keys_drvdata *ddata,
					  char *buf, unsigned int type,
					  bool only_disabled)
{
	int n_events = get_n_events_by_type(type);
	unsigned long *bits;
	ssize_t ret;
	int i;

	bits = kcalloc(BITS_TO_LONGS(n_events), sizeof(*bits), GFP_KERNEL);
	if (!bits)
		return -ENOMEM;

	for (i = 0; i < ddata->n_buttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];

		if (bdata->button->type != type)
			continue;

		if (only_disabled && !bdata->disabled)
			continue;

		__set_bit(bdata->button->code, bits);
	}

	ret = bitmap_scnlistprintf(buf, PAGE_SIZE - 2, bits, n_events);
	buf[ret++] = '\n';
	buf[ret] = '\0';

	kfree(bits);

	return ret;
}

/**
 * gpio_keys_attr_store_helper() - enable/disable buttons based on given bitmap
 * @ddata: pointer to drvdata
 * @buf: buffer from userspace that contains stringified bitmap
 * @type: button type (%EV_KEY, %EV_SW)
 *
 * This function parses stringified bitmap from @buf and disables/enables
 * GPIO buttons accordingly. Returns 0 on success and negative error
 * on failure.
 */
static ssize_t gpio_keys_attr_store_helper(struct gpio_keys_drvdata *ddata,
					   const char *buf, unsigned int type)
{
	int n_events = get_n_events_by_type(type);
	unsigned long *bits;
	ssize_t error;
	int i;

	bits = kcalloc(BITS_TO_LONGS(n_events), sizeof(*bits), GFP_KERNEL);
	if (!bits)
		return -ENOMEM;

	error = bitmap_parselist(buf, bits, n_events);
	if (error)
		goto out;

	/* First validate */
	for (i = 0; i < ddata->n_buttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];

		if (bdata->button->type != type)
			continue;

		if (test_bit(bdata->button->code, bits) &&
		    !bdata->button->can_disable) {
			error = -EINVAL;
			goto out;
		}
	}

	mutex_lock(&ddata->disable_lock);

	for (i = 0; i < ddata->n_buttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];

		if (bdata->button->type != type)
			continue;

		if (test_bit(bdata->button->code, bits))
			gpio_keys_disable_button(bdata);
		else
			gpio_keys_enable_button(bdata);
	}

	mutex_unlock(&ddata->disable_lock);

out:
	kfree(bits);
	return error;
}

#define ATTR_SHOW_FN(name, type, only_disabled)				\
static ssize_t gpio_keys_show_##name(struct device *dev,		\
				     struct device_attribute *attr,	\
				     char *buf)				\
{									\
	struct platform_device *pdev = to_platform_device(dev);		\
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);	\
									\
	return gpio_keys_attr_show_helper(ddata, buf,			\
					  type, only_disabled);		\
}

ATTR_SHOW_FN(keys, EV_KEY, false);
ATTR_SHOW_FN(switches, EV_SW, false);
ATTR_SHOW_FN(disabled_keys, EV_KEY, true);
ATTR_SHOW_FN(disabled_switches, EV_SW, true);

/*
 * ATTRIBUTES:
 *
 * /sys/devices/platform/gpio-keys/keys [ro]
 * /sys/devices/platform/gpio-keys/switches [ro]
 */
static DEVICE_ATTR(keys, S_IRUGO, gpio_keys_show_keys, NULL);
static DEVICE_ATTR(switches, S_IRUGO, gpio_keys_show_switches, NULL);

#define ATTR_STORE_FN(name, type)					\
static ssize_t gpio_keys_store_##name(struct device *dev,		\
				      struct device_attribute *attr,	\
				      const char *buf,			\
				      size_t count)			\
{									\
	struct platform_device *pdev = to_platform_device(dev);		\
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);	\
	ssize_t error;							\
									\
	error = gpio_keys_attr_store_helper(ddata, buf, type);		\
	if (error)							\
		return error;						\
									\
	return count;							\
}

ATTR_STORE_FN(disabled_keys, EV_KEY);
ATTR_STORE_FN(disabled_switches, EV_SW);

/*
 * ATTRIBUTES:
 *
 * /sys/devices/platform/gpio-keys/disabled_keys [rw]
 * /sys/devices/platform/gpio-keys/disables_switches [rw]
 */
static DEVICE_ATTR(disabled_keys, S_IWUSR | S_IRUGO,
		   gpio_keys_show_disabled_keys,
		   gpio_keys_store_disabled_keys);
static DEVICE_ATTR(disabled_switches, S_IWUSR | S_IRUGO,
		   gpio_keys_show_disabled_switches,
		   gpio_keys_store_disabled_switches);

static struct attribute *gpio_keys_attrs[] = {
	&dev_attr_keys.attr,
	&dev_attr_switches.attr,
	&dev_attr_disabled_keys.attr,
	&dev_attr_disabled_switches.attr,
	NULL,
};

static struct attribute_group gpio_keys_attr_group = {
	.attrs = gpio_keys_attrs,
};


static struct work_struct __dump_log_work;
#include <linux/kmod.h>
void dump_log_work(struct work_struct *work)
{
	int i = 2;

	char *argv[] = { "/system/bin/dump_log", NULL };
	char *envp[] = { "HOME=/" , NULL };


	for(; i > 0; i--) {
		set_vib_enable(500);
		msleep(1000);
	}

	call_usermodehelper(argv[0], argv, envp, UMH_WAIT_PROC);

}

static unsigned int b_press;

static void gpio_keys_gpio_report_event(struct gpio_button_data *bdata)
{
	const struct gpio_keys_button *button = bdata->button;
	struct input_dev *input = bdata->input;
	unsigned int type = button->type ?: EV_KEY;
	int state = (gpio_get_value_cansleep(button->gpio) ? 1 : 0) ^ button->active_low;
	int volume_up_key, volume_down_key, power_key;//ASUS_BSP + [ASDF]long press power key 6sec,reset device..

	volume_up_key = vol_up_gpio;
	volume_down_key = vol_down_gpio;
	power_key = pwr_gpio;//ASUS_BSP + [ASDF]long press power key 6sec,reset device..
//ASUS BSP freddy+++ fix TT 281235:Power key suspend/resume fail
	if ((g_keystate != state) && (!state))
	{
		printk("[Gpio_keys] %s: short_pressed case \n",__func__);
		state = g_keystate;
	}
//ASUS BSP freddy--- fix TT 281235:Power key suspend/resume fail
	printk("[Gpio_keys] %s:key code=%d  state=%s \n",__func__,
		button->code,state ? "press" : "release");  //ASUS BSP freddy+ add log for report keycode function.
//ASUS_BSP + [ASDF]long press power key 6sec,reset device.. ++
	if (gpio_get_value_cansleep(power_key) == 0)
	{
		schedule_work(&__wait_for_power_key_6s_work);
	}
//ASUS_BSP + [ASDF]long press power key 6sec,reset device.. --
	//printk("GPIO_%d=%d , GPIO_%d=%d , GPIO_%d=%d\n", vol_up_gpio, gpio_get_value_cansleep(vol_up_gpio), vol_down_gpio, gpio_get_value_cansleep(vol_down_gpio), pwr_gpio, gpio_get_value_cansleep(pwr_gpio));


	if (type == EV_ABS) {
		if (state)
			input_event(input, type, button->code, button->value);
	} else {
		input_event(input, type, button->code, !!state);

		if(state) {
			if(button->code == 114)
				b_press |= 0x01;

			if(button->code == 115)
				b_press |= 0x02;

			if(button->code == 116 && b_press == 3)
				schedule_work(&__dump_log_work);
		}
		else {
			if(button->code == 114)
				b_press &= ~(0x01);

			if(button->code == 115)
				b_press &= ~(0x02);
		}
	}
	input_sync(input);
}

static void gpio_keys_gpio_work_func(struct work_struct *work)
{
	struct gpio_button_data *bdata =
		container_of(work, struct gpio_button_data, work);
//ASUS_BSP + [ASDF]long press power key 6sec,reset device.. ++
	if (gpio_get_value_cansleep(pwr_gpio) == 0){
		press_time = jiffies;
	} else {
		press_time = 0xFFFFFFFF;
	}
//ASUS_BSP + [ASDF]long press power key 6sec,reset device.. --

	gpio_keys_gpio_report_event(bdata);
}

static void gpio_keys_gpio_timer(unsigned long _data)
{
	struct gpio_button_data *bdata = (struct gpio_button_data *)_data;
//ASUS BSP freddy+++ fix TT 281235:Power key suspend/resume fail
	const struct gpio_keys_button *button = bdata->button;
	g_keystate = (gpio_get_value(button->gpio) ? 1 : 0) ^ button->active_low;//memo button->active_low==1;
	printk("[Gpio_keys] after debounce,gpio=%d key state=%d \n",button->gpio, g_keystate);
//ASUS BSP freddy--- fix TT 281235:Power key suspend/resume fail
	schedule_work(&bdata->work);
}

static irqreturn_t gpio_keys_gpio_isr(int irq, void *dev_id)
{
	struct gpio_button_data *bdata = dev_id;
//ASUS BSP freddy+++ fix TT 281235:Power key suspend/resume fail
	const struct gpio_keys_button *button = bdata->button;
	int state = (gpio_get_value(button->gpio) ? 1 : 0) ^ button->active_low;//memo button->active_low==1;
	BUG_ON(irq != bdata->irq);
	g_keycheck_abort = 1;
//ASUS BSP freddy--- fix TT 281235:Power key suspend/resume fail
	if (bdata->timer_debounce)
		mod_timer(&bdata->timer,
			jiffies + msecs_to_jiffies(bdata->timer_debounce));
	else
		schedule_work(&bdata->work);
	printk("[Gpio_keys]debouncing,irq_number=%d key state=%d \n",irq, state);
	return IRQ_HANDLED;
}

static void gpio_keys_irq_timer(unsigned long _data)
{
	struct gpio_button_data *bdata = (struct gpio_button_data *)_data;
	struct input_dev *input = bdata->input;
	unsigned long flags;

	spin_lock_irqsave(&bdata->lock, flags);
	if (bdata->key_pressed) {
		input_event(input, EV_KEY, bdata->button->code, 0);
		input_sync(input);
		bdata->key_pressed = false;
	}
	spin_unlock_irqrestore(&bdata->lock, flags);
}

static irqreturn_t gpio_keys_irq_isr(int irq, void *dev_id)
{
	struct gpio_button_data *bdata = dev_id;
	const struct gpio_keys_button *button = bdata->button;
	struct input_dev *input = bdata->input;
	unsigned long flags;

	BUG_ON(irq != bdata->irq);

	spin_lock_irqsave(&bdata->lock, flags);

	if (!bdata->key_pressed) {
		input_event(input, EV_KEY, button->code, 1);
		input_sync(input);

		if (!bdata->timer_debounce) {
			input_event(input, EV_KEY, button->code, 0);
			input_sync(input);
			goto out;
		}

		bdata->key_pressed = true;
	}

	if (bdata->timer_debounce)
		mod_timer(&bdata->timer,
			jiffies + msecs_to_jiffies(bdata->timer_debounce));
out:
	spin_unlock_irqrestore(&bdata->lock, flags);
	return IRQ_HANDLED;
}

static int __devinit gpio_keys_setup_key(struct platform_device *pdev,
					 struct input_dev *input,
					 struct gpio_button_data *bdata,
					 struct gpio_keys_button *button) //ASUS BSP freddy+ remove const
{
	const char *desc = button->desc ? button->desc : "gpio_keys";
	struct device *dev = &pdev->dev;
	irq_handler_t isr;
	unsigned long irqflags;
	int irq, error;

	bdata->input = input;
	bdata->button = button;
	spin_lock_init(&bdata->lock);

	if (gpio_is_valid(button->gpio)) {

		error = gpio_request(button->gpio, desc);
		if (error < 0) {
			dev_err(dev, "Failed to request GPIO %d, error %d\n",
				button->gpio, error);
			return error;
		}

		error = gpio_direction_input(button->gpio);
		if (error < 0) {
			dev_err(dev,
				"Failed to configure direction for GPIO %d, error %d\n",
				button->gpio, error);
			goto fail;
		}

		if (button->debounce_interval) {
			error = gpio_set_debounce(button->gpio,
					button->debounce_interval * 1000);
			/* use timer if gpiolib doesn't provide debounce */
			if (error < 0)
				bdata->timer_debounce =
						button->debounce_interval;
		}

		irq = gpio_to_irq(button->gpio);
		if (irq < 0) {
			error = irq;
			dev_err(dev,
				"Unable to get irq number for GPIO %d, error %d\n",
				button->gpio, error);
			goto fail;
		}
		bdata->irq = irq;

		INIT_WORK(&bdata->work, gpio_keys_gpio_work_func);
		setup_timer(&bdata->timer,
			    gpio_keys_gpio_timer, (unsigned long)bdata);

		isr = gpio_keys_gpio_isr;
		irqflags = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING;

	} else {
		if (!button->irq) {
			dev_err(dev, "No IRQ specified\n");
			return -EINVAL;
		}
		bdata->irq = button->irq;

		if (button->type && button->type != EV_KEY) {
			dev_err(dev, "Only EV_KEY allowed for IRQ buttons.\n");
			return -EINVAL;
		}

		bdata->timer_debounce = button->debounce_interval;
		setup_timer(&bdata->timer,
			    gpio_keys_irq_timer, (unsigned long)bdata);

		isr = gpio_keys_irq_isr;
		irqflags = 0;
	}

	input_set_capability(input, button->type ?: EV_KEY, button->code);

	/*
	 * If platform has specified that the button can be disabled,
	 * we don't want it to share the interrupt line.
	 */
	if (!button->can_disable)
		irqflags |= IRQF_SHARED;

	error = request_any_context_irq(bdata->irq, isr, irqflags, desc, bdata);
	if (error < 0) {
		dev_err(dev, "Unable to claim irq %d; error %d\n",
			bdata->irq, error);
		goto fail;
	}

	return 0;

fail:
	if (gpio_is_valid(button->gpio))
		gpio_free(button->gpio);

	return error;
}

//ASUS BSP freddy++ for P01_debug :
	//(mode1=Debug Mode),(mode2=BACK/MENU Mode),(mode0=Normal Mode)
#ifdef  CONFIG_PROC_FS
#define debug_GPIO_KEY_PROC_FILE  "driver/P01_debug_key"
// "proc/driver/P01_debug_key"
static struct proc_dir_entry *debug_key_proc_file;
#include <linux/syscalls.h>
#include <linux/fs.h>
#include <linux/file.h>
static mm_segment_t oldfs;
static void initKernelEnv(void)
{
     oldfs = get_fs();
     set_fs(KERNEL_DS);
}
static void deinitKernelEnv(void)
{
     set_fs(oldfs);
}
static ssize_t debug_key_proc_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	char messages[256];
    struct gpio_keys_drvdata *ddata = input_get_drvdata(g_input_dev);
    int i;
    memset(messages, 0, sizeof(messages));
    if (len > 256)
    {
    	len = 256;
    }
    if (copy_from_user(messages, buff, len))
    {
    	return -EFAULT;
    }
    initKernelEnv();
	if(strncmp(messages, "1", 1) == 0)  // '1' is Debug Mode
    {
    	for (i=0;i<3;i++)
	    {
	        if (ddata->data[i].button->gpio  == pwr_gpio)
	            ddata->data[i].button->code = POWER_KEY_TEST;
	        if (ddata->data[i].button->gpio  == vol_up_gpio)
	            ddata->data[i].button->code = KEY_VOLUMEUP;
	        if (ddata->data[i].button->gpio  == vol_down_gpio)
	            ddata->data[i].button->code = KEY_VOLUMEDOWN;
	    }
        ddata->pad_button_code.power_key = POWER_KEY_TEST;
		ddata->pad_button_code.vol_up = PAD_KEY_VOLUP;
        ddata->pad_button_code.vol_down = PAD_KEY_VOLDOWN;
		printk("[Gpio_keys] Debug Mode!!!\n");
	}
    else if(strncmp(messages, "0", 1) == 0) // '0' is Normal Mode
    {
		for (i=0;i<3;i++)
        {
        	if (ddata->data[i].button->gpio  == pwr_gpio)
            ddata->data[i].button->code = KEY_POWER;
        	if (ddata->data[i].button->gpio  == vol_up_gpio)
            ddata->data[i].button->code = KEY_VOLUMEUP;
        	if (ddata->data[i].button->gpio  == vol_down_gpio)
            ddata->data[i].button->code = KEY_VOLUMEDOWN;
		}
	    ddata->pad_button_code.power_key = PAD_KEY_POWER;
	    ddata->pad_button_code.vol_up = PAD_KEY_VOLUP;
	    ddata->pad_button_code.vol_down = PAD_KEY_VOLDOWN;
	    printk("[Gpio_keys] Normal Mode!!!\n");
	}
    else if(strncmp(messages, "2", 1) == 0) // '2' is BACK/MENU Mode
    {

		for (i=0;i<3;i++)
        {
        	if (ddata->data[i].button->gpio  == pwr_gpio)
            ddata->data[i].button->code = KEY_POWER;
        	if (ddata->data[i].button->gpio  == vol_up_gpio)
            ddata->data[i].button->code = KEY_MENU;
        	if (ddata->data[i].button->gpio  == vol_down_gpio)
            ddata->data[i].button->code = KEY_BACK;
		}
	    ddata->pad_button_code.power_key = PAD_KEY_POWER;
	    ddata->pad_button_code.vol_up = KEY_MENU;
	    ddata->pad_button_code.vol_down = KEY_BACK;
		printk("[Gpio_keys] BACK/MENU Mode!!!\n");
	}
    for (i=0;i<3;i++) //show log to confirm the gpio pins have been changed mdoe.
    {
       printk("[Gpio_keys] code[%d] is GPIO %d \n", ddata->data[i].button->code, ddata->data[i].button->gpio);
    }
    deinitKernelEnv();
     return len;
}
static struct file_operations debug_key_proc_ops = {
	.write = debug_key_proc_write,
};
static void create_debug_key_proc_file(void)
{
	struct gpio_keys_drvdata *ddata = input_get_drvdata(g_input_dev);

    printk("[Gpio_keys] create_debug_key_proc_file.\n");
    debug_key_proc_file = create_proc_entry(debug_GPIO_KEY_PROC_FILE, 0666, NULL);
    if (debug_key_proc_file) {
    	debug_key_proc_file->proc_fops = &debug_key_proc_ops;
    }
    // init PAD keycode.
    ddata->pad_button_code.vol_up = PAD_KEY_VOLUP;
    ddata->pad_button_code.vol_down = PAD_KEY_VOLDOWN;
    ddata->pad_button_code.power_key = PAD_KEY_POWER;
}
static void remove_debug_key_proc_file(void)
{
	extern struct proc_dir_entry proc_root;
    printk("[Gpio_keys] remove_debug_key_proc_file.\n");
    remove_proc_entry(debug_GPIO_KEY_PROC_FILE, &proc_root);
}
#endif
static int gpio_keys_open(struct input_dev *input)
{
	struct gpio_keys_drvdata *ddata = input_get_drvdata(input);

	return ddata->enable ? ddata->enable(input->dev.parent) : 0;
}

static void gpio_keys_close(struct input_dev *input)
{
	struct gpio_keys_drvdata *ddata = input_get_drvdata(input);

	if (ddata->disable)
		ddata->disable(input->dev.parent);
}

/*
 * Handlers for alternative sources of platform_data
 */
#ifdef CONFIG_OF
/*
 * Translate OpenFirmware node properties into platform_data
 */
static int gpio_keys_get_devtree_pdata(struct device *dev,
			    struct gpio_keys_platform_data *pdata)
{
	struct device_node *node, *pp;
	int i;
	struct gpio_keys_button *buttons;
	u32 reg;

	node = dev->of_node;
	if (node == NULL)
		return -ENODEV;

	memset(pdata, 0, sizeof *pdata);

	pdata->rep = !!of_get_property(node, "autorepeat", NULL);
	pdata->name = of_get_property(node, "input-name", NULL);

	/* First count the subnodes */
	pdata->nbuttons = 0;
	pp = NULL;
	while ((pp = of_get_next_child(node, pp)))
		pdata->nbuttons++;

	if (pdata->nbuttons == 0)
		return -ENODEV;

	buttons = kzalloc(pdata->nbuttons * (sizeof *buttons), GFP_KERNEL);
	if (!buttons)
		return -ENOMEM;

	pp = NULL;
	i = 0;
	while ((pp = of_get_next_child(node, pp))) {
		enum of_gpio_flags flags;

		if (!of_find_property(pp, "gpios", NULL)) {
			pdata->nbuttons--;
			dev_warn(dev, "Found button without gpios\n");
			continue;
		}
		buttons[i].gpio = of_get_gpio_flags(pp, 0, &flags);
		buttons[i].active_low = !(flags & OF_GPIO_ACTIVE_LOW);  //ASUS BSP freddy+ inverse for let press key state==1

		if (of_property_read_u32(pp, "linux,code", &reg)) {
			dev_err(dev, "Button without keycode: 0x%x\n", buttons[i].gpio);
			goto out_fail;
		}
		buttons[i].code = reg;

		buttons[i].desc = of_get_property(pp, "label", NULL);

		if (of_property_read_u32(pp, "linux,input-type", &reg) == 0)
			buttons[i].type = reg;
		else
			buttons[i].type = EV_KEY;

		buttons[i].wakeup = !!of_get_property(pp, "gpio-key,wakeup", NULL);

		if (of_property_read_u32(pp, "debounce-interval", &reg) == 0)
			buttons[i].debounce_interval = reg;
		else
			buttons[i].debounce_interval = 5;

		i++;
	}

	pdata->buttons = buttons;

	return 0;

out_fail:
	kfree(buttons);
	return -ENODEV;
}

static struct of_device_id gpio_keys_of_match[] = {
	{ .compatible = "gpio-keys", },
	{ },
};
MODULE_DEVICE_TABLE(of, gpio_keys_of_match);

#else

static int gpio_keys_get_devtree_pdata(struct device *dev,
			    struct gpio_keys_platform_data *altp)
{
	return -ENODEV;
}

#define gpio_keys_of_match NULL

#endif

static void gpio_remove_key(struct gpio_button_data *bdata)
{
	free_irq(bdata->irq, bdata);
	if (bdata->timer_debounce)
		del_timer_sync(&bdata->timer);
	cancel_work_sync(&bdata->work);
	if (gpio_is_valid(bdata->button->gpio))
		gpio_free(bdata->button->gpio);
}

static int __devinit gpio_keys_probe(struct platform_device *pdev)
{
	struct gpio_keys_platform_data *pdata = pdev->dev.platform_data; //ASUS BSP freddy+ remove const
	struct gpio_keys_drvdata *ddata;
	struct device *dev = &pdev->dev;
	struct gpio_keys_platform_data alt_pdata;
	struct input_dev *input;
	int i, error;
	int wakeup = 0;
	INIT_WORK(&__dump_log_work, dump_log_work);
	INIT_WORK(&__wait_for_slowlog_work, wait_for_slowlog_work);
    INIT_WORK(&__wait_for_power_key_6s_work, wait_for_power_key_6s_work);//ASUS_BSP + [ASDF]long press power key 6sec,reset device..
//ASUS BSP freddy +++ for init fake key wakelock.
        wake_lock_init(&pwr_key_wake_lock, WAKE_LOCK_SUSPEND, "pwr_key_lock");
	 printk("[Progress][Gpio_keys] Probe starts\n");

//ASUS BSP freddy --- for finit fake key wakelock.

	if (!pdata) {
		error = gpio_keys_get_devtree_pdata(dev, &alt_pdata);
		if (error)
			return error;
		pdata = &alt_pdata;
	}

	ddata = kzalloc(sizeof(struct gpio_keys_drvdata) +
			pdata->nbuttons * sizeof(struct gpio_button_data),
			GFP_KERNEL);
	input = input_allocate_device();
	if (!ddata || !input) {
		dev_err(dev, "failed to allocate state\n");
		error = -ENOMEM;
		goto fail1;
	}

	ddata->input = input;
	ddata->n_buttons = pdata->nbuttons;
	ddata->enable = pdata->enable;
	ddata->disable = pdata->disable;
	mutex_init(&ddata->disable_lock);

	platform_set_drvdata(pdev, ddata);
	input_set_drvdata(input, ddata);

	input->name = pdata->name ? : pdev->name;
	input->phys = "gpio-keys/input0";
	input->dev.parent = &pdev->dev;
	input->open = gpio_keys_open;
	input->close = gpio_keys_close;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0001;
	input->id.version = 0x0100;
    g_input_dev = input;  //ASUS BSP freddy+ get input_dev info for PAD-report keyevent and PAD-P01_debug
	/* Enable auto repeat feature of Linux input subsystem */
	if (pdata->rep)
		__set_bit(EV_REP, input->evbit);

	for (i = 0; i < pdata->nbuttons; i++) {
		struct gpio_keys_button *button = &pdata->buttons[i]; //ASUS BSP freddy+ remover const
		struct gpio_button_data *bdata = &ddata->data[i];

		error = gpio_keys_setup_key(pdev, input, bdata, button);
		if (error)
			goto fail2;

		if (button->wakeup)
			wakeup = 1;
	}
//ASUS BSP freddy++
	input_set_capability(input, EV_KEY, PAD_KEY_VOLUP);
	input_set_capability(input, EV_KEY, PAD_KEY_VOLDOWN);
	input_set_capability(input, EV_KEY, PAD_KEY_POWER);
//ASUS BSP freddy--

	error = sysfs_create_group(&pdev->dev.kobj, &gpio_keys_attr_group);
	if (error) {
		dev_err(dev, "Unable to export keys/switches, error: %d\n",
			error);
		goto fail2;
	}

	error = input_register_device(input);
	if (error) {
		dev_err(dev, "Unable to register input device, error: %d\n",
			error);
		goto fail3;
	}

	/* get current state of buttons that are connected to GPIOs */
	for (i = 0; i < pdata->nbuttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];
		if (gpio_is_valid(bdata->button->gpio))
			gpio_keys_gpio_report_event(bdata);
	}
	input_sync(input);

	device_init_wakeup(&pdev->dev, wakeup);
	//ASUS BSP freddy++ for P01_debug
#ifdef  CONFIG_PROC_FS
        create_debug_key_proc_file();
        input_set_capability(input, EV_KEY, KEY_MENU);
        input_set_capability(input, EV_KEY, KEY_BACK);
        input_set_capability(input, EV_KEY, POWER_KEY_TEST);
#endif
	//ASUS BSP freddy-- for P01_debug

	 printk("[Progress][Gpio_keys] Probe ends\n");
	return 0;

 fail3:
	sysfs_remove_group(&pdev->dev.kobj, &gpio_keys_attr_group);
 fail2:
	while (--i >= 0)
		gpio_remove_key(&ddata->data[i]);

	platform_set_drvdata(pdev, NULL);
 fail1:
	input_free_device(input);
	kfree(ddata);
	/* If we have no platform_data, we allocated buttons dynamically. */
	if (!pdev->dev.platform_data)
		kfree(pdata->buttons);

	return error;
}

static int __devexit gpio_keys_remove(struct platform_device *pdev)
{
	struct gpio_keys_drvdata *ddata = platform_get_drvdata(pdev);
	struct input_dev *input = ddata->input;
	int i;

	sysfs_remove_group(&pdev->dev.kobj, &gpio_keys_attr_group);

	device_init_wakeup(&pdev->dev, 0);

	for (i = 0; i < ddata->n_buttons; i++)
		gpio_remove_key(&ddata->data[i]);

	input_unregister_device(input);
//ASUS BSP freddy++ for P01_debug
#ifdef  CONFIG_PROC_FS
    remove_debug_key_proc_file();
#endif
//ASUS BSP freddy-- for P01_debug

	/*
	 * If we had no platform_data, we allocated buttons dynamically, and
	 * must free them here. ddata->data[0].button is the pointer to the
	 * beginning of the allocated array.
	 */
	if (!pdev->dev.platform_data)
		kfree(ddata->data[0].button);

	kfree(ddata);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int gpio_keys_suspend(struct device *dev)
{
	struct gpio_keys_drvdata *ddata = dev_get_drvdata(dev);
	int i;
//ASUS BSP freddy+++ fix TT 281235:Power key suspend/resume fail
	g_keycheck_abort = 0;
	printk("[Gpio_keys] clear g_keycheck_abort = %d\n",g_keycheck_abort);
//ASUS BSP freddy--- fix TT 281235:Power key suspend/resume fail
	if (device_may_wakeup(dev)) {
		for (i = 0; i < ddata->n_buttons; i++) {
			struct gpio_button_data *bdata = &ddata->data[i];
			if (bdata->button->wakeup)
				enable_irq_wake(bdata->irq);
		}
	}

	return 0;
}

static int gpio_keys_resume(struct device *dev)
{
	struct gpio_keys_drvdata *ddata = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < ddata->n_buttons; i++) {
		struct gpio_button_data *bdata = &ddata->data[i];
		if (bdata->button->wakeup && device_may_wakeup(dev))
		{
			disable_irq_wake(bdata->irq);
		}
	}

	return 0;
}
#endif

//ASUS BSP freddy++
static int gpio_keys_suspend_noirq(struct device *dev)
{
//ASUS BSP freddy+++ fix TT 281235:Power key suspend/resume fail
	if (g_keycheck_abort)
	{
		printk("[Gpio_keys] noirq_check: suspend_abort\n");
		return -EBUSY;
	}
//ASUS BSP freddy--- fix TT 281235:Power key suspend/resume fail
       g_bResume=0;
       printk("[Gpio_keys]%s:,Resume sts= %x,keylock sts= %x\r\n",__func__,g_bResume,g_bpwr_key_lock_sts);

       return 0;
}

static int gpio_keys_resume_noirq(struct device *dev)
{
        g_bResume=1;
        printk("[Gpio_keys]%s:,Resume sts= %x,keylock sts= %x\r\n",__func__,g_bResume,g_bpwr_key_lock_sts);

        return 0;
}
//ASUS BSP freddy--
//ASUS BSP freddy++ Register device PM callbacks
static const struct dev_pm_ops gpio_keys_pm_ops = {
	.suspend	= gpio_keys_suspend,
	.resume		= gpio_keys_resume,
//ASUS BSP freddy++
        .suspend_noirq  = gpio_keys_suspend_noirq,
        .resume_noirq   = gpio_keys_resume_noirq,
//ASUS BSP freddy--
};
//static SIMPLE_DEV_PM_OPS(gpio_keys_pm_ops, gpio_keys_suspend, gpio_keys_resume);
//ASUS BSP freddy-- Register device PM callbacks
static struct platform_driver gpio_keys_device_driver = {
	.probe		= gpio_keys_probe,
	.remove		= __devexit_p(gpio_keys_remove),
	.driver		= {
		.name	= "gpio-keys",
		.owner	= THIS_MODULE,
		.pm	= &gpio_keys_pm_ops,
		.of_match_table = gpio_keys_of_match,
	}
};

//ASUS BSP freddy++ for pad key porting
static void Pad_keys_report_event(int button_code, int press)
{
	printk("[Gpio_keys]%s: start++ \n",__func__);

	if (press){     //press
                  if (button_code == PAD_KEY_POWER) {

                                printk("[Gpio_keys]%s- p05 pwr key:%x,Resume sts= %x,keylock sts= %x\r\n",__func__,press,g_bResume,g_bpwr_key_lock_sts);
                                if (g_bResume) {
                                        wake_lock_timeout(&pwr_key_wake_lock, 3 * HZ);
                                        g_bpwr_key_lock_sts=1;
                                        g_bResume=0;
                                        printk(KERN_INFO "[Gpio_keys]Wakelock 3 sec for PWR key on P05\r\n");
                                }
                                else if(g_bpwr_key_lock_sts) {
                                        wake_unlock(&pwr_key_wake_lock);
                                        g_bpwr_key_lock_sts=0;
                                        printk(KERN_INFO "[Gpio_keys]Unlock 3 sec for PWR key on P05\r\n");
                                }
                   }

       }




    printk("[Gpio_keys]PAD key keycode=%d  state=%s\n",
				button_code, press ? "press" : "release");
    input_event(g_input_dev, EV_KEY, button_code, press);
    input_sync(g_input_dev);
}
//ASUS BSP freddy++ for pad key porting
static int mp_PadKeyevent_report(struct notifier_block *this, unsigned long event, void *ptr)
{
	struct gpio_keys_drvdata *ddata = input_get_drvdata(g_input_dev);
	switch (event)
	{
		case P01_ADD:
		{
			printk("[Gpio_keys] PAD ADD.\r\n");
			return NOTIFY_DONE;
		}
		case P01_REMOVE:
		{
			printk("[Gpio_keys] PAD REMOVE.\r\n");
			if (pad_volUp_still_press){
				printk("[Gpio_keys] fake VOLUP_release for padmode.\n ");
				Pad_keys_report_event(PAD_KEY_VOLUP, 0);
				pad_volUp_still_press = 0;
			}
			if (pad_volDown_still_press){
				printk("[Gpio_keys] fake VOLDOWN_release for padmode.\n ");
				Pad_keys_report_event(PAD_KEY_VOLDOWN, 0);
				pad_volDown_still_press = 0;
			}
			return NOTIFY_DONE;
		}
        case P01_VOLUP_KEY_PRESSED:
		{
			printk("[Gpio_keys] PAD VOLUMEUP PRESS.\r\n");
		#ifdef  CONFIG_PROC_FS
	                Pad_keys_report_event(ddata->pad_button_code.vol_up, 1);
		#else
	                Pad_keys_report_event(PAD_KEY_VOLUP, 1);

		#endif
				pad_volUp_still_press = 1;  //ASUS BSP freddy++ for check pad key still press when PAD Remove
            return NOTIFY_DONE;
		}
        case P01_VOLUP_KEY_RELEASED:
		{
			printk("[Gpio_keys] PAD VOLUMEUP RELEASE.\r\n");
		#ifdef  CONFIG_PROC_FS
	                Pad_keys_report_event(ddata->pad_button_code.vol_up, 0);
		#else
	                Pad_keys_report_event(PAD_KEY_VOLUP, 0);

		#endif
				pad_volUp_still_press = 0;	//ASUS BSP freddy++ for check pad key still press when PAD Remove
            return NOTIFY_DONE;
		}
        case P01_VOLDN_KEY_PRESSED:
		{
			printk("[Gpio_keys] PAD VOLUMEDOWN PRESS.\r\n");
		#ifdef  CONFIG_PROC_FS
	                Pad_keys_report_event(ddata->pad_button_code.vol_down, 1);
		#else
	                Pad_keys_report_event(PAD_KEY_VOLDOWN, 1);

		#endif
				pad_volDown_still_press = 1;	//ASUS BSP freddy++ for check pad key still press when PAD Remove
            return NOTIFY_DONE;
		}
        case P01_VOLDN_KEY_RELEASED:
		{
			printk("[Gpio_keys] PAD VOLUMEDOWN RELEASE.\r\n");
		#ifdef  CONFIG_PROC_FS
	                Pad_keys_report_event(ddata->pad_button_code.vol_down, 0);
		#else
	                Pad_keys_report_event(PAD_KEY_VOLDOWN, 0);

		#endif
				pad_volDown_still_press = 0;	//ASUS BSP freddy++ for check pad key still press when PAD Remove
        		return NOTIFY_DONE;
		}
        case P01_PWR_KEY_PRESSED:
		{
			printk("[Gpio_keys] PAD POWERKEY PRESS.\r\n");
		#ifdef  CONFIG_PROC_FS
	                Pad_keys_report_event(ddata->pad_button_code.power_key, 1);
		#else
	                Pad_keys_report_event(PAD_KEY_POWER, 1);
		#endif
			return NOTIFY_DONE;
		}
        case P01_PWR_KEY_RELEASED:
		{
			printk("[Gpio_keys] PAD POWERKEY RELEASE.\r\n");
		#ifdef  CONFIG_PROC_FS
	                Pad_keys_report_event(ddata->pad_button_code.power_key, 0);
		#else
	                Pad_keys_report_event(PAD_KEY_POWER, 0);
		#endif
			return NOTIFY_DONE;
		}
        default:
			return NOTIFY_DONE;

        }
}
//ASUS BSP freddy-- for pad key porting
//ASUS BSP freddy++ for pad key porting
static struct notifier_block mp_notifier = {
        .notifier_call = mp_PadKeyevent_report,
        .priority = VIBRATOR_MP_NOTIFY,
};
//ASUS BSP freddy-- for pad key porting
static int __init gpio_keys_init(void)
{
	register_microp_notifier(&mp_notifier);  //ASUS BSP freddy+ for pad key porting
	vol_up_gpio = 92;
	vol_down_gpio = 95;
	pwr_gpio = 28;
	return platform_driver_register(&gpio_keys_device_driver);
}

static void __exit gpio_keys_exit(void)
{
	platform_driver_unregister(&gpio_keys_device_driver);
}

late_initcall(gpio_keys_init);
module_exit(gpio_keys_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Phil Blundell <pb@handhelds.org>");
MODULE_DESCRIPTION("Keyboard driver for GPIOs");
MODULE_ALIAS("platform:gpio-keys");

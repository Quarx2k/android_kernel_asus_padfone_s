/*
 * LED Class Core
 *
 * Copyright (C) 2005 John Lenz <lenz@cs.wisc.edu>
 * Copyright (C) 2005-2007 Richard Purdie <rpurdie@openedhand.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/timer.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include <linux/leds.h>
#include "leds.h"
#include <linux/proc_fs.h>//ASUS_BSP darrency_lin ++ add asus flashlight node
#include <asm/uaccess.h>//ASUS_BSP darrency_lin ++ add asus flashlight node

#define LED_BUFF_SIZE 50

//++ASUS_BSP: Louis
/*wait microp*/
#ifdef CONFIG_EEPROM_NUVOTON
#include <linux/microp_api.h>
#include <linux/microp_notify.h>
#include <linux/microp_notifier_controller.h>	//ASUS_BSP Lenter+
extern int pad_set_backlight(int);
#endif
/*wait microp*/
#include <linux/mutex.h> 
static struct mutex thermal_bl_mutex;
//extern int a80_set_pmic_backlight(int);
//extern void asus_set_bl_brightness(int);
//#ifdef ASUS_A91_PROJECT
extern void backlight_IC_5V_Ctrl(int stat);
extern int backlight_mode_state;
extern bool S5V_enable;
enum device_mode {
    phone = 0,
    pad,
};
//#endif
//--ASUS_BSP: Louis
static bool g_led_cdev_flag = false;
static struct class *leds_class;

//ASUS_BSP Bryant: Fix flash light turn on when device pull up form Pad mode. +++
static bool g_led_flash_cdev_flag = false;
struct led_classdev *g_led_flash_cdev;
//ASUS_BSP Bryant: Fix flash light turn on when device pull up form Pad mode. +++

//ASUS_BSP darrency_lin ++ add asus flashlight node
struct led_classdev *g_led_flash_cdev_asusflashlight;
//ASUS_BSP darrency_lin -- add asus flashlight node

//ASUS_BSP jacob kung: add for debug mask ++
#include <linux/module.h>
/* Debug levels */
#define NO_DEBUG       0
#define DEBUG_POWER     1
#define DEBUG_INFO  2
#define DEBUG_VERBOSE 5
#define DEBUG_RAW      8
#define DEBUG_TRACE   10

static int debug = DEBUG_INFO;

module_param(debug, int, 0644);

MODULE_PARM_DESC(debug, "Activate debugging output");

#define led_debug(level, ...) \
		if (debug >= (level)) \
			pr_info(__VA_ARGS__);
//ASUS_BSP jacob kung: add for debug mask --

//ASUS_BSP : darrency_lin ++
#define PROC_ENTRY_ASUS_FLASH  "driver/asus_flash_brightness"
static struct proc_dir_entry *proc_asus;

static void led_update_brightness(struct led_classdev *led_cdev)
{
	if (led_cdev->brightness_get)
		led_cdev->brightness = led_cdev->brightness_get(led_cdev);
}

static ssize_t led_brightness_show(struct device *dev, 
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);

	/* no lock needed for this */
	led_update_brightness(led_cdev);

	return snprintf(buf, LED_BUFF_SIZE, "%u\n", led_cdev->brightness);
}

// ASUS_BSP: Louis +++
static unsigned long switch_brightness = 53;
static unsigned long g_brightness = 53;
static unsigned long g_thermal_fading = 100;
struct led_classdev *g_led_cdev;

static unsigned long cal_bl_fading_val(unsigned long state)
{
	if (state <= 20)
		{
			state = state;
		}
	else 
		{
	 if (state >= 17 && state <= 220)
	        state = state * g_thermal_fading / 100;

	 if(state < 17 && state > 0)
	        state = 17;
		}
    return state;
}

static ssize_t led_brightness_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	ssize_t ret = -EINVAL;
	char *after;
	unsigned long state = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;

	if (g_led_cdev_flag == false && strncmp(led_cdev->name, "lcd-backlight", sizeof("lcd-backlight"))==0){
		g_led_cdev = led_cdev;
		g_led_cdev_flag = true;
	//ASUS_BSP Bryant: Fix flash light turn on when device pull up form Pad mode. +++
	} else if (g_led_flash_cdev_flag == false && strncmp(led_cdev->name, "led:flash_0", sizeof("led:flash_0"))==0){
		g_led_flash_cdev = led_cdev;
		g_led_flash_cdev_flag = true;
	}
	//ASUS_BSP Bryant: Fix flash light turn on when device pull up form Pad mode. ---
	mutex_lock(&thermal_bl_mutex);
	if (strncmp(led_cdev->name, "lcd-backlight", sizeof("lcd-backlight")) == 0){
			switch_brightness = g_brightness = state;
			state = cal_bl_fading_val(g_brightness);
	}
	led_debug(DEBUG_VERBOSE,"[BL] (%s): user set value = %d name = %s backlight = %d  \n", __func__,(int)state,led_cdev->name,(int)g_brightness);

	if (isspace(*after))
		count++;

	if (count == size) {
		ret = count;

		if (state == LED_OFF)
			led_trigger_remove(led_cdev);
//#ifdef ASUS_A91_PROJECT
/*wait microp*/
#ifdef CONFIG_EEPROM_NUVOTON
		if (backlight_mode_state == pad)
		{
			if (strncmp(led_cdev->name, "lcd-backlight", sizeof("lcd-backlight")) == 0)
			{
				led_debug(DEBUG_VERBOSE,"[BL] (%s): pad_set_backlight = %d \n", __func__,(int)g_brightness);
				led_cdev->brightness = g_brightness;
				pad_set_backlight(g_brightness);
			}
			else
				led_set_brightness(led_cdev, state);
		}
		/*else if((state >=2000 && state <=2255))
			{
				led_debug(DEBUG_VERBOSE,"[BL] (%s): asus_set_bl_brightness = %d \n", __func__,(int)state);
				asus_set_bl_brightness(state);
			}
		*/
		else
#endif
			{

/*wait microp*/
				led_debug(DEBUG_VERBOSE,"[BL] (%s): led_set_brightness = %d \n", __func__,(int)state);
				if (S5V_enable == 0 && state != 0 && g_ASUS_hwID == A90_EVB0)
					backlight_IC_5V_Ctrl(1);
				led_set_brightness(led_cdev, state);
			}
//#endif
#ifdef ASUS_ME771KL_PROJECT
		/*if((state >=2000 && state <=2255))
			{
				led_debug(DEBUG_VERBOSE,"[BL] (%s): asus_set_bl_brightness = %d \n", __func__,(int)state);
				asus_set_bl_brightness(state);
			}
		else
		*/
			{
				led_debug(DEBUG_VERBOSE,"[BL] (%s): led_set_brightness = %d \n", __func__,(int)state);
				led_set_brightness(led_cdev, state);
			}
#endif
    mutex_unlock(&thermal_bl_mutex);	
	}
	return ret;
}


static ssize_t brightness_thermal_fading_store (struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
    struct led_classdev *led_cdev = dev_get_drvdata(dev);
    char *after;
    unsigned long state = simple_strtoul(buf, &after, 10);

    if (backlight_mode_state == pad) {
        printk("[Backlight] %s: do not ajust backlight caused by thermal in pad mode\n", __func__);
        return size;
    }

    mutex_lock(&thermal_bl_mutex);

    if (state > 100)
        state = 100;
    else if (state < 50)
        state = 50;

    g_thermal_fading = state;

   if (g_brightness >= 17 && g_brightness <= 220)
        state = g_brightness * g_thermal_fading / 100;

    if (state < 17 && state > 0)
        state = 17;

    printk("[BACKLIGHT] (%s): g_thermal_fading = %lu\n", __FUNCTION__ , g_thermal_fading);

    led_set_brightness(led_cdev, state);

    mutex_unlock(&thermal_bl_mutex);

    return size;
}

static ssize_t brightness_thermal_fading_show (struct device *dev, 
        struct device_attribute *attr, char *buf)
{
    struct led_classdev *led_cdev = dev_get_drvdata(dev);

    /* no lock needed for this */
    led_update_brightness(led_cdev);

    return snprintf(buf, LED_BUFF_SIZE, "%lu\n", g_thermal_fading);
}
//ASUS_BSP: Louis ---

static ssize_t led_max_brightness_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	ssize_t ret = -EINVAL;
	unsigned long state = 0;

	ret = strict_strtoul(buf, 10, &state);
	if (!ret) {
		ret = size;
		if (state > LED_FULL)
			state = LED_FULL;
		led_cdev->max_brightness = state;
		led_set_brightness(led_cdev, led_cdev->brightness);
	}

	return ret;
}

static ssize_t led_max_brightness_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);

	return snprintf(buf, LED_BUFF_SIZE, "%u\n", led_cdev->max_brightness);
}

static struct device_attribute led_class_attrs[] = {
#ifdef ASUS_FACTORY_BUILD
	__ATTR(brightness, 0666, led_brightness_show, led_brightness_store),
#else
	__ATTR(brightness, 0644, led_brightness_show, led_brightness_store),
#endif
	__ATTR(thermal_brightness_fading, 0644, brightness_thermal_fading_show, brightness_thermal_fading_store),
	__ATTR(max_brightness, 0644, led_max_brightness_show,
			led_max_brightness_store),
#ifdef CONFIG_LEDS_TRIGGERS
#ifdef ASUS_FACTORY_BUILD
	__ATTR(trigger, 0666, led_trigger_show, led_trigger_store),
#else
	__ATTR(trigger, 0664, led_trigger_show, led_trigger_store),
#endif
#endif
	__ATTR_NULL,
};

static void led_timer_function(unsigned long data)
{
	struct led_classdev *led_cdev = (void *)data;
	unsigned long brightness;
	unsigned long delay;

	if (!led_cdev->blink_delay_on || !led_cdev->blink_delay_off) {
		led_set_brightness(led_cdev, LED_OFF);
		return;
	}

	brightness = led_get_brightness(led_cdev);
	if (!brightness) {
		/* Time to switch the LED on. */
		brightness = led_cdev->blink_brightness;
		delay = led_cdev->blink_delay_on;
	} else {
		/* Store the current brightness value to be able
		 * to restore it when the delay_off period is over.
		 */
		led_cdev->blink_brightness = brightness;
		brightness = LED_OFF;
		delay = led_cdev->blink_delay_off;
	}

	led_set_brightness(led_cdev, brightness);

	mod_timer(&led_cdev->blink_timer, jiffies + msecs_to_jiffies(delay));
}

/**
 * led_classdev_suspend - suspend an led_classdev.
 * @led_cdev: the led_classdev to suspend.
 */
void led_classdev_suspend(struct led_classdev *led_cdev)
{
	led_cdev->flags |= LED_SUSPENDED;
	led_cdev->brightness_set(led_cdev, 0);
}
EXPORT_SYMBOL_GPL(led_classdev_suspend);

/**
 * led_classdev_resume - resume an led_classdev.
 * @led_cdev: the led_classdev to resume.
 */
void led_classdev_resume(struct led_classdev *led_cdev)
{
	led_cdev->brightness_set(led_cdev, led_cdev->brightness);
	led_cdev->flags &= ~LED_SUSPENDED;
}
EXPORT_SYMBOL_GPL(led_classdev_resume);

static int led_suspend(struct device *dev, pm_message_t state)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);

	if (led_cdev->flags & LED_CORE_SUSPENDRESUME)
		led_classdev_suspend(led_cdev);

	return 0;
}

static int led_resume(struct device *dev)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);

	if (led_cdev->flags & LED_CORE_SUSPENDRESUME)
		led_classdev_resume(led_cdev);

	return 0;
}

/**
 * led_classdev_register - register a new object of led_classdev class.
 * @parent: The device to register.
 * @led_cdev: the led_classdev structure for this device.
 */
int led_classdev_register(struct device *parent, struct led_classdev *led_cdev)
{
	led_cdev->dev = device_create(leds_class, parent, 0, led_cdev,
				      "%s", led_cdev->name);
	if (IS_ERR(led_cdev->dev))
		return PTR_ERR(led_cdev->dev);

#ifdef CONFIG_LEDS_TRIGGERS
	init_rwsem(&led_cdev->trigger_lock);
#endif
	/* add to the list of leds */
	down_write(&leds_list_lock);
	list_add_tail(&led_cdev->node, &leds_list);
	up_write(&leds_list_lock);

	if (!led_cdev->max_brightness)
		led_cdev->max_brightness = LED_FULL;

	led_update_brightness(led_cdev);

	init_timer(&led_cdev->blink_timer);
	led_cdev->blink_timer.function = led_timer_function;
	led_cdev->blink_timer.data = (unsigned long)led_cdev;

#ifdef CONFIG_LEDS_TRIGGERS
	led_trigger_set_default(led_cdev);
#endif

	g_led_flash_cdev_asusflashlight = led_cdev; //ASUS_BSP darrency_lin ++ add asus flashlight node

	printk(KERN_DEBUG "Registered led device: %s\n",
			led_cdev->name);

	return 0;
}
EXPORT_SYMBOL_GPL(led_classdev_register);

/**
 * led_classdev_unregister - unregisters a object of led_properties class.
 * @led_cdev: the led device to unregister
 *
 * Unregisters a previously registered via led_classdev_register object.
 */
void led_classdev_unregister(struct led_classdev *led_cdev)
{
#ifdef CONFIG_LEDS_TRIGGERS
	down_write(&led_cdev->trigger_lock);
	if (led_cdev->trigger)
		led_trigger_set(led_cdev, NULL);
	up_write(&led_cdev->trigger_lock);
#endif

	/* Stop blinking */
	led_brightness_set(led_cdev, LED_OFF);

	device_unregister(led_cdev->dev);

	down_write(&leds_list_lock);
	list_del(&led_cdev->node);
	up_write(&leds_list_lock);
}
EXPORT_SYMBOL_GPL(led_classdev_unregister);

//ASUS_BSP:Louis +++
//#ifdef ASUS_A91_PROJECT
/*wait microp*/
#ifdef CONFIG_EEPROM_NUVOTON
static int change_backlight_mode(struct notifier_block *this, unsigned long event, void *ptr)
{
        static int thermal_curr;
        unsigned long phone_bl;
		int LCD = 0;

        switch (event) {
            case P01_ADD:
				//asus_set_bl_brightness(0);
                backlight_mode_state = pad;
		LCD = AX_MicroP_getLCMID();
		printk("[BL][mod] %s change to Pad Pad type %d \n",__func__,LCD);
		thermal_curr = g_thermal_fading;
                g_thermal_fading = 100;
		//if ((A80_SR2 <= g_A68_hwID) && (g_A68_hwID <= A80_SR4) ){
		//		printk("[BL][mod] %s turn off Phone backlight for pmic device\n",__func__);		
		//		a80_set_pmic_backlight(0);
		//	}
		if (S5V_enable == 1){
			backlight_IC_5V_Ctrl(0);
		}
                pad_set_backlight(switch_brightness);
                return NOTIFY_DONE;

            case P01_REMOVE:
                backlight_mode_state = phone;
		printk("[BL][mod] %s change to Phone\n",__func__);
                g_thermal_fading = thermal_curr;
                phone_bl = cal_bl_fading_val(switch_brightness);
		if (S5V_enable == 0 && phone_bl != 0 && g_ASUS_hwID == A90_EVB0){
			backlight_IC_5V_Ctrl(1);
		}
		if (g_led_cdev_flag == false){
			printk("[BL][mod] %s g_led_cdev not init, use default value !!\n",__func__);
			return NOTIFY_DONE;
		}
                led_set_brightness(g_led_cdev, phone_bl);
		//asus_set_bl_brightness(phone_bl);//workround for A86 crash when leave P05
		return NOTIFY_DONE;

            default:
                return NOTIFY_DONE;
        }
}

static struct notifier_block my_hs_notifier = {
        .notifier_call = change_backlight_mode,
        .priority = VIBRATOR_MP_NOTIFY,
};
#endif
//#endif
/*wait microp*/
//ASUS_BSP:Louis ---
//ASUS_BSP darrency_lin ++ add asus flashlight node
u8 g_asus_input_value;
static int flash_ledclass_read_asus (char *page, char **start, off_t off, int count,
	int *eof, void *data_unused)
{
	int len = 0;

	len = scnprintf(page, 1024, "%d\n", g_asus_input_value);
	if (len <= off+count) *eof = 1;
	*start = page + off;
	 len -= off;
	 if (len > count) len = count;
	 if (len < 0) len = 0;

	return len;
}

static int flash_ledclass_write_asus (struct file *file, const char __user *buffer,
       unsigned long count, void *data)
{
	//ssize_t ret = 0;
	char buf[] = "0x00000000";
	unsigned long len = min((unsigned long)sizeof(buf) - 1, count);
       unsigned long state;

	if (copy_from_user(buf, buffer, len)) return count;
	buf[len] = 0;
	if (sscanf(buf, "%hhu", &g_asus_input_value) != 1) {
		return -EINVAL;
	}

	if (g_asus_input_value <= 0)
		state = LED_OFF;
       else
		state = LED_FULL;

	led_set_brightness(g_led_flash_cdev_asusflashlight, state);

	return strnlen(buf, count);
}
//ASUS_BSP darrency_lin -- add asus flashlight node

static int __init leds_init(void)
{
	leds_class = class_create(THIS_MODULE, "leds");
	if (IS_ERR(leds_class))
		return PTR_ERR(leds_class);
	leds_class->suspend = led_suspend;
	leds_class->resume = led_resume;
	leds_class->dev_attrs = led_class_attrs;

    //ASUS_BSP: Louis +++
    mutex_init(&thermal_bl_mutex); 
//#ifdef ASUS_A91_PROJECT
/*wait microp*/
#ifdef CONFIG_EEPROM_NUVOTON
    register_microp_notifier(&my_hs_notifier);
    notify_register_microp_notifier(&my_hs_notifier, "led_class"); //ASUS_BSP Lenter+
#endif
/*wait microp*/
//#endif
    //ASUS_BSP: Louis ---
//ASUS_BSP darrency_lin ++ add asus flashlight node
	if ((proc_asus = create_proc_entry(PROC_ENTRY_ASUS_FLASH, S_IRUGO | S_IWUGO, NULL))) {
		proc_asus->read_proc = flash_ledclass_read_asus;
		proc_asus->write_proc = flash_ledclass_write_asus;
	}
//ASUS_BSP darrency_lin -- add asus flashlight node
	return 0;
}

static void __exit leds_exit(void)
{
	if (proc_asus) remove_proc_entry(PROC_ENTRY_ASUS_FLASH, NULL);//ASUS_BSP darrency_lin ++ add asus flashlight node
	class_destroy(leds_class);
}

subsys_initcall(leds_init);
module_exit(leds_exit);

MODULE_AUTHOR("John Lenz, Richard Purdie");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("LED Class Interface");

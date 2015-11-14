/*
 * asus_dock_battery.c - Driver for Asus PAD battery.
 *
 * Copyright (C) 2015 Quarx2k <agent00791@gmail.com>, https://github.com/Quarx2k
 *
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 */
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/microp.h>
#include <linux/microp_notify.h>
#include <linux/microp_pin_def.h>
#include <linux/microp_api.h>
#include <linux/microp_notify.h>
#include <linux/microp_notifier_controller.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/i2c.h>
#include <linux/power_supply.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>

#undef NEED_MANUAL_UPDATE

#ifdef NEED_MANUAL_UPDATE
#define SAMPLING_RATE_MS 500
static unsigned long sampling_rate;
static struct workqueue_struct *wq;
static struct delayed_work update_battery;
#endif

struct kobject *microp_kobj;
static bool isCableIn = false;

static int asus_bat_microp_event_handler (
	struct notifier_block *this,
	unsigned long event,
	void *ptr);

static int asus_bat_pad_get_property(
	struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val);

static struct notifier_block asus_bat_microp_notifier = {
        .notifier_call = asus_bat_microp_event_handler,
};

static enum power_supply_property pad_bat_properties[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_HEALTH
};

static char *pm_power_supplied_to[] = {
	"dock_battery",
};

static struct power_supply pad_bat_psy = {
	.name		= "dock_battery",
	.type		= POWER_SUPPLY_TYPE_DOCK_BATTERY,
	.supplied_to = pm_power_supplied_to,
	.num_supplicants = ARRAY_SIZE(pm_power_supplied_to),
	.properties	= pad_bat_properties,
	.num_properties	= ARRAY_SIZE(pad_bat_properties),
	.get_property	= asus_bat_pad_get_property,
};
#ifdef NEED_MANUAL_UPDATE
static void __ref asus_batt_update(struct work_struct *work) {
	if (AX_MicroP_IsP01Connected() {
		power_supply_changed(&pad_bat_psy);
	}
	/* Make a dedicated work_queue for CPU0 */
	queue_delayed_work_on(0, wq, &update_battery, sampling_rate);
}
#endif

static int asus_bat_report_pad_status(void) {
	int value = 0;
	int capacity = 0;
	//int result = 0;
	//result = AX_MicroP_get_ChargingStatus(0);
	capacity = AX_MicroP_readBattCapacity(0);

	if (!isCableIn)
		value = POWER_SUPPLY_STATUS_DISCHARGING;
	else if (isCableIn)
		value = POWER_SUPPLY_STATUS_CHARGING;
	else if (capacity == 100)
		value = POWER_SUPPLY_STATUS_FULL;
	else
		value = POWER_SUPPLY_STATUS_UNKNOWN;

	return value;
}

static void set_microp_vbus(int level)
{
	int rt;

	rt = AX_MicroP_setGPIOOutputPin(OUT_uP_VBUS_EN, level);

	if (rt<0) {
		printk("[BAT] microp set vbus error: %d\n", level);
	} else if (rt == 0) {
		printk("[BAT] microp set vbus success: %d\n", level);
	}

}

static int asus_bat_microp_event_handler (
	struct notifier_block *this,
	unsigned long event,
	void *ptr)
{
	switch (event) {
	case P01_ADD:
		printk("[BAT] %s() +++, P01_ADD \r\n", __FUNCTION__);
		set_microp_vbus(1);  //Disable vbus when pad  in;
		break;	
	case P01_REMOVE: // means P01 removed
		printk("[BAT] %s() +++, P01_REMOVE \r\n", __FUNCTION__);
		set_microp_vbus(0);  //Disable vbus when pad out;
		break;
	case P01_BATTERY_POWER_BAD: // P01 battery low
		printk("[BAT] %s() +++, P01_BATTERY_POWER_BAD \r\n", __FUNCTION__);
		break;
	case P01_AC_USB_IN:
		printk("[BAT] %s() +++, P01_AC_USB_IN \r\n", __FUNCTION__);
		isCableIn = true;
		break;
	case P01_AC_USB_OUT:
		printk("[BAT] %s() +++, P01_AC_USB_OUT \r\n", __FUNCTION__);
		isCableIn = false;
		break;
	case DOCK_INIT_READY:
		printk("[BAT] %s() +++, DOCK_INIT_READY \r\n", __FUNCTION__);
		//asus_bat_set_dock_bat_present(true);
		break;
	case DOCK_PLUG_OUT:
		printk("[BAT] %s() +++, DOCK_PLUG_OUT \r\n", __FUNCTION__);
		//asus_bat_set_dock_bat_present(false);
		break;
	case DOCK_EXT_POWER_PLUG_IN_READY: // means dock charging
		printk("[BAT] %s() +++, DOCK_EXT_POWER_PLUG_IN \r\n", __FUNCTION__);
		break;
	case DOCK_EXT_POWER_PLUG_OUT_READY:	// means dock discharging
		printk("[BAT] %s() +++, DOCK_EXT_POWER_PLUG_OUT \r\n", __FUNCTION__);
		break;		
	case DOCK_BATTERY_POWER_BAD_READY:
		printk("[BAT] %s() +++, DOCK_BATTERY_POWER_BAD \r\n", __FUNCTION__);
		break;
	default:
		//mutex_unlock(&asus_bat->microp_evt_lock);
		//printk("[BAT] %s(), not listened evt: %lu \n", __FUNCTION__, event);
		return NOTIFY_DONE;
	}

//	mutex_unlock(&asus_bat->microp_evt_lock);

//	asus_bat_update_all_bat();
	power_supply_changed(&pad_bat_psy);

	return NOTIFY_DONE;
}

static int asus_battery_suspend(struct device *dev)
{
#ifdef NEED_MANUAL_UPDATE
	pr_info("Battery work has suspended\n");
   	flush_workqueue(wq);
	cancel_delayed_work_sync(&update_battery);
#endif
	return 0;
}

static int asus_battery_resume(struct device *dev)
{
#ifdef NEED_MANUAL_UPDATE
	pr_info("Battery work has resumed \n");
	queue_delayed_work_on(0, wq, &update_battery, 1);
#endif
	return 0;
}

static int asus_bat_pad_get_property (
	struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	bool pad_present = false;
	pr_debug( "[BAT] %s(), name: %s , property: %d \r\n", __FUNCTION__, psy->name, (int)psp);

	if (0 == AX_MicroP_IsP01Connected())
		pad_present = false;
	else
		pad_present = true;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		if (pad_present) {
			pr_debug("[BAT] %s(), pad present \r\n", __FUNCTION__);
			val->intval = 1;
		} else {
			//printk("[BAT] %s(), pad not present \r\n", __FUNCTION__);
			val->intval = 0;
		}
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (pad_present) {
			val->intval = AX_MicroP_readBattCapacity(0);
		} else {
			//printk("[BAT] pad not present, cannot get cap\n");
			val->intval = -1;
		}
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if (pad_present) {
			val->intval = AX_MicroP_get_BattVoltage(0);
		} else {
			//printk("[BAT] pad not present, cannot get voltage\n");
			val->intval = -1;
		}
		break;
	case POWER_SUPPLY_PROP_TEMP:
		if (pad_present) {
			val->intval = AX_MicroP_get_BattTemp(0);
		} else {
			//printk("[BAT] pad not present, cannot get teme\n");
			val->intval = -1;
		}
		break;
	case POWER_SUPPLY_PROP_STATUS:
		if (pad_present) {
			val->intval = asus_bat_report_pad_status();
		} else {
			//printk("[BAT] pad not present, status unknown\n");
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		}
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		if (pad_present) {
			val->intval = AX_MicroP_get_AvgCurrent(1);
		} else {
			//printk("[BAT] pad not present, status unknown\n");
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		}
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		if (pad_present) {
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		} else {
			//printk("[BAT] pad not present, status unknown\n");
			val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
		}
		break;
	default:
		//printk("[BAT] %s(), unknown psp:%d \n", __FUNCTION__, (int)psp);
		return -EINVAL;
	}
	return 0;
}

static int asus_bat_probe(struct platform_device *pdev) 
{

	int err = 0;

// /sys/microp props for healthd;
	printk("Start probe Asus Bat\n");

	microp_kobj = kobject_create_and_add("microp", NULL) ;
	
	if (microp_kobj == NULL) {
		pr_warn("%s: microp_kobj create_and_add failed\n", __func__);

	} else {
/*
		if (sysfs_create_file(microp_kobj, &dev_attr_battery_capacity.attr))
			pr_warn("%s: sysfs_create_file failed for battery capacity\n", __func__);
*/
	}
	
	err = power_supply_register(&pdev->dev, &pad_bat_psy);
	if (err < 0) {
		printk(KERN_ERR "power_supply_register pad_bat_psy failed, err = %d\n", err);
		return -1;
	}

	register_microp_notifier(&asus_bat_microp_notifier);
	notify_register_microp_notifier(&asus_bat_microp_notifier, "asus_bat");

#ifdef NEED_MANUAL_UPDATE
	sampling_rate = msecs_to_jiffies(SAMPLING_RATE_MS);
	wq = create_singlethread_workqueue("battery_update_workqueue");

	if (!wq)
		return -ENOMEM;

	INIT_DELAYED_WORK(&update_battery, asus_batt_update);
	queue_delayed_work_on(0, wq, &update_battery, sampling_rate);
#endif
	printk("End probe Asus Bat\n");

	return 0;
}
static int asus_bat_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct dev_pm_ops asusbat_pm_ops = {
    .suspend = asus_battery_suspend,
    .resume = asus_battery_resume,
};

static struct platform_driver asus_bat_driver = {
	.probe	= asus_bat_probe,
	.remove	= asus_bat_remove,
	.driver	= {
		.name	= "asus_bat",
		.owner	= THIS_MODULE,
		.pm	= &asusbat_pm_ops,
	},
};

static int __init asus_bat_init(void)
{
	return platform_driver_register(&asus_bat_driver);
}

static void __exit asus_bat_exit(void)
{
	platform_driver_unregister(&asus_bat_driver);
}

late_initcall(asus_bat_init);
module_exit(asus_bat_exit);

MODULE_AUTHOR("Quarx2k <agent00791@gmail.com>");
MODULE_DESCRIPTION("Asus Dock Battery Driver");
MODULE_LICENSE("GPL");


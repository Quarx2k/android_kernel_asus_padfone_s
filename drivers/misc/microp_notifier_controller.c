
//                     ASUSTek Computer Inc.
//         Copyright (c) 2010 ASUSTek Computer inc, Taipei.

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/microp_notify.h>

#define MP_REGISTER_NUM	19


struct mp_register_info{
	char *name;
	bool registered;
	struct notifier_block *nb;
};


static struct mp_register_info mp_register_status[] = {
	{"msm_otg", false, NULL},
	{"led_class", false, NULL},
	{"a68_backlight", false, NULL},
	{"rmi_f01", false, NULL},
	{"nt11003_10p", false, NULL},
	{"atmel_mxt_pad_ts", false, NULL},
	{"sis_i2c", false, NULL},
	{"gpio_key", false, NULL},
	{"uvc_driver", false, NULL},
	{"cm36283", false, NULL},
	{"al3010", false, NULL},
	{"axc_batteryservice", false, NULL},
	{"axc_smb346charger", false, NULL},	
	{"asus_bat", false, NULL},
	{"pm8xxx_vibrator", false, NULL},
	{"ami306_6050", false, NULL},
	{"mpu_dev_6050", false, NULL},
	{"wcd9310", false, NULL},
        {"cap1106", false, NULL}
};


void notify_register_microp_notifier(struct notifier_block *nb, char* driver_name)
{	
	int i = 0;
	
	for(i=0; i<MP_REGISTER_NUM; i++)
	{
		if( strcmp(mp_register_status[i].name, driver_name)==0 )
		{
			mp_register_status[i].registered = true;
			mp_register_status[i].nb = nb;
		}
	}

	return;
}
EXPORT_SYMBOL_GPL(notify_register_microp_notifier);

void notify_unregister_microp_notifier(struct notifier_block *nb, char* driver_name)
{
	int i = 0;
	
	for(i=0; i<MP_REGISTER_NUM; i++)
	{
		if( strcmp(mp_register_status[i].name, driver_name)==0 )
		{
			mp_register_status[i].registered = false;
			mp_register_status[i].nb = NULL;
		}
	}

	return;
}
EXPORT_SYMBOL_GPL(notify_unregister_microp_notifier);

static ssize_t mp_notifier_read_proc(struct file *filp, char *buf, size_t count, loff_t *offp) 
{
    int i = 0;
    int registered_count = 0;
    char usage_message[256];
	
	sprintf(usage_message, "\nUsage: [register|unregister] [");
	sprintf(buf, "registered driver:\n");	
	for(i=0; i<MP_REGISTER_NUM; i++)
	{
		if(mp_register_status[i].registered)
		{
			strcat(buf, mp_register_status[i].name);
			
			if(mp_register_status[i].nb == NULL)
			{
				strcat(buf, " nb is NULL");
			}
			
			strcat(buf, "\n");
			registered_count++;
		}
		
		strcat(usage_message, mp_register_status[i].name);
		if(i < MP_REGISTER_NUM-1)
		{
			strcat(usage_message, "|");
		}
	}
	
	if(registered_count == 0)
	{
		strcat(buf, "NONE !!!\n");
	}
	
	strcat(usage_message, "]\n");
	strcat(buf, usage_message);
	
	return strlen(buf);
}

static ssize_t mp_notifier_write_proc(struct file *filp, const char *buff, size_t len, loff_t *off)
{
	char msg[128];
	char tok[128];
	int i = 0;
	
	if (len > 128){
		printk("[mp_notifier_controller] input length error\n");
		return len;
	}

	memset(msg, 0x0, 128);
	if (copy_from_user(msg, buff, len))
		return -EFAULT;

	//printk("[mp_notifier_controller] input msg = %s, length=%d\n", msg, strlen(msg));
	
	memcpy(tok, msg, 8);
	tok[8] = '\0';
	if( strcmp(tok, "register")==0 ){
		printk("[mp_notifier_controller] token1 = register, strlen = %d\n", strlen(tok));

		memcpy(tok, msg+9, strlen(msg)-9-1);
		tok[strlen(msg)-9-1] = '\0';
		
		for(i=0; i<MP_REGISTER_NUM; i++)
		{
			if( strcmp(mp_register_status[i].name, tok)==0 )
			{
				printk("[mp_notifier_controller] token2 = %s\n", mp_register_status[i].name);
				mp_register_status[i].registered = true;
				if(mp_register_status[i].nb != NULL){
					printk("[mp_notifier_controller] register %s\n", mp_register_status[i].name);
					register_microp_notifier(mp_register_status[i].nb);
				}
			}
		}
	}
	else{
		memcpy(tok, msg, 10);
		tok[10] = '\0';
		
		if( strcmp(tok, "unregister")==0 ){
			printk("[mp_notifier_controller] token1 = unregister\n");
			
			memcpy(tok, msg+11, strlen(msg)-11-1);
			tok[strlen(msg)-11-1] = '\0';
			
			for(i=0; i<MP_REGISTER_NUM; i++)
			{
				if( strcmp(mp_register_status[i].name, tok)==0 )
				{
					printk("[mp_notifier_controller] token2 = %s\n", mp_register_status[i].name);
					mp_register_status[i].registered = false;
					if(mp_register_status[i].nb != NULL){
						printk("[mp_notifier_controller] unregister %s\n", mp_register_status[i].name);
						unregister_microp_notifier(mp_register_status[i].nb);
					}
				}
			}
		}
		else{
			printk("[mp_notifier_controller]input message error\n");
		}
	}

	return len;
}

static struct file_operations key_proc_ops = {
	.write = mp_notifier_write_proc,
	.read = mp_notifier_read_proc,
};

static int mp_notifier_controller_probe(struct platform_device *pdev)
{
	struct proc_dir_entry *entry;
	
	entry = proc_create("mp_notifier_status", 0666, NULL, &key_proc_ops);
	if (entry == NULL) {
		printk("[microp_notifier_controller] create_proc_entry fail\r\n");
	}
	
    return 0;
}

static struct platform_driver mp_notifier_controller_driver = {
	.probe	= mp_notifier_controller_probe,
	.driver	= {
		.name	= "microp_notifier_controller",
	},
};

static struct platform_device mp_notifier_controller_device = {
	.name = "microp_notifier_controller",
	.id = -1,
};

static int __init mp_notifier_controller_init(void)
{
	int ret;
	
	ret = platform_driver_register(&mp_notifier_controller_driver);

	if (!ret) {
		ret = platform_device_register(&mp_notifier_controller_device);
		if (ret)
			platform_driver_unregister(&mp_notifier_controller_driver);
	}
	
	return ret;
}

static void __exit mp_notifier_controller_exit(void)
{

}

module_init(mp_notifier_controller_init);
module_exit(mp_notifier_controller_exit);

MODULE_DESCRIPTION("microp notifier controller driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Lenter_Chang");





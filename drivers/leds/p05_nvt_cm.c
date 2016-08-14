/*
 * P05  ASUS HSIC driver.
 *
 * Copyright (c) 2013, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/time.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <mach/board.h>
#include <mach/gpio.h>
#include <linux/i2c.h>

#ifdef CONFIG_EEPROM_NUVOTON 
#include <linux/microp_notify.h>
#include <linux/microp_notifier_controller.h>
#include <linux/microp_api.h>
#include <linux/microp_pin_def.h>
#endif

#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>

#elif defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#define CM_EN               0x134
#define CM_GLOBAL_FUNC      0x800
#define CM_HUE              0x801
#define CM_SAT              0x802
#define CM_INT              0x803
#define CM_BRS              0x804
#define CM_HSIC_EN          0x805
#define CM_PEAK_EN          0x858
#define GAIN                0x859

#define HUE_BASE            0x810
#define SAT_BASE            0x828
#define INT_BASE            0x840

enum cm_mode {
    HUE = 0,
    SATURATION,
    INTENSITY,
};

struct asus_pad_hsic {
    int enable;
    int hsic_mode;
    int cm_level;
    char vivid_setting[24];
};

struct NT71890_data {
	struct i2c_client *client;
	struct mutex lock;
    struct input_dev   *input_dev;
#if defined(CONFIG_FB)
    struct notifier_block fb_notif;
#endif
};

struct i2c_client *NT71890_client = NULL;
static int nt71890_write_reg(int reg_addr, int data, int len);
static int nt71890_read_reg(int reg_addr, int data_len, void *data);
static struct workqueue_struct *nt71890_workqueue = NULL;
static struct delayed_work nt71890_splendid_work;
static struct asus_pad_hsic g_hue_data, g_saturation_data, g_intensity_data;

static int g_nt71890_switch_earlysuspend = 0;
static int g_Pad_SMAP_update = 0;
static int g_Pad_Splendid_On = 0;
static bool g_PadAttach = false;
#ifdef CONFIG_EEPROM_NUVOTON 
static int PAD_HW = 0;
#endif

//cabc +++
static struct delayed_work nt71890_turn_off_cabc_work;
void nvt71890_cabc_set(bool bOn);
//cabc ---

void nvt_hsic_set(struct asus_pad_hsic *hsic_info)
{
    int i, base_reg = HUE_BASE;
    unsigned char buff;

    if (hsic_info->enable) {

        if (g_PadAttach) 
        {
            nt71890_read_reg(CM_EN, 1, &buff);
            nt71890_write_reg(CM_EN, (buff | 1<<6), 3);
        }

        switch (hsic_info->hsic_mode) {
            case HUE:
                g_hue_data.cm_level = hsic_info->cm_level;
                g_hue_data.enable = 1;

                if (g_PadAttach)
                {
                    if ((hsic_info->cm_level >= 0x0 && hsic_info->cm_level <= 0xf) || 
                        (hsic_info->cm_level >= 0xf0 && hsic_info->cm_level <= 0xff)) {
                        nt71890_read_reg(CM_GLOBAL_FUNC, 1, &buff);
                        nt71890_write_reg(CM_GLOBAL_FUNC, (buff | 1<<2), 3);
                        nt71890_write_reg(CM_HUE, hsic_info->cm_level, 3);
                    }
                }

            break;

            case SATURATION:
                memcpy(g_saturation_data.vivid_setting, hsic_info->vivid_setting,
                    sizeof(hsic_info->vivid_setting));
                g_saturation_data.enable = 1;

                if (g_PadAttach) 
                {
                    nt71890_read_reg(CM_HSIC_EN, 1, &buff);
                    nt71890_write_reg(CM_HSIC_EN, (buff | 0x5), 3);

                    base_reg = SAT_BASE;

                    for(i=0; i<24; i++) {
                        nt71890_write_reg(base_reg, hsic_info->vivid_setting[i], 3);
                        base_reg++;
                    }
                }

                if (hsic_info->vivid_setting[0] == 0x80 && hsic_info->vivid_setting[4] == 0x80 &&
                     hsic_info->vivid_setting[8] == 0x80 && hsic_info->vivid_setting[12] == 0x80 &&
                     hsic_info->vivid_setting[16] == 0x80 && hsic_info->vivid_setting[20] == 0x80) {
                    g_Pad_SMAP_update = 0;
                }
                else {
                    g_Pad_SMAP_update = 1;
                }

            break;

            case INTENSITY:
                g_intensity_data.cm_level = hsic_info->cm_level;
                g_intensity_data.enable = 1;

                if (g_PadAttach)
                {
                    nt71890_read_reg(CM_GLOBAL_FUNC, 1, &buff);
                    nt71890_write_reg(CM_GLOBAL_FUNC, (buff | 0x1), 3);
                    nt71890_write_reg(CM_INT, hsic_info->cm_level, 3);
                }

            break;

            default:
                printk("[P05][CM] Unknown HSIC mode\n");
                return;
        }

        g_Pad_Splendid_On = 1;
    }
    else {
        if (g_PadAttach)
        {
            nt71890_read_reg(CM_EN, 1, &buff);
            nt71890_write_reg(CM_EN, (buff & 0xBf), 3); //bit 7 set to 0

            nt71890_read_reg(CM_HSIC_EN, 1, &buff);
            nt71890_write_reg(CM_HSIC_EN, (buff & 0xf0), 3);

            g_Pad_SMAP_update = 0;
            g_Pad_Splendid_On = 0;
        }
    }

    return;
}
EXPORT_SYMBOL(nvt_hsic_set);

//TCON CABC control +++
static void set_p05_cabc_off_func(struct work_struct *work)
{
#ifdef CONFIG_EEPROM_NUVOTON 
    if (PAD_HW == PAD_P92L || PAD_HW == PAD_P93L)
        return;
#endif

    if (g_PadAttach) {
        nvt71890_cabc_set(0);
    }
}

void nvt71890_cabc_set(bool bOn)
{
    unsigned char buff;

    if (g_PadAttach) {
#ifdef CONFIG_EEPROM_NUVOTON 
        AX_MicroP_setGPIOOutputPin(OUT_uP_LCD_I2C_SW_EN, 1);
#endif
        if (bOn) {
            printk("[P05][CABC] Turn on CABC\n");
            nt71890_read_reg(CM_EN, 1, &buff);
            nt71890_write_reg(CM_EN, (buff | 1 << 5), 3);
        }
        else {
            printk("[P05][CABC] Turn off CABC\n");
            nt71890_read_reg(CM_EN, 1, &buff);
            nt71890_write_reg(CM_EN, (buff & 0xDF), 3);
        }
#ifdef CONFIG_EEPROM_NUVOTON 
        AX_MicroP_setGPIOOutputPin(OUT_uP_LCD_I2C_SW_EN, 0);
#endif
    }
}
EXPORT_SYMBOL(nvt71890_cabc_set);
//TCON CABC control ---

static int nt71890_write_reg(int reg_addr, int data, int len)
{
    int err = 0;
    char buf[len];

    struct i2c_msg msg[] = {
        {
            .addr = NT71890_client->addr,
            .flags = 0, //write
            .len = len,
            .buf = buf,
        },
    };

    if (!NT71890_client->adapter) {
        printk("%s: nt71890_write_reg fail\n", __func__);
        return -ENODEV;
    }

    buf[0] = reg_addr >> 8;
    buf[1] = (reg_addr & 0xff);
    buf[2] = data;

    err = i2c_transfer(NT71890_client->adapter, msg, ARRAY_SIZE(msg));

    if (err != ARRAY_SIZE(msg)) {
        printk("[P05][CM] nt71890_write_reg err: reg=0x%x, data=%x, err = %d\n", reg_addr, buf[2], err);
    }

    return err; // return postive is expected.
}

static int nt71890_read_reg(int reg_addr, int data_len, void *data)
{
    int err = 0;
    char buf[2];

    struct i2c_msg msg[] = {
        {
            .addr = NT71890_client->addr,
            .flags = 0, //write
            .len = 2,   //2 byte register address
            .buf = buf,
        },
        {
            .addr = NT71890_client->addr,
            .flags = I2C_M_RD, //read
            .len = data_len,
            .buf = data,
        }
    };

    buf[0] = reg_addr >> 8;
    buf[1] = (reg_addr & 0xff);

    if (!NT71890_client->adapter) {
        printk("%s: nt71890_read_reg fail\n", __func__);
        return -ENODEV;
    }

    memset(&data, 0, sizeof(data));

    err = i2c_transfer(NT71890_client->adapter, msg, ARRAY_SIZE(msg));

    if (err != ARRAY_SIZE(msg)) {
        printk("[P05][CM] nt71890_read_reg err : reg=0x%x,  err= %d\n", reg_addr, err);
    }

    return err;
}

static void nt71890_restore_splendid_work(struct work_struct *work)
{
    printk("[P05][CM] ++ %s, hue(0x%x), g_Pad_SMAP_update(%d), intensity(0x%x) \n", __func__, g_hue_data.cm_level, g_Pad_SMAP_update ,g_intensity_data.cm_level);

    if (g_PadAttach)
    {
        if (g_hue_data.cm_level != 0x0) {
            nvt_hsic_set(&g_hue_data);
        }
        if (g_Pad_SMAP_update) {
            nvt_hsic_set(&g_saturation_data);
        }
        if (g_intensity_data.cm_level != 0x80) {
            nvt_hsic_set(&g_intensity_data);
        }
    }

    printk("[P05][CM] -- %s \n", __func__);
}

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void nt71890_early_suspend(struct early_suspend *handler)
{
    unsigned char buff;

    if (g_PadAttach && g_Pad_Splendid_On) {
        printk("[P05][CM] ++ %s, g_Pad_Splendid_On(%d) \n", __func__, g_Pad_Splendid_On);

        nt71890_read_reg(CM_EN, 1, &buff);
        nt71890_write_reg(CM_EN, (buff & 0xBf), 3);

        nt71890_read_reg(CM_HSIC_EN, 1, &buff);
        nt71890_write_reg(CM_HSIC_EN, (buff & 0xf0), 3);

        printk("[P05][CM] -- %s \n", __func__);
    }
    g_nt71890_switch_earlysuspend = 1;
}

static void nt71890_late_resume(struct early_suspend *handler)
{
    if (g_nt71890_switch_earlysuspend) {
        if (g_Pad_Splendid_On && g_PadAttach) {
            queue_delayed_work(nt71890_workqueue, &nt71890_splendid_work, 200 );
        }
        if (g_PadAttach) {
            printk("[P05][CM] ++ %s \n", __func__);
            //nvt71890_cabc_set(0);
            queue_delayed_work(nt71890_workqueue, &nt71890_turn_off_cabc_work, 220);   //delay 220ms for 71890 initialize
            printk("[P05][CM] -- %s \n", __func__);
        }
    }
    g_nt71890_switch_earlysuspend = 0;
}

static struct early_suspend NT71890_early_suspend_desc = {
    .level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
    .suspend = nt71890_early_suspend,
    .resume = nt71890_late_resume,
};

#elif defined(CONFIG_FB)

static void nt71890_early_suspend(void)
{
    unsigned char buff;

    if (g_PadAttach && g_Pad_Splendid_On) {
        printk("[P05][CM] ++ %s, g_Pad_Splendid_On(%d) \n", __func__, g_Pad_Splendid_On);

        nt71890_read_reg(CM_EN, 1, &buff);
        nt71890_write_reg(CM_EN, (buff & 0xBf), 3);

        nt71890_read_reg(CM_HSIC_EN, 1, &buff);
        nt71890_write_reg(CM_HSIC_EN, (buff & 0xf0), 3);

        printk("[P05][CM] -- %s \n", __func__);
    }
    g_nt71890_switch_earlysuspend = 1;
}

static void nt71890_late_resume(void)
{
    if (g_nt71890_switch_earlysuspend) {
        if (g_Pad_Splendid_On && g_PadAttach) {
            queue_delayed_work(nt71890_workqueue, &nt71890_splendid_work, 200 );
        }
        if (g_PadAttach) {
            printk("[P05][CM] ++ %s \n", __func__);
            //nvt71890_cabc_set(0);
            queue_delayed_work(nt71890_workqueue, &nt71890_turn_off_cabc_work, 220);   //delay 220ms for 71890 initialize
            printk("[P05][CM] -- %s \n", __func__);
        }

        g_nt71890_switch_earlysuspend = 0;
    }
}

static int nvt_71890_fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    static int blank_old = 0;
    int *blank;

    if (evdata && evdata->data && event == FB_EVENT_BLANK) {
        blank = evdata->data;
        if (*blank == FB_BLANK_UNBLANK) {
            if (blank_old == FB_BLANK_POWERDOWN) {
                blank_old = FB_BLANK_UNBLANK;
                nt71890_late_resume();
            }
        } else if (*blank == FB_BLANK_POWERDOWN) {
            if (blank_old == 0 || blank_old == FB_BLANK_UNBLANK) {
                blank_old = FB_BLANK_POWERDOWN;
                nt71890_early_suspend();
            }
        }
    }

    return 0;
}
#endif

#ifdef CONFIG_I2C_STRESS_TEST
#include <linux/i2c_testcase.h>
#define I2C_TEST_LCM_FAIL (-1)
static int Test_NVT_LCM(struct i2c_client *client)
{
    int lnResult = I2C_TEST_PASS;
    int error;
    unsigned char buff;

    i2c_log_in_test_case("Test_LCM_TCON++\n");

#ifdef CONFIG_EEPROM_NUVOTON 
    AX_MicroP_setGPIOOutputPin(OUT_uP_LCD_I2C_SW_EN, 1);
#endif

    error = nt71890_read_reg(CM_EN, 1, &buff);
    if (error < 0) {
        lnResult = I2C_TEST_LCM_FAIL;
    }

#ifdef CONFIG_EEPROM_NUVOTON 
    AX_MicroP_setGPIOOutputPin(OUT_uP_LCD_I2C_SW_EN, 0);
#endif

    i2c_log_in_test_case("Test_LCM_TCON--\n");
    return lnResult;
};

static struct i2c_test_case_info DisplayTestCaseInfo[] =
{
    __I2C_STRESS_TEST_CASE_ATTR(Test_NVT_LCM),
};
#endif

#ifdef CONFIG_EEPROM_NUVOTON 
static int tcon_mircop_event_handler(struct notifier_block *this, unsigned long event, void *ptr)
{
    switch (event) {
        case P01_ADD:
            g_PadAttach = true;
            PAD_HW = AX_MicroP_getPadModel();
            if (g_Pad_Splendid_On) {
                if (delayed_work_pending(&nt71890_splendid_work)){
                    printk("[P05][CM] Begin cancel work \r\n");
                    cancel_delayed_work_sync(&nt71890_splendid_work);
                    printk("[P05][CM] Finish cancel work \r\n");
                }
                printk("[P05][CM] P05 Add, restore Splendid Setting\n");
                queue_delayed_work(nt71890_workqueue, &nt71890_splendid_work, 2*HZ);
            }
            if (delayed_work_pending(&nt71890_turn_off_cabc_work)){
                cancel_delayed_work_sync(&nt71890_turn_off_cabc_work);
            }
            queue_delayed_work(nt71890_workqueue, &nt71890_turn_off_cabc_work, 2*HZ);
            return NOTIFY_DONE;

        case P01_REMOVE:
            g_PadAttach = false;
            return NOTIFY_DONE;

        default:
            return NOTIFY_DONE;
    }
}

static struct notifier_block my_hs_notifier = {
        .notifier_call = tcon_mircop_event_handler,
        .priority = LCD_MP_NOTIFY,
};
#endif

static int init_nt71890(void)
{
    printk("[P05][CM]: init_nt71890 +.\n");

    nt71890_workqueue = create_singlethread_workqueue("NT71890_wq");
    INIT_DELAYED_WORK(&nt71890_splendid_work, nt71890_restore_splendid_work);
    INIT_DELAYED_WORK(&nt71890_turn_off_cabc_work, set_p05_cabc_off_func);

    g_hue_data.hsic_mode = HUE;
    g_saturation_data.hsic_mode = SATURATION;
    g_intensity_data.hsic_mode = INTENSITY;

    g_hue_data.cm_level = 0x0;  //set to default
    g_intensity_data.cm_level = 0x80;  //set to default

    printk("[P05][CM] init_nt71890 -.\n");
    return 1;
}

static int __devinit NT71890_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct NT71890_data *data;
	int err = 0;
	int g_NT71890ProbeError = 0;

	printk("[P05][CM]++ NT71890_probe\n");

	data = kzalloc(sizeof(struct NT71890_data), GFP_KERNEL);
	if (!data)	{
		g_NT71890ProbeError = -ENOMEM;
		return -ENOMEM;
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))	{
		g_NT71890ProbeError = -EIO;
		goto exit_kfree;
	}

    memset(&g_hue_data, 0, sizeof(struct asus_pad_hsic));
    memset(&g_saturation_data, 0, sizeof(struct asus_pad_hsic));
    memset(&g_intensity_data, 0, sizeof(struct asus_pad_hsic));

	data->client = client;
	i2c_set_clientdata(client, data);
	mutex_init(&data->lock);

	//store i2c client structure
	NT71890_client = client;

    init_nt71890();

	g_NT71890ProbeError = 0;

#if defined(CONFIG_HAS_EARLYSUSPEND)
    register_early_suspend( &NT71890_early_suspend_desc );
#elif defined(CONFIG_FB)
    data->fb_notif.notifier_call = nvt_71890_fb_notifier_callback;
    err = fb_register_client(&data->fb_notif);
    if (err)
        dev_err(&client->dev, "Unable to register fb_notifier: %d\n", err);
#endif

#ifdef CONFIG_I2C_STRESS_TEST
    printk("[Display][NT71890] Add i2c test case\n");
    i2c_add_test_case(client, "P05LCM", ARRAY_AND_SIZE(DisplayTestCaseInfo));
#endif

	printk("[P05][CM]-- NT71890_probe\n");
	return 0;

exit_kfree:
	g_NT71890ProbeError = err;
	kfree(data);
	printk("[P05][CM] NT71890_probe fail !! : %d\n", err);
	return err;
}

static int __devexit NT71890_remove(struct i2c_client *client)
{
    struct NT71890_data *data = i2c_get_clientdata(client);

	kfree(i2c_get_clientdata(client));
#if defined(CONFIG_HAS_EARLYSUSPEND)
    unregister_early_suspend( &NT71890_early_suspend_desc );
#elif defined(CONFIG_FB)
    fb_unregister_client(&data->fb_notif);
#endif

	return 0;
}

static const struct i2c_device_id NT71890_id[] = {
	{ "nt71890", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, NT71890_id);

static const struct of_device_id nvt_cm_tcon_match[] = {
    {.compatible = "nvt,nt71890_cm"},
    {}
};

static struct i2c_driver NT71890_driver = {
	.driver = {
		.name	= "nt71890",
		.owner	= THIS_MODULE,
        .of_match_table = nvt_cm_tcon_match,
	},
	.probe	= NT71890_probe,
	.remove	= __devexit_p(NT71890_remove),
	.id_table = NT71890_id,
};

static int __init NT71890_init(void)
{
	int err = 0;
	printk("[P05][CM] NT71890_init+++\n");

	err = i2c_add_driver(&NT71890_driver);
	if (err != 0)	{
		printk("[P05][CM] load NT71890 driver failed, Error : %d\n",err);
		i2c_del_driver(&NT71890_driver);
		printk("[P05][CM] i2c_del_driver--\n");
	}
#ifdef CONFIG_EEPROM_NUVOTON 
    register_microp_notifier(&my_hs_notifier);
    notify_register_microp_notifier(&my_hs_notifier, "nt71890");
#endif
	printk("[P05][CM] NT71890_init---\n");
	return err; 
}

static void __exit NT71890_exit(void)
{
	i2c_del_driver(&NT71890_driver);
}

module_init(NT71890_init);
module_exit(NT71890_exit);

MODULE_DESCRIPTION("ASUS NT71890 TCON driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:board-8974");

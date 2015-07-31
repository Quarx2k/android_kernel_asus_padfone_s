/*
 * leds-msm-pmic.c - MSM PMIC LEDs driver.
 *
 * Copyright (c) 2009, Code Aurora Forum. All rights reserved.
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
//#include <mach/pmic.h>
#include <linux/pwm.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <linux/time.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <mach/board.h>
#include <linux/leds.h>
#include <mach/gpio.h>
#include <mach/irqs.h>
/*wait microp*/
#ifdef CONFIG_EEPROM_NUVOTON
#include <linux/microp_notify.h>
#include <linux/microp_notifier_controller.h>	//ASUS_BSP Lenter+
#include <linux/microp.h>
#include <linux/microp_api.h>
#include <linux/microp_pin_def.h>
#endif
/*wait microp*/
#include "../video/msm/mdss/mdss_dsi.h"  //ASUS_BSP: Louis


//#define A68_PWM_FREQ_HZ 22000L
//#define A68_PWM_PERIOD_USEC (USEC_PER_SEC / A68_PWM_FREQ_HZ)
//#define A68_PWM_LEVEL 255L
//#define A68_PWM_DUTY_LEVEL (A68_PWM_PERIOD_USEC * 2550L / A68_PWM_LEVEL)
//#define A80_PWM_FREQ_HZ 2        //195 , 9375 , 12500 , 18750
//#define PM8XXX_LPG_CTL_REGS		7
#define MAX_BACKLIGHT_BRIGHTNESS 255
#define MIN_BACKLIGHT_BRIGHTNESS 0
//#define PM8921_GPIO_BASE        NR_GPIO_IRQS
//#define PM8921_GPIO_PM_TO_SYS(pm_gpio)  (pm_gpio - 1 + PM8921_GPIO_BASE)

//DEFINE_SEMAPHORE(cabl_bl_sem);
//DECLARE_COMPLETION(brightness_comp);

void asus_set_bl_brightness(struct mdss_dsi_ctrl_pdata *ctrl, int value);
void backlight_IC_5V_Ctrl(int stat);
//static int a80_mipi_set_backlight(int);
//static int a80_set_pmic_backlight(int);
//static struct a86_backlight_data *backlight_pdata;
//static struct pwm_device *bl_lpm;
static int backlight_value = 90;
//static bool bl_busy = false;
//static int a80_pmic_set_pwm = 0;
//int pwm_frequency[] = {195,9375,12500,18750};

//ASUS_BSP jacob kung: add for debug mask ++
#include <linux/module.h>
/* Debug levels */
#define NO_DEBUG       0
#define DEBUG_POWER     1
#define DEBUG_INFO  2
#define DEBUG_VERBOSE 5
#define DEBUG_RAW      8
#define DEBUG_TRACE   10

static int debug = NO_DEBUG;

module_param(debug, int, 0644);

MODULE_PARM_DESC(debug, "Activate debugging output");

#define backlight_debug(level, ...) \
		if (debug >= (level)) \
			pr_info(__VA_ARGS__);
//ASUS_BSP jacob kung: add for debug mask --

//#ifdef ASUS_A91_PROJECT
#ifdef CONFIG_EEPROM_NUVOTON
int pad_set_backlight(int);	/*wait microp*/
extern int AX_MicroP_setPWMValue(uint8_t);	/*wait microp*/
static struct workqueue_struct *backlight_workqueue;
static struct delayed_work turn_off_panel_work;
#endif
static int backlight_previous_value = 5;
bool S5V_enable = 0;
int backlight_mode_state = 0;
enum backlight_mode {
    phone = 0,
    pad,
};
//#endif

// +++ ASUS_BSP: jacob kung
struct backlight_parameter {
		int min_level;
	    int max_level;
	    int bl_div;
};
struct a86_backlight_data {
		struct regulator *S5V_regulator;
	    int max_backlight_level;
	    int min_backlight_level;
	    int timaout_backlight_level;
	    int default_backlight_level;
		struct backlight_parameter normal_mod;
		struct backlight_parameter outdoor_mod;
		struct backlight_parameter auto_mod;
		struct backlight_parameter factory;
		struct backlight_parameter pad_normal_mod;
		struct backlight_parameter pad_outdoor_mod;
		struct backlight_parameter pad_auto_mod;
		//int *gpio;
}a86_backlight_pdata;
// --- ASUS_BSP: jacob kung

struct platform_device a86_led_device = {
    .name           = "a86-backlight",
    .id             = 0,
    .dev            = {
        .platform_data = &a86_backlight_pdata,
    },
};

#if 0
struct  pm8xxx_pwm_period define_pwm_parameter[] = {
		{PM_PWM_SIZE_9BIT,PM_PWM_CLK_19P2MHZ,PM_PWM_PDIV_6,5},//195Hz
		{PM_PWM_SIZE_9BIT,PM_PWM_CLK_19P2MHZ,PM_PWM_PDIV_2,1},//9.375KHz
		{PM_PWM_SIZE_9BIT,PM_PWM_CLK_19P2MHZ,PM_PWM_PDIV_3,0},//12.5KHz
		{PM_PWM_SIZE_9BIT,PM_PWM_CLK_19P2MHZ,PM_PWM_PDIV_2,0},//18.75 KHz
	};
struct pwm_device {
	int			pwm_id;		/* = bank/channel id */
	int			in_use;
	const char		*label;
	struct pm8xxx_pwm_period	period;
	int			pwm_value;
	int			pwm_period;
	int			pwm_duty;
	u8			pwm_lpg_ctl[PM8XXX_LPG_CTL_REGS];
	u8			pwm_ctl1;
	u8			pwm_ctl2;
	int			irq;
	struct pm8xxx_pwm_chip	*chip;
	int			bypass_lut;
	int			dtest_mode_supported;
};
enum brightness_mode {
    NORMAL = 0,
    AUTO,
    OUTDOOR,
};
void cabl_bl_scale(int cabl_scale, int bl_min_lvl, int mode)
{
    int new_level = 0, index;
    if (backlight_value >= bl_min_lvl) 
    {
        switch (mode) {
            case NORMAL:
                index = 0;
                new_level = backlight_value * cabl_scale / 1000;
                if (new_level < index + bl_min_lvl) {
                    new_level = 30;
                }
            break;
            case AUTO:
                index = 2000;
                new_level = (backlight_value - index) * cabl_scale / 1000 + index;
                if (new_level < index + bl_min_lvl) {
                    new_level = 2030;
                }
            break;
            case OUTDOOR:
                index = 1000;
                new_level = (backlight_value - index) * cabl_scale / 1000 + index;
                if (new_level < index + bl_min_lvl) {
                    new_level = 1030;
                }
            break;
            default:
                return;
        }
        down(&cabl_bl_sem);
        if (new_level == backlight_value) {
            up(&cabl_bl_sem);
            return;
        }
        printk(KERN_DEBUG"[BL][cabl] mode(%d), scale(%d), new_level(%d), old_level(%d)\n",mode ,cabl_scale , new_level, backlight_value);
        if (bl_busy) {
            wait_for_completion_timeout(&brightness_comp, msecs_to_jiffies(500));
        }
		if (g_A68_hwID <= A80_SR1)
        	a80_mipi_set_backlight(new_level);
		else if (A80_SR2 <= g_A68_hwID )
			a80_set_pmic_backlight(new_level);
        up(&cabl_bl_sem);
    }
}
EXPORT_SYMBOL(cabl_bl_scale);
static int a80_set_pmic_backlight(value)
{
	int ret = 0;
	int index =  0;
	struct pm8xxx_pwm_period *pwm_period;
	bl_busy = TRUE;
	if (bl_lpm) 
		{
		if (!a80_pmic_set_pwm)
			{
			printk("[BL]Set pmic PWM frequency+++\n");
			pwm_period = &bl_lpm->period;
			pwm_period->pwm_size = define_pwm_parameter[A80_PWM_FREQ_HZ].pwm_size;
			pwm_period->clk = define_pwm_parameter[A80_PWM_FREQ_HZ].clk;
			pwm_period->pre_div = define_pwm_parameter[A80_PWM_FREQ_HZ].pre_div;
			pwm_period->pre_div_exp = define_pwm_parameter[A80_PWM_FREQ_HZ].pre_div_exp;
			bl_lpm->pwm_period = pwm_frequency[A80_PWM_FREQ_HZ];
			
			ret = pm8xxx_pwm_config_period(bl_lpm,pwm_period);
			if (ret) 
				{
				pr_err("pwm_config on config_period failed %d\n", ret);
               			 return ret;
				}
			
			printk("[BL]Set pmic PWM frequency---\n");
			a80_pmic_set_pwm = 1;
			}
//=========================================================================
		printk(KERN_DEBUG"[BL]Set pmic backlight value == %d\n",value);
		 if(value >= 2000 && value <= 2255) {
       			 value -= 2000;
   		 }
#ifdef ASUS_FACTORY_BUILD  
		index = ((value - 20) * 533)/255 + 20;
            	//20~255 mapping 21~511
            	if (index >= 511) {
                	index = 511;
            	}
            	else if (index <= 0) {
                	index = 0;
            	}
#else
		if (value <= 255) {  //normal mode
			index = ((value - 20) * 319)/ 235 + 21;
			printk(KERN_DEBUG"[BL]Set pmic backlight normal index == %d\n",index);
                	//20~255 mapping 21~340; max: ? nits, min: ? nits, default: ? nits( ? %)
			 if (index >= 340) {
                    	index = 340;
                	}
                	else if (index <= 0) {
                    	index = 0;
                	}	
		}
		else if (value > 1000 && value <= 1255) //outdoor mode
        	{
                	index = ((value - 1020) * 381)/ 235 + 130;
			//1020~1255 mapping 130~511; max: ? nits, min: ? nits( ? %)
					printk(KERN_DEBUG"[BL]Set pmic backlight outdoor index == %d\n",index);
                	if (index >= 511) {
                		index = 511;
                	}
                	else if (index <= 0) {
                		index = 0;
                	}
		}
#endif
//=========================================================================
			printk(KERN_DEBUG"[BL]Set pmic PWM value == %d \n",index);
	        ret = pm8xxx_pwm_config_pwm_value(bl_lpm,index);
		if (ret) 
			{
			pr_err("pwm_config on lpm failed %d\n", ret);
			return ret;
			}
		if (index) 
			{
			ret = pwm_enable(bl_lpm);
			if (ret)
				pr_err("pwm enable/disable on lpm failed"
	                        "for bl =(%d)\n",  value);
			} 
		else{
			pwm_disable(bl_lpm);
	        }
		}
	complete(&brightness_comp);
    bl_busy = FALSE;
    return 0;
}
#endif
//#ifdef ASUS_A91_PROJECT
//a86+++
//+++ ASUS BSP jacob for S5V control
void backlight_IC_5V_Ctrl(int stat)
{
	struct a86_backlight_data *pdata = a86_led_device.dev.platform_data;
	int ret = 0;
		printk("[BL] ### backlight_IC_5V_Ctrl = %d ###\n", stat);
		if(stat) {
			ret = regulator_enable(pdata->S5V_regulator);
			if (ret == 0){
				S5V_enable = 1;
				printk("[BL] 5V is enabled\n");
				}
			else
				printk("[BL] 5V enabled failed %d \n",ret);
			}
		else{
			ret = regulator_disable(pdata->S5V_regulator);
			if (ret == 0){
				S5V_enable = 0;
				printk("[BL] 5V is disabled\n");
				}
			else
				printk("[BL] 5V disable failed %d \n",ret);
			}
}
//--- ASUS BSP jacob for S5V control
static int a86_set_backlight(struct mdss_dsi_ctrl_pdata *ctrl, int value)
{
	struct a86_backlight_data *pdata = a86_led_device.dev.platform_data;
    int index = 0;

    if (value == 0) {
        printk("[BL] %s turn off Phone backlight\n",__func__);
        asus_set_brightness(ctrl,0);
		if (S5V_enable == 1 && g_ASUS_hwID == A90_EVB0)
			backlight_IC_5V_Ctrl(0);
        return 0;
    }

    if (value == pdata->timaout_backlight_level) {
        printk("[BL] %s Auto dim backlight value\n",__func__);
        index = pdata->timaout_backlight_level;
    }
    else if (value >= 0 && value <= 255){
        		index = value;
        		if (index > pdata->factory.max_level) {
            			index = pdata->factory.max_level;
        			}
				else if(index <= pdata->factory.min_level) {
						index = pdata->factory.min_level;
					}
    	}
	else
		index = pdata->default_backlight_level;//value not in spec, do set default value
#if 0
    //bl_busy = TRUE; //not use, just for cabl
#ifdef ASUS_FACTORY_BUILD  
		backlight_debug(DEBUG_INFO,"[BL] %s Factory backlight setting !!\n",__func__);
 		if (value >= 2000 && value <= 2255) 
				value = value - 2000;
		if (value >= 0 && value <= 255){
        		index = ((value - 20) * 235)/pdata->factory.bl_div + pdata->factory.min_level;
        		if (index > pdata->factory.max_level) {
            			index = pdata->factory.max_level;
        			}
				else if(index <= pdata->factory.min_level) {
						index = pdata->factory.min_level;
					}
			}
		else 
			index = pdata->default_backlight_level;
#else
	    if (value <= 255) { //normal mode
			backlight_debug(DEBUG_INFO,"[BL] %s normal mode !!\n",__func__);
			index = ((value - 20) * 235)/pdata->normal_mod.bl_div + pdata->normal_mod.min_level;
			if (index >= pdata->normal_mod.max_level)
				index = pdata->normal_mod.max_level;
			else if (index <= pdata->normal_mod.min_level)
				index = pdata->normal_mod.min_level;
		}
        else if (value > 1000 && value <= 1255) {
			backlight_debug(DEBUG_INFO,"[BL] %s outdoor mode !!\n",__func__);
            index = ((value - 1020) * 235)/pdata->outdoor_mod.bl_div + pdata->outdoor_mod.min_level;
		    if (index > pdata->outdoor_mod.max_level)
                index = pdata->outdoor_mod.max_level;
		    else if (index <= pdata->outdoor_mod.min_level)
			    index = pdata->outdoor_mod.min_level;
        }
		else if (value > 2000 && value <= 2255) {
			backlight_debug(DEBUG_INFO,"[BL] %s autodoor mode !!\n",__func__);
            index = ((value - 2020) * 235)/pdata->auto_mod.bl_div + pdata->auto_mod.min_level;
		    if (index > pdata->auto_mod.max_level)
                index = pdata->auto_mod.max_level;
		    else if (index <= pdata->auto_mod.min_level)
			    index = pdata->auto_mod.min_level;
        }
#endif
	    else
            index = pdata->default_backlight_level;//value not in spec, do set default value
    }
#endif
    asus_set_brightness(ctrl, index);
    if((index > pdata->timaout_backlight_level) & (backlight_previous_value != 0) & ( (backlight_previous_value == 5) ||(abs(backlight_previous_value - index) >= 30))){
		backlight_previous_value = index;
    	}


    //complete(&brightness_comp); //not use, just for cabl
    //bl_busy = FALSE; //not use, just for cabl


    return 0;
}
//a80---
//#endif
#ifdef ASUS_ME771KL_PROJECT
//ME771KL+++
static int ME771KL_set_backlight(struct mdss_dsi_ctrl_pdata *ctrl,int value)
{
	struct a86_backlight_data *pdata = a86_led_device.dev.platform_data;
    int index = 0;

    if (value == 0) {
        printk("[BL] %s turn off Phone backlight\n",__func__);
        asus_set_brightness(ctrl,0);
        return 0;
    }

    if (value == pdata->timaout_backlight_level) {
        printk("[BL] %s Auto dim backlight value\n",__func__);
        index = pdata->timaout_backlight_level;
    }
    else if (value >= 0 && value <= 255){
			index = value;
        	if (index > pdata->factory.max_level) {
            		index = pdata->factory.max_level;
        		}
			else if(index <= pdata->factory.min_level) {
					index = pdata->factory.min_level;
				}
    	}
    //bl_busy = TRUE; //not use, just for cabl
#if 0
#ifdef ASUS_FACTORY_BUILD  
		backlight_debug(DEBUG_INFO,"[BL] %s Factory backlight setting !!\n",__func__);
 		if (value >= 2000 && value <= 2255) 
				value = value - 2000;
		if (value >= 0 && value <= 255){
        		index = ((value - 20) * 235)/pdata->factory.bl_div + pdata->factory.min_level;
        		if (index > pdata->factory.max_level) {
            			index = pdata->factory.max_level;
        			}
				else if(index <= pdata->factory.min_level) {
						index = pdata->factory.min_level;
					}
			}
		else 
			index = pdata->default_backlight_level;
#else
	    if (value <= 255) { //normal mode
			backlight_debug(DEBUG_INFO,"[BL] %s normal mode !!\n",__func__);
			index = ((value - 20) * 235)/pdata->normal_mod.bl_div + pdata->normal_mod.min_level;
			if (index >= pdata->normal_mod.max_level)
				index = pdata->normal_mod.max_level;
			else if (index <= pdata->normal_mod.min_level)
				index = pdata->normal_mod.min_level;
		}
        else if (value > 1000 && value <= 1255) {
			backlight_debug(DEBUG_INFO,"[BL] %s outdoor mode !!\n",__func__);
            index = ((value - 1020) * 235)/pdata->outdoor_mod.bl_div + pdata->outdoor_mod.min_level;
		    if (index > pdata->outdoor_mod.max_level)
                index = pdata->outdoor_mod.max_level;
		    else if (index <= pdata->outdoor_mod.min_level)
			    index = pdata->outdoor_mod.min_level;
        }
		else if (value > 2000 && value <= 2255) {
			backlight_debug(DEBUG_INFO,"[BL] %s autodoor mode !!\n",__func__);
            index = ((value - 2020) * 235)/pdata->auto_mod.bl_div + pdata->auto_mod.min_level;
		    if (index > pdata->auto_mod.max_level)
                index = pdata->auto_mod.max_level;
		    else if (index <= pdata->auto_mod.min_level)
			    index = pdata->auto_mod.min_level;
        }
	    else
            index = pdata->default_backlight_level;//value not in spec, do set default value
#endif
    }
#endif

    asus_set_brightness(ctrl,index);
    if((index > pdata->timaout_backlight_level) & (backlight_previous_value != 0) & ( (backlight_previous_value == 5) ||(abs(backlight_previous_value - index) >= 30))){
		backlight_previous_value = index;
    	}

    //complete(&brightness_comp); //not use, just for cabl
    //bl_busy = FALSE; //not use, just for cabl
    return 0;
}
//ME771KL---
#endif

//a68++++
#if 0
static int a68_set_backlight(value)
{
    int ret, duty_us;
    int index = 0;
    static bool CABC_On = true;
    if(value >= 2000 && value <= 2255) {
        value -= 2000;
    }
    if (g_A68_hwID >= A68_SR2)  //driver ic support
    {
        if (value == 0 || value == 1000) {
            printk("[BL] turn off A68 backlight\n");
            sharp_set_brightness(0);
            return 0;
        }
#ifdef ASUS_FACTORY_BUILD
        if (g_A68_hwID == A68_SR2) 
        {
            index = ((value - 20) * ((153*10000)/ 235)) / 10000 + 8;
            //factory: 20~550nits
            if (index >= 160) {
                index = 160;
            }
            else if (index <= 0) {
                index = 0;
            }
        }
        else    //for ER
        {
            index = ((value - 20) * ((245*10000)/ 235)) / 10000 + 11;
            //20~255 mapping 11~255
            if (index >= 255) {
                index = 255;
            }
            else if (index <= 0) {
                index = 0;
            }
        }
#else
        if (value <= 255)   //normal mode
        {
            if (g_A68_hwID == A68_SR2) 
            {
                index = ((value - 20) * ((80*10000)/ 235)) / 10000 + 8;
                //20~255 mapping 8~87; max:300nits, min:20nits (3%), default:100nits (12%)
                if (index >= 87) {
                    index = 87;
                }
                else if (index <= 0) {
                    index = 0;
                }
            }
            else 
            {
                index = ((value - 20) * ((125*10000)/ 235)) / 10000 + 11;
                //20~255 mapping 11~135; max:300nits, min:20nits, default:100nits(17%)
                if (index >= 135) {
                    index = 135;
                }
                else if (index <= 0) {
                    index = 0;
                }
            }
        }
        else if (value > 1000 && value <= 1255) //outdoor mode
        {
            if (g_A68_hwID == A68_SR2) 
            {
                index = ((value - 1020) * ((120*10000)/ 235)) / 10000 + 41;
                //1020~1255 mapping 41~160; max:550nits, min:140nits(16%)
                if (index >= 160) {
                    index = 160;
                }
                else if (index <= 0) {
                    index = 0;
                }
            }
            else
            {
                index = ((value - 1020) * ((192*10000)/ 235)) / 10000 + 64;
                //1020~1255 mapping 64~255; max:550nits, min:140nits(25%)
                if (index >= 255) {
                    index = 255;
                }
                else if (index <= 0) {
                    index = 0;
                }
            }
        }
#endif
// ++ cabc on/off
        if (index <= 28 && CABC_On == true)
        {
            sharp_set_cabc(0);
            CABC_On = false;
        }
        else if (index > 28 && CABC_On == false)
        {
            sharp_set_cabc(3);
            CABC_On = true;
        }
// -- cabc on/off
        sharp_set_brightness(index);
    }
    else  //pmic support
    {
        if (bl_lpm) 
        {
            if(value == MAX_BACKLIGHT_BRIGHTNESS) {
                duty_us = A68_PWM_PERIOD_USEC;
                ret = pwm_config(bl_lpm, 
                        duty_us, A68_PWM_PERIOD_USEC);
            }
            else {
                ret = pwm_config(bl_lpm, 
                        A68_PWM_DUTY_LEVEL * (long) value / 2550L, A68_PWM_PERIOD_USEC);
            }
            if (ret) {
                pr_err("pwm_config on lpm failed %d\n", ret);
                return ret;
            }
            if (value) {
                ret = pwm_enable(bl_lpm);
                if (ret)
                    pr_err("pwm enable/disable on lpm failed"
                        "for bl =(%d)\n",  value);
            } else {
                pwm_disable(bl_lpm);
            }
        }
    }
    return 0;
}
#endif
//a68----
//#ifdef ASUS_A91_PROJECT
#ifdef CONFIG_EEPROM_NUVOTON
//extern int is_pad_power_off;
int pad_set_backlight(int value)
{
	struct a86_backlight_data *pdata = a86_led_device.dev.platform_data;
    int ret, index = 0;
    static int previous_value = 0;
    if (value == 0) {
        printk("[BL] turn off Pad backlight\n");
        AX_MicroP_setPWMValue(0);
        queue_delayed_work(backlight_workqueue, &turn_off_panel_work, msecs_to_jiffies(3000));
        previous_value = 0;
        return 0;
    	}
//ASUS BSP Wei +++
//     if(is_pad_power_off){
//	 	printk("[BL] pad mode shutdowning don't set pad backlight\n");
//		return 0;
//     }
//ASUS BSP Wei ---	 
     if (value == pdata->timaout_backlight_level) {
        printk("[BL] %s Auto dim backlight value\n",__func__);
        index = pdata->timaout_backlight_level;
     	}
    else if (value >= 0 && value <= 255){
    	backlight_debug(DEBUG_INFO,"[BL] %s set backlight !!\n",__func__);
        index = ((value - pdata->pad_normal_mod.min_level) * 235) / pdata->pad_normal_mod.bl_div + pdata->pad_normal_mod.min_level;
        //17~255 mapping 17~250;

        if (index >= pdata->pad_normal_mod.max_level) {
            index = pdata->pad_normal_mod.max_level;
        }
        else if (index <= pdata->pad_normal_mod.min_level) {
            index = pdata->pad_normal_mod.min_level;
        }
    }
#if 0
    if (value > 2000 && value <= 2255)  //auto mode 30~300nits
    {
        value -= 2000;
        index = ((value - 10) * 235) / pdata->pad_auto_mod.bl_div + pdata->pad_auto_mod.min_level;
        if (index >= pdata->pad_auto_mod.max_level) {
            index = pdata->pad_auto_mod.max_level;
        }
        else if (index <= pdata->pad_auto_mod.min_level) {
            index = pdata->pad_auto_mod.min_level;
        }
    }
    if (value <= 255)   //normal mode
    {
        index = ((value - 10) * 235) / pdata->pad_normal_mod.bl_div + pdata->pad_normal_mod.min_level;
        //20~255 mapping 18~154; max:180nits, min:20nits (7%), default:100nits (33%)
        if (index >= pdata->pad_normal_mod.max_level) {
            index = pdata->pad_normal_mod.max_level;
        }
        else if (index <= pdata->pad_normal_mod.min_level) {
            index = pdata->pad_normal_mod.min_level;
        }
    }
    else if (value > 1000 && value <= 1255) //outdoor mode
    {
        index = ((value - 1010) * 235) / pdata->pad_outdoor_mod.bl_div + pdata->pad_outdoor_mod.min_level;
        //1020~1255 mapping 118~255; max:300nits, min:140nits (46%)
        if (index >= pdata->pad_outdoor_mod.max_level) {
            index = pdata->pad_outdoor_mod.max_level;
        }
        else if (index <= pdata->pad_outdoor_mod.min_level) {
            index = pdata->pad_outdoor_mod.min_level;
        }
    }
#endif
	else
		index = pdata->pad_outdoor_mod.min_level;
	backlight_debug(DEBUG_INFO,"[BL] set pad backlight index = %d\n",index);

	ret = AX_MicroP_setPWMValue(index);
	if((index > pdata->timaout_backlight_level) & (backlight_previous_value != 0) & ((backlight_previous_value == 5) ||(abs(backlight_previous_value - index) >= 30))){
			backlight_previous_value = index;
		}

    if (ret < 0) {
        printk("(%s): P03 set backlight fail\n", __func__);
    }

    if ((previous_value == 0) && (index > 0))
    {
        if (delayed_work_pending(&turn_off_panel_work))
        {
            cancel_delayed_work_sync(&turn_off_panel_work);
        }
        AX_MicroP_setGPIOOutputPin(OUT_uP_LCD_EN,1);
        printk("[BL] (%s): P03 turn on panel\n", __func__);
    }

    previous_value = index;

    return ret;
}
#endif
//#endif

//static int lcd_backlight_registered;
void asus_set_bl_brightness(struct mdss_dsi_ctrl_pdata *ctrl, int value)
{
	 //if (g_ASUS_hwID <= A86_EVB)
	 	//backlight_debug(DEBUG_INFO,"[BL] asus_set_bl_brightness Set backlight value == %d \n",value);
/*
    static bool bFirst = true;
    int rc;
    struct pm_gpio gpio26_param = {
        .direction = PM_GPIO_DIR_OUT,
        .output_buffer = PM_GPIO_OUT_BUF_CMOS,
        .output_value = 0,
        .pull = PM_GPIO_PULL_NO,
        .vin_sel = 2,
        .out_strength = PM_GPIO_STRENGTH_HIGH,
        .function = PM_GPIO_FUNC_2,
        .inv_int_pol = 0,
        .disable_pin = 0,
    };
    if (g_ASUS_hwID < A86_EVB)
    {
        if(bFirst) 
        {
            rc = pm8xxx_gpio_config(PM8921_GPIO_PM_TO_SYS(26), &gpio26_param);
            if (rc) {
                pr_err("gpio_config 26 failed (2), rc=%d\n", rc);
                return;
            }
            bFirst = false;
        }
    }
*/
    backlight_value = value;
//#ifdef ASUS_A91_PROJECT
        if (backlight_mode_state == phone) {
		    backlight_debug(DEBUG_INFO,"[BL] (%s): a86_set_backlight %d \n", __func__,backlight_value);
            a86_set_backlight(ctrl, backlight_value);
        }
#ifdef CONFIG_EEPROM_NUVOTON
        else if (backlight_mode_state == pad) {
		    backlight_debug(DEBUG_INFO,"[BL] (%s): pad_set_backlight %d \n", __func__,backlight_value);
            pad_set_backlight(backlight_value);
        }
#endif    
//#endif

#ifdef ASUS_ME771KL_PROJECT
	backlight_debug(DEBUG_INFO,"[BL] (%s): a86_set_backlight %d \n", __func__,backlight_value);
	ME771KL_set_backlight(ctrl,backlight_value);
#endif
/*
    else if (g_A68_hwID >= A80_EVB && g_A68_hwID <= A80_SR1) 
    {
        if (backlight_mode_state == phone) {
            a80_mipi_set_backlight(backlight_value);
        }
        else if (backlight_mode_state == pad) {
			pad_set_backlight(backlight_value);
        }
    }
	else 
    {
    	if(bFirst) 
        {
            rc = pm8xxx_gpio_config(PM8921_GPIO_PM_TO_SYS(26), &gpio26_param);
            if (rc) {
                pr_err("gpio_config 26 failed (2), rc=%d\n", rc);
                return;
            }
            bFirst = false;
        }
        if (backlight_mode_state == phone) {
            a80_set_pmic_backlight(backlight_value);
        }
        else if (backlight_mode_state == pad) {
			pad_set_backlight(backlight_value);
        }
    }
*/
}
//#ifdef ASUS_A91_PROJECT
#ifdef CONFIG_EEPROM_NUVOTON
static void set_p03_panel_off_func(struct work_struct *work)
{
    printk("[BL] (%s): P03 turn off panel\n", __func__);
    AX_MicroP_setGPIOOutputPin(OUT_uP_LCD_EN,0);
    AX_MicroP_setGPIOOutputPin(OUT_uP_PAD_LOW_BAT,0);
}
#endif
//#endif

/*
static struct led_classdev a86_backlight_led = {
    .name       = "a86-backlight",
    .brightness = MAX_BACKLIGHT_BRIGHTNESS,
    //.brightness_set = asus_set_bl_brightness,
};
*/
static int asus_bl_parse_dt(struct platform_device *pdev,struct a86_backlight_data *pdata)
{
	struct device_node *np = pdev->dev.of_node;
	u32 res[6];
	int rc = 0;
//	pars max/min BL level
	rc = of_property_read_u32_array(np, "pan-backlight-levels", res, 2);
	if (rc) {
		pr_err("%s:%d, backlight-level not specified\n",__func__, __LINE__);
		return -EINVAL;
	}
	pdata->min_backlight_level = (!rc ? res[0] : 1);
	pdata->max_backlight_level = (!rc ? res[1] : 255);
	printk("[BL]pdata->min_backlight_level = %d\n",pdata->min_backlight_level);
	printk("[BL]pdata->max_backlight_level = %d\n",pdata->max_backlight_level);
	
//	pars timeout/default BL level
	rc = of_property_read_u32_array(np, "bl-tout-defau-levels", res, 2);
	if (rc) {
		pr_err("%s:%d, bl-tout-defau-level not specified\n",__func__, __LINE__);
		return -EINVAL;
	}
	pdata->timaout_backlight_level = (!rc ? res[0] : 1);
	pdata->default_backlight_level = (!rc ? res[1] : 255);
	printk("[BL]pdata->timaout_backlight_level = %d\n",pdata->timaout_backlight_level);
	printk("[BL]pdata->default_backlight_level = %d\n",pdata->default_backlight_level);
#if 0
//	pars normal mode BL level settint
	rc = of_property_read_u32_array(np, "normod-bl-param", res, 3);
	if (rc) {
		pr_err("%s:%d, bl-tout-defau-level not specified\n",__func__, __LINE__);
		return -EINVAL;
	}
	pdata->normal_mod.min_level = (!rc ? res[0] : 1);
	pdata->normal_mod.max_level = (!rc ? res[1] : 255);
	pdata->normal_mod.bl_div = (!rc ? res[2] : 255);
	printk("[BL]pdata->normod.min_level = %d\n",pdata->normal_mod.min_level);
	printk("[BL]pdata->normod.max_level = %d\n",pdata->normal_mod.max_level);
	printk("[BL]pdata->normod.bl_div = %d\n",pdata->normal_mod.bl_div);
//	pars outdoor mode BL level settint
	rc = of_property_read_u32_array(np, "outmod-bl-param", res, 3);
	if (rc) {
		pr_err("%s:%d, bl-tout-defau-level not specified\n",__func__, __LINE__);
		return -EINVAL;
	}
	pdata->outdoor_mod.min_level = (!rc ? res[0] : 1);
	pdata->outdoor_mod.max_level = (!rc ? res[1] : 255);
	pdata->outdoor_mod.bl_div = (!rc ? res[2] : 255);
	printk("[BL]pdata->outmod.min_level = %d\n",pdata->outdoor_mod.min_level);
	printk("[BL]pdata->outmod.max_level = %d\n",pdata->outdoor_mod.max_level);
	printk("[BL]pdata->outmod.bl_div = %d\n",pdata->outdoor_mod.bl_div);
	
//	pars auotmode BL level settint
	rc = of_property_read_u32_array(np, "autmod-bl-param", res, 3);
	if (rc) {
		pr_err("%s:%d, bl-tout-defau-level not specified\n",__func__, __LINE__);
		return -EINVAL;
	}
	pdata->auto_mod.min_level = (!rc ? res[0] : 1);
	pdata->auto_mod.max_level = (!rc ? res[1] : 255);
	pdata->auto_mod.bl_div = (!rc ? res[2] : 255);
	printk("[BL]pdata->autmod.min_level = %d\n",pdata->auto_mod.min_level);
	printk("[BL]pdata->autmod.max_level = %d\n",pdata->auto_mod.max_level);
	printk("[BL]pdata->autmod.bl_div = %d\n",pdata->auto_mod.bl_div);
#endif
//	pars Factory BL level settint
	rc = of_property_read_u32_array(np, "factory-bl-param", res, 3);
	if (rc) {
		pr_err("%s:%d, bl-tout-defau-level not specified\n",__func__, __LINE__);
		return -EINVAL;
	}
	pdata->factory.min_level = (!rc ? res[0] : 1);
	pdata->factory.max_level = (!rc ? res[1] : 255);
	pdata->factory.bl_div = (!rc ? res[2] : 255);
	printk("[BL]pdata->factory.min_level = %d\n",pdata->factory.min_level);
	printk("[BL]pdata->factory.max_level = %d\n",pdata->factory.max_level);
	printk("[BL]pdata->factory.bl_div = %d\n",pdata->factory.bl_div);
//#ifdef ASUS_A91_PROJECT
//	pars Pad_normal BL level settint
	rc = of_property_read_u32_array(np, "pad-normod-bl-param", res, 3);
	if (rc) {
		pr_err("%s:%d, bl-tout-defau-level not specified\n",__func__, __LINE__);
		return -EINVAL;
	}
	pdata->pad_normal_mod.min_level = (!rc ? res[0] : 1);
	pdata->pad_normal_mod.max_level = (!rc ? res[1] : 255);
	pdata->pad_normal_mod.bl_div = (!rc ? res[2] : 255);
	printk("[BL]pdata->pad_normal_mod.min_level = %d\n",pdata->pad_normal_mod.min_level);
	printk("[BL]pdata->pad_normal_mod.max_level = %d\n",pdata->pad_normal_mod.max_level);
	printk("[BL]pdata->pad_normal_mod.bl_div = %d\n",pdata->pad_normal_mod.bl_div);
#if 0
//	pars Pad_outdoor BL level settint
	rc = of_property_read_u32_array(np, "pad-outmod-bl-param", res, 3);
	if (rc) {
		pr_err("%s:%d, bl-tout-defau-level not specified\n",__func__, __LINE__);
		return -EINVAL;
	}
	pdata->pad_outdoor_mod.min_level = (!rc ? res[0] : 1);
	pdata->pad_outdoor_mod.max_level = (!rc ? res[1] : 255);
	pdata->pad_outdoor_mod.bl_div = (!rc ? res[2] : 255);
	printk("[BL]pdata->pad_outdoor_mod.min_level = %d\n",pdata->pad_outdoor_mod.min_level);
	printk("[BL]pdata->pad_outdoor_mod.max_level = %d\n",pdata->pad_outdoor_mod.max_level);
	printk("[BL]pdata->pad_outdoor_mod.bl_div = %d\n",pdata->pad_outdoor_mod.bl_div);
//	pars Pad_outdoor BL level settint
	rc = of_property_read_u32_array(np, "pad-autmod-bl-param", res, 3);
	if (rc) {
		pr_err("%s:%d, bl-tout-defau-level not specified\n",__func__, __LINE__);
		return -EINVAL;
	}
	pdata->pad_auto_mod.min_level = (!rc ? res[0] : 1);
	pdata->pad_auto_mod.max_level = (!rc ? res[1] : 255);
	pdata->pad_auto_mod.bl_div = (!rc ? res[2] : 255);
	printk("[BL]pdata->pad_auto_mod.min_level = %d\n",pdata->pad_auto_mod.min_level);
	printk("[BL]pdata->pad_auto_mod.max_level = %d\n",pdata->pad_auto_mod.max_level);
	printk("[BL]pdata->pad_auto_mod.bl_div = %d\n",pdata->pad_auto_mod.bl_div);
#endif

//#endif
	return rc;
}

static int a86_backlight_probe(struct platform_device *pdev)
{
	int ret =0;
	int rc =0;
//	u32 res[6];
    struct a86_backlight_data *pdata;
	//struct device_node *np = pdev->dev.of_node;
	//a86_led_device = kzalloc(sizeof(struct platform_device), GFP_KERNEL);
	//if (!a86_led_device) {
		//printk("%s: failed to allocate driver data\n", __func__);
		//ret = -ENOMEM;
		//goto exit;
	//}
	pdata = kzalloc(sizeof(struct a86_backlight_data), GFP_KERNEL);
	if(!pdata){
		printk("%s: failed to allocate device data\n", __func__);
		ret = -EINVAL;
		goto exit;
	}

	 printk( "[BL]a86_backlight_probe+++\n");            

    if (pdata == NULL) {
        pr_err("%s.invalid platform data.\n", __func__);
        ret = -ENODEV;
		goto exit;
	}

	rc = asus_bl_parse_dt(pdev, pdata);
	if (rc)
		return rc;
//#ifdef ASUS_A91_PROJECT
	//to get regulator S5v  BOOST-5V
	pdata->S5V_regulator = devm_regulator_get(&pdev->dev, "BL_5V");
	if (IS_ERR( pdata->S5V_regulator)) {
		ret = PTR_ERR(pdata->S5V_regulator);
		printk("Regulator get 5v failed rc=%d\n", ret);
		}
	//backlight_IC_5V_Ctrl(1);
#ifdef CONFIG_EEPROM_NUVOTON
    backlight_workqueue  = create_singlethread_workqueue("P03BACKLIGHTWORKQUEUE");
    INIT_DELAYED_WORK(&turn_off_panel_work, set_p03_panel_off_func);
#endif
//#endif
	memcpy(&a86_led_device.dev.platform_data, &pdata, sizeof(pdata));

//    if (backlight_pdata != NULL) {
//        bl_lpm = pwm_request(backlight_pdata->gpio[0],
//            "backlight");
//   }

//    if (bl_lpm == NULL || IS_ERR(bl_lpm)) {
//        pr_err("%s pwm_request() failed\n", __func__);
 //       bl_lpm = NULL;
 //  }
//    pr_debug("bl_lpm = %p lpm = %d\n", bl_lpm,
//        backlight_pdata->gpio[0]);

/*
    if (led_classdev_register(&pdev->dev, &a86_backlight_led))
    {
        printk("[BL]led_classdev_register failed\n");            
    }
    else
    {
    	printk( "[BL]a86_backlight_probe----OK\n");    
        lcd_backlight_registered = 1;
    }
*/    

		         
exit:
	DEV_DBG("[BL]a86_backlight_probe----failed\n");
	return ret;
}

static int a86_backlight_remove(struct platform_device *pdev)
{
/*
    if (lcd_backlight_registered) {
        lcd_backlight_registered = 0;
        led_classdev_unregister(&a86_backlight_led);
    }
    */
    return 0;
}
// not use/use in led-class.c+++
/*
static int change_backlight_mode(struct notifier_block *this, unsigned long event, void *ptr)
{
        switch (event) {
        case P01_ADD:
                backlight_mode_state = pad;
                //p03_set_backlight(backlight_value);   //plug-in/out set bl from led-class
                return NOTIFY_DONE;
        case P01_REMOVE:
                backlight_mode_state = phone;
                //a68_set_backlight(backlight_value);   //plug-in/out set bl from led-class
                return NOTIFY_DONE;
        default:
                return NOTIFY_DONE;
        }
}
static struct notifier_block my_hs_notifier = {
        .notifier_call = change_backlight_mode,
        .priority = VIBRATOR_MP_NOTIFY,
};
*/
// not use/use in led-class.c----
static const struct of_device_id BL_dt_match[] = {
	{ .compatible = "qcom,A86-backlight",},
	{}
};
static struct platform_driver this_driver = {
    .probe  = a86_backlight_probe,
    .remove = a86_backlight_remove,
    .driver = {
        .name   = "a86-backlight",
		.of_match_table = BL_dt_match,
    },
};

static int __init msm_pmic_led_init(void)
{
    //register_microp_notifier(&my_hs_notifier);
    //notify_register_microp_notifier(&my_hs_notifier, "a68_backlight"); //ASUS_BSP Lenter+
    printk("[BL]Backlight inits\n");
    return platform_driver_register(&this_driver);
}
module_init(msm_pmic_led_init);

static void __exit msm_pmic_led_exit(void)
{
    platform_driver_unregister(&this_driver);
}
module_exit(msm_pmic_led_exit);

MODULE_DESCRIPTION("MSM PMIC8921 A68 backlight driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:board-8064");

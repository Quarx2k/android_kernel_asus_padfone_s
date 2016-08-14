/*
 * eeprom_nuvoton.c - driver for EEPROM of Nuvoton
 *
 * Copyright (C) 2011 Sina Chou <sina_chou@asus.com>
 *
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/microp.h>
#include <linux/microp_notify.h>
#include <linux/microp_pin_def.h>
#include <linux/uaccess.h>
#include <linux/microp_api.h>
#include <linux/switch.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <linux/proc_fs.h>
#include <linux/asusdebug.h>
#include <linux/of_gpio.h>

#ifdef CONFIG_FASTBOOT
#include <linux/fastboot.h>
#endif //#ifdef CONFIG_FASTBOOT

#define TEGRA_MICROP_NAME "microp"

#if 0
static char rf_switch_status[10];
static int g_gpio_rf_sw_1=-1;
static int g_gpio_rf_sw_2=-1;

#endif
unsigned int g_firmwareSize = 16*1024;

extern int PadFone_IN_OUT(int isin); 
static int is_first_bootup = 1;
extern enum DEVICE_HWID g_ASUS_hwID;


//static unsigned long jiffies_hp_plugIn=0 ; 


struct microP_info {
	struct i2c_client *i2c_client;
	struct delayed_work work;
	struct delayed_work initP01;
	struct delayed_work deinitPad;
	struct delayed_work poweroffP01;
        struct delayed_work reportPadFail;
};


struct nuvoton_microp_command{
    char *name;;
    u8 addr;
    u8 len;
    enum readwrite{
                E_READ=0,
                E_WRITE=1,
                E_READWRITE=2,
                E_NOUSE=3,

    }rw;

    enum   usedMode{
                E_MODE_APROM=0,
                E_MODE_LDROM=1,
                E_MODE_BOTH=2,
    }mode;

    enum   supportID{
                Prj_A68=0x01,
                Prj_A80=0x10,
    }prj;

};


/*
*       addr: length
*       
*/
struct nuvoton_microp_command uP_CMD_Table[]={
        {"hw_id",                   0x00,  4,   E_READ,            E_MODE_BOTH,     Prj_A68|Prj_A80},         //MICROP_HW_ID
        {"fw_ver",                 0x01,  2,   E_READ,             E_MODE_BOTH,     Prj_A68|Prj_A80},         //MICROP_FW_VER
        {"input_lvl",               0x20,  4,   E_READ,             E_MODE_BOTH,     Prj_A68|Prj_A80},         //MICROP_GPIO_INPUT_LEVEL
        {"out_lvl",                  0x21,  4,   E_READWRITE,   E_MODE_BOTH,     Prj_A68|Prj_A80},         //MICROP_GPIO_OUTPUT_LEVEL
        {"charging_status",     0x30,  1,   E_READ,             E_MODE_APROM,     Prj_A68|Prj_A80},         //MICROP_CHARGING_STATUS
        {"gauge_id",               0x31,  4,   E_READ,             E_MODE_APROM,     Prj_A68|Prj_A80},         //MICROP_GAUGE_ID
        {"gauge_cap",            0x32,  1,   E_READ,             E_MODE_APROM,     Prj_A68|Prj_A80},         //MICROP_GAUGE_CAP
        {"gauge_tmp",           0x33,  1,   E_READ,              E_MODE_APROM,     Prj_A68|Prj_A80},         //MICROP_GAUGE_TMP
        {"usb_det",                0x34,  1,   E_READ,              E_MODE_APROM,     Prj_A68|Prj_A80},         //MICROP_USB_DET
        {"pwm",                     0x40,  1,   E_READWRITE,    E_MODE_BOTH,     Prj_A68|Prj_A80},         //MICROP_PWM
        {"intr_status",             0x41,  4,   E_READ,             E_MODE_APROM,     Prj_A68|Prj_A80},         //MICROP_INTR_STATUS
        {"intr_en",                 0x42,  4,   E_READWRITE,    E_MODE_APROM,     Prj_A68|Prj_A80},         //MICROP_INTR_EN
        {"boot_sel",                0x50,  1,   E_READ,             E_MODE_BOTH,     Prj_A68|Prj_A80},         //MICROP_BOOT_SELECTION
        {"boot_ld",                 0x51,  1,   E_WRITE,            E_MODE_APROM,     Prj_A68|Prj_A80},         //MICROP_SET_BOOT_LDROM
        {"boot_ap",                0x52,  1,   E_WRITE,            E_MODE_LDROM,     Prj_A68|Prj_A80},         //MICROP_SET_BOOT_APROM
        {"prog_ap",                0x53,  32,  E_WRITE,           E_MODE_LDROM,     Prj_A68|Prj_A80},         //MICROP_PROGRAM_APROM
        {"ap_chksum",           0x54,  4,  E_READ,              E_MODE_APROM,     Prj_A68|Prj_A80},          //MICROP_APROM_CHECKSUM
        {"software_off",           0x55,  4,  E_WRITE,          E_MODE_APROM,     Prj_A68|Prj_A80},          //MICROP_SOFTWARE_OFF
        {"ldrom_id",               0x03,  2,  E_READ,             E_MODE_APROM,     Prj_A68|Prj_A80},          //MICROP_LDROM_ID_CODE
        {"out_lvr_bit_set",      0x22,  4,  E_WRITE,            E_MODE_BOTH,     Prj_A68|Prj_A80},          //MICROP_GPIO_OUTPUT_BIT_SET
        {"out_lvr_bit_clr",      0x23,  4,  E_WRITE,            E_MODE_BOTH,     Prj_A68|Prj_A80},          //MICROP_GPIO_OUTPUT_BIT_CLR        
        {"intr_en_bit_set",      0x43,  4,  E_WRITE,           E_MODE_APROM,     Prj_A68|Prj_A80},          //MICROP_INTR_EN_BIT_SET
        {"intr_en_bit_clr",       0x44,  4,  E_WRITE,           E_MODE_APROM,     Prj_A68|Prj_A80},          //MICROP_INTR_EN_BIT_CLR        
        {"a68_ready",            0x05,  1,  E_READWRITE,    E_MODE_APROM,     Prj_A68|Prj_A80},          //MICROP_IND_A68_READY
        {"ind_a68_sleep",       0x56,  1,  E_WRITE,             E_MODE_APROM,     Prj_A68|Prj_A80},          //MICROP_IND_A68_SLEEP
        {"ind_a68_resume",    0x57,  1,  E_WRITE,             E_MODE_APROM,     Prj_A68|Prj_A80},          //MICROP_IND_A68_RESUME
        {"adc_level",              0x24,  2,  E_READ,               E_MODE_APROM,     Prj_A68|Prj_A80},          //MICROP_ADC_LEVEL
        {"up_opstate",            0x58,  1,  E_READ,               E_MODE_APROM,     Prj_A68|Prj_A80},          //MICROP_OPERATING_STATE
        {"mhl_id",                  0x06,  2,  E_READ,               E_MODE_APROM,     Prj_A68|Prj_A80},               //MICROP_MHL_ID
        {"hw_factory_isn",     0x04,  32,  E_READ,              E_MODE_BOTH,       Prj_A68|Prj_A80},          //MICROP_ISN                
        {"prog_progress",     0x59,  4,    E_READWRITE,      E_MODE_LDROM,     Prj_A68|Prj_A80},          //MICROP_LD_PROG_PROGRESS
        {"power_on_reason",     0x07,  1,  E_READ,             E_MODE_APROM,     Prj_A68|Prj_A80},          //MICROP_POWER_ON_REASON        
        {"disable_charging_batt_60p",     0x08,  1,  E_READWRITE,     E_MODE_APROM,     Prj_A68|Prj_A80},          //MICROP_DISABLE_CHARGING_FOR_FACTORY        
        {"mhl_nvram_state",               0x09,  1,  E_READ,               E_MODE_APROM,     Prj_A68|Prj_A80},          //MICROP_GET_MVRAM_STATE_FOR_FACTORY
        {"light_sensor_kData",               0x0A,  8,  E_READWRITE,    E_MODE_APROM,     Prj_A68|Prj_A80},          //MICROP_CALIBRATION_DATA        
        {"gauge_voltage",               0x35,  2,  E_READ,               E_MODE_APROM,     Prj_A68|Prj_A80},          //MICROP_GAUGE_VOLTAGE        
        {"gauge_avg_current",               0x36,  2,  E_READ,               E_MODE_APROM,     Prj_A68|Prj_A80},          //MICROP_GAUGE_AVG_CURRENT                
        {"ignore_a68ready",               0x0b,  1,  E_READWRITE,           E_MODE_APROM,     Prj_A68|Prj_A80},          //MICROP_ALWAYS_IGNORE_A68READY
        {"lightsensor_result",                        0x10,  1,  E_READ,              E_MODE_APROM,     Prj_A68|Prj_A80},          //MICROP_LS_RESULT        
        {"lightsensor_init",                            0x11,  1,  E_WRITE,             E_MODE_APROM,     Prj_A68|Prj_A80},          //MICROP_LS_INIT
        {"lightsensor_adc",                           0x12,  2,  E_READWRITE,     E_MODE_APROM,     Prj_A68|Prj_A80},          //MICROP_LS_ADC
        {"lightsensor_set_threHigh",              0x13,  2,  E_WRITE,             E_MODE_APROM,     Prj_A68|Prj_A80},          //MICROP_LS_SET_THRESHOLD_LOW
        {"lightsensor_set_threLow",               0x14,  2,  E_WRITE,             E_MODE_APROM,     Prj_A68|Prj_A80},          //MICROP_LS_SET_THRESHOLD_HIGH
        {"compass_para",                            0x0c,  9,  E_READWRITE,     E_MODE_BOTH,       Prj_A80},          //MICROP_COMPASS_PARAMETER
        {"microp_oem_ver",                         0x0f,  24,  E_READ,              E_MODE_APROM,     Prj_A80},          //MICROP_OEM_FW_VERSION
        {"extend_io_status",                         0x25,  1,  E_READ,               E_MODE_APROM,     Prj_A80},          //MICROP_EXTEND_IO_STATUS
        {"usb_type",                                    0x37,  1,  E_READ,               E_MODE_APROM,     Prj_A80},          //MICROP_USB_TYPE
        {"power_on",                                    0x5c,  1,  E_WRITE,             E_MODE_APROM,     Prj_A80},          //MICROP_MICROP_POWER_ON
        {"OEM_SSN",                                    0x0d,  32,  E_READWRITE,     E_MODE_APROM,     Prj_A80},          //MICROP_OEM_SSN
        {"p05_hwid",                            0x26,  1,  E_READ,     E_MODE_BOTH,     Prj_A80},          //MICROP_PAD_HWID
        {"led_control",                            0x28,  2,  E_READWRITE,     E_MODE_APROM,     Prj_A80},          // MICROP_LED_CONTROL
        {"OEM_IMEI",                             0x0e,  32,  E_READWRITE,     E_MODE_BOTH,     Prj_A80},          //MICROP_OEM_IMEI

};
 
//const unsigned int microP_hw_ID=0x00005200;
unsigned int g_microp_ver=0;
unsigned int g_ldrom_ver=0;
unsigned int g_curr_uP_mode=0;
unsigned int g_pad_pwrkey_pre_pressed=0;
static int g_padModel;
static struct microP_info *g_uP_info=NULL;
uint8_t *img_buf=NULL;
static u32 g_slave_addr=0;
static int g_microp_irq_gpio=0;
static unsigned int g_fw_update_progress=0;
unsigned int g_b_isP01Connected=0;
unsigned int g_b_isP01USBConnected=0;
unsigned int g_i2c_bus_suspended=0;
// ASUS_BSP +++ Peter_Lu "For Pad I2C suspend/resume issue"
unsigned int g_i2c_bus_3v3_state=1;	// 1 : 3v3 on, 2 : 3v3 off
// ASUS_BSP ---
struct switch_dev p01_switch_dev;
struct switch_dev p01_switch_usb_cable;
struct switch_dev pad_err_notify;
struct mutex microp_mutex_lock;
extern void msleep(unsigned int msecs);
static uint8_t g_uPadStationUnderPoweroff=0;
static uint8_t g_uPadUnderFailHandling=0;
struct workqueue_struct *microp_slow_job_wq = NULL;
struct workqueue_struct *microp_ins_rev_wq = NULL;
struct workqueue_struct *microp_intr_wq = NULL;
int no_chk_pad_cap=0;

unsigned int g_i2c_microp_busy=0;



enum _microp_state{
        st_DISCONNECTED=0,
        st_CONNECTED=1,
        st_INITIALIZING=2,
};

enum _microp_poweroff_cause{
	E_OFF_BATTERY_BAD=0,
	E_OFF_NOT_SUPPORT_PAD=1,
};


module_param(no_chk_pad_cap,int, 0644);


uint8_t g_uPadErrStatus=0;

// disable it in default
int g_prop_virtualRemoveEnabled=1;

static int g_prop_Allow_UsingP05=0;	//default not allow P05 usage

void reportPadStationI2CFail(char *devname);
int isFirmwareUpdating(void);
int is_Mode_APROM(void){
    return (g_curr_uP_mode==E_MODE_APROM)?1:0;
}

static void microp_reconnected(void);


extern bool asus_padstation_exist_realtime(void); //ASUS_BSP:joe1_++ 

// need modification
bool pad_exist(void){
        return asus_padstation_exist_realtime()?1:0; //ASUS_BSP:joe1_++
}
static int uP_i2c_read(u8 addr, int len, void *data)
{
        int i=0;
        int retries=8;
        int status=0;

        struct i2c_msg msg[] = {
            {
                    .addr = g_slave_addr,
                    .flags = 0,
                    .len = 1,
                    .buf = &addr,
            },
            {
                    .addr = g_slave_addr,
                    .flags = I2C_M_RD,
                    .len = len,
                    .buf = data,
            },
        };
    
        if(g_uP_info){
            do{    
                    status = i2c_transfer(g_uP_info->i2c_client->adapter, msg, ARRAY_SIZE(msg));
                    if ((status < 0) && (i < retries) && pad_exist()){
			msleep(50);
                        printk("%s retry %d I2C cmd:0x%2x, status=0x%x\r\n", __FUNCTION__, i, addr ,status);                                
                        i++;
                     }
            } while ((status < 0) && (i < retries) && pad_exist());
        }

        if(status < 0){
                printk("I2C cmd:0x%2x status=0x%x read failed\r\n", addr, status);
                reportPadStationI2CFail("MicroP");
        }

        return status;
        
}



static int uP_i2c_write(u8 addr, int len, void *data)
{
        int i=0;
        int status=0;
        u8 buf[len + 1];
        struct i2c_msg msg[] = {
            {
                .addr = g_slave_addr,
                .flags = 0,
                .len = len + 1,
                .buf = buf,
            },
        };
        int retries = 8;

        buf[0] = addr;
        memcpy(buf + 1, data, len);

        do {                
                status = i2c_transfer(g_uP_info->i2c_client->adapter, msg, ARRAY_SIZE(msg));
                if ((status < 0) && (i < retries) && pad_exist()){                    
                    msleep(50);							
                    printk("%s retry %d, I2C cmd:0x%2x, status=0x%x\r\n", __FUNCTION__, i, addr, status);
                    i++;
                }
       } while ((status < 0) && (i < retries) && pad_exist());


        if (status < 0) {
                printk("I2C cmd:0x%2x status=0x%x write failed\r\n", addr, status);
                reportPadStationI2CFail("MicroP");
        }
	return status;
}

int isCMDSupportedForRead(int cmd){
	int ret=1;

	if(E_WRITE==uP_CMD_Table[cmd].rw 
			|| E_NOUSE==uP_CMD_Table[cmd].rw 
			|| (E_MODE_BOTH!=uP_CMD_Table[cmd].mode && g_curr_uP_mode!=uP_CMD_Table[cmd].mode))
				ret=0;

	return ret;
}
int isCMDSupportedForWrite(int cmd){
	int ret=1;

	if(E_READ==uP_CMD_Table[cmd].rw 
			|| E_NOUSE==uP_CMD_Table[cmd].rw 
			|| (E_MODE_BOTH!=uP_CMD_Table[cmd].mode && g_curr_uP_mode!=uP_CMD_Table[cmd].mode))
				ret=0;

	return ret;
}

/*
* return read data length of the reg
*/

int uP_nuvoton_read_reg(int cmd, void *data){
    int status=0;
    if(cmd>=0 && cmd < ARRAY_SIZE(uP_CMD_Table)){
        if(!isCMDSupportedForRead(cmd)){ // skip read for these command
                      pr_debug("%s:skip cmd mode=%d, g_curr_uP_mode=%d\r\n", __FUNCTION__, uP_CMD_Table[cmd].mode, g_curr_uP_mode);
        }
        else   
            status=uP_i2c_read(uP_CMD_Table[cmd].addr, uP_CMD_Table[cmd].len, data);
    }
    else
        printk("MicroP: unknown cmd\r\n");
            
    return status;
}

EXPORT_SYMBOL_GPL(uP_nuvoton_read_reg);

/*
* return read data length of the reg
*/

int uP_nuvoton_write_reg(int cmd, void *data){
    int status=0;
    if(cmd>=0 && cmd < ARRAY_SIZE(uP_CMD_Table)){
        if(!isCMDSupportedForWrite(cmd)){ // skip read for these command               
                      pr_debug("%s:skip cmd mode=%d, g_curr_uP_mode=%d\r\n", __FUNCTION__, uP_CMD_Table[cmd].mode, g_curr_uP_mode);
        }
        else
            status=uP_i2c_write(uP_CMD_Table[cmd].addr, uP_CMD_Table[cmd].len, data);
    }
    else
        printk("MicroP: unknown cmd\r\n");
            
    return status;
}

EXPORT_SYMBOL_GPL(uP_nuvoton_write_reg);

/*
* return 1 if microp is connected
*/


int isMicroPConnected(void){    
       int status;
       int ret=0;
       int reg_id=0;
       int op_state=0;
       int retries=40; // 1.2 seconds to try
       uint32_t st_jiffies;
#ifndef ASUS_FACTORY_BUILD                                   
       uint8_t always_ignore=0;
        uint8_t uc_val=0;

#endif
       uint8_t a68_ready=0x96;
       //uint16_t  magic_resume_value=0x0069;
       printk("%s \r\n", __FUNCTION__);
       status=uP_nuvoton_read_reg(MICROP_FW_VER,&reg_id);
       if(status > 0 && reg_id > 0){
            printk("MicroP found! fw ver=0x%x\r\n",reg_id);
            g_microp_ver=reg_id;
            
            uP_nuvoton_read_reg(MICROP_BOOT_SELECTION,&g_curr_uP_mode);
            printk("Current Mode=%s\r\n",(g_curr_uP_mode==0)?"APROM":"LDROM");            

            if(is_Mode_APROM())
                    uP_nuvoton_read_reg(MICROP_LDROM_ID_CODE,&g_ldrom_ver);
            else
                    g_ldrom_ver=g_microp_ver;
            
            printk("LDROM ver=0x%x\r\n",g_ldrom_ver);            

        


            if(is_Mode_APROM()){
                
                    uP_nuvoton_read_reg(MICROP_OPERATING_STATE,&op_state);
                    st_jiffies=jiffies;
                    while(st_MICROP_Active!=op_state  && retries>0){												
						msleep(30);
						printk("<try: %d, %d>", 40-retries, op_state);
                                                /*  dont do resume here. In some case, it causes bugs
						if(st_MICROP_Sleep==op_state){
							status=uP_nuvoton_write_reg(MICROP_IND_A68_RESUME, &magic_resume_value);																					
							msleep(30);
                                                } 
                                                */ 
						uP_nuvoton_read_reg(MICROP_OPERATING_STATE,&op_state);
						retries--;
                    }            
                    printk("==> takes %lu jiffies~~\r\n", jiffies - st_jiffies);				
                    if(retries == 0)
                            printk("microp state failed!!\r\n");
                    else {
                            g_i2c_bus_3v3_state=1;	//reset 3v3 state global to 'on'
                            printk("state=> Active\r\n");
#ifndef ASUS_FACTORY_BUILD                            
                            status=uP_nuvoton_read_reg(MICROP_ALWAYS_IGNORE_A68READY,&always_ignore);
                            if(status && 0x96==always_ignore){
                                    {
                                            always_ignore=0;
                                            printk("%s: clear \"MICROP_ALWAYS_IGNORE_A68READY\" flag \r\n", __FUNCTION__);                                    
                                            uP_nuvoton_write_reg(MICROP_ALWAYS_IGNORE_A68READY,&always_ignore);                         
                                    }
                            }


                            status=uP_nuvoton_read_reg(MICROP_DISABLE_CHARGING_FOR_FACTORY,&uc_val);                            
                            if(status && 0xcc==uc_val){                            
                                            uc_val=0;
                                            printk("%s: clear \"MICROP_DISABLE_CHARGING_FOR_FACTORY\" flag \r\n", __FUNCTION__);                                    
                                            uP_nuvoton_write_reg(MICROP_DISABLE_CHARGING_FOR_FACTORY,&uc_val);                         
                            }

#endif                            
                            status=uP_nuvoton_write_reg(MICROP_IND_A68_READY,&a68_ready);
                            if(status > 0)
                                    ret=1;             
                    }
            }
            else{
                printk("entering LDROM: %d\r\n", is_Mode_APROM());
                ret=1; 
            }
       }else{
            printk("%s : not connected\r\n", __FUNCTION__);
       }
       return ret;
}

EXPORT_SYMBOL_GPL(isMicroPConnected);



/*
*
*       for debug
*
*
*/
#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/seq_file.h>

static int dbg_microp_regs_dump(struct seq_file *s, void *unused)
{
        int i=0;
        struct nuvoton_microp_command cmd;
        uint64_t val;
        int16_t val2;
        char sz_PadIsn[32]={0};
        char sz_oemVerStr[24]={0};        
        seq_printf(s, "\nMicroP CMD       ADDR        VAL       MODE\r\n");
        seq_printf(s, "============================== %d,\r\n",ARRAY_SIZE(uP_CMD_Table));
        for(i=0;i<ARRAY_SIZE(uP_CMD_Table);i++){                
            cmd=uP_CMD_Table[i];
            val=0;
            if(MICROP_ISN==i || MICROP_OEM_SSN==i || MICROP_OEM_IMEI==i){
                    uP_nuvoton_read_reg(i,sz_PadIsn);
                    seq_printf(s, "%32s\t%4x\t%32s(ASCII)\r\n",cmd.name, cmd.addr, sz_PadIsn);
            }
            else if(MICROP_GAUGE_AVG_CURRENT==i){
                    uP_nuvoton_read_reg(i,&val2);
                   seq_printf(s, "%32s\t%4x\t%32d mA(d)\r\n",cmd.name, cmd.addr, val2);
            }
            else if(MICROP_GAUGE_VOLTAGE==i){
                    uP_nuvoton_read_reg(i,&val);
                   seq_printf(s, "%32s\t%4x\t%32llu mV(d)\r\n",cmd.name, cmd.addr, val);
            }
            else if(MICROP_OEM_FW_VERSION==i){
                    uP_nuvoton_read_reg(i,sz_oemVerStr);
                    seq_printf(s, "%32s\t%4x\t%32s(ASCII)\r\n",cmd.name, cmd.addr, sz_oemVerStr);
            }

            else{
                    uP_nuvoton_read_reg(i,&val);
                   seq_printf(s, "%32s\t%4x\t%32llx(h)\r\n",cmd.name, cmd.addr, val);

            }
        }
	return 0;
}

static int dbg_microp_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_microp_regs_dump, &inode->i_private);
}
static const struct file_operations debug_fops = {
	.open		= dbg_microp_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};


#endif


static int updateMicroPFirmware(unsigned long arg);

/************************************
*
* macro definition
*
*************************************/
// check reg_intr_sta val

// Pad-Related
#define IsPadACUSBInOut()                  ( ((reg_intr_sta>>INTR_STA_AC_USB_IN_OUT) & 0x1)?1:0 )
#define IsPadVolDnEvt()               ( ((reg_intr_sta>>INTR_STA_VOL_DOWN) & 0x1)?1:0 )
#define IsPadVolUpEvt()               ( ((reg_intr_sta>>INTR_STA_VOL_UP) & 0x1)?1:0 )
#define IsPadPwrBtnEvt()             ( ((reg_intr_sta>>INTR_STA_PWR_BTN) & 0x1)?1:0 )
#define IsPadPwrBtnPressEvt()             ( ((reg_intr_sta>>INTR_STA_FLAG_PWRKEY_PRESS) & 0x1)?1:0 )
#define IsPadPwrBtnReleaseEvt()             ( ((reg_intr_sta>>INTR_STA_FLAG_PWRKEY_RELEASE) & 0x1)?1:0 )
#define IsPadHavingLSEvt()                 ( ((reg_intr_sta>>INTR_STA_ALS_INT) & 0x1)?1:0 )
#define IsPadHavingBat5Percent()                 ( ((reg_intr_sta>>INTR_STA_IND_GAUGE_5P) & 0x1)?1:0 )
#define IsPadHavingProxAct()                 ( ((reg_intr_sta>>INTR_STA_CAP_PROX_ACT) & 0x1)?1:0 )
#define IsPadPwrBtnLongPressEvt()             ( ((reg_intr_sta>>INTR_STA_LONG_PRESS_PWRKEY) & 0x1)?1:0 )
#define IsPadRevSleepReminder()             ( ((reg_intr_sta>>INTR_STA_SLEEP_REMINDER) & 0x1)?1:0 )
#define IsPadNoIntrEvt()                            ((reg_intr_sta==0)?1:0)
#define IsPadBatStatusChangeEvt()               (((reg_intr_sta >> INTR_STA_BAT_STATUS_CHANGE) & 0x1)?1:0)
#define IsPadShutdownByEC()               (((reg_intr_sta >> INTR_STA_IND_SHUTDOWN_BY_EC) & 0x1)?1:0)



/*
* check reg_input val
*/



// Pad
#define IsPadACUSBIn()                      ( ((reg_input>>IN_AC_USB_IN) & 0x1)?1:0 )
#define IsPadACUSBIn_P05()                      ( ((reg_input>>IN_AC_USB_IN) & 0x1)?0:1 )
#define IsPadVolUpPressed()                ( ((reg_input>>IN_VOL_UP_R) & 0x1)?0:1 )
#define IsPadVolDnPressed()                ( ((reg_input>>IN_VOL_DOWN_R) & 0x1)?0:1 )
#define IsPadPwrBtnPressed()                ( ((reg_input>>IN_PWR_BTN_R) & 0x1)?0:1 )
#define IsPadProxHavingEvts()                  ( ((reg_input>>IN_CAP_RPOX_INT_R) & 0x1)?0:1 )


extern int micropSendNotify(unsigned long val);
extern void bl_pad_add_func(struct work_struct *work);






int isAlwaysPowerOnMicroP(void){    

       int status;
       unsigned usb_detect=0;
       status=uP_nuvoton_read_reg(MICROP_USB_DET,&usb_detect);
       if(status){
            printk("%s usb type=%d\r\n", __FUNCTION__, usb_detect);
            if(usb_detect==P01_CABLE_CHARGER)
                return 1;
       }
       return 0;

}





static void microp_reconnected(void)
{
	printk("MicroP: Trigger Driver Re-Connecting \r\n");
       if(g_uP_info){
                printk("%s,  g_b_isP01Connected=%d\r\n",__FUNCTION__,  g_b_isP01Connected);
                if(st_CONNECTED!=g_b_isP01Connected){
                        g_b_isP01Connected=st_INITIALIZING; // 2: hdmi cable inserted but not intialized ready
                        switch_set_state(&p01_switch_dev,st_INITIALIZING);
                        //schedule_delayed_work(&g_uP_info->initP01,0);
                        queue_delayed_work(microp_ins_rev_wq, &g_uP_info->initP01, 0);
                }
                else{
                        printk("MicroP: already connected skip\r\n");
                }
        }
       else{
            printk("MicroP: not yet init \r\n");
       }
       
}


/*
*
* Export to Display driver to inform HDMI status change
*
*/
void notify_microp_hdmi_insert(void)
{
	is_first_bootup = 0;
	
	msleep(400);
	printk("MicroP: HDMI -> INSERT \r\n");
       if(g_uP_info){
                printk("%s,  g_b_isP01Connected=%d\r\n",__FUNCTION__,  g_b_isP01Connected);
                if(st_CONNECTED!=g_b_isP01Connected){
                        g_b_isP01Connected=st_INITIALIZING; // 2: hdmi cable inserted but not intialized ready
                        switch_set_state(&p01_switch_dev,st_INITIALIZING);
                        //schedule_delayed_work(&g_uP_info->initP01,0);
                        queue_delayed_work(microp_ins_rev_wq, &g_uP_info->initP01, 0);
                }
                else{
                        printk("MicroP: already connected skip\r\n");
                }
        }
       else{
            printk("MicroP: not yet init \r\n");
       }
       
}
EXPORT_SYMBOL_GPL(notify_microp_hdmi_insert);

void notify_microp_hdmi_remove(int virtual)
{
       if(virtual){
	        printk("MicroP: HDMI -> VIRTUAL REMOVE \r\n");
       }
       else{
               printk("MicroP: HDMI -> REMOVE \r\n");
       }
       
#ifdef CONFIG_FASTBOOT
	if(is_fastboot_enable()){
		ready_to_wake_up_and_send_power_key_press_event_in_fastboot( true );
	}
#endif //#ifdef CONFIG_FASTBOOT
       
       if(g_uP_info){
                printk("%s,  g_b_isP01Connected=%d\r\n",__FUNCTION__,  g_b_isP01Connected);
                if(st_DISCONNECTED!=g_b_isP01Connected){
                        g_b_isP01Connected=st_DISCONNECTED;
                        switch_set_state(&p01_switch_dev,st_DISCONNECTED);
                        PadFone_IN_OUT(0); //nofiy wlan pad is out
                        queue_delayed_work(microp_ins_rev_wq, &g_uP_info->deinitPad, 0);
                        
                        if(virtual){
                                switch_set_state(&pad_err_notify, g_uPadErrStatus);
                        }

                }
                else
                        printk("MicroP: P01 already removed, skip \r\n");                                   
        }
       else{
            printk("MicroP: not yet init \r\n");
       }

       
}
EXPORT_SYMBOL_GPL(notify_microp_hdmi_remove);



void TriggerPadStationPowerOff(int reason){
	int l_pwm_value=0;	
	int rounds=5,i=0;
	if(g_uP_info && microp_slow_job_wq){
		if(reason==E_OFF_NOT_SUPPORT_PAD){
			//blink the screen first before off pad
			l_pwm_value=AX_MicroP_getPWMValue();
			if(l_pwm_value > 0){
				for(i=0;i<rounds;i++){
					AX_MicroP_setPWMValue(0);
					msleep(1000);
					AX_MicroP_setPWMValue(l_pwm_value);
					msleep(1000);

				}
			}
		}

		if(!g_uPadStationUnderPoweroff){
		    printk("power off PAD: reason:%d\r\n", reason);
		    queue_delayed_work(microp_slow_job_wq, &g_uP_info->poweroffP01,0);
		}else{
		    printk("power off work is onGoing now\r\n");
		}
		}
	else{
		printk("MicroP: not yet init \r\n");
	}

}

void reportPadStationI2CFail(char *devname){
        int ret=0;
        printk("%s: ++, trigger by [%s]\r\n", __FUNCTION__, devname);
        if(g_prop_virtualRemoveEnabled && st_CONNECTED==g_b_isP01Connected){ // report failed only if the state is under 'Connected'
            if (g_uPadUnderFailHandling){
                printk("ReportFail Work Ongoing, skip queuing\r\n"); 
            }
            else{
                ret=queue_delayed_work(microp_intr_wq, &g_uP_info->reportPadFail, 0); 
                if (ret>0) {
                    printk("reportPadFail queue ok\r\n"); 
                }
            }
        }
        else{
            printk("%s: <%d,%d>\r\n", __FUNCTION__, g_prop_virtualRemoveEnabled, g_b_isP01Connected);
        }

        printk("%s: --\r\n", __FUNCTION__);
}


EXPORT_SYMBOL_GPL(TriggerPadStationPowerOff);
EXPORT_SYMBOL_GPL(reportPadStationI2CFail);
// for test +++




#ifdef CONFIG_I2C_STRESS_TEST

#include <linux/i2c_testcase.h>
#define I2C_TEST_FAIL_MICROP_READ_I2C (-1)
#define I2C_TEST_FAIL_MICROP_WROND_CHIP_ID (-2)

static int TestMicroPChipID(struct i2c_client *apClient)
{

	int lnResult = I2C_TEST_PASS;
       unsigned int l_micropID=0;
	uint8_t PWMvalue=33;

	i2c_log_in_test_case("TestMicroPChipID++\n");
	lnResult = uP_i2c_read(0x00, 4,&l_micropID);//MiCROP_HW_ID
	if(lnResult <= 0){

		i2c_log_in_test_case("Fail to get microp id\n");

		lnResult = I2C_TEST_FAIL_MICROP_READ_I2C;
		return lnResult;
	}
        else
	{
	    i2c_log_in_test_case("Get chip id=%d",l_micropID);	  
            lnResult=I2C_TEST_PASS;
	}

	lnResult = uP_i2c_write(0x40, 1,&PWMvalue);//PWM Value
	if(lnResult <= 0){

		i2c_log_in_test_case("PWMvalue Fail\n");

		lnResult = I2C_TEST_FAIL_MICROP_READ_I2C;
		return lnResult;
	}
        else
	{
		i2c_log_in_test_case("PWMvalue Success\n");
		lnResult=I2C_TEST_PASS;
	}
		
	i2c_log_in_test_case("TestMicroPChipID--\n");

	return lnResult;
};

static struct i2c_test_case_info gMicroPTestCaseInfo[] =
{
	__I2C_STRESS_TEST_CASE_ATTR(TestMicroPChipID),
};


#endif
// for test ---

//ASUS  BSP Eason_Chang +++
extern void ASUS_pet_watchdog_v2(void);
//ASUS  BSP Eason_Chang ---

static void initP01(struct work_struct *work){
	int reg_intr_sta_check = 0;
	int retry_intr_cnt = 10;
	int reg_intr_sta=0;
	int reg_input=0;    
        //int pad_cap=0;
	uint8_t i2cdata[32]={0};
	uint8_t usb_type=0;       
        uint16_t  magic_resume_value=0x0069;
	unsigned int pinID=0;

	printk("%s +%d\r\n",__FUNCTION__,gpio_get_value(9));

//ASUS_BSP++  20130819 Sina: if the taks will hold cpu resources for 1 seconds or more 
//                           , it has to pet watchdog to avoid watchdog barking

	//ASUS  BSP Eason_Chang +++
	ASUS_pet_watchdog_v2();
	//ASUS  BSP Eason_Chang ---


        // make sure microp opstate is ACTIVE mode if it is under sleep mode
        uP_nuvoton_write_reg(MICROP_IND_A68_RESUME, &magic_resume_value);


	if(isMicroPConnected()==1){
                
		if(st_CONNECTED!=g_b_isP01Connected){
			g_b_isP01Connected=st_CONNECTED;
			// default: not enable  HP Hookkey Intr bit 12
			
#ifdef ASUS_FACTORY_BUILD                        
			i2cdata[0]=0xf7;
			i2cdata[1]=0xA1;
			i2cdata[2]=0x1f;
#else
			i2cdata[0]=0xf7;
			i2cdata[1]=0xA1;
			i2cdata[2]=0x1f;
#endif

			uP_nuvoton_write_reg(MICROP_INTR_EN,i2cdata);

			micropSendNotify(P01_ADD);
			switch_set_state(&p01_switch_dev,st_CONNECTED);
			g_uPadErrStatus=0;      // reset error status to 0
			switch_set_state(&pad_err_notify, 0);

                   PadFone_IN_OUT(1); //notify wlan, pad is in

		}
		else{
			printk("MicroP: P01 already added, skip \r\n");
		}
	}
	else{
		printk("uP_Init: failed to access microp\r\n");
		g_b_isP01Connected=st_DISCONNECTED;
		switch_set_state(&p01_switch_dev,st_DISCONNECTED);
	}

	if(st_CONNECTED==g_b_isP01Connected){
		uP_nuvoton_read_reg(MICROP_INTR_STATUS,&reg_intr_sta);
		uP_nuvoton_read_reg(MICROP_GPIO_INPUT_LEVEL,&reg_input);
		uP_nuvoton_read_reg(MICROP_USB_DET, &usb_type);
		printk("skip first intr status = %x, input = %x, usb_det=%x, intr=%d\r\nCheck Devices Status on Pad\r\n",reg_intr_sta,reg_input, usb_type,gpio_get_value(9));
		if(usb_type==P01_CABLE_CHARGER || usb_type==P01_CABLE_USB){
			printk(KERN_INFO "uP_Init: AC/USB in !!\n");
			micropSendNotify(P01_AC_USB_IN);
		}
		else if(usb_type==P01_CABLE_OTG){
			printk(KERN_INFO "uP_Init: OTG in !!\n");
			micropSendNotify(PAD_USB_OTG_ENABLE);
		}

		/*Eason: Phone insert to Pad while pad plugged usb, sent uevent+++*/
		if(P01_CABLE_USB == usb_type) {
				g_b_isP01USBConnected=1;
				switch_set_state(&p01_switch_usb_cable,g_b_isP01USBConnected);
				printk("[Microp]set usb plugged uevent:%d\n",g_b_isP01USBConnected);
		}
		/*Eason: Phone insert to Pad while pad plugged usb, sent uevent---*/ 
			
		//Eason: When init P01 check intr status already handled+++
		uP_nuvoton_read_reg(MICROP_INTR_STATUS,&reg_intr_sta_check);
		while( (0!=reg_intr_sta_check)&&(retry_intr_cnt >=0 ) )
		{
			printk("%s intr status fail retry %d :%d\n",__FUNCTION__,retry_intr_cnt, reg_intr_sta_check);
			uP_nuvoton_read_reg(MICROP_INTR_STATUS,&reg_intr_sta_check);
			retry_intr_cnt --;
		}
		//Eason: When init P01 check intr status already handled---

                
                
                if (is_Mode_APROM()) {
			//get model name
			g_padModel=AX_MicroP_getPadModel();
			printk("[MICROP] pad model : %s, g_prop_Allow_UsingP05=%d\n", (g_padModel==PAD_P92L ||g_padModel==PAD_P93L)?"P92L/P93L":"NON P92L", g_prop_Allow_UsingP05);
			if(0==g_prop_Allow_UsingP05 && g_padModel!=PAD_P92L && g_padModel!=PAD_P93L){
			         printk("***[MICROP] NOT ALLOWED NON P92L  PAD***\r\n");
			         TriggerPadStationPowerOff(E_OFF_NOT_SUPPORT_PAD);
			}
			else if(g_padModel==PAD_P93L){
				//P93L dont have cap sensor, because the pin is not pulled up, to avoid floating interrupts, dont turn of interrupt-en bit				
				printk("***[MICROP] PAD_P93L disable cap intr. \r\n");
				pinID=INTR_EN_CAP_RPOX_INT;
				uP_nuvoton_write_reg(MICROP_INTR_EN_BIT_CLR, &pinID);
			}
		    
                }
                
                
	}
	printk("%s -%d\r\n",__FUNCTION__,gpio_get_value(9));
}


static void deinitPad(struct work_struct *work){

        printk("%s +\r\n",__FUNCTION__);

        printk("MicroP: cancel work...\r\n");
        cancel_delayed_work_sync(&g_uP_info->initP01);
        printk("MicroP: Done\r\n");
        // cancel all works of init wq

        micropSendNotify(P01_REMOVE);

	/*Eason: Phone insert to Pad while pad plugged usb, sent uevent+++*/
	g_b_isP01USBConnected=0;
	switch_set_state(&p01_switch_usb_cable,g_b_isP01USBConnected);
	printk("[Microp]set usb plugged uevent:%d\n",g_b_isP01USBConnected);
	/*Eason: Phone insert to Pad while pad plugged usb, sent uevent---*/ 

        printk("%s -\r\n",__FUNCTION__);
}

static void reportPadFailWork(struct work_struct *work){
       printk("%s ++ \r\n", __FUNCTION__);
       g_uPadUnderFailHandling=1;
       if(g_i2c_bus_suspended)
           printk("%s: Bus Suspended: Skip\r\n", __FUNCTION__);                                            
       else if(st_CONNECTED==g_b_isP01Connected && is_Mode_APROM()){
           if(pad_exist()){
               printk("%s: [MicroP] Triggerd Virtual Remove\r\n", __FUNCTION__);
               g_uPadErrStatus=3; //i2c error
               notify_microp_hdmi_remove(1);
           }
           else{
               printk("%s: [MicroP] Pad Not Exist. Triggerd Normal Remove\r\n", __FUNCTION__);
               notify_microp_hdmi_remove(0);
           }
       }
       g_uPadUnderFailHandling=0;
       printk("%s -- \r\n", __FUNCTION__);
}

static void microp_poweroff(struct work_struct *work){
        unsigned short off=0xAA;
        uint32_t bit_vbus=1<<OUT_uP_VBUS_EN;
        g_uPadStationUnderPoweroff=1;
        printk("[%s] ++\r\n", __FUNCTION__);
        // turn off vbus first
        if(!isAlwaysPowerOnMicroP()) //if no charger, turn off vbus
            uP_nuvoton_write_reg(MICROP_GPIO_OUTPUT_BIT_CLR,  &bit_vbus);
        
        
        uP_nuvoton_write_reg(MICROP_SOFTWARE_OFF,  &off);
//        msleep(5500); //wait P01 truly power off its power. microp fw need 5 seconds to power off
        micropSendNotify(P01_BATTERY_POWER_BAD);           
        g_uPadStationUnderPoweroff=0;
        printk("[%s] REMOVE PAD DUE TO BATT CAPACITY IS LOW!!\r\n", __FUNCTION__);
        g_uPadErrStatus=4; // power bad
        notify_microp_hdmi_remove(1);
        
        printk("[%s] --\r\n", __FUNCTION__);
}

void checkPadLowCapacity(void){
    int pad_cap=0;
    uint8_t pad_chargingStatus=0;
    uP_nuvoton_read_reg(MICROP_CHARGING_STATUS, &pad_chargingStatus);
    uP_nuvoton_read_reg(MICROP_GAUGE_CAP, &pad_cap); 
    printk("checkPadLowCapacity: <s:%d,c:%d> \r\n",pad_chargingStatus,pad_cap);
    if(pad_cap <=5 && P01_CHARGING_NO==pad_chargingStatus){
        printk(" turnoff pad due to low cap and not charging\r\n");
        TriggerPadStationPowerOff(E_OFF_BATTERY_BAD);
    }
}
static void microP_work(struct work_struct *work)
{
	int reg_intr_sta=0;
	int reg_input=0;
	uint8_t reg_powerOnReason=0;
	uint8_t usb_type=0;       
	uint8_t a68_ready=0x96;     
       
#if 0       
	unsigned int  test_cnt=0;
#endif
	pr_debug("MicroP wq <%d,%d>+++\r\n", g_b_isP01Connected, gpio_get_value(g_microp_irq_gpio));
	if(is_first_bootup == 1){
		printk("is_first_bootup = 1, don't handle interrupt from microp\n");
		uP_nuvoton_read_reg(MICROP_INTR_STATUS,&reg_intr_sta);
		uP_nuvoton_read_reg(MICROP_GPIO_INPUT_LEVEL,&reg_input);
		return;
	}
	
	if(st_CONNECTED==g_b_isP01Connected){
		while(0==gpio_get_value(g_microp_irq_gpio)){
			if(g_i2c_bus_suspended){
				printk("MicroP Intr: i2c not ready. pospone intr handling for 30ms\r\n");
				msleep(30);
				queue_delayed_work(microp_intr_wq, &g_uP_info->work,0);
				return;
			}                                            

			if( is_Mode_APROM()){
				uP_nuvoton_write_reg(MICROP_IND_A68_READY,&a68_ready);
				uP_nuvoton_read_reg(MICROP_INTR_STATUS,&reg_intr_sta);
				uP_nuvoton_read_reg(MICROP_GPIO_INPUT_LEVEL,&reg_input);
				pr_debug("MicroP intr: status=%8x, input=%8x\r\n",reg_intr_sta,reg_input);

				if(IsPadNoIntrEvt()){
					pr_debug("MicroP No Event !! wait 5ms.\r\n"); //wait shared irq line rebounced back
					msleep(5); //wait gpio 107 from low -> high again
				}
				else{        
					if(IsPadRevSleepReminder()){
						printk("Rev MicroP Sleep Reminder!!\r\n");
					}
                                        if(IsPadShutdownByEC()){
                                                printk("MicroP PAD Shutdown by EC !!\r\n");
                                                micropSendNotify(P01_BATTERY_POWER_BAD);
                                        }
					if(IsPadHavingBat5Percent()){
                                                printk("Pad BAT 6P -> 5P!!\r\n");
                                                checkPadLowCapacity();
					}
					if(IsPadHavingProxAct() && IsPadProxHavingEvts()){
							printk("MicroP HavingProxAct!!\r\n");
							micropSendNotify(PAD_CAP_SENSOR_STATUS_CHANGE);
					}

					if(IsPadACUSBInOut()){
						uP_nuvoton_read_reg(MICROP_USB_DET, &usb_type);

						if(usb_type==P01_CABLE_CHARGER){
							printk("MicroP Pad AC In!!\r\n"); 
#ifndef ASUS_FACTORY_BUILD
							//Without early turn on Pad power when ac/usb in for factory version
							AX_MicroP_enterResuming();
#endif
							micropSendNotify(P01_AC_USB_IN);
						}
						else if(usb_type==P01_CABLE_USB){
							printk("MicroP Pad USB In!!\r\n");
#ifndef ASUS_FACTORY_BUILD
							//Without early turn on Pad power when ac/usb in for factory version
							AX_MicroP_enterResuming();
#endif
							micropSendNotify(P01_AC_USB_IN);
							g_b_isP01USBConnected=1;
						}
						else if(usb_type==P01_CABLE_OTG){
							printk("MicroP Pad OTG In!!\r\n");
							micropSendNotify(PAD_USB_OTG_ENABLE);
						}
						else if(usb_type==P01_CABLE_NO){
							printk("MicroP Pad AC_USB_OUT!!\r\n");
#ifndef ASUS_FACTORY_BUILD
							//Without early turn on Pad power when ac/usb in for factory version
							AX_MicroP_enterResuming();
#endif
							micropSendNotify(P01_AC_USB_OUT);
							g_b_isP01USBConnected=0;
                                                        checkPadLowCapacity();
						}
						switch_set_state(&p01_switch_usb_cable,g_b_isP01USBConnected);
						printk("[Microp]set usb plugged uevent:%d\n",g_b_isP01USBConnected);
					}

					if(IsPadVolDnEvt()){
						printk("MicroP VolDnKey !!\r\n");
						if(IsPadVolDnPressed()){                                    
							micropSendNotify(P01_VOLDN_KEY_PRESSED);
						}
						else
							micropSendNotify(P01_VOLDN_KEY_RELEASED);                                            
					}

					if(IsPadVolUpEvt()){
						printk("MicroP VolUpKey !!\r\n");
						if(IsPadVolUpPressed()){
							micropSendNotify(P01_VOLUP_KEY_PRESSED);
						}
						else
							micropSendNotify(P01_VOLUP_KEY_RELEASED);
					}

					if(IsPadPwrBtnEvt()){

						if(IsPadPwrBtnPressed())	 //pressed
						{
							printk("MicroP PwrKey Dn, Pre:%d \r\n",g_pad_pwrkey_pre_pressed);							
							micropSendNotify(P01_PWR_KEY_PRESSED);
							g_pad_pwrkey_pre_pressed=1;
						}
						else if(!IsPadPwrBtnPressed())	//released
						{
							printk("MicroP PwrKey Up, Pre:%d \r\n",g_pad_pwrkey_pre_pressed);
	 						if(g_pad_pwrkey_pre_pressed==0){
								printk("MicroP Re-send PwrKey dn first!!\r\n");						
								micropSendNotify(P01_PWR_KEY_PRESSED);	//might lose press evt
	 						}
								
							micropSendNotify(P01_PWR_KEY_RELEASED);
							g_pad_pwrkey_pre_pressed=0;
						}
					}
					//Eason: add PWR key fake key---

					if(IsPadHavingLSEvt()){
						printk("MicroP Light Sensor Having Event !!\r\n");
						micropSendNotify(P01_LIGHT_SENSOR);
					}

					if(IsPadBatStatusChangeEvt()){
						printk("MicroP bat status change event !!\r\n");
						micropSendNotify(P05_BAT_STATUS_CHANGE);
					}                                        
				}                                 
			}
			else{
				printk("LDROM mode skip Evt, g_curr_uP_mode=%d\r\n", g_curr_uP_mode);
				break;
			}
		}
	}
	else if(st_DISCONNECTED==g_b_isP01Connected){
		msleep(400);
		uP_nuvoton_read_reg(MICROP_POWER_ON_REASON, &reg_powerOnReason);
		if(reg_powerOnReason > 0){
			printk("MicroP PowerOn reason=%d\r\n", reg_powerOnReason);
			microp_reconnected();
		}

	}

	pr_debug("MicroP wq <%d, %d>---\r\n", g_b_isP01Connected,gpio_get_value(g_microp_irq_gpio));
}



static irqreturn_t microP_irq_handler(int irq, void *dev_id)
{
        struct microP_info *info =
            (struct microP_info *)dev_id;
        queue_delayed_work(microp_intr_wq, &info->work, 0);
        
        return IRQ_HANDLED;

}

//Eason: workaround for Pad touch may hold intr gpio low when init+++
/*
* Microp & touch share IRQ
* Pad touch may hold intr gpio low(can't receive next irq state) when init. 
* If microp event comes in this period, handler can't handle, will cause both Padtouch & Pad can't recieve irq.
*/
void forceDoMicropIrqHandler(void)
{
	if( 0== gpio_get_value(g_microp_irq_gpio) )
	{
		queue_delayed_work(microp_intr_wq, &g_uP_info->work, 0);
		printk("%s\n",__FUNCTION__);
	}
}
//Eason: workaround for Pad touch may hold intr gpio low when init---


int getCmdID(int addr){
    int i=0;
    for(i=0;i < ARRAY_SIZE(uP_CMD_Table);i++){
        if(addr==uP_CMD_Table[i].addr)
            return i;
    }
    return -1;
}


int isFirmwareUpdating(void){        
    return (!is_Mode_APROM());
}

/*
 *       Firmware update procedure via I2C
 *      - Change to LDROM using CMD 0x51
 *      - 
 *
 *
*/

static int updateMicroPFirmware(unsigned long arg){
	uint8_t i2cdata[32]={0};

	int i=0;
	int j=0;
	int val=0;
	int ret=0;
	int num_updated_bytes=0;

	uint32_t u32_tmp_addr=0;
	uint32_t u32_current_prog_addr=0;
	uint32_t u32_pre_success_prog_addr=0;        

	uint8_t a68_ready=0x96;
		
	/*
	uint32_t backup_output=0;
	uint32_t backup_intr_en=0;
	*/
	uint32_t reg_input=0;
	if(img_buf==NULL)
              printk("F/W alloc size %x\r\n", g_firmwareSize);
		img_buf = kzalloc(sizeof(uint8_t)*g_firmwareSize, GFP_KERNEL);
        
	if(!img_buf){
		pr_err("%s: Unable to allocate memory!\n",__FUNCTION__);
		return -ENOMEM;
	}

	memset(img_buf,0,sizeof(uint8_t)*g_firmwareSize);

	/* first element of arg must be u32 with id of module to talk to */
	if(copy_from_user(img_buf, (const void __user *)arg, sizeof(uint8_t)*g_firmwareSize)) {
		pr_err("%s: Failed to copy arg from user", __FUNCTION__);
		return -EFAULT;
	}

	uP_nuvoton_read_reg(MICROP_GPIO_INPUT_LEVEL,&reg_input);

	g_fw_update_progress=1;
        
	printk("MicroP disable irq first\r\n");
	disable_irq(g_uP_info->i2c_client->irq);

	if(uP_nuvoton_read_reg(MICROP_BOOT_SELECTION,&g_curr_uP_mode))
		printk("Microp %s\r\n",g_curr_uP_mode?"@LDROM":"@APROM");


	i2cdata[0] = 0x5A;
	uP_nuvoton_write_reg(MICROP_SET_BOOT_LDROM, i2cdata);

	if(uP_nuvoton_read_reg(MICROP_BOOT_SELECTION,&g_curr_uP_mode)){
		printk("Boot selection => %s\r\n",g_curr_uP_mode?"@LDROM":"@APROM");
		if(g_curr_uP_mode!=1){
			printk("Cannot Change To LDROM: Fatal Error\r\n");
			g_fw_update_progress=0;
			return -EAGAIN;
		}
		uP_nuvoton_read_reg(MICROP_FW_VER,&g_microp_ver);
		printk("MicroP: enter into LDROM ver=0x%x\r\n", g_microp_ver);

	}

	u32_current_prog_addr=0;
	ret=uP_nuvoton_write_reg(MICROP_LD_PROG_PROGRESS, &u32_current_prog_addr); //reset program counter
	if(ret){
		ret=uP_nuvoton_read_reg(MICROP_LD_PROG_PROGRESS, &u32_current_prog_addr); //read program counter                
		if(ret){
			printk("MicroP update : initial LD_PROG=%x\r\n", u32_current_prog_addr);                                
			if(u32_current_prog_addr!=0)
				goto update_fw_failed;
		}
	}
	if(ret < 0){
		printk("MicroP update failed: I2C write MICROP_LD_PROG_PROGRESS failed\r\n");
		goto update_fw_failed;
	}

	for (num_updated_bytes = 0; num_updated_bytes < (g_firmwareSize);){
		for (j = 0;j < 32;j ++){
			i2cdata[j] = *(img_buf+u32_current_prog_addr+j);
		}
		ret=uP_nuvoton_write_reg(MICROP_PROGRAM_APROM, i2cdata);
		if(ret < 0){
			printk("MicroP update failed: I2C write failed\r\n");
			goto update_fw_failed;
		}

		msleep(50);
		do{
			u32_tmp_addr=u32_current_prog_addr;
			ret=uP_nuvoton_read_reg(MICROP_LD_PROG_PROGRESS, &u32_current_prog_addr);
			if(ret < 0){
				printk("MicroP update failed: I2C read progress failed\r\n");
				goto update_fw_failed;
			}

			if((u32_current_prog_addr>=0 && u32_current_prog_addr <= g_firmwareSize) && (u32_tmp_addr==u32_current_prog_addr))
				break;
			else
				msleep(20);
		}while(1);

		printk("<cur: %x, pre: %x>\r\n", u32_current_prog_addr, u32_pre_success_prog_addr);

		if(u32_current_prog_addr == (u32_pre_success_prog_addr+0x20)){
			u32_pre_success_prog_addr=u32_current_prog_addr;
			num_updated_bytes+=32;
			i += 32;
			g_fw_update_progress=((num_updated_bytes*100) - 1)/g_firmwareSize;                                
			if(num_updated_bytes % 1024 ==0){
				printk("==> [ %d %%]\r\n", g_fw_update_progress);
			}
			if(u32_current_prog_addr==g_firmwareSize){
				printk("update [0x3fe0]~[0x4000] , num_updated_bytes=0x%x => [ 99 %%]\r\n", num_updated_bytes);
				break;
			}
		}
		else{
			printk("MicroP update: addr 0x[%x]~ 0x[%x] might have failure, re-send\r\n", u32_pre_success_prog_addr, u32_current_prog_addr);
			u32_current_prog_addr=u32_pre_success_prog_addr=(u32_pre_success_prog_addr & 0xfffffe00);
			num_updated_bytes=u32_current_prog_addr;

			printk(" MicroP update: trying to set prog addr 0x[%8x]\r\n", u32_current_prog_addr);                                
			ret=uP_nuvoton_write_reg(MICROP_LD_PROG_PROGRESS, &u32_current_prog_addr);
			if(ret < 0)
				goto update_fw_failed;
			ret=uP_nuvoton_read_reg(MICROP_LD_PROG_PROGRESS, &u32_current_prog_addr);
			if(ret < 0)
				goto update_fw_failed;
			printk(" MicroP update: get prog addr 0x[%8x]\r\n", u32_current_prog_addr);
		}
	}

	printk("MicroP done\r\n");
	msleep(10);
	printk("MicroP Update APROM finish..rebooting\r\n");

	i2cdata[0] = 0xA5;
	uP_nuvoton_write_reg(MICROP_SET_BOOT_APROM, i2cdata);

	msleep(3000);

	if(uP_nuvoton_read_reg(MICROP_BOOT_SELECTION,&g_curr_uP_mode))
		printk("MicroP After Update: Microp %s\r\n",g_curr_uP_mode?"@LDROM":"@APROM");

	if(g_curr_uP_mode==1){
		printk("MicroP update failed: cannot enter into APROM mode\r\n");
		ret=-EAGAIN;
		goto update_fw_failed;
	}

	uP_nuvoton_read_reg(MICROP_FW_VER,&g_microp_ver);
	printk("MicroP: back to APROM new ver=0x%x\r\n", g_microp_ver);

	uP_nuvoton_write_reg(MICROP_IND_A68_READY,&a68_ready);
	AX_MicroP_setPWMValue(33);


/* need modification
	printk("switch backlight on\r\n");
	bl_pad_add_func(NULL);
*/
	uP_nuvoton_read_reg(MICROP_INTR_STATUS,&val);
	uP_nuvoton_read_reg(MICROP_INTR_STATUS,&val);
	printk("MicroP flush discard intr status=%8x\r\n",val);

/*
	uP_nuvoton_write_reg(MICROP_GPIO_OUTPUT_LEVEL, &backup_output);
	uP_nuvoton_write_reg(MICROP_INTR_EN, &backup_intr_en);
       
	uP_nuvoton_read_reg(MICROP_GPIO_OUTPUT_LEVEL,&backup_output);
	uP_nuvoton_read_reg(MICROP_INTR_EN,&backup_intr_en);
	printk("after restore out, intr_en = <%x, %x>\r\n", backup_output, backup_intr_en);
*/

	g_fw_update_progress=100;     
       
update_fw_failed:
	enable_irq(g_uP_info->i2c_client->irq);
	printk("MicroP F/W done. enable irq again\r\n");
	if(img_buf)
		kfree(img_buf);
	img_buf=NULL;

	uP_nuvoton_read_reg(MICROP_GPIO_INPUT_LEVEL,&reg_input);

	micropSendNotify(PAD_UPDATE_FINISH);

	return ret;
}
/*
* file operation implementation
*/

static int asus_microp_open(struct inode *inode, struct file *flip){
        pr_debug("%s\r\n",__FUNCTION__);
        return 0;
}
static int asus_microp_release(struct inode *inode, struct file *flip){
        pr_debug("%s\r\n",__FUNCTION__);
        return 0;

}
static long asus_microp_ioctl(struct file *flip, unsigned int cmd, unsigned long arg){

	int ret=0;
	
#ifdef ASUS_FACTORY_BUILD
	unsigned int sel_offset = 0;
	uint8_t set_factory_mode = 0xCC;
#endif

       int pad_hwid=-1;
       char sz_oemVerStr[24]={0};
	pr_info("%s\r\n",__FUNCTION__);
           
	if(_IOC_TYPE(cmd) != ASUS_MICROP_IOC_TYPE)
		return -ENOTTY;
	if(_IOC_NR(cmd) > ASUS_MICROP_MAX_NR)
		return -ENOTTY;

	if(g_uP_info==NULL)
		return -EFAULT;
       
	switch (cmd) {
		case ASUS_MICROP_FW_UPDATE:
			pr_info("ioctl: ASUSEC_FW_UPDATE ++\r\n");
			ret=updateMicroPFirmware(arg);
			pr_info("ioctl: ASUSEC_FW_UPDATE --\r\n");
			break;
		case ASUS_MICROP_CHECK_CONNECTED:                    
			ret=__put_user((g_b_isP01Connected==st_CONNECTED),(int __user *)arg);
			break;
		case ASUS_MICROP_GET_FW_VERSION:
			if(isMicroPConnected())
				ret=__put_user(g_microp_ver,(int __user *)arg);
			else
				ret=-EINVAL;
			break;
		case ASUS_MICROP_ON_OFF_GPS_ON_PAD:
			pr_info("ioctl: ASUS_MICROP_ON_OFF_GPS_ON_PAD (NOT USED)++\\r\n");
/*                                    
			if(isMicroPConnected()){
				ret=__get_user(val,(int __user *)arg);                        
			}
			else
				ret=-EINVAL;
			if(ret >=0){
				pr_info("ioctl: ASUS_MICROP_ON_OFF_GPS_ON_PAD val=%dr\n", val);
				AX_MicroP_setGPIOOutputPin(OUT_uP_GPS_LNA_EN,val);                            
			}
*/                        
			pr_info("ioctl: ASUS_MICROP_ON_OFF_GPS_ON_PAD  (NOT USED) --\r\n");                    
			break;
		case ASUS_MICROP_GET_LDROM_VERSION:
			if(isMicroPConnected())
				ret=__put_user(g_ldrom_ver,(int __user *)arg);
			else
				ret=-EINVAL;

			break;
		case ASUS_MICROP_GET_PADPHONE_HW_ID:
		{
			int hwid = AX_MicroP_getHWID();
			ret=__put_user(hwid, (int __user *)arg);
			break;
		}
#ifdef ASUS_FACTORY_BUILD
		case ASUS_MICROP_POWER_ON_LED_O:
			uP_nuvoton_write_reg(MICROP_DISABLE_CHARGING_FOR_FACTORY, &set_factory_mode);
			sel_offset = 1 << OUT_uP_LED_O;
			ret = uP_nuvoton_write_reg(MICROP_GPIO_OUTPUT_BIT_SET, &sel_offset);
			break;
		case ASUS_MICROP_POWER_OFF_LED_O:
			uP_nuvoton_write_reg(MICROP_DISABLE_CHARGING_FOR_FACTORY, &set_factory_mode);
			sel_offset = 1 << OUT_uP_LED_O;
			ret = uP_nuvoton_write_reg(MICROP_GPIO_OUTPUT_BIT_CLR, &sel_offset);
			break;
		case ASUS_MICROP_POWER_ON_LED_G:
			uP_nuvoton_write_reg(MICROP_DISABLE_CHARGING_FOR_FACTORY, &set_factory_mode);
			sel_offset = 1 << OUT_uP_LED_G;
			ret = uP_nuvoton_write_reg(MICROP_GPIO_OUTPUT_BIT_SET, &sel_offset);
			break;
		case ASUS_MICROP_POWER_OFF_LED_G:
			uP_nuvoton_write_reg(MICROP_DISABLE_CHARGING_FOR_FACTORY, &set_factory_mode);
			sel_offset = 1 << OUT_uP_LED_G;
			ret = uP_nuvoton_write_reg(MICROP_GPIO_OUTPUT_BIT_CLR, &sel_offset);
			break;
#endif
               case ASUS_MICROP_IS_UNDER_APROM:
			if(isMicroPConnected())
				ret=__put_user(is_Mode_APROM(),(int __user *)arg);
			else
				ret=-EINVAL;
                     pr_debug("ioctl: ASUS_MICROP_IS_UNDER_APROM: ret=%d\r\n", is_Mode_APROM());
                     break;
               case ASUS_MICROP_PAD_MODEL_NAME:
			if(isMicroPConnected()){
                                uP_nuvoton_read_reg(MICROP_OEM_FW_VERSION,sz_oemVerStr);
                                printk("ioctl: ASUS_MICROP_PAD_MODEL_NAME: ret=%s\r\n", sz_oemVerStr);
                            	if (copy_to_user((void __user *)arg, sz_oemVerStr, strlen(sz_oemVerStr) + 1))
                                          ret = -EFAULT;                                        
                     }
			else
				ret=-EINVAL;
                
                     break;
		case ASUS_MICROP_PAD_HW_ID:
			if(isMicroPConnected()){
				pad_hwid=AX_MicroP_getHWID();
				if(pad_hwid >= 0)
					ret=__put_user(pad_hwid,(int __user *)arg);
				else 
					ret=-EINVAL;
			}
			else
				ret=-EINVAL;
				
			break;
		default:
			pr_err("Invalid ioctl code\r\n");
			ret= -EINVAL;
			break;
        }

	return ret;
}


static struct file_operations asus_microp_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = asus_microp_ioctl,
	.open = asus_microp_open,
	.release = asus_microp_release,
};

static struct miscdevice asus_microp_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = TEGRA_MICROP_NAME,
	.fops = &asus_microp_fops,
	.mode= 0666,
};


static ssize_t pad_ReportI2CFail_show(struct device *dev, struct device_attribute *attr, char *buf)
{
        int count =0;
        if(g_prop_virtualRemoveEnabled)
            count+=sprintf(buf,"virtual remove enabled\r\n");
        else
            count+=sprintf(buf,"virtual remove disabled\r\n");

        return count;
}
static ssize_t pad_ReportI2CFail_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
       int val=0;

       val = simple_strtol(buf, NULL, 10);       

        if(val > 0){
                printk("[MicroP] enable Virtual Remove!!\r\n");
                g_prop_virtualRemoveEnabled=1;
        }
        else{
                printk("[MicroP] disable Virtual Remove!!\r\n");
                g_prop_virtualRemoveEnabled=0;
        }
 
 
	return count;
}


static ssize_t Allow_UsingP05_show(struct device *dev, struct device_attribute *attr, char *buf)
{
        int count =0;
        if(g_prop_Allow_UsingP05)
            count+=sprintf(buf,"[MicroP] P05 Enabled!!\r\n");
        else
            count+=sprintf(buf,"[MicroP] P05 Disabled!!\r\n");

        return count;
}
static ssize_t Allow_UsingP05_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
       int val=0;

       val = simple_strtol(buf, NULL, 10);       

        if(val > 0){
                printk("[MicroP] Enable Using P05!!\r\n");
                g_prop_Allow_UsingP05=1;
        }
        else{
                printk("[MicroP] Disable Using P05!!\r\n");
                g_prop_Allow_UsingP05=0;
        }
 
 
	return count;
}

/*
* show/restory interface
*/
static int g_cmd_idx=0;
static ssize_t reg_show(struct device *dev, struct device_attribute *attr,
				 char *buf)
{
       uint64_t val=0;
       int count =0;
       int status=0;
       char sz_isn[32]={0};
       
	if(st_CONNECTED!=g_b_isP01Connected){
            	count+=sprintf(buf,"microp: failed, not connected\r\n");
		return count;
	}

       if(uP_CMD_Table[g_cmd_idx].len >= 32){
               status=uP_nuvoton_read_reg(g_cmd_idx, sz_isn); 
              if(status > 0){                    
                    count+=sprintf(buf, "\"%16s\": addr[0x%2x] = %32s(ASCII)\r\n",uP_CMD_Table[g_cmd_idx].name, 
                                                                                uP_CMD_Table[g_cmd_idx].addr, 
                                                                                sz_isn);
               }
               else 
                    count+=sprintf(buf,"reg read failed %d\r\n",status);
       }
       else{
               status=uP_nuvoton_read_reg(g_cmd_idx,&val);
               if(status > 0){                    
                    count+=sprintf(buf, "\"%16s\": addr[0x%2x] = %16llx(h)\r\n",uP_CMD_Table[g_cmd_idx].name, 
                                                                                uP_CMD_Table[g_cmd_idx].addr, 
                                                                                val);
               }
               else 
                    count+=sprintf(buf,"reg read failed %d\r\n",status);

       }
        
	return count;
}
static ssize_t reg_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
    	int addr=0;
       uint64_t val=0;
	if(st_CONNECTED!=g_b_isP01Connected){
		pr_info("microp: failed, not connected\r\n");
		return 0;
	}
       if(buf[0]=='-'){
            addr = simple_strtol(buf+1, NULL, 16);       
            g_cmd_idx=getCmdID(addr);
            pr_info("set addr=0x%2x, ID=%d\r\n",addr,g_cmd_idx);
            if(g_cmd_idx==-1)
                    g_cmd_idx=0;
       }
       else{    //write value to idx
                if(g_cmd_idx<0 && g_cmd_idx > ARRAY_SIZE(uP_CMD_Table))
                        pr_info("set addr first\r\n");
                else{
                        val=simple_strtol(buf, NULL, 16);
                        pr_info("write val 0x%llx to addr [0x%x]\r\n", val, uP_CMD_Table[g_cmd_idx].addr);
                        uP_nuvoton_write_reg(g_cmd_idx, &val);
                }
       }

	return count;
}


/*
* show/restory interface
*/
static ssize_t pad_update_progress_show(struct device *dev, struct device_attribute *attr,
				 char *buf)
{
	return sprintf(buf, "%d\r\n",g_fw_update_progress);
}



static ssize_t isn_show(struct device *dev, struct device_attribute *attr,
				 char *buf)
{
	char isn[32]={0};
	int count =0;
	int status=0;

	if(st_CONNECTED!=g_b_isP01Connected){
		count+=sprintf(buf,"microp: failed, not connected\r\n");
		return count;
	}

	status=uP_nuvoton_read_reg(MICROP_ISN,isn);
	if(status > 0){                    
		count+=sprintf(buf, "%32s\r\n", isn);
	}
	else 
		count+=sprintf(buf,"0\r\n");

	return count;
}

static ssize_t oem_ssn_show(struct device *dev, struct device_attribute *attr,
				 char *buf)
{
	char oem_ssn[32]="";
	int count =0;
	int status=0;

	if(st_CONNECTED!=g_b_isP01Connected){
		count+=sprintf(buf,"microp: failed, not connected\r\n");
		return count;
	}

	status=uP_nuvoton_read_reg(MICROP_OEM_SSN,oem_ssn);
	if(status > 0){                    
		count+=sprintf(buf, "%32s\r\n", oem_ssn);
	}
	else 
		count+=sprintf(buf,"0\r\n");

	return count;
}

static ssize_t oem_ssn_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	char oem_ssn[32]="";
	if(st_CONNECTED!=g_b_isP01Connected){
		pr_info("microp: failed, not connected\r\n");
		return 0;
	}
	if(buf){
		strcpy(oem_ssn,buf);
		printk("try to write oem isn %32s\r\n", oem_ssn);
		uP_nuvoton_write_reg(MICROP_OEM_SSN, oem_ssn);
	}
	return count;
}

static ssize_t oem_imei_show(struct device *dev, struct device_attribute *attr,
				 char *buf)
{
	char oem_imei[32]="";
	int count =0;
	int status=0;

	if(st_CONNECTED!=g_b_isP01Connected){
		count+=sprintf(buf,"microp: failed, not connected\r\n");
		return count;
	}

	status=uP_nuvoton_read_reg(MICROP_OEM_IMEI,oem_imei);
	if(status > 0){                    
		count+=sprintf(buf, "%32s\r\n", oem_imei);
	}
	else 
		count+=sprintf(buf,"0\r\n");

	return count;
}

static ssize_t oem_imei_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	char oem_imei[32]="";
	if(st_CONNECTED!=g_b_isP01Connected){
		pr_info("microp: failed, not connected\r\n");
		return 0;
	}
	if(buf){
		strcpy(oem_imei,buf);
		printk("try to write oem imei %32s\r\n", oem_imei);
		uP_nuvoton_write_reg(MICROP_OEM_IMEI, oem_imei);
	}
	return count;
}

static ssize_t pad_mydp_new_sku_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count=0;
	
	if(st_CONNECTED!=g_b_isP01Connected){
		count += sprintf(buf,"0\r\n");   // return 0 if Pad not connected
	}
	else{
		if(AX_MicroP_IsMydpNewSKU() == 1){
			count += sprintf(buf,"1\r\n");        // return 1 if mydp is new sku
		}
		else{
			count += sprintf(buf,"0\r\n");        
		}
	}
	
	return count;
}


static ssize_t pad_model_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count=0;
	
	if(st_CONNECTED!=g_b_isP01Connected){
		count += sprintf(buf,"Not Connected\r\n");   // return 0 if Pad not connected
	}
	else{
		switch(AX_MicroP_getPadModel()){
			case PAD_P05_P05E:
				count += sprintf(buf,"PAD_P05_P05E\r\n");		
				break;
			case PAD_P71L:
				count += sprintf(buf,"PAD_P71L\r\n");		
				break;
			case PAD_P101G:
				count += sprintf(buf,"PAD_P101G\r\n");		
				break;
			case PAD_P92L:
				count += sprintf(buf,"PAD_P92L\r\n");		
				break;
			case PAD_P93L:
				count += sprintf(buf,"PAD_P93L\r\n");		
				break;
			case PAD_P05C:
				count += sprintf(buf,"PAD_P05C\r\n");		
				break;
			default:
				count += sprintf(buf,"UnKnown\r\n");		
				break;
		}
	}
	
	return count;
}

static DEVICE_ATTR(pad_ReportI2CFail, 0644, pad_ReportI2CFail_show, pad_ReportI2CFail_store);
static DEVICE_ATTR(Allow_UsingP05, 0644, Allow_UsingP05_show, Allow_UsingP05_store);
#ifdef ASUS_FACTORY_BUILD
	static DEVICE_ATTR(reg, 0666, reg_show, reg_store);
#else
	static DEVICE_ATTR(reg, 0664, reg_show, reg_store);
#endif
static DEVICE_ATTR(pad_update_progress, 0644, pad_update_progress_show, NULL);
static DEVICE_ATTR(isn, 0644, isn_show, NULL);
#ifdef ASUS_FACTORY_BUILD
static DEVICE_ATTR(oem_ssn, 0666, oem_ssn_show, oem_ssn_store);
static DEVICE_ATTR(oem_imei, 0666, oem_imei_show, oem_imei_store);
#else
static DEVICE_ATTR(oem_ssn, 0664, oem_ssn_show, oem_ssn_store);
static DEVICE_ATTR(oem_imei, 0664, oem_imei_show, oem_imei_store);
#endif
static DEVICE_ATTR(pad_mydp_new_sku, 0644, pad_mydp_new_sku_show, NULL);
static DEVICE_ATTR(pad_model, 0444, pad_model_show, NULL);


//gpio switch class related-functions

static ssize_t p01_switch_name(struct switch_dev *sdev, char *buf)
{
       char model_name[30]={0};
       if(st_CONNECTED==g_b_isP01Connected)
                sprintf(model_name,"p%03x",g_microp_ver);
       else
                sprintf(model_name,"unknown");
       
	return sprintf(buf, "%s\n", model_name);
}


static ssize_t p01_switch_state(struct switch_dev *sdev, char *buf)
{        
	return sprintf(buf, "%s\n", (g_b_isP01Connected)?"1":"0");
}



static ssize_t p01_usb_switch_name(struct switch_dev *sdev, char *buf)
{
       
	return sprintf(buf, "Asus-P01\n");
}


static ssize_t p01_usb_switch_state(struct switch_dev *sdev, char *buf)
{
//	return sprintf(buf, "%s\n", (g_b_isP01USBConnected ? "1" : "0"));
       return sprintf(buf, "%d\n", g_b_isP01USBConnected);
}



static ssize_t pad_notify_switch_state(struct switch_dev *sdev, char *buf)
{
	int count=0;
       count+= sprintf(buf,"%d\n", g_uPadErrStatus);
       return count;
}

//Eason: add microp suspend_noirq to check if IRQ handler work pending+++
static int microp_suspend_noirq(struct device *dev)
{
	if( (st_CONNECTED==g_b_isP01Connected)&&(0==gpio_get_value(9)) )
	{
		printk("[MicroP]%s:IRQ wait for complete\n",__FUNCTION__);
		return -EBUSY;
	}
	printk("[MicroP]%s ,pin level:%d\r\n",__FUNCTION__,gpio_get_value(9));
	return 0;
}
//Eason: add microp suspend_noirq to check if IRQ handler work pending---

static int microp_suspend( struct device *dev )
{
       printk("%s ++\r\n",__FUNCTION__);
   
       if(AX_MicroP_IsP01Connected()){              
#ifdef ASUS_FACTORY_BUILD                        
               AX_MicroP_enablePinInterrupt(INTR_EN_VOL_DOWN | INTR_EN_VOL_UP|INTR_EN_CAP_RPOX_INT, 0);
#else
               AX_MicroP_enablePinInterrupt(INTR_EN_VOL_DOWN | INTR_EN_VOL_UP, 0);
#endif
        }
       enable_irq_wake(g_uP_info->i2c_client->irq);
	 printk("%s --\r\n",__FUNCTION__);
	return 0;
}

static int microp_resume( struct device *dev )
{
       // for debug
       uint32_t reg_input=0, out_reg=0, state=0 ;
       uint8_t reg_powerOnReason;
       printk("%s ++\r\n",__FUNCTION__);
       disable_irq_wake(g_uP_info->i2c_client->irq);
       if(AX_MicroP_IsP01Connected() && pad_exist()){       
#ifdef ASUS_FACTORY_BUILD                               
               AX_MicroP_enablePinInterrupt(INTR_EN_VOL_DOWN | INTR_EN_VOL_UP|INTR_EN_CAP_RPOX_INT, 1);
#else
               AX_MicroP_enablePinInterrupt(INTR_EN_VOL_DOWN | INTR_EN_VOL_UP, 1);
#endif

            if(uP_nuvoton_read_reg(MICROP_GPIO_INPUT_LEVEL, &reg_input) > 0 
                && uP_nuvoton_read_reg(MICROP_GPIO_OUTPUT_LEVEL, &out_reg) > 0
                && uP_nuvoton_read_reg(MICROP_OPERATING_STATE, &state) > 0)
                    printk("[PAD DEBUG] state=%d, uP In=0x%8x, Out=0x%8x, gpio 9=%d\r\n", state, reg_input, out_reg, gpio_get_value(9));
                    ASUSEvtlog("[PAD DEBUG] state=%d, uP In=0x%8x, Out=0x%8x, gpio 9=%d\r\n", state, reg_input, out_reg, gpio_get_value(9));
                    if(st_MICROP_Off==state){
                            uP_nuvoton_read_reg(MICROP_POWER_ON_REASON, &reg_powerOnReason);
                            printk("reg_powerOnReason=%d\r\n", reg_powerOnReason);
                            if(E_ON_PWR_KEY_LONGPRESS==reg_powerOnReason){
                                    printk("resume might due to long press pwr key\r\n");
                                    if(IsPadPwrBtnPressed()){
                                                printk("compensate sending pwr key pressed\r\n");
                                                micropSendNotify(P01_PWR_KEY_PRESSED);
                                                g_pad_pwrkey_pre_pressed=1;
                                    }
                            }
                    }
       }
              
       printk("%s --\r\n",__FUNCTION__);
	return 0;
}


static int microP_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct microP_info *info;
	int ret=0;
#if 0    
	struct proc_dir_entry *entry;       
#endif
	pr_info("microP: %s +++\n", __FUNCTION__);
	g_uP_info = info = kzalloc(sizeof(struct microP_info), GFP_KERNEL);
	if (!info) {
		pr_err("microP: Unable to allocate memory!\n");
		return -ENOMEM;
	}

	g_slave_addr=client->addr;
	info->i2c_client = client;
	i2c_set_clientdata(client, info);

	microp_slow_job_wq = create_singlethread_workqueue("microp_slow_job");
	microp_ins_rev_wq = create_singlethread_workqueue("microp_ins_rev_wq");
	microp_intr_wq = create_singlethread_workqueue("microp_intr_wq");

	// initialize work struct
	INIT_DELAYED_WORK(&info->work, microP_work);
	INIT_DELAYED_WORK(&info->initP01, initP01);
	INIT_DELAYED_WORK(&info->deinitPad, deinitPad);
	INIT_DELAYED_WORK(&info->poweroffP01, microp_poweroff);
        INIT_DELAYED_WORK(&info->reportPadFail, reportPadFailWork);
        
	// init mutex
	mutex_init(&microp_mutex_lock);

	g_microp_irq_gpio = of_get_named_gpio_flags(client->dev.of_node, "microp,irq-gpio",0,NULL);

	ret = gpio_request(g_microp_irq_gpio, client->name);
	if (ret < 0){
		pr_err("microP: gpio_request fail!\n");
		goto init_uP_fail;
	}

	ret = gpio_direction_input(g_microp_irq_gpio);
	if (ret < 0){
		pr_err("microP: gpio_direction_input fail!\n");
		goto init_uP_fail;
	}

	ret = request_irq(gpio_to_irq(g_microp_irq_gpio), microP_irq_handler,
		IRQF_TRIGGER_FALLING | IRQF_SHARED , client->name, info);

	if (ret < 0){
		pr_err("microP: gpio_direction_input fail!\n");
		goto init_uP_fail;
	}

	ret = misc_register(&asus_microp_device);
	if (ret < 0) {
		pr_err("%s: Unable to register misc device!\n", __FUNCTION__);
		goto init_uP_fail;
	}

	ret = device_create_file(&client->dev, &dev_attr_reg);
	if (ret < 0) {
		pr_err("%s: Unable to create file! %d\n",  __FUNCTION__,ret);
	}

	ret = device_create_file(&client->dev, &dev_attr_pad_update_progress);
	if (ret < 0) {
		pr_err("%s: Unable to create file! %d\n", __FUNCTION__,ret);
	}
    

	ret = device_create_file(&client->dev, &dev_attr_pad_ReportI2CFail);
	if (ret < 0) {
		pr_err("%s: Unable to create file! %d\n", __FUNCTION__,ret);
	}

	ret = device_create_file(&client->dev, &dev_attr_Allow_UsingP05);
	if (ret < 0) {
		pr_err("%s: Unable to create file! %d\n", __FUNCTION__,ret);
	}

	ret = device_create_file(&client->dev, &dev_attr_isn);
	if (ret < 0) {
		pr_err("%s: Unable to create file! %d\n", __FUNCTION__,ret);
	}

	ret = device_create_file(&client->dev, &dev_attr_oem_ssn);
	if (ret < 0) {
		pr_err("%s: Unable to create file! %d\n", __FUNCTION__,ret);
	}

	ret = device_create_file(&client->dev, &dev_attr_oem_imei);
	if (ret < 0) {
		pr_err("%s: Unable to create file! %d\n", __FUNCTION__,ret);
	}

	ret = device_create_file(&client->dev, &dev_attr_pad_mydp_new_sku);
	if (ret < 0) {
		pr_err("%s: Unable to create file! %d\n", __FUNCTION__,ret);
	}

	ret = device_create_file(&client->dev, &dev_attr_pad_model);
	if (ret < 0) {
		pr_err("%s: Unable to create file! %d\n", __FUNCTION__,ret);
	}	
	

	
	// registered as gpio switch device
	p01_switch_dev.name="pfs_pad_ec";
	p01_switch_dev.print_state=p01_switch_state;
	p01_switch_dev.print_name=p01_switch_name;
	ret=switch_dev_register(&p01_switch_dev);

	if (ret < 0){
		pr_err("%s: Unable to register switch dev! %d\n", __FUNCTION__,ret);
	}

	p01_switch_usb_cable.name="usbcable";
	p01_switch_usb_cable.print_state=p01_usb_switch_state;
	p01_switch_usb_cable.print_name=p01_usb_switch_name;
	ret=switch_dev_register(&p01_switch_usb_cable);
	if (ret < 0){
		pr_err("%s: Unable to register switch dev! %d\n", __FUNCTION__,ret);
	}

	pad_err_notify.name="pad_notify";
	pad_err_notify.print_state=pad_notify_switch_state;

	ret=switch_dev_register(&pad_err_notify);        
	if (ret < 0){
		pr_err("%s: Unable to register switch dev! %d\n",  __FUNCTION__,ret);
	}


#ifdef CONFIG_DEBUG_FS
	debugfs_create_file("microp", S_IRUGO, NULL, NULL, &debug_fops);
#endif    

#ifdef CONFIG_I2C_STRESS_TEST
	i2c_add_test_case(info->i2c_client, "MicroP",ARRAY_AND_SIZE(gMicroPTestCaseInfo));
#endif

	pr_info("microP: addr= %x, pin=%d\r\n", g_slave_addr,  g_microp_irq_gpio);
	pr_info("microP: %s ---\n", __FUNCTION__);
	return 0;
	
init_uP_fail:
	return ret;
}

static int microP_remove(struct i2c_client *client)
{
	struct microP_info *info = i2c_get_clientdata(client);
       g_uP_info=NULL;
       switch_dev_unregister(&p01_switch_dev);
       misc_deregister(&asus_microp_device);
	kfree(info);
	return 0;
}

const struct i2c_device_id microP_id[] = {
    {"microp", 0},
    {}
};

MODULE_DEVICE_TABLE(i2c, microP_id);

static struct of_device_id microp_i2c_table[] = {
	{ .compatible = "nuvoton,microp",},
	{ },
};

//Eason: add microp suspend_noirq to check if IRQ handler work pending+++
static const struct dev_pm_ops microp_nuvoton_pm_ops = {
	 .suspend_noirq = microp_suspend_noirq,
	 .suspend= microp_suspend,
	 .resume= microp_resume,
};	
//Eason: add microp suspend_noirq to check if IRQ handler work pending---

static struct i2c_driver microp_nuvoton_driver = {
	.driver = {
		.name   = "microp",
		.owner  = THIS_MODULE,
		.of_match_table = microp_i2c_table,
		.pm = &microp_nuvoton_pm_ops,
	},
	.probe      = microP_probe,
	.remove     = microP_remove,

	
	.id_table   = microP_id,
};

MODULE_AUTHOR("Lenter Chang <lenter_chang@asus.com>");
MODULE_DESCRIPTION("EEPROM_Nuvoton driver");
MODULE_LICENSE("GPL");

module_i2c_driver(microp_nuvoton_driver);

//fs_initcall_sync(microP_init);
//module_init(microP_init);
//module_exit(microP_exit);

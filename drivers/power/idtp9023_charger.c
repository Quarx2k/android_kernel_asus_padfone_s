/*
        Wireless charger IDT P9023 Implementation
*/
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <asm/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/module.h>
#include "idtp9023_charger.h"
#include <linux/slab.h>
#include <linux/power_supply.h>
extern void msleep(unsigned int msecs);
static u32 g_IDTP9023_slave_addr=0;
static int g_IDTP9023_reg_value=0;
static int g_IDTP9023_reg_address=0;
//Eason: read RxID +++
//static int IDTP9023_value_since_reg0xA854_shift[6];
//Eason: read RxID ---
static struct IDTP9023_info *g_IDTP9023_info = NULL;

struct IDTP9023_platform_data{
        int intr_gpio;
};
/*
static enum power_supply_property pm_power_props_wireless[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
};

static char *pm_power_supplied_to[] = {
	"battery",
};
*/
struct IDTP9023_info {
       struct i2c_client *i2c_client;
       struct IDTP9023_platform_data *pdata;
       struct power_supply wireless_psy;
       bool wireless_charging;
};

struct IDTP9023_command {
    char *name;;
    u8 addr;
    u8 len;
    enum readwrite {
        E_READ=0,
        E_WRITE=1,
        E_READWRITE=2,
        E_NOUSE=3,
    }rw;
};

enum IDTP9023_CMD_ID {
    IDTP9023_CNTL,
};

struct IDTP9023_command IDTP9023_CMD_Table[] = {
    {"EOP", 0x00, 2, E_READWRITE}, //EOP
};

#if 0
static int IDTP9023_i2c_read(__u8 *buf_addr, int len, void *data)
{
        int i=0;
        int retries=6;
        int status=0;



	struct i2c_msg msg[] = {
		{
			.addr = g_IDTP9023_slave_addr,
			.flags = 0,
			.len = 2,  // register address len+1:u8 , len+2:u16 
			.buf = buf_addr,
		},
		{
			.addr = g_IDTP9023_slave_addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = data,
		},
	};

    pr_debug("[BAT][WC][I2c]IDTP9023_i2c_read+++\n");
    
        if(g_IDTP9023_info){
            do{    
                pr_debug("[BAT][WC][I2c]IDTP9023_i2c_transfer\n");
        		status = i2c_transfer(g_IDTP9023_info->i2c_client->adapter,
        			msg, ARRAY_SIZE(msg));
                
        		if ((status < 0) && (i < retries)){
        			    msleep(5);
                      
                                printk("[BAT][WC][I2c]%s retry %d\r\n", __FUNCTION__, i);                                
                                i++;
                     }
        	    } while ((status < 0) && (i < retries));
        }
        if(status < 0)
	{
            printk("[BAT][WC][I2c]IDTP9023: i2c read error %d \n", status);

       }
    pr_debug("[BAT][WC][I2c]IDTP9023_i2c_read---\n");
    

        return status;
        
}


static int IDTP9023_read_reg(int cmd, void *data)
{
    int status=0;
		__u8 buf_addr[3];
		buf_addr[0] = 0x88;
		buf_addr[1] = 0x00;

		//Eason: read RxID +++
		if( (4 <= g_IDTP9023_reg_address) && (9 >= g_IDTP9023_reg_address) ) 
		{
			buf_addr[0] = 0x88;
			buf_addr[1] = 0x04;
		}
		//Eason: read RxID ---
		
		 status=IDTP9023_i2c_read(buf_addr, 1, data);
            
    return status;
}

static void IDTP9023_proc_read(void)
{    
       int status;
       int16_t reg_value=0;//for 2 byte I2c date should use int16_t instead int, let negative value can show correctly. ex:current
       printk("[BAT][WC][Proc]%s \r\n", __FUNCTION__);
       status=IDTP9023_read_reg(g_IDTP9023_reg_address,&reg_value);
       g_IDTP9023_reg_value = reg_value;

       if(status > 0 && reg_value >= 0){
            printk("[BAT][WC][Proc] found! IDTP9023=%d\r\n",reg_value);
       }
}
//ASUS BSP Eason_Chang IDTP9023 +++
#endif
static int IDTP9023_i2c_write(u16 addr, int len, void *data)
{
    int i=0;
	int status=0;
	u8 buf[len + 2]; // register address len+1:u8 , len+2:u16 

	static struct i2c_msg msg;
	int retries = 6;

	msg.addr = g_IDTP9023_slave_addr;
	msg.flags = 0; //write
	msg.len = len + 2, // register address len+1:u8 , len+2:u16 
	msg.buf = buf;


if(3== g_IDTP9023_reg_address) 
{
	memset(buf, 0, len+2);
	buf[0] = 0x88;
	buf[1] = 0x00;
	memcpy(buf + 2, data, len);//register address buf+1:u8 , buf+2:u16 
}
//Eason: read RxID +++
if( (4 <= g_IDTP9023_reg_address) && (9 >= g_IDTP9023_reg_address) ) 
{
	memset(buf, 0, len+2);
	buf[0] = 0x88;
	buf[1] = 0x01;
	memcpy(buf + 2, data, len);//register address buf+1:u8 , buf+2:u16 
}
//Eason: read RxID ---

	do {
		status = i2c_transfer(g_IDTP9023_info->i2c_client->adapter,
			&msg, 1);
        	if ((status < 0) && (i < retries)){
                    msleep(5);
           
                    printk("[BAT][WC][I2c]%s retry %d\r\n", __FUNCTION__, i);
                    i++;
              }
       } while ((status < 0) && (i < retries));

	if (status < 0) 
	{
             printk("[BAT][WC][I2c]IDTP9023: i2c write error %d \n", status);
	}

	return status;
}

static int IDTP9023_write_reg(int cmd, void *data)
{
    int status=0;

	uint16_t addr = 0x8800;

	if(3== g_IDTP9023_reg_address)
		IDTP9023_i2c_write(addr,4,data);		
	//Eason: read RxID +++
	else if( (4 <= g_IDTP9023_reg_address) && (9 >= g_IDTP9023_reg_address) ) 
	{
		IDTP9023_i2c_write(addr,3,data);
	}
	//Eason: read RxID ---

    return status;
}

static void IDTP9023_proc_write(void)
{    
       int status;
       uint8_t i2cdata[32]={0};

       i2cdata[0] = g_IDTP9023_reg_value;
       printk("[BAT][WC][Proc]%s:%d \r\n", __FUNCTION__,g_IDTP9023_reg_value);

	 if(3 == g_IDTP9023_reg_address)
	{
		i2cdata[0]=0xAA;
		i2cdata[1]=0x00;
		i2cdata[2]=0xEF;
		i2cdata[3]=0x01;
	}
	//Eason: read RxID +++
	else if( (4 <= g_IDTP9023_reg_address) && (9 >= g_IDTP9023_reg_address) ) 
	{
		i2cdata[0]=0xA8;
		i2cdata[1]=(0x50 + g_IDTP9023_reg_address);
		i2cdata[2]=0x02;
	}
	//Eason: read RxID ---
	
       status=IDTP9023_write_reg(g_IDTP9023_reg_address,i2cdata);

       if(status > 0 ){
            printk("[BAT][WC][Proc] proc write\r\n");
       }
}
#if 0
static ssize_t IDTP9023_read_proc(char *page, char **start, off_t off, int count, 
            	int *eof, void *data)
{
	IDTP9023_proc_read();
	return sprintf(page, "0x%02X\n", g_IDTP9023_reg_value);
}
static ssize_t IDTP9023_write_proc(struct file *filp, const char __user *buff, 
	            unsigned long len, void *data)
{
	int val;

	char messages[256];

	if (len > 256) {
		len = 256;
	}

	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}
	
	val = (int)simple_strtol(messages, NULL, 10);


	g_IDTP9023_reg_value = val;

      IDTP9023_proc_write();
    
    printk("[BAT][WC][Proc]mode:%d\n",g_IDTP9023_reg_value);
	
	return len;
}

void static create_IDTP9023_proc_file(void)
{
	struct proc_dir_entry *IDTP9023_proc_file = create_proc_entry("driver/IDTP9023", 0644, NULL);

	if (IDTP9023_proc_file) {
		IDTP9023_proc_file->read_proc = IDTP9023_read_proc;
		IDTP9023_proc_file->write_proc = IDTP9023_write_proc;
	}
    else {
		printk("[BAT][WC][Proc]proc file create failed!\n");
    }

	return;
}

static ssize_t IDTP9023Address_read_proc(char *page, char **start, off_t off, int count, 
            	int *eof, void *data)
{
	return sprintf(page, "%d\n", g_IDTP9023_reg_address);
}
static ssize_t IDTP9023Address_write_proc(struct file *filp, const char __user *buff, 
	            unsigned long len, void *data)
{
	int val;

	char messages[256];

	if (len > 256) {
		len = 256;
	}

	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}
	
	val = (int)simple_strtol(messages, NULL, 10);


	g_IDTP9023_reg_address = val;  
    
    printk("[BAT][WC][Proc]IDTP9023Address:%d\n",val);
	
	return len;
}

void static create_IDTP9023Address_proc_file(void)
{
	struct proc_dir_entry *IDTP9023Address_proc_file = create_proc_entry("driver/IDTP9023Addr", 0644, NULL);

	if (IDTP9023Address_proc_file) {
		IDTP9023Address_proc_file->read_proc = IDTP9023Address_read_proc;
		IDTP9023Address_proc_file->write_proc = IDTP9023Address_write_proc;
	}
    else {
		printk("[BAT][WC][Proc]Addr proc file create failed!\n");
    }

	return;
}

//Eason: read RxID +++
static void Read_IDTP9023RxID(void)
{
	int i;
	
	for(i=0;i<=5;i++)
	{
		g_IDTP9023_reg_address = i + 4;
		IDTP9023_proc_write();
		g_IDTP9023_reg_address = 0;
		IDTP9023_proc_read();
		IDTP9023_value_since_reg0xA854_shift[i] = g_IDTP9023_reg_value;
	}	
}

static ssize_t IDTP9023RxID_read_proc(char *page, char **start, off_t off, int count, 
            	int *eof, void *data)
{
	Read_IDTP9023RxID();
	return 	sprintf(page, "%02X%02X%02X%02X%02X%02X\n", IDTP9023_value_since_reg0xA854_shift[0]
							,IDTP9023_value_since_reg0xA854_shift[1]
							,IDTP9023_value_since_reg0xA854_shift[2]
							,IDTP9023_value_since_reg0xA854_shift[3]
							,IDTP9023_value_since_reg0xA854_shift[4]
							,IDTP9023_value_since_reg0xA854_shift[5]);
}
static ssize_t IDTP9023RxID_write_proc(struct file *filp, const char __user *buff, 
	            unsigned long len, void *data)
{
	int val;

	char messages[256];

	if (len > 256) {
		len = 256;
	}

	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}
	
	val = (int)simple_strtol(messages, NULL, 10);

    
    printk("[BAT][WC][Proc]IDTP9023RxID:%d\n",val);
	
	return len;
}

void static create_IDTP9023RxID_proc_file(void)
{
	struct proc_dir_entry *IDTP9023RxID_proc_file = create_proc_entry("driver/IDTP9023_RxID", 0644, NULL);

	if (IDTP9023RxID_proc_file) {
		IDTP9023RxID_proc_file->read_proc = IDTP9023RxID_read_proc;
		IDTP9023RxID_proc_file->write_proc = IDTP9023RxID_write_proc;
	}
    else {
		printk("[BAT][WC][Proc][RxID]Addr proc file create failed!\n");
    }

	return;
}
//Eason: read RxID ---
#endif
void IDTP9023_RxTurnOffTx(void)
{
	g_IDTP9023_reg_address = 3;
	IDTP9023_proc_write();
	printk("[BAT][WC]%s\n",__FUNCTION__);
}

#define GPIO_WC_PD_DET 20
struct workqueue_struct *wc_intr_wq = NULL;
struct delayed_work wc_worker;

static irqreturn_t wc_pd_det_handler(int irq, void *dev_id)
{
	queue_delayed_work(wc_intr_wq, &wc_worker, 0);        
	  printk("[BAT][WC]%s\n",__FUNCTION__);
        return IRQ_HANDLED;
}

static IDTP9023_PIN wc_gGpio_pin[]=
{
    {//WC_RESET
        .gpio = 85,
        .name = "WC_RESET",
        .in_out_flag = 1,
        .init_value = 0,      
    },

    {//WC_PD_DET
        .gpio = 20,
        .name = "WC_PD_DET",
        .in_out_flag = 0,
        .handler = wc_pd_det_handler,
        .trigger_flag= IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING,      
    },
};

bool Is_WC_connect(void)
{
	int connect;

	//Eason_Chang: PF500KL WC_PD_DET reverse. default high, with wireless low+++
	if (g_ASUS_hwID >= PF500KL_PR){	
		connect = !gpio_get_value(GPIO_WC_PD_DET);
	} else {
		connect = gpio_get_value(GPIO_WC_PD_DET);
	}
	//Eason_Chang: PF500KL WC_PD_DET reverse. default high, with wireless low---
	
	if (connect)
		return true;
	else
		return false;
}

extern void setWirelessCharger(bool enable);

static void wc_work(struct work_struct *data)
{
	//Eason_Chang: PF500KL WC_PD_DET reverse. default high, with wireless low+++

	if(g_ASUS_hwID >= PF500KL_PR) {
		if(!gpio_get_value(GPIO_WC_PD_DET))
		{
			printk("Enable Wireless charger\n");
			setWirelessCharger(true);
			g_IDTP9023_info->wireless_charging = true;
			g_IDTP9023_info->wireless_psy.type = POWER_SUPPLY_TYPE_WIRELESS;
		}	
		else
		{
			g_IDTP9023_reg_address = 0;//set default
			setWirelessCharger(false);
			g_IDTP9023_info->wireless_charging = false;
			//g_IDTP9023_info->wireless_psy.type = POWER_SUPPLY_TYPE_BATTERY;
			printk("Disable Wireless charger\n");
		}	
	} else {
	//Eason_Chang: PF500KL WC_PD_DET reverse. default high, with wireless low---	
		if(gpio_get_value(GPIO_WC_PD_DET))
		{
			printk("Enable Wireless charger\n");
			setWirelessCharger(true);
			g_IDTP9023_info->wireless_charging = true;
			//g_IDTP9023_info->wireless_psy.type = POWER_SUPPLY_TYPE_WIRELESS;
		}	
		else
		{
			//Eason: read RxID +++
			printk("Disable Wireless charger\n");
			g_IDTP9023_reg_address = 0;//set default
			g_IDTP9023_info->wireless_charging = false;
			//g_IDTP9023_info->wireless_psy.type = POWER_SUPPLY_TYPE_BATTERY;
		}	
	}
	//power_supply_changed(&g_IDTP9023_info->wireless_psy);
}

static int AXC_IDTP9023_GPIO_setting(void)
{
	IDTP9023_PIN_DEF i;
	int err = 0;
	printk("%s+++\n",__FUNCTION__);

	for(i = 0; i< IDTP9023_PIN_COUNT;i++){

		//rquest
		err = gpio_request(wc_gGpio_pin[i].gpio, wc_gGpio_pin[i].name);
		if (err < 0) {
			printk("[BAT][WC]gpio_request %s failed,err = %d\n", wc_gGpio_pin[i].name, err);
			goto err_exit;
		}
		
		//input
       		if(wc_gGpio_pin[i].in_out_flag == 0){
	            	err = gpio_direction_input(wc_gGpio_pin[i].gpio);
   
        	    	if (err  < 0) {
        	        	printk( "[BAT][WC]gpio_direction_input %s failed, err = %d\n", wc_gGpio_pin[i].name, err);
        	        	goto err_exit;
        	    	}

			if(wc_gGpio_pin[i].handler != NULL){

                		wc_gGpio_pin[i].irq = gpio_to_irq(wc_gGpio_pin[i].gpio);

				printk("[BAT][WC]:GPIO:%d,IRQ:%d\n",wc_gGpio_pin[i].gpio,  wc_gGpio_pin[i].irq);
                
				if(true == wc_gGpio_pin[i].irq_enabled){
                    
                    			enable_irq_wake(wc_gGpio_pin[i].irq);

              			}

                		err = request_irq(wc_gGpio_pin[i].irq , 
                    				  wc_gGpio_pin[i].handler, 
                    				  wc_gGpio_pin[i].trigger_flag,
					          wc_gGpio_pin[i].name, 
                    				  NULL);

                		if (err  < 0) {
                    			printk( "[BAT][CHG][SMB]request_irq %s failed, err = %d\n",wc_gGpio_pin[i].name,err);
                    		goto err_exit;
                		}

            		}
		//output
		}else{
			 gpio_direction_output(wc_gGpio_pin[i].gpio, wc_gGpio_pin[i].init_value);            
        	}
	}
	printk("%s---\n",__FUNCTION__);
	return 0;

err_exit:
	    for(i = 0; i<IDTP9023_PIN_COUNT;i++){        
        		gpio_free(wc_gGpio_pin[i].gpio);
    	    }
	printk("[BAT][WC]IDTP9023_Charger_InitGPIO_err_exit\n");  
	return err;  
}
/*
static int pm_power_get_property_wireless(struct power_supply *psy,
					  enum power_supply_property psp,
					  union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = g_IDTP9023_info->wireless_charging;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
*/

static int IDT_P9023_i2c_probe(struct i2c_client *client, const struct i2c_device_id *devid)
{

	struct IDTP9023_info *info;
	int rc = 0;

	wc_intr_wq = create_singlethread_workqueue("wirelessChg_intr_wq");
	INIT_DELAYED_WORK(&wc_worker,wc_work);

	//Eason when device boot on wireless Tx, set wireless charging status +++
	queue_delayed_work(wc_intr_wq, &wc_worker, 10 * HZ); 
	//Eason when device boot on wireless Tx, set wireless charging status ---
	printk("%s+++\n",__FUNCTION__);

	g_IDTP9023_info = info = kzalloc(sizeof(struct IDTP9023_info), GFP_KERNEL);
	g_IDTP9023_slave_addr = client->addr;
	info->i2c_client = client;
	i2c_set_clientdata(client, info);

	//create_IDTP9023_proc_file();
	//create_IDTP9023Address_proc_file();
	//Eason: read RxID +++
	//create_IDTP9023RxID_proc_file();
	//Eason: read RxID ---

	if (0 != AXC_IDTP9023_GPIO_setting()) 
	{
		printk( "[BAT][WC]Charger gpio can't init\n");
	}
/*
	g_IDTP9023_info->wireless_psy.name = "wireless";
	g_IDTP9023_info->wireless_psy.type = POWER_SUPPLY_TYPE_WIRELESS;
	g_IDTP9023_info->wireless_psy.supplied_to = pm_power_supplied_to;
	g_IDTP9023_info->wireless_psy.num_supplicants = ARRAY_SIZE(pm_power_supplied_to);
	g_IDTP9023_info->wireless_psy.num_supplicants = ARRAY_SIZE(pm_power_supplied_to);
	g_IDTP9023_info->wireless_psy.properties = pm_power_props_wireless;
	g_IDTP9023_info->wireless_psy.num_properties = ARRAY_SIZE(pm_power_props_wireless);
	g_IDTP9023_info->wireless_psy.get_property = pm_power_get_property_wireless;
	rc = power_supply_register(&client->dev, &g_IDTP9023_info->wireless_psy);*/
	if (rc < 0) {
		pr_err("wlc: power_supply_register wireless failed rx = %d\n",
			      rc);
		//goto free_chip;
	}

	printk("%s---\n",__FUNCTION__);
	return 0;
}

static int IDT_P9023_i2c_remove(struct i2c_client *client)
{
	return 0;
}

const struct i2c_device_id IDTP9023_i2c_id[] = {
    {"IDT_P9023_i2c", 0},
    {}
};

MODULE_DEVICE_TABLE(i2c, IDTP9023_i2c_id);


static struct of_device_id IDTP9023_match_table[] = {
	{ .compatible = "IDT,P9023",},
	{ },
};

static struct i2c_driver IDTP9023_i2c_driver = {
    .driver = {
        .name = "IDT_P9023_i2c",
        .owner  = THIS_MODULE,
        .of_match_table = IDTP9023_match_table,
     },
    .probe = IDT_P9023_i2c_probe,
    .remove = IDT_P9023_i2c_remove,
    .id_table = IDTP9023_i2c_id,
};    

static int __init IDTP9023_init(void)
{
    printk("[BAT][IDTP9023]init\n");
	//Eason_Chang:A90FF dont do wireless charger+++
	if(g_ASUS_hwID == A90_EVB0)
	{
		printk("[BAT]%s FAIL\n",__FUNCTION__);
		return -EINVAL; 
	}
	//Eason_Chang:A90FF dont do wireless charger---	
    return i2c_add_driver(&IDTP9023_i2c_driver);
}

static void __exit IDTP9023_exit(void)
{
    printk("[BAT][IDTP9023]exit\n");
    i2c_del_driver(&IDTP9023_i2c_driver);

}

module_init(IDTP9023_init);
module_exit(IDTP9023_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ASUS Wireless charger IDTP9023 driver");
MODULE_VERSION("1.0");
MODULE_AUTHOR("Eason Chang <eason1_chang@asus.com>");

/*
+ * Copyright (C) 2010 Trusted Logic S.A.
  *
  * This program is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation; either version 2 of the License, or
  * (at your option) any later version.
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program; if not, write to the Free Software
  * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
  *
  */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/nfc/pn544.h>


#include <linux/of_gpio.h>

//struct pn544_i2c_platform_data *pdata;

//+++ASUS_BSP:Used for config PMIC's GPIO+++
/*#include <linux/mfd/pm8xxx/pm8921.h>
#define PM8921_GPIO_BASE                NR_GPIO_IRQS
#define PM8921_GPIO_PM_TO_SYS(pm_gpio)  (pm_gpio - 1 + PM8921_GPIO_BASE)
*/
//static int gpio43;
//---ASUS_BSP:Used for config PMIC's GPIO---

#define MAX_BUFFER_SIZE	512

struct pn544_dev	{
	wait_queue_head_t	read_wq;
	struct mutex		read_mutex;
	struct i2c_client	*client;
	struct miscdevice	pn544_device;
	unsigned int 		ven_gpio;
	unsigned int 		firm_gpio;
	unsigned int		irq_gpio;
	bool			irq_enabled;
	spinlock_t		irq_enabled_lock;
 };

struct pn544_dev *pn544_dev=NULL;

//#include <linux/wait.h>
#include <linux/wakelock.h>
static struct wake_lock nfc_wake_lock;
//static u32 g_slave_addr=0;

static void pn544_disable_irq(struct pn544_dev *pn544_dev)
 {
	unsigned long flags;

	spin_lock_irqsave(&pn544_dev->irq_enabled_lock, flags);
	if (pn544_dev->irq_enabled) {
		disable_irq_nosync(pn544_dev->client->irq);
		pn544_dev->irq_enabled = false;
 	}

	spin_unlock_irqrestore(&pn544_dev->irq_enabled_lock, flags);
}


static irqreturn_t pn544_dev_irq_handler(int irq, void *dev_id)
{
	struct pn544_dev *pn544_dev = dev_id;

	pn544_disable_irq(pn544_dev);

	/* Wake up waiting readers */
	wake_up(&pn544_dev->read_wq);
 
	return IRQ_HANDLED;
}
 

static ssize_t pn544_dev_read(struct file *filp, char __user *buf, size_t count, loff_t *offset)
 {
	struct pn544_dev *pn544_dev = filp->private_data;
	char tmp[MAX_BUFFER_SIZE];
	int ret;
 
	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	mutex_lock(&pn544_dev->read_mutex);

	if (!gpio_get_value(pn544_dev->irq_gpio)) {
		if (filp->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			goto fail;
		}
		pn544_dev->irq_enabled = true;
		enable_irq(pn544_dev->client->irq);
		ret = wait_event_interruptible(pn544_dev->read_wq, gpio_get_value(pn544_dev->irq_gpio));
		pn544_disable_irq(pn544_dev);
		if (ret)
			goto fail;
 	}

	/* Read data */
	ret = i2c_master_recv(pn544_dev->client, tmp, count);

	mutex_unlock(&pn544_dev->read_mutex);

	/* pn544 seems to be slow in handling I2C read requests
	 * so add 1ms delay after recv operation */
	udelay(1000);

	if (ret < 0) {
		pr_err("%s: i2c_master_recv returned %d\n", __func__, ret);
		return ret;
 	}

	if (ret > count) {
		pr_err("%s: received too many bytes from i2c (%d)\n",
			__func__, ret);
		return -EIO;
 	}

	if (copy_to_user(buf, tmp, ret)) {
		pr_warning("%s : failed to copy to user space\n", __func__);
		return -EFAULT;
 	}
	return ret;

fail:
	mutex_unlock(&pn544_dev->read_mutex);
	return ret;

}
 

static ssize_t pn544_dev_write(struct file *filp, const char __user *buf, size_t count, loff_t *offset)
{
	struct pn544_dev  *pn544_dev;
	char tmp[MAX_BUFFER_SIZE];
	int ret;
 
	pn544_dev = filp->private_data;
 
	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	if (copy_from_user(tmp, buf, count)) {
		pr_err("%s : failed to copy from user space\n", __func__);
		return -EFAULT;
 	}

	pr_debug("%s : writing %zu bytes.\n", __func__, count);

	/* Write data */
	ret = i2c_master_send(pn544_dev->client, tmp, count);

	if (ret != count) {
		pr_err("%s : i2c_master_send returned %d\n", __func__, ret);
		ret = -EIO;
 	}

	/* pn544 seems to be slow in handling I2C write requests
	 * so add 1ms delay after I2C send oparation */
	udelay(1000);

	return ret;
}


static int pn544_dev_open(struct inode *inode, struct file *filp)
 {
	struct pn544_dev *pn544_dev = container_of(filp->private_data,
						struct pn544_dev,
						pn544_device);

	filp->private_data = pn544_dev;

	pr_debug("%s : %d,%d\n", __func__, imajor(inode), iminor(inode));
 
	return 0;
 }
 

static long  pn544_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct pn544_dev *pn544_dev = filp->private_data;
 	switch (cmd) {
	case PN544_SET_PWR:
		if (arg == 2) {
			/* power on with firmware download (requires hw reset)
			 */
			pr_info("[Nfc]power on with firmware\n");
			gpio_set_value(pn544_dev->ven_gpio, 1);
			msleep(20);
			//if (pn544_dev->firm_gpio) gpio_set_value(pn544_dev->firm_gpio, 1);
			gpio_set_value(pn544_dev->firm_gpio, 1);
			msleep(20);
			gpio_set_value(pn544_dev->ven_gpio, 0);
			msleep(100);
			gpio_set_value(pn544_dev->ven_gpio, 1);
			msleep(20);
		} else if (arg == 1) {
			/* power on */
			pr_info("[Nfc]power on\n");
			//if (pn544_dev->firm_gpio) gpio_set_value(pn544_dev->firm_gpio, 0);
			gpio_set_value(pn544_dev->firm_gpio, 0);
			gpio_set_value(pn544_dev->ven_gpio, 1);
			msleep(100);
		} else  if (arg == 0) {
			/* power off */
			pr_info("[Nfc]power off\n");
			//if (pn544_dev->firm_gpio) gpio_set_value(pn544_dev->firm_gpio, 0);
			gpio_set_value(pn544_dev->firm_gpio, 0);		
			gpio_set_value(pn544_dev->ven_gpio, 0);
			msleep(100);
 		} else {
			pr_err("%s bad arg %lu\n", __func__, arg);
			return -EINVAL;
 		}
 		break;
 	default:
		pr_err("%s bad ioctl %u\n", __func__, cmd);
		return -EINVAL;
 	}

 	return 0;
 }


static const struct file_operations pn544_dev_fops = {
	.owner	= THIS_MODULE,
	.llseek	= no_llseek,
	.read	= pn544_dev_read,
	.write	= pn544_dev_write,
	.open	= pn544_dev_open,
	.unlocked_ioctl  = pn544_dev_ioctl,
 };


#ifdef CONFIG_I2C_STRESS_TEST
#include <linux/i2c_testcase.h>
#include <linux/proc_fs.h>
#include <linux/file.h>
#include <linux/syscalls.h>

//#define Nfc_debug_PROC_FILE  "driver/nfc_debug"
//static struct proc_dir_entry *nfc_debug_proc_file;

#define I2C_TEST_NFC_FAIL (-1)

static int pn544_i2c_read(int count)
{
	char tmp[MAX_BUFFER_SIZE];
	int ret, i;

	if (!gpio_get_value(pn544_dev->irq_gpio)) {
		enable_irq(pn544_dev->client->irq);
		ret = wait_event_interruptible(pn544_dev->read_wq, gpio_get_value(pn544_dev->irq_gpio));
		pn544_disable_irq(pn544_dev);
		if (ret){
			printk("[Nfc]wait_event_interruptible() fails");
			goto fail;
		}
 	}

	/* Read data */
	for(i=0; i<2; i++) {
		ret = i2c_master_recv(pn544_dev->client, tmp, count);

		/* pn544 seems to be slow in handling I2C read requests
		 * so add 1ms delay after recv operation */
		udelay(1000);
	
		if (ret < 0) {
			printk("%s: i2c_master_recv returned %d\n", __func__, ret);
 		}
		else if (ret > count) {
			printk("%s: received too many bytes from i2c (%d)\n", __func__, ret);
 		}
		else {
			return I2C_TEST_PASS;
		}

		printk("[NFC] i2c_master_recv() retry....%d\n", i);
	}

	//printk("%s : reading %zu bytes.\n", __func__, count);

	#if 0
	printk("[Nfc]Received Data:\n");
	for (i = 1; i < count; i++) {
		printk("0x%02x, ", tmp[i] );
	}
	printk("\n");
	#endif
	/*if (copy_to_user(buf, tmp, ret)) {
		pr_warning("%s : failed to copy to user space\n", __func__);
		return -EFAULT;
 	}*/

fail:
	return I2C_TEST_NFC_FAIL;

}

static int pn544_i2c_write(char *tmp, int count)
{
	int ret, i;

	i2c_log_in_test_case("%s:writing %zu bytes.\n", __func__, count);

	/* Write data */
	for(i=0; i<2; i++) {		
		ret = i2c_master_send(pn544_dev->client, tmp, count);

		/* pn544 seems to be slow in handling I2C write requests
		 * so add 1ms delay after I2C send oparation */
		udelay(1000);

		if (ret != count) {
			pr_err("%s : i2c_master_send returned %d\n", __func__, ret);
 		}
		else {
			return I2C_TEST_PASS;
		}

		printk("[NFC] i2c_master_send() retry....%d\n", i);

	}

	return I2C_TEST_NFC_FAIL;
}

#define CMD_COL 9

static int DoI2Coperation(char *cmd, int CmdNum, int cmdLength[], int cmdResLength[], int cmdWithTwoResponse)
{
    	int cmdResLengthIndex = 0;    

    	int i;
    	int ret = I2C_TEST_PASS;

   	for(i = 0; i < CmdNum; i++){
       	    i2c_log_in_test_case("[Nfc] CMD=%d\n", i);
	    ret = pn544_i2c_write(cmd, cmdLength[i]);
	    if(ret == I2C_TEST_NFC_FAIL)
	       return ret;	
	    cmd=cmd+CMD_COL;
	    msleep(200); 	
	    if(cmdResLength[cmdResLengthIndex]!=0){  //If a cmd is an Auto ACK, the chip will not response...
    	        ret= pn544_i2c_read(cmdResLength[cmdResLengthIndex]);
	        if(ret == I2C_TEST_NFC_FAIL)
	           return ret;	        
		cmdResLengthIndex++;
		if(i == cmdWithTwoResponse){ //If a cmd has 2 responses, do i2c_read again...here,only cmd3 has 2 responses...
	    	        ret = pn544_i2c_read(cmdResLength[cmdResLengthIndex]);
         	        if(ret == I2C_TEST_NFC_FAIL)
	                   return ret;
			cmdResLengthIndex++;
		}
		/*else {
		    printk("cmd %d has no 2 responses.\n", i);
		}*/
	    }
	    else {
		cmdResLengthIndex++;
		//printk("cmdResLengthIndex=%d\n", cmdResLengthIndex);
	    }	
	}

	return ret;
}

/*
* xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
* INIT PN544
* xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx

// no retry allowed
**********************************************
* Send U-RSET frame
(0)PC -> IFD : 
	05 F9 04 00 C3 E5 

IFD -> PC : 
	03 E6 17 A7 


**********************************************
* Open Pipe Admin
(1)PC -> IFD : 
	05 80 81 03 EA 39 
  IFD -> PC : 
	05 81 81 80 A5 D5 
       //Unpacked data = [81 80 ]
(2)PC -> IFD : AUTO ACK
	03 C1 AA F2 


**********************************************
* Admin ClearAllPipe

(3)PC -> IFD : 
	05 89 81 14 CA C1 


IFD -> PC : 
	03 C2 31 C0 

IFD -> PC : 
	05 8A 81 80 03 FC 
//	Unpacked data = [81 80 ]

(4)PC -> IFD : AUTO ACK
	03 C2 31 C0 


**********************************************
* ReOpen Pipe Admin

(5)PC -> IFD : 
	05 92 81 03 C7 09 


IFD -> PC : 
	05 93 81 80 88 E5 
//	Unpacked data = [81 80 ]

(6)
PC -> IFD : AUTO ACK
	03 C3 B8 D1 


* xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
* Show PN544 Functionality
* xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx


**********************************************
* Host creates pipe on IdMgt gate

(7)PC -> IFD : 
	08 9B 81 10 05 00 05 0B 14 
   IFD -> PC : 
	0A 9C 81 80 01 05 00 05 02 FC DA 
        //	Unpacked data = [81 80 01 05 00 05 02 ]

(8)PC -> IFD : AUTO ACK
	03 C4 07 A5 


**********************************************
* Host opens pipe on IdMgt gate:
(9)PC -> IFD : 
	05 A4 82 03 D8 73 

   IFD -> PC : 
	05 A5 82 80 97 9F 
//	Unpacked data = [82 80 ]

(10)PC -> IFD : AUTO ACK
	03 C5 8E B4 


**********************************************
* Host gets Software Version:
* Expected Result: Unpacked data = [82 80 xx xx xx ]
(11)PC -> IFD : 
	06 AD 82 02 01 30 46 

    IFD -> PC : 
	08 AE 82 80 A7 6C 0C 04 DF 
        //	Unpacked data = [82 80 A7 6C 0C ]
(12)PC -> IFD : AUTO ACK
	03 C6 15 86 


**********************************************
* Host gets Hardware Version:
* Expected Result: Unpacked data = [82 80 xx xx xx ]
// PN544 C2 HW expected
(13)PC -> IFD : 
	06 B6 82 02 03 96 66 
    IFD -> PC : 
	08 B7 82 80 62 00 03 E2 BF 
    //Unpacked data = [82 80 62 00 03 ]

(14)PC -> IFD : AUTO ACK
    03 C7 9C 97 
*/

static int i2c_stress_test_turn_on(void)
{

    char cmd[][CMD_COL]={ {0x05, 0xF9, 0x04, 0x00, 0xC3, 0xE5},//cmd 0
	    	    	  {0x05, 0x80, 0x81, 0x03, 0xEA, 0x39},//cmd 1
		          {0x03, 0xC1, 0xAA, 0xF2},            //cmd 2
		          {0x05, 0x89, 0x81, 0x14, 0xCA, 0xC1},//cmd 3
		          {0x03 ,0xC2, 0x31, 0xC0},             //cmd 4
		          {0x05, 0x92, 0x81, 0x03, 0xC7, 0x09},//cmd5
		          {0x03, 0xC3, 0xB8, 0xD1 },            //cmd6
			  {0x08, 0x9B, 0x81, 0x10, 0x05, 0x00, 0x05, 0x0B, 0x14},//cmd7
			  {0x03, 0xC4, 0x07, 0xA5},//cmd8
 			  {0x05, 0xA4, 0x82, 0x03, 0xD8, 0x73},//cmd9
			  {0x03, 0xC5, 0x8E, 0xB4},//cmd10
                          {0x06, 0xAD, 0x82, 0x02, 0x01, 0x30, 0x46},//cmd11
                          {0x03, 0xC6, 0x15, 0x86},//cmd12
                          {0x06, 0xB6, 0x82, 0x02, 0x03, 0x96, 0x66},//cmd13
                          {0x03, 0xC7, 0x9C, 0x97}//cmd14
	                };

    int cmdLength[]={6,6,4,6,4,6,4,9,4,6,4,7,4,7,4};
                        //cmd0   //cmd1  //cmde2 
    int cmdResLength[]={     4,   6,      0,         
	    	        //cmd3   //cmd4  //cmd5	 
		          4, 6,   0,      6,
		        //cmd6   //cmd7  //cmd8
		             0,   11,     0,
                        //cmd9   //cmd10 //cmd11
                             6,    0,      9,
                        //cmd12  //cmd13  //cmd14
                             0,    9,      0
                       };
    int CmdNum= (sizeof(cmd)/sizeof(cmd[0]));
    int cmdWithTwoResponse=3;

    i2c_log_in_test_case("power on Nfc\n");
    gpio_set_value(pn544_dev->firm_gpio, 0);
    gpio_set_value(pn544_dev->ven_gpio, 1);
    msleep(100);
    return DoI2Coperation(&cmd[0][0], CmdNum, cmdLength, cmdResLength, cmdWithTwoResponse);
}


/*
* xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
* DE-INIT PN544
* xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx


**********************************************
* Host closes pipe on IdMgt gate:
(0)PC -> IFD : 
	05 BF 82 04 54 AB 
   IFD -> PC : 
	05 B8 82 80 7D E5 
        //	Unpacked data = [82 80 ]
(1)PC -> IFD : AUTO ACK
	03 C0 23 E3 


**********************************************
* Host deletes pipe on IdMgt gate:
(2)PC -> IFD : 
	06 80 81 11 02 EA 20 


IFD -> PC : 
	05 81 81 80 A5 D5 
//	Unpacked data = [81 80 ]

(3)PC -> IFD : AUTO ACK
	03 C1 AA F2 
**********************************************
*/
static int i2c_stress_test_turn_off(void)
{
   char cmd[][CMD_COL]={ {0x05, 0xBF, 0x82, 0x04, 0x54, 0xAB},//cmd0
		         {0x03, 0xC0, 0x23, 0xE3 }, //cmd1
		         {0x06, 0x80, 0x81, 0x11, 0x02, 0xEA, 0x20 },//cmd2
		         {0x03, 0xC1, 0xAA, 0xF2 }//cmd3
	               };

    int cmdLength[]={6,4,7,4};
                         //cmd0  //cmd1  //cmde2  //cmd3
    int cmdResLength[]={    6,      0,      6,       0};
    int CmdNum= (sizeof(cmd)/sizeof(cmd[0]));
    int cmdWithTwoResponse=-1; //De-init process does not have 2-response cmd..

    int ret = DoI2Coperation(&cmd[0][0], CmdNum, cmdLength, cmdResLength, cmdWithTwoResponse);

    msleep(300);
    i2c_log_in_test_case("power off Nfc\n");
    gpio_set_value(pn544_dev->firm_gpio, 0);		
    gpio_set_value(pn544_dev->ven_gpio, 0);

    return ret;
}

static int StartI2CTest(void)
{
    int ret; 
    ret = i2c_stress_test_turn_on();
    if(ret == I2C_TEST_NFC_FAIL){
	printk("[Nfc]i2c_stress_test_turn_on FAIL\n");
        return ret;
    }
    else
	i2c_log_in_test_case("[Nfc]i2c_stress_test_turn_on PASS\n");

    ret = i2c_stress_test_turn_off();
    if(ret == I2C_TEST_NFC_FAIL){
	printk("[Nfc]i2c_stress_test_turn_off FAIL\n");
        return ret;
    }
    else{
	i2c_log_in_test_case("[Nfc]i2c_stress_test_turn_off PASS\n");
    }

    return ret;
}

#if 0
static ssize_t nfc_debug_proc_write(struct file *filp, const char *buff, size_t len, loff_t *off)
{
    char messages[256];
    int ret;
	
    memset(messages, 0, sizeof(messages));

    printk("[Nfc]Start I2C stress test\n");

    if (copy_from_user(messages, buff, len))
    {
        return -EFAULT;
    }

    if(strncmp(messages, "1", 1) == 0)
    {
	printk("start the Nfc i2c test...\n");
	ret = StartI2CTest();
    }    
    return len;
}

static struct file_operations nfc_debug_proc_ops = {
    .write = nfc_debug_proc_write,
};

static void create_nfc_debug_proc_file(void)
{
    printk("[Nfc] create_nfc_debug_proc_file\n");
    nfc_debug_proc_file = create_proc_entry(Nfc_debug_PROC_FILE, 0666, NULL);
    if (nfc_debug_proc_file) {
        nfc_debug_proc_file->proc_fops = &nfc_debug_proc_ops;
    } 
}
#endif


static int TestNfcI2C(struct i2c_client *apClient)
{
	int Result;

	i2c_log_in_test_case("TestNfcI2C ++\n");
	
	i2c_log_in_test_case("start the Nfc i2c test...\n");
	Result = StartI2CTest();	
	if(Result == I2C_TEST_NFC_FAIL)
	       return Result;
	i2c_log_in_test_case("TestNfcI2C --\n");

	return Result;
}

static struct i2c_test_case_info NfcTestCaseInfo[] =
{
     __I2C_STRESS_TEST_CASE_ATTR(TestNfcI2C),
};
#endif
//#define NFC_GPIO_IRQ	59
//#define NFC_GPIO_EN	79
//#define NFC_GPIO_DOWNLOAD	24

static int pn544_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret, irq_num;

	pn544_dev = kzalloc(sizeof(struct pn544_dev), GFP_KERNEL);
	if (pn544_dev == NULL) {
		printk("failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_exit;
 	}

	pn544_dev->client   = client; 

	/* Get data that is defined in board specific code. */
	pn544_dev->irq_gpio = of_get_named_gpio_flags(client->dev.of_node, "nxp,irq-gpio",0,NULL);
	ret = gpio_request(pn544_dev->irq_gpio, "nfc_int");
	if (ret)
	{
		printk("nfc_int fail\n");
		return  -ENODEV;
	}

	pn544_dev->ven_gpio = of_get_named_gpio_flags(client->dev.of_node, "nxp,nfc-ven",0,NULL);
	ret = gpio_request(pn544_dev->ven_gpio, "nfc_ven");
	if (ret)
	{
		printk("nfc_ven fail\n");
		goto err_ven;
	}

	pn544_dev->firm_gpio = of_get_named_gpio_flags(client->dev.of_node, "nxp,download-gpio",0,NULL);
        if(gpio_is_valid(pn544_dev->firm_gpio)) {
		gpio_request(pn544_dev->firm_gpio, "nfc_download");
	}

	ret = gpio_direction_input(pn544_dev->irq_gpio);
	if (ret < 0) {
		pr_err("%s :not able to set Nfc's irq_gpio as input\n", __func__);
		goto err_ven;
 	}

	ret = gpio_direction_output(pn544_dev->ven_gpio, 0);
	if (ret < 0) {
		pr_err("%s : not able to set Nfc's ven_gpio as output\n", __func__);
		goto err_firm;
	}

	ret = gpio_direction_output(pn544_dev->firm_gpio, 0);
	if (ret < 0) {
		pr_err("%s : not able to set nfc_download as output\n",__func__);
		goto err_firm;
	}

	#ifdef CONFIG_I2C_STRESS_TEST
    	i2c_log_in_test_case("Nfc add test case+\n");
	i2c_add_test_case(pn544_dev->client, "NfcTest",ARRAY_AND_SIZE(NfcTestCaseInfo));
	#endif

	/* init mutex and queues */
	init_waitqueue_head(&pn544_dev->read_wq);
	mutex_init(&pn544_dev->read_mutex);
	spin_lock_init(&pn544_dev->irq_enabled_lock);

	pn544_dev->pn544_device.minor = MISC_DYNAMIC_MINOR;
	pn544_dev->pn544_device.name = "pn544";
	pn544_dev->pn544_device.fops = &pn544_dev_fops;

	ret = misc_register(&pn544_dev->pn544_device);
	if (ret) {
		pr_err("%s : misc_register failed\n", __FILE__);
		goto err_misc_register;
 	}
 
//-	dev_dbg(&client->dev, "%s: info: %p, pdata %p, client %p\n",
//-		__func__, info, pdata, client);
	/* request irq.  the irq is set whenever the chip has data available
	 * for reading.  it is cleared when all data has been read.
	 */

	wake_lock_init(&nfc_wake_lock, WAKE_LOCK_SUSPEND, "nfc_wake_lock");

	irq_num = gpio_to_irq(pn544_dev->irq_gpio);
	if (irq_num< 0)
	{
		printk("Unable to get irq number for GPIO \n");
	}

	pr_info("%s : requesting IRQ %d\n", __func__, irq_num);

	pn544_dev->client->irq=irq_num;
	pn544_dev->irq_enabled = true;
	//ret = request_irq(client->irq, pn544_dev_irq_handler, IRQF_TRIGGER_HIGH, client->name, pn544_dev);
	ret = request_irq(irq_num, pn544_dev_irq_handler, IRQF_TRIGGER_HIGH, "pn544", pn544_dev);
	if (ret != 0) {
		printk("request_irq failed\n");
		goto err_request_irq_failed;
	}

	pn544_disable_irq(pn544_dev);
	//i2c_set_clientdata(client, pn544_dev);

 	return 0;

err_request_irq_failed:
	misc_deregister(&pn544_dev->pn544_device);
err_misc_register:
	mutex_destroy(&pn544_dev->read_mutex);
	kfree(pn544_dev);

err_exit:
	if (pn544_dev->firm_gpio) gpio_free(pn544_dev->firm_gpio);
err_firm:
	gpio_free(pn544_dev->ven_gpio);
err_ven:
	gpio_free(pn544_dev->irq_gpio);
//err_add:
	return ret;

}


static int pn544_i2c_remove(struct i2c_client *client)
 {
	struct pn544_dev *pn544_dev;

	pn544_dev = i2c_get_clientdata(client);
	free_irq(client->irq, pn544_dev);
	misc_deregister(&pn544_dev->pn544_device);
	mutex_destroy(&pn544_dev->read_mutex);
	gpio_free(pn544_dev->irq_gpio);
	gpio_free(pn544_dev->ven_gpio);
	if (pn544_dev->firm_gpio)
		gpio_free(pn544_dev->firm_gpio);
	kfree(pn544_dev);
 
 	return 0;
 }


static int nfc_suspend(struct i2c_client *client, pm_message_t mesg)
{
	printk("[nfc] ++pn544_suspend\n");
	if((gpio_get_value(pn544_dev->firm_gpio) == 0) && (gpio_get_value(pn544_dev->ven_gpio) == 1))
	{
		printk("[nfc] ++pn544_suspend - enable_irq_wake\n");
		enable_irq_wake(gpio_to_irq(pn544_dev->irq_gpio));    //enable irq_gpio as wakeup source when NFC suspend
	}
	printk("[nfc] --pn544_suspend\n");
   	return 0;
}

static int nfc_resume(struct i2c_client *client)
{
	printk("[nfc] ++pn544_resume\n");
	if((gpio_get_value(pn544_dev->firm_gpio) == 0) && (gpio_get_value(pn544_dev->ven_gpio) == 1))
	{
		printk("[nfc] ++pn544_resume - disable_irq_wake\n");
		disable_irq_wake(gpio_to_irq(pn544_dev->irq_gpio));    //disable irq_gpio as wakeup source when NFC resume
	}
	wake_lock_timeout(&nfc_wake_lock, 1 * HZ);
	printk("[nfc] --pn544_resume\n");
   	return 0;
}


static const struct i2c_device_id pn544_id[] = {
	{ "pn544", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, pn544_id);

#ifdef CONFIG_OF
static struct of_device_id pn544_match_table[] = {	
	{.compatible = "nxp,pn544",},
	{},
};
#else
#define pn544_match_table NULL
#endif

static struct i2c_driver pn544_i2c_driver = {
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "pn544",
		.of_match_table = pn544_match_table,
 	},
	.probe		= pn544_i2c_probe,
	.suspend = nfc_suspend,
	.resume = nfc_resume,
	.remove		= pn544_i2c_remove,
	.id_table	= pn544_id,
};

module_i2c_driver(pn544_i2c_driver);

MODULE_AUTHOR("Sylvain Fonteneau");
MODULE_DESCRIPTION("NFC PN544 driver");
MODULE_LICENSE("GPL");

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
	.remove		= pn544_i2c_remove,
	.id_table	= pn544_id,
};

module_i2c_driver(pn544_i2c_driver);

MODULE_AUTHOR("Sylvain Fonteneau");
MODULE_DESCRIPTION("NFC PN544 driver");
MODULE_LICENSE("GPL");

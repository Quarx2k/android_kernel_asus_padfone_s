#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#ifdef CONFIG_EEPROM_NUVOTON
#include <linux/microp.h>
#include <linux/microp_api.h>
#include <linux/microp_pin_def.h>
#endif


#define DEV_INFO(args... )  pr_info("[STPD2600] " args)

struct i2c_client *g_stdp2600_client = NULL;




static int sdtp_write_reg( u8 reg, int len, void *data)
{
	     int err = 0;
	     unsigned char buff[len+1];
	
	     struct i2c_msg msg[] = {
	      		{
	      		 .addr = g_stdp2600_client->addr,
	      		.flags = 0, //write
	      		.len = len + 1,
	      		.buf = buff,
	       	},
	     };
	
	       buff[0] = reg;
	      memcpy(buff + 1, data, len);

	      if (!g_stdp2600_client->adapter)
            		return -ENODEV;
	
	      err = i2c_transfer(g_stdp2600_client->adapter, msg, ARRAY_SIZE(msg));
	
	      if (err != ARRAY_SIZE(msg))
	            DEV_INFO("%s: %d\n",__func__,err);
	
	      return err; 
}

static int sdtp_read_reg(  int len, void *data)
{
	     int err = 0;
	    
	
	     struct i2c_msg msg[] = {
	      		{
	      		 .addr = g_stdp2600_client->addr,
	      		.flags = I2C_M_RD, 
	      		.len = len ,
	      		.buf = data,
	       	},
	     };

	      if (!g_stdp2600_client->adapter)
            		return -ENODEV;
		memset(data,0,sizeof(data));
		
	      err = i2c_transfer(g_stdp2600_client->adapter, msg, ARRAY_SIZE(msg));
	
	      if (err != ARRAY_SIZE(msg))
	            DEV_INFO("%s: %d\n",__func__,err);
	
	      return err; 
}
static int stdp_get_fw_ver(struct device * dev, struct device_attribute * attr, char * buf)
{
	
	       int err = -1;
	       unsigned char buff[2] = {0,0};
	       #ifdef CONFIG_EEPROM_NUVOTON
		err=AX_MicroP_setGPIOOutputPin( OUT_uP_STDP_I2C_EN, 1);
		#endif
		if(err< 0 ){
			DEV_INFO("AX_MicroP_setGPIOOutputPin error(%d)! \n",err);
			goto exit;
		}
	       //.....Sent 0x00 , 0x05 , 0x0A
	       buff[0] = 0x05;
	       buff[1] = 0x0A;
	       err = sdtp_write_reg( 0x00, 2, &buff);
	       if(err < 0){
	              DEV_INFO("sdtp_write_reg error(%d)! \n",err);
			memset(buff,0,sizeof(buff));
			goto exit;
		 }
	      buff[0] = 0x00;
	       buff[1] = 0x00;
	       err = sdtp_read_reg( 2, &buff);
	       if(err < 0){
	             DEV_INFO("sdtp_read_reg error(%d)!\n",err);
		      memset(buff,0,sizeof(buff));
	       }
exit:
		#ifdef CONFIG_EEPROM_NUVOTON
		AX_MicroP_setGPIOOutputPin( OUT_uP_STDP_I2C_EN, 0);
		#endif

	       return sprintf(buf, "%x,%x\n", buff[0],buff[1]);
}



#ifdef CONFIG_I2C_STRESS_TEST
#include <linux/i2c_testcase.h>
#define I2C_TEST_STDP_FAIL (-1)
static int TestSTDPRead(struct i2c_client *client)
{
	int lnResult = I2C_TEST_PASS;
	char buf[8];
	i2c_log_in_test_case("TestSTDP++\n");
	stdp_get_fw_ver(NULL,NULL,buf);
	if(!strcmp(buf,"0,0\n")){
		lnResult=I2C_TEST_STDP_FAIL;
		printk("%s, STDP2600 I2C error \n",__func__);
	}
	i2c_log_in_test_case("TestSTDP--\n");
	return lnResult;
};
static struct i2c_test_case_info gSTDPTestCaseInfo[] =
{
	__I2C_STRESS_TEST_CASE_ATTR(TestSTDPRead),
};
#endif

static DEVICE_ATTR(fw_ver, S_IRWXU | S_IRWXG | S_IROTH, stdp_get_fw_ver, NULL);

static struct attribute *sdtp_attributes[] = {
	       &dev_attr_fw_ver.attr,
	  	 NULL
};

static const struct attribute_group sdtp_attr_group = {
	    .name = "SDTP2600",
	    .attrs = sdtp_attributes,
};
static int stdp2600_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id){
	int ret = 0;
	DEV_INFO(" %s:++++++++\n",__func__);
//	memcpy(&g_stdp2600_client, &client, sizeof(client));
	g_stdp2600_client=client;
	ret = sysfs_create_group(&client->dev.kobj, &sdtp_attr_group);
	if(ret)
		DEV_INFO("Create I2C group fail\n");
	
#ifdef CONFIG_I2C_STRESS_TEST

	i2c_add_test_case(client, "STDP2600",ARRAY_AND_SIZE(gSTDPTestCaseInfo));

#endif
	DEV_INFO("%s:----------\n",__func__);
	return 0;

}
static struct of_device_id stdp2600_match_table[]={
{.compatible="stpd,stpd2600",},
{},
};


static const struct i2c_device_id stdp2600_id[] = {
	{ "stdp2600", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, stdp2600_id);

static struct i2c_driver stdp2600_driver = {
	.driver  = {
		.name  = "stdp2600",
		.owner  = THIS_MODULE,
		.of_match_table = stdp2600_match_table,
	},

	.probe  = stdp2600_i2c_probe,
	.id_table  = stdp2600_id,  
};
module_i2c_driver(stdp2600_driver);
MODULE_DESCRIPTION("STDP2600");
MODULE_AUTHOR("Wei_Lai <Wei_Lai@asus.com>");
MODULE_LICENSE("GPL");

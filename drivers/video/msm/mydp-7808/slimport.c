/*
 * Copyright(c) 2012, Analogix Semiconductor. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
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
#include "slimport_tx_drv.h"
#include "slimport.h"
#include <linux/stat.h>
#include <linux/regulator/consumer.h> //+++ ASUS BSP Bernard for regulator control
#include <linux/of_gpio.h> //+++ ASUS BSP Bernard for device tree: get gpio
#include <linux/err.h> //+++ ASUS BSP Bernard for debug error
//asus wei lai+++
#include <linux/of_platform.h>
#include "slimport_tx_reg.h"
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h> 
#endif
#include<linux/irq.h>
//asus wei lai---
#include <linux/of_irq.h>//+++ ASUS BSP Bernard for dts, get irq 
//asus wei lai+++
#ifdef CONFIG_EEPROM_NUVOTON
#include <linux/microp.h>
#include <linux/microp_api.h>
#include <linux/microp_pin_def.h>
#endif
//asus wei lai---
struct i2c_client *anx7808_client;

int g_i2c_error_count = 0;
unchar g_hdmi_rx_vsync_change = 0;
static int cableInserted = 0;
//+++ ASUS BSP Bernard
//asus_debug_mask(MYDP);

#ifndef ASUS_DEV_WARN
#define ASUS_DEV_WARN(args...) printk("[MYDP] " args)
#endif

#ifndef ASUS_DEV_INFO
#define ASUS_DEV_INFO(args... ) printk("[MYDP] " args)
#endif
//--- ASUS BSP Bernard

#ifndef EYE_TEST
//Top tree, default enable HDCP
bool hdcp_enable = 1;
#endif
extern int Is_HDMI_power_off; //ASUS BSP wei +++
 int g_MyDP_HDCP_FAIL_COUNT=0;//ASUS BSP wei +++
int g_isMyDP_poweron=0; //ASUS BSP wei +++
struct anx7808_data *g_anx7808;//ASUS BSP wei +++
static bool g_b_SwitchCarkitBootPlugin=false; //+++ ASUS BSP Bernard
//ASUS BSP wei lai +++
int dp_pd_value=0;
int read_swing_value=0;
int write_swing_value=0;
int myDP_TV_mode=0;
int g_dump_7730_reg = 0;
int myDP_force_pad_mode=0;
int g_dump_7808_reg = 0;
int g_force_swing_value = 1;
int g_swing_value=0;  // 2:600mv  -->  0:200mv
int g_pre_emphis_value=1;  // 3.5db
int myDP_DP_Dongle = 0;
int g_hdmi_hdcp_fail=0; //ASUS BSP wei+++
int g_Pad_LT_Fail_Count=0;//ASUS BSP wei+++
int trigger_sw_reset=0; //ASUS BSP wei +++
int g_myDP_test_pattern = 0; //+++ ASUS BSP Bernard
int g_pattern_type = 0; //+++ ASUS BSP Bernard
int g_myDP_SSC_on = 1; //+++ ASUS BSP Bernard
int g_myDP_type = 2; //+++ ASUS BSP Bernard, pad:2
int mydp_diffclk_flag = 0; //+++ ASUS BSP Bernard
extern int mydp_ac_charger_flag; //+++ ASUS BSP Bernard
int resume_trigger=0;
int g_carkit_flag = 0; //+++ ASUS BSP Bernard
//ASUS BSP Wei +++ 
int g_enable_dynamic_ssc=0;
int spreading_ctrl1=0xc0;
int spreading_ctrl2=0x00;
int spreading_ctrl3=0x75;


int isHdmiConnected(void) {
	printk("Cable: %d\n", cableInserted);
	return cableInserted;
}

struct mydp_command mydp_cmd_tb[] = {
	{DYM_SSC,sizeof(DYM_SSC),mydp_dymSSC}
#if 0
	{DPCDR,		sizeof(DPCDR),	mydp_dpcdr},		//read 1 byte content from SP rx
	{DPCDW,		sizeof(DPCDW),	mydp_dpcdw},		//write 1 byte content to SP rx
	{DUMPDPCD,	sizeof(DUMPDPCD),	mydp_dumpdpcd},//dump 256 bytes from SP
	{RD,			sizeof(RD),		mydp_rd}, 	//read 1 byte from i2c slave add
	{WR,			sizeof(WR),		mydp_wr},		//write 1 byte to i2c slave add
	{DUMP,			sizeof(DUMP),	mydp_dump},  //dump 256 bytes from i2c slave add
	{RDRX,			sizeof(RDRX),	mydp_rdrx},	//read 1 byte from i2c slave add over C-wire
	{WRRX,			sizeof(WRRX),	mydp_wrrx},	//write 1 byte to i2c slave add over C-wire
	{DUMPRX,		sizeof(DUMPRX),	mydp_dumprx},//dump 256 bytes from i2c slave add over C-wire
	{SHOW,			sizeof(SHOW),	mydp_show}, 		//show video information from anx7808
	{ERRCHK,		sizeof(ERRCHK),	mydp_errchk},	//check error on the lanes
	{SWING,		sizeof(SWING),	mydp_swing},		//set output swing	
	{EMP,			sizeof(EMP),		mydp_emp},		//set pre-emphasis

#endif
};

void mydp_write(char *msg)
{
	int i = 0;
	for(; i<ARRAY_SIZE(mydp_cmd_tb);i++){
		if(strncmp(msg, mydp_cmd_tb[i].id, mydp_cmd_tb[i].len-1)==0){
			mydp_cmd_tb[i].func(msg, mydp_cmd_tb[i].len-1);
			return;
		}	
	}
	DEV_DBG("### error Command no found in  %s ###\n", __FUNCTION__);
	return;
}
ssize_t mydp_write_proc(struct file *filp, const char __user *buff, 
	            unsigned long len, void *data)
{
	char messages[256];

	if (len > 256) {
		len = 256;
	}
	DEV_DBG("#### %s ###\n", __FUNCTION__);
	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}

	mydp_write(messages);
	//DEV_DBG("[MyDP]\n");
	
	return len;
}
static void create_MYDP_proc_file(void)
{
	struct proc_dir_entry *mydp_proc_file = create_proc_entry(MYDP_PROC, 0, NULL);

	if (mydp_proc_file) {
		mydp_proc_file->write_proc =mydp_write_proc;
	}
	else {
		ASUS_DEV_WARN("proc file create failed!\n");
    	}

	return;
}

//ASUS BSP Wei ---
#ifdef CONFIG_I2C_STRESS_TEST
#include <linux/i2c_testcase.h>
#define I2C_TEST_MYDP_FAIL (-1)
static int TestMyDPRead(struct i2c_client *client)
{
	int lnResult = I2C_TEST_PASS;
	int error;
	unchar c1,c2;
	i2c_log_in_test_case("TestMyDP++\n");
	disable_irq(client->irq);
	sp_tx_hardware_poweron(client);
	error = sp_read_reg(TX_P2, SP_TX_DEV_IDL_REG, &c1);
	if (error < 0) {
		lnResult = I2C_TEST_MYDP_FAIL;
	}
	error = sp_read_reg(TX_P2, SP_TX_DEV_IDH_REG, &c2);
	if (error < 0) {
		lnResult = I2C_TEST_MYDP_FAIL;
	}
	sp_tx_hardware_powerdown( client);
	enable_irq(client->irq);
	msleep(300);
	i2c_log_in_test_case("TestMyDP--\n");
	return lnResult;
};


extern int AX_MicroP_enterSleeping(void);
extern int AX_MicroP_enterResuming(void);
static int TestMyDPRead1(struct i2c_client *client)
{
	int lnResult = I2C_TEST_PASS;

	i2c_log_in_test_case("TestMyDP++\n");
	AX_MicroP_enterSleeping();
	msleep(1500);
	AX_MicroP_enterResuming();
	AX_MicroP_setPWMValue(133);
#if 0
	mutex_lock(&g_anx7808->lock);
	disable_irq(client->irq);
	sp_tx_sw_error_power_down();

	mutex_unlock(&g_anx7808->lock);
	flush_workqueue(g_anx7808 ->workqueue);
	enable_irq(client->irq);
	printk("------------------test3-----------\n");
#endif
	msleep(5000);
	i2c_log_in_test_case("TestMyDP--\n");
	return lnResult;
};

static struct i2c_test_case_info gMyDPTestCaseInfo[] =
{
	__I2C_STRESS_TEST_CASE_ATTR(TestMyDPRead),
};
static struct i2c_test_case_info gMyDPTestCaseInfo1[] =
{
	__I2C_STRESS_TEST_CASE_ATTR(TestMyDPRead1),	
};
//--- ASUS BSP Bernard

#endif
int dump_7808_reg_info(void)
{
	unchar ch, cl;
	static uint cur_h_res=0,cur_v_res=0;
	uint tmp;
	sp_read_reg(RX_P0, HDMI_RX_HTOTAL_LOW, &cl);
	sp_read_reg(RX_P0, HDMI_RX_HTOTAL_HIGH, &ch);
	tmp = ch;
	tmp= (tmp<< 8) + cl;

	if(tmp!=cur_h_res){
		ASUS_DEV_WARN("ANX7808 RX old cur_h_res = 0x%x\n", cur_h_res);
		ASUS_DEV_WARN("ANX7808 RX new cur_h_res = 0x%x\n", tmp);
		cur_h_res=tmp;
	}
	sp_read_reg(RX_P0, HDMI_RX_VTOTAL_LOW, &cl);
	sp_read_reg(RX_P0, HDMI_RX_VTOTAL_HIGH, &ch);
	tmp = ch;
	tmp = (tmp << 8) + cl;
	
	if(tmp!=cur_v_res){
		ASUS_DEV_WARN("ANX7808 RX old cur_v_res = 0x%x\n", cur_v_res);
		ASUS_DEV_WARN("ANX7808 RX new cur_v_res = 0x%x\n", tmp);
		cur_v_res=tmp;
	}
	return 0;
}

int dump_7808_reg_pclk(void)
{
	ulong str_clk = 0;
	unchar c;
 	ulong pclk;
 	ulong m_val, n_val;
	str_clk = 162;

	sp_read_reg(TX_P0, M_VID_2, &c);
	m_val = c * 0x10000;
	sp_read_reg(TX_P0, M_VID_1, &c);
	m_val = m_val + c * 0x100;
	sp_read_reg(TX_P0, M_VID_0, &c);
	m_val = m_val + c;

	sp_read_reg(TX_P0, N_VID_2, &c);
	n_val = c * 0x10000;
	sp_read_reg(TX_P0, N_VID_1, &c);
	n_val = n_val + c * 0x100;
	sp_read_reg(TX_P0, N_VID_0, &c);
	n_val = n_val + c;

	str_clk = str_clk * m_val;
	pclk = str_clk;
	pclk = pclk / n_val;
	DEV_DBG("   M = %lu, N = %lu, PCLK = %ld MHz\n", m_val, n_val, pclk);
	return 0;
}
void sp_tx_hdmi_error_power_down(void);
extern void reportPadStationI2CFail(char *devname);
static int dp_reset_pd_function(const char *val, struct kernel_param *kp)
{
	int ret=0;
	int old_val = dp_pd_value;
	if (ret)
		return ret;

	if (dp_pd_value > 0xf)  {
		dp_pd_value = old_val;
		return -EINVAL;
	}

	ret = param_set_int(val, kp);
	if(dp_pd_value==0){
		unchar ch, cl;
		uint n;
		sp_read_reg(RX_P0, HDMI_RX_HTOTAL_LOW, &cl);
		sp_read_reg(RX_P0, HDMI_RX_HTOTAL_HIGH, &ch);
		n = ch;
		n = (n << 8) + cl;

		ASUS_DEV_WARN("ANX7808 RX cur_h_res = 0x%x\n", n);
		sp_read_reg(RX_P0, HDMI_RX_VTOTAL_LOW, &cl);
		sp_read_reg(RX_P0, HDMI_RX_VTOTAL_HIGH, &ch);
		n = ch;
		n = (n << 8) + cl;
		ASUS_DEV_WARN("ANX7808 RX cur_v_res = 0x%x\n", n);

		sp_read_reg(RX_P0, HDMI_RX_VID_PCLK_CNTR_REG, &cl);
		ASUS_DEV_WARN("ANX7808 RX cur_pix_clk = 0x%x\n", cl);

		sp_read_reg(RX_P0, HDMI_RX_HDMI_STATUS_REG, &cl);
		ASUS_DEV_WARN("ANX7808 RX dvi_status = 0x%x\n",  ((cl & HDMI_MODE) == HDMI_MODE));	
	}
	else if (dp_pd_value==1) {
		hdcp_enable=0;
	}
	else if (dp_pd_value==2) {
		hdcp_enable=1;
	}
	else if (dp_pd_value==3) {
	#ifdef CONFIG_EEPROM_NUVOTON
		reportPadStationI2CFail("MyDP");
	#endif
	}	
	else if (dp_pd_value==4) {
		sp_tx_hardware_poweron(anx7808_client);				
	}
	else if (dp_pd_value==5) {
		sp_tx_hardware_powerdown(anx7808_client);		
	}	
	else if (dp_pd_value==6) {
		unchar  c1=0;
/*		
		unchar addr_tmp = 0x50;
		switch(addr_tmp)
		{
			case 0x50: c= 0; break;
			case 0x8c: c= 1; break;
			case 0x70: c= 7; break;
			case 0x72: c = 5; break;
			case 0x7a: c = 6; break;
			default : break;

		}
*/		
		i2c_master_read_reg(0, 0x0B, &c1);
		ASUS_DEV_WARN("7730reg 0x50, 0x0B=%x\n", (uint)c1);
		msleep(1);
		i2c_master_read_reg(5, 0xE3, &c1);
		ASUS_DEV_WARN("7730reg 0x72, 0xE3=%x\n", (uint)c1);		
		msleep(1);
		i2c_master_read_reg(0, 0x06, &c1);
		ASUS_DEV_WARN("7730reg 0x50, 0x06=%x\n", (uint)c1);
		msleep(1);
		i2c_master_read_reg(5, 0x06, &c1);
		ASUS_DEV_WARN("7730reg 0x72, 0x06=%x\n", (uint)c1);				
		i2c_master_read_reg(0, 0x05, &c1);
		ASUS_DEV_WARN("7730reg 0x50, 0x05=%x\n", (uint)c1);				
	}	
	else if (dp_pd_value==7) {
		unchar c=0;
		
		ASUS_DEV_WARN("write 7730reg 0, 0x06 bit5\n");				
		i2c_master_read_reg(0,0x06, &c);
		c = c | 0x20; 
		i2c_master_write_reg(0,0x06, c);
	}	
	else if (dp_pd_value==8) {
		unchar c=0;
		
		ASUS_DEV_WARN("write 7730reg 0, 0x06 bit3\n");				
		i2c_master_read_reg(0,0x06, &c);
		c = c | 0x08; 
		i2c_master_write_reg(0,0x06, c);
	}	
	else if (dp_pd_value==9) {
		unchar c=0;
		
		ASUS_DEV_WARN("write 7730reg 0, 0x06 bit2\n");				
		i2c_master_read_reg(0,0x06, &c);
		c = c | 0x04; 
		i2c_master_write_reg(0,0x06, c);
	}		
	else if (dp_pd_value==10) {
		
		unchar c=0;

		ASUS_DEV_WARN("write 7730reg 0, 0x06 bit0\n");				
		i2c_master_read_reg(0,0x06, &c);
		c = c | 0x01; 
		i2c_master_write_reg(0,0x06, c);
	}		
	else if (dp_pd_value==11) {				
		unchar c=0;
		
		ASUS_DEV_WARN("write 7730reg 5, 0x06 bit5\n");				
		i2c_master_read_reg(5,0x06, &c);
		c = c | 0x20; 
		i2c_master_write_reg(5,0x06, c);		
	}	
	else if (dp_pd_value==12) {
		myDP_TV_mode = 1;
	}			
	else if (dp_pd_value==13) {
		myDP_TV_mode = 0;
	}			
	else if (dp_pd_value==14) {
		sp_tx_hdmi_error_power_down();
	}			
	else if (dp_pd_value==15) {

	}			
	return 0;
}

extern void sp_tx_phy_auto_test(void);

//+++ ASUS BSP Bernard; for OTG
bool get_otg_state(void){
	return g_b_SwitchCarkitBootPlugin;
}
//--- ASUS BSP Bernard; for OTG

static int read_swing_function(const char *val, struct kernel_param *kp)
{
	int ret=0;
	unchar c=0;
	int old_val = read_swing_value;

	if (ret)
		return ret;

	if (read_swing_value > 0xf)  {
		read_swing_value = old_val;
		return -EINVAL;
	}
	ret = param_set_int(val, kp);
	if (read_swing_value==1)
	{
		sp_read_reg(TX_P0, 0xa3, &c);
		ASUS_DEV_WARN("TX_P0  0xa3: %x\n",c);
	}
	else if (read_swing_value==2)  // test tx phy 
	{
		sp_tx_phy_auto_test();
	}
	else if (read_swing_value==3) 
	{
		unchar bSwing, bEmp;
		sp_read_reg(TX_P0, 0xA3, &bSwing);
		sp_write_reg(TX_P0, 0xA3, (bSwing&~0x03)|0x02);
		ASUS_DEV_WARN("##lane0,Swing600mv\n");	
		sp_read_reg(TX_P0, 0xA3, &bEmp);
		sp_write_reg(TX_P0, 0xA3, (bEmp&~0x18)|0x08);
		ASUS_DEV_WARN("## lane0,emp 3.5db\n");
	}
	else if (read_swing_value==4)
	{
		unchar c, c1;
		sp_read_reg(TX_P0, SP_TX_LT_SET_REG, &c);
		sp_read_reg(TX_P0, SP_TX_LINK_BW_SET_REG, &c1);
		DEV_NOTICE("HW LT succeed,LANE0_SET = %.2x,", (uint) c);
		DEV_NOTICE("link_bw = %.2x\n", (uint) c1);	
	}	
	else if (read_swing_value==5)
	{
		g_dump_7730_reg = 1;
	}
	else if (read_swing_value==6)
	{
		g_dump_7730_reg = 0;
	}
	else if (read_swing_value==7)
	{
		unchar c=0;
		ASUS_DEV_WARN("HW reset === 7730reg 0x72, 0x06 bit0\n");				
		i2c_master_read_reg(5,0x06, &c);
		c = c | 0x01; 
		i2c_master_write_reg(5,0x06, c);		
	}
	else if (read_swing_value==8)
	{
		unchar c=0;
		int i;
		
		ASUS_DEV_WARN("******* ANX7730 reg 0x50 DUMP ============\n");
		for (i=0; i <=0xff; i++)
		{
			i2c_master_read_reg(0, i , &c);
			if  (i%0xf)
				ASUS_DEV_WARN("0x%x = (%x), ", i, c);										
			else	
				ASUS_DEV_WARN("0x%x = (%x)\n", i, c);										
				
		}

		ASUS_DEV_WARN("******* ANX7730 reg 0x72 DUMP ============\n");
		for (i=0; i <=0xff; i++)
		{
			i2c_master_read_reg(5, i , &c);
			if  (i%0xf)
				ASUS_DEV_WARN("0x%x = (%x), ", i, c);										
			else	
				ASUS_DEV_WARN("0x%x = (%x)\n", i, c);										
		}		
	}
	else if (read_swing_value==9)
		myDP_force_pad_mode =1;
	else if (read_swing_value==10)
		myDP_force_pad_mode = 0;
	else if (read_swing_value==11)
	{
		g_dump_7808_reg	= 1;
	}
	else if (read_swing_value==12)
	{
		g_dump_7808_reg	= 0;
	}
	else if(read_swing_value == 13)
	{
		g_enable_dynamic_ssc=0;

	}
	else if(read_swing_value == 14)
	{
		g_enable_dynamic_ssc=1;
	}

	return 0;
}
static int write_swing_function(const char *val, struct kernel_param *kp)
{
	int ret=0;
	int old_val = write_swing_value;
	unchar c=0;
	unchar bSwing=0; //ASUS BSP Bernard +++

	if (ret)
		return ret;

	if (write_swing_value > 0xf)  {
		write_swing_value = old_val;
		return -EINVAL;
	}
	ret = param_set_int(val, kp);
	 if (write_swing_value==0)
	{
		g_force_swing_value = 1;	
	}
	else if (write_swing_value==1)
	{
		g_force_swing_value = 0;
	}
	else if (write_swing_value==2)
	{	
		if(g_swing_value == 0) {
			DEV_DBG("it has been 200mv swing/emp.\n");
			return 0;
		}
		if(sp_tx_asus_pad && (sp_tx_system_state == STATE_PLAY_BACK)) {
			g_swing_value	= 0;
			ASUS_DEV_WARN("7808 swing = 200mv\n");
			sp_write_reg(TX_P0, 0xA3, (g_swing_value)  | (g_pre_emphis_value << 3) );    
			sp_read_reg(TX_P0, 0xA3, &bSwing);
              	DEV_DBG("mydp 200mv swing/emp = 0x%x\n", bSwing);
		}
		else  {
			g_swing_value	= 0;
			ASUS_DEV_WARN("7808 swing = 200mv\n");
		}
	}
	else if (write_swing_value==3)
	{
		if(g_swing_value == 1) {
			DEV_DBG("it has been 400mv swing/emp.\n");
			return 0;
		}
		if(sp_tx_asus_pad && (sp_tx_system_state == STATE_PLAY_BACK)) {
			g_swing_value	= 1;
			ASUS_DEV_WARN("7808 swing = 400mv\n");
			sp_write_reg(TX_P0, 0xA3, (g_swing_value)  | (g_pre_emphis_value << 3) );    
			sp_read_reg(TX_P0, 0xA3, &bSwing);
	              DEV_DBG("mydp 400mv swing/emp = 0x%x\n", bSwing);
		}
		else {
			g_swing_value	= 1;
			ASUS_DEV_WARN("7808 swing = 400mv\n");
		}
	}
	else if (write_swing_value==4)
	{
		if(g_swing_value == 2) {
			DEV_DBG("it has been 600mv swing/emp.\n");
			return 0;
		}
		if(sp_tx_asus_pad && (sp_tx_system_state == STATE_PLAY_BACK)) {
			g_swing_value	= 2;
			ASUS_DEV_WARN("7808 swing = 600mv\n");
			sp_write_reg(TX_P0, 0xA3, (g_swing_value)  | (g_pre_emphis_value << 3) );    
			sp_read_reg(TX_P0, 0xA3, &bSwing);
               	DEV_DBG("mydp 600mv swing/emp = 0x%x\n", bSwing);
		}
		else {
			g_swing_value	= 2;
			ASUS_DEV_WARN("7808 swing = 600mv\n");
		}

	}
	else if (write_swing_value==5)
	{
		g_swing_value	= 3;
		ASUS_DEV_WARN("7808 swing = 800mv\n");
		
	}
	else if (write_swing_value==6)
	{
					
                sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x06, 1, &c);
                ASUS_DEV_WARN("sp_tx_config_hdmi_pad , ADJUST_REQUEST_LANE0_1 = 0x%x\n", c);		
		
	}
	//+++ ASUS BSP Bernard, for test pattern
	else if (write_swing_value==7)
	{
		g_myDP_test_pattern = 1;
		g_pattern_type = 0;
	}
	else if (write_swing_value==8)
	{
		g_myDP_test_pattern = 1;
		g_pattern_type = 1;
	}
	else if (write_swing_value==9)
	{
		g_myDP_test_pattern = 1;
		g_pattern_type = 2;
	}
	else if (write_swing_value==10)
	{
		g_myDP_test_pattern = 1;
		g_pattern_type = 3;
	}
	else if (write_swing_value==11)
	{
		g_myDP_test_pattern = 0;
		g_pattern_type = 0;
	}
	else if (write_swing_value==12)
	{
		g_myDP_type = 0;
	}else if (write_swing_value==13)
	{
		g_myDP_type = 1;
	}else if (write_swing_value==14)
	{
		g_myDP_type = 2;
	}
	else if (write_swing_value==15)
	{
		g_myDP_SSC_on = 1;
	}
	else if (write_swing_value ==16)
	{
		g_myDP_SSC_on = 0;
	}
	//--- ASUS BSP Bernard, for test pattern
	
	return 0;
}

module_param_call(dp_pd_value, dp_reset_pd_function, param_get_int,
			&dp_pd_value, 0644);

#ifdef CONFIG_HAS_EARLYSUSPEND

static void dp7808_early_suspend(struct early_suspend *handler){
	return ; //now force , still implement
}

static void dp7808_late_resume(struct early_suspend *handler){
	return ; //now force , still implement
}

static struct early_suspend dp7808_early_suspend_desc = {
    .level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
    .suspend = dp7808_early_suspend,
    .resume = dp7808_late_resume,
};
#endif
//ASUS BSP wei lai ---
#ifdef CONFIG_ASUS_HDMI
extern int measure_pad_plugin_time;
#endif

#if 1

static int anx7808_suspend(struct i2c_client *client, pm_message_t mesg)
{	

	ASUS_DEV_WARN("anx7808_suspend (not implement) +++\n");
#ifdef CONFIG_ASUS_HDMI
	Is_HDMI_power_off=0; //ASUS wei +++
#endif

	if(gpio_get_value(75)){
			ASUS_DEV_WARN("resume_trigger = 1 \n");
			resume_trigger=1;
	}
//	measure_pad_plugin_time=0;
	ASUS_DEV_WARN("anx7808_suspend (not implement) ---\n");	
	
	return 0;
}
static int anx7808_resume(struct i2c_client *client)
{

	ASUS_DEV_WARN("anx7808_resume (not implement) ++++\n");
	   
	ASUS_DEV_WARN("anx7808_resume (not implement) ----\n");
	
	return 0;
}
#endif

module_param_call(read_swing_value, read_swing_function, param_get_int,
			&read_swing_value, 0644);
module_param_call(write_swing_value, write_swing_function, param_get_int,
			&write_swing_value, 0664);

//ASUS BSP wei lai +++
static void (*notify_carkit_in_out_func_ptr)(int) = NULL;

void dp_switch_carkit(bool enable)
{	
	g_b_SwitchCarkitBootPlugin= enable;	//+++ ASUS BSP Bernard
        if(NULL != notify_carkit_in_out_func_ptr)
	{
             ASUS_DEV_WARN("[dp] carkit cable notify (%d)\n", enable);
             (*notify_carkit_in_out_func_ptr) (enable);
        }
	ASUS_DEV_INFO("dp_switch_carkit+++++ \n");
}
//ASUS BSP wei lai ---


int dp_registerCarkitInOutNotificaition(void (*callback)(int))
{
	ASUS_DEV_WARN("%s +++\n",__FUNCTION__);

	notify_carkit_in_out_func_ptr = callback;
	return 0;
}




int sp_read_reg(uint8_t slave_addr, uint8_t offset, uint8_t *buf)
{
	int ret = 0;
       if ((g_i2c_error_count) > 20)
               return -1;

	anx7808_client->addr = (slave_addr >> 1);
	ret = i2c_smbus_read_byte_data(anx7808_client, offset);
	if (ret < 0) {
		DEV_ERR("%s: failed to read i2c addr=%x\n",
			__func__, slave_addr);
		if ((++g_i2c_error_count) > 20)
		{
			ASUS_DEV_WARN("myDP read i2c error, power down\n");
			//sp_tx_hardware_recovery(anx7808_client);
		}	
		return ret;
	}
	else
	{
		g_i2c_error_count = 0;
	}		
	*buf = (uint8_t) ret;

	return 0;
}

int sp_write_reg(uint8_t slave_addr, uint8_t offset, uint8_t value)
{
	int ret = 0;

	if ((g_i2c_error_count) > 20)
               return -1;

	anx7808_client->addr = (slave_addr >> 1);
	ret = i2c_smbus_write_byte_data(anx7808_client, offset, value);
	if (ret < 0) {
		DEV_ERR("%s: failed to write i2c addr=%x\n",
			__func__, slave_addr);
		if ((++g_i2c_error_count) > 20)
		{
			ASUS_DEV_WARN("myDP write i2c error, power down\n");
			//sp_tx_hardware_recovery(anx7808_client);
		}		
	}
	else
	{
		g_i2c_error_count = 0;
	}	
	
	return ret;
}

#if 0
//ASUS BSP wei +++
extern void hdmi_hdcp_recheck(void);
//ASUS BSP wei ---
#endif 

//+++ ASUS BSP Bernard for mydp 1V control
void sp_hardware_1V_Ctrl(int stat, struct anx7808_platform_data *pdata)
{
	ASUS_DEV_WARN("### stat of mydp 1V control = %d ###\n", stat);
	if(stat) {
		regulator_enable(pdata->sp_regulator);
		ASUS_DEV_INFO("1V is enabled\n");
	}
	else{
		regulator_disable(pdata->sp_regulator);
		ASUS_DEV_INFO("1V is disabled\n");
	}
}
//--- ASUS BSP Bernard for mydp 1V control

#ifdef CONFIG_ASUS_HDMI
extern void ASUS_HDMI_power_on(struct platform_device *pdev,int);
#endif

void sp_tx_hardware_poweron(struct i2c_client *client)
{
	struct anx7808_platform_data *pdata = client->dev.platform_data;

	gpio_set_value(pdata->gpio_rst, 0);
	msleep(2);

//+++ ASUS BSP Bernard: for SR, inverter is disabled
	if (g_ASUS_hwID <= A90_EVB){
		gpio_set_value(pdata->gpio_pw_dwn, 1);
		msleep(2);
	}
      else if (g_ASUS_hwID >= A91_SR1){
	  	gpio_set_value(pdata->gpio_pw_dwn, 0);
		msleep(2);
	}
//--- ASUS BSP Bernard: for SR, inverter is disabled

	sp_hardware_1V_Ctrl(1, pdata); //+++ ASUS BSP Bernard for 1v enable
	msleep(2);
	gpio_set_value(pdata->gpio_rst, 1);

//ASUS Wei_Lai+++
	trigger_sw_reset=0;
    g_i2c_error_count = 0;  
	g_MyDP_HDCP_FAIL_COUNT=0;
	hdcp_enable=1;
	g_hdmi_hdcp_fail=0;
	g_Pad_LT_Fail_Count=0;
#ifdef CONFIG_ASUS_HDMI
	if(Is_HDMI_power_off && gpio_get_value(75)){
		ASUS_HDMI_power_on(g_anx7808->hdmi_pdev,1);
		Is_HDMI_power_off=0;
	}
#endif
//ASUS Wei_Lai---
	DEV_DBG("%s: anx7808 power on\n", __func__);
}

void sp_tx_hardware_powerdown(struct i2c_client *client)
{
	struct anx7808_platform_data *pdata = client->dev.platform_data;

	gpio_set_value(pdata->gpio_rst, 0);
	msleep(2);

	sp_hardware_1V_Ctrl(0, pdata); //+++ ASUS BSP Bernard for 1v disable
	msleep(2);
	
//+++ ASUS BSP Bernard: for SR, inverter is disabled
	if (g_ASUS_hwID <= A90_EVB){
		gpio_set_value(pdata->gpio_pw_dwn, 0);
		msleep(2);
	}
      else if (g_ASUS_hwID >= A91_SR1){
	  	gpio_set_value(pdata->gpio_pw_dwn, 1);
		msleep(2);
	}
//--- ASUS BSP Bernard: for SR, inverter is disabled
	resume_trigger=0;
	g_isMyDP_poweron=0;
	g_i2c_error_count = 0;
//ASUS Wei_Lai+++
	g_MyDP_HDCP_FAIL_COUNT=0;
	hdcp_enable=1;
	g_hdmi_hdcp_fail=0;
	trigger_sw_reset=0;
	g_Pad_LT_Fail_Count=0; 
//ASUS Wei_Lai---
	mydp_ac_charger_flag=0; //+++ ASUS BSP Bernard
	DEV_DBG("%s: anx7808 power down\n", __func__);
}

//+++ ASUS BSP Bernard, check 480p for VGA dongle
int myDP_cable_type(void)
{
	return sp_tx_rx_type;
}
//--- ASUS BSP Bernard, check 480p for VGA dongle

#ifndef EYE_TEST

#ifdef ASUS_A86_PROJECT
extern void nv_touch_mode(int); //+++ ASUS BSP Bernard: touch for hans
#endif

static void getPadSKU(void){
	int Pad_HW_ID = -1;
#ifdef CONFIG_EEPROM_NUVOTON	
	int MicroP_tryCount=5;
#endif
//ASUS_BSP +++ : larry lai for pad solution
				if(sp_tx_asus_pad==1) 
				{
#ifdef CONFIG_EEPROM_NUVOTON	
						while(MicroP_tryCount){
							Pad_HW_ID = AX_MicroP_IsMydpNewSKU();
							if(Pad_HW_ID==-1)
								msleep(100);
							else 
								break;
							MicroP_tryCount--;
						}
							
#else
					Pad_HW_ID = -1;  // default TV mode
#endif
					//ASUS BSP Wei +++ because microp function not ready in booting, in A91 MyDP default DP pad SKU.
#ifdef CONFIG_ASUS_HDMI
					if(g_Pad_Bootup && !g_Android_Boot_Complete){
						if(g_ASUS_hwID >= A91_SR1)
							Pad_HW_ID=1;
					}
#endif
					//ASUS BSP Wei --- because microp function not ready in booting, in A91 MyDP default DP pad SKU.
					
					if (Pad_HW_ID == 1)
					{
						DEV_DBG("### DP Pad detect ###\n");				
						sp_tx_asus_pad = 1;	
						myDP_DP_Dongle = 1;
					}
					else if (Pad_HW_ID == 0)
					{
						DEV_DBG("### HDMI Pad detect ###\n");				
						sp_tx_asus_pad = 1;
						myDP_DP_Dongle = 0;
					}
					else
					{
						if (myDP_force_pad_mode)
						{
							if (myDP_DP_Dongle)
							{
								DEV_DBG("### DP Pad detect ###\n");				
								sp_tx_asus_pad = 1;	
								myDP_DP_Dongle = 1;
							}
							else
							{
								DEV_DBG("### HDMI Pad detect ###\n");				
								sp_tx_asus_pad = 1;
								myDP_DP_Dongle = 0;							
							}
						}
						else
						{
							DEV_DBG("### Fail detect Pad , force TV mode ###\n");									
							sp_tx_asus_pad = 0;
							myDP_DP_Dongle = 0;											
						}
					}
				}
			
			#ifdef ASUS_A86_PROJECT
				else
					if (g_ASUS_hwID != A86_SR3)
						nv_touch_mode(4);	//+++ ASUS BSP Bernard: touch for hans
			#endif
			
//ASUS_BSP --- : larry lai for pad solution

}

#ifdef CONFIG_ASUS_HDMI
//Mickey+++, wait for boot complete if we don't boot in pad
bool wait_for_android_boot_complete(void);
extern bool g_Android_Boot_Complete;
//Mickey---
extern int pad_insert;
#endif

#ifdef CONFIG_ON_SEMI_VIBRATOR
extern void MyDP_notify_vibrator_padInsert(void);
#endif

//ASUS BSP Bernard +++
#include <asm/uaccess.h>
#include <linux/fs.h>
#define DEV_VIBRATOR "/sys/class/timed_output/vibrator/enable"
 void mydp_set_vib_enable(int value)
{
    char timeout_ms[5];
    static mm_segment_t oldfs;
    struct file *fp = NULL;
    loff_t pos_lsts = 0;

    sprintf(timeout_ms, "%d", value);
    oldfs = get_fs();
    set_fs(KERNEL_DS);
    fp = filp_open( DEV_VIBRATOR, O_RDWR|O_CREAT|O_TRUNC, 0664 );
    if(IS_ERR_OR_NULL(fp)) {
        printk("ASDF: fail to open vibrator.");
        return;
    }
    if(fp->f_op != NULL && fp->f_op->write != NULL){
        pos_lsts = 0;
        fp->f_op->write(fp, timeout_ms, strlen(timeout_ms), &pos_lsts);
    } else {
        printk("ASDF: fail to write value.\n");
    }
    filp_close(fp, NULL);
    set_fs(oldfs);
    printk("ASDF: set vibrator enable. (%s ms)\n", timeout_ms);
}
//ASUS BSP Bernard ---

static void slimport_cable_plug_proc(struct anx7808_data *anx7808)
{
	int ret = 0;
///	printk("+++++++++++++slimport_cable_plug_proc+++++++++++++++++++\n");
#ifdef CONFIG_ASUS_HDMI
	bool needWait=false;

    //Mickey+++, wait for boot complete if we don't boot in pad
    if (!g_Pad_Bootup && !g_Android_Boot_Complete) {
        needWait=wait_for_android_boot_complete();
        ASUS_DEV_WARN("Didn't boot up from PAD and android boot completed\n");
	}
	//Mickey---
#endif

	if (gpio_get_value_cansleep(anx7808->pdata->gpio_cbl_det) ) {
		msleep(20);
		if (gpio_get_value_cansleep(anx7808->pdata->gpio_cbl_det)) {
			if (sp_tx_pd_mode) {

				sp_tx_hardware_poweron(anx7808_client);
				
			//	queue_delayed_work(anx7808->workqueue, &anx7808->vibratorWork, 0);
				
				sp_tx_pd_mode = 0;
//ANX : (ver:20130105) diff with ANX slimport driver, comment it ??? 			
				sp_tx_power_on(SP_TX_PWR_REG);
				sp_tx_power_on(SP_TX_PWR_TOTAL);
//ANX : (ver:0.2)
				sp_tx_pull_down_id(TRUE);

				hdmi_rx_initialization();
				sp_tx_initialization();
//ASUS_BSP +++ : larry lai for pad solution
				if (!sp_tx_asus_pad)
				{
					//+++ ASUS BSP Bernard
					ret = clk_prepare_enable(anx7808->mydp_diff_clk);
					if(ret !=0)
						ASUS_DEV_WARN("Clk_prepare_enable fail %d\n",ret);
					mydp_diffclk_flag = 1;
					//--- ASUS BSP Bernard
					sp_tx_vbus_poweron();
					msleep(200);
				}
				else
				{
#ifdef CONFIG_ASUS_HDMI
					if(pad_insert){
#ifdef CONFIG_ON_SEMI_VIBRATOR
						schedule_work(&anx7808->vibratorWork);
#endif
						pad_insert = 0;
					}
#endif
					msleep(20);					
				}
				
			//	hdmi_hdcp_recheck();//ASUS BSP wei +++
				
				getPadSKU();     //ASUS BSP wei +++

//ASUS_BSP --- : larry lai for pad solution				
				if (!sp_tx_get_cable_type()) {
					DEV_ERR("%s:AUX ERR\n", __func__);
					sp_tx_vbus_powerdown();
//ANX : (ver:0.2)					
					sp_tx_pull_down_id(FALSE);
					sp_tx_power_down(SP_TX_PWR_REG);
					sp_tx_power_down(SP_TX_PWR_TOTAL);
					sp_tx_hardware_powerdown(anx7808_client);
					sp_tx_pd_mode = 1;
					sp_tx_link_config_done = 0;
					sp_tx_hw_lt_enable = 0;
					sp_tx_hw_lt_done = 0;
					sp_tx_rx_type = RX_NULL;
					sp_tx_rx_type_backup = RX_NULL;
//ANX +++: (ver:20130105) pad solution
					sp_tx_asus_pad = 0;
//ANX ---: (ver:20130105) pad solution
					sp_tx_set_sys_state(STATE_CABLE_PLUG);
					return;
				}
				sp_tx_rx_type_backup = sp_tx_rx_type;
			}
			switch(sp_tx_rx_type) {
			case RX_HDMI:
				if(sp_tx_get_hdmi_connection()){
					ASUS_DEV_INFO("==== (RX_HDMI) ===\n");
//ANX +++: (ver:20130105) pad solution					
					if(sp_tx_asus_pad) {
						hdmi_rx_set_hpd(1);
						hdmi_rx_set_termination(1);
						sp_tx_set_sys_state(STATE_CONFIG_HDMI);
					} else {
						sp_tx_set_sys_state(STATE_PARSE_EDID);
					}
//ANX ---: (ver:20130105) pad solution					
					}
				break;
			case RX_DP:
				if(sp_tx_get_dp_connection())
				{
					ASUS_DEV_INFO("==== (RX_DP) ===\n");				
					if(sp_tx_asus_pad) {
						//skip EDID read
						hdmi_rx_set_hpd(1);
						hdmi_rx_set_termination(1);
						sp_tx_set_sys_state(STATE_CONFIG_HDMI);
					} else {				
						sp_tx_set_sys_state(STATE_PARSE_EDID);
					}
				}
				break;
			case RX_VGA:
				if(sp_tx_get_vga_connection()){
					sp_tx_send_message(MSG_CLEAR_IRQ); 
					sp_tx_set_sys_state(STATE_PARSE_EDID);
				}
				break;
			case RX_NULL:
			default:
				break;
			}
		}
	} else if (sp_tx_pd_mode == 0) {
		sp_tx_vbus_powerdown();
//ANX : (ver:0.2)		
		sp_tx_pull_down_id(FALSE);
		sp_tx_power_down(SP_TX_PWR_REG);
		sp_tx_power_down(SP_TX_PWR_TOTAL);
		sp_tx_hardware_powerdown(anx7808_client);
		sp_tx_pd_mode = 1;
		sp_tx_link_config_done = 0;
		sp_tx_hw_lt_enable = 0;
		sp_tx_hw_lt_done = 0;
		sp_tx_rx_type = RX_NULL;
		sp_tx_rx_type_backup = RX_NULL;
//ANX +++: (ver:20130105) pad solution		
		sp_tx_asus_pad = 0;
//ANX ---: (ver:20130105) pad solution		
		sp_tx_set_sys_state(STATE_CABLE_PLUG);
	}else{
#ifdef CONFIG_ON_SEMI_VIBRATOR
		int vib_count=10;
		if(needWait && gpio_get_value(75)){
			while(vib_count--&& gpio_get_value(75)){
                mydp_set_vib_enable(600);
				msleep(1500);
			}
		}
#endif
	}
}
#endif

#ifndef EYE_TEST
static void slimport_edid_proc(void)
{
	sp_tx_aux_polling_enable(0); //+++ ASUS BSP Bernard: ANX 4.0
	
	sp_tx_edid_read();

	if (bedid_break)
		DEV_ERR("%s: EDID corruption!\n", __func__);
	sp_tx_aux_polling_enable(1); //+++ ASUS BSP Bernard: ANX 4.0
	hdmi_rx_set_hpd(1);
	hdmi_rx_set_termination(1);
	sp_tx_set_sys_state(STATE_CONFIG_HDMI);
}
#endif

//ANX +++: (ver:20130105) EDID read issue with QCOM solution
int slimport_read_edid_block(int block, uint8_t *edid_buf)
{
	if (block == 0) {
		memcpy(edid_buf, bedid_firstblock, sizeof(bedid_firstblock));
	} else if (block == 1) {
		memcpy(edid_buf, bedid_extblock, sizeof(bedid_extblock));
	} else {
		pr_err("%s: block number %d is invalid\n", __func__, block);
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(slimport_read_edid_block);

unchar slimport_get_link_bw(void)
{
	return slimport_link_bw;
}
EXPORT_SYMBOL(slimport_get_link_bw);
//ANX ---: (ver:20130105) EDID read issue with QCOM solution

//+++ ASUS BSP Bernard, for test pattern
void sp_tx_test_pattern(void)
{
	sp_write_reg( TX_P2, 0x09, 0x10) ;//# Set 8bpp, RGB input
	sp_write_reg( TX_P2, 0x11, 0x00) ;//# Set BIST video sync pol (b1=VSync, b0=HSync; High=0,low=1),(see PH/PV in tables)

	if (g_myDP_type == 0) //720p
	{
		sp_write_reg( TX_P2, 0x12, 0xee) ;//# Set BIST video Vtotal (VTOT_L)
		sp_write_reg( TX_P2, 0x13, 0x02) ;//# (VTOT_H)
		sp_write_reg( TX_P2, 0x14, 0xd0) ;//# Set BIST video Vactive (Vact_L)
		sp_write_reg( TX_P2, 0x15, 0x02) ;//# (Vact_H)
		sp_write_reg( TX_P2, 0x16, 0x05) ;//# Set BIST video V Front Porch (VFP)
		sp_write_reg( TX_P2, 0x17, 0x05) ;//# Set BIST video V Sync Width (VSW)
		sp_write_reg( TX_P2, 0x18, 0x14) ;//# Set BIST video V Back Porch (VBP)
		sp_write_reg( TX_P2, 0x19, 0x72) ;//# Set BIST video Htotal (HTOT_L)
		sp_write_reg( TX_P2, 0x1a, 0x06) ;//# (HTOT_H)
		sp_write_reg( TX_P2, 0x1b, 0x00) ;//# Set BIST video Hactive (Hact_L)
		sp_write_reg( TX_P2, 0x1c, 0x05) ;//# (Hact_H)
		sp_write_reg( TX_P2, 0x1d, 0x6e) ;//# Set BIST video H Front Porch (HFP_L)
		sp_write_reg( TX_P2, 0x1e, 0x00) ;//# (HFP_H)
		sp_write_reg( TX_P2, 0x1f, 0x28) ;//# Set BIST video H Sync Width (HSW_L)
		sp_write_reg( TX_P2, 0x20, 0x00) ;//# (HSW_H)
		sp_write_reg( TX_P2, 0x21, 0xdc) ;//# Set BIST video H Back Porch (HBP_L)
		sp_write_reg( TX_P2, 0x22, 0x00) ;//# (HBP_H)
	}
	else if(g_myDP_type == 1) // 1080p60, tv
	{
		sp_write_reg( TX_P2, 0x12, 0x65) ;//# Set BIST video Vtotal (VTOT_L)
		sp_write_reg( TX_P2, 0x13, 0x04) ;//# (VTOT_H)
		sp_write_reg( TX_P2, 0x14, 0x38) ;//# Set BIST video Vactive (Vact_L)
		sp_write_reg( TX_P2, 0x15, 0x04) ;//# (Vact_H)
		sp_write_reg( TX_P2, 0x16, 0x04) ;//# Set BIST video V Front Porch (VFP)
		sp_write_reg( TX_P2, 0x17, 0x05) ;//# Set BIST video V Sync Width (VSW)
		sp_write_reg( TX_P2, 0x18, 0x24) ;//# Set BIST video V Back Porch (VBP)
		sp_write_reg( TX_P2, 0x19, 0x98) ;//# Set BIST video Htotal (HTOT_L)
		sp_write_reg( TX_P2, 0x1a, 0x08) ;//# (HTOT_H)
		sp_write_reg( TX_P2, 0x1b, 0x80) ;//# Set BIST video Hactive (Hact_L)
		sp_write_reg( TX_P2, 0x1c, 0x07) ;//# (Hact_H)
		sp_write_reg( TX_P2, 0x1d, 0x58) ;//# Set BIST video H Front Porch (HFP_L)
		sp_write_reg( TX_P2, 0x1e, 0x00) ;//# (HFP_H)
		sp_write_reg( TX_P2, 0x1f, 0x2c) ;//# Set BIST video H Sync Width (HSW_L)
		sp_write_reg( TX_P2, 0x20, 0x00) ;//# (HSW_H)
		sp_write_reg( TX_P2, 0x21, 0x94) ;//# Set BIST video H Back Porch (HBP_L)
		sp_write_reg( TX_P2, 0x22, 0x00) ;//# (HBP_H)
	}
	else if(g_myDP_type == 2) // 1200p, pad
	{
		sp_write_reg( TX_P2, 0x12, 0xd3) ;//# Set BIST video Vtotal (VTOT_L)
		sp_write_reg( TX_P2, 0x13, 0x04) ;//# (VTOT_H)
		sp_write_reg( TX_P2, 0x14, 0xb0) ;//# Set BIST video Vactive (Vact_L)
		sp_write_reg( TX_P2, 0x15, 0x04) ;//# (Vact_H)
		sp_write_reg( TX_P2, 0x16, 0x04) ;//# Set BIST video V Front Porch (VFP)
		sp_write_reg( TX_P2, 0x17, 0x06) ;//# Set BIST video V Sync Width (VSW)
		sp_write_reg( TX_P2, 0x18, 0x1a) ;//# Set BIST video V Back Porch (VBP)
		sp_write_reg( TX_P2, 0x19, 0xd0) ;//# Set BIST video Htotal (HTOT_L)
		sp_write_reg( TX_P2, 0x1a, 0x07) ;//# (HTOT_H)
		sp_write_reg( TX_P2, 0x1b, 0x80) ;//# Set BIST video Hactive (Hact_L)
		sp_write_reg( TX_P2, 0x1c, 0x07) ;//# (Hact_H)
		sp_write_reg( TX_P2, 0x1d, 0x08) ;//# Set BIST video H Front Porch (HFP_L)
		sp_write_reg( TX_P2, 0x1e, 0x00) ;//# (HFP_H)
		sp_write_reg( TX_P2, 0x1f, 0x10) ;//# Set BIST video H Sync Width (HSW_L)
		sp_write_reg( TX_P2, 0x20, 0x00) ;//# (HSW_H)
		sp_write_reg( TX_P2, 0x21, 0x38) ;//# Set BIST video H Back Porch (HBP_L)
		sp_write_reg( TX_P2, 0x22, 0x00) ;//# (HBP_H)
	}

	switch (g_pattern_type) 
	{
		case 0:
			DEV_DBG("myDP video BIST type=0\n");
			sp_write_reg( TX_P2, 0x0B, 0x08) ;//# Enable video BIST			
			break;
		case 1:
			DEV_DBG("myDP video BIST type=1\n");
			sp_write_reg( TX_P2, 0x0B, 0x09) ;//# Enable video BIST			
			break;
		case 2:
			DEV_DBG("myDP video BIST type=2\n");
			sp_write_reg( TX_P2, 0x0B, 0x0a) ;//# Enable video BIST			
			break;
		case 3:
			DEV_DBG("myDP video BIST type=3\n");
			sp_write_reg( TX_P2, 0x0B, 0x0b) ;//# Enable video BIST			
			break;
		default:
			DEV_DBG("myDP default video BIST type=0\n");
			sp_write_reg( TX_P2, 0x0B, 0x08) ;//# Enable video BIST			
			break;
	}	
}
//+++ ASUS BSP Bernard, for test pattern

#ifndef EYE_TEST
static void slimport_config_output(void)
{	
	unchar c2=0;
	sp_read_reg (RX_P0, HDMI_RX_INT_STATUS2_REG, &c2);
	//ASUS BSP wei +++
	DEV_DBG("%s:  c2=%x\n",__func__,c2);
	if ( c2 & HDCP_ERR)   {
		g_hdmi_hdcp_fail++;
		if(g_hdmi_hdcp_fail>10){
			g_hdmi_hdcp_fail=0;
			sp_tx_sw_error_power_down();
		}
		return;
	}
	//ASUS BSP wei ---
	sp_tx_clean_hdcp();
	sp_tx_set_colorspace();
	sp_tx_avi_setup();
	sp_tx_config_packets(AVI_PACKETS);
	
	//+++ ASUS BSP Bernard
	if (g_myDP_test_pattern)
		sp_tx_test_pattern();
	//--- ASUS BSP Bernard
	
	sp_tx_enable_video_input(1);
	sp_tx_set_sys_state(STATE_HDCP_AUTH);
}

extern uint sp_tx_link_err_check_1(void);
static void slimport_playback_proc(void)
{
	unchar c1 = 0;
	unchar c=0;
	unchar c2=0,cl=0,ch=0;

	if (g_dump_7808_reg)
	{
		dump_7808_reg_info();
		//dump_7808_reg_pclk();
	}

	if (g_dump_7730_reg)
	{
		sp_tx_link_err_check_1();
		sp_read_reg(TX_P2, SP_TX_TOTAL_PIXEL_STA_L, &c);
		sp_read_reg(TX_P2, SP_TX_TOTAL_PIXEL_STA_H, &c1);

		if(c1!=0x7||c!=0xd0)
		{
			DEV_DBG("ANX7808 TX cur_h_res = 0x%x,0x%x\n", c,c1);
		//	ASUSEvtlog("[mydp]ANX7808 TX cur_h_res = 0x%x,0x%x\n", c,c1);
		}
		sp_read_reg(TX_P2, SP_TX_TOTAL_LINE_STA_L, &c);
		sp_read_reg(TX_P2, SP_TX_TOTAL_LINE_STA_H, &c1);

		if(c1!=0x4||c!=0xd3)
		{
		 	DEV_DBG("ANX7808 TX cur_v_res = 0x%x\n,0x%x\n", c,c1);
		//	ASUSEvtlog("[mydp]ANX7808 TX cur_v_res = 0x%x\n,0x%x\n", c,c1);
		}
		sp_read_reg(RX_P0, HDMI_RX_HTOTAL_LOW, &cl);
		sp_read_reg(RX_P0, HDMI_RX_HTOTAL_HIGH, &ch);
		if(ch!=0x7||cl!=0xd0)
		{
			DEV_DBG("ANX7808 RX cur_h_res = 0x%x 0x%x\n",ch,cl);
		//	ASUSEvtlog("[mydp]ANX7808 RX cur_h_res = 0x%x 0x%x\n", ch,cl);
		}
		sp_read_reg(RX_P0, HDMI_RX_VTOTAL_LOW, &cl);
		sp_read_reg(RX_P0, HDMI_RX_VTOTAL_HIGH, &ch);
		if(ch!=0x4||cl!=0xd3){
			DEV_DBG("ANX7808 RX cur_v_res = 0x%x 0x%x\n", ch,cl);
		//	ASUSEvtlog("[mydp]ANX7808 RX cur_v_res = 0x%x 0x%x\n", ch,cl);
		}
        if (!myDP_DP_Dongle)
		{
//========================================	
// disable 7730 OCM

		i2c_master_write_reg(7,0xf0, 0);
		i2c_master_read_reg(7,0xf0, &c);

		if(c != 0 )
		    i2c_master_write_reg(7,0xf0, 0);
//========================================

		i2c_master_read_reg(0, 0xcb, &c1);															
		i2c_master_read_reg(1, 0xd1, &c);					
		//i2c_master_read_reg(0, 0x31, &c2);				
		
		i2c_master_write_reg(0, 0xcb, c1);					
		i2c_master_write_reg(1, 0xd1, c);					
	
		if ((c1 & 0x08) || (c & 0x80))
		{
		    if (g_hdmi_rx_vsync_change < 0xFF)
			    g_hdmi_rx_vsync_change++;
				
			DEV_DBG("# 7730 video FIFO error , 0xcb= (%x), RX ISR6 = (%x)\n",  c1, c);
		//	ASUSEvtlog("[myDP]7730 video FIFO error , 0xcb= (%x), RX ISR6 = (%x)\n", c1, c);
		}
		
//		printk("========== ANX7730 reg 0x50 DUMP ============\n");
/*
		for (i=0x20; i <=0x30; i++)
		{
			i2c_master_read_reg(0, i , &c);
			if  (i%0xf)
				printk("0x%x = (%x), ", i, c);										
			else	
				printk("0x%x = (%x)\n", i, c);										
						
		}
*/
		i2c_master_read_reg(0, 0x24 , &c1);
		i2c_master_read_reg(0, 0x25 , &c2);

		if ((c1 != 0x7) || (c2 != 0xd0))
		{
			DEV_DBG("Fail 7730 0x50 H total = (%x, %x)", c1, c2);
		//	ASUSEvtlog("[myDP]Fail 7730 0x50 H total = (%x, %x)", c1, c2);			
		}
						
//		printk("========== ANX7730 reg 0x72 DUMP ============\n");
/*		
		for (i=0x24; i <=0x34; i++)
		{
			i2c_master_read_reg(5, i , &c);
			if  (i%0xf)
				printk("0x%x = (%x), ", i, c);										
			else	
				printk("0x%x = (%x)\n", i, c);										
		}
*/		
		i2c_master_read_reg(5, 0x2b , &c1);
		i2c_master_read_reg(5, 0x2c , &c2);

		if ((c1 != 0xd0) || (c2 != 0x7))
		{
			DEV_DBG("Fail 7730 0x72 H total = (%x, %x)", c1, c2);	
			//ASUSEvtlog("[myDP]Fail 7730 0x72 H total = (%x, %x)", c1, c2);									
		}
//		
//========================================
// re-enable 7730 OCM
		i2c_master_write_reg(7,0xf0, 0xe6);
		i2c_master_write_reg(7,0xf0, 0xe6);
//========================================		
		}
	}
}
#endif
//extern int check_mdp4_dtv(void);

#ifndef EYE_TEST
//int config_hdmi_try_count=2;

static void slimport_main_proc(struct anx7808_data *anx7808)
{
	mutex_lock(&anx7808->lock);

	if (!sp_tx_pd_mode) {
		sp_tx_int_irq_handler();
		hdmi_rx_int_irq_handler();
	}

	if (sp_tx_system_state == STATE_CABLE_PLUG)
		slimport_cable_plug_proc(anx7808);

	if (sp_tx_system_state == STATE_PARSE_EDID)
		slimport_edid_proc();
//ANX +++: (ver0.4)
	if (sp_tx_system_state == STATE_CONFIG_HDMI) {
 		if(sp_tx_asus_pad) 
		{
			int try_count= 20;
			hdmi_asus_rx_initialization(); //ASUS BSP wei +++
			while (try_count) {
				if (!sp_tx_config_hdmi_input())
					break;
				else	
					msleep(50);
				try_count --;
			};

			ASUS_DEV_WARN("try_count = %d\n", try_count);
			#if 1
			if ( !try_count)
			{
				ASUS_DEV_INFO("after check RX input, still TMDS not ready, go link , then power down\n");
				sp_tx_set_sys_state(STATE_LINK_TRAINING);
				//config_hdmi_try_count--;
			}
			else
			{	
				//config_hdmi_try_count=2;
				ASUS_DEV_INFO("TMDS ready, go to next process\n");	
			}
			#endif
			if (sp_tx_system_state == STATE_LINK_TRAINING){
				sp_tx_sw_error_power_down();
#if 0
				if(config_hdmi_try_count==0){
					sp_tx_sw_error_power_down();
					config_hdmi_try_count=2;
				}else{
					hdmi_asus_rx_initialization(); //ASUS BSP wei +++
					sp_tx_set_sys_state(STATE_CONFIG_HDMI);
				}
#endif
									
			}else	{
				int count=5;
				while(sp_tx_config_hdmi_pad()){
					if(count==0)
						break;
					count--;
				}
			}
		}
		else 
			sp_tx_config_hdmi_input();
	}
	if (sp_tx_system_state == STATE_LINK_TRAINING) {
		if(!sp_tx_asus_pad) {
			if (!sp_tx_lt_pre_config())
				sp_tx_hw_link_training();
		}
		else
		{// pad mode not do link training, so power down chip
				//ASUS BSP wei +++
			sp_tx_config_hdmi_pad();
			//sp_tx_set_sys_state(STATE_CONFIG_OUTPUT);
				//ASUS BSP wei ---
			//sp_tx_sw_error_power_down();			
		}
	}
//ANX ---: (ver0.4)
	if (sp_tx_system_state == STATE_CONFIG_OUTPUT)
		slimport_config_output();

	if (sp_tx_system_state == STATE_HDCP_AUTH) {
		if (hdcp_enable && 
			((sp_tx_rx_type == RX_HDMI) ||
			( sp_tx_rx_type ==RX_DP) || ( sp_tx_rx_type ==RX_VGA)) ) {
//ANX +++: (ver:20130105) pad solution			
			if(sp_tx_asus_pad)
				sp_tx_sw_hdcp_process();
			else
				sp_tx_hdcp_process();
//ANX ---: (ver:20130105) pad solution
		} else {
			sp_tx_power_down(SP_TX_PWR_HDCP);
			sp_tx_video_mute(0);
			sp_tx_show_infomation();
			sp_tx_set_sys_state(STATE_PLAY_BACK);
		}
	}

	if (sp_tx_system_state == STATE_PLAY_BACK)
		slimport_playback_proc();

	mutex_unlock(&anx7808->lock);
}
#endif

static uint8_t anx7808_chip_detect(void)
{
	return sp_tx_chip_located();
}

#ifdef EYE_TEST
extern void sp_tx_eye_diagram_test(void);
#endif
static void anx7808_chip_initial(void)
{
#ifdef EYE_TEST
	sp_tx_eye_diagram_test();
#else
	sp_tx_variable_init();
	sp_tx_vbus_powerdown();
	sp_tx_hardware_powerdown(anx7808_client);
	sp_tx_set_sys_state(STATE_CABLE_PLUG);
#endif
}

static void anx7808_free_gpio(struct anx7808_data *anx7808)
{
	gpio_free(anx7808->pdata->gpio_cbl_det);
	gpio_free(anx7808->pdata->gpio_rst);
	gpio_free(anx7808->pdata->gpio_pw_dwn);
	gpio_free(anx7808->pdata->gpio_usb_id);
}
static int anx7808_init_gpio(struct anx7808_data *anx7808)
{
	int ret = 0;

	DEV_DBG("@@@@ anx7808 init gpio @@@@\n");
//+++ CHIP POWER DOWN +++	
	ret = gpio_request(anx7808->pdata->gpio_pw_dwn, "anx_pw_dwn_ctl");
	if (ret) {
		DEV_ERR("%s : failed to request gpio %d\n", __func__,
				anx7808->pdata->gpio_pw_dwn);
		goto err0;
	}
      //+++ ASUS BSP Bernard: for SR, inverter is disabled
	 if (g_ASUS_hwID <= A90_EVB)
	 	gpio_direction_output(anx7808->pdata->gpio_pw_dwn, 0);
	else if (g_ASUS_hwID >= A91_SR1)
		gpio_direction_output(anx7808->pdata->gpio_pw_dwn, 1);
	//--- ASUS BSP Bernard: for SR, inverter is disabled
//--- CHIP POWER DOWN ---

//+++ MyDP RESET +++
	ret = gpio_request(anx7808->pdata->gpio_rst, "anx7808_reset_n");
	if (ret) {
		DEV_ERR("%s : failed to request gpio %d\n", __func__,
				anx7808->pdata->gpio_rst);
		goto err1;
	}
	gpio_direction_output(anx7808->pdata->gpio_rst, 0);
//--- MyDP RESET ---

//+++ CABLE DETECT +++
	ret = gpio_request(anx7808->pdata->gpio_cbl_det, "anx7808_cbl_det");
	if (ret) {
		DEV_ERR("%s : failed to request gpio %d\n", __func__,
				anx7808->pdata->gpio_cbl_det);
		goto err2;
	}
	gpio_direction_input(anx7808->pdata->gpio_cbl_det);
//--- CABLE DETECT ---

//+++ USB ID +++
//+++ ASUS BSP Bernard: usb ID & usb select
	ret = gpio_request(anx7808->pdata->gpio_usb_id, "anx7808_usb_hs_id");
	if (ret) {
		DEV_ERR("%s : failed to request gpio %d\n", __func__,
				anx7808->pdata->gpio_usb_id);
		goto err3;
	}
	gpio_direction_input(anx7808->pdata->gpio_usb_id); 
//--- USB ID ---

	gpio_set_value(anx7808->pdata->gpio_rst, 0);
	//+++ ASUS BSP Bernard: for SR, inverter is disabled
	 if (g_ASUS_hwID <= A90_EVB)
		gpio_set_value(anx7808->pdata->gpio_pw_dwn, 0);
	 else if (g_ASUS_hwID >= A91_SR1)
	 	gpio_set_value(anx7808->pdata->gpio_pw_dwn, 1);
	 //--- ASUS BSP Bernard: for SR, inverter is disabled
	
	goto out;

err3:
	gpio_free(anx7808->pdata->gpio_usb_id);
err2:
	gpio_free(anx7808->pdata->gpio_cbl_det);
err1:
	gpio_free(anx7808->pdata->gpio_rst);
err0:
	gpio_free(anx7808->pdata->gpio_pw_dwn);
out:
	return ret;
}

static int  anx7808_system_init(void)
{
	int ret = 0;

	ret = anx7808_chip_detect();
	if (ret == 0) {
		DEV_ERR("%s : failed to detect anx7808\n", __func__);
		return -ENODEV;
	}

	anx7808_chip_initial();
	return 0;
}

static irqreturn_t anx7808_cbl_det_isr(int irq, void *data)
{
	struct anx7808_data *anx7808 = (struct anx7808_data *)data;
	//ANX +++: (ver0.4)	
	int status;
	//ANX ---: (ver0.4)	
	
	if (gpio_get_value(anx7808->pdata->gpio_cbl_det) && (g_i2c_error_count<=20))  {
       		wake_lock(&anx7808->slimport_lock);
		cableInserted = 1;
		DEV_NOTICE("%s : detect cable insertion\n", __func__);
			queue_delayed_work(anx7808->workqueue, &anx7808->work, 0);

	
	} else {
		DEV_NOTICE("%s : detect cable removal\n", __func__);
		cableInserted = 0;
//ASUS_BSP: joe1_++: clear edid while hdmi plug out
		memset( bedid_firstblock, 0, sizeof(bedid_firstblock) );
		memset( bedid_extblock, 0, sizeof(bedid_extblock) );

		DEV_DBG("%s: clear bedid\n", __func__);
//ASUS_BSP: joe1_--: clear edid while hdmi plug out
//+++ ASUS BSP Bernard
		if(mydp_diffclk_flag == 1){
			clk_disable_unprepare(anx7808->mydp_diff_clk);
			mydp_diffclk_flag = 0;
		}
//--- ASUS BSP Bernard
//ANX +++: (ver0.4)		
		status = cancel_delayed_work_sync(&anx7808->work);
		if(status == 0)
			flush_workqueue(anx7808 ->workqueue);
		wake_unlock(&anx7808->slimport_lock);

#ifdef ASUS_A86_PROJECT
	if (g_ASUS_hwID != A86_SR3)
		nv_touch_mode(0);	//+++ ASUS BSP Bernard: touch for hans, 20130624
#endif

		wake_lock_timeout(&anx7808->slimport_lock, msecs_to_jiffies(5000));
//ANX ---: (ver0.4)		
	}
	return IRQ_HANDLED;
}

static void anx7808_work_func(struct work_struct *work)
{
#ifndef EYE_TEST
	struct anx7808_data *td = container_of(work, struct anx7808_data,
								work.work);

	slimport_main_proc(td);
	queue_delayed_work(td->workqueue, &td->work,
			msecs_to_jiffies(300));
#endif
}

//+++ ASUS BSP Bernard
static irqreturn_t dp_usb_id_detect_handler(int irq, void *dev_id){
	
	struct anx7808_platform_data *pdata = anx7808_client->dev.platform_data;
	struct anx7808_data *anx7808 = (struct anx7808_data *)dev_id;
	ASUS_DEV_WARN("[dp_usb_id_detect_handler]+++++++++++++\n");
	ASUS_DEV_INFO("@@@@ usb_id = %d @@@@\n", pdata->gpio_usb_id);
	if(gpio_get_value(pdata->gpio_cbl_det)==1)goto exit;   //asus wei lai +++
	
	if((gpio_get_value(pdata->gpio_usb_id)==1) && (g_carkit_flag == 1)){
		ASUS_DEV_WARN("dp_switch_carkit is false\n");
		dp_switch_carkit(false);
		g_carkit_flag = 0;
	}
	else
		queue_delayed_work(anx7808->workqueue, &anx7808->carKitwork, 50);

exit:
	return IRQ_HANDLED;
}

#ifdef CONFIG_ON_SEMI_VIBRATOR
static void  anx7808_vibrator_func(struct work_struct *work){
//	struct anx7808_data *td = container_of(work, struct anx7808_data,
//		 						vibratorWork.work);
//	if(pad_insert){
				MyDP_notify_vibrator_padInsert();
				//pad_insert=0;
	//}

}
#endif

static void anx7808_carKitwork_func(struct work_struct *work)
{
#ifndef EYE_TEST
	
	struct anx7808_data *td = container_of(work, struct anx7808_data,
		 						carKitwork.work);
	if (gpio_get_value_cansleep(td ->pdata->gpio_usb_id) ==0 ){
		msleep(50);
		if((gpio_get_value_cansleep(td ->pdata->gpio_cbl_det) ==0) && (g_carkit_flag == 0)){
			dp_switch_carkit(true);
			g_carkit_flag = 1;
			ASUS_DEV_WARN("dp_switch_carkit is true+++++\n");
		}
		else {
			g_b_SwitchCarkitBootPlugin = false;	//+++ ASUS BSP Bernard
		}
	}
	
#endif
}
//--- ASUS BSP Bernard

//+++ ASUS BSP Bernard: for sysfs, ATD test
static ssize_t anx7808_id_show(struct device *dev, struct device_attribute* attr, char* buf)
{
	uint ID = 0;
	struct anx7808_platform_data *pdata = dev->platform_data;
	ID = pdata->ID;
	return snprintf(buf, PAGE_SIZE, "%d\n", ID);
}
static DEVICE_ATTR(ChipID, S_IRWXU | S_IRWXG| S_IROTH, anx7808_id_show, NULL);

static struct attribute *anx7808_fs_attrs[] = {
	&dev_attr_ChipID.attr,
	NULL,
};
static struct attribute_group anx7808_fs_attrs_group = {
	.attrs = anx7808_fs_attrs,
};
//--- ASUS BSP Bernard: for sysfs, ATD test

//ASUS BSP Wei +++
static int anx7808_get_hdmi_dt_data(struct device_node *intr,struct anx7808_data * anx7808){
	struct device_node *hdmi_tx_node = NULL;
	struct platform_device *hdmi_pdev = NULL;
	hdmi_tx_node = of_parse_phandle(intr, "slimport,hdmi-tx-map", 0);
	if (!hdmi_tx_node) {
		pr_err("%s: can't find hdmi phandle\n", __func__);
		goto error;
	}

	hdmi_pdev = of_find_device_by_node(hdmi_tx_node);
	if (!hdmi_pdev) {
		pr_err("%s: can't find the device by node\n", __func__);
		goto error;
	}
	pr_debug("%s: hdmi_pdev [0X%x] to pdata->pdev\n",
	       __func__, (unsigned int)hdmi_pdev);

	anx7808->hdmi_pdev = hdmi_pdev;
	return 0;
error:
	anx7808->hdmi_pdev = NULL;
	return -1;
}
//ASUS BSP Wei ---

int gMyDPCTSconfig = 0;



static ssize_t MyDPCTSconfig_proc_write(struct file *file, const char __user *buf,
	size_t count, loff_t *ppos)
{
       char num[10];
       memset(num, 0, sizeof(num));

       /* no data be written */
       if (!count) {
	       return 0;
	}

       /* Input size is too large to write our buffer(num) */
       if (count > (sizeof(num) - 1)) {
       return -EINVAL;
       }

       if (copy_from_user(num, buf, count)) {
       return -EFAULT;
	}

       if (strncmp(num, "0", 1) == 0) {
       		gMyDPCTSconfig = 0;
       } else if (strncmp(num, "1", 1) == 0) {
		gMyDPCTSconfig = 1;
	} else {
		printk("gMyDPCTSconfig unknown data!!\n");
	}

       return count;
}
static const struct file_operations proc_gMyDPCTSconfig_operations = {

       .write = MyDPCTSconfig_proc_write,
};
static int anx7808_i2c_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device_node *intr = client->dev.of_node;//+++ ASUS BSP Bernard add for interrupt
	
	struct anx7808_data *anx7808;
	int ret = 0;
	int irq=0;	 //+++ ASUS BSP Bernard
	struct kobject *kobj; //+++ ASUS BSP Bernard add for sysfs
	
	DEV_DBG("+++ anx780x_i2c_probe +++\n");
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_I2C_BLOCK)) {
		DEV_ERR("%s: i2c bus does not support the anx7808\n", __func__);
		ret = -ENODEV;
		goto exit;
	}

	anx7808 = kzalloc(sizeof(struct anx7808_data), GFP_KERNEL);
	if (!anx7808) {
		DEV_ERR("%s: failed to allocate driver data\n", __func__);
		ret = -ENOMEM;
		goto exit;
	}

	anx7808->mydp_diff_clk = clk_get_sys("slimport","ref_clk"); //+++ ASUS BSP Bernard
	if(!anx7808->mydp_diff_clk){
		ret = -EINVAL;
		goto exit;
	}
	
//+++ ASUS BSP Bernard for device tree support
	anx7808->pdata = kzalloc(sizeof(struct anx7808_platform_data), GFP_KERNEL);
	if(!anx7808->pdata){
		ret = -EINVAL;
		goto err0;
	}
	//to get gpio information
	if(client->dev.of_node){
		anx7808->pdata ->gpio_rst = of_get_named_gpio_flags(client->dev.of_node, 
				"slimport,gpio_rst",0, &anx7808->pdata->gpio_rst_flags);
		anx7808->pdata ->gpio_cbl_det = of_get_named_gpio_flags(client->dev.of_node, 
				"slimport,gpio_cbl_det",0, &anx7808->pdata->gpio_cbl_det_flags);
		anx7808->pdata ->gpio_pw_dwn = of_get_named_gpio_flags(client->dev.of_node, 
				"slimport,gpio_pw_dwn",0, &anx7808->pdata->gpio_pw_dwn_flags);
		anx7808->pdata ->gpio_usb_id= of_get_named_gpio_flags(client->dev.of_node, 
				"slimport,gpio_usb_id",0, &anx7808->pdata->gpio_usb_id_flags);
		ASUS_DEV_INFO("### get GPIO information ###");
	}
	ASUS_DEV_INFO("gpio_rst[%d], gpio_cbl_det[%d], gpio_pw_dwn[%d], Slave address[%02x]\n",
			anx7808->pdata->gpio_rst,
			anx7808->pdata->gpio_cbl_det,
			anx7808->pdata->gpio_pw_dwn,
			client->addr);		
	ASUS_DEV_INFO("gpio_usb_id[%d]\n", anx7808->pdata ->gpio_usb_id);

	//to get regulator 1v  
	anx7808->pdata->sp_regulator = regulator_get(&client->dev, "vcc_1v");
	if (IS_ERR( anx7808->pdata->sp_regulator)) {
		ret = PTR_ERR(anx7808->pdata->sp_regulator);
		dev_err(&client->dev, "Regulator get 1v failed rc=%d\n", ret);
	}

	memcpy(&anx7808_client, &client, sizeof(client));
	memcpy(&anx7808_client->dev.platform_data, &anx7808->pdata, sizeof(anx7808->pdata));
//--- ASUS BSP Bernard for device tree support

	mutex_init(&anx7808->lock);

//+++ ASUS BSP Bernard: test i2c sysfs, ATD test
	kobj = (struct kobject*) &client->dev.kobj;
	if (!anx7808->pdata) {
		ret = -EINVAL;
		goto err0;
	}
	 ret=sysfs_create_group(kobj,&anx7808_fs_attrs_group);
	 if(ret){
	 	DEV_ERR("%s: create sysfs fail\n", __func__);
	 }
	 else{
	 	if(kobj->name)
	 		DEV_DBG("kobj->name=%s", kobj->name);
	 }
//--- ASUS BSP Bernard: test i2c sysfs, ATD test

	ret = anx7808_init_gpio(anx7808);
	if (ret) {
		DEV_ERR("%s: failed to initialize gpio\n", __func__);
		goto err1;
	}


	INIT_DELAYED_WORK(&anx7808->work, anx7808_work_func);
//+++ ASUS BSP Bernard
	INIT_DELAYED_WORK(&anx7808->carKitwork, anx7808_carKitwork_func);
//---  ASUS BSP Bernard

#ifdef CONFIG_ON_SEMI_VIBRATOR
	INIT_WORK(&anx7808->vibratorWork, anx7808_vibrator_func);
#endif

	anx7808->workqueue = create_singlethread_workqueue("anx7808_work");
	if (anx7808->workqueue == NULL) {
		DEV_ERR("%s: failed to create work queue\n", __func__);
		ret = -ENOMEM;
		goto err2;
	}
//ASUS BSP wei lai +++
#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend( &dp7808_early_suspend_desc );
#endif
	create_MYDP_proc_file();
	proc_create("MyDPCTSconfig",  S_IWUSR, NULL,&proc_gMyDPCTSconfig_operations);
//ASUS BSP wei lai ---
	ret = anx7808_system_init();
	if (ret) {
		DEV_ERR("%s: failed to initialize anx7808\n", __func__);
		goto err2;
	}
	//+++ ASUS BSP Bernard: request irq, interrupt by dts
	client->irq = irq_of_parse_and_map(intr, 1);
	//--- ASUS BSP Bernard: request irq, interrupt by dts
	
	if (client->irq < 0) {
		DEV_ERR("%s : failed to get gpio irq\n", __func__);
		goto err3;
	}

	ret = request_threaded_irq(client->irq, NULL, anx7808_cbl_det_isr,
					IRQF_TRIGGER_RISING
					| IRQF_TRIGGER_FALLING,
					"anx7808_cabel_det", anx7808);
	ASUS_DEV_WARN("@@@@@@@@@ irq_number = %d @@@@@@@@@@\n",client->irq); //+++ ASUS BSP Bernard, 20130426
	if (ret  < 0) {
		DEV_ERR("%s : failed to request irq\n", __func__);
		goto err3;
	}
//+++ ASUS BSP Bernard: to change to new function name
	//ret = set_irq_wake(client->irq, 1);
	ret = irq_set_irq_wake(client->irq, 1); //+++ ASUS BSP Bernard, 20130426
//--- ASUS BSP Bernard: to change to new function name
	if (ret  < 0) {
		pr_err("%s : Request irq for cable detect"
			"interrupt wake set fail\n", __func__);
		goto err3;
	}

//+++ ASUS BSP Bernard
	irq = irq_of_parse_and_map(intr, 0);
	if (irq < 0) {
		ASUS_DEV_WARN( "%s: could not get USB_ID_DETECT IRQ resource, error=%d ", __func__, irq);		
	}
	ret = request_irq(irq, dp_usb_id_detect_handler,
		IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING , "dp usb id mode", anx7808);

	if (ret < 0) {
		ASUS_DEV_WARN( "%s: FACTORY USB IRQ#%d request failed with error=%d \n", __func__, irq, ret);				
	}
	
//---  ASUS BSP Bernard
	wake_lock_init(&anx7808->slimport_lock, WAKE_LOCK_SUSPEND, "slimport_wake_lock");


	if(-1==anx7808_get_hdmi_dt_data(intr,anx7808))
		DEV_ERR("MyDP get HDMI device tree error\n");

	g_anx7808=anx7808;  //ASUS BSP wei +++		
	//+++ ASUS BSP Bernard: for OTG
	if( (gpio_get_value(anx7808->pdata->gpio_usb_id) ==0)&& (g_i2c_error_count==0) 
		&& !(gpio_get_value(75)))  {
		if(gpio_get_value(anx7808->pdata->gpio_cbl_det)==0){
			//wake_lock(&anx7808->slimport_lock);
			ASUS_DEV_WARN("@@@@@ OTG @@@@@@@@@@@@\n");
			//g_b_SwitchCarkitBootPlugin= true;
			queue_delayed_work(anx7808->workqueue, &anx7808->carKitwork, 0);
		}
	}

	//--- ASUS BSP Bernard: for OTG
	//ASUS Wei+++
	if (gpio_get_value(anx7808->pdata->gpio_cbl_det) && (g_i2c_error_count==0))  {
		msleep(10);
		if(gpio_get_value(anx7808->pdata->gpio_cbl_det)==1){
			wake_lock(&anx7808->slimport_lock);
			DEV_NOTICE("%s : detect cable insertion\n", __func__);
			queue_delayed_work(anx7808->workqueue, &anx7808->work, 1000);
		}
	}
	//ASUS wei ---
//ASUS BSP wei lai +++
#ifdef CONFIG_I2C_STRESS_TEST

	i2c_add_test_case(client, "MyDP7808",ARRAY_AND_SIZE(gMyDPTestCaseInfo));
	i2c_add_test_case(client, "MyDP7808-hdmi",ARRAY_AND_SIZE(gMyDPTestCaseInfo1));
	
#endif
//ASUS BSP wei lai ---
	goto exit;

err3:
	free_irq(client->irq, anx7808);
err2:
	destroy_workqueue(anx7808->workqueue);
err1:
	anx7808_free_gpio(anx7808);
err0:
	kfree(anx7808);
exit:
	DEV_DBG("--- anx780x_i2c_probe ---\n");
	return ret;
}

//+++ ASUS BSP Bernard Ong for i2c device tree
#ifdef CONFIG_OF
static struct of_device_id anx7808_match_table[]={
{.compatible="analogix,slimport",},
{},
};
#else
#define amx7808_match_table NULL
#endif

static const struct i2c_device_id anx7808_id[] = {
	{ "anx7808", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, anx7808_id);

static struct i2c_driver anx7808_driver = {
	.driver  = {
		.name  = "anx7808",
		.owner  = THIS_MODULE,
	//+++ ASUS BSP Bernard for match driver name
		.of_match_table = anx7808_match_table,
	//--- ASUS BSP Bernard for match driver name
	},

	.probe  = anx7808_i2c_probe,
	.id_table  = anx7808_id,
#if 1	
      .suspend = anx7808_suspend,
      .resume = anx7808_resume,	
#endif	  
};
module_i2c_driver(anx7808_driver);
//--- ASUS BSP Bernard Ong for i2c device tree

MODULE_DESCRIPTION("Slimport  transmitter ANX7808 driver");
MODULE_AUTHOR("FeiWang <fwang@analogixsemi.com>");
MODULE_LICENSE("GPL");

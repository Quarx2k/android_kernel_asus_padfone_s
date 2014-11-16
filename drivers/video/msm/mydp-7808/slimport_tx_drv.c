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
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include "slimport_tx_drv.h"
#include "slimport_tx_reg.h"
#include "slimport.h"

#include <linux/i2c.h> //+++ ASUS BSP Bernard add
#include <linux/gpio.h> //+++ ASUS BSP Bernard add

// ASUS BSP Wei +++
extern int g_enable_dynamic_ssc;
extern int spreading_ctrl1;
extern int spreading_ctrl2;
extern int spreading_ctrl3;

//+++ ASUS BSP Bernard
extern_asus_debug_mask(MYDP);

#ifndef ASUS_DEV_WARN
#define ASUS_DEV_WARN(args...) asus_printk(MYDP, ASUS_DEBUG_WARNING, "[MYDP] " args)
#endif

#ifndef ASUS_DEV_INFO
#define ASUS_DEV_INFO(args... ) asus_printk(MYDP, ASUS_DEBUG_INFO1, "[MYDP] " args)
#endif
//--- ASUS BSP Bernard


void mydp_dymSSC(const char *msg, int index){
	sscanf(msg,"%*s 0x%x 0x%x 0x%x",&spreading_ctrl1,&spreading_ctrl2,&spreading_ctrl3);
	DEV_DBG("#### %s : spreading_ctrl1= %d , spreading_ctrl2 = %d, spreading_ctrl3= %d ###\n", __FUNCTION__,spreading_ctrl1,spreading_ctrl2,spreading_ctrl3);
}


//ASUS BSP Wei ---

extern ktime_t wakeup_starttime;
extern bool measure_pad_wakeup_time;
static int pad_resume_time_mask = 0;
module_param_named(debug_mask, pad_resume_time_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

static unchar bytebuf[MAX_BUF_CNT];

/* EDID access break */
unchar bedid_break;
static unchar bedid_checksum;
unchar bedid_extblock[128] = {0};
unchar bedid_firstblock[128] = {0};


static ulong pclk;
static ulong m_val, n_val;
enum SP_LINK_BW sp_tx_bw;
bool sp_tx_link_config_done;//link config done flag
bool sp_tx_hw_lt_done;//hardware linktraining done indicator
bool sp_tx_hw_lt_enable;//hardware linktraining enable
bool sp_tx_test_lt;
static unchar sp_tx_test_bw;
static bool sp_tx_test_edid;
static unchar sp_tx_ds_vid_stb_cntr;
unchar slimport_link_bw;

/* for HDCP */
static unchar sp_tx_hdcp_auth_pass;
static unchar sp_tx_hdcp_auth_fail_counter;
static unchar sp_tx_hdcp_capable_chk;
static unchar sp_tx_hw_hdcp_en;
static unchar sp_tx_hdcp_auth_done;

enum RX_CBL_TYPE sp_tx_rx_type;
enum RX_CBL_TYPE  sp_tx_rx_type_backup;
unchar sp_tx_pd_mode;
//ANX +++: (ver:20130105)
bool sp_tx_asus_pad;
bool sp_tx_ksv_srm_pass;
static unchar AN[8] ;
static unchar AKSV[5];
static unchar BKSV_M[5];
static unchar BKSV[5];
//ANX +++: (ver:20130105)

//+++ ASUS BSP Bernard, for determining AC charger 
static bool g_b_SwitchChargerBootPlugin = false;
int mydp_ac_charger_flag = 0; 
//--- ASUS BSP Bernard, for determining AC charger

static struct AudiInfoframe sp_tx_audioinfoframe;
static struct Packet_AVI sp_tx_packet_avi;
static struct Packet_SPD sp_tx_packet_spd;
static struct Packet_MPEG sp_tx_packet_mpeg;
enum SP_TX_System_State sp_tx_system_state;

/* ***************************************************************** */

/* GLOBAL VARIABLES DEFINITION FOR HDMI START */

/* ***************************************************************** */

static unchar g_hdmi_dvi_status;
static unchar g_cur_pix_clk;
static unchar g_video_stable_cntr;
static unchar g_audio_stable_cntr;
static unchar g_sync_expire_cntr;
static unchar g_hdcp_err_cnt;

static ulong g_cur_h_res;
static ulong g_cur_v_res;
static unchar g_video_muted;
static unchar g_audio_muted;
static unchar g_cts_got;
static unchar g_audio_got;
extern int trigger_sw_reset;   //ASUS BSP wei +++
extern struct anx7808_data *g_anx7808;//ASUS BSP wei +++
static enum HDMI_RX_System_State hdmi_system_state;

/* ***************************************************************** */

/* GLOBAL VARIABLES DEFINITION FOR HDMI END */

/* ***************************************************************** */

void sp_tx_variable_init(void)
{

	sp_tx_hdcp_auth_fail_counter = 0;
	sp_tx_hdcp_auth_pass = 0;
	sp_tx_hw_hdcp_en = 0;
	sp_tx_hdcp_capable_chk = 0;
	sp_tx_hdcp_auth_done = 0;
	sp_tx_pd_mode = 1;
	sp_tx_rx_type = RX_NULL;
	sp_tx_rx_type_backup = RX_NULL;
	sp_tx_hw_lt_done = 0;
	sp_tx_hw_lt_enable = 0;
	sp_tx_link_config_done = 0;
	sp_tx_ds_vid_stb_cntr = 0;

	bedid_break = 0;
	bedid_checksum = 0;
	sp_tx_test_edid = 0;
	sp_tx_test_bw = 0;
	sp_tx_test_lt = 0;
	sp_tx_bw = BW_54G;
//ANX : (ver:20130105) pad solution	
	sp_tx_asus_pad = 0;
	slimport_link_bw = 0;
//ANX : (ver:20130105) pad solution	
	sp_tx_ksv_srm_pass = 0;
}

static void sp_tx_api_m_gen_clk_select(unchar bspreading)
{
	unchar c;

	sp_read_reg(TX_P0, SP_TX_M_CALCU_CTRL, &c);
	if (bspreading) {
		c |= M_GEN_CLK_SEL;
		sp_write_reg(TX_P0, SP_TX_M_CALCU_CTRL, c);
	} else {
		c &= ~M_GEN_CLK_SEL;
		sp_write_reg(TX_P0, SP_TX_M_CALCU_CTRL, c);
	}
}

static void sp_tx_link_phy_initialization(void)
{
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG0, 0x19);
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG4, 0x1b);//3
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG7, 0x22);
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG9, 0x23);

	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG14, 0x09);//3
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG17, 0x16);
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG19, 0x1F);

	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG1, 0x26);
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG5, 0x28);//3
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG8, 0x2F);

	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG15, 0x10);//3
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG18, 0x1F);

	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG2, 0x36);
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG6, 0x3c);//3
		sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG16, 0x18);

	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG3, 0x3F);
}

void sp_tx_initialization(void)
{
    	unchar c;

	sp_read_reg(TX_P0, SP_TX_EXTRA_ADDR_REG, &c);
	c |= I2C_EXTRA_ADDR |I2C_STRETCH_DISABLE;
	sp_write_reg(TX_P0, SP_TX_EXTRA_ADDR_REG, c);
	
	sp_read_reg(TX_P0, SP_TX_HDCP_CTRL , &c);
	c |= LINK_POLLING;
	c &= ~AUTO_START;
	c &= ~AUTO_EN;
	sp_write_reg(TX_P0, SP_TX_HDCP_CTRL, c);

	sp_read_reg(TX_P0, SP_TX_LINK_DEBUG_REG , &c);
	c |= M_VID_DEBUG;
	sp_write_reg(TX_P0, SP_TX_LINK_DEBUG_REG, c);

	sp_read_reg(TX_P0, SP_TX_DEBUG_REG1, &c);
	c |= FORCE_HPD | FORCE_PLL_LOCK | POLLING_EN;
	sp_write_reg(TX_P0, SP_TX_DEBUG_REG1, c);

	sp_read_reg(TX_P2, SP_TX_PLL_FILTER_CTRL11, &c);
	c |= AUX_TERM_50OHM;
	sp_write_reg(TX_P2, SP_TX_PLL_FILTER_CTRL11, c);

	sp_read_reg(TX_P2, SP_TX_PLL_FILTER_CTRL6, &c);
	c &= ~P5V_PROTECT_PD;
	c &= ~SHORT_PROTECT_PD;
	sp_write_reg(TX_P2, SP_TX_PLL_FILTER_CTRL6, c);

	sp_read_reg(TX_P2, SP_TX_ANALOG_DEBUG_REG2, &c);
	c |= POWERON_TIME_1P5MS;
	sp_write_reg(TX_P2, SP_TX_ANALOG_DEBUG_REG2, c);

	sp_read_reg(TX_P0, SP_TX_HDCP_CTRL0_REG, &c);
	c |= BKSV_SRM_PASS;
	c |= KSVLIST_VLD;
	sp_write_reg(TX_P0, SP_TX_HDCP_CTRL0_REG, c);

	sp_write_reg(TX_P2, SP_TX_ANALOG_CTRL, 0xC5);
	sp_write_reg(TX_P0, I2C_GEN_10US_TIMER0, 0x0e);
	sp_write_reg(TX_P0, I2C_GEN_10US_TIMER1, 0x01);

	c = AUTO_POLLING_DISABLE;
	sp_write_reg(TX_P0, SP_TX_DP_POLLING_CTRL_REG, c);
	/*Short the link check timer for HDCP CTS item1a-07*/
	sp_write_reg(TX_P0, SP_TX_LINK_CHK_TIMER, 0x1D);

	sp_read_reg(TX_P0, SP_TX_MISC_CTRL_REG, &c);
	c |= EQ_TRAINING_LOOP;
	sp_write_reg(TX_P0, SP_TX_MISC_CTRL_REG, c);
//ANX +++: (ver:20130105)
	sp_write_reg(TX_P2, SP_COMMON_INT_MASK1, 0X00);
	sp_write_reg(TX_P2, SP_COMMON_INT_MASK2, 0X00);
	sp_write_reg(TX_P2, SP_COMMON_INT_MASK3, 0X00);
	sp_write_reg(TX_P2, SP_COMMON_INT_MASK4, 0X00);
	sp_write_reg(TX_P2, SP_INT_MASK, 0X90);
	sp_write_reg(TX_P2, SP_TX_INT_CTRL_REG, 0X01);
//ANX ---: (ver:20130105)
	sp_write_reg(TX_P0, 0x20, 0xa2);
	sp_write_reg(TX_P0, 0x21, 0x7e);
	sp_write_reg(TX_P0, 0x1f, 0x04);
	sp_tx_link_phy_initialization();
	sp_tx_api_m_gen_clk_select(1);

}

void sp_tx_power_down(enum SP_TX_POWER_BLOCK sp_tx_pd_block)
{
	unchar c;

	sp_read_reg(TX_P2, SP_POWERD_CTRL_REG, &c);
	if (sp_tx_pd_block == SP_TX_PWR_REG)
		c |= REGISTER_PD;
	else if(sp_tx_pd_block == SP_TX_PWR_HDCP)
		c |= HDCP_PD;
	else if(sp_tx_pd_block == SP_TX_PWR_AUDIO)
		c |= AUDIO_PD;
	else if(sp_tx_pd_block == SP_TX_PWR_VIDEO)
		c |= VIDEO_PD;
	else if(sp_tx_pd_block == SP_TX_PWR_LINK)
		c |= LINK_PD;
	else if(sp_tx_pd_block == SP_TX_PWR_TOTAL)
		c |= TOTAL_PD;

	sp_write_reg(TX_P2, SP_POWERD_CTRL_REG, c);

	 DEV_DBG("sp_tx_power_down");

}

void sp_tx_power_on(enum SP_TX_POWER_BLOCK sp_tx_pd_block)
{
	unchar c;

	sp_read_reg(TX_P2, SP_POWERD_CTRL_REG, &c);
	if (sp_tx_pd_block == SP_TX_PWR_REG)
		c &= ~REGISTER_PD;
	else if(sp_tx_pd_block == SP_TX_PWR_HDCP)
		c &= ~HDCP_PD;
	else if(sp_tx_pd_block == SP_TX_PWR_AUDIO)
		c &= ~AUDIO_PD;
	else if(sp_tx_pd_block == SP_TX_PWR_VIDEO)
		c &= ~VIDEO_PD;
	else if(sp_tx_pd_block == SP_TX_PWR_LINK)
		c &= ~LINK_PD;
	else if(sp_tx_pd_block == SP_TX_PWR_TOTAL)
		c &= ~TOTAL_PD;

	sp_write_reg(TX_P2, SP_POWERD_CTRL_REG, c);
	DEV_DBG("sp_tx_power_on");

}
//ANX +++: (ver:0.2)
void sp_tx_pull_down_id(bool enable)
{
	unchar c;
	if(enable) {
		sp_read_reg(TX_P2, SP_TX_ANAOG_DBG_REG1, &c);
		c |= SP_TX_ANAOG_DBG_REG1;
		sp_write_reg(TX_P2, SP_TX_ANAOG_DBG_REG1, c);

	} else {
		sp_read_reg(TX_P2, SP_TX_ANAOG_DBG_REG1, &c);
		c &= ~SP_TX_ANAOG_DBG_REG1;
		sp_write_reg(TX_P2, SP_TX_ANAOG_DBG_REG1, c);
	}
		
}
//ANX ---: (ver:0.2)

void sp_tx_rst_aux(void)
{
	unchar c, c1;

	sp_read_reg(TX_P0, SP_TX_DEBUG_REG1, &c1);
	c = c1;
	c1 &= ~HPD_POLLING_EN;
	c1 &= ~POLLING_EN;
	sp_write_reg(TX_P0, SP_TX_DEBUG_REG1, c1);

	sp_read_reg(TX_P2, SP_TX_RST_CTRL2_REG, &c1);
	c1 |= AUX_RST;
	sp_write_reg(TX_P2, SP_TX_RST_CTRL2_REG, c1);
	msleep(1);
	c1 &= ~AUX_RST;
	sp_write_reg(TX_P2, SP_TX_RST_CTRL2_REG, c1);

	/* enable  polling  after reset AUX-ANX.Fei-2011.9.19 */
	sp_write_reg(TX_P0, SP_TX_DEBUG_REG1, c);
}

static unchar sp_tx_wait_aux_finished(void)
{
	unchar c;
	unchar cCnt;
	cCnt = 0;

	sp_read_reg(TX_P0, SP_TX_AUX_STATUS, &c);

	while (c & AUX_BUSY) {
		cCnt++;
		sp_read_reg(TX_P0, SP_TX_AUX_STATUS, &c);
		if (cCnt > 100) {
			DEV_ERR("AUX Operaton does not finished, and time out.\n");
			break;
		}
	}

	if (c & 0x0F) {
		DEV_ERR("aux operation failed %.2x\n", (uint)c);
		return 0;
	} else
		return 1;

}

unchar sp_tx_aux_dpcdread_bytes(unchar addrh, unchar addrm,
	unchar addrl, unchar cCount, unchar *pBuf)
{
	unchar c, i;
	unchar bOK;

	sp_write_reg(TX_P0, SP_TX_BUF_DATA_COUNT_REG, 0x80);
	c = ((cCount - 1) << 4) | 0x09;
	sp_write_reg(TX_P0, SP_TX_AUX_CTRL_REG, c);

	sp_write_reg(TX_P0, SP_TX_AUX_ADDR_7_0_REG, addrl);
	sp_write_reg(TX_P0, SP_TX_AUX_ADDR_15_8_REG, addrm);

	sp_read_reg(TX_P0, SP_TX_AUX_ADDR_19_16_REG, &c);
	c = (c & 0xf0) | addrh;
	sp_write_reg(TX_P0, SP_TX_AUX_ADDR_19_16_REG, c);

	sp_read_reg(TX_P0, SP_TX_AUX_CTRL_REG2, &c);
	c |= AUX_OP_EN;
	sp_write_reg(TX_P0, SP_TX_AUX_CTRL_REG2, c);

	msleep(2);

	bOK = sp_tx_wait_aux_finished();

	if (!bOK) {
		DEV_ERR("aux read failed\n");
		sp_tx_rst_aux();
		return AUX_ERR;
	}

	for (i = 0; i < cCount; i++) {
		sp_read_reg(TX_P0, SP_TX_BUF_DATA_0_REG + i, &c);

		*(pBuf + i) = c;

		if (i >= MAX_BUF_CNT)
			break;
	}

	return AUX_OK;

}

static unchar sp_tx_aux_dpcdwrite_bytes(unchar addrh, unchar addrm,
	unchar addrl, unchar cCount, unchar *pBuf)
{
	unchar c, i;
	unchar bOK;

	sp_write_reg(TX_P0, SP_TX_BUF_DATA_COUNT_REG, 0x80);

	c =  ((cCount - 1) << 4) | 0x08;
	sp_write_reg(TX_P0, SP_TX_AUX_CTRL_REG, c);

	sp_write_reg(TX_P0, SP_TX_AUX_ADDR_7_0_REG, addrl);
	sp_write_reg(TX_P0, SP_TX_AUX_ADDR_15_8_REG, addrm);

	sp_read_reg(TX_P0, SP_TX_AUX_ADDR_19_16_REG, &c);
	c = (c & 0xf0) | addrh;
	sp_write_reg(TX_P0, SP_TX_AUX_ADDR_19_16_REG, c);

	for (i = 0; i < cCount; i++) {
		c = *pBuf;
		pBuf++;
		sp_write_reg(TX_P0, SP_TX_BUF_DATA_0_REG + i, c);

		if (i >= MAX_BUF_CNT)
			break;
	}

	sp_read_reg(TX_P0, SP_TX_AUX_CTRL_REG2, &c);
	c |= 0x01;
	sp_write_reg(TX_P0, SP_TX_AUX_CTRL_REG2, c);

	bOK = sp_tx_wait_aux_finished();

	if (bOK)
		return AUX_OK;
	else
		return AUX_ERR;


}

static void sp_tx_aux_dpcdwrite_byte(unchar addrh, unchar addrm,
	unchar addrl, unchar data1)
{

	unchar c;

	sp_write_reg(TX_P0, SP_TX_BUF_DATA_COUNT_REG, 0x80);
	c = (0 << 4) | 0x08;
	sp_write_reg(TX_P0, SP_TX_AUX_CTRL_REG, c);


	sp_write_reg(TX_P0, SP_TX_AUX_ADDR_7_0_REG, addrl);
	sp_write_reg(TX_P0, SP_TX_AUX_ADDR_15_8_REG, addrm);


	sp_read_reg(TX_P0, SP_TX_AUX_ADDR_19_16_REG, &c);
	c = (c & 0xf0) | addrh;
	sp_write_reg(TX_P0, SP_TX_AUX_ADDR_19_16_REG, c);


	sp_write_reg(TX_P0, SP_TX_BUF_DATA_0_REG, data1);


	sp_read_reg(TX_P0, SP_TX_AUX_CTRL_REG2, &c);
	c |= AUX_OP_EN;
	sp_write_reg(TX_P0, SP_TX_AUX_CTRL_REG2, c);

	sp_tx_wait_aux_finished();

	return;
}


void sp_tx_set_colorspace(void)
{
	unchar c;
	unchar color_space;

	sp_read_reg(RX_P1, HDMI_RX_AVI_DATA00_REG, &color_space);
	color_space &= 0x60;
	sp_read_reg(TX_P2, SP_TX_VID_CTRL2_REG, &c);
	c = (c & 0xfc) | color_space >> 5;
	sp_write_reg(TX_P2, SP_TX_VID_CTRL2_REG, c);
//ANX +++: (ver:20130105)
	switch(sp_tx_rx_type){
	case RX_VGA:
		sp_read_reg(TX_P2, SP_TX_VID_CTRL2_REG, &color_space);
		if((color_space & 0x03)== 0x01)  {
			sp_read_reg(TX_P2, SP_TX_VID_CTRL5_REG, &c);
			c |= RANGE_Y2R;
			c |= CSPACE_Y2R;
			sp_write_reg(TX_P2, SP_TX_VID_CTRL5_REG, c);
			
			sp_read_reg(RX_P1, (HDMI_RX_AVI_DATA00_REG + 3), &c);
			if((c ==0x04)||(c ==0x05)||(c ==0x10)||
				(c ==0x13)||(c ==0x14)||(c ==0x1F)||
				(c ==0x20)||(c ==0x21)||(c ==0x22)){
				sp_read_reg(TX_P2, SP_TX_VID_CTRL5_REG, &c);
				c |= CSC_STD_SEL;
				sp_write_reg(TX_P2, SP_TX_VID_CTRL5_REG, c);
			}else {
				sp_read_reg(TX_P2, SP_TX_VID_CTRL5_REG, &c);
				c &= ~CSC_STD_SEL;
				sp_write_reg(TX_P2, SP_TX_VID_CTRL5_REG, c);
			}
			
			sp_read_reg(TX_P2, SP_TX_VID_CTRL6_REG, &c);
			c |= VIDEO_PROCESS_EN;
			c |= UP_SAMPLE;
			sp_write_reg(TX_P2, SP_TX_VID_CTRL6_REG, c);
		} else if((color_space & 0x03) == 0x02)  {
			sp_read_reg(TX_P2, SP_TX_VID_CTRL5_REG, &c);
			c |= RANGE_Y2R;
			c |= CSPACE_Y2R;
			sp_write_reg(TX_P2, SP_TX_VID_CTRL5_REG, c);

			sp_read_reg(RX_P1, (HDMI_RX_AVI_DATA00_REG + 3), &c);
			if((c ==0x04)||(c ==0x05)||(c ==0x10)||
			(c ==0x13)||(c ==0x14)||(c ==0x1F)||
			(c ==0x20)||(c ==0x21)||(c ==0x22)){
				sp_read_reg(TX_P2, SP_TX_VID_CTRL5_REG, &c);
				c |= CSC_STD_SEL;
				sp_write_reg(TX_P2, SP_TX_VID_CTRL5_REG, c);
			}else {
				sp_read_reg(TX_P2, SP_TX_VID_CTRL5_REG, &c);
				c &= ~CSC_STD_SEL;
				sp_write_reg(TX_P2, SP_TX_VID_CTRL5_REG, c);
			}

			sp_read_reg(TX_P2, SP_TX_VID_CTRL6_REG, &c);
			c |= VIDEO_PROCESS_EN;
			c &= ~UP_SAMPLE;
			sp_write_reg(TX_P2, SP_TX_VID_CTRL6_REG, c);
		} else if((color_space & 0x03) == 0x00)  {
			sp_read_reg(TX_P2, SP_TX_VID_CTRL5_REG, &c);
			c &= ~RANGE_Y2R;
			c &= ~CSPACE_Y2R;
			c &= ~CSC_STD_SEL;
			sp_write_reg(TX_P2, SP_TX_VID_CTRL5_REG, c);

			sp_read_reg(TX_P2, SP_TX_VID_CTRL6_REG, &c);
			c &=~ VIDEO_PROCESS_EN;
			c &= ~UP_SAMPLE;
			sp_write_reg(TX_P2, SP_TX_VID_CTRL6_REG, c);
		}
		break;
	case RX_DP:
	case RX_HDMI:
		break;
	default:
		break;
	}
//ANX ---: (ver:20130105)	
}

static void sp_tx_vsi_setup(void)
{
	unchar c;
	int i;

	for (i = 0; i < 10; i++) {
		sp_read_reg(RX_P1, (HDMI_RX_MPEG_DATA00_REG + i), &c);
		sp_tx_packet_mpeg.MPEG_data[i] = c;
	}
}


static void sp_tx_mpeg_setup(void)
{
	unchar c;
	int i;

	for (i = 0; i < 10; i++) {
		sp_read_reg(RX_P1, (HDMI_RX_MPEG_DATA00_REG + i), &c);
		sp_tx_packet_mpeg.MPEG_data[i] = c;
	}
}




static void sp_tx_get_int_status(enum INTStatus IntIndex, unchar *cStatus)
{
	unchar c;

	sp_read_reg(TX_P2, SP_COMMON_INT_STATUS1 + IntIndex, &c);
	sp_write_reg(TX_P2, SP_COMMON_INT_STATUS1 + IntIndex, c);

	*cStatus = c;
}

static unchar sp_tx_get_pll_lock_status(void)
{
	unchar c;

	sp_read_reg(TX_P0, SP_TX_DEBUG_REG1, &c);
	if (c & DEBUG_PLL_LOCK)
		return 1;
	else
		return 0;

}



static void sp_tx_lvttl_bit_mapping(void)
{

	enum HDMI_color_depth hdmi_input_color_depth = Hdmi_legacy;
	unchar c, c1;
	unchar vid_bit, value;

	sp_read_reg(RX_P0, HDMI_RX_VIDEO_STATUS_REG1, &c1);
	c1 &= COLOR_DEPTH;
	if (c1 == 0x00)
		hdmi_input_color_depth = Hdmi_legacy;
	else if (c1 == 0x40)
		hdmi_input_color_depth = Hdmi_24bit;
	else if (c1 == 0x50)
		hdmi_input_color_depth = Hdmi_30bit;
	else if (c1 == 0x60)
		hdmi_input_color_depth = Hdmi_36bit;
	else
		pr_warn("HDMI input color depth is not supported .\n");

	switch (hdmi_input_color_depth) {
	case Hdmi_legacy:
	case Hdmi_24bit:
		sp_read_reg(TX_P2, SP_TX_VID_CTRL1_REG, &c);
		c = c & ~IN_BIT_SEl;
		sp_write_reg(TX_P2, SP_TX_VID_CTRL1_REG, c);

		sp_read_reg(TX_P2, SP_TX_VID_CTRL2_REG, &c);
		c = (c & 0x8f) | IN_BPC_8BIT;
		sp_write_reg(TX_P2, SP_TX_VID_CTRL2_REG, c);

		for (c = 0; c < 24; c++) {
			vid_bit = SP_TX_VID_BIT_CTRL0_REG + c;
			value = c;
			sp_write_reg(TX_P2, vid_bit, value);
		}


		break;
	case Hdmi_30bit:
		sp_read_reg(TX_P2, SP_TX_VID_CTRL1_REG, &c);
		c = c & ~IN_BIT_SEl;
		sp_write_reg(TX_P2, SP_TX_VID_CTRL1_REG, c);

		sp_read_reg(TX_P2, SP_TX_VID_CTRL2_REG, &c);
		c = (c & 0x8f) | IN_BPC_10BIT;
		sp_write_reg(TX_P2, SP_TX_VID_CTRL2_REG, c);

		for (c = 0; c < 10; c++) {
			vid_bit = SP_TX_VID_BIT_CTRL0_REG + c;
			value = 0x02 + c;
			sp_write_reg(TX_P2, vid_bit, value);
		}

		for (c = 0; c < 10; c++) {
			vid_bit = SP_TX_VID_BIT_CTRL10_REG + c;
			value = 0x0E + c;
			sp_write_reg(TX_P2, vid_bit, value);
		}

		for (c = 0; c < 10; c++) {
			vid_bit = SP_TX_VID_BIT_CTRL20_REG + c;
			value = 0x1A + c;
			sp_write_reg(TX_P2, vid_bit, value);
		}

		break;
	case Hdmi_36bit:
		sp_read_reg(TX_P2, SP_TX_VID_CTRL1_REG, &c);
		c = c & ~IN_BIT_SEl;
		sp_write_reg(TX_P2, SP_TX_VID_CTRL1_REG, c);

		sp_read_reg(TX_P2, SP_TX_VID_CTRL2_REG, &c);
		c = ((c & 0x8f) | IN_BPC_12BIT);
		sp_write_reg(TX_P2, SP_TX_VID_CTRL2_REG, c);

		for (c = 0; c < 36; c++) {
			vid_bit = SP_TX_VID_BIT_CTRL0_REG + c;
			value = c;
			sp_write_reg(TX_P2, vid_bit, value);
		}
		break;
	default:
		break;
	}

	if (sp_tx_test_edid) {
		/*set color depth to 18-bit for link cts*/
		sp_read_reg(TX_P2, SP_TX_VID_CTRL2_REG, &c);
		c = (c & 0x8f);
		sp_write_reg(TX_P2, SP_TX_VID_CTRL2_REG, c);
		sp_tx_test_edid = 0;
		DEV_DBG("***color space is set to 18bit***");
	}
	sp_read_reg(RX_P1, HDMI_RX_AVI_DATA00_REG, &c);
	if (c & 0x60) {
		sp_write_reg(TX_P0, SP_TX_VID_BLANK_SET1, 0x80);
		sp_write_reg(TX_P0, SP_TX_VID_BLANK_SET1, 0x00);
		sp_write_reg(TX_P0, SP_TX_VID_BLANK_SET1, 0x80);
	}

}


void sp_tx_enable_video_input(unchar enable)
{
	unchar c;
	unchar c1;

	if (enable) {
		sp_read_reg(TX_P2, SP_TX_VID_CTRL1_REG, &c);
		c = (c & 0xf7) | VIDEO_EN;
		sp_write_reg(TX_P2, SP_TX_VID_CTRL1_REG, c);

		sp_read_reg(TX_P2, SP_TX_VID_CTRL1_REG, &c1);
		DEV_DBG("%s, 0x72, 0x08 = 0x%x \n",__func__, c1 );

//ANX +++: (ver:20130105)		
//		sp_write_reg(TX_P2, SP_COMMON_INT_MASK1, 0xf5);
//		sp_write_reg(TX_P2, SP_COMMON_INT_STATUS1, 0x0a);
//ANX ---: (ver:20130105)
		DEV_DBG("Slimport Video is enabled!\n");

	} else {
		sp_read_reg(TX_P2, SP_TX_VID_CTRL1_REG, &c);
		c &= ~VIDEO_EN;
		sp_write_reg(TX_P2, SP_TX_VID_CTRL1_REG, c);
		DEV_DBG("Slimport Video is disabled!\n");

	}
}

static void sp_tx_enhancemode_set(void)
{
	unchar c;
	sp_tx_aux_dpcdread_bytes(0x00, 0x00, DPCD_MAX_LANE_COUNT, 1, &c);

	if (c & ENHANCED_FRAME_CAP) {

		sp_read_reg(TX_P0, SP_TX_SYS_CTRL4_REG, &c);
		c |= ENHANCED_MODE;
		sp_write_reg(TX_P0, SP_TX_SYS_CTRL4_REG, c);

		sp_tx_aux_dpcdread_bytes(0x00, 0x01,
			DPCD_LANE_COUNT_SET, 1, &c);
		c |= ENHANCED_FRAME_EN;
		sp_tx_aux_dpcdwrite_byte(0x00, 0x01,
			DPCD_LANE_COUNT_SET, c);

		DEV_DBG("Enhance mode enabled\n");
	} else {

		sp_read_reg(TX_P0, SP_TX_SYS_CTRL4_REG, &c);
		c &= ~ENHANCED_MODE;
		sp_write_reg(TX_P0, SP_TX_SYS_CTRL4_REG, c);

		sp_tx_aux_dpcdread_bytes(0x00, 0x01,
			DPCD_LANE_COUNT_SET, 1, &c);
		c &= ~ENHANCED_FRAME_EN;
		sp_tx_aux_dpcdwrite_byte(0x00, 0x01,
			DPCD_LANE_COUNT_SET, c);

		DEV_DBG("Enhance mode disabled\n");
	}
}

static void sp_tx_hdcp_reauth(void)
{
	unchar c;
	sp_read_reg(TX_P0, SP_TX_HDCP_CTRL0_REG, &c);
	c |= RE_AUTH;
	sp_write_reg(TX_P0, SP_TX_HDCP_CTRL0_REG, c);
	c &= ~RE_AUTH;
	sp_write_reg(TX_P0, SP_TX_HDCP_CTRL0_REG, c);
}

static void sp_tx_clean_hdcp_status(void)
{

	sp_write_reg(TX_P0, SP_TX_HDCP_CTRL0_REG, 0x00);
	sp_tx_hdcp_reauth();
}

static void sp_tx_hdcp_encryption_disable(void)
{
	unchar c;
	sp_read_reg(TX_P0, SP_TX_HDCP_CTRL0_REG, &c);
	c &= ~ENC_EN;
	sp_write_reg(TX_P0, SP_TX_HDCP_CTRL0_REG, c);
}

static void sp_tx_hdcp_encryption_enable(void)
{
	unchar c;
	sp_read_reg(TX_P0, SP_TX_HDCP_CTRL0_REG, &c);
	c |= ENC_EN;
	sp_write_reg(TX_P0, SP_TX_HDCP_CTRL0_REG, c);
}

static void sp_tx_hw_hdcp_enable(void)
{
	unchar c;
	sp_read_reg(TX_P0, SP_TX_HDCP_CTRL0_REG, &c);
	c &= ~ENC_EN;
	c &= ~HARD_AUTH_EN;
	sp_write_reg(TX_P0, SP_TX_HDCP_CTRL0_REG, c);
	sp_read_reg(TX_P0, SP_TX_HDCP_CTRL0_REG, &c);
	c |= HARD_AUTH_EN;
	c |= BKSV_SRM_PASS;
	c |= KSVLIST_VLD;
	c |= ENC_EN;
	sp_write_reg(TX_P0, SP_TX_HDCP_CTRL0_REG, c);
	sp_write_reg(TX_P0, SP_TX_WAIT_R0_TIME, 0xb0);
	sp_write_reg(TX_P0, SP_TX_WAIT_KSVR_TIME, 0xc8);
//ANX +++: (ver:20130105)
	//sp_write_reg(TX_P2, SP_COMMON_INT_MASK2, 0xfc);
//ANX ---: (ver:20130105)
	DEV_DBG("Hardware HDCP is enabled.\n");

}

void sp_tx_clean_hdcp(void)
{

	sp_tx_hdcp_auth_fail_counter = 0;
	sp_tx_hdcp_auth_pass = 0;
	sp_tx_hw_hdcp_en = 0;
	sp_tx_hdcp_capable_chk = 0;
	sp_tx_hdcp_auth_done = 0;
	sp_tx_clean_hdcp_status();
	DEV_DBG("HDCP Clean!\n");
}

static void sp_tx_pclk_calc(unchar *hbr_rbr)
{
	ulong str_clk = 0;
	unchar c;
	unchar link_bw_current = *hbr_rbr;


	switch (link_bw_current) {

      case 0x14:
		str_clk = 540;
		break;
      case 0x0a:
		str_clk = 270;
		break;
      case 0x06:
		str_clk = 162;
		break;
	default:
		break;

	}

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
}

void sp_tx_show_infomation(void)
{
	unchar c, c1;
	uint h_res, h_act, v_res, v_act;
	uint h_fp, h_sw, h_bp, v_fp, v_sw, v_bp;
	ulong fresh_rate;

	DEV_DBG("*******SP Video Information*******\n");

	sp_read_reg(TX_P0, SP_TX_LINK_BW_SET_REG, &c);
	if (c == 0x06) {
		DEV_DBG("BW = 1.62G\n");
		sp_tx_pclk_calc(&c);
	} else if (c == 0x0a) {
		DEV_DBG("BW = 2.7G\n");
		sp_tx_pclk_calc(&c);
	} else if (c == 0x14) {
		DEV_DBG("BW = 5.4G\n");
		sp_tx_pclk_calc(&c);
	}
#ifdef SSC_EN
	DEV_DBG("   SSC On");
#else
	DEV_DBG("   SSC Off");
#endif

	DEV_DBG("   M = %lu, N = %lu, PCLK = %ld MHz\n", m_val, n_val, pclk);

	sp_read_reg(TX_P2, SP_TX_TOTAL_LINE_STA_L, &c);
	sp_read_reg(TX_P2, SP_TX_TOTAL_LINE_STA_H, &c1);

	v_res = c1;
	v_res = v_res << 8;
	v_res = v_res + c;

	sp_read_reg(TX_P2, SP_TX_ACT_LINE_STA_L, &c);
	sp_read_reg(TX_P2, SP_TX_ACT_LINE_STA_H, &c1);

	v_act = c1;
	v_act = v_act << 8;
	v_act = v_act + c;

	sp_read_reg(TX_P2, SP_TX_TOTAL_PIXEL_STA_L, &c);
	sp_read_reg(TX_P2, SP_TX_TOTAL_PIXEL_STA_H, &c1);

	h_res = c1;
	h_res = h_res << 8;
	h_res = h_res + c;

	sp_read_reg(TX_P2, SP_TX_ACT_PIXEL_STA_L, &c);
	sp_read_reg(TX_P2, SP_TX_ACT_PIXEL_STA_H, &c1);

	h_act = c1;
	h_act = h_act << 8;
	h_act = h_act + c;

	sp_read_reg(TX_P2, SP_TX_H_F_PORCH_STA_L, &c);
	sp_read_reg(TX_P2, SP_TX_H_F_PORCH_STA_H, &c1);

	h_fp = c1;
	h_fp = h_fp << 8;
	h_fp = h_fp + c;

	sp_read_reg(TX_P2, SP_TX_H_SYNC_STA_L, &c);
	sp_read_reg(TX_P2, SP_TX_H_SYNC_STA_H, &c1);

	h_sw = c1;
	h_sw = h_sw << 8;
	h_sw = h_sw + c;

	sp_read_reg(TX_P2, SP_TX_H_B_PORCH_STA_L, &c);
	sp_read_reg(TX_P2, SP_TX_H_B_PORCH_STA_H, &c1);

	h_bp = c1;
	h_bp = h_bp << 8;
	h_bp = h_bp + c;

	sp_read_reg(TX_P2, SP_TX_V_F_PORCH_STA, &c);
	v_fp = c;

	sp_read_reg(TX_P2, SP_TX_V_SYNC_STA, &c);
	v_sw = c;

	sp_read_reg(TX_P2, SP_TX_V_B_PORCH_STA, &c);
	v_bp = c;
	DEV_DBG("   Total resolution is %d * %d\n", h_res, v_res);
	DEV_DBG("   HF=%d, HSW=%d, HBP=%d\n", h_fp, h_sw, h_bp);
	DEV_DBG("   VF=%d, VSW=%d, VBP=%d\n", v_fp, v_sw, v_bp);
	DEV_DBG("   Active resolution is %d * %d ", h_act, v_act);

        if (h_res && v_res)
	{
	fresh_rate = pclk * 1000;
	fresh_rate = fresh_rate / h_res;
	fresh_rate = fresh_rate * 1000;
	fresh_rate = fresh_rate / v_res;
	DEV_DBG(" @ %ldHz\n", fresh_rate);
	}
	sp_read_reg(TX_P0, SP_TX_VID_CTRL, &c);

	if ((c & 0x06) == 0x00)
		DEV_DBG("   ColorSpace: RGB,");
	else if ((c & 0x06) == 0x02)
		DEV_DBG("   ColorSpace: YCbCr422,");
	else if ((c & 0x06) == 0x04)
		DEV_DBG("   ColorSpace: YCbCr444,");

	sp_read_reg(TX_P0, SP_TX_VID_CTRL, &c);

	if ((c & 0xe0) == 0x00)
		DEV_DBG("  6 BPC");
	else if ((c & 0xe0) == 0x20)
		DEV_DBG("  8 BPC");
	else if ((c & 0xe0) == 0x40)
		DEV_DBG("  10 BPC");
	else if ((c & 0xe0) == 0x60)
		DEV_DBG("  12 BPC");
//ANX +++: (ver:20130105)
	if(sp_tx_rx_type == RX_HDMI) {
		sp_tx_aux_dpcdread_bytes(0x00, 0x05, 0x23, 1, bytebuf);
		DEV_DBG("ANX7730 BC current FW Ver %.2x \n", (uint)(bytebuf[0]&0x7f));
	}
//ANX ---: (ver:20130105)
	DEV_DBG("***********************************\n");

}

static void sp_tx_aux_wr(unchar offset)
{
	unchar c, cnt;
	cnt = 0;

	sp_write_reg(TX_P0, SP_TX_BUF_DATA_0_REG, offset);
	sp_write_reg(TX_P0, SP_TX_AUX_CTRL_REG, 0x04);
	sp_write_reg(TX_P0, SP_TX_AUX_CTRL_REG2, 0x01);
	msleep(10);
	sp_read_reg(TX_P0, SP_TX_AUX_CTRL_REG2, &c);
	while (c & AUX_OP_EN) {
		msleep(10);
		cnt++;

		if (cnt == 10) {
			DEV_ERR("write break\n");
			cnt = 0;
			bedid_break = 1;
			break;
		}

		sp_read_reg(TX_P0, SP_TX_AUX_CTRL_REG2, &c);
	}

}

static void sp_tx_aux_rd(unchar len_cmd)
{
	unchar c, cnt;
	cnt = 0;

	sp_write_reg(TX_P0, SP_TX_AUX_CTRL_REG, len_cmd);
	sp_write_reg(TX_P0, SP_TX_AUX_CTRL_REG2, 0x01);
	msleep(10);
	sp_read_reg(TX_P0, SP_TX_AUX_CTRL_REG2, &c);

	while (c & AUX_OP_EN) {
		msleep(10);
		cnt++;

		if (cnt == 10) {
			DEV_ERR("read break\n");
			sp_tx_rst_aux();
			bedid_break = 1;
			break;
		}

		sp_read_reg(TX_P0, SP_TX_AUX_CTRL_REG2, &c);
	}

}

unchar sp_tx_chip_located(void)
{
	unchar c1, c2;
	unchar c3; //+++ ASUS BSP Bernard add
	struct anx7808_platform_data *pdata; //+++ ASUS BSP Bernard add

	ASUS_DEV_INFO("### start check myDP chip ###\n");
	
	sp_tx_hardware_poweron(anx7808_client);
	sp_read_reg(TX_P2, SP_TX_DEV_IDL_REG, &c1);
	sp_read_reg(TX_P2, SP_TX_DEV_IDH_REG, &c2);
//+++ ASUS BSP Bernard: extra for the 7805 env.
	sp_read_reg(TX_P2, SP_TX_DEV_REV_REG , &c3);
	ASUS_DEV_WARN("dev IDL = %.2x, deb IDH = %.2x, REV= %.2x\n",c1,c2,c3);
//--- ASUS BSP Bernard: extra for the 7805 env.
      
	if ((c1 == 0x08) && (c2 == 0x78)) {
		DEV_DBG("ANX7808 is found.\n");
		//+++ ASUS BSP Bernard: read ChipID for ATD test
		pdata = anx7808_client->dev.platform_data;
		//pdata->ID = (c2 <<8) | (c1);
		pdata->ID = 1; 
		pdata->IDH = c2;
		pdata->IDL = c1;
		//--- ASUS BSP Bernard: read ChipID for ATD test
		return 1;
	}
//+++ ASUS BSP Bernard: to simulate the 7808 env. by 7805
	else if ((c1==0x05) && (c2==0x78)&&(c3==0xca)){
		DEV_DBG("ANX7805 Reversion CA\n");
		//+++ ASUS BSP Bernard: read ChipID for ATD test
		pdata = anx7808_client->dev.platform_data;
		//pdata->ID = (c2 <<8) | (c1);
		pdata->ID = 1; 
		pdata->IDH = c2;
		pdata->IDL = c1;
		//--- ASUS BSP Bernard: read ChipID for ATD test
		return 1;
	}
//+++ ASUS BSP Bernard: to simulate the 7808 env. by 7805
	else {
		sp_tx_hardware_powerdown(anx7808_client);
		DEV_DBG("ANX7808 is not found.\n");
		//+++ ASUS BSP Bernard: for ATD test
		pdata->ID = 0; 
		//--- ASUS BSP Bernard: for ATD test
		return 0;
	}

}

void sp_tx_vbus_poweron(void)
{
	unchar c;
	int i;

	for (i = 0; i < 5; i++) {
		sp_read_reg(TX_P2, SP_TX_PLL_FILTER_CTRL6, &c);
		c &= ~P5V_PROTECT_PD;
		c &= ~SHORT_PROTECT_PD;
		sp_write_reg(TX_P2, SP_TX_PLL_FILTER_CTRL6,  c);

		sp_read_reg(TX_P2, SP_TX_PLL_FILTER_CTRL11, &c);
		c &= ~V33_SWITCH_ON;
		sp_write_reg(TX_P2, SP_TX_PLL_FILTER_CTRL11, c);
		c |= V33_SWITCH_ON;
		sp_write_reg(TX_P2, SP_TX_PLL_FILTER_CTRL11, c);

		sp_read_reg(TX_P2, SP_TX_PLL_FILTER_CTRL6, &c);
		if (!(c & 0xc0)) {
			DEV_ERR("3.3V output enabled\n");
			return;
		}else{
			DEV_ERR("VBUS power can not be supplied\n");
		}
	}

}

void sp_tx_vbus_powerdown(void)
{
	unchar c;

	sp_read_reg(TX_P2, SP_TX_PLL_FILTER_CTRL11, &c);
	c &= ~V33_SWITCH_ON;
	sp_write_reg(TX_P2, SP_TX_PLL_FILTER_CTRL11, c);

	sp_read_reg(TX_P2, SP_TX_PLL_FILTER_CTRL6, &c);
	c |= P5V_PROTECT_PD | SHORT_PROTECT_PD;
	sp_write_reg(TX_P2, SP_TX_PLL_FILTER_CTRL6, c);
	DEV_NOTICE("3.3V output disabled\n");
}

static void sp_tx_spread_enable(unchar benable)
{
	unchar c;

	sp_read_reg(TX_P0, SSC_CTRL_REG1, &c);

	if (benable) {
		c |= SPREAD_AMP;
		sp_write_reg(TX_P0, SSC_CTRL_REG1, c);

		sp_read_reg(TX_P2, SP_TX_RST_CTRL2_REG, &c);
		c |= SSC_RST;
		sp_write_reg(TX_P2, SP_TX_RST_CTRL2_REG, c);
		c &= ~SSC_RST;
		sp_write_reg(TX_P2, SP_TX_RST_CTRL2_REG, c);

		sp_tx_aux_dpcdread_bytes(0x00, 0x01,
			DPCD_DOWNSPREAD_CTRL, 1, &c);
		c |= SPREAD_AMPLITUDE;
		sp_tx_aux_dpcdwrite_byte(0x00, 0x01, DPCD_DOWNSPREAD_CTRL, c);

	} else {

		c &= ~SPREAD_AMP;
		sp_write_reg(TX_P0, SSC_CTRL_REG1, c);

		sp_tx_aux_dpcdread_bytes(0x00, 0x01,
			DPCD_DOWNSPREAD_CTRL, 1, &c);
		c &= ~SPREAD_AMPLITUDE;
		sp_tx_aux_dpcdwrite_byte(0x00, 0x01, DPCD_DOWNSPREAD_CTRL, c);
	}

}

static void sp_tx_config_ssc(enum SP_LINK_BW linkbw)
{
	unchar c;


	sp_write_reg(TX_P0, SSC_CTRL_REG1, 0x00);
	sp_tx_aux_dpcdread_bytes(0x00, 0x00, DPCD_MAX_DOWNSPREAD, 1, &c);

#ifndef SSC_1
/*	DEV_DBG("*** Config SSC 0.4% ***");*/

	if (linkbw == BW_54G) {
		sp_write_reg(TX_P0, SP_TX_DOWN_SPREADING_CTRL1, 0xc0);
		sp_write_reg(TX_P0, SP_TX_DOWN_SPREADING_CTRL2, 0x00);
		sp_write_reg(TX_P0, SP_TX_DOWN_SPREADING_CTRL3, 0x75);

	} else if (linkbw == BW_27G) {
		sp_write_reg(TX_P0, SP_TX_DOWN_SPREADING_CTRL1, 0x5f);
		sp_write_reg(TX_P0, SP_TX_DOWN_SPREADING_CTRL2, 0x00);
		sp_write_reg(TX_P0, SP_TX_DOWN_SPREADING_CTRL3, 0x75);
	} else {
		sp_write_reg(TX_P0, SP_TX_DOWN_SPREADING_CTRL1, 0x9e);
		sp_write_reg(TX_P0, SP_TX_DOWN_SPREADING_CTRL2, 0x00);
		sp_write_reg(TX_P0, SP_TX_DOWN_SPREADING_CTRL3, 0x6d);
	}

#else
/*	DEV_DBG("*** Config SSC 1% ***");*/

	if (linkbw == BW_54G) {
		sp_write_reg(TX_P0, SP_TX_DOWN_SPREADING_CTRL1, 0xdd);
		sp_write_reg(TX_P0, SP_TX_DOWN_SPREADING_CTRL2, 0x01);
		sp_write_reg(TX_P0, SP_TX_DOWN_SPREADING_CTRL3, 0x76);
	} else if (linkbw == BW_27G) {
		sp_write_reg(TX_P0, SP_TX_DOWN_SPREADING_CTRL1, 0xef);
		sp_write_reg(TX_P0, SP_TX_DOWN_SPREADING_CTRL2, 0x00);
		sp_write_reg(TX_P0, SP_TX_DOWN_SPREADING_CTRL3, 0x76);
	} else {
		sp_write_reg(TX_P0, SP_TX_DOWN_SPREADING_CTRL1, 0x8e);
		sp_write_reg(TX_P0, SP_TX_DOWN_SPREADING_CTRL2, 0x01);
		sp_write_reg(TX_P0, SP_TX_DOWN_SPREADING_CTRL3, 0x6d);
	}

#endif

	sp_tx_spread_enable(1);

}


static void sp_tx_audioinfoframe_setup(void)
{
	int i;
	unchar c;
	sp_read_reg(RX_P1, HDMI_RX_AUDIO_TYPE_REG, &c);
	sp_tx_audioinfoframe.type = c;
	sp_read_reg(RX_P1, HDMI_RX_AUDIO_VER_REG, &c);
	sp_tx_audioinfoframe.version = c;
	sp_read_reg(RX_P1, HDMI_RX_AUDIO_LEN_REG, &c);
	sp_tx_audioinfoframe.length = c;
	for (i = 0; i < 11; i++) {
		sp_read_reg(RX_P1, (HDMI_RX_AUDIO_DATA00_REG + i), &c);
		sp_tx_audioinfoframe.pb_byte[i] = c;
	}

}

static void sp_tx_enable_audio_output(unchar benable)
{
	unchar c;

	sp_read_reg(TX_P0, SP_TX_AUD_CTRL, &c);

	if (benable) {
		c |= AUD_EN;
		sp_write_reg(TX_P0, SP_TX_AUD_CTRL, c);

		sp_tx_audioinfoframe_setup();
		sp_tx_config_packets(AUDIF_PACKETS);
	} else {
		c &= ~AUD_EN;
		sp_write_reg(TX_P0, SP_TX_AUD_CTRL, c);

		sp_read_reg(TX_P0, SP_TX_PKT_EN_REG, &c);
		c &= ~AUD_IF_EN;
		sp_write_reg(TX_P0, SP_TX_PKT_EN_REG, c);
	}

}

static void sp_tx_get_link_bw(unchar *bwtype)
{
	unchar c;

	sp_read_reg(TX_P0, SP_TX_LINK_BW_SET_REG, &c);

	*bwtype = c;

}

static void sp_tx_config_audio(void)
{
	unchar c, g_BW;
	int i;
	ulong M_AUD, LS_Clk = 0;
	ulong AUD_Freq = 0;
	DEV_NOTICE("##Config audio ##");
	sp_tx_power_on(SP_TX_PWR_AUDIO);
	sp_read_reg(RX_P0, 0xCA, &c);

	switch (c & 0x0f) {
	case 0x00:
		AUD_Freq = 44.1;
		break;
	case 0x02:
		AUD_Freq = 48;
		break;
	case 0x03:
		AUD_Freq = 32;
		break;
	case 0x08:
		AUD_Freq = 88.2;
		break;
	case 0x0a:
		AUD_Freq = 96;
		break;
	case 0x0c:
		AUD_Freq = 176.4;
		break;
	case 0x0e:
		AUD_Freq = 192;
		break;
	default:
		break;
	}

	sp_tx_get_link_bw(&g_BW);

	switch (g_BW) {
	case BW_162G:
		LS_Clk = 162000;
		break;
	case BW_27G:
		LS_Clk = 270000;
		break;
	case BW_54G:
		LS_Clk = 540000;
		break;
	default:
		break;
	}

	DEV_DBG("AUD_Freq = %ld , LS_CLK = %ld\n", AUD_Freq, LS_Clk);

	M_AUD = ((512 * AUD_Freq) / LS_Clk) * 32768;
	M_AUD = M_AUD + 0x05;
	sp_write_reg(TX_P1, SP_TX_AUD_INTERFACE_CTRL4, (M_AUD & 0xff));
	M_AUD = M_AUD >> 8;
	sp_write_reg(TX_P1, SP_TX_AUD_INTERFACE_CTRL5, (M_AUD & 0xff));
	sp_write_reg(TX_P1, SP_TX_AUD_INTERFACE_CTRL6, 0x00);

	sp_read_reg(TX_P1, SP_TX_AUD_INTERFACE_CTRL0, &c);
	c &= ~AUD_INTERFACE_DISABLE;
	sp_write_reg(TX_P1, SP_TX_AUD_INTERFACE_CTRL0, c);

	sp_read_reg(TX_P1, SP_TX_AUD_INTERFACE_CTRL2, &c);
	c |= M_AUD_ADJUST_ST;
	sp_write_reg(TX_P1, SP_TX_AUD_INTERFACE_CTRL2, c);

	/* configure layout and channel number */
	sp_read_reg(RX_P0, HDMI_RX_HDMI_STATUS_REG, &c);

	if (c & HDMI_AUD_LAYOUT) {
		sp_read_reg(TX_P2, SP_TX_AUD_CH_NUM_REG5, &c);
		c |= CH_NUM_8 |AUD_LAYOUT;
		sp_write_reg(TX_P2, SP_TX_AUD_CH_NUM_REG5, c);
	} else {
		sp_read_reg(TX_P2, SP_TX_AUD_CH_NUM_REG5, &c);
		c &= ~CH_NUM_8;
		c &= ~AUD_LAYOUT;
		sp_write_reg(TX_P2, SP_TX_AUD_CH_NUM_REG5, c);
	}

	/* transfer audio chaneel status from HDMI Rx to Slinmport Tx */
	for (i = 0; i < 5; i++) {
		sp_read_reg(RX_P0, (HDMI_RX_AUD_IN_CH_STATUS1_REG + i), &c);
		sp_write_reg(TX_P2, (SP_TX_AUD_CH_STATUS_REG1 + i), c);
	}

	/* enable audio */
	sp_tx_enable_audio_output(1);

}


static void sp_tx_get_rx_bw(unchar bMax, unchar *cBw)
{
	//+++ ASUS BSP Bernard, for hdmi check bandwidth i2c error
	if (sp_tx_pd_mode){
		DEV_ERR("sp_tx_get_rx_bw +++ %d\n", sp_tx_pd_mode);
		*cBw = 0;
		return;
	}
	//--- ASUS BSP Bernard, for hdmi check bandwidth i2c error	
	if (bMax)
		sp_tx_aux_dpcdread_bytes(0x00, 0x00,
		DPCD_MAX_LINK_RATE, 1, cBw);
	else
		sp_tx_aux_dpcdread_bytes(0x00, 0x01,
		DPCD_LINK_BW_SET, 1, cBw);

}

static void sp_tx_edid_read_initial(void)
{
	unchar c;

	sp_write_reg(TX_P0, SP_TX_AUX_ADDR_7_0_REG, 0x50);
	sp_write_reg(TX_P0, SP_TX_AUX_ADDR_15_8_REG, 0);
	sp_read_reg(TX_P0, SP_TX_AUX_ADDR_19_16_REG, &c);
	c &= 0xf0;
	sp_write_reg(TX_P0, SP_TX_AUX_ADDR_19_16_REG, c);
}

static unchar sp_tx_aux_edidread_byte(unchar offset)
{

	unchar c, i, edid[16], data_cnt, cnt;
	unchar bReturn = 0;


	cnt = 0;

	sp_tx_aux_wr(offset);
	sp_tx_aux_rd(0xf5);

	if ((offset == 0x00) || (offset == 0x80))
		bedid_checksum = 0;

	data_cnt = 0;
	while (data_cnt < 16) {
		sp_read_reg(TX_P0, SP_TX_BUF_DATA_COUNT_REG, &c);
		c = c & 0x1f;
		if (c != 0) {
			for (i = 0; i < c; i++) {
				sp_read_reg(TX_P0,
					       SP_TX_BUF_DATA_0_REG + i,
					       &edid[i + data_cnt]);
				bedid_checksum = bedid_checksum
					+ edid[i + data_cnt];
			}
		} else {
			sp_write_reg(TX_P0, SP_TX_AUX_CTRL_REG, 0x01);
			c = ADDR_ONLY_BIT | AUX_OP_EN;
			sp_write_reg(TX_P0, SP_TX_AUX_CTRL_REG2, c);
			sp_read_reg(TX_P0, SP_TX_AUX_CTRL_REG2, &c);
			while (c & 0x01) {
				msleep(1);
				cnt++;

				if (cnt == 10) {
					DEV_ERR("read break");
					sp_tx_rst_aux();
					bedid_break = 1;
					bReturn = 0x01;
				}

				sp_read_reg(TX_P0,
					       SP_TX_AUX_CTRL_REG2, &c);
			}

			bReturn = 0x02;
			return bReturn;
		}

		data_cnt = data_cnt + c;
		if (data_cnt < 16) {
			sp_tx_rst_aux();
			msleep(10);
			c = 0x05 | ((0x0f - data_cnt) << 4);
			sp_tx_aux_rd(c);
		}
	}

	sp_write_reg(TX_P0, SP_TX_AUX_CTRL_REG, 0x01);
	c = ADDR_ONLY_BIT | AUX_OP_EN;
	sp_write_reg(TX_P0, SP_TX_AUX_CTRL_REG2, c);
	sp_read_reg(TX_P0, SP_TX_AUX_CTRL_REG2, &c);

	while(c & AUX_OP_EN)
		sp_read_reg(TX_P0, SP_TX_AUX_CTRL_REG2, &c);

	if (offset < 0x80) {

		for (i = 0; i < 16; i++)
			bedid_firstblock[offset + i] = edid[i];
	} else if (offset >= 0x80) {

		for (i = 0; i < 16; i++)
			bedid_extblock[offset - 0x80 + i] = edid[i];
	}

	if ((offset == 0x70) || (offset == 0xf0)) {
		bedid_checksum  &= 0xff;
		bedid_checksum = bedid_checksum - edid[15];
		bedid_checksum = ~bedid_checksum + 1;
		if (bedid_checksum != edid[15])
			bedid_checksum = edid[15];
	}
#ifdef EDID_DEBUG_PRINT

		for (i = 0; i < 16; i++) {
			if ((i & 0x0f) == 0)
				DEV_DBG("\n edid: [%.2x]  %.2x  ",
				  (unsigned int)offset, (uint)edid[i]);
			else
				DEV_DBG("%.2x  ", (uint)edid[i]);

			if ((i & 0x0f) == 0x0f)
				DEV_DBG("\n");
		}

#endif

	return bReturn;
}

static void sp_tx_parse_segments_edid(unchar segment, unchar offset)
{
	unchar c, cnt;
	int i;

	sp_write_reg(TX_P0, SP_TX_AUX_CTRL_REG, 0x04);
	sp_write_reg(TX_P0, SP_TX_AUX_ADDR_7_0_REG, 0x30);
	c = ADDR_ONLY_BIT | AUX_OP_EN;
	sp_write_reg(TX_P0, SP_TX_AUX_CTRL_REG2, c);
	sp_read_reg(TX_P0, SP_TX_AUX_CTRL_REG2, &c);
	sp_tx_wait_aux_finished();
	sp_read_reg(TX_P0, SP_TX_AUX_CTRL_REG, &c);
	sp_write_reg(TX_P0, SP_TX_BUF_DATA_0_REG, segment);

	sp_write_reg(TX_P0, SP_TX_AUX_CTRL_REG, 0x04);
	sp_read_reg(TX_P0, SP_TX_AUX_CTRL_REG2, &c);
	c &= ~ADDR_ONLY_BIT;
	c |= AUX_OP_EN;
	sp_write_reg(TX_P0, SP_TX_AUX_CTRL_REG2, c);
	cnt = 0;
	sp_read_reg(TX_P0, SP_TX_AUX_CTRL_REG2, &c);
	while (c & AUX_OP_EN) {
		msleep(1);
		cnt++;
		if (cnt == 10) {
			DEV_ERR("write break");
			sp_tx_rst_aux();
			cnt = 0;
			bedid_break = 1;
			return;
		}

		sp_read_reg(TX_P0, SP_TX_AUX_CTRL_REG2, &c);

	}

	sp_write_reg(TX_P0, SP_TX_AUX_ADDR_7_0_REG, 0x50);
	c = ADDR_ONLY_BIT | AUX_OP_EN;
	sp_write_reg(TX_P0, SP_TX_AUX_CTRL_REG2, c);
	sp_tx_aux_wr(offset);
	c = ADDR_ONLY_BIT | AUX_OP_EN;
	sp_write_reg(TX_P0, SP_TX_AUX_CTRL_REG2, c);
	sp_tx_aux_rd(0xf5);
	cnt = 0;
	for (i = 0; i < 16; i++) {
		sp_read_reg(TX_P0, SP_TX_BUF_DATA_COUNT_REG, &c);
		while ((c & 0x1f) == 0) {
			//msleep(2);
			cnt++;
			sp_read_reg(TX_P0,
				       SP_TX_BUF_DATA_COUNT_REG, &c);

			if (cnt == 10) {
				DEV_ERR("read break");
				sp_tx_rst_aux();
				bedid_break = 1;
				return;
			}
		}

		sp_read_reg(TX_P0, SP_TX_BUF_DATA_0_REG + i, &c);
	}

	sp_write_reg(TX_P0, SP_TX_AUX_CTRL_REG, 0x01);
	c = ADDR_ONLY_BIT | AUX_OP_EN;
	sp_write_reg(TX_P0, SP_TX_AUX_CTRL_REG2, c);
	sp_read_reg(TX_P0, SP_TX_AUX_CTRL_REG2, &c);

	while(c & AUX_OP_EN)
		sp_read_reg(TX_P0, SP_TX_AUX_CTRL_REG2, &c);

}

static unchar sp_tx_get_edid_block(void)
{
	unchar c;
	sp_tx_aux_wr(0x00);
	sp_tx_aux_rd(0x01);
	sp_read_reg(TX_P0, SP_TX_BUF_DATA_0_REG, &c);


	sp_tx_aux_wr(0x7e);
	sp_tx_aux_rd(0x01);
	sp_read_reg(TX_P0, SP_TX_BUF_DATA_0_REG, &c);
	DEV_DBG("EDID Block = %d\n", (int)(c + 1));

	if (c > 3)
		c = 1;

	return c;
}

static void sp_tx_addronly_set(unchar bSet)
{
	unchar c;

	sp_read_reg(TX_P0, SP_TX_AUX_CTRL_REG2, &c);
	if (bSet) {
		c |= ADDR_ONLY_BIT;
		sp_write_reg(TX_P0, SP_TX_AUX_CTRL_REG2, c);
	} else {
		c &= ~ADDR_ONLY_BIT;
		sp_write_reg(TX_P0, SP_TX_AUX_CTRL_REG2, c);
	}
}

static void sp_tx_load_packet(enum PACKETS_TYPE type)
{
	int i;
	unchar c;

	switch (type) {
	case AVI_PACKETS:
		sp_write_reg(TX_P2, SP_TX_AVI_TYPE, 0x82);
		sp_write_reg(TX_P2, SP_TX_AVI_VER, 0x02);
		sp_write_reg(TX_P2, SP_TX_AVI_LEN, 0x0d);

		for (i = 0; i < 13; i++) {
			sp_write_reg(TX_P2, SP_TX_AVI_DB0 + i,
					sp_tx_packet_avi.AVI_data[i]);
		}

		break;

	case SPD_PACKETS:
		sp_write_reg(TX_P2, SP_TX_SPD_TYPE, 0x83);
		sp_write_reg(TX_P2, SP_TX_SPD_VER, 0x01);
		sp_write_reg(TX_P2, SP_TX_SPD_LEN, 0x19);

		for (i = 0; i < 25; i++) {
			sp_write_reg(TX_P2, SP_TX_SPD_DB0 + i,
					sp_tx_packet_spd.SPD_data[i]);
		}

		break;

	case VSI_PACKETS:
		sp_write_reg(TX_P2, SP_TX_MPEG_TYPE, 0x81);
		sp_write_reg(TX_P2, SP_TX_MPEG_VER, 0x01);
		sp_read_reg(RX_P1, HDMI_RX_MPEG_LEN_REG, &c);
		sp_write_reg(TX_P2, SP_TX_MPEG_LEN, c);

		for (i = 0; i < 10; i++) {
			sp_write_reg(TX_P2, SP_TX_MPEG_DB0 + i,
					sp_tx_packet_mpeg.MPEG_data[i]);
		}

		break;
	case MPEG_PACKETS:
		sp_write_reg(TX_P2, SP_TX_MPEG_TYPE, 0x85);
		sp_write_reg(TX_P2, SP_TX_MPEG_VER, 0x01);
		sp_write_reg(TX_P2, SP_TX_MPEG_LEN, 0x0d);

		for (i = 0; i < 10; i++) {
			sp_write_reg(TX_P2, SP_TX_MPEG_DB0 + i,
					sp_tx_packet_mpeg.MPEG_data[i]);
		}

		break;
	case AUDIF_PACKETS:
		sp_write_reg(TX_P2, SP_TX_AUD_TYPE, 0x84);
		sp_write_reg(TX_P2, SP_TX_AUD_VER, 0x01);
		sp_write_reg(TX_P2, SP_TX_AUD_LEN, 0x0a);
		for (i = 0; i < 10; i++) {
			sp_write_reg(TX_P2, SP_TX_AUD_DB0 + i,
					sp_tx_audioinfoframe.pb_byte[i]);
		}

		break;

	default:
		break;
	}
}


void sp_tx_config_packets(enum PACKETS_TYPE bType)
{
	unchar c;

	switch (bType) {
	case AVI_PACKETS:

		sp_read_reg(TX_P0, SP_TX_PKT_EN_REG, &c);
		c &= ~AVI_IF_EN;
		sp_write_reg(TX_P0, SP_TX_PKT_EN_REG, c);
		sp_tx_load_packet(AVI_PACKETS);

		sp_read_reg(TX_P0, SP_TX_PKT_EN_REG, &c);
		c |= AVI_IF_UD;
		sp_write_reg(TX_P0, SP_TX_PKT_EN_REG, c);

		sp_read_reg(TX_P0, SP_TX_PKT_EN_REG, &c);
		c |= AVI_IF_EN;
		sp_write_reg(TX_P0, SP_TX_PKT_EN_REG, c);

		break;

	case SPD_PACKETS:
		sp_read_reg(TX_P0, SP_TX_PKT_EN_REG, &c);
		c &= ~SPD_IF_EN;
		sp_write_reg(TX_P0, SP_TX_PKT_EN_REG, c);
		sp_tx_load_packet(SPD_PACKETS);

		sp_read_reg(TX_P0, SP_TX_PKT_EN_REG, &c);
		c |= SPD_IF_UD;
		sp_write_reg(TX_P0, SP_TX_PKT_EN_REG, c);

		sp_read_reg(TX_P0, SP_TX_PKT_EN_REG, &c);
		c |=  SPD_IF_EN;
		sp_write_reg(TX_P0, SP_TX_PKT_EN_REG, c);

		break;

	case VSI_PACKETS:
		sp_read_reg(TX_P0, SP_TX_PKT_EN_REG, &c);
		c &= ~MPEG_IF_EN;
		sp_write_reg(TX_P0, SP_TX_PKT_EN_REG, c);

		sp_tx_load_packet(VSI_PACKETS);

		sp_read_reg(TX_P0, SP_TX_PKT_EN_REG, &c);
		c |= MPEG_IF_UD;
		sp_write_reg(TX_P0, SP_TX_PKT_EN_REG, c);

		sp_read_reg(TX_P0, SP_TX_PKT_EN_REG, &c);
		c |= MPEG_IF_EN;
		sp_write_reg(TX_P0, SP_TX_PKT_EN_REG, c);

		break;
	case MPEG_PACKETS:
		sp_read_reg(TX_P0, SP_TX_PKT_EN_REG, &c);
		c &= ~MPEG_IF_EN;
		sp_write_reg(TX_P0, SP_TX_PKT_EN_REG, c);


		sp_tx_load_packet(MPEG_PACKETS);

		sp_read_reg(TX_P0, SP_TX_PKT_EN_REG, &c);
		c |= MPEG_IF_UD;
		sp_write_reg(TX_P0, SP_TX_PKT_EN_REG, c);

		sp_read_reg(TX_P0, SP_TX_PKT_EN_REG, &c);
		c |= MPEG_IF_EN;
		sp_write_reg(TX_P0, SP_TX_PKT_EN_REG, c);
		break;
	case AUDIF_PACKETS:
		sp_read_reg(TX_P0, SP_TX_PKT_EN_REG, &c);
		c &= ~AUD_IF_EN;
		sp_write_reg(TX_P0, SP_TX_PKT_EN_REG, c);


		sp_tx_load_packet(AUDIF_PACKETS);

		sp_read_reg(TX_P0, SP_TX_PKT_EN_REG, &c);
		c |= AUD_IF_UP;
		sp_write_reg(TX_P0, SP_TX_PKT_EN_REG, c);

		sp_read_reg(TX_P0, SP_TX_PKT_EN_REG, &c);
		c |= AUD_IF_EN;
		sp_write_reg(TX_P0, SP_TX_PKT_EN_REG, c);

		break;

	default:
		break;
	}

}


void sp_tx_avi_setup(void)
{
	unchar c;
	int i;
	for (i = 0; i < 13; i++) {
		sp_read_reg(RX_P1, (HDMI_RX_AVI_DATA00_REG + i), &c);
		sp_tx_packet_avi.AVI_data[i] = c;
	}
//ANX +++: (ver:20130105)	
	switch(sp_tx_rx_type){
	case RX_VGA:
		sp_tx_packet_avi.AVI_data[0] &=~0x60;
		break;
	case RX_HDMI:
	case RX_DP:
		break;
	default:
		break;
	}
//ANX ---: (ver:20130105)
}


static unchar sp_tx_bw_lc_sel(void)
{
	unchar over_bw = 0;
	uint pixel_clk = 0;
	enum HDMI_color_depth hdmi_input_color_depth = Hdmi_legacy;
	unchar c;

	DEV_DBG("input pclk = %d\n", (unsigned int)pclk);

	sp_read_reg(RX_P0, HDMI_RX_VIDEO_STATUS_REG1, &c);
	c &= COLOR_DEPTH;

	if (c == 0x00)
		hdmi_input_color_depth = Hdmi_legacy;
	else if (c == 0x40)
		hdmi_input_color_depth = Hdmi_24bit;
	else if (c == 0x50)
		hdmi_input_color_depth = Hdmi_30bit;
	else if (c == 0x60)
		hdmi_input_color_depth = Hdmi_36bit;
	else
		pr_warn("HDMI input color depth is not supported .\n");

	switch (hdmi_input_color_depth) {
	case Hdmi_legacy:
	case Hdmi_24bit:
		pixel_clk = pclk;
		break;
	case Hdmi_30bit:
		pixel_clk = pclk * 5 / 4;
		break;
	case Hdmi_36bit:
		pixel_clk = pclk * 3 / 2;
		break;
	default:
		break;
	}

	if (pixel_clk <= 54) {
		sp_tx_bw = BW_162G;
		over_bw = 0;
	} else if ((54 < pixel_clk) && (pixel_clk <= 90)) {
		if (sp_tx_bw >= BW_27G) {
			sp_tx_bw = BW_27G;
			over_bw = 0;
		} else
			over_bw = 1;
	} else if ((90 < pixel_clk) && (pixel_clk <= 180)) {
		if (sp_tx_bw >= BW_54G) {
			sp_tx_bw = BW_54G;
			over_bw = 0;
		} else
			over_bw = 1;
	} else
		over_bw = 1;

	if (over_bw)
		DEV_ERR("over bw!\n");
	else
		DEV_NOTICE("The optimized BW =%.2x\n", sp_tx_bw);

	return over_bw;

}
extern int g_swing_value;
extern int g_pre_emphis_value;
extern int g_force_swing_value;
extern int myDP_DP_Dongle;

unchar sp_tx_hw_link_training(void)
{

	unchar c;

	if (!sp_tx_hw_lt_enable) {

		DEV_NOTICE("Hardware link training");
		if (!sp_tx_get_pll_lock_status()) {
			DEV_ERR("PLL not lock!");
			return 1;
		}
		sp_write_reg(TX_P0, SP_TX_LINK_BW_SET_REG, sp_tx_bw);
		sp_tx_enhancemode_set();

	sp_tx_aux_dpcdread_bytes(0x00, 0x06, 0x00, 0x01, &c);
	c |= 0x01;
	sp_tx_aux_dpcdwrite_byte(0x00, 0x06, 0x00, c);

//ANX +++: (ver:20130105)
		sp_read_reg(TX_P2, SP_INT_MASK, &c);
		sp_write_reg(TX_P2, SP_INT_MASK, c |0X20);
//ANX ---: (ver:20130105)
		sp_write_reg(TX_P0, SP_TX_LT_CTRL_REG, SP_TX_LT_EN);

		sp_tx_hw_lt_enable = 1;
		return 1;

	}


	if (sp_tx_hw_lt_done) {
		sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x02, 1, bytebuf);

		DEV_DBG(" LANE0_1_STATUS = 0x%x\n", bytebuf[0]);
		if ((bytebuf[0] & 0x07) != 0x07) {
			sp_tx_hw_lt_enable = 0;
			sp_tx_hw_lt_done = 0;
			return 1;
		} else {
			sp_tx_hw_lt_done = 1;

			if (!sp_tx_test_lt) {
				/*In link cts4.3.1.9, need to link training
				from the lowest swing, so swing adjust
				needs to be moved here*/
				sp_write_reg(TX_P0, SP_TX_LT_SET_REG, 0x0a);
				if (sp_tx_link_err_check()) {
					c = PRE_EMP_LEVEL1
						| DRVIE_CURRENT_LEVEL1;
					sp_write_reg(TX_P0,
						SP_TX_LT_SET_REG, c);

					if (sp_tx_link_err_check())
						sp_write_reg(TX_P0,
						SP_TX_LT_SET_REG, 0x0a);
				}
				c = 0;
				sp_read_reg(TX_P0, SP_TX_LT_SET_REG, &c);
				DEV_DBG("after link_training , swing = %d\n", c);

			sp_read_reg(TX_P0, SP_TX_LINK_BW_SET_REG, &c);
			if (c != sp_tx_bw) {
				sp_tx_hw_lt_done = 0;
				sp_tx_hw_lt_enable = 0;
				return 1;
			}
			}
			sp_tx_set_sys_state(STATE_CONFIG_OUTPUT);
			return 0;
		}
	}

	return 1;
}

uint sp_tx_link_err_check(void)
{
	uint errl = 0, errh = 0;

	sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x10, 2, bytebuf);
	msleep(5);
	sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x10, 2, bytebuf);
	errh = bytebuf[1];

	if (errh & 0x80) {
		errl = bytebuf[0];
		errh = (errh & 0x7f) << 8;
		errl = errh + errl;
	}

	DEV_ERR(" Err of Lane = %d, swing = %d\n", errl, g_swing_value);
    if (errl > 5){
		ASUSEvtlog(" Err of Lane = %d, swing = %d\n", errl, g_swing_value);
    }	
	return errl;
}

uint sp_tx_link_err_check_1(void)
{
	uint errl = 0, errh = 0;

	sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x10, 2, bytebuf);
	msleep(5);
	sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x10, 2, bytebuf);
	errh = bytebuf[1];

	if (errh & 0x80) {
		errl = bytebuf[0];
		errh = (errh & 0x7f) << 8;
		errl = errh + errl;
	}

       if (errl){
		DEV_ERR(" ## Err of Lane = %d\n", errl);
		ASUSEvtlog(" ## Err of Lane = %d\n", errl);
       }
	   
	return errl;
}

extern int g_myDP_SSC_on;
unchar sp_tx_lt_pre_config(void)
{
	unchar c;
	unchar link_bw = 0;


	if (!sp_tx_link_config_done) {
		sp_tx_get_rx_bw(1, &c);
		switch (c) {
			case 0x06:
				sp_tx_bw=BW_162G;
				break;
			case 0x0a:
				sp_tx_bw=BW_27G;
				break;
			case 0x14:
				sp_tx_bw=BW_54G;
				break;
			default:
				sp_tx_bw=BW_54G;
				break;
		}


		if ((sp_tx_bw != BW_27G) && (sp_tx_bw != BW_162G)
			&& (sp_tx_bw != BW_54G))
			return 1;

		sp_tx_power_on(SP_TX_PWR_VIDEO);
		sp_tx_video_mute(1);

		sp_tx_enable_video_input(1);
		sp_read_reg(TX_P0, SP_TX_SYS_CTRL2_REG, &c);
		sp_write_reg(TX_P0, SP_TX_SYS_CTRL2_REG, c);
		sp_read_reg(TX_P0, SP_TX_SYS_CTRL2_REG, &c);

		if (c & CHA_STA) {			DEV_ERR("Stream clock not stable!\n");
			return 1;
		}

		sp_read_reg(TX_P0, SP_TX_SYS_CTRL3_REG, &c);
		sp_write_reg(TX_P0, SP_TX_SYS_CTRL3_REG, c);
		sp_read_reg(TX_P0, SP_TX_SYS_CTRL3_REG, &c);

		if (!(c & STRM_VALID)) {			DEV_ERR("video stream not valid!\n");
			return 1;
		}

		sp_write_reg(TX_P0, SP_TX_LINK_BW_SET_REG, 0x14);
		sp_tx_get_link_bw(&link_bw);
		sp_tx_pclk_calc(&link_bw);

		if (sp_tx_test_lt) {
			sp_tx_bw = sp_tx_test_bw;
			sp_tx_test_lt = 0;
			/*Link CTS 4.3.3.1, need to send the video timing
			640x480p@60Hz, 18-bit*/
			sp_read_reg(TX_P2, SP_TX_VID_CTRL2_REG, &c);
			c = (c & 0x8f);
			sp_write_reg(TX_P2, SP_TX_VID_CTRL2_REG, c);
		} else {
			/* Optimize the LT to get minimum power consumption */
			if (sp_tx_bw_lc_sel()) {
				DEV_ERR("****Over bandwidth****\n");
				return 1;
			}
		}

		/*Diable video before link training to enable idle pattern*/
		sp_tx_enable_video_input(0);
//#ifdef SSC_EN
		if (g_myDP_SSC_on)
			sp_tx_config_ssc(sp_tx_bw);
//#else
		else
			sp_tx_spread_enable(0);
//#endif

		sp_read_reg(TX_P0, SP_TX_ANALOG_PD_REG, &c);
		c |= CH0_PD;
		sp_write_reg(TX_P0, SP_TX_ANALOG_PD_REG, c);
		msleep(1);
		c &= ~CH0_PD;
		sp_write_reg(TX_P0, SP_TX_ANALOG_PD_REG, c);
		
		sp_read_reg(TX_P0, SP_TX_PLL_CTRL_REG, &c);
		c |= PLL_RST;
		sp_write_reg(TX_P0, SP_TX_PLL_CTRL_REG, c);
	       msleep(1);
		c &=~PLL_RST;
		sp_write_reg(TX_P0, SP_TX_PLL_CTRL_REG, c);
		sp_tx_link_config_done = 1;
	}

	return 0;
}

void sp_tx_video_mute(unchar enable)
{
	unchar c;

	if (enable) {
		sp_read_reg(TX_P2, SP_TX_VID_CTRL1_REG, &c);
		c |= VIDEO_MUTE;
		sp_write_reg(TX_P2, SP_TX_VID_CTRL1_REG, c);
	} else {
		sp_read_reg(TX_P2, SP_TX_VID_CTRL1_REG, &c);
		c &= ~VIDEO_MUTE;
		sp_write_reg(TX_P2, SP_TX_VID_CTRL1_REG, c);
	}

}

//+++ ASUS BSP Bernard: ANX 4.0
void sp_tx_aux_polling_enable(bool benable)
{
	unchar c;

	sp_read_reg(TX_P2, SP_TX_DEBUG_REG1, &c);
	if (benable)
		c |= POLLING_EN;
	else
		c &= ~POLLING_EN;

	sp_write_reg(TX_P2, SP_TX_DEBUG_REG1, c);
}
//--- ASUS BSP Bernard: ANX 4.0

void sp_tx_send_message(enum SP_TX_SEND_MSG message)
{
	unchar c;

	switch (message) {
	case MSG_OCM_EN:
		sp_tx_aux_dpcdwrite_byte(0x00, 0x05, 0x25, 0x5a);
		break;

	case MSG_INPUT_HDMI:
		sp_tx_aux_dpcdwrite_byte(0x00, 0x05, 0x26, 0x01);
		break;

	case MSG_INPUT_DVI:
		sp_tx_aux_dpcdwrite_byte(0x00, 0x05, 0x26, 0x00);
		break;

	case MSG_CLEAR_IRQ:
		sp_tx_aux_dpcdread_bytes(0x00, 0x04, 0x10, 1, &c);
		c |= 0x01;
		sp_tx_aux_dpcdwrite_byte(0x00, 0x04, 0x10, c);
		break;
	}

}
//ANX +++: (ver:20130105)
#define SOURCE_AUX_OK	1
#define SOURCE_AUX_ERR	0
#define SOURCE_REG_OK	1
#define SOURCE_REG_ERR	0

#define SINK_DEV_SEL  0x005f0
#define SINK_ACC_OFS  0x005f1
#define SINK_ACC_REG  0x005f2

static bool source_aux_read_7730dpcd(long addr,unchar cCount,unchar * pBuf)
{
    unchar c;
    unchar addr_l;
    unchar addr_m;
    unchar addr_h;
    addr_l = (unchar)addr;
    addr_m = (unchar)(addr>>8);
    addr_h = (unchar)(addr>>16);
    c = 0;
    while (1) {
        if (sp_tx_aux_dpcdread_bytes(addr_h,addr_m,addr_l,cCount,pBuf) == AUX_OK)
            return SOURCE_AUX_OK;
        c++;
        if (c > 3) {
            return SOURCE_AUX_ERR;
        }
    }
}

static bool source_aux_write_7730dpcd(long addr,unchar cCount,unchar * pBuf)
{
    unchar c;
    unchar addr_l;
    unchar addr_m;
    unchar addr_h;
    addr_l = (unchar)addr;
    addr_m = (unchar)(addr>>8);
    addr_h = (unchar)(addr>>16);
    c = 0;
    while (1) {
        if (sp_tx_aux_dpcdwrite_bytes(addr_h,addr_m,addr_l,cCount,pBuf) == AUX_OK)
            return SOURCE_AUX_OK;
        c++;
        if (c > 3) {
            return SOURCE_AUX_ERR;
        }
    }
}

bool i2c_master_read_reg(unchar Sink_device_sel, unchar offset, unchar * Buf)
{
    unchar sbytebuf[2]= {0};
    long a0, a1;

    a0 = SINK_DEV_SEL;
    a1 = SINK_ACC_REG;
    sbytebuf[0] = Sink_device_sel;
    sbytebuf[1] = offset;

    if(source_aux_write_7730dpcd(a0,2,sbytebuf) == SOURCE_AUX_OK)
    {
    	if(source_aux_read_7730dpcd(a1,1,Buf)==SOURCE_AUX_OK)
			return SOURCE_REG_OK;
    }

    return SOURCE_REG_ERR;
}

bool i2c_master_write_reg(unchar Sink_device_sel,unchar offset, unchar value)
{
    unchar sbytebuf[3]= {0};//, c;
    long a0;

    a0 = SINK_DEV_SEL;
    sbytebuf[0] = Sink_device_sel;
    sbytebuf[1] = offset;
    sbytebuf[2] = value;

    if(source_aux_write_7730dpcd(a0,3,sbytebuf) == SOURCE_AUX_OK)
		return SOURCE_REG_OK;
    else
		return SOURCE_REG_ERR;
}

//ANX ---: (ver:20130105)

unchar sp_tx_get_cable_type(void)
{
	unchar SINK_OUI[8] = { 0 };
	unchar ds_port_preset = 0;
	unchar ds_port_recoginze = 0;
	int i,j;
//ANX +++: (ver:20130105)
	unchar c, ocm_status;
//ANX ---: (ver:20130105)

//ANX +++: (ver:20130105) pad solution				
	if(sp_tx_asus_pad) {

		if (!myDP_DP_Dongle)
		{
		DEV_NOTICE("Downstream is ASUS Pad HDMI");
		sp_tx_send_message(MSG_OCM_EN);
		sp_tx_rx_type = RX_HDMI;

#if 0  // for A80 flashing issue			
//asus bsp: tune hdmi rx eq_boost				
		sp_read_reg(RX_P0, HDMI_RX_TMDS_CTRL_REG1, &c);
		printk("[myDP] before hdmi RX HDMI_RX_TMDS_CTRL_REG1 = 0x%x\n", c);
		c &= ~0x70;
		printk("[myDP] write hdmi RX HDMI_RX_TMDS_CTRL_REG1 = 0x%x\n", c);
		sp_write_reg(RX_P0, HDMI_RX_TMDS_CTRL_REG1, c);

		sp_read_reg(RX_P0, HDMI_RX_TMDS_CTRL_REG1, &c);
		printk("[myDP] after hdmi RX HDMI_RX_TMDS_CTRL_REG1 = 0x%x\n", c);
#endif
	
		/*disable ANX7730 OCM*/
		i2c_master_read_reg(7,0xf0, &c);
		ocm_status = c;
		i2c_master_write_reg(7,0xf0, 0);
		i2c_master_read_reg(7,0xf0, &c);
		if(c != 0 )
		 i2c_master_write_reg(7,0xf0, 0);
		
		/*disable ANX7730 HDCP auto enable*/
		i2c_master_read_reg(7,0x04, &c);
		c &=~0xf0;
		i2c_master_write_reg(7,0x04, c);
		/*disable ANX7730 HDCP mismatch*/
		i2c_master_read_reg(7,0x1d, &c);
		c &=~0xc0;
		i2c_master_write_reg(7,0x1d, c);

		i2c_master_write_reg(7,0xf0, 0xe6);
	      i2c_master_write_reg(7,0xf0, 0xe6);

		ds_port_recoginze = 1;

		ASUS_DEV_INFO("--- Downstream ASUS Pad HDMI ---\n");
		}
		else
		{
		DEV_NOTICE("Downstream is ASUS Pad DP\n");
		sp_tx_rx_type = RX_DP;		
		ds_port_recoginze = 1;

		ASUS_DEV_INFO("--- Downstream ASUS Pad DP ---\n");
		
		}
		if (ds_port_recoginze)
			return 1;
	}
//ANX ---: (ver:20130105) pad solution

	for (i = 0; i < 5; i++) {
		if (AUX_ERR == sp_tx_aux_dpcdread_bytes(0x00, 0x00,
			DPCD_DSPORT_PRESENT, 1, &ds_port_preset)) {
			DEV_ERR(" AUX access error");
			/*Add time delay for VGA dongle bootup*/
			msleep(250);
			continue;
		}

		for (j = 0; j < 0x0c; j++)
			sp_tx_aux_dpcdread_bytes(0x00, 0x00, j, 1, bytebuf);


		DEV_DBG("ds_port_preset = %x\n", ds_port_preset);

		
		switch (ds_port_preset & 0x07) {
		case 0x00:
			sp_tx_rx_type = RX_DP;
			ds_port_recoginze = 1;
			DEV_NOTICE("Downstream is DP dongle.");
			break;
		case 0x03:
			sp_tx_aux_dpcdread_bytes(0x00, 0x04, 0x00, 8, SINK_OUI);

			if (((SINK_OUI[0] == 0x00) && (SINK_OUI[1] == 0x22)
			    && (SINK_OUI[2] == 0xb9) && (SINK_OUI[3] == 0x61)
			    && (SINK_OUI[4] == 0x39) && (SINK_OUI[5] == 0x38)
			    && (SINK_OUI[6] == 0x33))||
			    ((SINK_OUI[0] == 0x00) && (SINK_OUI[1] == 0x22)
			    && (SINK_OUI[2] == 0xb9) && (SINK_OUI[3] == 0x73)
			    && (SINK_OUI[4] == 0x69) && (SINK_OUI[5] == 0x76)
			    && (SINK_OUI[6] == 0x61))) {
				DEV_NOTICE("Downstream is VGA dongle.");
				sp_tx_rx_type = RX_VGA;
			} else {
				sp_tx_rx_type = RX_DP;
				DEV_NOTICE("Downstream is general DP2VGA converter.");
			}
			ds_port_recoginze = 1;
			break;
		case 0x05:
			sp_tx_aux_dpcdread_bytes(0x00, 0x04, 0x00, 8, SINK_OUI);

			if ((SINK_OUI[0] == 0xb9) && (SINK_OUI[1] == 0x22)
			    && (SINK_OUI[2] == 0x00)
			    && (SINK_OUI[3] == 0x00) && (SINK_OUI[4] == 0x00)
			    && (SINK_OUI[5] == 0x00)
			    && (SINK_OUI[6] == 0x00)) {
				DEV_NOTICE("Downstream is HDMI dongle.");
				sp_tx_send_message(MSG_OCM_EN);
				sp_tx_rx_type = RX_HDMI;
			} else {
				sp_tx_rx_type = RX_DP;
				DEV_NOTICE("Downstream is general DP2HDMI converter.");
			}
			ds_port_recoginze = 1;
			break;
		default:
			DEV_ERR("Downstream can not recognized.");
			sp_tx_rx_type = RX_NULL;
			ds_port_recoginze = 0;
			break;

		}

		if (ds_port_recoginze)
			return 1;

	}

	return 0;
}

bool sp_tx_get_hdmi_connection(void)
{
	unchar c;
//ASUS BSP +++ : larry lai for pad solution
	if (!sp_tx_asus_pad)
	{
		msleep(200);
	}
	else
	{
		msleep(20);	
	}
//ASUS BSP --- : larry lai for pad solution
	
	sp_tx_aux_dpcdread_bytes(0x00, 0x05, 0x18, 1, &c);
	if ((c & 0x41) == 0x41) 
		return TRUE;
	else
		return FALSE;

}
//ANX +++: (ver:20130105) pad solution
//+++ ASUS BSP Bernard: for mydp mini-porting
#ifndef CONFIG_ASUS_HDMI
bool MyDP_asus_padstation_exist_realtime(void)
{
	return gpio_get_value_cansleep(75);
}
#else

extern bool asus_padstation_exist_realtime(void);

bool MyDP_asus_padstation_exist_realtime(void)
{
	return asus_padstation_exist_realtime();
}
#endif
//+++ ASUS BSP Bernard: for mydp mini-porting
extern int myDP_TV_mode;
extern int myDP_force_pad_mode;

bool sp_tx_get_asus_id(void)
{
   bool ret = 0;
 /*	 Need to be filled by ASUS.
	 When the pad is plugged, this function should return true,
	 otherwise it should return false.
 */
 	int try_count=5;
 	if (myDP_force_pad_mode)
 	{
		ASUS_DEV_WARN("force pad mode\n");
 		return 1;
 	}
 	if (!myDP_TV_mode)
 	{

			while(try_count){
				ret = MyDP_asus_padstation_exist_realtime();
				ASUS_DEV_WARN("check Pad status = (%d)\n", ret);
				if(ret==1)
					return ret;
				else
					msleep(200);
				try_count--;
			}
			return ret;
 	}
	else
	{
		ASUS_DEV_WARN("TV mode\n");
		return 0;
	}
}
//ANX ---: (ver:20130105) pad solution

bool sp_tx_get_vga_connection(void)
{
	unchar c;
	sp_tx_aux_dpcdread_bytes(0x00, 0x02, DPCD_SINK_COUNT, 1, &c);
	if (c & 0x01)
		return TRUE;
	else
		return FALSE;
}

static bool sp_tx_get_ds_video_status(void)
{
	unchar c;
	sp_tx_aux_dpcdread_bytes(0x00, 0x05, 0x27, 1, &c);
	if (c & 0x01)
		return TRUE;
	else
		return FALSE;
}
extern int resume_trigger;
bool sp_tx_get_dp_connection(void)
{
	unchar c;

//ASUS BSP +++ : larry lai for pad solution
	if(sp_tx_asus_pad) 
	{
		ASUS_DEV_INFO("sp_tx_get_dp_connection (ignore DP connection check)\n");

		msleep(50);
//=============================================================================
//ASUS BSP +++ : larry#20140106 speed up suspend/resume timing in pad
#if 1 //debug DP sink count status, can ignore these code
        	sp_tx_aux_dpcdread_bytes(0x00, 0x02, DPCD_SINK_COUNT, 1, &c);
			DEV_DBG("SINK count %x\n", c);
	
            sp_tx_aux_dpcdread_bytes(0x00, 0x00, 0x04, 1, &c);
			DEV_DBG("DP SINK charging feature %x\n", c);
            if (c & 0x20)
               sp_tx_aux_dpcdwrite_byte(0x00, 0x06, 0x00, 0x20);
#endif
            ASUS_DEV_INFO("sp_tx_get_dp_connection by pass check sink count\n");
			return TRUE;															

//ASUS BSP --- : larry#20140106 speed up suspend/resume timing in pad
//=============================================================================

	}
//ASUS BSP --- : larry lai for pad solution

	sp_tx_aux_dpcdread_bytes(0x00, 0x02, DPCD_SINK_COUNT, 1, &c);
	if (c & 0x1f) {
		sp_tx_aux_dpcdread_bytes(0x00, 0x00, 0x04, 1, &c);
		if (c & 0x20)
			sp_tx_aux_dpcdwrite_byte(0x00, 0x06, 0x00, 0x20);

		DEV_DBG("sp_tx_get_dp_connection true\n");
		return TRUE;
	} else {	
		DEV_DBG("sp_tx_get_dp_connection false\n");
		return FALSE;
		}
}

void sp_tx_edid_read(void)
{
	unchar i, j, edid_block = 0, segment = 0, offset = 0;
	unchar c;
	/*Add bandwidth check to support low 
	resolution for VGA and myDP monitor*/
	sp_tx_get_rx_bw(1, &c);
	slimport_link_bw = c;

	sp_tx_edid_read_initial();

	bedid_break = 0;


	sp_tx_addronly_set(1);
	sp_write_reg(TX_P0, SP_TX_AUX_CTRL_REG, 0x04);
	//sp_write_reg(TX_P0, SP_TX_AUX_CTRL_REG2, 0x01);
	sp_write_reg(TX_P0, SP_TX_AUX_CTRL_REG2, 0x03); //+++ ASUS BSP Bernard: ANX 4.0
	sp_tx_wait_aux_finished();

	edid_block = sp_tx_get_edid_block();

	if (edid_block < 2) {
		edid_block = 8 * (edid_block + 1);
		for (i = 0; i < edid_block; i++) {
			if (!bedid_break)
				sp_tx_aux_edidread_byte(i * 16);
			msleep(10);
		}

		sp_tx_addronly_set(0);

	} else {

		for (i = 0; i < 16; i++) {
			if (!bedid_break)
				sp_tx_aux_edidread_byte(i * 16);
		}

		sp_tx_addronly_set(0);

		if (!bedid_break) {
			edid_block = (edid_block + 1);

			for (i = 0; i < ((edid_block - 1) / 2); i++) {
				DEV_DBG("EXT 256 EDID block");
				segment = i + 1;

				for (j = 0; j < 16; j++) {
					sp_tx_parse_segments_edid(segment,
								  offset);
					offset = offset + 0x10;
				}
			}

			if (edid_block % 2) {
				DEV_DBG("Last block");
				segment = segment + 1;

				for (j = 0; j < 8; j++) {
					sp_tx_parse_segments_edid(segment,
								  offset);
					offset = offset + 0x10;
				}
			}
		}
	}

	sp_tx_addronly_set(0);
	sp_tx_rst_aux();

	sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x18,1,bytebuf);
	if (bytebuf[0] & 0x04) {

		DEV_DBG("check sum = %.2x\n",  (uint)bedid_checksum);
		bytebuf[0] = bedid_checksum;
		sp_tx_aux_dpcdwrite_bytes(0x00, 0x02, 0x61, 1, bytebuf);
		
		bytebuf[0] = 0x04;
		sp_tx_aux_dpcdwrite_bytes(0x00, 0x02, 0x60, 1, bytebuf);
		DEV_DBG("Test EDID done\n");

	}
	/*Link CTS4.3.1.1, link training needs to be
	fast right after EDID reading*/
	sp_tx_get_rx_bw(1, &c);
	sp_write_reg(TX_P0, SP_TX_LINK_BW_SET_REG, c);
	sp_tx_enhancemode_set();
	sp_write_reg(TX_P0, SP_TX_LT_CTRL_REG, SP_TX_LT_EN);
	/*Release the HPD after the EEPROM loaddown*/
	for(i=0; i < 10; i++) {
		sp_read_reg(TX_P0, SP_TX_HDCP_KEY_STATUS, &c);
		if((c&0x07) == 0x05)
			return;
		else
			msleep(10);
	}
}

static void sp_tx_pll_changed_int_handler(void)
{

	if (sp_tx_system_state > STATE_PARSE_EDID) {
		if (!sp_tx_get_pll_lock_status()) {
			DEV_ERR("PLL:_______________PLL not lock!");
			sp_tx_clean_hdcp();
			sp_tx_set_sys_state(STATE_LINK_TRAINING);
			sp_tx_link_config_done = 0;
			sp_tx_hw_lt_done = 0;
			sp_tx_hw_lt_enable = 0;
		}
	}
}


static void sp_tx_auth_done_int_handler(void)
{
	unchar c;

	sp_read_reg(TX_P0, SP_TX_HDCP_STATUS, &c);
	DEV_DBG("Register 0x70:0x00:%x",c);
	if (c & SP_TX_HDCP_AUTH_PASS) {
		sp_tx_aux_dpcdread_bytes(0x06, 0x80, 0x2A, 2, bytebuf);
		if (bytebuf[1] & 0x08) {
			/* max cascade read, fail */
			sp_tx_video_mute(1);
			sp_tx_clean_hdcp();
			DEV_ERR("Re-authentication!");
		} else {
			DEV_NOTICE("Authentication pass in Auth_Done");
			sp_tx_hdcp_auth_pass = 1;
			sp_tx_hdcp_auth_fail_counter = 0;
		}

	} else {
		DEV_ERR("Authentication failed in AUTH_done");
		sp_tx_hdcp_auth_pass = 0;
		sp_tx_hdcp_auth_fail_counter++;

		if (sp_tx_hdcp_auth_fail_counter >= SP_TX_HDCP_FAIL_TH) {
			sp_tx_video_mute(1);
			sp_tx_clean_hdcp();
		} else {
			sp_tx_video_mute(1);
			sp_tx_clean_hdcp();
			DEV_ERR("Re-authentication!\n");

			if (sp_tx_system_state > STATE_CONFIG_OUTPUT) {
				sp_tx_set_sys_state
				    (STATE_HDCP_AUTH);
				return;
			}

		}
	}

	sp_tx_hdcp_auth_done = 1;
}

static void sp_tx_link_chk_fail_int_handler(void)
{
	if (sp_tx_system_state >= STATE_HDCP_AUTH) {
		sp_tx_set_sys_state(STATE_HDCP_AUTH);
		sp_tx_clean_hdcp();
		DEV_ERR("IRQ:____________HDCP Sync lost!");
	}
}



static void sp_tx_lt_done_int_handler(void)
{
	unchar c, c1;

	if ((sp_tx_hw_lt_done) || (sp_tx_system_state != STATE_LINK_TRAINING))
		return;

	sp_read_reg(TX_P0, SP_TX_LT_CTRL_REG, &c);
	if (c & 0x70) {
		c = (c & 0x70) >> 4;
		DEV_ERR("HW LT failed in interrupt,");
		DEV_ERR("ERR code = %.2x\n", (uint) c);

		sp_tx_link_config_done = 0;
		sp_tx_hw_lt_enable = 0;
		sp_tx_hw_lt_done = 0;
		sp_tx_set_sys_state(STATE_LINK_TRAINING);
		msleep(50);

	} else {

		sp_tx_hw_lt_done = 1;
		sp_read_reg(TX_P0, SP_TX_LT_SET_REG, &c);
		sp_read_reg(TX_P0, SP_TX_LINK_BW_SET_REG, &c1);
		DEV_NOTICE("HW LT succeed,LANE0_SET = %.2x,", (uint) c);
		DEV_NOTICE("link_bw = %.2x\n", (uint) c1);
	}


}

void pad_sp_tx_lt_done_int_handler(void)
{
	unchar c, c1;


	sp_read_reg(TX_P0, SP_TX_LT_CTRL_REG, &c);
	if (c & 0x70) {
		c = (c & 0x70) >> 4;
		DEV_ERR("HW LT failed in interrupt,");
		DEV_ERR("ERR code = %.2x\n", (uint) c);

//		sp_tx_link_config_done = 0;
//		sp_tx_hw_lt_enable = 0;
//		sp_tx_hw_lt_done = 0;
//		sp_tx_set_sys_state(STATE_LINK_TRAINING);
		msleep(100);
	} else {

//		sp_tx_hw_lt_done = 1;
		sp_read_reg(TX_P0, SP_TX_LT_SET_REG, &c);
		sp_read_reg(TX_P0, SP_TX_LINK_BW_SET_REG, &c1);
		DEV_NOTICE("HW LT succeed,LANE0_SET = %.2x,", (uint) c);
		DEV_NOTICE("link_bw = %.2x\n", (uint) c1);
		msleep(100);		
	}

}


static void sp_tx_link_change_int_handler(void)
{

	unchar lane0_1_status, sl_cr, al;

	if (sp_tx_system_state < STATE_CONFIG_OUTPUT)
		return;

	sp_tx_link_err_check();

	sp_tx_aux_dpcdread_bytes(0x00, 0x02, DPCD_LANE_ALIGN_UD, 1,
				 bytebuf);
	al = bytebuf[0];
	sp_tx_aux_dpcdread_bytes(0x00, 0x02, DPCD_LANE0_1_STATUS, 1, bytebuf);
	lane0_1_status = bytebuf[0];


	if (((lane0_1_status & 0x01) == 0) || ((lane0_1_status & 0x04) == 0))
		sl_cr = 0;
	else
		sl_cr = 1;
	if (((al & 0x01) == 0) || (sl_cr == 0)) {
		if ((al & 0x01) == 0)
			DEV_ERR("Lane align not done\n");

		if (sl_cr == 0)
			DEV_ERR("Lane clock recovery not done\n");
		sp_tx_get_cable_type();
		if(sp_tx_rx_type_backup !=  sp_tx_rx_type) {
			sp_tx_vbus_powerdown();
//ANX +++: (ver:0.2)			
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
//ANX : (ver:20130105) pad solution			
			sp_tx_asus_pad = 0;
			sp_tx_set_sys_state(STATE_CABLE_PLUG);
		} else if((sp_tx_system_state > STATE_LINK_TRAINING)
		    && sp_tx_link_config_done) {
			sp_tx_link_config_done = 0;
			sp_tx_set_sys_state(STATE_LINK_TRAINING);
			DEV_ERR("IRQ:____________re-LT request!");
		}
	}

}

#ifdef CONFIG_ASUS_HDMI
//ASUS BSP wei +++
int Is_HDMI_power_off=0;
extern void ASUS_HDMI_power_on(struct platform_device *pdev,int);
//ASUS BSP wei ---
#endif

static void sp_tx_polling_err_int_handler(void)
{
	unchar c;
	int i;


	if ((sp_tx_system_state < STATE_CABLE_PLUG) || sp_tx_pd_mode)
		return;

	for (i = 0; i < 5; i++) {
		sp_tx_aux_dpcdread_bytes(0x00, 0x00, 0x00, 1, &c);

		if (c == 0x11)
			return;

		msleep(2);
	}

	if (sp_tx_pd_mode == 0) {
		DEV_ERR("Cwire polling is corrupted,power down ANX7808.\n");
#ifdef CONFIG_ASUS_HDMI
		//ASUS BSP Wei +++
		if(sp_tx_asus_pad==1 && gpio_get_value(75)){
			ASUS_HDMI_power_on(g_anx7808->hdmi_pdev,0);
			Is_HDMI_power_off=1;
		}
		//ASUS BSP Wei ---
#endif
		sp_tx_clean_hdcp();
		sp_tx_vbus_powerdown();
//ANX +++: (ver:0.2)		
		sp_tx_pull_down_id(FALSE);
		sp_tx_power_down(SP_TX_PWR_TOTAL);
		sp_tx_power_down(SP_TX_PWR_REG);
		sp_tx_hardware_powerdown(anx7808_client);
		sp_tx_set_sys_state(STATE_CABLE_PLUG);
		sp_tx_pd_mode = 1;
		sp_tx_link_config_done = 0;
		sp_tx_hw_lt_enable = 0;
		sp_tx_hw_lt_done = 0;
		sp_tx_rx_type = RX_NULL;
		sp_tx_rx_type_backup = RX_NULL;
//ANX : (ver:20130105) pad solution		
		sp_tx_asus_pad = 0;
	}
}
//ASUS BSP wei +++
extern int g_MyDP_HDCP_FAIL_COUNT;
extern bool hdcp_enable;
void sp_tx_sw_error_power_down(void);
//ASUS BSP wei ---

//+++ ASUS BSP Bernard, for determining AC charger
static void (*notify_ac_charger_func_ptr)(int) = NULL;

void dp_ac_charger(bool enable)
{	
	g_b_SwitchChargerBootPlugin= enable;	
	if(NULL != notify_ac_charger_func_ptr){
		DEV_DBG("AC Charger Notification (%d)\n", enable);
		(*notify_ac_charger_func_ptr) (enable);
	}
}

int dp_registerChargerInOutNotification(void (*callback)(int))
{
	ASUS_DEV_WARN("%s +++\n",__FUNCTION__);

	notify_ac_charger_func_ptr = callback;
	return 0;
}
EXPORT_SYMBOL(dp_registerChargerInOutNotification);
//--- ASUS BSP Bernard, for determining AC charger

static void sp_tx_irq_isr(void)
{
	unchar c, c1, lane0_1_status, sl_cr, al;
	unchar IRQ_Vector, Int_vector1, Int_vector2;
	unchar test_vector;
	unchar fw_ver[MAX_BUF_CNT];

	sp_tx_aux_dpcdread_bytes(0x00, 0x02, DPCD_SERVICE_IRQ_VECTOR, 1,
		bytebuf);
	IRQ_Vector = bytebuf[0];
	sp_tx_aux_dpcdwrite_bytes(0x00, 0x02, DPCD_SERVICE_IRQ_VECTOR, 1,
		bytebuf);
	sp_tx_aux_dpcdread_bytes(0x00, 0x05, 0x23, 1, fw_ver);
	//DEV_DBG("ANX7730 BC current FW Ver %.2x \n", (uint)(fw_ver[0]&0x7f));

	//+++ ASUS BSP Bernard
	/*charging irq*/
	if ((IRQ_Vector & SINK_SPECIFIC_IRQ )&& (fw_ver[0]&0x7f) >= 0x15 ) {
		if (AUX_OK == sp_tx_aux_dpcdread_bytes
			(0x00, 0x05, 0x22, 1, &c1)) {
			if ((c1&0x01) == 0x00) {
				if(mydp_ac_charger_flag == 0){
					DEV_DBG("AC charging!\n");
					dp_ac_charger(true);
					mydp_ac_charger_flag = 1;
				}
			} 
			else {
				//DEV_DBG("Not AC charging!\n");
				mydp_ac_charger_flag = 0;
			}
		}
	}
	//--- ASUS BSP Bernard

	/* HDCP IRQ */
	if (IRQ_Vector & CP_IRQ) {
		if (sp_tx_hdcp_auth_pass) {
			sp_tx_aux_dpcdread_bytes(0x06, 0x80, 0x29, 1, &c1);
			if (c1 & 0x04) {
				if (sp_tx_system_state >=
					STATE_HDCP_AUTH) {
					sp_tx_clean_hdcp();
					sp_tx_set_sys_state
					    (STATE_HDCP_AUTH);
					DEV_ERR("IRQ:____________HDCP Sync lost!\n");
					g_MyDP_HDCP_FAIL_COUNT++;
					if(g_MyDP_HDCP_FAIL_COUNT>5){
						g_MyDP_HDCP_FAIL_COUNT=0;
						DEV_ERR("%s:MyDP_HDCP_SYNC_LOST ++++\n",__func__);
						ASUSEvtlog("%s:MyDP_HDCP_SYNC_LOST ++++\n",__func__);
						hdcp_enable=0;
					}
				}
			}
		}
	}

	/* specific int */
	if ((IRQ_Vector & SINK_SPECIFIC_IRQ) && (sp_tx_rx_type == RX_HDMI)) {


		sp_tx_aux_dpcdread_bytes(0x00, 0x05, DPCD_SPECIFIC_INTERRUPT1,
					 1, &Int_vector1);
		sp_tx_aux_dpcdwrite_byte(0x00, 0x05, DPCD_SPECIFIC_INTERRUPT1,
					 Int_vector1);

		sp_tx_aux_dpcdread_bytes(0x00, 0x05, DPCD_SPECIFIC_INTERRUPT2,
					 1, &Int_vector2);
		sp_tx_aux_dpcdwrite_byte(0x00, 0x05, DPCD_SPECIFIC_INTERRUPT2,
					 Int_vector2);


		if ((Int_vector1 & 0x01) == 0x01) {
			sp_tx_aux_dpcdread_bytes(0x00, 0x05, 0x18, 1, &c);
			if (c & 0x01)
				DEV_NOTICE("Downstream HDMI is pluged!\n");
		}

		if ((Int_vector1 & 0x02) == 0x02) {
			sp_tx_aux_dpcdread_bytes(0x00, 0x05, 0x18, 1, &c);
			if ((c & 0x01) != 0x01) {
				DEV_NOTICE("Downstream HDMI is unpluged!\n");

				if ((sp_tx_system_state >
				     STATE_CABLE_PLUG)
				    && (!sp_tx_pd_mode)) {
					sp_tx_clean_hdcp();
//ANX : (ver0.4)					
					sp_tx_pull_down_id(FALSE);					
					sp_tx_power_down(SP_TX_PWR_REG);
					sp_tx_power_down(SP_TX_PWR_TOTAL);
					sp_tx_hardware_powerdown(anx7808_client);
					sp_tx_set_sys_state(STATE_CABLE_PLUG);
					sp_tx_pd_mode = 1;
					sp_tx_link_config_done = 0;
					sp_tx_hw_lt_done = 0;
					sp_tx_hw_lt_enable = 0;
//ANX : (ver:20130105) pad solution					
					sp_tx_asus_pad = 0;
					return;
				}
			}
		}

		if (((Int_vector1 & 0x04) == 0x04)
		    && (sp_tx_system_state > STATE_CONFIG_OUTPUT)) {

			DEV_ERR("Rx specific  IRQ: Link is down!\n");

			sp_tx_aux_dpcdread_bytes(0x00, 0x02,
						 DPCD_LANE_ALIGN_UD,
						 1, bytebuf);
			al = bytebuf[0];

			sp_tx_aux_dpcdread_bytes(0x00, 0x02,
						 DPCD_LANE0_1_STATUS, 1,
						 bytebuf);
			lane0_1_status = bytebuf[0];

			if (((lane0_1_status & 0x01) == 0)
			    || ((lane0_1_status & 0x04) == 0))
				sl_cr = 0;
			else
				sl_cr = 1;

			if (((al & 0x01) == 0) || (sl_cr == 0)) {
				if ((al & 0x01) == 0)
					DEV_ERR("Lane align not done\n");

				if (sl_cr == 0)
					DEV_ERR("Lane clock recovery not done\n");

				sp_tx_get_cable_type();
				if(sp_tx_rx_type_backup !=  sp_tx_rx_type) {
					sp_tx_vbus_powerdown();
//ANX : (ver0.4)					
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
//ANX : (ver:20130105) pad solution					
					sp_tx_asus_pad = 0;
					sp_tx_set_sys_state(STATE_CABLE_PLUG);
					return;
				} else if ((sp_tx_system_state > STATE_LINK_TRAINING)
				    && sp_tx_link_config_done) {
					sp_tx_link_config_done = 0;
					sp_tx_hw_lt_enable = 0;
					sp_tx_hw_lt_done = 0;
					sp_tx_set_sys_state
						(STATE_LINK_TRAINING);
					DEV_ERR("IRQ:____________re-LT request!\n");
				}
			}

			sp_tx_aux_dpcdread_bytes(0x00, 0x05, 0x18, 1, &c);
			if (!(c & 0x40)) {
				if ((sp_tx_system_state > STATE_CABLE_PLUG)
				    && (!sp_tx_pd_mode)) {
					sp_tx_clean_hdcp();
//ANX +++: (ver0.4)
					sp_tx_vbus_powerdown();
					sp_tx_pull_down_id(FALSE);
//ANX ---: (ver0.4)					
					sp_tx_power_down(SP_TX_PWR_REG);
					sp_tx_power_down(SP_TX_PWR_TOTAL);
					sp_tx_hardware_powerdown(anx7808_client);
					sp_tx_set_sys_state(STATE_CABLE_PLUG);
					sp_tx_pd_mode = 1;
					sp_tx_link_config_done = 0;
					sp_tx_hw_lt_done = 0;
					sp_tx_hw_lt_enable = 0;
//ANX : (ver:20130105) pad solution					
					sp_tx_asus_pad = 0;
					return;
				}
			}
			//ASUS BSP wei +++
			if(1==trigger_sw_reset && sp_tx_system_state > STATE_LINK_TRAINING&& sp_tx_asus_pad){
				DEV_ERR("IRQ:________sw_reset\n");
				sp_tx_sw_error_power_down();
			}
			//ASUS BSP wei ---
		}

		if ((Int_vector1 & 0x08) == 0x08) {
			DEV_DBG("Downstream HDCP is done!\n");

			if ((Int_vector1 & 0x10) != 0x10)
				DEV_DBG("Downstream HDCP is passed!\n");
			else {
				if (sp_tx_system_state > STATE_CONFIG_OUTPUT) {
					sp_tx_video_mute(1);
					sp_tx_clean_hdcp();
					sp_tx_set_sys_state(STATE_HDCP_AUTH);
					DEV_ERR("Re-authentication due to downstream HDCP failure!");
				}
			}
		}

		if ((Int_vector1 & 0x20) == 0x20) {
			DEV_ERR(" Downstream HDCP link integrity check fail!");

			if (sp_tx_system_state > STATE_HDCP_AUTH) {
				sp_tx_set_sys_state(STATE_HDCP_AUTH);
				sp_tx_clean_hdcp();
				DEV_ERR("IRQ:____________HDCP Sync lost!");
			}
		}

		if ((Int_vector1 & 0x40) == 0x40)
			DEV_DBG("Receive CEC command from upstream done!");


		if ((Int_vector1 & 0x80) == 0x80)
			DEV_DBG("CEC command transfer to downstream done!");

//ASUS BSP wei lai +++
		if ((Int_vector2 & 0x04) == 0x04 &&sp_tx_system_state!=STATE_CABLE_PLUG) {
//ASUS BSP wei lai ---
			sp_tx_aux_dpcdread_bytes(0x00, 0x05, 0x18, 1, &c);

			if ((c & 0x40) == 0x40)
				DEV_NOTICE("Downstream HDMI termination is detected!\n");
		}


		/* specific int */
	} else if ((IRQ_Vector & SINK_SPECIFIC_IRQ) && (sp_tx_rx_type != RX_HDMI)) {

		sp_tx_send_message(MSG_CLEAR_IRQ);
		sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x00, 1, &c);
		if (!(c & 0x01)) {
			if ((sp_tx_system_state > STATE_CABLE_PLUG)
			    && (!sp_tx_pd_mode)) {
				
				sp_tx_clean_hdcp();

//ANX +++: (ver0.4)					
			    	sp_tx_vbus_powerdown();
			       sp_tx_pull_down_id(FALSE);
//ANX ---: (ver0.4)				   
				sp_tx_power_down(SP_TX_PWR_TOTAL);
				sp_tx_power_down(SP_TX_PWR_REG);
//ANX : (ver0.4)				
///				sp_tx_vbus_powerdown();
				sp_tx_hardware_powerdown(anx7808_client);

				sp_tx_pd_mode = 1;
				sp_tx_link_config_done = 0;
				sp_tx_hw_lt_enable = 0;
				sp_tx_hw_lt_done = 0;
//ANX : (ver:20130105) pad solution				
				sp_tx_asus_pad = 0;
				sp_tx_set_sys_state(STATE_CABLE_PLUG);
				return;
			}
		}

		sp_tx_aux_dpcdread_bytes(0x00, 0x02,
			DPCD_LANE_ALIGN_UD, 1, bytebuf);
		al = bytebuf[0];
		sp_tx_aux_dpcdread_bytes(0x00, 0x02,
			DPCD_LANE0_1_STATUS, 1, bytebuf);
		lane0_1_status = bytebuf[0];

		if (((lane0_1_status & 0x01) == 0)
		    || ((lane0_1_status & 0x04) == 0))
			sl_cr = 0;
		else
			sl_cr = 1;
		if (((al & 0x01) == 0) || (sl_cr == 0)) {
			if ((al & 0x01) == 0)
				DEV_ERR("Lane align not done\n");

			if (sl_cr == 0)
				DEV_ERR("Lane clock recovery not done\n");

				sp_tx_get_cable_type();
				if(sp_tx_rx_type_backup!= sp_tx_rx_type) {
					sp_tx_vbus_powerdown();
//ANX : (ver0.4)					
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
//ANX : (ver:20130105) pad solution					
					sp_tx_asus_pad = 0;
					sp_tx_set_sys_state(STATE_CABLE_PLUG);
					return;
				} else if ((sp_tx_system_state > STATE_LINK_TRAINING) 
			    && sp_tx_link_config_done) {
				sp_tx_link_config_done = 0;
				sp_tx_hw_lt_enable = 0;
				sp_tx_hw_lt_done = 0;
				sp_tx_set_sys_state(STATE_LINK_TRAINING);
				DEV_ERR("IRQ:____________re-LT request!");
			}
		}
	}

		/* AUTOMATED TEST IRQ */
	if (IRQ_Vector & TEST_IRQ) {
		sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x18, 1, bytebuf);
		test_vector = bytebuf[0];
		if (test_vector & 0x01) {
			sp_tx_test_lt = 1;
	
			sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x19,1,bytebuf);
			sp_tx_test_bw = bytebuf[0];
			DEV_DBG(" test_bw = %.2x\n", (uint)sp_tx_test_bw);

			sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x60,1,bytebuf);
			bytebuf[0] = bytebuf[0] | TEST_ACK;
//ANX : (ver:20130105)			
//			sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x60,1, bytebuf);
			sp_tx_aux_dpcdwrite_bytes(0x00, 0x02, 0x60,1, bytebuf);

			DEV_DBG("Set TEST_ACK!\n");
			sp_tx_set_sys_state(STATE_LINK_TRAINING);
			DEV_DBG("IRQ:test-LT request!\n");


		}


		if (test_vector & 0x04) {
			sp_tx_set_sys_state(STATE_PARSE_EDID);
			sp_tx_test_edid = 1;
			DEV_DBG("Test EDID Requested!\n");
		}
		/*phy test pattern*/
		if (test_vector & 0x08) {
			sp_tx_phy_auto_test();
			
			sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x60,1,bytebuf);
			bytebuf[0] = bytebuf[0] | 0x01;
//ANX : (ver:20130105)			
//			sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x60, 1,bytebuf);
			sp_tx_aux_dpcdwrite_bytes(0x00, 0x02, 0x60, 1,bytebuf);

		}
	}

}

static void sp_tx_sink_irq_int_handler(void)
{
	sp_tx_irq_isr();
}


void sp_tx_hdcp_process(void)
{
	unchar c;
	int i;
	
	if(MyDP_asus_padstation_exist_realtime()==1){
		sp_tx_power_down(SP_TX_PWR_HDCP);
		sp_tx_video_mute(0);
		sp_tx_set_sys_state(STATE_PLAY_BACK);
		ASUSEvtlog("[MyDP] MyDP run TV mode in Pad \n");
		return;
	}
//ANX +++: (ver0.4)
	sp_read_reg(RX_P1,HDMI_RX_HDCP_STATUS_REG, &c); 
	if(!(c & AUTH_EN)) {
		if (sp_tx_rx_type ==RX_VGA)
		{
//			printk("RX VGA hdcp not support playback\n");				
			return;
		}

//ASUS_BSP: larry , ignore check upstream hdcp status, force do hdcp process
#if 0
		else
		{
			printk("rx upstream hdcp check\n");
			sp_tx_power_down(SP_TX_PWR_HDCP);
			sp_tx_video_mute(0);
			sp_tx_set_sys_state(STATE_PLAY_BACK);
			return;
		}	
#endif	

	}
//ANX ---: (ver0.4)	
	if (!sp_tx_hdcp_capable_chk) {
		sp_tx_hdcp_capable_chk = 1;

		sp_tx_aux_dpcdread_bytes(0x06, 0x80, 0x28, 1, &c);
		if (!(c & 0x01)) {
			DEV_ERR("Sink is not capable HDCP");
//ANX +++: (ver0.4)			
			sp_tx_power_down(SP_TX_PWR_HDCP);
			//sp_tx_video_mute(0);
			sp_tx_video_mute(1);
//ANX ---: (ver0.4)			
			sp_tx_set_sys_state(STATE_PLAY_BACK);
			return;
		}
	}
	/*In case ANX730 video can not get ready*/
	if(sp_tx_rx_type == RX_HDMI) {
		if (!sp_tx_get_ds_video_status()) {
			if (sp_tx_ds_vid_stb_cntr ==
				SP_TX_DS_VID_STB_TH) {
				sp_tx_vbus_powerdown();
//ANX : (ver0.4)							
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
				sp_tx_ds_vid_stb_cntr = 0;
//ANX : (ver:20130105) pad solution					
				sp_tx_asus_pad = 0;
				sp_tx_set_sys_state(STATE_CABLE_PLUG);
			} else {
				sp_tx_ds_vid_stb_cntr++;
				msleep(100);
			}
			return;
			
		} else {
			sp_tx_ds_vid_stb_cntr = 0;
		}
       }

	if (!sp_tx_hw_hdcp_en) {
		/*Issue HDCP after the HDMI Rx key loaddown*/
		sp_read_reg(RX_P1,HDMI_RX_HDCP_STATUS_REG, &c); 
		if(c & AUTH_EN) {
			for(i=0; i < 10; i++) {
			sp_read_reg(RX_P1, HDMI_RX_HDCP_STATUS_REG, &c);
			if(c&LOAD_KEY_DONE) 
				break;
			else
				msleep(10);
			}
		}
		sp_tx_power_on(SP_TX_PWR_HDCP);
		msleep(50);
		sp_tx_hw_hdcp_enable();
		sp_tx_hw_hdcp_en = 1;
	}

	if (sp_tx_hdcp_auth_done) {
		sp_tx_hdcp_auth_done = 0;

		if (sp_tx_hdcp_auth_pass) {

			sp_tx_hdcp_encryption_enable();
			sp_tx_video_mute(0);
			DEV_NOTICE("@@@@@@@hdcp_auth_pass@@@@@@\n");

		} else {
			sp_tx_hdcp_encryption_disable();
			sp_tx_video_mute(1);
			DEV_NOTICE("*********hdcp_auth_failed*********\n");
			return;
		}

		sp_tx_set_sys_state(STATE_PLAY_BACK);
		sp_tx_show_infomation();
	}

}

//ANX +++: (ver:20130105)	
static void sp_tx_sw_wr_bksv(void)
{
	unchar i,c;
	for(i=0;i<5;i++) {
		sp_write_reg(TX_P0, 0x12+i, BKSV[i]);
		sp_read_reg(TX_P0, 0x12+i, &c);
//ANX : (ver0.4)		
		//DEV_DBG("Write in _BKSV%.1x  = %.2x\n",(uint)i, (uint)c); 			
	}	
}
static void sp_tx_sw_wr_aksv(void)
{ 	unchar i;
	for(i=0;i<5;i++) {
		sp_tx_aux_dpcdwrite_byte(0x06,0x80,0x07+i, AKSV[i]);
//ANX : (ver0.4)		
		//DEV_DBG("Write in _AKSV%.1x  = %.2x\n",(uint)i, (uint)(AKSV[i])); 			
	}
}

static void sp_tx_sw_wr_an(void)
{
	unchar i;
	for(i=0;i<8;i++) {
//ANX : (ver0.4)	
		//DEV_DBG("Write in _AN%.1x  = %.2x\n",(uint)i, (uint)(AN[i])); 	
		sp_tx_aux_dpcdwrite_byte(0x06,0x80,0x0c+i, AN[i]);	      
	}
}
static void sp_tx_sw_gen_an(void)
{
      unchar c, i;
	sp_read_reg(TX_P0, 0x01, &c);
	sp_write_reg(TX_P0, 0x01, c|0x80);
	sp_read_reg(TX_P0, 0x01, &c);
	sp_write_reg(TX_P0, 0x01, c&0x7F);  
       
	for(i=0;i<8;i++) {
		sp_read_reg(TX_P0, 0x0A+i, AN+i);
//ANX : (ver0.4)		
		//DEV_DBG("Generated AN%.1x  = %.2x\n",(uint)i, (uint)(AN[i])); 
	}
}
static void sp_tx_sw_rd_aksv(void)
{
        unchar i;
        for(i=0;i<5;i++) {
            sp_read_reg(TX_P0, 0x05+i, AKSV+i);
//ANX : (ver0.4)			
            //DEV_DBG("AKSV%.1x = %.2x\n", (uint)i,(uint)(AKSV[i])); 
         }  
}

static void sp_tx_sw_rd_bksv(void)
{
	unchar i,j;
	unchar bksv[5];
	unchar c = 0;
	sp_tx_aux_dpcdread_bytes(0x06,0x80,0x00, 5, bksv);
	for (i =0; i<5; i++) {
		BKSV[i] = bksv[i];
		BKSV_M[i] = BKSV[i]; 
//ANX : (ver0.4)		
		//DEV_DBG("BKSV%.1x = %.2x\n", (uint)i,(uint)(BKSV[i])); 
	}
	for(i=0;i<5;i++){           
		for(j=0;j<8;j++){
			c = c+( BKSV_M[i] &0x01);
			BKSV_M[i] =  BKSV_M[i] >>1;
		}
	}
    
	if(c != 20){
		DEV_DBG("BKSV check error");        
		sp_tx_ksv_srm_pass = 0;
	} else {
		sp_tx_ksv_srm_pass = 1;
	}
}
void sp_tx_sw_hdcp_process(void)
{ 
	unchar  c;
	unchar  r0_rx_lb, r0_rx_hb; 
	unchar r0_tx_lb, r0_tx_hb;   


	if (!sp_tx_hdcp_capable_chk) {
		sp_tx_hdcp_capable_chk = 1;

		sp_tx_aux_dpcdread_bytes(0x06, 0x80, 0x28, 1, &c);
		if (!(c & 0x01)) {
			DEV_ERR("Sink is not capable HDCP\n");
			sp_tx_video_mute(0);
			sp_tx_set_sys_state(STATE_PLAY_BACK);
			return;
		}
	}
  
	// read BKSV from RX and check it
	sp_tx_sw_rd_bksv();   
    
	if ( sp_tx_ksv_srm_pass) {
		
		sp_read_reg(TX_P0, SP_TX_HDCP_CTRL0_REG, &c);        
		sp_write_reg(TX_P0, SP_TX_HDCP_CTRL0_REG, c|0x43);
//ANX : (ver0.4)		
		DEV_DBG("BKSV check passed\n");

		//generate AN from TX
		sp_tx_sw_gen_an();
		sp_tx_sw_rd_aksv();
		
		//write BKSV to TX
		sp_tx_sw_wr_bksv();                

		// write AN, AKSV to Rx   
		sp_tx_sw_wr_an();
		sp_tx_sw_wr_aksv();

		msleep(100);//delay 100 ms to read R0'   

		// read R0 from TX
		sp_read_reg(TX_P0, 0x17, &r0_tx_lb);
		sp_read_reg(TX_P0, 0x18, &r0_tx_hb);
//ANX : (ver0.4)
		//DEV_DBG("r0_tx_lb = %.2x\n", (uint)r0_tx_lb); 
		//DEV_DBG("r0_tx_hb = %.2x\n", (uint)r0_tx_hb); 
		//DEV_DBG("R0_transmitter = %4.2x\n", ( (uint)r0_tx_hb<<8)+(uint)r0_tx_lb);    

		//read R0'  from RX
		//sp_tx_aux_dpcdread_bytes(0x06,0x80,0x29, 1, &c);
		//DEV_DBG("BSTATUS = %.2x\n", (uint)c);

		//read R0'  from RX
		sp_tx_aux_dpcdread_bytes(0x06,0x80,0x05, 1, &c);
		r0_rx_lb =  c;
		sp_tx_aux_dpcdread_bytes(0x06,0x80,0x06, 1, &c);
		r0_rx_hb = c;
//ANX : (ver0.4)
		//DEV_DBG("r0_rx_lb = %.2x\n", (uint)r0_rx_lb); 
		//DEV_DBG("r0_rx_hb = %.2x\n", (uint)r0_rx_hb); 
		//DEV_DBG("R0'_receiver = %4.2x\n", ((uint)r0_rx_hb<<8)+(uint)r0_rx_lb);        

		if ((r0_rx_hb == r0_tx_hb)&&(r0_rx_lb==r0_tx_lb)) {
			sp_tx_hdcp_auth_pass = 1;
//ANX : (ver0.4)			
			//DEV_DBG("R0 = R0'");
			sp_read_reg(TX_P0, SP_TX_HDCP_CTRL0_REG, &c);
			sp_write_reg(TX_P0, SP_TX_HDCP_CTRL0_REG, c |0x14);


			//sp_read_reg(TX_P0, SP_TX_HDCP_CTRL0_REG, &c);
			//DEV_DBG("HDCP Control resgister 0 = %.2x\n", (uint)c);  

			//sp_read_reg(TX_P0, SP_TX_HDCP_STATUS, &c);
			//DEV_DBG("HDCP STATUS register  = %.2x\n", (uint)c);                      

			DEV_DBG("@@@@@@@hdcp_auth_pass@@@@@@\n");
			//sp_tx_hdcp_encryption_enable();
			sp_tx_video_mute(0);


		} else {           

			DEV_DBG("R0 != R0\n");
			sp_tx_hdcp_encryption_disable();
			sp_tx_video_mute(1);
			DEV_ERR("*********hdcp_auth_failed*********\n");
			return;
		}       
	}else  {
		sp_tx_hdcp_encryption_disable();
		sp_tx_video_mute(1);
		DEV_ERR("*********hdcp_auth_failed due to bksv check error*********\n");
	}
	sp_tx_set_sys_state(STATE_PLAY_BACK);
	sp_tx_show_infomation();
}
//ANX ---: (ver:20130105)	

//ASUS BSP Wei +++
#ifdef CONFIG_ASUS_HDMI
	extern ktime_t  plugin_starttime;
	extern int measure_pad_plugin_time;
#endif
//ASUS BSP Wei ---

extern int g_isMyDP_poweron; //ASUS BSP wei +++

void sp_tx_set_sys_state(enum SP_TX_System_State ss)
{
	ktime_t rettime;
	u64 usecs64;
	int usecs;
	DEV_NOTICE("SP_TX To System State: ");

	switch (ss) {
	case STATE_INIT:
		sp_tx_system_state = STATE_INIT;
		DEV_NOTICE("STATE_INIT");
		break;
	case STATE_CABLE_PLUG:
		sp_tx_system_state = STATE_CABLE_PLUG;
		DEV_NOTICE("STATE_CABLE_PLUG");
		break;
	case STATE_PARSE_EDID:
		sp_tx_system_state = STATE_PARSE_EDID;
		DEV_NOTICE("SP_TX_READ_PARSE_EDID");
		break;
	case STATE_CONFIG_HDMI:
		sp_tx_system_state = STATE_CONFIG_HDMI;
		DEV_NOTICE("STATE_CONFIG_HDMI");
		break;
	case STATE_CONFIG_OUTPUT:
		g_isMyDP_poweron = 1; //ASUS BSP wei +++
		sp_tx_system_state = STATE_CONFIG_OUTPUT;
		DEV_NOTICE("STATE_CONFIG_OUTPUT");
		break;
	case STATE_LINK_TRAINING:
		sp_tx_system_state = STATE_LINK_TRAINING;
		sp_tx_link_config_done = 0;
		sp_tx_hw_lt_enable = 0;
		sp_tx_hw_lt_done = 0;
		DEV_NOTICE("STATE_LINK_TRAINING");
		break;
	case STATE_HDCP_AUTH:
		sp_tx_system_state = STATE_HDCP_AUTH;
		DEV_NOTICE("STATE_HDCP_AUTH");
		break;
	case STATE_PLAY_BACK:
		sp_tx_system_state = STATE_PLAY_BACK;
		DEV_NOTICE("STATE_PLAY_BACK");
//ASUS BSP++ Vincent
		if(measure_pad_wakeup_time){
			rettime = ktime_get();
			usecs64 = ktime_to_ns(ktime_sub(rettime, wakeup_starttime));
			do_div(usecs64, NSEC_PER_USEC);
			usecs = usecs64;					
			measure_pad_wakeup_time = false;
			if(pad_resume_time_mask || ((usecs / USEC_PER_MSEC) + 400 > 2500)){
				printk("\n[PM]Pad MYDP ready after %ld ms\n", usecs / USEC_PER_MSEC);			
				printk("[PM]Pad system-wakeup takes about %ld ms\n", (usecs / USEC_PER_MSEC) + 400);
			}
		}
//ASUS BSP-- Vincent	

//ASUS BSP Wei +++
#ifdef CONFIG_ASUS_HDMI
		if(measure_pad_plugin_time){
			rettime = ktime_get();
			usecs64 = ktime_to_ns(ktime_sub(rettime, plugin_starttime));
			do_div(usecs64, NSEC_PER_USEC);
			usecs = usecs64;					
			measure_pad_plugin_time=0;
			DEV_NOTICE("MYDP ready takes about  %ld ms \n", usecs / USEC_PER_MSEC);			
		}
#endif
//ASUS BSP Wei ---
			
		
		break;
	default:
		break;
	}
}
extern unchar g_hdmi_rx_vsync_change;
void sp_tx_int_irq_handler(void)
{
	unchar c1, c2, c3, c4, c5;

	sp_tx_get_int_status(COMMON_INT_1, &c1);
	sp_tx_get_int_status(COMMON_INT_2, &c2);
	sp_tx_get_int_status(COMMON_INT_3, &c3);
	sp_tx_get_int_status(COMMON_INT_4, &c4);
	sp_tx_get_int_status(SP_INT_STATUS, &c5);

    if (sp_tx_asus_pad) {
		if(c1!=0x80||(c2!=0x40&&c2!=0x0)||c3!=0x0||c4!=0x0||(c5!=0x0&&c5!=0x2)){
	   	    DEV_DBG("sp_tx_int   C1=0x%x,c2=0x%x,c3=0x%x,c4=0x%x,c5=0x%x\n",c1,c2,c3,c4,c5);
		    if (g_hdmi_rx_vsync_change > 1)
			{
			  //  ASUSEvtlog("[myDP]sp_tx_int   C1=0x%x,c2=0x%x,c3=0x%x,c4=0x%x,c5=0x%x\n",c1,c2,c3,c4,c5);
			}
		}
	}
	if (c1 & PLL_LOCK_CHG)
		sp_tx_pll_changed_int_handler();

	if (c2 & HDCP_AUTH_DONE)
		sp_tx_auth_done_int_handler();

	if (c3 & HDCP_LINK_CHK_FAIL)
		sp_tx_link_chk_fail_int_handler();

	if (c5 & DPCD_IRQ_REQUEST){
	//ASUS BSP wei +++
		if(c1==0x80&&c2==0x0&&c3==0x0&&c4==0x0&&c5==0x80&& sp_tx_asus_pad)
			trigger_sw_reset=1;
	//ASUS BSP wei ---
		sp_tx_sink_irq_int_handler();
	}

	if (c5 & POLLING_ERR)
		sp_tx_polling_err_int_handler();

	if (c5 & TRAINING_Finish)
		sp_tx_lt_done_int_handler();

	if (c5 & LINK_CHANGE)
		sp_tx_link_change_int_handler();

}


#ifdef EYE_TEST
void sp_tx_eye_diagram_test(void)
{
	unchar c;
	int i;

	sp_write_reg(TX_P2, 0x05, 0x00);

// this should comment for eye test, or will i2c error
//	sp_write_reg(TX_P2, SP_TX_DP_ADDR_REG1, 0xbc);

	sp_read_reg(TX_P2, SP_TX_RST_CTRL_REG, &c);
	c |= SW_RST;
	sp_write_reg(TX_P2, SP_TX_RST_CTRL_REG, c);
	c &= ~SW_RST;
	sp_write_reg(TX_P2, SP_TX_RST_CTRL_REG, c);
		sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG0, 0x19);
//ANX : (ver:20130105)		
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG16, 0x18);
		

	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG0, 0x16);
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG4, 0x1b);
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG7, 0x22);
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG9, 0x23);
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG14, 0x09);
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG17, 0x16);
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG19, 0x1F);
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG1, 0x26);
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG5, 0x28);
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG8, 0x2F);
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG15, 0x10);
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG18, 0x1F);
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG2, 0x36);
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG6, 0x3c);
	sp_write_reg(TX_P1, SP_TX_LT_CTRL_REG3, 0x3F);
	/* set link bandwidth 5.4G */
	sp_write_reg(TX_P0, 0xa0, 0x14);
	/* set swing 400mv3.5db=0x09, 600mv=0x02, 600mv3.5db=0x0a, 400mv6db=0x11 */
	/* for pass eye, set swing to 600mV 3.5db */
	sp_write_reg(TX_P0, 0xa3, 0xa);
//ANX +++: (ver:20130105)		
	/* send cep patterns */
		sp_write_reg(TX_P0, 0xa2, 0x14);
		sp_write_reg(TX_P0, 0xA9, 0x00);
		sp_write_reg(TX_P0, 0xAA, 0x01);
//ANX ---: (ver:20130105)	

		sp_write_reg(TX_P2, SP_TX_ANALOG_CTRL, 0xC5);
		sp_write_reg(TX_P0, I2C_GEN_10US_TIMER0, 0x0E);
		sp_write_reg(TX_P0, I2C_GEN_10US_TIMER1, 0x01);


	for (i = 0; i < 256; i++) {
		sp_read_reg(0x72, i, &c);

		if ((i & 0x0f) == 0)
			printk(KERN_INFO "\n [%.2x]	%.2x  ",
			(unsigned int)i, (unsigned int)c);
		else
			printk(KERN_INFO "%.2x  ", (unsigned int)c);

		if ((i & 0x0f) == 0x0f)
			printk(KERN_INFO "\n-------------------------------------");
	}

	printk(KERN_INFO "\n");



	for (i = 0; i < 256; i++) {
// this should modify for eye test, or will i2c error	
		sp_read_reg(0x70, i, &c);

		if ((i & 0x0f) == 0)
			printk(KERN_INFO "\n [%.2x]	%.2x  ",
			(unsigned int)i,
			       (unsigned int)c);
		else
			printk(KERN_INFO "%.2x  ", (unsigned int)c);

		if ((i & 0x0f) == 0x0f)
			printk(KERN_INFO "\n-------------------------------------");
	}

	printk(KERN_INFO "*******Eye Diagram Test Pattern is sent********\n");

}

#endif

/* ***************************************************************** */

/* Functions defination for HDMI Input */

/* ***************************************************************** */

void hdmi_rx_set_hpd(unchar enable)
{
	unchar c;

	if (enable) {
		/* set HPD high */

		sp_read_reg(TX_P2, SP_TX_VID_CTRL3_REG, &c);
		c |= HPD_OUT;
		sp_write_reg(TX_P2, SP_TX_VID_CTRL3_REG, c);

		DEV_NOTICE("HPD high is issued\n");
	} else {

		/* set HPD low */

		sp_read_reg(TX_P2, SP_TX_VID_CTRL3_REG, &c);
		c &= ~HPD_OUT;
		sp_write_reg(TX_P2, SP_TX_VID_CTRL3_REG, c);

		DEV_NOTICE("HPD low is issued\n");
	}

}

void hdmi_rx_set_termination(unchar enable)
{
	unchar c;

	if (enable) {
		/* set termination high */

		sp_read_reg(RX_P0, HDMI_RX_TMDS_CTRL_REG6, &c);
		c &= ~TERM_PD;
		sp_write_reg(RX_P0, HDMI_RX_TMDS_CTRL_REG6, c);
		DEV_NOTICE("Termination high is issued\n");
	} else {

		/* set termination low */

		sp_read_reg(RX_P0, HDMI_RX_TMDS_CTRL_REG6, &c);
		c |= TERM_PD;
		sp_write_reg(RX_P0, HDMI_RX_TMDS_CTRL_REG6, c);
		DEV_NOTICE("Termination low is issued\n");
	}

}

static void hdmi_rx_restart_audio_chk(void)
{
	if (hdmi_system_state == HDMI_AUDIO_CONFIG) {
		DEV_DBG("WAIT_AUDIO: hdmi_rx_restart_audio_chk.\n");
		g_cts_got = 0;
		g_audio_got = 0;
	}
}


static void hdmi_rx_set_sys_state(enum HDMI_RX_System_State ss)
{
	if (hdmi_system_state != ss) {
		ASUS_DEV_INFO("");
		hdmi_system_state = ss;

		switch (ss) {
		case HDMI_CLOCK_DET:
			DEV_NOTICE("HDMI_RX:  HDMI_CLOCK_DET");
			break;
		case HDMI_SYNC_DET:
			DEV_NOTICE("HDMI_RX:  HDMI_SYNC_DET");
			break;
		case HDMI_VIDEO_CONFIG:
			DEV_NOTICE("HDMI_RX:  HDMI_VIDEO_CONFIG");
			break;
		case HDMI_AUDIO_CONFIG:
			DEV_NOTICE("HDMI_RX:  HDMI_AUDIO_CONFIG");
			hdmi_rx_restart_audio_chk();
			break;
		case HDMI_PLAYBACK:
			DEV_NOTICE("HDMI_RX:  HDMI_PLAYBACK");
			break;
		default:
			break;
		}
	}
}

static void hdmi_rx_mute_video(void)
{
	unchar c;

	DEV_DBG("Mute Video.");
	sp_read_reg(RX_P0, HDMI_RX_HDMI_MUTE_CTRL_REG, &c);
	c |=  VID_MUTE;
	sp_write_reg(RX_P0, HDMI_RX_HDMI_MUTE_CTRL_REG, c);
	g_video_muted = 1;
}

static void hdmi_rx_unmute_video(void)
{
	unchar c;

	DEV_DBG("Unmute Video.");
	sp_read_reg(RX_P0, HDMI_RX_HDMI_MUTE_CTRL_REG, &c);
	c &= ~VID_MUTE;
	sp_write_reg(RX_P0, HDMI_RX_HDMI_MUTE_CTRL_REG, c);
	g_video_muted = 0;
}

static void hdmi_rx_mute_audio(void)
{
	unchar c;

	DEV_DBG("Mute Audio.");
	sp_read_reg(RX_P0, HDMI_RX_HDMI_MUTE_CTRL_REG, &c);
	c |= AUD_MUTE;
	sp_write_reg(RX_P0, HDMI_RX_HDMI_MUTE_CTRL_REG, c);
	g_audio_muted = 1;
}

static void hdmi_rx_unmute_audio(void)
{
	unchar c;

	DEV_DBG("Unmute Audio.");
	sp_read_reg(RX_P0, HDMI_RX_HDMI_MUTE_CTRL_REG, &c);
	c &= ~AUD_MUTE;
	sp_write_reg(RX_P0, HDMI_RX_HDMI_MUTE_CTRL_REG, c);
	g_audio_muted = 0;
}

static unchar hdmi_rx_is_video_change(void)
{
	unchar ch, cl;
	ulong n;
	sp_read_reg(RX_P0, HDMI_RX_HTOTAL_LOW, &cl);
	sp_read_reg(RX_P0, HDMI_RX_HTOTAL_HIGH, &ch);
	n = ch;
	n = (n << 8) + cl;

	if ((g_cur_h_res < (n - 10)) || (g_cur_h_res > (n + 10))) {
		DEV_ERR("H_Res changed.");
		DEV_ERR("Current H_Res = %ld\n", n);
		return 1;
	}

	sp_read_reg(RX_P0, HDMI_RX_VTOTAL_LOW, &cl);
	sp_read_reg(RX_P0, HDMI_RX_VTOTAL_HIGH, &ch);
	n = ch;
	n = (n << 8) + cl;

	if ((g_cur_v_res < (n - 10)) || (g_cur_v_res > (n + 10))) {
		DEV_ERR("V_Res changed.\n");
		DEV_ERR("Current V_Res = %ld\n", n);
		return 1;
	}

	sp_read_reg(RX_P0, HDMI_RX_HDMI_STATUS_REG, &cl);

	cl &= HDMI_MODE;

	if (g_hdmi_dvi_status != cl) {
		DEV_ERR("DVI to HDMI or HDMI to DVI Change.");
		return 1;
	}

	return 0;
}

static void hdmi_rx_get_video_info(void)
{
	unchar ch, cl;
	uint n;

	sp_read_reg(RX_P0, HDMI_RX_HTOTAL_LOW, &cl);
	sp_read_reg(RX_P0, HDMI_RX_HTOTAL_HIGH, &ch);
	n = ch;
	n = (n << 8) + cl;
	g_cur_h_res = n;
	sp_read_reg(RX_P0, HDMI_RX_VTOTAL_LOW, &cl);
	sp_read_reg(RX_P0, HDMI_RX_VTOTAL_HIGH, &ch);
	n = ch;
	n = (n << 8) + cl;
	g_cur_v_res = n;

	sp_read_reg(RX_P0, HDMI_RX_VID_PCLK_CNTR_REG, &cl);
	g_cur_pix_clk = cl;
	sp_read_reg(RX_P0, HDMI_RX_HDMI_STATUS_REG, &cl);
	g_hdmi_dvi_status = ((cl & HDMI_MODE) == HDMI_MODE);
	
}

static void hdmi_rx_show_video_info(void)
{
	unchar c, c1;
	unchar cl, ch;
	ulong n;
	ulong h_res, v_res;

	sp_read_reg(RX_P0, HDMI_RX_HACT_LOW, &cl);
	sp_read_reg(RX_P0, HDMI_RX_HACT_HIGH, &ch);
	n = ch;
	n = (n << 8) + cl;
	h_res = n;

	sp_read_reg(RX_P0, HDMI_RX_VACT_LOW, &cl);
	sp_read_reg(RX_P0, HDMI_RX_VACT_HIGH, &ch);
	n = ch;
	n = (n << 8) + cl;
	v_res = n;

	DEV_DBG("");
	DEV_DBG("*****************HDMI_RX Info*******************");
	DEV_DBG("HDMI_RX Is Normally Play Back.\n");
	sp_read_reg(RX_P0, HDMI_RX_HDMI_STATUS_REG, &c);

    if(c & HDMI_MODE)
		DEV_DBG("HDMI_RX Mode = HDMI Mode.\n");
	else
		DEV_DBG("HDMI_RX Mode = DVI Mode.\n");

	sp_read_reg(RX_P0, HDMI_RX_VIDEO_STATUS_REG1, &c);
    if(c & VIDEO_TYPE)
		v_res += v_res;


	DEV_DBG("HDMI_RX Video Resolution = %ld * %ld ", h_res, v_res);
	sp_read_reg(RX_P0, HDMI_RX_VIDEO_STATUS_REG1, &c);

    if(c & VIDEO_TYPE)
		DEV_DBG("    Interlace Video.");
	else
		DEV_DBG("    Progressive Video.");

	sp_read_reg(RX_P0, HDMI_RX_SYS_CTRL1_REG, &c);

	if ((c & 0x30) == 0x00)
		DEV_DBG("Input Pixel Clock = Not Repeated.\n");
	else if ((c & 0x30) == 0x10)
		DEV_DBG("Input Pixel Clock = 2x Video Clock. Repeated.\n");
	else if ((c & 0x30) == 0x30)
		DEV_DBG("Input Pixel Clock = 4x Vvideo Clock. Repeated.\n");

	if ((c & 0xc0) == 0x00)
		DEV_DBG("Output Video Clock = Not Divided.\n");
	else if ((c & 0xc0) == 0x40)
		DEV_DBG("Output Video Clock = Divided By 2.\n");
	else if ((c & 0xc0) == 0xc0)
		DEV_DBG("Output Video Clock = Divided By 4.\n");

	if (c & 0x02)
		DEV_DBG("Output Video Using Rising Edge To Latch Data.\n");
	else
		DEV_DBG("Output Video Using Falling Edge To Latch Data.\n");

	DEV_DBG("Input Video Color Depth = ");
	sp_read_reg(RX_P0, 0x70, &c1);
	c1 &= 0xf0;

	if (c1 == 0x00)
		DEV_DBG("Legacy Mode.\n");
	else if (c1 == 0x40)
		DEV_DBG("24 Bit Mode.\n");
	else if (c1 == 0x50)
		DEV_DBG("30 Bit Mode.\n");
	else if (c1 == 0x60)
		DEV_DBG("36 Bit Mode.\n");
	else if (c1 == 0x70)
		DEV_DBG("48 Bit Mode.\n");

	DEV_DBG("Input Video Color Space = ");
	sp_read_reg(RX_P1, HDMI_RX_AVI_DATA00_REG, &c);
	c &= 0x60;

	if (c == 0x20)
		DEV_DBG("YCbCr4:2:2 .\n");
	else if (c == 0x40)
		DEV_DBG("YCbCr4:4:4 .\n");
	else if (c == 0x00)
		DEV_DBG("RGB.\n");
	else
		DEV_DBG("Unknow 0x44 = 0x%.2x\n", (int)c);

	sp_read_reg(RX_P1, HDMI_RX_HDCP_STATUS_REG, &c);

    if(c & AUTH_EN)
		DEV_DBG("Authentication is attempted.");
	else
		DEV_DBG("Authentication is not attempted.");

	for (cl = 0; cl < 20; cl++) {
		sp_read_reg(RX_P1, HDMI_RX_HDCP_STATUS_REG, &c);

        if(c & DECRYPT_EN)
			break;
		else
			mdelay(10);
	}

	if (cl < 20)
		DEV_DBG("Decryption is active.");
	else
		DEV_DBG("Decryption is not active.");


	DEV_DBG("********************************************************");
	DEV_DBG("");
}

static void hdmi_rx_show_audio_info(void)
{
	unchar c;

	DEV_DBG("Audio Fs = ");
    sp_read_reg(RX_P0,HDMI_RX_AUD_IN_CH_STATUS4_REG, &c); 
	c &= 0x0f;

	switch (c) {
	case 0x00:
		DEV_DBG("44.1 KHz.");
		break;
	case 0x02:
		DEV_DBG("48 KHz.");
		break;
	case 0x03:
		DEV_DBG("32 KHz.");
		break;
	case 0x08:
		DEV_DBG("88.2 KHz.");
		break;
	case 0x0a:
		DEV_DBG("96 KHz.");
		break;
	case 0x0e:
		DEV_DBG("192 KHz.");
		break;
	default:
		break;
	}

	DEV_DBG("");
}


static void hdmi_rx_init_var(void)
{
	hdmi_rx_set_sys_state(HDMI_CLOCK_DET);
	g_cur_h_res = 0;
	g_cur_v_res = 0;
	g_cur_pix_clk = 0;
	g_video_muted = 1;
	g_audio_muted = 1;
	g_audio_stable_cntr = 0;
	g_video_stable_cntr = 0;
	g_sync_expire_cntr = 0;
	g_hdcp_err_cnt = 0;
	g_hdmi_dvi_status = VID_DVI_MODE;
	g_cts_got = 0;
	g_audio_got = 0;

}

static void hdmi_rx_tmds_phy_initialization(void)
{
	sp_write_reg(RX_P0,HDMI_RX_TMDS_CTRL_REG2, 0x00);
	sp_write_reg(RX_P0,HDMI_RX_TMDS_CTRL_REG4, 0x28);
	sp_write_reg(RX_P0,HDMI_RX_TMDS_CTRL_REG5, 0xe3);
	sp_write_reg(RX_P0,HDMI_RX_TMDS_CTRL_REG7, 0x70);
	sp_write_reg(RX_P0,HDMI_RX_TMDS_CTRL_REG19, 0x00);
	sp_write_reg(RX_P0,HDMI_RX_TMDS_CTRL_REG21, 0x04);
	sp_write_reg(RX_P0,HDMI_RX_TMDS_CTRL_REG22, 0x38);

}
//ASUS BSP wei +++
void hdmi_asus_rx_initialization(void){
	unchar c;

	hdmi_rx_init_var();
	
	c = AUD_MUTE | VID_MUTE;
	sp_write_reg(RX_P0, HDMI_RX_HDMI_MUTE_CTRL_REG, c);
	
	sp_read_reg(RX_P0, HDMI_RX_CHIP_CTRL_REG, &c);
	c |= MAN_HDMI5V_DET;
	c |= PLLLOCK_CKDT_EN;
	c |= DIGITAL_CKDT_EN;
	sp_write_reg(RX_P0, HDMI_RX_CHIP_CTRL_REG, c);

	sp_read_reg(RX_P0, HDMI_RX_AEC_CTRL_REG, &c);
	c |= AVC_OE;
	sp_write_reg(RX_P0, HDMI_RX_AEC_CTRL_REG, c);
#if 0
	sp_read_reg(RX_P0, HDMI_RX_SRST_REG, &c);
	c |= HDCP_MAN_RST;
	sp_write_reg(RX_P0, HDMI_RX_SRST_REG, c);
	msleep(1);
	sp_read_reg(RX_P0, HDMI_RX_SRST_REG, &c);
	c &= ~HDCP_MAN_RST;
	sp_write_reg(RX_P0, HDMI_RX_SRST_REG, c);
#endif
	sp_read_reg(RX_P0, HDMI_RX_SRST_REG, &c);
	c |= SW_MAN_RST;
	sp_write_reg(RX_P0, HDMI_RX_SRST_REG, c);
	msleep(1);
	c  &= ~SW_MAN_RST;
	sp_write_reg(RX_P0, HDMI_RX_SRST_REG, c);

	sp_read_reg(RX_P0, HDMI_RX_SRST_REG, &c);
	c |= TMDS_RST;
	sp_write_reg(RX_P0, HDMI_RX_SRST_REG, c);

       c = AEC_EN07 | AEC_EN06 | AEC_EN05 | AEC_EN02;
	sp_write_reg(RX_P0, HDMI_RX_AEC_EN0_REG, c);
	c = AEC_EN12 |AEC_EN10 | AEC_EN09 | AEC_EN08;
	sp_write_reg(RX_P0, HDMI_RX_AEC_EN1_REG, c);
	c = AEC_EN23 |AEC_EN22 | AEC_EN21 | AEC_EN20;
	sp_write_reg(RX_P0, HDMI_RX_AEC_EN2_REG, 0xf0);

	sp_read_reg(RX_P0, HDMI_RX_AEC_CTRL_REG, &c);
	c |=  AVC_EN;
	sp_write_reg(RX_P0, HDMI_RX_AEC_CTRL_REG, c);

	sp_read_reg(RX_P0, HDMI_RX_AEC_CTRL_REG, &c);
	c |= AAC_EN;
	sp_write_reg(RX_P0, HDMI_RX_AEC_CTRL_REG, c);

	sp_read_reg(RX_P0, HDMI_RX_SYS_PWDN1_REG, &c);
	c &= ~PWDN_CTRL;
	sp_write_reg(RX_P0, HDMI_RX_SYS_PWDN1_REG, c);

	sp_write_reg(RX_P0, HDMI_RX_INT_MASK1_REG, 0x00);
	sp_write_reg(RX_P0, HDMI_RX_INT_MASK2_REG, 0x00);
	sp_write_reg(RX_P0, HDMI_RX_INT_MASK3_REG, 0x00);
	sp_write_reg(RX_P0, HDMI_RX_INT_MASK4_REG, 0x00);
	sp_write_reg(RX_P0, HDMI_RX_INT_MASK5_REG, 0x00);
	sp_write_reg(RX_P0, HDMI_RX_INT_MASK6_REG, 0x00);
	sp_write_reg(RX_P0, HDMI_RX_INT_MASK7_REG, 0x00);


	/* Range limitation for RGB input */
	sp_read_reg(RX_P0, HDMI_RX_VID_DATA_RNG_CTRL_REG, &c);
	c |= R2Y_INPUT_LIMIT;
	sp_write_reg(RX_P0, HDMI_RX_VID_DATA_RNG_CTRL_REG, c);

       c = CEC_RST;
	sp_write_reg(RX_P0, HDMI_RX_CEC_CTRL_REG, c);
	c = CEC_SPEED_27M;
	sp_write_reg(RX_P0, HDMI_RX_CEC_SPEED_CTRL_REG, c);
	c = CEC_RX_EN;
	sp_write_reg(RX_P0, HDMI_RX_CEC_CTRL_REG, c);
	hdmi_rx_tmds_phy_initialization();

	//ASUS_BSP : larry pad mode fix h/v initial data for speed up performance

	if(sp_tx_asus_pad){
		g_cur_h_res = 2000;
		g_cur_v_res= 1235;
		g_hdmi_dvi_status=1;
	}
	DEV_NOTICE("ASUS HDMI Rx is initialized...\n"); 
}
//ASUS BSP wei ---
void hdmi_rx_initialization(void)
{
	unchar c;

	hdmi_rx_init_var();
	
	c = AUD_MUTE | VID_MUTE;
	sp_write_reg(RX_P0, HDMI_RX_HDMI_MUTE_CTRL_REG, c);
	
	sp_read_reg(RX_P0, HDMI_RX_CHIP_CTRL_REG, &c);
	c |= MAN_HDMI5V_DET;
	c |= PLLLOCK_CKDT_EN;
	c |= DIGITAL_CKDT_EN;
	sp_write_reg(RX_P0, HDMI_RX_CHIP_CTRL_REG, c);

	sp_read_reg(RX_P0, HDMI_RX_AEC_CTRL_REG, &c);
	c |= AVC_OE;
	sp_write_reg(RX_P0, HDMI_RX_AEC_CTRL_REG, c);

	sp_read_reg(RX_P0, HDMI_RX_SRST_REG, &c);
	c |= HDCP_MAN_RST;
	sp_write_reg(RX_P0, HDMI_RX_SRST_REG, c);
	msleep(1);
	sp_read_reg(RX_P0, HDMI_RX_SRST_REG, &c);
	c &= ~HDCP_MAN_RST;
	sp_write_reg(RX_P0, HDMI_RX_SRST_REG, c);

	sp_read_reg(RX_P0, HDMI_RX_SRST_REG, &c);
	c |= SW_MAN_RST;
	sp_write_reg(RX_P0, HDMI_RX_SRST_REG, c);
	msleep(1);
	c  &= ~SW_MAN_RST;
	sp_write_reg(RX_P0, HDMI_RX_SRST_REG, c);

	sp_read_reg(RX_P0, HDMI_RX_SRST_REG, &c);
	c |= TMDS_RST;
	sp_write_reg(RX_P0, HDMI_RX_SRST_REG, c);

       c = AEC_EN07 | AEC_EN06 | AEC_EN05 | AEC_EN02;
	sp_write_reg(RX_P0, HDMI_RX_AEC_EN0_REG, c);
	c = AEC_EN12 |AEC_EN10 | AEC_EN09 | AEC_EN08;
	sp_write_reg(RX_P0, HDMI_RX_AEC_EN1_REG, c);
	c = AEC_EN23 |AEC_EN22 | AEC_EN21 | AEC_EN20;
	sp_write_reg(RX_P0, HDMI_RX_AEC_EN2_REG, 0xf0);

	sp_read_reg(RX_P0, HDMI_RX_AEC_CTRL_REG, &c);
	c |=  AVC_EN;
	sp_write_reg(RX_P0, HDMI_RX_AEC_CTRL_REG, c);

	sp_read_reg(RX_P0, HDMI_RX_AEC_CTRL_REG, &c);
	c |= AAC_EN;
	sp_write_reg(RX_P0, HDMI_RX_AEC_CTRL_REG, c);

	sp_read_reg(RX_P0, HDMI_RX_SYS_PWDN1_REG, &c);
	c &= ~PWDN_CTRL;
	sp_write_reg(RX_P0, HDMI_RX_SYS_PWDN1_REG, c);

	sp_write_reg(RX_P0, HDMI_RX_INT_MASK1_REG, 0x00);
	sp_write_reg(RX_P0, HDMI_RX_INT_MASK2_REG, 0x00);
	sp_write_reg(RX_P0, HDMI_RX_INT_MASK3_REG, 0x00);
	sp_write_reg(RX_P0, HDMI_RX_INT_MASK4_REG, 0x00);
	sp_write_reg(RX_P0, HDMI_RX_INT_MASK5_REG, 0x00);
	sp_write_reg(RX_P0, HDMI_RX_INT_MASK6_REG, 0x00);
	sp_write_reg(RX_P0, HDMI_RX_INT_MASK7_REG, 0x00);


	/* Range limitation for RGB input */
	sp_read_reg(RX_P0, HDMI_RX_VID_DATA_RNG_CTRL_REG, &c);
	c |= R2Y_INPUT_LIMIT;
	sp_write_reg(RX_P0, HDMI_RX_VID_DATA_RNG_CTRL_REG, c);

       c = CEC_RST;
	sp_write_reg(RX_P0, HDMI_RX_CEC_CTRL_REG, c);
	c = CEC_SPEED_27M;
	sp_write_reg(RX_P0, HDMI_RX_CEC_SPEED_CTRL_REG, c);
	c = CEC_RX_EN;
	sp_write_reg(RX_P0, HDMI_RX_CEC_CTRL_REG, c);
	hdmi_rx_tmds_phy_initialization();
	
	if(sp_tx_get_asus_id())
			sp_tx_asus_pad=1;
	else 
			sp_tx_asus_pad=0;

	
	if(!sp_tx_asus_pad) {
	hdmi_rx_set_hpd(0);
	hdmi_rx_set_termination(0);
	}
	
	DEV_NOTICE("HDMI Rx is initialized...\n"); 
}

static void hdmi_rx_clk_det_int(void)
{
	unchar c;

	DEV_NOTICE("*HDMI_RX Interrupt: Pixel Clock Change.\n");
	if (sp_tx_system_state > STATE_CONFIG_HDMI) {
		hdmi_rx_mute_audio();
		hdmi_rx_mute_video();
		sp_tx_video_mute(1);
		sp_tx_enable_video_input(0);
		sp_tx_enable_audio_output(0);
		sp_tx_set_sys_state(STATE_CONFIG_HDMI);

		if (hdmi_system_state > HDMI_CLOCK_DET)
			hdmi_rx_set_sys_state(HDMI_CLOCK_DET);
	}

	sp_read_reg(RX_P0, HDMI_RX_SYS_STATUS_REG, &c);

	if (c & TMDS_CLOCK_DET) {
		DEV_ERR("Pixel clock existed.\n");

		if (hdmi_system_state == HDMI_CLOCK_DET)
			hdmi_rx_set_sys_state(HDMI_SYNC_DET);
	} else {
		if (hdmi_system_state > HDMI_CLOCK_DET)
			hdmi_rx_set_sys_state(HDMI_CLOCK_DET);
		DEV_ERR("Pixel clock lost.\n");
		g_sync_expire_cntr = 0;
	}
}

static void hdmi_rx_sync_det_int(void)
{
	unchar c;

	DEV_NOTICE("*HDMI_RX Interrupt: Sync Detect.");

	if (sp_tx_system_state > STATE_CONFIG_HDMI) {
		hdmi_rx_mute_audio();
		hdmi_rx_mute_video();
		sp_tx_video_mute(1);
		sp_tx_enable_video_input(0);
		sp_tx_enable_audio_output(0);
		sp_tx_set_sys_state(STATE_CONFIG_HDMI);

		if (hdmi_system_state > HDMI_SYNC_DET)
			hdmi_rx_set_sys_state(HDMI_SYNC_DET);
	}

	sp_read_reg(RX_P0, HDMI_RX_SYS_STATUS_REG, &c);
	if (c & TMDS_DE_DET) {
		DEV_NOTICE("Sync found.");

		if (hdmi_system_state == HDMI_SYNC_DET)
			hdmi_rx_set_sys_state(HDMI_VIDEO_CONFIG);

		g_video_stable_cntr = 0;
		hdmi_rx_get_video_info();
	} else {
		DEV_ERR("Sync lost.");

		if ((c & TMDS_CLOCK_DET) &&
			(hdmi_system_state > HDMI_SYNC_DET))
			hdmi_rx_set_sys_state(HDMI_SYNC_DET);
		else
			hdmi_rx_set_sys_state(HDMI_CLOCK_DET);
	}
}

static void hdmi_rx_hdmi_dvi_int(void)
{
	unchar c;

	DEV_NOTICE("*HDMI_RX Interrupt: HDMI-DVI Mode Change.");
	sp_read_reg(RX_P0, HDMI_RX_HDMI_STATUS_REG, &c);
	hdmi_rx_get_video_info();

	if ((c & HDMI_MODE) == HDMI_MODE) {
		DEV_NOTICE("hdmi_rx_hdmi_dvi_int: HDMI MODE.");

		if (hdmi_system_state == HDMI_PLAYBACK)
			hdmi_rx_set_sys_state(HDMI_AUDIO_CONFIG);
	} else {
		hdmi_rx_unmute_audio();
	}
}

static void hdmi_rx_avmute_int(void)
{
	unchar avmute_status, c;

	sp_read_reg(RX_P0, HDMI_RX_HDMI_STATUS_REG,
		       &avmute_status);
	if (avmute_status & MUTE_STAT) {
		DEV_NOTICE("HDMI_RX AV mute packet received.");

		if (!g_video_muted)
			hdmi_rx_mute_video();

		if (!g_audio_muted)
			hdmi_rx_mute_audio();

		c = avmute_status & (~MUTE_STAT);
		sp_write_reg(RX_P0, HDMI_RX_HDMI_STATUS_REG, c);
	}

}

static void hdmi_rx_cts_rcv_int(void)
{
	unchar c;
	g_cts_got = 1;
	sp_read_reg(RX_P0, HDMI_RX_SYS_STATUS_REG, &c);

	if ((hdmi_system_state == HDMI_AUDIO_CONFIG)
		&& (c & TMDS_DE_DET)) {
		if (g_cts_got && g_audio_got) {
			if (g_audio_stable_cntr >= AUDIO_STABLE_TH) {
				hdmi_rx_unmute_audio();
				hdmi_rx_unmute_video();
				g_audio_stable_cntr = 0;
				hdmi_rx_show_audio_info();
				hdmi_rx_set_sys_state(HDMI_PLAYBACK);
				sp_tx_config_audio();
			} else {
				g_audio_stable_cntr++;
			}
		} else {
			g_audio_stable_cntr = 0;
		}
	}
}

static void hdmi_rx_audio_rcv_int(void)
{
	unchar c;
	g_audio_got = 1;

	sp_read_reg(RX_P0, HDMI_RX_SYS_STATUS_REG, &c);

	if ((hdmi_system_state == HDMI_AUDIO_CONFIG)
		&& (c & TMDS_DE_DET)) {
		if (g_cts_got && g_audio_got) {
			if (g_audio_stable_cntr >= AUDIO_STABLE_TH) {
				hdmi_rx_unmute_audio();
				hdmi_rx_unmute_video();
				g_audio_stable_cntr = 0;
				hdmi_rx_show_audio_info();
				hdmi_rx_set_sys_state(HDMI_PLAYBACK);
				sp_tx_config_audio();
			} else {
				g_audio_stable_cntr++;
			}
		} else {
			g_audio_stable_cntr = 0;
		}
	}
}

static void hdmi_rx_hdcp_error_int(void)
{
	g_audio_got = 0;
	g_cts_got = 0;

	if (g_hdcp_err_cnt >= 40) {
		g_hdcp_err_cnt = 0;
		DEV_ERR("Lots of hdcp error occured ...");
		hdmi_rx_mute_audio();
		hdmi_rx_mute_video();

		/* issue hotplug */
		hdmi_rx_set_hpd(0);
		msleep(10);
		hdmi_rx_set_hpd(1);

	} else if ((hdmi_system_state == HDMI_CLOCK_DET)
		   || (hdmi_system_state == HDMI_SYNC_DET)) {
		g_hdcp_err_cnt = 0;
	} else {
		g_hdcp_err_cnt++;
	}

}



static void hdmi_rx_new_avi_int(void)
{

	DEV_NOTICE("*HDMI_RX Interrupt: New AVI Packet.");
	sp_tx_avi_setup();
	sp_tx_config_packets(AVI_PACKETS);
}

static void hdmi_rx_new_gcp_int(void)
{
	unchar c;
	sp_read_reg(RX_P1, HDMI_RX_GENERAL_CTRL, &c);
	if (c&SET_AVMUTE) {
		if (!g_video_muted)
			hdmi_rx_mute_video();
		if (!g_audio_muted)
			hdmi_rx_mute_audio();
		
	} else if (c&CLEAR_AVMUTE) {
		if ((g_video_muted) &&
			(hdmi_system_state >HDMI_VIDEO_CONFIG))
			hdmi_rx_unmute_video();
		if ((g_audio_muted) &&
			(hdmi_system_state >HDMI_AUDIO_CONFIG))
			hdmi_rx_unmute_audio();
	}
}

static void hdmi_rx_new_vsi_int(void)
{
	unchar c;
	unchar hdmi_video_format,vsi_header,v3d_structure;
	DEV_ERR("*HDMI_RX Interrupt: NEW VSI packet.\n");
	sp_read_reg(TX_P0, SP_TX_3D_VSC_CTRL, &c);
		if (!(c&INFO_FRAME_VSC_EN)) {
			sp_read_reg(RX_P1, HDMI_RX_MPEG_TYPE_REG, &vsi_header);
			sp_read_reg(RX_P1, HDMI_RX_MPEG_DATA03_REG,
				&hdmi_video_format);
			if ((vsi_header == 0x81) &&
				((hdmi_video_format & 0xe0) == 0x40)) {
				DEV_DBG("3D VSI packet is detected. Config VSC packet\n");
				/*use mpeg packet as mail box
				to send vsi packet*/
				sp_tx_vsi_setup();
				sp_tx_config_packets(VSI_PACKETS);


				sp_read_reg(RX_P1, HDMI_RX_MPEG_DATA05_REG,
					&v3d_structure);
			switch(v3d_structure&0xf0){
			case 0x00://frame packing
				v3d_structure = 0x02;
				break;
			case 0x20://Line alternative
				v3d_structure = 0x03;
				break;
			case 0x30://Side-by-side(full)
				v3d_structure = 0x04;
				break;
			default:
				v3d_structure = 0x00;
				DEV_ERR("3D structure is not supported\n");
				break;
			}

				sp_write_reg(TX_P0, SP_TX_VSC_DB1,
					v3d_structure);
			sp_read_reg(TX_P0, SP_TX_3D_VSC_CTRL, &c);
			c |= INFO_FRAME_VSC_EN;
			sp_write_reg(TX_P0, SP_TX_3D_VSC_CTRL, c);

			sp_read_reg(TX_P0, SP_TX_PKT_EN_REG, &c);
			c &= ~SPD_IF_EN;
			sp_write_reg(TX_P0, SP_TX_PKT_EN_REG, c);

			sp_read_reg(TX_P0, SP_TX_PKT_EN_REG, &c);
			c |= SPD_IF_UD;
			sp_write_reg(TX_P0, SP_TX_PKT_EN_REG, c);

			sp_read_reg(TX_P0, SP_TX_PKT_EN_REG, &c);
			c |= SPD_IF_EN;
			sp_write_reg(TX_P0, SP_TX_PKT_EN_REG, c);

		}

	}
}

static void hdmi_rx_no_vsi_int(void)
{
	unchar c;
	sp_read_reg(TX_P0, SP_TX_3D_VSC_CTRL, &c);
		if (c&INFO_FRAME_VSC_EN) {
		DEV_ERR("No new VSI is received, disable  VSC packet\n");
		c &= ~INFO_FRAME_VSC_EN;
		sp_write_reg(TX_P0, SP_TX_3D_VSC_CTRL, c);
		sp_tx_mpeg_setup();
		sp_tx_config_packets(MPEG_PACKETS);
	}

}

//ASUS_BSP: joe1_++: for hpd re-check
#define HPD_RECHECK 10
#ifdef CONFIG_ASUS_HDMI
extern void hdmi_hpd_state_recheck(void);
#endif
//ASUS_BSP: joe1_--: for hpd re-check

int sp_tx_config_hdmi_input(void)
{
	unchar c;
    unchar avmute_status,sys_status;
	unchar return_value = 1;
	//static int hdmi_tryCount=0;
	static int iFailCount = 0; //ASUS_BSP: joe1_++: for hpd re-check
	unchar video_stable_cntr = VIDEO_STABLE_TH;
	
//	hdmi_tryCount++;
	//if(hdmi_tryCount%10==0|| hdmi_tryCount<5 )
	//	DEV_NOTICE("sp_tx_config_hdmi_input +++");
	
	sp_read_reg(RX_P0,HDMI_RX_SYS_STATUS_REG, &sys_status);
	if ((sys_status & TMDS_CLOCK_DET)
	    && (hdmi_system_state == HDMI_CLOCK_DET))
		hdmi_rx_set_sys_state(HDMI_SYNC_DET);

	if (hdmi_system_state == HDMI_SYNC_DET) {
		sp_read_reg(RX_P0, HDMI_RX_SYS_STATUS_REG, &c);

		if (!(c & TMDS_DE_DET)) {
			if (g_sync_expire_cntr >= SCDT_EXPIRE_TH) {
				DEV_ERR("No sync for long time.");
				/* misc reset */
				sp_read_reg(RX_P0, HDMI_RX_TMDS_CTRL_REG18, &c);
				c |= PLL_RESET;
				sp_write_reg(RX_P0, HDMI_RX_TMDS_CTRL_REG18, c);
				msleep(2);
				sp_read_reg(RX_P0, HDMI_RX_TMDS_CTRL_REG18, &c);
				c &= ~PLL_RESET;
				sp_write_reg(RX_P0, HDMI_RX_TMDS_CTRL_REG18, c);
				hdmi_rx_set_sys_state(HDMI_CLOCK_DET);
				g_sync_expire_cntr = 0;
			} else {
				g_sync_expire_cntr++;
			}
				goto error; //ASUS_BSP: joe1_++: for hpd re-check
		} else {
			g_sync_expire_cntr = 0;
			hdmi_rx_set_sys_state(HDMI_VIDEO_CONFIG);
		}

	}

	if (hdmi_system_state < HDMI_VIDEO_CONFIG)
	{
		goto error; //ASUS_BSP: joe1_++: for hpd re-check
	}

	if (sp_tx_asus_pad)
	{
		video_stable_cntr =  1; // ASUS_BSP : larry : pad mode only count 1 times for speed up performance
		DEV_NOTICE("WAIT_VIDEO: pad mode skip count video.\n");		
	}

	if (hdmi_rx_is_video_change()) {
		DEV_ERR("Video Changed , mute video and mute audio");
		g_video_stable_cntr = 0;

		if (!g_video_muted)
			hdmi_rx_mute_video();
		
		if (!sp_tx_asus_pad) {
			if (!g_audio_muted)
				hdmi_rx_mute_audio();
		}
		
	} else if (g_video_stable_cntr < video_stable_cntr) {
		g_video_stable_cntr++;
		DEV_NOTICE("WAIT_VIDEO: Wait for video stable cntr.\n");
	} else if (hdmi_system_state == HDMI_VIDEO_CONFIG) {
		sp_read_reg(RX_P0,HDMI_RX_HDMI_STATUS_REG, &avmute_status);
		if (!(avmute_status & MUTE_STAT)) {
			if (!sp_tx_asus_pad) {
				hdmi_rx_get_video_info();
				hdmi_rx_unmute_video();
				sp_tx_lvttl_bit_mapping();
				sp_tx_set_sys_state(STATE_LINK_TRAINING);
				hdmi_rx_show_video_info();
				sp_tx_power_down(SP_TX_PWR_AUDIO);

			if (g_hdmi_dvi_status) {
				DEV_NOTICE("HDMI mode: Video is stable.");
				sp_tx_send_message(MSG_INPUT_HDMI);
				hdmi_rx_set_sys_state(HDMI_AUDIO_CONFIG);
			} else {
				DEV_NOTICE("DVI mode: Video is stable.");
				sp_tx_send_message(MSG_INPUT_DVI);
				hdmi_rx_unmute_audio();
				hdmi_rx_set_sys_state(HDMI_PLAYBACK);
			}
		}
			return_value = 0;
		}
	}

	hdmi_rx_get_video_info();

//ASUS_BSP: joe1_++: for hpd re-check
error:
	if (return_value)
	{
		iFailCount++;

		if ( (iFailCount % HPD_RECHECK) == 0 )
		{
			ASUS_DEV_WARN("%s: iFailCount=%d; recheck hpd status!  (return_value=%d) \n", __func__, iFailCount,return_value);
#ifdef CONFIG_ASUS_HDMI
		//	hdmi_hpd_state_recheck();
#endif
		}
	}
	else
	{
		iFailCount = 0;
	//	hdmi_tryCount=0;
		ASUS_DEV_INFO("%s: clear iFailCount=>%d  (return_value=%d) \n", __func__, iFailCount,return_value);
	}
//ASUS_BSP: joe1_++: for hpd re-check

//	if(hdmi_tryCount%10==0 || hdmi_tryCount<5)
//		DEV_NOTICE("sp_tx_config_hdmi_input (return_value=%d)---", return_value);
	return return_value;
}
//ANX +++: (ver0.4)			

extern int g_Pad_LT_Fail_Count;
extern void reportPadStationI2CFail(char *devname);
unchar sp_tx_config_hdmi_pad(void)
{

	unchar c;
	unchar bSwing;

	DEV_NOTICE("sp_tx_config_hdmi_pad+++\n");


	sp_write_reg(RX_P0, HDMI_RX_HDMI_MUTE_CTRL_REG, 0x00);  //unmute_video_audio

	sp_write_reg(TX_P2, SP_TX_VID_CTRL1_REG, 0x03); //SDR, bit sel

	sp_write_reg(TX_P2, SP_TX_VID_CTRL2_REG, 0x10); //8bits

	/* bit mapping
	for (c = 0; c < 24; c++) {
	vid_bit = SP_TX_VID_BIT_CTRL0_REG + c;
	value = c;
	sp_write_reg(TX_P2, vid_bit, value);
	}

	*/
	//hdmi_rx_show_video_info();
	/*power down audio*/
	sp_write_reg(TX_P2, SP_POWERD_CTRL_REG, 0x10);

	/*sp_tx_send_message(MSG_INPUT_DVI);*/
	if (!myDP_DP_Dongle)
	{
	// specific command to inform 7730 that upstream is DVI format or HDMI format
		sp_tx_aux_dpcdwrite_byte(0x00, 0x05, 0x26, 0x00);
	}
	//hdmi_rx_unmute_audio();
	//hdmi_rx_set_sys_state(HDMI_PLAYBACK);

	sp_tx_bw=BW_54G;

//	sp_write_reg(TX_P2, SP_TX_VID_CTRL1_REG, 0x83); //enable video input
/*
	for (i=0;i<8;i++)
	{
		sp_read_reg(TX_P0, SP_TX_SYS_CTRL3_REG, &c);
		if (!(c & STRM_VALID))
			return 1; 
		else
			break;
	}

*/
	sp_write_reg(TX_P0, SP_TX_LINK_BW_SET_REG, 0x14);		

	/*Diable video before link training to enable idle pattern*/
	sp_write_reg(TX_P2, SP_TX_VID_CTRL1_REG, 0x03);

	/*enable SSC*/
	sp_write_reg(TX_P0, SSC_CTRL_REG1, 0x00);
/*	
	DEV_DBG("*** Config SSC 0.9 ***");
	
	sp_write_reg(TX_P0, SP_TX_DOWN_SPREADING_CTRL1, 0xdd);
	sp_write_reg(TX_P0, SP_TX_DOWN_SPREADING_CTRL2, 0x01);
	sp_write_reg(TX_P0, SP_TX_DOWN_SPREADING_CTRL3, 0x76);
*/	
	ASUS_DEV_INFO("*** Config SSC 0.4 ***");

	if(g_enable_dynamic_ssc){
		DEV_NOTICE("%s: enable dynamic ssc SPREADING_CTRL1:%d,SPREADING_CTRL2:%d,SPREADING_CTRL3:%d, \n",__func__,spreading_ctrl1,spreading_ctrl2,spreading_ctrl3);
		sp_write_reg(TX_P0, SP_TX_DOWN_SPREADING_CTRL1, spreading_ctrl1);
		sp_write_reg(TX_P0, SP_TX_DOWN_SPREADING_CTRL2, spreading_ctrl2);
		sp_write_reg(TX_P0, SP_TX_DOWN_SPREADING_CTRL3, spreading_ctrl3);	
	}else {
		sp_write_reg(TX_P0, SP_TX_DOWN_SPREADING_CTRL1, 0xc0);
		sp_write_reg(TX_P0, SP_TX_DOWN_SPREADING_CTRL2, 0x00);
		sp_write_reg(TX_P0, SP_TX_DOWN_SPREADING_CTRL3, 0x75);	
	}
	
	sp_write_reg(TX_P0, SSC_CTRL_REG1, 0x10); 

	sp_read_reg(TX_P2, SP_TX_RST_CTRL2_REG, &c);
	c |= SSC_RST;
	sp_write_reg(TX_P2, SP_TX_RST_CTRL2_REG, c);
	c &= ~SSC_RST;
	sp_write_reg(TX_P2, SP_TX_RST_CTRL2_REG, c);

	sp_tx_aux_dpcdread_bytes(0x00, 0x01,
	DPCD_DOWNSPREAD_CTRL, 1, &c);
	c |= SPREAD_AMPLITUDE;
	sp_tx_aux_dpcdwrite_byte(0x00, 0x01, DPCD_DOWNSPREAD_CTRL, c);

	sp_write_reg(TX_P0, SP_TX_ANALOG_PD_REG, 0x01);
	msleep(1);
	sp_write_reg(TX_P0, SP_TX_ANALOG_PD_REG, 0x00);

	sp_write_reg(TX_P0, SP_TX_PLL_CTRL_REG, 0x50);
	msleep(1);
	sp_write_reg(TX_P0, SP_TX_PLL_CTRL_REG, 0x10);
	sp_tx_link_config_done = 1;

	if (g_force_swing_value)
	{
//		g_swing_value = 2;   // 600 mv
//		g_pre_emphis_value = 1;   // 3.5db
		
		sp_write_reg(TX_P0, 0xA3, (g_swing_value)  | (g_pre_emphis_value << 3) );	
		sp_read_reg(TX_P0, 0xA3, &bSwing);
		ASUS_DEV_WARN("@@@ lane0,Swing/Emp = %x @@@\n", bSwing);	  //0x2E , 600 mv , 3.5db	 
	}
	else
	{
		ASUS_DEV_WARN("@@@ not force swing value @@@\n");			
	}

	sp_tx_enhancemode_set();

	sp_tx_aux_dpcdread_bytes(0x00, 0x06, 0x00, 0x01, &c);
	c |= 0x01;
	sp_tx_aux_dpcdwrite_byte(0x00, 0x06, 0x00, c);

	sp_read_reg(TX_P2, SP_INT_MASK, &c);
	sp_write_reg(TX_P2, SP_INT_MASK, c |0X20);
	sp_write_reg(TX_P0, SP_TX_LT_CTRL_REG, SP_TX_LT_EN);


//	if (!myDP_DP_Dongle)
	{	
		pad_sp_tx_lt_done_int_handler();

		sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x02, 1, bytebuf);		
		ASUS_DEV_WARN("sp_tx_config_hdmi_pad , LANE0_1_STATUS = 0x%x\n", bytebuf[0]);
		
		if ((bytebuf[0] & 0x07) != 0x07) {
			sp_tx_hw_lt_enable = 0;
			sp_tx_hw_lt_done = 0;
			return 1;
		} else {
			sp_tx_hw_lt_done = 1;
			sp_read_reg(TX_P0, 0xA3, &bSwing);	
			ASUS_DEV_WARN("after link training, lane0,Swing/Emp = %d\n", bSwing);	
			if (myDP_DP_Dongle)
			{
	                        if (g_force_swing_value)
        	                {
					if (bSwing != 0x2E)
					{
						if (bSwing == 0x0 && g_Pad_LT_Fail_Count<5)	//ASUS BSP wei +++
						{
					                     sp_tx_hw_lt_enable = 0;
	                        				sp_tx_hw_lt_done = 0;
								g_Pad_LT_Fail_Count++;	//ASUS BSP wei +++
                        				return 1;							
						}
						//ASUS BSP wei +++
						if(g_Pad_LT_Fail_Count>=5){
							g_Pad_LT_Fail_Count=0;
							reportPadStationI2CFail("MyDP");
							DEV_NOTICE("____________Hardware LT Fail in Pad mode \n");
							return 1;	
						}
						//ASUS BSP wei ---
						sp_write_reg(TX_P0, 0xA3, (g_swing_value)  | (g_pre_emphis_value << 3) );	
						sp_read_reg(TX_P0, 0xA3, &bSwing);
						ASUS_DEV_WARN("lane0,Swing/Emp = %x @@@\n", bSwing);	  //0x2E , 600 mv , 3.5db	 
						sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x06, 1, &c);
				              ASUS_DEV_WARN("sp_tx_config_hdmi_pad , ADJUST_REQUEST_LANE0_1 = 0x%x\n", c);
					}
				}
			}
			sp_tx_set_sys_state(STATE_CONFIG_OUTPUT);
		}
	}
	DEV_NOTICE("sp_tx_config_hdmi_pad---\n");
	
	return 0;

} 
//ANX ---: (ver0.4)			

void hdmi_rx_int_irq_handler(void)
{
	unchar c1, c2, c3, c4, c5, c6, c7;

//ANX +++: (ver0.4)			
	if ((hdmi_system_state < HDMI_CLOCK_DET)
	    || (sp_tx_system_state < STATE_CONFIG_HDMI))
//larry debug irq err handle for pad mode	    
//	    ||(sp_tx_asus_pad))
		return;
//ANX ---: (ver0.4)			

	sp_read_reg(RX_P0, HDMI_RX_INT_STATUS1_REG, &c1);
	sp_write_reg(RX_P0, HDMI_RX_INT_STATUS1_REG, c1);

	sp_read_reg(RX_P0, HDMI_RX_INT_STATUS2_REG, &c2);
	sp_write_reg(RX_P0, HDMI_RX_INT_STATUS2_REG, c2);

	sp_read_reg(RX_P0, HDMI_RX_INT_STATUS3_REG, &c3);
	sp_write_reg(RX_P0, HDMI_RX_INT_STATUS3_REG, c3);

	sp_read_reg(RX_P0, HDMI_RX_INT_STATUS4_REG, &c4);
	sp_write_reg(RX_P0, HDMI_RX_INT_STATUS4_REG, c4);

	sp_read_reg(RX_P0, HDMI_RX_INT_STATUS5_REG, &c5);
	sp_write_reg(RX_P0, HDMI_RX_INT_STATUS5_REG, c5);

	sp_read_reg(RX_P0, HDMI_RX_INT_STATUS6_REG, &c6);
	sp_write_reg(RX_P0, HDMI_RX_INT_STATUS6_REG, c6);

	sp_read_reg(RX_P0, HDMI_RX_INT_STATUS7_REG, &c7);
	sp_write_reg(RX_P0, HDMI_RX_INT_STATUS7_REG, c7);

       if (sp_tx_asus_pad) {

		if ((c1!=0x0||c2!=0x0||c3!=0x0||c4!=0x80||c5!=0x0|| (c6!=0x1 && c6!=0x0) ||c7!=0x20))
		{
            DEV_DBG("hdmi rx :c1=%x,c2=%x,c3=%x,c4=%x,c5=%x,c6=%x,c7=%x\n",c1,c2,c3,c4,c5,c6,c7);

		    //ASUS_BSP larry add event log +++
		    if (g_hdmi_rx_vsync_change < 0xFF)
			g_hdmi_rx_vsync_change++;

		    if (g_hdmi_rx_vsync_change > 1)
		    	{
			    //ASUSEvtlog("[myDP]hdmi rx:c4=0x%x,vsync change=0x%x\n",c4, g_hdmi_rx_vsync_change);
		        DEV_DBG("vsync change = 0x%x\n", g_hdmi_rx_vsync_change);					 
		    	}
			
		    //ASUS_BSP larry add event log ---
		}
			//ASUS BSP wei +++
		  	if ( (c2 & HDCP_ERR)&& sp_tx_system_state> STATE_CONFIG_OUTPUT)   {
						DEV_NOTICE("%s: HDMI to MyDP HDCP Failed\n",__func__);
						sp_tx_sw_error_power_down();
#if 0
						hdmi_rx_mute_audio();
						hdmi_rx_mute_video();
						sp_tx_set_sys_state(STATE_LINK_TRAINING);

#endif
		  	}
			//ASUS BSP wei +++
		return; // pad mode , not handle HDMI RX int
	}
	if (c1 & CKDT_CHANGE)
		hdmi_rx_clk_det_int();

	if (c1 & SCDT_CHANGE)  
		hdmi_rx_sync_det_int();

	if (c1 & HDMI_DVI)          
		hdmi_rx_hdmi_dvi_int();

	if (c1 & SET_MUTE)           
		hdmi_rx_avmute_int();

	if (c6 & NEW_AVI) 
		hdmi_rx_new_avi_int();
	
	if (c7 & NEW_VS)
		hdmi_rx_new_vsi_int();

	if (c7 & NO_VSI) 
		hdmi_rx_no_vsi_int();
	

	if((c6 & NEW_AUD) || (c3 & AUD_MODE_CHANGE)) 
		hdmi_rx_restart_audio_chk();


	if (c6 & CTS_RCV)  
		hdmi_rx_cts_rcv_int();
	

	if (c5 & AUDIO_RCV)  
		hdmi_rx_audio_rcv_int();


	if ( c2 & HDCP_ERR)   
	    hdmi_rx_hdcp_error_int();

	if (c6 & NEW_CP) 
		hdmi_rx_new_gcp_int();
}

void sp_tx_phy_auto_test(void)
{

	unchar bSwing, bEmp;
	unchar c1, c;
	/*DPCD 0x219 TEST_LINK_RATE*/
	sp_tx_aux_dpcdread_bytes(0x0, 0x02, 0x19, 1, bytebuf);
	DEV_DBG("DPCD:0x00219 = %.2x\n", (uint)bytebuf[0]);
	switch (bytebuf[0]) {
	case 0x06:
		sp_write_reg(TX_P0, SP_TX_LINK_BW_SET_REG, 0x06);
		DEV_DBG("test BW= 1.62Gbps\n");
		break;
	case 0x0a:
		sp_write_reg(TX_P0, SP_TX_LINK_BW_SET_REG, 0x0a);
		DEV_DBG("test BW= 2.7Gbps\n");
		break;
	case 0x14:
		sp_write_reg(TX_P0, SP_TX_LINK_BW_SET_REG, 0x14);
		DEV_DBG("test BW= 5.4Gbps\n");
		break;
	}
	/*DPCD 0x248 PHY_TEST_PATTERN*/
	sp_tx_aux_dpcdread_bytes(0x0, 0x02, 0x48, 1, bytebuf);
	DEV_DBG("DPCD:0x00248 = %.2x\n", (uint)bytebuf[0]);
	switch (bytebuf[0]) {
	case 0:
		DEV_DBG("No test pattern selected\n");
		break;
	case 1:
		sp_write_reg(TX_P0, SP_TX_TRAINING_PTN_SET_REG, 0x04);
		DEV_DBG("D10.2 Pattern\n");
		break;
	case 2:
		sp_write_reg(TX_P0, SP_TX_TRAINING_PTN_SET_REG, 0x08);
		DEV_DBG("Symbol Error Measurement Count\n");
		break;
	case 3:
		sp_write_reg(TX_P0, SP_TX_TRAINING_PTN_SET_REG, 0x0c);
		DEV_DBG("PRBS7 Pattern\n");
		break;
	case 4:
		sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x50, 0xa, bytebuf);
		sp_write_reg(TX_P1, 0x80, bytebuf[0]);
		sp_write_reg(TX_P1, 0x81, bytebuf[1]);
		sp_write_reg(TX_P1, 0x82, bytebuf[2]);
		sp_write_reg(TX_P1, 0x83, bytebuf[3]);
		sp_write_reg(TX_P1, 0x84, bytebuf[4]);
		sp_write_reg(TX_P1, 0x85, bytebuf[5]);
		sp_write_reg(TX_P1, 0x86, bytebuf[6]);
		sp_write_reg(TX_P1, 0x87, bytebuf[7]);
		sp_write_reg(TX_P1, 0x88, bytebuf[8]);
		sp_write_reg(TX_P1, 0x89, bytebuf[9]);
		sp_write_reg(TX_P0, SP_TX_TRAINING_PTN_SET_REG, 0x30);
		DEV_DBG("80bit custom pattern transmitted\n");
		break;
	case 5:
		sp_write_reg(TX_P0, 0xA9, 0x00);
		sp_write_reg(TX_P0, 0xAA, 0x01);
		sp_write_reg(TX_P0, SP_TX_TRAINING_PTN_SET_REG, 0x14);
		DEV_DBG("HBR2 Compliance Eye Pattern\n");
		break;
	}
	sp_tx_aux_dpcdread_bytes(0x00, 0x00, 0x03, 1, bytebuf);
	DEV_DBG("DPCD:0x00003 = %.2x\n", (uint)bytebuf[0]);
	switch (bytebuf[0] & 0x01) {
	case 0:
		sp_tx_spread_enable(0);
		DEV_DBG("SSC OFF\n");
		break;
	case 1:
		sp_read_reg(TX_P0, SP_TX_LINK_BW_SET_REG, &c1);
		sp_tx_bw = c1;
		sp_tx_config_ssc(sp_tx_bw);
		DEV_DBG("SSC ON\n");
		break;
	}
	/*get swing adjust request*/
	sp_tx_aux_dpcdread_bytes(0x00, 0x02, 0x06, 1, bytebuf);
	DEV_DBG("DPCD:0x00206 = %.2x\n", (uint)bytebuf[0]);
	c1 = bytebuf[0] & 0x03;
	sp_read_reg(TX_P0, 0xA2, &c);
	if ((c == 0x0c) &&
		((bytebuf[0]&0x0f) == 0x02)) {
		/*PRBS7 pattern, eye diagram, swing2/emp 0*/
		sp_write_reg(TX_P0, 0xA3, 0x0a);
		DEV_DBG("eye request,lane0,400/6db\n");
	} else if (c == 0x14) {
		/*CEP pattern*/
		sp_write_reg(TX_P0, 0xA3, 0x0a);
		DEV_DBG("cep pattern,lane0,600/3.5db\n");
	} else {
		switch (c1) {
		case 0x00:
			sp_read_reg(TX_P0, 0xA3, &bSwing);
			sp_write_reg(TX_P0, 0xA3, (bSwing&~0x03)|0x00);
			DEV_DBG("lane0,Swing200mv\n");
			break;
		case 0x01:
			sp_read_reg(TX_P0, 0xA3, &bSwing);
			sp_write_reg(TX_P0, 0xA3, (bSwing&~0x03)|0x01);
			DEV_DBG("lane0,Swing400mv\n");
			break;
		case 0x02:
			sp_read_reg(TX_P0, 0xA3, &bSwing);
			sp_write_reg(TX_P0, 0xA3, (bSwing&~0x03)|0x02);
			DEV_DBG("lane0,Swing600mv\n");
			break;
		case 0x03:
			sp_read_reg(TX_P0, 0xA3, &bSwing);
			sp_write_reg(TX_P0, 0xA3, (bSwing&~0x03)|0x03);
			DEV_DBG("lane0,Swing800mv\n");
			break;
		default:
			break;
		}
		/*get emphasis adjust request*/
		c1 = (bytebuf[0] & 0x0c);
		c1 = c1 >> 2;
		switch (c1) {
		case 0x00:
			sp_read_reg(TX_P0, 0xA3, &bEmp);
			sp_write_reg(TX_P0, 0xA3, (bEmp&~0x18)|0x00);
			DEV_DBG("lane0,emp 0db\n");
			break;
		case 0x01:
			sp_read_reg(TX_P0, 0xA3, &bEmp);
			sp_write_reg(TX_P0, 0xA3, (bEmp&~0x18)|0x08);
			DEV_DBG("lane0,emp 3.5db\n");
			break;
		case 0x02:
			sp_read_reg(TX_P0, 0xA3, &bEmp);
			sp_write_reg(TX_P0, 0xA3, (bEmp&~0x18)|0x10);
			DEV_DBG("lane0,emp 6db\n");
			break;

		default:
			break;
		}
	}

}

void sp_tx_sw_error_power_down(void)
{
	sp_tx_vbus_powerdown();
//ANX : (ver0.4)					
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
//ANX : (ver:20130105) pad solution					
	sp_tx_asus_pad = 0;
	sp_tx_set_sys_state(STATE_CABLE_PLUG);
}

//ASUS BSP wei +++

void sp_tx_hdmi_error_power_down(void)
{
	mutex_lock(&g_anx7808->lock);
	if(g_anx7808 && sp_tx_pd_mode==0){
		ASUS_DEV_WARN("HDMI call MyDP error power down\n");
		sp_tx_vbus_powerdown();
	//ANX : (ver0.4)					
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
	//ANX : (ver:20130105) pad solution					
		sp_tx_asus_pad = 0;
		sp_tx_set_sys_state(STATE_CABLE_PLUG);
		
	}
	mutex_unlock(&g_anx7808->lock);
}
//ASUS BSP wei ---

//+++ ASUS BSP Bernard, use to select different bw
unchar sp_get_link_bw(void)
{
	unchar c;
	unchar link_bw = 0;

	if(sp_tx_asus_pad){
		link_bw=BW_54G;
	}else{
		mutex_lock(&g_anx7808->lock);
		sp_tx_get_rx_bw(1, &c);
		mutex_unlock(&g_anx7808->lock);
		switch (c) {
			case 0x06:
				link_bw=BW_162G;
				break;
			case 0x0a:
				link_bw=BW_27G;
				break;
			case 0x14:
				link_bw=BW_54G;
				break;
			default:
				link_bw=BW_54G;
				break;
		}
	}
	return link_bw;
}
//--- ASUS BSP Bernard, use to select different bw

MODULE_DESCRIPTION("Slimport transmitter ANX7808 driver");
MODULE_AUTHOR("FeiWang <fwang@analogixsemi.com>");
MODULE_LICENSE("GPL");

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

#ifndef _SLIMPORT_H
#define _SLIMPORT_H
//#define CONFIG_I2C_STRESS_TEST

#include <linux/wakelock.h> //ASUS BSP wei +++
#include <linux/clk.h> //+++ ASUS BSP Bernard

#define DEBUG

#ifdef DEBUG
#define DEV_DBG(args... )  pr_info("[MYDP] " args)
#else
#define DEV_DBG(args... ) (void)0
#endif

#define DEV_NOTICE(args... ) pr_notice("[MYDP] " args)
#define DEV_ERR(args... ) pr_err("[MYDP] " args)

//+++ ASUS BSP Bernard for dts
struct anx7808_platform_data
{
	struct regulator *sp_regulator;
	int gpio_3v3_en_7808;
	int gpio_rst;
	int gpio_cbl_det;
	int gpio_pw_dwn;
	int gpio_usb_id;
//	int gpio_usb_sel;
	u32 gpio_3v3_en_flags;
	u32 gpio_rst_flags;
	u32 gpio_cbl_det_flags;
	u32 gpio_pw_dwn_flags;
	u32 gpio_usb_id_flags;
//	u32 gpio_usb_sel_flags;
	unchar IDL, IDH, Rev; //chip id, reversion
	uint ID;//chip id
};
//--- ASUS BSP Bernard for dts

//ASUS BSP wei lai +++
struct anx7808_data {
	struct anx7808_platform_data    *pdata;
	struct delayed_work    work;
//ASUS BSP wei lai +++
	struct delayed_work    carKitwork;
//ASUS BSP wei lai ---
	struct workqueue_struct    *workqueue;
	struct mutex    lock;
	struct wake_lock slimport_lock;
	struct clk *mydp_diff_clk; //+++ ASUS BSP Bernard
	struct platform_device *hdmi_pdev; //ASUS BSP Wei +++
	struct work_struct vibratorWork;
}; 
//ASUS BSP wei lai ---
#define SSC_EN
#define HDCP_EN
#if 0
#define SSC_1
#define EYE_TEST
#define EDID_DEBUG_PRINT
#endif

#define AUX_ERR  1
#define AUX_OK   0

extern bool  sp_tx_hw_lt_done;
extern bool  sp_tx_hw_lt_enable;
extern bool	sp_tx_link_config_done ;
extern enum SP_TX_System_State sp_tx_system_state;
extern enum RX_CBL_TYPE sp_tx_rx_type;
extern enum RX_CBL_TYPE  sp_tx_rx_type_backup;
extern unchar sp_tx_pd_mode;
//ANX +++: (ver:20130105) EDID read issue with QCOM solution
extern bool sp_tx_asus_pad;
//ANX ---: (ver:20130105) EDID read issue with QCOM solution
extern unchar bedid_break;
extern struct i2c_client *anx7808_client;

int sp_read_reg(uint8_t slave_addr, uint8_t offset, uint8_t *buf);
int sp_write_reg(uint8_t slave_addr, uint8_t offset, uint8_t value);
//+++ ASUS BSP Bernard for mydp 1V control
void sp_hardware_1V_Ctrl(int stat, struct anx7808_platform_data *pdata);
//--- ASUS BSP Bernard for mydp 1V control
void sp_tx_hardware_poweron(struct i2c_client *client);
void sp_tx_hardware_powerdown(struct i2c_client *client);
//ANX +++: (ver:20130105) pad solution
int slimport_read_edid_block(int block, uint8_t *edid_buf);
unchar slimport_get_link_bw(void);
//ANX +++: (ver:20130105) pad solution

#endif

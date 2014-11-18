#ifndef __PF500KL_er1_GPIO_PINMUX_H
#define __PF500KL_er1_GPIO_PINMUX_H

//#include "pf500kl_gpio_pinmux_setting.h"
#include "a91_gpio_pinmux_setting.h"

#define KS8851_IRQ_GPIO 94

static struct msm_gpiomux_config pf500kl_er1_msm8974_gpio_configs[] = {
#if 1 // defined(CONFIG_KS8851) || defined(CONFIG_KS8851_MODULE)
	// +++ ASUS_BSP : add for asus touch
	{
		/* TP IRQ */
		.gpio = 0,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ts_int_cfg,
			[GPIOMUX_ACTIVE]    = &ts_int_cfg,
		},

	},
	// --- ASUS_BSP : add for asus touch
	{
		.gpio      = 1,		/* JACK_IN_DET */
		.settings = {
			[GPIOMUX_SUSPENDED] = &hs_detect,
		},
	},
    //ASUS_BSP: Eason for Wireless Charger IC +++
	{//WC_EN2
		.gpio      = 2,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_i2c_config,
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
		},
	},

	{//WC_EN1      
		.gpio = 3,
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_i2c_config,
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
		},
	},
    //ASUS_BSP: Eason for Wireless Charger IC ---
#endif
	{
		.gpio      = 4,			/* BLSP2 UART TX */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_uart_config,
		},
	},
	{
		.gpio      = 5,			/* BLSP2 UART RX */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_uart_config,
		},
	},
	{
		.gpio      = 6,		/* BLSP1 QUP2 I2C_DAT */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
			[GPIOMUX_ACTIVE]    = &gpio_i2c_config,
		},
	},
	{
		.gpio      = 7,		/* BLSP1 QUP2 I2C_CLK */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
			[GPIOMUX_ACTIVE]    = &gpio_i2c_config,
		},
	},
	{
		/* TP power EN */
		.gpio = 8,
		.settings = {
			[GPIOMUX_SUSPENDED] = &O_H_cfg,
			[GPIOMUX_ACTIVE]    = &O_H_cfg,
		},
	},
#if 0 	
#if defined(CONFIG_KS8851) || defined(CONFIG_KS8851_MODULE)
	{
		.gpio      = 8,		/* BLSP1 QUP SPI_CS1_N */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_spi_cs1_config,
		},
	},
	{
		.gpio      = 9,		/* BLSP1 QUP SPI_CS2A_N */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_spi_cs2_config,
		},
	},
#endif
#endif	// --- ASUS_BSP : add for asus display
	//ASUS_BSP lenter+++
	{ //microp_interrupt
		.gpio = 9,
		.settings = {
			[GPIOMUX_SUSPENDED] = &microp_interrupt,
			[GPIOMUX_ACTIVE]    = &microp_interrupt,
		},
	},
	//ASUS_BSP lenter---
//+++ ASUS BSP Bernard: MyDP I2C
	{
		.gpio      = 10,		/* BLSP1 QUP2 I2C_DAT I2C-3s*/
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
			[GPIOMUX_ACTIVE]    = &gpio_i2c_config,
		},
	},
	{
		.gpio      = 11,		/* BLSP1 QUP2 I2C_CLK I2C-3*/
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
			[GPIOMUX_ACTIVE]    = &gpio_i2c_config,
		},
	},
//--- ASUS BSP Bernard: MyDP I2C
    {
        .gpio      = 12,
        .settings = {
            [GPIOMUX_ACTIVE]    = &mdp_vsync_active_cfg,
            [GPIOMUX_SUSPENDED] = &mdp_vsync_suspend_cfg,
        },
    },
    //ASUS_BSP: Louis, add for panel vsync  ---
	// --- ASUS_BSP : add for asus display
	{
		.gpio		= 13,		/* HS_PATH_EN */
		.settings = {
			[GPIOMUX_ACTIVE] = &hs_path_en,
		},
	},
	{
		.gpio = 14, 
		.settings = {
			[GPIOMUX_ACTIVE]    = &I_NP_cfg, //freddy A91_HWID pin
			[GPIOMUX_SUSPENDED] = &I_NP_cfg,
		},
	},
	{
		.gpio = 15, /* CAM_MCLK0 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[0],
			[GPIOMUX_SUSPENDED] = &cam_settings[1],
		},
	},
	{
		.gpio = 16, 
		.settings = {
			[GPIOMUX_ACTIVE]    = &I_NP_cfg, //freddy A91_HWID pin
			[GPIOMUX_SUSPENDED] = &I_NP_cfg,
		},
	},
	{
		.gpio = 17, 
		.settings = {
			[GPIOMUX_ACTIVE]    = &I_NP_cfg, //freddy A91_MDMID0 pin
			[GPIOMUX_SUSPENDED] = &I_NP_cfg,
		},
	},
	{
		.gpio = 18, 
		.settings = {
			[GPIOMUX_ACTIVE]    = &I_NP_cfg, //freddy A91_MDMID1 pin
			[GPIOMUX_SUSPENDED] = &I_NP_cfg,
		},
	},
	{
	    //ASUS_BSP: Sina ++
		.gpio = 19, /* CCI_I2C_SDA0 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &O_L_cfg,   //unused
			[GPIOMUX_SUSPENDED] = &O_L_cfg,  //unused
		},
	},
	//ASUS_BSP: Eason for Wireless Charger IC +++
	{//WC_PD_DET
		.gpio = 20, 
		.settings = {
			[GPIOMUX_ACTIVE]    = &I_PD_cfg,     
			[GPIOMUX_SUSPENDED] = &I_PD_cfg,     
		},

	},
	//ASUS_BSP: Eason for Wireless Charger IC  ---
	//ASUS_BSP +++ LiJen "[A86][Camera][NA][Others]Camera mini porting"
	{
		.gpio = 21, /* CCI_I2C_SDA1 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[12],
			[GPIOMUX_SUSPENDED] = &cam_settings[13],
		},
	},
	{
		.gpio = 22, /* CCI_I2C_SCL1 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[12],
			[GPIOMUX_SUSPENDED] = &cam_settings[13],
		},
	},
	//ASUS_BSP --- LiJen "[A86][Camera][NA][Others]Camera mini porting"
	{
		   
		.gpio = 23, 
		.settings = {
			[GPIOMUX_ACTIVE]    = &I_NP_cfg, //freddy A91_HWID pin
			[GPIOMUX_SUSPENDED] = &I_NP_cfg,     //unused
		},
	},
	{
		.gpio = 24, /* FLASH_LED_NOW */
		.settings = {
			[GPIOMUX_ACTIVE]    = &I_NP_cfg, //freddy A91_MDMID2 pin
			[GPIOMUX_SUSPENDED] = &I_NP_cfg,     
	   
		},
	},
	//ASUS_BSP +++ LiJen "[A86][Camera][NA][Others]Camera mini porting"
	{
		.gpio = 25, /* ISP_INT */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[5],
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[1],
		},
	},
	//ASUS_BSP --- LiJen "[A86][Camera][NA][Others]Camera mini porting"
    //ASUS_BSP: Louis for LCD_ID ++
	{
		.gpio = 26,
		.settings = {
			[GPIOMUX_ACTIVE]    = &I_NP_cfg,
			[GPIOMUX_SUSPENDED] = &I_NP_cfg,
		},
	},
    //ASUS_BSP: Louis for LCD_ID --
	//ASUS_BSP: Eason for Charger IC smb346+++
	{	 //SMB346_INOK_N
		.gpio = 27,
		.settings = {
			[GPIOMUX_ACTIVE]    = &I_NP_cfg,
			[GPIOMUX_SUSPENDED] = &I_NP_cfg,
		},
	},
	//ASUS_BSP: Eason for Charger IC smb346---
	//ASUS BSP freddy++ set for power key
	{
		.gpio = 28, /* pwr key */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_power_keys_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_power_keys_cfg,
		},
	},
	//ASUS BSP freddy--
//ASUS_BSP lenter+++
	{
		.gpio = 29,
		.settings = {
			//ASUS_BSP +++ Peter Lu "For Pad i2c drive-strength issue"
			[GPIOMUX_SUSPENDED] = &gpio_i2c_4ma_config,
			[GPIOMUX_ACTIVE]    = &gpio_i2c_4ma_config,
		},
	},
	{
		.gpio = 30,
		.settings = {
			//ASUS_BSP +++ Peter Lu "For Pad i2c drive-strength issue"
			[GPIOMUX_SUSPENDED] = &gpio_i2c_4ma_config,
			[GPIOMUX_ACTIVE]    = &gpio_i2c_4ma_config,
		},
	},
	//ASUS_BSP lenter---
//+++ ASUS_BSP Bernard: MyDP chip pw down
	{
		.gpio      = 31,		
		.settings = {
			[GPIOMUX_ACTIVE]    = &O_H_cfg,
			[GPIOMUX_SUSPENDED] = &O_H_cfg,
		},
	},
//--- ASUS_BSP Bernard: MyDP chip pw down
	{
		.gpio = 32,
		.settings = {
			[GPIOMUX_ACTIVE]    = &hdmi_active_1_cfg,
			[GPIOMUX_SUSPENDED] = &hdmi_suspend_cfg,
		},
	},
	{
		.gpio = 33,
		.settings = {
			[GPIOMUX_ACTIVE]    = &hdmi_active_1_cfg,
			[GPIOMUX_SUSPENDED] = &hdmi_suspend_cfg,
		},
	},
	{
		.gpio = 34,
		.settings = {
			[GPIOMUX_ACTIVE]    = &hdmi_active_2_cfg,
			[GPIOMUX_SUSPENDED] = &hdmi_suspend_cfg,
		},
	},
	{
		.gpio = 36,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio = 37,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio = 38,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio = 39,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio = 40,
		.settings = {
			[GPIOMUX_ACTIVE]    = &wcnss_5wire_active_cfg,
			[GPIOMUX_SUSPENDED] = &wcnss_5wire_suspend_cfg,
		},
	},
	{
		.gpio	= 41,	/* BLSP2 UART7 TX */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_uart7_active_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_uart7_suspend_cfg,
		},
	},
	{
		.gpio	= 42,	/* BLSP2 UART7 RX */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_uart7_active_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_uart7_suspend_cfg,
		},
	},
	{
		.gpio	= 43,	/* BLSP2 UART7 CTS */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_uart7_active_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_uart7_suspend_cfg,
		},
	},
	{
		.gpio	= 44,	/* BLSP2 UART7 RFR */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_uart7_active_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_uart7_suspend_cfg,
		},
	},
//+++ ASUS BSP Bernard: for mydp, reset 
	{
		.gpio = 45,
		.settings = {
			[GPIOMUX_ACTIVE]    = &O_L_cfg,
			[GPIOMUX_SUSPENDED] = &O_L_cfg,
		},
	},
//ASUS_BSP: Eason for Charger IC smb346+++
        {       //BATT_LOW_N
                .gpio = 46,
                .settings = {
                        [GPIOMUX_ACTIVE]    = &I_PU_cfg,
                        [GPIOMUX_SUSPENDED] = &I_PU_cfg,
                },
        },
//ASUS_BSP: Eason for Charger IC smb346---
    //ASUS_BSP: Louis for LCD ++
    {
        .gpio = 47,     //LCD_RST_N
        .settings = {
            [GPIOMUX_ACTIVE]    = &O_H_cfg,
            [GPIOMUX_SUSPENDED] = &O_H_cfg,
        },
    },
    {
        .gpio = 48,     //STB2_EN
        .settings = {
            [GPIOMUX_ACTIVE]    = &O_H_cfg,
            [GPIOMUX_SUSPENDED] = &O_H_cfg,
        },
    },
    //ASUS_BSP: Louis for LCD --
	//ASUS_BSP LiJen +++
	{
		.gpio      = 49,		/* BLSP1 QUP SPI_DATA_MOSI */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_spi_config,
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[1],
		},
	},
	{
		.gpio      = 50,		/* BLSP1 QUP SPI_DATA_MISO */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_spi_config,
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[1],
		},
	},
	{
		.gpio      = 51,		/* BLSP1 QUP SPI_CS0_N */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_spi_config,
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[0],
		},
	},
	{
		.gpio      = 52,		/* BLSP1 QUP SPI_CLK */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_spi_config,
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[0],
		},
	},    	    
#if 0	
	{
		.gpio = 50,               /* HSIC_HUB_INT_N */
		.settings = {
			[GPIOMUX_ACTIVE] = &hsic_hub_act_cfg,
			[GPIOMUX_SUSPENDED] = &hsic_sus_cfg,
		},
	},
#endif
	//ASUS_BSP LiJen ---
	
	{
		.gpio      = 53,		
		.settings = {
			[GPIOMUX_ACTIVE] = &I_NP_cfg, //freddy A91_HWID4 pin
			[GPIOMUX_SUSPENDED] = &I_NP_cfg,
		},
	},
	
//ASUS_BSP: Louis for STB1_EN ++
	{
		.gpio      = 54,		/* BLSP2 QUP4 SPI_DATA_MISO */
		.settings = {
			[GPIOMUX_ACTIVE] = &O_H_cfg,
			[GPIOMUX_SUSPENDED] = &O_H_cfg,
		},
	},
//ASUS_BSP: Louis for STB1_EN --
	//+++ ASUS BSP Eason BLSP10
	{
		.gpio = 55, /* CCI_I2C_SDA1 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_i2c_config,
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
		},
	},
	{
		.gpio = 56, /* CCI_I2C_SCL1 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_i2c_config,
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
		},
	},
	//--- ASUS BSP Eason BLSP10
	//ASUS_BSP +++ LiJen "[A86][Camera][NA][Others]Camera mini porting"
	{
		.gpio = 57, /* CAM_MCLK0 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[11],
			[GPIOMUX_SUSPENDED] = &cam_settings[1],
		},
	},  		
	//ASUS_BSP --- LiJen "[A86][Camera][NA][Others]Camera mini porting"	
	{
		.gpio = 58,
		.settings = {
			[GPIOMUX_ACTIVE]    = &I_NP_cfg, //freddy A91_HWID3 pin
			[GPIOMUX_SUSPENDED] = &I_NP_cfg,
		},
	},

	//+++Porting NFC's kernel+++ // irq
	{
		.gpio      = 59,		
                .settings = {
                        [GPIOMUX_ACTIVE]    = &nfc_intr_cfg,
                        [GPIOMUX_SUSPENDED] = &nfc_intr_cfg,
                },
	},
	//---Porting NFC's kernel---

	// +++ ASUS_BSP : add for asus touch
	{
		.gpio      = 60,		/* TOUCH RESET */
		.settings = {
			[GPIOMUX_ACTIVE] = &ts_reset_cfg,
			[GPIOMUX_SUSPENDED] = &ts_reset_cfg,
		},
	},
	// --- ASUS_BSP : add for asus touch
//+++ ASUS BSP Bernard: for mydp, cable detect
	{
		.gpio      = 61,		
		.settings = {
			[GPIOMUX_ACTIVE] = &I_NP_cfg,
			[GPIOMUX_SUSPENDED] = &I_NP_cfg,
		},
	},      	 
//--- ASUS BSP Bernard: for mydp, cable detect 	
	{
		.gpio = 62,
		.settings = {
			[GPIOMUX_ACTIVE]    = &sd_card_det_active_config,
			[GPIOMUX_SUSPENDED] = &sd_card_det_sleep_config,
		},
	},
	{
		.gpio	= 63,		/* SYS_RST_N */
		.settings = {
			[GPIOMUX_SUSPENDED] = &taiko_reset,
		},
	},
// ASUS_BSP +++ Jason Chang "[A86][Sensor][NA][Spec] Porting haptic sensor"
	{
		.gpio = 64,
		.settings = {
			[GPIOMUX_ACTIVE]    = &O_L_cfg,
			[GPIOMUX_SUSPENDED] = &O_L_cfg,
			//[GPIOMUX_ACTIVE]    = &O_H_cfg,
			//[GPIOMUX_SUSPENDED] = &O_H_cfg,
		},
	},
// ASUS_BSP --- Jason Chang "[A86][Sensor][NA][Spec] Porting haptic sensor"
    //ASUS_BSP: Sina ++
	{
		.gpio = 65,
		.settings = {
			[GPIOMUX_ACTIVE]    = &O_L_cfg,     // unused
			[GPIOMUX_SUSPENDED] = &O_L_cfg, // unused
		},
	},
    //ASUS_BSP: Sina --
//ASUS_BSP +++ Jason Chang "[A86][Sensor][NA][Spec] Porting 9-axis sensor"
	{
		.gpio = 66,             // Gyro
		.settings = {
			[GPIOMUX_SUSPENDED] = &I_NP_cfg,
			[GPIOMUX_ACTIVE] = &I_NP_cfg,
		},
	},
//ASUS_BSP --- Jason Chang "[A86][Sensor][NA][Spec] Porting 9-axis sensor"

	{
		.gpio = 67,
		.settings = {
			[GPIOMUX_SUSPENDED] = &pri_auxpcm_sus_cfg,
			[GPIOMUX_ACTIVE] = &pri_auxpcm_act_cfg,
		},
	},
	{
		.gpio = 68,
		.settings = {
			[GPIOMUX_SUSPENDED] = &pri_auxpcm_sus_cfg,
			[GPIOMUX_ACTIVE] = &pri_auxpcm_act_cfg,
		},
	},
//Touch PCB0 +++
	{
		.gpio = 69,
		.settings = {
			[GPIOMUX_SUSPENDED] = &I_NP_cfg,
			[GPIOMUX_ACTIVE] = &I_NP_cfg,
		},
	},
//Touch PCB0 ---
	{
		.gpio	= 70,		/* slimbus clk */
		.settings = {
			[GPIOMUX_SUSPENDED] = &slimbus,
		},
	},
	{
		.gpio	= 71,		/* slimbus data */
		.settings = {
			[GPIOMUX_SUSPENDED] = &slimbus,
		},
	},
	{
		.gpio	= 72,		/* CDC_INT */
		.settings = {
			[GPIOMUX_SUSPENDED] = &taiko_int,
		},
	},

//+++ ASUS_BSP Peter_lu : light/proximity sensor
	{       //PROXM_OUT
                .gpio = 74,
                .settings = {
                        [GPIOMUX_SUSPENDED] = &prox_sensor_interrupt,
			   [GPIOMUX_ACTIVE]    = &prox_sensor_interrupt,
                },
        },
//--- ASUS_BSP Peter_lu : light/proximity sensor

//+++ ASUS_BSP: P06 plug in status
    {
        .gpio = 75,
        .settings = {
            [GPIOMUX_ACTIVE] = &I_NP_cfg,
            [GPIOMUX_SUSPENDED] = &I_NP_cfg,
        },
    },
//--- ASUS_BSP: P06 plug in status
//ASUS_BSP +++ Jiunhau_Wang "[A86][Sensor][NA][Spec] Porting 9-axis sensor"
	{
		.gpio   = 76,           // E-compass
		.settings = {
			[GPIOMUX_SUSPENDED] = &I_NP_cfg,
			[GPIOMUX_ACTIVE] =  &I_NP_cfg,
		},
	},

	//+++Porting NFC's kernel+++
        {    //NFC_VEN
                .gpio = 79,
                .settings = {
                        [GPIOMUX_SUSPENDED] = &O_L_cfg,
                },
        },
	//---Porting NFC's kernel---

//ASUS_BSP --- Jiunhau_Wang "[A86][Sensor][NA][Spec] Porting 9-axis sensor"
	//ASUS_BSP +++ LiJen "[A86][Camera][NA][Others]Camera mini porting"
        {
		.gpio = 78, /* ISP_SUSPEND */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[5],
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[1],
		},
	},
	//ASUS_BSP --- LiJen "[A86][Camera][NA][Others]Camera mini porting"
	{
		.gpio	= 80,		/* HS_HOOK_DET */
		.settings = {
			[GPIOMUX_SUSPENDED] = &hs_button_detect,
		},
	},
//ASUS BSP freddy++ vibrator gpio_rst pin
	{
		.gpio      = 81,		
		.settings = {
			[GPIOMUX_SUSPENDED] = &O_L_cfg,
			[GPIOMUX_ACTIVE] =  &O_L_cfg,
		},
	},
//ASUS BSP freddy++ vibrator gpio_rst pin
//+++ ASUS_BSP Bernard: USB_HS_ID
	{

		.gpio = 82,		/*USB_HS_ID*/
		.settings = {
			[GPIOMUX_SUSPENDED] = &I_NP_cfg,
			[GPIOMUX_ACTIVE]    = &I_NP_cfg,
		},
	},
//--- ASUS_BSP Bernard: USB_HS_ID
	//ASUS_BSP +++ LiJen "[A86][Camera][NA][Others]Camera mini porting"
	{
		.gpio      = 84,		/* 1.2M_MCLK_EN_N */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[5],
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[1],
		},
	},
	//ASUS_BSP --- LiJen "[A86][Camera][NA][Others]Camera mini porting"
	//ASUS_BSP: Eason for Wireless Charger IC bq51220+++
	{       //WC_POK
                .gpio = 85,
                .settings = {
                        [GPIOMUX_ACTIVE]    = &I_PD_cfg,
                        [GPIOMUX_SUSPENDED] = &I_PD_cfg,
                },
	},
	//ASUS_BSP: Eason for Wireless Charger IC bq51220---
	//Hank: Set SMB346 IRQ GPIO output low+++
	{
		.gpio = 86,
		.settings = {
			[GPIOMUX_SUSPENDED] = &O_L_cfg,
			[GPIOMUX_ACTIVE] = &O_L_cfg,
		},
	},
	//Hank: Set SMB346 IRQ GPIO output low---
//+++ ASUS BSP Bernard, mydp_i2c_sda
	{
		.gpio = 87, 
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_i2c_config,
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
		},
	},
//--- ASUS BSP Bernard, mydp_i2c_sda
//+++ ASUS BSP Bernard, mydp_i2c_clk
	{
		.gpio = 88, 
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_i2c_config,
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
		},
	},
//--- ASUS BSP Bernard, mydp_i2c_clk
//+++ ASUS BSP Bernard, mydp_int
	{
		.gpio = 89, 
		.settings = {
			[GPIOMUX_ACTIVE]    = &I_NP_cfg,
			[GPIOMUX_SUSPENDED] = &I_NP_cfg,
		},
	},
//--- ASUS BSP Bernard, mydp_int
//Touch PCB1 +++
	{
		.gpio = 90,
		.settings = {
			[GPIOMUX_SUSPENDED] = &I_NP_cfg,
			[GPIOMUX_ACTIVE] = &I_NP_cfg,
		},
	},
//Touch PCB1 ---
        //ASUS_BSP +++ LiJen "[A86][Camera][NA][Others]Camera mini porting"
        {
		.gpio = 91, /* FLED_DRIVER_ENT */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[5],
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[1],
		},
	},
        //ASUS_BSP --- LiJen "[A86][Camera][NA][Others]Camera mini porting"	
	// +++ ASUS_BSP: add for miniporting	
	{
		.gpio = 92, /* vol up */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_volup_keys_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_volup_keys_cfg,

		},
	},
	{
		.gpio = 95, /* vol down */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_voldown_keys_cfg,
			[GPIOMUX_SUSPENDED] = &gpio_voldown_keys_cfg,
		}
	},
	// --- ASUS_BSP: add for miniporting	
        //ASUS_BSP +++ LiJen "[A86][Camera][NA][Others]Camera mini porting"
        {
		.gpio = 94, /* ISP_RESET */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[5],
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[1],
		},
	},
        //ASUS_BSP --- LiJen "[A86][Camera][NA][Others]Camera mini porting"
	{
		.gpio = 144,               /*HSIC_STROBE */
		.settings = {
			[GPIOMUX_ACTIVE] = &hsic_act_cfg,
			[GPIOMUX_SUSPENDED] = &hsic_sus_cfg,
		},
	},
	{
		.gpio = 145,               /* HSIC_DATA */
		.settings = {
			[GPIOMUX_ACTIVE] = &hsic_act_cfg,
			[GPIOMUX_SUSPENDED] = &hsic_sus_cfg,
		},
	},
};

#endif  /* __PF500KL_er1_GPIO_PINMUX_H  */

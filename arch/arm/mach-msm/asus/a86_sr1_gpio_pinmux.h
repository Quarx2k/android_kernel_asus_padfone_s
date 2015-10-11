#ifndef __a86_sr1_GPIO_PINMUX_H
#define __a86_sr1_GPIO_PINMUX_H

#include "a86_gpio_pinmux_setting.h"

#define KS8851_IRQ_GPIO 94

static struct msm_gpiomux_config a86_sr1_msm8974_gpio_configs[] = {
// +++ ASUS_BSP : add for display
#if 1 // defined(CONFIG_KS8851) || defined(CONFIG_KS8851_MODULE)
	{
		.gpio      = 0,		/* BLSP1 QUP SPI_DATA_MOSI */
		.settings = {
            [GPIOMUX_ACTIVE]    = &mdp_vsync_active_cfg,
			[GPIOMUX_SUSPENDED] = &mdp_vsync_suspend_cfg,
		},
	},
// --- ASUS_BSP : add for display
	{
		.gpio      = 1,		/* JACK_IN_DET */
		.settings = {
			[GPIOMUX_SUSPENDED] = &hs_detect,
		},
	},
    //ASUS_BSP: Louis for BL_EN ++
    {
        .gpio      = 2,
        .settings = {
            [GPIOMUX_ACTIVE] = &O_H_cfg,
            [GPIOMUX_SUSPENDED] = &O_H_cfg,
        },
    },
    //ASUS_BSP: Louis for BL_EN --

	{
		.gpio      = 3,		/* BLSP1 QUP SPI_CLK */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_spi_config,
		},
	},
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
		},
	},
	{
		.gpio      = 7,		/* BLSP1 QUP2 I2C_CLK */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
		},
	},
	// +++ ASUS_BSP : add for asus touch
	{
		/* TP power EN */
		.gpio = 8,
		.settings = {
			[GPIOMUX_SUSPENDED] = &O_H_cfg,
			[GPIOMUX_ACTIVE]    = &O_H_cfg,
		},
	},
	// --- ASUS_BSP : add for asus touch
#if 0 	// +++ ASUS_BSP : add for asus display
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
	//ASUS_BSP SinaChou+++
	{ //microp_interrupt
		.gpio = 9,
		.settings = {
			[GPIOMUX_SUSPENDED] = &microp_interrupt,
			[GPIOMUX_ACTIVE]    = &microp_interrupt,
		},
	},
	//ASUS_BSP lenter---
	
	// +++ ASUS_BSP : add for asus display
	{
		.gpio      = 10,		/* BLSP1 QUP2 I2C_DAT I2C-3s*/
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
		},
	},
	{
		.gpio      = 11,		/* BLSP1 QUP2 I2C_CLK I2C-3*/
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
		},
	},
	{
		/* TP IRQ */
		.gpio = 12,
		.settings = {
			[GPIOMUX_SUSPENDED] = &ts_int_cfg,
			[GPIOMUX_ACTIVE]    = &ts_int_cfg,
		},
	},
	// --- ASUS_BSP : add for asus display
	{
		.gpio		= 13,		/* HS_PATH_EN */
		.settings = {
			[GPIOMUX_ACTIVE] = &hs_path_en,
		},
	},
	{
		.gpio = 14, /* HW_ID0 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_hw_id_config,
			[GPIOMUX_SUSPENDED] = &gpio_hw_id_config,
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
		.gpio = 16, /* HW_ID2 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_hw_id_config,
			[GPIOMUX_SUSPENDED] = &gpio_hw_id_config,
		},
	},
	{
		.gpio = 17, /* MDM_0 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &I_NP_cfg,
			[GPIOMUX_SUSPENDED] = &I_NP_cfg,
		},
	},
	{
		.gpio = 18, /* MDM_1 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &I_NP_cfg,
			[GPIOMUX_SUSPENDED] = &I_NP_cfg,
		},
	},
	{
		.gpio = 19, /* CCI_I2C_SDA0 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[0],
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[0],
		},
	},
	{
		.gpio = 20, /* CCI_I2C_SCL0 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[0],
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[0],
		},
	},
	{
		.gpio = 21, /* CCI_I2C_SDA1 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[0],
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[0],
		},
	},
	{
		.gpio = 22, /* CCI_I2C_SCL1 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[0],
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[0],
		},
	},
	{
		.gpio = 23, /* HW_ID1 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_hw_id_config,
			[GPIOMUX_SUSPENDED] = &gpio_hw_id_config,
		},
	},
	{
		.gpio = 24, /* MDM_2 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &I_NP_cfg,
			[GPIOMUX_SUSPENDED] = &I_NP_cfg,
		},
	},
	{
		.gpio = 25, /* WEBCAM2_RESET_N */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[0],
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[1],
		},
	},
    //ASUS_BSP: Louis for LCD_ID ++
	{
		.gpio = 26,
		.settings = {
			[GPIOMUX_ACTIVE]    = &I_NP_cfg,
			[GPIOMUX_SUSPENDED] = &I_NP_cfg,
		},
	},
    //ASUS_BSP: Louis for LCD_ID --
	{
		.gpio = 27, /* OIS_SYNC */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[0],
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[1],
		},
	},
	{
		.gpio = 28, /* WEBCAM1_STANDBY */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[0],
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[1],
		},
	},
	//ASUS_BSP lenter+++
	{
		.gpio = 29,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
			[GPIOMUX_ACTIVE]    = &gpio_i2c_config,
		},
	},
	{
		.gpio = 30,
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
			[GPIOMUX_ACTIVE]    = &gpio_i2c_config,
		},
	},
	//ASUS_BSP lenter---
    //ASUS_BSP: Louis for STB1_EN ++
	{
		.gpio = 31,
		.settings = {
			[GPIOMUX_ACTIVE]    = &O_H_cfg,
			[GPIOMUX_SUSPENDED] = &O_H_cfg,
		},
	},
    //ASUS_BSP: Louis for STB1_EN --
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
	{
		.gpio      = 45,	/* BLSP2 UART8 TX */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_uart_config,
		},
	},
	{
		.gpio      = 46,	/* BLSP2 UART8 RX */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_uart_config,
		},
	},
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
	{
		.gpio = 50,               /* HSIC_HUB_INT_N */
		.settings = {
			[GPIOMUX_ACTIVE] = &hsic_hub_act_cfg,
			[GPIOMUX_SUSPENDED] = &hsic_sus_cfg,
		},
	},
	{
		.gpio = 53, /* HW_ID4 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_hw_id_config,
			[GPIOMUX_SUSPENDED] = &gpio_hw_id_config,
		},
	},
	{
		.gpio      = 54,		/* BLSP2 QUP4 SPI_DATA_MISO */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_spi_config,
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[1],
		},
	},
	{
		.gpio      = 55,		/* BLSP2 QUP4 SPI_CS0_N */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_spi_config,
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[0],
		},
	},
	{
		.gpio      = 56,		/* BLSP2 QUP4 SPI_CLK */
		.settings = {
			[GPIOMUX_ACTIVE] = &gpio_spi_config,
			[GPIOMUX_SUSPENDED] = &gpio_suspend_config[0],
		},
	},
	{
		.gpio = 58, /* HW_ID3 */
		.settings = {
			[GPIOMUX_ACTIVE]    = &gpio_hw_id_config,
			[GPIOMUX_SUSPENDED] = &gpio_hw_id_config,
		},
	},
	// +++ ASUS_BSP : add for asus touch
	{
		.gpio      = 60,		/* TOUCH RESET */
		.settings = {
			[GPIOMUX_ACTIVE] = &ts_reset_cfg,
			[GPIOMUX_SUSPENDED] = &ts_reset_cfg,
		},
	},
	// --- ASUS_BSP : add for asus touch
	{
		.gpio      = 61,		/* TOUCH IRQ */
		.settings = {
			[GPIOMUX_ACTIVE] = &atmel_int_act_cfg,
			[GPIOMUX_SUSPENDED] = &atmel_int_sus_cfg,
		},
	},
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
    //ASUS_BSP: Louis for STB1_EN ++
	{
		.gpio = 65,
		.settings = {
			[GPIOMUX_ACTIVE]    = &O_H_cfg,
			[GPIOMUX_SUSPENDED] = &O_H_cfg,
		},
	},
    //ASUS_BSP: Louis for STB1_EN --
//ASUS_BSP +++ Jiunhau_Wang "[A86][Sensor][NA][Spec] Porting 9-axis sensor"
	{
		.gpio = 66,             // Gyro
		.settings = {
			[GPIOMUX_SUSPENDED] = &I_NP_cfg,
			[GPIOMUX_ACTIVE] = &I_NP_cfg,
		},
	},
//ASUS_BSP --- Jiunhau_Wang "[A86][Sensor][NA][Spec] Porting 9-axis sensor"
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
//ASUS_BSP +++ Jiunhau_Wang "[A86][Sensor][NA][Spec] Porting 9-axis sensor"
	{
		.gpio   = 76,           // E-compass
		.settings = {
			[GPIOMUX_SUSPENDED] = &I_NP_cfg,
			[GPIOMUX_ACTIVE] =  &I_NP_cfg,
		},
	},
//ASUS_BSP --- Jiunhau_Wang "[A86][Sensor][NA][Spec] Porting 9-axis sensor"
	{
		.gpio	= 80,		/* HS_HOOK_DET */
		.settings = {
			[GPIOMUX_SUSPENDED] = &hs_button_detect,
		},
	},
//ASUS_BSP +++ Jiunhau_Wang "[A86][Sensor][NA][Spec] Porting 9-axis sensor"
	{
		.gpio      = 81,		// E-compass ready
		.settings = {
			[GPIOMUX_SUSPENDED] = &I_NP_cfg,
			[GPIOMUX_ACTIVE] =  &I_NP_cfg,
		},
	},
//ASUS_BSP --- Jiunhau_Wang "[A86][Sensor][NA][Spec] Porting 9-axis sensor"
	{
//+++ ASUS_BSP Bernard: USB_HS_ID & mydp_chip_pw_down
		.gpio = 82,		/*USB_HS_ID*/
		.settings = {
			[GPIOMUX_SUSPENDED] = &I_NP_cfg,
			[GPIOMUX_ACTIVE]    = &I_NP_cfg,
		},
	},
	{
		.gpio      = 83,		/*mydp_chip_pw_down*/
		.settings = {
			[GPIOMUX_ACTIVE]    = &O_H_cfg,
			[GPIOMUX_SUSPENDED] = &O_H_cfg,
		},
	},
//--- ASUS_BSP Bernard: USB_HS_ID & mydp_chip_pw_down
	{
		.gpio      = 84,		/* BLSP11 QUP I2C_CLK */
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_i2c_config,
		},
	},
	{
		.gpio = 86,
		.settings = {
			[GPIOMUX_SUSPENDED] = &hap_lvl_shft_suspended_config,
			[GPIOMUX_ACTIVE] = &hap_lvl_shft_active_config,
		},
	},
	{
		.gpio = 89, /* CAM1_STANDBY_N */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[0],
			[GPIOMUX_SUSPENDED] = &cam_settings[1],
		},
	},
	{
		.gpio = 90, /* CAM1_RST_N */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[0],
			[GPIOMUX_SUSPENDED] = &cam_settings[1],
		},
	},
	{
		.gpio = 91, /* CAM2_STANDBY_N */
		.settings = {
			[GPIOMUX_ACTIVE]    = &cam_settings[0],
			[GPIOMUX_SUSPENDED] = &cam_settings[1],
		},
	},
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
#if defined(CONFIG_KS8851) || defined(CONFIG_KS8851_MODULE)
	{
		.gpio = KS8851_IRQ_GPIO,//94
		.settings = {
			[GPIOMUX_SUSPENDED] = &gpio_eth_config,
		}
	},
#endif
	{
		.gpio = 106,               /* */
		.settings = {
			[GPIOMUX_ACTIVE] = &NC_cfg,
			[GPIOMUX_SUSPENDED] = &NC_cfg,
		},
	},
	{
		.gpio = 107,               /* */
		.settings = {
			[GPIOMUX_ACTIVE] = &NC_cfg,
			[GPIOMUX_SUSPENDED] = &NC_cfg,
		},
	},
	{
		.gpio = 108,               /* */
		.settings = {
			[GPIOMUX_ACTIVE] = &NC_cfg,
			[GPIOMUX_SUSPENDED] = &NC_cfg,
		},
	},
	{
		.gpio = 117,               /* */
		.settings = {
			[GPIOMUX_ACTIVE] = &NC_cfg,
			[GPIOMUX_SUSPENDED] = &NC_cfg,
		},
	},
	{
		.gpio = 118,               /* */
		.settings = {
			[GPIOMUX_ACTIVE] = &NC_cfg,
			[GPIOMUX_SUSPENDED] = &NC_cfg,
		},
	},
	{
		.gpio = 135,               /* */
		.settings = {
			[GPIOMUX_ACTIVE] = &NC_cfg,
			[GPIOMUX_SUSPENDED] = &NC_cfg,
		},
	},
	{
		.gpio = 136,               /* */
		.settings = {
			[GPIOMUX_ACTIVE] = &NC_cfg,
			[GPIOMUX_SUSPENDED] = &NC_cfg,
		},
	},
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

#endif  /* __a68_SR1_1_GPIO_PINMUX_H  */

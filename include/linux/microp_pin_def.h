#ifndef MICROP_PIN_DEF_H
#define MICROP_PIN_DEF_H

enum MICROP_INPUT{
        IN_USB_ID_R=0,  //ER
        IN_DOCK_AP_WAKE_R=1,
        IN_CHG_STAT_R=1,    //ER    
        IN_ALS_INT_R=2,
        IN_NO_USE_2=3,
        IN_VOL_DOWN_R=4,
        IN_VOL_UP_R=5,
        IN_PWR_BTN_R=6,
        IN_AC_USB_IN=7,
        IN_HANDSET_IN_R=8,
        IN_NO_USE_3=9,       
        IN_DOCK_IN_R=10,
        IN_O_LID_R=11,         //  LID_CLOSE
        IN_LCD_ID=12,
        IN_NO_USE_4=13,
        IN_TS_ID=14,
        IN_CAP_RPOX_INT_R=15,
        IN_TCA6408A_INT=16,   // only for  P05
        IN_HS_HOOK_DET_R=17,   // only for  P05
        IN_JACK_IN_DET_R=18,   // only for  P05
        IN_PEN_DET=19,
        IN_PEN_IRQ=20,
};



enum MICROP_INTR_MASK{
        INTR_EN_NO_USE_1     =   0x1<<0,
        INTR_EN_NO_USE_2   =   0x1<<1,
        INTR_EN_ALS_INT          =   0x1<<2,
        INTR_EN_NO_USE_3   =   0x1<<3,
        INTR_EN_VOL_DOWN      =   0x1<<4,
        INTR_EN_VOL_UP            =  0x1<<5,
        INTR_EN_PWR_BTN       =  0x1<<6,
        INTR_EN_AC_USB_IN  =  0x1<<7,
        INTR_EN_HANDSET_IN = 0x1<<8,
        INTR_EN_NO_USE_4   =   0x1<<9,       // DOCK_PB
        INTR_EN_DOCK_IN   =   0x1<<10,
        INTR_EN_O_LID         =   0x1<<11,          //  LID_CLOSE
        INTR_EN_RESERVED_1          =   0x1<<12,
        INTR_EN_COMPASS_RDY     =    0x1<<13,
        INTR_EN_RESERVED_2        =    0x1<<14,
        INTR_EN_CAP_RPOX_INT    =   0x1<<15,
        INTR_EN_TCA6408A_INT    =   0x1<<16,    // only for  P05
        INTR_EN_HS_HOOK_DET    =   0x1<<17,     // only for  P05
        INTR_EN_JACK_IN_DET    =   0x1<<18,     // only for  P05
        INTR_EN_PEN_DET		=	0x1<<19,
        INTR_EN_PEN_IRQ		=	0x1<<20,

};


enum MICROP_INTR_STATUS{
        INTR_STA_NO_USE_1=0,
        INTR_STA_NO_USE_2=1,
        INTR_STA_ALS_INT=2,
        INTR_STA_NO_USE_3=3,
        INTR_STA_VOL_DOWN=4,
        INTR_STA_VOL_UP=5,
        INTR_STA_PWR_BTN=6,
        INTR_STA_AC_USB_IN_OUT=7,
        INTR_STA_HANDSET_IN=8,
        INTR_STA_NO_USE_4=9,
        INTR_STA_DOCK_IN_OUT=10,
        INTR_STA_DOCK_LID=11,
        INTR_STA_RESERVED_1=12,
        INTR_STA_COMPASS_RDY=13,
        INTR_STA_RESERVED_2=14,
        INTR_STA_CAP_PROX_ACT=15,
        INTR_STA_IND_SHUTDOWN_BY_EC=16,
        INTR_STA_LONG_PRESS_PWRKEY=17,
        INTR_STA_IND_GAUGE_5P=18,
        INTR_STA_IND_RSTS_WDT=19,        
        INTR_STA_POWER_ON=20,
        INTR_STA_SLEEP_REMINDER=25,
        INTR_STA_JACK_IN_DET=26,                
        INTR_STA_HS_HOOK_DET=27,        
        INTR_STA_TCA6408A_INT=28,
        INTR_STA_FLAG_PWRKEY_PRESS=29,
        INTR_STA_FLAG_PWRKEY_RELEASE=30,
        INTR_STA_BAT_STATUS_CHANGE=31,
};



/*
 *      g_microp_ver >=5 has the following defintion
*/
enum MICROP_OUTPUT{
        OUT_uP_NO_USE_1=0,
        OUT_uP_DOCK_BST_EN_R=1,       // PB_BOOST
        OUT_uP_USB_SW_HST=2,
        OUT_uP_NO_USE_2=3,
        OUT_uP_TS_RST_R=4,
        OUT_uP_SPK_EN=5,
        OUT_uP_AUD_PWR_EN=6,
        OUT_uP_5V_PWR_EN=7,
        OUT_uP_CAM_PWR_EN=8,
        OUT_uP_NO_USE_3=9,   
        OUT_uP_HUB_PWR_EN=10,
        OUT_uP_LCD_EN=11,
        OUT_uP_MHL_CBUS_EN=12,
        OUT_uP_PAD_LOW_BAT=13,
        OUT_uP_EN_3V3_1V2=14,
        OUT_uP_NO_USE_4=15, 
        OUT_uP_MCU_OTG_EN_R=16,
        OUT_uP_VBUS_EN=17,
        OUT_uP_TS_PWR_EN=18, //ER
        OUT_uP_MHL_5V_EN=19, //ER
        OUT_uP_CAP_PWR_EN=20, // only for  P05
        OUT_uP_PEN_PWR_EN=21, // only for  P05        
        OUT_uP_LED_O=22, // only for  P05        
        OUT_uP_LED_G=23, // only for  P05                
        OUT_uP_MIC_SEL=24, // only for  P05                        
        OUT_uP_NO_USE_5=25,
        OUT_uP_MYDB_CWIRE_EN=26,
        OUT_uP_NO_USE_6=27,
        OUT_uP_STDP_I2C_EN=28,
        OUT_uP_LCD_I2C_SW_EN=29,
        OUT_uP_STDP_RST_=30,
        OUT_uP_CAP_I2C_MOS_EN=31, 
        OUT_uP_SIZE,
};



#endif

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/microp.h>
#include <linux/microp_api.h>
#include <linux/microp_pin_def.h>
#include <linux/microp_notify.h>
extern int uP_nuvoton_read_reg(int cmd, void *data);
extern int uP_nuvoton_write_reg(int cmd, void *data);
extern int isFirmwareUpdating(void);
extern void msleep(unsigned int msecs);
extern unsigned int g_b_isP01Connected;
extern unsigned int g_microp_ver;
extern int micropSendNotify(unsigned long val);
extern struct mutex microp_mutex_lock;
extern unsigned int g_microp_ver;
extern unsigned int g_ldrom_ver;
extern unsigned int g_i2c_bus_suspended;
extern unsigned int g_i2c_microp_busy;
// ASUS_BSP +++ Peter_Lu "For Pad I2C suspend/resume issue"
extern unsigned int g_i2c_bus_3v3_state;

static bool SPK_EN_DuringSetting = false;//Eason fix [TT469667]

int AX_MicroP_Is_3V3_ON(void)	{
        return (g_i2c_bus_3v3_state==1)?1:0;
}
EXPORT_SYMBOL_GPL(AX_MicroP_Is_3V3_ON);
// ASUS_BSP ---


void AX_MicroP_Bus_Suspending(int susp){
    if(susp)
        g_i2c_bus_suspended=1;
    else
        g_i2c_bus_suspended=0;
}

EXPORT_SYMBOL_GPL(AX_MicroP_Bus_Suspending);


/*
*       Check the status of P01 connectness
*       return value: 1: P01 connected
*/

int AX_MicroP_IsP01Connected(void){
        return (g_b_isP01Connected==1)?1:0;
}
EXPORT_SYMBOL_GPL(AX_MicroP_IsP01Connected);


/*
*       Check the status of AC/USB if it is inserted
*       return value: 0: plugged out, 1: plugged in, <0: err
*/

int AX_MicroP_IsACUSBIn(void){
        int pin=-1;
        pin=AX_MicroP_getGPIOPinLevel(IN_AC_USB_IN);
        if(pin<0)
            return pin;
        return (pin==1)?1:0;
}
EXPORT_SYMBOL_GPL(AX_MicroP_IsACUSBIn);


/*
*   @AX_MicroP_get_ChargingStatus
*  input: target
*           0: p01 battery
*           1: dock battery
*    return: 0 for 'no charging', 1 for 'charging', 2 for 'charged full', <0 value means something error
*/

int AX_MicroP_get_ChargingStatus(int target){
	int regval=0;
       int ret=0;
       int16_t avg_current=0;       
       if(AX_MicroP_IsP01Connected()==0){
            printk("%s: P01 removed\r\n",__FUNCTION__);
            return -1;
       }

       if(isFirmwareUpdating()){
            printk("%s: P01 is updating, retry later\r\n",__FUNCTION__);
            return -2;
       }


       if(target==Batt_P01){
                // redundant read to wakeup microp from power-down mode and trigger microp to read h/w gauge again
                ret=uP_nuvoton_read_reg(MICROP_GAUGE_AVG_CURRENT,&avg_current);  
                if(ret){
                        printk("%s: avg_current: %d mA\r\n", __FUNCTION__, avg_current);
                }
                ret=uP_nuvoton_read_reg(MICROP_CHARGING_STATUS, &regval);         
                ret=uP_nuvoton_read_reg(MICROP_CHARGING_STATUS,&regval);  

                                
                if(ret <= 0 || regval==255)
                            regval=P01_CHARGING_ERR;
                else if(regval==0)
                            regval=P01_CHARGING_NO;
                else if(regval==1){
//Eason: Pad plug usb show icon & cap can increase+++
#if 0
                            if(AX_MicroP_get_USBDetectStatus(Batt_P01)==P01_CABLE_CHARGER)
                                    regval=P01_CHARGING_ONGOING;
                            else
                                    regval=P01_CHARGING_NO;
#endif
				regval=P01_CHARGING_ONGOING;
//Eason: Pad plug usb show icon & cap can increase+++
                }
                else if(regval==2)
                            regval=P01_CHARGING_FULL;

                printk("%s: charging status: %d\r\n",__FUNCTION__,  regval);                    
        }
       else{
                printk(KERN_ERR "%s: known target %d\r\n", __FUNCTION__, target);
       }
       
	return regval;
}

EXPORT_SYMBOL_GPL(AX_MicroP_get_ChargingStatus);

/*
*   @AX_MicroP_get_USBDetectStatus
*  input: target
*           0: p01 battery
*           1: dock battery
*    return: 0 for 'no charger/usb', 1 for 'charger', 2 for 'USB', <0 value means something error
*
*/ 

int AX_MicroP_get_USBDetectStatus(int target){
    	int regval=0;
       int ret=0;
       unsigned int adc_volt=0;
       if(AX_MicroP_IsP01Connected()==0){
            printk("%s: P01 removed\r\n",__FUNCTION__);
            return -1;
       }

       if(isFirmwareUpdating()){
            printk("%s: P01 is updating, retry later\r\n",__FUNCTION__);
            return -2;
       }


       if(target==Batt_P01){

                ret=uP_nuvoton_read_reg(MICROP_ADC_LEVEL,&adc_volt);
                ret=uP_nuvoton_read_reg(MICROP_USB_DET,&regval);

                printk("<adc, usb_det>=<%x,%d>\r\n", adc_volt, regval);
                if(ret <= 0 || regval==255)
                        regval=P01_CABLE_UNKNOWN;
                else if(regval==0){
                        regval=P01_CABLE_NO;
                }
                else if(adc_volt >= 3000 || regval==1)
                        regval=P01_CABLE_CHARGER;
                else if(regval==2)
                        regval=P01_CABLE_USB;
                    
        }
       else{
                printk(KERN_ERR "%s: known target %d\r\n", __FUNCTION__, target);
       }

       return regval;
}
EXPORT_SYMBOL_GPL(AX_MicroP_get_USBDetectStatus);



/*
*  GPIO direct control
*  @ AX_MicroP_getGPIOPinLevel
*  input: 
            - pinID
*  return: 0 for low, 1 for high, <0 value means something error
*
*/


int AX_MicroP_getGPIOPinLevel(int pinID){
	int regval=0;
       int ret=0;
       int gpiolevel=0;
       unsigned int sel_offset=1<<pinID;
       pr_debug("[MicroP] try to get GPIO pin=%d\r\n",pinID);
       if(AX_MicroP_IsP01Connected()==0){
            printk("%s: P01 removed\r\n",__FUNCTION__);
            return -1;
       }
       
	ret=uP_nuvoton_read_reg(MICROP_GPIO_INPUT_LEVEL,&regval);
       if(ret > 0){
                gpiolevel=(regval & sel_offset)?1:0;
       }

	return ((ret < 0)?ret:gpiolevel);
}

EXPORT_SYMBOL_GPL(AX_MicroP_getGPIOPinLevel);
/*
*  @ AX_MicroP_setGPIOOutputPin
*  input: 
*           - pinID
*           - level: 0 for low, 1 for high
*  return: the status of operation. 0 for success, <0 value means something error
*/



int AX_MicroP_setGPIOOutputPin(int pinID, int level){
       unsigned int sel_offset=0;
       unsigned int sel_offset_5V_EN = 1<<OUT_uP_5V_PWR_EN;	   
       int ret=0;
       int level_pinID=0;
       unsigned int out_reg=0;

       printk("[MicroP] set GPIO pin=%d, level=%d\r\n",pinID, level);

        if(AX_MicroP_IsP01Connected()==0){
            printk("%s: P01 removed\r\n",__FUNCTION__);
            return -1;
       }

        if(isFirmwareUpdating()){
            printk("%s: P01 is updating, retry later\r\n",__FUNCTION__);
            return -2;
       }

        if(pinID >= OUT_uP_SIZE){
                printk("%s: index error =%d\r\n",__FUNCTION__, pinID);                   
                return -1;
        }

	//workaround: for P92L , first OPEN 5V_EN, and delay 210ms before open SPK_EN

	//Eason fix [TT469667]+++
	/*
	  5v boost will be shutdown by firmware in microp sleep mode while vbus or SPK_EN be shutdown(while vbus & SPK_EN are both low)
	  If clear vbus(HS_5V_EN), then set 5v boost(5V_PWR_EN), then clear vbus(HS_5V_EN) before SPK_EN is set high
	  audio may no sound
	*/
	if(  true== SPK_EN_DuringSetting && (0 == level && OUT_uP_VBUS_EN==pinID) && (PAD_P92L==AX_MicroP_getPadModel()||PAD_P93L==AX_MicroP_getPadModel()) ){
		msleep(250);
	}
	//Eason fix [TT469667]---

	if( OUT_uP_SPK_EN==pinID && (PAD_P92L==AX_MicroP_getPadModel()||PAD_P93L==AX_MicroP_getPadModel()) ){		
		//Eason fix [TT469667]+++
		if(level)
			SPK_EN_DuringSetting = true;
		//Eason fix [TT469667]---
		//Eason fix [TT469667]: take off this judge, because if 5V boost trigged set by vbus, may does not do following workaround
		/*if(0==AX_MicroP_getGPIOOutpu5V_PWR_EN may set high by set HS_5V_EN, SPK_ENtPinLevel(OUT_uP_5V_PWR_EN)){*/ 
			printk("[%s][workaround] 5V_EN is off, enable and then delay 210ms\r\n", __FUNCTION__);
			uP_nuvoton_write_reg(MICROP_GPIO_OUTPUT_BIT_SET, &sel_offset_5V_EN );
			msleep(110);
                        AX_MicroP_getGPIOOutputPinLevel(OUT_uP_5V_PWR_EN); // have a dummy read to avoid microp enter into idle
                        msleep(100);
		//}
	}
        sel_offset=1<<pinID;
        if(level)
                ret=uP_nuvoton_write_reg(MICROP_GPIO_OUTPUT_BIT_SET,&sel_offset);
        else
                ret=uP_nuvoton_write_reg(MICROP_GPIO_OUTPUT_BIT_CLR,&sel_offset);

	//Eason fix [TT469667]+++
	if( OUT_uP_SPK_EN==pinID && (PAD_P92L==AX_MicroP_getPadModel()||PAD_P93L==AX_MicroP_getPadModel()) ){
		if(level)
			SPK_EN_DuringSetting = false;
	}
	//Eason fix [TT469667]---

       // check after set /clear
       ret=uP_nuvoton_read_reg(MICROP_GPIO_OUTPUT_LEVEL, &out_reg);
       if(ret >=0){
            level_pinID=(out_reg & sel_offset)?1:0;
            printk("[MicroP] AfterSet GPIO pin=%d, level=%d, Out=0x%8x\r\n", pinID, level_pinID, out_reg);
            if(level_pinID!=level){
                    printk("[MicroP] [Debug] cur state =%d\r\n", AX_MicroP_getOPState());                    
                    ret=-3;
            }
       }

       
	return ((ret < 0)?ret:0);

}

EXPORT_SYMBOL_GPL(AX_MicroP_setGPIOOutputPin);


/*
*  @ AX_MicroP_getGPIOOutputPinLevel
*  input:
*           - pinID

*  return: 0 for low, 1 for high, <0 value means something error
*/

int AX_MicroP_getGPIOOutputPinLevel(int pinID){
	int regval=0;
	int ret=0;
	int gpiolevel=0;
	unsigned int sel_offset=1<<pinID;
       pr_debug("[MicroP] try to get GPIO_OutPut pin=%d\r\n",pinID);


        if(AX_MicroP_IsP01Connected()==0){
		printk("%s: P01 removed\r\n",__FUNCTION__);               
		return -1;
	}

	ret=uP_nuvoton_read_reg(MICROP_GPIO_OUTPUT_LEVEL,&regval);
	if(ret > 0){
		gpiolevel=(regval & sel_offset)?1:0;
	}
       
	return ((ret < 0)?ret:gpiolevel);
}

EXPORT_SYMBOL_GPL(AX_MicroP_getGPIOOutputPinLevel);


/*
*  @AX_MicroP_enableInterrupt
*  input: 
*            - intrpin: input pin id
*            -  enable: 0 for 'disable', 1 for 'enable'
*  return: 0 for success, <0 value means something error
*/

int AX_MicroP_enablePinInterrupt(unsigned int pinID, int enable){

       int ret=0;
       int st_microp;       
       printk("[MicroP] enable Pin Intr pin=0x%x, enable=%d\r\n",pinID, enable);


       if(AX_MicroP_IsP01Connected()==0){
            printk("%s: P01 removed\r\n",__FUNCTION__);
            return -1;
       }

       if(isFirmwareUpdating()){
            printk("%s: P01 is updating, retry later\r\n",__FUNCTION__);
            return -2;
       }
       
        uP_nuvoton_read_reg(MICROP_OPERATING_STATE, &st_microp);
        if(st_MICROP_Off==st_microp){
            printk("%s: microp is under off state, skip intr set/clr operations\r\n",__FUNCTION__);
            return -3;
        }
            
        if(enable){
                ret=uP_nuvoton_write_reg(MICROP_INTR_EN_BIT_SET,&pinID);
        }
        else{
                ret=uP_nuvoton_write_reg(MICROP_INTR_EN_BIT_CLR,&pinID);                                                
        }
        
	return ((ret < 0)?ret:0);

}

EXPORT_SYMBOL_GPL(AX_MicroP_enablePinInterrupt);




// return = 0: success
//           <0: error
int AX_MicroP_setPWMValue(uint8_t value){
    int flag=0;
    uint8_t value_get = 255;
    if(value < 0 && value > 255){
        printk("PWM set value not valid\r\n");
        return -1;
    }
    if(AX_MicroP_IsP01Connected()==0){
        printk("%s: P01 removed\r\n",__FUNCTION__);
        return -1;
    }

    if(isFirmwareUpdating()){
        printk("%s: P01 is updating, retry later\r\n",__FUNCTION__);
        return -2;
    }
    printk("%s: set value=%d\r\n", __FUNCTION__, value);
    flag=uP_nuvoton_write_reg(MICROP_PWM, &value);
	
    uP_nuvoton_read_reg(MICROP_PWM, &value_get);
    if(value != value_get)
    {
	 flag=uP_nuvoton_write_reg(MICROP_PWM, &value);
	 uP_nuvoton_read_reg(MICROP_PWM, &value_get);
	 if(value != value_get)
	 {
	 	printk("%s: set PWM fail=%d\n",__FUNCTION__,value_get);
	 	return -1;
	 }else{
	 	printk("%s: get PWM retry=%d\n",__FUNCTION__,value_get);
	 }
    }else{
       printk("%s: get PWM=%d\n",__FUNCTION__,value_get);
    }
	
    return (flag>=0)?0:flag;
}

// return >=0: success
//           <0: error
int AX_MicroP_getPWMValue(void){
    uint8_t value=0;
    int flag=0;
    flag=uP_nuvoton_read_reg(MICROP_PWM, &value);
    if(flag>=0){
            printk("%s: get value=%d\r\n", __FUNCTION__, value);
            return (int)value;
    }
    else
            return flag;
}

EXPORT_SYMBOL_GPL(AX_MicroP_setPWMValue);
EXPORT_SYMBOL_GPL(AX_MicroP_getPWMValue);


int AX_MicroP_enterSleeping(void){
    int flag=0;
    uint16_t  value=0x0055;
    printk("%s\r\n", __FUNCTION__);
    flag=uP_nuvoton_write_reg(MICROP_IND_A68_SLEEP, &value);
// ASUS_BSP +++ Peter_Lu "For Pad I2C suspend/resume issue"
    g_i2c_bus_3v3_state = 0;
// ASUS_BSP ---
    return (flag>=0)?0:flag;
}


int AX_MicroP_enterResuming(void){
    int flag=0;
    int retries=15;
    int cur_state=st_MICROP_Unknown;
    uint16_t  value=0x0069;
    uint32_t st_jiffies=jiffies;
    printk("%s\r\n", __FUNCTION__);
    flag=uP_nuvoton_write_reg(MICROP_IND_A68_RESUME, &value);
    if(flag){
        g_i2c_microp_busy=1;
        flag=uP_nuvoton_read_reg(MICROP_OPERATING_STATE, &cur_state);        
    }
    
    while (flag>=0 && st_MICROP_Active != cur_state && retries-- > 0) {            
            msleep(30);
            flag=uP_nuvoton_read_reg(MICROP_OPERATING_STATE, &cur_state);        
    }
    if(flag >=0 && retries > 0)
        printk("Success: takes %lu jiffies~~\r\n", jiffies - st_jiffies);
    else
        printk("\r\nFailed!!\r\n");
// ASUS_BSP +++ Peter_Lu "For Pad I2C suspend/resume issue"
    g_i2c_bus_3v3_state = 1;
// ASUS_BSP ---
    g_i2c_microp_busy=0;
    return (flag>=0)?0:flag;
}

EXPORT_SYMBOL_GPL(AX_MicroP_enterSleeping);
EXPORT_SYMBOL_GPL(AX_MicroP_enterResuming);

/*
*  @AX_MicroP_readBattCapacity
*  input: target
*           0: p01 battery
*           1: dock battery
*  return: value >=0: success, value < 0: error
*/



extern int asusdec_dock_battery_callback(void);
extern int isAlwaysPowerOnMicroP(void);
int AX_MicroP_readBattCapacity(int target){
        static int dock_cap=0;
        static int pad_cap=0;
        
        int flag=0;

        if(AX_MicroP_IsP01Connected()==0){
            printk("%s: P01 removed\r\n",__FUNCTION__);
            goto failed;
       }

        if(isFirmwareUpdating()){
            printk("%s: P01 is updating, retry later\r\n",__FUNCTION__);
            goto failed;
       }


        if(target==1){
            
         }
        else{

                    if(isFirmwareUpdating()){
                            printk("%s: P01 updating, not support read cap now\r\n",__FUNCTION__);
                            goto failed;
                    }

                    // redundant read to wakeup microp from power-down mode and trigger microp to read h/w gauge again
                    flag=uP_nuvoton_read_reg(MICROP_GAUGE_CAP, &pad_cap); 

                    flag=uP_nuvoton_read_reg(MICROP_GAUGE_CAP,&pad_cap);
                    if(flag < 0){
                            printk("%s: read cap fail due to i2c flag=%d\r\n",__FUNCTION__, flag);
                            goto failed;
                    }
        }
        printk("%s: target[%s]=%d\r\n", __FUNCTION__, (target==1)?"dock":"p01",(target==1)?dock_cap:pad_cap);

failed:        
        return (target==1)?dock_cap:pad_cap;
}



int AX_IsPadUsing_MHL_H(void){
       int regval=0;
       int ret=0;
       int gpiolevel=0;
       unsigned int sel_offset=1<<IN_LCD_ID;

       if(AX_MicroP_IsP01Connected()==0){
            printk("%s: P01 removed\r\n",__FUNCTION__);
            return -1;
       }

       if(isFirmwareUpdating()){
            printk("%s: P01 is updating, retry later\r\n",__FUNCTION__);
            return -2;
       }
       
       ret=uP_nuvoton_read_reg(MICROP_GPIO_INPUT_LEVEL,&regval);
       if(ret > 0){
                gpiolevel=(regval & sel_offset)?1:0;
       }

       return ((ret < 0)?ret:gpiolevel);
}

EXPORT_SYMBOL_GPL(AX_IsPadUsing_MHL_H);

int AX_MicroP_getOPState(void){
       int regval=0;
       int ret=0;

       if(AX_MicroP_IsP01Connected()==0){
            printk("%s: P01 removed\r\n",__FUNCTION__);
            return -1;
       }

       if(isFirmwareUpdating()){
            printk("%s: P01 is updating, retry later\r\n",__FUNCTION__);
            return -2;
       }

       ret=uP_nuvoton_read_reg(MICROP_OPERATING_STATE,&regval);

       return ((ret < 0)?ret:regval);
}

EXPORT_SYMBOL_GPL(AX_MicroP_getOPState);


/*
*  return value: please refer to microp spec
*  return value < 0 for erros
*/

int AX_MicroP_getMHLNvramState(void){
       int regval=0;
       int ret=0;

       if(AX_MicroP_IsP01Connected()==0){
            printk("%s: P01 removed\r\n",__FUNCTION__);
            return -1;
       }

       if(isFirmwareUpdating()){
            printk("%s: P01 is updating, retry later\r\n",__FUNCTION__);
            return -2;
       }

       ret=uP_nuvoton_read_reg(MICROP_GET_MVRAM_STATE_FOR_FACTORY,&regval);

       return ((ret < 0)?ret:regval);
}

EXPORT_SYMBOL_GPL(AX_MicroP_getMHLNvramState);


int AX_MicroP_writeKDataOfLightSensor(uint64_t data){
       int ret=0;

       if(AX_MicroP_IsP01Connected()==0){
            printk("%s: P01 removed\r\n",__FUNCTION__);
            return -1;
       }

       if(isFirmwareUpdating()){
            printk("%s: P01 is updating, retry later\r\n",__FUNCTION__);
            return -2;
       }

       ret=uP_nuvoton_write_reg(MICROP_CALIBRATION_DATA,&data);

       return ((ret < 0)?ret:0);
}

EXPORT_SYMBOL_GPL(AX_MicroP_writeKDataOfLightSensor);

uint64_t AX_MicroP_readKDataOfLightSensor(void){
       int ret=0;
       uint64_t kdata=0;
       if(AX_MicroP_IsP01Connected()==0){
            printk("%s: P01 removed\r\n",__FUNCTION__);
            return -1;
       }

       if(isFirmwareUpdating()){
            printk("%s: P01 is updating, retry later\r\n",__FUNCTION__);
            return -2;
       }

       ret=uP_nuvoton_read_reg(MICROP_CALIBRATION_DATA,&kdata);

       return ((ret < 0)?ret:kdata);

}
EXPORT_SYMBOL_GPL(AX_MicroP_readKDataOfLightSensor);


int AX_MicroP_writeCompassData(char* data, int length)
{
	int ret = -1;
	int i = 0;
	int j = 0;
	char write_buf[9];
	char read_buf[512];
	
	if(data == NULL){
		printk("%s: data is null\r\n",__FUNCTION__);
		return -1;
	}
	
	if( ((length%8)!=0) || length>512){
		printk("%s: length is wrong (%d)\r\n",__FUNCTION__, length);
		return -1;
	}
	
	if(AX_MicroP_IsP01Connected()==0){
		printk("%s: Pad is not connected\r\n",__FUNCTION__);
		return -1;
	}
	
	if(isFirmwareUpdating()){
		printk("%s: Pad is updating, retry later\r\n",__FUNCTION__);
		return -2;
	}
	
	for(i=0; i<length; i+=8){
		write_buf[0] = i/4;
		memcpy(write_buf+1, data+i, 8);
		
		ret = uP_nuvoton_write_reg(MICROP_COMPASS_PARAMETER, write_buf);
		if(ret < 0){
			printk("%s: write reg error\r\n",__FUNCTION__);
			return -1;
		}
		
		memset(read_buf, 0, 9);
		read_buf[0] = i/4;
		ret = uP_nuvoton_read_reg(MICROP_COMPASS_PARAMETER, read_buf);
		if(ret < 0){
			printk("%s: read reg error\r\n",__FUNCTION__);
			return -1;
		}
		else{
			for(j=0; j<8; j++){
				if(read_buf[j] != write_buf[j]){
					printk("%s: check compass data error, i = %d\r\n",__FUNCTION__, i);
					return -1;
				}
			}
		}
	}
	
	return 0;
}
EXPORT_SYMBOL_GPL(AX_MicroP_writeCompassData);

int AX_MicroP_readCompassData(char* data, int length)
{
	int ret = -1;
	int i = 0;
	char read_buf[9];
	
	if(data == NULL){
		printk("%s: data is null\r\n",__FUNCTION__);
		return -1;
	}
	
	if( ((length%8)!=0) || length>512){
		printk("%s: length is wrong (%d)\r\n",__FUNCTION__, length);
		return -1;
	}
	
	if(AX_MicroP_IsP01Connected()==0){
		printk("%s: Pad is not connected\r\n",__FUNCTION__);
		return -1;
	}
	
	if(isFirmwareUpdating()){
		printk("%s: Pad is updating, retry later\r\n",__FUNCTION__);
		return -2;
	}
	
	for(i=0; i<length; i+=8){
		memset(read_buf, 0, 9);
		
		read_buf[0] = (i/4) | 0x80;
		ret = uP_nuvoton_write_reg(MICROP_COMPASS_PARAMETER, read_buf);
		if(ret < 0){
			printk("%s: write reg error\r\n",__FUNCTION__);
			return -1;
		}
		
		read_buf[0] = i/4;
		ret = uP_nuvoton_read_reg(MICROP_COMPASS_PARAMETER, read_buf);
		if(ret < 0){
			printk("%s: read reg error\r\n",__FUNCTION__);
			return -1;
		}
		else{
			memcpy(data+i, read_buf+1, 8);
		}
	}
	
	return 0;
}
EXPORT_SYMBOL_GPL(AX_MicroP_readCompassData);


/*
*  @get_MicroP_HUB_SLEEP_STATUS
*  return: value = 1: turn on, value = 0: turn off
*/
/*     TBD

int get_MicroP_HUB_SLEEP_STATUS(void){
	return AX_MicroP_getGPIOOutputPinLevel(OUT_uP_HUB_SLEEP);
}

EXPORT_SYMBOL_GPL(get_MicroP_HUB_SLEEP_STATUS);
*/

#ifdef CONFIG_ASUSDEC

int get_EC_DOCK_IN_STATUS(void){
    return AX_MicroP_getGPIOPinLevel(IN_DOCK_IN_R);
};


int get_EC_AP_WAKE_STATUS(void){
    return AX_MicroP_getGPIOPinLevel(IN_DOCK_AP_WAKE_R);
};


int set_EC_REQUEST_VALUE(int value){
	return AX_MicroP_setGPIOOutputPin(OUT_uP_EC_REQUEST_R,value);
};


int get_EC_HALL_SENSOR_STATUS(void){
	return AX_MicroP_getGPIOPinLevel(IN_O_LID_R);
};


int EC_Init_Complete(void){
        printk("[MicroP] EC -> MicroP: Init Ready +++\r\n");
        micropSendNotify(DOCK_INIT_READY);
        printk("[MicroP] EC -> MicroP: Init Ready ---\r\n");
        return 0;
}

int EC_Get_EXT_POWER_PLUG_IN_Ready(void){
        printk("[MicroP][%s] +++\r\n", __FUNCTION__);
        micropSendNotify(DOCK_EXT_POWER_PLUG_IN_READY);
        printk("[MicroP][%s] ---\r\n", __FUNCTION__);
        return 0;
}

int EC_Get_EXT_POWER_PLUG_OUT_Ready(void){
        printk("[MicroP][%s] +++\r\n", __FUNCTION__);
        micropSendNotify(DOCK_EXT_POWER_PLUG_OUT_READY);
        printk("[MicroP][%s] ---\r\n", __FUNCTION__);
        return 0;
}

int EC_Get_DOCK_BATTERY_POWER_BAD_READY(void){
        printk("[MicroP][%s] +++\r\n", __FUNCTION__);
        micropSendNotify(DOCK_BATTERY_POWER_BAD_READY);
        printk("[MicroP][%s] ---\r\n", __FUNCTION__);
        return 0;
}

/*
*       Check the status of Dock connectness
*       return value: 1: Dock connected and ready
*/
int AX_MicroP_IsDockReady(void){
	return asusdec_Dock_Ready_status()?1:0;
}

#else

int get_EC_DOCK_IN_STATUS(void){
    return 0;
};


int get_EC_AP_WAKE_STATUS(void){
    return 0;
};


int set_EC_REQUEST_VALUE(int value){
    return 0;
};

int get_EC_HALL_SENSOR_STATUS(void){
    return 0;
};


int EC_Init_Complete(void){
        return 0;
}


int EC_Get_EXT_POWER_PLUG_IN_Ready(void){
        return 0;
}


int EC_Get_EXT_POWER_PLUG_OUT_Ready(void){
        return 0;
}

int EC_Get_DOCK_BATTERY_POWER_BAD_READY(void){
        return 0;
}

/*
*       Check the status of Dock connectness
*       return value: 1: Dock connected and ready
*/
int AX_MicroP_IsDockReady(void){
    return 0;
}
#endif

EXPORT_SYMBOL_GPL(get_EC_DOCK_IN_STATUS);
EXPORT_SYMBOL_GPL(get_EC_AP_WAKE_STATUS);
EXPORT_SYMBOL_GPL(set_EC_REQUEST_VALUE);
EXPORT_SYMBOL_GPL(get_EC_HALL_SENSOR_STATUS);
EXPORT_SYMBOL_GPL(EC_Init_Complete);
EXPORT_SYMBOL_GPL(EC_Get_EXT_POWER_PLUG_IN_Ready);
EXPORT_SYMBOL_GPL(EC_Get_EXT_POWER_PLUG_OUT_Ready);
EXPORT_SYMBOL_GPL(EC_Get_DOCK_BATTERY_POWER_BAD_READY);
EXPORT_SYMBOL_GPL(AX_MicroP_IsDockReady);

int AX_MicroP_initLightsensor(uint8_t bOn){
       int ret=0;
       
       if(AX_MicroP_IsP01Connected()==0){
            printk("%s: P01 removed\r\n",__FUNCTION__);
            return -1;
       }

       if(isFirmwareUpdating()){
            printk("%s: P01 is updating, retry later\r\n",__FUNCTION__);
            return -2;
       }       
       printk("%s: lightSensor => %s\r\n", __FUNCTION__, (bOn?"ON":"OFF"));
       ret=uP_nuvoton_write_reg(MICROP_LS_INIT,&bOn);

       return ((ret < 0)?ret:0);
}

int AX_MicroP_getLightsensorInitResult(void){
       int ret=0;
       uint8_t l_initResult=0;
       if(AX_MicroP_IsP01Connected()==0){
            printk("%s: P01 removed\r\n",__FUNCTION__);
            return -1;
       }

       if(isFirmwareUpdating()){
            printk("%s: P01 is updating, retry later\r\n",__FUNCTION__);
            return -2;
       }

       ret=uP_nuvoton_read_reg(MICROP_LS_RESULT,&l_initResult);

       return ((ret < 0)?ret:l_initResult);

}
int AX_MicroP_getLightsensorADC(void){
       int ret=0;
       uint16_t l_adc=0;
       if(AX_MicroP_IsP01Connected()==0){
            printk("%s: P01 removed\r\n",__FUNCTION__);
            return -1;
       }

       if(isFirmwareUpdating()){
            printk("%s: P01 is updating, retry later\r\n",__FUNCTION__);
            return -2;
       }

       ret=uP_nuvoton_write_reg(MICROP_LS_ADC,&l_adc);
	msleep(2);
	ret=uP_nuvoton_read_reg(MICROP_LS_ADC,&l_adc);

       return ((ret < 0)?ret:l_adc);


}
int AX_MicroP_setLightsensor_TS_Low(uint16_t val){
       int ret=0;
       
       if(AX_MicroP_IsP01Connected()==0){
            printk("%s: P01 removed\r\n",__FUNCTION__);
            return -1;
       }
       if(isFirmwareUpdating()){
            printk("%s: P01 is updating, retry later\r\n",__FUNCTION__);
            return -2;
       }
       
       printk("%s: val=0x%x\r\n", __FUNCTION__, val);
       
       ret=uP_nuvoton_write_reg(MICROP_LS_SET_THRESHOLD_LOW,&val);
       
       return ((ret < 0)?ret:0);

}
int AX_MicroP_setLightsensor_TS_High(uint16_t val){
       int ret=0;
       
       if(AX_MicroP_IsP01Connected()==0){
            printk("%s: P01 removed\r\n",__FUNCTION__);
            return -1;
       }
       if(isFirmwareUpdating()){
            printk("%s: P01 is updating, retry later\r\n",__FUNCTION__);
            return -2;
       }       
       
       printk("%s: val=0x%x\r\n", __FUNCTION__, val);
       
       ret=uP_nuvoton_write_reg(MICROP_LS_SET_THRESHOLD_HIGH,&val);

       return ((ret < 0)?ret:0);
}

EXPORT_SYMBOL_GPL(AX_MicroP_initLightsensor);
EXPORT_SYMBOL_GPL(AX_MicroP_getLightsensorInitResult);
EXPORT_SYMBOL_GPL(AX_MicroP_getLightsensorADC);
EXPORT_SYMBOL_GPL(AX_MicroP_setLightsensor_TS_Low);
EXPORT_SYMBOL_GPL(AX_MicroP_setLightsensor_TS_High);

int AX_MicroP_getHWID(void){
	int ret = 0;
	uint8_t hwid = 0; 

	if(AX_MicroP_IsP01Connected()==0){
		printk("%s: P01 removed\r\n",__FUNCTION__);
		return -1;
	}
#if 0 //hwid also support in boot mode
	if(isFirmwareUpdating()){
		printk("%s: P01 is updating, retry later\r\n",__FUNCTION__);
		return -2;
	}

#endif
	ret = uP_nuvoton_read_reg(MICROP_PAD_HWID, &hwid);
	
	if(ret < 0){
		printk("%s: read reg error\r\n",__FUNCTION__);
		return -1;
	}
	else {
		hwid = (hwid & 0x3);
		
		if(hwid == 0){
			ret = P92_SR1_HWID;
		}
		else if(hwid == 1){
			ret = P92_ER1_HWID;
		}
		else if(hwid == 2){
			ret = P92_ER2_HWID;
		}
		else if(hwid == 3){
			ret = P92_PR_HWID;
		}
	}
	
	return ret;
}
EXPORT_SYMBOL_GPL(AX_MicroP_getHWID);
int AX_MicroP_getPadModel(void){
	int ret = PAD_UNKNOWN_MODEL;
       char sz_oemVerStr[24]={0};
        uint8_t hwid = 0;
	if(AX_MicroP_IsP01Connected()==0){
		printk("%s: P01 removed\r\n",__FUNCTION__);
		return -1;
	}
	
#if 0 //hwid also support in boot mode	
	if(isFirmwareUpdating()){
		printk("%s: P01 is updating, retry later\r\n",__FUNCTION__);
		return -2;
	}
#endif

        ret=uP_nuvoton_read_reg(MICROP_OEM_FW_VERSION,sz_oemVerStr);
	ret = uP_nuvoton_read_reg(MICROP_PAD_HWID, &hwid);

	if(ret < 0){
		printk("%s: read reg error\r\n",__FUNCTION__);
		return -1;
	}
	else {
			printk("%s: sz_oemVerStr=%s, hwid=0x%x\r\n",__FUNCTION__, sz_oemVerStr, hwid);

		       if(0==strncmp(sz_oemVerStr, "PFECA", 5)){
		                    ret=PAD_P05_P05E;
		       } 
		       else if(0==strncmp(sz_oemVerStr, "PFECB", 5)){
		                    ret=PAD_P71L;
		        }
		       else if(0==strncmp(sz_oemVerStr, "PFECC", 5)){
		                    ret=PAD_P101G;
		        }
		       else if(0==strncmp(sz_oemVerStr, "PFECD", 5)){                            
                                    // bit5 of reg[0x26], 1: P93L, 0:P92L
                                    if(hwid & 0x20)
                                        ret=PAD_P93L;
                                    else
                                        ret=PAD_P92L;
		        }
		       else if(0==strncmp(sz_oemVerStr, "PFECE", 5)){
		                    ret=PAD_P05C;
		        }
		       else{
				printk("%s: get Pad Model fail\n", __FUNCTION__) ;
				ret=PAD_UNKNOWN_MODEL;
		       }

	}
	
	return ret;
}

EXPORT_SYMBOL_GPL(AX_MicroP_getPadModel);

/*
*  return value
    1: Laibao
    0: Wintek

*/

int AX_MicroP_getTSID(void){
	int ret = 0;
	uint8_t hwid = 0; 

	if(AX_MicroP_IsP01Connected()==0){
		printk("%s: P01 removed\r\n",__FUNCTION__);
		return -1;
	}
	
	if(isFirmwareUpdating()){
		printk("%s: P01 is updating, retry later\r\n",__FUNCTION__);
		return -2;
	}
       
	ret = uP_nuvoton_read_reg(MICROP_PAD_HWID, &hwid);
	
	if(ret < 0){
		printk("%s: read reg error\r\n",__FUNCTION__);
		return -1;
	}
	else {
		ret = (hwid & 0x18) >> 3;
	}
	
	return ret;
}
EXPORT_SYMBOL_GPL(AX_MicroP_getTSID);


/*
*  return value
    1: CPT
    0: Panasonic 

*/
int AX_MicroP_getLCMID(void){
	int ret = 0;
	uint8_t hwid = 0; 

	if(AX_MicroP_IsP01Connected()==0){
		printk("%s: P01 removed\r\n",__FUNCTION__);
		return -1;
	}
	
	if(isFirmwareUpdating()){
		printk("%s: P01 is updating, retry later\r\n",__FUNCTION__);
		return -2;
	}
       
	ret = uP_nuvoton_read_reg(MICROP_PAD_HWID, &hwid);
	
	if(ret < 0){
		printk("%s: read reg error\r\n",__FUNCTION__);
		return -1;
	}
	else {
		ret = (hwid & 0x4) >> 2;
	}	
	return ret;
}
EXPORT_SYMBOL_GPL(AX_MicroP_getLCMID);

int AX_MicroP_IsMydpNewSKU(void){
	int ret = 0;
	uint8_t hwid = 0; 
	
	ret = uP_nuvoton_read_reg(MICROP_PAD_HWID, &hwid);
	
	if(ret < 0){
		printk("%s: read reg error\r\n",__FUNCTION__);
		return -1;
	}
	else {
		ret = (hwid & 0x80) >> 7;
	}
	
	return ret;
}
EXPORT_SYMBOL_GPL(AX_MicroP_IsMydpNewSKU);

int AX_MicroP_ControlLED(uint8_t styleofOrange, uint8_t styleofGreen){
	int ret = 0;

       uint8_t ledconbytes[2]={0,0}; //[0]: orange [1]: green

       

        ledconbytes[1] = styleofGreen;
        ledconbytes[0] = styleofOrange;


        
	ret = uP_nuvoton_write_reg(MICROP_LED_CONTROL, &ledconbytes);
	
	if(ret < 0){
		printk("%s: write reg error\r\n",__FUNCTION__);
		return -1;
	}

       return ((ret < 0)?ret:0);
}
EXPORT_SYMBOL_GPL(AX_MicroP_ControlLED);



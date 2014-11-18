/*
 * Definitions for CM36283 proximity sensor with ALS.
 */
#ifndef CM36283_H
#define CM36283_H

#include <linux/ioctl.h>
//#include <asm-arm/arch/regs-gpio.h>

#define CM36283_I2C_ADDRESS     0x60     //slave address for i2c

/* CM36283 Register  (Please refer to CM36283 Specifications) */
#define CM36283_ALS_CONF        0x00    //write control cmd for ambient light sensor.
#define CM36283_ALS_THDH        0x01    //write high interrupt threshold of ambient light data.
#define CM36283_ALS_THDL        0x02    //write low interrupt threshold of ambient light data.
#define CM36283_PS_CONF         0x03    //write control cmd for proximity sensor.
#define CM36283_PS_MODE         0x04    //write ps interrupt mode selection.
#define CM36283_PS_CANC         0x05    //write ps cancellation level setting.
#define CM36283_PS_THD          0x06    //write ps interrupt threshold setting.
#define CM36283_PS_DATA         0x08    //read ps output data.
#define CM36283_ALS_DATA        0x09    //read als output data.
#define CM36283_INT_FLAGS       0x0B    //read als/ps interrupt flags.
#define CM36283_SENSORS_ID      0x0C    //read device ID.


/* Bit Description for each cmd */
#define INIT_ALS	0x0
#define INIT_PS		0x0
#define DEFAULT_PS_THRESHOLD_lo	10
#define DEFAULT_PS_THRESHOLD_hi 16


enum cm36283_sensors {
	PS_SENSOR=0,
	ALS_SENSOR,
};

struct cm36283_platform_data {
    int   (*init_platform_hw)(void);//(struct i2c_client *client);
    int   (*exit_platform_hw)(void);//(struct i2c_client *client);
    u8    (*read_int_pin_state)(void);
};

/* IOCTLs for KXTF9 misc. device library */
/*
#define KXTF9IO						   	0x86
#define KXTF9_IOCTL_INIT                  		_IO(KXTF9IO, 0x01)
#define KXTF9_IOCTL_READ_CHIPINFO         	_IOR(KXTF9IO, 0x02, int)
#define KXTF9_IOCTL_READ_SENSORDATA     	_IOR(KXTF9IO, 0x03, int)
#define KXTF9_IOCTL_READ_POSTUREDATA    	_IOR(KXTF9IO, 0x04, int)
#define KXTF9_IOCTL_READ_CALIDATA         	_IOR(KXTF9IO, 0x05, int)
#define KXTF9_IOCTL_READ_CONTROL          	_IOR(KXTF9IO, 0x06, int)
#define KXTF9_IOCTL_SET_CONTROL           	_IOW(KXTF9IO, 0x07, int)
#define KXTF9_IOCTL_SET_MODE              		_IOW(KXTF9IO, 0x08, int)
*/

/* IOCTLs for TEST tools */
#define LS_IOC_MAGIC					0xf4
#define READ_LIGHT_DATA_ON_OFF				_IOW(LS_IOC_MAGIC, 1, int)
#define ATD_ASK_LIGHTDATA 				_IOW(LS_IOC_MAGIC, 6, int*) 
#define IOCTL_ENABLE_CTRL 				_IOW(LS_IOC_MAGIC, 7, int)
#define CM36283_ANDROID_OR_OMS				_IOW(LS_IOC_MAGIC, 8, int)

#define PS_IOC_MAGIC					0xf5
#define PS_POLL_READ_GPIO_ON_OFF			_IOW(PS_IOC_MAGIC, 1, int)
#define ATD_ASK_PR_STATUS 				_IOW(PS_IOC_MAGIC, 6, int*)

#endif


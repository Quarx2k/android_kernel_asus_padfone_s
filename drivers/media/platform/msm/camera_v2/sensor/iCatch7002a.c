/*
 * kernel/drivers/media/video/tegra
 *
 * iCatch SPCA7002A ISP driver
 *
 * Copyright (C) 2012 ASUS Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

////////////////////////////////////////////////////////
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/syscalls.h>

#undef _CAM_SENSOR_DETECT_
#ifdef _CAM_SENSOR_DETECT_
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <mach/clk.h>
#endif
#include <linux/switch.h>
#include <iCatch7002a.h>
#include <linux/proc_fs.h>
#include <linux/cred.h>
#include <linux/security.h>
#include <linux/workqueue.h>
#include "app_i2c_lib_icatch.h"
#include <linux/time.h>
////////////////////////////////////////////////////////
#define I7002A_SDEV_NAME "camera"
#define SPI_CMD_BYTE_READ 	0x03
#define SPI_CMD_RD_ID 		0x9F
#define SPI_CMD_WRT_EN		0x06
#define SPI_CMD_BYTE_PROG 	0x02
#define SPI_CMD_RD_STS		0x05
#define SPI_CMD_BYTE_PROG_AAI	0xAD
#define SPI_CMD_WRT_STS_EN	0x50
#define SPI_CMD_WRT_STS 	0x01
#define SPI_CMD_WRT_DIS 	0x04
#define SPI_CMD_ERASE_ALL	0xC7
#define	SPI_CMD_SECTOR_ERASE		0x20
#define	SPI_CMD_32KB_BLOCK_ERASE	0x52
#define	SPI_CMD_64KB_BLOCK_ERASE	0xD8

#define SENSOR_ID_MI1040	0x2481
#define SENSOR_ID_OV2720	0x2720
#define SENSOR_ID_IMX175	0x175

#define ISP_FLASH_I2C_WRITE_BYTES 64

#define NEED_UPDATE		       0x0
#define UPDATE_UNNECESSARY	0x1
#define UPDATE_TIMEOUT			0x2
#define UPDATE_WRONG_FLASHID	0x3

#define BIN_FILE_HEADER_SIZE 32

#define LED_PMIC_FLASH_STROBE_PATH "/sys/class/leds/led:flash_0/strobe"
#define LED_PMIC_TORCH_STROBE_PATH "/sys/class/leds/led:flash_torch/strobe"
#define LED_PMIC_FLASH_ENABLE_PATH "/sys/class/leds/led:flash_0/brightness"
#define LED_PMIC_TORCH_ENABLE_PATH "/sys/class/leds/led:flash_torch/brightness"
#define LED_PMIC_FLASH_MXA_BRIGHTNESS 1000
#define LED_PMIC_TORCH_MXA_BRIGHTNESS 110

#define ICATCH7002A_DELAY_TEST
#ifdef ICATCH7002A_DELAY_TEST
static u32 iCatch7002a_init_delay= 5;
static u32 iCatch7002a_preview_delay=100;
static u32 touch_focus_enable=0;
#endif
#define CAM_TWO_MODE
#ifdef CAM_TWO_MODE
static unsigned int g_div = 100;
//static int g_initialized_1280_960=0;
//static int g_initialized_1080p=0;
#endif
static int exif_debug=0; //Asus jason_yeh add exif_debug message
#define ISP_TORCH_INT_GPIO 19
#define ISP_INT_GPIO 25

////////////////////////////////////////////////////////
char debug_buf[1024];

unsigned int version_num_in_isp = 0xffffff;
static unsigned int b_fw_is_valid = 1;
//struct pm_qos_request pm_qos_req_list;

static bool g_spi_is_support_1M = true;
static char  BIN_FILE_WITH_PATH[] = "/asusfw/camera/BOOT.BIN";
char  BIN_FILE_WITH_PATH_A91[] = "/asusfw/camera/BOOT_A91.BIN";
static char* UPDATE_FILE_WITH_PATH;
static int iCatch_is_updating = 0;
static bool g_enable_roi_debug = false;
extern struct msm_sensor_ctrl_t mt9m114_s_ctrl;
extern struct msm_sensor_ctrl_t ov2720asus_s_ctrl;
//struct msm_sensor_ctrl_t ov2720asus_s_ctrl;
static int dbg_i7002a_page_index = 2047;
static bool g_afmode=0; //0: Auto AF, 1:Full search AF
static int g_flash_mode = 0;	// ASUS_BSP jim3_lin "LED mode for EXIF"

static bool sensor_opened = false;
/* Used for calculating the iCatch fw update progress */
static int g_page_count = -1;
static int g_total_page_count = -1;

unsigned char g_camera_status = 1; //ASUS_BSP Stimber "Add ATD proc interface"

//ASUS_BSP +++ Stimber "Implement the interface for calibration"
u32 is_calibration = 0;
static u32 calibrating = 0;
bool g_calibrating = false;
//ASUS_BSP --- Stimber "Implement the interface for calibration

static u32 single_image = 0;	//ASUS_BSP Stimber "Interface for single image"
static bool g_TAEenable = 0;	//0: Disable, 1:Enable

static int g_is_hdr_on = 0;	//0: Disable, 1:Enable //ASUS_BSP stimber "Implement HDR feature"
static int g_is_nr_on = 0;	//0: Disable, 1:Enable //ASUS_BSP stimber "Implement NR feature"
int g_is_eis_on = 0;	//0: Disable, 1:Enable //ASUS_BSP bill_chen "Implement image stabilization"

static bool  is_calibration_table_set = true;
bool g_isAFCancel = false;
bool g_isAFDone = true;

//ASUS_BSP +++ Stimber "Modify for preview/video frame rate"
#define DEFAULT_SETTING 0
static int g_LastFixFPS = 0;
static int g_LastMaxExp = 0;
static int g_LastMiniISO = 0;
static int g_LastVideoMode = 0;
static int g_LastMaxFPS = 0;
//ASUS_BSP --- Stimber "Modify for preview/video frame rate"

static bool g_is_max_exp_on = false;

struct completion g_iCatch_comp;
struct completion g_iCatch_torch_comp;
static bool caf_mode = false;
struct work_struct icatch_torch_on_wq;
struct work_struct icatch_torch_off_wq;
static struct workqueue_struct *icatch_torch_workqueue;
int g_pre_res = MSM_SENSOR_INVALID_RES;
int g_cur_res = MSM_SENSOR_INVALID_RES;
bool iCatch_first_open = true;
bool g_ISPbootup = false;  //ASUS_BSP LiJen "ISP flash power state"
bool g_is1stFrame = false; //ASUS_BSP jim3_lin "Add log while kernel got 1st frame after streamon"
bool frontcam_capture_status = false;
bool g_isISP_IRQ_eanble = false;
int g_pre_torch_irq_val=-1;
static int g_pmic_flash_status=PMIC_FLASH_STATUS_FLASH_OFF;

#define FRAME_COUNT_INIT 5
int g_frame_count = FRAME_COUNT_INIT;

//Focus Parameter
cam_area_t g_AF_ROI;
cam_area_t g_AE_ROI;
cam_focus_mode_type g_AF_MODE;
struct exif_cfg g_JpegExif; 

int AF_timecount=0;//ASUS BSP ryan_kuo+++ use for cancel AF if AF take too much time 
uint16_t cur_ev=0; //ASUS_BSP : darrency_lin ++ fix ae setting error when take picture in lock ae/af/awb

/*********************** boot from host ************************/
//ASUS_BSP+++ jim3_lin "Add for ATD CameraTest"
extern int spi_rx_transfer(struct spi_device *spi,u8 *rx_buf,unsigned int len);
extern int spi_test_transfer(struct spi_device *spi,u8 *tx_buf,unsigned int len);
extern struct spi_device *g_spi;
//ASUS_BSP--- jim3_lin "Add for ATD CameraTest"
u8 *g_pbootBuf = NULL;
bool g_is_fw_loaded = false;
bool g_is_fw_back_cal_loaded = false;
bool g_is_fw_front_cal_loaded = false;
/*************************************************************/


////////////////////////////////////////////////////////

/* iCatch Camera Firmware Header
 * It locates on the end of the bin file.
 * Total: 32 bytes.
 * byte[0] ~ byte[7]: 0xFF's
 * byte[8] ~ byte[11]: Compensation for Overall Checksum
 * byte[12] ~ byte[15]: Overall Checksum

 * byte[16] ~ byte[20]: 0xFF's
 * byte[21]: Front Format
 * byte[22]: Rear Lane#
 * byte[23]: Front Lane#
 * byte[24] ~ byte[25]: Rear Sensor ID
 * byte[26] ~ byte[27]: Front sensor ID
 * byte[28] ~ byte[31]: FW Version
 */

enum iCatch_fw_update_status{
	ICATCH_FW_UPDATE_SUCCESS,
	ICATCH_FW_UPDATE_FAILED,        
	ICATCH_FW_IS_BURNING,	
       ICATCH_FW_NO_CMD,
};
static enum iCatch_fw_update_status fw_update_status = ICATCH_FW_NO_CMD;

enum iCatch_flash_type{
	ICATCH_FLASH_TYPE_ST,
	ICATCH_FLASH_TYPE_SST,
};
static enum iCatch_flash_type flash_type = ICATCH_FLASH_TYPE_ST;

struct sensor_reg {
	u16 addr;
	u16 val;
};

int sensor_read_reg(struct i2c_client *client, u16 addr, u16 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[4];
	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = (client->addr >> 1);
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8) (addr >> 8);;
	data[1] = (u8) (addr & 0xff);

	msg[1].addr = (client->addr >> 1);
	msg[1].flags = I2C_M_RD;

	msg[1].len = 1;
	msg[1].buf = data + 2;

	err = i2c_transfer(client->adapter, msg, 2);

	if (err != 2)
		return -EINVAL;

	memcpy(val, data+2, 1);
	*val=*val&0xff;

	return 0;
}

int sensor_write_reg(struct i2c_client *client, u16 addr, u16 val)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[4];
	int retry = 0;
	if (!client->adapter)
		return -ENODEV;

       if(((addr >> 8 ) != 0x40) && ((addr >> 8 ) != 0x41) && ((addr >> 8 ) != 0x10)){ //LiJen: filter SPI command
            pr_info("%s reg(0x%x)=0x%x",__func__,addr,val);
       }

	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);
	data[2] = (u8) (val & 0xff);

	msg.addr = (client->addr >> 1);
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;
	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;
		retry++;
		pr_err("yuv_sensor : i2c transfer failed, retrying %x %x\n",
		       addr, val);
//		pr_err("yuv_sensor : i2c transfer failed, count %x, err= 0x%x\n",
//		       __FUNCTION__, __LINE__, msg.addr, err);
		msleep(2); //LiJen: increate i2c retry delay to avoid ISP i2c fail issue
	} while (retry <= SENSOR_MAX_RETRIES);

	if(err == 0) {
		printk("%s(%d): i2c_transfer error, but return 0!?\n", __FUNCTION__, __LINE__);
		err = 0xAAAA;
	}

	return err;
}

static int sensor_sequential_write_reg(struct i2c_client *client, unsigned char *data, u16 datasize)
{
	int err;
	struct i2c_msg msg;
	int retry = 0;

              return 0;
	if (datasize==0)
		return 0;
	if (!client->adapter)
		return -ENODEV;

	msg.addr = (client->addr >> 1);
	msg.flags = 0;
	msg.len = datasize;
	msg.buf = data;
	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;
		retry++;
		//pr_err("yuv_sensor : i2c transfer failed, retrying %x %x\n",
		      // addr, val);
		pr_err("yuv_sensor : i2c transfer failed, count %x \n",
		       msg.addr);
//		msleep(3);
	} while (retry <= SENSOR_MAX_RETRIES);

	return err;
}

static int build_sequential_buffer(unsigned char *pBuf, u16 width, u16 value) {
	u32 count = 0;

	switch (width)
	{
	  case 0:
	  // possibly no address, some focusers use this
	  break;

	  // cascading switch
	  case 32:
	    pBuf[count++] = (u8)((value>>24) & 0xFF);
	  case 24:
	    pBuf[count++] = (u8)((value>>16) & 0xFF);
	  case 16:
	    pBuf[count++] = (u8)((value>>8) & 0xFF);
	  case 8:
	    pBuf[count++] = (u8)(value & 0xFF);
	    break;

	  default:
	    printk("Unsupported Bit Width %d\n", width);
	    break;
	}
	return count;

}

static int sensor_write_table(struct i2c_client *client,
			      const struct sensor_reg table[])
{
	int err;
	const struct sensor_reg *next;
	u16 val;
	unsigned char data[10];
	u16 datasize = 0;

	//for (next = table; next->addr != SENSOR_TABLE_END; next++) {
	next = table;
	while (next->addr != SENSOR_TABLE_END) {
		if (next->addr == SENSOR_WAIT_MS) {
			msleep(next->val);
			next +=1;
			continue;
		}
		if (next->addr == SEQ_WRITE_START) {
			next += 1;
			while (next->addr !=SEQ_WRITE_END) {
				if (datasize==0) {//
					datasize += build_sequential_buffer(&data[datasize], 16, next->addr);
					datasize += build_sequential_buffer(&data[datasize], 8, next->val);
				}
				else
					datasize += build_sequential_buffer(&data[datasize], 8, next->val);
				if (datasize==10) {
					sensor_sequential_write_reg(client, data, datasize);
					datasize = 0;
				}
				next += 1;
			}
			sensor_sequential_write_reg(client, data, datasize); //flush out the remaining buffer.
			datasize = 0;
		}
		else {
			val = next->val;

			err = sensor_write_reg(client, next->addr, val);
			if (err) {
				printk("%s(%d): isensor_write_reg ret= 0x%x\n", __FUNCTION__, __LINE__, err);
				return err;
			}
		}
		next += 1;
	}
	return 0;
}

int I2C_SPIInit(void)
{
	int ret = 0;
	struct sensor_reg SPI_init_seq[] = {
		{0x0026, 0xc0},
		{0x4051, 0x01}, /* spien */
		{0x40e1, 0x00}, /* spi mode */
		{0x40e0, 0x11}, /* spi freq */
		{SENSOR_TABLE_END, 0x0000}
	};

	ret = sensor_write_table(mt9m114_s_ctrl.sensor_i2c_client->client, SPI_init_seq);
	if(ret) {
		printk("%s: init fail. ret= 0x%x\n", __FUNCTION__, ret);
	}
	return ret;
}

/*-------------------------------------------------------------------------
 *  File Name : I2C_SPIInit
 *  return SUCCESS: normal
           FAIL: if wait spi flash time out
 *------------------------------------------------------------------------*/
u32 I2C_SPIFlashPortWait(void)
{
    //u32 cnt = WAIT_COUNT;
#if 0
    while(I2CDataRead(0x40e6) != 0x00){
        cnt--;
        if(cnt == 0x00)
        {
            printf("serial flash port wait time out!!\n");
            return FAIL;
        }
    }
#endif
    return 0;
}

/*-------------------------------------------------------------------------
 *  File Name : I2C_SPIFlashPortWrite
 *  return SUCCESS: normal
           FAIL:    if wait spi flash time out
 *------------------------------------------------------------------------*/
u32 I2C_SPIFlashPortWrite(u32 wData)
{
    sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e3,(u8)wData);
    return I2C_SPIFlashPortWait();
}

u32 I2C_SPIFlashPortRead(void)
{
	u16 ret;
    
      sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e4, &ret);
	/* polling SPI state machine ready */
#if 0
    if (I2C_SPIFlashPortWait() != SUCCESS) {
        return 0;
    }
#endif
      sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e5, &ret);

    return (u32)ret;
}

u32 I2C_SPIFlashRead(
	u32 addr,
	u32 pages,
	u8 *pbuf
)
{
	u32 err = 0;
	u32 i, size=0;
	u32 pageSize = 0x100;

	addr = addr * pageSize;
	size = pages*pageSize;

	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7, 0x00);
	/* Write one byte command*/
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e3, SPI_CMD_BYTE_READ);
	/* Send 3 bytes address*/
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e3, (u8)(addr >> 16));
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e3, (u8)(addr >> 8));
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e3, (u8)(addr));

	for (i = 0; i < size ; i++) {
		*pbuf = I2C_SPIFlashPortRead();
		if((i%256)==0)
			printk("%s: page count: 0x%x\n", __FUNCTION__, (i/256));
		pbuf ++;
	}

	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7, 0x01);

	return err;
}

u32 I2C_SPIFlashReadId(void)
{
	u8 id[3];
	u32 ID;

	id[0] = 0;
	id[1] = 0;
	id[2] = 0;

	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);

	/*read ID command*/
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e3,SPI_CMD_RD_ID);

#if 0
	if (err != SUCCESS) {
		printf("Get serial flash ID failed\n");
		return 0;
	}
#endif

	id[0] = I2C_SPIFlashPortRead();    /* Manufacturer's  ID */
	id[1] = I2C_SPIFlashPortRead();    /* Device ID          */
	id[2] = I2C_SPIFlashPortRead();    /* Manufacturer's  ID */

	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);
	printk("ID %2x %2x %2x\n", id[0], id[1], id[2]);

	ID = ((u32)id[0] << 16) | ((u32)id[1] << 8) | \
    ((u32)id[2] << 0);

	return ID;
}

static const u32 stSpiIdInfo[30] =
{
	/*EON*/
	0x001C3117,
	0x001C2016,
	0x001C3116,
	0x001C3115,
	0x001C3114,
	0x001C3113,
	/*Spansion*/
	0x00012018,
	0x00010216,
	0x00010215,
	0x00010214,
	/*ST*/
	0x00202018,
	0x00202017,
	0x00202016,
	0x00202015,
	0x00202014,
	/*MXIC*/
	0x00C22018,
	0x00C22017,
	0x00C22016,
	0x00C25e16,
	0x00C22015,
	0x00C22014,
	0x00C22013,
	/*Winbond*/
	0x00EF3017,
	0x00EF3016,
	0x00EF3015,
	0x00EF3014,
	0x00EF3013,
	0x00EF5013,
	0x00EF5014,/* For A68 1MB serial flash ID */
	/*Fail*/
	0x00000000,
};

static const u32 sstSpiIdInfo[6] =
{
	/*ESMT*/
	0x008C4016,
	/*SST*/
	0x00BF254A,
	0x00BF2541,
	0x00BF258E,
	0x00BF258D,
	/*Fail*/
	0x00000000,
};

u32
BB_SerialFlashTypeCheck(
	u32 id
)
{
	u32 i=0;
	u32 fullID = 1;
	u32 shift = 0, tblId, type = 0;

	/* check whether SST type serial flash */
	while( 1 ){
		tblId = sstSpiIdInfo[i] >> shift;
		if( id == tblId ) {
			printk("SST type serial flash\n");
			type = 2;
			break;
		}
		if( id == 0x00FFFFFF || id == 0x00000000) {
			return 0;
		}
		if( sstSpiIdInfo[i] == 0x00000000 ) {
			#if 0
			if( fullID ){
				fullID = 0;/* sarch partial ID */
				i = 0;
				shift = 16;
				id = id >> shift;
				continue;
			}
			#endif
			type = 3;
			break;
		}
		i ++;
	}
	if( type == 2 )
		return type;

	i = 0;
	/* check whether ST type serial flash */
	while( 1 ){
		tblId = stSpiIdInfo[i] >> shift;
		if( id == tblId ) {
			printk("ST Type serial flash\n");
			type = 1;
			break;
		}
		if( id == 0x00FFFFFF || id == 0x00000000) {
			return 0;
		}
		if( stSpiIdInfo[i] == 0x00000000 ) {
			if( fullID ){
				fullID = 0;/* sarch partial ID */
				i = 0;
				shift = 16;
				id = id >> shift;
				continue;
			}
			type = 3;
			break;
		}
		i ++;
	}

	return type;
}

int I2C_SPIFlashWrEnable(void)
{
	int ret = 0;

	struct msm_camera_i2c_reg_setting SPI_FlashWrEnable_setting;
	struct msm_camera_i2c_reg_array I2C_SPIFlashWrEnable_seq[]={
		{0x40e7, 0x00},
		{0x40e3, SPI_CMD_WRT_EN},
		{0x40e7, 0x01},
		};		
	SPI_FlashWrEnable_setting.reg_setting=I2C_SPIFlashWrEnable_seq;
 	SPI_FlashWrEnable_setting.addr_type=MSM_CAMERA_I2C_WORD_DATA;
	SPI_FlashWrEnable_setting.data_type=MSM_CAMERA_I2C_BYTE_DATA;
	SPI_FlashWrEnable_setting.size=3;

#if 0
	struct sensor_reg I2C_SPIFlashWrEnable_seq[] = {
		{0x40e7, 0x00},
		{0x40e3, SPI_CMD_WRT_EN},
		{0x40e7, 0x01},
		{SENSOR_TABLE_END, 0x0000}
	};


	ret = sensor_write_table(mt9m114_s_ctrl.sensor_i2c_client->client, &SPI_FlashWrEnable_setting);
#endif
	//printk("I2C_SPIFlashWrEnable \n");
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e3,SPI_CMD_WRT_EN);
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);

	if(ret) {
		printk("%s: fail. ret= 0x%x\n", __FUNCTION__, ret);
	}
	return ret;
}

u32 I2C_SPIStsRegRead(void)
{
	u32 ret;

	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e3,SPI_CMD_RD_STS);
	ret = I2C_SPIFlashPortRead();

	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);

	return ret;
}

void I2C_SPITimeOutWait(u32 poll, u32 *ptimeOut)
{
    /* MAX_TIME for SECTOR/BLOCK ERASE is 25ms */
    u32 sts;
    u32 time = 0;
    while (1) {
        sts = I2C_SPIStsRegRead();
        if (!(sts & poll))	/* sfStatusRead() > 4.8us */ {
            break;
        }
        time ++;
        if( *ptimeOut < time ) {
            printk("iCatch: TimeOut %d, sts=0x%x, poll=0x%x\n",time,sts,poll);
            break;
        }
    }
}

int I2C_SPIStChipErase(
	void
)
{
	u32 timeout;
	int ret = 0;
	printk("iCatch: ST Chip Erasing...\n");

	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e3,SPI_CMD_WRT_STS);
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e3,0x02);
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);

	ret = I2C_SPIFlashWrEnable();
	if (ret) {
		printk("iCatch: ST Chip Erase fail, ret= 0x%x\n", ret);
		return ret;
	}

	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e3,SPI_CMD_ERASE_ALL);
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);

	timeout = 0xffffffff;
	I2C_SPITimeOutWait(0x01, &timeout);
#if 0
	ros_thread_sleep(1);
#endif
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);
	printk("iCatch: ST Chip Erased\n");
	return 0;
}

int I2C_SPISstChipErase(void)
{
	u32 timeout;
	int ret = 0;
	printk("iCatch: SST Chip Erasing...\n");

	ret = I2C_SPIFlashWrEnable();
	if (ret) {
		printk("iCatch: SST Chip Erase fail, ret= 0x%x\n", ret);
		return ret;
	}

	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e3,SPI_CMD_WRT_STS_EN); /*Write Status register command*/
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);

	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e3,SPI_CMD_WRT_STS);
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e3,0x02);
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);

	I2C_SPIFlashWrEnable();

	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e3,SPI_CMD_ERASE_ALL);
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);

	timeout = 0xffffffff;
	I2C_SPITimeOutWait(0x01, &timeout);
	//msleep(500);
	printk("iCatch: SST Chip Erased\n");
	return 0;
}

void writeUpdateProgresstoFile(int page_left, int total_page_num)
{
	struct file *fp_progress = NULL;
	mm_segment_t old_fs;
	loff_t offset = 0;
	char str_progress[4];
	int percentage = 0;

       if(g_ASUS_hwID < A91_SR1){
              percentage = 100 * (total_page_num - page_left + 1)/total_page_num;
        	if(page_left % 32 == 1){
        		printk("%s: page:0x%x; percentage= %d;\n", __FUNCTION__, page_left, percentage);
        		fp_progress = filp_open("/data/isp_fw_update_progress", O_RDWR | O_CREAT, S_IRUGO | S_IWUGO);
        		if ( IS_ERR_OR_NULL(fp_progress) ){
        			filp_close(fp_progress, NULL);
        			printk("%s: open %s fail\n", __FUNCTION__, "/data/isp_fw_update_progress");
        		}
        		old_fs = get_fs();
        		set_fs(KERNEL_DS);
        		offset = 0;
        		if (fp_progress->f_op != NULL && fp_progress->f_op->write != NULL){
        			sprintf(str_progress, "%d\n", percentage);
        			fp_progress->f_op->write(fp_progress,
        				str_progress,
        				strlen(str_progress),
        				&offset);
        		}else
        			pr_err("%s: f_op might be null\n", __FUNCTION__);
        		set_fs(old_fs);
        		filp_close(fp_progress, NULL);
        	}            
       }else{
              percentage = 100;
		fp_progress = filp_open("/data/isp_fw_update_progress", O_RDWR | O_CREAT, S_IRUGO | S_IWUGO);
		if ( IS_ERR_OR_NULL(fp_progress) ){
			filp_close(fp_progress, NULL);
			printk("%s: open %s fail\n", __FUNCTION__, "/data/isp_fw_update_progress");
		}
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		offset = 0;
		if (fp_progress->f_op != NULL && fp_progress->f_op->write != NULL){
			sprintf(str_progress, "%d\n", percentage);
			fp_progress->f_op->write(fp_progress,
				str_progress,
				strlen(str_progress),
				&offset);
		}else
			pr_err("%s: f_op might be null\n", __FUNCTION__);
		set_fs(old_fs);
		filp_close(fp_progress, NULL);            
       }
}

void writeUpdateResultFile(void)
{
	struct file *fp_result = NULL;
	mm_segment_t old_fs;
	loff_t offset = 0;
	char str_result[4];

	printk("%s: fw_update_status= %d;\n", __FUNCTION__, fw_update_status);
    
	fp_result = filp_open("/data/isp_fw_update_result", O_RDWR | O_CREAT, S_IRUGO | S_IWUGO);
	if ( IS_ERR_OR_NULL(fp_result) ){
		filp_close(fp_result, NULL);
		printk("%s: open %s fail\n", __FUNCTION__, "/data/isp_fw_update_result");
	}
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	offset = 0;
	if (fp_result->f_op != NULL && fp_result->f_op->write != NULL){
		sprintf(str_result, "%d\n", fw_update_status);
		fp_result->f_op->write(fp_result,
			str_result,
			strlen(str_result),
			&offset);
	}else
		pr_err("%s: f_op might be null\n", __FUNCTION__);
		set_fs(old_fs);
		filp_close(fp_result, NULL);

}

#if 1
u32 I2C_SPIFlashWrite(
	u32 addr,
	u32 pages,
	u8 *pbuf
)
{
	u32 i, err = 0;
	u32 pageSize = 0x100;

	addr = addr * pageSize;

	printk("iCatch: ST type writing...\n");
	g_total_page_count = (int)pages;

	while( pages ) {
		g_page_count = (int)pages;
		writeUpdateProgresstoFile(g_page_count, g_total_page_count);

		//I2C_SPIFlashWrEnable();
		//hsI2CDataWrite(0x40e7,0x00);
		sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
		//I2C_SPIFlashPortWrite(SPI_CMD_BYTE_PROG); /* Write one byte command*/
		sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e3,SPI_CMD_BYTE_PROG);
		// I2C_SPIFlashPortWrite((u8)(addr >> 16)); /* Send 3 bytes address*/
		sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e3,(u8)(addr >> 16));
		// I2C_SPIFlashPortWrite((u8)(addr >> 8));
		sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e3,(u8)(addr >> 8));
		// I2C_SPIFlashPortWrite((u8)(addr));
		sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e3,(u8)(addr));

		for (i = 0; i < pageSize ; i++) {
			// How about "Early return" here?
			// I2C_SPIFlashPortWrite(*pbuf);
			sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e3,(u8)(*pbuf));
			pbuf++;
		}
		// hsI2CDataWrite(0x40e7,0x01);
		sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);
		addr += pageSize;
		pages --;
		// tmrUsWait(2000);
		udelay(2000);
	}
	printk("iCatch: ST type writing Done\n");
	return err;
}
#endif

void I2C_SPISstStatusWrite(u8 dat)
{
	u32 timeout, poll;

	I2C_SPIFlashWrEnable();

	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e3,SPI_CMD_WRT_STS_EN);
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);

	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e3,SPI_CMD_WRT_STS);
	printk("%s: dat=%d\n", __FUNCTION__, dat);
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e3,dat);
	printk("%s: dat=%d; Done.\n", __FUNCTION__, dat);
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);

	poll = 0x01;
#if 0
	if( spiDev.bus != SPI_1BIT_MODE ) {/* 1 bit mode */
		poll = 0x80;
	} else {
		poll = 0x01;
	}
#endif
    timeout = 100000;
    I2C_SPITimeOutWait(poll, &timeout);
    //msleep(500);
    return;
}

u32 I2C_SPISstFlashWrite(
	u32 addr,
	u32 pages,
	u8 *pbuf
)
{
	u32 i, err = 0;
	u32 pageSize = 0x100;
	u32 timeout = 100000;

	addr = addr * pageSize;

	printk("iCatch: SST type writing...\n");
	I2C_SPISstStatusWrite(0x40);

	g_total_page_count = (int)pages;

	while( pages ) {
		g_page_count = (int)pages;
		writeUpdateProgresstoFile(g_page_count, g_total_page_count);

		I2C_SPIFlashWrEnable();
		sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
		/* Write one byte command*/
		sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e3,SPI_CMD_BYTE_PROG_AAI);
		/* Send 3 bytes address*/
		sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e3,(u8)(addr >> 16));
		sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e3,(u8)(addr >> 8));
		sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e3,(u8)(addr));
        
		sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e3,(u8)(*pbuf));
		pbuf++;
		sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e3,(u8)(*pbuf));
		pbuf++;
		sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);
		timeout = 100000;
		I2C_SPITimeOutWait(0x01,&timeout);

		for (i = 2; i < pageSize ; i = i+2) {
			sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
			sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e3,SPI_CMD_BYTE_PROG_AAI);
			sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e3,(u8)(*pbuf));
			pbuf++;
			sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e3,(u8)(*pbuf));
			pbuf++;
			sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);
			timeout = 100000;
			I2C_SPITimeOutWait(0x01,&timeout);
		}

		sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
		sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e3,SPI_CMD_WRT_DIS);
		sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);

		addr += pageSize;
		pages --;

		sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
		sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e3,SPI_CMD_WRT_DIS);
		sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);
	}
	printk("iCatch: SST type writing Done.\n");
	return err;
}

/* get_one_page_from_i7002a():
 *   Dump the ISP page whose index is "which_page" to "pagebuf".
 *   mclk, power & rst are requisite for getting correct page data.
 */
void get_one_page_from_i7002a(int which_page, u8* pagebuf)
{
	int i = 0;
	int ret = 0;
       u32 id;

       sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x1011,0x01); //Stop ISP working
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x70c4,0x00);
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x70c5,0x00);

	ret = I2C_SPIInit();
	if (ret) {
		g_camera_status = 0;	//ASUS_BSP Stimber "Add ATD proc interface"
		printk("%s: get nothing. ret= %d", __FUNCTION__, ret);
		return;
	}

	id = I2C_SPIFlashReadId();
    if(id==0) {
		printk("read id failed\n");
        g_camera_status = 0;	//ASUS_BSP Stimber "Add ATD proc interface"
	}

	I2C_SPIFlashRead(which_page, 1, pagebuf);

#if 1 // dump to kmsg ?
	printk("page#%d:\n", which_page);
	for(i=0; i < 0x100; i++) {
		if(i%16 == 0)
			printk("[%04x]", i);
		printk("%02X ",  pagebuf[i]);
		if(i%16 == 15)
			printk("\n");
	}
#endif
}

unsigned int get_fw_version_in_bin(char* binfile_path)
{
	u8 bin_file_header[BIN_FILE_HEADER_SIZE];
	unsigned int version_num_in_bin = 0xFFFFFF;
       //struct stat statbuf;

	int fp;
	mm_segment_t old_fs;
	unsigned long bootbin_size = 0;
	pr_info("version_num_in_isp : %d\n", version_num_in_isp);
	if (version_num_in_isp != 0xffffff)
	return version_num_in_isp;
	/* Calculate BOOT.BIN file size. */
    	old_fs = get_fs();
	set_fs(KERNEL_DS);
	fp = sys_open(binfile_path, O_RDONLY, 0);
	if (fp > 0){
		pr_info("filp_open success fp:%d\n", fp);
              //sys_newfstat(fp, &statbuf);
              //bootbin_size = statbuf.st_size;
              if (true == g_spi_is_support_1M) {
                bootbin_size = 1024*1024;
              }else{
                bootbin_size = 512*1024;
              }
              sys_lseek(fp, bootbin_size - BIN_FILE_HEADER_SIZE, 0);
              sys_read(fp, bin_file_header, BIN_FILE_HEADER_SIZE);
		sys_close(fp);
	} else{
		pr_err("iCatch \"%s\" open error\n", binfile_path);
		return version_num_in_bin;
	}
    	set_fs(old_fs);
	version_num_in_bin = (bin_file_header[30] << 16) | (bin_file_header[29] << 8) | bin_file_header[28];
       return version_num_in_bin;
}

unsigned int get_fw_version_in_isp(void)
{
	u8 tmp_page[0x100];
	unsigned int vn = 0xABCDEF;
	int i = 0;
	int retry = 3;
	bool b_ok;

	for (i = 0; i < retry; i++) {
		int j =0;
		b_ok = true;

              if(true == g_spi_is_support_1M){
		    /* The fw veriosn is in the page with the index, 4095.*/
                  get_one_page_from_i7002a(4095, tmp_page);
              }else{
		    /* The fw veriosn is in the page with the index, 2047.*/
		    get_one_page_from_i7002a(2047, tmp_page);
              }

		/* The header format looks like:
		 * FF FF FF FF FF FF FF FF XX XX XX XX XX XX XX
		 * FF FF FF FF FF XX XX XX XX XX XX XX XX XX XX
		 */
		for (j = 0; j < 8; j++) {
			if (tmp_page[0x100 - BIN_FILE_HEADER_SIZE + j] != 0xFF) {
				printk("%s: tmp_page[0x%X]= %02X\n", __FUNCTION__,
					0x100 - BIN_FILE_HEADER_SIZE + j,
					tmp_page[0x100 - BIN_FILE_HEADER_SIZE + j]);
				b_ok = false;
				break;
			}
		}
		if (b_ok == true)
			break;
		else {
			printk("%s: wrong page data? Try again (%d).\n", __FUNCTION__, i);
			msleep(10);
		}
	}

	if (b_ok == true)
		vn = (tmp_page[0xFF - 1] <<16) | (tmp_page[0xFF - 2] << 8) | tmp_page[0xFF -3];

       version_num_in_isp = vn;
	printk("%s: vn=0x%X\n", __FUNCTION__, vn);
	return vn;
}

//ASUS_BSP +++ LiJen "[A68][ISP][NA][Fix] vote don't enter low power mode when ISP flash"
#if 0
#define MSM_V4L2_SWFI_LATENCY 3
static void iCatch_pm_qos_update_latency(bool vote)
{
	if (vote){
		pm_qos_add_request(&pm_qos_req_list,
				PM_QOS_CPU_DMA_LATENCY,
				PM_QOS_DEFAULT_VALUE);
		pm_qos_update_request(&pm_qos_req_list,
				MSM_V4L2_SWFI_LATENCY);
       }
	else{
		pm_qos_update_request(&pm_qos_req_list,
				PM_QOS_DEFAULT_VALUE);
		pm_qos_remove_request(&pm_qos_req_list);
       }
}
#endif
//ASUS_BSP --- LiJen "[A68][ISP][NA][Fix] vote don't enter low power mode when ISP flash"

static int sensor_write_reg_bytes(struct i2c_client *client, u16 addr, unsigned char* val, u32 bytes)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[bytes+2];
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;
       
	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);

       memcpy((data+2), val, bytes);

	msg.addr = (client->addr >> 1);
	msg.flags = 0;
	msg.len = bytes+2;
	msg.buf = data;

	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;
		retry++;
//		pr_err("yuv_sensor : i2c transfer failed, retrying %x %llu\n", addr, val);
		pr_err("yuv_sensor : i2c transfer failed, count %x \n", msg.addr);
	} while (retry <= 3);

	return err;
}

int sensor_read_reg_bytes(struct i2c_client *client, u16 addr, unsigned char* val, u32 bytes)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[bytes+2];
	if (!client->adapter)
		return -ENODEV;
       
	msg[0].addr = (client->addr >> 1);
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);

	msg[1].addr = (client->addr >> 1);
	msg[1].flags = I2C_M_RD;

	msg[1].len = bytes;
	msg[1].buf = data + 2;

	err = i2c_transfer(client->adapter, msg, 2);
       
	if (err != 2)
		return -EINVAL;

	memcpy(val, data+2, bytes);

	return 0;
}

unsigned int get_fw_version_in_isp_fromISP(void)
{
    u32 read_bytes=3;
    unsigned char read_data[read_bytes];
    unsigned int vn = 0xABCDEF;
    int retry = 0;

    //msleep(30); // wait ISP load code
    do{
        msleep(10);
        sensor_read_reg_bytes(mt9m114_s_ctrl.sensor_i2c_client->client, 0x72ca, read_data, read_bytes);
        vn = (read_data[2] << 16)|(read_data[1] << 8)|read_data[0];        
        retry ++;
        pr_info("%s  wrong version data? Try again (0x%x)(0x%x)(0x%x)\n",__func__,read_data[2],read_data[1],read_data[0]);
    }while(vn==0 && retry<20);
    
    version_num_in_isp = vn;
    printk("%s: vn=0x%X\n", __FUNCTION__, vn);
    return vn;
}

u32 I2C_7002DmemWr(
	u32 bankNum,
	u32 byteNum,
	u8* pbuf
)
{
	u32 i, j, bank;
       const u32 bytes = ISP_FLASH_I2C_WRITE_BYTES;
       unsigned char tmp[bytes];
       int rc=0;

	bank = 0x40+bankNum;
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x10A6,bank);

	for(i=0;i<byteNum;i+=bytes) {
		//seqI2CDataWrite((0x1800+i),(pbuf+i)); /* sequentially write DMEM */
              memset(&tmp, 0, sizeof(tmp));
        
               for(j=0;j<bytes;j++){
                    tmp[j]=(*(pbuf + i + j));
               }        

              rc=sensor_write_reg_bytes(mt9m114_s_ctrl.sensor_i2c_client->client, (0x1800+i), tmp, bytes);
              if(rc < 0){
                    pr_err("%s sensor_write_reg_bytes fail rc=%d\n",__func__,rc);
                    return rc;
              }
	}

	bank = 0x40 + ((bankNum+1)%2);
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x10A6,bank);

       return 0;
}

u32 I2C_SPIFlashWrite_DMA(
	u32 addr,
	u32 pages,
	u8 *pbuf
)
{
    u32 i, err = 0;
    u32 pageSize = 0x100, size;
    u32 rsvSec1, rsvSec2;
    u32 dmemBank = 0;
    u32 chk1=0;
    u16 chk2, temp = 0;
    int rc=0;

    rsvSec1 = pages*pageSize - 0x7000;
    rsvSec2 = pages*pageSize - 0x1000;
    addr = addr * pageSize;
    
    /* Set DMA bytecnt as 256-1 */
    sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x4170,0xff);
    sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x4171,0x00);
    sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x4172,0x00);    

    /* Set DMA bank & DMA start address */
    sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x1084,0x01);
    sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x1080,0x00);
    sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x1081,0x00);
    sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x1082,0x00);    

    /* enable DMA checksum and reset checksum */
    sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x4280,0x01);
    sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x4284,0x00);
    sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x4285,0x00);
    sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x4164,0x00);    

    size = pages * pageSize;
    for(i=0;i<size;i++)
    {
	if((i>=rsvSec2) || (i <rsvSec1))
	{
		chk1 += *(pbuf+i);
	}
	if(chk1>=0x10000)
	{
		chk1 -= 0x10000;
	}
   }

    g_total_page_count = (int)pages;
    
    while( pages ) {
	g_page_count = (int)pages;
	writeUpdateProgresstoFile(g_page_count, g_total_page_count);        
    
   	 if((pages%0x40)==0)
   	 {
		//printk("page:0x%x\n",pages);
    	}
   	 if((addr>=rsvSec1) && (addr <rsvSec2))
   	 {
		addr += 0x1000;
               		pbuf += 0x1000;
		pages -= 0x10;
		continue;
    	}
    	if((pages==1))
   	 {
		for (i = 0; i < pageSize ; i++) {
            		printk("%2x ",*(pbuf+i));
		if((i%0x10)==0x0f) printk("\n");
    		}
    	}

    	dmemBank = pages % 2;
    	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x1081,dmemBank*0x20);
    	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x1084,(1<<dmemBank));
    	rc=I2C_7002DmemWr(dmemBank,pageSize,pbuf);		
       if(rc<0){
            pr_err("%s fail\n",__func__);
            return rc;
       }		
     	I2C_SPIFlashWrEnable();
       sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
     	I2C_SPIFlashPortWrite(SPI_CMD_BYTE_PROG);               /* Write one byte command*/
     	I2C_SPIFlashPortWrite((u8)(addr >> 16));               /* Send 3 bytes address*/
     	I2C_SPIFlashPortWrite((u8)(addr >> 8));
     	I2C_SPIFlashPortWrite((u8)(addr));

    	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x4160,0x01);
    	udelay(100);/* wait for DMA done */
    	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);
    	pbuf += pageSize;
    	addr += pageSize;
    	pages --;
    }
	
    udelay(500);/* wait for DMA done */

    sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x4285, &temp);
    sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x4284, &chk2);  
    chk2 = chk2 | (temp<<8);
    pr_info("checksum: 0x%x 0x%x\n",chk1,chk2);

    return err;
}

u32 I2C_SPISectorErase(
	u32 address,
	u32 stFlag
)
{
	u32 timeout;
	pr_info("addr:0x%x\n",address);
	if(!stFlag)
	{
            I2C_SPIFlashWrEnable();
	
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
            I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS_EN);				/*Write Status register command*/
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);
	}
	
       sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
	I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS);				/*Write Status register command*/
	I2C_SPIFlashPortWrite(0x02);
  	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);

	I2C_SPIFlashWrEnable();

  	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
	I2C_SPIFlashPortWrite(SPI_CMD_SECTOR_ERASE);
	I2C_SPIFlashPortWrite(address >> 16);	/* A23~A16 */
	I2C_SPIFlashPortWrite(address >> 8);		/* A15~A08 */
	I2C_SPIFlashPortWrite(address);			/* A07~A00 */
  	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);
	
	//timeout = 5;
	timeout = 5000000;
	I2C_SPITimeOutWait(0x01, &timeout);
	
	return 0;
}

u32 I2C_SPI32KBBlockErase(
	u32 address,
	u32 stFlag
)
{
	u32 timeout;
	pr_info("addr:0x%x\n",address);
	if(!stFlag)
	{
           I2C_SPIFlashWrEnable();

           sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
	    I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS_EN);				/*Write Status register command*/
    	    sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);
	}

  	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
	I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS);				/*Write Status register command*/
	I2C_SPIFlashPortWrite(0x02);
  	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);

	I2C_SPIFlashWrEnable();

  	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
	I2C_SPIFlashPortWrite(SPI_CMD_32KB_BLOCK_ERASE);
	I2C_SPIFlashPortWrite(address >> 16);	/* A23~A16 */
	I2C_SPIFlashPortWrite(address >> 8);		/* A15~A08 */
	I2C_SPIFlashPortWrite(address);			/* A07~A00 */
  	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);
	
	timeout = 5000000;
	I2C_SPITimeOutWait(0x01, &timeout);
	printk("I2C_SPI32KBBlockErase ok\n");
	return 0;
}

u32 I2C_SPI64KBBlockErase(
	u32 address,
	u32 stFlag
)
{
	u32 timeout;
	pr_info("addr:0x%x\n",address);
	if(!stFlag)
	{
            I2C_SPIFlashWrEnable();
	
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
            I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS_EN);				/*Write Status register command*/
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);
	}

  	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
	I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS);				/*Write Status register command*/
	I2C_SPIFlashPortWrite(0x02);
  	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);

	I2C_SPIFlashWrEnable();

  	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x00);
	I2C_SPIFlashPortWrite(SPI_CMD_64KB_BLOCK_ERASE);
	I2C_SPIFlashPortWrite(address >> 16);	/* A23~A16 */
	I2C_SPIFlashPortWrite(address >> 8);		/* A15~A08 */
	I2C_SPIFlashPortWrite(address);			/* A07~A00 */
  	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e7,0x01);
	
	timeout = 5000000;
	I2C_SPITimeOutWait(0x01, &timeout);
	
	return 0;
}

void
BB_EraseSPIFlash(
	u32 type,
	u32 spiSize
)
{
	u8 typeFlag=0;
	u32 i, temp1;
	if( type == 2 )/* SST */
	{
		typeFlag = 0;
	}
	else if( type == 1 || type == 3 )/* ST */
	{
		typeFlag = 1;
	}else{
            pr_err("%s type(%d) is not support\n",__func__,type);
            return;
       }
    
	/*printf("spiSize:0x%x\n",spiSize);*/
	if(spiSize == (512*1024))
	{
		/* skip 0x7B000 ~ 0x7EFF, to keep calibration data */
		temp1 = (spiSize / 0x10000)-1;
		for(i=0;i<temp1;i++)
		{
			I2C_SPI64KBBlockErase(i*0x10000,typeFlag);
		}
		I2C_SPI32KBBlockErase(temp1*0x10000,typeFlag);
		temp1 = temp1*0x10000 + 0x8000;
		for(i=temp1;i<spiSize-0x5000;i+=0x1000)
		{
			I2C_SPISectorErase(i,typeFlag);
		}
		I2C_SPISectorErase(spiSize-0x1000,typeFlag);
	}
	else if(spiSize == (1024*1024))
	{
		/* only erase 256*3KB */
		temp1 = ((spiSize*3/4) / 0x10000);
		for(i=0;i<temp1;i++)
		{
			I2C_SPI64KBBlockErase(i*0x10000,typeFlag);
		}
		I2C_SPI32KBBlockErase((temp1+1),typeFlag);
		I2C_SPISectorErase(spiSize-0x1000,typeFlag);
	}
}

void
BB_WrSPIFlash(char* binfile_path)
{
	u32 id, type;
	u32 pages;

	u8 *pbootBuf;
	u8 bin_file_header[BIN_FILE_HEADER_SIZE];
	u8 checksum1_in_bin[2], checksum2_in_bin[2];
	u8 checksum1_in_isp[2], checksum2_in_isp[2];
	unsigned int version_num_in_bin = 0xFFFFFF;
	int firmware2_offset;
	u8 tmp_page[0x100];

	struct file *fp = NULL;
	mm_segment_t old_fs;
	struct inode *inode;
	int bootbin_size = 0;
	int i, ret = 0;

       //iCatch_pm_qos_update_latency(true); //ASUS_BSP LiJen "[A68][ISP][NA][Fix] vote don't enter low power mode when ISP flash"
	fw_update_status = ICATCH_FW_IS_BURNING;

	/* Calculate BOOT.BIN file size. */
	fp = filp_open(binfile_path, O_RDONLY, 0);

	if ( !IS_ERR_OR_NULL(fp) ){
		pr_info("filp_open success fp:%p\n", fp);
		inode = fp->f_dentry->d_inode;
		bootbin_size = inode->i_size;
		printk("%s: fp->f_dentry->d_inode->i_size=%d\n", __FUNCTION__, bootbin_size);
		//pbootBuf = kmalloc(bootbin_size, GFP_KERNEL);
		pbootBuf = vmalloc(bootbin_size);
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		if(fp->f_op != NULL && fp->f_op->read != NULL){
			int byte_count= 0;
			printk("Start to read %s\n", binfile_path);

			byte_count = fp->f_op->read(fp, pbootBuf, bootbin_size, &fp->f_pos);

			if (byte_count <= 0) {
				printk("iCatch: EOF or error. last byte_count= %d;\n", byte_count);
				//kfree(pbootBuf);       
				vfree(pbootBuf);
				fw_update_status = ICATCH_FW_UPDATE_FAILED;
				goto end;
			} else
				printk("iCatch: BIN file size= %d bytes\n", bootbin_size);

#if 0
			for(i=0; i < bootbin_size; i++) {
				printk("%c", pbootBuf[i]);
			}
			printk("\n");
#endif
		}
		set_fs(old_fs);
		filp_close(fp, NULL);
	} else if(PTR_ERR(fp) == -ENOENT) {
		pr_err("iCatch \"%s\" not found error\n", binfile_path);
		fw_update_status = ICATCH_FW_UPDATE_FAILED;
		goto end;
	} else{
		pr_err("iCatch \"%s\" open error\n", binfile_path);
		fw_update_status = ICATCH_FW_UPDATE_FAILED;
		goto end;
	}

	for (i=0; i < BIN_FILE_HEADER_SIZE; i++)
	{
		bin_file_header[i] = pbootBuf[bootbin_size - BIN_FILE_HEADER_SIZE + i];
		printk("%s: bin_file_header[%d]= 0x%x\n", __FUNCTION__, i,bin_file_header[i]);
	}
	version_num_in_bin = (bin_file_header[30] << 16) | (bin_file_header[29] << 8) | bin_file_header[28];

	/* Get the checksum in bin file.
	 *   firmware2_offset
	 *     = fw1 header size
	 *     + fw1 DMEM FICDMEM size
	 *     + fw1 IMEM size
	 */
	memcpy(checksum1_in_bin, pbootBuf + 10, 2);

	firmware2_offset = 16 +
		((pbootBuf[3] << 24) | (pbootBuf[2] << 16) | (pbootBuf[1] << 8) | pbootBuf[0]) +
		((pbootBuf[7] << 24) | (pbootBuf[6] << 16) | (pbootBuf[5] << 8) | pbootBuf[4]);
	memcpy(checksum2_in_bin, pbootBuf + firmware2_offset + 10, 2);

	printk("%s: checksum in bin:%02X %02X; %02X %02X\n", __FUNCTION__,
		checksum1_in_bin[0],checksum1_in_bin[1],checksum2_in_bin[0], checksum2_in_bin[1]);

	ret = I2C_SPIInit();
	if (ret) {
		printk("%s: SPI init fail. ret= 0x%x", __FUNCTION__, ret);
		//kfree(pbootBuf);
		vfree(pbootBuf);
		fw_update_status = ICATCH_FW_UPDATE_FAILED;
		goto end;
	}

	id = I2C_SPIFlashReadId();

	if(id==0) {
		printk("read id failed\n");
		//kfree(pbootBuf);
		vfree(pbootBuf);
		fw_update_status = ICATCH_FW_UPDATE_FAILED;
		goto end;
	}

	type = BB_SerialFlashTypeCheck(id);
	if(type == 0) {
		printk("BB_SerialFlashTypeCheck(%d) failed\n", id);
		//kfree(pbootBuf);
		vfree(pbootBuf);
		fw_update_status = ICATCH_FW_UPDATE_FAILED;
		goto end;
	}

	pages = bootbin_size/0x100;

	printk("%s: pages:0x%x\n", __FUNCTION__, pages);

	BB_EraseSPIFlash(type,bootbin_size);

	/* Writing Flash here */
	if( type == 2 ) {
		flash_type = ICATCH_FLASH_TYPE_SST;
		printk("SST operation\n");
		#if 0
		ret = I2C_SPISstChipErase();
		if(ret) {
			printk("%s: SST erase fail.\n", __FUNCTION__);
			//kfree(pbootBuf);
			vfree(pbootBuf);
			fw_update_status = ICATCH_FW_UPDATE_FAILED;
			goto end;
		}
		#endif
		I2C_SPISstFlashWrite(0, pages, pbootBuf);
	} else if( type == 1 || type == 3 ) {
		flash_type = ICATCH_FLASH_TYPE_ST;
		printk("ST operation\n");
		#if 0
		ret = I2C_SPIStChipErase();
		if(ret) {
			printk("%s: ST erase fail.\n", __FUNCTION__);
                     //kfree(pbootBuf);
                     vfree(pbootBuf);
			fw_update_status = ICATCH_FW_UPDATE_FAILED;
			goto end;
		}
		#endif
		//I2C_SPIFlashWrite(0, pages, pbootBuf);
		ret=I2C_SPIFlashWrite_DMA(0, pages, pbootBuf);
                if(ret<0){
                    printk("I2C_SPIFlashWrite_DMA fail\n");
                    fw_update_status = ICATCH_FW_UPDATE_FAILED;
               }
	} else {
		printk("type unknown: %d; Won't update iCatch FW.\n", type);
		fw_update_status = ICATCH_FW_UPDATE_FAILED;
		//kfree(pbootBuf);
		vfree(pbootBuf);
		goto end;
	}
	//kfree(pbootBuf);
	vfree(pbootBuf);

	/* Check the update reult. */
	/* Compare Check sum here */
	get_one_page_from_i7002a(0, tmp_page);
	memcpy(checksum1_in_isp, tmp_page + 10, 2);

	if (memcmp(checksum1_in_isp, checksum1_in_bin, 2) == 0) {
		/* checksum1 PASS */
		firmware2_offset = 16 +
			((tmp_page[3] << 24) | (tmp_page[2] << 16) | (tmp_page[1] << 8) | tmp_page[0]) +
			((tmp_page[7] << 24) | (tmp_page[6] << 16) | (tmp_page[5] << 8) | tmp_page[4]);

		get_one_page_from_i7002a(firmware2_offset >> 8, tmp_page);
		memcpy(checksum2_in_isp, tmp_page + 10, 2);

		if (memcmp(checksum2_in_isp, checksum2_in_bin, 2) == 0) {
			/* checksum2 PASS */
			version_num_in_isp = get_fw_version_in_isp();
			if (version_num_in_isp == version_num_in_bin) {
				/* version number PASS */
				fw_update_status = ICATCH_FW_UPDATE_SUCCESS;
				printk("%s: ICATCH FW UPDATE SUCCESS.\n", __FUNCTION__);
			} else {
				/* version number FAIL */
				fw_update_status = ICATCH_FW_UPDATE_FAILED;
				printk("%s: check version FAIL: ISP(0x%06X) != BIN(0x%06X)\n", __FUNCTION__, version_num_in_isp, version_num_in_bin);
				version_num_in_isp = 0xABCDEF;
			}
		} else {
			/* checksum2 FAIL */
			fw_update_status = ICATCH_FW_UPDATE_FAILED;
			printk("%s: checksum2 FAIL: ISP(%02X %02X) != BIN(%02X %02X)\n",
				__FUNCTION__, checksum2_in_isp[0], checksum2_in_isp[1],
				checksum2_in_bin[0], checksum2_in_bin[1]);
			version_num_in_isp = 0xABCDEF;
		}
	} else {
		/* checksum1 FAIL */
		fw_update_status = ICATCH_FW_UPDATE_FAILED;
		printk("%s: checksum1 FAIL: ISP(%02X %02X) != BIN(%02X %02X)\n",
			__FUNCTION__, checksum1_in_isp[0], checksum1_in_isp[1],
			checksum1_in_bin[0], checksum1_in_bin[1]);
		version_num_in_isp = 0xABCDEF;
	}

end:
       writeUpdateResultFile();
       //iCatch_pm_qos_update_latency(false); //ASUS_BSP LiJen "[A68][ISP][NA][Fix] vote don't enter low power mode when ISP flash"
}

#ifdef WITH_INT		
irqreturn_t iCatch_irq_handler(int irq, void *data)
{  
        pr_info("isp_int\n");
        complete(&g_iCatch_comp);
        //disable_irq(mt9m114_s_ctrl.sensordata->sensor_platform_info->isp_int);
	return IRQ_HANDLED;
}
#endif 

void iCatch_debug(void)
{
    u16 readval, i;
    u32 read_bytes=128;
    unsigned char read_data[read_bytes];
       
    pr_info("%s +++\n",__func__);
    sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x007a, &readval);
    pr_info("%s reg(0x007a)=0x%x\n",__func__,readval);
    sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x007b, &readval);
    pr_info("%s reg(0x007b)=0x%x\n",__func__,readval);   
    sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9080, &readval);
    pr_info("%s reg(0x9080)=0x%x\n",__func__,readval);   
    sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x905c, &readval);
    pr_info("%s reg(0x905c)=0x%x\n",__func__,readval);   
    sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7070, &readval);
    pr_info("%s reg(0x7070)=0x%x\n",__func__,readval);   
    sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7072, &readval);
    pr_info("%s reg(0x7072)=0x%x\n",__func__,readval);   
    sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7073, &readval);
    pr_info("%s reg(0x7073)=0x%x\n",__func__,readval);   
    sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7074, &readval);
    pr_info("%s reg(0x7074)=0x%x\n",__func__,readval);   
    sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7075, &readval);
    pr_info("%s reg(0x7075)=0x%x\n",__func__,readval);   
    sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x90cc, &readval);
    pr_info("%s reg(0x90cc)=0x%x\n",__func__,readval);   
    sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x90cd, &readval);
    pr_info("%s reg(0x90cd)=0x%x\n",__func__,readval);   
    sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x90ce, &readval);
    pr_info("%s reg(0x90ce)=0x%x\n",__func__,readval);   
    sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x90cf, &readval);
    pr_info("%s reg(0x90cf)=0x%x\n",__func__,readval);   
    sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x72f8, &readval);
    pr_info("%s reg(0x72f8)=0x%x\n",__func__,readval); 

    //Get ISP Status
    sensor_read_reg_bytes(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7200, read_data, read_bytes);
    for(i=0;i<read_bytes;i++){
        pr_info("%s reg(0x72%2x)=0x%x\n",__func__,i, read_data[i]);     
    }
    
    pr_info("%s ---\n",__func__);
}

void wait_for_next_frame(void){
#ifdef WITH_INT	
        u32 timeout_count = 1;

        //enable_irq(mt9m114_s_ctrl.sensordata->sensor_platform_info->isp_int);
        //INIT_COMPLETION(g_iCatch_comp);
        timeout_count = wait_for_completion_timeout(&g_iCatch_comp, 4*HZ);
        if (!timeout_count) {
            pr_err("%s: interrupt timedout\n", __func__);
            iCatch_debug();
        } else {
            pr_debug("%s interrupt done\n",__func__);
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x72f8, 0x04);
        }             
#else
        u16 testval, i;

        for (i=0;i<200;i++)
        {
            sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x72f8, &testval);
            printk("testval=0x%X, i=%d\n",testval,i);
            if (testval & 0x04) {
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x72f8, 0x04);
            sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x72f8, &testval);
            printk("Clear testval=0x%X, i=%d\n",testval,i);
            break;
            }
            //printk("testval=0x%X, i=%d\n",testval,i);
            msleep(iCatch7002a_init_delay);
        }
        
        if(i==200){
            iCatch_debug();
        }
#endif    
}

void wait_for_AE_ready(void){
    u16 testval, i;
             
    iCatch_first_open = false;
                
    for (i=0;i<200;i++)
    {
        sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x72c3, &testval);
        printk("testval=0x%X, i=%d\n",testval,i);
        if (testval & 0x01) {
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x72f8, 0x04);
            sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x72f8, &testval);
            printk("Clear testval=0x%X, i=%d\n",testval,i);
            break;
        }
        //printk("testval=0x%X, i=%d\n",testval,i);
        msleep(iCatch7002a_init_delay);
    }
    
    if(i==200){
        iCatch_debug();
    }    
}

void wait_for_AWB_ready(void){
    u16 testval, i;
             
    iCatch_first_open = false;
                
    for (i=0;i<15;i++)
    {
        sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x72c3, &testval);
        printk("testval=0x%X, i=%d\n",testval,i);
        if (testval & 0x02) {
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x72f8, 0x04);
            sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x72f8, &testval);
            printk("Clear testval=0x%X, i=%d\n",testval,i);
            break;
        }
        //printk("testval=0x%X, i=%d\n",testval,i);
        msleep(20);
    }
}

void enable_isp_interrupt(void){
    int rc = 0;
        
    //Mask ISP interrupt
    sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x72fc, 0x04);
    sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x72f8, 0xff);
    //Enable ISP interrupt 
    init_completion(&g_iCatch_comp);
    INIT_COMPLETION(g_iCatch_comp);    
    rc = request_irq(gpio_to_irq(ISP_INT_GPIO), iCatch_irq_handler,
	    IRQF_TRIGGER_RISING, "iCatch_int", 0);    
    if (rc < 0)
        pr_err("enable isp interrupt fail\n");  

    g_isISP_IRQ_eanble = true;
    //enable_irq(ISP_INT_GPIO);  
}

void disable_isp_interrupt(void){
    if(g_isISP_IRQ_eanble){
        disable_irq(gpio_to_irq(25));
        free_irq(gpio_to_irq(25), 0);
        g_isISP_IRQ_eanble = false;
    }
}

static void iCatch_torch_on_work(struct work_struct *work)
{
	pr_info("%s E\n",__func__);
	led_pmic_torch_enable(true,false);
	pr_info("%s X\n",__func__);
	return;
}

static void iCtach_flash_status_restore(void)
{
    switch(g_pmic_flash_status){
        case PMIC_FLASH_STATUS_FLASH_ON:
            led_pmic_flash_enable(true,true);
            break;        
        case PMIC_FLASH_STATUS_FLASH_OFF:
            break;
        default:
            pr_err("%s: not support status(%d)\n",__func__, g_pmic_flash_status);
            break;
     }
    
}

static void iCatch_torch_off_work(struct work_struct *work)
{
	pr_info("%s E\n",__func__);
       led_pmic_torch_enable(false,false);
       //led_pmic_flash_enable(true,true);
       iCtach_flash_status_restore();
	pr_info("%s X\n",__func__);
	return;
}

irqreturn_t iCatch_torch_irq_handler(int irq, void *data)
{  
        int irq_val;
        
        complete(&g_iCatch_torch_comp);
        irq_val = gpio_get_value(ISP_TORCH_INT_GPIO);

        if(g_pre_torch_irq_val == irq_val){
            pr_info("pre_irq_val(%d) == irq_val(%d)\n",g_pre_torch_irq_val,irq_val);
            return IRQ_HANDLED;
        }
        
        pr_info("isp_torch_int, irq_val(%d)\n",irq_val);
        if(irq_val == 1){
            queue_work(icatch_torch_workqueue, &icatch_torch_on_wq);
        }else if(irq_val == 0){
            queue_work(icatch_torch_workqueue, &icatch_torch_off_wq);
        }
        g_pre_torch_irq_val = irq_val;
            
        return IRQ_HANDLED;
}

void enable_isp_torch_interrupt(void){
    int rc = 0;

    //Enable ISP interrupt 
    init_completion(&g_iCatch_torch_comp);
    INIT_COMPLETION(g_iCatch_torch_comp);    
    rc = request_irq(gpio_to_irq(ISP_TORCH_INT_GPIO), iCatch_torch_irq_handler,
	    IRQF_TRIGGER_RISING |IRQF_TRIGGER_FALLING, "iCatch_torch_int", 0);    
    if (rc < 0)
        pr_err("enable isp torch interrupt fail\n");  

    //enable_irq(ISP_TORCH_INT_GPIO);  
}

void disable_isp_torch_interrupt(void){
    disable_irq(gpio_to_irq(ISP_TORCH_INT_GPIO));
    free_irq(gpio_to_irq(ISP_TORCH_INT_GPIO), 0);
}

void set_isp_calibration_table(int table){

    if(is_calibration_table_set == false){
        is_calibration_table_set = true;
        sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x717f, table);
    }
}

//ASUS_BSP +++ Stimber "Modify for preview/video frame rate"
void setFixFPS(int settingVal)
{
    int default_value = 0x00;
    
    if (settingVal == DEFAULT_SETTING) { //restore to default value
        if (g_LastFixFPS != DEFAULT_SETTING) { // Need to restore
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7124, default_value);    // clear to restore
            g_LastFixFPS = DEFAULT_SETTING;
        } //else: regValue==g_LastMiniISO==default value : do nothing
    } else {
        sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7124, settingVal);
        g_LastFixFPS = settingVal;
    }
}

void setMaxExp(int settingVal)
{
    int default_value = 0x00;
    
    if (settingVal == DEFAULT_SETTING) { //restore to default value
        if (g_LastMaxExp != DEFAULT_SETTING) { // Need to restore
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7125, default_value);    // clear to restore
            g_LastMaxExp = DEFAULT_SETTING;
        } //else: regValue==g_LastMiniISO==default value : do nothing
    } else {
        sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7125, settingVal);
        g_LastMaxExp = settingVal;
    }
}

void setMiniISO(int settingVal)
{
    int default_value = 0x00;
    int reg_val = 0x00;
    
    if (settingVal == DEFAULT_SETTING) { //restore to default value
        if (g_LastMiniISO != DEFAULT_SETTING) { // Need to restore
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7129, default_value);    // clear to restore
            g_LastMiniISO = DEFAULT_SETTING;
        } //else: regValue==g_LastMiniISO==default value : do nothing
    } else {
        switch(settingVal){
            case 50:
                reg_val = 0x01;
                break;
            case 100:
                reg_val = 0x02;
                break;
            case 200:
                reg_val = 0x03;
                break;
            case 400:
                reg_val = 0x04;
                break;
            case 800:
                reg_val = 0x05;
                break;
            case 1600:
                reg_val = 0x06;
                break;
            default:
                reg_val = 0x00;
                break;
        }
        sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7129, reg_val);
        g_LastMiniISO = settingVal;
    }
}

void setCaptureVideoMode(int settingVal)
{
    int default_value = 0x00;
    
    if (settingVal == DEFAULT_SETTING) { //restore to default value
        if (g_LastVideoMode != DEFAULT_SETTING) { // Need to restore
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7132, default_value);    // clear to restore
            g_LastVideoMode = DEFAULT_SETTING;
        } //else: regValue==g_LastVideoMode==default value : do nothing
    } else {
        sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7132, settingVal);
        g_LastVideoMode = settingVal;
    }
}

void setMaxFPS(int settingVal)
{
    int default_value = 0x00;
    
    if (settingVal == DEFAULT_SETTING) { //restore to default value
        if (g_LastMaxFPS != DEFAULT_SETTING) { // Need to restore
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7126, default_value);    // clear to restore
            g_LastMaxFPS = DEFAULT_SETTING;
        } //else: regValue==g_LastMaxFPS==default value : do nothing
    } else {
        sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7126, settingVal);
        g_LastMaxFPS = settingVal;
    }
}
//ASUS_BSP --- Stimber "Modify for preview/video frame rate"


int sensor_set_mode_main_camera(int  res, bool bSpeedUpPreview)
{
       bool l_edgeExifEnable = false;
//       const u32 streamon_bytes = 3; //reg(0x7120), reg(0x7121)
//       unsigned char data[streamon_bytes];     
       g_is1stFrame = true;//ASUS_BSP jim3_lin "Add log while kernel got 1st frame after streamon"
       g_frame_count = FRAME_COUNT_INIT;

       if(true == iCatch_first_open){                  
            //wait_for_AE_ready();
            enable_isp_interrupt();
            set_isp_calibration_table(0x01);
            iCatch_enable_autonight(true); //ASUS_BSP jim3_lin "Implement DIT postprocess-Mode2"
       }

       //ASUS_BSP +++ bill_chen "Implement image stabilization"
       if(g_is_eis_on)
          iCatch_enable_autonight(false);
       //ASUS_BSP --- bill_chen "Implement image stabilization"

       if((MSM_SENSOR_RES_FULL_SINGLE_CAPTURE == res||
            MSM_SENSOR_RES_FULL_BURST_CAPTURE== res ||
            MSM_SENSOR_RES_3M_BURST_CAPTURE == res ) && 
            (CAM_FLASH_MODE_ON ==g_flash_mode || CAM_FLASH_MODE_AUTO ==g_flash_mode)){
            led_pmic_flash_enable(true, true); //init pmic led flash
       }       
       //ASUS_BSP+++ jim3_lin "Avoid 1st preview frame after zsl capture is effected by flash"
       // Move to Mt9m114.c mt9m114_sensor_config():CFG_SET_STOP_STREAM
#if 0
       //burst capture abort
       if((MSM_SENSOR_RES_FULL_BURST_CAPTURE == g_pre_res
             || MSM_SENSOR_RES_10M_BURST_CAPTURE == g_pre_res
             || MSM_SENSOR_RES_3M_BURST_CAPTURE == g_pre_res
             || (MSM_SENSOR_RES_FULL_SINGLE_CAPTURE == g_pre_res && g_is_nr_on))
             && g_pre_res != res){
             pr_info("%s: Burst capture abort\n",__func__);
             sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7122, 0x01);
       }
#endif
       //ASUS_BSP--- jim3_lin "Avoid 1st preview frame after zsl capture is effected by flash"

       if(g_is_max_exp_on){
           setMaxExp(0);
			g_is_max_exp_on = false;
           pr_info("%s: Reset max exp to default\n", __func__);
       }
       
       switch(res){
            case MSM_SENSOR_RES_QTR:            //MODE_1
                pr_info("%s: MODE_1\n",__func__);
                if (bSpeedUpPreview) { //ASUS_BSP YM Performance: to config stream early, when the previous captureStream is off, and now turn on preview stream only				
                       pr_info("%s: MODE_1: Stream on only\n",__func__);
                } else {
                //LiJen: Workaround: ignore MODE_7->MODE_1 & MODE_8->MODE_1 interrupt. 
                if((g_pre_res == MSM_SENSOR_RES_FULL_SINGLE_CAPTURE) || (g_pre_res == MSM_SENSOR_RES_FULL_BURST_CAPTURE)){
                    sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x72f8, 0x04);
                    INIT_COMPLETION(g_iCatch_comp);
                    pr_info("%s INIT_COMPLETION\n",__func__);
                }
                
                setFixFPS(0);
                setMiniISO(0);
                setCaptureVideoMode(0); //default : capture preview mode
                
                sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7106, 0x06);//preview resolution
                sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7120, 0x00);//swtich preview mode  
                //data[0]=0x00; //for multiwrite reg(0x7120)
                } 
                break;
            case MSM_SENSOR_RES_4BIN:           //MODE_2
                pr_info("%s: MODE_2\n",__func__);

                setFixFPS(0);
                setMiniISO(400);
                setCaptureVideoMode(2); //High speed video preview mode
                
                sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7106, 0x05);//preview resolution
                sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7120, 0x00);//swtich preview mode 
                //data[0]=0x00; //for multiwrite reg(0x7120)
                
                break;
            case MSM_SENSOR_RES_FULL_HD:    //MODE_3
                pr_info("%s: MODE_3\n",__func__);

                setFixFPS(0);
                setMiniISO(400);
                setCaptureVideoMode(2); //High speed video preview mode
                
                //sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7106, 0x02); //1920*1080 //preview resolution
                sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7106, 0x0d); //1920*1088
                sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7120, 0x00);//swtich preview mode 
                //data[0]=0x00; //for multiwrite reg(0x7120)

                break;
            case MSM_SENSOR_RES_FULL:           //MODE_4
                pr_info("%s: MODE_4\n",__func__);
				 //Stimber: Workaround: ignore MODE_8->MODE_4 interrupt.
                if((g_pre_res == MSM_SENSOR_RES_FULL_BURST_CAPTURE)){
                    sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x72f8, 0x04);
                    INIT_COMPLETION(g_iCatch_comp);
                    pr_info("%s INIT_COMPLETION\n",__func__);
                }

				setFixFPS(0);
                setMiniISO(0);
                setCaptureVideoMode(3); //Zsl capture

//ASUS_BSP +++ Stimber "Implement the interface for calibration"
    			if(is_calibration){
    				pr_info("Enter to calibration mode!!!\n");
                    //ASUS_BSP+++ jim3_lin "Disable for A91_SR1 load code from host"
#if 0
    				sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x1011, 0x01);//cpu reset
    				sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x941C, 0x04);
    				sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9010, 0x01);
    				sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9010, 0x00);
    				sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x1306, 0x02);//calibration
    				sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x1011, 0x00);
#endif
                    //ASUS_BSP--- jim3_lin "Disable for A91_SR1 load code from host"

    				sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7106, 0x08);//preview mode
    				sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7120, 0x00);
    				//data[0]=0x00; //for multiwrite reg(0x7120)

                    g_calibrating = true;
//ASUS_BSP --- Stimber "Implement the interface for calibration"
                 }else{
    	            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7106, 0x08);//preview resolution
    	            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7120, 0x00);//swtich preview mode          
    	            //data[0]=0x00; //for multiwrite reg(0x7120)
                 }

                break;
            case MSM_SENSOR_RES_10M:            //MODE_5
		        pr_info("%s: MODE_5\n",__func__);

                setFixFPS(0);
                setMiniISO(0);
                setCaptureVideoMode(3); //Zsl capture
                
                sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7106, 0x11);//preview resolution 4160x2340
                sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7120, 0x00);//swtich preview mode              
                //data[0]=0x00; //for multiwrite reg(0x7120)
        
                break;
            case MSM_SENSOR_RES_HYBRID:     //MODE_6
                pr_info("%s: MODE_6\n",__func__);
                
                setFixFPS(0);
                setMiniISO(0);
                setCaptureVideoMode(1); //Normal video preview mode
                
                sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7106, 0x14);//preview resolution 4k2k 3840x2160
                sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7120, 0x00);//swtich preview mode
                //data[0]=0x00; //for multiwrite reg(0x7120)
                              
                break;
            case MSM_SENSOR_RES_FULL_SINGLE_CAPTURE:    //MODE_7
                pr_info("%s: MODE_7\n",__func__);
		//ASUS_BSP : darrency_lin ++ fix ae setting error when take picture in lock ae/af/awb
		 pr_info(" cur ev = 0x%x ",cur_ev);
		 sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x71EB, 0x05);    //AE lock
		sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7103, cur_ev);
		//ASUS_BSP : darrency_lin -- fix ae setting error when take picture in lock ae/af/awb

                //LiJen: Workaround: ignore MODE_1->MODE_7 interrupt. 
                if((g_pre_res == MSM_SENSOR_RES_QTR)){
                    sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x72f8, 0x04);
                    INIT_COMPLETION(g_iCatch_comp);
                    pr_info("%s INIT_COMPLETION\n",__func__);
                }         

                setFixFPS(0);
                setMiniISO(0);
                setCaptureVideoMode(3); //Zsl capture
                    
                if(g_is_hdr_on){ //HDR
                    pr_info("[Camera] HDR mode\n");
		            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7127, 0x15);//HDR posotive +1.5EV
		            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7128, 0x15);//HDR nagative -1.5EV
		      
                    sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x710f, 0x01);//capture mode - HDR
                    sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7120, 0x01);//swtich capture mode
                    //data[0]=0x01;	//for multiwrite reg(0x7120)
                }else if(g_is_nr_on){ //NR
                    pr_info("[Camera] NR mode\n");
                    sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x710f, 0x03);//swtich burst capture mode
                    if(MSM_SENSOR_RES_FULL == g_pre_res){
                        sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7120, 0x02);//swtich capture flash mode
                        //data[0]=0x02; //for multiwrite reg(0x7120)
                    }else{
                        sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7120, 0x03);//swtich capture flash mode
                        //data[0]=0x03; //for multiwrite reg(0x7120)
                    } 
                }else{
                    sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x710f, 0x00);//swtich single capture mode
                    if(MSM_SENSOR_RES_FULL == g_pre_res){
                        sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7120, 0x02);//swtich capture flash mode
                        //data[0]=0x02; //for multiwrite reg(0x7120)
                    }else{
                        sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7120, 0x03);//swtich capture flash mode
                        //data[0]=0x03; //for multiwrite reg(0x7120)
                    }
                }
                break;
            case MSM_SENSOR_RES_FULL_BURST_CAPTURE:    //MODE_8
                pr_info("%s: MODE_8\n",__func__);
				 //Stimber: Workaround: ignore MODE_4->MODE_8 & MODE_1->MODE_8 interrupt.
                if((g_pre_res == MSM_SENSOR_RES_FULL) || (g_pre_res == MSM_SENSOR_RES_QTR)){
                    sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x72f8, 0x04);
                    INIT_COMPLETION(g_iCatch_comp);
                    pr_info("%s INIT_COMPLETION\n",__func__);
                }

                setFixFPS(0);
                setMiniISO(0);
                setCaptureVideoMode(3); //Zsl capture
                
                sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x710f, 0x03);//swtich burst capture mode
                if(MSM_SENSOR_RES_FULL == g_pre_res){
                	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7120, 0x02);//swtich capture flash mode
                	//data[0]=0x02; //for multiwrite reg(0x7120)
                }else{
                	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7120, 0x03);//swtich capture flash mode
                	//data[0]=0x03; //for multiwrite reg(0x7120)
                }                   

                break;
            case MSM_SENSOR_RES_10M_BURST_CAPTURE:    //MODE_10
                pr_info("%s: MODE_10\n",__func__);

                setFixFPS(0);
                setMiniISO(0);
                setCaptureVideoMode(3); //Zsl capture
                
                sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x710f, 0x03);//swtich burst capture mode
                sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7120, 0x02);//swtich capture flash mode  
                //data[0]=0x02; //for multiwrite reg(0x7120)

                break; 
            case MSM_SENSOR_RES_3M_BURST_CAPTURE:    //MODE_11
                pr_info("%s: MODE_11\n",__func__);
				 //Stimber: Workaround: ignore MODE_4->MODE_11 & MODE_1->MODE_11 interrupt.
                if((g_pre_res == MSM_SENSOR_RES_FULL) || (g_pre_res == MSM_SENSOR_RES_QTR)){
                    sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x72f8, 0x04);
                    INIT_COMPLETION(g_iCatch_comp);
                    pr_info("%s INIT_COMPLETION\n",__func__);
                }

                setFixFPS(0);
                setMiniISO(0);
                
                //ASUS_BSP+++ jim3_lin "Implement DIT postprocess-Mode2"
                //setCaptureVideoMode(3); //Zsl capture
                if( g_pre_res == MSM_SENSOR_RES_QTR ){ // for Hi-Light flag1 analog bin MODE_1->MODE_11->MODE_1
                    sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x710f, 0x03);//ASUS_BSP jim3_lin "Fix no torch on Hi-Light burst snapshot force flash mode"
                }
                else{
                    setCaptureVideoMode(3); //Zsl capture
                    sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x710f, 0x03);//swtich burst capture mode
                }
                //ASUS_BSP--- jim3_lin "Implement DIT postprocess-Mode2"

                sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7120, 0x02);//swtich capture flash mode               
                //data[0]=0x02; //for multiwrite reg(0x7120)

                break;
            default:
                pr_err("%s: not support resolution res %d\n",__func__, res);
        	  return -EINVAL;                          
                break;
       }

       //Enable EXIF                  
       // enableExif: when res is not equal to mode 1, mode2, mode3, mode6
       if(res == MSM_SENSOR_RES_4BIN ||
          res == MSM_SENSOR_RES_FULL_HD ||
          res == MSM_SENSOR_RES_HYBRID){
            l_edgeExifEnable = false;
       }else{
            l_edgeExifEnable = true;
       }

       if(res == MSM_SENSOR_RES_QTR ||
          res == MSM_SENSOR_RES_FULL ||
          res == MSM_SENSOR_RES_10M){
            iCatch_enable_exif(l_edgeExifEnable  , true);
       }else if(res == MSM_SENSOR_RES_FULL_SINGLE_CAPTURE ||
          res == MSM_SENSOR_RES_FULL_BURST_CAPTURE ||
          res == MSM_SENSOR_RES_10M_BURST_CAPTURE ||
          res == MSM_SENSOR_RES_3M_BURST_CAPTURE){
            iCatch_enable_exif(l_edgeExifEnable, false);
       }else{
            iCatch_enable_exif(false, false);
       }
      
       g_pre_res = res;                
	return 0;
}

int32_t sensor_set_mode_second_camera(int res)
{
u16 readval70; //ASUS_BSP : darrency_lin ++ for fix issue that preview hung up in switching resolution
u16 readval72;//ASUS_BSP : darrency_lin ++ for fix issue that preview hung up in switching resolution
u16 readval73;//ASUS_BSP : darrency_lin ++ for fix issue that preview hung up in switching resolution
int readregtimes=0;//ASUS_BSP : darrency_lin ++ for fix issue that preview hung up in switching resolution

       if(true == iCatch_first_open){                  
            //wait_for_AE_ready();
            iCatch_enable_autonight(true); //ASUS_BSP jim3_lin "Implement DIT postprocess-Mode2"
       }

       //ASUS_BSP +++ bill_chen "Implement image stabilization"
       if(g_is_eis_on)
          iCatch_enable_autonight(false);
       //ASUS_BSP --- bill_chen "Implement image stabilization"

       switch(res){
            case MSM_SENSOR_RES_QTR:            //MODE_1 - Capture
                pr_info("%s: MODE_1 - Capture\n",__func__);
		  frontcam_capture_status = true;
		setCaptureVideoMode(0);

		//ASUS_BSP : darrency_lin ++ fix front camera ae setting error when take picture in lock ae/af/awb
		 pr_info(" front cur ev = 0x%x ",cur_ev);
		 sensor_write_reg(ov2720asus_s_ctrl.sensor_i2c_client->client, 0x71EB, 0x05);    //AE lock
		sensor_write_reg(ov2720asus_s_ctrl.sensor_i2c_client->client, 0x7103, cur_ev);
		//ASUS_BSP : darrency_lin -- fix front camera ae setting error when take picture in lock ae/af/awb

                if(g_is_hdr_on){ //HDR
                    pr_info("[Camera] HDR mode\n");
                    sensor_write_reg(ov2720asus_s_ctrl.sensor_i2c_client->client, 0x7127, 0x15);//HDR posotive +1.5EV
                    sensor_write_reg(ov2720asus_s_ctrl.sensor_i2c_client->client, 0x7128, 0x15);//HDR nagative -1.5EV
                    sensor_write_reg(ov2720asus_s_ctrl.sensor_i2c_client->client, 0x710f, 0x01);//capture mode - HDR
   //                 sensor_write_reg(ov2720asus_s_ctrl.sensor_i2c_client->client, 0x7106, 0x16); //1600x1200
                    sensor_write_reg(ov2720asus_s_ctrl.sensor_i2c_client->client, 0x7120, 0x01);//swtich capture mode                    
                }else{         
                    sensor_write_reg(ov2720asus_s_ctrl.sensor_i2c_client->client, 0x7106, 0x16); //1600x1200

//ASUS_BSP : darrency_lin ++ for fix issue that preview hung up in switching resolution
				do{
				msleep(15);
				sensor_read_reg(ov2720asus_s_ctrl.sensor_i2c_client->client, 0x7070, &readval70);
				//pr_info(" 0x7070 readval = 0x%x", readval70);
				sensor_read_reg(ov2720asus_s_ctrl.sensor_i2c_client->client, 0x7072, &readval72);
				//pr_info(" 0x7072 readval = 0x%x", readval72);
				sensor_read_reg(ov2720asus_s_ctrl.sensor_i2c_client->client, 0x7073, &readval73);
				//pr_info(" 0x7073 readval = 0x%x", readval73);
				readregtimes++ ;
				}while((readval70!=0x40  || readval72!=0x40 || readval73!=0x06)&(readregtimes!=50));
                           if(readregtimes>=30)
			           pr_info(" %s  check register times = %d ",__func__, readregtimes);
                           if(readregtimes==50)
				      return  -1;
//ASUS_BSP : darrency_lin -- for fix issue that preview hung up in switching resolution
                    sensor_write_reg(ov2720asus_s_ctrl.sensor_i2c_client->client, 0x7120, 0x00);  //preview mode
                    return 0;
                }
                break;                             
            case MSM_SENSOR_RES_4BIN:            //MODE_2 - Preview
                pr_info("%s: MODE_2 - Preview\n",__func__);
                setCaptureVideoMode(1);//ASUS_BSP : darrency_lin ++ for fix issue that preview hung up in switching resolution
                sensor_write_reg(ov2720asus_s_ctrl.sensor_i2c_client->client, 0x7106, 0x1A);  //800x600
                sensor_write_reg(ov2720asus_s_ctrl.sensor_i2c_client->client, 0x7120, 0x00);  //preview mode
                break;                          
            case MSM_SENSOR_RES_FULL_HD:       //MODE_3 - HD video recording
                pr_info("%s: MODE_3 - Video\n",__func__);
                setCaptureVideoMode(1);
                sensor_write_reg(ov2720asus_s_ctrl.sensor_i2c_client->client, 0x7106, 0x04);  //1280x720
                sensor_write_reg(ov2720asus_s_ctrl.sensor_i2c_client->client, 0x7120, 0x00);  //preview mode
                break;            
            case MSM_SENSOR_RES_FULL:       //MODE_4 - BF Preview
                pr_info("%s: MODE_4 - BF Preview \n",__func__);
                setCaptureVideoMode(1);
                sensor_write_reg(ov2720asus_s_ctrl.sensor_i2c_client->client, 0x7106, 0x1A);  // 800x600
                sensor_write_reg(ov2720asus_s_ctrl.sensor_i2c_client->client, 0x7120, 0x00);  //preview mode
                break;
            default:
                pr_err("%s: not support resolution res %d\n",__func__, res);
        	  return -EINVAL;                          
                break;                
        }
        return 0;
}

#define CONFIG_I2C_READ_WRITE
#ifdef CONFIG_I2C_READ_WRITE
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#define DBG_TXT_BUF_SIZE 256
static char debugTxtBuf[DBG_TXT_BUF_SIZE];
static u32 i2c_get_value;

static ssize_t i2c_set_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t i2c_get_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dbg_iCatch7002a_chip_power_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

#if 1
static ssize_t dbg_i7002a_fw_in_isp_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dbg_i7002a_fw_in_isp_read(struct file *file, char __user *buf, size_t count,
				loff_t *ppos)
{
#if 0 
	int len = 0;
	int tot = 0;
	//char debug_buf[1024];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;

//	printk("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n", __FUNCTION__, buf, count, ppos, *ppos);

	if (*ppos)
		return 0;	/* the end */

	/* [Project id]-[FrontSensor]-[FW Version]*/   
	if (front_chip_id == SENSOR_ID_OV2720) {
		len = snprintf(bp, dlen, "%02X-%02X-%06X\n", tegra3_get_project_id(), 1, version_num_in_isp);
		tot += len; bp += len; dlen -= len;
	} else if (front_chip_id == SENSOR_ID_MI1040){
		/* mi1040 chip_id= 0x2481 */
		len = snprintf(bp, dlen, "%02X-%02X-%06X\n", tegra3_get_project_id(), 2, version_num_in_isp);
		tot += len; bp += len; dlen -= len;
	} else {
		len = snprintf(bp, dlen, "%02X-%02X-%06X\n", tegra3_get_project_id(), 0, version_num_in_isp);
		tot += len; bp += len; dlen -= len;
	}

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;
	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
#endif    
       return 0;
}
#endif

static ssize_t dbg_i7002a_page_dump_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static int dbg_i7002a_page_dump_read(struct file *file, char __user *buf, size_t count,
				loff_t *ppos)
{
#if 0
    static int pfd  = 0;
    unsigned char buffer[512000];
    
    pfd = open("/data/BOOT_DUMP.BIN", O_WRONLY | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
    write(pfd , buffer, bytes);
    close(pfd);
#endif        
#if 1
	int len = 0;
	int tot = 0;
//	char debug_buf[1024];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;
	int i =0;
	u8 mypage[0x100];

//	printk("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n", __FUNCTION__, buf, count, ppos, *ppos);

	if (*ppos)
		return 0;	/* the end */

       if(true == g_spi_is_support_1M){
            dbg_i7002a_page_index = 4095;
       }else{
	     dbg_i7002a_page_index = 2047;
       }
              
	len = snprintf(bp, dlen, "page_index=%d (0x%X)\n", dbg_i7002a_page_index, dbg_i7002a_page_index);
	tot += len; bp += len; dlen -= len;

	get_one_page_from_i7002a(dbg_i7002a_page_index, mypage);
	for(i=0; i < 0x100; i++) {
		if(i%16 == 0) {
			len = snprintf(bp, dlen, "[%03X] ", i);
			tot += len; bp += len; dlen -= len;
		}
		len = snprintf(bp, dlen, "%02X ", mypage[i]);
		tot += len; bp += len; dlen -= len;

		if(i%16 == 15) {
			len = snprintf(bp, dlen, "\n");
			tot += len; bp += len; dlen -= len;
		}
	}

	//if (copy_to_user(buf, debug_buf, tot))
	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;
	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
#endif 
       return 0;
}
#if 0
void BB_I2CDumpDmem( 
        u32 dmemAddr, 
        u32 size 
) 
{ 
        u32 i, reg10a6=0, reg10a7=0, regData, temp1, temp2; 
        u8* pbuf; 
        u32 fd; 
        u8 ucFileName[30]; 

        pr_info("%s:: size: %d\n",  __func__, size); 

        pbuf = osMemAlloc(size); 
        pr_info("pbuf:0x%x\n",(u32)pbuf); 
        
        strcpy(ucFileName, "/data/DUMPDAT.BIN"); 

        sp5kFsFileDelete(ucFileName); 
        fd = sp5kFsFileOpen(ucFileName, SP5K_FS_OPEN_CREATE); 

        //I2CDataWrite(0x70c4,0x00); 
        //I2CDataWrite(0x70c5,0x00); 
        //I2CDataWrite(0x10a6,0x00); 
        //I2CDataWrite(0x10a7,0x00); 
        sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x70c4,0x00); 
        sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x70c5,0x00); 
        sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x10a6,0x00); 
        sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x10a7,0x00); 

        for(i=0;i<size;i++) 
        { 
                temp1 = 0x40|(dmemAddr / 0x2000); 
                temp2 = (dmemAddr - (temp1-0x40)*0x2000) / 0x800; 
                regData = 0x1800 + dmemAddr - (temp1-0x40)*0x2000 - temp2*0x800; 
                if(temp1 != reg10a6) 
                { 
                        reg10a6 = temp1; 
                        //hsI2CDataWrite(0x10a6,reg10a6); 
                        sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x10a6,reg10a6);
                } 
                if(temp2 != reg10a7) 
                { 
                        reg10a7 = temp2; 
                        //hsI2CDataWrite(0x10a7,reg10a7); 
                        sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x10a7,reg10a7);
                } 
                //*(pbuf+i) = hsI2CDataRead(regData); 
                *(pbuf+i) = sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x40e5);
                dmemAddr++; 
        } 
        /* inform 7002 read operation done */ 
        //I2CDataWrite(0x72C1,0x00); 
        //I2CDataWrite(0x10a6,0x00); 
        //I2CDataWrite(0x10a7,0x00); 
        sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x72C1,0x00); 
        sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x10a6,0x00); 
        sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x10a7,0x00);         
        
        sp5kFsFileWrite(fd, pbuf, size); 
        sp5kFsFileClose(fd); 

        osMemFree(pbuf); 
} 
#endif
static ssize_t dbg_i7002a_bin_dump_dmem_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dbg_i7002a_bin_dump_dmem_read(struct file *file, char __user *buf, size_t count,
				loff_t *ppos)
{
	int len, tot = 0;
	//char debug_buf[1024];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;
//	int i = 0;
	//int ret = 0;
	char* mybin;
	struct file *fp_bin_dump = NULL;
	mm_segment_t old_fs;
	loff_t offset = 0;

        u32 i, reg10a6=0, reg10a7=0, regData, temp1, temp2; 
        u32 dmemAddr=0x28000;//starting dump address 
        u32 size=0x28000;//dump size 
        u16 temp;
        
//	printk("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n", __FUNCTION__, buf, count, ppos, *ppos);

	if (*ppos)
		return 0;	/* the end */

        sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x70c4,0x00); 
        sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x70c5,0x00); 
        sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x10a6,0x00); 
        sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x10a7,0x00);

	mybin = kmalloc(size, GFP_KERNEL);

        for(i=0;i<size;i++) 
        { 
                temp1 = 0x40|(dmemAddr / 0x2000); 
                temp2 = (dmemAddr - (temp1-0x40)*0x2000) / 0x800; 
                regData = 0x1800 + dmemAddr - (temp1-0x40)*0x2000 - temp2*0x800; 
                if(temp1 != reg10a6) 
                { 
                        reg10a6 = temp1; 
                        sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x10a6,reg10a6);
                } 
                if(temp2 != reg10a7) 
                { 
                        reg10a7 = temp2; 
                        sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x10a7,reg10a7);
                } 
                sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, regData, &temp);
                if(i <= 32){
                    pr_info("%d ",temp);
                }
                sprintf((mybin+i) , "%d", temp);
                dmemAddr++; 
        } 
        /* inform 7002 read operation done */ 
        sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x72C1,0x00); 
        sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x10a6,0x00); 
        sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x10a7,0x00);  

	/* Dump to /data/bin_dump.bin */
	fp_bin_dump = filp_open("/data/DUMPDAT.BIN", O_RDWR | O_CREAT, S_IRUGO | S_IWUGO);
	if ( IS_ERR_OR_NULL(fp_bin_dump) ){
		filp_close(fp_bin_dump, NULL);
		len = snprintf(bp, dlen, "%s: open %s fail\n", __FUNCTION__, "/data/bin_dump.bin");
		tot += len; bp += len; dlen -= len;
	}
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	offset = 0;
	if (fp_bin_dump->f_op != NULL && fp_bin_dump->f_op->write != NULL){
		fp_bin_dump->f_op->write(fp_bin_dump,
			mybin,
			size,
			&offset);
	} else {
		len = snprintf(bp, dlen, "%s: f_op might be null\n", __FUNCTION__);
		tot += len; bp += len; dlen -= len;
	}
	set_fs(old_fs);
	filp_close(fp_bin_dump, NULL);
	kfree(mybin);

	len = snprintf(bp, dlen, "%s: Dump Complete.\n", __FUNCTION__);
	tot += len; bp += len; dlen -= len;

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;
	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static ssize_t dbg_i7002a_bin_dump_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dbg_i7002a_bin_dump_read(struct file *file, char __user *buf, size_t count,
				loff_t *ppos)
{
	int len, tot = 0;
	//char debug_buf[1024];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;
//	int i = 0;
	int ret = 0;
	char* mybin;
	struct file *fp_bin_dump = NULL;
	mm_segment_t old_fs;
	loff_t offset = 0;

//	printk("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n", __FUNCTION__, buf, count, ppos, *ppos);

	if (*ppos)
		return 0;	/* the end */

	//I2CDataWrite(0x70c4,0x00);
	//I2CDataWrite(0x70c5,0x00);
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x70c4,0x00);
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x70c5,0x00);

	ret = I2C_SPIInit();
	if (ret) {
		printk("%s: get nothing. ret= %d", __FUNCTION__, ret);
		return ret;
	}

	I2C_SPIFlashReadId();

	if (true == g_spi_is_support_1M) {
		//mybin = kmalloc(1024*1024, GFP_KERNEL);
		mybin = vmalloc(1024*1024);
		I2C_SPIFlashRead(0, 4096, mybin);
	}
	else {
		//mybin = kmalloc(512*1024, GFP_KERNEL);
		mybin = vmalloc(512*1024);
		I2C_SPIFlashRead(0, 2048, mybin);
	}
    
	/* Dump to /data/bin_dump.bin */
	fp_bin_dump = filp_open("/data/bin_dump.bin", O_RDWR | O_CREAT, S_IRUGO | S_IWUGO);
	if ( IS_ERR_OR_NULL(fp_bin_dump) ){
		filp_close(fp_bin_dump, NULL);
		len = snprintf(bp, dlen, "%s: open %s fail\n", __FUNCTION__, "/data/bin_dump.bin");
		tot += len; bp += len; dlen -= len;
	}
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	offset = 0;
	if (fp_bin_dump->f_op != NULL && fp_bin_dump->f_op->write != NULL){
		if (true == g_spi_is_support_1M)
			fp_bin_dump->f_op->write(fp_bin_dump, mybin, 1024*1024, &offset);
		else
			fp_bin_dump->f_op->write(fp_bin_dump, mybin, 512*1024, &offset);        
	} else {
		len = snprintf(bp, dlen, "%s: f_op might be null\n", __FUNCTION__);
		tot += len; bp += len; dlen -= len;
	}
	set_fs(old_fs);
	filp_close(fp_bin_dump, NULL);
	//kfree(mybin);
	vfree(mybin);

	len = snprintf(bp, dlen, "%s: Dump Complete.\n", __FUNCTION__);
	tot += len; bp += len; dlen -= len;

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;
	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static ssize_t dbg_i7002a_fw_header_dump_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

#define DUMP_HEADER(mypage) do {	\
		for(i = 0; i < 0x100; i++) {	\
			if(i%16 == 0) {	\
				len = snprintf(bp, dlen, "[%02X] ", i);	\
				tot += len; bp += len; dlen -= len;	\
			}	\
			len = snprintf(bp, dlen, "%02X ", mypage[i]);	\
			tot += len; bp += len; dlen -= len;	\
			if(i%16 == 15) {	\
				len = snprintf(bp, dlen, "\n");	\
				tot += len; bp += len; dlen -= len;	\
			}	\
		}	\
	} while (0)


static ssize_t dbg_i7002a_fw_header_dump_read(struct file *file, char __user *buf, size_t count,
				loff_t *ppos)
{
#if 0
	int len = 0;
	int tot = 0;
	char debug_buf[3072];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;
	int i =0;
	u8 fw1page[0x100];
	u8 fw2page[0x100];
	u8 overallpage[0x100];
	int fw2_header_page_index, fw2_offset = 0;

//	printk("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n", __FUNCTION__, buf, count, ppos, *ppos);

	if (*ppos)
		return 0;	/* the end */

	i7002a_isp_on(1);

	/* dump fw1 header */
	get_one_page_from_i7002a(0, fw1page);
	len = snprintf(bp, dlen, "fw1: page[%d]:\n", 0);
	tot += len; bp += len; dlen -= len;
	DUMP_HEADER(fw1page);

	msleep(40);

	/* dump fw2 header */
	fw2_offset = 16 +
		((fw1page[3] << 24) | (fw1page[2] << 16) | (fw1page[1] << 8) | fw1page[0]) +
		((fw1page[7] << 24) | (fw1page[6] << 16) | (fw1page[5] << 8) | fw1page[4]);
	fw2_header_page_index = fw2_offset >> 8;
	get_one_page_from_i7002a(fw2_header_page_index, fw2page);
	len = snprintf(bp, dlen, "fw2: page[%d]:\n", fw2_header_page_index);
	tot += len; bp += len; dlen -= len;
	DUMP_HEADER(fw2page);

	msleep(40);

	/* dump overall header */
	get_one_page_from_i7002a(2047, overallpage);
	len = snprintf(bp, dlen, "Overall: page[%d]:\n", 2047);
	tot += len; bp += len; dlen -= len;
	DUMP_HEADER(overallpage);

	if (front_chip_id == SENSOR_ID_OV2720) {
		if(memcmp(ov2720_fw1page_0_1_21, fw1page, 0x100) == 0) {
			if(memcmp(ov2720_fw2page_0_1_21, fw2page, 0x100) == 0) {
				if(memcmp(ov2720_overallpage_0_1_21, overallpage, 0x100) == 0) {
					b_fw_is_valid = 1;
				} else {
					len = snprintf(bp, dlen, "%s(%d): wrong overall page.\n", __FUNCTION__, __LINE__);
					tot += len; bp += len; dlen -= len;
					b_fw_is_valid = 0;
				}
			} else {
				len = snprintf(bp, dlen, "%s(%d): wrong fw2 page.\n", __FUNCTION__, __LINE__);
				tot += len; bp += len; dlen -= len;
				b_fw_is_valid = 0;
			}
		} else {
			len = snprintf(bp, dlen, "%s(%d): wrong fw1 page.\n", __FUNCTION__, __LINE__);
			tot += len; bp += len; dlen -= len;
			b_fw_is_valid = 0;
		}
	} else if (front_chip_id == SENSOR_ID_MI1040) {
		if(memcmp(mi1040_fw1page_0_1_21, fw1page, 0x100) == 0) {
			if(memcmp(mi1040_fw2page_0_1_21, fw2page, 0x100) == 0) {
				if(memcmp(mi1040_overallpage_0_1_21, overallpage, 0x100) == 0) {
					b_fw_is_valid = 1;
				} else {
					len = snprintf(bp, dlen, "%s(%d): wrong overall page.\n", __FUNCTION__, __LINE__);
					tot += len; bp += len; dlen -= len;
					b_fw_is_valid = 0;
				}
			} else {
				len = snprintf(bp, dlen, "%s(%d): wrong fw2 page.\n", __FUNCTION__, __LINE__);
				tot += len; bp += len; dlen -= len;
				b_fw_is_valid = 0;
			}
		} else {
			len = snprintf(bp, dlen, "%s(%d): wrong fw1 page.\n", __FUNCTION__, __LINE__);
			tot += len; bp += len; dlen -= len;
			b_fw_is_valid = 0;
		}
	} else {
		len = snprintf(bp, dlen, "%s(%d): Unknown Front Camera ID.\n", __FUNCTION__, __LINE__);
		tot += len; bp += len; dlen -= len;
		b_fw_is_valid = 0;
	}

	i7002a_isp_on(0);

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;
	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
#endif
        return 0;
}

static ssize_t dbg_fw_update_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dbg_fw_update_read(struct file *file, char __user *buf, size_t count,
				loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[512];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;

	//printk("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n", __FUNCTION__, buf, count, ppos, *ppos);

	if (*ppos)
		return 0;	/* the end */

	switch(fw_update_status) {
	case ICATCH_FW_NO_CMD:
		len = snprintf(bp, dlen, "Never issue fw update cmd yet.\n");
		tot += len; bp += len; dlen -= len;
		break;

	case ICATCH_FW_IS_BURNING:
		if ((g_page_count >= 0) && (g_page_count <= g_total_page_count)) {
			int time_left = 0;
			if (flash_type == ICATCH_FLASH_TYPE_ST)
				time_left = g_page_count * 8 / 100;
			else
				time_left = g_page_count / 4;

			len = snprintf(bp, dlen, "FW update progress: %d/%d; Timeleft= %d secs\n", g_total_page_count - g_page_count + 1, g_total_page_count, time_left);
			tot += len; bp += len; dlen -= len;
		} else {
			len = snprintf(bp, dlen, "g_page_count=%d; total=%d\n", g_page_count, g_total_page_count);
			tot += len; bp += len; dlen -= len;
		}
		break;

	case ICATCH_FW_UPDATE_SUCCESS:
		len = snprintf(bp, dlen, "FW Update Complete!\n");
		tot += len; bp += len; dlen -= len;
		break;

	case ICATCH_FW_UPDATE_FAILED:
		len = snprintf(bp, dlen, "FW Update FAIL!\n");
		tot += len; bp += len; dlen -= len;
		break;

	default:
		len = snprintf(bp, dlen, "FW Update Status Unknown: %d\n", fw_update_status);
		tot += len; bp += len; dlen -= len;
	}

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;
	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static ssize_t dbg_fw_update_write(struct file *file, const char __user *buf, size_t count,
				loff_t *ppos)
{
	char debug_buf[256];
	int cnt;
	char bin_path[80];

	//printk("%s: buf=%p, count=%d, ppos=%p\n", __FUNCTION__, buf, count, ppos);
	if (count > sizeof(debug_buf))
		return -EFAULT;
	if (copy_from_user(debug_buf, buf, count))
		return -EFAULT;

	debug_buf[count] = '\0';	/* end of string */
	cnt = sscanf(debug_buf, "%s", bin_path);

	/* burning */
	printk("%s: BB_WrSPIFlash()++\n", __FUNCTION__);
	BB_WrSPIFlash(bin_path);
	printk("%s: BB_WrSPIFlash()--\n", __FUNCTION__);
    
	return count;
}

static ssize_t i2c_set_write(struct file *file, const char __user *buf, size_t count,
				loff_t *ppos)
{
  int len;
  int arg[2];
  //int gpio, set;

  //char gpioname[8];

//  printk("%s: buf=%p, count=%d, ppos=%p\n", __FUNCTION__, buf, count, ppos);
  arg[0]=0;

	if (*ppos)
		return 0;	/* the end */

//+ parsing......
  len=(count > DBG_TXT_BUF_SIZE-1)?(DBG_TXT_BUF_SIZE-1):(count);
  if (copy_from_user(debugTxtBuf,buf,len))
		return -EFAULT;

  debugTxtBuf[len]=0; //add string end

  sscanf(debugTxtBuf, "%x %x", &arg[0], &arg[1]);
  printk("argument is arg1=0x%x arg2=0x%x\n",arg[0], arg[1]);


  *ppos=len;
  sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, arg[0], arg[1]);

	return len;	/* the end */
}
/*
static ssize_t i2c_config_read(struct file *file, char __user *buf, size_t count,
				loff_t *ppos)
{

}
*/

static ssize_t i2c_get_write(struct file *file, const char __user *buf, size_t count,
				loff_t *ppos)
{
  int len;
  int arg = 0;
  //int gpio, set;

  //char gpioname[8];

//  printk("%s: buf=%p, count=%d, ppos=%p\n", __FUNCTION__, buf, count, ppos);


	if (*ppos)
		return 0;	/* the end */

//+ parsing......
  len=(count > DBG_TXT_BUF_SIZE-1)?(DBG_TXT_BUF_SIZE-1):(count);
  if (copy_from_user(debugTxtBuf,buf,len))
		return -EFAULT;

  debugTxtBuf[len]=0; //add string end

  sscanf(debugTxtBuf, "%x", &arg);
  printk("argument is arg=0x%x\n",arg);


  *ppos=len;
  sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, arg, (u16 *)&i2c_get_value);

	return len;	/* the end */
}

static ssize_t i2c_get_read(struct file *file, char __user *buf, size_t count,
				loff_t *ppos)
{
	int len = 0;
	char *bp = debugTxtBuf;

       if (*ppos)
		return 0;	/* the end */
	len = snprintf(bp, DBG_TXT_BUF_SIZE, "the value is 0x%x\n", i2c_get_value);

	if (copy_to_user(buf, debugTxtBuf, len))
		return -EFAULT;
       *ppos += len;
	return len;

}

static ssize_t dbg_iCatch7002a_vga_status_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dbg_iCatch7002a_vga_status_read(struct file *file, char __user *buf, size_t count,
				loff_t *ppos)
{
#if 0
	int len = 0;
	int tot = 0;
	//char debug_buf[1024];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;

	u16 chip_id, tmp = 0x0;

	//printk("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n", __FUNCTION__, buf, count, ppos, *ppos);

	if (*ppos)
		return 0;	/* the end */

	if (sensor_opened == false) {
		if (1) {
			//info->pdata->power_on();
			//tegra_camera_mclk_on_off(1);
			msleep(100);
		} else {
			len = snprintf(bp, dlen, "iCatch7002a info isn't enough for power_on.\n");
			tot += len; bp += len; dlen -= len;
		}
	}

	/*Start - Power on sensor & enable clock - Front I2C (OV2720)*/
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x0084, 0x14); /* To sensor clock divider */
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x0034, 0xFF); /* Turn on all clock */
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9030, 0x3f);
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9031, 0x04);
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9034, 0xf3);
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9035, 0x04);

	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9032, 0x02);
	msleep(10);
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9032, 0x00);
	msleep(10);
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9033, 0x00);
	msleep(10);
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9033, 0x04);
	msleep(10);
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9034, 0xf2);
	/*End - Power on sensor & enable clock */

	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9008, 0x00); /* Need to check with vincent */
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9009, 0x00);
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x900A, 0x00);
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x900B, 0x00);

	/*Start - I2C Read*/
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9138, 0x30); /* Sub address enable */
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9140, 0x6C); /* Slave address      */
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9100, 0x03); /* Read mode          */
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9110, 0x30); /* Register addr MSB  */
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9112, 0x0a); /* Register addr LSB  */
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9104, 0x01); /* Trigger I2C read   */

	msleep(10);
	sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9111, &tmp);

	//printk("0x%x\n", tmp);
	chip_id = (tmp << 8) & 0xFF00;

	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9110, 0x30); /* Register addr MSB  */
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9112, 0x0b); /* Register addr LSB  */
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9104, 0x01); /* Trigger I2C read   */

	msleep(10);
	sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9111, &tmp);
	//printk("0x%x\n", tmp);
	chip_id = chip_id  | (tmp & 0xFF);

	if (chip_id == SENSOR_ID_OV2720) {
		len = snprintf(bp, dlen, "1\n");
		tot += len; bp += len; dlen -= len;
	} else {
#if 0
		len = snprintf(bp, dlen, "back chip_id= 0x%x\n", chip_id);
		tot += len; bp += len; dlen -= len;
#endif
		/* Check if mi1040 is available. */
		sensor_write_table(mt9m114_s_ctrl.sensor_i2c_client->client, query_mi1040_id_msb_seq);
		sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9111, &tmp);

		chip_id = (tmp << 8) & 0xFF00;

		sensor_write_table(mt9m114_s_ctrl.sensor_i2c_client->client, query_mi1040_id_lsb_seq);
		sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9111, &tmp);
		chip_id = chip_id  | (tmp & 0xFF);

		printk("0x%x\n", chip_id);

		if (chip_id == SENSOR_ID_MI1040) {
			/* mi1040 chip_id= 0x2481 */
			len = snprintf(bp, dlen, "1\n");
			tot += len; bp += len; dlen -= len;
		} else {
			len = snprintf(bp, dlen, "0\n");
			tot += len; bp += len; dlen -= len;
		}
	}

	if (sensor_opened == false) {
		if (1) {
			//tegra_camera_mclk_on_off(0);
			//info->pdata->power_off();
		} else {
			len = snprintf(bp, dlen, "iCatch7002a info isn't enough for power_off.\n");
			tot += len; bp += len; dlen -= len;
		}
	}

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;
	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
#endif
       return 0;
}


static ssize_t dbg_iCatch7002a_camera_status_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dbg_iCatch7002a_camera_status_read(struct file *file, char __user *buf, size_t count,
				loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[256];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;

	u16 chip_id, tmp = 0x0;

	//printk("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n", __FUNCTION__, buf, count, ppos, *ppos);

	if (*ppos)
		return 0;	/* the end */

	if (sensor_opened == false) {
		if (1) {
			//info->pdata->power_on();
			//tegra_camera_mclk_on_off(1);
			msleep(100);
		} else {
			len = snprintf(bp, dlen, "iCatch7002a info isn't enough for power_on.\n");
			tot += len; bp += len; dlen -= len;
		}
	}
	/* SONY IMX175 */
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x0084, 0x14); /* To sensor clock divider */
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x0034, 0xFF); /* Turn on all clock */
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9030, 0x3f);
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9031, 0x04);
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9034, 0xf2);
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9035, 0x04);
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9032, 0x00);
	msleep(10);
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9032, 0x20);
	msleep(10);
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9032, 0x30);
	msleep(10);
	/*End - Power on sensor & enable clock */
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9008, 0x00); /* Need to check with vincent */
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9009, 0x00);
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x900A, 0x00);
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x900B, 0x00);

	/*Start - I2C Read*/
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9238, 0x30); /* Sub address enable */
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9240, 0x20); /* Slave address      */
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9200, 0x03); /* Read mode          */
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9210, 0x00); /* Register addr MSB  */
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9212, 0x00); /* Register addr LSB  */
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9204, 0x01); /* Trigger I2C read   */

	msleep(10);
	sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9211, &tmp);
	// printk("0x%x\n", tmp);
	chip_id = (tmp << 8) & 0xFF00;

	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9210, 0x00); /* Register addr MSB  */
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9212, 0x01); /* Register addr LSB  */
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9204, 0x01); /* Trigger I2C read   */

	msleep(10);
	sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x9211, &tmp);
	// printk("0x%x\n", tmp);
	chip_id = chip_id  | (tmp & 0xFF);

	if (chip_id == SENSOR_ID_IMX175) {
		len = snprintf(bp, dlen, "1\n");
		tot += len; bp += len; dlen -= len;
	} else {
#if 0
		len = snprintf(bp, dlen, "back chip_id= 0x%x\n", chip_id);
		tot += len; bp += len; dlen -= len;
#endif
		len = snprintf(bp, dlen, "0\n");
		tot += len; bp += len; dlen -= len;
	}

	if (sensor_opened == false) {
		if (1) {
			//tegra_camera_mclk_on_off(0);
			//info->pdata->power_off();
		} else {
			len = snprintf(bp, dlen, "iCatch7002a info isn't enough for power_off.\n");
			tot += len; bp += len; dlen -= len;
		}
	}

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;
	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static ssize_t dbg_iCatch7002a_chip_power_write(struct file *file, const char __user *buf, size_t count,
				loff_t *ppos)
{
    int len;
    int arg;
    //int gpio, set;

    //char gpioname[8];

    //  printk("%s: buf=%p, count=%d, ppos=%p\n", __FUNCTION__, buf, count, ppos);
    arg=0;

    if (*ppos)
        return 0;	/* the end */

    //+ parsing......
    len=(count > DBG_TXT_BUF_SIZE-1)?(DBG_TXT_BUF_SIZE-1):(count);
    if (copy_from_user(debugTxtBuf,buf,len))
    		return -EFAULT;

    debugTxtBuf[len]=0; //add string end

    sscanf(debugTxtBuf, "%x", &arg);
    printk("argument is arg=0x%x\n",arg);


    *ppos=len;
    //sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, arg[0], arg[1]);
    if (arg==1)  //power on
    {
        pr_info("ISP power_on\n");
        mt9m114_s_ctrl.func_tbl->sensor_power_up(&mt9m114_s_ctrl);
        //imx091_power_up((struct msm_camera_sensor_info *)&mt9m114_s_ctrl.sensordata, false);
    }else //power down 
    {
        pr_info("ISP power_down\n");
        //imx091_power_down((struct msm_camera_sensor_info *)&mt9m114_s_ctrl.sensordata, false);
        mt9m114_s_ctrl.func_tbl->sensor_power_down(&mt9m114_s_ctrl);
    }
    	return len;	/* the end */
}

static const struct file_operations dbg_i7002a_fw_in_isp_fops = {
	.open		= dbg_i7002a_fw_in_isp_open,
	.read		= dbg_i7002a_fw_in_isp_read,
};

static const struct file_operations dbg_i7002a_page_dump_fops = {
	.open		= dbg_i7002a_page_dump_open,
	.read		= dbg_i7002a_page_dump_read,
};

static const struct file_operations dbg_i7002a_bin_dump_fops = {
	.open		= dbg_i7002a_bin_dump_open,
	.read		= dbg_i7002a_bin_dump_read,
};

static const struct file_operations dbg_i7002a_bin_dump_dmem_fops = {
	.open		= dbg_i7002a_bin_dump_dmem_open,
	.read		= dbg_i7002a_bin_dump_dmem_read,
};

static const struct file_operations dbg_fw_update_fops = {
	.open		= dbg_fw_update_open,
	.read		= dbg_fw_update_read,
	.write = dbg_fw_update_write,
};

static const struct file_operations i2c_set_fops = {
	.open		= i2c_set_open,
	//.read		= i2c_config_read,
	//.llseek		= seq_lseek,
	//.release	= single_release,
	.write = i2c_set_write,
};

static const struct file_operations i2c_get_fops = {
	.open		= i2c_get_open,
	.read		= i2c_get_read,
	//.llseek		= seq_lseek,
	//.release	= single_release,
	.write = i2c_get_write,
};

static const struct file_operations dbg_iCatch7002a_vga_status_fops = {
	.open		= dbg_iCatch7002a_vga_status_open,
	.read		= dbg_iCatch7002a_vga_status_read,
};

static const struct file_operations dbg_iCatch7002a_camera_status_fops = {
	.open		= dbg_iCatch7002a_camera_status_open,
	.read		= dbg_iCatch7002a_camera_status_read,
};

static const struct file_operations dbg_i7002a_fw_header_dump_fops = {
	.open		= dbg_i7002a_fw_header_dump_open,
	.read		= dbg_i7002a_fw_header_dump_read,
};

static const struct file_operations iCatch7002a_power_fops = {
	.open		= dbg_iCatch7002a_chip_power_open,
	//.read		= i2c_get_read,
	//.llseek		= seq_lseek,
	//.release	= single_release,
	.write = dbg_iCatch7002a_chip_power_write,
};
//ASUS_BSP+++ jim3_lin "Add for ATD CameraTest"
#define RX_BUF_SIZE 256
char SpiRxBuf[RX_BUF_SIZE];
static char *rxbuf = NULL;
static int rxsize = 0;

static ssize_t Calibration_spi_open(struct inode *inode, struct file *file)
{
    file->private_data = inode->i_private;
    return 0;
}

static ssize_t Calibration_spi_read(struct file *file, char __user *buf, size_t count,
                loff_t *ppos)
{
    unsigned int p=*ppos;
    int rc=0;
        if (p){
        printk("ppos is %02x\n",p);
        return 0;   /* the end */
        }
    if((rxbuf == NULL)||(rxsize == 0)){
        printk("Have no receive any buf\n");
        return 0;
    }

    if ((rc=copy_to_user(buf, rxbuf, rxsize))!=0){
        printk("copy to user fail rc = %d \n",rc);
        return -EFAULT;
    }

    *ppos += rxsize;
    /*
    for(rc=0;rc<rxsize;rc++)
        printk("rxbuf[%d]=0x%02x\n",rc,rxbuf[rc]);
    */
    return rxsize;

}
static ssize_t Calibration_spi_write(struct file *file, const char __user *buf, size_t count,
                loff_t *ppos)
{
  int len,rc;
  char *rx_buf;


  unsigned int rx_size;
        if (*ppos)
        return 0;   /* the end */

//+ parsing......
  len=(count > DBG_TXT_BUF_SIZE-1)?(DBG_TXT_BUF_SIZE-1):(count);
  if (copy_from_user(SpiRxBuf,buf,len))
        return -EFAULT;
    //printk("copy from user : %s, len is %d\n",SpiRxBuf,len);
  SpiRxBuf[len]=0; //add string end

  sscanf(SpiRxBuf, "%d",&rx_size);

  if (rxbuf)
  {
      kfree(rxbuf);
  }

  if((rx_buf = kmalloc(rx_size,GFP_KERNEL)) == NULL){
    printk("requre rx buf fail\n");
    return -EFAULT;
  }
  memset(rx_buf, 0 , rx_size);
  rxbuf = rx_buf;
  rxsize = rx_size;
  *ppos=len;
  rc = spi_rx_transfer(g_spi, rx_buf, rxsize);

  if(rc < 0)
    printk("spi rx fail : rc = %d\n",rc);


  return len;   /* the end */
}
static const struct file_operations Calibration_spi_fops = {
    .open       = Calibration_spi_open,
    .read       = Calibration_spi_read,
    .write      = Calibration_spi_write,
};
//ASUS_BSP--- jim3_lin "Add for ATD CameraTest"

int icatch_i2c_debuginit(void)
{
       struct dentry *dent = debugfs_create_dir("i7002a", NULL);

	(void) debugfs_create_file("fw_in_isp", S_IRWXUGO,
					dent, NULL, &dbg_i7002a_fw_in_isp_fops);

	(void) debugfs_create_file("page_dump", S_IRWXUGO,
					dent, NULL, &dbg_i7002a_page_dump_fops);

	(void) debugfs_create_file("bin_dump", S_IRWXUGO,
					dent, NULL, &dbg_i7002a_bin_dump_fops);

    	(void) debugfs_create_file("bin_dump_dmem", S_IRWXUGO,
					dent, NULL, &dbg_i7002a_bin_dump_dmem_fops);
        
	(void) debugfs_create_file("fw_header_dump", S_IRWXUGO,
					dent, NULL, &dbg_i7002a_fw_header_dump_fops);

	(void) debugfs_create_file("fw_update", S_IRWXUGO,
					dent, NULL, &dbg_fw_update_fops);

	(void) debugfs_create_file("i2c_set", S_IRWXUGO,
					dent, NULL, &i2c_set_fops);

	debugfs_create_u32("b_fw_is_valid", S_IRWXUGO, dent, &b_fw_is_valid);

	(void) debugfs_create_file("i2c_get", S_IRWXUGO,
					dent, NULL, &i2c_get_fops);
	(void) debugfs_create_file("camera_status", S_IRWXUGO, dent, NULL, &dbg_iCatch7002a_camera_status_fops);
	(void) debugfs_create_file("vga_status", S_IRWXUGO, dent, NULL, &dbg_iCatch7002a_vga_status_fops);
	(void) debugfs_create_file("iCatch_chip_power", S_IRWXUGO, dent, NULL, &iCatch7002a_power_fops);
    (void) debugfs_create_file("Calibration_spi_rx",S_IRWXUGO, dent,NULL, &Calibration_spi_fops);//ASUS_BSP jim3_lin "Add for ATD CameraTest"

//ASUS_BSP +++ Stimber "Implement the interface for calibration"
	if (debugfs_create_u32("is_calibration", S_IRWXUGO, dent, &is_calibration) == NULL) 
	{			
		printk(KERN_ERR "%s(%d): debugfs_create_u32: debug fail\n",	__FILE__, __LINE__);
		return -1;		
	}	
	
	if (debugfs_create_u32("calibrating", S_IRWXUGO, dent, &calibrating) == NULL) 
	{			
		printk(KERN_ERR "%s(%d): debugfs_create_u32: debug fail\n", __FILE__, __LINE__);
		return -1;		
	}
//ASUS_BSP --- Stimber "Implement the interface for calibration"
//ASUS_BSP +++ Stimber "Interface for single image"
	if (debugfs_create_u32("single_image", S_IRWXUGO, dent, &single_image) == NULL) 
	{			
		printk(KERN_ERR "%s(%d): debugfs_create_u32: debug fail\n", __FILE__, __LINE__);
		return -1;		
	}
//ASUS_BSP --- Stimber "Interface for single image"

#ifdef ICATCH7002A_DELAY_TEST
              if (debugfs_create_u32("iCatch7002a_delay", S_IRWXUGO, dent, &iCatch7002a_init_delay)
		== NULL) {
                printk(KERN_ERR "%s(%d): debugfs_create_u32: debug fail\n",
		__FILE__, __LINE__);
                return -1;
              }
	if (debugfs_create_u32("iCatch7002a_preview_delay", S_IRWXUGO, dent, &iCatch7002a_preview_delay)
		== NULL) {
                printk(KERN_ERR "%s(%d): debugfs_create_u32: debug fail\n",
		__FILE__, __LINE__);
                return -1;
              }
              if (debugfs_create_u32("touch_focus_enable", S_IRWXUGO, dent, &touch_focus_enable)
		== NULL) {
                printk(KERN_ERR "%s(%d): debugfs_create_u32: debug fail\n",
		__FILE__, __LINE__);
                return -1;
              }
#endif
             debugfs_create_u32("page_index",S_IRWXUGO, dent, &dbg_i7002a_page_index);
#ifdef CAM_TWO_MODE
             debugfs_create_u32("div",S_IRWXUGO, dent, &g_div);
	return 0;
#endif
}

#endif

//ASUS_BSP +++ LiJen "[A68][ISP][NA][Others]add proc file for AP ISP update"
// create proc file
#ifdef	CONFIG_PROC_FS
#define	ICATCH_PROC_FILE	"driver/iCatch"
static struct proc_dir_entry *iCatch_proc_file;

#define	ICATCH_FIRMWARE_VERSION_PROC_FILE	"driver/isp_fw_version"
static struct proc_dir_entry *iCatch_fw_version_proc_file;

static ssize_t iCatch_fw_version_proc_read(char *page, char **start, off_t off, int count,
            	int *eof, void *data)
{
	int len=0;

       if(g_ASUS_hwID < A91_SR1){ 
            //do nothimh
       }else{
            version_num_in_isp = get_fw_version_in_bin(BIN_FILE_WITH_PATH_A91);
       }
       
	if(*eof == 0){
		if(count>8) {
                     if(g_ASUS_hwID < A91_SR1){ 
                          len+=sprintf(page+len, "%x\n", version_num_in_isp);
                     }else{
                          len+=sprintf(page+len, "%x\n", get_fw_version_in_bin(BIN_FILE_WITH_PATH_A91));
                     }
			pr_info("%s:X string=%s", __func__, (char *)page);
		} else {
			len=-1;
		}
		*eof = 1;
	}
    
	return len;
}

static ssize_t iCatch_fw_version_proc_write(struct file *filp, const char __user *buff,
	            unsigned long len, void *data)
{
	pr_info("%s\n",__func__);
	return len;
}

static ssize_t iCatch_proc_read(char *page, char **start, off_t off, int count, 
            	int *eof, void *data)
{
	int len=0;

	if(*eof == 0){
		u32 BinVersionNum = 0;
              u32 ISPVersionNum = 0;
              
		if(g_ASUS_hwID < A91_SR1){ 
                     BinVersionNum = get_fw_version_in_bin(BIN_FILE_WITH_PATH);
			ISPVersionNum = version_num_in_isp;
			pr_info("ISPVersionNum=%x, BinVersionNum=%x--\n", ISPVersionNum, BinVersionNum);
			if((ISPVersionNum==0xffffff)||(ISPVersionNum<BinVersionNum)) {
				len+=sprintf(page+len, "%x\n%x\n%x\n", NEED_UPDATE, ISPVersionNum, BinVersionNum);
				//len+=sprintf(page+len, "%x\n", NEED_UPDATE);
			}else{
				len+=sprintf(page+len, "%x\n%x\n%x\n", UPDATE_UNNECESSARY, ISPVersionNum, BinVersionNum);
				//len+=sprintf(page+len, "%x\n", UPDATE_UNNECESSARY);
			}
		}else{
		       BinVersionNum = get_fw_version_in_bin(BIN_FILE_WITH_PATH_A91);
			ISPVersionNum = BinVersionNum;
			len+=sprintf(page+len, "%x\n%x\n%x\n", UPDATE_UNNECESSARY, ISPVersionNum, BinVersionNum);
		}
		*eof = 1;
		pr_info("%s:X string=%s", __func__, (char *)page);
	}
	return len;
}

static int setuid_asus(uid_t uid, uid_t *old_uid)
{
    const struct cred *old;
    struct cred *new;
    int retval;

    new = prepare_creds();
    if (!new)
        return -ENOMEM;
    old = current_cred();

    retval = -EPERM;
#if 0
    pr_info("%s %u,%u,%u\n",__func__,uid,old->uid,new->suid);
    if (uid == old->uid || uid == new->suid) {
        pr_err("%s fail1\n",__func__);
        goto error;
    }
#endif

    if (NULL != old_uid)
        *old_uid = old->euid;

    new->fsuid = new->euid = uid;
    new->fsgid = new->egid = uid;

    retval = security_task_fix_setuid(new, old, LSM_SETID_ID);
    if (retval < 0){
        goto error;
    }

    return commit_creds(new);

error:
    abort_creds(new);
    pr_info("%s retval(%d)\n",__func__,retval);
    return retval;
}


int cp(const char *to, const char *from)
{
    int fd_to, fd_from;
    char *pbuf;
    int bootbin_size = 1024*1024;
    mm_segment_t old_fs;
    ssize_t nread;
    uid_t old_uid;

    if(to==NULL || from==NULL){
        pr_err("copy file == NULL");
        return -1;
    }else{
        pr_info("copy %s to %s\n", from, to);
    }

    old_fs = get_fs();
    set_fs(KERNEL_DS);

    setuid_asus(0, &old_uid);
    
    fd_from = sys_open(from, O_RDONLY, 0);
    if (fd_from < 0){
        pr_err("open %s fail\n",from);
        return -1;
    }

    pbuf = vmalloc(bootbin_size);

    fd_to = sys_open(to, O_WRONLY | O_CREAT, 0644);
    if (fd_to < 0){
        pr_err("open %s fail\n",to);
        goto out_error;
    }
    
    while (nread = sys_read(fd_from, pbuf, bootbin_size), nread > 0)
    {
        char *out_ptr = pbuf;
        ssize_t nwritten;

        do {
            nwritten = sys_write(fd_to, out_ptr, nread);

            if (nwritten >= 0)
            {
                nread -= nwritten;
                out_ptr += nwritten;
            }
            else
            {
                goto out_error;
            }
        } while (nread > 0);
    }

    if (nread == 0)
    {
        if (sys_close(fd_to) < 0)
        {
            fd_to = -1;
            goto out_error;
        }
        set_fs(old_fs);
        setuid_asus(old_uid, NULL);
        sys_close(fd_from);
        vfree(pbuf);
        /* Success! */
        return 0;
    }

  out_error:
    sys_close(fd_from);

    if (fd_to >= 0)
        sys_close(fd_to);

    set_fs(old_fs);
    setuid_asus(old_uid, NULL);
    vfree(pbuf);
    return -1;
}

static ssize_t iCatch_proc_write(struct file *filp, const char __user *buff, 
	            unsigned long len, void *data)
{
	static char messages[256]="";
       //static char cmd[256]="";

	if (len > 256)
		len = 256;

	memset(messages, 0, 256);
	if (copy_from_user(messages, buff, len))
		return -EFAULT;
        
	pr_info("%s %s\n", __func__, messages);
        
    	if (strlen(messages)<=0) {
    	     pr_info("command not support\n");
    	} else {
    	       if('d' == messages[0]){
                        if('1' == messages[1]){
                            g_enable_roi_debug = true;
                            pr_info("%s Enable ROI debug\n",__func__);
                        }else if('0' == messages[1]){
                            g_enable_roi_debug = false;
                            pr_info("%s Disable ROI debug\n",__func__);
                        }
    	       }else{	
            		struct file *fp = NULL;
            		int str_len = strlen(messages);
            		messages[str_len-1] = 0;

            		if(iCatch_is_updating==0) {
            			iCatch_is_updating=1;
                            if(g_ASUS_hwID < A91_SR1){
                			pr_info("test filp_open %s--\n", messages);
                			fp = filp_open(messages, O_RDONLY, S_IRWXU|S_IRWXG|S_IRWXO);
                			if ( !IS_ERR_OR_NULL(fp) ){
                				UPDATE_FILE_WITH_PATH = messages;
                				filp_close(fp, NULL);
                			} else {
                				pr_info("choose system BOOT.BIN\n");
                				UPDATE_FILE_WITH_PATH = BIN_FILE_WITH_PATH;
                			}
                			
                			/* Update ISP firmware*/
                                g_ISPbootup = true;
                                mt9m114_s_ctrl.func_tbl->sensor_power_up(&mt9m114_s_ctrl);
                                BB_WrSPIFlash(UPDATE_FILE_WITH_PATH);
                                mt9m114_s_ctrl.func_tbl->sensor_power_down(&mt9m114_s_ctrl);
                                iCatch_is_updating=0;
                                g_ISPbootup = false;
                           }else{
                                if( ('f' == messages[0])){ //CMD: echo f > /proc/driver/iCatch
                                    fw_update_status = ICATCH_FW_UPDATE_SUCCESS;
                                    g_is_fw_loaded = false;
                                    g_is_fw_back_cal_loaded = false;
                                    g_is_fw_front_cal_loaded = false;    
                                }else{ //CMD: echo /data/.tmp/BOOT.BIN > /proc/driver/iCatch
                                    if(cp(BIN_FILE_WITH_PATH_A91, messages)<0){
                                        fw_update_status = ICATCH_FW_UPDATE_FAILED;
                                    }else{
                                        fw_update_status = ICATCH_FW_UPDATE_SUCCESS;
                                        g_is_fw_loaded = false;
                                        g_is_fw_back_cal_loaded = false;
                                        g_is_fw_front_cal_loaded = false;                                    
                                    }
                                }
                                writeUpdateProgresstoFile(0,0);
                                writeUpdateResultFile();
                                iCatch_is_updating=0;
                           }
            		} 
    	       }
    	}

	return len;
}

void create_iCatch_proc_file(void)
{
    iCatch_proc_file = create_proc_entry(ICATCH_PROC_FILE, 0666, NULL);
    if (iCatch_proc_file) {
		iCatch_proc_file->read_proc = iCatch_proc_read;
		iCatch_proc_file->write_proc = iCatch_proc_write;
    } else{
        pr_err("proc file create failed!\n");
    }

    iCatch_fw_version_proc_file = create_proc_entry(ICATCH_FIRMWARE_VERSION_PROC_FILE, 0666, NULL);
    if (iCatch_fw_version_proc_file) {
		iCatch_fw_version_proc_file->read_proc = iCatch_fw_version_proc_read;
		iCatch_fw_version_proc_file->write_proc = iCatch_fw_version_proc_write;
    } else{
        pr_err("proc file iCatch fw version create failed!\n");
    }

    icatch_i2c_debuginit();
}

void remove_iCatch_proc_file(void)
{
    extern struct proc_dir_entry proc_root;
    pr_info("%s\n",__func__);
    remove_proc_entry(ICATCH_PROC_FILE, &proc_root);
    remove_proc_entry(ICATCH_FIRMWARE_VERSION_PROC_FILE, &proc_root);
}
#endif // end of CONFIG_PROC_FS
//ASUS_BSP --- LiJen "[A68][ISP][NA][Others]add proc file for AP ISP update"


//ASUS_BSP +++ Stimber "Implement EXIF info for camera with ISP"
void iCatch_function_control(bool is_edge, bool is_preExif, bool is_Yavg, bool is_scene_info)
{
       u16 read_byte = 0, write_byte = 0;

        sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x729b, &read_byte);

	 if(is_edge){
	     write_byte = read_byte | 0x02;
	 }else{
	     write_byte = read_byte & (~0x02);
	 }
     
	 if(is_Yavg){
	     write_byte = write_byte | 0x10;
	 }else{
	     write_byte = write_byte & (~0x10);
	 }        

	 if(is_scene_info){
	     write_byte = write_byte | 0x20;
	 }else{
	     write_byte = write_byte & (~0x20);
	 }

	 if(is_preExif){
	     write_byte = write_byte | 0x04;
	 }else{
	     write_byte = write_byte & (~0x04);
	 }
     
     //ASUS_BSP+++ jim3_lin "Implement DIT postprocess-Mode2"
     //write_byte = write_byte | 0x40; //ASUS_BSP jim3_lin "Implement DIT postprocess-Enable AutoNight"
     //ASUS_BSP--- jim3_lin "Implement DIT postprocess-Mode2"
     
     sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x711b, write_byte);       
}

//ASUS_BSP+++ jim3_lin "Implement DIT postprocess-Mode2"
void iCatch_enable_autonight(bool enable)
{
     u16 read_byte = 0, write_byte = 0;
     pr_info("%s %d\n", __func__, enable);
     sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x729b, &read_byte);

     if(enable){
         write_byte = write_byte | 0x40;
     }else{
         write_byte = write_byte & (~0x40);
     }        

     sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x711b, write_byte);
}
//ASUS_BSP--- jim3_lin "Implement DIT postprocess-Mode2"

void iCatch_enable_exif(bool enable, bool is_preExif)
{
#if 0 //iCatch Allen:default is set this
        if (enable) {
           /* AE/AWB lock */
    	    sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x71EB, 0x01);
           msleep(8);  //LiJen: wait for ISP ready      
        }
#endif        

        //enable edge, preExif, Y-average  and scene_info
        iCatch_function_control(enable,is_preExif,enable,enable);    
}

bool iCatch_exif_flow_control(void)
{
    //static int frame_count = 0; 
    int rc = false;

    if(++g_frame_count == FRAME_PERIOD){
        g_frame_count = 0;
        rc = true;
    }else{
        rc = false;
    }
    return rc;
}

void iCatch_get_exif(struct exif_cfg *exif)
{
	short iso = 0;
	u16 ISO_lowbyte = 0;
	u16 ISO_highbyte = 0;
	u16 et_numerator = 0;
	u16 et_denominator_byte1 = 1;
	u16 et_denominator_byte2 = 0;
	u16 et_denominator_byte3 = 0;
	u16 flash_mode;
       u32 edge = 0;
       u16 Yaverage=0;
       u16 scene_info=0;

       u32 exif_bytes;
       unsigned char exif_data[14];
       u16 read_byte = 0;

#ifndef ASUS_SHIP_BUILD	   
       unsigned char info_3a_data[24];
#endif

       if( !is_calibration && ( //ASUS_BSP jim3_lin "Add for ATD CameraTest"
          frontcam_capture_status||
          g_cur_res == MSM_SENSOR_RES_FULL ||
          g_cur_res == MSM_SENSOR_RES_10M ||
          g_cur_res == MSM_SENSOR_RES_FULL_SINGLE_CAPTURE ||
          g_cur_res == MSM_SENSOR_RES_FULL_BURST_CAPTURE ||
          g_cur_res == MSM_SENSOR_RES_10M_BURST_CAPTURE ||
          g_cur_res == MSM_SENSOR_RES_3M_BURST_CAPTURE ||
 //         (g_mi1040_power == true && g_cur_res ==MSM_SENSOR_RES_FULL) ||
          ((g_cur_res == MSM_SENSOR_RES_QTR || g_cur_res == MSM_SENSOR_RES_HYBRID )&& iCatch_exif_flow_control()))) {//ASUS_BSP jim3_lin "Add for ATD CameraTest"

           exif_bytes = 14; //0x72b0~0x72bd  //enable edge information

           //Get EXIF information from ISP
           sensor_read_reg_bytes(mt9m114_s_ctrl.sensor_i2c_client->client, 0x72b0, exif_data, exif_bytes);

    	    /* EXIF Exposure time */
           et_numerator = exif_data[0];
           et_denominator_byte1 = exif_data[1];
           et_denominator_byte2 = exif_data[2];
           et_denominator_byte3 = exif_data[3];

           /* EXIF ISO */
           ISO_lowbyte = exif_data[7];
           ISO_highbyte = exif_data[8];
           iso = (ISO_highbyte << 8) | ISO_lowbyte;

            /* EXIF Flash mode 
            bit0: flash need trigger info for ZSL
            bit1: flash mode for exif
            */
            //flash need trigger info for ZSL
            sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x72c3, &read_byte);	
            if(read_byte & 0x04){
                flash_mode = 1;
            }else{
                flash_mode = 0;
            }
            
            //flash mode for exif
            if(g_flash_mode == 0){
            	flash_mode = 0<<1 | flash_mode;
            }else if(g_flash_mode ==1){
            	flash_mode = 1<<1 | flash_mode;
            }else{
            	flash_mode = exif_data[9]<<1 | flash_mode;
            }

            /* EXIF Edge */
            edge = (exif_data[13]<<24)|(exif_data[12]<<16)|(exif_data[11]<<8)|exif_data[10];
            //pr_info("(0x%x)(0x%x)(0x%x)(0x%x)\n",exif_data[10],exif_data[11],exif_data[12],exif_data[13]);

            /* EXIF Yaverage */
            sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x72be, &read_byte);
            Yaverage = read_byte;
            
            /* EXIF Scene information */
            sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x72bf, &read_byte);
            scene_info = read_byte;

            exif->iso = iso;
            exif->exp_time_num = et_numerator;
            exif->exp_time_denom = (et_denominator_byte3 << 16)|(et_denominator_byte2 << 8)|et_denominator_byte1;
            exif->flash_mode = flash_mode;
            exif->edge = edge;
            exif->Yaverage = Yaverage;
            exif->scene_info = scene_info;

#ifndef ASUS_SHIP_BUILD
            exif_bytes = 24;
            sensor_read_reg_bytes(mt9m114_s_ctrl.sensor_i2c_client->client, 0x72d8, info_3a_data, exif_bytes); 
            memcpy(exif->info_3a, info_3a_data, 24);
#endif
	
            if((g_cur_res != MSM_SENSOR_RES_QTR)||frontcam_capture_status){
		exif_debug++;
		if(exif_debug%100==0)
        	  pr_info("[EXIF] ISO(%d), ET(%d/%d), FLASH(%d), EDGE(%d), Yaverage(%d), Scene_info(%d)\n", exif->iso, exif->exp_time_num, exif->exp_time_denom, exif->flash_mode,exif->edge,exif->Yaverage,exif->scene_info);
		  exif_debug=1;
            }
	     frontcam_capture_status =false;		
            memcpy(&g_JpegExif, exif, sizeof(struct exif_cfg));
       }else{
            //pr_info("%s ignore\n",__func__);
            memcpy(exif, &g_JpegExif, sizeof(struct exif_cfg));
            return;
      }
}
//ASUS_BSP --- Stimber "Implement EXIF info for camera with ISP"

//ASUS_BSP +++ Stimber "Implement ISP settings - EV mode"	
void iCatch_set_ev_mode(int16_t mode)
{
	uint16_t ev_val =0;
	pr_info("%s +++ mode(%d)\n",__func__,mode);

	if(mode>=-12 && mode <=12){	 // AP: -2EV=-12, 0EV=0, +2EV=12
		ev_val = 6-(mode/2);		        //ISP: -2EV=12, 0EV=6, +2EV=0
		sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7103, ev_val);
		//ASUS_BSP : darrency_lin ++ fix ae setting error when take picture in lock ae/af/awb
		cur_ev = ev_val;
		//ASUS_BSP : darrency_lin -- fix ae setting error when take picture in lock ae/af/awb
	}else{
		pr_info("%s -- mode(%d) is not support \n",__func__,mode);
		return;
	}
			
	pr_info("%s --- ev_val(0x%x)\n",__func__,ev_val);
}
//ASUS_BSP --- Stimber "Implement ISP settings - EV mode"	

// ASUS_BSP +++ jim3_lin "Implement ISO/WB/Flicker/3A-Lock/LED mode for ISP"

// ASUS_BSP +++ jim3_lin "Implement ISP settings - ISO mode"
void iCatch_set_iso_mode(int16_t mode)
{
    pr_info("%s +++ mode(%d)\n",__func__,mode);
    switch(mode)
    {
        case CAM_ISO_MODE_AUTO: 
            // pr_info("jim CAM_ISO_MODE_AUTO");
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7110, 0x00);
            break;
        case CAM_ISO_MODE_50:   
            // pr_info("jim CAM_ISO_MODE_50");
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7110, 0x01);
            break;
        case CAM_ISO_MODE_100:
            // pr_info("jim CAM_ISO_MODE_100");
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7110, 0x02);
            break;
        case CAM_ISO_MODE_200:
            // pr_info("jim CAM_ISO_MODE_200");
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7110, 0x03);
            break;
        case CAM_ISO_MODE_400:
            // pr_info("jim CAM_ISO_MODE_400");
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7110, 0x04);
            break;
        case CAM_ISO_MODE_800:
            // pr_info("jim CAM_ISO_MODE_800");
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7110, 0x05);
            break;
        case CAM_ISO_MODE_1600:
            // pr_info("jim CAM_ISO_MODE_1600");
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7110, 0x06);
            break;
        default:
            pr_info("%s --- mode(%d) is not support \n",__func__,mode);
            return;
    }
    pr_info("%s ---\n",__func__);
}
// ASUS_BSP --- jim3_lin "Implement ISP settings - ISO mode"

// ASUS_BSP +++ jim3_lin "Implement ISP settings - WB mode"
void iCatch_set_wb_mode(int16_t mode)
{
        pr_info("%s +++ mode(%d)\n",__func__,mode);

        switch(mode)
        {
            case CAM_WB_MODE_AUTO: 
                // pr_info("jim CAM_WB_MODE_AUTO");
                sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x710A, 0x00);//auto
                break;
            case CAM_WB_MODE_INCANDESCENT:
                // pr_info("jim CAM_WB_MODE_INCANDESCENT");
                sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x710A, 0x06);//Incandescent
                break;
            case CAM_WB_MODE_DAYLIGHT:
                // pr_info("jim CAM_WB_MODE_DAYLIGHT");
                sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x710A, 0x01);//Daylight
                break;
            case CAM_WB_MODE_FLUORESCENT:
                // pr_info("jim CAM_WB_MODE_FLUORESCENT");
                sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x710A, 0x05);//Fluorescent_H
                break;
            case CAM_WB_MODE_CLOUDY_DAYLIGHT:
                // pr_info("jim CAM_WB_MODE_CLOUDY_DAYLIGHT");
                sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x710A, 0x02);//Cloudy
                break;
            case CAM_WB_MODE_SHADE:
                // pr_info("jim CAM_WB_MODE_SHADE");
                sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x710A, 0x03);//Shade
                break;
            default:
                pr_info("%s --- mode(%d) is not support \n",__func__,mode);
                return;
        }
        pr_info("%s ---\n",__func__);
}
// ASUS_BSP --- jim3_lin "Implement ISP settings - WB mode"

// ASUS_BSP +++ jim3_lin "Implement ISP settings - Flicker mode"
void iCatch_set_flicker_mode(int16_t mode)
{
    pr_info("%s +++ mode(%d)\n",__func__,mode);
    switch(mode)
    {
#if 0   // iCatch ISP doesn't support auto flicker
        case CAMERA_ANTIBANDING_AUTO:
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7101, 0x00);
            break;
#endif
        case CAM_ANTIBANDING_MODE_50HZ:
            // pr_info("jim CAM_ANTIBANDING_MODE_50HZ");
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7101, 0x01);
            break;
        case CAM_ANTIBANDING_MODE_60HZ:
            // pr_info("jim CAM_ANTIBANDING_MODE_60HZ");
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7101, 0x02);
            break;
        default:
            pr_info("%s --- mode(%d) is not support \n",__func__,mode);
            return;
    }
    pr_info("%s ---\n",__func__);
}
// ASUS_BSP --- jim3_lin "Implement ISP settings - Flicker mode"

// ASUS_BSP +++ jim3_lin "Implement ISP settings - AEC-Lock mode"
void iCatch_set_aeclock_mode(int16_t mode)
{
    pr_info("%s +++ mode(%d)\n",__func__,mode);
    switch(mode)
    {
        case 0: 
            // pr_info("jim acelock unlock");
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x71EB, 0x05);    //AE lock
            break;
        case 1: 
            // pr_info("jim acelock lock");
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x71EB, 0x03);    //AE unlock
            break;
        default:
            pr_info("%s --- mode(%d) is not support \n",__func__,mode);
            return;
    }
    pr_info("%s ---\n",__func__);
}
// ASUS_BSP --- jim3_lin "Implement ISP settings - AEC-Lock mode"

// ASUS_BSP +++ jim3_lin "Implement ISP settings - AWB-Lock mode"
void iCatch_set_awblock_mode(int16_t mode)
{
    pr_info("%s +++ mode(%d)\n",__func__,mode);
    switch(mode)
    {
        case 0: 
            // pr_info("jim awblock unlock");
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x71EB, 0x06);    //AWB unlock
            break;
        case 1: 
            // pr_info("jim awblock lock");
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x71EB, 0x04);    //AWB lock
            break;
        default:
            pr_info("%s --- mode(%d) is not support \n",__func__,mode);
            return;
    }
    pr_info("%s ---\n",__func__);
}
// ASUS_BSP --- jim3_lin "Implement ISP settings - AWB-Lock mode"

//ASUS_BSP+++ jim3_lin "Pass G sensor data to ISP"
void iCatch_set_sensor_val( uint32_t value ){
    int8_t val_x=0, val_y=0, val_z=0;
    pr_info("%s +++ mode(%d)\n",__func__,value);
    value = value % 1000000000;
    val_x = ( value / 1000000 )-128;
    value = value % 1000000;
    val_y = ( value / 1000 )-128;
    value = value % 1000;
    val_z = ( value )-128;

    pr_info("%s setGsensor(%d,%d,%d)\n",__func__, val_x, val_y, val_z);
    sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x00e0, val_x);
    sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x00e1, val_y);
    sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x1300, val_z);
    pr_info("%s ---\n",__func__);
}
//ASUS_BSP--- jim3_lin "Pass G sensor data to ISP"

// ASUS_BSP +++ jim3_lin "Implement ISP settings - LED mode"
void iCatch_set_led_mode(int16_t mode)
{
    pr_info("%s +++ mode(%d)\n",__func__,mode);
    switch(mode)
    {
        case CAM_FLASH_MODE_ON: 
            // pr_info("jim CAM_FLASH_MODE_ON");
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7104, 0x02);
            g_flash_mode = 1;
            break;
        case CAM_FLASH_MODE_OFF: 
            // pr_info("jim CAM_FLASH_MODE_OFF");
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7104, 0x01);
            g_flash_mode = 0;
            break;
        case CAM_FLASH_MODE_AUTO: 
            // pr_info("jim CAM_FLASH_MODE_AUTO");
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7104, 0x00);
            g_flash_mode = 2;
            break;
        case CAM_FLASH_MODE_TORCH: 
            // pr_info("jim CAM_FLASH_MODE_TORCH");
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7104, 0x04);
            g_flash_mode = 3;
            break;
        default:
            pr_info("%s --- mode(%d) is not support \n",__func__,mode);
            return;
    }
    pr_info("%s ---\n",__func__);
}
// ASUS_BSP --- jim3_lin "Implement ISP settings - LED mode"
// ASUS_BSP --- jim3_lin "Implement ISO/WB/Flicker/3A-Lock/LED mode for ISP"

// ASUS_BSP +++ jim3_lin "Implement Scene/Effect mode for ISP"
// ASUS_BSP +++ jim3_lin "Implement ISP settings - Scene mode"
void iCatch_set_scene_mode(int16_t mode)
{
    pr_info("%s +++ mode(%d)\n",__func__,mode);
       
    switch(mode)
    {
        case CAM_SCENE_MODE_AUTO:
        case CAM_SCENE_MODE_OFF:
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7109, 0x00);        
            break;  
        case CAM_SCENE_MODE_LANDSCAPE:
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7109, 0x06);
            break;
        case CAM_SCENE_MODE_SNOW:
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7109, 0x0B);
            break;
        case CAM_SCENE_MODE_SUNSET:
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7109, 0x0E);
            break;
        case CAM_SCENE_MODE_NIGHT:
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7109, 0x07);
            break;          
        case CAM_SCENE_MODE_PORTRAIT:
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7109, 0x0A);
            break;  
        case CAM_SCENE_MODE_BACKLIGHT:
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7109, 0x16);
            break;              
        case CAM_SCENE_MODE_SPORTS:
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7109, 0x0C);
            break;  
        case CAM_SCENE_MODE_FLOWERS: //mapping vivid in asusAP
            //sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7109, 0x13);
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7102, 0x05);  // set effect vivid
            break;          
        case CAM_SCENE_MODE_PARTY:
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7109, 0x09);
            break;  
        case CAM_SCENE_MODE_BEACH: 
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7109, 0x03);
            break;              
        case CAM_SCENE_MODE_ANTISHAKE:
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7109, 0x0D);
            break;              
        case CAM_SCENE_MODE_CANDLELIGHT:
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7109, 0x04);
            break;                  
        case CAM_SCENE_MODE_FIREWORKS:      
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7109, 0x05);
            break;                  
        case CAM_SCENE_MODE_NIGHT_PORTRAIT:   
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7109, 0x08);
            break;                  
        case CAM_SCENE_MODE_ACTION:
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7109, 0x01);
            break;               
        case CAM_SCENE_MODE_THEATRE:               
        case CAM_SCENE_MODE_AR:      
        default:
            pr_info("%s --- mode(%d) is not support \n",__func__,mode);
            return;
    }   
    pr_info("%s ---\n",__func__);
}
// ASUS_BSP --- jim3_lin "Implement ISP settings - Scene mode"
// ASUS_BSP +++ jim3_lin "Implement ISP settings - Effect mode"
void iCatch_set_effect_mode(int16_t mode)
{
    pr_info("%s +++ mode(%d)\n",__func__,mode);
       
    switch(mode)
    {
        case CAM_EFFECT_MODE_OFF: 
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7102, 0x00);
            break;
        case CAM_EFFECT_MODE_MONO:    
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7102, 0x04);
            break;
        case CAM_EFFECT_MODE_NEGATIVE:    
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7102, 0x02);
            break;
        case CAM_EFFECT_MODE_SEPIA:
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7102, 0x03);
            break;                
        case CAM_EFFECT_MODE_AQUA:
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7102, 0x01);
            break;                           
        //ASUS_BSP +++
        case CAM_EFFECT_MODE_AURA:
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7102, 0x06);
            break;                  
        case CAM_EFFECT_MODE_VINTAGE:
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7102, 0x07);
            break;                     
        case CAM_EFFECT_MODE_VINTAGE2:
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7102, 0x08);
            break;                     
        case CAM_EFFECT_MODE_LOMO:
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7102, 0x09);
            break;                     
        case CAM_EFFECT_MODE_RED:
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7102, 0x0A);
            break;                     
        case CAM_EFFECT_MODE_BLUE:
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7102, 0x0B);
            break;                     
        case CAM_EFFECT_MODE_GREEN:
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7102, 0x0C);
            break;                     
        //ASUS_BSP ---    
        case CAM_EFFECT_MODE_SOLARIZE:              
        case CAM_EFFECT_MODE_POSTERIZE:
        case CAM_EFFECT_MODE_WHITEBOARD:
        case CAM_EFFECT_MODE_BLACKBOARD:                
        case CAM_EFFECT_MODE_EMBOSS:
        case CAM_EFFECT_MODE_SKETCH:
        case CAM_EFFECT_MODE_NEON:                
        default:
            pr_info("%s --- mode(%d) is not support \n",__func__,mode);
            return;
    }
    pr_info("%s ---\n",__func__);
}
// ASUS_BSP --- jim3_lin "Implement ISP settings - Effect mode"
// ASUS_BSP --- jim3_lin "Implement Scene/Effect mode for ISP"

// ASUS_BSP +++ jim3_lin "Implement 3DNR-workaround/WDR/Aura mode for ISP"
// ASUS_BSP +++ jim3_lin "Implement ISP settings - WDR mode"
void iCatch_set_wdr_mode(int16_t mode)
{
    u16 read_byte = 0, write_byte = 0;
    pr_info("%s +++ mode(%d)\n",__func__,mode);

    sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x729b, &read_byte);
    //pr_info("read_byte(0x%x)",read_byte);
    if (mode) {
        write_byte = read_byte | 0x1;
    } else {
        write_byte = read_byte & (~0x1);
    }
    //pr_info("write_byte(0x%x)",write_byte);
    sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x711b, write_byte);
    pr_info("%s ---\n",__func__);
}
// ASUS_BSP --- jim3_lin "Implement ISP settings - WDR mode"

// ASUS_BSP +++ jim3_lin "Implement ISP settings - Aura mode"
void iCatch_set_aura_value(int16_t mode)
{
    pr_info("%s +++ mode(%d)\n",__func__,mode);
    sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7119, mode);
    pr_info("%s ---\n",__func__);
}
// ASUS_BSP --- jim3_lin "Implement ISP settings - Aura mode"
// ASUS_BSP --- jim3_lin "Implement 3DNR-workaround/WDR/Aura mode for ISP"

// ASUS_BSP +++ jim3_lin "Implement DIT postprocess"
void iCatch_set_ultrapixel(int16_t mode){
    pr_info("%s +++ mode(%d)\n",__func__,mode);
    if( mode == 1 ){
        sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7134, 0x01);   
    }
    else{
        sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7134, 0x00); 
    }
    pr_info("%s ---\n",__func__);
}
// ASUS_BSP --- jim3_lin "Implement DIT postprocess"

//ASUS_BSP +++ jim3_lin "Fix Scene/Effect/WB setting order"
void iCatch_set_wb_effect_scene( uint16_t mode ){
    uint16_t wb,effect,scene;
    int i=0;
    pr_info("%s +++ mode(%d)\n",__func__,mode);
    wb=mode/10000; effect=(mode%10000)/100; scene=mode%100;
    pr_info("%s wb:%d,effect:%d,scene:%d",__func__, wb,effect,scene);
    for( i=0;i<2;i++){
        if( i==0 ){
            if( wb == 0 ) iCatch_set_wb_mode(wb);
            if( effect == 0 ) iCatch_set_effect_mode(effect);
            if( scene == 0 ) iCatch_set_scene_mode(scene);
        } else{
            if( wb != 0 ) iCatch_set_wb_mode(wb);
            if( effect != 0 ) iCatch_set_effect_mode(effect);
            if( scene != 0 ) iCatch_set_scene_mode(scene);
        }
    }
    pr_info("%s ---\n",__func__);
}
//ASUS_BSP +++ jim3_lin "Fix Scene/Effect/WB setting order"

//ASUS_BSP +++ LiJen "[A68][13M][NA][Others]add 13M camera TAF support
void iCatch_set_touch_AF(cam_focus_mode_type mode, int16_t coordinate_x, int16_t coordinate_y, int16_t rectangle_h, int16_t rectangle_w, bool ROI_trigger){
	u32 af_w=0x80, af_h=0x80, af_x=0x33, af_y=0x33;
       const u32 roi_bytes = 11;
       unsigned char data[roi_bytes];    
	pr_info("%s +++\n",__func__);
	pr_info("%s: coordinate_x:0x%x coordinate_y:0x%x rectangle_h:0x%x rectangle_w:0x%x\n", __func__, coordinate_x, coordinate_y, rectangle_h, rectangle_w);

	// get preview resolution from ISP
	if(coordinate_x == -1){
		af_x = 0x80;  // ISP default
	}else if(coordinate_x > 0x0400){
		af_x = 0x0400;
	}else if(coordinate_x < 0){
		af_x = 0x0;
	}else{
		af_x = coordinate_x;
	}
	
	if(coordinate_y == -1){
		af_y = 0x80;  // ISP default
	}else if(coordinate_y > 0x0400){
		af_y = 0x0400;
	}else if(coordinate_y < 0){
		af_y = 0x0;
	}else{
		af_y = coordinate_y;
	}

	if(rectangle_w == -1){
		af_w = 0x33;  // ISP default
	}else if(rectangle_w > 0x0400){
		af_w = 0x0400;
	}else if(rectangle_w < 0){
		af_w = 0x0;
	}else{
		af_w = rectangle_w;
	}

	if(rectangle_h == -1){
		af_h = 0x33;  // ISP default
	}else if(rectangle_h > 0x0400){
		af_h = 0x0400;
	}else if(rectangle_h < 0){
		af_h = 0x0;
	}else{
		af_h = rectangle_h;
	}	

	pr_info("%s: af_x:0x%x af_y:0x%x af_w:0x%x af_h:0x%x g_TAEenable:%d\n", __func__, af_x, af_y, af_w, af_h,g_TAEenable);

       // set focus coodinate and retangle
       if(CAM_FOCUS_MODE_CONTINOUS_VIDEO == mode || CAM_FOCUS_MODE_CONTINOUS_PICTURE == mode){
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7105, 0x00); // iCatch:ROI only vaild in focus mode = auto
       }
       //AF ROI
       sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7188, 0x01);//ROI on
#if 0       
       sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7140, (af_w >> 8));
       sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7141, (af_w & 0xFF));
       sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7142, (af_x >> 8));
       sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7143, (af_x & 0xFF));
       sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7144, (af_y >> 8));
       sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7145, (af_y & 0xFF));
#else 
       //0x7140
       data[0] = (af_w >> 8);
       //0x7141
       data[1] = 0x41;
       data[2] = (af_w & 0xFF);
       //0x7142
       data[3] = 0x42;
       data[4] = (af_x >> 8);
       //0x7143
       data[5] = 0x43;
       data[6] = (af_x & 0xFF);
       //0x7144
       data[7] = 0x44;
       data[8] = (af_y >> 8);
       //0x7145
       data[9] = 0x45;
       data[10] = (af_y & 0xFF);
       sensor_write_reg_bytes(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7140, data, roi_bytes);
#endif
#if 0 //LiJen: remove asus_tae interface 
       //AE trigger
       if(g_TAEenable == true){
           sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x714e, 0x02);      
       }else{
           sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x714e, 0x00);//ASUS_BSP jim3_lin "Resume central average metering on SmartAF after TAE"
       }
#endif
       
       // AF trigger
       if(ROI_trigger){
           sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7146, 0x01);    
           sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x72a0, 0x01);     // Clean AF state bit
           sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x72a1, 0x01); 
       }
              
	pr_info("%s ---\n",__func__);
}

void iCatch_wait_AF_done(void){
        int retry=0;
        u16 status;
			
        //Wait AF interrupt
        g_isAFDone = false;
        do{
                mutex_unlock(mt9m114_s_ctrl.msm_sensor_mutex);
                if(retry==0){
                    msleep(120); // LiJen: wait for ISP AF process
                }else{
                    msleep(15);
                }
                mutex_lock(mt9m114_s_ctrl.msm_sensor_mutex);
                if(g_isAFCancel == true){
                    sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x714f, 0x01); 
                    pr_info("%s g_isAFCancel = ture\n",__func__);
                    break;
                }
                 sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x72a0, &status);
                 pr_info("status=0x%X, retry=%d\n",status,retry);
                 retry += 1;
        } while((status != 0x00) && (retry < 135));    
        g_isAFDone = true;
}

void iCatch_start_AF(bool on, cam_focus_mode_type mode, int16_t coordinate_x, int16_t coordinate_y, int16_t rectangle_h, int16_t rectangle_w)
{
	//int retry=0;	
       //u16 status;
	pr_info("%s +++ Param(%d,%d,%d,%d,%d,%d)\n",__func__,on,mode,coordinate_x,coordinate_y,rectangle_h,rectangle_w);
	g_isAFCancel = false;		
	//ASUS_BSP +++ LiJen "[A60K][8M][NA][Others]implement cancel autofocus in 8M camera with ISP"
	if(on){
            switch(mode) {
                case CAM_FOCUS_MODE_MACRO:
                    sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7105, 0x01); 
                    //Any point focus setting
        	      iCatch_set_touch_AF(mode, coordinate_x,coordinate_y,rectangle_h,rectangle_w, true);
                    break;
                case CAM_FOCUS_MODE_CONTINOUS_VIDEO:
                case CAM_FOCUS_MODE_CONTINOUS_PICTURE:
                    //Any point focus setting
        	      iCatch_set_touch_AF(mode, coordinate_x,coordinate_y,rectangle_h,rectangle_w, true);                    
                    sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7105, 0x03);  
                    break;                         
                case CAM_FOCUS_MODE_AUTO:
                    if(g_afmode == 0){
                        sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7105, 0x00); // auto af
                    }else{
                        sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7105, 0x04); // full search af
                    }
                    //Any point focus setting
        	      iCatch_set_touch_AF(mode, coordinate_x,coordinate_y,rectangle_h,rectangle_w, true);                    
                    //Wait AF done
                    //iCatch_wait_AF_done();   
                    break;                                              
                case CAM_FOCUS_MODE_FIXED: //LiJen: normal focus instead of full search af
                    sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7105, 0x04);   
                    //Any point focus setting
        	      iCatch_set_touch_AF(mode, coordinate_x,coordinate_y,rectangle_h,rectangle_w, true);                    
                    //Wait AF done
                    //iCatch_wait_AF_done();                        
                    break;                     
                case CAM_FOCUS_MODE_INFINITY:
                    sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7105, 0x02);  
                    goto end;                    
                case CAM_FOCUS_MODE_MAX:  
                default:
                    pr_info("%s mode(%d) is not support \n",__func__,mode);
                    goto end;          
            }

            //Enable ROI debug
            if(true == g_enable_roi_debug){
                if(CAM_FOCUS_MODE_AUTO != mode){
                    //Wait AF done
            	      //iCatch_wait_AF_done();         
                }
                sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x243f, 0x01);
            }                  
       
	}else{
		//Cancel AutoFocus
              //AF_START: AF release
              g_isAFCancel = true;
              //sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x714f, 0x01);  
		pr_info("Cancel autofocus\n");
	}
    
end:    
	pr_info("%s ---\n",__func__);
}

uint16_t iCatch_get_AF_result(struct msm_sensor_ctrl_t *s_ctrl)
{
	u16 afresult,afdone;
       enum sensor_af_t status;
	
//	pr_info("%s +++\n",__func__);
       g_isAFDone = false;
       
	if(!caf_mode){
		//Read AF result
		sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x72a0, &afdone);
		sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x72a1, &afresult);
//              pr_info("afdone(%d), afresult(%d)\n",afdone,afresult);
              if(afdone == 0x00){  //AF idle
                    if(afresult == 0x00){
                        status = SENSOR_AF_FOCUSSED;   //AF success
			     							pr_info("%s  AF success\n",__func__);
                    }else{
                        status = SENSOR_AF_NOT_FOCUSSED;   //AF fail
			     							pr_info("%s  AF fail\n",__func__);
				  }
				  AF_timecount = 0; //after afdone reset AF_timecount  ASUS BSP Ryan_Kuo
				  printk("%s(%d) AF done. \n", __func__, __LINE__);
              }else{    //AF busy
                    if(true == g_isAFCancel){
                        status = SENSOR_AF_CANCEL;
                    }else{
                    	AF_timecount++;//ASUS BSP Ryan_kuo+++
                      status = SENSOR_AF_SCANNING;
					if (AF_timecount >39){//ASUS BSP Ryan_kuo+++
						status = SENSOR_AF_NOT_FOCUSSED;
						AF_timecount = 0; //after AF timeout reset AF_timecount  ASUS_BSP  Bryant_Yu
						printk("%s(%d) AF timeout \n", __func__, __LINE__);
					}
				}
			}
	}
	else{
		status = SENSOR_AF_NOT_FOCUSSED;
		pr_err("CAF is starting\n");
	}

       if(status == SENSOR_AF_SCANNING){
            g_isAFDone = false;
       }else{
            g_isAFDone = true;
            //Enable ROI debug
            if(true == g_enable_roi_debug){
                sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x243f, 0x01);
            }              
       }
       
	//pr_info("%s status(%d)---\n",__func__,status);
	return status;
}
//ASUS_BSP --- LiJen "[A68][13M][NA][Others]add 13M camera TAF support

//ASUS_BSP +++ LiJen "[Camera][NA][Fix]implement TAE ROI"
void iCatch_set_touch_AE(int16_t coordinate_x, int16_t coordinate_y, int16_t rectangle_h, int16_t rectangle_w)
{
	u32 ae_w=0x80, ae_x=0x33, ae_y=0x33;
       const u32 roi_bytes = 11;
       unsigned char data[roi_bytes];    
	pr_info("%s +++\n",__func__);
       pr_info("%s: coordinate_x:0x%x coordinate_y:0x%x rectangle_h:0x%x rectangle_w:0x%x\n", __func__, coordinate_x, coordinate_y, rectangle_h, rectangle_w);
       
       if((coordinate_x == -1) && (coordinate_y == -1) && (rectangle_h == -1) && (rectangle_w == -1)){
           //Disable TAE
           sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x714E, 0x00);
       }else{
            // get preview resolution from ISP
            if(coordinate_x == -1){
                ae_x = 0x80;  // ISP default
            }else if(coordinate_x > 0x0400){
                ae_x = 0x0400;
            }else if(coordinate_x < 0){
                ae_x = 0x0;
            }else{
                ae_x = coordinate_x;
            }

            if(coordinate_y == -1){
                ae_y = 0x80;  // ISP default
            }else if(coordinate_y > 0x0400){
                ae_y = 0x0400;
            }else if(coordinate_y < 0){
                ae_y = 0x0;
            }else{
                ae_y = coordinate_y;
            }

            if(rectangle_w == -1){
                ae_w = 0x33;  // ISP default
            }else if(rectangle_w > 0x0400){
                ae_w = 0x0400;
            }else if(rectangle_w < 0){
                ae_w = 0x0;
            }else{
                ae_w = rectangle_w;
            }
       
       //AE ROI on
           sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7188, 0x01);

           //set TAE ROI
           //0x7148
           data[0] = (ae_w >> 8);
           //0x7149
           data[1] = 0x49;
           data[2] = (ae_w & 0xFF);
           //0x714A
           data[3] = 0x4A;
           data[4] = (ae_x >> 8);
           //0x714B
           data[5] = 0x4B;
           data[6] = (ae_x & 0xFF);
           //0x714C
           data[7] = 0x4C;
           data[8] = (ae_y >> 8);
           //0x714D
           data[9] = 0x4D;
           data[10] = (ae_y & 0xFF);
           sensor_write_reg_bytes(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7148, data, roi_bytes);     
           
           // AE Trigger
           sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x714E, 0x01);
       }
       pr_info("%s ---\n",__func__);
}
//ASUS_BSP --- LiJen "[Camera][NA][Fix]implement TAE ROI"

//ASUS_BSP +++ Stimber "implement hal general set command"
void iCatch_set_hal_general_cmd(struct kernel_mct_event_control_parm_t *hal_cmd)
{
	pr_info("%s +++ type = (%d)\n",__func__, (cam_intf_parm_type_t)hal_cmd->type);

	switch((cam_intf_parm_type_t)hal_cmd->type)
	{
		case CAM_INTF_PARM_EXPOSURE_COMPENSATION:{	//EV mode
			iCatch_set_ev_mode(*(int *)hal_cmd->parm_data);
		}
		break;
		/* Add newer hal parm below */

        // ASUS_BSP +++ jim3_lin "Implement ISO/WB/Flicker/3A-Lock/LED mode for ISP"
        case CAM_INTF_PARM_ISO:
            iCatch_set_iso_mode( *(int *)hal_cmd->parm_data );
            break;

        case CAM_INTF_PARM_WHITE_BALANCE:
            iCatch_set_wb_mode( *(int *)hal_cmd->parm_data ); // (enum config3a_wb_t)
            break;
        case CAM_INTF_PARM_ANTIBANDING:
            iCatch_set_flicker_mode( *(int *)hal_cmd->parm_data );
            break;

        case CAM_INTF_PARM_AEC_LOCK:
            iCatch_set_aeclock_mode( *(int *)hal_cmd->parm_data );
            break;
        case CAM_INTF_PARM_AWB_LOCK:
            iCatch_set_awblock_mode( *(int *)hal_cmd->parm_data );
            break;
        case CAM_INTF_PARM_LED_MODE:
            iCatch_set_led_mode( *(int *)hal_cmd->parm_data );
            break;
        // ASUS_BSP --- jim3_lin "Implement ISO/WB/Flicker/3A-Lock/LED mode for ISP"

        //ASUS_BSP+++ jim3_lin "Pass G sensor data to ISP"
        case ASUS_CAM_INTF_PARM_SET_SENSOR_VAL:
            iCatch_set_sensor_val( *(int *)hal_cmd->parm_data );
            break;
        //ASUS_BSP--- jim3_lin "Pass G sensor data to ISP"

        // ASUS_BSP +++ jim3_lin "Implement Scene/Effect mode for ISP"
        case CAM_INTF_PARM_BESTSHOT_MODE:
            iCatch_set_scene_mode( *(int *)hal_cmd->parm_data );
            break;
        case CAM_INTF_PARM_EFFECT:
            iCatch_set_effect_mode( *(int *)hal_cmd->parm_data );
            break;
        // ASUS_BSP --- jim3_lin "Implement Scene/Effect mode for ISP"

        // ASUS_BSP +++ jim3_lin "Implement 3DNR-workaround/WDR/Aura mode for ISP"
        case ASUS_CAM_INTF_PARM_WDR:
            iCatch_set_wdr_mode( *(int *)hal_cmd->parm_data );
            break;
        case ASUS_CAM_INTF_PARM_AURA_VALUE:
            iCatch_set_aura_value( *(int *)hal_cmd->parm_data );
            break;
        // ASUS_BSP --- jim3_lin "Implement 3DNR-workaround/WDR/Aura mode for ISP"

        // ASUS_BSP +++ jim3_lin "Implement DIT postprocess"
        case ASUS_CAM_INTF_PARM_ULTRAPIXEL:
            iCatch_set_ultrapixel( *(int *)hal_cmd->parm_data );
            break;
        // ASUS_BSP --- jim3_lin "Implement DIT postprocess"
        //ASUS_BSP +++ jim3_lin "Fix Scene/Effect/WB setting order"
        case ASUS_CAM_INTF_PARM_TRIGGER_EVENT:
            iCatch_set_wb_effect_scene( *(uint16_t *)hal_cmd->parm_data );
            break;
        //ASUS_BSP +++ jim3_lin "Fix Scene/Effect/WB setting order"
//ASUS_BSP +++ LiJen "[A68][Camera][NA][Fix]implement ROI autofocus"        
        case CAM_INTF_PARM_AF_ROI:
            memcpy(&g_AF_ROI, hal_cmd->parm_data, sizeof(cam_area_t));
            //pr_info("CAM_INTF_PARM_AF_ROI: %d,%d,%d,%d\n",g_AF_ROI.rect.left,g_AF_ROI.rect.top,g_AF_ROI.rect.height,g_AF_ROI.rect.width);
            break;                    
        case CAM_INTF_PARM_FOCUS_MODE:   
            iCatch_set_af_mode( *(uint16_t *)hal_cmd->parm_data );
            break;     
//ASUS_BSP --- LiJen "[A68][Camera][NA][Fix]implement ROI autofocus"    
//ASUS_BSP +++ LiJen "[Camera][NA][Fix]implement TAE ROI"        
        case CAM_INTF_PARM_AEC_ROI:
            memcpy(&g_AE_ROI, hal_cmd->parm_data, sizeof(cam_area_t));
            //pr_info("CAM_INTF_PARM_AE_ROI: %d,%d,%d,%d\n",g_AE_ROI.rect.left,g_AE_ROI.rect.top,g_AE_ROI.rect.height,g_AE_ROI.rect.width);
            iCatch_set_touch_AE(g_AE_ROI.rect.left, g_AE_ROI.rect.top, g_AE_ROI.rect.height, g_AE_ROI.rect.width);
            break;                        
//ASUS_BSP --- LiJen "[Camera][NA][Fix]implement TAE ROI"     
//ASUS_BSP +++ LiJen "Implement asus TAE mode"
        case ASUS_CAM_INTF_PARM_TAE:
            g_TAEenable = (*(int *)hal_cmd->parm_data);
            break;
//ASUS_BSP --- LiJen "Implement asus TAE mode"         

//ASUS_BSP +++ Stimber "Implement HDR/3D-NR features"       
		case ASUS_CAM_INTF_PARM_3DNR:
            g_is_nr_on = (*(int *)hal_cmd->parm_data);
            break;
        case ASUS_CAM_INTF_PARM_HDR:
            g_is_hdr_on = (*(int *)hal_cmd->parm_data);
            break;
//ASUS_BSP --- Stimber "Implement HDR/3D-NR features"
//ASUS_BSP +++ Stimber "Implement gyro detect mode"
        case ASUS_CAM_INTF_PARM_GYRO:
            iCatch_set_gyro_mode(*(uint8_t *)hal_cmd->parm_data);
            break;
//ASUS_BSP --- Stimber "Implement gyro detect mode"
//ASUS_BSP +++ bill_chen "Implement image stabilization"
        case ASUS_CAM_INTF_PARM_EIS:
            g_is_eis_on = (*(int *)hal_cmd->parm_data);
            if(g_is_eis_on)
            	iCatch_enable_autonight(false);
            else
            	iCatch_enable_autonight(true);				
            break;
//ASUS_BSP --- bill_chen "Implement image stabilization"
//ASUS_BSP +++ Stimber "Implement max frame rate mode"
        case ASUS_CAM_INTF_PARM_MAX_FPS:
            iCatch_set_max_fps_mode(*(uint8_t *)hal_cmd->parm_data);
            break;
//ASUS_BSP --- Stimber "Implement max frame rate mode"
        default:
            break;
	}
	pr_info("%s ---\n",__func__);
}
//ASUS_BSP --- Stimber "implement hal general set command"

void iCatch_set_af_mode(int16_t mode)
{
    pr_info("%s +++ mode(%d)\n",__func__,mode);

    g_AF_MODE=mode;
    caf_mode = false;
    switch(mode)
    {
        case CAM_FOCUS_MODE_AUTO: 
            break;
        case CAM_FOCUS_MODE_INFINITY: 
            iCatch_set_touch_AF(mode, 0x0, 0x0, 0x0, 0x0, false);  // ROI: reset as 0
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7105, 0x02);  
            break;
        case CAM_FOCUS_MODE_MACRO: 
            sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7105, 0x01); 
            break;
        case CAM_FOCUS_MODE_CONTINOUS_VIDEO: 
        case CAM_FOCUS_MODE_CONTINOUS_PICTURE: 
	    caf_mode = true;
           //iCatch_set_touch_AF(g_AF_MODE, 0x1cf, 0x1cf, 0x80, 0x80, false); // ROI: set center area
	    sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7105, 0x03); //CAF mode   
	    sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7146, 0x01); //AF trigger
            break; 
        case CAM_FOCUS_MODE_EDOF:             
        case CAM_FOCUS_MODE_FIXED:
        default:
            pr_info("%s --- mode(%d) is not support \n",__func__,mode);
            return;
    }
    pr_info("%s ---\n",__func__);  
}

//ASUS_BSP +++ Stimber "Implement gyro detect mode"
void iCatch_set_gyro_mode(uint8_t mode)
{
    pr_info("%s +++ mode(%d)\n",__func__,mode);

    if(mode == 1){    	// GRYO detect Moving
        setMaxExp(60);	//set Preview Max. Exposure Time as 1/60
        g_is_max_exp_on = true;
	}else{            	// GYRO detect Stop
        setMaxExp(0); 	//disable Max. Exposure Time function
        g_is_max_exp_on = false;
	}
    pr_info("%s ---\n",__func__);  
}
//ASUS_BSP --- Stimber "Implement gyro detect mode"

//ASUS_BSP +++ Stimber "Implement max frame rate mode"
void iCatch_set_max_fps_mode(uint8_t fps)
{
    pr_info("%s +++ mode(%d)\n",__func__,fps);
    setMaxFPS(fps);
    pr_info("%s ---\n",__func__);  
}
//ASUS_BSP --- Stimber "Implement max frame rate mode"

#if 0

//ASUS_BSP +++ LiJen "[A68][13M][NA][Others]implement CAF mode"
void iCatch_set_caf_mode(bool continuous)
{
	pr_info("%s +++ as %d\n",__func__,continuous);

	if(continuous){
	    caf_mode = true;
	    sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7105, 0x03); //CAF mode
	}
	else{
	    caf_mode = false;
	    sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7105, 0x00); // Auto mode
	}

	pr_info("%s ---\n",__func__);
}
//ASUS_BSP --- LiJen "[A68][13M][NA][Others]implement CAF mode"

//ASUS_BSP +++ LiJen "[A68][13M][NA][Others]implement general command"
void iCatch_set_general_cmd(struct general_cmd_cfg *cmd)
{
       u16 read_byte = 0, write_byte = 0;
	pr_info("%s +++ id(%d), value(%d)\n",__func__,cmd->cmd_id, cmd->cmd_value);
       
	switch(cmd->cmd_id)
	{
		case GENERAL_CMD_WDR:	
                    sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x729b, &read_byte);
                    //pr_info("read_byte(0x%x)",read_byte);
                     if (cmd->cmd_value) {
                        write_byte = read_byte | 0x1;
                     } else {
                        write_byte = read_byte & (~0x1);
                     }
                    //pr_info("write_byte(0x%x)",write_byte);
                    sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x711b, write_byte);
                    break;
		case GENERAL_CMD_HDR:	
			g_is_hdr_on = cmd->cmd_value;	//ASUS_BSP Stimber "Implement HDR feature"
                    break;                    
		case GENERAL_CMD_GYRO:	
                    if(cmd->cmd_value == 1){    // GRYO detect Moving
                        //sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7126, 0x01);
                        setMaxExp(60);//set Preview Max. Exposure Time as 1/60
					}else{                                  // GYRO detect Stop
                        //sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x7126, 0x00);
                        setMaxExp(0);//disable Max. Exposure Time function
					}
                    break;
		case GENERAL_CMD_TAE:	
                    if(cmd->cmd_value == 1){
                        g_TAEenable = 1;
                    }else{
                        g_TAEenable = 0;
                        sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x714e, 0x00);
                    }
                    break;  
		case GENERAL_CMD_FIX_FPS:	//ASUS_BSP Stimber "Implement fixed fps for video"
			if(cmd->cmd_value){
				pr_info("Fixed fps\n");
				setFixFPS(30);  //fix 30 fps
                setMaxExp(0);   //reset max exp.
                setMiniISO(0);  //reset min iso to default
			}else{
				pr_info("Dynamic fps\n");
				setFixFPS(0);   //reset fps fix
                setMaxExp(30);  //set max exp. to 1/30
                setMiniISO(0);  //reset min iso to default
			}
			break;
		default:
			pr_info("%s cmd_id(%d) is not support \n",__func__,cmd->cmd_id);
			break;
       }
       
	pr_info("%s ---\n",__func__);
}
//ASUS_BSP --- LiJen "[A68][13M][NA][Others]implement general command"

//ASUS_BSP +++ LiJen "[A68][13M][NA][Others]implement get general command"
void iCatch_get_general_cmd(struct general_cmd_cfg *cmd)
{
       u16 read_byte = 0;
	pr_info("%s +++ id(%d), value(%d)\n",__func__,cmd->cmd_id, cmd->cmd_value);
       
	switch(cmd->cmd_id)
	{
		case GENERAL_CMD_GET_FLASH_STATUS:	
                    sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x72c3, &read_byte);
                    if(read_byte & 0x04){   // flash status: flash on
                        pr_info("isp flash on\n");
                        cmd->cmd_value = 1;
                    }else{
                        pr_info("isp flash off\n");
                        cmd->cmd_value = 0;                        
                    }
                    break;                  
		default:
			pr_info("%s cmd_id(%d) is not support \n",__func__,cmd->cmd_id);
			break;
       }
       
	pr_info("%s ---\n",__func__);
}
//ASUS_BSP --- LiJen "[A68][13M][NA][Others]implement get general command"

void iCatch_checkAFMode(void)
{
    struct file *fp = NULL;

    fp = filp_open("/data/.tmp/fullsearch", O_RDONLY, S_IRWXU|S_IRWXG|S_IRWXO);
    if ( !IS_ERR_OR_NULL(fp) ){
        g_afmode = 1;
        filp_close(fp, NULL);
    } else {
        g_afmode = 0;
    }
    pr_info("%s g_afmode(%d)\n",__func__,g_afmode);
}
#endif 

void iCatch_release_sensor(void)
{
        int retry=0;
        u16 status;

//ASUS_BSP +++ PJ "[A91][Camera][NA][Others] wait DIT process AF release"
	//set AF status busy
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x72a7, 0x01);
	//set AF abort and go to infinity
	sensor_write_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x714f, 0x00);
	//polling AF status ilde
	pr_info("%s : [PJ] wait DIT process AF Start. \n",__func__);
	do{
		if(retry!=0)
	   		msleep(7);
		else
			msleep(20);
		
                 sensor_read_reg(mt9m114_s_ctrl.sensor_i2c_client->client, 0x72a7, &status);
 //                pr_info("status=0x%X, retry=%d\n",status,retry);
                 retry += 1;
        } while((status != 0x00) && (retry < 12)); 
	pr_info("%s : [PJ] wait DIT process AF End. \n",__func__);
	if(retry<10)
		pr_info("%s : [PJ] DIT process AF done success and retry = %d. \n",__func__,retry);
//ASUS_BSP --- PJ "[A91][Camera][NA][Others] wait DIT process AF release"	    
	g_isAFDone = true;//ASUS BSP Evan : fix sometimes release icath sensor faiL;
}

//ASUS_BSP +++ LiJen "[A86][Camera][NA][Others]add switch file for Camera FW update"
char CAMERA_VERSION_INFO_PATH[] = "camera";
struct switch_dev camera_fw_switch_dev;

static ssize_t camera_fw_switch_name(struct switch_dev *sdev, char *buf)
{
       if(g_ASUS_hwID < A91_SR1){ 
            return sprintf(buf, "%06x\n", version_num_in_isp);        
       }else{
            return sprintf(buf, "%06x\n", get_fw_version_in_bin(BIN_FILE_WITH_PATH_A91));
       }
}

static ssize_t camera_fw_switch_state(struct switch_dev *sdev, char *buf)
{        
	return sprintf(buf, "%s\n", "1");
}

void create_iCatch_switch_file(void)
{
       int ret = 0;
       
	camera_fw_switch_dev.name = CAMERA_VERSION_INFO_PATH;
	camera_fw_switch_dev.print_state = camera_fw_switch_state;
	camera_fw_switch_dev.print_name = camera_fw_switch_name;

       // registered switch device
       ret=switch_dev_register(&camera_fw_switch_dev);
	if (ret < 0){
		pr_err("%s result(%d)\n",__func__, ret);
	}
}
void remove_iCatch_switch_file(void)
{
	// unregistered switch device
	switch_dev_unregister(&camera_fw_switch_dev);
}
//ASUS_BSP --- LiJen "[A86][Camera][NA][Others]add switch file for Camera FW update"

void iCatch_init(void)
{
    //is_calibration = 0;
    single_image = 0;
    g_TAEenable = 0;
    g_is_hdr_on = 0;	//0: Disable, 1:Enable //ASUS_BSP stimber "Implement HDR feature"
    g_is_nr_on = 0;	    //0: Disable, 1:Enable //ASUS_BSP stimber "Implement NR feature"
    g_is_eis_on = 0;    //0: Disable, 1:Enable //ASUS_BSP bill_chen "Implement image stabilization"
    is_calibration_table_set = true;    
    g_isAFCancel = false;
    g_isAFDone = true;
    g_LastFixFPS = 0;
    g_LastMaxExp = 0;
    g_LastMiniISO = 0;
    g_LastVideoMode = 0;
    g_cur_res = MSM_SENSOR_INVALID_RES;
    g_pre_res = MSM_SENSOR_INVALID_RES;
    g_AF_MODE = CAM_FOCUS_MODE_AUTO;
    g_isISP_IRQ_eanble = false;
    g_is_max_exp_on = false;
    g_pbootBuf = NULL;
    g_is_fw_loaded = false;
    g_is_fw_back_cal_loaded = false;
    g_is_fw_front_cal_loaded = false;
    memset(&g_AF_ROI, 0, sizeof(cam_area_t));
    memset(&g_AE_ROI, 0, sizeof(cam_area_t));

    if(g_pbootBuf == NULL){
        g_pbootBuf = kmalloc(ISP_FW_SIZE, GFP_KERNEL);
    }  
    //create_iCatch_proc_file();
}

void iCatch_deinit(void)
{
    if(g_pbootBuf != NULL){
        kfree(g_pbootBuf);
        g_pbootBuf = NULL;
    }  
}

void iCatch_create_workqueue(void){
    INIT_WORK(&icatch_torch_on_wq, iCatch_torch_on_work);
    INIT_WORK(&icatch_torch_off_wq, iCatch_torch_off_work);
    icatch_torch_workqueue = create_singlethread_workqueue("iCatch_torch_wq");
}

void  iCatch_destroy_workqueue(void){
    destroy_workqueue(icatch_torch_workqueue);
}

int32_t led_pmic_flash_hw_strobe_enable(int enable)
{
    int rc=0;
    struct file *fp = NULL;
    mm_segment_t old_fs;
    loff_t offset = 0;
    char str[1];
    
    fp = filp_open(LED_PMIC_FLASH_STROBE_PATH, O_RDONLY, 0);
    if ( IS_ERR_OR_NULL(fp) ){
        filp_close(fp, NULL);
        printk("%s: open %s fail\n", __FUNCTION__, LED_PMIC_FLASH_STROBE_PATH);
    }
    old_fs = get_fs();
    set_fs(KERNEL_DS);
    offset = 0;
    if (fp->f_op != NULL && fp->f_op->write != NULL){
        sprintf(str, "%d\n", enable);
        fp->f_op->write(fp,
        str,
        strlen(str),
        &offset);
    }else{
        pr_err("%s: f_op might be null\n", __FUNCTION__);
        set_fs(old_fs);
        filp_close(fp, NULL);
    }          

    return rc;
}

int32_t led_pmic_torch_hw_strobe_enable(int enable)
{
    int rc=0;
    struct file *fp = NULL;
    mm_segment_t old_fs;
    loff_t offset = 0;
    char str[1];
    
    fp = filp_open(LED_PMIC_TORCH_STROBE_PATH, O_RDONLY, 0);
    if ( IS_ERR_OR_NULL(fp) ){
        filp_close(fp, NULL);
        printk("%s: open %s fail\n", __FUNCTION__, LED_PMIC_TORCH_STROBE_PATH);
    }
    old_fs = get_fs();
    set_fs(KERNEL_DS);
    offset = 0;
    if (fp->f_op != NULL && fp->f_op->write != NULL){
        sprintf(str, "%d\n", enable);
        fp->f_op->write(fp,
        str,
        strlen(str),
        &offset);
    }else{
        pr_err("%s: f_op might be null\n", __FUNCTION__);
        set_fs(old_fs);
        filp_close(fp, NULL);
    }          

    return rc;
}

int32_t led_pmic_flash_power_enable(int enable)
{
    int rc=0;
    struct file *fp = NULL;
    mm_segment_t old_fs;
    loff_t offset = 0;
    char str[1];
    
    fp = filp_open(LED_PMIC_FLASH_ENABLE_PATH, O_RDONLY, 0);
    if ( IS_ERR_OR_NULL(fp) ){
        filp_close(fp, NULL);
        printk("%s: open %s fail\n", __FUNCTION__, LED_PMIC_FLASH_ENABLE_PATH);
    }
    old_fs = get_fs();
    set_fs(KERNEL_DS);
    offset = 0;
    if (fp->f_op != NULL && fp->f_op->write != NULL){
        sprintf(str, "%d\n", enable);
        fp->f_op->write(fp,
        str,
        strlen(str),
        &offset);
    }else{
        pr_err("%s: f_op might be null\n", __FUNCTION__);
        set_fs(old_fs);
        filp_close(fp, NULL);
    }          

    return rc;
}

int32_t led_pmic_torch_power_enable(int enable)
{
    int rc=0;
    struct file *fp = NULL;
    mm_segment_t old_fs;
    loff_t offset = 0;
    char str[1];
    
    fp = filp_open(LED_PMIC_TORCH_ENABLE_PATH, O_RDONLY, 0);
    if ( IS_ERR_OR_NULL(fp) ){
        filp_close(fp, NULL);
        printk("%s: open %s fail\n", __FUNCTION__, LED_PMIC_TORCH_ENABLE_PATH);
    }
    old_fs = get_fs();
    set_fs(KERNEL_DS);
    offset = 0;
    if (fp->f_op != NULL && fp->f_op->write != NULL){
        sprintf(str, "%d\n", enable);
        fp->f_op->write(fp,
        str,
        strlen(str),
        &offset);
    }else{
        pr_err("%s: f_op might be null\n", __FUNCTION__);
        set_fs(old_fs);
        filp_close(fp, NULL);
    }          

    return rc;
}

int32_t led_pmic_flash_enable(int enable, int hw_strobe)
{
    int rc=0;
    pr_info("%s E (%d,%d)\n",__func__,enable,hw_strobe);
    if(g_ASUS_hwID < A91_SR1){
            //do nothing
    }else{
        if(enable){  //enable      
            led_pmic_flash_hw_strobe_enable(hw_strobe);//switch hw trigger 
            led_pmic_flash_power_enable(0);//need disable first               
            led_pmic_flash_power_enable(LED_PMIC_FLASH_MXA_BRIGHTNESS);  //max brightness
//ASUS_BSP: Darrency_lin ++ add delay for LED trigger timing issue at not enogut brightness in flash mode
	    msleep(6);
//ASUS_BSP: Darrency_lin  add delay for LED trigger timing issue at not enogut brightness in flash mode
            g_pmic_flash_status = PMIC_FLASH_STATUS_FLASH_ON;
        }else{ //disable
            led_pmic_flash_power_enable(0);
            g_pmic_flash_status = PMIC_FLASH_STATUS_FLASH_OFF;
        }
    }
    pr_info("%s X\n",__func__);
    return rc;
}

int32_t led_pmic_torch_enable(int enable, int hw_strobe)
{
    int rc=0;
    pr_info("%s E (%d,%d)\n",__func__,enable,hw_strobe);
    if(g_ASUS_hwID < A91_SR1){
            //do nothing
    }else{
        if(enable){  //enable
            led_pmic_torch_hw_strobe_enable(hw_strobe);//switch hw trigger
            led_pmic_flash_power_enable(0);//need disable first  
            msleep(5);
            led_pmic_torch_power_enable(LED_PMIC_TORCH_MXA_BRIGHTNESS);  //max brightness
        }else{ //disable
            led_pmic_torch_power_enable(0);
        }
    }
    pr_info("%s X\n",__func__);
    return rc;
}

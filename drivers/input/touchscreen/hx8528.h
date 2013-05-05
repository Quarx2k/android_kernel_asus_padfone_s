#ifndef _LINUX_HX8528_H
#define _LINUX_HX8528_H

//=============================================================================================================
//
//	Segment : Himax Define Options 
//
//=============================================================================================================
//TODO START : Select the function you need!
//------------------------------------------// Support Function Enable :
#define HX_TP_SYS_DIAG			// Support Sys : Diag function, default is open
#define HX_TP_SYS_REGISTER		// Support Sys : Register function, default is open
#define HX_TP_SYS_DEBUG_LEVEL		// Support Sys : Debug Level function, default is open
#define HX_TP_SYS_FLASH_DUMP		// Support Sys : Flash dump function, default is open
#define HX_TP_SYS_SELF_TEST		// Support Sys : Self Test Function, default is open
#define HX_TP_SYS_HITOUCH		// Support Sys : Hi-touch command, default is open
//#define HX_EN_SEL_BUTTON		// Support Self Virtual key,default is close
#define HX_EN_MUT_BUTTON		// Support Mutual Virtual Key,default is close
#define HX_RST_PIN_FUNC			// Support HW Reset, default is open
//#define HX_PORTING_DEB_MSG		// Support Driver Porting Message, default is close
//TODO END

//------------------------------------------// Support Different IC. Select one at one time.
#define HX_85XX_A_SERIES_PWON		1
#define HX_85XX_B_SERIES_PWON		2
#define HX_85XX_C_SERIES_PWON		3
#define HX_85XX_D_SERIES_PWON		4

//------------------------------------------// Supoort ESD Issue
#ifdef HX_RST_PIN_FUNC
#define ENABLE_CHIP_RESET_MACHINE	// Support Chip Reset Workqueue, default is open
#endif

#ifdef ENABLE_CHIP_RESET_MACHINE 
#define HX_TP_SYS_RESET			// Support Sys : HW Reset function, default is open
#endif

//------------------------------------------// Support FW Bin checksum method,mapping with Hitouch *.bin
#define HX_TP_BIN_CHECKSUM_SW		1
#define HX_TP_BIN_CHECKSUM_HW		2
#define HX_TP_BIN_CHECKSUM_CRC		3

//=============================================================================================================
//
//	Segment : Himax Define Variable
//
//=============================================================================================================
//TODO START : Modify follows deinfe variable
#define HX_KEY_MAX_COUNT             4			// Max virtual keys
#define DEFAULT_RETRY_CNT            3			// For I2C Retry count
//TODO END

//TODO START : Modify follows power gpio / interrupt gpio / reset gpio
//------------------------------------------// power supply , i2c , interrupt gpio
//#define HIMAX_PWR_GPIO		59
//#define HIMAX_INT_GPIO		60
//#define HIMAX_RST_GPIO		62
//TODO END

//TODO START : Modify the I2C address
//------------------------------------------// I2C
#define HIMAX_I2C_ADDR		0x49
#define HIMAX_TS_NAME		"himax-ts"
//TODO END

//------------------------------------------// Input Device
#define INPUT_DEV_NAME	"Himax-touchscreen"	

//------------------------------------------// Flash dump file
#define FLASH_DUMP_FILE "/sdcard/Flash_Dump.bin"

//------------------------------------------// Diag Coordinate dump file
#define DIAG_COORDINATE_FILE "/sdcard/Coordinate_Dump.txt"

//------------------------------------------// Virtual key
#define HX_VKEY_0   KEY_BACK
#define HX_VKEY_1   KEY_HOMEPAGE
#define HX_VKEY_2   KEY_MENU
#define HX_VKEY_3   104
#define HX_KEY_ARRAY    {HX_VKEY_0, HX_VKEY_1, HX_VKEY_2, HX_VKEY_3}

//------------------------------------------// Himax TP COMMANDS -> Do not modify the below definition
#define HX_CMD_NOP                   0x00   /* no operation */
#define HX_CMD_SETMICROOFF           0x35   /* set micro on */
#define HX_CMD_SETROMRDY             0x36   /* set flash ready */
#define HX_CMD_TSSLPIN               0x80   /* set sleep in */
#define HX_CMD_TSSLPOUT              0x81   /* set sleep out */
#define HX_CMD_TSSOFF                0x82   /* sense off */
#define HX_CMD_TSSON                 0x83   /* sense on */
#define HX_CMD_ROE                   0x85   /* read one event */
#define HX_CMD_RAE                   0x86   /* read all events */
#define HX_CMD_RLE                   0x87   /* read latest event */
#define HX_CMD_CLRES                 0x88   /* clear event stack */
#define HX_CMD_TSSWRESET             0x9E   /* TS software reset */
#define HX_CMD_SETDEEPSTB            0xD7   /* set deep sleep mode */
#define HX_CMD_SET_CACHE_FUN         0xDD   /* set cache function */
#define HX_CMD_SETIDLE               0xF2   /* set idle mode */
#define HX_CMD_SETIDLEDELAY          0xF3   /* set idle delay */
#define HX_CMD_SELFTEST_BUFFER       0x8D   /* Self-test return buffer */
#define HX_CMD_MANUALMODE            0x42
#define HX_CMD_FLASH_ENABLE          0x43
#define HX_CMD_FLASH_SET_ADDRESS     0x44
#define HX_CMD_FLASH_WRITE_REGISTER  0x45
#define HX_CMD_FLASH_SET_COMMAND     0x47
#define HX_CMD_FLASH_WRITE_BUFFER    0x48
#define HX_CMD_FLASH_PAGE_ERASE      0x4D
#define HX_CMD_FLASH_SECTOR_ERASE    0x4E
#define HX_CMD_CB                    0xCB
#define HX_CMD_EA                    0xEA
#define HX_CMD_4A                    0x4A
#define HX_CMD_4F                    0x4F
#define HX_CMD_B9                    0xB9
#define HX_CMD_76                    0x76

#define HIMAX_COORDS_ARR_SIZE		4
#define HIMAX_CHANNEL_ARR_SIZE		2

#ifdef ASUS_FACTORY_BUILD
#define HX_RW_ATTR (S_IRUGO|S_IWUGO)
#define HX_WO_ATTR (S_IWUGO)
#define HX_RO_ATTR (S_IRUGO)
#else
#define HX_RW_ATTR (S_IRUGO|S_IWUSR)
#define HX_WO_ATTR (S_IWUSR)
#define HX_RO_ATTR (S_IRUGO)
#endif


enum {
	DEF_POINT_INFO	= 1U << 0,
	ALL_POINT_INFO	= 1U << 1,
	DET_POINT_INFO	= 1U << 2,
	
	SUS_INFO	= 1U << 5,
};


//=============================================================================================================
//
//	Segment : Himax Include Header file / Data Structure
//
//=============================================================================================================

struct himax_i2c_platform_data
{
	uint16_t version;
	int abs_x_min;
	int abs_x_max;
	int abs_y_min;
	int abs_y_max;
	int intr_gpio;
	u32 irq_gpio_flags;
	int rst_gpio;
	u32 reset_gpio_flags;
	int x_channel;
	int y_channel;
	int pcb_id0;
	u32 pcb_id0_flags;
	int pcb_id1;
	u32 pcb_id1_flags;
};

struct himax_ts_data
{
	spinlock_t irq_lock;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct workqueue_struct *himax_wq;
	struct work_struct work;

 	struct workqueue_struct *usb_wq;
	struct work_struct usb_detect_work;
	int usb_status;

	int (*power)(int on);
	struct notifier_block fb_notif;
	int intr_gpio;
	// Firmware Information
	int fw_ver;
	int fw_id;
	int x_resolution;
	int y_resolution;
	// For Firmare Update 
	struct miscdevice firmware;
	struct attribute_group attrs;
	struct switch_dev touch_sdev;
	int abs_x_max;
	int abs_y_max;
	int rst_gpio;
	int init_success;
	int pcb_id0;
	int pcb_id1;
	bool irq_is_disable;
	struct regulator *vdd;

	// Wakelock Protect start
	struct wake_lock wake_lock;
	// Wakelock Protect end

	// Mutexlock Protect Start
	struct mutex mutex_lock;
	// Mutexlock Protect End

//----[HX_TP_SYS_FLASH_DUMP]--------------------------------------------------------------------------start
#ifdef HX_TP_SYS_FLASH_DUMP
	struct workqueue_struct *flash_wq;
	struct work_struct flash_work;
#endif
//----[HX_TP_SYS_FLASH_DUMP]----------------------------------------------------------------------------end

//----[ENABLE_CHIP_RESET_MACHINE]---------------------------------------------------------------------start
#ifdef ENABLE_CHIP_RESET_MACHINE
	struct delayed_work himax_chip_reset_work;
#endif
//----[ENABLE_CHIP_RESET_MACHINE]-----------------------------------------------------------------------end

};

static int i2c_himax_read(struct i2c_client *client, uint8_t command,
	uint8_t *data, uint8_t length, uint8_t toRetry);
static int i2c_himax_write(struct i2c_client *client, uint8_t command,
	uint8_t *data, uint8_t length, uint8_t toRetry);
static int i2c_himax_master_write(struct i2c_client *client, uint8_t *data,
	uint8_t length, uint8_t toRetry);
static int i2c_himax_write_command(struct i2c_client *client, uint8_t command,
	uint8_t toRetry);

static int himax_lock_flash(void);
static int himax_unlock_flash(void);

static int himax_hang_shaking(void); 					// Hand shaking function
static int himax_ts_poweron(struct himax_ts_data *ts_modify);		// Power on
static int himax_touch_sysfs_init(void);			// Sys filesystem initial
static void himax_touch_sysfs_deinit(void);			// Sys filesystem de-initial

#if defined(CONFIG_EEPROM_NUVOTON)
static struct notifier_block touch_mp_notifier;
static void attach_padstation_work(struct work_struct *work);
static void detach_padstation_work(struct work_struct *work);
static int touch_mp_event(struct notifier_block *this, unsigned long event, void *ptr);
#endif

void hx_irq_disable(struct himax_ts_data *ts);
void hx_irq_enable(struct himax_ts_data *ts);

static void himax_cable_status(struct work_struct *work);


//----[HX_LOADIN_CONFIG]--------------------------------------------------------------------------------start
#ifdef HX_LOADIN_CONFIG
unsigned char c1[] =
{ 0x37, 0xFF, 0x08, 0xFF, 0x08 };
unsigned char c2[] =
{ 0x3F, 0x00 };
unsigned char c3[] =
{ 0x62, 0x01, 0x00, 0x31, 0x04, 0x00, 0x00, 0x00, 0x00, 0x22, 0x22 };
unsigned char c4[] =
{ 0x63, 0x01, 0x00, 0x41, 0x03, 0x00, 0x00, 0x00, 0x00, 0x22, 0x22 };
unsigned char c5[] =
{ 0x64, 0x01, 0x00, 0x31, 0x04, 0x00, 0x00, 0x00, 0x00, 0x22, 0x22 };
unsigned char c6[] =
{ 0x65, 0x01, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x22, 0x12 };
unsigned char c7[] =
{ 0x66, 0x01, 0x00, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11, 0x11 };
unsigned char c8[] =
{ 0x67, 0x01, 0x00, 0x21, 0x03, 0x00, 0x00, 0x00, 0x00, 0x11, 0x11 };
unsigned char c9[] =
{ 0x68, 0x01, 0x00, 0x31, 0x02, 0x00, 0x00, 0x00, 0x00, 0x11, 0x11 };
unsigned char c10[] =
{ 0x69, 0x01, 0x00, 0x21, 0x03, 0x00, 0x00, 0x00, 0x00, 0x11, 0x11 };
unsigned char c11[] =
{ 0x6A, 0x01, 0x00, 0x31, 0x02, 0x00, 0x00, 0x00, 0x00, 0x11, 0x11 };
unsigned char c12[] =
{ 0x6B, 0x01, 0x00, 0x21, 0x03, 0x00, 0x00, 0x00, 0x00, 0x11, 0x11 };
unsigned char c13[] =
{ 0x6C, 0x01, 0x00, 0x31, 0x02, 0x00, 0x00, 0x00, 0x00, 0x11, 0x21 };
unsigned char c14[] =
{ 0x6D, 0x01, 0x00, 0x21, 0x03, 0x00, 0x00, 0x00, 0x00, 0x22, 0x22 };
unsigned char c15[] =
{ 0x6E, 0x01, 0x00, 0x31, 0x02, 0x00, 0x00, 0x00, 0x00, 0x22, 0x22 };
unsigned char c16[] =
{ 0x6F, 0x01, 0x00, 0x21, 0x03, 0x00, 0x00, 0x00, 0x00, 0x22, 0x22 };
unsigned char c17[] =
{ 0x70, 0x01, 0x00, 0x31, 0x02, 0x00, 0x00, 0x00, 0x00, 0x22, 0x22 };
unsigned char c18[] =
{ 0x7B, 0x03 };
unsigned char c19[] =
{ 0x7C, 0x00, 0xD8, 0x8C };
unsigned char c20[] =
{ 0x7F, 0x00, 0x04, 0x0A, 0x0A, 0x04, 0x00, 0x00, 0x00 };
unsigned char c21[] =
{ 0xA4, 0x94, 0x62, 0x94, 0x86 };
unsigned char c22[] =
{ 0xB4, 0x06, 0x01, 0x01, 0x01, 0x01, 0x03, 0x0C, 0x04, 0x07, 0x04, 0x07, 0x04,
	0x07, 0x00 };
unsigned char c23[] =
{ 0xB9, 0x01, 0x36 };
unsigned char c24[] =
{ 0xBA, 0x00 };
unsigned char c25[] =
{ 0xBB, 0x00 };
unsigned char c26[] =
{ 0xBC, 0x00, 0x00, 0x00, 0x00 };
unsigned char c27[] =
{ 0xBD, 0x04, 0x0C };
unsigned char c28[] =
{ 0xC2, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
unsigned char c29[] =
{ 0xC5, 0x0A, 0x1D, 0x00, 0x10, 0x1A, 0x1E, 0x0B, 0x1D, 0x08, 0x16 };
unsigned char c30[] =
{ 0xC6, 0x17, 0x10, 0x19 };
unsigned char c31[] =
{ 0xC9, 0x00, 0x00, 0x00, 0x15, 0x17, 0x19, 0x1F, 0x1B, 0x1D, 0x21, 0x23, 0x25,
	0x27, 0x29, 0x2B, 0x2D, 0x2F, 0x16, 0x18, 0x1A, 0x20, 0x1C, 0x1E, 0x22,
	0x24, 0x26, 0x28, 0x2A, 0x2C, 0x2E, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00 };
unsigned char c32[] =
{ 0xCB, 0x01, 0xF5, 0xFF, 0xFF, 0x01, 0x00, 0x05, 0x00, 0x9F, 0x00, 0x00, 0x00 };
unsigned char c33[] =
{ 0xD0, 0x06, 0x01 };
unsigned char c34[] =
{ 0xD3, 0x06, 0x01 };
unsigned char c35[] =
{ 0xD5, 0x29, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00 };

unsigned char c36[] =
{ 0x40, 0x01, 0x5A	//0xFE00
	, 0x7F, 0x00, 0xF0, 0x11, 0x00, 0x00	//0xFE02
	, 0x55, 0x20, 0x1E, 0x22, 0x03, 0x09, 0x0C, 0x20, 0x20, 0x20, 0x20, 0x14,
	0x20, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };//start:0x00 ,size 31	//0xFE08

unsigned char c37[] =
{ 0x40, 0xA5, 0x00, 0x80, 0x82, 0x85, 0x00	//0xFE1E
	, 0x30, 0x28, 0x06, 0x08, 0x83, 0x33, 0x00, 0x00 //0xFE24
	, 0x11, 0x11, 0x00, 0x00	//0xFE2C
	, 0x16, 0x0F, 0x00, 0x0A, 0x00, 0x00	//0xFE30
	, 0x10, 0x02, 0x1E, 0x64, 0x00, 0x00 }; // start 0x1E :size 31		//0xFE36

unsigned char c38[] =
{ 0x40, 0x40, 0x47, 0x47, 0x02, 0x14, 0x00, 0x00, 0x00 //0xFE3C
	, 0x04, 0x03, 0x12, 0x08, 0x08, 0x00, 0x00, 0x00 }; // start:0x3C ,size 17		//0xFE44

unsigned char c39[] =
{ 0x40, 0x18, 0x0A, 0x05, 0x00, 0x00, 0xD8, 0x8C, 0x00, 0x00, 0x42, 0x03, 0x00,
	0x00, 0x00, 0x00, 0x00		//0xFE4C
	, 0x88, 0x06, 0x20, 0x00, 0x12, 0x0F, 0x0F, 0x30 }; //start 0x4C,size 25		//0xFE5C

unsigned char c40[] =
{ 0x40, 0x10, 0x12, 0x20, 0x32, 0x01, 0x04, 0x07, 0x09 //0xFE64
	, 0xAA, 0x6E, 0x32, 0x00	//0xFE6C
	, 0x0F, 0x1C, 0xA0, 0x13	//0xFE70
	, 0x00, 0x00, 0x04, 0x38, 0x07, 0x80 }; //start 0x64,size 23	//0xFE74

unsigned char c41[] =
{ 0x40, 0x02, 0xEE, 0x07, 0xB1, 0x55, 0x0C, 0x07, 0x00, 0x60, 0x01, 0xE7, 0x4A,
	0xD7, 0x07	//0xFE7A
	, 0x03, 0xCD, 0x09, 0xFB, 0x58, 0xB7, 0x00, 0x00, 0x7F, 0x02, 0x85, 0x4C,
	0xBD, 0x00 };	//start 0x7A,size 29	//0xFE88

unsigned char c42[] =
{ 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; //start 0x96,size 9	//0xFE96

unsigned char c43_1[] =
{ 0x40, 0x00, 0xFF, 0x15, 0x28, 0x01, 0xFF, 0x16, 0x29, 0x02, 0xFF, 0x1B, 0x2A,
	0x03, 0xFF, 0x1C, 0xFF, 0x04, 0xFF, 0x1D, 0xFF, 0x05, 0x0F, 0x1E, 0xFF,
	0x06, 0x10, 0x1F, 0xFF, 0x07, 0x11, 0x20 }; //start 0x9E,size 32
unsigned char c43_2[] =
{ 0x40, 0xFF, 0x08, 0x12, 0x21, 0xFF, 0x09, 0x13, 0x22, 0xFF, 0x0A, 0x14, 0x23,
	0xFF, 0x0B, 0x17, 0x24, 0xFF, 0x0C, 0x18, 0x25, 0xFF, 0x0D, 0x19, 0x26,
	0xFF, 0x0E, 0x1A, 0x27, 0xFF }; //start 0xBD,size 29

unsigned char c44_1[] =
{ 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; //start 0xDA,size 32
unsigned char c44_2[] =
{ 0x40, 0x00, 0x00, 0x00, 0x00, 0x00 }; //0xF9 size 6
unsigned char c45[] =
{ 0x40, 0x1D, 0x00 }; //start 0xFE,size 3	//0xFEFE
#endif
//----[HX_LOADIN_CONFIG]----------------------------------------------------------------------------------end


#endif

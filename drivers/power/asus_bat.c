//ASUS_BSP +++ Josh_Liao "add asus battery driver"
/* asus_bat.c */
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/workqueue.h>
#include <linux/power_supply.h>
#include <linux/mutex.h>
#ifdef CONFIG_EEPROM_NUVOTON
#include <linux/microp_notify.h>
#include <linux/microp_notifier_controller.h>	//ASUS_BSP Lenter+


#include <linux/microp_api.h>
#endif /*CONFIG_EEPROM_NUVOTON */
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/asus_bat.h>
#include <linux/mfd/pm8xxx/batt-alarm.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/mfd/pm8xxx/batt-alarm.h>
#include <linux/asus_chg.h>
//ASUS_BSP  +++ Eason_Chang "add P01 charge"
#include <linux/mfd/pm8xxx/pm8921-charger.h>
//ASUS_BSP  --- Eason_Chang "add P01 charge"
//ASUS_BSP  +++ Eason_Chang "add BAT low debounce"
#include <linux/delay.h>
//ASUS_BSP  --- Eason_Chang "add BAT low debounce"
//ASUS_BSP  +++ Eason_Chang "add BAT info time"
#include <linux/rtc.h>
//ASUS_BSP  --- Eason_Chang "add BAT info time"

//ASUS_BSP  +++ Eason_Chang "add BAT earlysuspend"
#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
//ASUS_BSP  --- Eason_Chang "add BAT earlysuspend"
//Hank: 1025 use frame buffer notifier to implement earlysuspend+++
#elif defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>

struct notifier_block bat_fb_notif;
//Hank: 1025 use frame buffer notifier to implement earlysuspend---
#endif


//ASUS_BSP +++ Josh_Liao "sw gauge v2"
#include "../power/gauge/test_gauge.h"
//ASUS_BSP --- Josh_Liao "sw gauge v2"
//ASUS_BSP Eason_Chang add event log +++
#include <linux/asusdebug.h>
//ASUS_BSP Eason_Chang add event log ---
//ASUS_BSP Eason_Chang in Pad setChg +++
#ifdef CONFIG_CHARGER_ASUS
static bool IsInPadSetChg= false;
#endif
//ASUS_BSP Eason_Chang in Pad setChg ---
/******************************************************************************/

#define ASUS_BAT_DEF_UPDATE_RATE  180	// the update rate is 3 minutes in default                 

#define ASUS_BAT_PROC_FILE  "driver/asus_bat"
#define ASUS_BAT_PROC_FILE_PERMISSION  0777

#define ASUS_BAT_PROC_MAX_BUFF_SIZE  256

#define ASUS_BAT_LOW_GPIO   29

#define PAD_BAT  0
#define DOCK_BAT  1

/* battery command */
#define BAT_CMD_INTERVAL					2
#define BAT_CMD_READ						'r'
#define BAT_CMD_CHARGING_STATUS			'c'
#define BAT_CMD_CHARGING_MODE			'm'
#define BAT_CMD_CHARGER_CABLE			'r'
#define BAT_CMD_BAT_PRESENT				'p'
#define BAT_CMD_BAT_LIFE					'l'
#define BAT_CMD_WRITE						'w'
#define BAT_CMD_HELP						'h'
#define BAT_CMD_PHONE_ID					'0'
#define BAT_CMD_PAD_ID  					'1'
#define BAT_CMD_DOCK_ID  					'2'
#ifdef CONFIG_BATTERY_ASUS_SERVICE
#define BAT_CMD_FIX_INTERVAL			'i'
#define BAT_CMD_FIX_LAST_UPDATE_INTERVAL			'l'
#endif
#ifdef CONFIG_PM8921_CHARGER
#define BAT_CMD_CHG_STATE_MONITOR_INTERVAL 'p'
#endif
#define BAT_CMD_ENABLE_TEST				't'
#ifdef CONFIG_ASUS_POWER_UTIL
#define BAT_CMD_ENABLE_UNIT_TEST		'e'
#define BAT_CMD_ENABLE_DOITLATER_TEST		'd'
#define BAT_CMD_ENABLE_FEEDING_FILE_INPUT_TEST		'i'
#endif
#ifdef CONFIG_CHARGER_ASUS_FSM
#define BAT_CMD_ENABLE_FSM_STRESS_TEST		'f'
#define BAT_CMD_ENABLE_FSM_SINGLE_STATE_TEST		's'
#endif
#ifdef CONFIG_BATTERY_ASUS_SERVICE
#define BAT_CMD_ENABLE_SERVICE_TEST		's'
#endif

//ASUS_BSP +++ Josh_Liao "sw gauge v2"
#define BAT_CMD_TEST_SWG		's'
#define BAT_CMD_SWG_CAP		'c'
#define BAT_CMD_SWG_CABLE		'b'
#define BAT_CMD_SWG_BAT_FULL	'f'
#define BAT_CMD_SWG_FILTER_CAP 'l'
//ASUS_BSP --- Josh_Liao "sw gauge v2"

//ASUS_BSP +++ Eason_Chang BalanceMode
#define BAT_CMD_ENABLE_BALANCE 'b' 
#define BAT_CMD_BALANCE_ISBALANCE 'i'
#define BAT_CMD_BALANCE_ISTEST 'e'
#define BAT_CMD_BALANCE_STARTRATIO 's'
#define BAT_CMD_BALANCE_STOPRATIO 't'
#define BAT_CMD_BALANCE_A66CAP 'a'
#define BAT_CMD_BALANCE_PADCAP 'p'
//ASUS_BSP --- Eason_Chang BalanceMode

#define BAT_CMD_ALL_INFO					'a'
#define BAT_CMD_UPDATE					'u'
#define BAT_CMD_SET						's'

#define BAT_STATUS_TXT     "/data/data/battery_status"

#define BAT_TRUE_TXT						"true"
#define BAT_FALSE_TXT						"false"
#define BAT_LOW_TXT						"battery_low"


#define BAT_LOW_SHUTDOWN_CAP			0

#define BAT_LOW_VOLT						3400

#define BAT_FIRST_UPDATE_DELAY			20

#define TIME_FOR_FIRST_START_BATTERY_SERVER_IN_SECONDS (0)

/******************************************************************************/
enum asus_bat_type {
	ASUS_BAT_PHONE = 0,
	ASUS_BAT_PAD,
	ASUS_BAT_DOCK,
	ASUS_BAT_UNKNOWN
};

enum asus_bat_charging_status {
	ASUS_BAT_CHARGING_ERR = 0,
	ASUS_BAT_CHARGING_NONE,
	ASUS_BAT_CHARGING_ONGOING,
	ASUS_BAT_CHARGING_FULL,
	ASUS_BAT_CHARGING_UNKNOWN
};

#if 0
enum asus_bat_charger_cable {
	ASUS_BAT_CHARGER_NONE = 0,
	ASUS_BAT_CHARGER_USB,
	ASUS_BAT_CHARGER_AC,
	ASUS_BAT_CHARGER_OTHER_BAT,
	ASUS_BAT_CHARGER_UNKNOWN
};
#endif

const static char *charging_txt[] = {
	"CHARGING_ERR",
	"CHARGING_NONE",
	"CHARGING_ONGOING",
	"CHARGING_FULL",
	"CHARGING_UNKNOWN"
};

const static char *charger_txt[] = {
	"CHARGER_NONE",
	"CHARGER_USB",
	"CHARGER_AC",
	"CHARGER_OTHER_BAT",
	"CHARGER_UNKNOWN"
};

const static char *ps_charging_txt[] = {
	"POWER_SUPPLY_STATUS_UNKNOWN",
	"POWER_SUPPLY_STATUS_CHARGING",
	"POWER_SUPPLY_STATUS_DISCHARGING",
	"POWER_SUPPLY_STATUS_NOT_CHARGING",
	"POWER_SUPPLY_STATUS_FULL",
};

struct asus_bat_basic {
	bool present;
	bool bat_low;
	int capacity;
	enum asus_bat_charger_cable charger;
	enum asus_bat_charging_status charging;
	int volt;		// voltage			
	int curr;		// current
	struct delayed_work bat_low_work;
};

/******************************************************************************/
struct asus_bat_all_info {
	int phone_bat_low_gpio;
	int phone_bat_low_irq;
	struct mutex hs_evt_lock;
	struct mutex microp_evt_lock;
//	struct mutex microp_rw_lock;
	struct delayed_work bat_update_work;
	struct asus_bat_basic phone_b;
	struct asus_bat_basic pad_b;
	struct asus_bat_basic dock_b;
	bool enable_test;
	int bat_update_work_interval;
};

struct asus_bat_phone_bat_struct *phone_bat_info = NULL;


static bool g_done_first_periodic_update = false;

/******************************************************************************/
#ifdef CONFIG_EEPROM_NUVOTON
static int asus_bat_pad_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val);

static int asus_bat_pad_ac_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val);
#endif

#ifdef CONFIG_ASUSDEC
static int asus_bat_dock_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val);

static int asus_bat_dock_ac_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val);
#endif

#ifdef CONFIG_EEPROM_NUVOTON
static int asus_bat_microp_event_handler(struct notifier_block *this,
	unsigned long event, void *ptr);
#endif /* CONFIG_EEPROM_NUVOTON */

static void asus_bat_update_all_bat(void);

/******************************************************************************/
struct workqueue_struct *asus_bat_wq = NULL;
static struct asus_bat_all_info *asus_bat = NULL;

/******************************************************************************/
//ASUS_BSP +++ Eason_Chang BalanceMode
static int BatteryService_BalanceMode_IsBalTest = 0; //default 0.Change to 1 if BalTest selected
static int BatteryService_BalanceMode_StopRatio = 1;
static int BatteryService_BalanceMode_StartRatio = 9; // divid 2  is the real StartRatiio
static int BatteryService_BalanceMode_A66_CAP = 66;  //just a special default value
//static int BatteryService_BalanceMode_Pad_CAP = 33;
//ASUS_BSP --- Eason_Chang BalanceMode

//ASUS_BSP  +++ Eason_Chang "sw gauge support"
#include "gauge/axc_gaugefactory.h"
#include "charger/axc_chargerfactory.h"
#define BATTERY_LAST_CHANGED_LEVEL (1) 
#define BATTERY_LAST_LEVEL (2)
//#include "asus_bat_dbg.h"

#include "gauge/sampledata/axc_sampledatafactory.h"
#include "gauge/sampledata/axc_intsampledata.h"
#include "gauge/axi_gauge.h"
//static bool g_has_fixed_bat_life = false;
//static int g_fixed_bat_life = 0;
static int  gBatteryLife = 100;
int ReportBatteryServiceP02Cap(void);//P02gauge
int ReportBatteryServiceDockCap(void);
extern bool reportDockInitReady(void);
extern bool reportDockExtPowerPlug(void);
//Hank get voltage from TIgauge++
extern int get_Volt_from_TIgauge(void);
//Hank get voltage from TIgauge--
//Eason takeoff Battery shutdown +++
bool g_AcUsbOnline_Change0 = false;
extern void AcUsbPowerSupplyChange(void);
//Eason takeoff Battery shutdown ---

//ASUS BSP Eason Chang coincell+++
#include "../../include/linux/mfd/pm8xxx/misc.h" 
int pm8xxx_coincell_chg_config(struct pm8xxx_coincell_chg *chg_config);
struct pm8xxx_coincell_chg bat_chg_config={
            .state = PM8XXX_COINCELL_CHG_ENABLE,
            .voltage = PM8XXX_COINCELL_VOLTAGE_2p5V,
            .resistor = PM8XXX_COINCELL_RESISTOR_800_OHMS,
};
//ASUS BSP Eason Chang coincell---
//Eason boot up in BatLow situation, take off cable can shutdown+++
bool g_BootUp_IsBatLow = false;
//Eason boot up in BatLow situation, take off cable can shutdown---
//Eason if doesn't get correct ADC Vol&Curr at first update Cap show unknow status +++ 
extern bool g_adc_get_correct_VolCurr;
//Eason if doesn't get correct ADC Vol&Curr at first update Cap show unknow status ---
//Eason: LowCapCpuThrottle +++
static int LowCapCpuThrottle = 0 ;
static struct kobject *kobj;
//Eason: LowCapCpuThrottle ---
//Eason:let mpdicision check cpu throttle status+++
extern bool IsInCpuThrottle;
//Eason:let mpdicision check cpu throttle status---
//Hank TIgauge Update++
bool g_IsInRomMode = false;
//Hank TIgauge Update--
//Eason: Factory5060Mode+++
#ifdef ASUS_FACTORY_BUILD
extern bool g_5060modeCharging;
#endif
//Eason: Factory5060Mode---

#include <linux/rtc.h>
AXI_Charger * getAsusCharger(void)
{
    static AXI_Charger *lpCharger = NULL;

    if(lpCharger != NULL){

        return lpCharger;
    }
//ASUS_BSP Eason_Chang 1120 porting +++
/*
    if(g_A60K_hwID >=A66_HW_ID_ER2){

              AXC_ChargerFactory_GetCharger(E_SMB346_CHARGER_TYPE,&lpCharger);

    }else{

            AXC_ChargerFactory_GetCharger(E_PM8921_CHARGER_TYPE,&lpCharger);
    }
*/
		//Eason_Chang:for A90 internal ChgGau+++
		if(g_ASUS_hwID != A90_EVB0)
		{
			AXC_ChargerFactory_GetCharger(E_PM8941_CHARGER_TYPE,&lpCharger);
		}else{
			 AXC_ChargerFactory_GetCharger(E_SMB346_CHARGER_TYPE,&lpCharger);
		}	  
//ASUS_BSP Eason_Chang 1120 porting ---    

    return lpCharger;
}

#ifdef CONFIG_BATTERY_ASUS_SERVICE
#include "service/AXI_BatteryServiceFacade.h"
AXI_BatteryServiceFacade *loService = NULL;
#endif


bool asus_bat_has_done_first_periodic_update(void)
{
	return g_done_first_periodic_update;
}


static void asus_bat_set_phone_bat_capacity(int bat_cap)
{	
	if (NULL == asus_bat) {
		printk(DBGMSK_BAT_ERR "[BAT]%s(), null ptr \r\n", __FUNCTION__);
		return;
	}

	asus_bat->phone_b.capacity = bat_cap;
	return;
}


static void asus_bat_set_phone_bat_status(int bat_status)
{	
	if (NULL == asus_bat) {
		printk(DBGMSK_BAT_ERR "[BAT]%s(), null ptr \r\n", __FUNCTION__);
		return;
	}
	switch (bat_status) {
	case POWER_SUPPLY_STATUS_DISCHARGING:
		asus_bat->phone_b.charging = ASUS_BAT_CHARGING_NONE;
		break;
	case POWER_SUPPLY_STATUS_CHARGING:
		asus_bat->phone_b.charging = ASUS_BAT_CHARGING_ONGOING;
		break;
	case POWER_SUPPLY_STATUS_NOT_CHARGING:
		asus_bat->phone_b.charging = ASUS_BAT_CHARGING_NONE;		
		break;
	case POWER_SUPPLY_STATUS_FULL:
		asus_bat->phone_b.charging = ASUS_BAT_CHARGING_FULL;
		break;
	default:
		asus_bat->phone_b.charging = ASUS_BAT_CHARGING_UNKNOWN;
	}

	pr_debug( "[BAT]%s(), phone charging:%d \r\n", __FUNCTION__, asus_bat->phone_b.charging);

	return;
}


int asus_bat_get_phone_bat_capacity(void)
{
	if (NULL == asus_bat) {
		printk(DBGMSK_BAT_ERR "[BAT]%s(), null ptr \r\n", __FUNCTION__);
		return 0;
	}

	return asus_bat->phone_b.capacity;
}


/* For B-MMI usage */
void asus_bat_write_phone_bat_capacity_tofile(void)
{	
	struct file *fp_bat_sts = NULL;
	int used = 0;
	mm_segment_t old_fs;
	unsigned char bat_cap_s[4];
	loff_t pos_bat = 0;

	if (NULL == asus_bat) {
		printk(DBGMSK_BAT_ERR"[BAT]%s(), null ptr \r\n", __FUNCTION__);
		return;
	}

	used = snprintf(bat_cap_s, sizeof(bat_cap_s) ,"%d", asus_bat->phone_b.capacity);
	pr_debug( "[BAT] %s(), phone bat capacity:%d, write bat capcity string:%s, used:%d \r\n", 
		__FUNCTION__, asus_bat->phone_b.capacity, bat_cap_s, used);
	
 	old_fs = get_fs();
 	set_fs(KERNEL_DS);

	fp_bat_sts = filp_open(BAT_STATUS_TXT, O_WRONLY|O_CREAT, 0640);
	if (!IS_ERR(fp_bat_sts)) {
		fp_bat_sts->f_op->write(fp_bat_sts, bat_cap_s, used, &pos_bat);
		set_fs(old_fs);
		filp_close(fp_bat_sts, NULL);
	} else {
		printk(DBGMSK_BAT_ERR "[BAT] error!! bat_status filp open failed. \r\n");
	}

	return;
}
EXPORT_SYMBOL_GPL(asus_bat_write_phone_bat_capacity_tofile);

static void asus_bat_set_phone_bat_present(bool present)
{
	asus_bat->phone_b.present = present;

	return;
}

static void asus_bat_set_pad_bat_present(bool present)
{
	asus_bat->pad_b.present = present;

	return;
}

static void asus_bat_set_dock_bat_present(bool present)
{
	asus_bat->dock_b.present = present;
	return;

}

static bool asus_bat_is_pad_bat_present(void)
{
	if (NULL == asus_bat)
		return false;

	return asus_bat->pad_b.present;
}

static int asus_bat_get_pad_bat_capacity(void)
{	
	return asus_bat->pad_b.capacity;
}

static void asus_bat_set_pad_bat_capacity(int bat_cap)
{	
	asus_bat->pad_b.capacity = bat_cap;
	return;
}

static int asus_bat_get_pad_bat_capacity_byhw(void)
{
#ifdef CONFIG_EEPROM_NUVOTON
	int bat_cap = AX_MicroP_readBattCapacity(PAD_BAT);
#else
	int bat_cap = -1;
#endif /* CONFIG_EEPROM_NUVOTON */

	if (bat_cap < 0) {
		printk(DBGMSK_BAT_ERR "[BAT]error!! in %s(), read bat capacity error\n", __FUNCTION__);
	}
	
	return bat_cap;
}

static bool asus_bat_is_phone_bat_present(void)
{
	if (NULL == asus_bat)
		return false;

	return asus_bat->phone_b.present;

}


static bool asus_bat_is_dock_bat_present(void)
{
	if (NULL == asus_bat)
		return false;

	return asus_bat->dock_b.present;

}

static int asus_bat_get_dock_bat_capacity(void)
{	
	return asus_bat->dock_b.capacity;
}

static void asus_bat_set_dock_bat_capacity(int bat_cap)
{	
	asus_bat->dock_b.capacity = bat_cap;
	return;
}
#ifdef CONFIG_ASUSDEC
static int asus_bat_get_dock_bat_capacity_byhw(void)
{
#ifdef CONFIG_EEPROM_NUVOTON
	int bat_cap = AX_MicroP_readBattCapacity(DOCK_BAT);
#else
	int bat_cap = -1;
#endif /* CONFIG_EEPROM_NUVOTON */

	if (bat_cap < 0) {
		printk(DBGMSK_BAT_ERR "[BAT]error!! in %s(), read bat capacity error \r\n", __FUNCTION__);
	}
	
	return bat_cap;
}
#endif
// charger status function
#if 0
static enum asus_bat_charger_cable asus_bat_get_phone_charger_byhw(void)
{
	enum asus_bat_charger_cable charger;


	return charger;
}
#endif

static enum asus_bat_charger_cable asus_bat_get_pad_charger_byhw(void)
{
	enum asus_bat_charger_cable charger = ASUS_BAT_CHARGER_NONE;
#ifdef CONFIG_EEPROM_NUVOTON
	int pad_charger = AX_MicroP_get_USBDetectStatus(Batt_P01);
	switch(pad_charger) {
	case P01_CABLE_NO:
		charger = ASUS_BAT_CHARGER_NONE;
		break;
	case P01_CABLE_CHARGER:
		charger = ASUS_BAT_CHARGER_AC;
		break;
	case P01_CABLE_USB:
		charger = ASUS_BAT_CHARGER_USB;
		break;
	default:
		charger = ASUS_BAT_CHARGER_UNKNOWN;
		printk(DBGMSK_BAT_ERR "[BAT] error !! in %s()\n", __FUNCTION__);

	}
#endif /* CONFIG_EEPROM_NUVOTON */

	pr_debug("[BAT]%s(), pad charger by hw: %s \r\n", __FUNCTION__, charger_txt[charger]);
	return charger;
}

static enum asus_bat_charger_cable asus_bat_get_dock_charger_byhw(void)
{
	enum asus_bat_charger_cable charger = ASUS_BAT_CHARGER_NONE;
#ifdef CONFIG_EEPROM_NUVOTON	
	int dock_charger = AX_MicroP_get_USBDetectStatus(Batt_Dock);
	switch(dock_charger) {
	case P01_CABLE_NO:
		charger = ASUS_BAT_CHARGER_NONE;
		break;
	case P01_CABLE_CHARGER:
		charger = ASUS_BAT_CHARGER_AC;
		break;
	case P01_CABLE_USB:
		charger = ASUS_BAT_CHARGER_USB;
		break;
	default:
		charger = ASUS_BAT_CHARGER_UNKNOWN;
		printk(DBGMSK_BAT_ERR "[BAT] error !! in %s()\n", __FUNCTION__);
	}
#endif /* CONFIG_EEPROM_NUVOTON */

	pr_debug("[BAT]%s(), dock charger by hw: %s \r\n", __FUNCTION__, charger_txt[charger]);
	return charger;
}

static enum asus_bat_charger_cable asus_bat_get_phone_charger(void)
{
	return asus_bat->phone_b.charger;
}

//Eason: A68 new balance mode +++	
/*
static enum asus_bat_charger_cable asus_bat_get_pad_charger(void)
{
	return asus_bat->pad_b.charger;
}
*/
//Eason: A68 new balance mode ---
#ifdef CONFIG_ASUSDEC
static enum asus_bat_charger_cable asus_bat_get_dock_charger(void)
{
	return asus_bat->dock_b.charger;
}
#endif
#if 0
static enum asus_bat_charger_cable asus_bat_get_phone_charger(void)
{
	return asus_bat->phone_b.charger;
}
#endif


static void asus_bat_set_phone_charger(enum asus_bat_charger_cable charger )
{
	asus_bat->phone_b.charger = charger;
	return;
}

static void asus_bat_set_pad_charger(enum asus_bat_charger_cable charger )
{
	asus_bat->pad_b.charger = charger;
	return;
}

static void asus_bat_set_dock_charger(enum asus_bat_charger_cable charger )
{
	asus_bat->dock_b.charger = charger;
	return;
}



// charging mode function
#if 0
static enum asus_bat_charging_status asus_bat_get_phone_charging_byhw(void)
{
	enum asus_bat_charging_status charging;
//TODO	
	return charging;
}
#endif

static enum asus_bat_charging_status asus_bat_get_pad_charging_byhw(void)
{
	enum asus_bat_charging_status charging = ASUS_BAT_CHARGING_UNKNOWN;

#ifdef CONFIG_EEPROM_NUVOTON
	switch (AX_MicroP_get_ChargingStatus(Batt_P01)) {
	case P01_CHARGING_ERR:
		charging = ASUS_BAT_CHARGING_ERR;
		break;
	case P01_CHARGING_NO:
		charging = ASUS_BAT_CHARGING_NONE;
		break;
	case P01_CHARGING_ONGOING:
		charging = ASUS_BAT_CHARGING_ONGOING;
		break;	
	case P01_CHARGING_FULL:
		charging = ASUS_BAT_CHARGING_FULL;
		break;
	default:
		printk(DBGMSK_BAT_ERR "[BAT] error !! in %s()\n", __FUNCTION__);	

	}
#endif /* CONFIG_EEPROM_NUVOTON */

	pr_debug( "[BAT]%s(), charging:%s\n", __FUNCTION__, charging_txt[charging]);

	return charging;
}

static enum asus_bat_charging_status asus_bat_get_dock_charging_byhw(void)
{
	enum asus_bat_charging_status charging = ASUS_BAT_CHARGING_UNKNOWN;

#ifdef CONFIG_EEPROM_NUVOTON		
	switch (AX_MicroP_get_ChargingStatus(Batt_Dock)) {
	case P01_CHARGING_ERR:
		charging = ASUS_BAT_CHARGING_ERR;
		break;
	case P01_CHARGING_NO:
		charging = ASUS_BAT_CHARGING_NONE;
		break;
	case P01_CHARGING_ONGOING:
		charging = ASUS_BAT_CHARGING_ONGOING;
		break;	
	case P01_CHARGING_FULL:
		charging = ASUS_BAT_CHARGING_FULL;
		break;
	default:
		printk(DBGMSK_BAT_ERR "[BAT] error !! in %s()\n", __FUNCTION__);	

	}
#endif /* CONFIG_EEPROM_NUVOTON */

	pr_debug( "[BAT]%s(), charging:%s\n", __FUNCTION__, charging_txt[charging]);

	return charging;
}

static enum asus_bat_charging_status asus_bat_get_phone_charging(void)
{
	return asus_bat->phone_b.charging;
}

//Eason: A68 new balance mode +++	
/*
static enum asus_bat_charging_status asus_bat_get_pad_charging(void)
{
	return asus_bat->pad_b.charging;
}
*/
//Eason: A68 new balance mode ---
#ifdef CONFIG_ASUSDEC
static enum asus_bat_charging_status asus_bat_get_dock_charging(void)
{
	return asus_bat->dock_b.charging;
}
#endif

static void asus_bat_set_phone_charging(enum asus_bat_charging_status charging )
{
	asus_bat->phone_b.charging = charging;
	pr_debug("[BAT]%s(), charging:%s\n", __FUNCTION__, charging_txt[charging]);
	return;
}

static void asus_bat_set_pad_charging(enum asus_bat_charging_status charging )
{
	asus_bat->pad_b.charging = charging;
	pr_debug( "[BAT]%s(), charging:%s\n", __FUNCTION__, charging_txt[charging]);
	return;
}

static void asus_bat_set_dock_charging(enum asus_bat_charging_status charging )
{
	asus_bat->dock_b.charging = charging;
	pr_debug( "[BAT]%s(), charging:%s\n", __FUNCTION__, charging_txt[charging]);
	return;
}


typedef void (*bat_cmd_func)(const char *msg, int index);

static void bat_cmd_read_phone_charging_status(const char *msg, int index)
{
	// by hw for test
	int charging;
	charging = phone_bat_info->get_prop_bat_status_byhw();
	printk(DBGMSK_BAT_INFO "[BAT] %s(), msg[%d]=%c, charging:%s \r\n",
		__FUNCTION__, index, msg[index], charging_txt[charging]);

	return ;
}

static void bat_cmd_read_pad_charging_status(const char *msg, int index)
{
	// by hw for test
	int charging;
	charging = asus_bat_get_pad_charging_byhw();
	printk(DBGMSK_BAT_INFO "[BAT] %s(), msg[%d]=%c, charging:%s \r\n",
		__FUNCTION__, index, msg[index], charging_txt[charging]);

	return ;
}

static void bat_cmd_read_dock_charging_status(const char *msg, int index)
{
	// by hw for test
	int charging;
	charging = asus_bat_get_dock_charging_byhw();
	printk(DBGMSK_BAT_INFO "[BAT] %s(), msg[%d]=%c, charging:%s \r\n",
		__FUNCTION__, index, msg[index], charging_txt[charging]);

	return ;
}

static void bat_cmd_read_phone_charger_cable(const char *msg, int index)
{
	int charger = asus_bat_get_phone_charger();
	printk(DBGMSK_BAT_INFO "[BAT] %s(), msg[%d]=%c, charger:%s \r\n",
		__FUNCTION__, index, msg[index], charger_txt[charger]);

	return;
}

static void bat_cmd_read_pad_charger_cable(const char *msg, int index)
{
	// by hw for test
	int charger;
	charger = asus_bat_get_pad_charger_byhw();
	printk(DBGMSK_BAT_INFO "[BAT] %s(), msg[%d]=%c, charger:%s \r\n",
		__FUNCTION__, index, msg[index], charger_txt[charger]);

	return;
}

static void bat_cmd_read_dock_charger_cable(const char *msg, int index)
{
	// by hw for test
	int charger;
	charger = asus_bat_get_dock_charger_byhw();
	printk(DBGMSK_BAT_INFO "[BAT] %s(), msg[%d]=%c, charger:%s \r\n",
		__FUNCTION__, index, msg[index], charger_txt[charger]);

	return;
}

static void bat_cmd_read_phone_bat_life(const char *msg, int index)
{
	int bat_life;
	printk(DBGMSK_BAT_INFO "[BAT] %s(), msg[%d]=%c \r\n", __FUNCTION__, index, msg[index]);
	bat_life = asus_bat_get_phone_bat_capacity();
	printk(DBGMSK_BAT_INFO "[BAT] %s(), bat_life:%d \r\n", __FUNCTION__, bat_life);
	return ;
}

static void bat_cmd_read_pad_bat_life(const char *msg, int index)
{
	int bat_life;
	printk(DBGMSK_BAT_INFO "[BAT] %s(), msg[%d]=%c \r\n", __FUNCTION__, index, msg[index]);
	bat_life = asus_bat_get_pad_bat_capacity();
	printk(DBGMSK_BAT_INFO "[BAT] %s(), bat_life:%d \r\n", __FUNCTION__, bat_life);

	return ;
}

static void bat_cmd_read_dock_bat_life(const char *msg, int index)
{
	int bat_life;
	printk(DBGMSK_BAT_INFO "[BAT] %s(), msg[%d]=%c \r\n", __FUNCTION__, index, msg[index]);
	bat_life = asus_bat_get_dock_bat_capacity();
	printk(DBGMSK_BAT_INFO "[BAT] %s(), bat_life:%d \r\n", __FUNCTION__, bat_life);

	return ;
}

static void bat_cmd_read_phone_bat_present(const char *msg, int index)
{
	printk(DBGMSK_BAT_INFO "[BAT] %s(), msg[%d]=%c \r\n", __FUNCTION__, index, msg[index]);
	if (asus_bat_is_phone_bat_present()) {
		printk(DBGMSK_BAT_INFO "[BAT] %s(), phone bat present \r\n", __FUNCTION__);
	} else {
		printk(DBGMSK_BAT_INFO "[BAT] %s(), phone bat not present \r\n", __FUNCTION__);
	}
	return ;
}

static void bat_cmd_read_pad_bat_present(const char *msg, int index)
{
	printk(DBGMSK_BAT_INFO "[BAT] %s(), msg[%d]=%c \r\n", __FUNCTION__, index, msg[index]);
	if (asus_bat_is_pad_bat_present()) {
		printk(DBGMSK_BAT_INFO "[BAT] %s(), pad bat present \r\n", __FUNCTION__);
	} else {
		printk(DBGMSK_BAT_INFO "[BAT] %s(), pad bat not present \r\n", __FUNCTION__);
	}

	return ;
}

static void bat_cmd_read_dock_bat_present(const char *msg, int index)
{
	printk(DBGMSK_BAT_INFO "[BAT] %s(), msg[%d]=%c \r\n", __FUNCTION__, index, msg[index]);
	if (asus_bat_is_dock_bat_present()) {
		printk(DBGMSK_BAT_INFO "[BAT] %s(), dock bat present \r\n", __FUNCTION__);
	} else {
		printk(DBGMSK_BAT_INFO "[BAT] %s(), dock bat not present \r\n", __FUNCTION__);
	}

	return ;
}

static void bat_cmd_write_phone_charging_status(const char *msg, int index)
{
	int val;
	index += BAT_CMD_INTERVAL;
	val = (int)simple_strtol(&msg[index], NULL, 10);

	printk(DBGMSK_BAT_INFO "[BAT] %s(), msg[%d]=%c, val:%d \r\n", __FUNCTION__, index, msg[index], val);

	asus_bat_set_phone_charging(val);

	return ;
}

static void bat_cmd_set_phone_charging_mode(const char *msg, int index)
{
	int val;
	index += BAT_CMD_INTERVAL;
	val = (int)simple_strtol(&msg[index], NULL, 10);

	printk(DBGMSK_BAT_INFO "[BAT] %s(), msg[%d]=%c, val:%d \r\n", __FUNCTION__, index, msg[index], val);

	if ((val >= ASUS_CHG_SRC_UNKNOWN) || (val < ASUS_CHG_SRC_NONE)) {
		printk(DBGMSK_BAT_ERR "[BAT] Error!! unknown charging mode. \r\n");
		return;
	}

	asus_chg_set_chg_mode(val);

	return ;
}

static void bat_cmd_write_pad_charging_status(const char *msg, int index)
{
	int val;
	index += BAT_CMD_INTERVAL;
	val = (int)simple_strtol(&msg[index], NULL, 10);

	printk(DBGMSK_BAT_INFO "[BAT] %s(), msg[%d]=%c, val:%d \r\n", __FUNCTION__, index, msg[index], val);


	asus_bat_set_pad_charging(val);
		
	return ;
}


static void bat_cmd_write_dock_charging_status(const char *msg, int index)
{
	int val;
	index += BAT_CMD_INTERVAL;
	val = (int)simple_strtol(&msg[index], NULL, 10);

	printk(DBGMSK_BAT_INFO "[BAT] %s(), msg[%d]=%c, val:%d \r\n", __FUNCTION__, index, msg[index], val);

	asus_bat_set_dock_charging(val);
	
	return ;
}



static void bat_cmd_write_phone_charger_cable(const char *msg, int index)
{
	int val;
	index += BAT_CMD_INTERVAL;
	val = (int)simple_strtol(&msg[index], NULL, 10);

	printk(DBGMSK_BAT_INFO "[BAT] %s(), msg[%d]=%c, val:%d \r\n", __FUNCTION__, index, msg[index], val);

	asus_bat_set_phone_charger(val);

	return ;
}


static void bat_cmd_write_pad_charger_cable(const char *msg, int index)
{
	int val;
	index += BAT_CMD_INTERVAL;
	val = (int)simple_strtol(&msg[index], NULL, 10);

	printk(DBGMSK_BAT_INFO "[BAT] %s(), msg[%d]=%c, val:%d \r\n", __FUNCTION__, index, msg[index], val);

	asus_bat_set_pad_charger(val);


	return ;
}


static void bat_cmd_write_dock_charger_cable(const char *msg, int index)
{
	int val;
	index += BAT_CMD_INTERVAL;
	val = (int)simple_strtol(&msg[index], NULL, 10);

	printk(DBGMSK_BAT_INFO "[BAT] %s(), msg[%d]=%c, val:%d \r\n", __FUNCTION__, index, msg[index], val);

	asus_bat_set_dock_charger(val);

	return ;
}

static void bat_cmd_write_phone_bat_life(const char *msg, int index)
{
	int val;
	index += BAT_CMD_INTERVAL;
	val = (int)simple_strtol(&msg[index], NULL, 10);

	printk(DBGMSK_BAT_INFO "[BAT] %s(), msg[%d]=%c, val:%d \r\n", __FUNCTION__, index, msg[index], val);

    //ASUS_BSP +++ Eason_Chang BalanceMode
    //asus_bat_set_phone_bat_capacity(val);
    BatteryService_BalanceMode_A66_CAP = val;
    //ASUS_BSP --- Eason_Chang BalanceMode

	return ;
}

static void bat_cmd_write_phone_bat_present(const char *msg, int index)
{
	int val;
	index += BAT_CMD_INTERVAL;
	val = (int)simple_strtol(&msg[index], NULL, 10);

	printk(DBGMSK_BAT_INFO "[BAT] %s(), msg[%d]=%c, val:%d \r\n", __FUNCTION__, index, msg[index], val);

	asus_bat_set_phone_bat_present(val);

	return ;
}

static void bat_cmd_write_pad_bat_life(const char *msg, int index)
{
	int val;
	index += BAT_CMD_INTERVAL;
	val = (int)simple_strtol(&msg[index], NULL, 10);

	printk(DBGMSK_BAT_INFO "[BAT] %s(), msg[%d]=%c, val:%d \r\n", __FUNCTION__, index, msg[index], val);

	asus_bat_set_pad_bat_capacity(val);

	return ;
}

static void bat_cmd_write_pad_bat_present(const char *msg, int index)
{
	int val;
	index += BAT_CMD_INTERVAL;
	val = (int)simple_strtol(&msg[index], NULL, 10);

	printk(DBGMSK_BAT_INFO "[BAT] %s(), msg[%d]=%c, val:%d \r\n", __FUNCTION__, index, msg[index], val);

	asus_bat_set_pad_bat_present(val);

	return ;
}


static void bat_cmd_write_dock_bat_life(const char *msg, int index)
{
	int val;
	index += BAT_CMD_INTERVAL;
	val = (int)simple_strtol(&msg[index], NULL, 10);

	printk(DBGMSK_BAT_INFO "[BAT] %s(), msg[%d]=%c, val:%d \r\n", __FUNCTION__, index, msg[index], val);


	asus_bat_set_dock_bat_capacity(val);

	return ;
}

static void bat_cmd_write_dock_bat_present(const char *msg, int index)
{
	int val;
	index += BAT_CMD_INTERVAL;
	val = (int)simple_strtol(&msg[index], NULL, 10);

	printk(DBGMSK_BAT_INFO "[BAT] %s(), msg[%d]=%c, val:%d \r\n", __FUNCTION__, index, msg[index], val);

	asus_bat_set_dock_bat_present(val);

	return ;
}
static void bat_cmd_read_all_info(const char *msg, int index )
{
	printk(DBGMSK_BAT_INFO "[BAT] %s(), msg[%d]=%c \r\n", __FUNCTION__, index, msg[index]);

	printk(DBGMSK_BAT_INFO "[BAT] phone bat present: %s \r\n", asus_bat->phone_b.present?BAT_TRUE_TXT:BAT_FALSE_TXT);
	printk(DBGMSK_BAT_INFO "[BAT] pad bat present: %s \r\n", asus_bat->pad_b.present?BAT_TRUE_TXT:BAT_FALSE_TXT);
	printk(DBGMSK_BAT_INFO "[BAT] dock bat present: %s \r\n", asus_bat->dock_b.present?BAT_TRUE_TXT:BAT_FALSE_TXT);

	printk(DBGMSK_BAT_INFO "[BAT] phone bat life: %d \r\n", asus_bat->phone_b.capacity);
	printk(DBGMSK_BAT_INFO "[BAT] pad bat life: %d \r\n", asus_bat->pad_b.capacity);
	printk(DBGMSK_BAT_INFO "[BAT] dock bat life: %d \r\n", asus_bat->dock_b.capacity);

	printk(DBGMSK_BAT_INFO "[BAT] phone charging status: %s \r\n", charging_txt[asus_bat->phone_b.charging]);
	printk(DBGMSK_BAT_INFO "[BAT] pad charging status: %s \r\n", charging_txt[asus_bat->pad_b.charging]);
	printk(DBGMSK_BAT_INFO "[BAT] dock charging status: %s \r\n", charging_txt[asus_bat->dock_b.charging]);	

	printk(DBGMSK_BAT_INFO "[BAT] phone charger cable: %s \r\n", charger_txt[asus_bat->phone_b.charger]);
	printk(DBGMSK_BAT_INFO "[BAT] pad charger cable: %s \r\n", charger_txt[asus_bat->pad_b.charger]);
	printk(DBGMSK_BAT_INFO "[BAT] dock charger cable: %s \r\n", charger_txt[asus_bat->dock_b.charger]);	

	printk(DBGMSK_BAT_INFO "[BAT] enable test:%s \r\n", asus_bat->enable_test?BAT_TRUE_TXT:BAT_FALSE_TXT);
	printk(DBGMSK_BAT_INFO "[BAT] update bat interval:%d \r\n", asus_bat->bat_update_work_interval);


	printk(DBGMSK_BAT_INFO "[BAT] phone bat status from HW: %d \r\n", phone_bat_info->get_prop_bat_status_byhw());
	printk(DBGMSK_BAT_INFO "[BAT] phone bat volt from TIgauge: %dmV \r\n", get_Volt_from_TIgauge());
	printk(DBGMSK_BAT_INFO "[BAT] phone bat curr from HW: %d \r\n", phone_bat_info->get_prop_batt_curr_byhw());
	
	return ;
}
#ifdef CONFIG_BATTERY_ASUS_SERVICE
#include "service/AXC_BatteryServiceTest.h"
static void bat_cmd_fix_interval(const char *msg, int index )
{
       bool fixed;

       int interval;

       AXC_BatteryServiceTest *test =  getBatteryServiceTest();

	index += BAT_CMD_INTERVAL;

	printk(DBGMSK_BAT_INFO "[BAT] %s(), msg[%d]=%c \r\n", __FUNCTION__, index, msg[index]);

	fixed = (bool)((int)simple_strtol(&msg[index], NULL, 10) ?true:false);

       if(fixed){
        
            index += BAT_CMD_INTERVAL;

            interval = (int)simple_strtol(&msg[index], NULL, 10);

            test->changePollingInterval(test, true, interval);

            	printk(DBGMSK_BAT_INFO "[BAT] %s(), fix update interval:%d \r\n", __FUNCTION__, interval);

       }else{
       
           test->changePollingInterval(test, false, 0);

           	printk(DBGMSK_BAT_INFO "[BAT] %s(), release fix update interval\r\n", __FUNCTION__);

       }	

	return ;
}
static void bat_cmd_fix_last_update_interval(const char *msg, int index )
{
       bool fixed;

       int interval;

       AXC_BatteryServiceTest *test =  getBatteryServiceTest();

	index += BAT_CMD_INTERVAL;

	printk(DBGMSK_BAT_INFO "[BAT] %s(), msg[%d]=%c \r\n", __FUNCTION__, index, msg[index]);

	fixed = (bool)((int)simple_strtol(&msg[index], NULL, 10) ?true:false);

       if(fixed){
        
            index += BAT_CMD_INTERVAL;

            interval = (int)simple_strtol(&msg[index], NULL, 10);

            test->changeFilterLastUpdateInterval(test, true, interval);

            	printk(DBGMSK_BAT_INFO "[BAT] %s(), fix update interval:%d \r\n", __FUNCTION__, interval);

       }else{
       
             test->changeFilterLastUpdateInterval(test, false, 0);

           	printk(DBGMSK_BAT_INFO "[BAT] %s(), release fix update interval\r\n", __FUNCTION__);

       }	

	return ;

}
#endif //CONFIG_BATTERY_ASUS_SERVICE
#ifdef CONFIG_PM8921_CHARGER
void change_chg_statue_check_interval(int interval);

static void bat_cmd_fix_chg_state_monitor_interval(const char *msg, int index )
{

       int interval;

	index += BAT_CMD_INTERVAL;

	printk(DBGMSK_BAT_INFO "[BAT] %s(), msg[%d]=%c \r\n", __FUNCTION__, index, msg[index]);

	interval = (int)simple_strtol(&msg[index], NULL, 10);

       change_chg_statue_check_interval(interval);

	return ;

}
#endif //CONFIG_PM8921_CHARGER

static void bat_cmd_write_test(const char *msg, int index )
{
	int val;

	index += BAT_CMD_INTERVAL;

	printk(DBGMSK_BAT_INFO "[BAT] %s(), msg[%d]=%c \r\n", __FUNCTION__, index, msg[index]);

	val = (int)simple_strtol(&msg[index], NULL, 10);

	if (val >= 1) {
		asus_bat->enable_test = true;
		printk(DBGMSK_BAT_INFO "[BAT] enable test");
	} else {
		asus_bat->enable_test = false;
		printk(DBGMSK_BAT_INFO "[BAT] disable test");
	}

	return ;
}

//TODO
static void bat_cmd_get_help(const char *msg, int index )
{
	printk(DBGMSK_BAT_INFO "[BAT] %s(), msg[%d]=%c \r\n", __FUNCTION__, index, msg[index]);
// print help info
	return ;
}

static void bat_cmd_update(const char *msg, int index )
{
	printk(DBGMSK_BAT_INFO "[BAT] %s(), msg[%d]=%c \r\n", __FUNCTION__, index, msg[index]);

	asus_bat_update_all_bat();
	return ;
}

				   
struct asus_bat_proc_cmd {
	char id;
	const bat_cmd_func func;
	struct asus_bat_proc_cmd *cmd_tbl;
	int cmd_tbl_size;
};


struct asus_bat_proc_cmd phone_bat_read_cmd_tbl[] = {
	{BAT_CMD_CHARGING_STATUS, bat_cmd_read_phone_charging_status, NULL, 0},
	{BAT_CMD_CHARGER_CABLE, bat_cmd_read_phone_charger_cable, NULL, 0},		
	{BAT_CMD_BAT_LIFE, bat_cmd_read_phone_bat_life, NULL, 0},
	{BAT_CMD_BAT_PRESENT, bat_cmd_read_phone_bat_present, NULL, 0},
};

struct asus_bat_proc_cmd pad_bat_read_cmd_tbl[] = {
	{BAT_CMD_CHARGING_STATUS, bat_cmd_read_pad_charging_status, NULL, 0},
	{BAT_CMD_CHARGER_CABLE, bat_cmd_read_pad_charger_cable, NULL, 0},
	{BAT_CMD_BAT_LIFE, bat_cmd_read_pad_bat_life, NULL, 0},
	{BAT_CMD_BAT_PRESENT, bat_cmd_read_pad_bat_present, NULL, 0},
};

struct asus_bat_proc_cmd dock_bat_read_cmd_tbl[] = {
	{BAT_CMD_CHARGING_STATUS, bat_cmd_read_dock_charging_status, NULL, 0},
	{BAT_CMD_CHARGER_CABLE, bat_cmd_read_dock_charger_cable, NULL, 0},		
	{BAT_CMD_BAT_LIFE, bat_cmd_read_dock_bat_life, NULL, 0},
	{BAT_CMD_BAT_PRESENT, bat_cmd_read_dock_bat_present, NULL, 0},
};

struct asus_bat_proc_cmd phone_bat_write_cmd_tbl[] = {
	{BAT_CMD_CHARGING_STATUS, bat_cmd_write_phone_charging_status, NULL, 0},
	{BAT_CMD_CHARGER_CABLE, bat_cmd_write_phone_charger_cable, NULL, 0},
	{BAT_CMD_BAT_LIFE, bat_cmd_write_phone_bat_life, NULL, 0},
	{BAT_CMD_BAT_PRESENT, bat_cmd_write_phone_bat_present, NULL, 0},
};

struct asus_bat_proc_cmd pad_bat_write_cmd_tbl[] = {
	{BAT_CMD_CHARGING_STATUS, bat_cmd_write_pad_charging_status, NULL, 0},
	{BAT_CMD_CHARGER_CABLE, bat_cmd_write_pad_charger_cable, NULL, 0},
	{BAT_CMD_BAT_LIFE, bat_cmd_write_pad_bat_life, NULL, 0},
	{BAT_CMD_BAT_PRESENT, bat_cmd_write_pad_bat_present, NULL, 0},
};

struct asus_bat_proc_cmd dock_bat_write_cmd_tbl[] = {
	{BAT_CMD_CHARGING_STATUS, bat_cmd_write_dock_charging_status, NULL, 0},
	{BAT_CMD_CHARGER_CABLE, bat_cmd_write_dock_charger_cable, NULL, 0},
	{BAT_CMD_BAT_LIFE, bat_cmd_write_dock_bat_life, NULL, 0},
	{BAT_CMD_BAT_PRESENT, bat_cmd_write_dock_bat_present, NULL, 0},
};

struct asus_bat_proc_cmd bat_read_cmd_tbl[] = {
	{BAT_CMD_PHONE_ID, NULL, phone_bat_read_cmd_tbl, ARRAY_SIZE(phone_bat_read_cmd_tbl)},
	{BAT_CMD_PAD_ID, NULL, pad_bat_read_cmd_tbl, ARRAY_SIZE(pad_bat_read_cmd_tbl)},
	{BAT_CMD_DOCK_ID, NULL, dock_bat_read_cmd_tbl, ARRAY_SIZE(dock_bat_read_cmd_tbl)},
	{BAT_CMD_ALL_INFO, bat_cmd_read_all_info, NULL, 0},
};

#ifdef CONFIG_ASUS_POWER_UTIL
#include "util/AXC_DoitLaterFactory.h"
static AXI_DoitLater* gpDoitLater = NULL ;
static AXI_DoitLaterTask *gpDoitLaterTask = NULL;
static void gDoitLaterTask_dotask(struct AXI_DoitLaterTask *task)
{
    static unsigned int times = 0;

    printk("%s times = %d",__FUNCTION__,times++);

    msleep(100);

    gpDoitLater->start(gpDoitLater,gpDoitLaterTask,0,1);

}
static AXI_DoitLaterTask gDoitLaterTask = {
    .dotask = gDoitLaterTask_dotask,
};

static void bat_cmd_enable_doitLater_test(const char *msg, int index)
{
    int val;

    if(NULL == gpDoitLater){

        gpDoitLater = get_DoitLater(E_ONE_SHOT_WORKER_TYPE);

        gpDoitLaterTask = &gDoitLaterTask;
    }


    index += BAT_CMD_INTERVAL;
    val = (int)simple_strtol(&msg[index], NULL, 10);

    printk("[BAT] %s(), msg[%d]=%c, val:%d \r\n", __FUNCTION__, index, msg[index], val);

    if (val >= 1) {

        printk("doitLater_test start\n");

        gpDoitLater->start(gpDoitLater,&gDoitLaterTask,0,1);
        
    } else {
    
        gpDoitLater->stop(gpDoitLater);

        printk("doitLater_test stop\n");
    }

    return ;
}
#include "util/AXC_FeedingFileInputTest.h"
#define TEST_BUF_SIZE (5)
#define CHECK_PATTERN (0x05)
static char gFeedingFileInputParser_Buf[TEST_BUF_SIZE];
static AXC_FeedingFileInputParser *gpFeedingFileInputParser = NULL;
static AXC_FeedingFileInputTest gFeedingFileInputTest;
void gFeedingFileInputParser_parse(struct AXC_FeedingFileInputParser *test, unsigned int size)
{
    int index = 0;

    BUG_ON(size != TEST_BUF_SIZE);  

    for(index = 0; index< TEST_BUF_SIZE; index++){

        BUG_ON(CHECK_PATTERN != gFeedingFileInputParser_Buf[index]);
    }    

    printk("feeding_file_input_test_test stop\n");

}
char * gFeedingFileInputParser_getBuffer(struct AXC_FeedingFileInputParser *test, unsigned int *size)
{
    *size = TEST_BUF_SIZE;

    return gFeedingFileInputParser_Buf;
}
static AXC_FeedingFileInputParser gFeedingFileInputParser = {
    .parse = gFeedingFileInputParser_parse,
    .getBuffer = gFeedingFileInputParser_getBuffer,
};

static void bat_cmd_enable_feeding_file_input_test(const char *msg, int index)
{
    int val;

    if(NULL == gpFeedingFileInputParser){

        gpFeedingFileInputParser = &gFeedingFileInputParser;

        AXC_FeedingFileInputTest_constructor(&gFeedingFileInputTest,
            "stress_test",
            gpFeedingFileInputParser);

        memset(gFeedingFileInputParser_Buf,CHECK_PATTERN,TEST_BUF_SIZE);

        gFeedingFileInputTest.mImpl.createFileForFeeding(&gFeedingFileInputTest,
            gFeedingFileInputParser_Buf,
            TEST_BUF_SIZE);
    }


    index += BAT_CMD_INTERVAL;
    val = (int)simple_strtol(&msg[index], NULL, 10);

    printk("[BAT] %s(), msg[%d]=%c, val:%d \r\n", __FUNCTION__, index, msg[index], val);

    if (val >= 1) {

        printk("feeding_file_input_test_test start\n");

        gFeedingFileInputTest.miParent.test(&gFeedingFileInputTest.miParent, NULL);
        
    } 
    
    return ;
}
#endif //#ifdef CONFIG_ASUS_POWER_UTIL
#ifdef CONFIG_CHARGER_ASUS_FSM
#include "fsm/AXC_FSM_Stress_Tester.h"
static void bat_cmd_enable_fsm_stress_test(const char *msg, int index)
{
	int val;

      AXC_FSM_Stress_Tester* lpTester = getFSM_Stress_Tester();

	index += BAT_CMD_INTERVAL;
	val = (int)simple_strtol(&msg[index], NULL, 10);

	printk("[BAT] %s(), msg[%d]=%c, val:%d \r\n", __FUNCTION__, index, msg[index], val);

	if (val >= 1) {
             lpTester->mImpl.enable(lpTester, true);
	} else {
	      lpTester->mImpl.enable(lpTester, false);
	}

	return ;
}
#include "fsm/AXC_Charging_FSM.h"

static void bat_cmd_enable_fsm_single_state_test(const char *msg, int index)
{
	int pre_state,event;

      AXC_Charging_FSM *lpFSM = getChargingFSM(E_ASUS_A66_FSM_CHARGING_TYPE,NULL);

	index += BAT_CMD_INTERVAL;
    
	pre_state = (int)simple_strtol(&msg[index], NULL, 10);

	index += BAT_CMD_INTERVAL;
    
	event = (int)simple_strtol(&msg[index], NULL, 10);

       lpFSM->currentState = (AXE_Charging_State)pre_state;

       //precondition
       switch((AXE_Charging_State)pre_state){
           case DISCHARGING_STATE:
            lpFSM->chargerType = NO_CHARGER_TYPE;
            lpFSM->lastChargingErrorReason = NO_ERROR;
            break;
           case CHARGING_STATE:
               lpFSM->chargerType = HIGH_CURRENT_CHARGER_TYPE;
               lpFSM->lastChargingErrorReason = NO_ERROR;
            break;
           case CHARGING_STOP_STATE:
               lpFSM->chargerType = HIGH_CURRENT_CHARGER_TYPE;
               lpFSM->lastChargingErrorReason = HOT_CHARGER_ERROR;
            break;
           case CHARGING_FULL_STATE:
               lpFSM->chargerType = HIGH_CURRENT_CHARGER_TYPE;
               lpFSM->lastChargingErrorReason = NO_ERROR;
            break;
           case CHARGING_FULL_KEEP_STATE:
               lpFSM->chargerType = NORMAL_CURRENT_CHARGER_TYPE;
               lpFSM->lastChargingErrorReason = NO_ERROR;
            break;
           case CHARGING_FULL_KEEP_STOP_STATE:
               lpFSM->chargerType = HIGH_CURRENT_CHARGER_TYPE;
               lpFSM->lastChargingErrorReason = UNKNOWN_ERROR;
            break;
            default:
                printk("Can't mapping any state\n");
                return;
                break;
       }
       //event
        switch((AXE_Charger_Event)event){
            case CABLE_IN_EVENT:
                lpFSM->onCableInOut(lpFSM,LOW_CURRENT_CHARGER_TYPE);
             break;
            case CABLE_OUT_EVENT:
                lpFSM->onCableInOut(lpFSM,NO_CHARGER_TYPE);
             break;
            case CHARGING_DONE_EVENT:
                lpFSM->onChargingStop(lpFSM,CHARGING_DONE);
             break;
            case CHARGING_RESUME_EVENT:
                lpFSM->onChargingStart(lpFSM);
             break;
            case CHARGING_STOP_EVENT:
                lpFSM->onChargingStop(lpFSM,CHARGING_TIMEOUT_ERROR);
             break;
             default:
                 printk("Can't mapping any event\n");
                 return;
                 break;
        }

       printk("FSM state = %d, type=%d, last errorcode=%d"
        ,lpFSM->currentState
        ,lpFSM->chargerType
        ,lpFSM->lastChargingErrorReason);

	return ;
}




#endif // #ifdef CONFIG_CHARGER_ASUS_FSM

#ifdef CONFIG_BATTERY_ASUS_SERVICE
static int charging_current = -1;
static void changeChargingCurrent(struct AXI_BatteryServiceFacadeCallback *callback,AXE_Charger_Type chargertype)
{
    switch(chargertype){

        case NO_CHARGER_TYPE:
            charging_current = 0;
            break;
        case ILLEGAL_CHARGER_TYPE:            
        case LOW_CURRENT_CHARGER_TYPE:
            charging_current = 500;
            break;
        case NORMAL_CURRENT_CHARGER_TYPE:
            charging_current = 1000;
            break;
        case HIGH_CURRENT_CHARGER_TYPE:
            charging_current = 1500;
            break;
        default:
            charging_current = -1;
            break;
    }

}
static void onServiceStatusUpdated(struct AXI_BatteryServiceFacadeCallback *callback)
{

}
static AXI_BatteryServiceFacadeCallback gServiceCallback = {
   . changeChargingCurrent = changeChargingCurrent,
    .onServiceStatusUpdated = onServiceStatusUpdated,
};
static void bat_cmd_enable_service_stress_test(const char *msg, int index)
{
	int val;

       AXI_BatteryServiceFacade *lpService = getBatteryService(&gServiceCallback);

	index += BAT_CMD_INTERVAL;
	val = (int)simple_strtol(&msg[index], NULL, 10);

	printk("[BAT] %s(), msg[%d]=%c, val:%d \r\n", __FUNCTION__, index, msg[index], val);

	if (val >= 1) {
            lpService->onCableInOut(lpService,NO_CHARGER_TYPE);
            BUG_ON(lpService->getChargingStatus(lpService) != BAT_DISCHARGING_STATUS);
            BUG_ON(0  != charging_current);
            lpService->onCableInOut(lpService,LOW_CURRENT_CHARGER_TYPE);
            BUG_ON(lpService->getChargingStatus(lpService) != BAT_CHARGING_STATUS);
            BUG_ON(500  != charging_current);
            lpService->onCableInOut(lpService,NO_CHARGER_TYPE);
            lpService->onCableInOut(lpService,NORMAL_CURRENT_CHARGER_TYPE);
            BUG_ON(lpService->getChargingStatus(lpService) != BAT_CHARGING_STATUS);
            BUG_ON(1000  != charging_current);
            lpService->onCableInOut(lpService,NO_CHARGER_TYPE);
            lpService->onCableInOut(lpService,HIGH_CURRENT_CHARGER_TYPE);
            BUG_ON(lpService->getChargingStatus(lpService) != BAT_CHARGING_STATUS);
            BUG_ON(1500 != charging_current);
            lpService->onChargingStop(lpService,CHARGING_DONE);
            BUG_ON(lpService->getChargingStatus(lpService) != BAT_CHARGING_FULL_STATUS);
            BUG_ON(0 != charging_current);             
	} else {
	      
	}

	return ;
}
#endif


//ASUS_BSP +++ Josh_Liao "sw gauge v2"
static void bat_cmd_test_swg_cap(const char *msg, int index)
{
	int val;

	index += BAT_CMD_INTERVAL;
	val = (int)simple_strtol(&msg[index], NULL, 10);

	printk("[swg] %s(), msg[%d]=%c, val:%d \r\n", __FUNCTION__, index, msg[index], val);

	test_gauge_askCapacity();

	return ;
}

static void bat_cmd_test_swg_cable(const char *msg, int index)
{
	int val;

	index += BAT_CMD_INTERVAL;
	val = (int)simple_strtol(&msg[index], NULL, 10);

	printk("[swg] %s(), msg[%d]=%c, val:%d \r\n", __FUNCTION__, index, msg[index], val);

	if (1 == val) {
		test_gauge_notifyCableInOut(true);
	} else if (0 == val) {
		test_gauge_notifyCableInOut(false);
	}
	
	return;
}

static void bat_cmd_test_swg_bat_full(const char *msg, int index)
{
	int val;

	index += BAT_CMD_INTERVAL;
	val = (int)simple_strtol(&msg[index], NULL, 10);

	printk("[swg] %s(), msg[%d]=%c, val:%d \r\n", __FUNCTION__, index, msg[index], val);

	if (1 == val) {
		test_gauge_notifyBatFullChanged(true);
	} else if (0 == val) {
		test_gauge_notifyBatFullChanged(false);
	}

	return;
}


static void bat_cmd_test_swg_filter_cap(const char *msg, int index)
{
	int val;

	index += BAT_CMD_INTERVAL;
	val = (int)simple_strtol(&msg[index], NULL, 10);

	printk("[swg] %s(), msg[%d]=%c, val:%d \r\n", __FUNCTION__, index, msg[index], val);

	test_gauge_filterCap();
	
	return;
}

//ASUS_BSP +++ Eason_Chang BalanceMode


static void bat_balance_isbalTest(const char *msg, int index)
{
	int val;

	index += BAT_CMD_INTERVAL;
	val = (int)simple_strtol(&msg[index], NULL, 10);

	printk("[BAT][Bal] %s(), msg[%d]=%c, val:%d \r\n", __FUNCTION__, index, msg[index], val);
    printk("[BAT][Bal]isTest:%d\n",val);
	BatteryService_BalanceMode_IsBalTest = val;
	
	return;
}


static void bat_balance_stopratio(const char *msg, int index)  
{
	int val;

	index += BAT_CMD_INTERVAL;
	val = (int)simple_strtol(&msg[index], NULL, 10);

	printk("[BAT][Bal] %s(), msg[%d]=%c, val:%d \r\n", __FUNCTION__, index, msg[index], val);
    printk("[BAT][Bal]stopR:%d\n",val);
	BatteryService_BalanceMode_StopRatio = val;
	
	return;
}    


static void bat_balance_startratio(const char *msg, int index)  
{
	int val;

	index += BAT_CMD_INTERVAL;
	val = (int)simple_strtol(&msg[index], NULL, 10);

	printk("[BAT][Bal] %s(), msg[%d]=%c, val:%d \r\n", __FUNCTION__, index, msg[index], val);
    printk("[BAT][Bal]startR:%d\n",val);
   	BatteryService_BalanceMode_StartRatio = val;
	
	return;
} 

/*
static void bat_balance_A66Cap(const char *msg, int index)  
{
	int val;

	index += BAT_CMD_INTERVAL;
	val = (int)simple_strtol(&msg[index], NULL, 10);

	printk("[BAT][Bal] %s(), msg[%d]=%c, val:%d \r\n", __FUNCTION__, index, msg[index], val);

	BatteryService_BalanceMode_A66_CAP = val;
	
	return;
}

static void bat_balance_PadCap(const char *msg, int index)  
{
	int val;

	index += BAT_CMD_INTERVAL;
	val = (int)simple_strtol(&msg[index], NULL, 10);

	printk("[BAT][Bal] %s(), msg[%d]=%c, val:%d \r\n", __FUNCTION__, index, msg[index], val);

	BatteryService_BalanceMode_Pad_CAP = val;
	
	return;
}
*/


int IsBalanceTest(void){
    return BatteryService_BalanceMode_IsBalTest;
}    

int GetBalanceModeStopRatio(void){
    return BatteryService_BalanceMode_StopRatio;
} 

int GetBalanceModeStartRatio(void){
    return BatteryService_BalanceMode_StartRatio;
} 

int GetBalanceModeA66CAP(void){
    return BatteryService_BalanceMode_A66_CAP;
}

//ASUS_BSP --- Eason_Chang BalanceMode

//ASUS_BSP --- Josh_Liao "sw gauge v2"

struct asus_bat_proc_cmd bat_stress_test_cmd_tbl[] = {
#ifdef CONFIG_CHARGER_ASUS_FSM
	{BAT_CMD_ENABLE_FSM_STRESS_TEST, bat_cmd_enable_fsm_stress_test, NULL, 0},
	{BAT_CMD_ENABLE_FSM_SINGLE_STATE_TEST, bat_cmd_enable_fsm_single_state_test, NULL, 0},
#endif
#ifdef CONFIG_ASUS_POWER_UTIL
       {BAT_CMD_ENABLE_DOITLATER_TEST,bat_cmd_enable_doitLater_test, NULL, 0},
       {BAT_CMD_ENABLE_FEEDING_FILE_INPUT_TEST,bat_cmd_enable_feeding_file_input_test, NULL, 0},
#endif
#ifdef CONFIG_BATTERY_ASUS_SERVICE
       {BAT_CMD_ENABLE_SERVICE_TEST,bat_cmd_enable_service_stress_test, NULL, 0},
#endif

};

//ASUS_BSP +++ Josh_Liao "sw gauge v2"
struct asus_bat_proc_cmd bat_swg_test_cmd_tbl[] = {
	{BAT_CMD_SWG_CAP, bat_cmd_test_swg_cap, NULL, 0},
	{BAT_CMD_SWG_CABLE, bat_cmd_test_swg_cable, NULL, 0},
	{BAT_CMD_SWG_BAT_FULL, bat_cmd_test_swg_bat_full, NULL, 0},	
	{BAT_CMD_SWG_FILTER_CAP, bat_cmd_test_swg_filter_cap, NULL, 0},		
};
//ASUS_BSP --- Josh_Liao "sw gauge v2"


//ASUS_BSP +++ Eason_Chang BalanceMode
struct asus_bat_proc_cmd bat_balance_test_cmd_tbl[] = {	
    //{BAT_CMD_BALANCE_ISBALANCE, bat_balance_isbalance, NULL, 0},
    {BAT_CMD_BALANCE_ISTEST, bat_balance_isbalTest, NULL, 0},    
    {BAT_CMD_BALANCE_STARTRATIO, bat_balance_startratio, NULL, 0},
    {BAT_CMD_BALANCE_STOPRATIO, bat_balance_stopratio, NULL, 0},
    //{BAT_CMD_BALANCE_A66CAP, bat_balance_A66Cap, NULL, 0},
    //{BAT_CMD_BALANCE_PADCAP, bat_balance_PadCap, NULL, 0},  

};    
//ASUS_BSP --- Eason_Chang BalanceMode

struct asus_bat_proc_cmd bat_write_cmd_tbl[] = {
	{BAT_CMD_PHONE_ID, NULL, phone_bat_write_cmd_tbl, ARRAY_SIZE(phone_bat_write_cmd_tbl)},
	{BAT_CMD_PAD_ID, NULL, pad_bat_write_cmd_tbl, ARRAY_SIZE(pad_bat_write_cmd_tbl)},
	{BAT_CMD_DOCK_ID, NULL, dock_bat_write_cmd_tbl, ARRAY_SIZE(dock_bat_write_cmd_tbl)},
#ifdef CONFIG_BATTERY_ASUS_SERVICE
	{BAT_CMD_FIX_INTERVAL, bat_cmd_fix_interval, NULL, 0},
	{BAT_CMD_FIX_LAST_UPDATE_INTERVAL, bat_cmd_fix_last_update_interval, NULL, 0},
#endif
#ifdef CONFIG_PM8921_CHARGER
        {BAT_CMD_CHG_STATE_MONITOR_INTERVAL, bat_cmd_fix_chg_state_monitor_interval, NULL, 0},
#endif
	{BAT_CMD_ENABLE_TEST, bat_cmd_write_test, NULL, 0},
#ifdef CONFIG_CHARGER_ASUS_FSM
   {BAT_CMD_ENABLE_UNIT_TEST, NULL,bat_stress_test_cmd_tbl, ARRAY_SIZE(bat_stress_test_cmd_tbl)},
#endif
//ASUS_BSP +++ Josh_Liao "sw gauge v2"
   {BAT_CMD_TEST_SWG, NULL,bat_swg_test_cmd_tbl, ARRAY_SIZE(bat_swg_test_cmd_tbl)},
//ASUS_BSP --- Josh_Liao "sw gauge v2"

//ASUS_BSP +++ Eason_Chang BalanceMode
   {BAT_CMD_ENABLE_BALANCE, NULL,bat_balance_test_cmd_tbl, ARRAY_SIZE(bat_balance_test_cmd_tbl)},
//ASUS_BSP --- Eason_Chang BalanceMode

};

struct asus_bat_proc_cmd phone_bat_set_cmd_tbl[] = {
	{BAT_CMD_CHARGING_MODE, bat_cmd_set_phone_charging_mode, NULL, 0},
};

struct asus_bat_proc_cmd bat_set_cmd_tbl[] = {
	{BAT_CMD_PHONE_ID, NULL, phone_bat_set_cmd_tbl, ARRAY_SIZE(phone_bat_set_cmd_tbl)},
};

struct asus_bat_proc_cmd bat_cmd_tbl[] = {
	{ BAT_CMD_READ, NULL, bat_read_cmd_tbl, ARRAY_SIZE(bat_read_cmd_tbl) },	// read cmd
	{ BAT_CMD_HELP, bat_cmd_get_help, NULL, 0},								// help cmd
	{ BAT_CMD_WRITE, NULL, bat_write_cmd_tbl, ARRAY_SIZE(bat_write_cmd_tbl)},	// write cmd
	{ BAT_CMD_SET, NULL, bat_set_cmd_tbl, ARRAY_SIZE(bat_set_cmd_tbl)},	// set cmd	
	{ BAT_CMD_UPDATE, bat_cmd_update, NULL, 0},
};
#ifdef CONFIG_EEPROM_NUVOTON
static enum power_supply_property pad_bat_properties[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CAPACITY,
};

static enum power_supply_property pad_ac_properties[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
};

#endif
#ifdef CONFIG_ASUSDEC
static enum power_supply_property dock_bat_properties[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CAPACITY,
};

static enum power_supply_property dock_ac_properties[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
};
#endif

#ifdef CONFIG_EEPROM_NUVOTON
//ASUS BSP Eason_Chang : A86 porting +++
static char *pm_power_supplied_to[] = {
	"battery",
};
//ASUS BSP Eason_Chang : A86 porting ---
#endif

/******************************************************************************/
#ifdef CONFIG_EEPROM_NUVOTON
static struct power_supply pad_bat_psy = {
	.name		= "pad_bat",
	.type		= POWER_SUPPLY_TYPE_PAD_BAT,
	.supplied_to = pm_power_supplied_to,
       .num_supplicants = ARRAY_SIZE(pm_power_supplied_to),
	.properties	= pad_bat_properties,
	.num_properties	= ARRAY_SIZE(pad_bat_properties),
	.get_property	= asus_bat_pad_get_property,
};
static struct power_supply pad_ac_psy = {
	.name		= "pad_ac",
	.type		= POWER_SUPPLY_TYPE_PAD_AC,
	.supplied_to = pm_power_supplied_to,
       .num_supplicants = ARRAY_SIZE(pm_power_supplied_to),
	.properties	= pad_ac_properties,
	.num_properties	= ARRAY_SIZE(pad_ac_properties),
	.get_property	= asus_bat_pad_ac_get_property,
};
#endif

#ifdef CONFIG_ASUSDEC
static struct power_supply dock_bat_psy = {
	.name		= "dock_bat",
	.type		= POWER_SUPPLY_TYPE_DOCK_BAT,
	.supplied_to = pm_power_supplied_to,
       .num_supplicants = ARRAY_SIZE(pm_power_supplied_to),
	.properties	= dock_bat_properties,
	.num_properties	= ARRAY_SIZE(dock_bat_properties),
	.get_property	= asus_bat_dock_get_property,
};
static struct power_supply dock_ac_psy = {
	.name		= "dock_ac",
	.type		= POWER_SUPPLY_TYPE_DOCK_AC,
	.supplied_to = pm_power_supplied_to,
       .num_supplicants = ARRAY_SIZE(pm_power_supplied_to),
	.properties	= dock_ac_properties,
	.num_properties	= ARRAY_SIZE(dock_ac_properties),
	.get_property	= asus_bat_dock_ac_get_property,
};
#endif

#ifdef CONFIG_EEPROM_NUVOTON
static struct notifier_block asus_bat_microp_notifier = {
        .notifier_call = asus_bat_microp_event_handler,
};
#endif /* CONFIG_EEPROM_NUVOTON */

/******************************************************************************/
// when A66 Cap = 0% shutdown device no matter if has cable+++ 
#ifdef CONFIG_ASUSDEC
void Dock_AC_PowerSupplyChange(void)
{
    power_supply_changed(&dock_ac_psy);
    printk("[BAT]Dock AC PowerSupplyChange\n");
}
#endif
#ifdef CONFIG_EEPROM_NUVOTON
void Pad_AC_PowerSupplyChange(void)
{
    power_supply_changed(&pad_ac_psy);
    printk("[BAT]Pad AC PowerSupplyChange\n");
}
#endif 
// when A66 Cap = 0% shutdown device no matter if has cable---

// Asus BSP Eason_Chang +++ function for AXI_BatteryServiceFacade
extern void asus_bat_status_change(void);
//ASUS BSP Eason_Chang : A86 porting +++
#if 0
extern void asus_chg_set_chg_mode_forBatteryservice(enum asus_chg_src Batteryservice_src);
#endif
//ASUS BSP Eason_Chang : A86 porting ---

//ASUS BSP Eason_Chang get Cap from TIgauge+++
extern enum DEVICE_HWID g_A68_hwID;
//ASUS BSP Eason_Chang get Cap from TIgauge---
static int asus_getChargingStatus(void)
{  
   int result = POWER_SUPPLY_STATUS_DISCHARGING; 
   AXE_BAT_CHARGING_STATUS status = loService->getChargingStatus(loService);
   
   switch(status){
   case BAT_CHARGING_STATUS:
			//Eason: when Phone 100% can't show charging, workaround when in Pad 100% unplug then plug extChg show charge icon+++
			if(100==loService->getCapacity(loService))
			{
				pr_debug("[BAT]A66 100p cant show CHARGING\n");
				result = POWER_SUPPLY_STATUS_FULL;
			}else{
				result = POWER_SUPPLY_STATUS_CHARGING;
			}
			//Eason: when Phone 100% can't show charging, workaround when in Pad 100% unplug then plug extChg show charge icon ---
                break;
   case BAT_DISCHARGING_STATUS:
             	result = POWER_SUPPLY_STATUS_DISCHARGING;
                break;
   case BAT_NOT_CHARGING_STATUS:
             	result = POWER_SUPPLY_STATUS_NOT_CHARGING;
                break;             
   case BAT_CHARGING_FULL_STATUS:
	            result = POWER_SUPPLY_STATUS_FULL;
                break;
   default:
                printk("[BAT]%s():ERROR mapping\n",__FUNCTION__);
    
   }
   
   if(true == g_IsInRomMode)
   {
	   result = POWER_SUPPLY_STATUS_UNKNOWN;
	   printk("[BAT]%s():Gauge in Rom Mode or I2C Error, show UNKNOWN\n",__FUNCTION__);
   }
   
   return result;
}

static int asus_getCapacity(void)
{
//Eason: prevent charger mode get capacity at qpnp-charger.c before asus_bat.c ready+++
	if(NULL==loService)
	{
		printk("[BAT]Cap not ready, report fake 50\n");
		return 50;
	}	
//Eason: prevent charger mode get capacity at qpnp-charger.c before asus_bat.c ready---
   
   printk("[BAT]Cap:%d\n",loService->getCapacity(loService));  

   return loService->getCapacity(loService);

}
//Eason report capacity to TIgauge when gauge read error, don't update capacity+++
int ReportCapToTIGauge(void)
{
	return asus_getCapacity();
}
//Eason report capacity to TIgauge when gauge read error, don't update capacity---

//Eason repoert capacity to gauge for judge interval+++
int ReportCapToGaugeForJudgeInterval(void)
{
	return asus_getCapacity();
}
//Eason repoert capacity to gauge for judge interval---

//Eason: AICL work around +++
int smb346_getCapacity(void)
{
	int phone_soc;

	phone_soc = asus_getCapacity();
	return phone_soc;
}
//Eason: AICL work around ---

int pm8941_getCapacity(void)
{
	return asus_getCapacity();
}
	
#include "AXI_Basic_Define.h"
void asus_onCableInOut(enum asus_chg_src src)
{
    if(NULL == loService){

        return;
    }

    switch(src){ 
		case ASUS_CHG_SRC_NONE:
            loService->onCableInOut(loService,NO_CHARGER_TYPE);
            break;
        case ASUS_CHG_SRC_DC:
            loService->onCableInOut(loService,HIGH_CURRENT_CHARGER_TYPE);
            break;
        case ASUS_CHG_SRC_UNKNOWN:
            loService->onCableInOut(loService,ILLEGAL_CHARGER_TYPE);
            break;
		case ASUS_CHG_SRC_USB:
			loService->onCableInOut(loService,LOW_CURRENT_CHARGER_TYPE);
			break;
        case ASUS_CHG_SRC_PAD_BAT:
            loService->onCableInOut(loService,NORMAL_CURRENT_CHARGER_TYPE);//NORMAL_CURRENT_CHARGER_TYPE
            break;
        default:
            printk("[BAT]%s():error src\n",__FUNCTION__);
            break;
    }
}

/*
void asus_onChargingStop(int chg_stop_reason)
{   
    switch(chg_stop_reason){ 
		case exCHGDOWN:
            loService->onChargingStop(loService,CHARGING_DONE);
            break;
        case exBATT_REMOVED_ERROR:
            loService->onChargingStop(loService,BATT_REMOVED_ERROR);
            break;
		case exBATTEMP_HOT_ERROR:	
			loService->onChargingStop(loService,HOT_CHARGER_ERROR);
            break;
        case exBATTEMP_COLD_ERROR:
		case exTEMP_OK_ERROR:
        case exDCIN_OV:
        case exDCIN_UV:
        case exUNREG_extDC:
        case ex8921FSM_change:    
            loService->onChargingStop(loService, UNKNOWN_ERROR);
            break;			
        default:
            printk("[BAT]%s():error reason\n",__FUNCTION__);
            break;
    }
}
*/

void asus_onChargingStart(void)
{   
    loService->onChargingStart(loService);
}

static void asus_onBatteryLowAlarm(bool isBattLow)
{
    loService->onBatteryLowAlarm(loService,isBattLow);
}

/*
static void asus_onBatteryRemoved(int Batmov_state)
{  
    loService->onBatteryRemoved(loService,Batmov_state);
}
*/


static void asus_onBatterySuspend(void)
{
    loService->suspend(loService);
}

static void asus_onBatteryResume(void)
{
    loService->resume(loService, 0);
}

void AfterthawProcessResumeBatteryService(void)
{   
    printk("[BAT]thaw Process\n");
    asus_onBatteryResume();
}


static void AsusBatChangeChargingCurrent(struct AXI_BatteryServiceFacadeCallback *callback,AXE_Charger_Type chargertype)
{
//ASUS BSP Eason_Chang : A86 porting +++
#if 0
    switch(chargertype){

        case NO_CHARGER_TYPE:
			asus_chg_set_chg_mode_forBatteryservice(ASUS_CHG_SRC_NONE);
            break;
        case ILLEGAL_CHARGER_TYPE:
            asus_chg_set_chg_mode_forBatteryservice(ASUS_CHG_SRC_UNKNOWN);
            break;
        case LOW_CURRENT_CHARGER_TYPE:
            asus_chg_set_chg_mode_forBatteryservice(ASUS_CHG_SRC_USB);
            break;
        case NORMAL_CURRENT_CHARGER_TYPE:
			asus_chg_set_chg_mode_forBatteryservice(ASUS_CHG_SRC_PAD_BAT);
            break;
        case HIGH_CURRENT_CHARGER_TYPE:
            asus_chg_set_chg_mode_forBatteryservice(ASUS_CHG_SRC_DC);
            break;
        default:
            printk("[BAT]%s():error chargertype\n",__FUNCTION__);
            break;
    }
#endif
//ASUS BSP Eason_Chang : A86 porting ---
}

extern int get_temp_for_ASUSswgauge(void);
extern int get_voltage_for_ASUSswgauge(void);
extern int get_current_for_ASUSswgauge(void);
extern int get_Temp_from_TIgauge(void);
extern int get_Curr_from_TIgauge(void);
static int getVBAT(struct AXI_BatteryServiceFacadeCallback *callback)
{
	if(g_ASUS_hwID == A90_EVB0 )	
		return get_Volt_from_TIgauge();
	else
		return get_voltage_for_ASUSswgauge();
}
static int getIBAT(struct AXI_BatteryServiceFacadeCallback *callback)
{
	if(g_ASUS_hwID == A90_EVB0 )	
		return get_Curr_from_TIgauge();
	else
		return get_current_for_ASUSswgauge();
}
static int getTempBAT(struct AXI_BatteryServiceFacadeCallback *callback)
{
	if(g_ASUS_hwID == A90_EVB0 )	
		return get_Temp_from_TIgauge();
	else
		return get_temp_for_ASUSswgauge();
}
static void AsusBatOnServiceStatusUpdated(struct AXI_BatteryServiceFacadeCallback *callback)
{
   asus_bat_status_change();
}
static AXI_BatteryServiceFacadeCallback goServiceCallback = {
    .changeChargingCurrent = AsusBatChangeChargingCurrent,
    .onServiceStatusUpdated = AsusBatOnServiceStatusUpdated,
    .getVBAT = getVBAT,
    .getIBAT = getIBAT,
    .getTempBAT = getTempBAT,
};



// Asus BSP Eason_Chang --- function for AXI_BatteryServiceFacade








/* asus_bat_handle_cmd_tbl */
static void asus_bat_handle_cmd_tbl(
	const char *msg, 
	int index,
	struct asus_bat_proc_cmd *cmd_tbl,
	int cmd_tbl_size)
{
	int i;

	printk(DBGMSK_BAT_INFO "[BAT] %s(), cmd:%c, idx:%d, tbl size:%d \r\n", 
		__FUNCTION__, msg[index], index, cmd_tbl_size);

	for (i = 0; i < cmd_tbl_size; i++) {
		if (msg[index] == cmd_tbl[i].id) {
			if (NULL != cmd_tbl[i].cmd_tbl) { 
				index += BAT_CMD_INTERVAL;
				asus_bat_handle_cmd_tbl(msg, index, cmd_tbl[i].cmd_tbl, cmd_tbl[i].cmd_tbl_size);
				return;
			} else if (NULL != cmd_tbl[i].func) {
				cmd_tbl[i].func(msg, index);
				return;
			} else {
				printk(DBGMSK_BAT_ERR "[BAT] %s(), no func tbl", __FUNCTION__);
				return;
			}
		}
	}

	if (i == cmd_tbl_size) {
		printk(DBGMSK_BAT_ERR "[BAT]error!! in %s(), no cmd found in tbl", __FUNCTION__);
	}

	return;
}

void asus_bat_report_phone_charger(enum asus_bat_charger_cable charger)
{
	if (NULL == asus_bat)
		return;

	// set charger
	if (!asus_bat->enable_test) {
		asus_bat_set_phone_charger(charger);
		pr_debug( "[BAT] %s(), charger:%s \r\n", __FUNCTION__, charger_txt[charger]);
	}
	return;
}

int asus_bat_report_phone_capacity(int phone_bat_capacity)
{
	int bat_cap=0;

    pr_debug("%s()+++",__FUNCTION__);
	if (NULL == asus_bat) {
		return phone_bat_capacity;
	}

	if (!asus_bat->enable_test)
		asus_bat_set_phone_bat_capacity(phone_bat_capacity);

	if (asus_bat->phone_b.bat_low) {
//		if (!phone_bat_info->get_prop_bat_chgin_byhw()){
		//printk(DBGMSK_BAT_INFO "[BAT] %s(), bat low to shutdown, set cap as:%d \r\n", 
		//	__FUNCTION__, BAT_LOW_SHUTDOWN_CAP);
//		asus_bat_set_phone_bat_capacity(BAT_LOW_SHUTDOWN_CAP);
			bat_cap = BAT_LOW_SHUTDOWN_CAP;
//		}
	} else {	
   		//ASUS_BSP  +++ Eason_Chang "sw gauge support"
		//bat_cap = asus_bat_get_phone_bat_capacity(); //Qualcomm BMS
    	bat_cap = gBatteryLife;  //ASUS SW gauge
   		//ASUS_BSP  --- Eason_Chang "sw gauge support"
	}

	pr_debug( "[BAT] %s(), phone_bat_capacity:%d \r\n", __FUNCTION__, bat_cap);

	//return bat_cap;
//Eason takeoff Battery shutdown +++	
	if(true == g_AcUsbOnline_Change0)
    {
        printk("[BAT]Cap:change capacity 0 to shutdown\n");
        return 0;
    }else{    
        return asus_getCapacity();
    }    
//Eason takeoff Battery shutdown ---    
}
EXPORT_SYMBOL_GPL(asus_bat_report_phone_capacity);


int asus_bat_report_phone_status(int phone_bat_status)
{

	if(NULL == asus_bat) {
		return phone_bat_status;
	}

	if (!asus_bat->enable_test) {		// from hw
		asus_bat_set_phone_bat_status(phone_bat_status);
	} else {		// from variable
		switch (asus_bat_get_phone_charger()) {
		case ASUS_BAT_CHARGER_NONE:
			phone_bat_status = POWER_SUPPLY_STATUS_DISCHARGING;
			break;	
		case ASUS_BAT_CHARGER_USB:
		case ASUS_BAT_CHARGER_AC:	
			if (ASUS_BAT_CHARGING_ONGOING == asus_bat_get_phone_charging()) {
				phone_bat_status = POWER_SUPPLY_STATUS_CHARGING;
			} else {
				phone_bat_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
			}
			break;
		default:
			phone_bat_status = POWER_SUPPLY_STATUS_UNKNOWN;
			printk(DBGMSK_BAT_ERR "[BAT] error !! in %s() \r\n", __FUNCTION__);
		}
	}

	pr_debug(DBGMSK_BAT_INFO "[BAT] %s(), ps charing:%s \r\n", __FUNCTION__,ps_charging_txt[phone_bat_status] );
	
	//return phone_bat_status;
	return asus_getChargingStatus();//
}
EXPORT_SYMBOL_GPL(asus_bat_report_phone_status);

int asus_bat_report_phone_present(int phone_bat_present)
{
	if(NULL == asus_bat) {
		return phone_bat_present;
	}

	if (!asus_bat->enable_test) {
		if (phone_bat_present > 0) {
			asus_bat_set_phone_bat_present(true);
		} else {
			asus_bat_set_phone_bat_present(false);
		}
	}

	if (asus_bat_is_phone_bat_present()) {
		return 1;
	} else {
		return 0;
	}
}
EXPORT_SYMBOL_GPL(asus_bat_report_phone_present);


static int asus_bat_report_pad_capacity(void)
{
	int pad_bat_capacity;

	BUG_ON(NULL == asus_bat);
	
	if (!asus_bat->enable_test) {
		pad_bat_capacity = asus_bat_get_pad_bat_capacity_byhw();
		asus_bat_set_pad_bat_capacity(pad_bat_capacity);
	}

	pad_bat_capacity = asus_bat_get_pad_bat_capacity();

   

    pad_bat_capacity = ReportBatteryServiceP02Cap();//P02 gauge
    printk(DBGMSK_BAT_INFO "[BAT]Pad_Cap:%d \r\n", pad_bat_capacity);
	return pad_bat_capacity;//P02 gauge
}
//ASUS_BSP +++ Eason_Chang BalanceMode
int BatteryServiceReportPADCAP(void)
{   
    return asus_bat_report_pad_capacity();
}
//ASUS_BSP --- Eason_Chang BalanceMode

//ASUS_BSP +++ Eason_Chang BalanceMode
int BatteryServiceGetPADCAP(void)
{   
    //return asus_bat_report_pad_capacity();
    return asus_bat_get_pad_bat_capacity();
}
//ASUS_BSP --- Eason_Chang BalanceMode
#ifdef CONFIG_ASUSDEC
static int asus_bat_report_dock_capacity(void)
{
	int dock_bat_capacity;

	BUG_ON(NULL == asus_bat);

	pr_debug( "[BAT] %s()\n", __FUNCTION__);
	
	if (!asus_bat->enable_test) {
		dock_bat_capacity = asus_bat_get_dock_bat_capacity_byhw();
		asus_bat_set_dock_bat_capacity(dock_bat_capacity);
	}

	dock_bat_capacity = asus_bat_get_dock_bat_capacity();

	pr_info("[BAT] dock_real:%d \r\n", dock_bat_capacity);

    dock_bat_capacity = ReportBatteryServiceDockCap();
    pr_info("[BAT]Dock_Cap:%d \r\n",  dock_bat_capacity);

	return dock_bat_capacity;
}
#endif
#ifdef CONFIG_EEPROM_NUVOTON
static int asus_bat_report_pad_status(void)
{
	int pad_charging_sts;

#if 0 /*Eason: When Pad plug cable, pad battery status will change to charging or full no matter what Pad battery status+++*/
	//Eason: A68 new balance mode +++ test pad capacity need enable_test but will cause pad status error mark this
	/*	
	if (asus_bat->enable_test) { // from variable
		switch (asus_bat_get_pad_charger()) {
		case ASUS_BAT_CHARGER_NONE:
			pad_charging_sts = POWER_SUPPLY_STATUS_DISCHARGING;
			break;	
		case ASUS_BAT_CHARGER_USB:
		case ASUS_BAT_CHARGER_AC:	
			if (ASUS_BAT_CHARGING_ONGOING == asus_bat_get_pad_charging()) {
				pad_charging_sts = POWER_SUPPLY_STATUS_CHARGING;
			} else {
				pad_charging_sts = POWER_SUPPLY_STATUS_NOT_CHARGING;
			}
			break;
		default:
			printk(DBGMSK_BAT_ERR "[BAT] error !! in %s() \r\n", __FUNCTION__);
		}

	} else { 
	*/
	//Eason: A68 new balance mode ---

	// from hw
		//Eason: Pad plug usb show icon & cap can increase+++
		//if (ASUS_BAT_CHARGER_AC != asus_bat_get_pad_charger_byhw()) {
		if ( (ASUS_BAT_CHARGER_AC != asus_bat_get_pad_charger_byhw())&&(ASUS_BAT_CHARGER_USB != asus_bat_get_pad_charger_byhw()) ){	
		//Eason: Pad plug usb show icon & cap can increase---	
			pad_charging_sts = POWER_SUPPLY_STATUS_DISCHARGING;

		} else {
			switch(asus_bat_get_pad_charging_byhw()){
			case ASUS_BAT_CHARGING_ONGOING:
				if(asus_bat_report_pad_capacity() == 100)
					pad_charging_sts = POWER_SUPPLY_STATUS_FULL;     
				else
					pad_charging_sts = POWER_SUPPLY_STATUS_CHARGING;
				break;
			case ASUS_BAT_CHARGING_FULL:
				pad_charging_sts = POWER_SUPPLY_STATUS_FULL;
				break;
			case ASUS_BAT_CHARGING_NONE:                    
				pad_charging_sts = POWER_SUPPLY_STATUS_NOT_CHARGING;
				break;                            
			default:
				pad_charging_sts = POWER_SUPPLY_STATUS_UNKNOWN;
			}
		}
	//}//Eason: A68 new balance mode 

	pr_info( "[BAT] %s(), ps charing:%d \r\n", __FUNCTION__, pad_charging_sts);

#else
	/*Eason: When Pad plug cable, pad battery status will change to charging or full no matter what Pad battery status+++*/
	int pad_charging_sts_hw = 0;
	
	if ( (ASUS_BAT_CHARGER_AC != asus_bat_get_pad_charger_byhw())&&(ASUS_BAT_CHARGER_USB != asus_bat_get_pad_charger_byhw()) ){
		pad_charging_sts = POWER_SUPPLY_STATUS_DISCHARGING;
	} else {
		pad_charging_sts_hw = asus_bat_get_pad_charging_byhw();
		if(asus_bat_report_pad_capacity() == 100)
				pad_charging_sts = POWER_SUPPLY_STATUS_FULL;     
		else
				pad_charging_sts = POWER_SUPPLY_STATUS_CHARGING;	
	}

	pr_info( "[BAT] %s(), pad_charing sts:%d, hw_sts:%d \r\n", __FUNCTION__, pad_charging_sts, pad_charging_sts_hw);
	/*Eason: When Pad plug cable, pad battery status will change to charging or full no matter what Pad battery status----*/
#endif

	return pad_charging_sts;
}
#endif
#ifdef CONFIG_ASUSDEC
static int asus_bat_report_dock_status(void)
{
	int dock_charging_sts;

	if (asus_bat->enable_test) { // from variable

		switch (asus_bat_get_dock_charger()) {
		case ASUS_BAT_CHARGER_NONE:
			dock_charging_sts = POWER_SUPPLY_STATUS_DISCHARGING;
			break;	
		case ASUS_BAT_CHARGER_USB:
		case ASUS_BAT_CHARGER_AC:	
			if (ASUS_BAT_CHARGING_ONGOING == asus_bat_get_dock_charging()) {
				dock_charging_sts = POWER_SUPPLY_STATUS_CHARGING;
			} else {
				dock_charging_sts = POWER_SUPPLY_STATUS_NOT_CHARGING;
			}
			break;
		default:
			printk(DBGMSK_BAT_ERR "[BAT] error !! in %s() \r\n", __FUNCTION__);
		}

	} else { // from hw
		if (ASUS_BAT_CHARGER_AC != asus_bat_get_dock_charger_byhw()) {
			dock_charging_sts = POWER_SUPPLY_STATUS_DISCHARGING;
		} else {
			switch(asus_bat_get_dock_charging_byhw()){
			case ASUS_BAT_CHARGING_ONGOING:
				dock_charging_sts = POWER_SUPPLY_STATUS_CHARGING;                            
				break;
			case ASUS_BAT_CHARGING_FULL:
				dock_charging_sts = POWER_SUPPLY_STATUS_FULL;
				break;
			case ASUS_BAT_CHARGING_NONE:                    
				dock_charging_sts = POWER_SUPPLY_STATUS_NOT_CHARGING;
				break;                            
			default:
				dock_charging_sts = POWER_SUPPLY_STATUS_UNKNOWN;
			}
		}

	}

	pr_info("[BAT] %s(), ps charing:%d \r\n", __FUNCTION__,dock_charging_sts);
	
	return dock_charging_sts;
}
#endif
void BatteryService_P02update(void)
{
    asus_bat_update_all_bat();
}

// Just for pad and dock
static void asus_bat_update_all_bat(void)
{
	pr_debug( "[BAT] %s()n", __FUNCTION__);

#ifdef CONFIG_EEPROM_NUVOTON  //ASUS_BSP Eason_Chang 1120 porting +++
	if (AX_MicroP_IsP01Connected()) {
		power_supply_changed(&pad_bat_psy);
		pr_info("[BAT] pad present, pad bat update\n");
	}
#endif //CONFIG_EEPROM_NUVOTON//ASUS_BSP Eason_Chang 1120 porting ---

#ifdef CONFIG_ASUSDEC
    if (true==reportDockInitReady()){
    	if (AX_MicroP_IsDockReady()) {
    		power_supply_changed(&dock_bat_psy);
    		pr_info("[BAT] dock present, dock bat update\n");
    	}
    }
#endif
	return;
}
#ifdef CONFIG_EEPROM_NUVOTON
extern void AcUsbPowerSupplyChange_pm8941(void);//Eason fix Padmode in charger mode remove AC don't shutdown
static void asus_bat_update_pad_ac_online(void)
{

    power_supply_changed(&pad_ac_psy);
    AcUsbPowerSupplyChange_pm8941();
    printk("[BAT] pad ac online update\n");

	return;
}

void asus_bat_update_PadAcOnline(void)
{
    asus_bat_update_pad_ac_online();
}
#endif
#ifdef CONFIG_ASUSDEC 
static void asus_bat_update_dock_ac_online(void)
{
 
    power_supply_changed(&dock_ac_psy);
    printk("[BAT] dock ac online update\n");

	return;

}
void asus_bat_update_DockAcOnline(void)
{
    asus_bat_update_dock_ac_online();
}
#endif
#ifdef CONFIG_EEPROM_NUVOTON
static int asus_bat_pad_get_property(
	struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	bool pad_present = false; //ASUS_BSP Eason_Chang 1120 porting
	pr_debug( "[BAT] %s(), name: %s , property: %d \r\n", __FUNCTION__, psy->name, (int)psp);

#ifdef CONFIG_EEPROM_NUVOTON  //ASUS_BSP Eason_Chang 1120 porting +++
	if (0 == AX_MicroP_IsP01Connected())
		pad_present = false;
	else
		pad_present = true;
#endif //CONFIG_EEPROM_NUVOTON//ASUS_BSP Eason_Chang 1120 porting ---    

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		if (pad_present) {
			pr_debug( "[BAT] %s(), pad present \r\n", __FUNCTION__);
			val->intval = 1;
		} else {
			pr_debug( "[BAT] %s(), pad not present \r\n", __FUNCTION__);
			val->intval = 0;
		}
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (pad_present) {
			val->intval = asus_bat_report_pad_capacity();
		} else {
			pr_debug ("[BAT] pad not present, cannot get cap\n");
			val->intval = -1;
		}
		break;
	case POWER_SUPPLY_PROP_STATUS:
		if (pad_present) {
			val->intval = asus_bat_report_pad_status();
		} else {
			pr_debug ("[BAT] pad not present, status unknown\n");
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		}
		break;
	default:
		printk(DBGMSK_BAT_ERR "[BAT] %s(), unknown psp:%d \n", __FUNCTION__, (int)psp);
		return -EINVAL;
	}
	return 0;
}
#endif
#ifdef CONFIG_ASUSDEC
static int asus_bat_dock_get_property(
	struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	bool dock_present = false;
	
	pr_debug( "[BAT] %s(), name: %s , property: %d \r\n", __FUNCTION__, psy->name, (int)psp);
#ifdef CONFIG_ASUSDEC
    if(true==reportDockInitReady()){ 
	    if (0 == AX_MicroP_IsDockReady())
		    dock_present = false;
	    else
		    dock_present = true;
    }
#endif	
	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		if (dock_present) {
			pr_debug( "[BAT] %s(), dock present\n", __FUNCTION__);
			val->intval = 1;
		} else {
			pr_debug( "[BAT] %s(), dock not present\n", __FUNCTION__);
			val->intval = 0;
		}
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (dock_present) {
			val->intval = asus_bat_report_dock_capacity();
		} else {
			pr_debug("[BAT] dock not present, cannot get cap\n");
			val->intval = -1;
		}
		break;
	case POWER_SUPPLY_PROP_STATUS:
		if (dock_present) {
			val->intval = asus_bat_report_dock_status();
		} else {
			pr_debug ("[BAT] dock not present, status unknown\n");
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		}
		break;
	default:
		printk(DBGMSK_BAT_ERR "[BAT] %s(), unknown psp:%d \n", __FUNCTION__, (int)psp);
		return -EINVAL;
	}
	return 0;
}
#endif //#ifdef CONFIG_ASUSDEC
//Eason: in Pad AC powered, judge AC powered true+++
int InP03JudgeACpowered(void)
{
#ifdef CONFIG_EEPROM_NUVOTON
	if( 1 == AX_MicroP_IsP01Connected())
	{
		if(ASUS_BAT_CHARGER_AC == asus_bat_get_pad_charger_byhw())
			return 1;
		else
			return 0;
	}else
#endif
			return 0;
}
//Eason: in Pad AC powered, judge AC powered true---
#ifdef CONFIG_EEPROM_NUVOTON

static int asus_bat_pad_ac_get_property(
	struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	bool pad_present = false;//ASUS_BSP Eason_Chang 1120 porting

	pr_debug( "[BAT] %s(), name: %s , property: %d \r\n", __FUNCTION__, psy->name, (int)psp);

#ifdef CONFIG_EEPROM_NUVOTON  //ASUS_BSP Eason_Chang 1120 porting +++
	if (0 == AX_MicroP_IsP01Connected())
		pad_present = false;
	else
		pad_present = true;
#endif //CONFIG_EEPROM_NUVOTON//ASUS_BSP Eason_Chang 1120 porting ---    


	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:
		if (pad_present) {
			//Eason: Pad plug usb show icon & cap can increase+++
			//if (ASUS_BAT_CHARGER_AC == asus_bat_get_pad_charger_byhw()) { 
			if(  (ASUS_BAT_CHARGER_AC == asus_bat_get_pad_charger_byhw())||(ASUS_BAT_CHARGER_USB == asus_bat_get_pad_charger_byhw()) ){ 
			//Eason: Pad plug usb show icon & cap can increase---	
				val->intval = 1;
			} else {
				val->intval = 0;
			}
		} else {
			val->intval = 0;
		}

        if(true == g_AcUsbOnline_Change0)
        {
                val->intval = 0;
                printk("[BAT][Chg][Pad]: set online 0 to shutdown device\n");   
        }

		//Eason: Factory5060Mode+++
		#ifdef ASUS_FACTORY_BUILD
		if( false == g_5060modeCharging)
		{
                val->intval = 0;
                printk("[BAT][Factory][5060][pad_ac]: set online 0 to show Notcharging icon\n");   
		}
		#endif	
		//Eason: Factory5060Mode---
		
		pr_debug("[BAT] pad ac online(%d)\n", val->intval);
		break;
	default:
		pr_err("[BAT] %s(), unknown psp:%d \n", __FUNCTION__, (int)psp);
		return -EINVAL;
	}
	return 0;
}

#endif
#ifdef CONFIG_ASUSDEC
static int asus_bat_dock_ac_get_property(
	struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	bool dock_present = false;

	pr_debug( "[BAT] %s(), name: %s , property: %d \r\n", __FUNCTION__, psy->name, (int)psp);
#ifdef CONFIG_ASUSDEC
    if(true==reportDockInitReady()){
    	if (0 == AX_MicroP_IsDockReady())
    		dock_present = false;
    	else
    		dock_present = true;
    }
#endif
	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:
		if (dock_present) {
			if (ASUS_BAT_CHARGER_AC == asus_bat_get_dock_charger_byhw()) {
				val->intval = 1;
			} else {
				val->intval = 0;
			}
		} else {
		    if (reportDockExtPowerPlug()) {    
				val->intval = 1;
			} else {
				val->intval = 0;
			}
		}

        if(true == g_AcUsbOnline_Change0)
        {
                val->intval = 0;
                printk("[BAT][Chg][dock]: set online 0 to shutdown device\n");   
        }
		pr_debug("[BAT] dock ac online(%d)\n", val->intval);
		break;
	default:
		pr_err("[BAT] %s(), unknown psp:%d \n", __FUNCTION__, (int)psp);
		return -EINVAL;
	}
	return 0;
}
#endif

#ifdef CONFIG_EEPROM_NUVOTON
static int asus_bat_microp_event_handler(
	struct notifier_block *this,
	unsigned long event,
	void *ptr)
{
	mutex_lock(&asus_bat->microp_evt_lock);
	switch (event) {
	case P01_ADD:
		printk(DBGMSK_BAT_INFO "[BAT] %s() +++, P01_ADD \r\n", __FUNCTION__);
		asus_bat_set_pad_bat_present(true);
		//ASUS_BSP  +++ Eason_Chang "add P01 charge"
		//asus_chg_set_chg_mode(ASUS_CHG_SRC_PAD_BAT);//do in AXC_BatteryServiec.c Balance mode
		//ASUS_BSP  --- Eason_Chang "add P01 charge"
		break;	
	case P01_REMOVE: // means P01 removed
		printk(DBGMSK_BAT_INFO "[BAT] %s() +++, P01_REMOVE \r\n", __FUNCTION__);
		asus_bat_set_pad_bat_present(false);
		//ASUS_BSP  +++ Eason_Chang "add P01 charge"
		//asus_chg_set_chg_mode(ASUS_CHG_SRC_PAD_NONE);//Hank: remove unnecessary setcharger 
		//ASUS_BSP  --- Eason_Chang "add P01 charge"
		break;
	case P01_BATTERY_POWER_BAD: // P01 battery low
		printk(DBGMSK_BAT_INFO "[BAT] %s() +++, P01_BATTERY_POWER_BAD \r\n", __FUNCTION__);
		break;
/* 	case P01_BATTERY_TO_CHARGING:
		printk(DBGMSK_BAT_INFO "[BAT] %s() +++, P01_BATTERY_TO_CHARGING \r\n", __FUNCTION__);
		break;
	case P01_BATTERY_TO_NON_CHARGING:
		printk(DBGMSK_BAT_INFO "[BAT] %s() +++, P01_BATTERY_TO_NON_CHARGING \r\n", __FUNCTION__);
		break;
*/	
	case P01_AC_USB_IN:
		printk(DBGMSK_BAT_INFO "[BAT] %s() +++, P01_AC_USB_IN \r\n", __FUNCTION__);
		//msleep(1500);
		break;
	case P01_AC_USB_OUT:
		printk(DBGMSK_BAT_INFO "[BAT] %s() +++, P01_AC_USB_OUT \r\n", __FUNCTION__);
		break;
	case DOCK_INIT_READY:
		printk(DBGMSK_BAT_INFO "[BAT] %s() +++, DOCK_INIT_READY \r\n", __FUNCTION__);
		asus_bat_set_dock_bat_present(true);
		break;
	case DOCK_PLUG_OUT:
		printk(DBGMSK_BAT_INFO "[BAT] %s() +++, DOCK_PLUG_OUT \r\n", __FUNCTION__);
		asus_bat_set_dock_bat_present(false);
		break;
	case DOCK_EXT_POWER_PLUG_IN_READY: // means dock charging
		printk(DBGMSK_BAT_INFO "[BAT] %s() +++, DOCK_EXT_POWER_PLUG_IN \r\n", __FUNCTION__);
		break;
	case DOCK_EXT_POWER_PLUG_OUT_READY:	// means dock discharging
		printk(DBGMSK_BAT_INFO "[BAT] %s() +++, DOCK_EXT_POWER_PLUG_OUT \r\n", __FUNCTION__);
		break;		
	case DOCK_BATTERY_POWER_BAD_READY:
		printk(DBGMSK_BAT_INFO "[BAT] %s() +++, DOCK_BATTERY_POWER_BAD \r\n", __FUNCTION__);
		break;
	default:
		mutex_unlock(&asus_bat->microp_evt_lock);
		//printk(DBGMSK_BAT_INFO "[BAT] %s(), not listened evt: %lu \n", __FUNCTION__, event);
		return NOTIFY_DONE;
	}

	mutex_unlock(&asus_bat->microp_evt_lock);

	asus_bat_update_all_bat();

	return NOTIFY_DONE;
}
#endif /* CONFIG_EEPROM_NUVOTON */
extern void pm8921_chg_set_chg_mode(enum asus_chg_src chg_src);
#ifdef CONFIG_CHARGER_ASUS
extern void asus_chg_set_chg_mode(enum asus_chg_src chg_src)
{
    printk("%s,src=%d\n",__FUNCTION__,chg_src );//ASUS_BSP Eason_Chang 1120 porting

//ASUS_BSP Eason_Chang 1120 porting +++
#if 0//ASUS_BSP Eason_Chang 1120 porting
        if(g_A60K_hwID >=A66_HW_ID_ER2){

            AXI_Charger *lpCharger;

            lpCharger = getAsusCharger();

            switch (chg_src) {
                case ASUS_CHG_SRC_USB:
                    lpCharger->SetCharger(lpCharger,LOW_CURRENT_CHARGER_TYPE);
                    break;
                case ASUS_CHG_SRC_DC:
                    lpCharger->SetCharger(lpCharger,HIGH_CURRENT_CHARGER_TYPE);
                    break;
                case ASUS_CHG_SRC_PAD_BAT:
                    lpCharger->SetCharger(lpCharger,NORMAL_CURRENT_CHARGER_TYPE);
                    break;
                case ASUS_CHG_SRC_NONE:
                    lpCharger->SetCharger(lpCharger,NO_CHARGER_TYPE);
                    break;
                case ASUS_CHG_SRC_UNKNOWN:
                    lpCharger->SetCharger(lpCharger,ILLEGAL_CHARGER_TYPE);
                    break;
                default:
                    break;
            }
        }else{
            pm8921_chg_set_chg_mode(chg_src);

        }
#endif//ASUS_BSP Eason_Chang 1120 porting
        {
            AXI_Charger *lpCharger;

            lpCharger = getAsusCharger();

		if(ASUS_CHG_SRC_PAD_BAT==chg_src)
		{
			IsInPadSetChg = true;
		}else if(ASUS_CHG_SRC_PAD_NONE==chg_src)
		{
			IsInPadSetChg = false;
		}

		if(false==IsInPadSetChg)
		{
	            switch (chg_src) {
	                case ASUS_CHG_SRC_USB:
			      //printk("[BAT]SetCharger  LOW_CURRENT_CHARGER_TYPE\n");
	                    lpCharger->SetCharger(lpCharger,LOW_CURRENT_CHARGER_TYPE);
	                    break;
	                case ASUS_CHG_SRC_DC:
			      //printk("[BAT]SetCharger  HIGH_CURRENT_CHARGER_TYPE\n");
	                    lpCharger->SetCharger(lpCharger,HIGH_CURRENT_CHARGER_TYPE);
	                    break;
	                case ASUS_CHG_SRC_NONE:
			   case ASUS_CHG_SRC_PAD_NONE:
			      //printk("[BAT]SetCharger  NO_CHARGER_TYPE\n");	
	                    lpCharger->SetCharger(lpCharger,NO_CHARGER_TYPE);
	                    break;
	                case ASUS_CHG_SRC_UNKNOWN:
			      //printk("[BAT]SetCharger  ILLEGAL_CHARGER_TYPE\n");
	                    lpCharger->SetCharger(lpCharger,ILLEGAL_CHARGER_TYPE);
	                    break;
			   //ASUS_BSP:Eason for Wireless charger+++
		#ifdef CONFIG_IDTP9023_CHARGER
			   case ASUS_CHG_SRC_WC:
			      //printk("[BAT]SetCharger  ILLEGAL_CHARGER_TYPE\n");
	                    lpCharger->SetCharger(lpCharger,WC_CHARGER_TYPE);
	                    break;	
		#endif		
			   //ASUS_BSP:Eason for Wireless charger---
	                default:
	                    break;
	            }
		}else if(true==IsInPadSetChg){
	            switch (chg_src) {
	                case ASUS_CHG_SRC_PAD_BAT:
	                    lpCharger->SetCharger(lpCharger,NORMAL_CURRENT_CHARGER_TYPE);
	                    break;
	                default:
	                    break;
	            }
		}
        }
//ASUS_BSP Eason_Chang 1120 porting ---        
}
#endif
void asus_bat_set_phone_bat(struct asus_bat_phone_bat_struct *pbs)
{
	BUG_ON(NULL == pbs);
	pr_debug( "[BAT] %s()", __FUNCTION__);

	phone_bat_info= pbs;

	return;
}
EXPORT_SYMBOL_GPL(asus_bat_set_phone_bat);

#if 0
void asus_bat_set_pad_bat(struct power_supply *psy)
{
	BUG_ON(NULL == psy);
	pad_bat_psy = psy;	
	return;
}

void asus_bat_set_dock_bat(struct power_supply *psy)
{
	BUG_ON(NULL == psy);
	dock_bat_psy = psy;	
	return;
}
#endif

/* asus_bat_periodic_update - periodically update battery status
  */
static void asus_bat_periodic_update(struct work_struct *work)
{
	if (!g_done_first_periodic_update) {
		g_done_first_periodic_update = true;
		printk(DBGMSK_BAT_INFO "[BAT] done first periodic update \r\n");
	}

	asus_bat_update_all_bat();

	queue_delayed_work(asus_bat_wq,
		&asus_bat->bat_update_work, (asus_bat->bat_update_work_interval)*HZ);
}




//Eason takeoff Battery shutdown +++
static void checkIfTakeOffBat(int TakeOffBatVolt)
{
	//Hank TakeOffBatVolt get from TIgauge++
    if(TakeOffBatVolt<=1000)
    //Hank TakeOffBatVolt get from TIgauge--
    {
         g_AcUsbOnline_Change0 = true;
         AcUsbPowerSupplyChange();
         
         asus_bat_status_change();
         printk("[BAT][remove]:take off Battery,shutdown device\n");
    }
            
}
//Eason takeoff Battery shutdown ---




static void asus_phone_bat_low_work(struct work_struct *work)
{
    int bat_low_gpio_value;
	int i, count=0;
//Eason takeoff Battery shutdown +++    
    int batLowWorkVoltage;
	if(g_ASUS_hwID == A90_EVB0 )
	{
		batLowWorkVoltage = get_Volt_from_TIgauge();
	}else{
		batLowWorkVoltage = get_voltage_for_ASUSswgauge();
	}
//Eason takeoff Battery shutdown ---    
	bat_low_gpio_value = gpio_get_value(asus_bat->phone_bat_low_gpio);  // bat_low:0  not_bat_low:1
	printk(DBGMSK_BAT_INFO "[BAT] phone bat volt from TIgauge: %dmV \r\n", batLowWorkVoltage);//Eason takeoff Battery shutdown
    printk(DBGMSK_BAT_INFO "[BAT] %s(),bat_low_gpio_value: %d \r\n", __FUNCTION__, bat_low_gpio_value);

//Eason takeoff Battery shutdown +++
    checkIfTakeOffBat(batLowWorkVoltage);
//Eason takeoff Battery shutdown ---
	if (!bat_low_gpio_value) {
        for(i=0;i<10;i++){
        	if(!gpio_get_value(asus_bat->phone_bat_low_gpio)){
				count++;
				msleep(10);
			}else{
			    printk( "[BAT] %s(),leave_bat_low_debounce gpio 0: %d \r\n", __FUNCTION__,\
					                                  gpio_get_value(asus_bat->phone_bat_low_gpio));
			    break;
			}
        }   

        if(count==10){
			printk( "[BAT]  %s(), set phone bat low as true\r\n", __FUNCTION__);
			asus_bat->phone_b.bat_low = true;
			asus_onBatteryLowAlarm(asus_bat->phone_b.bat_low);
		}
	} else {
        for(i=0;i<10;i++){
        	if(gpio_get_value(asus_bat->phone_bat_low_gpio)){
				count++;
				msleep(10);
			}else{
			    printk(DBGMSK_BAT_INFO "[BAT] %s(),leave_bat_low_debounce gpio 1: %d \r\n", __FUNCTION__,\
					                                  gpio_get_value(asus_bat->phone_bat_low_gpio));
			    break;
			}
        }

        if(count==10){
			printk(DBGMSK_BAT_INFO "[BAT]  %s(), set phone bat low as false\r\n", __FUNCTION__);
			asus_bat->phone_b.bat_low = false;
            //Eason boot up in BatLow situation, take off cable can shutdown+++
            g_BootUp_IsBatLow = false;
            //Eason boot up in BatLow situation, take off cable can shutdown---
			asus_onBatteryLowAlarm(asus_bat->phone_b.bat_low);
        }
	}

/*
	//if (POWER_SUPPLY_STATUS_CHARGING != phone_bat_info->get_prop_bat_status_byhw()) {
	if (!phone_bat_info->get_prop_bat_chgin_byhw()) {
		printk(DBGMSK_BAT_INFO "[BAT]  %s(), set phone bat low as true\r\n", __FUNCTION__);
		asus_bat->phone_b.bat_low = true;
	} else {
		printk(DBGMSK_BAT_INFO "[BAT]  %s(), set phone bat low as false\r\n", __FUNCTION__);
		asus_bat->phone_b.bat_low = false;
	}
*/
	phone_bat_info->phone_bat_power_supply_changed();
}

//Hank:move bootup_check_BatLow() from driver probe to capacity work to speed up driver probe+++
#if 0
//Eason boot up in BatLow situation, take off cable can shutdown+++
static void bootup_check_BatLow(void)
{
    int bat_low_gpio_value;
	int i, count=0;

	bat_low_gpio_value = gpio_get_value(asus_bat->phone_bat_low_gpio);  // bat_low:0  not_bat_low:1
    printk(DBGMSK_BAT_INFO "[BAT][BootUp] %s(),bat_low_gpio_value: %d \r\n", __FUNCTION__, bat_low_gpio_value);


	if (!bat_low_gpio_value) {
        for(i=0;i<10;i++){
        	if(!gpio_get_value(asus_bat->phone_bat_low_gpio)){
				count++;
				msleep(10);
			}else{
			    printk( "[BAT][BootUp] %s(),leave_bat_low_debounce gpio 0: %d \r\n", __FUNCTION__,\
					                                  gpio_get_value(asus_bat->phone_bat_low_gpio));
			    break;
			}
        }   

        if(count==10){
			printk( "[BAT][BootUp]  %s(), set phone bat low as true\r\n", __FUNCTION__);
			g_BootUp_IsBatLow = true;
		}
	} else {
        for(i=0;i<10;i++){
        	if(gpio_get_value(asus_bat->phone_bat_low_gpio)){
				count++;
				msleep(10);
			}else{
			    printk(DBGMSK_BAT_INFO "[BAT][BootUp] %s(),leave_bat_low_debounce gpio 1: %d \r\n", __FUNCTION__,\
					                                  gpio_get_value(asus_bat->phone_bat_low_gpio));
			    break;
			}
        }

        if(count==10){
			printk(DBGMSK_BAT_INFO "[BAT][BootUp]  %s(), set phone bat low as false\r\n", __FUNCTION__);
			g_BootUp_IsBatLow = false;
        }
	}
	

}
//Eason boot up in BatLow situation, take off cable can shutdown---
#endif
//Hank:move bootup_check_BatLow() from driver probe to capacity work to speed up driver probe---
static struct proc_dir_entry *bat_proc_file = NULL;

#if 0
static void asus_bat_diag_read_cmd(const char *msg)
{
	switch (msg[2]) {
	case '0':	//phone
		break;
	case '1':	//pad
		break;
	case '2':	//dock
		break;
	default:
		printk(KERN_ERR "[BAT] error cmd in %s()", __FUNCTION__);
	}

	return;
}

static void asus_bat_diag_help_cmd(const char *msg)
{
	return;
}

static void asus_bat_diag_handle_all_cmd(const char *msg)
{
	printk(DBGMSK_BAT_INFO "[BAT] %s()+++", __FUNCTION__);

	switch (msg[0]) {
	case 'r':  //read status
		asus_bat_diag_read_cmd(msg);
		break;
	case 'h':  //show help info
		asus_bat_diag_help_cmd(msg);
		break;
	default:
		printk(KERN_ERR "[BAT] error cmd in %s()", __FUNCTION__);
	}
	return;
}
#endif

static ssize_t asus_bat_read_proc(char *page, char **start, off_t off, int count, 
	int *eof, void *data)
{
	printk(DBGMSK_BAT_INFO "[BAT] %s() +++\n", __FUNCTION__);

	return 0;
}

static ssize_t asus_bat_write_proc(struct file *filp, const char __user *buff, 
	unsigned long len, void *data)
{
	char msg[ASUS_BAT_PROC_MAX_BUFF_SIZE];
	if (len > ASUS_BAT_PROC_MAX_BUFF_SIZE)
		len = ASUS_BAT_PROC_MAX_BUFF_SIZE;

	printk(DBGMSK_BAT_INFO "[BAT] %s() +++\n", __FUNCTION__);

	if (copy_from_user(msg, buff, len))
		return -EFAULT;


	asus_bat_handle_cmd_tbl(msg, 0, bat_cmd_tbl, ARRAY_SIZE(bat_cmd_tbl));

#if 0
	asus_bat_diag_handle_all_cmd(msg);
#endif

	return len;
}

static void asus_bat_create_bat_proc_file(void)
{
	pr_debug( "[BAT] %s()+++", __FUNCTION__);

	bat_proc_file = create_proc_entry(
		ASUS_BAT_PROC_FILE,
		ASUS_BAT_PROC_FILE_PERMISSION,
		NULL);

	if (NULL == bat_proc_file) {
		printk(KERN_ERR "[BAT] bat proc file created failed!\n");
		return;
	}

	bat_proc_file->read_proc = asus_bat_read_proc;
	bat_proc_file->write_proc = asus_bat_write_proc;
	return;
}

//Eason: LowCapCpuThrottle +++
void notifyMpdecisionCpuThrottle(bool IsCpuThrottle)
{
	char *envp[3];
	envp[0] = "lowCapCpuThrottle_event";
	envp[1] = NULL;

	if(true == IsCpuThrottle)
	{
		kobject_uevent_env(kobj,KOBJ_OFFLINE,envp);	
		printk("[BAT][ASUS][Throttle]OFF\n");
	}
	else
	{
		kobject_uevent_env(kobj,KOBJ_ONLINE,envp);
		printk("[BAT][Throttle]ON\n");
	}
}

static ssize_t LowCapCpuThrottle_read_proc(char *page, char **start, off_t off, int count, 
            	int *eof, void *data)
{
	return sprintf(page, "%d\n", IsInCpuThrottle);//Eason:let mpdicision check cpu throttle status
}
static ssize_t LowCapCpuThrottle_write_proc(struct file *filp, const char __user *buff, 
	            unsigned long len, void *data)
{
	int val;

	char messages[256];

	if (len > 256) {
		len = 256;
	}

	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}
	
	val = (int)simple_strtol(messages, NULL, 10);

	LowCapCpuThrottle = val;

	if(1==LowCapCpuThrottle)
	{
		notifyMpdecisionCpuThrottle(true);
		IsInCpuThrottle = true;
	}else{
		notifyMpdecisionCpuThrottle(false);
		IsInCpuThrottle = false;
	}
     
        printk("[BAT][ASUS][Throttle]mode:%d\n",val);

	return len;
}

void static create_LowCapCpuThrottle_proc_file(void)
{
	struct proc_dir_entry *LowCapCpuThrottle_proc_file = create_proc_entry("driver/LowCapCpuThrottle", 0644, NULL);

	if (LowCapCpuThrottle_proc_file) {
		LowCapCpuThrottle_proc_file->read_proc = LowCapCpuThrottle_read_proc;
		LowCapCpuThrottle_proc_file->write_proc = LowCapCpuThrottle_write_proc;
	}
	else {
		printk("[BAT][ASUS][Throttle]LowCapCpuThrottle proc file create failed!\n");
	}

	return;
}
//Eason: LowCapCpuThrottle ---

//Hank: add charge ststus proc file+++
/*
static ssize_t asus_bat_charge_status_read_proc(char *page, char **start, off_t off, int count, 
            	int *eof, void *data)
{
	if(asus_getChargingStatus() == POWER_SUPPLY_STATUS_FULL || asus_getChargingStatus() == POWER_SUPPLY_STATUS_CHARGING)
		return sprintf(page, "%d\n", 1);
	else
		return sprintf(page, "%d\n", 0);
}
static ssize_t asus_bat_charge_status_write_proc(struct file *filp, const char __user *buff, 
	            unsigned long len, void *data)
{
	int val;

	char messages[256];

	if (len > 256) {
		len = 256;
	}

	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}
	
	val = (int)simple_strtol(messages, NULL, 10);


    
    	printk("[BAT]Do not support write\n");
	
	return len;
}

void static asus_bat_charge_status_create_proc_file(void)
{
	struct proc_dir_entry *chargeststus_proc_file = create_proc_entry("driver/charge_status", 0644, NULL);

	if (chargeststus_proc_file) {
		chargeststus_proc_file->read_proc = asus_bat_charge_status_read_proc;
		chargeststus_proc_file->write_proc = asus_bat_charge_status_write_proc;
	}
    else {
		printk("[BAT]proc file create failed!\n");
    }

	return;
}
*/
//Hank: add charge ststus proc file---

#if 0
void asus_bat_pad_bat_add(void)
{

}

void asus_bat_pad_bat_remove(void)
{

}

void asus_bat_dock_bat_add(void)
{

}

void asus_bat_dock_bat_remove(void)
{

}

// register_microp_notifier


static int asus_bat_microp_event_handler(void)
{

	asus_bat_pad_bat_add();
	return 0;
}

static int asus_bat_ec_event_handler(void)
{

	asus_bat_dock_bat_add();
	return 0;
}
#endif

#define DEFAULT_THRESHOLD_LOWER		3400
#define DEFAULT_HOLD_TIME		PM8XXX_BATT_ALARM_HOLD_TIME_16_MS
#define DEFAULT_USE_PWM			1
#define DEFAULT_PWM_SCALER		9
#define DEFAULT_PWM_DIVIDER		8



static int asus_battery_suspend(struct device *dev)
{
    printk("[BAT]suspend\r\n");
   
    asus_onBatterySuspend();
    return 0;
}

static int asus_battery_resume(struct device *dev)
{
	//printk("[BAT]resume\r\n");
    asus_onBatteryResume();
      
    return 0;
}

//Hank: 1025 use frame buffer notifier to implement earlysuspend+++
#ifdef CONFIG_FB
static void asus_onBatteryForceResume(void)
{
    loService->forceResume(loService, 0);
}

static void asus_onBatteryDockSuspend(void)
{
    loService->dockSuspend(loService);
}

//ASUS BSP+++  Eason_Chang  SW gauge resume
static void asus_battery_early_suspend(void)
{
	//printk("[BAT]asus_battery_early_suspend\r\n");
	asus_onBatteryDockSuspend();
}

static void asus_battery_late_resume(void)
{
	//printk("[BAT]late_resume\r\n");
	asus_onBatteryForceResume();
}

static int asus_bat_fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	static int blank_old = 0;
	int *blank;

	if (evdata && evdata->data && event == FB_EVENT_BLANK) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK) {
			if (blank_old == FB_BLANK_POWERDOWN) {
				blank_old = FB_BLANK_UNBLANK;
				asus_battery_late_resume();
			}
		} else if (*blank == FB_BLANK_POWERDOWN) {
			if (blank_old == 0 || blank_old == FB_BLANK_UNBLANK) {
				blank_old = FB_BLANK_POWERDOWN;
				asus_battery_early_suspend();
			}
		}
	}

	return 0;
}
//ASUS BSP---  Eason_Chang SW gauge resume
#endif //#ifdef CONFIG_FB
//Hank: 1025 use frame buffer notifier to implement earlysuspend---

#ifdef CONFIG_HAS_EARLYSUSPEND
static void asus_onBatteryForceResume(void)
{
    loService->forceResume(loService, 0);
}

static void asus_onBatteryDockSuspend(void)
{
    loService->dockSuspend(loService);
}

//ASUS BSP+++  Eason_Chang  SW gauge resume
static void asus_battery_early_suspend(struct early_suspend *h)
{
	//printk("[BAT]asus_battery_early_suspend\r\n");
	asus_onBatteryDockSuspend();
}

static void asus_battery_late_resume(struct early_suspend *h)
{
	//printk("[BAT]late_resume\r\n");
	asus_onBatteryForceResume();
}

struct early_suspend asus_battery_early_suspend_handler = {
    .level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN, 
    .suspend = asus_battery_early_suspend,
    .resume = asus_battery_late_resume,
};
//ASUS BSP---  Eason_Chang SW gauge resume
#endif //#ifdef CONFIG_HAS_EARLYSUSPEND

/* Network device notifier chain handler. */
static int bat_low_alarm(struct notifier_block *this, unsigned long event,
			     void *ptr)
{
	pr_debug( "[BAT] %s(), event: %ld \r\n", __FUNCTION__, event);

	pr_debug( "[BAT] phone bat volt from TIgauge: %dmV \r\n", get_Volt_from_TIgauge());
	pr_debug( "[BAT] phone bat curr from HW: %d \r\n", phone_bat_info->get_prop_batt_curr_byhw());

	if (POWER_SUPPLY_STATUS_CHARGING != phone_bat_info->get_prop_bat_status_byhw()) {
		printk(DBGMSK_BAT_INFO "[BAT]  %s(), set phone bat low as true\r\n", __FUNCTION__);
		asus_bat->phone_b.bat_low = true;
	} else {
		printk(DBGMSK_BAT_INFO "[BAT]  %s(), set phone bat low as false\r\n", __FUNCTION__);
		asus_bat->phone_b.bat_low = false;
	}

	phone_bat_info->phone_bat_power_supply_changed();

	return NOTIFY_DONE;
}


static struct notifier_block bat_alarm_notifier = {
	.notifier_call = bat_low_alarm,
};

static int asus_bat_bat_alarm_cfg(void)
{
	int rc;

	pr_debug( "[BAT] %s()+++ \r\n", __FUNCTION__);
			
	/* Use default values when no platform data is provided. */
	rc = pm8xxx_batt_alarm_threshold_set(PM8XXX_BATT_ALARM_LOWER_COMPARATOR,
		DEFAULT_THRESHOLD_LOWER);
	if (rc) {
		printk( DBGMSK_BAT_ERR "[BAT] threshold_set failed, rc=%d\n", rc);
		goto done;
	}

	rc = pm8xxx_batt_alarm_hold_time_set(DEFAULT_HOLD_TIME);
	if (rc) {
		printk( DBGMSK_BAT_ERR "[BAT] hold_time_set failed, rc=%d\n", rc);
		goto done;
	}

	rc = pm8xxx_batt_alarm_pwm_rate_set(DEFAULT_USE_PWM,
			DEFAULT_PWM_SCALER, DEFAULT_PWM_DIVIDER);
	if (rc) {
		printk( DBGMSK_BAT_ERR "[BAT] pwm_rate_set failed, rc=%d\n", rc);
		goto done;
	}

	rc = pm8xxx_batt_alarm_register_notifier(&bat_alarm_notifier);
	if (rc) {
		printk( DBGMSK_BAT_ERR "[BAT] batt alarm register failed, rc=%d\n", rc);
		goto done;
	}

	rc = pm8xxx_batt_alarm_enable(PM8XXX_BATT_ALARM_LOWER_COMPARATOR);
	if (rc) {
		printk( DBGMSK_BAT_ERR"[BAT] disable lower failed, rc=%d\n", rc);
		goto done;
	}

done:
	return rc;

}


static irqreturn_t asus_bat_phone_bat_low_handler(int irq, void *dev_id)
{
	printk(DBGMSK_BAT_INFO "[BAT]%s() triggered \r\n", __FUNCTION__);

	schedule_delayed_work(&asus_bat->phone_b.bat_low_work, 0); 

	return IRQ_HANDLED;
}

static int asus_bat_probe(struct platform_device *pdev)
{
	int err=-1;
	struct resource *res;
// Asus BSP Eason_Chang +++ function for AXI_BatteryServiceFacade
    AXI_BatteryServiceFacade *batteryloService = getBatteryService(&goServiceCallback);
    loService = batteryloService;

// Asus BSP Eason_Chang --- function for AXI_BatteryServiceFacade
	printk( "[BAT] %s() +++\n", __FUNCTION__);

	asus_bat = kzalloc(sizeof(struct asus_bat_all_info), GFP_KERNEL);
	if (!asus_bat) {
		printk(KERN_ERR "[BAT] could not allocate memory\n");
		return -ENOMEM;
	}

	
	#ifdef CONFIG_EEPROM_NUVOTON
	err = power_supply_register(&pdev->dev, &pad_ac_psy);
	if (err < 0) {
		printk(KERN_ERR "power_supply_register pad_ac_psy failed, err = %d\n", err);
		goto unregister_dock_bat;
	}
	
	err = power_supply_register(&pdev->dev, &pad_bat_psy);
	if (err < 0) {
		printk(KERN_ERR "power_supply_register pad_bat_psy failed, err = %d\n", err);
		goto unregister_pad_bat;
	}
	
	#endif
	
	#ifdef CONFIG_ASUSDEC 
	err = power_supply_register(&pdev->dev, &dock_bat_psy);
	if (err < 0) {
		printk(KERN_ERR "power_supply_register dock_bat_psy failed, err = %d\n", err);
		goto unregister_pad_bat;
	}

	err = power_supply_register(&pdev->dev, &dock_ac_psy);
	if (err < 0) {
		printk(KERN_ERR "power_supply_register dock_ac_psy failed, err = %d\n", err);
		goto unregister_pad_ac;
	}
	#endif
	
#ifdef CONFIG_EEPROM_NUVOTON
	register_microp_notifier(&asus_bat_microp_notifier);
	notify_register_microp_notifier(&asus_bat_microp_notifier, "asus_bat"); //ASUS_BSP Lenter+
#endif /* CONFIG_EEPROM_NUVOTON */

	asus_bat_set_pad_bat_present(false);
	asus_bat_set_dock_bat_present(false);

	mutex_init(&asus_bat->hs_evt_lock);
	mutex_init(&asus_bat->microp_evt_lock);

	


	INIT_DELAYED_WORK(&asus_bat->phone_b.bat_low_work, asus_phone_bat_low_work);


	asus_bat->bat_update_work_interval = ASUS_BAT_DEF_UPDATE_RATE;

	INIT_DELAYED_WORK(&asus_bat->bat_update_work, asus_bat_periodic_update);

	asus_bat_wq = create_singlethread_workqueue("asus_bat_workqueue");
	if (!asus_bat_wq) {
		err= -ESRCH;
		goto free_asus_bat_wq;
	}

	queue_delayed_work(asus_bat_wq, &asus_bat->bat_update_work, HZ * BAT_FIRST_UPDATE_DELAY);

	/* set bat low alarm */
	if (0) {
		asus_bat_bat_alarm_cfg();
	}

//if (0) {
	res = platform_get_resource_byname(pdev, IORESOURCE_IO, "bat_low_gpio");
	if (res) {
		asus_bat->phone_bat_low_gpio = res->start;
		pr_debug( "[BAT]bat_low_gpio: %d\r\n", asus_bat->phone_bat_low_gpio);
	} else {
		printk(DBGMSK_BAT_ERR "[BAT]can't find bat low gpio\r\n");
		goto err_get_resource;
	}
	
	/* config bat low gpio */
	err  = gpio_request(asus_bat->phone_bat_low_gpio, pdev->name);
	if (err < 0) {
		printk(DBGMSK_BAT_ERR "gpio_request ASUS_BAT_LOW_GPIO failed, err = %d\n", err);
		goto free_asus_bat_wq;
	}

	err = gpio_direction_input(asus_bat->phone_bat_low_gpio);
	if (err  < 0) {
		printk(DBGMSK_BAT_ERR "gpio_direction_input ASUS_BAT_LOW_GPIO failed, err = %d\n", err);
		goto free_gpio;
	}

	asus_bat->phone_bat_low_irq = gpio_to_irq(asus_bat->phone_bat_low_gpio);

	enable_irq_wake(asus_bat->phone_bat_low_irq );

//+++++ASUS_BSP  Eason_Chang : rising BATlow	
	err = request_irq(asus_bat->phone_bat_low_irq , asus_bat_phone_bat_low_handler, \
			IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, pdev->name, NULL);
//-----ASUS_BSP  Eason_Chang : rising BATlow	

    	if (err) {
		printk(DBGMSK_BAT_ERR "request_irq ASUS_BAT_LOW_GPIO failed, err = %d\n", err);
		goto free_gpio;
    	} 
//}

     //Hank:move bootup_check_BatLow() from driver probe to capacity work to speed up driver probe+++
     #if 0
    //Eason boot up in BatLow situation, take off cable can shutdown+++
    bootup_check_BatLow();
    //Eason boot up in BatLow situation, take off cable can shutdown---
    #endif	
    //Hank:move bootup_check_BatLow() from driver probe to capacity work to speed up driver probe---
    
    //Eason Calculate Cap after judge BatLow+++
    loService ->forceResume(loService,TIME_FOR_FIRST_START_BATTERY_SERVER_IN_SECONDS);
    //Eason Calculate Cap after judge BatLow---
    
	asus_bat_create_bat_proc_file();
	//Hank: add charge status proc file+++
	//asus_bat_charge_status_create_proc_file();
	//Hank: add charge status proc file---
	//Eason: LowCapCpuThrottle +++
	create_LowCapCpuThrottle_proc_file();
	//Eason: LowCapCpuThrottle ---

//ASUS BSP+++  Eason_Chang  SW gauge resume
#if defined(CONFIG_HAS_EARLYSUSPEND)
	register_early_suspend(&asus_battery_early_suspend_handler);
//ASUS BSP---  Eason_Chang  SW gauge resume
//Hank: 1025 use frame buffer notifier to implement earlysuspend+++
#elif defined(CONFIG_FB)
	bat_fb_notif.notifier_call = asus_bat_fb_notifier_callback;
	err= fb_register_client(&bat_fb_notif);
	if (err)
		dev_err(&pdev->dev, "Unable to register fb_notifier: %d\n", err);
#endif
//Hank: 1025 use frame buffer notifier to implement earlysuspend---
    

//ASUS BSP Eason Chang coincell+++
    pm8xxx_coincell_chg_config(&bat_chg_config);
//ASUS BSP Eason Chang coincell---

	//Eason Send uevent to userspace+++
	kobj = &pdev->dev.kobj;
	//Eason Send uevent to userspace---

	printk( "[BAT] %s() ---\n", __FUNCTION__);
	
	return 0;


free_gpio:
	gpio_free(ASUS_BAT_LOW_GPIO);
err_get_resource:


free_asus_bat_wq:
	cancel_delayed_work(&asus_bat->bat_update_work);
	destroy_workqueue(asus_bat_wq);
#ifdef CONFIG_ASUSDEC  
	power_supply_unregister(&dock_ac_psy);
#endif

#ifdef CONFIG_ASUSDEC  
unregister_pad_ac:

#ifdef CONFIG_EEPROM_NUVOTON
	power_supply_unregister(&pad_ac_psy);
#endif

#endif

#ifdef CONFIG_EEPROM_NUVOTON
unregister_dock_bat:
#ifdef CONFIG_ASUSDEC  
	power_supply_unregister(&dock_bat_psy);
#endif
#endif


#ifdef CONFIG_EEPROM_NUVOTON
unregister_pad_bat:
	power_supply_unregister(&pad_bat_psy);
#endif

	kfree(asus_bat);
	return err;

}

static int asus_bat_remove(struct platform_device *pdev)
{
	gpio_free(ASUS_BAT_LOW_GPIO);
	cancel_delayed_work(&asus_bat->bat_update_work);
	destroy_workqueue(asus_bat_wq);
	#ifdef CONFIG_ASUSDEC  
	power_supply_unregister(&dock_ac_psy);
	power_supply_unregister(&dock_bat_psy);
	#endif
	#ifdef CONFIG_EEPROM_NUVOTON
	power_supply_unregister(&pad_ac_psy);
	power_supply_unregister(&pad_bat_psy);
	#endif
	kfree(asus_bat);
	asus_bat = NULL;
	return 0;
}

static const struct dev_pm_ops asusbat_pm_ops = {
    .suspend = asus_battery_suspend,
    .resume = asus_battery_resume,
};

static struct platform_driver asus_bat_driver = {
	.probe	= asus_bat_probe,
	.remove	= asus_bat_remove,
	.driver	= {
		.name	= "asus_bat",
		.owner	= THIS_MODULE,
		.pm	= &asusbat_pm_ops,
	},
};

static int __init asus_bat_init(void)
{
	return platform_driver_register(&asus_bat_driver);
}

static void __exit asus_bat_exit(void)
{
	platform_driver_unregister(&asus_bat_driver);
}

late_initcall(asus_bat_init);
module_exit(asus_bat_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ASUS battery virtual driver");
MODULE_VERSION("1.0");
MODULE_AUTHOR("Josh Liao <josh_liao@asus.com>");

//ASUS_BSP --- Josh_Liao "add asus battery driver"


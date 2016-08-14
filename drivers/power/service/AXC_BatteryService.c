#ifdef CONFIG_BATTERY_ASUS_SERVICE
#include <AXC_BatteryService.h>
#include <linux/kernel.h>
#include <linux/rtc.h>
//ASUS BSP Eason_Chang +++ batteryservice to fsm
#include "../fsm/AXC_Charging_FSM.h"
AXC_Charging_FSM *lpFSM;
//ASUS BSP Eason_Chang --- batteryservice to fsm
//ASUS BSP Eason_Chang +++ batteryservice to gauge
#include "../gauge/axc_gaugefactory.h"
#include "../gauge/axi_gauge.h"
#include "../gauge/AXI_CapacityFilter.h"
#include "../capfilter/axc_cap_filter_factory.h"
#include "../capfilter/axi_cap_filter.h"
#include "../capfilter/axc_cap_filter_a66.h"
#include "../capfilter/axc_cap_filter_p02.h"
#include <linux/time.h>
//ASUS BSP Eason_Chang --- batteryservice to gauge
//ASUS_BSP +++ Eason_Chang BalanceMode

#include <linux/asus_chg.h>
#include <linux/notifier.h>
#ifdef CONFIG_EEPROM_NUVOTON 
#include <linux/microp_api.h> 

 //ASUS_BSP Eason_Chang 1120 porting +++
#include <linux/microp_pin_def.h>
#include <linux/microp_notify.h>
#include <linux/microp_notifier_controller.h>	//ASUS_BSP Lenter+
//ASUS_BSP Eason_Chang 1120 porting ---

#include <linux/microp.h> 
#endif //CONFIG_EEPROM_NUVOTON/

//ASUS_BSP +++ Peter_lu "suspend for Battery0% in  fastboot mode issue"
#ifdef CONFIG_FASTBOOT
#include <linux/fastboot.h>
#endif //#ifdef CONFIG_FASTBOOT
//ASUS_BSP ---

//Eason: do ForcePowerBankMode+++
extern int uP_nuvoton_write_reg(int cmd, void *data);
//Eason: do ForcePowerBankMode---

//[ChiaYuan][+++]Add for Broadcast busy state
#include "../../../kernel/power/power.h"
#include <linux/suspend.h>
#define HIGH_POWER_TIMEOUT   1000*60*60		//1 hour timeout
int Total_BatCur = 0;
int HighPower_Count = 0;
bool bTimerEnable = false;
int IsHighPower = 0;
extern suspend_state_t pm_autosleep_state(void);
void HighPower_timer_expired(unsigned long data);
DEFINE_TIMER(HighPower_timer, HighPower_timer_expired, 0, 0);	
//[ChiaYuan][---]Add for Broadcast busy state

#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include "../charger/axi_charger.h" 
static int IsBalanceMode = 1;//default 1.  0:PowerbankMode, 1:balanceMode, 2:ForcePowerBankMode
extern int IsBalanceTest(void);
extern int GetBalanceModeStartRatio(void);
extern int GetBalanceModeStopRatio(void);
extern int GetBalanceModeA66CAP(void);
extern int BatteryServiceGetPADCAP(void);
extern int BatteryServiceReportPADCAP(void);
#ifdef CONFIG_SENSORS_CAP1106
extern void Check_BalanceMode(int);
#endif
static int IsBalanceCharge = 1;
static int IsPowerBankCharge = 1;
static int LastTimeIsBalMode = 1;
static struct AXC_BatteryService *balance_this=NULL;
//Eason: A68 new balance mode +++
#ifndef ASUS_FACTORY_BUILD
static bool IsSystemdraw = false;
static bool IsBalanceSuspendStartcharge = false;
static bool IsKeepChgFrom15pTo19p = false;//Eason:balance mode keep charge from Cap 15 to 19
#endif
//Eason: A68 new balance mode ---
//ASUS_BSP --- Eason_Chang BalanceMode
//ASUS_BSP +++ Eason_Chang add event log +++
#include <linux/asusdebug.h> 
//ASUS_BSP +++ Eason_Chang add event log ---
#include <linux/wakelock.h>
#include <linux/gpio.h> //Eason:get cable In/Out at first time ask Cap
//#define smb346INOK 27 //Eason: boot up in BatLow situation, take off cable can shutdown//ASUS BSP Eason_Chang : A86 porting
//Eason: charger_limit_enable is used to decide if Do_Factory5060Mode+++
#ifdef ASUS_FACTORY_BUILD
static bool charger_limit_enable = false;//only do 5060 mode in factory branch, let bsp version can normal charging
bool g_5060modeCharging = true;//only set online 0 to show Notcharging icon in factory branch, let bsp version can normal show online status
#endif
//Eason: charger_limit_enable is used to decide if Do_Factory5060Mode---

#ifdef CONFIG_PM_8941_CHARGER
extern int pm8941_is_dc_usb_in(void);
extern int pm8941_is_full(void);
#endif
//ASUS_BSP +++ Eason_Chang charger
extern AXI_Charger * getAsusCharger(void);
static struct AXI_Charger *gpCharger = NULL;
//ASUS_BSP --- Eason_Chang charger
//ASUS BSP Eason_Chang : A86 porting +++
#if 0
//Eason bms notify +++
extern void pm8921_bms_charging_began(void);
extern void pm8921_bms_charging_end(int is_battery_full);
//Eason bms notify +++
#endif
//ASUS BSP Eason_Chang : A86 porting ---
#ifdef CONFIG_ASUSDEC  
extern int AX_MicroP_IsECDockIn(void); 
extern void asus_bat_update_DockAcOnline(void);
#endif
extern void BatteryService_P02update(void);
extern bool reportRtcReady(void);

extern void asus_bat_update_PadAcOnline(void);

#define SUSPEND_DISCHG_CURRENT 10
#define DOCK_SUSPEND_DISCHG_CURRENT 18
#define MAX_DISCHG_CURRENT    700
#define USB_CHG_CURRENT       500
#define A91_ILLEGAL_CHG_CURRENT       500
#define USB3p0_ILLEGAL_CURRENT		900
#ifdef CONFIG_IDTP9023_CHARGER  
#define WIRELESS_CHG_CURRENT        700
#endif
#define PAD_CHG_CURRENT       900
#define DOCK_DISCHG_CURRENT   500
//#ifdef ASUS_A91_PROJECT
#define AC_CHG_CURRENT        900
//#endif
#ifdef ASUS_ME771KL_PROJECT
#define AC_CHG_CURRENT        2000
#endif
#define AC_SUSPEND_CHG_CURRENT 1200

#define BAT_CAP_REPLY_ERR	-1
#define RESUME_UPDATE_TIME   600      //10 min
#define RESUME_UPDATE_TIMEwhenCapLess20  600  //10min
#define RESUME_UPDATE_TIMEwhenBATlow  300  //10min
#define FORCERESUME_UPDATE_TIME   300  //5 min
#define DOCKRESUME_UPDATE_TIME   300  //5 min
#define RTC_READY_DELAY_TIME   20
#define KEEP_CAPACITY_TIME 300

//Hank temperature monitor+++
#define DEFAULT_POLLING_INTERVAL 180
#define DEFAULT_MONITOR_INTERVAL 60
//Hank temperature monitor---

//Eason:cap queue  interval plugs a small value, prevent less than target interval
#define DEFAULT_CAP_QUEUE_INTERVAL_PLUGGED 2

//Hank enterRomMode_test++++
int enterRomMode_test = 0;//0:gauge active mode, 1:gauge Rom mode
//Hank unlock gauge+++
extern void TIgauge_LockStep(void);
extern void TIgauge_UnlockStep1(void);
extern void TIgauge_UnlockStep2(void);
//Hank unlock gauge---
extern void TIgauge_EnterRomMode(void);
extern void exitRomMode(void);
extern bool  check_is_RomMode(void);
extern bool  check_is_ActiveMode(void);
//Hank enterRomMode_test----
//Hank read BatteryID+++
extern int64_t read_battery_id(void);
//Hank read BatteryID---

//Eason set alarm +++
#include <linux/android_alarm.h>
static struct alarm bat_alarm;
static struct alarm batLow_alarm;
static struct alarm cableIn_alarm;
static DEFINE_SPINLOCK(bat_alarm_slock);
static DEFINE_SPINLOCK(batLow_alarm_slock);
static DEFINE_SPINLOCK(cableIn_alarm_slock);
struct wake_lock bat_alarm_wake_lock;
struct wake_lock batLow_alarm_wake_lock;
struct wake_lock cableIn_alarm_wake_lock;
//Eason: wirelessChg workaround+++
struct wake_lock WC_softstart_wake_lock;
struct wake_lock WC_DCIN300_wake_lock;
//Eason: wirelessChg workaround---
static DECLARE_WAIT_QUEUE_HEAD(bat_alarm_wait_queue);
static DECLARE_WAIT_QUEUE_HEAD(batLow_alarm_wait_queue);
static DECLARE_WAIT_QUEUE_HEAD(cableIn_alarm_wait_queue);
static uint32_t alarm_enabled;
static uint32_t batLowAlarm_enabled;
static uint32_t cableInAlarm_enabled;
extern void alarm_start_range(struct alarm *alarm, ktime_t start, ktime_t end);
#define RTCSetInterval 610
//Eason: dynamic set Pad alarm +++
//#define RTCSetIntervalwhenCapLess20  610
#ifndef ASUS_FACTORY_BUILD
#define RTCSetIntervalwhenBalSuspendStopChg 3610
#define RTCSetIntervalwhenAlarmIntervalLess3min 190
   #ifdef CONFIG_EEPROM_NUVOTON    
   static int RTCSetIntervalwhenBalanceMode = RTCSetInterval;
   #endif
static bool InSuspendNeedDoPadAlarmHandler=false;//in suspend set true, in late resume set false,
												//Pad alarm handler need to do only when display off												
#endif
//Eason: dynamic set Pad alarm ---
#define RTCSetIntervalwhenBATlow  310
#define RTCSetIntervalwhenCABLEIn  1810
#define CapChangeRTCInterval 20
//Eason set alarm ---
// when A66 Cap = 0% shutdown device no matter if has cable+++ 
extern bool g_AcUsbOnline_Change0;
extern void AcUsbPowerSupplyChange(void);
#ifdef CONFIG_ASUSDEC  
extern void Dock_AC_PowerSupplyChange(void);
#endif
#ifdef CONFIG_EEPROM_NUVOTON
extern void Pad_AC_PowerSupplyChange(void);
#endif
// when A66 Cap = 0% shutdown device no matter if has cable---
//Eason boot up in BatLow situation, take off cable can shutdown+++
extern bool g_BootUp_IsBatLow;
//Eason boot up in BatLow situation, take off cable can shutdown---
//Eason : In suspend have same cap don't update savedTime +++
bool SameCapDontUpdateSavedTime = false;
extern bool g_RTC_update;
//Eason : In suspend have same cap don't update savedTime ---
//Eason : prevent thermal too hot, limit charging current in phone call+++
extern bool g_audio_limit;
static bool IsPhoneOn = false;
//Eason : prevent thermal too hot, limit charging current in phone call---
//Eason : when thermal too hot, limit charging current +++ 
extern bool g_padMic_On; 
extern int g_thermal_limit;
static bool IsThermalHot = false;
//Eason : when thermal too hot, limit charging current ---

//Eason judge smb346 full +++
extern bool smb346_IsFull(void);
//Eason judge smb346 full ---
//ASUS BSP Eason add A68 charge mode +++
extern void setFloatVoltage(int StopPercent);
//ASUS BSP Eason add A68 charge mode ---
//Eason: AICL work around +++
bool g_alreadyCalFirstCap = false;
//Eason: AICL work around ---
//Eason: choose Capacity type SWGauge/BMS +++
int g_CapType = 0;// 0:SWgauge 1:BMS
#define DEFAULT_CAP_TYPE_VALUE 0
//ASUS BSP Eason_Chang : A86 porting +++
extern int get_BMS_capacity(void);
//ASUS BSP Eason_Chang : A86 porting ---
//Eason: choose Capacity type SWGauge/BMS ---
//ASUS BSP: Eason check correct BMS RUC+++
extern bool gIsBMSerror;
//ASUS BSP: Eason check correct BMS RUC---
//Hank: A86 no use+++	 
//Eason get BMS Capacity for EventLog+++
static int gBMS_Cap=50;
//Eason get BMS Capacity for EventLog---
//Eason show BMS cap to user+++
static int g_SWgauge_lastCapacity = 50;
//Eason show BMS cap to user---
//Eason: remember last BMS Cap to filter+++
//static int last_BMS_Cap=0;
int gDiff_BMS=0;
//Eason: remember last BMS Cap to filter---
//Hank A86 no use---
//Eason:fix Cap drop too slowly in unattended mode+++
int filRealSusT;   //filter Real Suspend Time
//Eason:fix Cap drop too slowly in unattended mode---
//Eason: if BatLow keep 15 min, shutdown devices+++
static time_t batLowTriggerTime;
bool g_batLowLongTimeShut = false;
#define BATLOW_KEEPTIME_SHUTDOWN 900 //15min
//Eason: if BatLow keep 15 min, shutdown devices---
//Eason : if last time is 10mA +++
static bool IsLastTimeMah10mA = false;
static bool IfUpdateSavedTime = false;	
//Eason : if last time is 10mA ---
//Eason: Pad draw rule +++
#include "../charger/axc_Smb346Charger.h"
//Eason: Pad draw rule ---
//Eason: MPdecisionCurrent +++
static int MPdecisionCurrent=0;
extern int get_current_for_ASUSswgauge(void);
//Eason: MPdecisionCurrent ---
//Hank:A86 slowly drop+++
extern int get_Curr_from_TIgauge(void);
extern int gBatteryTemp;
int gCurr_TIgauge=0;
//Hank:A86 slowly drop---
//Eason: LowCapCpuThrottle +++
bool IsInCpuThrottle = false;
//Eason: LowCapCpuThrottle ---
//Eason: check FC flag to disable charge, prevent Battery over charge, let gauge FCC abnormal+++
extern void setSmb346PreventOverFCC(void);
extern bool get_FC_flage_from_TIgauge(void);
//Eason: check FC flag to disable charge, prevent Battery over charge, let gauge FCC abnormal---

//Hank: set higher hot temperature limit when CPU on+++
extern void setSmb346HigherHotTempLimit(void);
//Hank: set higher hot temperature limit when CPU on---
//Hank: set default hot temperature limit when CPU off+++
//#ifdef ASUS_A91_PROJECT
extern void setSmb346DefaultHotTempLimit(void);
//#endif
#ifdef ASUS_ME771KL_PROJECT
extern void setSmb345DefaultHotTempLimit(void);
#endif
//Hank: set default hot temperature limit when CPU off---
extern bool DffsDown;
extern bool SMB346_INOK_Online;

//Eason A91 porting+++
extern int getIfonline(void);
extern void AXC_bq51220_Charger_Init(void);
//Eason A91 porting---
//Eason : Wireless PMA spec Rx turn off Tx +++
#ifdef CONFIG_IDTP9023_CHARGER
#define WC_PD_DET_GPIO 20
static time_t g_WcEocTime; 
extern void IDTP9023_RxTurnOffTx(void);
extern int pm8941_get_prop_batt_temp(void);
#endif
//Eason : Wireless PMA spec Rx turn off Tx ---
//ASUS_BSP Eason_Chang: wireless mode (1)Cap>=80% &(2)Temp>45degC set VDD_MAX(0x1040) low +++
#define GPIO_WC_PD_DET 20
bool g_ASUS_WC_set_low_VDDMAX = false;
bool g_PastTime_set_WC_low_VDD = false;
extern void ASUS_Wireless_set_VDDMAX(void);
//ASUS_BSP Eason_Chang: wireless mode (1)Cap>=80% &(2)Temp>45degC set VDD_MAX(0x1040) low ---
//ASUS_BSP Eason_Chang:add WirelessChg soft start+++
bool g_ASUS_WC_CHG_DONE_set_DCIN300 = false;
bool g_PastTime_set_WC_DCIN300 = false;
//ASUS_BSP Eason_Chang:add WirelessChg soft start---

//Eason: judge status by user Cap +++
#ifdef CONFIG_PM_8941_CHARGER
extern int pm8941_getCapacity(void);
#endif
//Eason: judge status by user Cap ---

//Hank: when update gauge need to disable charge+++
extern bool DisChg;

//Clay: get battery FCC & voltage +++
extern int get_vm_bms_fcc(void);
extern int pm8941_get_prop_battery_voltage_now(void);
//Clay: get battery voltage ---

//Clay: get battery Temp only when report Capacity +++
extern int report_temp;
//Clay: get battery Temp only when report Capacity ---
void UpGaugeSetChg(bool enabled)
{
 	gpCharger->EnableCharging(gpCharger,enabled);
	DisChg = !enabled;
	printk("[BAT][Gau]UpGaugeSetChg:%d\n",enabled);
}
//Hank: when update gauge need to disable charge---

#ifdef CONFIG_PM_8941_CHARGER
void asus_fsm_chargingstop(AXE_Charging_Error_Reason reason)
{
	#ifdef ASUS_FACTORY_BUILD
	if(false == charger_limit_enable)
	#endif		
	{
		printk("[BAT]%s\n",__FUNCTION__);
	
		if(balance_this != NULL){
			if(balance_this->mbInit){
				balance_this->fsm->onChargingStop(balance_this->fsm, reason);
			}
		}
	}	
}

void asus_fsm_chargingstart(void)
{
	#ifdef ASUS_FACTORY_BUILD
	if(false == charger_limit_enable)
	#endif	
	{
		printk("[BAT]%s\n",__FUNCTION__);
		
		if(balance_this != NULL){
			if(balance_this->mbInit){
				balance_this->fsm->onChargingStart(balance_this->fsm);
			}
		}
	}	
}
#endif

//Eason : prevent thermal too hot, limit charging current in phone call+++
extern void setChgDrawCurrent(void);
static void judgePhoneOnCurLimit(void)
{
/*	
	if( (true==IsPhoneOn)&&(balance_this->A66_capacity>20) )
   	{ 
		g_audio_limit = true;
		printk("[BAT][Ser]:judge g_audio_limit true\n");
		   

		setChgDrawCurrent();

	}else{
		g_audio_limit = false;
		printk("[BAT][Ser]:judge g_audio_limit false\n");


		setChgDrawCurrent();

	}
*/	
}

//Hank enterRomMode_test++++
static ssize_t enterRomMode_read_proc(char *page, char **start, off_t off, int count, 
            	int *eof, void *data)
{
	return sprintf(page, "EnterRom:%d, IsRom:%d\n", enterRomMode_test,check_is_RomMode());
}
static ssize_t enterRomMode_write_proc(struct file *filp, const char __user *buff, 
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
 
	enterRomMode_test = val;  

	if(1==val)
	{
		TIgauge_UnlockStep1();
		TIgauge_UnlockStep2();
		TIgauge_EnterRomMode();
	}else if(0==val)
	{
		exitRomMode();
		//TIgauge_LockStep(); X: 4000 in funtion explain_dffs_string already do this (axc_TI_HWgauge_Update_A80.c)
	}
    
   	 printk("[BAT][enterRomMode_test]:%d,Rom:%d,Active:%d\n",val,check_is_RomMode(),check_is_ActiveMode());//check Rom mode first
	
	return len;
}

void static create_enterRomMode_proc_file(void)
{
	struct proc_dir_entry *enterRomMode_proc_file = create_proc_entry("driver/enterRomMode", 0666, NULL);

	if (enterRomMode_proc_file) {
		enterRomMode_proc_file->read_proc = enterRomMode_read_proc;
		enterRomMode_proc_file->write_proc = enterRomMode_write_proc;
	}
	    else {
		printk("[BAT][Bal] enterRomMode create failed!\n");
    }

	return;
}
//Hank enterRomMode_test----

//[ChiaYuan][+++]Add for Broadcast busy state
void HighPower_timer_expired(unsigned long data)
{	
	int AveCur = 0;

	if (HighPower_Count != 0) {
		AveCur = Total_BatCur/HighPower_Count;
	}
	printk(KERN_INFO "[Power]Timer expired. Count:%d, AveCur:%d\r\n", HighPower_Count, AveCur);
	if (AveCur > 100) {
		if (IsHighPower==0) {
			IsHighPower=1;
		}
		else if (IsHighPower==1) {
			IsHighPower=2;
		}
		else if (IsHighPower==2) {
			IsHighPower=1;
		}
		printk(KERN_INFO "[Power]Set IsHighPower:%d \r\n", IsHighPower);
	}
	//Reset all calculation variables
	Total_BatCur=0;
	HighPower_Count=0;
	mod_timer(&HighPower_timer, jiffies + msecs_to_jiffies(HIGH_POWER_TIMEOUT));
}

static ssize_t HighPower_read_proc(char *page, char **start, off_t off, int count, 
            	int *eof, void *data)
{
	return sprintf(page, "%d\n", IsHighPower);
}

static ssize_t HighPower_write_proc(struct file *filp, const char __user *buff, 
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
	
	IsHighPower = val;
     
        printk("[Power]Write HighPower:%d\n",IsHighPower);

	return len;
}

void static create_HighPower_proc_file(void)
{
	struct proc_dir_entry *HighPower_proc_file = create_proc_entry("driver/IsHighPower", 0644, NULL);
	if (HighPower_proc_file) {
		HighPower_proc_file->read_proc = HighPower_read_proc;
		HighPower_proc_file->write_proc = HighPower_write_proc;
	}
	else {
		printk("[Power] HighPower_proc_file create failed!\n");
	}
}
//[ChiaYuan][---]Add for Broadcast busy state

//Hank read BatteryID++++
#ifdef ASUS_FACTORY_BUILD
extern long long g_BatteryID_value;
static ssize_t BatteryID_read_proc(char *page, char **start, off_t off, int count, 
            	int *eof, void *data)
{
	read_battery_id();
	return sprintf(page, "BatteryID: %lld\n", g_BatteryID_value);
}
static ssize_t BatteryID_write_proc(struct file *filp, const char __user *buff, 
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
    
   	 printk("[BAT][SER][BatteryID]mode:%d\n",val);//check Rom mode first
	
	return len;
}

void static create_BatteryID_proc_file(void)
{
	struct proc_dir_entry *BatteryID_proc_file = create_proc_entry("driver/BatteryID", 0644, NULL);

	if (BatteryID_proc_file) {
		BatteryID_proc_file->read_proc = BatteryID_read_proc;
		BatteryID_proc_file->write_proc = BatteryID_write_proc;
	}
	    else {
		printk("[BAT][SER] BatteryID create failed!\n");
    }

	return;
}
#endif
//Hank read BatteryID----

void SetLimitCurrentInPhoneCall(bool phoneOn)
{  
   if(phoneOn)
   {
        IsPhoneOn = true;
        printk("[BAT][Ser]:Phone call on\n");
   }else{
    	IsPhoneOn = false;
    	g_audio_limit = false;
    	printk("[BAT][Ser]:Phone call off\n");
   }


       	judgePhoneOnCurLimit();

}
//Eason : prevent thermal too hot, limit charging current in phone call---
//ASUS BSP Eason_Chang : A86 porting +++//////////////////////////////////
static void pm8921_bms_charging_end(int is_battery_full)
{}
static void pm8921_bms_charging_began(void)
{}
 //Hank: A86 no use+++	 
#if 0 //CONFIG_BMS_ASUS
static int get_BMS_capacity(void)
{ 
	return 100;
}
#endif
 //Hank: A86 no use---
//ASUS BSP Eason_Chang : A86 porting ---///////////////////////////////////

//Eason : Wireless PMA spec Rx turn off Tx +++
/*
*  wireless charger PMA spec
*  When coil temp(mapping bat temp) over 50.0 degreeC, force Rx turn off Tx
*/
#ifdef CONFIG_IDTP9023_CHARGER
static time_t getWcEocIntervalSinceLastUpdate(void)
{
	struct timespec mtNow;
    
	time_t intervalSinceLastUpdate;
    
	mtNow = current_kernel_time();

	if(mtNow.tv_sec >= g_WcEocTime)
	{
		printk("[BAT][Ser]%s:%ld,%ld\n",__FUNCTION__,mtNow.tv_sec,g_WcEocTime);
            
		intervalSinceLastUpdate = mtNow.tv_sec - g_WcEocTime;


		//cause system time didn't work at first time update capacity (8secs) 
		//filter intervalSinceLastUpdate more than one month
		if(intervalSinceLastUpdate > 2592000)
		{
			printk("[BAT][SER]%s():wrongInt %ld \n",__func__,intervalSinceLastUpdate);
			intervalSinceLastUpdate = 180;
		}        
	}
	else
	{  
		printk("[BAT][Ser]%s:OVERFLOW....%ld,%ld\n",__FUNCTION__,mtNow.tv_sec,g_WcEocTime);              
		//todo: to do the correct calculation here....
		intervalSinceLastUpdate = 180;
	}


    return intervalSinceLastUpdate ; 
}
#define WC_EOC_TURNOFF_TX_TIMER 1800
static inline time_t  updateNowTime(struct AXC_BatteryService *_this);
static void judgeWirelessEocTurnOffTx(void)
{
	time_t WcEocInterval;
	WcEocInterval  = getWcEocIntervalSinceLastUpdate();

	if( (100==gBMS_Cap) && (WcEocInterval >= WC_EOC_TURNOFF_TX_TIMER) && (1 == gpio_get_value(WC_PD_DET_GPIO)) )
	{
		printk("[BAT][WC][PMA][SPEC]WIRELESS EOC KEEP 30MIN, RX TURN TX:%ld\n",WcEocInterval);
		IDTP9023_RxTurnOffTx();
	}
	
	g_WcEocTime = updateNowTime(balance_this);
	
}

#define WC_COIL_HOT_TEMP 550
static void judgeWirelessCoilTempTurnOffTx(void)
{
	int batt_temp = pm8941_get_prop_batt_temp();
	if( batt_temp >= WC_COIL_HOT_TEMP )
	{
		printk("[BAT][WC][PMA]ERROR: COIL TEMP OVER 50 degc, RX TURN TX:%d\n",batt_temp);
		IDTP9023_RxTurnOffTx();
	}

}
#endif
//Eason : Wireless PMA spec Rx turn off Tx ---

#ifdef CONFIG_PM_8941_CHARGER
//ASUS_BSP Eason_Chang:add WirelessChg soft start+++
extern void prepare_WC_soft_start(void);
extern void set_DCIN_300mA(void);
extern void WC_soft_start(void);
void judge_Cap100_WC_DCIN300(void)
{

	//Eason_Chang: PF500KL WC_PD_DET reverse. default high, with wireless low+++
	if(g_ASUS_hwID >= PF500KL_PR){
		if( gBMS_Cap>=95 )
		{
			g_ASUS_WC_CHG_DONE_set_DCIN300 = true;

			if( (false==g_PastTime_set_WC_DCIN300)&&(true==g_ASUS_WC_CHG_DONE_set_DCIN300)&&(1 == !gpio_get_value(GPIO_WC_PD_DET)) )
			{
				//need set DCIN300mA delay 6secs cause in function IDTP9023_handler_judgeWireless after check soft start(msleep(2000)*2)
				wake_lock_timeout(&WC_DCIN300_wake_lock, 7 * HZ);
				schedule_delayed_work(&balance_this->WirelessChgDCIN300Worker, 6*HZ);
				g_PastTime_set_WC_DCIN300 = true;
				printk("[BAT][WC][workAround]%s:enable\n",__FUNCTION__);
			}
		}else if (70>=gBMS_Cap){
			g_ASUS_WC_CHG_DONE_set_DCIN300 = false;

			if( (true==g_PastTime_set_WC_DCIN300)&&(false==g_ASUS_WC_CHG_DONE_set_DCIN300)&&(1 == !gpio_get_value(GPIO_WC_PD_DET)) )
			{
				schedule_delayed_work(&balance_this->WirelessChgSoftStartWorker, 0*HZ);
				g_PastTime_set_WC_DCIN300 = false;
				printk("[BAT][WC][workAround]%s:disable\n",__FUNCTION__);
			}
		}	
	}else{
	//Eason_Chang: PF500KL WC_PD_DET reverse. default high, with wireless low---
		if( (true==balance_this->BatteryService_IsFULL)&&(gBMS_Cap>=99) )
		{
			g_ASUS_WC_CHG_DONE_set_DCIN300 = true;

			if( (false==g_PastTime_set_WC_DCIN300)&&(true==g_ASUS_WC_CHG_DONE_set_DCIN300)&&(1 == gpio_get_value(GPIO_WC_PD_DET)) )
			{
				//need set DCIN300mA delay 6secs cause in function IDTP9023_handler_judgeWireless after check soft start(msleep(2000)*2)
				wake_lock_timeout(&WC_DCIN300_wake_lock, 7 * HZ);
				schedule_delayed_work(&balance_this->WirelessChgDCIN300Worker, 6*HZ);
				g_PastTime_set_WC_DCIN300 = true;
				printk("[BAT][WC][workAround]%s:enable\n",__FUNCTION__);
			}
		}else if (70>=gBMS_Cap){
			g_ASUS_WC_CHG_DONE_set_DCIN300 = false;

			if( (true==g_PastTime_set_WC_DCIN300)&&(false==g_ASUS_WC_CHG_DONE_set_DCIN300)&&(1 == gpio_get_value(GPIO_WC_PD_DET)) )
			{
				schedule_delayed_work(&balance_this->WirelessChgSoftStartWorker, 0*HZ);
				g_PastTime_set_WC_DCIN300 = false;
				printk("[BAT][WC][workAround]%s:disable\n",__FUNCTION__);
			}
		}	
	}
}
//ASUS_BSP Eason_Chang:add WirelessChg soft start---

//ASUS_BSP Eason_Chang: wireless mode (1)Cap>=80% &(2)Temp>45degC set VDD_MAX(0x1040) low +++
static void judgeWirelessLow_VDD_MAX(int temp, int bmsCap)
{
	//Eason_Chang: PF500KL WC_PD_DET reverse. default high, with wireless low+++
	if(g_ASUS_hwID >= PF500KL_PR){
		if( (80<=gBMS_Cap)&&(450<temp) )
		{
			g_ASUS_WC_set_low_VDDMAX = true; 
			

			if( (false==g_PastTime_set_WC_low_VDD)&&(true==g_ASUS_WC_set_low_VDDMAX)&&(1 == !gpio_get_value(GPIO_WC_PD_DET)) )
			{
				ASUS_Wireless_set_VDDMAX();
				g_PastTime_set_WC_low_VDD = true;
				printk("[BAT][WC][workAround]%s:enable\n",__FUNCTION__);
			}

		}else if( 71>gBMS_Cap )
		{
			g_ASUS_WC_set_low_VDDMAX = false;
			
			if( (true==g_PastTime_set_WC_low_VDD)&&(false==g_ASUS_WC_set_low_VDDMAX)&&(1 == !gpio_get_value(GPIO_WC_PD_DET)) )
			{
				prepare_WC_soft_start();//ASUS_BSP Eason_Chang:add WirelessChg soft start
				ASUS_Wireless_set_VDDMAX();
				schedule_delayed_work(&balance_this->WirelessChgSoftStartWorker, 0*HZ);//ASUS_BSP Eason_Chang:add WirelessChg soft start
				g_PastTime_set_WC_low_VDD = false;
				printk("[BAT][WC][workAround]%s:disable\n",__FUNCTION__);
			} 
			
		}
		
	}else{
		
		if( (80<=gBMS_Cap)&&(450<temp) )
		{
			g_ASUS_WC_set_low_VDDMAX = true; 
			
			if( (false==g_PastTime_set_WC_low_VDD)&&(true==g_ASUS_WC_set_low_VDDMAX)&&(1 == gpio_get_value(GPIO_WC_PD_DET)) )
			{
				ASUS_Wireless_set_VDDMAX();
				g_PastTime_set_WC_low_VDD = true;
				printk("[BAT][WC][workAround]%s:enable\n",__FUNCTION__);
			}

		}else if( 71>gBMS_Cap )
		{
			g_ASUS_WC_set_low_VDDMAX = false;
			
			if( (true==g_PastTime_set_WC_low_VDD)&&(false==g_ASUS_WC_set_low_VDDMAX)&&(1 == gpio_get_value(GPIO_WC_PD_DET)) )
			{
				prepare_WC_soft_start();//ASUS_BSP Eason_Chang:add WirelessChg soft start
				ASUS_Wireless_set_VDDMAX();
				schedule_delayed_work(&balance_this->WirelessChgSoftStartWorker, 0*HZ);//ASUS_BSP Eason_Chang:add WirelessChg soft start
				g_PastTime_set_WC_low_VDD = false;
				printk("[BAT][WC][workAround]%s:disable\n",__FUNCTION__);
			} 
			
		}
	}
}

void  IDTP9023_handler_judgeWireless(void)
{
	int pmicTemp;
	pmicTemp = pm8941_get_prop_batt_temp();

	printk("[BAT][WC][workAround]%s\n",__FUNCTION__);
	schedule_delayed_work(&balance_this->WirelessChgSoftStartWorker, 0*HZ);
	judge_Cap100_WC_DCIN300();
	judgeWirelessLow_VDD_MAX(pmicTemp,gBMS_Cap);
}
//ASUS_BSP Eason_Chang: wireless mode (1)Cap>=80% &(2)Temp>45degC set VDD_MAX(0x1040) low ---


#endif

//Eason : when thermal too hot, limit charging current +++
static void judgeThermalCurrentLimit(void)
{
    if( true==IsThermalHot)
    {
	        if(balance_this->A66_capacity>=15)
		 {
	            g_thermal_limit = 3;
	        }
		 else if(balance_this->A66_capacity>=8)
		 {
	             g_thermal_limit = 2;
	        }
		 else
		 {
	             g_thermal_limit = 1;
	        }
	        printk("[BAT][SER][Thermal]:judge g_thermal_limit true\n");    
    }
    else
    {
	        g_thermal_limit = 0;
	        printk("[BAT][SER][Thermal]:judge g_thermal_limit false\n");

    }
    if(1==getIfonline())
    	setChgDrawCurrent();
        
    
}

void notifyThermalLimit(int thermalnotify)
{

	if(0==thermalnotify){
			IsThermalHot = false;
			g_thermal_limit = false;
			printk("[BAT][SER][Thermal]:Thermal normal notify\n");
	}else{
			IsThermalHot = true;
			printk("[BAT][SER][Thermal]:Thermal hot notify!!\n");
	}

    judgeThermalCurrentLimit();
    
}
//Eason : when thermal too hot, limit charging current ---

//Eason: LowCapCpuThrottle +++
#define CPU_THROTTLE_START_BATCAP	14
#define CPU_THROTTLE_STOP_BATCAP		15	
#define CPU_THROTTLE_START_CURR		2000

extern void notifyMpdecisionCpuThrottle(bool IsCpuThrottle);
static void judgeCpuThrottleByCap(void)
{	
	if( (balance_this->A66_capacity <= CPU_THROTTLE_START_BATCAP)
		&&(gCurr_TIgauge >= CPU_THROTTLE_START_CURR)&&(false == IsInCpuThrottle) )
	{
			notifyMpdecisionCpuThrottle(true);
			IsInCpuThrottle = true;
			printk("[BAT][SER][Throttle]throttle:%d,curr:%d\n",IsInCpuThrottle,gCurr_TIgauge);
	}
	else if( (balance_this->A66_capacity >= CPU_THROTTLE_STOP_BATCAP)&&(true == IsInCpuThrottle) )
	{
			notifyMpdecisionCpuThrottle(false);
			IsInCpuThrottle = false;
			printk("[BAT][SER][Throttle]throttle:%d,curr:%d\n",IsInCpuThrottle,gCurr_TIgauge);
	}
}
//Eason: LowCapCpuThrottle ---

//Eason:  Pad draw rule compare thermal +++
static bool DecideIfPadDockHaveExtChgAC(void);
PadDrawLimitCurrent_Type JudgePadRuleDrawLimitCurrent(bool isSuspendCharge)
{
#ifdef CONFIG_EEPROM_NUVOTON
	if( 1==AX_MicroP_IsP01Connected() )
	{
		if((balance_this->A66_capacity <= 8) ||(2==IsBalanceMode) || isSuspendCharge)//ForcePowerBankMode draw 900
		{
				return PadDraw900;	
		}else if( (true == DecideIfPadDockHaveExtChgAC())&&(1==IsBalanceMode) )//only do this rule in balanceMode
		{
				if(balance_this->A66_capacity <= 15)
				{
						return PadDraw700;
				}else{
						if( (balance_this->A66_capacity-balance_this->Pad_capacity)>=20 )
									return PadDraw300;
						else if( (balance_this->A66_capacity-balance_this->Pad_capacity)>=10 )
									return PadDraw500;
						else
									return PadDraw700;
				}

		}else{
				return PadDraw700;
		}
	}else{
		return PadDraw700;
	}
#else
	return PadDraw700;
#endif
}
//Eason:  Pad draw rule compare thermal ---

//ASUS_BSP Eason when audio on, draw 500mA from Pad ++++
extern void setChgDrawPadCurrent(bool audioOn);
void SetPadCurrentDependOnAudio(bool audioOn)
{
/*
   if( 1==AX_MicroP_IsP01Connected() )
   {
   	setChgDrawPadCurrent(audioOn);
   }
*/   
}
//ASUS_BSP Eason when audio on, draw 500mA from Pad ---

//ASUS BSP Eason add A68 charge mode +++
#ifdef CONFIG_EEPROM_NUVOTON
static int decidePowerBankChgModeStopPercent(void)
{
	int StopPercent = 90;

	StopPercent = balance_this->A66_capacity + 2*balance_this->Pad_capacity;

	if(StopPercent >= 90)
	{
		StopPercent = 90;
	}

        printk("[BAT][smb346]PowerBank Stop Percent: %d \n",StopPercent);	
	return StopPercent;
}

static int decideBalanceChgModeStopPercent(void)
{
	int StopPercent = 90;

	StopPercent =  (balance_this->A66_capacity + 2*balance_this->Pad_capacity)/2;

	if(StopPercent >= 90)
	{
		StopPercent = 90;
	}else if(StopPercent <= 15){
		StopPercent = min( (balance_this->A66_capacity + 2*balance_this->Pad_capacity), 15 );
      }

        printk("[BAT][smb346]Balance Stop Percent: %d \n",StopPercent);	
	return StopPercent;
}


static void do_PadBalanceMode_inChgMode(void)
{
	int NeedStopPercent = 90;

	if(1 == IsBalanceMode)// Balance Mode
	{
		NeedStopPercent = decideBalanceChgModeStopPercent();
		setFloatVoltage(NeedStopPercent);
	}else{//PowerBank Mode
		NeedStopPercent = decidePowerBankChgModeStopPercent();
		setFloatVoltage(NeedStopPercent);
	}
}

void decideIfDo_PadBalanceModeInChgMode(void)
{
	int PadChgCable = 1;

	if(1==AX_MicroP_IsP01Connected())
	{	
		PadChgCable = AX_MicroP_get_USBDetectStatus(Batt_P01);
	
		if(1 == PadChgCable)
		{
			printk("[BAT][smb346]with extChg dont do PadChgMode\n");
		}else{
			printk("[BAT][smb346]without extChg need do PadChgMode\n");
			do_PadBalanceMode_inChgMode();			
		}
	}
}
#endif //CONFIG_EEPROM_NUVOTON
//ASUS BSP Eason add A68 charge mode ---

//ASUS BSP Eason_Chang +++ batteryservice to fsm
static void AXC_BatteryService_reportPropertyCapacity(struct AXC_BatteryService *_this, int refcapacity);

#ifdef CONFIG_ASUSDEC
int ReportBatteryServiceDockCap(void)
{
    return balance_this->Dock_capacity;
}
#endif

int ReportBatteryServiceP02Cap(void)
{
    return balance_this->Pad_capacity;
}    
static void BatteryService_enable_ChargingFsm(AXC_BatteryService *_this)
{
    if(NULL == _this->fsm){

        _this->fsm = getChargingFSM(E_ASUS_A66_FSM_CHARGING_TYPE,&_this->fsmCallback);

        _this->fsmState = _this->fsm->getState(_this->fsm);
    }
}  
//ASUS BSP Eason_Chang --- batteryservice to fsm
//ASUS BSP Eason_Chang +++ batteryservice to gauge
static void BatteryService_enable_Gauge(AXC_BatteryService *_this)
{
    if(NULL == _this->gauge){

        AXC_GaugeFactory_GetGaugeV2(E_SW_GAUGE_V2_TYPE , &_this->gauge, &_this->gaugeCallback);
    }
    if(NULL == _this->P02gauge){

        AXC_GaugeFactory_GetGaugeV2(E_HW_GAUGE_PAD_TYPE , &_this->P02gauge, &_this->P02gaugeCallback);
    }
    #ifdef CONFIG_ASUSDEC	
    if(NULL == _this->Dockgauge){

        AXC_GaugeFactory_GetGaugeV2(E_HW_GAUGE_DOCK_TYPE, &_this->Dockgauge, &_this->DockgaugeCallback);
    }
    #endif
}
//ASUS BSP Eason_Chang --- batteryservice to gauge

static void BatteryService_enable_Filter(AXC_BatteryService *_this)
{
    if(NULL == _this->gpCapFilterA66)
    {

       if(g_ASUS_hwID == A90_EVB0)
       	AXC_Cap_Filter_Get(E_CAP_FILTER_PHONE_A66, &_this->gpCapFilterA66, 2250);
	else if( (A91_SR1==g_ASUS_hwID)||(A91_SR2==g_ASUS_hwID) )
		AXC_Cap_Filter_Get(E_CAP_FILTER_PHONE_A66, &_this->gpCapFilterA66, 2400);
	else
		AXC_Cap_Filter_Get(E_CAP_FILTER_PHONE_A66, &_this->gpCapFilterA66, 2300);
    }
    if(NULL == _this->gpCapFilterP02)
    {

       AXC_Cap_Filter_Get(E_CAP_FILTER_PAD_P02, &_this->gpCapFilterP02, 4300);
    }
    #ifdef CONFIG_ASUSDEC
    if(NULL == _this->gpCapFilterDock)
    {

       AXC_Cap_Filter_Get(E_CAP_FILTER_DOCK, &_this->gpCapFilterDock, 3300);
    }
    #endif

}
//ASUS_BSP  +++ Eason_Chang charger
static void NotifyForChargerStateChanged(struct AXI_Charger *apCharger, AXE_Charger_Type aeCharger_Mode)
{
	#ifdef CONFIG_BATTERY_ASUS_SERVICE
    	if(NULL == balance_this)
	{
        	return;
    	}
    	balance_this->miParent.onCableInOut(&balance_this->miParent,aeCharger_Mode);
    	balance_this->isMainBatteryChargingDone = false;     
	#endif
}
static void onChargingStart(struct AXI_Charger *apCharger, bool startCharging)
{


}
//ASUS_BSP  --- Eason_Chang charger

//Hank: A86 1025 No microp porting+++



#ifndef ASUS_FACTORY_BUILD
#ifdef CONFIG_EEPROM_NUVOTON  //ASUS_BSP Eason_Chang 1120 porting +++
#if 0	//sina 2014/11/26: micro fw takes care of 5v on/off		
static void set_5VPWR_EN(int level)
{
    int rt;
 	   rt = AX_MicroP_setGPIOOutputPin(OUT_uP_5V_PWR_EN, level);
    if (rt<0){
           printk("[BAT][Bal]microp5VPWR set error\n");
    }else if(rt == 0){
           printk("[BAT][Bal]microp5VPWR set success\n");
    }     
}
#endif
static int get_5VPWR_EN(void)
{
//#ifdef CONFIG_EEPROM_NUVOTON  //ASUS_BSP Eason_Chang 1120 porting +++
    return AX_MicroP_getGPIOOutputPinLevel(OUT_uP_5V_PWR_EN);
//#endif//CONFIG_CHARGER_MODE//ASUS_BSP Eason_Chang 1120 porting ---
}
#endif //CONFIG_EEPROM_NUVOTON//ASUS_BSP Eason_Chang 1120 porting ---
#endif//#ifndef ASUS_FACTORY_BUILD
//Eason: A68 new balance mode ---




//ASUS_BSP +++ Eason_Chang BalanceMode
static void set_microp_vbus(int level)
{
#ifdef CONFIG_EEPROM_NUVOTON  //ASUS_BSP Eason_Chang 1120 porting +++
    int rt;
 	   rt = AX_MicroP_setGPIOOutputPin(OUT_uP_VBUS_EN, level);
    if (rt<0){
           printk("[BAT][Bal]microp set error\n");
    }else if(rt == 0){
           printk("[BAT][Bal]microp set success\n");
    } 
#endif //CONFIG_EEPROM_NUVOTON//ASUS_BSP Eason_Chang 1120 porting ---    

}


static int get_microp_vbus(void)
{
#ifdef CONFIG_EEPROM_NUVOTON  //ASUS_BSP Eason_Chang 1120 porting +++
    return AX_MicroP_getGPIOOutputPinLevel(OUT_uP_VBUS_EN);
#else
    return 0;
#endif//CONFIG_CHARGER_MODE//ASUS_BSP Eason_Chang 1120 porting ---
}

#ifdef ASUS_FACTORY_BUILD
//Eason: take off 5060rule let A80 can always charge+++
extern enum DEVICE_HWID g_A68_hwID;
//Eason: take off 5060rule let A80 can always charge---
static void Do_Factory5060Mode(void)
{
   
   printk("[BAT][Factory]:DoFactory5060Mode+++\n");
   /*
   if(1==AX_MicroP_IsP01Connected())
   {
   	set_microp_vbus(1);
   }
   gpCharger->EnableCharging(gpCharger,true);
   balance_this->fsm->onChargingStart(balance_this->fsm);
   */
   
   if(balance_this->A66_capacity >= 60){

	   g_5060modeCharging = false;
         gpCharger->EnableCharging(gpCharger,false);
	  DisChg = true;
         balance_this->fsm->onChargingStop(balance_this->fsm,POWERBANK_STOP);

         printk("[BAT][Factory]mode:%d,StopChg,Vbus:%d\n"
                                        ,IsBalanceMode,get_microp_vbus());
   }else if(balance_this->A66_capacity <= 50){   

	   g_5060modeCharging = true;
         gpCharger->EnableCharging(gpCharger,true);
	  DisChg = false;
         balance_this->fsm->onChargingStart(balance_this->fsm);

         printk("[BAT][Factory]mode:%d,StartChg,Vbus:%d\n"
                                        ,IsBalanceMode,get_microp_vbus());
   }else{
         printk("[BAT][Factory]mode:%d,sameChg,Vbus:%d\n"
                                        ,IsBalanceMode,get_microp_vbus());
   }  
   printk("[BAT][Factory]:DoFactory5060Mode---\n");
   
}
#endif//#ifdef ASUS_FACTORY_BUILD

void Init_Microp_Vbus__Chg(void)
{
#ifndef ASUS_FACTORY_BUILD
         set_microp_vbus(1);
	  gpCharger->EnableCharging(gpCharger,true);
	  DisChg = false;
         IsBalanceCharge = 1;
         IsPowerBankCharge = 1;
         balance_this->fsm->onChargingStart(balance_this->fsm);
         printk("[BAT][Bal]InitVbus:%d,InitChg:%d\n",get_microp_vbus(),gpCharger->IsCharging(gpCharger));
#else 
	if( true == charger_limit_enable )
		 Do_Factory5060Mode();
#endif//#ifndef ASUS_FACTORY_BUILD
}    

void  openMicropVbusBeforeShutDown(void){  
         set_microp_vbus(1);
}    
//ASUS_BSP --- Eason_Chang BalanceMode
//ASUS_BSP +++ Eason_Chang BalanceMode
#ifdef CONFIG_EEPROM_NUVOTON 
#ifndef ASUS_FACTORY_BUILD

static void Do_PowerBankMode(void)
{
   
   printk("[BAT][Bal]:DoPowerBank+++\n");
   //set_microp_vbus(1);
   
   if(balance_this->A66_capacity >= 90){
            
         //set_microp_vbus(0);
         if(A90_EVB0==g_ASUS_hwID)//Hank: if vbus disconnect do not set charge
	   {
			if(get_microp_vbus())
				gpCharger->EnableCharging(gpCharger,false);
         }else
		 	gpCharger->EnableCharging(gpCharger,false);
		 
         DisChg = true;
         balance_this->fsm->onChargingStop(balance_this->fsm,POWERBANK_STOP);

         IsPowerBankCharge = 0;
         printk("[BAT][Bal]mode:%d,StopChg,Vbus:%d\n"
                                        ,IsBalanceMode,get_microp_vbus());
   }else if(balance_this->A66_capacity <= 70){   
         
         //set_microp_vbus(1);
         if(A90_EVB0==g_ASUS_hwID)//Hank: if vbus disconnect do not set charge
	  {
			if(get_microp_vbus())  
				gpCharger->EnableCharging(gpCharger,true);
         }else
			gpCharger->EnableCharging(gpCharger,true);
		 
	  DisChg = false;	 
         balance_this->fsm->onChargingStart(balance_this->fsm);

         IsPowerBankCharge = 1;
         printk("[BAT][Bal]mode:%d,StartChg,Vbus:%d\n"
                                        ,IsBalanceMode,get_microp_vbus());
   }else{
         printk("[BAT][Bal]mode:%d,sameChg,Vbus:%d\n"
                                        ,IsBalanceMode,get_microp_vbus());
   }  
   printk("[BAT][Bal]:DoPowerBank---\n");
   
}
#endif//#ifndef ASUS_FACTORY_BUILD


//Eason: A68 new balance mode +++	
static bool DecideIfPadDockHaveExtChgAC(void);
//Eason: A68 new balance mode ---

//Eason: dynamic set Pad alarm +++
#ifndef ASUS_FACTORY_BUILD
static void judgeIfneedDoBalanceModeWhenSuspend(void)
{

	if( true==IsKeepChgFrom15pTo19p )
	{
		 IsBalanceSuspendStartcharge = true;	
	}else if( (balance_this->A66_capacity>=85)||(balance_this->A66_capacity*10-balance_this->Pad_capacity*12 >=0) )
	{
		 IsBalanceSuspendStartcharge = false;

	}else if((balance_this->A66_capacity<=70)&&(balance_this->A66_capacity*10-balance_this->Pad_capacity*9 <=0) ){

 		 IsBalanceSuspendStartcharge = true;		 
	}
}
#endif
void SetRTCAlarm(void);
//Eason: dynamic set Pad alarm ---

static void BatteryServiceDoBalance(struct AXC_BatteryService *_this)
{
#ifndef ASUS_FACTORY_BUILD  //Hank: A86 1025 No microp porting+++
   int StartRatio;
   int StopRatio;

   printk("[BAT][Bal]:DoBalance +++\n");
   //gpCharger->EnableCharging(gpCharger,true);
   StartRatio = GetBalanceModeStartRatio();
   StopRatio = GetBalanceModeStopRatio();

   printk("[BAT][Bal]%d,%d,%d,%d,%d\n",
                      IsBalanceMode,StartRatio,StopRatio,
                      _this->A66_capacity,_this->Pad_capacity);

   if(1 == IsBalanceMode){

         LastTimeIsBalMode = 1;

	//Eason: A68 new balance mode +++	


		//Eason:balance mode keep charge from Cap 15 to 19+++
		if( (_this->A66_capacity>=20)||(false==IsKeepChgFrom15pTo19p) )//16~19 first dobalance will default set_microp_vbus(0) by false==IsKeepChgFrom15pTo19p
		{
			//when forceresume default turn off vbus +++
			if ((false==IsSystemdraw)&&(false == DecideIfPadDockHaveExtChgAC()))//can't take off this, cause if plug extChg and interval calculate Cap can't turn off vbus  
			{
					set_microp_vbus(0);//do this cause in suspend will turn on vbus to charge
					printk("[BAT][Bal]turn off vbus default\n");
			}			
			//when forceresume default turn off vbus ---	
			//judge if draw current to system but does not charge battery +++
			if((_this->A66_capacity>=90)||(_this->A66_capacity-_this->Pad_capacity*StopRatio>=0))
			{
					set_microp_vbus(0);
					//gpCharger->EnableCharging(gpCharger,false); //Hank: remove disable charging when cable off
					DisChg = true;
					_this->fsm->onChargingStop(_this->fsm,BALANCE_STOP);   
	             
					IsBalanceCharge = 0;
					IsSystemdraw = false;
					printk("[BAT][Bal]mode:%d,N_Vbus N_Chg,Vbus:%d,SysD:%d\n"
									,IsBalanceMode,get_microp_vbus(),IsSystemdraw);
					//ASUSEvtlog("[BAT][Bal]draw system[stop]\n");
	          
			}else if((_this->A66_capacity*10 - _this->Pad_capacity*StartRatio <= 0)
					&&(_this->A66_capacity <= 70 ))
			{
					set_microp_vbus(1);
					gpCharger->EnableCharging(gpCharger,false);
					DisChg = true;
					_this->fsm->onChargingStop(_this->fsm,BALANCE_STOP);
             
					IsBalanceCharge = 0;
					IsSystemdraw = true;
					printk("[BAT][Bal]mode:%d,Y_Vbus N_Chg,Vbus:%d,SysD:%d\n"
									,IsBalanceMode,get_microp_vbus(),IsSystemdraw);
					//ASUSEvtlog("[BAT][Bal]draw system[Start]\n");
			}
			//judge if draw current to system, but does not charge battery ---
		}
		//Eason:balance mode keep charge from Cap 15 to 19---
		
		//judge if charge to battery +++
		if(_this->A66_capacity>=20)
		{
				if(A90_EVB0==g_ASUS_hwID) //Hank: if vbus disconnect do not set charge	
				{
					if(get_microp_vbus())
							gpCharger->EnableCharging(gpCharger,false);
				}else
					gpCharger->EnableCharging(gpCharger,false);
				
				DisChg = true;
				_this->fsm->onChargingStop(_this->fsm,BALANCE_STOP);

				IsBalanceCharge = 0;
				IsKeepChgFrom15pTo19p = false;//Eason:balance mode keep charge from Cap 15 to 19
				
				printk("[BAT][Bal]mode:%d,F_Vbus N_Chg,Vbus:%d\n"
								,IsBalanceMode,get_microp_vbus());
				//ASUSEvtlog("[BAT][Bal]active charge[stop]\n");
				
		}else if(_this->A66_capacity<=15)
		{
				set_microp_vbus(1);
				gpCharger->EnableCharging(gpCharger,true);
				DisChg = false;
				_this->fsm->onChargingStart(_this->fsm);

				IsBalanceCharge = 1;
				IsKeepChgFrom15pTo19p = true;//Eason:balance mode keep charge from Cap 15 to 19
				
				printk("[BAT][Bal]mode:%d,Y_Vbus Y_Chg,Vbus:%d\n"
								,IsBalanceMode,get_microp_vbus());
				//ASUSEvtlog("[BAT][Bal]active charge[Start]\n");
		}
		//judge if charge to battery ---
		//Eason: dynamic set Pad alarm +++
		judgeIfneedDoBalanceModeWhenSuspend();
		//Eason: dynamic set Pad alarm ---
	//Eason: A68 new balance mode ---

         
   //}else if(0==IsBalanceMode && 1==LastTimeIsBalMode && 0==IsBalanceCharge){
   }else if(0==IsBalanceMode){
         
         LastTimeIsBalMode = 0;

         Do_PowerBankMode();
         
         
   }
   //Eason: do ForcePowerBankMode+++
   else if(2==IsBalanceMode){
   	
         LastTimeIsBalMode = 0;

         Do_PowerBankMode();
   }	
   //Eason: do ForcePowerBankMode---
	
   pr_debug("[BAT][Bal]LastBal:%d,IsBalChg:%d,IsBankChg:%d\n"
                            ,LastTimeIsBalMode,IsBalanceCharge,IsPowerBankCharge);
   
   printk("[BAT][Bal]:DoBalance ---\n");
#endif//#ifndef ASUS_FACTORY_BUILD //Hank: A86 1025 No microp porting+++
//#endif//CONFIG_EEPROM_NUVOTON
}
#endif //CONFIG_EEPROM_NUVOTON //Hank: A86 1025 No microp porting+++


static bool DecideIfPadDockHaveExtChgAC(void)
{
    bool IsPadDockExtChgAC = false;
    int PadChgCable = 0;
    bool DockChgCable = false;
    int IsDockIn = 0;
    
    #ifdef CONFIG_EEPROM_NUVOTON  //ASUS_BSP Eason_Chang 1120 porting +++
    PadChgCable = AX_MicroP_get_USBDetectStatus(Batt_P01);
    #endif //CONFIG_EEPROM_NUVOTON//ASUS_BSP Eason_Chang 1120 porting ---
    
    #ifdef CONFIG_ASUSDEC    
    IsDockIn = AX_MicroP_IsECDockIn(); 
    #endif    
    if(1==IsDockIn)
    {       
    	       #ifdef CONFIG_ASUSDEC  
              DockChgCable = balance_this->IsDockExtChgIn;
	       #endif
        	if(true==DockChgCable){
        		IsPadDockExtChgAC = true; 
        	}   
    }
    else
    {
            if(1==PadChgCable)
	     {
        		IsPadDockExtChgAC = true; 
            }   		
    }
    pr_debug("[BAT][Ser]:DockI:%d,PadAC:%d,DockAC:%d,ExtChg:%d\n",IsDockIn,PadChgCable,DockChgCable,IsPadDockExtChgAC);
 
    return IsPadDockExtChgAC;
}  

//Eason: do ForcePowerBankMode+++
void DoForcePowerBankMode(void)
{
#ifdef CONFIG_EEPROM_NUVOTON
	unsigned short off=0xAA;

	uP_nuvoton_write_reg(MICROP_SOFTWARE_OFF,  &off);
	printk("[BAT][Bal]:ForcePowerBankMode\n");
#endif


	
}
//Eason: do ForcePowerBankMode---

static ssize_t balanceChg_read_proc(char *page, char **start, off_t off, int count, 
            	int *eof, void *data)
{
	return sprintf(page, "%d\n", IsBalanceMode);
}
static ssize_t balanceChg_write_proc(struct file *filp, const char __user *buff, 
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


	IsBalanceMode = val;
	
	#ifdef CONFIG_SENSORS_CAP1106
	Check_BalanceMode(IsBalanceMode);
	#endif

//when takeoff extChg default turn off vbus +++
#ifndef ASUS_FACTORY_BUILD
	  IsSystemdraw=false;
	  IsBalanceSuspendStartcharge = false;
	  IsKeepChgFrom15pTo19p = false;//Eason:balance mode keep charge from Cap 15 to 19s
#endif
//when takeoff extChg default turn off vbus ---
  
#ifdef CONFIG_EEPROM_NUVOTON  //ASUS_BSP Eason_Chang 1120 porting +++    
    if(1==AX_MicroP_IsP01Connected()){

		if( false == DecideIfPadDockHaveExtChgAC()){ 
				Init_Microp_Vbus__Chg();
				BatteryServiceDoBalance(balance_this);
		}else{
				Init_Microp_Vbus__Chg();
		}
		//Eason: do ForcePowerBankMode+++
		if(2==IsBalanceMode){
				DoForcePowerBankMode();
		}
		//Eason: do ForcePowerBankMode---
    }
#endif //CONFIG_EEPROM_NUVOTON//ASUS_BSP Eason_Chang 1120 porting ---    
    
    printk("[BAT][Bal]mode:%d\n",val);
	
	return len;
}

void static create_balanceChg_proc_file(void)
{
	struct proc_dir_entry *balanceChg_proc_file = create_proc_entry("driver/balanceChg", 0644, NULL);

	if (balanceChg_proc_file) {
		balanceChg_proc_file->read_proc = balanceChg_read_proc;
		balanceChg_proc_file->write_proc = balanceChg_write_proc;
	}
    else {
		printk("[BAT][Bal]proc file create failed!\n");
    }

	return;
}

//Eason: MPdecisionCurrent +++
static ssize_t MPdecisionCurrent_read_proc(char *page, char **start, off_t off, int count, 
            	int *eof, void *data)
{
	if(g_ASUS_hwID == A90_EVB0)
			MPdecisionCurrent = get_Curr_from_TIgauge();
	else
			MPdecisionCurrent = get_current_for_ASUSswgauge();
	return sprintf(page, "%d\n", MPdecisionCurrent);
}
static ssize_t MPdecisionCurrent_write_proc(struct file *filp, const char __user *buff, 
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

	MPdecisionCurrent = val;
     
        printk("[BAT][Bal]mode:%d\n",val);

	return len;
}

void static create_MPdecisionCurrent_proc_file(void)
{
	struct proc_dir_entry *MPdecisionCurrent_proc_file = create_proc_entry("driver/MPdecisionCurrent", 0644, NULL);

	if (MPdecisionCurrent_proc_file) {
		MPdecisionCurrent_proc_file->read_proc = MPdecisionCurrent_read_proc;
		MPdecisionCurrent_proc_file->write_proc = MPdecisionCurrent_write_proc;
	}
	else {
		printk("[BAT]MPdecisionCurrent proc file create failed!\n");
	}

	return;
}
//Eason: MPdecisionCurrent ---

//Eason: charger_limit_enable is used to decide if Do_Factory5060Mode+++
#ifdef ASUS_FACTORY_BUILD
static struct proc_dir_entry *charger_limit_enable_proc_file = NULL;
static ssize_t charger_limit_enable_read_proc(char *page, char **start, off_t off, int count, 
	int *eof, void *data)
{
	return sprintf(page, "%d\n", charger_limit_enable);
}

static ssize_t charger_limit_enable_write_proc(struct file *filp, const char __user *buff, 
	unsigned long len, void *data)
{
	char messages[256];

	if (len > 256) {
		len = 256;
	}

	if (copy_from_user(messages, buff, len)) {
		return -EFAULT;
	}
	if(messages[0]== '0')
		{
			//Eason:when disable 5060_limit, enable chargering 
			if(1==getIfonline() )
			{
				gpCharger->EnableCharging(gpCharger,true);
				DisChg = false;
				balance_this->fsm->onChargingStart(balance_this->fsm);
			}
			//Eason:when disable 5060_limit, enable chargering 
		 
			printk("[BAT] set charger_limit_enable = false !\n");
			g_5060modeCharging = true;
			charger_limit_enable = false;
		}
	else
		if(messages[0] == '1')
		{
			printk("[BAT] set charger_limit_enable = true !\n");
			charger_limit_enable = true;
		}
		else
			printk("[BAT] charger_limit_enable write error %c \n",messages[0]);
	
	//Eason: Calculate Cap immediately after do 5060 mode+++
	 cancel_delayed_work_sync(&balance_this->BatteryServiceUpdateWorker);
	//Hank: cancel BatteryServiceUpdateWorker need calculate capacity+++
        balance_this->NeedCalCap = true;
        //Hank: cancel BatteryServiceUpdateWorker need calculate capacity---
        queue_delayed_work(balance_this->BatteryServiceCapUpdateQueue, \
                               &balance_this->BatteryServiceUpdateWorker,\
                               0 * HZ);
	  //Eason: Calculate Cap immediately after do 5060 mode---
	
	return len;		
}


static void charger_limit_enable_create_proc_file(void)
{	
	charger_limit_enable_proc_file = create_proc_entry(
		"driver/charger_limit_enable",
		0777,
		NULL);

	if (NULL == charger_limit_enable_proc_file) {
		printk(KERN_ERR "[BAT] charger_limit_enable_proc_file created failed!\n");
		return;
	}

	charger_limit_enable_proc_file->read_proc = charger_limit_enable_read_proc;
	charger_limit_enable_proc_file->write_proc = charger_limit_enable_write_proc;
	return;
}
#endif
//Eason: charger_limit_enable is used to decide if Do_Factory5060Mode---

static inline time_t  updateNowTime(struct AXC_BatteryService *_this)
{
    struct timespec mtNow;
    
    mtNow = current_kernel_time();    

    return mtNow.tv_sec;
}

//ASUS_BSP  +++ Eason_Chang "add BAT info time"
#if 0 
static void ReportTime(void)
{
	struct timespec ts;
	struct rtc_time tm;
	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);

	pr_debug ("[BAT][Ser] %d-%02d-%02d %02d:%02d:%02d\n"
		,tm.tm_year + 1900
		,tm.tm_mon + 1
		,tm.tm_mday
		,tm.tm_hour
		,tm.tm_min
		,tm.tm_sec);
}
#endif
//ASUS_BSP  --- Eason_Chang "add BAT info time"

void Init_BalanceMode_Flag(void)
{

         Init_Microp_Vbus__Chg();
		 
//Eason: A68 new balance mode +++			 
#ifndef ASUS_FACTORY_BUILD
#ifdef CONFIG_EEPROM_NUVOTON
	 IsBalanceSuspendStartcharge = false;//when plugIn Pad default false, or in doInBalanceModeWhenSuspend will keep last plugIn time status 
	 IsSystemdraw = false;//when plugIn Pad default false,
	 IsKeepChgFrom15pTo19p = false;//Eason:balance mode keep charge from Cap 15 to 19
#endif
#endif
//Eason: A68 new balance mode ---

         LastTimeIsBalMode = 1;
         IsBalanceCharge = 1;
         IsPowerBankCharge =1;
         balance_this->P02_savedTime = updateNowTime(balance_this);
}
//ASUS_BSP --- Eason_Chang BalanceMode
#ifdef CONFIG_ASUSDEC
bool reportDockInitReady(void)
{
    printk("[BAT][Bal]DockReady:%d\n",balance_this->IsDockInitReady);
    return balance_this->IsDockInitReady;
}

void setDockInitNotReady(void)
{
    balance_this->IsDockInitReady = false;
    printk("[BAT][Bal]setDockNotReady:%d\n",balance_this->IsDockInitReady);
}

bool reportDockExtPowerPlug(void)
{
    return balance_this->IsDockExtCableIn;
}

bool DockCapNeedUpdate(void)
{
    time_t nowDockResumeTime;
    time_t nowDockResumeInterval;
    bool needDoDockResume=false;

    nowDockResumeTime = updateNowTime(balance_this);
    nowDockResumeInterval = nowDockResumeTime - balance_this->Dock_savedTime;
    printk("[BAT][Ser]:DockResume()===:%ld,%ld,%ld\n"
            ,nowDockResumeTime,balance_this->Dock_savedTime,nowDockResumeInterval);

    if( true==balance_this->Dock_IsFirstAskCap ){
            needDoDockResume = true;
    }else if( nowDockResumeInterval >= DOCKRESUME_UPDATE_TIME
                            &&false==balance_this->IsCalculateCapOngoing){
                                            
            needDoDockResume = true;
    }

    return needDoDockResume;
}
#endif //#ifdef CONFIG_ASUSDEC

//ASUS_BSP +++ Eason_Chang BalanceMode
#ifdef CONFIG_EEPROM_NUVOTON
static int batSer_microp_event_handler(
	struct notifier_block *this,
	unsigned long event,
	void *ptr)
{
    unsigned long flags;
	pr_debug( "[BAT][Bal] %s() +++, evt:%lu \n", __FUNCTION__, event);

	switch (event) {
	case P01_ADD:
		printk( "[BAT][Bal]P01_ADD \r\n");
        asus_chg_set_chg_mode(ASUS_CHG_SRC_PAD_BAT);
        balance_this->P02_IsFirstAskCap = true;
        Init_BalanceMode_Flag();
        
        cancel_delayed_work_sync(&balance_this->BatteryServiceUpdateWorker);
	//Hank: cancel BatteryServiceUpdateWorker need calculate capacity+++
        balance_this->NeedCalCap = true;
        //Hank: cancel BatteryServiceUpdateWorker need calculate capacity---
        queue_delayed_work(balance_this->BatteryServiceCapUpdateQueue, \
                               &balance_this->BatteryServiceUpdateWorker,\
                               0 * HZ);

	//Eason: dynamic set Pad alarm +++
#ifdef ASUS_FACTORY_BUILD	
        schedule_delayed_work(&balance_this->SetRTCWorker, 1*HZ);
#endif
	//Eason: dynamic set Pad alarm ---
	 
        //Init_BalanceMode_Flag();
        //BatteryServiceDoBalance(balance_this);
		break;	
	case P01_REMOVE: // means P01 removed
	 #ifdef CONFIG_ASUSDEC
	 balance_this->IsDockInitReady = false;
        balance_this->IsDockExtCableIn = false;
	#endif
        g_padMic_On = false;//Eason:thermal limit charging current,cause setChgDrawPadCurrent only do inPad
		printk( "[BAT][Bal]P01_REMOVE \r\n");
        
        spin_lock_irqsave(&bat_alarm_slock, flags);
        alarm_try_to_cancel(&bat_alarm);
        spin_unlock_irqrestore(&bat_alarm_slock, flags);

        asus_chg_set_chg_mode(ASUS_CHG_SRC_PAD_NONE);
        asus_bat_update_PadAcOnline();//Eason: In charger mode need update current status
		break;
    case P01_AC_USB_IN:
        //msleep(800);//Eason ,need time delay to get PAD AC/USB
        asus_bat_update_PadAcOnline();    
        printk( "[BAT][Bal]P01_AC_USB_IN\r\n");

        if(true==DecideIfPadDockHaveExtChgAC()){
                Init_Microp_Vbus__Chg();
        }
		break;
    case P01_AC_USB_OUT:
        asus_bat_update_PadAcOnline();    
        printk( "[BAT][Bal]P01_AC_USB_OUT \r\n");
        schedule_delayed_work(&balance_this->CableOffWorker,1*HZ);//keep 100% 5 min
  //when takeoff extChg default turn off vbus +++
#ifndef ASUS_FACTORY_BUILD
	  IsSystemdraw=false;
	 IsBalanceSuspendStartcharge = false;
	 IsKeepChgFrom15pTo19p = false;//Eason:balance mode keep charge from Cap 15 to 19
#endif
  //when takeoff extChg default turn off vbus ---
        BatteryServiceDoBalance(balance_this);
		break;
#ifdef CONFIG_ASUSDEC 	
    case DOCK_EXT_POWER_PLUG_IN:
        balance_this->IsDockExtCableIn = true;
        asus_bat_update_DockAcOnline();
        printk( "[BAT][Bal]DOCK_EXT_PLUG_IN:%d\r\n",balance_this->IsDockExtCableIn);
        break;
    case DOCK_EXT_POWER_PLUG_OUT:
        balance_this->IsDockExtCableIn = false;
        asus_bat_update_DockAcOnline();
        printk( "[BAT][Bal]DOCK_EXT_PLUG_OUT:%d\r\n",balance_this->IsDockExtCableIn);
        break;
    case DOCK_EXT_POWER_PLUG_IN_READY: // means dock charging
        balance_this->IsDockExtChgIn = true;
		printk( "[BAT][Bal]DOCK_EXT_POWER_PLUG_IN:%d\r\n",balance_this->IsDockExtChgIn);
        queue_delayed_work(balance_this->BatteryServiceCapUpdateQueue,
                               &balance_this->BatEcAcWorker,
                               1 * HZ);
		break;
    case DOCK_EXT_POWER_PLUG_OUT_READY:	// means dock discharging
        balance_this->IsDockExtChgIn = false;
        printk( "[BAT][Bal]DOCK_EXT_POWER_PLUG_OUT_READY:%d \r\n",balance_this->IsDockExtChgIn);
        schedule_delayed_work(&balance_this->CableOffWorker,1*HZ);//keep 100% 5 min
        BatteryServiceDoBalance(balance_this);
		break;	

	case DOCK_PLUG_IN:  
		asus_bat_update_PadAcOnline();
        balance_this->Dock_IsFirstAskCap = true;
        balance_this->Dock_savedTime = updateNowTime(balance_this);
        break;

    case DOCK_INIT_READY:
        printk( "[BAT][Bal]DOCK_INIT_READY+++\n");
        balance_this->IsDockInitReady = true;
        
        if(1==AX_MicroP_get_USBDetectStatus(Batt_Dock))
        {
                balance_this->IsDockExtChgIn = true;
                Init_Microp_Vbus__Chg();
        }

        if(AX_MicroP_IsDockReady() && DockCapNeedUpdate())
        {
                
                cancel_delayed_work_sync(&balance_this->BatteryServiceUpdateWorker);
                queue_delayed_work(balance_this->BatteryServiceCapUpdateQueue, \
                               &balance_this->BatteryServiceUpdateWorker,\
                               0 * HZ);

        }        
        printk( "[BAT][Bal]DOCK_INIT_READY---:%d,%d,%d,%d \r\n"
            ,balance_this->IsDockInitReady,AX_MicroP_get_USBDetectStatus(Batt_Dock)
            ,balance_this->IsDockExtChgIn,balance_this->Dock_IsFirstAskCap);
        break;

    case DOCK_PLUG_OUT:
        balance_this->IsDockInitReady = false;
        balance_this->IsDockExtChgIn = false;
        BatteryServiceDoBalance(balance_this);
        printk( "[BAT][Bal]DOCK_PLUG_OUT:%d,%d\r\n"
            ,balance_this->IsDockInitReady,balance_this->IsDockExtChgIn);
        break;
#endif //#ifdef CONFIG_ASUSDEC 
	//Eason after Pad update firmware, update status +++
	case PAD_UPDATE_FINISH:
		printk( "[BAT][Bal]PAD_UPDATE_FINISH+++\n");
		schedule_delayed_work(&balance_this->UpdatePadWorker, 0*HZ);

		Init_Microp_Vbus__Chg();

		if(delayed_work_pending(&balance_this->BatteryServiceUpdateWorker))
		{
			cancel_delayed_work_sync(&balance_this->BatteryServiceUpdateWorker); 
		}
		//Hank: cancel BatteryServiceUpdateWorker need calculate capacity+++
		balance_this->NeedCalCap = true;
		//Hank: cancel BatteryServiceUpdateWorker need calculate capacity---  
		pr_debug("[BAT][SER][Pad]%s(PAD_UPDATE_FINISH) queue BatteryServiceUpdateWorker with calculate capacity\n",__func__);     
		queue_delayed_work(balance_this->BatteryServiceCapUpdateQueue, \
                               &balance_this->BatteryServiceUpdateWorker,\
                               0 * HZ);
	break;
	//Eason after Pad update firmware, update status ---

	//Hank : change pad icon immediately when pad firmware notify +++
	case P05_BAT_STATUS_CHANGE:
		printk( "[BAT][Bal]P05_BAT_STATUS_CHANGE+++\n");
			schedule_delayed_work(&balance_this->UpdatePadWorker, 0*HZ);
	break;
	//Hank : change pad icon immediately when pad firmware notify ---	

	default:
		pr_debug("[BAT][Bal] %s(), not listened evt: %lu \n", __FUNCTION__, event);
		return NOTIFY_DONE;
	}


	pr_debug("[BAT][Bal] %s() ---\n", __FUNCTION__);
	return NOTIFY_DONE;
}
#endif /* CONFIG_EEPROM_NUVOTON */
//ASUS_BSP --- Eason_Chang BalanceMode
//ASUS_BSP +++ Eason_Chang BalanceMode
#ifdef CONFIG_EEPROM_NUVOTON
static struct notifier_block batSer_microp_notifier = {
        .notifier_call = batSer_microp_event_handler,
};
#endif /* CONFIG_EEPROM_NUVOTON */
//ASUS_BSP --- Eason_Chang BalanceMode



static void CheckBatEcAc(struct work_struct *dat)
{
        if(true == DecideIfPadDockHaveExtChgAC()){
                Init_Microp_Vbus__Chg();
        }
}

//Eason: dynamic set Pad alarm +++
#ifndef ASUS_FACTORY_BUILD
  #ifdef CONFIG_EEPROM_NUVOTON    
static int CalBalanceInterval(void)
{
	int BalanceInterval;
	int StopInterval_Ratio_1p3;
	int StopInterval_90p;
	int StopInterval_20p;

	//f2=f1+(900*100/2100)*x1/3600 
	//P2=p1-25*(x1/3600)    , (900*5V)/(19*0.95)~=25
	//f2/p2<=1.3
	StopInterval_Ratio_1p3 = (balance_this->Pad_capacity*13104 - balance_this->A66_capacity*10080)/211;

	//f2=f1+(900*100/2100)*x1/3600 
	//f2<=90
	StopInterval_90p =  (7560 - (balance_this->A66_capacity*84));

	BalanceInterval=min(StopInterval_Ratio_1p3,StopInterval_90p);

	
	if(balance_this->A66_capacity<20)
	{
			//f2=f1+(900*100/2100)*x1/3600
			StopInterval_20p = (1680 - (balance_this->A66_capacity*84));
			
			BalanceInterval = max(StopInterval_20p,RTCSetIntervalwhenAlarmIntervalLess3min);
			printk("[BAT][Bal]:Phone less 20p:%d\n",BalanceInterval);
	}else if(BalanceInterval<=180)
	{
			BalanceInterval = RTCSetIntervalwhenAlarmIntervalLess3min;
			printk("[BAT][Bal]:interval less 180sec:%d\n",BalanceInterval);
	}else if(BalanceInterval>=3600)
	{
			BalanceInterval = RTCSetIntervalwhenBalSuspendStopChg;
			printk("[BAT][Bal]:interval >1hr :%d\n",BalanceInterval);
	}else{
			printk("[BAT][Bal]:interval :%d\n",BalanceInterval);
	}

	return BalanceInterval;
	
}

static int CalPowerBankInterval(void)
{
	int PowerBankInterval;
	int StopPowerBankInterval_90p;

	StopPowerBankInterval_90p =  (7560 - (balance_this->A66_capacity*84));

	PowerBankInterval = StopPowerBankInterval_90p;

	if(0==IsPowerBankCharge)//PowerBank Mode Stop condition
	{
			PowerBankInterval = RTCSetIntervalwhenBalSuspendStopChg;
			printk("[BAT][Bal][PwrB]:stop chg interval 1hr:%d\n",PowerBankInterval);
	}
	else if(StopPowerBankInterval_90p <= 180)
	{	
			PowerBankInterval = RTCSetIntervalwhenAlarmIntervalLess3min;	
			printk("[BAT][Bal][PwrB]:interval less 180sec:%d\n",PowerBankInterval);
	}else if(PowerBankInterval>=3600)
	{
			PowerBankInterval = RTCSetIntervalwhenBalSuspendStopChg;
			printk("[BAT][Bal][PwrB]:interval >1hr :%d\n",PowerBankInterval);
	}else{
			printk("[BAT][Bal][PwrB]:interval :%d\n",PowerBankInterval);
	}

	return  PowerBankInterval;
}

   
static void decideBalanceModeInterval(void)
{
	if(1==IsBalanceMode)
	{
		RTCSetIntervalwhenBalanceMode= CalBalanceInterval();
	}else if((0==IsBalanceMode)||(2==IsBalanceMode)){//Eason: do ForcePowerBankMode
		RTCSetIntervalwhenBalanceMode= CalPowerBankInterval();
	}
}
  #endif

static void DoWhenPadAlarmResume(void)
{	
    printk("[BAT][Ser]:PadAlarmResume()+++\n");

        balance_this->IsResumeUpdate = true;
        balance_this->IsResumeMahUpdate = true;
        balance_this->P02_IsResumeUpdate = true;

        if(delayed_work_pending(&balance_this->BatteryServiceUpdateWorker))
        {
            cancel_delayed_work_sync(&balance_this->BatteryServiceUpdateWorker);
        }
	//Hank: cancel BatteryServiceUpdateWorker need calculate capacity+++
	balance_this->NeedCalCap = true;
	 //Hank: cancel BatteryServiceUpdateWorker need calculate capacity--- 
        pr_debug("[BAT][SER][Pad]%s queue BatteryServiceUpdateWorker with calculate capacity\n",__func__);  
        queue_delayed_work(balance_this->BatteryServiceCapUpdateQueue, \
                               &balance_this->BatteryServiceUpdateWorker,\
                               0 * HZ);

        if( false == reportRtcReady()){
            queue_delayed_work(balance_this->BatteryServiceCapUpdateQueue,
                                   &balance_this->BatRtcReadyWorker,
                                   RTC_READY_DELAY_TIME * HZ);
        }

        printk("[BAT][Ser]:PadAlarmResume()---\n");
}
#endif
//Eason: dynamic set Pad alarm ---

//Eason set alarm +++
void SetRTCAlarm(void)
{
    int alarm_type = 0;
    uint32_t alarm_type_mask = 1U << alarm_type;
    unsigned long flags;
    struct timespec new_alarm_time;
    struct timespec mtNow;

    mtNow = current_kernel_time(); 
    new_alarm_time.tv_sec = 0;
    new_alarm_time.tv_nsec = 0;

    pr_debug("[BAT][alarm]:%ld.%ld\n",mtNow.tv_sec,mtNow.tv_nsec);

//Eason: dynamic set Pad alarm +++
#ifdef ASUS_FACTORY_BUILD
		new_alarm_time.tv_sec = mtNow.tv_sec+RTCSetInterval;
#else
	 #ifdef CONFIG_EEPROM_NUVOTON    
		if((1==AX_MicroP_IsP01Connected())&&( true == DecideIfPadDockHaveExtChgAC()))
		{
			new_alarm_time.tv_sec = mtNow.tv_sec+RTCSetInterval;
		}else if(( 0==IsBalanceMode)||( 2==IsBalanceMode))//PowerBankMode//Eason: do ForcePowerBankMode
		{			
			decideBalanceModeInterval();
			new_alarm_time.tv_sec = mtNow.tv_sec+RTCSetIntervalwhenBalanceMode;
		}else if( (true==IsBalanceSuspendStartcharge) && ( 1==IsBalanceMode))//BalanceMode need do suspend charge
		{
			decideBalanceModeInterval();
			new_alarm_time.tv_sec = mtNow.tv_sec+RTCSetIntervalwhenBalanceMode;
		}else{//BalanceMode dont need do suspend charge
			new_alarm_time.tv_sec = mtNow.tv_sec+RTCSetIntervalwhenBalSuspendStopChg;
		}
	#endif	
#endif
//Eason: dynamic set Pad alarm ---
    
    pr_debug("[BAT][alarm]:%ld,A66:%d\n",new_alarm_time.tv_sec,balance_this->A66_capacity);
    //ReportTime();
    spin_lock_irqsave(&bat_alarm_slock, flags);
    alarm_enabled |= alarm_type_mask;
    alarm_start_range(&bat_alarm,
    timespec_to_ktime(new_alarm_time),
    timespec_to_ktime(new_alarm_time));
    spin_unlock_irqrestore(&bat_alarm_slock, flags);

}

static void alarm_handler(struct alarm *alarm)
{
	unsigned long flags;

	printk("[BAT]battery alarm triggered\n");
	spin_lock_irqsave(&bat_alarm_slock, flags);

	wake_lock_timeout(&bat_alarm_wake_lock, 3 * HZ);
	wake_up(&bat_alarm_wait_queue);

	spin_unlock_irqrestore(&bat_alarm_slock, flags);
//Eason: dynamic set Pad alarm +++
#ifdef ASUS_FACTORY_BUILD
    SetRTCAlarm();
#else
	if(true==InSuspendNeedDoPadAlarmHandler)//Pad alarm handler need to do only when display off
	{
		queue_delayed_work(balance_this->BatteryServiceCapUpdateQueue, \
		                               &balance_this->PadAlarmResumeWorker,\
		                               0 * HZ);
	}
#endif
//Eason: dynamic set Pad alarm ---
}


static void SetBatLowRTCAlarm(void)
{
    int batLowAlarm_type = 0;
    uint32_t batLowAlarm_type_mask = 1U << batLowAlarm_type;
    unsigned long batlowflags;
    struct timespec new_batLowAlarm_time;
    struct timespec mtNow;

    mtNow = current_kernel_time(); 
    new_batLowAlarm_time.tv_sec = 0;
    new_batLowAlarm_time.tv_nsec = 0;

    printk("[BAT][alarm][BatLow]:%ld.%ld\n",mtNow.tv_sec,mtNow.tv_nsec);


    new_batLowAlarm_time.tv_sec = mtNow.tv_sec+RTCSetIntervalwhenBATlow;

    
    printk("[BAT][alarm][BatLow]:%ld,A66:%d\n",new_batLowAlarm_time.tv_sec
                                ,balance_this->BatteryService_IsBatLow);
    //ReportTime();
    spin_lock_irqsave(&batLow_alarm_slock, batlowflags);
    batLowAlarm_enabled |= batLowAlarm_type_mask;
    alarm_start_range(&batLow_alarm,
    timespec_to_ktime(new_batLowAlarm_time),
    timespec_to_ktime(new_batLowAlarm_time));
    spin_unlock_irqrestore(&batLow_alarm_slock, batlowflags);

} 

static void batLowAlarm_handler(struct alarm *alarm)
{
	unsigned long batlowflags;

	printk("[BAT][alarm]batLow alarm triggered\n");
	spin_lock_irqsave(&batLow_alarm_slock, batlowflags);

	wake_lock_timeout(&batLow_alarm_wake_lock, 3 * HZ);
	wake_up(&batLow_alarm_wait_queue);

	spin_unlock_irqrestore(&batLow_alarm_slock, batlowflags);
    SetBatLowRTCAlarm();
}

static void SetCableInRTCAlarm(void)
{
    int cableInAlarm_type = 0;
    uint32_t cableInAlarm_type_mask = 1U << cableInAlarm_type;
    unsigned long cableInflags;
    struct timespec new_cableInAlarm_time;
    struct timespec mtNow;

    mtNow = current_kernel_time(); 
    new_cableInAlarm_time.tv_sec = 0;
    new_cableInAlarm_time.tv_nsec = 0;

    //printk("[BAT][alarm][cableIn]:%ld.%ld\n",mtNow.tv_sec,mtNow.tv_nsec);


    new_cableInAlarm_time.tv_sec = mtNow.tv_sec+RTCSetIntervalwhenCABLEIn;

    
    printk("[BAT][SER][Alarm]%s(): NowTime:%ld, CableInAlarm:%ld, CableInOut:%d\n",__func__,mtNow.tv_sec,new_cableInAlarm_time.tv_sec
                                ,balance_this->BatteryService_IsCable);
    //ReportTime();
    spin_lock_irqsave(&cableIn_alarm_slock, cableInflags);
    cableInAlarm_enabled |= cableInAlarm_type_mask;
    alarm_start_range(&cableIn_alarm,
    timespec_to_ktime(new_cableInAlarm_time),
    timespec_to_ktime(new_cableInAlarm_time));
    spin_unlock_irqrestore(&cableIn_alarm_slock, cableInflags);

} 

static void cableInAlarm_handler(struct alarm *alarm)
{
	unsigned long cableInflags;

	printk("[BAT][alarm]cableIn alarm triggered\n");
	spin_lock_irqsave(&cableIn_alarm_slock, cableInflags);

	wake_lock_timeout(&cableIn_alarm_wake_lock, 3 * HZ);
	wake_up(&cableIn_alarm_wait_queue);

	spin_unlock_irqrestore(&cableIn_alarm_slock, cableInflags);
    SetCableInRTCAlarm();
}
//Eason set alarm ---

static void CheckBatRtcReady(struct work_struct *dat)
{
       AXC_BatteryService *_this = container_of(dat,AXC_BatteryService,\
                                                BatRtcReadyWorker.work);
       
       if( true == reportRtcReady())
       {
            _this->savedTime=updateNowTime(_this);
		//Eason: when change MaxMah clear interval+++
		_this->ForceSavedTime = updateNowTime(_this);
		//Eason: when change MaxMah clear interval---
            //ASUS_BSP Eason_Chang 1120 porting +++
	     #ifdef CONFIG_EEPROM_NUVOTON              
            if(1==AX_MicroP_IsP01Connected())
            {
                    _this->P02_savedTime = updateNowTime(_this);
            }
	     #endif	
	     #ifdef CONFIG_ASUSDEC
            if (true==reportDockInitReady())
	     {
                	if (AX_MicroP_IsDockReady())
			{
            			_this->Dock_savedTime = updateNowTime(_this);
            		}
            }
	     #endif //#ifdef CONFIG_ASUSDEC
	     //ASUS_BSP Eason_Chang 1120 porting ---               
            printk("[BAT][Ser]sys time ready\n");
       }else{
            queue_delayed_work(_this->BatteryServiceCapUpdateQueue,
                                   &_this->BatRtcReadyWorker,
                                   5 * HZ);
       }       
} 
//Hank: Tigauge Temperature Monitor+++
extern int get_Temp_from_TIgauge(void);
extern int smb346_write_reg(int cmd, void *data);
extern int smb346_read_reg(int cmd, void *data);
//Hank: add charger state check +++
#ifdef CONFIG_BAT_DEBUG
extern int smb346_state_check(void);
#endif
//Hank: add charger state check ---	

static int reg03_value = 0;
static int EnChg_GPIO85 = 0;
static int reg00_value = 0;
bool DisChg = false;
extern int ChargerTempLimit;
#define SMB346_EN_N 85

#ifdef ASUS_ME771KL_PROJECT
static void SMB345_GaugeTempMonitor(void)
{
	int gaugeTemp;
	int status = 0;
       uint8_t i2cdata[32]={0};

	status=smb346_read_reg(SMB346_CHG_CUR,&reg00_value);
		if(status > 0 && reg00_value >= 0)
              	pr_debug("[BAT][SER][SMB346] R00h = %d  \r\n",reg00_value);
		else
			printk("[BAT][SER][SMB346] R00h read error !!\r\n");
	status=smb346_read_reg(SMB346_FLOAT_VOLTAGE,&reg03_value);
		if(status > 0 && reg03_value >= 0)
              	pr_debug("[BAT][Ser][SMB346] R03h = %d  \r\n",reg03_value);
		else
			printk("[BAT][SER][SMB346] R03h read error !!\r\n");
	EnChg_GPIO85=gpio_get_value(SMB346_EN_N);
              	pr_debug("[BAT][Ser][SMB346] GPIO85 = %d  \r\n",EnChg_GPIO85);

	
	gaugeTemp = get_Temp_from_TIgauge();
	pr_debug("[BAT][SER][SMB346] Temperature = %d  \r\n",gaugeTemp);
	
	if(gaugeTemp >= 55 )//Hot Temperature Hard Limit
	{
		ChargerTempLimit = 1;
		printk("[BAT][SER][Temp]%s(): Temperature:%d => Hot Temperature Hard Limit\n",__func__,gaugeTemp);
		if(balance_this->TempLimit != 1 || reg03_value != 222 || EnChg_GPIO85 != 1 )
		{
			if(reg00_value != 146)
			{
				i2cdata[0] = 146;
				status=smb346_write_reg(SMB346_CHG_CUR,i2cdata);//Default Current
				if(status > 0 )
		              	printk("[BAT][SER][SMB346] Default Fast Charging Current \n");
		       	else
		       		printk("[BAT][SER][SMB346] Default Fast Charging Current Fail \n");
			}

			if(reg03_value != 222 )
			{
				i2cdata[0] = 222;
				status=smb346_write_reg(SMB346_FLOAT_VOLTAGE,i2cdata);//Down Voltage to 4.1V
				if(status > 0 )
		              	printk("[BAT][SER][SMB346] Down Float Voltage Success \n");
		       	else
		       		printk("[BAT][SER][SMB346] Down Float Voltage Fail \n");
				//gpio_set_value(85,1);//Disable charging
			}

			if(EnChg_GPIO85 != 1 )
			{
				gpCharger->EnableCharging(gpCharger,false);//Disable charging
		              printk("[BAT][SER][SMB346] Disable Charging \n");
			}
			balance_this->TempLimit = 1;
		}
	}
	else if(gaugeTemp >= 50 && 55 > gaugeTemp)//Hot Temratur Soft Limit
	{
		ChargerTempLimit = 2;
		printk("[BAT][SER][Temp]%s(): Temperature:%d=> Hot Temperature Soft Limit\n",__func__,gaugeTemp);
		if(balance_this->TempLimit != 2 || reg03_value != 222 )
		{
			if(reg00_value != 146)
			{
				i2cdata[0] = 146;
				status=smb346_write_reg(SMB346_CHG_CUR,i2cdata);//Default current
				if(status > 0 )
		              	printk("[BAT][SER][SMB346] Default Fast Charging Current \n");
		       	else
		       		printk("[BAT][SER][SMB346] Default Fast Charging Current Fail \n");
			}
		
			if(reg03_value != 222 )
			{	
				i2cdata[0] = 222;
				status=smb346_write_reg(SMB346_FLOAT_VOLTAGE,i2cdata);//Down Voltage to 4.1V
				if(status > 0 )
		              	printk("[BAT][SER][SMB346] Down Float Voltage Success \n");
		       	else
		       		printk("[BAT][SER][SMB346] Down Float Voltage Fail \n");
			}

			if(EnChg_GPIO85 != 0 && !DisChg)
			{
				gpCharger->EnableCharging(gpCharger,true);//Enable charging 
		              printk("[BAT][SER][SMB346] Enable Charging\n");
			}

			balance_this->TempLimit = 2;
		}
	}
	else if(gaugeTemp <= 10 && gaugeTemp > 0 )// Cold Temperature Soft Limit 
	{
		ChargerTempLimit = 3;
		printk("[BAT][SER][Temp]%s(): Temperature:%d => Cold Temperature Hard Limit\n",__func__,gaugeTemp);
		if(balance_this->TempLimit != 3 || reg00_value != 18)
		{
			if(reg00_value != 18)
			{
				i2cdata[0] = 18;
				status=smb346_write_reg(SMB346_CHG_CUR,i2cdata);//Draw current
				if(status > 0 )
		              	printk("[BAT][SER][SMB346] Down Charging Current Success \n");
		       	else
		       		printk("[BAT][SER][SMB346] Down Charging Current Fail \n");
			}

			if(reg03_value != 234)
			{
				i2cdata[0] = 234;
				status=smb346_write_reg(SMB346_FLOAT_VOLTAGE,i2cdata);//Default Voltage 
				if(status > 0 )
			              printk("[BAT][SER][SMB346] Default Float Voltage Success \n");
			       else
			       	printk("[BAT][SER][SMB346] Default Float Voltage Fail \n");
			}

			if(EnChg_GPIO85 != 0 && !DisChg)
			{
				gpCharger->EnableCharging(gpCharger,true);//Enable charging 
		              printk("[BAT][SER][SMB346] Enable Charging\n");
			}
			
			balance_this->TempLimit = 3;
		}
		
	}
	else if(gaugeTemp <= 0)// Cold Temperature Hard Limit 
	{
		ChargerTempLimit = 4;
		printk("[BAT][SER][Temp]%s(): Temperature:%d=> Cold Temperature Hard Limit\n",__func__,gaugeTemp);
		if(balance_this->TempLimit != 4  || EnChg_GPIO85 != 1 || reg00_value != 18)
		{
			if(reg00_value != 18)
			{
				i2cdata[0] = 18;
				status=smb346_write_reg(SMB346_CHG_CUR,i2cdata);//Draw current
				if(status > 0 )
		              	printk("[BAT][SER][SMB346] Down Charging Current Success \n");
		       	else
		       		printk("[BAT][SER][SMB346] Down Charging Current Fail \n");
			}

			if(reg03_value != 234)
			{
				i2cdata[0] = 234;
				status=smb346_write_reg(SMB346_FLOAT_VOLTAGE,i2cdata);//Default Voltage 
				if(status > 0 )
			              printk("[BAT][SER][SMB346] Default Float Voltage Success \n");
			       else
			       	printk("[BAT][SER][SMB346] Default Float Voltage Fail \n");
			}
			
			if(EnChg_GPIO85 != 1 )
			{
				gpCharger->EnableCharging(gpCharger,false);//Disable charging
		              printk("[BAT][SER][SMB346] Disable Charging \n");
			}

			balance_this->TempLimit = 4;
		}
		
	}
	else if(gaugeTemp < 50 && gaugeTemp > 10)//Normal Temperature
	{
		ChargerTempLimit = 0;
		printk("[BAT][SER][Temp]%s(): Temperature:%d => Normal Temperature \n",__func__,gaugeTemp);
		if(balance_this->TempLimit != 0 )//Normal Temperature only check TempLimit
		{

			if(reg00_value != 146)
			{
				i2cdata[0] = 146;
				status=smb346_write_reg(SMB346_CHG_CUR,i2cdata);//Default current
				if(status > 0 )
		              	printk("[BAT][SER][SMB346] Default Fast Charging Current \n");
		       	else
		       		printk("[BAT][SER][SMB346] Default Fast Charging Current Fail \n");
			}
			
			if(reg03_value != 234)
			{
				i2cdata[0] = 234;
				status=smb346_write_reg(SMB346_FLOAT_VOLTAGE,i2cdata);//Default Voltage 
				if(status > 0 )
			              printk("[BAT][SER][SMB346] Default Float Voltage Success \n");
			       else
			       	printk("[BAT][SER][SMB346] Default Float Voltage Fail \n");
			}


			if(EnChg_GPIO85 != 0 && !DisChg)
			{
				gpCharger->EnableCharging(gpCharger,true);//Enable charging 
		              printk("[BAT][SER][SMB346] Enable Charging\n");
			}

			balance_this->TempLimit = 0;
		}
			
		
       }
	else
	{
		printk("[BAT][SER][Temp]%s(): Temperature:%d => Other Temperature\n",__func__,gaugeTemp);
	}
	//Hank: add charger state check +++
	#ifdef CONFIG_BAT_DEBUG
	smb346_state_check();
	#endif
       //Hank: add charger state check ---	
}
#endif //#ifdef ASUS_ME771KL_PROJECT

//#ifdef ASUS_A91_PROJECT
extern void SMB346_Down_FlotVolt(void);
extern void SMB346_Default_FlotVolt(void) ;
extern void SMB346_Down_CCcurr(void);
extern void SMB346_Default_CCcurr(void);
static void SMB346_GaugeTempMonitor(void)
{
	int gaugeTemp;
	int status = 0;

	status=smb346_read_reg(SMB346_CHG_CUR,&reg00_value);
		if(status > 0 && reg00_value >= 0)
              	pr_debug("[BAT][SER][SMB346] R00h = %d  \r\n",reg00_value);
		else
			printk("[BAT][SER][SMB346] R00h read error !!\r\n");
	status=smb346_read_reg(SMB346_FLOAT_VOLTAGE,&reg03_value);
		if(status > 0 && reg03_value >= 0)
              	pr_debug("[BAT][Ser][SMB346] R03h = %d  \r\n",reg03_value);
		else
			printk("[BAT][SER][SMB346] R03h read error !!\r\n");
	EnChg_GPIO85=gpio_get_value(SMB346_EN_N);
              	pr_debug("[BAT][Ser][SMB346] GPIO85 = %d  \r\n",EnChg_GPIO85);
	
	gaugeTemp = get_Temp_from_TIgauge();
	pr_debug("[BAT][SER][SMB346] Temperature:%d \r\n",gaugeTemp);
	if(gaugeTemp >= 60 )//Hot Temperature Hard Limit
	{
		ChargerTempLimit = 1;
		printk("[BAT][SER][Temp]%s(): Temperature:%d => Hot Temperature Hard Limit\n",__func__,gaugeTemp);
		if(balance_this->TempLimit != 1 || reg03_value != 222 || EnChg_GPIO85 != 1 )
		{
			if(reg00_value != 145)
			{
				SMB346_Default_CCcurr();
			}

			if(reg03_value != 222 )
			{
				SMB346_Down_FlotVolt();
			}

			if(EnChg_GPIO85 != 1 )
			{
				gpCharger->EnableCharging(gpCharger,false);//Disable charging
		              printk("[BAT][SER][SMB346] Disable Charging \n");
			}

			balance_this->TempLimit = 1;
		}
	}
	else if(gaugeTemp >= 45 && 60> gaugeTemp)//Hot Temratur Soft Limit
	{
	 	ChargerTempLimit = 2;
		printk("[BAT][SER][Temp]%s(): Temperature:%d => Hot Temperature Soft Limit\n",__func__,gaugeTemp);
		if(balance_this->TempLimit != 2 || reg03_value != 222 )
		{
			if(reg00_value != 145)
			{
				SMB346_Default_CCcurr();
			}
		
			if(reg03_value != 222 )
			{	
				SMB346_Down_FlotVolt();
			}

			if(EnChg_GPIO85 != 0 && !DisChg)
			{
				gpCharger->EnableCharging(gpCharger,true);//Enable charging 
		              printk("[BAT][SER][SMB346] Enable Charging\n");
			}
			balance_this->TempLimit = 2;
		}
	}
	else if(gaugeTemp <= 10 && gaugeTemp > 0 )// Cold Temperature Soft Limit 
	{
		ChargerTempLimit = 3;
		printk("[BAT][SER][Temp]%s(): Temperature:%d=> Cold Temperature Soft Limit\n",__func__,gaugeTemp);
		if(balance_this->TempLimit != 3 || reg00_value != 18)
		{
			if(reg00_value != 18)
			{
				SMB346_Down_CCcurr();
			}

			if(reg03_value != 233)
			{
				SMB346_Default_FlotVolt();
			}

			if(EnChg_GPIO85 != 0 && !DisChg)
			{
				gpCharger->EnableCharging(gpCharger,true);//Enable charging 
		              printk("[BAT][SER][SMB346] Enable Charging\n");
			}
			
			balance_this->TempLimit = 3;
		}
		
	}
	else if(gaugeTemp <= 0)// Cold Temperature Hard Limit 
	{
		ChargerTempLimit = 4;
		printk("[BAT][SER][Temp]%s(): Temperature:%d => Cold Temperature Hard Limit\n",__func__,gaugeTemp);
		if(balance_this->TempLimit != 4 || EnChg_GPIO85 != 1 || reg00_value != 18)
		{
			if(reg00_value != 18)
			{
				SMB346_Down_CCcurr();
			}

			if(reg03_value != 233)
			{
				SMB346_Default_FlotVolt();
			}
			
			if(EnChg_GPIO85 != 1 )
			{
				gpCharger->EnableCharging(gpCharger,false);//Disable charging
		              printk("[BAT][SER][SMB346] Disable Charging \n");
			}

			balance_this->TempLimit = 4;
		}
		
	}
	else if(gaugeTemp < 45 && gaugeTemp > 10)//Normal Temperature
	{
		ChargerTempLimit = 0;
		printk("[BAT][SER][Temp]%s(): Temperature:%d => Normal Temperature \n",__func__,gaugeTemp);
		if(balance_this->TempLimit != 0 )//Normal Temperature only check TempLimit
		{

			if(reg00_value != 145)
			{
				SMB346_Default_CCcurr();
			}
			
			if(reg03_value != 233)
			{
				SMB346_Default_FlotVolt();
			}


			if(EnChg_GPIO85 != 0 && !DisChg)
			{
				gpCharger->EnableCharging(gpCharger,true);//Enable charging 
		              printk("[BAT][SER][SMB346] Enable Charging\n");
			}

			balance_this->TempLimit = 0;
		}
			
		
       }
	else
	{
		printk("[BAT][SER][Temp]%s(): Temperature:%d => Other Temperature\n",__func__,gaugeTemp);
	}
       //Hank: add charger state check +++
       #ifdef CONFIG_BAT_DEBUG
	smb346_state_check();
	#endif   
       //Hank: add charger state check ---		
}
//#endif //#ifdef ASUS_A91_PROJECT

static void checkCalCapTime(void)
{
	time_t nowTime;
	time_t nowKeep5MinInterval;
	time_t nowKeepInterval;
	nowTime = updateNowTime(balance_this);

	nowKeep5MinInterval = nowTime - balance_this->Keep5MinSavedTime;
	//Hank: Interval overflow handling+++	
      if(nowKeep5MinInterval <0)
      {
    		printk("[BAT][SER]%s():NowKeep5MinInterval overflow set interval KEEP_CAPACITY_TIME!\n",__func__);
		nowKeep5MinInterval = KEEP_CAPACITY_TIME;
      }
      //Hank: Interval overflow handling---

	nowKeepInterval = nowTime - balance_this->savedTime;
	//Hank: Interval overflow handling+++	
      if(nowKeepInterval <0)
      {
    		printk("[BAT][SER]%s():NowKeepInterval overflow set interval DEFAULT_POLLING_INTERVAL!\n",__func__);
		nowKeepInterval = DEFAULT_POLLING_INTERVAL;
      }
      //Hank: Interval overflow handling---
	
       pr_debug("[BAT][SER]%s()+++\n",__func__);      
       //Hank: if need keep 5Min check the interval if not check default interval+++
       if(balance_this->NeedKeep5Min)
       {
           printk("[BAT][SER]:Need keep 5Min \n");
		   if(nowKeep5MinInterval >= KEEP_CAPACITY_TIME){
			   balance_this->NeedCalCap = true;
			   balance_this->NeedKeep5Min = false;
			   printk("[BAT][SER]%s():already keep 5Min \n",__func__);
		   }else
			   pr_debug("[BAT][SER]%s():not already keep 5Min \n",__func__);
			
	  }
	  else
	  {
	  	   if(balance_this->test.ifFixedPollingInterval(&balance_this->test))
    		   {
        		   //Hank: test interval+++
			   if(!(balance_this->NeedCalCap) && (nowKeepInterval >= (balance_this->test.pollingInterval*3))){
				   balance_this->NeedCalCap = true;
				   printk("[BAT][SER]%s(): nowTime = %d, savedTime = %d, nowKeepInterval = %d \n default calculate capacity polling interval \n",__func__,(int)nowTime,(int)balance_this->savedTime,(int)nowKeepInterval);
			   }//Hank: default 3Min calculate capacity---
			   //Hank: capacity < 14% every 1Min calculate capacity+++
			   else if(!(balance_this->NeedCalCap) && (nowKeepInterval >= balance_this->test.pollingInterval) && balance_this-> A66_capacity < 14){
				   balance_this->NeedCalCap = true;
				   printk("[BAT][SER]%s():capacity < 14 calculate capacity every 1Min \n",__func__);
			   }
			   //Hank: test interval---
    		   }
		   else
		   {
			   //Hank: default 3Min calculate capacity+++
			   if(!(balance_this->NeedCalCap) && (nowKeepInterval >= DEFAULT_POLLING_INTERVAL)){
				   balance_this->NeedCalCap = true;
				   printk("[BAT][SER]%s(): nowTime = %d, savedTime = %d, nowKeepInterval = %d \n default calculate capacity polling interval \n",__func__,(int)nowTime,(int)balance_this->savedTime,(int)nowKeepInterval);
			   }//Hank: default 3Min calculate capacity---
			   //Hank: capacity < 14% every 1Min calculate capacity+++
			   else if(!(balance_this->NeedCalCap) && (nowKeepInterval >= DEFAULT_MONITOR_INTERVAL) && balance_this-> A66_capacity < 14){
				   balance_this->NeedCalCap = true;
				   printk("[BAT][SER]%s():capacity < 14 calculate capacity every 1Min \n",__func__);
			   }
			   //Hank: capacity < 14% every 1Min calculate capacity---
		   }
		   
		   
	   }
	   pr_debug("[BAT][SER]%s---\n",__func__);     
	   //Hank: if need keep 5Min check the interval if not check default interval---
}

static inline int AXC_BatteryService_getNextPollingInterval(struct AXC_BatteryService *_this)
{
/* Eason:cap queue  interval plugs a small value, prevent less than target interval.
*   In BatteryServiceCapSample() function checkCalCapTime() will check interval, then decide if calculate capacity.
*   In AXC_BatteryService_getNextPollingInterval() plug DEFAULT_CAP_QUEUE_INTERVAL_PLUGGED.
*/

    if(_this->test.ifFixedPollingInterval(&_this->test))
    {
        //return _this->test.pollingInterval;
        return (_this->test.pollingInterval+DEFAULT_CAP_QUEUE_INTERVAL_PLUGGED);//Eason:cap queue  interval plugs a small value, prevent less than target interval. 
    }
    else
    {
        //Hank: has cable or capacity < 14% return 1Min+++	
        //return _this->gauge->getNextPollingInterval(_this->gauge);
        if(_this->BatteryService_IsCable || _this-> A66_capacity < 14)
        	//return DEFAULT_MONITOR_INTERVAL;
        	return (DEFAULT_MONITOR_INTERVAL + DEFAULT_CAP_QUEUE_INTERVAL_PLUGGED);
	else
		//return DEFAULT_MONITOR_INTERVAL;
		return (DEFAULT_MONITOR_INTERVAL + DEFAULT_CAP_QUEUE_INTERVAL_PLUGGED);
        //Hank: has cable or capacity < 14% return 1Min---
    }

}
//Hank: Tigauge Temperature Monitor---
static int BatteryServiceGauge_OnCapacityReply(struct AXI_Gauge *gauge, struct AXI_Gauge_Callback *gaugeCb, int batCap, int result);
//Hank:move bootup_check_BatLow() from driver probe to capacity work to speed up driver probe+++
#define BATT_LOW 46
static void bootup_check_BatLow(void)
{
    int bat_low_gpio_value;
	int i, count=0;

	bat_low_gpio_value = gpio_get_value(BATT_LOW );  // bat_low:0  not_bat_low:1
    printk("[BAT][SER][BootUp] %s(),bat_low_gpio_value: %d \r\n", __FUNCTION__, bat_low_gpio_value);


	if (!bat_low_gpio_value) {
        for(i=0;i<10;i++){
        	if(!gpio_get_value(BATT_LOW )){
				count++;
				msleep(10);
			}else{
			    printk( "[BAT][SER][BootUp] %s(),leave_bat_low_debounce gpio 0: %d \r\n", __FUNCTION__,gpio_get_value(BATT_LOW ));
			    break;
			}
        }   

        if(count==10){
			printk( "[BAT][SER][BootUp]  %s(), set phone bat low as true\r\n", __FUNCTION__);
			g_BootUp_IsBatLow = true;
		}
	} else {
        for(i=0;i<10;i++){
        	if(gpio_get_value(BATT_LOW )){
				count++;
				msleep(10);
			}else{
			    printk( "[BAT][SER][BootUp] %s(),leave_bat_low_debounce gpio 1: %d \r\n", __FUNCTION__,gpio_get_value(BATT_LOW ));
			    break;
			}
        }

        if(count==10){
			printk("[BAT][SER][BootUp]  %s(), set phone bat low as false\r\n", __FUNCTION__);
			g_BootUp_IsBatLow = false;
        }
	}
	

}
//Eason: check FC flag to disable charge, prevent Battery over charge, let gauge FCC abnormal+++
void checkGaugeFCSetChg(void);
//Eason: check FC flag to disable charge, prevent Battery over charge, let gauge FCC abnormal---
//Hank:move bootup_check_BatLow() from driver probe to capacity work to speed up driver probe---
static void BatteryServiceCapSample(struct work_struct *dat)
{
       AXC_BatteryService *_this = container_of(dat,AXC_BatteryService,BatteryServiceUpdateWorker.work);
       wake_lock(&_this->cap_wake_lock);
	pr_debug("[BAT][SER]:%s+++ \n",__func__);

	//Hank: FirstForceResume do not need check Timing+++
	if(_this->IsFirstForceResume)
	{
		printk("[BAT][SER]%s():_this->IsFirstForceResume = true, set _this->IsFirstForceResume = false\n",__func__);
		//Hank first calculate capacity read battery id+++
		read_battery_id();
		//Hank first calculate capacity read battery id---

		//Hank:move bootup_check_BatLow() from driver probe to capacity work to speed up driver probe+++
		bootup_check_BatLow();
		//Hank:move bootup_check_BatLow() from driver probe to capacity work to speed up driver probe---
		_this->IsFirstForceResume = false;
	}
	else
	{
		pr_debug("[BAT][SER]%s():_this->IsFirstForceResume = false, do checkCalCapTime()\n",__func__);
       	checkCalCapTime();
	}
       //Hank: FirstForceResume do not need check Timing---
       
       if(_this->NeedCalCap)
       {
		 pr_debug("[BAT][SER]%s(): Need Calculate Capacity +++\n",__func__);
		 _this->IsCalculateCapOngoing = true;

		 //ASUS_BSP Eason_Chang 1120 porting +++
		#ifdef CONFIG_EEPROM_NUVOTON  
		#ifdef CONFIG_ASUSDEC
		if(true==reportDockInitReady())
		{  
			if (AX_MicroP_IsDockReady())
			{
				_this->Dockgauge->askCapacity(_this->Dockgauge);
			}
			else if(1==AX_MicroP_IsP01Connected())
			{
				printk("[BAT][SER]%s(): Dock bat error,Cap can't update,report P01\n",__func__);
				_this->P02gauge->askCapacity(_this->P02gauge);
			}
			else
			{
				printk("[BAT][SER]%s(): Dock bat error,Cap can't update,report A66\n",__func__);
				if(g_ASUS_hwID != A90_EVB0 )	
					_this->gauge->askCapacity(_this->gauge);
				else
					BatteryServiceGauge_OnCapacityReply(NULL, &_this->gaugeCallback, 100, 0);
				
			}
		}else
		#endif//#ifdef CONFIG_ASUSDEC
		if(1==AX_MicroP_IsP01Connected())
		{
			_this->P02gauge->askCapacity(_this->P02gauge);
		}
		else
		{
			 if(g_ASUS_hwID != A90_EVB0 )
				_this->gauge->askCapacity(_this->gauge);
			else
				BatteryServiceGauge_OnCapacityReply(NULL, &_this->gaugeCallback, 100, 0);
			
		}
		#else //#else CONFIG_EEPROM_NUVOTON  
		 	if(g_ASUS_hwID != A90_EVB0 )	
				_this->gauge->askCapacity(_this->gauge);
			else
				BatteryServiceGauge_OnCapacityReply(NULL, &_this->gaugeCallback, 100, 0);   
		#endif 
		//CONFIG_EEPROM_NUVOTON//ASUS_BSP Eason_Chang 1120 porting ---
			
		_this->NeedCalCap = false;
		pr_debug("[BAT][SER]%s(): Need Calculate Capacity ---\n",__func__);
	}

	//Eason: check FC flag to disable charge, prevent Battery over charge, let gauge FCC abnormal+++
	if(_this->A66_capacity >= 90)
	{
		checkGaugeFCSetChg();
	}
	//Eason: check FC flag to disable charge, prevent Battery over charge, let gauge FCC abnormal---
		
	//Hank: has cable monitor gauge temperature+++
	if(g_ASUS_hwID == A90_EVB0)//FW-1895 Reolad of non-volatile values during unplug
	{
		if(_this->BatteryService_IsCable && DffsDown == true)
		{
			//#ifdef ASUS_A91_PROJECT
			SMB346_GaugeTempMonitor();
			//#endif

			#ifdef ASUS_ME771KL_PROJECT
			SMB345_GaugeTempMonitor();
			#endif
		}
	}
	//Hank: has cable monitor gauge temperature---
		
	queue_delayed_work(_this->BatteryServiceCapUpdateQueue,&_this->BatteryServiceUpdateWorker,AXC_BatteryService_getNextPollingInterval(_this)* HZ);
        
        //Hank: because do temperature monitor every time move unlock & +++
	wake_unlock(&_this->cap_wake_lock);
	//Hank: because do temperature monitor every time move unlock ---	
	pr_debug("[BAT][SER]:%s()--- \n",__func__);

      
}

#ifdef CONFIG_ASUSDEC
static time_t Dock_BatteryService_getIntervalSinceLastUpdate(AXC_BatteryService  *_this)
{
    struct timespec mtNow;
    
    time_t Dock_intervalSinceLastUpdate;
    
    mtNow = current_kernel_time();

    if(_this->test.ifFixedFilterLastUpdateInterval(&_this->test)){
        
        Dock_intervalSinceLastUpdate = _this->test.filterLastUpdateInterval;
        
    }else if( true == _this->Dock_IsFirstAskCap){
    
        Dock_intervalSinceLastUpdate = 0;
          
    }else{

        if(mtNow.tv_sec >= _this->Dock_savedTime){

            pr_debug("[BAT][Ser][Dock]%s:%ld,%ld\n",__FUNCTION__,mtNow.tv_sec,_this->Dock_savedTime);
            
            Dock_intervalSinceLastUpdate = mtNow.tv_sec - _this->Dock_savedTime;

            //cause system time didn't work at first time update capacity (8secs) 
            //filter intervalSinceLastUpdate more than one month
            if(Dock_intervalSinceLastUpdate > 2592000){
                printk("[BAT][Ser][Dock]wrongInt %ld \n",Dock_intervalSinceLastUpdate);
                Dock_intervalSinceLastUpdate = 180;
            }    
         
        }else{
        
            printk("[BAT][Ser]%s:OVERFLOW....%ld,%ld\n",__FUNCTION__,mtNow.tv_sec,_this->Dock_savedTime);              
            //todo: to do the correct calculation here....
            Dock_intervalSinceLastUpdate = mtNow.tv_sec;
        }
    }

    return Dock_intervalSinceLastUpdate ; 
}
#endif//#ifdef CONFIG_ASUSDEC

static time_t P02_BatteryService_getIntervalSinceLastUpdate(AXC_BatteryService  *_this)
{
    struct timespec mtNow;
    
    time_t P02_intervalSinceLastUpdate;
    
    mtNow = current_kernel_time();

    if(_this->test.ifFixedFilterLastUpdateInterval(&_this->test)){
        
        P02_intervalSinceLastUpdate = _this->test.filterLastUpdateInterval;
        
    }else if( true == _this->P02_IsFirstAskCap){
    
        P02_intervalSinceLastUpdate = 0;
          
    }else{

        if(mtNow.tv_sec >= _this->P02_savedTime){

            pr_debug("[BAT][Ser][P02]%s:%ld,%ld\n",__FUNCTION__,mtNow.tv_sec,_this->P02_savedTime);
            
            P02_intervalSinceLastUpdate = mtNow.tv_sec - _this->P02_savedTime;

            //cause system time didn't work at first time update capacity (8secs) 
            //filter intervalSinceLastUpdate more than one month
            if(P02_intervalSinceLastUpdate > 2592000){
                printk("[BAT][Ser]wrongInt %ld \n",P02_intervalSinceLastUpdate);
                P02_intervalSinceLastUpdate = 180;
            }    
         
        }else{
        
            printk("[BAT][Ser]%s:OVERFLOW....%ld,%ld\n",__FUNCTION__,mtNow.tv_sec,_this->P02_savedTime);              
            //todo: to do the correct calculation here....
            P02_intervalSinceLastUpdate = mtNow.tv_sec;
        }
    }

    return P02_intervalSinceLastUpdate ; 
}

static time_t BatteryService_getIntervalSinceLastUpdate(AXC_BatteryService  *_this)
{
    struct timespec mtNow;
    
    time_t intervalSinceLastUpdate;
    
    mtNow = current_kernel_time();

    if(_this->test.ifFixedFilterLastUpdateInterval(&_this->test))
    {
        intervalSinceLastUpdate = _this->test.filterLastUpdateInterval;        
    }
    else if( true == _this->IsFirstAskCap)
    { 
        intervalSinceLastUpdate = 0;    
    }
    else
    {

        if(mtNow.tv_sec >= _this->savedTime)
	 {
            	pr_debug("[BAT][Ser]%s:%ld,%ld\n",__FUNCTION__,mtNow.tv_sec,_this->savedTime);
            
            	intervalSinceLastUpdate = mtNow.tv_sec - _this->savedTime;

		//Eason: if BatLow keep 15 min, shutdown devices, prevent gauge not low Cap+++
		if( (true==_this->BatteryService_IsBatLow) && ((mtNow.tv_sec - batLowTriggerTime )>BATLOW_KEEPTIME_SHUTDOWN) && (g_ASUS_hwID >= A90_EVB0) )
		{
				g_batLowLongTimeShut = true;
				ASUSEvtlog("[BAT][Fil][BatLow]Long tme\n");
		}
		//Eason: if BatLow keep 15 min, shutdown devices, prevent gauge not low Cap---

            	//cause system time didn't work at first time update capacity (8secs) 
            	//filter intervalSinceLastUpdate more than one month
            	if(intervalSinceLastUpdate > 2592000)
	     	{
                	printk("[BAT][SER]%s():wrongInt %ld \n",__func__,intervalSinceLastUpdate);
                	intervalSinceLastUpdate = 180;
            	}        
        }
	 else
	 {
        
            	printk("[BAT][Ser]%s:OVERFLOW....%ld,%ld\n",__FUNCTION__,mtNow.tv_sec,_this->savedTime);              
            	//todo: to do the correct calculation here....
            	intervalSinceLastUpdate = 180;
        }
    }

    return intervalSinceLastUpdate ; 
}

static int BatteryService_ChooseMaxMah(AXC_BatteryService  *_this, bool MahDrop)
{
    //Eason : In suspend have same cap don't update savedTime +++
    SameCapDontUpdateSavedTime = false;
    //Eason : In suspend have same cap don't update savedTime ---

    //Eason : if last time is 10mA +++
    if ( (NO_CHARGER_TYPE==_this->chargerType)||((NOTDEFINE_TYPE==_this->chargerType)) )
    {
    		//printk("dont change IsLastTimeMah10mA\n");
    }
    else
    {
	    	//Eason: when change MaxMah clear interval+++
	    	if(true == IsLastTimeMah10mA)
    		{
    			IfUpdateSavedTime = true;
    		}
	       IsLastTimeMah10mA = false;
		//Eason: when change MaxMah clear interval---
    }
    //Eason : if last time is 10mA ---

    switch(_this->chargerType)
    {
        case NO_CHARGER_TYPE:
             if(false == _this->HasCableBeforeSuspend && true==_this->IsResumeMahUpdate)
	      {
                	_this->IsResumeMahUpdate = false;
                	//Eason : In suspend have same cap don't update savedTime +++
                	SameCapDontUpdateSavedTime = true;
                	//Eason : In suspend have same cap don't update savedTime ---
			//Eason : if last time is 10mA +++
			IsLastTimeMah10mA = true;
			//Eason : if last time is 10mA ---
                	return SUSPEND_DISCHG_CURRENT;
             }
	      else
	      {
			//Eason : if last time is 10mA +++
			if(true == IsLastTimeMah10mA)
			{
				IfUpdateSavedTime = true;
			}
			IsLastTimeMah10mA = false;
			//Eason : if last time is 10mA ---
                	return MAX_DISCHG_CURRENT;
             }
		  
         case ILLEGAL_CHARGER_TYPE:
             if(false == MahDrop)
	      {
	      		if(A90_EVB0==g_ASUS_hwID)
                		return USB3p0_ILLEGAL_CURRENT;
			else
				return A91_ILLEGAL_CHG_CURRENT;//Eason: PM8941 illegal charger can only draw 500mA
             }
	      else
	      {
                	return MAX_DISCHG_CURRENT-USB_CHG_CURRENT;
             }
  
        case LOW_CURRENT_CHARGER_TYPE:
             if(false == MahDrop)
	      {
	      		if(A90_EVB0==g_ASUS_hwID)
                		return USB3p0_ILLEGAL_CURRENT;
			else
				return USB_CHG_CURRENT;//Eason: PM8941 usb can only draw 500mA
             }
	      else
	      {
                	return MAX_DISCHG_CURRENT-USB_CHG_CURRENT;
             }

#ifdef CONFIG_IDTP9023_CHARGER    
	  case WC_CHARGER_TYPE:    
		   return WIRELESS_CHG_CURRENT;
#endif  							
		  
        case NORMAL_CURRENT_CHARGER_TYPE:
                return PAD_CHG_CURRENT;
  
        case HIGH_CURRENT_CHARGER_TYPE:
             return AC_SUSPEND_CHG_CURRENT;
			 
        default:
             printk("[BAT][Ser]:%s():NO mapping\n",__FUNCTION__);
             if(true==_this->IsResumeMahUpdate)
	      {
                	_this->IsResumeMahUpdate = false;
                	//Eason : In suspend have same cap don't update savedTime +++
                	SameCapDontUpdateSavedTime = true;
                	//Eason : In suspend have same cap don't update savedTime ---
			//Eason : if last time is 10mA +++
			IsLastTimeMah10mA = true;
			//Eason : if last time is 10mA ---
                	return SUSPEND_DISCHG_CURRENT;
             }
	      else
	      {
			//Eason : if last time is 10mA +++
			if(true==IsLastTimeMah10mA)
			{
				IfUpdateSavedTime = true;
			}
			IsLastTimeMah10mA = false;
			//Eason : if last time is 10mA ---
                	return MAX_DISCHG_CURRENT;
             }   
        }     
}

static void CheckEoc(struct work_struct *dat)
{
    AXC_BatteryService *_this = container_of(dat,AXC_BatteryService,BatEocWorker.work);

    static int count = 0;
    
    if(NO_CHARGER_TYPE >= gpCharger->GetChargerStatus(gpCharger) ||!gpCharger->IsCharging(gpCharger))//if no charger && not being charging
    {
        count = 0;
        return;
    }

    if(count < 3)//recursive check
    {
	        int nCurrent =  _this->callback->getIBAT(_this->callback);
	        if(0 >= nCurrent &&  -90 < nCurrent && _this->A66_capacity >= 94)
		 {
	            count ++;
	            if(!delayed_work_pending(&_this->BatEocWorker))
		     {
		     		//why 10*Hz?
	                    queue_delayed_work(_this->BatteryServiceCapUpdateQueue, &_this->BatEocWorker,10 * HZ);
	            }

	        }
		 else
		 {
	            count = 0;
	            return;
	        }
     }
	
    printk("[BAT][Ser]%s:chg done\n",__FUNCTION__);
    _this->isMainBatteryChargingDone = true;
    return;
}

static void ResumeCalCap(struct work_struct *dat)
{
    time_t nowResumeTime;
    time_t nowResumeInterval;
    bool needDoResume=false;

    nowResumeTime = updateNowTime(balance_this);
    nowResumeInterval = nowResumeTime - balance_this->savedTime;
    //Hank: Interval overflow handling+++	
    if(nowResumeInterval <0)
    {
    		printk("[BAT][SER]%s():Interval overflow set interval RESUME_UPDATE_TIME!\n",__func__);
		nowResumeInterval = RESUME_UPDATE_TIME;
    }
    //Hank: Interval overflow handling---

    pr_debug("[BAT][SER]:resume queue+++\n");

    if(true == balance_this->BatteryService_IsBatLow 
        && nowResumeInterval > RESUME_UPDATE_TIMEwhenBATlow)
    {
           needDoResume = true; 
    }    
    else if(balance_this->A66_capacity <= CapChangeRTCInterval 
        && nowResumeInterval > RESUME_UPDATE_TIMEwhenCapLess20)
    {
           needDoResume = true;                
    }else if(nowResumeInterval > RESUME_UPDATE_TIME){
           needDoResume = true;
    }
    printk("[BAT][SER]:ResumeCalCap()===:%ld,%ld,%ld,A66:%d\n"
            ,nowResumeTime,balance_this->savedTime,nowResumeInterval,balance_this->A66_capacity);

    //ReportTime();

//Eason resume always calculate capacity no matter if in   Pad or CableIn or BatLow+++
   if(true==needDoResume)
   {
   	  //Eason set these flag when true==needDoResume+++
         balance_this->IsResumeUpdate = true;
         balance_this->IsResumeMahUpdate = true;
         balance_this->P02_IsResumeUpdate = true;
	  //Eason set these flag when true==needDoResume---	
	
        if(delayed_work_pending(&balance_this->BatteryServiceUpdateWorker))
        {
            cancel_delayed_work_sync(&balance_this->BatteryServiceUpdateWorker); 
        }
	 //Hank: need calculate capacity & temperature monitor+++
	balance_this->NeedCalCap = true;
	//Hank: need calculate capacity & temperature monitor--- 
        pr_debug("[Bat][SER]%s(): queue BatteryServiceUpdateWorker with calculate capacity\n",__func__);  
        queue_delayed_work(balance_this->BatteryServiceCapUpdateQueue, \
                               &balance_this->BatteryServiceUpdateWorker,\
                               0 * HZ);
    }
   else{
   	if(delayed_work_pending(&balance_this->BatteryServiceUpdateWorker))
	{
	            cancel_delayed_work_sync(&balance_this->BatteryServiceUpdateWorker);           
	 }
	  //Hank: temperature monitor only+++
	balance_this->NeedCalCap = false;
	  //Hank: temperature monitor only---
        pr_debug("[Bat][Ser]%s queue BatteryServiceUpdateWorker without calculate capacity\n",__func__);     
        queue_delayed_work(balance_this->BatteryServiceCapUpdateQueue, \
                               &balance_this->BatteryServiceUpdateWorker,\
                               0 * HZ);
   }
   pr_debug("[BAT][SER]:resume queue---\n");
//Eason resume always calculate capacity no matter if in   Pad or CableIn or BatLow---		
//Eason resume always calculate capacity no matter if in   Pad or CableIn or BatLow+++
#if 0		
#ifdef CONFIG_EEPROM_NUVOTON  //ASUS_BSP Eason_Chang 1120 porting +++
    if(1==AX_MicroP_IsP01Connected()&&true==needDoResume)
    {
        if(delayed_work_pending(&balance_this->BatteryServiceUpdateWorker))
        {
            cancel_delayed_work_sync(&balance_this->BatteryServiceUpdateWorker); 
        }    
        queue_delayed_work(balance_this->BatteryServiceCapUpdateQueue, \
                               &balance_this->BatteryServiceUpdateWorker,\
                               0 * HZ);
        printk("[BAT][Ser]:resume queue\n");
    }
    else 
#endif //CONFIG_EEPROM_NUVOTON//ASUS_BSP Eason_Chang 1120 porting ---        
        if(true==balance_this->BatteryService_IsBatLow && true==needDoResume)
    {
        if(delayed_work_pending(&balance_this->BatteryServiceUpdateWorker))
        {
            cancel_delayed_work_sync(&balance_this->BatteryServiceUpdateWorker); 
        }    
        queue_delayed_work(balance_this->BatteryServiceCapUpdateQueue, \
                               &balance_this->BatteryServiceUpdateWorker,\
                               0 * HZ);
        printk("[BAT][Ser]:bat Low resume queue\n");
    }
    else if(true==balance_this->BatteryService_IsCable && true==needDoResume)
    {
        if(delayed_work_pending(&balance_this->BatteryServiceUpdateWorker))
        {
            cancel_delayed_work_sync(&balance_this->BatteryServiceUpdateWorker); 
        }    
        queue_delayed_work(balance_this->BatteryServiceCapUpdateQueue, \
                               &balance_this->BatteryServiceUpdateWorker,\
                               0 * HZ);
        printk("[BAT][Ser]:cable in resume queue\n");        
    }
#endif
//Eason resume always calculate capacity no matter if in   Pad or CableIn or BatLow---	
}

static void CableOffKeep5Min(struct work_struct *dat)
{
    time_t nowCableOffTime;
    time_t nowCableOffInterval;
    bool needDoCableOffKeep5Min=false;

    nowCableOffTime = updateNowTime(balance_this);
    nowCableOffInterval = nowCableOffTime - balance_this->savedTime;

    if(nowCableOffInterval <  KEEP_CAPACITY_TIME)
    {
           needDoCableOffKeep5Min = true;
    }
    printk("[BAT][SER]:CableOffKeep5()===:%ld,%ld,%ld\n"
            ,nowCableOffTime,balance_this->savedTime,nowCableOffInterval);
    //ReportTime();

    if(true==needDoCableOffKeep5Min)
    {
        if(delayed_work_pending(&balance_this->BatteryServiceUpdateWorker))
        {
            cancel_delayed_work_sync(&balance_this->BatteryServiceUpdateWorker); 
        }
		
	 //Hank:	keep capacity 5Min save now time+++
	balance_this->Keep5MinSavedTime = updateNowTime(balance_this);
	//Hank:	keep capacity 5Min save now time---

	//Hank:	need keep capacity 5Min +++
	balance_this->NeedKeep5Min = true;
	balance_this->NeedCalCap = false;
	//Hank:	need keep capacity 5Min ---
	
       pr_debug("[Bat][SER]%s(): queue BatteryServiceUpdateWorker without calculate capacity\n",__func__);
       queue_delayed_work(balance_this->BatteryServiceCapUpdateQueue, \
                               &balance_this->BatteryServiceUpdateWorker,\
                               0 * HZ);
        
        balance_this->savedTime = updateNowTime(balance_this); 
	#ifdef CONFIG_EEPROM_NUVOTON  //ASUS_BSP Eason_Chang 1120 porting +++        
        if(1==AX_MicroP_IsP01Connected())
        {
                balance_this->P02_savedTime = updateNowTime(balance_this);
        } 
	#endif //CONFIG_EEPROM_NUVOTON

	#ifdef CONFIG_ASUSDEC
        if (true==reportDockInitReady())
	{
            	if (AX_MicroP_IsDockReady())
		{
        		balance_this->Dock_savedTime = updateNowTime(balance_this);
        	}
        }
	#endif //CONFIG_ASUSDEC
	//ASUS_BSP Eason_Chang 1120 porting ---        

	printk("[BAT][SER]:CableOffKeep5():savedtime:%ld,%ld,%ld\n"
            , balance_this->savedTime
            ,balance_this->P02_savedTime
            #ifdef CONFIG_ASUSDEC
            ,balance_this->Dock_savedTime
            #else
	      ,(long)0		
            #endif
	     
            );
    }    
}

//Eason: dynamic set Pad alarm +++		
#ifdef ASUS_FACTORY_BUILD	
static void PlugIntoP02SetRTC(struct work_struct *dat)
{
        SetRTCAlarm();
}
#else
static void PadRTCAlarmResume(struct work_struct *dat)
{
        DoWhenPadAlarmResume();
}
#endif
//Eason: dynamic set Pad alarm ---

static void BatLowTriggeredSetRTC(struct work_struct *dat)
{
        SetBatLowRTCAlarm();
}
//Eason cable in set alarm +++
static void CableInTriggeredSetRTC(struct work_struct *dat)
{
        SetCableInRTCAlarm();
}
//Eason cable in set alarm ---

//Eason bms notify +++
static void NotifyBmsChgBegan(struct work_struct *work)
{
	 pm8921_bms_charging_began();
}

static void NotifyBmsChgEnd(struct work_struct *work)
{
       if(true==balance_this->BatteryService_IsFULL )
   	{
   		pm8921_bms_charging_end(1);
	}else{
		pm8921_bms_charging_end(0);
	}
}
//Eason bms notify ---

//Eason after Pad update firmware, update status +++
static void UpdatePadInfo(struct work_struct *dat)
{
	//balance_this->callback->onServiceStatusUpdated(balance_this->callback);
#ifdef CONFIG_EEPROM_NUVOTON
	asus_bat_update_PadAcOnline();
#endif
}
//Eason after Pad update firmware, update status ---

//Hank in rom mode show "?" : bootUp check Rom mode queue+++
static void CheckIsGaugeRom(struct work_struct *dat)
{
	printk("[BAT][Ser]:bootUpCheckRomMode:%d\n",check_is_RomMode());
}
//Hank in rom mode show "?" : bootUp check Rom mode queue---

//Eason: use queue doBalanceMode in less 5min forceResume+++ 
static void forceResumeLess5minDobalanceWork(struct work_struct *dat)
{
#ifdef CONFIG_EEPROM_NUVOTON
		if(false == DecideIfPadDockHaveExtChgAC())
		{
	    		printk("[BAT][Ser]:less 5 min forceResume()+++\n");
	    		BatteryServiceDoBalance(balance_this);
			printk("[BAT][Ser]:less 5 min forceResume()---\n");	
		}	
#endif        
}
//Eason: use queue doBalanceMode in less 5min forceResume---
//ASUS_BSP Eason_Chang:add WirelessChg soft start+++
#ifdef CONFIG_PM_8941_CHARGER
static void WirelessChgSoftStart(struct work_struct *dat)
{
	wake_lock_timeout(&WC_softstart_wake_lock, 5 * HZ);
	WC_soft_start();
}

static void WirelessChgDCIN300(struct work_struct *dat)
{
	set_DCIN_300mA();
}
#endif
//ASUS_BSP Eason_Chang:add WirelessChg soft start---

//Eason: check FC flag to disable charge, prevent Battery over charge, let gauge FCC abnormal+++
/*
-	checkGaugeFCSetChg() need do after BatteryServiceDoBalance(), 
-     cause before enable chg, need to check if already disable in balance mode condition
-     Pad mode: need to check  FC status & flag DisChg.
-	Phone mode: only need to check FC status
*/
void checkGaugeFCSetChg(void)
{
	int EnChg_status;

	if(g_ASUS_hwID==A90_EVB0)//Eason_Chang:for A90 internal ChgGau
	{
		if(SMB346_INOK_Online)
		{
			setSmb346PreventOverFCC();//cause turn off vbus, chg setting will be reset. Can't only do when chg plug in
		}
		EnChg_status=gpio_get_value(SMB346_EN_N);
		
		if( (true == get_FC_flage_from_TIgauge())&&(g_ASUS_hwID == A90_EVB0) )
		{
			if( 0 == EnChg_status )
			{
				gpCharger->EnableCharging(gpCharger,false);   //disChg
				DisChg = true;
				printk("[BAT][SER][SMB346] Disable Charging FC\n");
			}	
		}else{
#ifdef CONFIG_EEPROM_NUVOTON	
			if(1==AX_MicroP_IsP01Connected())
			{
				if(true == DisChg)
				{
					//balance mode might Disable chg, don't enable;	
				}else{// false == DisChg, in balance mode nobody disable chg 
						if( 0!=EnChg_status )
						{
							gpCharger->EnableCharging(gpCharger,true);   
							DisChg = false;
							printk("[BAT][SER][SMB346] Enable Charging FC Pad\n");
						}	
				}
			}else
#endif		
			{
				if( 0!=EnChg_status )
				{
					gpCharger->EnableCharging(gpCharger,true);   
					DisChg = false;
					printk("[BAT][SER][SMB346] Enable Charging FC Phone\n");
				}	
			}	
		}
	}	//Eason_Chang:for A90 internal ChgGau
}

//Eason: check FC flag to disable charge, prevent Battery over charge, let gauge FCC abnormal---

static void BatteryService_InitWoker(AXC_BatteryService *_this)
{
        _this->BatteryServiceCapUpdateQueue \
            = create_singlethread_workqueue("BatteryServiceCapUpdateQueue");
        INIT_DELAYED_WORK(&_this->BatteryServiceUpdateWorker,\
                            BatteryServiceCapSample);

        INIT_DELAYED_WORK(&_this->BatRtcReadyWorker,\
                            CheckBatRtcReady);

        INIT_DELAYED_WORK(&_this->BatEcAcWorker,
                            CheckBatEcAc);

        INIT_DELAYED_WORK(&_this->BatEocWorker,
                            CheckEoc);

        INIT_DELAYED_WORK(&_this->ResumeWorker,
                            ResumeCalCap);

        INIT_DELAYED_WORK(&_this->CableOffWorker,
                            CableOffKeep5Min);
//Eason: dynamic set Pad alarm +++		
#ifdef ASUS_FACTORY_BUILD	
        INIT_DELAYED_WORK(&_this->SetRTCWorker,
                            PlugIntoP02SetRTC);
#else  
         INIT_DELAYED_WORK(&_this->PadAlarmResumeWorker,
                            PadRTCAlarmResume);
#endif
//Eason: dynamic set Pad alarm ---

        INIT_DELAYED_WORK(&_this->SetBatLowRTCWorker,
                            BatLowTriggeredSetRTC);

        INIT_DELAYED_WORK(&_this->SetCableInRTCWorker,
                            CableInTriggeredSetRTC); 
		
//Eason bms notify +++
	  INIT_DELAYED_WORK(&_this->BmsChgBeganWorker,NotifyBmsChgBegan);

        INIT_DELAYED_WORK(&_this->BmsChgEndWorker,NotifyBmsChgEnd);
//Eason bms notify ---		

	//Eason after Pad update firmware, update status +++
        INIT_DELAYED_WORK(&_this->UpdatePadWorker,UpdatePadInfo);
	//Eason after Pad update firmware, update status ---

	//Eason: use queue doBalanceMode in less 5min forceResume+++ 
	INIT_DELAYED_WORK(&_this->Less5MinDoBalanceWorker,forceResumeLess5minDobalanceWork);
	//Eason: use queue doBalanceMode in less 5min forceResume---
	
	//Hank in rom mode show "?" : bootUp check Rom mode queue+++
	//if( A80_EVB <= g_A68_hwID)
	//{
		INIT_DELAYED_WORK(&_this->CheckGaugeRomModeWorker,CheckIsGaugeRom);
	//}
	//Hank in rom mode show "?" : bootUp check Rom mode queue---
	//ASUS_BSP Eason_Chang:add WirelessChg soft start+++
#ifdef CONFIG_PM_8941_CHARGER	
		INIT_DELAYED_WORK(&_this->WirelessChgSoftStartWorker,WirelessChgSoftStart);
		INIT_DELAYED_WORK(&_this->WirelessChgDCIN300Worker,WirelessChgDCIN300);
#endif
	//ASUS_BSP Eason_Chang:add WirelessChg soft start---
}

static AXE_BAT_CHARGING_STATUS  AXC_BatteryService_getChargingStatus(struct AXI_BatteryServiceFacade * bat)
{
    AXE_BAT_CHARGING_STATUS status = BAT_DISCHARGING_STATUS; 

    AXC_BatteryService  *_this=
        container_of(bat, AXC_BatteryService, miParent);

    switch(_this->fsmState){ 
    case DISCHARGING_STATE:
         status = BAT_DISCHARGING_STATUS;
         break;
    case CHARGING_STATE:
         status = BAT_CHARGING_STATUS;
         break;
    case CHARGING_STOP_STATE:
#ifndef ASUS_FACTORY_BUILD       
    #ifdef CONFIG_EEPROM_NUVOTON  //ASUS_BSP Eason_Chang 1120 porting +++        
             //ASUS_BSP +++ Eason_Chang BalanceMode
             if(1==AX_MicroP_IsP01Connected()){       
                    status = BAT_NOT_CHARGING_STATUS;  
             }else{              
             //ASUS_BSP --- Eason_Chang BalanceMode
             status = BAT_CHARGING_STATUS;
             }
    #else//CONFIG_EEPROM_NUVOTON//ASUS_BSP Eason_Chang 1120 porting
                status = BAT_CHARGING_STATUS;
    #endif //CONFIG_EEPROM_NUVOTON//ASUS_BSP Eason_Chang 1120 porting ---
#else
                status = BAT_NOT_CHARGING_STATUS;
#endif//#ifndef ASUS_FACTORY_BUILD
         break;
    case CHARGING_FULL_STATE:
         status = BAT_CHARGING_FULL_STATUS;
         break;
    case CHARGING_FULL_KEEP_STATE:
         status = BAT_CHARGING_FULL_STATUS;
         break;
    case CHARGING_FULL_KEEP_STOP_STATE:
         status = BAT_CHARGING_FULL_STATUS;
         break;
    default:
         printk("[BAT][Ser]%s():status error\n",__FUNCTION__);
    }
    return status;
}
static int  AXC_BatteryService_getCapacity(struct AXI_BatteryServiceFacade * bat)
{
    AXC_BatteryService  *_this=
        container_of(bat, AXC_BatteryService, miParent);

	if( (A90_EVB0==g_ASUS_hwID)||(1==IsBalanceTest()) )
	   return _this->A66_capacity;
	//Eason show BMS cap to user+++
	else
	{
		if( gBMS_Cap <= 1)
		{
			if(true==g_BootUp_IsBatLow )
			{
				if(1==getIfonline())
				{
					printk("[BAT][BMS]BootUp IsBatLow, Cable on, dont shutdown:%d\n",gBMS_Cap);
					return 1;
				}else{
					printk("[BAT][BMS]BootUp IsBatLow, Cable off, need shutdown:%d\n",gBMS_Cap);

					#ifdef CONFIG_FASTBOOT//Eason: fix fastboot kernel poweroff when BAT cap not 0%
						if(is_fastboot_enable()){
							printk("[BAT][Fastboot]kernel_power_off\n");
							kernel_power_off();
						}
					#endif //#ifdef CONFIG_FASTBOOT
					
					return 0;
				}
			}else if(false==_this->BatteryService_IsBatLow){
				printk("[BAT][BMS]Bat low not yet:%d\n",gBMS_Cap);
				return 1;
			}else{
				printk("[BAT][BMS]less 1p & BatLow need shutdown:%d\n",gBMS_Cap);

				#ifdef CONFIG_FASTBOOT//Eason: fix fastboot kernel poweroff when BAT cap not 0%
					if(is_fastboot_enable()){
							printk("[BAT][Fastboot]kernel_power_off\n");
							kernel_power_off();
					}
				#endif //#ifdef CONFIG_FASTBOOT
					
				return 0;//BMS table 1% need mapping to 0%
			}
		}else if( (true==_this->BatteryService_IsFULL)&&(gBMS_Cap>=99) ) {
			printk("[BAT][BMS]more than 99p & FULL flag, show 100:%d\n",gBMS_Cap);
			return 100;
		}else if( (false==_this->BatteryService_IsFULL)&&(100 == gBMS_Cap) ) {
			printk("[BAT][BMS]100p but Not_FULL flag, show 99:%d\n",gBMS_Cap);
			return 99;			
		}else
			return gBMS_Cap;
	}	
	//Eason show BMS cap to user---
}
extern void led_set_charger_mode(uint8_t led_type); //ASUS BSP Deeo : LED charger mode +++
static void AXC_BatteryService_onCableInOut(struct AXI_BatteryServiceFacade *bat, AXE_Charger_Type type)
{
    	AXC_BatteryService  *_this = container_of(bat, AXC_BatteryService, miParent);

    	//Eason cable in set alarm +++
    	unsigned long cableInFlags;
    	//Eason cable in set alarm ---
	uint8_t led_type=0; //ASUS BSP Deeo : LED charger mode +++

	printk("[BAT][Ser]:onCableInOut()+++\n");
    	_this->chargerType = type ;
    	_this->fsm->onCableInOut(_this->fsm,type);

//Eason: judge status by user Cap +++
#ifdef CONFIG_PM_8941_CHARGER		
    	if( 100 == pm8941_getCapacity())
	{
          	_this->fsm->onChargingStop(_this->fsm,CHARGING_DONE);   
    	}    
#else
    	if( 100 == _this->A66_capacity)
	{
          	_this->fsm->onChargingStop(_this->fsm,CHARGING_DONE);   
    	}    
#endif
//Eason: judge status by user Cap ---

    	switch(type)
    	{
	        case NO_CHARGER_TYPE:
			//Hank: HW gauge do not need notify+++	
			if(g_ASUS_hwID != A90_EVB0 )
	             	_this->gauge->notifyCableInOut(_this->gauge,false);
			//Hank: HW gauge do not need notify---
			
	              _this->BatteryService_IsCable = false ;

			//Eason :when  low bat Cap draw large current  +++	 
			if(10 <= _this->A66_capacity )
			{
	             		schedule_delayed_work(&_this->CableOffWorker,1*HZ);//keep 100% 5 min
			}
			//Eason :when  low bat Cap draw large current  ---

			//Eason cable in set alarm +++
	             spin_lock_irqsave(&cableIn_alarm_slock, cableInFlags);
	             alarm_try_to_cancel(&cableIn_alarm);
	             spin_unlock_irqrestore(&cableIn_alarm_slock, cableInFlags);
	             //Eason cable in set alarm --- 
			//Eason : Wireless PMA spec Rx turn off Tx +++
#ifdef CONFIG_IDTP9023_CHARGER
			g_WcEocTime = updateNowTime(_this);
#endif
			//Eason : Wireless PMA spec Rx turn off Tx ---

				 //_this->BatteryService_IsFULL = false;
	             //_this->gauge->notifyBatFullChanged(_this->gauge,false);

			led_type = 0; //ASUS BSP Deeo : LED charger mode +++
	             break;
				 
	        case ILLEGAL_CHARGER_TYPE:
	        case LOW_CURRENT_CHARGER_TYPE:
	        case NORMAL_CURRENT_CHARGER_TYPE:
	        case HIGH_CURRENT_CHARGER_TYPE:
#ifdef CONFIG_IDTP9023_CHARGER    
		 case WC_CHARGER_TYPE:    
#endif  				

			//Hank: HW gauge do not need notify+++	
			if(g_ASUS_hwID != A90_EVB0 )	
		            	_this->gauge->notifyCableInOut(_this->gauge,true);
			//Hank: HW gauge do not need notify---

			_this->BatteryService_IsCable = true ;
		       //Eason: dynamic set Pad alarm, Pad has its own alrarm+++
			if( NORMAL_CURRENT_CHARGER_TYPE!=_this->chargerType)
		 	{
		             //Eason cable in set alarm +++
		             schedule_delayed_work(&_this->SetCableInRTCWorker, 0*HZ);
		             //Eason cable in set alarm ---
		 	}
			//Eason: dynamic set Pad alarm, Pad has its own alrarm---

			if(g_ASUS_hwID == A90_EVB0)
			{
				//Hank: cable in do temp monitor+++
				SMB346_GaugeTempMonitor();
				//Hank: cable in do temp monitor---
				//Eason: check FC flag to disable charge, prevent Battery over charge, let gauge FCC abnormal+++
				checkGaugeFCSetChg();
				//Eason: check FC flag to disable charge, prevent Battery over charge, let gauge FCC abnormal---
			}
			
			#ifdef ASUS_FACTORY_BUILD
			//Eason: take off 5060rule let A80 can always charge+++
			if( true == charger_limit_enable )
			{
				g_5060modeCharging = true;
	              	Do_Factory5060Mode();
			}		
			//Eason: take off 5060rule let A80 can always charge---
			#endif//#ifdef ASUS_FACTORY_BUILD
			led_type = 1; //ASUS BSP Deeo : LED charger mode +++
	              break;

		default:
             		printk("[BAT][Ser]:%s():NO mapping\n",__FUNCTION__);
             		break;
    	} 

	led_set_charger_mode(led_type);	//ASUS BSP Deeo : LED charger mode +++

	printk("[BAT][Ser]:onCableInOut():%d---\n",type);
}
static void AXC_BatteryService_onChargingStop(struct AXI_BatteryServiceFacade *bat,AXE_Charging_Error_Reason reason)
{/*
    AXC_BatteryService  *_this=
        container_of(bat, AXC_BatteryService, miParent);

    _this->fsm->onChargingStop(_this->fsm,reason);
    _this->BatteryService_IsCharging = false ;

    if (CHARGING_DONE == reason){
        
        _this->BatteryService_IsFULL = true;
        
        _this->gauge->notifyBatFullChanged(_this->gauge,true);

        wake_lock(&_this->cap_wake_lock);
        _this->gauge->askCapacity(_this->gauge);
    }
 */   
}
static void AXC_BatteryService_onChargingStart(struct AXI_BatteryServiceFacade *bat)
{/*
    AXC_BatteryService  *_this=
        container_of(bat, AXC_BatteryService, miParent);

    _this->fsm->onChargingStart(_this->fsm);
    _this->BatteryService_IsCharging = true ;

 */
}
static void AXC_BatteryService_onBatteryLowAlarm(struct AXI_BatteryServiceFacade *bat, bool isCurrentBattlow)
{
    AXC_BatteryService  *_this=
        container_of(bat, AXC_BatteryService, miParent);
    
    unsigned long batLowFlags;

    if(false==_this->BatteryService_IsBatLow && true==isCurrentBattlow)
    {
            schedule_delayed_work(&balance_this->SetBatLowRTCWorker, 0*HZ);
	     //Eason: if BatLow keep 15 min, shutdown devices+++
	     if(g_ASUS_hwID >= A90_EVB0)
	     {
			batLowTriggerTime = updateNowTime(balance_this);
			ASUSEvtlog("[BAT][Fil][BatLow]TriggerTime\n");
	     }
	     //Eason: if BatLow keep 15 min, shutdown devices---
    }
    else if(true==_this->BatteryService_IsBatLow && false==isCurrentBattlow)
    {
            spin_lock_irqsave(&batLow_alarm_slock, batLowFlags);
            alarm_try_to_cancel(&batLow_alarm);
            spin_unlock_irqrestore(&batLow_alarm_slock, batLowFlags);
	     //Eason: if BatLow keep 15 min, shutdown devices+++
	     g_batLowLongTimeShut = false;
	     ASUSEvtlog("[BAT][Fil][BatLow]Release\n");
	    //Eason: if BatLow keep 15 min, shutdown devices---
    }
    _this->BatteryService_IsBatLow = isCurrentBattlow ;


}   
static void AXC_BatteryService_onBatteryRemoved(struct AXI_BatteryServiceFacade * bat, bool isRemoved)
{
   // AXC_BatteryService  *_this=
    //    container_of(bat, AXC_BatteryService, miParent);

}
static void Record_P02BeforeSuspend(void)
{
#ifdef CONFIG_EEPROM_NUVOTON  //ASUS_BSP Eason_Chang 1120 porting +++
	if( (1==AX_MicroP_get_USBDetectStatus(Batt_P01))||(2==AX_MicroP_get_USBDetectStatus(Batt_P01)) )//Eason: Pad plug usb show icon & cap can increase
    {
		balance_this->P02_IsCable = true;
	}else{
		balance_this->P02_IsCable = false;
	}

    //balance_this->P02_ChgStatusBeforeSuspend = AX_MicroP_get_ChargingStatus(Batt_P01);
#endif //CONFIG_EEPROM_NUVOTON//ASUS_BSP Eason_Chang 1120 porting ---    
}
static int P02_ChooseMaxMahBeforeSuspend(void)
{
#ifdef CONFIG_EEPROM_NUVOTON
    if(true == DecideIfPadDockHaveExtChgAC())
    {
          return AC_CHG_CURRENT;
#ifdef CONFIG_ASUSDEC
    }else if(1==AX_MicroP_IsECDockIn() ){
          return USB_CHG_CURRENT;
#endif          
    }else{
	  return PAD_CHG_CURRENT;
    }
#else
    return PAD_CHG_CURRENT;
#endif
}

static int P02_ChooseMaxMah(void)
{
#ifdef CONFIG_EEPROM_NUVOTON
   if(true == DecideIfPadDockHaveExtChgAC()){
          return AC_CHG_CURRENT;
#ifdef CONFIG_ASUSDEC          
   }else if(1==AX_MicroP_IsECDockIn() ){
          return USB_CHG_CURRENT;
#endif          
   }else{
	  return PAD_CHG_CURRENT;
   }
#else
   return PAD_CHG_CURRENT;
#endif
}
 #ifdef CONFIG_ASUSDEC //ASUS_BSP Eason_Chang 1120 porting +++
static void Record_DockBeforeSuspend(void)
{
	if(1==AX_MicroP_get_USBDetectStatus(Batt_Dock))
    {
		balance_this->Dock_IsCable = true;
	}else{
		balance_this->Dock_IsCable = false;
	}

    //balance_this->Dock_ChgStatusBeforeSuspend = AX_MicroP_get_ChargingStatus(Batt_Dock);   
}
#endif //CONFIG_EEPROM_NUVOTON//ASUS_BSP Eason_Chang 1120 porting ---   
#ifdef CONFIG_ASUSDEC 
static int Dock_ChooseMaxMahBeforeSuspend(void)
{
    if(true == DecideIfPadDockHaveExtChgAC())
    {
          return AC_CHG_CURRENT;     
    }else{
	  return DOCK_SUSPEND_DISCHG_CURRENT;
    }
}

static int Dock_ChooseMaxMah(void)
{
#ifdef CONFIG_ASUSDEC  
   if(true == DecideIfPadDockHaveExtChgAC()){
          return AC_CHG_CURRENT;
#ifdef CONFIG_ASUSDEC          
   }else if(1==AX_MicroP_IsECDockIn() ){
          return USB_CHG_CURRENT;
#endif          
   }else{
	  return DOCK_DISCHG_CURRENT;
   }
#else
   	  return DOCK_DISCHG_CURRENT;
#endif
}
#endif
/*void cancelBatCapQueueBeforeSuspend(void){
    printk("[BAT] cancel Bat Cap Queue\n");
    cancel_delayed_work_sync(&balance_this->BatteryServiceUpdateWorker);
}*/
static void AXC_BatteryService_dockSuspend(struct AXI_BatteryServiceFacade *bat)
{
#ifdef CONFIG_ASUSDEC  
    AXC_BatteryService  *_this=
        container_of(bat, AXC_BatteryService, miParent);

    if (true==reportDockInitReady()){
        	//if (AX_MicroP_IsDockReady()){
            		Record_DockBeforeSuspend();
                    _this->Dock_HasCableBeforeSuspend = _this->Dock_IsCable;
                    _this->Dock_MaxMahBeforeSuspend = Dock_ChooseMaxMahBeforeSuspend();
                    printk("[BAT][Ser][Dock]:suspend:%d,%d\n"
                            ,_this->Dock_IsCable
                            //,_this->Dock_ChgStatusBeforeSuspend
                            ,_this->Dock_MaxMahBeforeSuspend);
        	//}
    }
#endif    
}

//Eason: A68 new balance mode +++
#ifndef ASUS_FACTORY_BUILD
#ifdef CONFIG_EEPROM_NUVOTON
void doInBalanceModeWhenSuspend(void)
{


	//when resume default turn off vbus +++
	IsSystemdraw = false;
	//when resume default turn off vbus ---

	if( false==IsBalanceSuspendStartcharge )
	{

		set_microp_vbus(0);
#if 0	//sina 2014/11/26: micro fw takes care of 5v on/off		
              if(st_MICROP_Sleep == AX_MicroP_getOPState()) // only turn off 5V when microp state is in sleep state to avoid i2c issue of touch IC
        		set_5VPWR_EN(0);
#endif 			  
		printk("[BAT][Bal]stop 5VPWR:%d,Vbus:%d\n"
                                        ,get_5VPWR_EN(),get_microp_vbus());
		 //gpCharger->EnableCharging(gpCharger,false); //Hank: remove disable charging when cable off
		 DisChg = true;
              balance_this->fsm->onChargingStop(balance_this->fsm,BALANCE_STOP);               
             IsBalanceCharge = 0;
	
	}else if(true==IsBalanceSuspendStartcharge){//remember this flag to do suspendCharge before suspendStopChg condition match

#if 0	//sina 2014/11/26: micro fw takes care of 5v on/off	
		set_5VPWR_EN(1);
#endif
		set_microp_vbus(1);
		printk("[BAT][Bal]start 5VPWR:%d,Vbus:%d\n"
                                        ,get_5VPWR_EN(),get_microp_vbus());	
		 gpCharger->EnableCharging(gpCharger,true);
		 DisChg = false;
              balance_this->fsm->onChargingStart(balance_this->fsm);                 
             IsBalanceCharge = 1;
			 
	}


}
#endif//CONFIG_EEPROM_NUVOTON
#endif//ASUS_FACTORY_BUILD
//Eason: A68 new balance mode ---



extern void setSmb346CC_Curr1250mA_Iterm50(void);
extern int g_flag_csvoice_fe_connected;
extern void setChgLimitThermalRuleDrawCurrent(bool isSuspendCharge);
static void AXC_BatteryService_suspend(struct AXI_BatteryServiceFacade *bat)
{
    	AXC_BatteryService  *_this = container_of(bat, AXC_BatteryService, miParent);

    	printk("[BAT][Ser]:suspend()+++\n");

	//Eason: dynamic set Pad alarm +++
	#ifndef ASUS_FACTORY_BUILD		
	InSuspendNeedDoPadAlarmHandler=true;
	#endif
	//Eason: dynamic set Pad alarm ---	

    	//if(false == _this->IsSuspend){

        //_this->IsSuspend = true;
        _this->HasCableBeforeSuspend = _this->BatteryService_IsCable;
        Record_P02BeforeSuspend();
        _this->P02_HasCableBeforeSuspend = _this->P02_IsCable;
        _this->P02_MaxMahBeforeSuspend = P02_ChooseMaxMahBeforeSuspend();
        
        //cancel_delayed_work_sync(&_this->BatteryServiceUpdateWorker); 

    //}

	//Eason: A68 new balance mode +++
	#ifndef ASUS_FACTORY_BUILD
	#ifdef CONFIG_EEPROM_NUVOTON
	if ((1==AX_MicroP_IsP01Connected())&&(1 == IsBalanceMode)&&(false == DecideIfPadDockHaveExtChgAC()))
	{
		printk("[BAT][Bal]Phone:%d,Pad:%d\n",_this->A66_capacity,_this->Pad_capacity);
		doInBalanceModeWhenSuspend();
	}
	if (1==AX_MicroP_IsP01Connected() && (1==getIfonline()) )
		setChgLimitThermalRuleDrawCurrent(true);
	#endif
	#endif	
	//Eason: A68 new balance mode ---

	//Eason: check FC flag to disable charge, prevent Battery over charge, let gauge FCC abnormal+++
	checkGaugeFCSetChg();
	//Eason: check FC flag to disable charge, prevent Battery over charge, let gauge FCC abnormal---
	
	//Hank:suspend and not in phone call set cc current 1250mA+++
	if(g_flag_csvoice_fe_connected == 0 && SMB346_INOK_Online && (A90_EVB0==g_ASUS_hwID))
	{
		setSmb346CC_Curr1250mA_Iterm50();
		printk("[BAT][Ser]%s()set cc current 1250mA\n",__func__);
	}
	else
	{
		pr_debug("[BAT][Ser]%s()phone call:%d cable-in:%d do not set cc current\n",__func__,g_flag_csvoice_fe_connected,SMB346_INOK_Online);
	}
	//Hank:suspend and not in phone call set cc current 1250mA---
	

	if(DffsDown == true && SMB346_INOK_Online && (A90_EVB0==g_ASUS_hwID))
	{
		//#ifdef ASUS_A91_PROJECT
		setSmb346DefaultHotTempLimit();
		//#endif
		#ifdef ASUS_ME771KL_PROJECT
		setSmb345DefaultHotTempLimit();
		#endif
	}

    	printk("[BAT][Ser]:suspend()---\n");

}
//Eason resume always calculate capacity no matter if in   Pad or CableIn or BatLow+++
#if 0
static void AXC_BatteryService_resume(struct AXI_BatteryServiceFacade *bat,int delayStartInSeconds)
{
    AXC_BatteryService  *_this=
        container_of(bat, AXC_BatteryService, miParent);

#ifdef CONFIG_EEPROM_NUVOTON  //ASUS_BSP Eason_Chang 1120 porting +++    
    if( (1==AX_MicroP_IsP01Connected())||(true==_this->BatteryService_IsCable) )
    {

        printk("[BAT][Ser]:resume()+++\n");
#ifndef ASUS_FACTORY_BUILD		
	  ASUSEvtlog("[BAT][Bal]resume:%d\n",IsBalanceSuspendStartcharge);	
#endif//ASUS_FACTORY_BUILD	
    //if(true == _this->IsSuspend){

        //_this->IsSuspend = false;

        _this->IsResumeUpdate = true;
        _this->IsResumeMahUpdate = true;
        _this->P02_IsResumeUpdate = true;

        /*if(delayed_work_pending(&_this->BatteryServiceUpdateWorker)){  
        cancel_delayed_work(&_this->BatteryServiceUpdateWorker);
        printk("[BAT][Ser]:resume pending\n");
        }*/
        schedule_delayed_work(&_this->ResumeWorker,1*HZ);
        wake_lock_timeout(&_this->resume_wake_lock,2* HZ);
        
        

        if( false == reportRtcReady()){
            queue_delayed_work(_this->BatteryServiceCapUpdateQueue,
                                   &_this->BatRtcReadyWorker,
                                   RTC_READY_DELAY_TIME * HZ);
        }
    //}

        printk("[BAT][Ser]:resume()---\n");
    }else 
#endif //CONFIG_EEPROM_NUVOTON//ASUS_BSP Eason_Chang 1120 porting ---    
    if(true==balance_this->BatteryService_IsBatLow)
    {
        printk("[BAT][Ser][BatLow]:resume()+++\n");

        _this->IsResumeUpdate = true;
        _this->IsResumeMahUpdate = true;
        _this->P02_IsResumeUpdate = true;

        schedule_delayed_work(&_this->ResumeWorker,1*HZ);
        wake_lock_timeout(&_this->resume_wake_lock,2* HZ);
        
        

        if( false == reportRtcReady()){
            queue_delayed_work(_this->BatteryServiceCapUpdateQueue,
                                   &_this->BatRtcReadyWorker,
                                   RTC_READY_DELAY_TIME * HZ);
        }

        printk("[BAT][Ser][BatLow]:resume()---\n");
    }
}
#endif//end #if 0 
//Eason resume always calculate capacity no matter if in   Pad or CableIn or BatLow---
#ifndef ASUS_FACTORY_BUILD	
extern void setChgLimitInPadWhenChgReset(void);
#endif
//Eason resume always calculate capacity no matter if in   Pad or CableIn or BatLow+++
extern void setSmb346CC_Curr900mA_Iterm50(void);
static void AXC_BatteryService_resume(struct AXI_BatteryServiceFacade *bat,int delayStartInSeconds)
{
    AXC_BatteryService  *_this=
        container_of(bat, AXC_BatteryService, miParent);

        printk("[BAT][Ser]:resume()+++\n");

	//Hank:resume set cc current 900mA+++
	if(SMB346_INOK_Online && (A90_EVB0==g_ASUS_hwID))
		setSmb346CC_Curr900mA_Iterm50();
	//Hank:resume set cc current 900mA---

	
#ifdef CONFIG_EEPROM_NUVOTON
	if(1==AX_MicroP_IsP01Connected())
	{
		#ifndef ASUS_FACTORY_BUILD		
	  	ASUSEvtlog("[BAT][Bal]resume:%d\n",IsBalanceSuspendStartcharge);
		if( 1==getIfonline() )
			setChgLimitInPadWhenChgReset();
		#endif//ASUS_FACTORY_BUILD
		
	}
	//Hank: set higher hot temperature limit when CPU on+++
	else
#endif		
		if(DffsDown == true && SMB346_INOK_Online && (A90_EVB0==g_ASUS_hwID))
	{
		setSmb346HigherHotTempLimit();
	}
	//Hank: set higher hot temperature limit when CPU on---



        schedule_delayed_work(&_this->ResumeWorker,1*HZ);
        wake_lock_timeout(&_this->resume_wake_lock,2* HZ);
        
        

        if( false == reportRtcReady()){
            queue_delayed_work(_this->BatteryServiceCapUpdateQueue,
                                   &_this->BatRtcReadyWorker,
                                   RTC_READY_DELAY_TIME * HZ);
        }

        printk("[BAT][Ser]:resume()---\n");

}
//Eason resume always calculate capacity no matter if in   Pad or CableIn or BatLow---

static void AXC_BatteryService_forceResume(struct AXI_BatteryServiceFacade *bat,int delayStartInSeconds)
{
    AXC_BatteryService  *_this=
        container_of(bat, AXC_BatteryService, miParent);

    time_t nowForceResumeTime;
    time_t nowForceResumeInterval;
    bool needDoForceResume=false;


//when forceresume default turn off vbus +++
#ifndef ASUS_FACTORY_BUILD		
	IsSystemdraw= false;
	IsBalanceSuspendStartcharge = false;
	//Eason: dynamic set Pad alarm +++
	InSuspendNeedDoPadAlarmHandler=false;
	//Eason: dynamic set Pad alarm ---	
#endif	
//when forceresume default turn off vbus ---

    nowForceResumeTime = updateNowTime(_this);
    nowForceResumeInterval = nowForceResumeTime - _this->savedTime;
    //Hank: Interval overflow handling+++	
    if(nowForceResumeInterval <0)
    {
    		printk("[BAT][SER]%s():Interval overflow set interval RESUME_UPDATE_TIME!\n",__func__);
		nowForceResumeInterval = RESUME_UPDATE_TIME;
    }
    //Hank: Interval overflow handling---

    pr_debug("[BAT][Ser]:forceResume()===:%ld,%ld,%ld\n"
            ,nowForceResumeTime,_this->savedTime,nowForceResumeInterval);

    if( true==_this->IsFirstForceResume ){
            needDoForceResume = true;
	     //Hank: set higher hot temperature limit when first force resume+++
	    if(DffsDown == true &&  SMB346_INOK_Online && (A90_EVB0==g_ASUS_hwID))
	    {
			setSmb346HigherHotTempLimit();
	    }
	    //Hank: set higher hot temperature limit when first force resume---
	    
            //Hank: move the set to BatteryServiceCapSample()+++
            //_this->IsFirstForceResume = false;
            //Hank: move the set to BatteryServiceCapSample()---
    }else if( nowForceResumeInterval >= FORCERESUME_UPDATE_TIME
                            &&false==_this->IsCalculateCapOngoing){
                                            
            needDoForceResume = true;
    }/*else{
            printk("[BAT][Ser]:forceResume queue 5min\n");
            queue_delayed_work(_this->BatteryServiceCapUpdateQueue, 
                               &_this->BatteryServiceUpdateWorker,
                               FORCERESUME_UPDATE_TIME * HZ);    
    }*/
    



    if( true==needDoForceResume )
    {
        printk("[BAT][Ser]:forceResume()+++\n");

    //if(true == _this->IsSuspend){

        //_this->IsSuspend = false;

        _this->IsResumeUpdate = true;
        _this->IsResumeMahUpdate = true;
        _this->P02_IsResumeUpdate = true;
	#ifdef CONFIG_ASUSDEC
        _this->Dock_IsResumeUpdate = true;
	#endif

        if(delayed_work_pending(&_this->BatteryServiceUpdateWorker))
        {
            cancel_delayed_work_sync(&_this->BatteryServiceUpdateWorker);
        }
	 //Hank: cancel BatteryServiceUpdateWorker need calculate capacity+++
	  balance_this->NeedCalCap = true;
	 //Hank: cancel BatteryServiceUpdateWorker need calculate capacity--- 
        pr_debug("[Bat][SER]%s(): queue BatteryServiceUpdateWorker with calculate capacity\n",__func__); 
        queue_delayed_work(_this->BatteryServiceCapUpdateQueue,&_this->BatteryServiceUpdateWorker,\
                               delayStartInSeconds * HZ);
        //ReportTime();

        if( false == reportRtcReady())
	 {
            queue_delayed_work(_this->BatteryServiceCapUpdateQueue,&_this->BatRtcReadyWorker,
                                   RTC_READY_DELAY_TIME * HZ);
        }
    //}

    }
//Eason: A68 new balance mode +++	
#ifndef ASUS_FACTORY_BUILD	
	//Eason: use queue doBalanceMode in less 5min forceResume+++
		/*
			Eason:put judge (false == DecideIfPadDockHaveExtChgAC() in worker Less5MinDoBalanceWorker to save time
			//else if((1==AX_MicroP_IsP01Connected())&&(1 == IsBalanceMode)&&(false == DecideIfPadDockHaveExtChgAC()))
		*/
    #ifdef CONFIG_EEPROM_NUVOTON
    else if((1==AX_MicroP_IsP01Connected())&&(1 == IsBalanceMode))			
    {
		queue_delayed_work(_this->BatteryServiceCapUpdateQueue,&_this->Less5MinDoBalanceWorker, 0*HZ);
	//Eason: use queue doBalanceMode in less 5min forceResume---
    }
    #endif	
#endif
    else
    {//Hank: Temperature monitor only+++
		balance_this->NeedCalCap = false;
		if(delayed_work_pending(&_this->BatteryServiceUpdateWorker))
        	{
            		cancel_delayed_work_sync(&_this->BatteryServiceUpdateWorker);
        	}
		pr_debug("[BAT][SER]%s(): queue BatteryServiceUpdateWorker without calculate capacity\n",__func__); 
		 queue_delayed_work(_this->BatteryServiceCapUpdateQueue, \
                               &_this->BatteryServiceUpdateWorker,\
                               0 * HZ);
    }//Hank: Temperature monitor only---	
    printk("[BAT][SER]:forceResume()---\n");
//Eason: A68 new balance mode ---

}
static void  AXC_BatteryService_constructor(struct AXC_BatteryService *_this,AXI_BatteryServiceFacadeCallback *callback)
{
    _this->callback = callback;

    if(false == _this->mbInit){

        //todo....add internal module creation & init here...
        BatteryService_enable_ChargingFsm(_this);// batteryservice to fsm
        BatteryService_enable_Gauge(_this);// batteryservice to fsm
        BatteryService_enable_Filter(_this);
        BatteryService_InitWoker(_this);
        //ASUS_BSP +++ Eason_Chang BalanceMode
        create_balanceChg_proc_file();
	 //Eason: MPdecisionCurrent +++
    	 create_MPdecisionCurrent_proc_file();
	 //Eason: MPdecisionCurrent ---
	 //Hank enterRomMode_test++++
	create_enterRomMode_proc_file();
	//Hank enterRomMode_test----
	//Hank read BatteryID+++
	#ifdef ASUS_FACTORY_BUILD
	create_BatteryID_proc_file();
	 //Eason: charger_limit_enable is used to decide if Do_Factory5060Mode+++
	charger_limit_enable_create_proc_file();
	//Eason: charger_limit_enable is used to decide if Do_Factory5060Mode---
	#endif
	//Hank read BatteryID---
        balance_this = _this;
        #ifdef CONFIG_EEPROM_NUVOTON
	        register_microp_notifier(&batSer_microp_notifier);
	        notify_register_microp_notifier(&batSer_microp_notifier, "axc_batteryservice"); //ASUS_BSP Lenter+
        #endif /* CONFIG_EEPROM_NUVOTON */
        //ASUS_BSP --- Eason_Chang BalanceMode
	create_HighPower_proc_file();//[ChiaYuan]Add for Broadcast busy state
        mutex_init(&_this->main_lock);
        mutex_init(&_this->filter_lock);
        wake_lock_init(&_this->cap_wake_lock, WAKE_LOCK_SUSPEND, "bat_cap");
        wake_lock_init(&_this->resume_wake_lock, WAKE_LOCK_SUSPEND, "resume_wake"); 
        //Eason set alarm +++
        alarm_init(&bat_alarm, 0, alarm_handler);
        alarm_init(&batLow_alarm, 0, batLowAlarm_handler);
        alarm_init(&cableIn_alarm, 0, cableInAlarm_handler);
        wake_lock_init(&bat_alarm_wake_lock, WAKE_LOCK_SUSPEND, "bat_alarm_wake");
        wake_lock_init(&batLow_alarm_wake_lock, WAKE_LOCK_SUSPEND, "batLow_alarm_wake");
        wake_lock_init(&cableIn_alarm_wake_lock, WAKE_LOCK_SUSPEND, "cableIn_alarm_wake");
        //Eason set alarm ---
        //Eason: wirelessChg workaround+++
        wake_lock_init(&WC_softstart_wake_lock, WAKE_LOCK_SUSPEND, "WC_softstart_wake");
        wake_lock_init(&WC_DCIN300_wake_lock, WAKE_LOCK_SUSPEND, "WC_DCIN300_wake");
        //Eason: wirelessChg workaround---
        //ASUS_BSP  +++ Eason_Chang charger
    	gpCharger = getAsusCharger();
    	gpCharger->Init(gpCharger);
    	gpCharger->RegisterChargerStateChanged(gpCharger, &balance_this->gChargerStateChangeNotifier, _this->chargerType);
        //ASUS_BSP  --- Eason_Chang charger
        _this->mbInit = true;
	
        //Hank in rom mode show "?" : bootUp check Rom mode queue+++
		//if( A80_EVB <= g_A68_hwID)
		//{
		schedule_delayed_work(&balance_this->CheckGaugeRomModeWorker, 20*HZ);
		//}
		//Hank in rom mode show "?" : bootUp check Rom mode queue---
    }
    
}

//ASUS BSP Eason_Chang +++ batteryservice to fsm
static void BatteryServiceFsm_OnChangeChargingCurrent(struct AXI_Charging_FSM_Callback *callback,AXE_Charger_Type chargertype)
{
    AXC_BatteryService  *_this=
        container_of(callback, AXC_BatteryService, fsmCallback);

        pr_debug("[BAT][Ser]%s()\n",__FUNCTION__);
    
    _this->callback->changeChargingCurrent(_this->callback,chargertype);
}

static void BatteryServiceFsm_OnStateChanged(struct AXI_Charging_FSM_Callback *callback)
{   
    AXE_Charging_State GetStateFromFsm;
    bool NeedUpdate = 0;
    AXC_BatteryService  *_this=
        container_of(callback, AXC_BatteryService, fsmCallback);
     
    GetStateFromFsm = _this->fsm->getState(_this->fsm);
    

    switch(_this->fsmState){
        case DISCHARGING_STATE:
             if(GetStateFromFsm == CHARGING_STATE){
                NeedUpdate = 1;
		   //Eason BMS notify +++
		   schedule_delayed_work(&_this->BmsChgBeganWorker, 0*HZ);
		   //Eason BMS notify ---	 
             }
            break;
        case CHARGING_STATE:
#ifdef CONFIG_EEPROM_NUVOTON  //ASUS_BSP Eason_Chang 1120 porting +++            
             //ASUS_BSP +++ Eason_Chang BalanceMode
             if(1==AX_MicroP_IsP01Connected()){                   
                  if(    GetStateFromFsm == DISCHARGING_STATE
                       ||GetStateFromFsm == CHARGING_FULL_STATE
                       ||GetStateFromFsm == CHARGING_STOP_STATE){
                       NeedUpdate = 1;  
			    //Eason BMS notify +++
			    schedule_delayed_work(&_this->BmsChgEndWorker, 0*HZ);
			    //Eason BMS notify ---
                  }     

             }else
             //ASUS_BSP --- Eason_Chang BalanceMode    
#endif //CONFIG_EEPROM_NUVOTON//ASUS_BSP Eason_Chang 1120 porting ---             
             if(GetStateFromFsm == DISCHARGING_STATE || GetStateFromFsm == CHARGING_FULL_STATE ){
                NeedUpdate = 1;
		   //Eason BMS notify +++
		   schedule_delayed_work(&_this->BmsChgEndWorker, 0*HZ);
		   //Eason BMS notify ---
             }
            break;
        case CHARGING_STOP_STATE:
#ifdef CONFIG_EEPROM_NUVOTON  //ASUS_BSP Eason_Chang 1120 porting +++              
            //ASUS_BSP +++ Eason_Chang BalanceMode
            if(1==AX_MicroP_IsP01Connected()){
                 if(     GetStateFromFsm == DISCHARGING_STATE
                       ||GetStateFromFsm == CHARGING_STATE){
                       NeedUpdate = 1;
                }
		   //Eason BMS notify +++
		   if(GetStateFromFsm == CHARGING_STATE)
		   {
		   	schedule_delayed_work(&_this->BmsChgBeganWorker, 0*HZ);
		   }	
		   //Eason BMS notify ---	 	 
            }else 
            //ASUS_BSP --- Eason_Chang BalanceMode  
#endif //CONFIG_EEPROM_NUVOTON//ASUS_BSP Eason_Chang 1120 porting ---             
            if(GetStateFromFsm == DISCHARGING_STATE ){
                NeedUpdate = 1;
		   //Eason BMS notify +++
		   schedule_delayed_work(&_this->BmsChgEndWorker, 0*HZ);
		   //Eason BMS notify ---
            }   
            break;
        case CHARGING_FULL_STATE:
            if(GetStateFromFsm == DISCHARGING_STATE ){
                NeedUpdate = 1;
		   //Eason BMS notify +++
		   schedule_delayed_work(&_this->BmsChgEndWorker, 0*HZ);
		   //Eason BMS notify ---				
            } 
            break;
        case CHARGING_FULL_KEEP_STATE:
            if(GetStateFromFsm == DISCHARGING_STATE ){
                NeedUpdate = 1;
		   //Eason BMS notify +++
		   schedule_delayed_work(&_this->BmsChgEndWorker, 0*HZ);
		   //Eason BMS notify ---
            }    
            break;
        case CHARGING_FULL_KEEP_STOP_STATE:
            if(GetStateFromFsm == DISCHARGING_STATE ){
                NeedUpdate = 1;
		   //Eason BMS notify +++
		   schedule_delayed_work(&_this->BmsChgEndWorker, 0*HZ);
		   //Eason BMS notify ---
            }
            break;
        default:
            printk("[BAT][ser]%s():NOT mapping\n",__FUNCTION__);
            break;
            
    }

    _this->fsmState = GetStateFromFsm;

    if( 1 == NeedUpdate){
    _this->callback->onServiceStatusUpdated(_this->callback);
    }
    
}

//ASUS BSP Eason_Chang --- batteryservice to fsm
//ASUS BSP Eason_Chang +++ batteryservice to gauge
/*
static inline int AXC_BatteryService_getNextPollingInterval(struct AXC_BatteryService *_this)
{

    if(_this->test.ifFixedPollingInterval(&_this->test)){

        return _this->test.pollingInterval;

    }else{

        return _this->gauge->getNextPollingInterval(_this->gauge);
    }

}
*/

static inline void  AXC_BatteryService_scheduleNextPolling(struct AXC_BatteryService *_this)
{
    pr_debug("[Bat][SER]%s(): queue BatteryServiceUpdateWorker without calculate capacity\n",__func__);
    balance_this->NeedCalCap = false; 
    queue_delayed_work(_this->BatteryServiceCapUpdateQueue,
                            &_this->BatteryServiceUpdateWorker,
                            AXC_BatteryService_getNextPollingInterval(_this)* HZ);
}

static bool Get_CapRiseWhenCableIn(int nowCap, int lastCap)
{
	//Eason: Cap<10 with cable, but Cap decrease. Let Cap drop, ignore rule of lastCap - nowCap>=5 +++
	if( (lastCap - nowCap > 0)&&(balance_this->A66_capacity<10) )
	{
		printk("[BAT][SER]%s():CapDropWhenCableInCapLessThan10\n",__func__);
		return false;
	}	
	//Eason: Cap<10 with cable, but Cap decrease. Let Cap drop, ignore rule of lastCap - nowCap>=5 ---
	else if(lastCap - nowCap >= 5)
	{
		printk("[BAT][SER]%s():CapDropWhenCableIn\n",__func__);
		return false; 
	}
	else
	{
		return true;
	}
}

//ASUS_BSP Eason: fix PM8941 eoc_work change status full, but cap is 99% +++
/* When PM8941 eoc_work change status full, if don't notify BatteryService,
    query capacity with function AXC_BatteryService_getCapacity at this time will show 99% full
    because  flag balance_this->BatteryService_IsFULL does not change to true.
*/
void notify_batteryService_statusFull(void)
{
	if(false == balance_this->BatteryService_IsFULL)
	{
		printk("[BAT]%s\n",__FUNCTION__);
		balance_this->BatteryService_IsFULL = true;
		//Eason: wirelessChg workaround+++
		judge_Cap100_WC_DCIN300();
		//Eason: wirelessChg workaround---
	}
}
//ASUS_BSP Eason: fix PM8941 eoc_work change status full, but cap is 99% ---

static void DoAfterDecideFull(struct AXC_BatteryService *_this)
{
    if(false == _this->BatteryService_IsFULL)
    {
        
        _this->BatteryService_IsFULL = true;
        _this->fsm->onChargingStop(_this->fsm,CHARGING_DONE);
	//Hank: A86 no use+++
	if(g_ASUS_hwID != A90_EVB0 )	
	        _this->gauge->notifyBatFullChanged(_this->gauge,true);
	//Hank: A86 no use---

    }

}

static void DoAfterDecideNotFull(struct AXC_BatteryService *_this)
{
    if (true==_this->BatteryService_IsFULL)
    {
    	     //Hank: A86 no use+++
	     if(g_ASUS_hwID != A90_EVB0 )	
	            _this->gauge->notifyBatFullChanged(_this->gauge,false);
	     //Hank: A86 no use--- 	 
            _this->fsm->onFullDrop(_this->fsm);
    }

     _this->BatteryService_IsFULL = false;
}
static void DecideIsFull(struct AXC_BatteryService *_this,int nowGaugeCap,bool hasCableInPastTime)
{
    bool chgStatus;
    bool IsPadDock_ExtChg = false;
    int nCurrent = _this->callback->getIBAT(_this->callback);

    chgStatus = gpCharger->IsCharging(gpCharger);

    printk("[BAT][SER]%s():Charging Status:%d, Battery Current:%d\n",__func__,chgStatus,nCurrent);

    if(chgStatus)//charging
    {
		if(!_this->isMainBatteryChargingDone)// if still charging....
		{
	            if(nowGaugeCap >= 94 && 0 >= nCurrent && -90 < nCurrent && !delayed_work_pending(&_this->BatEocWorker))
		     {
	                printk("[BAT][Ser]start eoc worker\n");
	                queue_delayed_work(_this->BatteryServiceCapUpdateQueue, &_this->BatEocWorker,10 * HZ);
	            }    
	       }
		else
		{ 
	            chgStatus = false;    
	       }
     }
     else//no charging
     {
            if( 0 >= nCurrent && -90 < nCurrent)
	     {
                    chgStatus = false;
            }
	     else if( 100 == balance_this->A66_capacity)
	     {
                    chgStatus = false;
                    printk("[BAT][Ser]:when cap100 don't judge current\n");
                    //Eason : when resume by take off cable can be judge full 
            }
	     else
	     {
                    chgStatus = true;
                    printk("[BAT][Ser]:chg current not in 0~-90, can't judge Full\n");
            }
     }

    //Eason for resume by EXTchg off can be full+++
    if(100 == balance_this->A66_capacity)
    {
        IsPadDock_ExtChg = true;
    }
    else
    {    
        IsPadDock_ExtChg = DecideIfPadDockHaveExtChgAC();
    }
    //Eason for resume by EXTchg off can be full---
    
    #ifdef CONFIG_EEPROM_NUVOTON  //ASUS_BSP Eason_Chang 1120 porting +++
    if(1==AX_MicroP_IsP01Connected())
    {
               if(CHARGING_FULL_STATE==balance_this->fsmState||CHARGING_FULL_KEEP_STATE==balance_this->fsmState
                    || CHARGING_FULL_KEEP_STOP_STATE==balance_this->fsmState)
               {
                     DoAfterDecideFull(_this);  
               }
	        else if((g_ASUS_hwID == A90_EVB0)&&(true==get_FC_flage_from_TIgauge()))
		 {
                     DoAfterDecideFull(_this);
               }
#ifdef CONFIG_PM_8941_CHARGER
		  else if( (true==pm8941_is_full())&&(g_ASUS_hwID != A90_EVB0) )
		  {
			  printk("[BAT][Ser]pm8941_is_full\n");
			//Eason : Wireless PMA spec Rx turn off Tx +++
			#ifdef CONFIG_IDTP9023_CHARGER
			g_WcEocTime = updateNowTime(_this);
			#endif
			//Eason : Wireless PMA spec Rx turn off Tx ---
			  DoAfterDecideFull(_this);
		  }
#endif
//Eason: check FC flag to disable charge, prevent Battery over charge, let gauge FCC abnormal +++
//- new termianl current is 50mA
/*
		 else if(true==hasCableInPastTime && nowGaugeCap >= 94 
                    && false==chgStatus && true==IsPadDock_ExtChg)//why pad need cable?
               {
                     DoAfterDecideFull(_this);
               }
*/
//Eason: check FC flag to disable charge, prevent Battery over charge, let gauge FCC abnormal---
		 else if((100 == balance_this->A66_capacity)&&(100==nowGaugeCap)&&(g_ASUS_hwID==A90_EVB0))
               {
                     DoAfterDecideFull(_this);
               }
		 else
		 {
                     DoAfterDecideNotFull(_this);
               } 
    }
    else
    #endif //CONFIG_EEPROM_NUVOTON//ASUS_BSP Eason_Chang 1120 porting ---    
    {    
               if(CHARGING_FULL_STATE==balance_this->fsmState || CHARGING_FULL_KEEP_STATE==balance_this->fsmState
                    || CHARGING_FULL_KEEP_STOP_STATE==balance_this->fsmState)
               {
                     DoAfterDecideFull(_this);
               }
	        else if((g_ASUS_hwID == A90_EVB0) && (true==get_FC_flage_from_TIgauge()) )
               {
                     DoAfterDecideFull(_this);
               }
#ifdef CONFIG_PM_8941_CHARGER
		  else if( (true==pm8941_is_full())&&(g_ASUS_hwID != A90_EVB0) )
		  {
			  printk("[BAT][Ser]pm8941_is_full\n");
			//Eason : Wireless PMA spec Rx turn off Tx +++
			#ifdef CONFIG_IDTP9023_CHARGER
			g_WcEocTime = updateNowTime(_this);
			#endif
			//Eason : Wireless PMA spec Rx turn off Tx ---
			  DoAfterDecideFull(_this);
		  }
#endif			
//Eason: check FC flag to disable charge, prevent Battery over charge, let gauge FCC abnormal +++
//- new termianl current is 50mA
/*
		 else if(true==hasCableInPastTime && nowGaugeCap >= 94 
                    && false==chgStatus )
               {
                     DoAfterDecideFull(_this);
               }
*/
//Eason: check FC flag to disable charge, prevent Battery over charge, let gauge FCC abnormal---
		 else if((100 == balance_this->A66_capacity)&&(100==nowGaugeCap)&&(g_ASUS_hwID==A90_EVB0))
               {
                     DoAfterDecideFull(_this);                     
               }
		 else
		 {
                     DoAfterDecideNotFull(_this);
               } 
    }           

} 

bool report_BatteryService_If_HasCable(void)
{
    bool hasCable = false;

    if (true == balance_this->IsFirstAskCable)
    {
    	if(1==getIfonline())
            {
                printk("[BAT][Ser]FirstAskCable report true\n");
                hasCable = true;
            }
    
    }else if (true == balance_this->IsResumeUpdate){
        hasCable = balance_this->HasCableBeforeSuspend;

    }else{
        hasCable = balance_this->BatteryService_IsCable; 
    }

    return hasCable;
}

//Eason: when change MaxMah clear interval+++
static time_t BatteryService_getForceIntervalSinceLastUpdate(AXC_BatteryService  *_this)
{
    struct timespec mtNow;
    
    time_t intervalSinceLastUpdate;
    
    mtNow = current_kernel_time();

    if(mtNow.tv_sec >= _this->ForceSavedTime)
    {

	        pr_debug("[BAT][Ser]%s:%ld,%ld\n",__FUNCTION__,mtNow.tv_sec,_this->ForceSavedTime);
	            
	        intervalSinceLastUpdate = mtNow.tv_sec - _this->ForceSavedTime;

	        if(intervalSinceLastUpdate > 2592000)
		 {
	            printk("[BAT][SER]%s():wrongInt %ld \n",__func__,intervalSinceLastUpdate);
	            intervalSinceLastUpdate = 180;
	        }    
    }
    else
    {    
	        printk("[BAT][Ser]%s:OVERFLOW....%ld,%ld\n",__FUNCTION__,mtNow.tv_sec,_this->ForceSavedTime);              
	        //todo: to do the correct calculation here....
	        intervalSinceLastUpdate = 180;
    }
    return intervalSinceLastUpdate ; 
}
//Eason: when change MaxMah clear interval---

//ASUS_BSP Eason:when shutdown device set smb346 charger to DisOTG mode +++
extern void UsbSetOtgSwitch(bool switchOtg);
static void set_DisOTGmode_whenCap_0(void)
{
	printk("[BAT]Cap 0 DisOTG+++\n ");
	UsbSetOtgSwitch(false);
	printk("[BAT]Cap 0 DisOTG---\n ");
}
//ASUS_BSP Eason:when shutdown device set smb346 charger to DisOTG mode ---

//ASUS BSP Eason_Chang get Cap from TIgauge+++
int get_Cap_from_TIgauge(void);
//ASUS BSP Eason_Chang get Cap from TIgauge---


//Eason show temp limit +++
extern int showSmb346TempLimitReason(void);
//Eason show temp limit ---
//Eason show AICLsetting & AICLresult+++
extern int showSmb346AICL_Setting(void);
extern int showSmb346AICL_Result(void);
//Eason show AICLsetting & AICLresult---
//check df version do Cap remapping+++
extern int g_gauge_df_version;
#define DF_VERSION_NOT_NEED_REMAP  0
#define DF_VERSION_REMAP_START        1
#define DF_VERSION_REMAP_STOP         10
#define NO_RESERVE_DF_MAP_NUM	100
static int noReserveDF_map_tbl[NO_RESERVE_DF_MAP_NUM + 1] = 
							   {  0,   0,  0,   0,  0,   0,   1,   2,  3,  4,
							       5,   6,  7,   8,  9, 10, 11, 12, 13, 14, 
							     15, 16, 17, 18, 19, 20, 21, 22, 23, 24,
							     25, 27, 28, 29, 30, 31, 32, 33, 34, 35,
							     36, 37, 38, 39, 40, 41, 43, 44, 45, 46,
							     47, 48, 49, 50, 51, 52, 53, 55, 56, 57,
							     58, 59, 60, 61, 62, 64, 65, 66, 67, 68,
							     69, 70, 71, 72, 73, 74, 75, 76, 77, 78,
							     80, 81, 82, 83, 84, 85, 86, 87, 88, 89,
							     90, 91, 92, 93, 94, 95, 96, 97, 98, 99,
							    100 };
//check df version do Cap remapping---
//Eason print temp info+++
extern long long gPcbTemp;
//Eason print temp info---


//ASUS_BSP Eason read PM8941 register value+++
extern u32 asus_pmic_arb_read(u32 offset);
//ASUS_BSP Eason read PM8941 register value---
extern int get_Volt_from_TIgauge(void);
extern int get_FCC_from_TIgauge(void);
static void AXC_BatteryService_reportPropertyCapacity(struct AXC_BatteryService *_this, int refcapacity)
{
    int A66_capacity;
    
    //int EC_capacity;
    int IsBalTest;//ASUS_BSP Eason_Chang BalanceMode

    int lastCapacity;
    int BatteryVolt;
    int BatteryFCC;	
    int maxMah;
    int pmicTemp;
    int pmicTemp_float;
    int WirelessChg;
    int Battery_volt;
    //ASUS_BSP Eason read PM8941 register value+++
    u32 pm8941_0x1444_value;
    u32 pm8941_0x105B_value;
    u32 pm8941_0x1040_value;
    u32 pm8941_0x1054_value;
    u32 pm8941_0x1049_value;
    u32 pm8941_0x1344_value;
    u32 pm8941_0x1010_value;
    u32 pm8941_0x1210_value;
    u32 pm8941_0x1044_value;
    u32 pm8941_0x105D_value;
   //ASUS_BSP Eason read PM8941 register value---

    bool hasCable = false;
    bool EnableBATLifeRise;
    bool maxMahDrop = false;
    //Eason boot up in BatLow situation, take off cable can shutdown+++
    bool IsBatLowtoFilter ;
    //Eason boot up in BatLow situation, take off cable can shutdown---

    time_t intervalSinceLastUpdate;
	
    //check df version do Cap remapping+++
    int reMap_refcapacity;
    if (g_ASUS_hwID >= A90_EVB0)
    {
		
		if( DF_VERSION_NOT_NEED_REMAP == g_gauge_df_version )
		{
				reMap_refcapacity = refcapacity;
		}else{
				reMap_refcapacity = noReserveDF_map_tbl[refcapacity];
				printk("[BAT][DFmap]:%d, %d\n",refcapacity ,reMap_refcapacity);
		}
		//reMap_refcapacity = noReserveDF_map_tbl[refcapacity];
		//printk("[BAT][SER][DFmap]:%d, %d\n",refcapacity ,reMap_refcapacity);
      }
      else
      {
		reMap_refcapacity = refcapacity;
       }
     //check df version do Cap remapping---	

    	mutex_lock(&_this->filter_lock);

    	intervalSinceLastUpdate  = BatteryService_getIntervalSinceLastUpdate(_this);
    

      //We need do ask capcaity to filter at first time, in case there is FULL orBATLow+++ 
     if(true == _this->IsFirstAskCap)
     {
       	 lastCapacity = reMap_refcapacity;
        	_this->IsFirstAskCap = false;
        
      }
      else
      {
      		//Eason show BMS cap to user+++
      		if(A90_EVB0==g_ASUS_hwID)
		{
			lastCapacity = _this->A66_capacity;
		}else{
			lastCapacity = g_SWgauge_lastCapacity;
		}
		//Eason show BMS cap to user---
      }
      //We need do ask capcaity to filter at first time, in case there is FULL orBATLow---

      //Initiatively check cable when power on+++
      if (true == _this->IsFirstAskCable)
      {
      		if(1==getIfonline())
              {
                	printk("[BAT][Ser]FirstAskCable true\n");
                	hasCable = true;
              }
        	_this->IsResumeUpdate = false;
        	_this->IsFirstAskCable = false;
        
      }
      //Initiatively check cable when power on---
      //If resume update capacity, check cable before suspend+++
      else if (true == _this->IsResumeUpdate)
      {
       	hasCable = _this->HasCableBeforeSuspend;
        	_this->IsResumeUpdate = false;
      }
      //If resume update capacity, check cable before suspend---
      //Default check cable +++
      else
      {
         	hasCable = _this->BatteryService_IsCable; 
      }
      //Default check cable ---
      
      _this->BatteryService_IsCharging = gpCharger->IsCharging(gpCharger);

      DecideIsFull(_this,reMap_refcapacity,hasCable);

      //Eason: BAT Cap can drop when cable in +++ 
      if( true == hasCable )//A66 has cable  
      {
	     #ifdef CONFIG_EEPROM_NUVOTON  //ASUS_BSP Eason_Chang 1120 porting +++    
            if(1==AX_MicroP_IsP01Connected())//cable of A66 is Pad
	     {
                    if( true == _this->BatteryService_IsCharging)//A66 is charging 
		      {
                            EnableBATLifeRise = Get_CapRiseWhenCableIn(reMap_refcapacity, lastCapacity);
                            if( false == EnableBATLifeRise)
				{
                                    maxMahDrop = true;
                            }
                    }
		      else if(true == _this->BatteryService_IsFULL)//A66 is not charging but full
		      {
                            //Eason for resume by EXTchg off can be full, after full can drop
                            EnableBATLifeRise = Get_CapRiseWhenCableIn(reMap_refcapacity, lastCapacity);
                            if( false == EnableBATLifeRise)
				{
                                    maxMahDrop = true;
                            }
                    }
		      else if(true == DecideIfPadDockHaveExtChgAC())//A66 have ext chg 
                    {
                            EnableBATLifeRise = Get_CapRiseWhenCableIn(reMap_refcapacity, lastCapacity);
                            if( false == EnableBATLifeRise)
				{
                                    maxMahDrop = true;
                            }                    
                    }
		      else//A66 is neither charging  nor full
                    {
                            EnableBATLifeRise = false;
                    }
            }
	     else//cable of A66 is not Pad
	     #endif //CONFIG_EEPROM_NUVOTON//ASUS_BSP Eason_Chang 1120 porting ---            
            {

                    EnableBATLifeRise = Get_CapRiseWhenCableIn(reMap_refcapacity, lastCapacity);
                    if( false == EnableBATLifeRise)
		      {
                            maxMahDrop = true;
                    }
            }      
    }
    else//A66 doesn't has cable 
    {
      		EnableBATLifeRise = hasCable;
    }

    maxMah = BatteryService_ChooseMaxMah(_this,maxMahDrop);
    //Eason : if last time is 10mA +++
    if(true==IfUpdateSavedTime)//only do when last time is 10mA
    {
		intervalSinceLastUpdate = BatteryService_getForceIntervalSinceLastUpdate(_this);//Eason: when change MaxMah clear interval
		IfUpdateSavedTime = false;
     }
     //Eason : if last time is 10mA ---	
     //Eason: BAT Cap can drop when cable in --- 

     //Eason boot up in BatLow situation, take off cable can shutdown+++
     if(true==g_BootUp_IsBatLow )
     {
	    if(1==getIfonline())
	    {
	        IsBatLowtoFilter = false;	
	        printk("[BAT][BootUp]BootUp IsBatLow, Cable on, BatLow false\n");
	    }
	    else
	    {
	        IsBatLowtoFilter = g_BootUp_IsBatLow;
	        printk("[BAT][BootUp]BootUp IsBatLow, Cable off, BatLow true\n");
	    }
     }
     else
     {
	     IsBatLowtoFilter = _this->BatteryService_IsBatLow;
     }
     //Eason boot up in BatLow situation, take off cable can shutdown---    
    //Hank: A86 no use+++	 
   if(g_ASUS_hwID != A90_EVB0 ) //CONFIG_BMS_ASUS
	   //Eason: remember last BMS Cap to filter+++
	   gDiff_BMS = lastCapacity - gBMS_Cap ;//for discharge drop
	   //Eason: remember last BMS Cap to filter---
   //Hank: A86 no use---

   //Hank:A86 slowly drop+++
   if(g_ASUS_hwID == A90_EVB0)
		gCurr_TIgauge = get_Curr_from_TIgauge();
   else
   		gCurr_TIgauge = get_current_for_ASUSswgauge();
   //Hank:A86 slowly drop---

   A66_capacity = _this->gpCapFilterA66->filterCapacity
                                    (_this->gpCapFilterA66,
                                      reMap_refcapacity, 
                                      lastCapacity,
                                      EnableBATLifeRise,
                                      _this->BatteryService_IsCharging,
                                      _this->BatteryService_IsFULL,
                                      IsBatLowtoFilter,
                                      maxMah,
                                      intervalSinceLastUpdate);

    //Eason add to check full & 100%+++
#ifdef CONFIG_PM_8941_CHARGER    
	//do DoAfterDecideNotFull, show charging status, prevent show Full but not 100%
	 if( (true==_this->BatteryService_IsFULL) && (100!=pm8941_getCapacity()) )
	{
		printk("[BAT][Ser]Full but not 100 ,restart charging status\n");
		DoAfterDecideNotFull(_this);
	}
#else 

    if( (true==_this->BatteryService_IsFULL && A66_capacity != 100) )
    {
        printk("[BAT][Ser]Full but not 100 ,restart charging\n");
        DoAfterDecideNotFull(_this);
    }

#endif
    //Eason add to check full & 100%---


    pr_debug("[BAT][Ser]report Capacity:%d,%d,%d,%d,%d,%d,%d,%d,%ld==>%d\n",
                                    reMap_refcapacity,
                                    lastCapacity,
                                      hasCable,
                                      EnableBATLifeRise,
                                      _this->BatteryService_IsCharging,
                                      _this->BatteryService_IsFULL,
                                      IsBatLowtoFilter,
                                      maxMah,
                                      intervalSinceLastUpdate,
                                      A66_capacity);
     //ASUS_BSP +++ Eason_Chang add event log +++
     if(g_ASUS_hwID == A90_EVB0)
     {
     		BatteryVolt = get_Volt_from_TIgauge();
		BatteryFCC = get_FCC_from_TIgauge();
		ASUSEvtlog("[BAT][Ser]report Capacity:%d,%d,%d,%d,%d,%d,%d,%d,%d,%ld==>%d,  FCC:%d  GaugeCur:%d,   Volt:%d, TempLimit:%d, AICL:%d,%d, DF:%d, BatTemp:%d  PcbTemp:%lld CC-Cur:0x%2X, Flot-Volt:0x%2X, GPIO85:%d\n",
							    refcapacity,
							    reMap_refcapacity,
		                                    lastCapacity,
		                                      hasCable,
		                                      EnableBATLifeRise,
		                                      _this->BatteryService_IsCharging,
		                                      _this->BatteryService_IsFULL,
		                                      IsBatLowtoFilter,
		                                      maxMah,
		                                      intervalSinceLastUpdate,
		                                      A66_capacity,
		                                      BatteryFCC,
							   gCurr_TIgauge,
							   BatteryVolt,
		                                      showSmb346TempLimitReason(),
		                                      showSmb346AICL_Setting(),
		                                      showSmb346AICL_Result(),
		                                      g_gauge_df_version,
		                                      gBatteryTemp/10,
		                                      gPcbTemp,
		                                      reg00_value,
		                                      reg03_value,
		                                      EnChg_GPIO85
		                                      );
		//[ChiaYuan][+++]Add for Broadcast busy state
		if (pm_autosleep_state()==PM_SUSPEND_MEM)
		{
			if (gCurr_TIgauge > 20) {
				if (bTimerEnable == false) {
					bTimerEnable = true ;
					mod_timer(&HighPower_timer, jiffies + msecs_to_jiffies(HIGH_POWER_TIMEOUT));
					printk(KERN_INFO "[Power]Enable HighPower Timer\r\n");
				}
				Total_BatCur = Total_BatCur + gCurr_TIgauge;
				HighPower_Count = HighPower_Count + 1;
			}
			else {
				if (bTimerEnable == true) {
					bTimerEnable = false ;
					del_timer(&HighPower_timer);
					printk(KERN_INFO "[Power]Disable HighPower Timer\r\n");
				}
				Total_BatCur=0;
				HighPower_Count=0;
				IsHighPower=0;
			}
		}
		//[ChiaYuan][---]Add for Broadcast busy state
	}
	else
	{
		//Hank: A86 no use+++	
		//ASUS_BSP Eason read PM8941 register value+++
		pm8941_0x1444_value = asus_pmic_arb_read(0x1444);
		pm8941_0x105B_value = asus_pmic_arb_read(0x105B);
		pm8941_0x1040_value = asus_pmic_arb_read(0x1040);
		pm8941_0x1054_value = asus_pmic_arb_read(0x1054);
		pm8941_0x1049_value = asus_pmic_arb_read(0x1049);
		pm8941_0x1344_value = asus_pmic_arb_read(0x1344);
		pm8941_0x1010_value = asus_pmic_arb_read(0x1010);
		pm8941_0x1210_value = asus_pmic_arb_read(0x1210);
		pm8941_0x1044_value = asus_pmic_arb_read(0x1044);
		pm8941_0x105D_value = asus_pmic_arb_read(0x105D);
		//ASUS_BSP Eason read PM8941 register value---	
		WirelessChg = gpio_get_value(GPIO_WC_PD_DET);
 		pmicTemp = pm8941_get_prop_batt_temp();
		report_temp = pmicTemp;
		pmicTemp_float = pmicTemp % 10;
		pmicTemp = pmicTemp / 10;
		Battery_volt = pm8941_get_prop_battery_voltage_now();
		if ((Battery_volt%1000) >= 500)
			Battery_volt = Battery_volt/1000 + 1;
		else
			Battery_volt = Battery_volt/1000;
		BatteryFCC = get_vm_bms_fcc();
		BatteryFCC = BatteryFCC / 1000;
     		ASUSEvtlog("[BAT][Ser]report Capacity ==>%d, FCC:%dmAh, BMS:%d, V:%dmV, Cur:%dmA, Temp:%d.%dC, %d,%d,%d,%d,%d,%d,%d,%d,%ld, diffBMS:%d, WC:%d, 0x1444:0x%x, 0x105B:0x%x, 0x1040:0x%x, 0x1054:0x%x, 0x1049:0x%x, 0x1344:0x%x, 0x1010:0x%x, 0x1210:0x%x, 0x1044:0x%x, 0x105D:0x%x\n",
                                      A66_capacity,
                                      BatteryFCC,
                                      gBMS_Cap,
                                      Battery_volt,
                                      gCurr_TIgauge,
                                      pmicTemp,
                                      pmicTemp_float,
						reMap_refcapacity,
                                      lastCapacity,
                                      hasCable,
                                      EnableBATLifeRise,
                                      _this->BatteryService_IsCharging,
                                      _this->BatteryService_IsFULL,
                                      IsBatLowtoFilter,
                                      maxMah,
                                      intervalSinceLastUpdate,
                                      gDiff_BMS,
                                      WirelessChg,
                                      pm8941_0x1444_value,
                                      pm8941_0x105B_value,
                                      pm8941_0x1040_value,
                                      pm8941_0x1054_value,
                                      pm8941_0x1049_value,
                                      pm8941_0x1344_value,
                                      pm8941_0x1010_value,
                                      pm8941_0x1210_value,
                                      pm8941_0x1044_value,
                                      pm8941_0x105D_value);
		//Hank: A86 no use ---

	}
	//ASUS_BSP --- Eason_Chang add event log ---   

	//Eason: remember last BMS Cap to filter+++
	//	last_BMS_Cap = gBMS_Cap;
	//Eason: remember last BMS Cap to filter---

	//ASUS_BSP +++ Eason_Chang BalanceMode : set A66_cap for cmd test 
       IsBalTest = IsBalanceTest();
       if( 1 == IsBalTest)
	{
        	A66_capacity = GetBalanceModeA66CAP();
              printk("[BAT][Bal][test]A66 cap: %d\n",A66_capacity );
        }
	//ASUS_BSP --- Eason_Chang BalanceMode : set A66_cap for cmd test 

       if(A66_capacity < 0 || A66_capacity >100)
       {
        	printk("[BAT][Ser]Filter return value fail!!!\n");
       }
	else 
	{
		//Eason show BMS cap to user+++
		if( (A90_EVB0==g_ASUS_hwID)||( 1 == IsBalTest) )
		{
			_this->A66_capacity = A66_capacity;
		}else{
			_this->A66_capacity = gBMS_Cap;
			g_SWgauge_lastCapacity = A66_capacity;
		}
		//Eason show BMS cap to user---

		//ASUS_BSP Eason_Chang 1120 porting +++
		#ifdef CONFIG_EEPROM_NUVOTON  
	       if(1==AX_MicroP_IsP01Connected())
		{
	              if( false == DecideIfPadDockHaveExtChgAC())
			{ 
	                    BatteryServiceDoBalance(_this);
	              }
			else
			{
	                    Init_Microp_Vbus__Chg();
	              }

			//Eason: dynamic set Pad alarm +++
			#ifndef ASUS_FACTORY_BUILD	  	
			 SetRTCAlarm();
			#endif
			//Eason: dynamic set Pad alarm ----
       	}
		#endif //CONFIG_EEPROM_NUVOTON   
		//ASUS_BSP Eason_Chang 1120 porting ---
		
       	// when A66 Cap = 0% shutdown device no matter if has cable+++ 
       	//if( 0==_this->A66_capacity )//Eason show BMS cap to user
       	if( ( 0==_this->A66_capacity )&&(A90_EVB0==g_ASUS_hwID) )
       	{
          		g_AcUsbOnline_Change0 = true;
			//ASUS_BSP Eason:when shutdown device set smb346 charger to DisOTG mode +++
			set_DisOTGmode_whenCap_0();
			//ASUS_BSP Eason:when shutdown device set smb346 charger to DisOTG mode ---
			//ASUS_BSP +++ Peter_lu "suspend for Battery0% in  fastboot mode issue"
#ifdef CONFIG_FASTBOOT
			if(is_fastboot_enable()){
					printk("[BAT][Fastboot]kernel_power_off\n");
					kernel_power_off();
			}else{
#endif //#ifdef CONFIG_FASTBOOT
			//ASUS_BSP --- Peter_lu
	          		AcUsbPowerSupplyChange();

				#ifdef CONFIG_ASUSDEC  
	          		Dock_AC_PowerSupplyChange();
				#endif

				#ifdef CONFIG_EEPROM_NUVOTON
		   		Pad_AC_PowerSupplyChange();
				#endif
			//ASUS_BSP +++ Peter_lu "suspend for Battery0% in  fastboot mode issue"
#ifdef CONFIG_FASTBOOT
			}
#endif //#ifdef CONFIG_FASTBOOT
			//ASUS_BSP --- Peter_lu
       	} //end if ( 0==_this->A66_capacity )
       	// when A66 Cap = 0% shutdown device no matter if has cable---

		//Eason : prevent thermal too hot, limit charging current in phone call+++
              judgePhoneOnCurLimit();
              judgeThermalCurrentLimit(); //when thermal too hot limit charging current
       	//Eason : prevent thermal too hot, limit charging current in phone call---
       	//Eason : Wireless PMA spec Rx turn off Tx +++
       	#ifdef CONFIG_IDTP9023_CHARGER
		judgeWirelessCoilTempTurnOffTx();
		judgeWirelessEocTurnOffTx();
		#endif
		//Eason : Wireless PMA spec Rx turn off Tx ---
		//ASUS_BSP Eason_Chang:add WirelessChg soft start+++
		judge_Cap100_WC_DCIN300();
		//ASUS_BSP Eason_Chang:add WirelessChg soft start---
		//ASUS_BSP Eason_Chang: wireless mode (1)Cap>=80% &(2)Temp>45degC set VDD_MAX(0x1040) low +++
		judgeWirelessLow_VDD_MAX(pmicTemp,gBMS_Cap);
		//ASUS_BSP Eason_Chang: wireless mode (1)Cap>=80% &(2)Temp>45degC set VDD_MAX(0x1040) low ---
		
		//Eason: LowCapCpuThrottle +++
       	if(g_ASUS_hwID >= A90_EVB0)
   		{
			judgeCpuThrottleByCap();
   		}
		//Eason: LowCapCpuThrottle ---

		#ifdef ASUS_FACTORY_BUILD
       	 if(1==getIfonline() )
        	{
        		//Eason: take off 5060rule let A80 can always charge+++
			if(true == charger_limit_enable)
            			Do_Factory5060Mode();
			//Eason: take off 5060rule let A80 can always charge---
        	}
		#endif//#ifdef ASUS_FACTORY_BUILD

		_this->callback->onServiceStatusUpdated(_this->callback);
    }   
    
    _this->IsCalculateCapOngoing = false;

     //Eason: AICL work around +++
     g_alreadyCalFirstCap = true;
     //Eason: AICL work around ---	

	//ASUS BSP Deeo : LED charger mode +++
	if( _this->BatteryService_IsFULL == true && _this->A66_capacity ==100 )
		led_set_charger_mode(2);
	//ASUS BSP Deeo : LED charger mode ---
    mutex_unlock(&_this->filter_lock);

}
static int BatteryServiceGauge_OnCapacityReply(struct AXI_Gauge *gauge, struct AXI_Gauge_Callback *gaugeCb, int batCap, int result)
{   

    AXC_BatteryService  *_this=
        container_of(gaugeCb, AXC_BatteryService, gaugeCallback);

     //Hank: A86 no use+++	 	
    //ASUS BSP: Eason check correct BMS RUC+++
    int BMSCap;
    //int BMS_diff_SWgauge;
    //ASUS BSP: Eason check correct BMS RUC---
    //Hank: A86 no use---

    //Eason : In suspend have same cap don't update savedTime +++
    int A66_LastTime_capacity;
    A66_LastTime_capacity = _this->A66_capacity;
    //Eason : In suspend have same cap don't update savedTime ---

    pr_debug("[BAT][SER]%s() +++ \n",__func__);		
    mutex_lock(&_this->main_lock);

      //Hank: A86 no use+++	 
    //Eason get BMS Capacity for EventLog+++
    BMSCap= get_BMS_capacity();
    gBMS_Cap =  BMSCap;
    //Eason get BMS Capacity for EventLog---
    //Hank: A86 no use---
    
	//ASUS BSP Eason_Chang get Cap from TIgauge+++
	if((BAT_CAP_REPLY_ERR==result)&&(g_ASUS_hwID !=A90_EVB0))
	{
	//ASUS BSP Eason_Chang get Cap from TIgauge---
            _this->IsResumeUpdate = false;
            _this->IsResumeMahUpdate = false;
            pr_err("[A66][BAT][Ser]:Error askCapacity\n");
    	}
	else
    	{
        	//Eason:TIgauge through filter prevent 0% while not BatLow+++ 
	  	if(g_ASUS_hwID == A90_EVB0)
	  	{
			AXC_BatteryService_reportPropertyCapacity( _this,get_Cap_from_TIgauge());
	  	}
		//Eason:TIgauge through filter prevent 0% while not BatLow---
		//Eason: choose Capacity type SWGauge/BMS +++ 
		else if(g_CapType == DEFAULT_CAP_TYPE_VALUE)
		{
			pr_debug("[BAT][Ser]:Cap type SWgauge\n");	
			AXC_BatteryService_reportPropertyCapacity(
		             _this,
		             batCap);

		}
		else
		{	
			//Hank: A86 no use+++
			#ifdef CONFIG_SWGAU_ASUS
        		//Eason get BMS Capacity for EventLog+++
        		#if 0
        		BMSCap= get_BMS_capacity();//ASUS BSP: Eason check correct BMS RUC
			#endif	
			printk("[BAT][Ser]:Cap type BMS:%d,%d\n",BMSCap,batCap);	
			//Eason get BMS Capacity for EventLog---
			//ASUS BSP: Eason check correct BMS RUC+++
			BMS_diff_SWgauge = BMSCap-batCap;
			if(BMS_diff_SWgauge<0)
				BMS_diff_SWgauge = -BMS_diff_SWgauge;
			
			if(BMS_diff_SWgauge<=5)
				gIsBMSerror = false;
			//ASUS BSP: Eason check correct BMS RUC---

			//ASUS BSP: Eason check correct BMS RUC+++
			if(true==gIsBMSerror){
				  	AXC_BatteryService_reportPropertyCapacity(_this,batCap);
					printk("[BAT][BMS]:Error BMS need to use SWgauge capacity\n");
					ASUSEvtlog("[BAT]:SWgauge\n");
			}else
			//ASUS BSP: Eason check correct BMS RUC---
			if((true==g_BootUp_IsBatLow)||(true==_this->BatteryService_IsBatLow))
			{
			      if(batCap<BMSCap){
				  	AXC_BatteryService_reportPropertyCapacity(_this,batCap);
					printk("[BAT][BMS][low]:use lower SWgauge capacity\n");
					ASUSEvtlog("[BAT]:SWgauge low\n");
			      	}else{
			      		AXC_BatteryService_reportPropertyCapacity(_this,BMSCap);
					ASUSEvtlog("[BAT]:BMS low\n");
		      		}
			}
			else
			{
		   		AXC_BatteryService_reportPropertyCapacity(_this,BMSCap);
				ASUSEvtlog("[BAT]:BMS\n");
			}
			#endif //CONFIG_SWGAU_ASUS
    			//Hank: A86 no use---
	  	}	
	 //Eason: choose Capacity type SWGauge/BMS ---	
		
        //Eason : In suspend have same cap don't update savedTime +++
        if( (A66_LastTime_capacity == _this->A66_capacity)&&(true==SameCapDontUpdateSavedTime)&&
            (false==g_RTC_update) )
        {
            printk("[BAT][Ser]:In suspend have same Cap dont update savedTime\n");
        }
	 else
        {
            g_RTC_update = false;
            _this->savedTime=updateNowTime(_this);
	     //Eason:fix Cap drop too slowly in unattended mode+++
	      filRealSusT = 0;
	     //Eason:fix Cap drop too slowly in unattended mode---
            
        }
        //Eason : In suspend have same cap don't update savedTime ---

	  //Eason: when change MaxMah clear interval+++
	  _this->ForceSavedTime = updateNowTime(_this);//for A68 will always update no matter if change MaxMAh
	  //Eason: when change MaxMah clear interval---
    }

     //Hank: schedule next polling in capacity update worker+++ 
     //AXC_BatteryService_scheduleNextPolling(_this);
    //Hank: schedule next polling in capacity update worker---

    //Hank: move unlock to BatteryServiceCapSample+++
    //wake_unlock(&_this->cap_wake_lock);
    //Hank: move unlock to BatteryServiceCapSample---

    mutex_unlock(&_this->main_lock);
    pr_debug("[BAT][SER]%s() --- \n",__func__);	
    return 0;
}
int BatteryServiceGauge_AskSuspendCharging(struct AXI_Gauge_Callback *gaugeCb)
{
    AXC_BatteryService  *_this=
         container_of(gaugeCb, AXC_BatteryService, gaugeCallback);
    _this->callback->changeChargingCurrent(_this->callback,NO_CHARGER_TYPE);
    //gpCharger->EnableCharging(gpCharger,false);//stop curr may need delay
    return 0;
}
int BatteryServiceGauge_AskResumeCharging(struct AXI_Gauge_Callback *gaugeCb)
{
    AXC_BatteryService  *_this=
         container_of(gaugeCb, AXC_BatteryService, gaugeCallback);
    _this->callback->changeChargingCurrent(_this->callback,_this->chargerType);
    //gpCharger->EnableCharging(gpCharger,true);//stop curr may need delay
    return 0;
}

static int Report_P02Cable(void)
{
#ifdef CONFIG_EEPROM_NUVOTON  //ASUS_BSP Eason_Chang 1120 porting +++
	if( (1==AX_MicroP_get_USBDetectStatus(Batt_P01))|| (2==AX_MicroP_get_USBDetectStatus(Batt_P01)) ){//Eason: Pad plug usb show icon & cap can increase
		balance_this->P02_IsCable = true;
	}else{
		balance_this->P02_IsCable = false;
	}
#endif //CONFIG_EEPROM_NUVOTON//ASUS_BSP Eason_Chang 1120 porting ---    

    return balance_this->P02_IsCable;
}
static void Report_P02ChgStatus(int P02Chg) 
{
    balance_this->P02_IsCharging = false;
    balance_this->P02_IsFULL = false; 

    if(1==P02Chg){
            balance_this->P02_IsCharging = true;
    }else if(2==P02Chg){
            balance_this->P02_IsFULL = true; 
    }       
}
//David: PAD cap remap table
#define P02_MAP_NUM	100
static int P02_map_tbl[P02_MAP_NUM + 1] = 
							   {  0,   0,  0,   0,  0,   0,   1,   2,  3,  4,
							       5,   6,  7,   8,  9, 10, 11, 12, 13, 14, 
							     15, 16, 17, 18, 19, 20, 21, 22, 23, 24,
							     25, 27, 28, 29, 30, 31, 32, 33, 34, 35,
							     36, 37, 38, 39, 40, 41, 42, 43, 44, 45,
							     46, 48, 49, 50, 51, 52, 53, 54, 55, 56,
							     57, 59, 60, 61, 62, 63, 64, 65, 66, 67,
							     68, 70, 71, 72, 73, 74, 75, 76, 77, 78,
							     80, 81, 82, 83, 85, 86, 87, 88, 89, 90,
							     91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 
							    100 };
static void P02_reportPropertyCapacity(struct AXC_BatteryService *_this, int P02_refcapacity)
{

    int Pad_capacity;
    int IsBalTest;//ASUS_BSP Eason_Chang BalanceMode

    int lastCapacity;

    int P02_maxMah;

    bool P02_hasCable;
    int  P02_chgStatus=0;//ASUS_BSP Eason_Chang 1120 porting

    time_t P02_intervalSinceLastUpdate;
    int Remap_P02_refcapacity = 0;

    mutex_lock(&_this->filter_lock);

    P02_intervalSinceLastUpdate  = P02_BatteryService_getIntervalSinceLastUpdate(_this);
    

    //We need do ask capcaity to filter at first time, in case there is FULL orBATLow 
    if(true == _this->P02_IsFirstAskCap){

        lastCapacity = P02_refcapacity;

        _this->P02_IsFirstAskCap = false;
        

    }else{

        lastCapacity = _this->Pad_capacity;

    }

    if (true == _this->P02_IsResumeUpdate){
        P02_hasCable = _this->P02_HasCableBeforeSuspend;
        //P02_chgStatus = _this->P02_ChgStatusBeforeSuspend;
        P02_maxMah = _this->P02_MaxMahBeforeSuspend;
        _this->P02_IsResumeUpdate = false;
 
    }else{
        P02_hasCable = Report_P02Cable();
        //P02_chgStatus = AX_MicroP_get_ChargingStatus(Batt_P01);
        P02_maxMah = P02_ChooseMaxMah();
    }

#ifdef CONFIG_EEPROM_NUVOTON  //ASUS_BSP Eason_Chang 1120 porting +++
    P02_chgStatus = AX_MicroP_get_ChargingStatus(Batt_P01);
#endif //CONFIG_EEPROM_NUVOTON//ASUS_BSP Eason_Chang 1120 porting ---
    Report_P02ChgStatus(P02_chgStatus);


    //David: Remap refcapacity from table
    if(P02_refcapacity >= 0 && P02_refcapacity <= P02_MAP_NUM )
    {
    	  Remap_P02_refcapacity = P02_map_tbl[P02_refcapacity];
    }
    else
    {
    	  Remap_P02_refcapacity = P02_refcapacity;
         printk("[BAT][Ser]P02 Cap remap error, out of table!\n");
    }

    Pad_capacity = _this->gpCapFilterP02->filterCapacity
                                     (_this->gpCapFilterP02,
                                      Remap_P02_refcapacity, lastCapacity,
                                      P02_hasCable,
                                      _this->P02_IsCharging,
                                      _this->P02_IsFULL,
                                      _this->P02_IsBatLow,
                                      P02_maxMah,
                                      P02_intervalSinceLastUpdate);

    pr_debug("[BAT][Ser][P02]report Capacity:%d,%d,%d,%d,%d,%d,%d,%d,%ld==>%d\n",
                                    P02_refcapacity,
                                    Remap_P02_refcapacity,
                                    lastCapacity,
                                      P02_hasCable,
                                      _this->P02_IsCharging,
                                      _this->P02_IsFULL,
                                      _this->P02_IsBatLow,
                                      P02_maxMah,
                                      P02_intervalSinceLastUpdate,
                                      Pad_capacity);
    //ASUS_BSP Eason_Chang add event log +++
    ASUSEvtlog("[BAT][Ser][P02]report Capacity:%d,%d,%d,%d,%d,%d,%d,%ld==>%d\n",
                                    P02_refcapacity,
                                    lastCapacity,
                                      P02_hasCable,
                                      _this->P02_IsCharging,
                                      _this->P02_IsFULL,
                                      _this->P02_IsBatLow,
                                      P02_maxMah,
                                      P02_intervalSinceLastUpdate,
                                      Pad_capacity);
    //ASUS_BSP Eason_Chang add event log ---

//ASUS_BSP +++ Eason_Chang  : set Pad_cap for cmd test
    IsBalTest = IsBalanceTest();
    if( 1 == IsBalTest){
            Pad_capacity = BatteryServiceGetPADCAP();
    }
//ASUS_BSP --- Eason_Chang  : set Pad_cap for cmd test 


    if(Pad_capacity < 0 || Pad_capacity >100){

        printk("[BAT][Ser]Filter return value fail!!!\n");
    }else if(_this->Pad_capacity == Pad_capacity){    
       pr_debug("[BAT][Ser]Pad have same cap:%d\n",Pad_capacity);
    }else if(_this->Pad_capacity != Pad_capacity){
       _this->Pad_capacity = Pad_capacity;
       
       BatteryService_P02update();
    }   
    
    //wake_unlock(&_this->cap_wake_lock);

    mutex_unlock(&_this->filter_lock);

}
static int P02Gauge_OnCapacityReply(struct AXI_Gauge *gauge, struct AXI_Gauge_Callback *gaugeCb, int batCap, int result)
{   

    AXC_BatteryService  *_this=
        container_of(gaugeCb, AXC_BatteryService, P02gaugeCallback);

    mutex_lock(&_this->main_lock);

    P02_reportPropertyCapacity(
        _this,
        batCap);

    _this->P02_savedTime=updateNowTime(_this);
    pr_debug("[BAT][Ser]:P02Gauge_OnCapacityReply\n");
    //ReportTime();

    mutex_unlock(&_this->main_lock);

     //Hank: A86 no SWgauge+++
    if(g_ASUS_hwID != A90_EVB0)
	    _this->gauge->askCapacity(_this->gauge);
    else
	    BatteryServiceGauge_OnCapacityReply(NULL, &_this->gaugeCallback, 100, 0);
    //Hank: A86 no SWgauge---	

    pr_debug("[BAT][Ser]:P02Gauge_askCapacity\n");
    //ReportTime();
    return 0;
}
int P02Gauge_AskSuspendCharging(struct AXI_Gauge_Callback *gaugeCb)
{ 
    return 0;
}
int P02Gauge_AskResumeCharging(struct AXI_Gauge_Callback *gaugeCb)
{
    return 0;
}

#ifdef CONFIG_ASUSDEC
static int Report_DockCable(void)
{
#ifdef CONFIG_EEPROM_NUVOTON  //ASUS_BSP Eason_Chang 1120 porting +++
	if(1==AX_MicroP_get_USBDetectStatus(Batt_Dock)){
		balance_this->Dock_IsCable = true;
	}else{
		balance_this->Dock_IsCable = false;
	}
#endif //CONFIG_EEPROM_NUVOTON//ASUS_BSP Eason_Chang 1120 porting ---    

    return balance_this->Dock_IsCable;
}

static void Report_DockChgStatus(int DockChg) 
{
    balance_this->Dock_IsCharging = false;
    balance_this->Dock_IsFULL = false; 

    if(1==DockChg){
            balance_this->Dock_IsCharging = true;
    }else if(2==DockChg){
            balance_this->Dock_IsFULL = true; 
    }       
}
static void Dock_reportPropertyCapacity(struct AXC_BatteryService *_this, int Dock_refcapacity)
{

    int Dock_capacity;

    int lastCapacity;

    int Dock_maxMah;

    bool Dock_hasCable;
    int  Dock_chgStatus=0;//ASUS_BSP Eason_Chang 1120 porting

    time_t Dock_intervalSinceLastUpdate;

    mutex_lock(&_this->filter_lock);

    Dock_intervalSinceLastUpdate  = Dock_BatteryService_getIntervalSinceLastUpdate(_this);
    

    //We need do ask capcaity to filter at first time, in case there is FULL orBATLow 
    if(true == _this->Dock_IsFirstAskCap){

        lastCapacity = Dock_refcapacity;

        _this->Dock_IsFirstAskCap = false;
        
    }else{

        lastCapacity = _this->Dock_capacity;

    }

    if (true == _this->Dock_IsResumeUpdate){
        Dock_hasCable = _this->Dock_HasCableBeforeSuspend;
        //Dock_chgStatus = _this->Dock_ChgStatusBeforeSuspend;
        Dock_maxMah = _this->Dock_MaxMahBeforeSuspend;
        printk("[BAT][Ser][Dock]ResumeUpdate\n");
        _this->Dock_IsResumeUpdate = false;
 
    }
    else
    {
        Dock_hasCable = Report_DockCable();
        //Dock_chgStatus = AX_MicroP_get_ChargingStatus(Batt_Dock);
        Dock_maxMah = Dock_ChooseMaxMah();
    }

#ifdef CONFIG_EEPROM_NUVOTON  //ASUS_BSP Eason_Chang 1120 porting +++
    Dock_chgStatus = AX_MicroP_get_ChargingStatus(Batt_Dock);
#endif //CONFIG_EEPROM_NUVOTON//ASUS_BSP Eason_Chang 1120 porting ---
    Report_DockChgStatus(Dock_chgStatus);

    Dock_capacity = _this->gpCapFilterDock->filterCapacity
                                     (_this->gpCapFilterDock,
                                      Dock_refcapacity, lastCapacity,
                                      Dock_hasCable,
                                      _this->Dock_IsCharging,
                                      _this->Dock_IsFULL,
                                      _this->Dock_IsBatLow,
                                      Dock_maxMah,
                                      Dock_intervalSinceLastUpdate);

    printk("[BAT][Ser][Dock]report Capacity:%d,%d,%d,%d,%d,%d,%d,%ld==>%d\n",
                                    Dock_refcapacity,
                                    lastCapacity,
                                      Dock_hasCable,
                                      _this->Dock_IsCharging,
                                      _this->Dock_IsFULL,
                                      _this->Dock_IsBatLow,
                                      Dock_maxMah,
                                      Dock_intervalSinceLastUpdate,
                                      Dock_capacity);

//ASUS_BSP +++ Eason_Chang  : set Pad_cap for cmd test
/*
    IsBalTest = IsBalanceTest();
    if( 1 == IsBalTest){
            Pad_capacity = BatteryServiceGetDockCAP();
    }
*/
//ASUS_BSP --- Eason_Chang  : set Pad_cap for cmd test 


    if(Dock_capacity < 0 || Dock_capacity >100){

        printk("[BAT][Ser][Dock]Filter return value fail!!!\n");
    }else if(_this->Dock_capacity == Dock_capacity){    
       printk("[BAT][Ser]Dock have same cap:%d\n",Dock_capacity);
    }else if(_this->Dock_capacity != Dock_capacity){
       _this->Dock_capacity = Dock_capacity;
       
       //BatteryService_Dockupdate();//Pad will do this together
    }   
    
    //wake_unlock(&_this->cap_wake_lock);

    mutex_unlock(&_this->filter_lock);

}
static int DockGauge_OnCapacityReply(struct AXI_Gauge *gauge, struct AXI_Gauge_Callback *gaugeCb, int batCap, int result)
{   

    AXC_BatteryService  *_this=
        container_of(gaugeCb, AXC_BatteryService, DockgaugeCallback);

    mutex_lock(&_this->main_lock);

    Dock_reportPropertyCapacity(
        _this,
        batCap);

    _this->Dock_savedTime=updateNowTime(_this);
    pr_debug("[BAT][Ser]:P02Gauge_OnCapacityReply\n");
    //ReportTime();

    mutex_unlock(&_this->main_lock);

    _this->P02gauge->askCapacity(_this->P02gauge);
    pr_debug("[BAT][Ser]:P02Gauge_askCapacity\n");
    //ReportTime();
    return 0;
}
int DockGauge_AskSuspendCharging(struct AXI_Gauge_Callback *gaugeCb)
{ 
    return 0;
}
int DockGauge_AskResumeCharging(struct AXI_Gauge_Callback *gaugeCb)
{
    return 0;
}
#endif //#ifdef CONFIG_ASUSDEC

//ASUS BSP Eason_Chang --- batteryservice to gauge

//static int BatteryService_CalculateBATCAP(AXC_BatteryService *_this)
//{             
//    return _this->gauge->GetBatteryLife(_this->gauge);
//}
static bool BatteryService_ifFixedPollingInterval(struct AXC_BatteryServiceTest *test)
{
    return (-1 != test->pollingInterval);
}
static bool BatteryService_ifFixedFilterLastUpdateInterval(struct AXC_BatteryServiceTest *test)
{
    return (-1 != test->filterLastUpdateInterval);

}
static void BatteryService_changePollingInterval(struct AXC_BatteryServiceTest *test,bool fixed,int interval)
{
    AXC_BatteryService  *_this=
        container_of(test, AXC_BatteryService, test);

    if(fixed){
        printk("%s:fix interval to %d\n",__FUNCTION__,interval);

        test->pollingInterval = interval;

        _this->miParent.suspend(&_this->miParent);

        _this->miParent.resume(&_this->miParent, interval);

    }else{
        printk("%s:don't fix interval\n",__FUNCTION__);

        test->pollingInterval = -1;

    }
}
static void BatteryService_changeFilterLastUpdateInterval(struct AXC_BatteryServiceTest *test,bool fixed,int interval)
{
    if(fixed){

        printk("%s:fix interval to %d\n",__FUNCTION__,interval);

        test->filterLastUpdateInterval = interval;

    }else{
        printk("%s:don't fix interval\n",__FUNCTION__);

        test->filterLastUpdateInterval = -1;
    }
}
static AXC_BatteryService g_AXC_BatteryService={
    .miParent = {
        .getChargingStatus = AXC_BatteryService_getChargingStatus,
        .getCapacity = AXC_BatteryService_getCapacity,
        .onCableInOut =AXC_BatteryService_onCableInOut,
        .onChargingStop =AXC_BatteryService_onChargingStop,
        .onChargingStart = AXC_BatteryService_onChargingStart,
        .onBatteryLowAlarm= AXC_BatteryService_onBatteryLowAlarm,
        .onBatteryRemoved = AXC_BatteryService_onBatteryRemoved,
        .suspend = AXC_BatteryService_suspend,
        .resume =AXC_BatteryService_resume,
        .forceResume = AXC_BatteryService_forceResume,
        .dockSuspend = AXC_BatteryService_dockSuspend,
    },
    //Hank: temperature monitor+++
    .NeedCalCap = false,
    .NeedKeep5Min  = false,
    .TempLimit = 0,
    //Hank: temperature monitor---
    .mbInit = false,
    .IsFirstForceResume = true,
    .callback = NULL,
    .A66_capacity = 50,//saved capacity //Hank: default capacity 50 prevent default full
    .Pad_capacity = 100,
    .ForceSavedTime = 0,//for A68 will always update no matter if change MaxMAh
    .savedTime = 0,//for A68 may dont update if change 10==MaxMAh
    .P02_savedTime = 0,
    .BatteryService_IsCable = false,
    .BatteryService_IsCharging = false,
    .BatteryService_IsFULL = false,
    .BatteryService_IsBatLow = false,
    .isMainBatteryChargingDone= false,
    .IsFirstAskCap = true,
    .IsFirstAskCable = true,
    .HasCableBeforeSuspend = false,
    .IsResumeUpdate = false,
    .IsResumeMahUpdate = false,
    .IsCalculateCapOngoing = false,
    .P02_IsCable = false,
    .P02_IsCharging = false,
    .P02_IsFULL = false,
    .P02_IsBatLow = false,
    .P02_IsFirstAskCap = true,
    .P02_HasCableBeforeSuspend = false,
    .P02_IsResumeUpdate = false,
    //.P02_ChgStatusBeforeSuspend = 0,
    .P02_MaxMahBeforeSuspend = 0,
    #ifdef CONFIG_ASUSDEC
    .Dock_capacity = 100,
    .Dock_savedTime = 0,
    .Dock_IsCable = false,
    .Dock_IsCharging = false,
    .Dock_IsFULL = false,
    .Dock_IsBatLow = false,
    .Dock_IsFirstAskCap = true,
    .Dock_HasCableBeforeSuspend = false,
    .Dock_IsResumeUpdate = false,
    //.Dock_ChgStatusBeforeSuspend = 0,
    .Dock_MaxMahBeforeSuspend = 0,
    .IsSuspend = true,
    .IsDockExtChgIn = false,
    .IsDockInitReady = false,
    #endif
    .gaugeCallback ={
        .onCapacityReply = BatteryServiceGauge_OnCapacityReply,
        .askSuspendCharging = BatteryServiceGauge_AskSuspendCharging,   
        .askResumeCharging = BatteryServiceGauge_AskResumeCharging,
        },// batteryservice to gauge
    .P02gaugeCallback ={
        .onCapacityReply = P02Gauge_OnCapacityReply,
        .askSuspendCharging = P02Gauge_AskSuspendCharging,   
        .askResumeCharging = P02Gauge_AskResumeCharging,
        },// batteryservice to gauge 
    #ifdef CONFIG_ASUSDEC
    .DockgaugeCallback ={
        .onCapacityReply = DockGauge_OnCapacityReply,
        .askSuspendCharging = DockGauge_AskSuspendCharging,   
        .askResumeCharging = DockGauge_AskResumeCharging,
        },// batteryservice to gauge
    #endif
    .chargerType =  NOTDEFINE_TYPE ,  // batteryservice to gauge
    .gauge = NULL,  // batteryservice to gauge
    .P02gauge = NULL,
    .gpCapFilterA66 = NULL,
    .gpCapFilterP02 = NULL,
    .defaultPollingInterval = DEFAULT_ASUSBAT_POLLING_INTERVAL , // batteryservice to gauge
    .fsmCallback ={
        .onChangeChargingCurrent = BatteryServiceFsm_OnChangeChargingCurrent,
        .onStateChanged = BatteryServiceFsm_OnStateChanged,
        },// batteryservice to fsm
    .fsm = NULL,                 // batteryservice to fsm
    .fsmState = NOTDEFINE_STATE ,// batteryservice to fsm
    .test = {
        .pollingInterval = -1,
        .filterLastUpdateInterval = -1,
        .ifFixedPollingInterval = BatteryService_ifFixedPollingInterval,
        .ifFixedFilterLastUpdateInterval =BatteryService_ifFixedFilterLastUpdateInterval,
        .changePollingInterval=BatteryService_changePollingInterval,
        .changeFilterLastUpdateInterval= BatteryService_changeFilterLastUpdateInterval,
    },
    .gChargerStateChangeNotifier={
        .Notify = NotifyForChargerStateChanged,
        .onChargingStart = onChargingStart,
        },
};

AXI_BatteryServiceFacade *getBatteryService(AXI_BatteryServiceFacadeCallback *callback)
{
    static AXI_BatteryServiceFacade *lpBatteryService = NULL;

    if(NULL == lpBatteryService){

        lpBatteryService = &g_AXC_BatteryService.miParent;

        AXC_BatteryService_constructor(&g_AXC_BatteryService, callback);
    }

    return lpBatteryService;
}

AXC_BatteryServiceTest *getBatteryServiceTest(void)
{
    return &g_AXC_BatteryService.test;
}

int getPowerBankCharge(void)
{
    return IsPowerBankCharge;
}

int getBalanceCharge(void)
{
    return IsBalanceCharge;
}
#endif //#ifdef CONFIG_BATTERY_ASUS_SERVICE









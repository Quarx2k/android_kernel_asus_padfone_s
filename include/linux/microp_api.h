#ifndef __API_MICROP_H
#define __API_MICROP_H




enum Batt_ID{
	Batt_P01=0,
	Batt_Dock=1,
};


enum p01_Cable_Status{
       P01_CABLE_UNKNOWN=-1,
       P01_CABLE_NO=0,
       P01_CABLE_CHARGER=1,
       P01_CABLE_USB=2,
       P01_CABLE_OTG=7,       
};


enum p01_Charging_Status{
	P01_CHARGING_ERR = -1,
	P01_CHARGING_NO = 0,
	P01_CHARGING_ONGOING=1,
	P01_CHARGING_FULL=2,
};

enum P05_HW_ID{
	P05_SR4_HWID=0,
	P05_SR5_HWID=1,
	P05_ER_HWID=2,
	P05_PR_HWID=3,
};

enum P92L_HW_ID{
	P92_EVB_HWID=0,
	P92_SR1_HWID=0,
	P92_ER1_HWID=1,
	P92_ER2_HWID=2,
	P92_PR_HWID=3,
};

enum PAD_HW_MODEL{
	PAD_UNKNOWN_MODEL = 0,
	PAD_P05_P05E = 1,
	PAD_P71L = 2,
	PAD_P101G = 3,
	PAD_P92L = 4,
	PAD_P05C = 5,
        PAD_P93L,
};


enum PAD_LED_COLOR{
        PAD_LED_O,
        PAD_LED_G,
};

enum PAD_LED_STYLE{
        PAD_ACT_OFF=0x0,
        PAD_ACT_ON=0x1,
        PAD_ACT_BLINK=0x2,
        PAD_ACT_BREATH=0x4,
};



/*
*       Check the status of P01 connectness
*       return value: 1: P01 connected
*/

int AX_MicroP_IsP01Connected(void);


/*
*       Check the status of Dock connectness
*       return value: 1: Dock connected and ready
*/
int AX_MicroP_IsDockReady(void);


/*
*       Check the status of Headphone if it is inserted
*       return value: 0: plugged out, 1: plugged in, <0: err


int AX_MicroP_IsHeadPhoneIn(void);

*/


/*
*       Check the status of AC/USB if it is inserted
*       return value: 0: plugged out, 1: plugged in, <0: err
*/

int AX_MicroP_IsACUSBIn(void);




/*
*       Check the status of Dock if it is inserted
*       return value: 0: plugged out, 1: plugged in, <0: err
*/

int AX_MicroP_IsECDockIn(void);



/*
*       Check the status of Dock battery if it is power-bad
*       return value: 1: PowerBad, <0: err
*/

int AX_MicroP_Is_ECBattPowerBad(void);


/*
*       Check the status of Dock Ext. Power if ext power is in
*       return value: 1: PowerBad, <0: err
*/
int AX_MicroP_Is_ECExtPowerCableIn(void);

/*
*   @AX_MicroP_get_BattTemp
*  input: target
*           0: p01 battery
*           1: dock battery
*    return: current batt temp;
*/

int AX_MicroP_get_BattTemp(int target);

/*
*   @AX_MicroP_get_AvgCurrent
*  input: target
*           0: p01 battery
*           1: dock battery
*    return: average mA
*/

int AX_MicroP_get_AvgCurrent(int target);

/*
*   @AX_MicroP_get_ChargingStatus
*  input: target
*           0: p01 battery
*           1: dock battery
*
* return: -1: charge error, 0: no charge, 1: charging normal, 2: charging full, < 0: other error
*/

int AX_MicroP_get_ChargingStatus(int target);

/*
*   @AX_MicroP_get_BattVolatge
*  input: target
*           0: p01 battery
*           1: dock battery
*    return: current batteyr voltage
*/

int AX_MicroP_get_BattVoltage(int target);

/*
*   @AX_MicroP_get_USBDetectStatus
*  input: target
*           0: p01 battery
*           1: dock battery
*
*  return: 0 for 'no charger/usb', 1 for 'charger', 2 for 'USB', '255' for 'unknown', <0 value means something error
*/ 

int AX_MicroP_get_USBDetectStatus(int target);

/*
*  GPIO direct control
*  @ AX_MicroP_getGPIOPinLevel
*  input: 
            - pinID
*  return: 0 for low, 1 for high, <0 value means something error
*

*  @ AX_MicroP_setGPIOOutputPin
*  input: 
*           - pinID
*           - level: 0 for low, 1 for high
*  return: the status of operation. 0 for success, <0 value means something error

*  @ AX_MicroP_getGPIOOutputPinLevel
*  input:
            - pinID
*  return: 0 for low, 1 for high, <0 value means something error
*/

int AX_MicroP_getGPIOPinLevel(int pinID);
int AX_MicroP_setGPIOOutputPin(int pinID, int level);
int AX_MicroP_getGPIOOutputPinLevel(int pinID);



// return =0: success
//           <0: error
int AX_MicroP_setPWMValue(uint8_t value);

// return >=0: success
//           <0: error
int AX_MicroP_getPWMValue(void);
int AX_MicroP_Is_3V3_ON(void);
void AX_MicroP_Bus_Suspending(int susp);
int AX_MicroP_enterSleeping(void);
int AX_MicroP_enterResuming(void);
/*
*  @AX_MicroP_enableInterrupt
*  input: 
*            - intrpin: input pin id
*            -  enable: 0 for 'disable', 1 for 'enable'
*  return: 0 for success, <0 value means something error
*/

int AX_MicroP_enablePinInterrupt(unsigned int pinID, int enable);



/*
*  @AX_MicroP_readBattCapacity
*  input: target
*           0: p01 battery
*           1: dock battery
*  return: 0 for success, <0 value means something error
*/


int AX_MicroP_readBattCapacity(int target);


/*
*  @AX_IsPadUsing_MHL_H
*  return: 1 for MHL_H
*  return: 0 for MHL_L
    else: something err
*/


int AX_IsPadUsing_MHL_H(void);


int AX_MicroP_getOPState(void);

int AX_MicroP_getMHLNvramState(void);
int AX_MicroP_writeKDataOfLightSensor(uint64_t data);
uint64_t AX_MicroP_readKDataOfLightSensor(void);

int AX_MicroP_writeCompassData(char* data, int length);
int AX_MicroP_readCompassData(char* data, int length);


/*
*      Export function for Dock Use
*/

int get_EC_DOCK_IN_STATUS(void);
int get_EC_AP_WAKE_STATUS(void);
int set_EC_REQUEST_VALUE(int value);
int get_EC_HALL_SENSOR_STATUS(void);
int get_MicroP_HUB_SLEEP_STATUS(void);
int EC_Init_Complete(void);
int EC_Get_EXT_POWER_PLUG_IN_Ready(void);
int EC_Get_EXT_POWER_PLUG_OUT_Ready(void);
int EC_Get_DOCK_BATTERY_POWER_BAD_READY(void);

int AX_MicroP_initLightsensor(uint8_t bOn);
int AX_MicroP_getLightsensorInitResult(void);
int AX_MicroP_getLightsensorADC(void);
int AX_MicroP_setLightsensor_TS_Low(uint16_t val);
int AX_MicroP_setLightsensor_TS_High(uint16_t val);
int AX_MicroP_getLCMID(void);
int AX_MicroP_getTSID(void);
int AX_MicroP_getHWID(void);
extern bool pad_exist(void);
int AX_MicroP_IsMydpNewSKU(void);
int AX_MicroP_ControlLED(uint8_t styleofOrange, uint8_t styleofGreen);
int AX_MicroP_getPadModel(void);
#endif

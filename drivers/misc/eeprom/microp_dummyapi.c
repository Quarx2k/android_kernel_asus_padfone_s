#ifndef CONFIG_EEPROM_NUVOTON
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/microp_api.h>
#include <linux/microp_notify.h>
int AX_MicroP_IsP01Connected(void){ return 0;}
int AX_MicroP_IsDockReady(void){ return 0;}
int AX_MicroP_IsACUSBIn(void){ return 0;}
int AX_MicroP_IsECDockIn(void){ return 0;}
int AX_MicroP_Is_ECBattPowerBad(void){ return 0;}
int AX_MicroP_Is_ECExtPowerCableIn(void){ return 0;}
int AX_MicroP_get_ChargingStatus(int target){ return 0;}
int AX_MicroP_get_USBDetectStatus(int target){ return 0;}
int AX_MicroP_getGPIOPinLevel(int pinID){ return 0;}
int AX_MicroP_setGPIOOutputPin(int pinID, int level){ return 0;}
int AX_MicroP_getGPIOOutputPinLevel(int pinID){ return 0;}
int AX_MicroP_setPWMValue(uint8_t value){ return 0;}
int AX_MicroP_getPWMValue(void){ return 0;}
int AX_MicroP_enterSleeping(void){ return 0;}
int AX_MicroP_enterResuming(void){ return 0;}
int AX_MicroP_enablePinInterrupt(unsigned int pinID, int enable){ return 0;}
int AX_MicroP_readBattCapacity(int target){ return 0;}
int AX_IsPadUsing_MHL_H(void){ return 0;}
int AX_MicroP_getOPState(void){ return 0;}
int AX_MicroP_getMHLNvramState(void){ return 0;}
int AX_MicroP_writeKDataOfLightSensor(uint64_t data){ return 0;}
uint64_t AX_MicroP_readKDataOfLightSensor(void){ return 0;}
int AX_MicroP_writeCompassData(char* data, int length){ return 0;}
int AX_MicroP_readCompassData(char* data, int length){ return 0;}
int get_EC_DOCK_IN_STATUS(void){ return 0;}
int get_EC_AP_WAKE_STATUS(void){ return 0;}
int set_EC_REQUEST_VALUE(int value){ return 0;}
int get_EC_HALL_SENSOR_STATUS(void){ return 0;}
int get_MicroP_HUB_SLEEP_STATUS(void){ return 0;}
int EC_Init_Complete(void){ return 0;}
int EC_Get_EXT_POWER_PLUG_IN_Ready(void){ return 0;}
int EC_Get_EXT_POWER_PLUG_OUT_Ready(void){ return 0;}
int EC_Get_DOCK_BATTERY_POWER_BAD_READY(void){ return 0;}
int AX_MicroP_initLightsensor(uint8_t bOn){ return 0;}
int AX_MicroP_getLightsensorInitResult(void){ return 0;}
int AX_MicroP_getLightsensorADC(void){ return 0;}
int AX_MicroP_setLightsensor_TS_Low(uint16_t val){ return 0;}
int AX_MicroP_setLightsensor_TS_High(uint16_t val){ return 0;}
int AX_MicroP_getTSID(void){ return 0;}
int AX_MicroP_getLCMID(void){ return 0;}
int AX_MicroP_getHWID(void){ return 0;}
int uP_nuvoton_write_reg(int cmd, void *data){ return 0;}
bool pad_exist(void){ return 0;}
int register_microp_notifier(struct notifier_block *nb){ return 0;}
int unregister_microp_notifier(struct notifier_block *nb){ return 0;}
void notify_register_microp_notifier(struct notifier_block *nb, char* driver_name){return;}
void notify_unregister_microp_notifier(struct notifier_block *nb, char* driver_name){return;}
int AX_MicroP_IsMydpNewSKU(void){return 0;}
#endif

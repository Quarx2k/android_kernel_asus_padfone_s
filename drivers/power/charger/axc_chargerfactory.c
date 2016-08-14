#include <linux/slab.h>
#include "axc_chargerfactory.h"
#include "axc_Smb346Charger.h"
#include "axc_PM8921Charger.h"
#include "axc_PM8941Charger.h"
#include <linux/asus_bat.h>
#include "axc_DummyCharger.h"
#include "../axi_powerglobal.h"


//#include "../asus_bat_dbg.h"
void AXC_ChargerFactory_GetCharger(AXE_CHARGER_TYPE aeChargerType , AXI_Charger **appCharger)
{
	if (NULL != *appCharger)  {
		printk(DBGMSK_BAT_ERR "[BAT][ChagerFactory]Memory leak...\n");
	}

	*appCharger =  NULL;
	switch(aeChargerType) {
#ifdef CONFIG_SMB_346_CHARGER
	case E_SMB346_CHARGER_TYPE:
{
		static AXC_SMB346Charger *lpCharger = NULL;
		if(NULL == lpCharger)
		{
			lpCharger = kzalloc(sizeof(AXC_SMB346Charger), GFP_KERNEL);
			BUG_ON(NULL == lpCharger);
		}		

		*appCharger = &lpCharger->msParentCharger ;
		AXC_SMB346Charger_Binding(*appCharger, aeChargerType);
}
		break;
#endif
	case E_PM8921_CHARGER_TYPE:
	{
		static AXC_PM8921Charger *lpCharger = NULL;
		if(NULL == lpCharger)
		{
			lpCharger = kzalloc(sizeof(AXC_PM8921Charger), GFP_KERNEL);
//			BUG_ON(NULL == lpCharger);
		}		

		*appCharger = &lpCharger->msParentCharger ;
		AXC_PM8921Charger_Binding(*appCharger, aeChargerType);
		break;
	}
#ifdef CONFIG_PM_8941_CHARGER
	case E_PM8941_CHARGER_TYPE:
	{
		static AXC_PM8941Charger *lpCharger = NULL;
		if(NULL == lpCharger){
			lpCharger = kzalloc(sizeof(AXC_PM8941Charger), GFP_KERNEL);
		}

		*appCharger = &lpCharger->msParentCharger ;
		AXC_PM8941Charger_Binding(*appCharger, aeChargerType);
		break;
	}
#endif
	
//#ifdef CONFIG_DUMMY_CHARGER		
    case  E_DUMMY_CHARGER_TYPE:
    {
            static AXC_DummyCharger *lpCharger = NULL;
			if(NULL == lpCharger)
            {
                lpCharger = kzalloc(sizeof(AXC_DummyCharger), GFP_KERNEL);
                assert(NULL != lpCharger);
            }		

            *appCharger = &lpCharger->msParentCharger ;
            AXC_DummyCharger_Binding(*appCharger, aeChargerType);
            break;
     }
//#endif
	default:
		printk(DBGMSK_BAT_ERR "[BAT][ChagerFactory]Not defined type...\n");
		break;
	}
	return;
}

void AXC_ChargerFactory_FreeCharger(AXI_Charger *apCharger)
{
	if (NULL == apCharger)
		return;

	switch(apCharger->GetType(apCharger)) {
 #ifdef CONFIG_SMB_346_CHARGER
	case E_SMB346_CHARGER_TYPE:
{
		AXC_SMB346Charger *lpCharger = container_of(apCharger, AXC_SMB346Charger, msParentCharger);
		kfree(lpCharger);
}
		break;
#endif
	case E_PM8921_CHARGER_TYPE:
	{
		AXC_PM8921Charger *lpCharger = container_of(apCharger, AXC_PM8921Charger, msParentCharger);
		kfree(lpCharger);
		break;
	}
#ifdef CONFIG_DUMMY_CHARGER			
	case E_DUMMY_CHARGER_TYPE:
   {
                AXC_DummyCharger *lpCharger = container_of(apCharger, AXC_DummyCharger, msParentCharger);
                kfree(lpCharger);
				break;
   }
#endif
	default:
		printk(DBGMSK_BAT_ERR "[BAT][FreeChager]Not defined type...\n");
		break;
	}
}

//Eason_Chang:for A90 internal ChgGau+++
extern int getIfonline_pm8941(void);
extern int getIfonline_smb346(void);
int getIfonline(void)
{

	if(g_ASUS_hwID != A90_EVB0)
		return getIfonline_pm8941();
	else
		return getIfonline_smb346();
}

extern void AcUsbPowerSupplyChange_pm8941(void);
extern void AcUsbPowerSupplyChange_smb346(void);
void AcUsbPowerSupplyChange(void)
{
	if(g_ASUS_hwID != A90_EVB0)
		AcUsbPowerSupplyChange_pm8941();
	else
		AcUsbPowerSupplyChange_smb346();
}

extern void setChgDrawCurrent_pm8941(void);
extern void setChgDrawCurrent_smb346(void);
void setChgDrawCurrent(void)
{
	if(g_ASUS_hwID != A90_EVB0)
		setChgDrawCurrent_pm8941();
	else
		setChgDrawCurrent_smb346();
}

extern void UsbSetOtgSwitch_pm8941(bool switchOtg);
extern void UsbSetOtgSwitch_smb346(bool switchOtg);
void UsbSetOtgSwitch(bool switchOtg)
{
	if(g_ASUS_hwID != A90_EVB0)
		UsbSetOtgSwitch_pm8941(switchOtg);
	else
		UsbSetOtgSwitch_smb346(switchOtg);
}

extern int registerChargerInOutNotificaition_pm8941(void (*callback)(int));
extern int registerChargerInOutNotificaition_smb346(void (*callback)(int));
int registerChargerInOutNotificaition(void (*callback)(int))
{
	if(g_ASUS_hwID != A90_EVB0)
		return registerChargerInOutNotificaition_pm8941(callback);
	else
		return registerChargerInOutNotificaition_smb346(callback);
}

#ifndef ASUS_FACTORY_BUILD
extern void setChgLimitThermalRuleDrawCurrent_pm8941(bool isSuspendCharge);
extern void setChgLimitThermalRuleDrawCurrent_smb346(bool isSuspendCharge);
void setChgLimitThermalRuleDrawCurrent(bool isSuspendCharge)
{
	if(g_ASUS_hwID != A90_EVB0)
		setChgLimitThermalRuleDrawCurrent_pm8941(isSuspendCharge);
 	else
		setChgLimitThermalRuleDrawCurrent_smb346(isSuspendCharge);
}

extern void setChgLimitInPadWhenChgReset_smb346(void);
extern void setChgLimitInPadWhenChgReset_pm8941(void);
void setChgLimitInPadWhenChgReset(void)
{
	if(g_ASUS_hwID != A90_EVB0)	
		setChgLimitInPadWhenChgReset_pm8941();
	else
		setChgLimitInPadWhenChgReset_smb346();
}
#endif
//Eason_Chang:for A90 internal ChgGau---